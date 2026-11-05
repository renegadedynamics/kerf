// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// Kerf addition (unit 7L): not a port of an OCCT class. tools/dsrec/main.cpp writes native OCCT's data structure in
// the same format; keep the two in step.

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using System.Globalization;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans.Legacy.Data;

/// <summary>
/// The legacy data structure as text, and back: everything a fillet or chamfer builder writes into it (shapes with their
/// same-domain data, points, curves with their 3D curves and surface/curve interferences, surfaces, every interference
/// with its kind, transition, indices, parameter and pcurve, the new surfaces of faces, the section edges).
/// </summary>
/// <remarks>
/// <para>
/// tools/dsrec writes native OCCT's data structure in this format just before <c>TopOpeBRepBuild_HBuilder::Perform</c>,
/// so the port's builders (units 7N1, 7N2) can compare theirs with native's byte for byte, and unit 7R can rebuild a
/// native data structure with <see cref="Read"/> to work from. The format (version 1), line by line:
/// </para>
/// <code>
/// dsdump 1
/// shapes &lt;n&gt;
/// shape &lt;i&gt; &lt;shape-ref&gt; keep &lt;0|1&gt; sdref &lt;i&gt; sdori &lt;UNSH|SAME|DIFF&gt; sdind &lt;i&gt; rank &lt;i&gt; sd &lt;n&gt; &lt;shape indices&gt;
/// interferences &lt;n&gt;          (then n interference lines: the shape's list)
/// points &lt;n&gt;
/// point &lt;i&gt; &lt;x&gt; &lt;y&gt; &lt;z&gt; &lt;tol&gt; keep &lt;0|1&gt;   or   point &lt;i&gt; unbound
/// interferences &lt;n&gt; ...
/// curves &lt;n&gt;
/// curve &lt;i&gt; &lt;tol&gt; range &lt;0|1&gt; &lt;first&gt; &lt;last&gt; keep &lt;0|1&gt; mother &lt;i&gt; index &lt;i&gt; walk &lt;0|1&gt; sci &lt;#id|-&gt; &lt;#id|-&gt;
/// s1 &lt;shape-ref&gt;
/// s2 &lt;shape-ref&gt;
/// c3d                         (then the curve as GeomTools::Write writes it)   or   c3d null
/// interferences &lt;n&gt; ...
/// surfaces &lt;n&gt;
/// surface &lt;i&gt; &lt;tol&gt; keep &lt;0|1&gt;   or   surface &lt;i&gt; unbound
/// surf                        (then the surface as GeomTools::Write writes it)
/// interferences &lt;n&gt; ...
/// extra &lt;n&gt;                   (the interferences only curves hold, as their sci)
/// newsurfaces &lt;n&gt;
/// newsurface &lt;shape-ref&gt;      (then surf ...)
/// sectionedges &lt;n&gt;
/// sectionedge &lt;shape-ref&gt;
/// isfafa &lt;0|1&gt;
/// end
/// </code>
/// <para>
/// A shape-ref is <c>null</c>, or <c>TYPE ORIENTATION in k</c> (the k-th shape of its type in
/// <c>TopExp::MapShapes(input, TYPE)</c>), or <c>TYPE ORIENTATION new</c> (a vertex adds its point). An interference
/// is written in full the first time it is met (<c>I #id CLASS before/after shapebefore/shapeafter
/// indexbefore/indexafter SUPPORTKIND support GEOMETRYKIND geometry</c>, then the parameter of a CPI, the GBound and
/// configuration of an SSHI or FEI, those and the parameter of an EVI, or a <c>pcurve</c> block for an SCI) and as
/// <c>R #id</c> afterwards, so shared interferences stay shared. Identifiers follow the order the dump meets them.
/// Reals are written as C's <c>%.17g</c>, geometry by GeomTools at precision 17.
/// </para>
/// </remarks>
internal static class TextDump
{
    private static readonly string[] TypeNames = ["COMPOUND", "COMPSOLID", "SOLID", "SHELL", "FACE", "WIRE", "EDGE", "VERTEX", "SHAPE"];
    private static readonly string[] OriNames = ["FORWARD", "REVERSED", "INTERNAL", "EXTERNAL"];
    private static readonly string[] StateNames = ["IN", "OUT", "ON", "UNKNOWN"];
    private static readonly string[] KindNames = ["POINT", "CURVE", "SURFACE", "VERTEX", "EDGE", "WIRE", "FACE", "SHELL", "SOLID", "COMPSOLID", "COMPOUND", "UNKNOWN"];
    private static readonly string[] ConfigNames = ["UNSH", "SAME", "DIFF"];

    private static string R(double v) => StreamUtilities.FormatReal(v, 17);

    private static string I(int v) => v.ToString(CultureInfo.InvariantCulture);

    /// <summary>The transition as the dump writes it: <c>OUT/IN FACE/FACE 0/0</c>.</summary>
    internal static string TransitionText(in DsTransition t)
        => StateNames[(int)t.Before()] + "/" + StateNames[(int)t.After()] + " " + TypeNames[(int)t.ShapeBefore()] + "/"
           + TypeNames[(int)t.ShapeAfter()] + " " + I(t.IndexBefore()) + "/" + I(t.IndexAfter());

    /// <summary>The text of the data structure <paramref name="ds"/>, whose shapes are referred to <paramref name="input"/>'s.</summary>
    internal static string Write(DsDataStructure ds, in Shape input)
    {
        Writer w = new(ds, input);
        return w.Run();
    }

    private sealed class Writer
    {
        private readonly DsDataStructure _ds;
        private readonly StringWriter _os = new(CultureInfo.InvariantCulture) { NewLine = "\n" };
        private readonly Dictionary<DsInterference, int> _ids = new(ReferenceEqualityComparer.Instance);
        private readonly List<DsInterference> _order = [];
        private readonly HashSet<int> _written = [];
        private readonly IndexedSet<Shape>[] _inMaps = new IndexedSet<Shape>[9];

        internal Writer(DsDataStructure ds, in Shape input)
        {
            _ds = ds;
            StreamUtilities.Precision(_os, 17);
            for (int t = 0; t < 9; t++)
            {
                _inMaps[t] = new IndexedSet<Shape>(ShapeHasher.Instance);
                if (t < 8 && !input.IsNull)
                {
                    Exploration.MapShapes(input, (ShapeType)t, _inMaps[t]);
                }
            }
        }

        private void W(string s) => _os.Write(s);

        private string ShapeRef(in Shape S)
        {
            if (S.IsNull)
            {
                return "null";
            }
            string s = TypeNames[(int)S.Type] + " " + OriNames[(int)S.Orientation];
            int k = (int)S.Type < 8 ? (_inMaps[(int)S.Type].IndexOf(S) + 1) : 0;
            if (k > 0)
            {
                return s + " in " + I(k);
            }
            s += " new";
            if (S.Type == ShapeType.Vertex)
            {
                Point3D p = Tool.Pnt(ShapeCasts.Vertex(S));
                s += " " + R(p.X) + " " + R(p.Y) + " " + R(p.Z);
            }
            return s;
        }

        private void Collect(NCollection_List<DsInterference> l)
        {
            for (NCollection_List<DsInterference>.Iterator it = new(l); it.More(); it.Next())
            {
                Collect(it.Value());
            }
        }

        private void Collect(DsInterference? i)
        {
            if (i is null || _ids.ContainsKey(i))
            {
                return;
            }
            _order.Add(i);
            _ids[i] = _order.Count;
        }

        private string IdRef(DsInterference? i) => i is null ? "-" : "#" + I(_ids[i]);

        private void Geom3d(string tag, Curve3D? c)
        {
            if (c is null)
            {
                W(tag + " null\n");
                return;
            }
            W(tag + "\n");
            global::Kerf.Engine.Exchange.Brep.GeomTools.Write(c, _os);
        }

        private void Geom2d(string tag, Curve2D? c)
        {
            if (c is null)
            {
                W(tag + " null\n");
                return;
            }
            W(tag + "\n");
            global::Kerf.Engine.Exchange.Brep.GeomTools.Write(c, _os);
        }

        private void GeomS(string tag, Surface? s)
        {
            if (s is null)
            {
                W(tag + " null\n");
                return;
            }
            W(tag + "\n");
            global::Kerf.Engine.Exchange.Brep.GeomTools.Write(s, _os);
        }

        private void Interf(DsInterference i)
        {
            int id = _ids[i];
            if (!_written.Add(id))
            {
                W("R #" + I(id) + "\n");
                return;
            }
            Type type = i.GetType();
            string cls = type == typeof(DsCurvePointInterference) ? "CPI"
                : type == typeof(DsSurfaceCurveInterference) ? "SCI"
                : type == typeof(DsSolidSurfaceInterference) ? "SSI"
                : type == typeof(DsShapeShapeInterference) ? "SSHI"
                : type == typeof(DsEdgeVertexInterference) ? "EVI"
                : type == typeof(DsFaceEdgeInterference) ? "FEI"
                : type == typeof(DsInterference) ? "INTF"
                : "OTHER";
            W("I #" + I(id) + " " + cls + " " + TransitionText(i.Transition()) + " " + KindNames[(int)i.SupportType()] + " "
              + I(i.Support()) + " " + KindNames[(int)i.GeometryType()] + " " + I(i.Geometry()));
            switch (i)
            {
                case DsCurvePointInterference cpi:
                    W(" " + R(cpi.Parameter()) + "\n");
                    break;
                case DsSurfaceCurveInterference sci:
                    W("\n");
                    Geom2d("pcurve", sci.PCurve());
                    break;
                case DsEdgeVertexInterference evi:
                    W(" " + (evi.GBound() ? "1" : "0") + " " + ConfigNames[(int)evi.Config()] + " " + R(evi.Parameter()) + "\n");
                    break;
                case DsShapeShapeInterference sshi:
                    W(" " + (sshi.GBound() ? "1" : "0") + " " + ConfigNames[(int)sshi.Config()] + "\n");
                    break;
                default:
                    W("\n");
                    break;
            }
        }

        private void List(NCollection_List<DsInterference> l)
        {
            W("interferences " + I(l.Extent()) + "\n");
            for (NCollection_List<DsInterference>.Iterator it = new(l); it.More(); it.Next())
            {
                Interf(it.Value());
            }
        }

        internal string Run()
        {
            DsPointExplorer pex = new(_ds, false);
            DsCurveExplorer cex = new(_ds, false);
            DsSurfaceExplorer sex = new(_ds, false);
            int nS = _ds.NbShapes(), nP = _ds.NbPoints(), nC = _ds.NbCurves(), nF = _ds.NbSurfaces();

            // the interferences' identifiers, in the order the dump meets them
            for (int i = 1; i <= nS; i++)
            {
                Collect(_ds.ShapeInterferences(i, false));
            }
            for (int i = 1; i <= nP; i++)
            {
                Collect(_ds.PointInterferences(i));
            }
            for (int i = 1; i <= nC; i++)
            {
                Collect(_ds.CurveInterferences(i));
            }
            for (int i = 1; i <= nF; i++)
            {
                Collect(_ds.SurfaceInterferences(i));
            }
            for (int i = 1; i <= nC; i++)
            {
                if (cex.IsCurve(i))
                {
                    Collect(_ds.Curve(i).GetSCI1());
                    Collect(_ds.Curve(i).GetSCI2());
                }
            }

            W("dsdump 1\n");
            W("shapes " + I(nS) + "\n");
            for (int i = 1; i <= nS; i++)
            {
                Shape s = _ds.Shape(i, false);
                NCollection_List<Shape> sd = _ds.ShapeSameDomain(i);
                W("shape " + I(i) + " " + ShapeRef(s) + " keep " + (_ds.KeepShape(i) ? "1" : "0") + " sdref " + I(_ds.SameDomainRef(i))
                  + " sdori " + ConfigNames[(int)_ds.SameDomainOri(i)] + " sdind " + I(_ds.SameDomainInd(i)) + " rank "
                  + I(_ds.AncestorRank(i)) + " sd " + I(sd.Extent()));
                for (NCollection_List<Shape>.Iterator it = new(sd); it.More(); it.Next())
                {
                    W(" " + I(_ds.Shape(it.Value(), false)));
                }
                W("\n");
                List(_ds.ShapeInterferences(i, false));
            }

            W("points " + I(nP) + "\n");
            for (int i = 1; i <= nP; i++)
            {
                if (!pex.IsPoint(i))
                {
                    W("point " + I(i) + " unbound\n");
                    continue;
                }
                DsPoint p = _ds.Point(i);
                W("point " + I(i) + " " + R(p.Point().X) + " " + R(p.Point().Y) + " " + R(p.Point().Z) + " " + R(p.Tolerance())
                  + " keep " + (p.Keep() ? "1" : "0") + "\n");
                List(_ds.PointInterferences(i));
            }

            W("curves " + I(nC) + "\n");
            for (int i = 1; i <= nC; i++)
            {
                if (!cex.IsCurve(i))
                {
                    W("curve " + I(i) + " unbound\n");
                    continue;
                }
                DsCurve c = _ds.Curve(i);
                bool r = c.Range(out double f, out double l);
                W("curve " + I(i) + " " + R(c.Tolerance()) + " range " + (r ? "1" : "0") + " " + R(r ? f : 0.0) + " " + R(r ? l : 0.0)
                  + " keep " + (c.Keep() ? "1" : "0") + " mother " + I(c.Mother()) + " index " + I(c.DSIndex()) + " walk "
                  + (c.IsWalk ? "1" : "0") + " sci " + IdRef(c.GetSCI1()) + " " + IdRef(c.GetSCI2()) + "\n");
                W("s1 " + ShapeRef(c.Shape1()) + "\n");
                W("s2 " + ShapeRef(c.Shape2()) + "\n");
                Geom3d("c3d", c.Curve());
                List(_ds.CurveInterferences(i));
            }

            W("surfaces " + I(nF) + "\n");
            for (int i = 1; i <= nF; i++)
            {
                if (!sex.IsSurface(i))
                {
                    W("surface " + I(i) + " unbound\n");
                    continue;
                }
                DsSurface F = _ds.Surface(i);
                W("surface " + I(i) + " " + R(F.Tolerance()) + " keep " + (F.Keep() ? "1" : "0") + "\n");
                GeomS("surf", F.Surface());
                List(_ds.SurfaceInterferences(i));
            }

            // the interferences only the curves hold
            List<DsInterference> extra = [];
            for (int k = 0; k < _order.Count; k++)
            {
                if (!_written.Contains(k + 1))
                {
                    extra.Add(_order[k]);
                }
            }
            W("extra " + I(extra.Count) + "\n");
            foreach (DsInterference anI in extra)
            {
                Interf(anI);
            }

            // the new surfaces of faces: the input's faces in order, then the data structure's other faces
            List<Shape> faces = [];
            IndexedSet<Shape> inFaces = _inMaps[(int)ShapeType.Face];
            for (int k = 0; k < inFaces.Count; k++)
            {
                faces.Add(inFaces[k]);
            }
            for (int i = 1; i <= nS; i++)
            {
                Shape s = _ds.Shape(i, false);
                if (s.Type == ShapeType.Face && !inFaces.Contains(s))
                {
                    faces.Add(s);
                }
            }
            int nNew = 0;
            foreach (Shape F in faces)
            {
                if (_ds.HasNewSurface(F))
                {
                    nNew++;
                }
            }
            W("newsurfaces " + I(nNew) + "\n");
            foreach (Shape F in faces)
            {
                if (_ds.HasNewSurface(F))
                {
                    W("newsurface " + ShapeRef(F) + "\n");
                    GeomS("surf", _ds.NewSurface(F));
                }
            }

            W("sectionedges " + I(_ds.NbSectionEdges()) + "\n");
            for (int i = 1; i <= _ds.NbSectionEdges(); i++)
            {
                W("sectionedge " + ShapeRef(_ds.SectionEdge(i, false)) + "\n");
            }
            W("isfafa " + (_ds.Isfafa() ? "1" : "0") + "\n");
            W("end\n");
            return _os.ToString();
        }
    }

    /// <summary>
    /// A data structure rebuilt from its text <paramref name="text"/> (as <see cref="Write"/> or tools/dsrec writes
    /// it), its shapes taken from <paramref name="input"/>.
    /// </summary>
    /// <exception cref="FormatException">The text is not a dump, or names a shape that is not in the input ("new").</exception>
    internal static DsHDataStructure Read(string text, in Shape input)
    {
        Reader r = new(new StringReader(text), input);
        return r.Run();
    }

    private sealed class Reader
    {
        private readonly TextReader _is;
        private readonly IndexedSet<Shape>[] _inMaps = new IndexedSet<Shape>[9];
        private readonly Dictionary<int, DsInterference> _ids = [];
        private readonly DsHDataStructure _hds = new();
        private DsDataStructure DS => _hds.ChangeDS();

        internal Reader(TextReader theIS, in Shape input)
        {
            _is = theIS;
            for (int t = 0; t < 9; t++)
            {
                _inMaps[t] = new IndexedSet<Shape>(ShapeHasher.Instance);
                if (t < 8 && !input.IsNull)
                {
                    Exploration.MapShapes(input, (ShapeType)t, _inMaps[t]);
                }
            }
        }

        private string[] Line()
        {
            string? line;
            do
            {
                line = _is.ReadLine() ?? throw new FormatException("TopOpeBRepDS dump: unexpected end");
            }
            while (line.Trim().Length == 0);
            return line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
        }

        private string[] Line(string tag)
        {
            string[] t = Line();
            if (t[0] != tag)
            {
                throw new FormatException("TopOpeBRepDS dump: '" + tag + "' expected, '" + string.Join(' ', t) + "' found");
            }
            return t;
        }

        private static int Int(string s) => int.Parse(s, NumberStyles.Integer, CultureInfo.InvariantCulture);

        private static double Real(string s) => s switch
        {
            "inf" => double.PositiveInfinity,
            "-inf" => double.NegativeInfinity,
            _ => double.Parse(s, NumberStyles.Float, CultureInfo.InvariantCulture),
        };

        private static int Index(string[] names, string s)
        {
            int k = Array.IndexOf(names, s);
            return k >= 0 ? k : throw new FormatException("TopOpeBRepDS dump: unknown name '" + s + "'");
        }

        // a shape-ref starting at t[k]; returns the shape and the next token's index
        private Shape Shape(string[] t, int k, out int next)
        {
            if (t[k] == "null")
            {
                next = k + 1;
                return new Shape();
            }
            int type = Index(TypeNames, t[k]);
            Orientation ori = (Orientation)Index(OriNames, t[k + 1]);
            if (t[k + 2] != "in")
            {
                throw new FormatException("TopOpeBRepDS dump: a shape not in the input cannot be rebuilt");
            }
            next = k + 4;
            return _inMaps[type][Int(t[k + 3]) - 1].Oriented(ori);
        }

        private Curve3D? Curve3d(string tag)
        {
            string[] t = Line(tag);
            if (t.Length > 1)
            {
                return null;
            }
            global::Kerf.Engine.Exchange.Brep.GeomTools.Read(out Curve3D? c, _is);
            return c;
        }

        private Curve2D? Curve2d(string tag)
        {
            string[] t = Line(tag);
            if (t.Length > 1)
            {
                return null;
            }
            global::Kerf.Engine.Exchange.Brep.GeomTools.Read(out Curve2D? c, _is);
            return c;
        }

        private Surface? Surf(string tag)
        {
            string[] t = Line(tag);
            if (t.Length > 1)
            {
                return null;
            }
            global::Kerf.Engine.Exchange.Brep.GeomTools.Read(out Surface? s, _is);
            return s;
        }

        private DsInterference Interf()
        {
            string[] t = Line();
            if (t[0] == "R")
            {
                return _ids[Int(t[1][1..])];
            }
            if (t[0] != "I")
            {
                throw new FormatException("TopOpeBRepDS dump: interference expected, '" + string.Join(' ', t) + "' found");
            }
            int id = Int(t[1][1..]);
            string cls = t[2];
            string[] st = t[3].Split('/'), sh = t[4].Split('/'), ix = t[5].Split('/');
            DsTransition T = new((Containment)Index(StateNames, st[0]), (Containment)Index(StateNames, st[1]),
                                            (ShapeType)Index(TypeNames, sh[0]), (ShapeType)Index(TypeNames, sh[1]));
            T.IndexBefore(Int(ix[0]));
            T.IndexAfter(Int(ix[1]));
            DsKind sk = (DsKind)Index(KindNames, t[6]);
            int s = Int(t[7]);
            DsKind gk = (DsKind)Index(KindNames, t[8]);
            int g = Int(t[9]);
            DsInterference i;
            switch (cls)
            {
                case "CPI":
                    i = new DsCurvePointInterference(T, sk, s, gk, g, Real(t[10]));
                    break;
                case "SCI":
                    i = new DsSurfaceCurveInterference(T, sk, s, gk, g, Curve2d("pcurve"));
                    break;
                case "SSI":
                    i = new DsSolidSurfaceInterference(T, sk, s, gk, g);
                    break;
                case "SSHI":
                    i = new DsShapeShapeInterference(T, sk, s, gk, g, t[10] == "1",
                                                                  (DsConfig)Index(ConfigNames, t[11]));
                    break;
                case "EVI":
                    i = new DsEdgeVertexInterference(T, sk, s, g, t[10] == "1",
                                                                  (DsConfig)Index(ConfigNames, t[11]), Real(t[12]));
                    i.GeometryType(gk);
                    break;
                case "FEI":
                    i = new DsFaceEdgeInterference(T, s, g, t[10] == "1", (DsConfig)Index(ConfigNames, t[11]));
                    i.SupportType(sk);
                    i.GeometryType(gk);
                    break;
                case "INTF":
                    i = new DsInterference(T, sk, s, gk, g);
                    break;
                default:
                    throw new FormatException("TopOpeBRepDS dump: interference class '" + cls + "' cannot be rebuilt");
            }
            _ids[id] = i;
            return i;
        }

        private void List(NCollection_List<DsInterference> l)
        {
            int n = Int(Line("interferences")[1]);
            for (int k = 0; k < n; k++)
            {
                l.Append(Interf());
            }
        }

        internal DsHDataStructure Run()
        {
            if (Line("dsdump")[1] != "1")
            {
                throw new FormatException("TopOpeBRepDS dump: unknown version");
            }

            // shapes: added first, so that the same-domain lists can name later ones
            int nS = Int(Line("shapes")[1]);
            List<string[]> shapeLines = [];
            List<NCollection_List<DsInterference>> shapeLists = [];
            for (int i = 1; i <= nS; i++)
            {
                string[] t = Line("shape");
                Shape s = Shape(t, 2, out int k);
                if (DS.AddShape(s) != i)
                {
                    throw new FormatException("TopOpeBRepDS dump: shape " + I(i) + " is not new");
                }
                shapeLines.Add(t[k..]);
                NCollection_List<DsInterference> l = new();
                List(l);
                shapeLists.Add(l);
            }
            for (int i = 1; i <= nS; i++)
            {
                string[] t = shapeLines[i - 1]; // keep k sdref r sdori o sdind k rank k sd n ...
                DS.ChangeKeepShape(i, t[1] == "1");
                DS.SameDomainRef(i, Int(t[3]));
                DS.SameDomainOri(i, (DsConfig)Index(ConfigNames, t[5]));
                DS.SameDomainInd(i, Int(t[7]));
                DS.AncestorRank(i, Int(t[9]));
                int nSD = Int(t[11]);
                for (int j = 0; j < nSD; j++)
                {
                    DS.ChangeShapeSameDomain(i).Append(DS.Shape(Int(t[12 + j]), false));
                }
                DS.ChangeShapeInterferences(i).Append(shapeLists[i - 1]);
            }

            int nP = Int(Line("points")[1]);
            for (int i = 1; i <= nP; i++)
            {
                string[] t = Line("point");
                if (t[2] == "unbound")
                {
                    DS.AddPoint(new DsPoint());
                    DS._points.Remove(i);
                    continue;
                }
                DsPoint p = new(new Point3D(Real(t[2]), Real(t[3]), Real(t[4])), Real(t[5]));
                p.ChangeKeep(t[7] == "1");
                DS.AddPoint(p);
                List(DS.ChangePointInterferences(i));
            }

            int nC = Int(Line("curves")[1]);
            List<(int, string, string)> scIs = [];
            for (int i = 1; i <= nC; i++)
            {
                string[] t = Line("curve");
                if (t[2] == "unbound")
                {
                    DS.AddCurve(new DsCurve());
                    DS._curves.Remove(i);
                    continue;
                }
                // curve i tol range r f l keep k mother m index x walk w sci a b
                Shape s1 = Shape(Line("s1"), 1, out _);
                Shape s2 = Shape(Line("s2"), 1, out _);
                DsCurve c = new(Curve3d("c3d"), Real(t[2]), t[14] == "1");
                if (t[4] == "1")
                {
                    c.SetRange(Real(t[5]), Real(t[6]));
                }
                c.ChangeKeep(t[8] == "1");
                c.ChangeMother(Int(t[10]));
                c.SetShapes(s1, s2);
                DS.AddCurve(ref c);
                DS.ChangeCurve(i).ChangeDSIndex(Int(t[12]));
                scIs.Add((i, t[16], t[17]));
                List(DS.ChangeCurveInterferences(i));
            }

            int nF = Int(Line("surfaces")[1]);
            for (int i = 1; i <= nF; i++)
            {
                string[] t = Line("surface");
                if (t[2] == "unbound")
                {
                    DS.AddSurface(new DsSurface());
                    DS.RemoveSurface(i);
                    continue;
                }
                DsSurface f = new(Surf("surf"), Real(t[2]));
                f.ChangeKeep(t[4] == "1");
                DS.AddSurface(f);
                List(DS.ChangeSurfaceInterferences(i));
            }

            int nX = Int(Line("extra")[1]);
            for (int k = 0; k < nX; k++)
            {
                Interf();
            }
            foreach ((int i, string a, string b) in scIs)
            {
                DS.ChangeCurve(i).SetSCI(a == "-" ? null : _ids[Int(a[1..])], b == "-" ? null : _ids[Int(b[1..])]);
            }

            int nN = Int(Line("newsurfaces")[1]);
            for (int k = 0; k < nN; k++)
            {
                Shape f = Shape(Line("newsurface"), 1, out _);
                DS.SetNewSurface(f, Surf("surf")!);
            }

            int nE = Int(Line("sectionedges")[1]);
            for (int k = 0; k < nE; k++)
            {
                DS.AddSectionEdge(ShapeCasts.Edge(Shape(Line("sectionedge"), 1, out _)));
            }
            DS.Isfafa(Line("isfafa")[1] == "1");
            Line("end");
            return _hds;
        }
    }
}
