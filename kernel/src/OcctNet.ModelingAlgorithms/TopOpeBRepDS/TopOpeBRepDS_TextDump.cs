// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// OCCT.NET addition (unit 7L): not a port of an OCCT class. tools/dsrec/main.cpp writes native OCCT's data structure in
// the same format; keep the two in step.

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using System.Globalization;
using static Kerf.Topology.ShapeType;

namespace Occt;

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
internal static class TopOpeBRepDS_TextDump
{
    private static readonly string[] theTypeNames = ["COMPOUND", "COMPSOLID", "SOLID", "SHELL", "FACE", "WIRE", "EDGE", "VERTEX", "SHAPE"];
    private static readonly string[] theOriNames = ["FORWARD", "REVERSED", "INTERNAL", "EXTERNAL"];
    private static readonly string[] theStateNames = ["IN", "OUT", "ON", "UNKNOWN"];
    private static readonly string[] theKindNames = ["POINT", "CURVE", "SURFACE", "VERTEX", "EDGE", "WIRE", "FACE", "SHELL", "SOLID", "COMPSOLID", "COMPOUND", "UNKNOWN"];
    private static readonly string[] theConfigNames = ["UNSH", "SAME", "DIFF"];

    private static string R(double v) => StreamUtilities.FormatReal(v, 17);

    private static string I(int v) => v.ToString(CultureInfo.InvariantCulture);

    /// <summary>The transition as the dump writes it: <c>OUT/IN FACE/FACE 0/0</c>.</summary>
    internal static string TransitionText(in TopOpeBRepDS_Transition T)
        => theStateNames[(int)T.Before()] + "/" + theStateNames[(int)T.After()] + " " + theTypeNames[(int)T.ShapeBefore()] + "/"
           + theTypeNames[(int)T.ShapeAfter()] + " " + I(T.IndexBefore()) + "/" + I(T.IndexAfter());

    /// <summary>The text of the data structure <paramref name="DS"/>, whose shapes are referred to <paramref name="theInput"/>'s.</summary>
    internal static string Write(TopOpeBRepDS_DataStructure DS, in Shape theInput)
    {
        Writer aW = new(DS, theInput);
        return aW.Run();
    }

    private sealed class Writer
    {
        private readonly TopOpeBRepDS_DataStructure DS;
        private readonly StringWriter OS = new(CultureInfo.InvariantCulture) { NewLine = "\n" };
        private readonly Dictionary<TopOpeBRepDS_Interference, int> Ids = new(ReferenceEqualityComparer.Instance);
        private readonly List<TopOpeBRepDS_Interference> Order = [];
        private readonly HashSet<int> Written = [];
        private readonly NCollection_IndexedMap<Shape>[] InMaps = new NCollection_IndexedMap<Shape>[9];

        internal Writer(TopOpeBRepDS_DataStructure theDS, in Shape theInput)
        {
            DS = theDS;
            StreamUtilities.Precision(OS, 17);
            for (int t = 0; t < 9; t++)
            {
                InMaps[t] = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
                if (t < 8 && !theInput.IsNull)
                {
                    Exploration.MapShapes(theInput, (ShapeType)t, InMaps[t]);
                }
            }
        }

        private void W(string s) => OS.Write(s);

        private string ShapeRef(in Shape S)
        {
            if (S.IsNull)
            {
                return "null";
            }
            string s = theTypeNames[(int)S.ShapeType()] + " " + theOriNames[(int)S.Orientation()];
            int k = (int)S.ShapeType() < 8 ? InMaps[(int)S.ShapeType()].FindIndex(S) : 0;
            if (k > 0)
            {
                return s + " in " + I(k);
            }
            s += " new";
            if (S.ShapeType() == ShapeType.Vertex)
            {
                Point3D P = Tool.Pnt(ShapeCasts.Vertex(S));
                s += " " + R(P.X()) + " " + R(P.Y()) + " " + R(P.Z());
            }
            return s;
        }

        private void Collect(NCollection_List<TopOpeBRepDS_Interference> L)
        {
            for (NCollection_List<TopOpeBRepDS_Interference>.Iterator it = new(L); it.More(); it.Next())
            {
                Collect(it.Value());
            }
        }

        private void Collect(TopOpeBRepDS_Interference? theI)
        {
            if (theI is null || Ids.ContainsKey(theI))
            {
                return;
            }
            Order.Add(theI);
            Ids[theI] = Order.Count;
        }

        private string IdRef(TopOpeBRepDS_Interference? theI) => theI is null ? "-" : "#" + I(Ids[theI]);

        private void Geom3d(string theTag, Curve3D? C)
        {
            if (C is null)
            {
                W(theTag + " null\n");
                return;
            }
            W(theTag + "\n");
            global::Kerf.Engine.Exchange.Brep.GeomTools.Write(C, OS);
        }

        private void Geom2d(string theTag, Curve2D? C)
        {
            if (C is null)
            {
                W(theTag + " null\n");
                return;
            }
            W(theTag + "\n");
            global::Kerf.Engine.Exchange.Brep.GeomTools.Write(C, OS);
        }

        private void GeomS(string theTag, Surface? S)
        {
            if (S is null)
            {
                W(theTag + " null\n");
                return;
            }
            W(theTag + "\n");
            global::Kerf.Engine.Exchange.Brep.GeomTools.Write(S, OS);
        }

        private void Interf(TopOpeBRepDS_Interference theI)
        {
            int id = Ids[theI];
            if (!Written.Add(id))
            {
                W("R #" + I(id) + "\n");
                return;
            }
            Type aType = theI.GetType();
            string cls = aType == typeof(TopOpeBRepDS_CurvePointInterference) ? "CPI"
                : aType == typeof(TopOpeBRepDS_SurfaceCurveInterference) ? "SCI"
                : aType == typeof(TopOpeBRepDS_SolidSurfaceInterference) ? "SSI"
                : aType == typeof(TopOpeBRepDS_ShapeShapeInterference) ? "SSHI"
                : aType == typeof(TopOpeBRepDS_EdgeVertexInterference) ? "EVI"
                : aType == typeof(TopOpeBRepDS_FaceEdgeInterference) ? "FEI"
                : aType == typeof(TopOpeBRepDS_Interference) ? "INTF"
                : "OTHER";
            W("I #" + I(id) + " " + cls + " " + TransitionText(theI.Transition()) + " " + theKindNames[(int)theI.SupportType()] + " "
              + I(theI.Support()) + " " + theKindNames[(int)theI.GeometryType()] + " " + I(theI.Geometry()));
            switch (theI)
            {
                case TopOpeBRepDS_CurvePointInterference aCPI:
                    W(" " + R(aCPI.Parameter()) + "\n");
                    break;
                case TopOpeBRepDS_SurfaceCurveInterference aSCI:
                    W("\n");
                    Geom2d("pcurve", aSCI.PCurve());
                    break;
                case TopOpeBRepDS_EdgeVertexInterference anEVI:
                    W(" " + (anEVI.GBound() ? "1" : "0") + " " + theConfigNames[(int)anEVI.Config()] + " " + R(anEVI.Parameter()) + "\n");
                    break;
                case TopOpeBRepDS_ShapeShapeInterference aSSHI:
                    W(" " + (aSSHI.GBound() ? "1" : "0") + " " + theConfigNames[(int)aSSHI.Config()] + "\n");
                    break;
                default:
                    W("\n");
                    break;
            }
        }

        private void List(NCollection_List<TopOpeBRepDS_Interference> L)
        {
            W("interferences " + I(L.Extent()) + "\n");
            for (NCollection_List<TopOpeBRepDS_Interference>.Iterator it = new(L); it.More(); it.Next())
            {
                Interf(it.Value());
            }
        }

        internal string Run()
        {
            TopOpeBRepDS_PointExplorer pex = new(DS, false);
            TopOpeBRepDS_CurveExplorer cex = new(DS, false);
            TopOpeBRepDS_SurfaceExplorer sex = new(DS, false);
            int nS = DS.NbShapes(), nP = DS.NbPoints(), nC = DS.NbCurves(), nF = DS.NbSurfaces();

            // the interferences' identifiers, in the order the dump meets them
            for (int i = 1; i <= nS; i++)
            {
                Collect(DS.ShapeInterferences(i, false));
            }
            for (int i = 1; i <= nP; i++)
            {
                Collect(DS.PointInterferences(i));
            }
            for (int i = 1; i <= nC; i++)
            {
                Collect(DS.CurveInterferences(i));
            }
            for (int i = 1; i <= nF; i++)
            {
                Collect(DS.SurfaceInterferences(i));
            }
            for (int i = 1; i <= nC; i++)
            {
                if (cex.IsCurve(i))
                {
                    Collect(DS.Curve(i).GetSCI1());
                    Collect(DS.Curve(i).GetSCI2());
                }
            }

            W("dsdump 1\n");
            W("shapes " + I(nS) + "\n");
            for (int i = 1; i <= nS; i++)
            {
                Shape S = DS.Shape(i, false);
                NCollection_List<Shape> SD = DS.ShapeSameDomain(i);
                W("shape " + I(i) + " " + ShapeRef(S) + " keep " + (DS.KeepShape(i) ? "1" : "0") + " sdref " + I(DS.SameDomainRef(i))
                  + " sdori " + theConfigNames[(int)DS.SameDomainOri(i)] + " sdind " + I(DS.SameDomainInd(i)) + " rank "
                  + I(DS.AncestorRank(i)) + " sd " + I(SD.Extent()));
                for (NCollection_List<Shape>.Iterator it = new(SD); it.More(); it.Next())
                {
                    W(" " + I(DS.Shape(it.Value(), false)));
                }
                W("\n");
                List(DS.ShapeInterferences(i, false));
            }

            W("points " + I(nP) + "\n");
            for (int i = 1; i <= nP; i++)
            {
                if (!pex.IsPoint(i))
                {
                    W("point " + I(i) + " unbound\n");
                    continue;
                }
                TopOpeBRepDS_Point P = DS.Point(i);
                W("point " + I(i) + " " + R(P.Point().X()) + " " + R(P.Point().Y()) + " " + R(P.Point().Z()) + " " + R(P.Tolerance())
                  + " keep " + (P.Keep() ? "1" : "0") + "\n");
                List(DS.PointInterferences(i));
            }

            W("curves " + I(nC) + "\n");
            for (int i = 1; i <= nC; i++)
            {
                if (!cex.IsCurve(i))
                {
                    W("curve " + I(i) + " unbound\n");
                    continue;
                }
                TopOpeBRepDS_Curve C = DS.Curve(i);
                bool r = C.Range(out double f, out double l);
                W("curve " + I(i) + " " + R(C.Tolerance()) + " range " + (r ? "1" : "0") + " " + R(r ? f : 0.0) + " " + R(r ? l : 0.0)
                  + " keep " + (C.Keep() ? "1" : "0") + " mother " + I(C.Mother()) + " index " + I(C.DSIndex()) + " walk "
                  + (C.IsWalk() ? "1" : "0") + " sci " + IdRef(C.GetSCI1()) + " " + IdRef(C.GetSCI2()) + "\n");
                W("s1 " + ShapeRef(C.Shape1()) + "\n");
                W("s2 " + ShapeRef(C.Shape2()) + "\n");
                Geom3d("c3d", C.Curve());
                List(DS.CurveInterferences(i));
            }

            W("surfaces " + I(nF) + "\n");
            for (int i = 1; i <= nF; i++)
            {
                if (!sex.IsSurface(i))
                {
                    W("surface " + I(i) + " unbound\n");
                    continue;
                }
                TopOpeBRepDS_Surface F = DS.Surface(i);
                W("surface " + I(i) + " " + R(F.Tolerance()) + " keep " + (F.Keep() ? "1" : "0") + "\n");
                GeomS("surf", F.Surface());
                List(DS.SurfaceInterferences(i));
            }

            // the interferences only the curves hold
            List<TopOpeBRepDS_Interference> anExtra = [];
            for (int k = 0; k < Order.Count; k++)
            {
                if (!Written.Contains(k + 1))
                {
                    anExtra.Add(Order[k]);
                }
            }
            W("extra " + I(anExtra.Count) + "\n");
            foreach (TopOpeBRepDS_Interference anI in anExtra)
            {
                Interf(anI);
            }

            // the new surfaces of faces: the input's faces in order, then the data structure's other faces
            List<Shape> aFaces = [];
            NCollection_IndexedMap<Shape> aInFaces = InMaps[(int)ShapeType.Face];
            for (int k = 1; k <= aInFaces.Extent(); k++)
            {
                aFaces.Add(aInFaces.FindKey(k));
            }
            for (int i = 1; i <= nS; i++)
            {
                Shape S = DS.Shape(i, false);
                if (S.ShapeType() == ShapeType.Face && aInFaces.FindIndex(S) == 0)
                {
                    aFaces.Add(S);
                }
            }
            int nNew = 0;
            foreach (Shape F in aFaces)
            {
                if (DS.HasNewSurface(F))
                {
                    nNew++;
                }
            }
            W("newsurfaces " + I(nNew) + "\n");
            foreach (Shape F in aFaces)
            {
                if (DS.HasNewSurface(F))
                {
                    W("newsurface " + ShapeRef(F) + "\n");
                    GeomS("surf", DS.NewSurface(F));
                }
            }

            W("sectionedges " + I(DS.NbSectionEdges()) + "\n");
            for (int i = 1; i <= DS.NbSectionEdges(); i++)
            {
                W("sectionedge " + ShapeRef(DS.SectionEdge(i, false)) + "\n");
            }
            W("isfafa " + (DS.Isfafa() ? "1" : "0") + "\n");
            W("end\n");
            return OS.ToString();
        }
    }

    /// <summary>
    /// A data structure rebuilt from its text <paramref name="theText"/> (as <see cref="Write"/> or tools/dsrec writes
    /// it), its shapes taken from <paramref name="theInput"/>.
    /// </summary>
    /// <exception cref="FormatException">The text is not a dump, or names a shape that is not in the input ("new").</exception>
    internal static TopOpeBRepDS_HDataStructure Read(string theText, in Shape theInput)
    {
        Reader aR = new(new StringReader(theText), theInput);
        return aR.Run();
    }

    private sealed class Reader
    {
        private readonly TextReader IS;
        private readonly NCollection_IndexedMap<Shape>[] InMaps = new NCollection_IndexedMap<Shape>[9];
        private readonly Dictionary<int, TopOpeBRepDS_Interference> Ids = [];
        private readonly TopOpeBRepDS_HDataStructure HDS = new();
        private TopOpeBRepDS_DataStructure DS => HDS.ChangeDS();

        internal Reader(TextReader theIS, in Shape theInput)
        {
            IS = theIS;
            for (int t = 0; t < 9; t++)
            {
                InMaps[t] = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
                if (t < 8 && !theInput.IsNull)
                {
                    Exploration.MapShapes(theInput, (ShapeType)t, InMaps[t]);
                }
            }
        }

        private string[] Line()
        {
            string? aLine;
            do
            {
                aLine = IS.ReadLine() ?? throw new FormatException("TopOpeBRepDS dump: unexpected end");
            }
            while (aLine.Trim().Length == 0);
            return aLine.Split(' ', StringSplitOptions.RemoveEmptyEntries);
        }

        private string[] Line(string theTag)
        {
            string[] t = Line();
            if (t[0] != theTag)
            {
                throw new FormatException("TopOpeBRepDS dump: '" + theTag + "' expected, '" + string.Join(' ', t) + "' found");
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

        private static int Index(string[] theNames, string s)
        {
            int k = Array.IndexOf(theNames, s);
            return k >= 0 ? k : throw new FormatException("TopOpeBRepDS dump: unknown name '" + s + "'");
        }

        // a shape-ref starting at t[k]; returns the shape and the next token's index
        private Shape Shape(string[] t, int k, out int theNext)
        {
            if (t[k] == "null")
            {
                theNext = k + 1;
                return new Shape();
            }
            int aType = Index(theTypeNames, t[k]);
            Orientation anOri = (Orientation)Index(theOriNames, t[k + 1]);
            if (t[k + 2] != "in")
            {
                throw new FormatException("TopOpeBRepDS dump: a shape not in the input cannot be rebuilt");
            }
            theNext = k + 4;
            return InMaps[aType].FindKey(Int(t[k + 3])).Oriented(anOri);
        }

        private Curve3D? Curve3d(string theTag)
        {
            string[] t = Line(theTag);
            if (t.Length > 1)
            {
                return null;
            }
            global::Kerf.Engine.Exchange.Brep.GeomTools.Read(out Curve3D? C, IS);
            return C;
        }

        private Curve2D? Curve2d(string theTag)
        {
            string[] t = Line(theTag);
            if (t.Length > 1)
            {
                return null;
            }
            global::Kerf.Engine.Exchange.Brep.GeomTools.Read(out Curve2D? C, IS);
            return C;
        }

        private Surface? Surf(string theTag)
        {
            string[] t = Line(theTag);
            if (t.Length > 1)
            {
                return null;
            }
            global::Kerf.Engine.Exchange.Brep.GeomTools.Read(out Surface? S, IS);
            return S;
        }

        private TopOpeBRepDS_Interference Interf()
        {
            string[] t = Line();
            if (t[0] == "R")
            {
                return Ids[Int(t[1][1..])];
            }
            if (t[0] != "I")
            {
                throw new FormatException("TopOpeBRepDS dump: interference expected, '" + string.Join(' ', t) + "' found");
            }
            int id = Int(t[1][1..]);
            string cls = t[2];
            string[] st = t[3].Split('/'), sh = t[4].Split('/'), ix = t[5].Split('/');
            TopOpeBRepDS_Transition T = new((Containment)Index(theStateNames, st[0]), (Containment)Index(theStateNames, st[1]),
                                            (ShapeType)Index(theTypeNames, sh[0]), (ShapeType)Index(theTypeNames, sh[1]));
            T.IndexBefore(Int(ix[0]));
            T.IndexAfter(Int(ix[1]));
            TopOpeBRepDS_Kind SK = (TopOpeBRepDS_Kind)Index(theKindNames, t[6]);
            int S = Int(t[7]);
            TopOpeBRepDS_Kind GK = (TopOpeBRepDS_Kind)Index(theKindNames, t[8]);
            int G = Int(t[9]);
            TopOpeBRepDS_Interference anI;
            switch (cls)
            {
                case "CPI":
                    anI = new TopOpeBRepDS_CurvePointInterference(T, SK, S, GK, G, Real(t[10]));
                    break;
                case "SCI":
                    anI = new TopOpeBRepDS_SurfaceCurveInterference(T, SK, S, GK, G, Curve2d("pcurve"));
                    break;
                case "SSI":
                    anI = new TopOpeBRepDS_SolidSurfaceInterference(T, SK, S, GK, G);
                    break;
                case "SSHI":
                    anI = new TopOpeBRepDS_ShapeShapeInterference(T, SK, S, GK, G, t[10] == "1",
                                                                  (TopOpeBRepDS_Config)Index(theConfigNames, t[11]));
                    break;
                case "EVI":
                    anI = new TopOpeBRepDS_EdgeVertexInterference(T, SK, S, G, t[10] == "1",
                                                                  (TopOpeBRepDS_Config)Index(theConfigNames, t[11]), Real(t[12]));
                    anI.GeometryType(GK);
                    break;
                case "FEI":
                    anI = new TopOpeBRepDS_FaceEdgeInterference(T, S, G, t[10] == "1", (TopOpeBRepDS_Config)Index(theConfigNames, t[11]));
                    anI.SupportType(SK);
                    anI.GeometryType(GK);
                    break;
                case "INTF":
                    anI = new TopOpeBRepDS_Interference(T, SK, S, GK, G);
                    break;
                default:
                    throw new FormatException("TopOpeBRepDS dump: interference class '" + cls + "' cannot be rebuilt");
            }
            Ids[id] = anI;
            return anI;
        }

        private void List(NCollection_List<TopOpeBRepDS_Interference> L)
        {
            int n = Int(Line("interferences")[1]);
            for (int k = 0; k < n; k++)
            {
                L.Append(Interf());
            }
        }

        internal TopOpeBRepDS_HDataStructure Run()
        {
            if (Line("dsdump")[1] != "1")
            {
                throw new FormatException("TopOpeBRepDS dump: unknown version");
            }

            // shapes: added first, so that the same-domain lists can name later ones
            int nS = Int(Line("shapes")[1]);
            List<string[]> aShapeLines = [];
            List<NCollection_List<TopOpeBRepDS_Interference>> aShapeLists = [];
            for (int i = 1; i <= nS; i++)
            {
                string[] t = Line("shape");
                Shape S = Shape(t, 2, out int k);
                if (DS.AddShape(S) != i)
                {
                    throw new FormatException("TopOpeBRepDS dump: shape " + I(i) + " is not new");
                }
                aShapeLines.Add(t[k..]);
                NCollection_List<TopOpeBRepDS_Interference> L = new();
                List(L);
                aShapeLists.Add(L);
            }
            for (int i = 1; i <= nS; i++)
            {
                string[] t = aShapeLines[i - 1]; // keep k sdref r sdori o sdind k rank k sd n ...
                DS.ChangeKeepShape(i, t[1] == "1");
                DS.SameDomainRef(i, Int(t[3]));
                DS.SameDomainOri(i, (TopOpeBRepDS_Config)Index(theConfigNames, t[5]));
                DS.SameDomainInd(i, Int(t[7]));
                DS.AncestorRank(i, Int(t[9]));
                int nSD = Int(t[11]);
                for (int j = 0; j < nSD; j++)
                {
                    DS.ChangeShapeSameDomain(i).Append(DS.Shape(Int(t[12 + j]), false));
                }
                DS.ChangeShapeInterferences(i).Append(aShapeLists[i - 1]);
            }

            int nP = Int(Line("points")[1]);
            for (int i = 1; i <= nP; i++)
            {
                string[] t = Line("point");
                if (t[2] == "unbound")
                {
                    DS.AddPoint(new TopOpeBRepDS_Point());
                    DS.myPoints.UnBind(i);
                    continue;
                }
                TopOpeBRepDS_Point P = new(new Point3D(Real(t[2]), Real(t[3]), Real(t[4])), Real(t[5]));
                P.ChangeKeep(t[7] == "1");
                DS.AddPoint(P);
                List(DS.ChangePointInterferences(i));
            }

            int nC = Int(Line("curves")[1]);
            List<(int, string, string)> aSCIs = [];
            for (int i = 1; i <= nC; i++)
            {
                string[] t = Line("curve");
                if (t[2] == "unbound")
                {
                    DS.AddCurve(new TopOpeBRepDS_Curve());
                    DS.myCurves.UnBind(i);
                    continue;
                }
                // curve i tol range r f l keep k mother m index x walk w sci a b
                Shape S1 = Shape(Line("s1"), 1, out _);
                Shape S2 = Shape(Line("s2"), 1, out _);
                TopOpeBRepDS_Curve C = new(Curve3d("c3d"), Real(t[2]), t[14] == "1");
                if (t[4] == "1")
                {
                    C.SetRange(Real(t[5]), Real(t[6]));
                }
                C.ChangeKeep(t[8] == "1");
                C.ChangeMother(Int(t[10]));
                C.SetShapes(S1, S2);
                DS.AddCurve(ref C);
                DS.ChangeCurve(i).ChangeDSIndex(Int(t[12]));
                aSCIs.Add((i, t[16], t[17]));
                List(DS.ChangeCurveInterferences(i));
            }

            int nF = Int(Line("surfaces")[1]);
            for (int i = 1; i <= nF; i++)
            {
                string[] t = Line("surface");
                if (t[2] == "unbound")
                {
                    DS.AddSurface(new TopOpeBRepDS_Surface());
                    DS.RemoveSurface(i);
                    continue;
                }
                TopOpeBRepDS_Surface F = new(Surf("surf"), Real(t[2]));
                F.ChangeKeep(t[4] == "1");
                DS.AddSurface(F);
                List(DS.ChangeSurfaceInterferences(i));
            }

            int nX = Int(Line("extra")[1]);
            for (int k = 0; k < nX; k++)
            {
                Interf();
            }
            foreach ((int i, string a, string b) in aSCIs)
            {
                DS.ChangeCurve(i).SetSCI(a == "-" ? null : Ids[Int(a[1..])], b == "-" ? null : Ids[Int(b[1..])]);
            }

            int nN = Int(Line("newsurfaces")[1]);
            for (int k = 0; k < nN; k++)
            {
                Shape F = Shape(Line("newsurface"), 1, out _);
                DS.SetNewSurface(F, Surf("surf")!);
            }

            int nE = Int(Line("sectionedges")[1]);
            for (int k = 0; k < nE; k++)
            {
                DS.AddSectionEdge(ShapeCasts.Edge(Shape(Line("sectionedge"), 1, out _)));
            }
            DS.Isfafa(Line("isfafa")[1] == "1");
            Line("end");
            return HDS;
        }
    }
}
