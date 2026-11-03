// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_Gluer.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_Gluer.lxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_Gluer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Measurement.Distance;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Extrema.ExtFlag;
using static Kerf.Geometry.Continuity;
using static Kerf.Engine.Features.Local.FeatureOperation;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Features.Local;

/// <summary>
/// Glues a new shape on a basis shape by some of its faces, which lie on faces of the basis shape (a fuse or a cut,
/// depending on the faces' orientations).
/// </summary>
/// <remarks>
/// <para>
/// As in OCCT, it writes into both shapes: it splits the basis shape (point representations of its vertices), adds
/// pcurves to the new shape's edges, and sets the continuities of the edges. So <see cref="Perform"/> splits, generates
/// and recodes the regularities in one working overlay (decision 1013, unit 10H5: <see cref="WorkingShapes"/>), and the
/// result holds new TShapes only for the caller's sub-shapes that changed. The shapes it hands out are frozen; ported
/// callers (BRepFeat) read the <c>Internal</c> forms.
/// </para>
/// </remarks>
public sealed class LocalGluer
{
    private bool _done;
    private Shape _sb = new();
    private Shape _sn = new();
    private Shape _res = new();
    private Orientation _ori;
    private FeatureOperation _ope;
    private readonly NCollection_IndexedDataMap<Shape, Shape> _mapEF = new(ShapeHasher.Instance);
    // OCCT iterates this map keyed by the edges' addresses; the port in insertion order
    private readonly NCollection_DataMap<Shape, Shape> _mapEE = new(ShapeHasher.Instance);
    private readonly NCollection_DataMap<Shape, NCollection_List<Shape>> _descF = new(ShapeHasher.Instance);
    private NCollection_List<Shape> _edges = new();
    private NCollection_List<Shape> _tgtEdges = new();
    private WorkingShapes _work = new();

    public LocalGluer()
    {
        _done = false;
        _ori = Internal;
        _ope = FeatureOperation.Invalid;
    }

    public LocalGluer(in Shape sbase, in Shape snew)
    {
        _done = false;
        _sb = sbase;
        _sn = snew;
        _ori = Internal;
        _ope = FeatureOperation.Invalid;
    }

    public void Init(in Shape sbase, in Shape snew)
    {
        _sb = sbase;
        _sn = snew;
        _mapEF.Clear();
        _mapEE.Clear();
        _descF.Clear();
        _done = false;
        _ori = Internal;
        _ope = FeatureOperation.Invalid;
    }

    /// <summary>Binds the face <paramref name="fnew"/> of the new shape to the face <paramref name="fbase"/> of the basis shape it lies on.</summary>
    public void Bind(in Face fnew, in Face fbase)
    {
        Explorer exp = new(_sn, ShapeType.Face);

        for (; exp.More(); exp.Next())
        {
            if (exp.Current().IsSame(fnew))
            {
                break;
            }
        }
        if (!exp.More())
        {
            throw new ConstructionErrorException();
        }

        Shape localFace = fnew.Oriented(exp.Current().Orientation());
        Face fnor = ShapeCasts.Face(localFace);

        for (exp.Init(_sb, ShapeType.Face); exp.More(); exp.Next())
        {
            if (exp.Current().IsSame(fbase))
            {
                break;
            }
        }
        if (!exp.More())
        {
            throw new ConstructionErrorException();
        }

        localFace = fbase.Oriented(exp.Current().Orientation());
        Face fbor = ShapeCasts.Face(localFace);
        Orientation ori = GetOrientation(fnor, fbor);

        if (_ori == Internal)
        {
            _ori = ori;
            if (_ori == Reversed)
            {
                _sn.Reverse();
                _ope = FeatureOperation.Cut;
            }
            else
            {
                _ope = FeatureOperation.Fuse;
            }
        }
        else if (ori != Forward)
        {
            _ope = FeatureOperation.Invalid;
        }

        for (exp.Init(fnor, ShapeType.Edge); exp.More(); exp.Next())
        {
            Edge edg = ShapeCasts.Edge(exp.Current());
            if (!_mapEF.Contains(edg))
            {
                _mapEF.Add(edg, fbor);
            }
            else if (!_mapEF.FindFromKey(edg).IsSame(fbor))
            {
                _mapEF.ChangeFromKey(edg).Nullify();
                // edg sur 2 face. a binder avec l`edge commun
            }
        }
        _mapEF.Add(fnor, fbor);
    }

    /// <summary>Binds the edge <paramref name="enew"/> of the new shape to the edge <paramref name="ebase"/> of the basis shape.</summary>
    public void Bind(in Edge enew, in Edge ebase)
    {
        if (_mapEE.IsBound(enew) && !_mapEE[enew].IsSame(ebase))
        {
            throw new ConstructionErrorException();
        }
        _mapEE.Bind(enew, ebase);
    }

    public FeatureOperation OpeType() => _ope;

    public void Perform()
    {
        int ind;
        if (_done)
        {
            return;
        }
        if (_sb.IsNull || _sn.IsNull || _mapEF.IsEmpty() || _ope == FeatureOperation.Invalid)
        {
            throw new ConstructionErrorException();
        }

        // Kerf: the basis shape and the new shape are written in the working overlay, shared with the wires on the shape
        // and the generator (unit 10H5)
        _work = new WorkingShapes();
        using WorkingOverlay.Scope scope = _work.Enter();
        Shape sb = _sb;
        Shape sn = _sn;

        WiresOnShape theWOnS = new(sb, _work);
        GluedShape theGS = new(sn);

        int lmap = _mapEF.Extent();

        for (ind = 1; ind <= lmap; ind++)
        {
            Shape s = _mapEF.FindKey(ind);
            if (s.ShapeType() == ShapeType.Edge)
            {
                Shape s2 = _mapEF[ind];
                if (!s2.IsNull)
                {
                    theWOnS.Bind(ShapeCasts.Edge(s), ShapeCasts.Face(s2));
                }
            }
            else
            { // TopAbs_FACE
                theGS.GlueOnFace(ShapeCasts.Face(s));
            }
        }

        foreach (KeyValuePair<Shape, Shape> itm in _mapEE)
        {
            theWOnS.Bind(ShapeCasts.Edge(itm.Key), ShapeCasts.Edge(itm.Value));
        }

        theWOnS.BindAll();

        if (!theWOnS.IsDone)
        {
            return;
        }

        Spliter theSplit = new(sb);
        theSplit.InternalPerform(theWOnS);
        if (!theSplit.IsDone)
        {
            return;
        }
        // Mise a jour des descendants
        Explorer exp = new(sb, ShapeType.Face);
        for (; exp.More(); exp.Next())
        {
            _descF.Bind(exp.Current(), new NCollection_List<Shape>(theSplit.InternalDescendantShapes(exp.Current())));
        }

        for (exp.Init(sn, ShapeType.Face); exp.More(); exp.Next())
        {
            NCollection_List<Shape> thelist = new();
            _descF.Bind(exp.Current(), thelist);
            if (Contains(theGS.OrientedFaces(), exp.Current()))
            {
                _descF[exp.Current()].Append(exp.Current());
            }
        }

        LocalGenerator theGen = new(theSplit.InternalResultingShape(), _work);
        theGen.InternalPerform(theGS);

        _done = theGen.IsDone;
        if (_done)
        {
            _res = theGen.InternalResultingShape();

            AddEdges(sn);

            // Mise a jour des descendants
            foreach (Shape key in _descF.Keys.ToList())
            {
                NCollection_List<Shape> newDesc = new();
                NCollection_List<Shape>.Iterator itl = new();
                for (itl.Initialize(_descF[key]); itl.More(); itl.Next())
                {
                    NCollection_List<Shape>.Iterator itl2 = new(theGen.InternalDescendantFace(ShapeCasts.Face(itl.Value())));
                    for (; itl2.More(); itl2.Next())
                    {
                        Face descface = ShapeCasts.Face(itl2.Value());
                        if (!descface.IsNull)
                        { // sinon la face a disparu
                            newDesc.Append(descface);
                        }
                    }
                }
                _descF[key] = newDesc;
            }
        }

        // recodage des regularites
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theMapEF1 = new(ShapeHasher.Instance),
                                                                                   theMapEF2 = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(sn, ShapeType.Edge, ShapeType.Face, theMapEF1);
        Exploration.MapShapesAndAncestors(_res, ShapeType.Edge, ShapeType.Face, theMapEF2);

        for (ind = 1; ind <= theMapEF1.Extent(); ind++)
        {
            Edge edg = ShapeCasts.Edge(theMapEF1.FindKey(ind));
            NCollection_List<Shape> ll = theMapEF1[ind];
            if (ll.Extent() == 2)
            {
                Face fac1 = ShapeCasts.Face(ll.First());
                Face fac2 = ShapeCasts.Face(ll.Last());
                Continuity thecont = Tool.Continuity(edg, fac1, fac2);
                if (thecont >= G1)
                {
                    // on essaie de recoder
                    int ind2 = theMapEF2.FindIndex(edg);
                    if (ind2 != 0)
                    {
                        NCollection_List<Shape> ll2 = theMapEF2[ind2];
                        if (ll2.Extent() == 2)
                        {
                            Face ff1 = ShapeCasts.Face(ll2.First());
                            Face ff2 = ShapeCasts.Face(ll2.Last());
                            if ((ff1.IsSame(fac1) && ff2.IsSame(fac2)) || (ff1.IsSame(fac2) && ff2.IsSame(fac1)))
                            {
                            }
                            else
                            {
                                BrepBuilder b = new();
                                b.Continuity(edg, ff1, ff2, thecont);
                            }
                        }
                    }
                }
            }
        }
        // creation de la liste d`edge
        theWOnS.InitEdgeIterator();
        while (theWOnS.MoreEdge())
        {
            Edge edg = theWOnS.InternalEdge();
            for (ind = 1; ind <= theMapEF2.Extent(); ind++)
            {
                Edge edg1 = ShapeCasts.Edge(theMapEF2.FindKey(ind));
                if (edg1.IsSame(edg))
                {
                    _edges.Append(edg);
                    // recodage eventuel des regularites sur cet edge
                    NCollection_List<Shape> l = theMapEF2[ind];
                    if (l.Extent() == 2)
                    {
                        Face fac1 = ShapeCasts.Face(l.First());
                        Face fac2 = ShapeCasts.Face(l.Last());
                        if (global::Kerf.Engine.Features.Local.LocOpe.TgtFaces(edg, fac1, fac2))
                        {
                            _tgtEdges.Append(edg);
                            Continuity thecont = Tool.Continuity(edg, fac1, fac2);
                            if (thecont < G1)
                            {
                                BrepBuilder b = new();
                                b.Continuity(edg, fac1, fac2, G1);
                            }
                        }
                    }
                }
            }
            theWOnS.NextEdge();
        }

        // recodage eventuel des regularites sur cet edge

        // Kerf: the end of the working overlay: new TShapes for the caller's sub-shapes changed (unit 10H5)
        if (_work.IsWritten)
        {
            var shapes = new List<Shape> { _res };
            foreach (KeyValuePair<Shape, NCollection_List<Shape>> item in _descF)
            {
                shapes.AddRange(item.Value);
            }
            shapes.AddRange(_edges);
            shapes.AddRange(_tgtEdges);
            _work.Finish(shapes);
            _res = _work.Final(_res);
            foreach (Shape key in _descF.Keys.ToList())
            {
                _descF[key] = _work.Final(_descF[key]);
            }
            _edges = _work.Final(_edges);
            _tgtEdges = _work.Final(_tgtEdges);
        }
    }

    public bool IsDone => _done;

    /// <summary>The glued shape (frozen).</summary>
    public Shape ResultingShape() => global::Kerf.Engine.Features.Local.WorkingShapes.Published(InternalResultingShape());

    /// <summary><see cref="ResultingShape"/> without freezing, for ported code.</summary>
    internal Shape InternalResultingShape()
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _res;
    }

    /// <summary>
    /// The faces of the result descending from <paramref name="f"/>, a face of the basis shape or of the new shape
    /// (frozen).
    /// </summary>
    public NCollection_List<Shape> DescendantFaces(in Face f) =>
        global::Kerf.Engine.Features.Local.WorkingShapes.Published(InternalDescendantFaces(f));

    /// <summary>
    /// <see cref="DescendantFaces"/> without freezing, for ported code.
    /// </summary>
    internal NCollection_List<Shape> InternalDescendantFaces(in Face f)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        Shape aF = f;
        if (_descF.IsBound(aF))
        {
            return _descF[aF];
        }
        return new NCollection_List<Shape>();
    }

    /// <summary>The basis shape (the caller's).</summary>
    public Shape BasisShape() => _sb;

    /// <summary>The new shape (the caller's, reversed for a cut).</summary>
    public Shape GluedShape() => _sn;

    /// <summary>The edges of the result along which the new shape is glued (frozen).</summary>
    public NCollection_List<Shape> Edges() => global::Kerf.Engine.Features.Local.WorkingShapes.Published(_edges);

    /// <summary><see cref="Edges"/> without freezing, for ported code.</summary>
    internal NCollection_List<Shape> InternalEdges() => _edges;

    /// <summary>The edges of <see cref="Edges"/> where the faces are tangent (frozen).</summary>
    public NCollection_List<Shape> TgtEdges() => global::Kerf.Engine.Features.Local.WorkingShapes.Published(_tgtEdges);

    /// <summary><see cref="TgtEdges"/> without freezing, for ported code.</summary>
    internal NCollection_List<Shape> InternalTgtEdges() => _tgtEdges;

    /// <summary>The working shapes of the last <see cref="Perform"/>, for ported code (BRepFeat's descendants).</summary>
    internal WorkingShapes WorkingShapes() => _work;

    private static Orientation GetOrientation(in Face fn, in Face fb)
    {
        Surface sn = Tool.Surface(fn)!;
        Surface sb = Tool.Surface(fb)!;

        // Find a point on Sb
        Explorer exp = new();
        Vector3D n1, n2;

        // Initialize extrema projector once for the target surface
        SurfaceAdaptor gas = new(sb);
        double tolU = gas.UResolution(global::Kerf.Engine.Foundation.Precision.Confusion());
        double tolV = gas.VResolution(global::Kerf.Engine.Foundation.Precision.Confusion());
        ExtPS extPS = new();
        extPS.Initialize(gas,
                           gas.FirstUParameter(),
                           gas.LastUParameter(),
                           gas.FirstVParameter(),
                           gas.LastVParameter(),
                           tolU,
                           tolV);
        extPS.SetFlag(ExtFlag_MIN);

        for (exp.Init(fn, ShapeType.Edge); exp.More(); exp.Next())
        {
            Edge edg = ShapeCasts.Edge(exp.Current());
            Curve2D c2d = Tool.CurveOnSurface(edg, fn, out double f, out double l)!;
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(f) && global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(l))
            {
                f = -100.0;
                l = 100.0;
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(f))
            {
                f = l - 200.0;
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(l))
            {
                l = f + 200.0;
            }
            double deltau = (l - f) / 20.0;
            for (int i = 1; i <= 21; i++)
            {
                c2d.D0(f + (i - 1) * deltau, out Point2D ptvtx);
                sn.D1(ptvtx.X(), ptvtx.Y(), out Point3D pvt, out Vector3D d1u, out Vector3D d1v);
                n1 = d1u.Crossed(d1v);
                if (n1.Magnitude() > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    n1 = n1.Normalized();
                    if (fn.Orientation() == Reversed)
                    {
                        n1 = n1.Reversed();
                    }

                    // Projection sur Sb
                    extPS.Perform(pvt);
                    if (extPS.IsDone)
                    {
                        double dist2min = RealLast();
                        int jmin = 0;
                        for (int j = 1; j <= extPS.ExtCount; j++)
                        {
                            if (extPS.SquareDistance(j) < dist2min)
                            {
                                jmin = j;
                                dist2min = extPS.SquareDistance(j);
                            }
                        }
                        if (jmin != 0)
                        {
                            extPS.Point(jmin).Parameter(out double uu, out double vv);
                            sb.D1(uu, vv, out pvt, out d1u, out d1v);
                            n2 = d1u.Crossed(d1v);
                            if (n2.Magnitude() > global::Kerf.Engine.Foundation.Precision.Confusion())
                            {
                                n2 = n2.Normalized();
                                if (fb.Orientation() == Reversed)
                                {
                                    n2 = n2.Reversed();
                                }
                                if (n1.Dot(n2) > 0.0)
                                {
                                    return Reversed;
                                }
                                return Forward;
                            }
                        }
                    }
                }
            }
        }
        return Internal;
    }

    private static bool Contains(NCollection_List<Shape> l, in Shape s)
    {
        NCollection_List<Shape>.Iterator it = new();
        for (it.Initialize(l); it.More(); it.Next())
        {
            if (it.Value().IsSame(s))
            {
                return true;
            }
        }
        return false;
    }

    // OCCT computes, for each face of the result not of the new shape, which edges of the new shape touch it, and
    // keeps nothing of it: ported for its cost and its exceptions. On the working new shape.
    private void AddEdges(in Shape sn)
    {
        Explorer exp = new(), expsb = new();
        exp.Init(sn, ShapeType.Edge);

        NCollection_IndexedMap<Shape> mapV = new(ShapeHasher.Instance),
                                             mapFPrism = new(ShapeHasher.Instance),
                                             mapE = new(ShapeHasher.Instance);
        Explorer vexp = new();
        int flag, i;

        Exploration.MapShapes(sn, ShapeType.Face, mapFPrism);

        for (expsb.Init(_res, ShapeType.Face); expsb.More(); expsb.Next())
        {
            if (!mapFPrism.Contains(expsb.Current()))
            {
                mapV.Clear();
                Exploration.MapShapes(expsb.Current(), ShapeType.Vertex, mapV);
                Exploration.MapShapes(expsb.Current(), ShapeType.Edge, mapE);
                for (exp.Init(sn, ShapeType.Edge); exp.More(); exp.Next())
                {
                    Edge e = ShapeCasts.Edge(exp.Current());
                    if (mapE.Contains(e))
                    {
                        continue;
                    }
                    flag = 0;
                    vexp.Init(e, ShapeType.Vertex);
                    for (; vexp.More(); vexp.Next())
                    {
                        Vertex v = ShapeCasts.Vertex(vexp.Current());
                        if (mapV.Contains(v))
                        {
                            flag = 1;
                        }
                    }
                    if (flag == 1)
                    {
                        vexp.Init(e, ShapeType.Vertex);
                        ExtPF ext = new();
                        ext.Initialize(ShapeCasts.Face(expsb.Current()));
                        flag = 0;
                        for (; vexp.More(); vexp.Next())
                        {
                            Vertex v = ShapeCasts.Vertex(vexp.Current());
                            if (!mapV.Contains(v))
                            {
                                ext.Perform(v, ShapeCasts.Face(expsb.Current()));
                                if (!ext.IsDone || ext.ExtCount == 0)
                                {
                                    flag = 0;
                                    break;
                                }
                                else
                                {
                                    double dist2min = ext.SquareDistance(1);
                                    for (i = 2; i <= ext.ExtCount; i++)
                                    {
                                        dist2min = Min(dist2min, ext.SquareDistance(i));
                                    }
                                    if (dist2min >= Tool.Tolerance(v) * Tool.Tolerance(v))
                                    {
                                        flag = 0;
                                        break;
                                    }
                                    else
                                    {
                                        flag = 1;
                                    }
                                }
                            }
                            else
                            {
                                flag = 1;
                            }
                        }
                        if (flag == 1)
                        {
                        }
                    }
                }
            }
        }
    }
}
