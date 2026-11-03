// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_RibSlot.hxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_RibSlot.lxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_RibSlot.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Classification;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Features.Local;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using Schematiq.Collections;
using static Kerf.Engine.Features.PerfSelection;
using static Kerf.Engine.Features.StatusError;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Engine.Features.Local.FeatureOperation;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Features;

/// <summary>
/// The root of the mechanical features (ribs and grooves) built from a contour: the profile computed from the contour
/// and the basis shape, the topological reconstruction of the result, and the history of the basis shape's faces.
/// </summary>
/// <remarks>
/// <para>A working object (decision 21): <see cref="MakeShape.Shape"/>, the history and the lists
/// publish what they return (frozen); ported code reads the <c>Internal</c> forms.</para>
/// <para>The basis shape and the contour are never written: the gluing (LocOpe_Gluer, decision 1013) and the
/// same-parameter pass OCCT runs on the glued result (decision 1052) write in a working overlay (unit 10H5).</para>
/// </remarks>
public abstract class RibSlot : MakeShape
{
    protected Point3D _firstPnt = new();
    protected Point3D _lastPnt = new();
    protected bool _fuse;
    protected bool _sliding;
    protected NCollection_DataMap<Shape, NCollection_List<Shape>> _map = new(ShapeHasher.Instance);
    protected NCollection_DataMap<Shape, NCollection_List<Shape>> _lfMap = new(ShapeHasher.Instance);
    protected Shape _fShape = new();
    protected Shape _lShape = new();
    protected PerfSelection _perfSelection;
    protected Wire _wire = new();
    protected Shape _sbase = new();
    protected Face _skface = new();
    protected Face _pbase = new();
    protected Shape _gShape = new();
    protected Shape _sUntil = new();
    protected NCollection_DataMap<Shape, Shape> _gluedF = new(ShapeHasher.Instance);
    protected NCollection_List<Shape> _newEdges = new();
    protected NCollection_List<Shape> _tgtEdges = new();
    protected NCollection_List<Shape> _facesForDraft = new();
    protected StatusError _statusError;

    // OCCT's Modified and Generated return a function-static list
    private readonly NCollection_List<Shape> _list = new();

    protected RibSlot()
    {
        _fuse = false;
        _sliding = false;
        _statusError = StatusError.OK;
    }

    /// <summary>General perform method: the topological reconstruction of the result.</summary>
    protected void LFPerform()
    {
        if (_sbase.IsNull || _pbase.IsNull || _skface.IsNull || _gShape.IsNull || _lfMap.IsEmpty())
        {
            _statusError = StatusError.NotInitialized;
            NotDone();
            return;
        }

        Explorer exp = new(), exp2 = new();
        int theOpe = 2;

        if (!_gluedF.IsEmpty())
        {
            theOpe = 1;
        }

        // Hope that there is just a solid in the result
        if (!_sUntil.IsNull)
        {
            for (exp2.Init(_sUntil, ShapeType.Face); exp2.More(); exp2.Next())
            {
                Shape funtil = exp2.Current();
                for (exp.Init(_sbase, ShapeType.Face); exp.More(); exp.Next())
                {
                    if (exp.Current().IsSame(funtil))
                    {
                        break;
                    }
                }
                if (!exp.More())
                {
                    break;
                }
            }
        }

        LocalGluer theGlue = new();

        // case of gluing
        if (theOpe == 1)
        {
            bool collage = true;
            FindEdges theFE = new();
            theGlue.Init(_sbase, _gShape);
            foreach (KeyValuePair<Shape, Shape> itm in _gluedF)
            {
                Face glface = ShapeCasts.Face(itm.Key);
                Face fac = ShapeCasts.Face(_gluedF[glface]);
                for (exp.Init(_gShape, ShapeType.Face); exp.More(); exp.Next())
                {
                    if (exp.Current().IsSame(glface))
                    {
                        break;
                    }
                }
                if (exp.More())
                {
                    collage = global::Kerf.Engine.Features.BRepFeat.IsInside(glface, fac);
                    if (!collage)
                    {
                        theOpe = 2;
                        break;
                    }
                    else
                    {
                        theGlue.Bind(glface, fac);
                        theFE.Set(glface, fac);
                        for (theFE.InitIterator(); theFE.More(); theFE.Next())
                        {
                            theGlue.Bind(theFE.EdgeFrom(), theFE.EdgeTo());
                        }
                    }
                }
            }

            FeatureOperation ope = theGlue.OpeType();
            if (ope == FeatureOperation.Invalid || (_fuse && ope != FeatureOperation.Fuse) || (!_fuse && ope != FeatureOperation.Cut) || (!collage))
            {
                theOpe = 2;
            }
        }

        // gluing is always applicable

        if (theOpe == 1)
        {
            theGlue.Perform();
            if (theGlue.IsDone)
            {
                UpdateDescendants(theGlue);
                _newEdges = new NCollection_List<Shape>(theGlue.InternalEdges());
                _tgtEdges = new NCollection_List<Shape>(theGlue.InternalTgtEdges());
                //
                Done();
                _shape = theGlue.InternalResultingShape();
                SameParameterOfTheGluedResult();
            }
            else
            {
                theOpe = 2;
            }
        }

        // case without gluing
        if (theOpe == 2)
        {
            FeatureBuilder theBuilder = new();
            NCollection_List<Shape> partsoftool = new();
            SolidClassifier oussa = new();
            bool bFlag;
            NCollection_List<Shape>.Iterator it = new();

            bFlag = _perfSelection != PerfSelection.NoSelection;
            //
            theBuilder.Init(_sbase, _gShape);
            theBuilder.SetOperation(_fuse ? 1 : 0, bFlag);
            //
            theBuilder.Perform();
            if (bFlag)
            {
                theBuilder.PartsOfTool(partsoftool);
                it.Initialize(partsoftool);
                if (it.More() && _perfSelection != PerfSelection.NoSelection)
                {
                    double toler = Tool.Tolerance(_pbase) * 2;
                    //
                    for (; it.More(); it.Next())
                    {
                        oussa.Load(it.Value());
                        oussa.Perform(_firstPnt, toler);
                        Containment sp1 = oussa.State();
                        oussa.Perform(_lastPnt, toler);
                        Containment sp2 = oussa.State();
                        if (sp1 != Out && sp2 != Out)
                        {
                            Shape s = it.Value();
                            theBuilder.KeepPart(s);
                        }
                    }
                }
                //
                theBuilder.PerformResult();
                _shape = theBuilder.InternalShape();
            }
            else
            {
                _shape = theBuilder.InternalShape();
            }
            Done();
        }
    }

    // OCCT runs BRepLib::SameParameter(myShape, 1.e-7, true) on the glued result in place. The result shares the
    // caller's frozen sub-shapes where the gluing left them unchanged: the pass runs in a working overlay (in place
    // inside an operation's one that takes writes), the result holds new TShapes only for what it changed, and the
    // history follows (decision 1052, unit 10H5).
    private void SameParameterOfTheGluedResult()
    {
        using WorkingOverlay? overlay = WorkingOverlay.BeginUnlessWriting(_shape);
        BrepLibrary.SameParameter(_shape, 1.0e-7, true);
        if (overlay is null)
        {
            return;
        }
        _shape = overlay.Finish(_shape);
        foreach (Shape key in _map.Keys.ToList())
        {
            _map[key] = Final(overlay, _map[key]);
        }
        _newEdges = Final(overlay, _newEdges);
        _tgtEdges = Final(overlay, _tgtEdges);
    }

    // The shapes of a list as they stand after a working overlay's end.
    private protected static NCollection_List<Shape> Final(WorkingOverlay overlay, NCollection_List<Shape> list)
    {
        NCollection_List<Shape> aList = new();
        for (NCollection_List<Shape>.Iterator it = new(list); it.More(); it.Next())
        {
            aList.Append(overlay.Final(it.Value()));
        }
        return aList;
    }

    /// <summary>True if the face <paramref name="f"/> has no descendant in the result.</summary>
    /// <exception cref="NoSuchObjectException"><paramref name="f"/> isn't a shape of the history.</exception>
    public override bool IsDeleted(in Shape f)
    {
        return _map.Find(f).IsEmpty();
    }

    /// <summary>The faces generated from the face <paramref name="f"/> (OCCT's virtual; unfrozen).</summary>
    internal override NCollection_List<Shape> InternalModified(in Shape f)
    {
        if (_map.IsBound(f))
        {
            _list.Clear();
            NCollection_List<Shape>.Iterator ite = new(_map.Find(f));
            for (; ite.More(); ite.Next())
            {
                Shape sh = ite.Value();
                if (!sh.IsSame(f))
                {
                    _list.Append(sh);
                }
            }
            return _list;
        }
        return _generated; // empty list
    }

    /// <summary>The faces created from the shape <paramref name="s"/> (OCCT's virtual; unfrozen).</summary>
    internal override NCollection_List<Shape> InternalGenerated(in Shape s)
    {
        if (s.ShapeType() != ShapeType.Face)
        {
            _generated.Clear();
            if (_lfMap.IsEmpty() || !_lfMap.IsBound(s))
            {
                if (_map.IsBound(s))
                { // check if filter on face or not
                    _list.Clear();
                    NCollection_List<Shape>.Iterator ite = new(_map.Find(s));
                    for (; ite.More(); ite.Next())
                    {
                        Shape sh = ite.Value();
                        if (!sh.IsSame(s))
                        {
                            _list.Append(sh);
                        }
                    }
                    return _list;
                }
                else
                {
                    return _generated;
                }
            }
            else
            {
                _generated.Clear();
                NCollection_List<Shape>.Iterator it = new(_lfMap.Find(s));
                _list.Clear();
                for (; it.More(); it.Next())
                {
                    if (_map.IsBound(it.Value()))
                    {
                        NCollection_List<Shape>.Iterator it1 = new(_map.Find(it.Value()));
                        for (; it1.More(); it1.Next())
                        {
                            Shape sh = it1.Value();
                            if (!sh.IsSame(s))
                            {
                                _list.Append(sh);
                            }
                        }
                    }
                }
                return _list;
            }
        }
        else
        {
            return _generated;
        }
    }

    // The history's lists hold sub-shapes of the result: publishing them publishes the result (decision 414).
    private protected override void BeforePublishingLists()
    {
        _shape.TShape()?.FreezeGraph();
    }

    protected void UpdateDescendants(LocalGluer g)
    {
        NCollection_List<Shape>.Iterator it = new(), it2 = new();

        foreach (KeyValuePair<Shape, NCollection_List<Shape>> itdm in _map)
        {
            Shape orig = itdm.Key;
            // OCCT iterates this address-keyed map into the list: insertion order here (decision 1054)
            NCollection_IndexedMap<Shape> newdsc = new(ShapeHasher.Instance);
            for (it.Initialize(itdm.Value); it.More(); it.Next())
            {
                Face fdsc = ShapeCasts.Face(it.Value());
                for (it2.Initialize(g.InternalDescendantFaces(fdsc)); it2.More(); it2.Next())
                {
                    newdsc.Add(it2.Value());
                }
            }
            _map.ChangeFind(orig).Clear();
            foreach (Shape itm in newdsc)
            {
                _map.ChangeFind(orig).Append(itm);
            }
        }
    }

    /// <summary>The shapes created at the bottom of the created form; it may be empty (frozen).</summary>
    public ISchematiqList<Shape> FirstShape()
    {
        BeforePublishingLists();
        return PublishedList(InternalFirstShape());
    }

    /// <summary><see cref="FirstShape"/> without freezing, for ported code.</summary>
    internal NCollection_List<Shape> InternalFirstShape()
    {
        if (!_fShape.IsNull)
        {
            return _map.Find(_fShape);
        }
        return _generated; // empty list
    }

    /// <summary>The shapes created at the top of the created form; it may be empty (frozen).</summary>
    public ISchematiqList<Shape> LastShape()
    {
        BeforePublishingLists();
        return PublishedList(InternalLastShape());
    }

    /// <summary><see cref="LastShape"/> without freezing, for ported code.</summary>
    internal NCollection_List<Shape> InternalLastShape()
    {
        if (!_lShape.IsNull)
        {
            return _map.Find(_lShape);
        }
        return _generated; // empty list
    }

    /// <summary>The faces on which drafts can be made afterwards (frozen).</summary>
    public ISchematiqList<Shape> FacesForDraft()
    {
        BeforePublishingLists();
        return PublishedList(_facesForDraft);
    }

    /// <summary><see cref="FacesForDraft"/> without freezing, for ported code.</summary>
    internal NCollection_List<Shape> InternalFacesForDraft() => _facesForDraft;

    /// <summary>The limiting and glueing edges generated by the feature (frozen).</summary>
    public ISchematiqList<Shape> NewEdges()
    {
        BeforePublishingLists();
        return PublishedList(_newEdges);
    }

    /// <summary><see cref="NewEdges"/> without freezing, for ported code.</summary>
    internal NCollection_List<Shape> InternalNewEdges() => _newEdges;

    /// <summary>The tangent edges among <see cref="NewEdges"/> (frozen).</summary>
    public ISchematiqList<Shape> TgtEdges()
    {
        BeforePublishingLists();
        return PublishedList(_tgtEdges);
    }

    /// <summary><see cref="TgtEdges"/> without freezing, for ported code.</summary>
    internal NCollection_List<Shape> InternalTgtEdges() => _tgtEdges;

    /// <summary>The status of the last construction.</summary>
    public StatusError CurrentStatusError() => _statusError;

    /// <summary>A proofing point somewhat inside the material side of the edge <paramref name="e"/>.</summary>
    protected Point3D CheckPoint(in Edge e, double bnd, PlaneSurface pln)
    {
        // Vector product : normal to plane X direction Wire
        // -> gives the material side
        // Proofing point somewhat inside the material side
        Curve3D cc = Tool.Curve(e, out double f, out double l)!;

        double par = (f + l) / 2.0;
        cc.D1(par, out Point3D pp, out Vector3D tgt);

        if (e.Orientation() == Reversed)
        {
            tgt = tgt.Reversed();
        }

        Vector3D d = -tgt.Crossed(new Vector3D(pln.Pln().Position().Direction())) / 10.0;
        pp = pp.Translated(d);

        return pp;
    }

    /// <summary>The normal of the face <paramref name="f"/> at <paramref name="p"/>, reversed for a forward face.</summary>
    protected Direction3D Normal(in Face f, in Point3D p)
    {
        double u, v;
        FaceSurfaceAdaptor AS = new(f, true);

        switch (AS.GetType())
        {
            case Plane:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(AS.Plane(), p, out u, out v);
                break;

            case SurfaceKind.Cylinder:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(AS.Cylinder(), p, out u, out v);
                break;

            case SurfaceKind.Cone:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(AS.Cone(), p, out u, out v);
                break;

            case SurfaceKind.Torus:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(AS.Torus(), p, out u, out v);
                break;

            default:
            {
                return new Direction3D(Direction3D.D.X);
            }
        }

        AS.D1(u, v, out Point3D pt, out Vector3D d1U, out Vector3D d1V);
        Direction3D n = new();
        global::Kerf.Engine.Geometry.CSLib.Normal(d1U, d1V, global::Kerf.Engine.Foundation.Precision.Confusion(), out DerivativeStatus st, ref n);
        if (f.Orientation() == Forward)
        {
            n = n.Reversed();
        }
        return n;
    }

    /// <summary>The parameter of the point <paramref name="p"/> on the conic <paramref name="c"/> (0 for other curves).</summary>
    public static double IntPar(Curve3D? c, in Point3D p)
    {
        if (c is null)
        {
            return 0.0;
        }

        CurveAdaptor ac = new(c);
        double u;

        switch (ac.GetType())
        {
            case Line:
                u = global::Kerf.Engine.Geometry.ElCLib.Parameter(ac.Line(), p);
                break;

            case Circle:
                u = global::Kerf.Engine.Geometry.ElCLib.Parameter(ac.Circle(), p);
                break;

            case Ellipse:
                u = global::Kerf.Engine.Geometry.ElCLib.Parameter(ac.Ellipse(), p);
                break;

            case Hyperbola:
                u = global::Kerf.Engine.Geometry.ElCLib.Parameter(ac.Hyperbola(), p);
                break;

            case Parabola:
                u = global::Kerf.Engine.Geometry.ElCLib.Parameter(ac.Parabola(), p);
                break;

            default:
                u = 0.0;
                break;
        }

        return u;
    }

    /// <summary>Extends the edge <paramref name="e"/> at its start (or its end) by a tenth of <paramref name="bnd"/>.</summary>
    protected void EdgeExtention(ref Edge e, double bnd, bool firstLast)
    {
        Curve3D cu = Tool.Curve(e, out double f, out double l)!;
        BoundedCurve3D c = new TrimmedCurve3D(cu, f, l);

        Edge E;

        if (cu.GetType() == typeof(LineCurve3D) || cu.GetType() == typeof(CircleCurve3D)
            || cu.GetType() == typeof(EllipseCurve3D) || cu.GetType() == typeof(HyperbolaCurve3D)
            || cu.GetType() == typeof(ParabolaCurve3D))
        {
            if (firstLast)
            {
                LibraryMakeEdge edg = new(cu, f - bnd / 10.0, l);
                E = ShapeCasts.Edge(edg.InternalShape());
            }
            else
            {
                LibraryMakeEdge edg = new(cu, f, l + bnd / 10.0);
                E = ShapeCasts.Edge(edg.InternalShape());
            }
        }
        else
        {
            LineCurve3D ln;
            Point3D pt;
            if (firstLast)
            {
                c.D1(f, out Point3D pnt, out Vector3D vct);
                ln = new LineCurve3D(pnt, new Direction3D(-vct));
                ln.D0(bnd / 1000.0, out pt);
                global::Kerf.Engine.Geometry.Library.GeomLib.ExtendCurveToPoint(ref c, pt, (int)G1, false);
                LibraryMakeEdge edg = new(c, pt, Tool.Pnt(Exploration.LastVertex(e, true)));
                E = ShapeCasts.Edge(edg.InternalShape());
            }
            else
            {
                c.D1(l, out Point3D pnt, out Vector3D vct);
                ln = new LineCurve3D(pnt, new Direction3D(vct));
                ln.D0(bnd / 1000.0, out pt);
                global::Kerf.Engine.Geometry.Library.GeomLib.ExtendCurveToPoint(ref c, pt, (int)G1, true);
                LibraryMakeEdge edg = new(c, Tool.Pnt(Exploration.FirstVertex(e, true)), pt);
                E = ShapeCasts.Edge(edg.InternalShape());
            }
        }
        e = E;
    }

    /// <summary>
    /// The face of <paramref name="faces"/> first met by lines through the point of <paramref name="cc"/> at
    /// <paramref name="par"/>, turned around the normal of <paramref name="pln"/>.
    /// </summary>
    public static Face ChoiceOfFaces(NCollection_List<Shape> faces, Curve3D cc, double par, double bnd,
                                            PlaneSurface pln)
    {
        Face fff = new();

        cc.D1(par, out Point3D pp, out Vector3D tgt);

        LineCurve3D l1 = new(pp, new Direction3D(tgt));

        NCollection_Sequence<Curve3D?> scur = new();
        int counter = 0;

        Axis3D axe = new(pp, pln.Position().Direction());
        for (int i = 1; i <= 8; i++)
        {
            Curve3D l = (Curve3D)l1.Rotated(axe, i * M_PI / 9.0);
            scur.Append(l);
            counter++;
        }

        NCollection_List<Shape>.Iterator it = new();
        it.Initialize(faces);
        double Par = RealLast();
        for (; it.More(); it.Next())
        {
            Face f = ShapeCasts.Face(it.Value());
            CSIntersector asi = new(f);
            asi.Perform(scur);
            if (!asi.IsDone)
            {
                continue;
            }
            for (int jj = 1; jj <= counter; jj++)
            {
                if (asi.NbPoints(jj) >= 1)
                {
                    double app = asi.Point(jj, 1).Parameter();
                    if (app >= 0 && app < Par)
                    {
                        Par = app;
                        fff = f;
                    }
                }
            }
        }

        return fff;
    }

    /// <summary>The bound of the boxes of the basis shape and the until shape, and the box's corners pushed outwards.</summary>
    protected double HeightMax(in Shape sbase, in Shape sUntil, ref Point3D p1, ref Point3D p2)
    {
        Box box = new();
        Bounds.Add(sbase, ref box);
        if (!sUntil.IsNull)
        {
            Bounds.Add(sUntil, ref box);
        }
        double[] c = new double[6];
        double bnd;
        box.Get(out c[0], out c[2], out c[4], out c[1], out c[3], out c[5]);
        bnd = c[0];
        for (int i = 0; i < 6; i++)
        {
            if (c[i] > bnd)
            {
                bnd = c[i];
            }
        }
        p1 = p1.WithCoord(c[0] - 2.0 * bnd, c[1] - 2.0 * bnd, c[2] - 2.0 * bnd);
        p2 = p2.WithCoord(c[3] + 2.0 * bnd, c[4] + 2.0 * bnd, c[5] + 2.0 * bnd);
        return bnd;
    }

    /// <summary>The faces, edges and vertices of the basis shape where the contour starts and ends.</summary>
    protected bool ExtremeFaces(bool revolRib, double bnd, PlaneSurface pln, ref Edge firstEdge,
                                ref Edge lastEdge, ref Face firstFace, ref Face lastFace,
                                ref Vertex firstVertex, ref Vertex lastVertex, ref bool onFirstFace,
                                ref bool onLastFace, ref bool ptOnFirstEdge, ref bool ptOnLastEdge,
                                ref Edge onFirstEdge, ref Edge onLastEdge)
    {
        bool data = true;
        firstFace.Nullify();
        lastFace.Nullify();
        firstEdge.Nullify();
        lastEdge.Nullify();
        ptOnFirstEdge = false;
        ptOnLastEdge = false;
        onFirstEdge.Nullify();
        onLastEdge.Nullify();

        CurveShapeIntersector inter = new();
        BrepBuilder b = new();
        Explorer ex1 = new();

        bool firstOK = false, lastOK = false;

        int numberOfEdges = 0;
        Explorer exp = new(_wire, ShapeType.Edge);

        for (; exp.More(); exp.Next())
        {
            numberOfEdges++;
        }

        // ---the wire includes only one edge
        if (numberOfEdges == 1)
        {
            exp.ReInit();
            Point3D firstpoint = new(), lastpoint = new();

            // Points limit the unique edge
            Edge E = ShapeCasts.Edge(exp.Current());
            Curve3D cc = Tool.Curve(E, out double f, out double l)!;
            Point3D p1 = Tool.Pnt(Exploration.FirstVertex(E, true));
            Point3D p2 = Tool.Pnt(Exploration.LastVertex(E, true));

            double firstPar = f;
            double lastPar = l;

            // ---Find if 2 points limiting the unique edge of the wire
            //    are on an edge or a vertex of the base shape
            bool ptOnFirstVertex = false;
            bool ptOnLastVertex = false;
            Vertex onFirstVertex = new(), onLastVertex = new();
            PtOnEdgeVertex(revolRib, _sbase, p1, firstVertex, lastVertex, ref ptOnFirstEdge, ref onFirstEdge,
                           ref ptOnFirstVertex, ref onFirstVertex);
            PtOnEdgeVertex(revolRib, _sbase, p2, firstVertex, lastVertex, ref ptOnLastEdge, ref onLastEdge,
                           ref ptOnLastVertex, ref onLastVertex);

            NCollection_Map<Shape> map = new(ShapeHasher.Instance);

            if (ptOnFirstEdge)
            {
                if (!ptOnFirstVertex)
                {
                    // Find FirstFace : face of the base shape containing OnFirstEdge
                    //                  meeting ChoiceOfFaces
                    Explorer ex4 = new(), ex5 = new();
                    ex4.Init(_sbase, ShapeType.Face);
                    NCollection_List<Shape> faces = new();
                    faces.Clear();
                    map.Clear();
                    for (; ex4.More(); ex4.Next())
                    {
                        Face fx = ShapeCasts.Face(ex4.Current());
                        if (!map.Add(fx))
                        {
                            continue;
                        }
                        ex5.Init(ex4.Current(), ShapeType.Edge);
                        for (; ex5.More(); ex5.Next())
                        {
                            Edge ee = ShapeCasts.Edge(ex5.Current());
                            if (ee.IsSame(onFirstEdge))
                            {
                                faces.Append(fx);
                            }
                        }
                    }
                    if (!faces.IsEmpty())
                    {
                        Face fff = ChoiceOfFaces(faces, cc, firstPar + bnd / 50.0, bnd / 50.0, pln);
                        if (!fff.IsNull)
                        {
                            firstFace = fff;
                        }
                    }
                }
                else if (ptOnFirstVertex)
                {
                    // Find FirstFace : face of the base shape containing OnFirstVertex
                    //                  meeting ChoiceOfFaces
                    Explorer ex4 = new(), ex5 = new();
                    ex4.Init(_sbase, ShapeType.Face);
                    NCollection_List<Shape> faces = new();
                    faces.Clear();
                    map.Clear();
                    for (; ex4.More(); ex4.Next())
                    {
                        Face fx = ShapeCasts.Face(ex4.Current());
                        if (!map.Add(fx))
                        {
                            continue;
                        }
                        ex5.Init(ex4.Current(), ShapeType.Vertex);
                        for (; ex5.More(); ex5.Next())
                        {
                            Vertex vv = ShapeCasts.Vertex(ex5.Current());
                            if (vv.IsSame(onFirstVertex))
                            {
                                faces.Append(fx);
                                break;
                            }
                        }
                    }
                    if (!faces.IsEmpty())
                    {
                        Face fff = ChoiceOfFaces(faces, cc, firstPar + bnd / 50.0, bnd / 50.0, pln);
                        if (!fff.IsNull)
                        {
                            firstFace = fff;
                        }
                    }
                }
                firstEdge = E;
                LibraryMakeVertex v = new(p1);
                firstVertex = v.InternalVertex();
                onFirstFace = true;
            }

            if (ptOnLastEdge)
            {
                if (!ptOnLastVertex)
                {
                    // Find LastFace : face of the base shape containing OnLastEdge
                    //                 meeting ChoiceOfFaces
                    Explorer ex4 = new(), ex5 = new();
                    ex4.Init(_sbase, ShapeType.Face);
                    NCollection_List<Shape> faces = new();
                    faces.Clear();
                    map.Clear();
                    for (; ex4.More(); ex4.Next())
                    {
                        Face fx = ShapeCasts.Face(ex4.Current());
                        if (!map.Add(fx))
                        {
                            continue;
                        }
                        ex5.Init(ex4.Current(), ShapeType.Edge);
                        for (; ex5.More(); ex5.Next())
                        {
                            Edge ee = ShapeCasts.Edge(ex5.Current());
                            if (ee.IsSame(onLastEdge))
                            {
                                faces.Append(fx);
                                break;
                            }
                        }
                    }
                    if (!faces.IsEmpty())
                    {
                        Face fff = ChoiceOfFaces(faces, cc, lastPar - bnd / 50.0, bnd / 50.0, pln);
                        if (!fff.IsNull)
                        {
                            lastFace = fff;
                        }
                    }
                }
                else if (ptOnLastEdge && ptOnLastVertex)
                {
                    // Find LastFace : face of the base shape containing OnLastVertex
                    //                 meeting ChoiceOfFaces
                    Explorer ex4 = new(), ex5 = new();
                    ex4.Init(_sbase, ShapeType.Face);
                    NCollection_List<Shape> faces = new();
                    faces.Clear();
                    map.Clear();
                    for (; ex4.More(); ex4.Next())
                    {
                        Face fx = ShapeCasts.Face(ex4.Current());
                        if (!map.Add(fx))
                        {
                            continue;
                        }
                        ex5.Init(ex4.Current(), ShapeType.Vertex);
                        for (; ex5.More(); ex5.Next())
                        {
                            Vertex vv = ShapeCasts.Vertex(ex5.Current());
                            if (vv.IsSame(onLastVertex))
                            {
                                faces.Append(fx);
                                break;
                            }
                        }
                    }
                    if (!faces.IsEmpty())
                    {
                        Face fff = ChoiceOfFaces(faces, cc, lastPar - bnd / 50.0, bnd / 50.0, pln);
                        if (!fff.IsNull)
                        {
                            lastFace = fff;
                        }
                    }
                }
                lastEdge = E;
                LibraryMakeVertex v = new(p2);
                lastVertex = v.InternalVertex();
                onLastFace = true;
            }

            if (!firstFace.IsNull && !lastFace.IsNull)
            {
                return true;
            }

            //--- FirstFace or LastFace was not found
            CSIntersector asi = new(_sbase);
            NCollection_Sequence<Curve3D?> scur = new();
            scur.Clear();
            scur.Append(cc);
            asi.Perform(scur);
            double lastpar, firstpar;
            if (asi.IsDone && asi.NbPoints(1) >= 2)
            {
                lastpar = asi.Point(1, asi.NbPoints(1)).Parameter();
                int lastindex = asi.NbPoints(1);
                if (lastpar > l)
                {
                    for (int jj = asi.NbPoints(1) - 1; jj >= 1; jj--)
                    {
                        double par = asi.Point(1, jj).Parameter();
                        if (par <= l)
                        {
                            lastpar = par;
                            lastindex = jj;
                            break;
                        }
                    }
                }
                int firstindex = lastindex - 1;
                firstpar = asi.Point(1, firstindex).Parameter();

                if (firstFace.IsNull)
                {
                    firstFace = asi.Point(1, firstindex).Face();
                    cc.D0(firstpar, out firstpoint);
                    LibraryMakeVertex v1 = new(firstpoint);
                    firstVertex = ShapeCasts.Vertex(v1.InternalShape());
                    firstEdge = E;
                }

                if (lastFace.IsNull)
                {
                    lastFace = asi.Point(1, lastindex).Face();
                    cc.D0(lastpar, out lastpoint);
                    LibraryMakeVertex v2 = new(lastpoint);
                    lastVertex = ShapeCasts.Vertex(v2.InternalShape());
                    lastEdge = E;
                }
            }
            else
            {
                data = false;
                return data;
            }

            if (!onFirstFace)
            {
                onFirstFace = p1.Distance(firstpoint) <= global::Kerf.Engine.Foundation.Precision.Confusion();
            }
            if (!onLastFace)
            {
                onLastFace = p2.Distance(lastpoint) <= global::Kerf.Engine.Foundation.Precision.Confusion();
            }

            data = !(firstFace.IsNull || lastFace.IsNull);
            return data;
        }

        // ---The wire consists of several edges
        else
        {
            WireExplorer ex = new(_wire);
            for (; ex.More(); ex.Next())
            {
                Edge E = ShapeCasts.Edge(ex.Current());
                Curve3D cur = Tool.Curve(E, out double f, out double l)!;
                f = f - bnd / 10000;
                l = l + bnd / 10000;
                TrimmedCurve3D curve;
                curve = new TrimmedCurve3D(cur, f, l, true);
                Point3D P2 = Tool.Pnt(Exploration.LastVertex(E, true));
                ex1.Init(_sbase, ShapeType.Face);
                Vertex theVertex = new();
                Edge theEdge = new();
                Face theFace = new();
                bool ptOnEdge = false;
                bool ptOnVertex = false;
                Edge onEdge = new();
                Vertex onVertex = new();
                double intpar;
                for (; ex1.More(); ex1.Next())
                {
                    Face curFace = ShapeCasts.Face(ex1.Current());
                    CurveAdaptor gac = new(curve);
                    inter.Init(curFace, gac, Tool.Tolerance(curFace));
                    if (!inter.More())
                    {
                        continue;
                    }
                    for (; inter.More(); inter.Next())
                    {
                        Point3D thePoint = inter.Pnt();
                        if (!firstVertex.IsNull)
                        {
                            Point3D point = Tool.Pnt(firstVertex);
                            if (point.Distance(thePoint) <= Tool.Tolerance(curFace))
                            {
                                continue;
                            }
                        }
                        intpar = IntPar(curve, thePoint);
                        theEdge = E;
                        theFace = curFace;
                        b.MakeVertex(ref theVertex, thePoint, global::Kerf.Engine.Foundation.Precision.Confusion());
                        if (!firstOK)
                        {
                            if (thePoint.Distance(P2) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                            {
                                continue;
                            }
                        }

                        // ---Find thepoint on an edge or a vertex of face f
                        PtOnEdgeVertex(revolRib, curFace, thePoint, firstVertex, lastVertex, ref ptOnEdge, ref onEdge,
                                       ref ptOnVertex, ref onVertex);

                        if (firstEdge.IsNull && !theEdge.IsNull && !theFace.IsNull && !theVertex.IsNull)
                        {
                            firstEdge = theEdge;
                            firstFace = theFace;
                            firstVertex = theVertex;
                            ptOnFirstEdge = ptOnEdge;
                            onFirstEdge = onEdge;
                            theEdge.Nullify();
                            theFace.Nullify();
                            theVertex.Nullify();
                            if (ptOnEdge && !ptOnVertex)
                            {
                                NCollection_List<Shape> faces = new();
                                faces.Clear();
                                faces.Append(firstFace);
                                Explorer ex2 = new();
                                ex2.Init(_sbase, ShapeType.Face);
                                for (; ex2.More(); ex2.Next())
                                {
                                    Face fx = ShapeCasts.Face(ex2.Current());
                                    Explorer ex3 = new();
                                    ex3.Init(fx, ShapeType.Edge);
                                    for (; ex3.More(); ex3.Next())
                                    {
                                        Edge e = ShapeCasts.Edge(ex3.Current());
                                        if (e.IsSame(onEdge) && !fx.IsSame(firstFace))
                                        {
                                            faces.Append(fx);
                                        }
                                    }
                                }
                                Face fff = ChoiceOfFaces(faces, curve, intpar + bnd / 10.0, bnd / 10.0, pln);
                                if (!fff.IsNull)
                                {
                                    firstFace = fff;
                                }
                            }
                            else if (ptOnEdge && ptOnVertex)
                            {
                                NCollection_List<Shape> faces = new();
                                faces.Clear();
                                faces.Append(firstFace);
                                Explorer ex2 = new();
                                ex2.Init(_sbase, ShapeType.Face);
                                for (; ex2.More(); ex2.Next())
                                {
                                    Face fx = ShapeCasts.Face(ex2.Current());
                                    Explorer ex3 = new();
                                    ex3.Init(fx, ShapeType.Vertex);
                                    for (; ex3.More(); ex3.Next())
                                    {
                                        Vertex v = ShapeCasts.Vertex(ex3.Current());
                                        if (v.IsSame(onVertex) && !fx.IsSame(firstFace))
                                        {
                                            faces.Append(fx);
                                        }
                                    }
                                }
                                Face fff = ChoiceOfFaces(faces, curve, intpar + bnd / 10.0, bnd / 10.0, pln);
                                if (!fff.IsNull)
                                {
                                    firstFace = fff;
                                }
                            }
                            if (!firstEdge.IsNull && !firstFace.IsNull && !firstVertex.IsNull)
                            {
                                firstOK = true;
                            }
                        }
                        if (lastEdge.IsNull && !theEdge.IsNull && !theFace.IsNull && !theVertex.IsNull
                            && !firstEdge.IsNull)
                        {
                            lastEdge = theEdge;
                            lastFace = theFace;
                            lastVertex = theVertex;
                            ptOnLastEdge = ptOnEdge;
                            onLastEdge = onEdge;
                            if (ptOnEdge && !ptOnVertex)
                            {
                                NCollection_List<Shape> faces = new();
                                faces.Clear();
                                faces.Append(lastFace);
                                Explorer ex2 = new();
                                ex2.Init(_sbase, ShapeType.Face);
                                for (; ex2.More(); ex2.Next())
                                {
                                    Face fx = ShapeCasts.Face(ex2.Current());
                                    Explorer ex3 = new();
                                    ex3.Init(fx, ShapeType.Edge);
                                    for (; ex3.More(); ex3.Next())
                                    {
                                        Edge e = ShapeCasts.Edge(ex3.Current());
                                        if (e.IsSame(onEdge) && !fx.IsSame(lastFace))
                                        {
                                            faces.Append(fx);
                                        }
                                    }
                                }
                                Face fff = ChoiceOfFaces(faces, curve, intpar - bnd / 10.0, bnd / 10.0, pln);
                                if (!fff.IsNull)
                                {
                                    lastFace = fff;
                                }
                            }
                            else if (ptOnEdge && ptOnVertex)
                            {
                                NCollection_List<Shape> faces = new();
                                faces.Clear();
                                faces.Append(lastFace);
                                Explorer ex2 = new();
                                ex2.Init(_sbase, ShapeType.Face);
                                for (; ex2.More(); ex2.Next())
                                {
                                    Face fx = ShapeCasts.Face(ex2.Current());
                                    Explorer ex3 = new();
                                    ex3.Init(fx, ShapeType.Vertex);
                                    for (; ex3.More(); ex3.Next())
                                    {
                                        Vertex v = ShapeCasts.Vertex(ex3.Current());
                                        if (v.IsSame(onVertex) && !fx.IsSame(lastFace))
                                        {
                                            faces.Append(fx);
                                        }
                                    }
                                }
                                Face fff = ChoiceOfFaces(faces, curve, intpar - bnd / 10.0, bnd / 10.0, pln);
                                if (!fff.IsNull)
                                {
                                    lastFace = fff;
                                }
                            }
                            if (!lastEdge.IsNull && !lastFace.IsNull && !lastVertex.IsNull)
                            {
                                lastOK = true;
                            }
                            break;
                        }
                    }
                }
            }

            if (firstOK && lastOK)
            {
                data = true;
                Point3D pp1 = Tool.Pnt(Exploration.FirstVertex(firstEdge, true));
                Point3D pp2 = Tool.Pnt(Exploration.LastVertex(lastEdge, true));
                Point3D p1 = Tool.Pnt(firstVertex);
                Point3D p2 = Tool.Pnt(lastVertex);
                if (p1.Distance(pp1) <= Tool.Tolerance(firstFace))
                {
                    onFirstFace = true;
                }
                if (p2.Distance(pp2) <= Tool.Tolerance(lastFace))
                {
                    onLastFace = true;
                }
                return true;
            }
            else
            {
                return false;
            }
        }
    }

    /// <summary>Finds whether the point is on an edge or a vertex of the shape.</summary>
    protected void PtOnEdgeVertex(bool revolRib, in Shape shape, in Point3D point, in Vertex firstVertex,
                                  in Vertex lastVertex, ref bool ptOnEdge, ref Edge onEdge,
                                  ref bool ptOnVertex, ref Vertex onVertex)
    {
        bool testOK;

        Explorer exp = new();
        exp.Init(shape, ShapeType.Edge);
        NCollection_Map<Shape> map = new(ShapeHasher.Instance);
        for (; exp.More(); exp.Next())
        {
            Edge e = ShapeCasts.Edge(exp.Current());
            if (!map.Add(e))
            {
                continue;
            }
            if (!revolRib)
            {
                if (Tool.Degenerated(e))
                {
                    continue;
                }
            }
            Curve3D ccc = Tool.Curve(e, out double fff, out double lll)!;
            if (!revolRib)
            {
                ccc = new TrimmedCurve3D(ccc, fff, lll);
            }
            ProjectPointOnCurve proj = new(point, ccc);
            testOK = false;
            if (!revolRib)
            {
                if (proj.NbPoints() == 1)
                {
                    testOK = true;
                }
            }
            else
            {
                if (proj.NbPoints() >= 1)
                {
                    testOK = true;
                }
            }
            if (testOK && proj.Distance(1) <= Tool.Tolerance(e))
            {
                ptOnEdge = true;
                onEdge = e;
                Vertex ev1 = Exploration.FirstVertex(e, true);
                Vertex ev2 = Exploration.LastVertex(e, true);
                Point3D ep1 = Tool.Pnt(ev1);
                Point3D ep2 = Tool.Pnt(ev2);
                if (point.Distance(ep1) <= Tool.Tolerance(ev1))
                {
                    ptOnVertex = true;
                    onVertex = ev1;
                    break;
                }
                else if (point.Distance(ep2) <= Tool.Tolerance(ev1))
                {
                    ptOnVertex = true;
                    onVertex = ev2;
                    break;
                }
                break;
            }
        }
    }

    /// <summary>The profile of a sliding rib or groove: part of the bounding box closed by the contour.</summary>
    protected bool SlidingProfile(ref Face prof, bool revolRib, double myTol, ref int concavite, PlaneSurface myPln,
                                  in Face bndFace, in Point3D checkPnt, in Face firstFace,
                                  in Face lastFace, in Vertex firstVertex, in Vertex lastVertex,
                                  in Edge firstEdge, in Edge lastEdge)
    {
        bool profileOK = true;
        // --case of sliding : construction of the wire of the profile
        // --> 1 part bounding box + 1 part wire
        //   attention to the compatibility of orientations

        Direction3D fn, ln;
        LibraryMakeWire ww = new();

        fn = Normal(firstFace, _firstPnt);
        ln = Normal(lastFace, _lastPnt);

        // Case of the groove (cut) <> rib (fuse)
        // -> we are in the material
        // -> make everything in 2d in the working plane : easier
        if (!_fuse)
        {
            fn = -fn;
            ln = -ln;
        }

        LineCurve3D ln1, ln2;
        Point3D pt = new();

        ln2 = new LineCurve3D(_firstPnt, fn);
        ln1 = new LineCurve3D(_lastPnt, ln);

        Curve2D ln2d1 = Projections.To2d(ln1, myPln.Pln())!;
        Curve2D ln2d2 = Projections.To2d(ln2, myPln.Pln())!;

        CurveCurveIntersection2D inter = new(ln2d1, ln2d2, global::Kerf.Engine.Foundation.Precision.Confusion());

        bool testOK = true;
        if (revolRib)
        {
            Direction3D d1, d2;
            d1 = ln1.Position().Direction();
            d2 = ln2.Position().Direction();
            if (d1.IsOpposite(d2, myTol))
            {
                double par1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(ln1.Lin(), _firstPnt);
                double par2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(ln2.Lin(), _lastPnt);
                if (par1 >= myTol || par2 >= myTol)
                {
                    concavite = 2; // parallel and concave
                    LibraryMakeEdge e1 = new(_lastPnt, _firstPnt);
                    ww.Add(e1.InternalEdge());
                }
            }
            if (d1.IsEqual(d2, myTol))
            {
                if (concavite == 3)
                {
                    testOK = false;
                }
            }
        }

        if (testOK)
        {
            if (inter.NbPoints() > 0)
            {
                Point2D P = inter.Point(1);
                myPln.D0(P.X(), P.Y(), out pt);
                double par = IntPar(ln1, pt);
                if (par > 0)
                {
                    concavite = 1; // concave
                }
            }
        }

        // ---Construction of the profile face
        if (concavite == 1)
        {
            // if concave : it is possible to extend first and last edges of the wire
            //              to the bounding box
            LibraryMakeEdge e1 = new(_lastPnt, pt);
            ww.Add(e1.InternalEdge());
            LibraryMakeEdge e2 = new(pt, _firstPnt);
            ww.Add(e2.InternalEdge());
        }
        else if (concavite == 3)
        {
            // BndEdge : edges of intersection with the bounding box
            Edge bndEdge1 = new(), bndEdge2 = new();
            // Points of intersection with the bounding box / Find Profile
            Point3D bndPnt1 = new(), bndPnt2 = new(), lastPnt = new();
            Explorer expl = new();
            expl.Init(bndFace, ShapeType.Wire);
            WireExplorer explo = new();
            Wire bndWire = ShapeCasts.Wire(expl.Current());
            explo.Init(bndWire);
            for (; explo.More(); explo.Next())
            {
                Edge e = ShapeCasts.Edge(explo.Current());
                Curve3D c = Tool.Curve(e, out double first, out double last)!;
                Curve2D c2d = Projections.To2d(c, myPln.Pln())!;
                CurveCurveIntersection2D intcln1 = new(ln2d1, c2d, global::Kerf.Engine.Foundation.Precision.Confusion());
                if (intcln1.NbPoints() > 0)
                {
                    Point2D p2d = intcln1.Point(1);
                    myPln.D0(p2d.X(), p2d.Y(), out Point3D p);
                    double parl = IntPar(ln1, p);
                    double parc = IntPar(c, p);
                    if (parc >= first && parc <= last && parl >= 0)
                    {
                        bndEdge1 = e;
                        bndPnt1 = p;
                    }
                }

                CurveCurveIntersection2D intcln2 = new(ln2d2, c2d, global::Kerf.Engine.Foundation.Precision.Confusion());
                if (intcln2.NbPoints() > 0)
                {
                    Point2D p2d = intcln2.Point(1);
                    myPln.D0(p2d.X(), p2d.Y(), out Point3D p);
                    double parl = IntPar(ln2, p);
                    double parc = IntPar(c, p);
                    if (parc >= first && parc <= last && parl >= 0)
                    {
                        bndEdge2 = e;
                        bndPnt2 = p;
                    }
                }
                if (!bndEdge1.IsNull && !bndEdge2.IsNull)
                {
                    break;
                }
            }

            if (bndEdge1.IsNull || bndEdge2.IsNull)
            {
                profileOK = false;
                return profileOK;
            }

            LibraryMakeEdge e1 = new(_lastPnt, bndPnt1);
            ww.Add(e1.InternalEdge());

            if (bndEdge1.IsSame(bndEdge2))
            {
                // Particular case : same edge -> simply determined path
                LibraryMakeEdge e2 = new(bndPnt1, bndPnt2);
                ww.Add(e2.InternalEdge());
                LibraryMakeEdge e3 = new(bndPnt2, _firstPnt);
                ww.Add(e3.InternalEdge());
            }
            else
            {
                explo.Init(bndWire);
                for (; explo.More(); explo.Next())
                {
                    Edge e = ShapeCasts.Edge(explo.Current());
                    if (e.IsSame(bndEdge1))
                    {
                        Point3D pp;
                        pp = Tool.Pnt(Exploration.LastVertex(e, true));
                        if (pp.Distance(bndPnt1) >= Tool.Tolerance(e))
                        {
                            lastPnt = pp;
                        }
                        LibraryMakeEdge e2 = new(bndPnt1, lastPnt);
                        ww.Add(e2.InternalEdge());
                        break;
                    }
                }

                if (explo.More())
                {
                    explo.Next();
                    if (explo.Current().IsNull)
                    {
                        explo.Init(bndWire);
                    }
                }
                else
                {
                    explo.Init(bndWire);
                }

                // Check if this is BndEdge2
                // -> if yes : it is required to turn to join FirstPnt
                // -> if no : add edges
                bool fin = false;
                while (!fin)
                {
                    Edge e = ShapeCasts.Edge(explo.Current());
                    if (!e.IsSame(bndEdge2))
                    {
                        Point3D pp;
                        pp = Tool.Pnt(Exploration.LastVertex(e, true));
                        LibraryMakeEdge ee = new(lastPnt, pp);
                        ww.Add(ee.InternalEdge());
                        lastPnt = pp;
                    }
                    else
                    {
                        // the path is closed
                        // -> since met BndEdge2, end of borders on BndFace
                        fin = true;
                        LibraryMakeEdge ee = new(lastPnt, bndPnt2);
                        ww.Add(ee.InternalEdge());
                        lastPnt = bndPnt2;
                    }
                    if (explo.More())
                    {
                        explo.Next();
                        if (explo.Current().IsNull)
                        {
                            explo.Init(bndWire);
                        }
                    }
                    else
                    {
                        explo.Init(bndWire);
                    }
                }

                LibraryMakeEdge e3 = new(bndPnt2, _firstPnt);
                ww.Add(e3.InternalEdge());
            }
        }

        // ---Construction of the profile

        // Explore the wire provided by the user
        // BRepTools_WireExplorer : correct order - without repetition <> TopExp : non ordered
        WireExplorer ex = new(_wire);

        Curve3D firstCurve = Tool.Curve(firstEdge, out double ff, out double ll)!;

        if (!firstEdge.IsSame(lastEdge))
        {
            Vertex flVert = Exploration.LastVertex(firstEdge, true);
            Point3D flPnt = Tool.Pnt(flVert);
            LibraryMakeEdge ef = new(firstCurve, _firstPnt, flPnt);
            ww.Add(ef.InternalEdge());
            for (; ex.More(); ex.Next())
            {
                Edge E = ex.Current();
                if (E.IsSame(firstEdge))
                {
                    break;
                }
            }
            ex.Next();
            for (; ex.More(); ex.Next())
            {
                Edge E = ex.Current();
                if (!E.IsSame(lastEdge))
                {
                    ww.Add(E);
                }
                else
                {
                    break;
                }
            }
            Curve3D lastCurve = Tool.Curve(lastEdge, out ff, out ll)!;
            Vertex lfVert = Exploration.FirstVertex(lastEdge, true);
            Point3D lfPnt = Tool.Pnt(lfVert);
            LibraryMakeEdge el = new(lastCurve, lfPnt, _lastPnt);
            ww.Add(el.InternalEdge());
        }
        else
        {
            // only one edge : particular processing
            double fpar = IntPar(firstCurve, _firstPnt);
            double lpar = IntPar(firstCurve, _lastPnt);
            Curve3D c;
            if (fpar > lpar)
            {
                c = firstCurve.Reversed();
            }
            else
            {
                c = firstCurve;
            }

            LibraryMakeEdge ef = new(c, _firstPnt, _lastPnt);
            ww.Add(ef.InternalEdge());
        }

        LibraryMakeFace f = new(myPln.Pln(), ww.InternalWire(), true);
        Face fac = ShapeCasts.Face(f.InternalShape());

        if (!global::Kerf.Engine.Booleans.Algo.BRepAlgo.IsValid(fac))
        {
            profileOK = false;
            return profileOK;
        }

        if (concavite != 3)
        {
            // if concave : face is OK
            prof = fac;
        }
        else
        {
            // if not concave
            // CheckPnt : point slightly inside the material side
            // Bndface  : face/cut of the bounding box in the plane of the profile
            FClass2d cl = new(fac, Tool.Tolerance(fac));
            global::Kerf.Engine.Geometry.ElSLib.Parameters(myPln.Pln(), checkPnt, out double u, out double v);
            Point2D checkpnt2d = new(u, v);
            if (cl.Perform(checkpnt2d, true) == Out)
            {
                // If face is not the correct part of BndFace take the complementary
                Cut c = new(bndFace, fac);
                Explorer exp = new(c.InternalShape(), ShapeType.Wire);
                Wire w = ShapeCasts.Wire(exp.Current());
                LibraryMakeFace ffx = new(w);
                prof = ShapeCasts.Face(ffx.InternalShape());
            }
            else
            {
                // If face is the correct part of BndFace  : face is OK
                prof = fac;
            }
        }

        if (!global::Kerf.Engine.Booleans.Algo.BRepAlgo.IsValid(prof))
        {
            profileOK = false;
            return profileOK;
        }
        return profileOK;
    }

    /// <summary>The profile of a rib or groove without sliding: the contour extended to the bounding box.</summary>
    protected bool NoSlidingProfile(ref Face prof, bool revolRib, double myTol, ref int concavite,
                                    PlaneSurface myPln, double bnd, in Face bndFace, in Point3D checkPnt,
                                    in Face firstFace, in Face lastFace, in Vertex firstVertex,
                                    in Vertex lastVertex, in Edge firstEdge, in Edge lastEdge,
                                    bool onFirstFace, bool onLastFace)
    {
        bool profileOK = true;

        Vertex theFV = new();
        theFV.Nullify();
        Point3D theFirstpoint = new();
        Edge theLastEdge = new();
        theLastEdge.Nullify();

        Point3D firstpoint, lastpoint;
        Vector3D firstvect, lastvect;
        Wire w = new();
        BrepBuilder bb = new();
        bb.MakeWire(ref w);
        Edge falseFirstEdge = new(), falseLastEdge = new(), falseOnlyOne = new();

        Curve3D firstCurve = Tool.Curve(firstEdge, out double f1, out double l1)!;
        Curve3D lastCurve = Tool.Curve(lastEdge, out double f2, out double l2)!;

        LineCurve3D firstln, lastln;
        firstCurve.D1(f1, out firstpoint, out firstvect);
        lastln = new LineCurve3D(firstpoint, new Direction3D(-firstvect));
        lastCurve.D1(l2, out lastpoint, out lastvect);
        firstln = new LineCurve3D(lastpoint, new Direction3D(lastvect));

        Point3D pt = new();

        Curve2D ln2d1 = Projections.To2d(firstln, myPln.Pln())!;
        Curve2D ln2d2 = Projections.To2d(lastln, myPln.Pln())!;

        CurveCurveIntersection2D inter = new(ln2d1, ln2d2, global::Kerf.Engine.Foundation.Precision.Confusion());

        bool testOK = true;
        if (revolRib)
        {
            Direction3D d1, d2;
            d1 = firstln.Position().Direction();
            d2 = lastln.Position().Direction();
            if (d1.IsOpposite(d2, myTol))
            {
                double par1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(firstln.Lin(), _firstPnt);
                double par2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(lastln.Lin(), _lastPnt);
                if (par1 >= myTol || par2 >= myTol)
                {
                    concavite = 2; // parallel and concave
                }
            }
            if (d1.IsEqual(d2, myTol))
            {
                if (concavite == 3)
                {
                    testOK = false;
                }
            }
        }

        if (testOK)
        {
            if (inter.NbPoints() > 0)
            {
                Point2D P = inter.Point(1);
                myPln.D0(P.X(), P.Y(), out pt);
                double par = IntPar(firstln, pt);
                if (par > 0)
                {
                    concavite = 1; // concave
                }
            }
        }

        // ---Construction of the face profile
        if (concavite == 3)
        {
            if (onFirstFace)
            {
                falseFirstEdge = firstEdge;
                EdgeExtention(ref falseFirstEdge, bnd, true);
                Vertex vv1 = Exploration.FirstVertex(falseFirstEdge, true);
                firstpoint = Tool.Pnt(vv1);
                Curve3D cc = Tool.Curve(falseFirstEdge, out double f, out double l)!;
                cc.D1(f, out firstpoint, out firstvect);
                lastln = new LineCurve3D(firstpoint, new Direction3D(-firstvect));
                if (firstEdge.IsSame(lastEdge))
                {
                    falseOnlyOne = falseFirstEdge;
                }
                ln2d2 = Projections.To2d(lastln, myPln.Pln())!;
            }
            if (onLastFace)
            {
                if (!firstEdge.IsSame(lastEdge))
                {
                    falseLastEdge = lastEdge;
                }
                else
                {
                    if (falseOnlyOne.IsNull)
                    {
                        falseOnlyOne = lastEdge;
                    }
                    falseLastEdge = falseOnlyOne;
                }
                EdgeExtention(ref falseLastEdge, bnd, false);
                if (firstEdge.IsSame(lastEdge))
                {
                    falseOnlyOne = falseLastEdge;
                }
                Vertex vv2 = Exploration.LastVertex(falseLastEdge, true);
                lastpoint = Tool.Pnt(vv2);
                Curve3D cc = Tool.Curve(falseLastEdge, out double f, out double l)!;
                cc.D1(l, out lastpoint, out lastvect);
                lastpoint = Tool.Pnt(vv2);
                firstln = new LineCurve3D(lastpoint, new Direction3D(lastvect));
                ln2d1 = Projections.To2d(firstln, myPln.Pln())!;
            }

            Edge bndEdge1 = new(), bndEdge2 = new();
            Point3D bndPnt1 = new(), bndPnt2 = new(), lastPnt = new();
            Explorer expl = new();
            expl.Init(bndFace, ShapeType.Wire);
            WireExplorer explo = new();
            Wire bndWire = ShapeCasts.Wire(expl.Current());
            explo.Init(bndWire);
            for (; explo.More(); explo.Next())
            {
                Edge e = ShapeCasts.Edge(explo.Current());
                Curve3D c = Tool.Curve(e, out double first, out double last)!;
                Curve2D c2d = Projections.To2d(c, myPln.Pln())!;
                CurveCurveIntersection2D intcln1 = new(ln2d1, c2d, global::Kerf.Engine.Foundation.Precision.Confusion());
                if (intcln1.NbPoints() > 0)
                {
                    Point2D p2d = intcln1.Point(1);
                    myPln.D0(p2d.X(), p2d.Y(), out Point3D p);
                    double parl = IntPar(firstln, p);
                    double parc = IntPar(c, p);
                    if (parc >= first && parc <= last && parl >= 0)
                    {
                        bndEdge1 = e;
                        bndPnt1 = p;
                    }
                }

                CurveCurveIntersection2D intcln2 = new(ln2d2, c2d, global::Kerf.Engine.Foundation.Precision.Confusion());
                if (intcln2.NbPoints() > 0)
                {
                    Point2D p2d = intcln2.Point(1);
                    myPln.D0(p2d.X(), p2d.Y(), out Point3D p);
                    double parl = IntPar(lastln, p);
                    double parc = IntPar(c, p);
                    if (parc >= first && parc <= last && parl >= 0)
                    {
                        bndEdge2 = e;
                        bndPnt2 = p;
                    }
                }
                if (!bndEdge1.IsNull && !bndEdge2.IsNull)
                {
                    break;
                }
            }

            if (bndEdge1.IsNull || bndEdge2.IsNull)
            {
                profileOK = false;
                return profileOK;
            }

            Edge ee1;
            if (theLastEdge.IsNull)
            {
                LibraryMakeEdge e1 = new(lastpoint, bndPnt1);
                ee1 = ShapeCasts.Edge(e1.InternalShape());
            }
            else
            {
                Vertex v1 = Exploration.LastVertex(theLastEdge, true);
                LibraryMakeVertex v2 = new(bndPnt1);
                LibraryMakeEdge e1 = new(v1, v2.InternalVertex());
                ee1 = ShapeCasts.Edge(e1.InternalShape());
            }
            bb.Add(w, ee1);
            theLastEdge = ee1;
            if (theFV.IsNull)
            {
                theFV = Exploration.FirstVertex(ee1, true);
                theFirstpoint = Tool.Pnt(theFV);
            }

            if (bndEdge1.IsSame(bndEdge2))
            {
                Edge ee2, ee3;
                if (theLastEdge.IsNull)
                {
                    LibraryMakeEdge e2 = new(bndPnt1, bndPnt2);
                    ee2 = ShapeCasts.Edge(e2.InternalShape());
                }
                else
                {
                    Vertex v1 = Exploration.LastVertex(theLastEdge, true);
                    LibraryMakeVertex v2 = new(bndPnt2);
                    LibraryMakeEdge e2 = new(v1, v2.InternalVertex());
                    ee2 = ShapeCasts.Edge(e2.InternalShape());
                }
                bb.Add(w, ee2);
                theLastEdge = ee2;
                if (theFV.IsNull)
                {
                    theFV = Exploration.FirstVertex(ee2, true);
                    theFirstpoint = Tool.Pnt(theFV);
                }
                if (theLastEdge.IsNull)
                {
                    LibraryMakeEdge e3 = new(bndPnt2, firstpoint);
                    ee3 = ShapeCasts.Edge(e3.InternalShape());
                }
                else
                {
                    Vertex v1 = Exploration.LastVertex(theLastEdge, true);
                    LibraryMakeVertex v2 = new(firstpoint);
                    LibraryMakeEdge e3 = new(v1, v2.InternalVertex());
                    ee3 = ShapeCasts.Edge(e3.InternalShape());
                }
                bb.Add(w, ee3);
                theLastEdge = ee3;
                if (theFV.IsNull)
                {
                    theFV = Exploration.FirstVertex(ee3, true);
                    theFirstpoint = Tool.Pnt(theFV);
                }
            }
            else
            {
                explo.Init(bndWire);
                for (; explo.More(); explo.Next())
                {
                    Edge e = ShapeCasts.Edge(explo.Current());
                    if (e.IsSame(bndEdge1))
                    {
                        Point3D pp;
                        pp = Tool.Pnt(Exploration.LastVertex(e, true));
                        if (pp.Distance(bndPnt1) > Tool.Tolerance(e))
                        {
                            lastPnt = pp;
                        }
                        Edge eee;
                        if (theLastEdge.IsNull)
                        {
                            LibraryMakeEdge e2 = new(bndPnt1, lastPnt);
                            eee = ShapeCasts.Edge(e2.InternalShape());
                        }
                        else
                        {
                            Vertex v1 = Exploration.LastVertex(theLastEdge, true);
                            LibraryMakeVertex v2 = new(lastPnt);
                            LibraryMakeEdge e2 = new(v1, v2.InternalVertex());
                            eee = ShapeCasts.Edge(e2.InternalShape());
                        }
                        bb.Add(w, eee);
                        theLastEdge = eee;
                        if (theFV.IsNull)
                        {
                            theFV = Exploration.FirstVertex(eee, true);
                            theFirstpoint = Tool.Pnt(theFV);
                        }
                        break;
                    }
                }

                if (explo.More())
                {
                    explo.Next();
                    if (explo.Current().IsNull)
                    {
                        explo.Init(bndWire);
                    }
                }
                else
                {
                    explo.Init(bndWire);
                }
                bool fin = false;
                while (!fin)
                {
                    Edge e = ShapeCasts.Edge(explo.Current());
                    if (!e.IsSame(bndEdge2))
                    {
                        Point3D pp;
                        pp = Tool.Pnt(Exploration.LastVertex(e, true));
                        Edge eee1;
                        if (theLastEdge.IsNull)
                        {
                            LibraryMakeEdge ee = new(lastPnt, pp);
                            eee1 = ShapeCasts.Edge(ee.InternalShape());
                        }
                        else
                        {
                            Vertex v1 = Exploration.LastVertex(theLastEdge, true);
                            LibraryMakeVertex v2 = new(pp);
                            LibraryMakeEdge ee = new(v1, v2.InternalVertex());
                            eee1 = ShapeCasts.Edge(ee.InternalShape());
                        }
                        bb.Add(w, eee1);
                        theLastEdge = eee1;
                        if (theFV.IsNull)
                        {
                            theFV = Exploration.FirstVertex(eee1, true);
                            theFirstpoint = Tool.Pnt(theFV);
                        }
                        lastPnt = pp;
                    }
                    else
                    {
                        fin = true;
                        Edge eee2;
                        if (theLastEdge.IsNull)
                        {
                            LibraryMakeEdge ee = new(lastPnt, bndPnt2);
                            eee2 = ShapeCasts.Edge(ee.InternalShape());
                        }
                        else
                        {
                            Vertex v1 = Exploration.LastVertex(theLastEdge, true);
                            LibraryMakeVertex v2 = new(bndPnt2);
                            LibraryMakeEdge ee = new(v1, v2.InternalVertex());
                            eee2 = ShapeCasts.Edge(ee.InternalShape());
                        }
                        bb.Add(w, eee2);
                        theLastEdge = eee2;
                        if (theFV.IsNull)
                        {
                            theFV = Exploration.FirstVertex(eee2, true);
                            theFirstpoint = Tool.Pnt(theFV);
                        }
                        lastPnt = bndPnt2;
                    }
                    if (explo.More())
                    {
                        explo.Next();
                        if (explo.Current().IsNull)
                        {
                            explo.Init(bndWire);
                        }
                    }
                    else
                    {
                        explo.Init(bndWire);
                    }
                }

                Edge eee3;
                if (theLastEdge.IsNull)
                {
                    LibraryMakeEdge e3 = new(bndPnt2, firstpoint);
                    eee3 = ShapeCasts.Edge(e3.InternalShape());
                }
                else
                {
                    Vertex v1 = Exploration.LastVertex(theLastEdge, true);
                    LibraryMakeVertex v2 = new(firstpoint);
                    LibraryMakeEdge e3 = new(v1, v2.InternalVertex());
                    eee3 = ShapeCasts.Edge(e3.InternalShape());
                }
                bb.Add(w, eee3);
                theLastEdge = eee3;
                if (theFV.IsNull)
                {
                    theFV = Exploration.FirstVertex(eee3, true);
                    theFirstpoint = Tool.Pnt(theFV);
                }
            }
        }

        if (concavite == 1)
        {
            Edge eee4;
            if (theLastEdge.IsNull)
            {
                LibraryMakeEdge e = new(pt, firstpoint);
                eee4 = ShapeCasts.Edge(e.InternalShape());
            }
            else
            {
                Vertex v1 = Exploration.LastVertex(theLastEdge, true);
                LibraryMakeVertex v2 = new(firstpoint);
                LibraryMakeEdge e = new(v1, v2.InternalVertex());
                eee4 = ShapeCasts.Edge(e.InternalShape());
            }
            bb.Add(w, eee4);
            if (theFV.IsNull)
            {
                theFV = Exploration.FirstVertex(eee4, true);
                theFirstpoint = Tool.Pnt(theFV);
            }
            theLastEdge = eee4;
        }

        if (firstEdge.IsSame(lastEdge))
        {
            if (!_lfMap.IsBound(firstEdge))
            {
                NCollection_List<Shape> thelist = new();
                _lfMap.Bind(firstEdge, thelist);
            }
            if (onFirstFace || onLastFace)
            {
                Edge theEdge;
                Curve3D cc = Tool.Curve(falseOnlyOne, out double f, out double l)!;
                if (!theLastEdge.IsNull)
                {
                    Vertex v1 = Exploration.LastVertex(theLastEdge, true);
                    Vertex v2;
                    Point3D pp = Tool.Pnt(Exploration.LastVertex(falseOnlyOne, true));
                    if (!theFV.IsNull && theFirstpoint.Distance(pp) <= myTol)
                    {
                        v2 = theFV;
                    }
                    else
                    {
                        v2 = Exploration.LastVertex(falseOnlyOne, true);
                    }
                    LibraryMakeEdge e = new(cc, v1, v2);
                    theEdge = ShapeCasts.Edge(e.InternalShape());
                }
                else
                {
                    Vertex v1 = Exploration.FirstVertex(falseOnlyOne, true);
                    Vertex v2;
                    Point3D pp = Tool.Pnt(Exploration.LastVertex(falseOnlyOne, true));
                    if (!theFV.IsNull && theFirstpoint.Distance(pp) <= myTol)
                    {
                        v2 = theFV;
                    }
                    else
                    {
                        v2 = Exploration.LastVertex(falseOnlyOne, true);
                    }
                    LibraryMakeEdge e = new(cc, v1, v2);
                    theEdge = ShapeCasts.Edge(e.InternalShape());
                }
                _lfMap[firstEdge].Append(theEdge);
                bb.Add(w, theEdge);
                if (theFV.IsNull)
                {
                    theFV = Exploration.FirstVertex(theEdge, true);
                }
                theLastEdge = theEdge;
            }
            else
            {
                Curve3D cc = Tool.Curve(firstEdge, out double f, out double l)!;
                Edge theEdge;
                if (!theLastEdge.IsNull)
                {
                    Vertex v1 = Exploration.LastVertex(theLastEdge, true);
                    Vertex v2;
                    // Attention case Wire Reversed -> LastVertex without true
                    Point3D pp = Tool.Pnt(Exploration.LastVertex(firstEdge));
                    if (!theFV.IsNull && theFirstpoint.Distance(pp) <= myTol)
                    {
                        v2 = theFV;
                    }
                    else
                    {
                        v2 = Exploration.LastVertex(firstEdge);
                    }
                    LibraryMakeEdge e = new(cc, v1, v2);
                    theEdge = ShapeCasts.Edge(e.InternalShape());
                }
                else
                {
                    Vertex v1 = Exploration.FirstVertex(firstEdge, true);
                    Vertex v2;
                    Point3D pp = Tool.Pnt(Exploration.LastVertex(firstEdge, true));
                    if (!theFV.IsNull && theFirstpoint.Distance(pp) <= myTol)
                    {
                        v2 = theFV;
                    }
                    else
                    {
                        v2 = Exploration.LastVertex(firstEdge, true);
                    }
                    LibraryMakeEdge e = new(cc, v1, v2);
                    theEdge = ShapeCasts.Edge(e.InternalShape());
                }
                _lfMap[firstEdge].Append(theEdge);
                bb.Add(w, theEdge);
                if (theFV.IsNull)
                {
                    theFV = Exploration.FirstVertex(theEdge, true);
                }
                theLastEdge = theEdge;
            }
        }
        else
        {
            if (!_lfMap.IsBound(firstEdge))
            {
                NCollection_List<Shape> thelist1 = new();
                _lfMap.Bind(firstEdge, thelist1);
            }
            if (!onFirstFace)
            {
                Edge theEdge;
                Curve3D cc = Tool.Curve(firstEdge, out double f, out double l)!;
                if (!theLastEdge.IsNull)
                {
                    Vertex v1 = Exploration.LastVertex(theLastEdge, true);
                    Vertex v2 = Exploration.LastVertex(firstEdge, true);
                    LibraryMakeEdge e = new(cc, v1, v2);
                    theEdge = ShapeCasts.Edge(e.InternalShape());
                }
                else
                {
                    theEdge = firstEdge;
                }
                _lfMap[firstEdge].Append(theEdge);
                bb.Add(w, theEdge);
                if (theFV.IsNull)
                {
                    theFV = Exploration.FirstVertex(theEdge, true);
                }
                theLastEdge = theEdge;
            }
            else
            {
                Edge theEdge;
                Curve3D cc = Tool.Curve(falseFirstEdge, out double f, out double l)!;
                if (!theLastEdge.IsNull)
                {
                    Vertex v1 = Exploration.LastVertex(theLastEdge, true);
                    Vertex v2 = Exploration.LastVertex(falseFirstEdge, true);
                    LibraryMakeEdge e = new(cc, v1, v2);
                    theEdge = ShapeCasts.Edge(e.InternalShape());
                }
                else
                {
                    theEdge = falseFirstEdge;
                }
                _lfMap[firstEdge].Append(theEdge);
                bb.Add(w, theEdge);
                if (theFV.IsNull)
                {
                    theFV = Exploration.FirstVertex(theEdge, true);
                }
                theLastEdge = theEdge;
            }

            WireExplorer ex = new(_wire);
            for (; ex.More(); ex.Next())
            {
                Edge E = ex.Current();
                if (E.IsSame(firstEdge))
                {
                    break;
                }
            }

            ex.Next();

            for (; ex.More(); ex.Next())
            {
                Edge E = ex.Current();
                if (!E.IsSame(lastEdge))
                {
                    if (!_lfMap.IsBound(E))
                    {
                        NCollection_List<Shape> thelist2 = new();
                        _lfMap.Bind(E, thelist2);
                    }
                    Edge eee;
                    Curve3D cc = Tool.Curve(E, out double f, out double l)!;
                    if (!theLastEdge.IsNull)
                    {
                        Vertex v1 = Exploration.LastVertex(theLastEdge, true);
                        Vertex v2 = Exploration.LastVertex(E, true);
                        LibraryMakeEdge e = new(cc, v1, v2);
                        eee = ShapeCasts.Edge(e.InternalShape());
                    }
                    else
                    {
                        eee = E;
                    }
                    _lfMap[E].Append(eee);
                    bb.Add(w, eee);
                    if (theFV.IsNull)
                    {
                        theFV = Exploration.FirstVertex(eee, true);
                    }
                    theLastEdge = eee;
                }
                else
                {
                    break;
                }
            }

            if (!onLastFace)
            {
                if (!firstEdge.IsSame(lastEdge))
                {
                    Edge edg = ShapeCasts.Edge(ex.Current());
                    if (!_lfMap.IsBound(edg))
                    {
                        NCollection_List<Shape> thelist3 = new();
                        _lfMap.Bind(edg, thelist3);
                    }
                    Edge eee;
                    Curve3D cc = Tool.Curve(edg, out double f, out double l)!;
                    if (!theLastEdge.IsNull)
                    {
                        Vertex v1 = Exploration.LastVertex(theLastEdge, true);
                        Vertex v2;
                        Point3D pp = Tool.Pnt(Exploration.LastVertex(edg, true));
                        if (!theFV.IsNull && theFirstpoint.Distance(pp) <= myTol)
                        {
                            v2 = theFV;
                        }
                        else
                        {
                            v2 = Exploration.LastVertex(edg, true);
                        }
                        LibraryMakeEdge e = new(cc, v1, v2);
                        eee = ShapeCasts.Edge(e.InternalShape());
                    }
                    else
                    {
                        Vertex v1 = Exploration.FirstVertex(edg, true);
                        Vertex v2;
                        Point3D pp = Tool.Pnt(Exploration.LastVertex(edg, true));
                        if (!theFV.IsNull && theFirstpoint.Distance(pp) <= myTol)
                        {
                            v2 = theFV;
                        }
                        else
                        {
                            v2 = Exploration.LastVertex(edg, true);
                        }
                        LibraryMakeEdge e = new(cc, v1, v2);
                        eee = ShapeCasts.Edge(e.InternalShape());
                    }
                    _lfMap[edg].Append(eee);
                    bb.Add(w, eee);
                    if (theFV.IsNull)
                    {
                        theFV = Exploration.FirstVertex(eee, true);
                    }
                    theLastEdge = eee;
                }
                else
                {
                    Edge eee;
                    if (!_lfMap.IsBound(lastEdge))
                    {
                        NCollection_List<Shape> thelist4 = new();
                        _lfMap.Bind(lastEdge, thelist4);
                    }
                    Curve3D cc = Tool.Curve(falseOnlyOne, out double f, out double l)!;
                    if (!theLastEdge.IsNull)
                    {
                        Vertex v1 = Exploration.LastVertex(theLastEdge, true);
                        Vertex v2;
                        Point3D pp = Tool.Pnt(Exploration.LastVertex(falseOnlyOne, true));
                        if (!theFV.IsNull && theFirstpoint.Distance(pp) <= myTol)
                        {
                            v2 = theFV;
                        }
                        else
                        {
                            v2 = Exploration.LastVertex(falseOnlyOne, true);
                        }
                        LibraryMakeEdge e = new(cc, v1, v2);
                        eee = ShapeCasts.Edge(e.InternalShape());
                    }
                    else
                    {
                        Vertex v1 = Exploration.FirstVertex(falseOnlyOne, true);
                        Vertex v2;
                        Point3D pp = Tool.Pnt(Exploration.LastVertex(falseOnlyOne, true));
                        if (!theFV.IsNull && theFirstpoint.Distance(pp) <= myTol)
                        {
                            v2 = theFV;
                        }
                        else
                        {
                            v2 = Exploration.LastVertex(falseOnlyOne, true);
                        }
                        LibraryMakeEdge e = new(cc, v1, v2);
                        eee = ShapeCasts.Edge(e.InternalShape());
                    }
                    _lfMap[lastEdge].Append(eee);
                    bb.Add(w, eee);
                    if (theFV.IsNull)
                    {
                        theFV = Exploration.FirstVertex(eee, true);
                    }
                    theLastEdge = eee;
                }
            }
            else
            {
                Edge eee;
                if (!_lfMap.IsBound(lastEdge))
                {
                    NCollection_List<Shape> thelist5 = new();
                    _lfMap.Bind(lastEdge, thelist5);
                }
                Curve3D cc = Tool.Curve(falseLastEdge, out double f, out double l)!;
                if (!theLastEdge.IsNull)
                {
                    Vertex v1 = Exploration.LastVertex(theLastEdge, true);
                    Vertex v2;
                    Point3D pp = Tool.Pnt(Exploration.LastVertex(falseLastEdge, true));
                    if (!theFV.IsNull && theFirstpoint.Distance(pp) <= myTol)
                    {
                        v2 = theFV;
                    }
                    else
                    {
                        v2 = Exploration.LastVertex(falseLastEdge, true);
                    }
                    LibraryMakeEdge e = new(cc, v1, v2);
                    eee = ShapeCasts.Edge(e.InternalShape());
                }
                else
                {
                    Vertex v1 = Exploration.FirstVertex(falseLastEdge, true);
                    Vertex v2;
                    Point3D pp = Tool.Pnt(Exploration.LastVertex(falseLastEdge, true));
                    if (!theFV.IsNull && theFirstpoint.Distance(pp) <= myTol)
                    {
                        v2 = theFV;
                    }
                    else
                    {
                        v2 = Exploration.LastVertex(falseLastEdge, true);
                    }
                    LibraryMakeEdge e = new(cc, v1, v2);
                    eee = ShapeCasts.Edge(e.InternalShape());
                }
                _lfMap[lastEdge].Append(eee);
                bb.Add(w, eee);
                if (theFV.IsNull)
                {
                    theFV = Exploration.FirstVertex(eee, true);
                }
                theLastEdge = eee;
            }
        }

        if (concavite == 1)
        {
            Edge eef;
            if (theLastEdge.IsNull)
            {
                LibraryMakeEdge ef = new(lastpoint, pt);
                eef = ShapeCasts.Edge(ef.InternalShape());
            }
            else
            {
                Vertex v1 = Exploration.LastVertex(theLastEdge, true);
                LibraryMakeVertex vv = new(pt);
                Vertex v2 = ShapeCasts.Vertex(vv.InternalShape());
                if (!theFV.IsNull && pt.Distance(theFirstpoint) <= myTol)
                {
                    v2 = theFV;
                }
                LibraryMakeEdge ef = new(v1, v2);
                eef = ShapeCasts.Edge(ef.InternalShape());
            }
            bb.Add(w, eef);
            if (theFV.IsNull)
            {
                theFV = Exploration.FirstVertex(eef, true);
            }
            theLastEdge = eef;
        }

        if (concavite == 2)
        {
            LibraryMakeEdge ee = new(lastpoint, firstpoint);
            Edge e = ee.InternalEdge();
            bb.Add(w, e);
        }

        w.Closed(Tool.IsClosed(w));
        LibraryMakeFace fa = new(myPln.Pln(), w, true);
        Face fac = ShapeCasts.Face(fa.InternalShape());

        if (!global::Kerf.Engine.Booleans.Algo.BRepAlgo.IsValid(fac))
        {
            profileOK = false;
            return profileOK;
        }

        if (concavite == 3)
        {
            FClass2d cl = new(fac, Tool.Tolerance(fac));
            global::Kerf.Engine.Geometry.ElSLib.Parameters(myPln.Pln(), checkPnt, out double u, out double v);
            Point2D checkpnt2d = new(u, v);
            if (cl.Perform(checkpnt2d, true) == Out)
            {
                Cut c = new(bndFace, fac);
                Explorer exp = new(c.InternalShape(), ShapeType.Wire);
                UpdateDescendants(c, c.InternalShape(), false);
                Wire ww = ShapeCasts.Wire(exp.Current());
                LibraryMakeFace ff = new(ww);
                prof = ShapeCasts.Face(ff.InternalShape());
            }
            else
            {
                prof = fac;
            }
        }
        else
        {
            prof = fac;
        }

        if (!global::Kerf.Engine.Booleans.Algo.BRepAlgo.IsValid(prof))
        {
            profileOK = false;
            return profileOK;
        }
        return profileOK;
    }

    protected void UpdateDescendants(BooleanOperation aBOP, in Shape s, bool skipFace = false)
    {
        NCollection_List<Shape>.Iterator it = new(), it2 = new();
        Explorer exp = new();

        foreach (KeyValuePair<Shape, NCollection_List<Shape>> itdm in _map)
        {
            Shape orig = itdm.Key;
            if (skipFace && orig.ShapeType() == ShapeType.Face)
            {
                continue;
            }
            // OCCT iterates this address-keyed map into the list: insertion order here (decision 1054)
            NCollection_IndexedMap<Shape> newdsc = new(ShapeHasher.Instance);

            for (it.Initialize(itdm.Value); it.More(); it.Next())
            {
                Shape sh = it.Value();
                if (sh.ShapeType() != ShapeType.Face)
                {
                    continue;
                }
                Face fdsc = ShapeCasts.Face(it.Value());
                for (exp.Init(s, ShapeType.Face); exp.More(); exp.Next())
                {
                    if (exp.Current().IsSame(fdsc))
                    { // preserved
                        newdsc.Add(fdsc);
                        break;
                    }
                }
                if (!exp.More())
                {
                    NCollection_List<Shape> lm = aBOP.InternalModified(fdsc);
                    it2.Initialize(lm);
                    for (; it2.More(); it2.Next())
                    {
                        Shape aS = it2.Value();
                        newdsc.Add(aS);
                    }
                }
            }
            _map.ChangeFind(orig).Clear();
            foreach (Shape itm in newdsc)
            {
                // check the belonging to the shape...
                for (exp.Init(s, ShapeType.Face); exp.More(); exp.Next())
                {
                    if (exp.Current().IsSame(itm))
                    {
                        _map.ChangeFind(orig).Append(itm);
                        break;
                    }
                }
            }
        }
    }
}
