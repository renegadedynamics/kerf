// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Face.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Face.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Healing.Checks.StatusCheck;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Healing.Checks;

/// <summary>
/// The checks of a face: it has a surface (<see cref="Minimum"/>); its wires don't intersect
/// (<see cref="IntersectWires"/>), one of them contains all the others (<see cref="ClassifyWires"/>), and they are
/// oriented accordingly (<see cref="OrientationOfWires"/>).
/// </summary>
/// <remarks>
/// <see cref="ClassifyWires"/> classifies each wire in a face of its own, an unfrozen copy of the face's TShape
/// sharing its surface, built for the call and never handed out; the checked face is only read.
/// </remarks>
public class FaceCheck : ResultCheck
{
    private bool _intdone;
    private StatusCheck _intres;
    private bool _imbdone;
    private StatusCheck _imbres;
    private bool _oridone;
    private StatusCheck _orires;

    private readonly Dictionary<Shape, NCollection_List<Shape>> _mapImb =
        new(ShapeHasher.Instance);

    private bool _gctrl;

    /// <summary>Checks the face on its own (<see cref="Minimum"/>).</summary>
    public FaceCheck(in Face f)
    {
        _intres = StatusCheck.NoError;
        _imbres = StatusCheck.NoError;
        _orires = StatusCheck.NoError;
        Init(f);
        _intdone = false;
        _imbdone = false;
        _oridone = false;
        _gctrl = true;
    }

    // OCCT's DataMapOfShapeBox2d: NCollection_DataMap<TopoDS_Shape, Bnd_Box2d> with the default hasher, which
    // compares shapes with operator== (IsEqual: the orientation counts)
    private static Dictionary<Shape, Box2d> NewDataMapOfShapeBox2d()
        => new(EqualityComparer<Shape>.Default);

    /// <summary>The status list of the face itself, read under the lock in parallel mode.</summary>
    private NCollection_List<StatusCheck> LockedStatusList()
    {
        bool locked = _isParallel;
        if (locked)
        {
            _mutex.Enter();
        }

        try
        {
            return _map.GetValue(_shape);
        }
        finally
        {
            if (locked)
            {
                _mutex.Exit();
            }
        }
    }

    /// <summary>Checks that the face has a surface.</summary>
    public override void Minimum()
    {
        if (!_min)
        {
            var newList = new NCollection_List<StatusCheck>();
            NCollection_List<StatusCheck> lst = _map.SetValueRef(_shape, newList);

            var tf = (BrepTFace)_shape.TShape()!;
            if (tf.Surface() is null)
            {
                BRepCheckCheck.Add(lst, StatusCheck.NoSurface);
            }
            else
            {
                // Flag natural restriction???
            }

            if (lst.IsEmpty())
            {
                lst.Append(StatusCheck.NoError);
            }

            _min = true;
        }
    }

    /// <summary>Checks only that the face is in <paramref name="s"/>.</summary>
    public override void InContext(in Shape s)
    {
        NCollection_List<StatusCheck> hList;
        {
            bool locked = _isParallel;
            if (locked)
            {
                _mutex.Enter();
            }

            try
            {
                if (_map.ContainsKey(s))
                {
                    return;
                }

                var newList = new NCollection_List<StatusCheck>();
                hList = _map.SetValueRef(s, newList);
            }
            finally
            {
                if (locked)
                {
                    _mutex.Exit();
                }
            }
        }

        NCollection_List<StatusCheck> lst = hList;

        var exp = new Explorer(s, ShapeType.Face);
        for (; exp.More(); exp.Next())
        {
            if (exp.Current().IsSame(_shape))
            {
                break;
            }
        }

        if (!exp.More())
        {
            BRepCheckCheck.Add(lst, StatusCheck.SubshapeNotInShape);
            return;
        }

        if (lst.IsEmpty())
        {
            lst.Append(StatusCheck.NoError);
        }
    }

    /// <summary>Nothing more than the minimum.</summary>
    public override void Blind()
    {
        if (!_blind)
        {
            // nothing more than in the minimum
            _blind = true;
        }
    }

    /// <summary>
    /// Checks that no wire is met twice and that no two wires intersect in the parametric space of the face, except
    /// at common vertices. With <paramref name="update"/>, the status is added to the face's statuses.
    /// </summary>
    public StatusCheck IntersectWires(bool update = false)
    {
        NCollection_List<StatusCheck> statusList = LockedStatusList();
        if (_intdone)
        {
            if (update)
            {
                BRepCheckCheck.Add(statusList, _intres);
            }

            return _intres;
        }

        _intdone = true;
        _intres = StatusCheck.NoError;
        // This method has to be called by an analyzer. It is assumed that
        // each edge has a correct 2d representation on the face.

        var exp1 = new Explorer();
        var exp2 = new Explorer();

        // the wires are mapped
        exp1.Init(_shape.Oriented(Forward), ShapeType.Wire);
        while (exp1.More())
        {
            if (!_mapImb.ContainsKey(exp1.Current()))
            {
                // OCCT.NET: OCCT binds a copy of one empty list; each wire gets its own
                _mapImb[exp1.Current()] = new NCollection_List<Shape>();
            }
            else
            {
                // the same wire is met twice...
                _intres = StatusCheck.RedundantWire;
                if (update)
                {
                    BRepCheckCheck.Add(statusList, _intres);
                }

                return _intres;
            }

            exp1.Next();
        }

        var c = new CurveAdaptor2D();
        double first, last;
        Dictionary<Shape, Box2d> mapShapeBox2d = NewDataMapOfShapeBox2d();
        for (exp1.Init(_shape, ShapeType.Wire); exp1.More(); exp1.Next())
        {
            Wire wire = ShapeCasts.Wire(exp1.Current());
            // create 2d boxes for all edges from wire
            var boxW = new Box2d();
            for (exp2.Init(wire, ShapeType.Edge); exp2.More(); exp2.Next())
            {
                Edge edge = ShapeCasts.Edge(exp2.Current());
                c.Load(Tool.CurveOnSurface(edge, ShapeCasts.Face(_shape), out first, out last)!);
                // To avoid exception in Segment if C1 is BSpline
                if (c.FirstParameter() > first)
                {
                    first = c.FirstParameter();
                }

                if (c.LastParameter() < last)
                {
                    last = c.LastParameter();
                }

                var boxE = new Box2d();
                Add2dCurve.Add(c, first, last, 0.0, ref boxE);
                mapShapeBox2d[edge] = boxE;
                boxW.Add(boxE);
            }

            mapShapeBox2d[wire] = boxW;
        }

        int nbwire, index, indexbis;
        nbwire = _mapImb.Count;

        index = 1;
        while (index < nbwire)
        {
            for (exp1.Init(_shape, ShapeType.Wire), indexbis = 0; exp1.More(); exp1.Next())
            {
                indexbis++;
                if (indexbis == index)
                {
                    break;
                }
            }

            Wire wir1 = ShapeCasts.Wire(exp1.Current());
            // to reduce the number of calls Intersect(wir1,wir2)
            Box2d box1 = new(), box2 = new();
            if (mapShapeBox2d.ContainsKey(wir1))
            {
                box1 = mapShapeBox2d.GetValue(wir1);
            }

            exp1.Next();
            for (; exp1.More(); exp1.Next())
            {
                Wire wir2 = ShapeCasts.Wire(exp1.Current());
                if (mapShapeBox2d.ContainsKey(wir2))
                {
                    box2 = mapShapeBox2d.GetValue(wir2);
                }

                if (!box1.IsVoid && !box2.IsVoid && box1.IsOut(box2))
                {
                    continue;
                }

                if (Intersect(wir1, wir2, ShapeCasts.Face(_shape), mapShapeBox2d))
                {
                    _intres = StatusCheck.IntersectingWires;
                    if (update)
                    {
                        BRepCheckCheck.Add(statusList, _intres);
                    }

                    return _intres;
                }
            }

            index++;
        }

        if (update)
        {
            BRepCheckCheck.Add(statusList, _intres);
        }

        return _intres;
    }

    /// <summary>
    /// Checks that one wire contains all the others, which contain nothing, or that no wire contains another (holes
    /// in an infinite face). Runs <see cref="IntersectWires"/> first. With <paramref name="update"/>, the status is
    /// added to the face's statuses.
    /// </summary>
    public StatusCheck ClassifyWires(bool update = false)
    {
        NCollection_List<StatusCheck> statusList = LockedStatusList();

        // It is assumed that each wire does not intersect any other one.
        if (_imbdone)
        {
            if (update)
            {
                BRepCheckCheck.Add(statusList, _imbres);
            }

            return _imbres;
        }

        _imbdone = true;
        _imbres = IntersectWires();
        if (_imbres != StatusCheck.NoError)
        {
            if (update)
            {
                BRepCheckCheck.Add(statusList, _imbres);
            }

            return _imbres;
        }

        int nbwire = _mapImb.Count;
        if (nbwire < 1)
        {
            if (update)
            {
                BRepCheckCheck.Add(statusList, _imbres);
            }

            return _imbres;
        }

        var b = new BrepBuilder();
        var exp1 = new Explorer();
        var exp2 = new Explorer();

        Face forwardFace = ShapeCasts.Face(_shape.Oriented(Forward));
        Dictionary<Shape, Box2d> wireBoxMap = NewDataMapOfShapeBox2d();
        // Precompute reliable UV bounds for cheap rejection of disjoint wire pairs.
        for (exp1.Init(forwardFace, ShapeType.Wire); exp1.More(); exp1.Next())
        {
            Wire wire = ShapeCasts.Wire(exp1.Current());
            var box = new Box2d();
            if (ComputeWireUVBounds(forwardFace, wire, ref box))
            {
                wireBoxMap[wire] = box;
            }
        }

        for (exp1.Init(forwardFace, ShapeType.Wire); exp1.More(); exp1.Next())
        {
            Wire wir1 = ShapeCasts.Wire(exp1.Current());
            // OCCT.NET: an unfrozen copy of the TShape, sharing the surface, never handed out
            Shape localShape = _shape.EmptyCopied();
            Face newFace = ShapeCasts.Face(localShape);

            newFace.Orientation(Forward);
            b.Add(newFace, wir1);

            var fClass2d = new FClass2d(newFace, global::Kerf.Engine.Foundation.Precision.PConfusion());
            bool wireBienOriente = false;
            if (fClass2d.PerformInfinitePoint() != Out)
            {
                wireBienOriente = true;
                // the given wire defines a hole
                _mapImb.Remove(wir1);
                _mapImb[wir1.Reversed()] = new NCollection_List<Shape>();
            }

            bool hasBox1 = wireBoxMap.TryGetValue(wir1, out Box2d box1);

            for (exp2.Init(forwardFace, ShapeType.Wire); exp2.More(); exp2.Next())
            {
                Wire wir2 = ShapeCasts.Wire(exp2.Current());
                if (wir2.IsSame(wir1))
                {
                    continue;
                }

                bool hasBox2 = wireBoxMap.TryGetValue(wir2, out Box2d box2);
                // Wires with disjoint UV bounds cannot contain one another.
                if (hasBox1 && hasBox2 && box1.IsOut(box2))
                {
                    continue;
                }

                if (IsInside(wir2, wireBienOriente, fClass2d, newFace))
                {
                    _mapImb.GetValue(wir1).Append(wir2);
                }
            }
        }

        // It is required to have 1 wire that contains all others, and the others should not
        // contain anything (case solid ended) or
        // the wires do not contain anything : in this case the wires should be
        // holes in an infinite face.
        Wire wext = new();
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> itm in _mapImb)
        {
            if (!itm.Value.IsEmpty())
            {
                if (wext.IsNull)
                {
                    wext = ShapeCasts.Wire(itm.Key);
                }
                else
                {
                    _imbres = StatusCheck.InvalidImbricationOfWires;
                    if (update)
                    {
                        BRepCheckCheck.Add(statusList, _imbres);
                    }

                    return _imbres;
                }
            }
        }

        if (!wext.IsNull)
        {
            // verifies that the list contains nbwire-1 elements
            if (_mapImb.GetValue(wext).Extent() != nbwire - 1)
            {
                _imbres = StatusCheck.InvalidImbricationOfWires;
                if (update)
                {
                    BRepCheckCheck.Add(statusList, _imbres);
                }

                return _imbres;
            }
        }

        // quit without errors
        if (update)
        {
            BRepCheckCheck.Add(statusList, _imbres);
        }

        return _imbres;
    }

    /// <summary>
    /// Checks that the outer wire is oriented as the face and the holes the other way. Runs
    /// <see cref="ClassifyWires"/> first. With <paramref name="update"/>, the status is added to the face's statuses.
    /// </summary>
    public StatusCheck OrientationOfWires(bool update = false)
    {
        NCollection_List<StatusCheck> statusList = LockedStatusList();
        // WARNING : it is assumed that the edges of a wire are correctly oriented
        bool infinite = _shape.Infinite();
        if (_oridone)
        {
            if (update)
            {
                BRepCheckCheck.Add(statusList, _orires);
            }

            return _orires;
        }

        _oridone = true;
        _orires = ClassifyWires();
        if (_orires != StatusCheck.NoError)
        {
            if (update)
            {
                BRepCheckCheck.Add(statusList, _orires);
            }

            return _orires;
        }

        int nbwire = _mapImb.Count;
        Wire wext = new();
        // OCCT's map iterator: the keys in the map's order (at most one wire contains others, ClassifyWires checked)
        Shape[] keys = [.. _mapImb.Select(p => p.Key)];
        int itm = 0;
        if (nbwire == 1)
        {
            if (!infinite)
            {
                wext = ShapeCasts.Wire(keys[itm]);
            }
        }
        else
        {
            for (; itm < keys.Length; itm++)
            {
                if (!_mapImb.GetValue(keys[itm]).IsEmpty())
                {
                    wext = ShapeCasts.Wire(keys[itm]);
                }
            }
        }

        if (wext.IsNull && !infinite)
        {
            if (nbwire > 0)
            {
                _orires = StatusCheck.InvalidImbricationOfWires;
            }

            if (update)
            {
                BRepCheckCheck.Add(statusList, _orires);
            }

            return _orires;
        }

        var exp = new Explorer(_shape.Oriented(Forward), ShapeType.Wire);
        for (; exp.More(); exp.Next())
        {
            Wire wir = ShapeCasts.Wire(exp.Current());
            if (!wext.IsNull && wir.IsSame(wext))
            {
                if (wir.Orientation() != wext.Orientation())
                {
                    // the exterior wire defines a hole
                    if (CheckThin(wir, _shape.Oriented(Forward)))
                    {
                        return _orires;
                    }

                    _orires = StatusCheck.BadOrientationOfSubshape;
                    if (update)
                    {
                        BRepCheckCheck.Add(statusList, _orires);
                    }

                    return _orires;
                }
            }
            else
            {
                for (itm = 0; itm < keys.Length; itm++)
                {
                    if (keys[itm].IsSame(wir))
                    {
                        break;
                    }
                }

                // No control on More()
                // OCCT.NET: OCCT reads the key of an exhausted iterator; every wire of the face is a key, so the
                // port throws if one isn't
                if (itm == keys.Length)
                {
                    throw new NoSuchObjectException("BRepCheck_Face::OrientationOfWires");
                }

                if (keys[itm].Orientation() == wir.Orientation())
                {
                    // the given wire does not define a hole
                    _orires = StatusCheck.BadOrientationOfSubshape;
                    if (update)
                    {
                        BRepCheckCheck.Add(statusList, _orires);
                    }

                    return _orires;
                }
            }
        }

        // quit without error
        // OCCT: adding NoError appends it after a fault already recorded (a face without surface, checked with
        // Update by the analyzer, has NoSurface then NoError)
        if (update)
        {
            BRepCheckCheck.Add(statusList, _orires);
        }

        return _orires;
    }

    /// <summary>Adds <see cref="UnorientableShape"/> to the face's statuses.</summary>
    public void SetUnorientable() => SetStatus(StatusCheck.UnorientableShape);

    /// <summary>Adds <paramref name="status"/> to the face's statuses.</summary>
    public void SetStatus(StatusCheck status)
    {
        bool locked = _isParallel;
        if (locked)
        {
            _mutex.Enter();
        }

        try
        {
            BRepCheckCheck.Add(_map.GetValue(_shape), status);
        }
        finally
        {
            if (locked)
            {
                _mutex.Exit();
            }
        }
    }

    /// <summary>
    /// True if <see cref="OrientationOfWires"/> found a fault, or (when it hasn't run) if the face is marked
    /// unorientable.
    /// </summary>
    public bool IsUnorientable()
    {
        if (_oridone)
        {
            return _orires != StatusCheck.NoError;
        }

        for (var itl = new NCollection_List<StatusCheck>.Iterator(_map.GetValue(_shape)); itl.More(); itl.Next())
        {
            if (itl.Value() == StatusCheck.UnorientableShape)
            {
                return true;
            }
        }

        return false;
    }

    /// <summary>Sets the geometric controls; turning them on makes the wire checks run again.</summary>
    public void GeometricControls(bool b)
    {
        if (_gctrl != b)
        {
            if (b)
            {
                _intdone = false;
                _imbdone = false;
                _oridone = false;
            }

            _gctrl = b;
        }
    }

    /// <summary>Whether the geometric controls are on.</summary>
    public bool GeometricControls() => _gctrl;

    private static bool Intersect(in Wire wir1,
                                  in Wire wir2,
                                  in Face f,
                                  Dictionary<Shape, Box2d> mapEdgeBox)
    {
        double inter2dTol = 1.0e-10;
        var exp1 = new Explorer();
        var exp2 = new Explorer();

        // Find common vertices of two wires - non-manifold case
        var mapW1 = new HashSet<Shape>(ShapeHasher.Instance);
        var commonVertices = new NCollection_Sequence<Shape>();
        for (exp1.Init(wir1, ShapeType.Vertex); exp1.More(); exp1.Next())
        {
            mapW1.Add(exp1.Current());
        }

        for (exp2.Init(wir2, ShapeType.Vertex); exp2.More(); exp2.Next())
        {
            Shape v = exp2.Current();
            if (mapW1.Contains(v))
            {
                commonVertices.Append(v);
            }
        }

        // MSV 03.04.2002: create pure surface adaptor to avoid UVBounds computation
        //                 due to performance problem
        var surf = new FaceSurfaceAdaptor(f, false);

        var pntSeq = new NCollection_Sequence<Point3D>();
        int i;
        for (i = 1; i <= commonVertices.Length(); i++)
        {
            Vertex v = ShapeCasts.Vertex(commonVertices[i]);
            Point2D p2d = Tool.Parameters(v, f);
            Point3D p = surf.Value(p2d.X(), p2d.Y());
            pntSeq.Append(p);
        }

        var c1 = new CurveAdaptor2D();
        var c2 = new CurveAdaptor2D();
        Point2D pfirst1, plast1, pfirst2, plast2;
        double first1, last1, first2, last2;
        var inter = new GInter();
        var myDomain1 = new TwoDDomain();
        var myDomain2 = new TwoDDomain();
        Box2d box1 = new(), box2 = new();

        for (exp1.Init(wir1, ShapeType.Edge); exp1.More(); exp1.Next())
        {
            Edge edg1 = ShapeCasts.Edge(exp1.Current());
            c1.Load(Tool.CurveOnSurface(edg1, f, out first1, out last1)!);
            // To avoid exception in Segment if C1 is BSpline - IFV
            if (c1.FirstParameter() > first1)
            {
                first1 = c1.FirstParameter();
            }

            if (c1.LastParameter() < last1)
            {
                last1 = c1.LastParameter();
            }

            box1.SetVoid();
            if (mapEdgeBox.ContainsKey(edg1))
            {
                box1 = mapEdgeBox.GetValue(edg1);
            }

            if (box1.IsVoid)
            {
                Add2dCurve.Add(c1, first1, last1, 0.0, ref box1);
            }

            for (exp2.Init(wir2, ShapeType.Edge); exp2.More(); exp2.Next())
            {
                Edge edg2 = ShapeCasts.Edge(exp2.Current());
                if (!edg1.IsSame(edg2))
                {
                    c2.Load(Tool.CurveOnSurface(edg2, f, out first2, out last2)!);
                    // To avoid exception in Segment if C2 is BSpline - IFV
                    if (c2.FirstParameter() > first2)
                    {
                        first2 = c2.FirstParameter();
                    }

                    if (c2.LastParameter() < last2)
                    {
                        last2 = c2.LastParameter();
                    }

                    box2.SetVoid();
                    if (mapEdgeBox.ContainsKey(edg2))
                    {
                        box2 = mapEdgeBox.GetValue(edg2);
                    }

                    if (box2.IsVoid)
                    {
                        Add2dCurve.Add(c2, first2, last2, 0.0, ref box2);
                    }

                    if (!box1.IsOut(box2))
                    {
                        Tool.UVPoints(edg1, f, out pfirst1, out plast1);
                        myDomain1.SetValues(pfirst1, first1, inter2dTol, plast1, last1, inter2dTol);
                        Tool.UVPoints(edg2, f, out pfirst2, out plast2);
                        myDomain2.SetValues(pfirst2, first2, inter2dTol, plast2, last2, inter2dTol);
                        inter.Perform(c1, myDomain1, c2, myDomain2, inter2dTol, inter2dTol);
                        if (!inter.IsDone)
                        {
                            return true;
                        }

                        if (inter.SegmentCount > 0)
                        {
                            if (pntSeq.IsEmpty())
                            {
                                return true;
                            }
                            else
                            {
                                int nbCoinc = 0;
                                for (i = 1; i <= inter.SegmentCount; i++)
                                {
                                    if (!inter.Segment(i).HasFirstPoint || !inter.Segment(i).HasLastPoint)
                                    {
                                        return true;
                                    }

                                    Point2D firstP2d = inter.Segment(i).FirstPoint().Value();
                                    Point2D lastP2d = inter.Segment(i).LastPoint().Value();
                                    Point3D firstP = surf.Value(firstP2d.X(), firstP2d.Y());
                                    Point3D lastP = surf.Value(lastP2d.X(), lastP2d.Y());
                                    for (int j = 1; j <= pntSeq.Length(); j++)
                                    {
                                        double tolv = Tool.Tolerance(ShapeCasts.Vertex(commonVertices[j]));
                                        if (firstP.IsEqual(pntSeq[j], tolv) || lastP.IsEqual(pntSeq[j], tolv))
                                        {
                                            nbCoinc++;
                                            break;
                                        }
                                    }
                                }

                                return nbCoinc != inter.SegmentCount;
                            }
                        }

                        if (inter.PointCount > 0)
                        {
                            if (pntSeq.IsEmpty())
                            {
                                return true;
                            }
                            else
                            {
                                int nbCoinc = 0;
                                for (i = 1; i <= inter.PointCount; i++)
                                {
                                    Point2D p2d = inter.Point(i).Value();
                                    Point3D p = surf.Value(p2d.X(), p2d.Y());
                                    for (int j = 1; j <= pntSeq.Length(); j++)
                                    {
                                        double tolv = Tool.Tolerance(ShapeCasts.Vertex(commonVertices[j]));
                                        tolv += 1.0e-8; // possible tolerance of intersection point
                                        double dd = p.SquareDistance(pntSeq[j]);
                                        if (dd <= tolv * tolv)
                                        {
                                            nbCoinc++;
                                            break;
                                        }
                                    }
                                }

                                return nbCoinc != inter.PointCount;
                            }
                        }
                    }
                }
            }
        }

        return false;
    }

    private static bool ComputeWireUVBounds(in Face face, in Wire wire, ref Box2d box)
    {
        if (!Tool.IsClosed(wire))
        {
            return false;
        }

        var aBox = new Box2d();
        for (var exp = new Explorer(wire, ShapeType.Edge); exp.More(); exp.Next())
        {
            Edge edge = ShapeCasts.Edge(exp.Current());
            Curve2D? pCurve = Tool.CurveOnSurface(edge, face, out double first, out double last);
            if (pCurve is null)
            {
                return false;
            }

            var curveAdaptor = new CurveAdaptor2D(pCurve);
            // IsInside() uses the original edge range, so a clamped box cannot safely reject the wire.
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(first) || global::Kerf.Engine.Foundation.Precision.IsInfinite(last)
                || first < curveAdaptor.FirstParameter() || last > curveAdaptor.LastParameter())
            {
                return false;
            }

            Add2dCurve.Add(curveAdaptor, first, last, global::Kerf.Engine.Foundation.Precision.PConfusion(), ref aBox);
        }

        if (aBox.IsVoid || aBox.IsOpenXmin || aBox.IsOpenXmax || aBox.IsOpenYmin
            || aBox.IsOpenYmax)
        {
            return false;
        }

        box = aBox;
        return true;
    }

    private static bool IsInside(in Wire wire,
                                 bool wireBienOriente,
                                 FClass2d fClass2d,
                                 in Face face)
    {
        double parameter, first, last;

        var explorer = new Explorer(wire, ShapeType.Edge);
        for (; explorer.More(); explorer.Next())
        {
            Edge edge = ShapeCasts.Edge(explorer.Current());
            Curve2D? curve2D = Tool.CurveOnSurface(edge, face, out first, out last);

            // Selects the parameter of point on the curve
            if (!global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(first) && !global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(last))
            {
                parameter = (first + last) * 0.5;

                // Edge is skipped if its parametric range is too small
                if (Math.Abs(parameter - first) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    continue;
                }

                // Edge is skipped if its length is too small
                Curve3D? curve = Tool.Curve(edge, out double first3D, out double last3D);
                if (curve is null)
                {
                    continue;
                }

                // Compute start point of edge
                // OCCT evaluates the 3D curve at the pcurve's first parameter
                Point3D points0 = curve.Value(first);
                // Compute middle point of edge
                Point3D points1 = curve.Value((first3D + last3D) / 2.0);
                if (points0.Distance(points1) < global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    continue;
                }
            }
            else
            {
                if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(first) && global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(last))
                {
                    parameter = 0.0;
                }
                else if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(first))
                {
                    parameter = last - 1.0;
                }
                else
                {
                    parameter = first + 1.0;
                }
            }

            // Find point on curve (edge)
            Point2D point2D = curve2D!.Value(parameter);
            // Compute the topological position of a point relative to face
            Containment state = fClass2d.Perform(point2D, false);

            if (wireBienOriente)
            {
                return state == Out;
            }
            else
            {
                return state == In;
            }
        }

        return false;
    }

    private static bool CheckThin(in Shape w, in Shape f)
    {
        Face aF = ShapeCasts.Face(f);
        Wire aW = ShapeCasts.Wire(w);

        int nbE = 0;
        var lE = new NCollection_List<Shape>();
        var exp = new Explorer(aW, ShapeType.Edge);
        for (; exp.More(); exp.Next())
        {
            Shape s = exp.Current();
            lE.Append(s);
            nbE++;
        }

        if (nbE != 2)
        {
            return false;
        }

        Edge e1 = ShapeCasts.Edge(lE.First());
        Edge e2 = ShapeCasts.Edge(lE.Last());

        Exploration.Vertices(e1, out Vertex v1, out Vertex v2);
        Exploration.Vertices(e2, out Vertex v3, out Vertex v4);

        if (v1.IsNull || v2.IsNull || v3.IsNull || v4.IsNull)
        {
            return false;
        }

        if (v1.IsSame(v2) || v3.IsSame(v4))
        {
            return false;
        }

        bool sF = false, sL = false;
        if (v1.IsSame(v3) || v1.IsSame(v4))
        {
            sF = true;
        }

        if (v2.IsSame(v3) || v2.IsSame(v4))
        {
            sL = true;
        }

        if (!sF || !sL)
        {
            return false;
        }

        Orientation e1or = e1.Orientation();
        Orientation e2or = e2.Orientation();

        Curve2D? pc1 = Tool.CurveOnSurface(e1, aF, out double f1, out double l1);
        Curve2D? pc2 = Tool.CurveOnSurface(e2, aF, out double f2, out double l2);

        if (pc1 is null || pc2 is null)
        {
            return false;
        }

        double d1 = Math.Abs(l1 - f1) / 100.0;
        double d2 = Math.Abs(l2 - f2) / 100.0;
        double m1 = (l1 + f1) * 0.5;
        double m2 = (l2 + f2) * 0.5;

        Point2D p1f = pc1.Value(m1 - d1);
        Point2D p1l = pc1.Value(m1 + d1);
        Point2D p2f = pc2.Value(m2 - d2);
        Point2D p2l = pc2.Value(m2 + d2);

        var vc1 = new Vector2D(p1f, p1l);
        var vc2 = new Vector2D(p2f, p2l);

        return (vc1 * vc2) < 0.0 || e1or != e2or;
    }
}
