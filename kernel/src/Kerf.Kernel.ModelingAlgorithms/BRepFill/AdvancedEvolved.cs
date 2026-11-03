// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_AdvancedEvolved.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_AdvancedEvolved.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2018-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans;
using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Classification;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Healing.Upgrade;
using Kerf.Engine.Measurement;
using Kerf.Engine.Measurement.Distance;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Measurement.Distance.SupportType;
using static Kerf.Engine.Construction.Filling.TransitionStyle;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Construction.Filling;

// BRepFill_AdvancedEvolved_DEBUG blocks (dumps and BinTools files) are left out, and with them the debug path.

/// <summary>
/// An evolved volume from a planar spine and a profile: the profile swept along the spine (a round-transition pipe
/// shell), closed with planar lids, cut by its self-intersections into a solid (BRepOffsetAPI_MakeEvolved's volume
/// mode).
/// </summary>
/// <remarks>
/// A working object (decision 21). It works on the sweep's own shapes, which it splits, unifies and whose vertex
/// tolerances it reduces in place; the spine and profile are copied at the entry when they are frozen, so that none
/// of the caller's vertices that the sweep might share is written, and the caller's sub-shapes left unchanged stand
/// in the result (decision 1047). <see cref="Shape"/> freezes the result.
/// </remarks>
public sealed class AdvancedEvolved
{
    private const uint BRepFill_AdvancedEvolved_Empty = 0;
    private const uint BRepFill_AdvancedEvolved_NotPlanarSpine = 1;
    private const uint BRepFill_AdvancedEvolved_SweepError = 2;
    private const uint BRepFill_AdvancedEvolved_NoLids = 3;
    private const uint BRepFill_AdvancedEvolved_NotSolid = 4;
    private const uint BRepFill_AdvancedEvolved_NotVolume = 5;
    private const uint BRepFill_AdvancedEvolved_OK = uint.MaxValue;

    private const double APipeLinearTolerance = 1.0e-4;
    private const double APipeAngularTolerance = 1.0e-2;

    private uint _errorStatus;
    private Wire _spine = new();
    private Wire _profile = new();
    private Shape _pipeShell = new();
    private Compound _topBottom = new(); // Lids can be split on several faces
    private Shape _result = new();
    private double _fuzzyValue;
    private bool _isParallel;

    /// <summary>An empty algorithm.</summary>
    public AdvancedEvolved()
    {
        _errorStatus = BRepFill_AdvancedEvolved_Empty;
        _fuzzyValue = 0.0;
        _isParallel = true;
    }

    /// <summary>True if the volume was built.</summary>
    public bool IsDone() => _errorStatus == BRepFill_AdvancedEvolved_OK;

    /// <summary>True if the volume was built; <paramref name="errorCode"/> is the status (uint.MaxValue when done).</summary>
    public bool IsDone(out uint errorCode)
    {
        errorCode = _errorStatus;
        return _errorStatus == BRepFill_AdvancedEvolved_OK;
    }

    /// <summary>The resulting shape (frozen).</summary>
    public Shape Shape()
    {
        _result.TShape()?.FreezeGraph();
        return _result;
    }

    /// <summary><see cref="Shape"/> without freezing, for ported code.</summary>
    internal Shape InternalShape() => _result;

    /// <summary>OCCT's directory for the debug shapes; the port writes none, so it is ignored.</summary>
    public void SetTemporaryDirectory(string path)
    {
    }

    /// <summary>Sets or unsets the parallel mode of the Booleans.</summary>
    public void SetParallelMode(bool val) => _isParallel = val;

    //=================================================================================================

    private bool PerformBoolean(NCollection_List<Shape> argsList, ref Shape result)
    {
        PaveFiller pf = new();

        pf.SetArguments(argsList);
        pf.SetRunParallel(_isParallel);
        pf.SetFuzzyValue(_fuzzyValue);

        pf.Perform();
        if (pf.HasErrors)
        {
            return false;
        }

        BooleanBuilder builder = new();
        builder.SetArguments(argsList);

        builder.SetRunParallel(_isParallel);
        builder.PerformWithFiller(pf);
        if (builder.HasErrors)
        {
            return false;
        }

        result = builder.InternalShape();
        return true;
    }

    //=================================================================================================

    private void GetSpineAndProfile(in Wire spine, in Wire profile)
    {
        _spine = spine;
        _profile = profile;

        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> mvep = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(profile, ShapeType.Vertex, ShapeType.Edge, mvep);

        Vector3D n2 = new();
        Point3D loc = new();

        for (int i = 1; i <= mvep.Extent(); i++)
        {
            Vertex vc = ShapeCasts.Vertex(mvep.FindKey(i));

            NCollection_List<Shape> le = mvep.FindFromIndex(i);

            if (le.Extent() < 2)
            {
                continue;
            }

            Edge e1 = ShapeCasts.Edge(le.First());
            Edge e2 = ShapeCasts.Edge(le.Last());

            EdgeCurveAdaptor ac1 = new(e1), ac2 = new(e2);

            double par1 = Tool.Parameter(vc, e1);
            double par2 = Tool.Parameter(vc, e2);

            ac1.D1(par1, out Point3D p, out Vector3D t1);
            ac1.D1(par2, out p, out Vector3D t2); // OCCT: anAC1, not anAC2

            n2 = t1.Crossed(t2);

            if (n2.SquareMagnitude() > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
            {
                loc = Tool.Pnt(vc);
                break;
            }
        }

        DistanceShapeShape extr = new();
        extr.LoadS1(spine);

        if (n2.SquareMagnitude() > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            Plane3D pln = new(loc, new Direction3D(n2));
            LibraryMakeFace mf = new(pln, profile);
            if (!mf.IsDone)
            {
                return;
            }

            extr.LoadS2(mf.InternalFace());
        }
        else
        {
            extr.LoadS2(profile);
        }

        if (!extr.Perform())
        {
            return;
        }

        int nbSol = extr.SolutionCount;
        if (nbSol < 1)
        {
            return;
        }

        double distMin = RealLast();
        int idxMin = 0;

        for (int solId = 1; solId <= nbSol; solId++)
        {
            double d = extr.Value(); // OCCT: the same value for every solution
            if (d > distMin)
            {
                continue;
            }

            distMin = d;
            idxMin = solId;
        }

        SupportType extrType2 = extr.SupportTypeShape2(idxMin);

        if (distMin < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            extrType2 = SupportType.IsInFace;
        }

        switch (extrType2)
        {
            case SupportType.IsInFace:
                if (extr.SupportTypeShape1(idxMin) == SupportType.IsVertex)
                {
                    Vertex v = ShapeCasts.Vertex(extr.SupportOnShape1(idxMin));
                    NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> mves =
                        new(ShapeHasher.Instance);
                    Exploration.MapShapesAndAncestors(spine, ShapeType.Vertex, ShapeType.Edge, mves);

                    NCollection_List<Shape> le = mves.FindFromKey(v);

                    Edge e1 = ShapeCasts.Edge(le.First());
                    Edge e2 = ShapeCasts.Edge(le.Last());

                    EdgeCurveAdaptor ac1 = new(e1), ac2 = new(e2);

                    double par1 = Tool.Parameter(v, e1);
                    double par2 = Tool.Parameter(v, e2);

                    ac1.D1(par1, out Point3D p, out Vector3D t1);
                    ac1.D1(par2, out p, out Vector3D t2); // OCCT: anAC1, not anAC2

                    // Find minimal sine
                    double sqT1 = Max(t1.SquareMagnitude(), 1.0 / global::Kerf.Engine.Foundation.Precision.Infinite());
                    double sqT2 = Max(t2.SquareMagnitude(), 1.0 / global::Kerf.Engine.Foundation.Precision.Infinite());

                    double sqSin1 = t1.CrossSquareMagnitude(n2) / sqT1;
                    double sqSin2 = t2.CrossSquareMagnitude(n2) / sqT2;

                    if (sqSin1 < sqSin2)
                    {
                        if (t1.Dot(n2) > 0.0)
                        {
                            _profile.Reverse();
                        }
                    }
                    else
                    {
                        if (t2.Dot(n2) > 0.0)
                        {
                            _profile.Reverse();
                        }
                    }
                }
                else // if (... == BRepExtrema_IsOnEdge)
                {
                    Edge e = ShapeCasts.Edge(extr.SupportOnShape1(idxMin));
                    EdgeCurveAdaptor ac = new(e);
                    extr.ParOnEdgeS1(idxMin, out double par);

                    ac.D1(par, out Point3D p, out Vector3D t1);

                    if (t1.Dot(n2) > 0.0)
                    {
                        _profile.Reverse();
                    }
                }
                break;

            case SupportType.IsOnEdge:
            case SupportType.IsVertex:
            {
                LibraryMakeFace mkFSpine = new(spine, true);
                if (!mkFSpine.IsDone)
                {
                    return;
                }

                Face fSpine = mkFSpine.InternalFace();
                // OCCT dereferences the down-cast plane without a check.
                PlaneSurface plnSpine = Tool.Surface(fSpine) as PlaneSurface ?? throw new NullReferenceException();
                Vector3D n1 = new(plnSpine.Axis().Direction());
                Vector3D tanV;

                if (extr.SupportTypeShape2(idxMin) == SupportType.IsVertex)
                {
                    Vertex v = ShapeCasts.Vertex(extr.SupportOnShape2(idxMin));
                    NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> mves =
                        new(ShapeHasher.Instance);
                    Exploration.MapShapesAndAncestors(profile, ShapeType.Vertex, ShapeType.Edge, mves);

                    NCollection_List<Shape> le = mves.FindFromKey(v);

                    Edge e1 = ShapeCasts.Edge(le.First());
                    Edge e2 = ShapeCasts.Edge(le.Last());

                    EdgeCurveAdaptor ac1 = new(e1), ac2 = new(e2);

                    double par1 = Tool.Parameter(v, e1);
                    double par2 = Tool.Parameter(v, e2);

                    ac1.D1(par1, out Point3D p, out Vector3D t1);
                    ac1.D1(par2, out p, out Vector3D t2); // OCCT: anAC1, not anAC2

                    // Find maximal cosine
                    double sqT1 = t1.SquareMagnitude();
                    double sqT2 = t2.SquareMagnitude();

                    if (sqT1 < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        sqT1 = RealLast();
                    }

                    if (sqT2 < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        sqT2 = RealLast();
                    }

                    double dp1 = t1.Dot(n1);
                    double dp2 = t2.Dot(n1);

                    if (dp1 * dp1 * sqT2 > dp2 * dp2 * sqT1)
                    {
                        // aDP1*aDP1/aSqT1 > aDP2*aDP2/aSqT2
                        tanV = t1;
                    }
                    else
                    {
                        tanV = t2;
                    }
                }
                else // if(anExtr.SupportTypeShape2(anIdxMin) == BRepExtrema_IsOnEdge)
                {
                    Edge e = ShapeCasts.Edge(extr.SupportOnShape2(idxMin));
                    EdgeCurveAdaptor ac = new(e);
                    extr.ParOnEdgeS2(idxMin, out double par);

                    ac.D1(par, out Point3D p, out tanV);
                }

                // The point in the profile, which is the nearest to the spine
                Point3D pnear = extr.PointOnShape2(idxMin);

                FaceClassifier fClass = new(fSpine, pnear, global::Kerf.Engine.Foundation.Precision.Confusion());
                if (fClass.State() != Out)
                {
                    if (n1.Dot(tanV) < 0.0)
                    {
                        _profile.Reverse();
                    }
                }
                else
                {
                    if (n1.Dot(tanV) > 0.0)
                    {
                        _profile.Reverse();
                    }
                }
            }
            break;
            default:
                break;
        }
    }

    //=================================================================================================

    private bool IsLid(in Face f, NCollection_IndexedMap<Shape> mapOfLids)
    {
        if (mapOfLids.IsEmpty())
        {
            return false;
        }

        if (Tool.Surface(f) is not PlaneSurface plnF)
        {
            return false;
        }

        for (int i = 1; i <= mapOfLids.Extent(); i++)
        {
            Face aF = ShapeCasts.Face(mapOfLids[i]);
            PlaneSurface? plane = Tool.Surface(aF) as PlaneSurface;

            if (ReferenceEquals(plane, plnF))
            {
                return true;
            }
        }

        return false;
    }

    //=================================================================================================

    /// <summary>
    /// Builds the evolved volume of the profile <paramref name="profile"/> along the planar spine
    /// <paramref name="spine"/>; <paramref name="solidReq"/> false gives a shell without the lids.
    /// </summary>
    public void Perform(in Wire spine, in Wire profile, double tolerance, bool solidReq = true)
    {
        _errorStatus = BRepFill_AdvancedEvolved_Empty;

        if (_fuzzyValue < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            _fuzzyValue = tolerance;
        }

        // OCCT.NET: frozen inputs are copied at the entry (decision 1047).
        CopyAtEntry spineEntry = new(spine);
        CopyAtEntry profileEntry = new(profile);

        GetSpineAndProfile(ShapeCasts.Wire(spineEntry.Shape()), ShapeCasts.Wire(profileEntry.Shape()));

        _pipeShell = new Shape();
        _topBottom = new Compound();
        _result = new Shape();

        try
        {
            PerformSweep();

            GetLids();

            if (_errorStatus != BRepFill_AdvancedEvolved_NotSolid)
            {
                return;
            }

            _result = _pipeShell;

            BuildSolid();

            if ((_errorStatus != BRepFill_AdvancedEvolved_OK) || solidReq)
            {
                return;
            }

            Shell shell = new();
            NCollection_IndexedMap<Shape> mfLids = new(ShapeHasher.Instance);
            Exploration.MapShapes(_topBottom, ShapeType.Face, mfLids);

            for (Explorer exp = new(_result, ShapeType.Face); exp.More(); exp.Next())
            {
                BrepBuilder bb = new();
                if (shell.IsNull)
                {
                    bb.MakeShell(ref shell);
                }

                Face f = ShapeCasts.Face(exp.Current());
                if (IsLid(f, mfLids))
                {
                    continue;
                }

                bb.Add(shell, f);
            }

            if (!shell.IsNull)
            {
                _result = shell;
            }
        }
        finally
        {
            // OCCT.NET: the caller's sub-shapes left unchanged stand in the result again.
            _result = profileEntry.Finish(spineEntry.Finish(_result));
        }
    }

    //=================================================================================================

    private void PerformSweep()
    {
        if (_errorStatus != BRepFill_AdvancedEvolved_Empty)
        {
            return;
        }

        _errorStatus = BRepFill_AdvancedEvolved_SweepError;

        PipeShell pipe = new(_spine);
        pipe.SetTolerance(APipeLinearTolerance, APipeLinearTolerance, APipeAngularTolerance);
        pipe.SetTransition(TransitionStyle.Round);
        pipe.Add(_profile, false, false);

        if (pipe.Build())
        {
            _errorStatus = BRepFill_AdvancedEvolved_NoLids;
            _pipeShell = pipe.InternalShape();
        }
    }

    //=================================================================================================

    private void GetLids()
    {
        if (_pipeShell.IsNull)
        {
            return;
        }

        if (Tool.IsClosed(_profile))
        {
            // No need in lids creation
            _errorStatus = BRepFill_AdvancedEvolved_NotSolid;
            return;
        }

        _errorStatus = BRepFill_AdvancedEvolved_NoLids;

        FindSurface fs = new(_spine, -1.0, true);
        if (fs.Surface() is not PlaneSurface surf)
        {
            _errorStatus = BRepFill_AdvancedEvolved_NotPlanarSpine;
            return;
        }

        double tol = Max(fs.Tolerance(), fs.ToleranceReached());
        tol += _fuzzyValue;
        Box profBox = new();
        Bounds.Add(_profile, ref profBox);
        double sqDiag = profBox.SquareExtent();
        // Square of the default angular tolerance in
        // BOPAlgo_Tools::EdgesToWires(...) and BOPAlgo_Tools::WiresToFaces(...) methods
        double sqAnguarTol = tol * tol / sqDiag;
        Direction3D normal = surf.Position().Direction();

        // Obtain free-edges from myPipeShell. All edges must be planar
        // and parallel to the plane of mySpine

        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> mapEF = new(ShapeHasher.Instance);

        Exploration.MapShapesAndAncestors(_pipeShell, ShapeType.Edge, ShapeType.Face, mapEF);

        NCollection_List<Shape> le = new();

        double dpMax = 0.0;
        for (int i = 1; i <= mapEF.Extent(); i++)
        {
            NCollection_List<Shape> listF = mapEF[i];

            if (listF.Extent() != 1)
            {
                continue;
            }

            Edge e = ShapeCasts.Edge(mapEF.FindKey(i));

            EdgeCurveAdaptor ac = new(e);
            if (!ac.Is3DCurve())
            {
                // We are not interested in degenerated edges.
                continue;
            }

            ac.D1(0.5 * (ac.FirstParameter() + ac.LastParameter()), out Point3D ptmp, out Vector3D tan);

            double sqModulus = tan.SquareMagnitude();
            if (sqModulus < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                continue;
            }

            double dp = tan.XYZ().Dot(normal.XYZ());
            if (Math.Abs(dp) > dpMax)
            {
                dpMax = Math.Abs(dp);
            }
            if (dp * dp > sqModulus * sqAnguarTol)
            {
                // Only planar edges are considered
                continue;
            }

            le.Append(e);
        }

        if (le.IsEmpty())
        {
            _errorStatus = BRepFill_AdvancedEvolved_NotPlanarSpine;
            return;
        }

        // Split interfered edges
        Shape freeEdges = new();
        if (!PerformBoolean(le, ref freeEdges))
        {
            _errorStatus = BRepFill_AdvancedEvolved_NotPlanarSpine;
            return;
        }

        // Collect all free edges to wires and create planar
        // top and bottom lids from these wires.
        BrepBuilder bb = new();
        Compound compW = new(), compF = new();
        bb.MakeCompound(ref compW);
        bb.MakeCompound(ref compF);
        bb.MakeCompound(ref _topBottom);
        double angTol = Math.Sqrt(sqAnguarTol);
        Shape compWS = compW, compFS = compF;
        BuilderTools.EdgesToWires(freeEdges, ref compWS, true, angTol);
        BuilderTools.WiresToFaces(compWS, ref compFS, angTol);

        {
            // Check orientation

            NCollection_IndexedMap<Shape> mapV = new(ShapeHasher.Instance);
            Exploration.MapShapes(_pipeShell, ShapeType.Vertex, mapV);
            Explorer exp = new(compFS, ShapeType.Face);
            for (; exp.More(); exp.Next())
            {
                Face f = ShapeCasts.Face(exp.Current());
                // OCCT dereferences the down-cast plane without a check.
                PlaneSurface pln = Tool.Surface(f) as PlaneSurface ?? throw new NullReferenceException();
                Xyz norm = pln.Position().Direction().XYZ();
                Xyz locP = pln.Position().Location().XYZ();

                bool isFound = false;

                for (int i = 1; i <= mapV.Extent(); i++)
                {
                    Vertex v = ShapeCasts.Vertex(mapV.FindKey(i));
                    Xyz p = Tool.Pnt(v).XYZ();

                    Xyz delta = p - locP;
                    double sqD = delta.SquareModulus();

                    if (sqD < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        continue;
                    }

                    double dp = delta.Dot(norm);

                    if (dp * dp < sqD * global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        // aP is in the plane
                        continue;
                    }

                    if (dp > 0.0)
                    {
                        bb.Add(_topBottom, f.Reversed());
                    }
                    else
                    {
                        bb.Add(_topBottom, f);
                    }

                    isFound = true;
                    break;
                }

                if (!isFound)
                {
                    bb.Add(_topBottom, f);
                }
            }
        }

        _errorStatus = BRepFill_AdvancedEvolved_NotSolid;
    }

    //=================================================================================================

    private void BuildSolid()
    {
        if (_errorStatus != BRepFill_AdvancedEvolved_NotSolid)
        {
            return;
        }

        _errorStatus = BRepFill_AdvancedEvolved_NotVolume;

        NCollection_Map<Shape> mapF = new(ShapeHasher.Instance);
        NCollection_List<Shape> lf = new(), lSplits = new();
        Explorer expF = new();

        for (expF.Init(_pipeShell, ShapeType.Face); expF.More(); expF.Next())
        {
            Face f = ShapeCasts.Face(expF.Current());
            if (!mapF.Add(f))
            {
                continue;
            }

            ReduceVertexTolerance(f);
            CheckSingularityAndAdd(f, _fuzzyValue, lf, lSplits);
        }

        {
            NCollection_List<Shape>.Iterator itrS = new(lSplits);
            for (; itrS.More(); itrS.Next())
            {
                Face f = ShapeCasts.Face(itrS.Value());
                lf.Append(f);
            }

            // Split interfered faces
            PerformBoolean(lf, ref _pipeShell);
        }

        lf.Clear();
        mapF.Clear();
        for (expF.Init(_pipeShell, ShapeType.Face); expF.More(); expF.Next())
        {
            Face f = ShapeCasts.Face(expF.Current());
            if (!mapF.Add(f))
            {
                continue;
            }

            lf.Append(f);
        }

        if (!_topBottom.IsNull)
        {
            Iterator itLids = new(_topBottom);
            for (; itLids.More(); itLids.Next())
            {
                Face f = ShapeCasts.Face(itLids.Value());
                lf.Append(f);
            }
        }

        MakerVolume mv = new();
        mv.SetArguments(lf);
        mv.SetFuzzyValue(_fuzzyValue);
        mv.SetIntersect(true);
        mv.SetRunParallel(_isParallel);
        mv.SetAvoidInternalShapes(true);
        mv.Perform();

        if (mv.HasErrors)
        {
            return;
        }

        _result = mv.InternalShape();

        RemoveExcessSolids(lSplits, _result, lf, mv);

        UnifyShape();
        RemoveInternalWires(_result);

        _errorStatus = BRepFill_AdvancedEvolved_OK;
    }

    //=================================================================================================

    private void UnifyShape()
    {
        UnifySameDomain unifier = new();

        unifier.Initialize(_result, true, true, false);
        unifier.SetSafeInputMode(true);
        unifier.AllowInternalEdges(false);
        unifier.SetLinearTolerance(APipeLinearTolerance);
        unifier.SetAngularTolerance(APipeAngularTolerance);
        unifier.Build();

        _result = unifier.InternalShape();
    }

    //=================================================================================================

    private void ExtractOuterSolid(ref Shape shape, NCollection_List<Shape> argsList)
    {
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> mapS = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(shape, ShapeType.Face, ShapeType.Solid, mapS);

        // theArgsList.Clear();
        NCollection_List<Shape> newList = new();
        int nbF = mapS.Extent();
        for (int i = 1; i <= nbF; ++i)
        {
            if (mapS[i].Extent() == 1)
            {
                newList.Append(mapS.FindKey(i));
            }
        }

        if (newList.IsEmpty())
        {
            return;
        }

        {
            NCollection_List<Shape>.Iterator itrF = new();

            bool isRemoved = true;
            while (isRemoved)
            {
                isRemoved = false;
                for (itrF.Initialize(argsList); itrF.More(); itrF.Next())
                {
                    Face f = ShapeCasts.Face(itrF.Value());
                    if (!ContainsInList(newList, f))
                    {
                        argsList.Remove(f);
                        isRemoved = true;
                        break;
                    }
                }
            }
        }

        MakerVolume mv = new();
        mv.SetArguments(newList);
        mv.SetIntersect(true);
        mv.SetRunParallel(_isParallel);
        mv.SetAvoidInternalShapes(true);
        mv.Perform();

        if (mv.HasErrors)
        {
            return;
        }

        shape = mv.InternalShape();
    }

    //=================================================================================================

    private void RemoveExcessSolids(NCollection_List<Shape> lSplits,
                                    in Shape shape,
                                    NCollection_List<Shape> argsList,
                                    MakerVolume mv)
    {
        if (_errorStatus != BRepFill_AdvancedEvolved_NotVolume)
        {
            return;
        }

        Shape resShape = shape;

        Explorer expSo = new();
        for (int i = 0; i < 2; i++)
        {
            expSo.Init(resShape, ShapeType.Solid);
            if (!expSo.More())
            {
                // No any solids
                _result = resShape;
                return;
            }

            expSo.Next();
            if (!expSo.More())
            {
                // Only one solid has been generated
                // OCCT: the explorer is past its only solid, so Current() is the explored shape itself (the
                // compound), which TopoDS::Solid casts unchecked in native's release build: the result is that
                // compound. The port keeps it without the cast (which would throw Standard_TypeMismatch).
                _result = expSo.Current();
                return;
            }

            if (i != 0)
            {
                break;
            }

            ExtractOuterSolid(ref resShape, argsList);
        }

        NCollection_List<Shape> solidList = new();

        // Look for all solids containing lids
        {
            expSo.Init(resShape, ShapeType.Solid);
            for (; expSo.More(); expSo.Next())
            {
                Solid sol = ShapeCasts.Solid(expSo.Current());
                NCollection_IndexedMap<Shape> mapF = new(ShapeHasher.Instance);
                Exploration.MapShapes(sol, mapF);

                bool areThereLids = false;
                Explorer expLids = new(_topBottom, ShapeType.Face);
                for (; expLids.More(); expLids.Next())
                {
                    areThereLids = true;
                    Face fLid = ShapeCasts.Face(expLids.Current());
                    int fIdx = mapF.FindIndex(fLid);
                    if (fIdx < 1)
                    {
                        continue;
                    }

                    Face fSol = ShapeCasts.Face(mapF.FindKey(fIdx));

                    if (fSol.IsEqual(fLid))
                    {
                        solidList.Append(sol);
                    }

                    break;
                }

                if (!areThereLids)
                {
                    break;
                }
            }

            if (solidList.Extent() < 1)
            {
                _result = resShape;
                return;
            }

            if (solidList.Extent() == 1)
            {
                _result = solidList.First();
                return;
            }

            if (solidList.Extent() > 0)
            {
                BrepBuilder bb = new();
                CompSolid compSol = new();
                bb.MakeCompSolid(ref compSol);
                NCollection_List<Shape>.Iterator itl = new(solidList);
                for (; itl.More(); itl.Next())
                {
                    Solid sol = ShapeCasts.Solid(itl.Value());
                    bb.Add(compSol, sol);
                }

                resShape = compSol;
                solidList.Clear();
            }
        }

        {
            // Remove Split faces from the list of arguments
            NCollection_List<Shape>.Iterator itl = new(lSplits);
            for (; itl.More(); itl.Next())
            {
                Face f = ShapeCasts.Face(itl.Value());
                argsList.Remove(f);
            }

            // Create a list of invalid faces. The face is invalid if
            // BOPAlgo_MakerVolume changes its orientation while creating solids.
            // Faces from theLSplits are not checked.
            NCollection_List<Shape> listInvFaces = new();
            for (itl.Initialize(argsList); itl.More(); itl.Next())
            {
                Face f = ShapeCasts.Face(itl.Value());
                foreach (Shape mod in mv.Modified(f))
                {
                    Face fm = ShapeCasts.Face(mod);

                    if (fm.Orientation() != f.Orientation())
                    {
                        listInvFaces.Append(fm);
                    }
                }
            }

            for (expSo.Init(resShape, ShapeType.Solid); expSo.More(); expSo.Next())
            {
                Solid so = ShapeCasts.Solid(expSo.Current());
                NCollection_IndexedMap<Shape> mapF = new(ShapeHasher.Instance);
                Exploration.MapShapes(so, ShapeType.Face, mapF);
                bool isToDelete = false;

                for (itl.Initialize(listInvFaces); itl.More(); itl.Next())
                {
                    Face f = ShapeCasts.Face(itl.Value());
                    if (mapF.Contains(f))
                    {
                        isToDelete = true;
                        break;
                    }
                }

                if (isToDelete)
                {
                    continue;
                }

                for (itl.Initialize(argsList); itl.More(); itl.Next())
                {
                    Face f = ShapeCasts.Face(itl.Value());
                    int idx = mapF.FindIndex(f);
                    if (idx == 0)
                    {
                        continue;
                    }

                    Face f1 = ShapeCasts.Face(mapF.FindKey(idx));

                    // aF and aF1 are same shapes. Check if they are equal.

                    if (!f.IsEqual(f1))
                    {
                        isToDelete = true;
                        break;
                    }
                }

                if (isToDelete)
                {
                    continue;
                }

                solidList.Append(so);
            }
        }

        if (solidList.Extent() < 1)
        {
            _result = resShape;
            return;
        }

        if (solidList.Extent() == 1)
        {
            _result = solidList.First();
            return;
        }

        {
            BrepBuilder bb = new();
            CompSolid cmpSol = new();
            bb.MakeCompSolid(ref cmpSol);

            for (NCollection_List<Shape>.Iterator itl = new(solidList); itl.More(); itl.Next())
            {
                Solid so = ShapeCasts.Solid(itl.Value());
                bb.Add(cmpSol, so);
            }

            _result = cmpSol;
        }
    }

    //=======================================================================
    // function : RebuildFaces
    // purpose  : Creates a wires from theEdges and puts it to the new face
    //            which is empty-copied from theSourceFace.
    //=======================================================================
    private static void RebuildFaces(NCollection_List<Shape> le,
                                     in Face sourceFace,
                                     NCollection_List<Shape> list)
    {
        // build new faces
        BuilderFace bf = new();

        Face f = ShapeCasts.Face(sourceFace.Oriented(Forward));

        bf.SetFace(f);
        bf.SetShapes(le);

        bf.Perform();

        NCollection_List<Shape> lfr = bf.Areas();

        if (lfr.IsEmpty())
        {
            list.Append(sourceFace);
            return;
        }

        NCollection_List<Shape>.Iterator itFR = new(lfr);
        for (; itFR.More(); itFR.Next())
        {
            Shape fr = ShapeCasts.Face(itFR.Value());
            list.Append(fr);
        }
    }

    //=======================================================================
    // function : MakeEdgeDegenerated
    // purpose  : Returns TRUE if degenerated edge has been created.
    //           Every degenerated edge (to split) must be added in theLEdges twice
    //           with different orientations. Moreover, Degenerated edges cannot be shared.
    //           Therefore, make copy of them before adding.
    //=======================================================================
    private static bool MakeEdgeDegenerated(in Vertex v,
                                            in Face face,
                                            in Point2D pf,
                                            in Point2D pl,
                                            NCollection_List<Shape> lEdges)
    {
        FaceSurfaceAdaptor anAS = new(face, false);

        double tol = 2.0 * Tool.Tolerance(v);
        double tolU = anAS.UResolution(tol), tolV = anAS.VResolution(tol);

        if ((Math.Abs(pf.X() - pl.X()) < tolU) && (Math.Abs(pf.Y() - pl.Y()) < tolV))
        {
            return false;
        }

        Vertex vf = ShapeCasts.Vertex(v.Oriented(Forward)),
                      vl = ShapeCasts.Vertex(v.Oriented(Reversed));

        Xy aV = pl.XY() - pf.XY();
        LineCurve2D l1 = new(pf, new Direction2D(aV));
        LineCurve2D l2 = new(pl, new Direction2D(aV.Reversed()));

        BrepBuilder bb = new();
        Edge edegen1 = new(), edegen2 = new();
        bb.MakeEdge(ref edegen1);
        bb.MakeEdge(ref edegen2);

        bb.UpdateEdge(edegen1, l1, face, global::Kerf.Engine.Foundation.Precision.Confusion());
        bb.UpdateEdge(edegen2, l2, face, global::Kerf.Engine.Foundation.Precision.Confusion());

        edegen1.Orientation(Forward);
        edegen2.Orientation(Forward);

        bb.Add(edegen1, vf);
        bb.Add(edegen1, vl);
        bb.Add(edegen2, vf);
        bb.Add(edegen2, vl);

        bb.Degenerated(edegen1, true);
        bb.Degenerated(edegen2, true);

        double lPar = aV.Modulus();
        bb.Range(edegen1, 0.0, lPar);
        bb.Range(edegen2, 0.0, lPar);

        lEdges.Append(edegen1);
        lEdges.Append(edegen2);

        return true;
    }

    //=================================================================================================

    private static void InsertEDegenerated(in Face face, NCollection_List<Shape> lEdges)
    {
        BrepBuilder bb = new();
        Wire wir = new();
        bb.MakeWire(ref wir);

        NCollection_List<Shape>.Iterator itr = new(lEdges);
        for (; itr.More(); itr.Next())
        {
            Edge e = ShapeCasts.Edge(itr.Value());
            bb.Add(wir, e);
        }

        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> mapVE = new(ShapeHasher.Instance);
        Exploration.MapShapesAndUniqueAncestors(wir, ShapeType.Vertex, ShapeType.Edge, mapVE);

        WireExplorer exp = new(wir, face);

        Edge e1 = exp.Current(), firstEdge, lastEdge = new();

        if (e1.IsNull)
        {
            // It is possible if aWir contains
            // only INTERNAL/EXTERNAL edges.

            return;
        }

        firstEdge = e1;
        exp.Next();

        // Map containing all vertices of degenerated edges
        NCollection_Map<Shape> mapVofDE = new(ShapeHasher.Instance);

        {
            Explorer expDE = new(wir, ShapeType.Edge);
            for (; expDE.More(); expDE.Next())
            {
                Edge e = ShapeCasts.Edge(expDE.Current());
                if (!Tool.Degenerated(e))
                {
                    continue;
                }

                Exploration.Vertices(e, out Vertex v1, out Vertex v2);

                // aV1 and aV2 are SAME vertices

                mapVofDE.Add(v1);
            }
        }

        for (; exp.More(); exp.Next())
        {
            Edge e2 = exp.Current();
            lastEdge = e2;

            Vertex vertCurr = exp.CurrentVertex();

            if (mapVofDE.Contains(vertCurr))
            {
                // Necessary degenerated edge has already been created.
                e1 = e2;
                continue;
            }

            Curve2D c1 = Tool.CurveOnSurface(e1, face, out double f, out double l)!,
                         c2 = Tool.CurveOnSurface(e2, face, out f, out l)!;
            f = Tool.Parameter(vertCurr, e1);
            l = Tool.Parameter(vertCurr, e2);
            Point2D pf = c1.Value(f), pl = c2.Value(l);

            if (MakeEdgeDegenerated(vertCurr, face, pf, pl, lEdges))
            {
                mapVofDE.Add(vertCurr);
                e1 = e2;
                continue;
            }

            bool found = mapVE.FindFromKey(vertCurr, out NCollection_List<Shape> eList);
            if (found && (eList.Extent() <= 2))
            {
                e1 = e2;
                continue;
            }

            // Case like cone with apex. In 2D space all is OK
            // (therefore BRepTools_WireExplorer processes this case
            // correctly). But in 3D-space, we have several edges with
            // the same vertex. Cone apex must be plugged by degenerated edge.

            bool hasDegenerated = false;
            // OCCT dereferences the seek's result without a check.
            itr.Initialize(eList ?? throw new NullReferenceException());
            for (; itr.More(); itr.Next())
            {
                Edge edge = ShapeCasts.Edge(itr.Value());
                if (Tool.Degenerated(edge))
                {
                    hasDegenerated = true;
                    break;
                }
            }

            if (hasDegenerated)
            {
                e1 = e2;
                continue;
            }

            // Look for the pair for anE1 and anE2 edges
            for (int i = 0; i < 2; i++)
            {
                Point2D point = i != 0 ? pl : pf;
                itr.Initialize(eList);
                for (; itr.More(); itr.Next())
                {
                    Edge edge = ShapeCasts.Edge(itr.Value());

                    if (edge.IsSame(e1) || edge.IsSame(e2))
                    {
                        continue;
                    }

                    Curve2D c = Tool.CurveOnSurface(edge, face, out f, out l)!;
                    f = Tool.Parameter(vertCurr, edge);
                    Point2D p = c.Value(f);

                    if (MakeEdgeDegenerated(vertCurr, face, point, p, lEdges))
                    {
                        mapVofDE.Add(vertCurr);
                        i = 2;
                        break;
                    }
                }
            }

            e1 = e2;
        }

        if (firstEdge.IsNull || lastEdge.IsNull)
        {
            return;
        }

        // TopExp::CommonVertex(...) does not work
        // if edges have more than one pair of common vertex
        //(e.g. two halves of circle). Here, we process this case.
        Vertex[] v = new Vertex[4];
        Exploration.Vertices(firstEdge, out v[0], out v[1]);
        if (!v[0].IsNull && v[0].IsSame(v[1]))
        {
            // Possible reason is the NOT-CLOSED edge
            // has only single vertex and is covered by it.
            return;
        }

        Exploration.Vertices(lastEdge, out v[2], out v[3]);
        if (!v[2].IsNull && v[2].IsSame(v[3]))
        {
            // Possible reason is the NOT-CLOSED edge
            // has only single vertex and is covered by it.
            return;
        }

        for (int idfe = 0; idfe < 2; idfe++)
        {
            for (int idle = 2; idle < 4; idle++)
            {
                if (!v[idfe].IsSame(v[idle]))
                {
                    continue;
                }

                if (mapVE.FindFromKey(v[idfe], out NCollection_List<Shape> eList) && (eList.Extent() > 2))
                {
                    // Causes:
                    //  1. Non-manifold topology.
                    //  2. Case such as: two edges meeting a seam at the same vertex, which must not be joined.
                    continue;
                }

                Curve2D c1 = Tool.CurveOnSurface(firstEdge, face, out double f, out double l)!,
                             c2 = Tool.CurveOnSurface(lastEdge, face, out f, out l)!;
                f = Tool.Parameter(v[idfe], firstEdge);
                l = Tool.Parameter(v[idle], lastEdge);
                Point2D pf = c1.Value(f), pl = c2.Value(l);

                MakeEdgeDegenerated(v[idfe], face, pf, pl, lEdges);
            }
        }
    }

    //=======================================================================
    // function : CheckSingularityAndAdd
    // purpose  : Returns TRUE if theF has been split
    //=======================================================================
    private bool CheckSingularityAndAdd(in Face f,
                                        double fuzzyToler,
                                        NCollection_List<Shape> listOfFaces,
                                        NCollection_List<Shape> listOfSplits)
    {
        FaceSurfaceAdaptor anAS = new(f, false);
        SurfaceKind sType = anAS.GetType();

        if (sType == SurfaceKind.OffsetSurface)
        {
            sType = anAS.BasisSurface()!.GetType();
        }

        if (sType == Plane)
        {
            NCollection_Map<Shape> me = new(ShapeHasher.Instance);
            NCollection_List<Shape> le = new();
            Explorer exp = new(f, ShapeType.Edge);
            for (; exp.More(); exp.Next())
            {
                Edge e = ShapeCasts.Edge(exp.Current());

                if (me.Add(e))
                {
                    le.Append(e);
                }
            }

            // Split interfered edges
            PaveFiller pf = new();
            pf.SetArguments(le);
            pf.SetRunParallel(_isParallel);

            pf.Perform();
            if (pf.HasErrors)
            {
                listOfFaces.Append(f);
                return false;
            }

            DS ds = pf.DS();
            if (ds.ShapeCount == ds.SourceShapeCount)
            {
                // Interfered edges have not been detected
                listOfFaces.Append(f);
                return false;
            }

            BooleanBuilder builder = new();
            builder.SetArguments(le);
            builder.SetRunParallel(_isParallel);
            builder.PerformWithFiller(pf);
            if (builder.HasErrors)
            {
                listOfFaces.Append(f);
                return false;
            }

            Shape edges = builder.InternalShape();

            BrepBuilder bb = new();
            Compound compW = new(), compF = new();
            bb.MakeCompound(ref compW);
            bb.MakeCompound(ref compF);
            Shape compWS = compW, compFS = compF;
            BuilderTools.EdgesToWires(edges, ref compWS, true);
            BuilderTools.WiresToFaces(compWS, ref compFS);

            me.Clear();
            exp.Init(compFS, ShapeType.Face);
            for (; exp.More(); exp.Next())
            {
                Face aF = ShapeCasts.Face(exp.Current());
                listOfSplits.Append(aF);
            }

            return true;
        }

        if ((sType != SurfaceKind.Cone) && (sType != SurfaceKind.Sphere) && (sType != SurfaceKind.BezierSurface)
            && (sType != SurfaceKind.BSplineSurface) && (sType != SurfaceOfRevolution))
        {
            listOfFaces.Append(f);
            return false;
        }

        BrepBuilder bb2 = new();

        Compound cWires = new();
        bb2.MakeCompound(ref cWires);

        bool isSplit = false;
        NCollection_List<Shape> listEdges = new();

        Face face = ShapeCasts.Face(f.Oriented(Forward));

        for (Iterator expW = new(face); expW.More(); expW.Next())
        {
            Wire wir = ShapeCasts.Wire(expW.Value());

            NCollection_List<Shape> lgf = new();
            Explorer eExp = new(wir, ShapeType.Edge);
            for (; eExp.More(); eExp.Next())
            {
                Edge e = ShapeCasts.Edge(eExp.Current());
                lgf.Append(e);
            }

            PaveFiller pf = new();
            pf.SetArguments(lgf);
            pf.SetFuzzyValue(fuzzyToler);
            pf.Perform();

            if (pf.HasErrors)
            {
                continue;
            }

            DS ds = pf.DS();
            if (ds.ShapeCount == ds.SourceShapeCount)
            {
                // No new shapes have been created
                continue;
            }

            BooleanBuilder builder = new();
            builder.SetArguments(lgf);
            builder.SetRunParallel(_isParallel);
            builder.SetNonDestructive(true);
            builder.PerformWithFiller(pf);
            if (builder.HasErrors)
            {
                continue;
            }

            NCollection_List<Shape> le = new();
            NCollection_List<Shape>.Iterator bItr = new(lgf);
            for (; bItr.More(); bItr.Next())
            {
                Edge sh = ShapeCasts.Edge(bItr.Value());
                var lm = builder.Modified(sh);
                if (lm.Count == 0 || Tool.Degenerated(sh))
                {
                    le.Append(sh);
                    continue;
                }

                foreach (Shape mod in lm)
                {
                    Edge em = ShapeCasts.Edge(mod);
                    le.Append(em);
                }
            }

            isSplit = true;
            InsertEDegenerated(face, le);
            listEdges.Append(le);
        }

        if (!isSplit)
        {
            listOfFaces.Append(f);
            return false;
        }

        RebuildFaces(listEdges, f, listOfSplits);

        NCollection_List<Shape>.Iterator itrS = new(listOfSplits);
        for (; itrS.More(); itrS.Next())
        {
            Face aF = ShapeCasts.Face(itrS.Value());
            listOfFaces.Append(aF.Oriented(f.Orientation()));
        }

        return true;
    }

    //=================================================================================================

    private static bool ContainsInList(NCollection_List<Shape> l, in Shape theObject)
    {
        NCollection_List<Shape>.Iterator it = new(l);
        for (; it.More(); it.Next())
        {
            if (it.Value().IsSame(theObject))
            {
                return true;
            }
        }
        return false;
    }

    //=======================================================================
    // function: FindInternals
    // purpose: Looks for internal shapes inside the face or solid
    //=======================================================================
    private static void FindInternals(in Shape s, NCollection_List<Shape> lInt)
    {
        Iterator itS = new(s);
        for (; itS.More(); itS.Next())
        {
            Shape ss = itS.Value();
            if (ss.Orientation() == Internal)
            {
                lInt.Append(ss);
            }
            else
            {
                Iterator itSS = new(ss);
                for (; itSS.More(); itSS.Next())
                {
                    if (itSS.Value().Orientation() == Internal)
                    {
                        lInt.Append(ss);
                        break;
                    }
                }
            }
        }
    }

    //=======================================================================
    // function: RemoveInternalWires
    // purpose: Removes internal wires from the faces
    //=======================================================================
    private static void RemoveInternalWires(in Shape shape)
    {
        Explorer expF = new(shape, ShapeType.Face);
        for (; expF.More(); expF.Next())
        {
            Shape f = expF.Current();
            NCollection_List<Shape> lwToRemove = new();
            FindInternals(f, lwToRemove);
            if (lwToRemove.Extent() != 0)
            {
                f.Free(true);
                NCollection_List<Shape>.Iterator itR = new(lwToRemove);
                for (; itR.More(); itR.Next())
                {
                    new BrepBuilder().Remove(f, itR.Value());
                }
                f.Free(false);
            }
        }
    }

    //=================================================================================================

    private static void ProcessVertex(in Vertex aV, NCollection_List<Shape> aLE, NCollection_List<Shape> aLF)
    {
        double tol, d2, tolMax2, tolE, param;
        Point3D pc3D;
        Point2D pc2D;
        Orientation orV;

        NCollection_List<Shape>.Iterator it = new();
        Explorer vExp = new();

        //
        tolMax2 = -1.0e6;
        //
        BrepTVertex tv = (BrepTVertex)aV.TShape()!;
        Point3D pv3D = tv.Pnt();
        tol = Tool.Tolerance(aV);
        //
        it.Initialize(aLE);
        for (; it.More(); it.Next())
        {
            Edge e = ShapeCasts.Edge(it.Value());
            //
            BrepTEdge te = (BrepTEdge)e.TShape()!;
            Placement eloc = e.Location();
            //
            vExp.Init(e, ShapeType.Vertex);
            for (; vExp.More(); vExp.Next())
            {
                Vertex vx = ShapeCasts.Vertex(vExp.Current());
                //
                if (!vx.IsSame(aV))
                {
                    continue;
                }
                //
                orV = vx.Orientation();
                if (orV != Forward && orV != Reversed)
                {
                    continue;
                }
                //
                NCollection_List<CurveRepresentation> lcr = te.Curves();
                NCollection_List<CurveRepresentation>.Iterator itcr = new(lcr);
                for (; itcr.More(); itcr.Next())
                {
                    CurveRepresentation cr = itcr.Value();
                    Placement loc = cr.Location();
                    Placement l = (eloc * loc).Predivided(aV.Location());
                    //
                    // 3D-Curve
                    if (cr.IsCurve3D)
                    {
                        Curve3D? c3D = cr.Curve3D();
                        //
                        if (c3D is null)
                        {
                            continue;
                        }
                        // 3D-point treatment
                        param = Tool.Parameter(vx, e);
                        pc3D = c3D.Value(param);
                        pc3D = pc3D.Transformed(l.Transformation());
                        d2 = pv3D.SquareDistance(pc3D);
                        if (d2 > tolMax2)
                        {
                            tolMax2 = d2;
                        }
                        //
                    } // if (cr->IsCurve3D())
                    //
                    // 2D-Curve
                    else if (cr.IsCurveOnSurface())
                    {
                        Curve2D? c2D = cr.PCurve();
                        if (c2D is null)
                        {
                            continue;
                        }
                        // Surface
                        Surface s = cr.Surface()!;
                        //
                        // 2D-point treatment
                        param = Tool.Parameter(vx, e, s, l);
                        pc2D = c2D.Value(param);
                        s.D0(pc2D.X(), pc2D.Y(), out pc3D);
                        pc3D = pc3D.Transformed(l.Transformation());
                        d2 = pv3D.SquareDistance(pc3D);
                        if (d2 > tolMax2)
                        {
                            tolMax2 = d2;
                        }
                    } // if (cr->IsCurveOnSurface())

                } // for (; itcr.More(); itcr.Next())
            } // for (; aVExp.More(); aVExp.Next())
        } // for (; anIt.More(); anIt.Next())
        // #########################################################
        //
        //  Reducing
        if (tolMax2 < 0.0)
        {
            return;
        }
        //
        tolMax2 = Math.Sqrt(tolMax2);
        if (tolMax2 > tol)
        {
            return;
        }
        //
        it.Initialize(aLE);
        for (; it.More(); it.Next())
        {
            Edge e = ShapeCasts.Edge(it.Value());

            tolE = Tool.Tolerance(e);
            if (tolMax2 < tolE)
            {
                tolMax2 = tolE;
            }
        }
        //
        it.Initialize(aLF);
        for (; it.More(); it.Next())
        {
            Face f = ShapeCasts.Face(it.Value());

            tolE = Tool.Tolerance(f);
            if (tolMax2 < tolE)
            {
                tolMax2 = tolE;
            }
        }
        //
        if (tolMax2 > tol)
        {
            return;
        }
        //
        // Update Tolerance
        // with a small margin
        tv.Tolerance(tolMax2 + tolMax2 * 0.0001);
    }

    //=================================================================================================

    private static void ReduceVertexTolerance(in Shape aS)
    {
        int i, nbV;
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> veMap = new(ShapeHasher.Instance),
            vfMap = new(ShapeHasher.Instance);

        Exploration.MapShapesAndUniqueAncestors(aS, ShapeType.Vertex, ShapeType.Edge, veMap);
        Exploration.MapShapesAndUniqueAncestors(aS, ShapeType.Vertex, ShapeType.Face, vfMap);

        nbV = veMap.Extent();
        for (i = 1; i <= nbV; i++)
        {
            Vertex v = ShapeCasts.Vertex(veMap.FindKey(i));
            NCollection_List<Shape> le = veMap[i];
            NCollection_List<Shape> lf = vfMap.FindFromKey(v);

            ProcessVertex(v, le, lf);
        }
    }
}
