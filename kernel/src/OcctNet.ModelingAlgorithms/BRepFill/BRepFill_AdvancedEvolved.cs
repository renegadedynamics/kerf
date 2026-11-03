// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_AdvancedEvolved.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_AdvancedEvolved.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2018-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.BRepExtrema_SupportType;
using static Occt.BRepFill_TransitionStyle;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Occt;

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
public sealed class BRepFill_AdvancedEvolved
{
    private const uint BRepFill_AdvancedEvolved_Empty = 0;
    private const uint BRepFill_AdvancedEvolved_NotPlanarSpine = 1;
    private const uint BRepFill_AdvancedEvolved_SweepError = 2;
    private const uint BRepFill_AdvancedEvolved_NoLids = 3;
    private const uint BRepFill_AdvancedEvolved_NotSolid = 4;
    private const uint BRepFill_AdvancedEvolved_NotVolume = 5;
    private const uint BRepFill_AdvancedEvolved_OK = uint.MaxValue;

    private const double aPipeLinearTolerance = 1.0e-4;
    private const double aPipeAngularTolerance = 1.0e-2;

    private uint myErrorStatus;
    private Wire mySpine = new();
    private Wire myProfile = new();
    private Shape myPipeShell = new();
    private Compound myTopBottom = new(); // Lids can be split on several faces
    private Shape myResult = new();
    private double myFuzzyValue;
    private bool myIsParallel;

    /// <summary>An empty algorithm.</summary>
    public BRepFill_AdvancedEvolved()
    {
        myErrorStatus = BRepFill_AdvancedEvolved_Empty;
        myFuzzyValue = 0.0;
        myIsParallel = true;
    }

    /// <summary>True if the volume was built.</summary>
    public bool IsDone() => myErrorStatus == BRepFill_AdvancedEvolved_OK;

    /// <summary>True if the volume was built; <paramref name="theErrorCode"/> is the status (uint.MaxValue when done).</summary>
    public bool IsDone(out uint theErrorCode)
    {
        theErrorCode = myErrorStatus;
        return myErrorStatus == BRepFill_AdvancedEvolved_OK;
    }

    /// <summary>The resulting shape (frozen).</summary>
    public Shape Shape()
    {
        myResult.TShape()?.FreezeGraph();
        return myResult;
    }

    /// <summary><see cref="Shape"/> without freezing, for ported code.</summary>
    internal Shape InternalShape() => myResult;

    /// <summary>OCCT's directory for the debug shapes; the port writes none, so it is ignored.</summary>
    public void SetTemporaryDirectory(string thePath)
    {
    }

    /// <summary>Sets or unsets the parallel mode of the Booleans.</summary>
    public void SetParallelMode(bool theVal) => myIsParallel = theVal;

    //=================================================================================================

    private bool PerformBoolean(NCollection_List<Shape> theArgsList, ref Shape theResult)
    {
        BOPAlgo_PaveFiller aPF = new();

        aPF.SetArguments(theArgsList);
        aPF.SetRunParallel(myIsParallel);
        aPF.SetFuzzyValue(myFuzzyValue);

        aPF.Perform();
        if (aPF.HasErrors())
        {
            return false;
        }

        BOPAlgo_Builder aBuilder = new();
        aBuilder.SetArguments(theArgsList);

        aBuilder.SetRunParallel(myIsParallel);
        aBuilder.PerformWithFiller(aPF);
        if (aBuilder.HasErrors())
        {
            return false;
        }

        theResult = aBuilder.InternalShape();
        return true;
    }

    //=================================================================================================

    private void GetSpineAndProfile(in Wire theSpine, in Wire theProfile)
    {
        mySpine = theSpine;
        myProfile = theProfile;

        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aMVEP = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(theProfile, ShapeType.Vertex, ShapeType.Edge, aMVEP);

        Vector3D aN2 = new();
        Point3D aLoc = new();

        for (int i = 1; i <= aMVEP.Extent(); i++)
        {
            Vertex aVC = ShapeCasts.Vertex(aMVEP.FindKey(i));

            NCollection_List<Shape> aLE = aMVEP.FindFromIndex(i);

            if (aLE.Extent() < 2)
            {
                continue;
            }

            Edge anE1 = ShapeCasts.Edge(aLE.First());
            Edge anE2 = ShapeCasts.Edge(aLE.Last());

            EdgeCurveAdaptor anAC1 = new(anE1), anAC2 = new(anE2);

            double aPar1 = Tool.Parameter(aVC, anE1);
            double aPar2 = Tool.Parameter(aVC, anE2);

            anAC1.D1(aPar1, out Point3D aP, out Vector3D aT1);
            anAC1.D1(aPar2, out aP, out Vector3D aT2); // OCCT: anAC1, not anAC2

            aN2 = aT1.Crossed(aT2);

            if (aN2.SquareMagnitude() > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
            {
                aLoc = Tool.Pnt(aVC);
                break;
            }
        }

        BRepExtrema_DistShapeShape anExtr = new();
        anExtr.LoadS1(theSpine);

        if (aN2.SquareMagnitude() > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            Plane3D aPln = new(aLoc, new Direction3D(aN2));
            BRepLib_MakeFace aMF = new(aPln, theProfile);
            if (!aMF.IsDone())
            {
                return;
            }

            anExtr.LoadS2(aMF.InternalFace());
        }
        else
        {
            anExtr.LoadS2(theProfile);
        }

        if (!anExtr.Perform())
        {
            return;
        }

        int aNbSol = anExtr.NbSolution();
        if (aNbSol < 1)
        {
            return;
        }

        double aDistMin = RealLast();
        int anIdxMin = 0;

        for (int aSolId = 1; aSolId <= aNbSol; aSolId++)
        {
            double aD = anExtr.Value(); // OCCT: the same value for every solution
            if (aD > aDistMin)
            {
                continue;
            }

            aDistMin = aD;
            anIdxMin = aSolId;
        }

        BRepExtrema_SupportType anExtrType2 = anExtr.SupportTypeShape2(anIdxMin);

        if (aDistMin < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            anExtrType2 = BRepExtrema_IsInFace;
        }

        switch (anExtrType2)
        {
            case BRepExtrema_IsInFace:
                if (anExtr.SupportTypeShape1(anIdxMin) == BRepExtrema_IsVertex)
                {
                    Vertex aV = ShapeCasts.Vertex(anExtr.SupportOnShape1(anIdxMin));
                    NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aMVES =
                        new(ShapeHasher.Instance);
                    Exploration.MapShapesAndAncestors(theSpine, ShapeType.Vertex, ShapeType.Edge, aMVES);

                    NCollection_List<Shape> aLE = aMVES.FindFromKey(aV);

                    Edge anE1 = ShapeCasts.Edge(aLE.First());
                    Edge anE2 = ShapeCasts.Edge(aLE.Last());

                    EdgeCurveAdaptor anAC1 = new(anE1), anAC2 = new(anE2);

                    double aPar1 = Tool.Parameter(aV, anE1);
                    double aPar2 = Tool.Parameter(aV, anE2);

                    anAC1.D1(aPar1, out Point3D aP, out Vector3D aT1);
                    anAC1.D1(aPar2, out aP, out Vector3D aT2); // OCCT: anAC1, not anAC2

                    // Find minimal sine
                    double aSqT1 = Max(aT1.SquareMagnitude(), 1.0 / global::Kerf.Engine.Foundation.Precision.Infinite());
                    double aSqT2 = Max(aT2.SquareMagnitude(), 1.0 / global::Kerf.Engine.Foundation.Precision.Infinite());

                    double aSqSin1 = aT1.CrossSquareMagnitude(aN2) / aSqT1;
                    double aSqSin2 = aT2.CrossSquareMagnitude(aN2) / aSqT2;

                    if (aSqSin1 < aSqSin2)
                    {
                        if (aT1.Dot(aN2) > 0.0)
                        {
                            myProfile.Reverse();
                        }
                    }
                    else
                    {
                        if (aT2.Dot(aN2) > 0.0)
                        {
                            myProfile.Reverse();
                        }
                    }
                }
                else // if (... == BRepExtrema_IsOnEdge)
                {
                    Edge anE = ShapeCasts.Edge(anExtr.SupportOnShape1(anIdxMin));
                    EdgeCurveAdaptor anAC = new(anE);
                    anExtr.ParOnEdgeS1(anIdxMin, out double aPar);

                    anAC.D1(aPar, out Point3D aP, out Vector3D aT1);

                    if (aT1.Dot(aN2) > 0.0)
                    {
                        myProfile.Reverse();
                    }
                }
                break;

            case BRepExtrema_IsOnEdge:
            case BRepExtrema_IsVertex:
            {
                BRepLib_MakeFace aMkFSpine = new(theSpine, true);
                if (!aMkFSpine.IsDone())
                {
                    return;
                }

                Face aFSpine = aMkFSpine.InternalFace();
                // OCCT dereferences the down-cast plane without a check.
                PlaneSurface aPlnSpine = Tool.Surface(aFSpine) as PlaneSurface ?? throw new NullReferenceException();
                Vector3D aN1 = new(aPlnSpine.Axis().Direction());
                Vector3D aTanV;

                if (anExtr.SupportTypeShape2(anIdxMin) == BRepExtrema_IsVertex)
                {
                    Vertex aV = ShapeCasts.Vertex(anExtr.SupportOnShape2(anIdxMin));
                    NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aMVES =
                        new(ShapeHasher.Instance);
                    Exploration.MapShapesAndAncestors(theProfile, ShapeType.Vertex, ShapeType.Edge, aMVES);

                    NCollection_List<Shape> aLE = aMVES.FindFromKey(aV);

                    Edge anE1 = ShapeCasts.Edge(aLE.First());
                    Edge anE2 = ShapeCasts.Edge(aLE.Last());

                    EdgeCurveAdaptor anAC1 = new(anE1), anAC2 = new(anE2);

                    double aPar1 = Tool.Parameter(aV, anE1);
                    double aPar2 = Tool.Parameter(aV, anE2);

                    anAC1.D1(aPar1, out Point3D aP, out Vector3D aT1);
                    anAC1.D1(aPar2, out aP, out Vector3D aT2); // OCCT: anAC1, not anAC2

                    // Find maximal cosine
                    double aSqT1 = aT1.SquareMagnitude();
                    double aSqT2 = aT2.SquareMagnitude();

                    if (aSqT1 < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        aSqT1 = RealLast();
                    }

                    if (aSqT2 < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        aSqT2 = RealLast();
                    }

                    double aDP1 = aT1.Dot(aN1);
                    double aDP2 = aT2.Dot(aN1);

                    if (aDP1 * aDP1 * aSqT2 > aDP2 * aDP2 * aSqT1)
                    {
                        // aDP1*aDP1/aSqT1 > aDP2*aDP2/aSqT2
                        aTanV = aT1;
                    }
                    else
                    {
                        aTanV = aT2;
                    }
                }
                else // if(anExtr.SupportTypeShape2(anIdxMin) == BRepExtrema_IsOnEdge)
                {
                    Edge anE = ShapeCasts.Edge(anExtr.SupportOnShape2(anIdxMin));
                    EdgeCurveAdaptor anAC = new(anE);
                    anExtr.ParOnEdgeS2(anIdxMin, out double aPar);

                    anAC.D1(aPar, out Point3D aP, out aTanV);
                }

                // The point in the profile, which is the nearest to the spine
                Point3D aPnear = anExtr.PointOnShape2(anIdxMin);

                BRepClass_FaceClassifier aFClass = new(aFSpine, aPnear, global::Kerf.Engine.Foundation.Precision.Confusion());
                if (aFClass.State() != Out)
                {
                    if (aN1.Dot(aTanV) < 0.0)
                    {
                        myProfile.Reverse();
                    }
                }
                else
                {
                    if (aN1.Dot(aTanV) > 0.0)
                    {
                        myProfile.Reverse();
                    }
                }
            }
            break;
            default:
                break;
        }
    }

    //=================================================================================================

    private bool IsLid(in Face theF, NCollection_IndexedMap<Shape> theMapOfLids)
    {
        if (theMapOfLids.IsEmpty())
        {
            return false;
        }

        if (Tool.Surface(theF) is not PlaneSurface aPlnF)
        {
            return false;
        }

        for (int i = 1; i <= theMapOfLids.Extent(); i++)
        {
            Face aF = ShapeCasts.Face(theMapOfLids[i]);
            PlaneSurface? aPlane = Tool.Surface(aF) as PlaneSurface;

            if (ReferenceEquals(aPlane, aPlnF))
            {
                return true;
            }
        }

        return false;
    }

    //=================================================================================================

    /// <summary>
    /// Builds the evolved volume of the profile <paramref name="theProfile"/> along the planar spine
    /// <paramref name="theSpine"/>; <paramref name="theSolidReq"/> false gives a shell without the lids.
    /// </summary>
    public void Perform(in Wire theSpine, in Wire theProfile, double theTolerance, bool theSolidReq = true)
    {
        myErrorStatus = BRepFill_AdvancedEvolved_Empty;

        if (myFuzzyValue < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            myFuzzyValue = theTolerance;
        }

        // OCCT.NET: frozen inputs are copied at the entry (decision 1047).
        CopyAtEntry aSpineEntry = new(theSpine);
        CopyAtEntry aProfileEntry = new(theProfile);

        GetSpineAndProfile(ShapeCasts.Wire(aSpineEntry.Shape()), ShapeCasts.Wire(aProfileEntry.Shape()));

        myPipeShell = new Shape();
        myTopBottom = new Compound();
        myResult = new Shape();

        try
        {
            PerformSweep();

            GetLids();

            if (myErrorStatus != BRepFill_AdvancedEvolved_NotSolid)
            {
                return;
            }

            myResult = myPipeShell;

            BuildSolid();

            if ((myErrorStatus != BRepFill_AdvancedEvolved_OK) || theSolidReq)
            {
                return;
            }

            Shell aShell = new();
            NCollection_IndexedMap<Shape> aMFLids = new(ShapeHasher.Instance);
            Exploration.MapShapes(myTopBottom, ShapeType.Face, aMFLids);

            for (Explorer anExp = new(myResult, ShapeType.Face); anExp.More(); anExp.Next())
            {
                BrepBuilder aBB = new();
                if (aShell.IsNull)
                {
                    aBB.MakeShell(ref aShell);
                }

                Face aF = ShapeCasts.Face(anExp.Current());
                if (IsLid(aF, aMFLids))
                {
                    continue;
                }

                aBB.Add(aShell, aF);
            }

            if (!aShell.IsNull)
            {
                myResult = aShell;
            }
        }
        finally
        {
            // OCCT.NET: the caller's sub-shapes left unchanged stand in the result again.
            myResult = aProfileEntry.Finish(aSpineEntry.Finish(myResult));
        }
    }

    //=================================================================================================

    private void PerformSweep()
    {
        if (myErrorStatus != BRepFill_AdvancedEvolved_Empty)
        {
            return;
        }

        myErrorStatus = BRepFill_AdvancedEvolved_SweepError;

        BRepFill_PipeShell aPipe = new(mySpine);
        aPipe.SetTolerance(aPipeLinearTolerance, aPipeLinearTolerance, aPipeAngularTolerance);
        aPipe.SetTransition(BRepFill_Round);
        aPipe.Add(myProfile, false, false);

        if (aPipe.Build())
        {
            myErrorStatus = BRepFill_AdvancedEvolved_NoLids;
            myPipeShell = aPipe.InternalShape();
        }
    }

    //=================================================================================================

    private void GetLids()
    {
        if (myPipeShell.IsNull)
        {
            return;
        }

        if (Tool.IsClosed(myProfile))
        {
            // No need in lids creation
            myErrorStatus = BRepFill_AdvancedEvolved_NotSolid;
            return;
        }

        myErrorStatus = BRepFill_AdvancedEvolved_NoLids;

        BRepLib_FindSurface aFS = new(mySpine, -1.0, true);
        if (aFS.Surface() is not PlaneSurface aSurf)
        {
            myErrorStatus = BRepFill_AdvancedEvolved_NotPlanarSpine;
            return;
        }

        double aTol = Max(aFS.Tolerance(), aFS.ToleranceReached());
        aTol += myFuzzyValue;
        Box aProfBox = new();
        BRepBndLib.Add(myProfile, ref aProfBox);
        double aSqDiag = aProfBox.SquareExtent();
        // Square of the default angular tolerance in
        // BOPAlgo_Tools::EdgesToWires(...) and BOPAlgo_Tools::WiresToFaces(...) methods
        double aSqAnguarTol = aTol * aTol / aSqDiag;
        Direction3D aNormal = aSurf.Position().Direction();

        // Obtain free-edges from myPipeShell. All edges must be planar
        // and parallel to the plane of mySpine

        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aMapEF = new(ShapeHasher.Instance);

        Exploration.MapShapesAndAncestors(myPipeShell, ShapeType.Edge, ShapeType.Face, aMapEF);

        NCollection_List<Shape> aLE = new();

        double aDPMax = 0.0;
        for (int i = 1; i <= aMapEF.Extent(); i++)
        {
            NCollection_List<Shape> aListF = aMapEF[i];

            if (aListF.Extent() != 1)
            {
                continue;
            }

            Edge anE = ShapeCasts.Edge(aMapEF.FindKey(i));

            EdgeCurveAdaptor anAC = new(anE);
            if (!anAC.Is3DCurve())
            {
                // We are not interested in degenerated edges.
                continue;
            }

            anAC.D1(0.5 * (anAC.FirstParameter() + anAC.LastParameter()), out Point3D aPtmp, out Vector3D aTan);

            double aSqModulus = aTan.SquareMagnitude();
            if (aSqModulus < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                continue;
            }

            double aDP = aTan.XYZ().Dot(aNormal.XYZ());
            if (Math.Abs(aDP) > aDPMax)
            {
                aDPMax = Math.Abs(aDP);
            }
            if (aDP * aDP > aSqModulus * aSqAnguarTol)
            {
                // Only planar edges are considered
                continue;
            }

            aLE.Append(anE);
        }

        if (aLE.IsEmpty())
        {
            myErrorStatus = BRepFill_AdvancedEvolved_NotPlanarSpine;
            return;
        }

        // Split interfered edges
        Shape aFreeEdges = new();
        if (!PerformBoolean(aLE, ref aFreeEdges))
        {
            myErrorStatus = BRepFill_AdvancedEvolved_NotPlanarSpine;
            return;
        }

        // Collect all free edges to wires and create planar
        // top and bottom lids from these wires.
        BrepBuilder aBB = new();
        Compound aCompW = new(), aCompF = new();
        aBB.MakeCompound(ref aCompW);
        aBB.MakeCompound(ref aCompF);
        aBB.MakeCompound(ref myTopBottom);
        double anAngTol = Math.Sqrt(aSqAnguarTol);
        Shape aCompWS = aCompW, aCompFS = aCompF;
        BOPAlgo_Tools.EdgesToWires(aFreeEdges, ref aCompWS, true, anAngTol);
        BOPAlgo_Tools.WiresToFaces(aCompWS, ref aCompFS, anAngTol);

        {
            // Check orientation

            NCollection_IndexedMap<Shape> aMapV = new(ShapeHasher.Instance);
            Exploration.MapShapes(myPipeShell, ShapeType.Vertex, aMapV);
            Explorer anExp = new(aCompFS, ShapeType.Face);
            for (; anExp.More(); anExp.Next())
            {
                Face aF = ShapeCasts.Face(anExp.Current());
                // OCCT dereferences the down-cast plane without a check.
                PlaneSurface aPln = Tool.Surface(aF) as PlaneSurface ?? throw new NullReferenceException();
                Xyz aNorm = aPln.Position().Direction().XYZ();
                Xyz aLocP = aPln.Position().Location().XYZ();

                bool isFound = false;

                for (int i = 1; i <= aMapV.Extent(); i++)
                {
                    Vertex aV = ShapeCasts.Vertex(aMapV.FindKey(i));
                    Xyz aP = Tool.Pnt(aV).XYZ();

                    Xyz aDelta = aP - aLocP;
                    double aSqD = aDelta.SquareModulus();

                    if (aSqD < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        continue;
                    }

                    double aDP = aDelta.Dot(aNorm);

                    if (aDP * aDP < aSqD * global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        // aP is in the plane
                        continue;
                    }

                    if (aDP > 0.0)
                    {
                        aBB.Add(myTopBottom, aF.Reversed());
                    }
                    else
                    {
                        aBB.Add(myTopBottom, aF);
                    }

                    isFound = true;
                    break;
                }

                if (!isFound)
                {
                    aBB.Add(myTopBottom, aF);
                }
            }
        }

        myErrorStatus = BRepFill_AdvancedEvolved_NotSolid;
    }

    //=================================================================================================

    private void BuildSolid()
    {
        if (myErrorStatus != BRepFill_AdvancedEvolved_NotSolid)
        {
            return;
        }

        myErrorStatus = BRepFill_AdvancedEvolved_NotVolume;

        NCollection_Map<Shape> aMapF = new(ShapeHasher.Instance);
        NCollection_List<Shape> aLF = new(), aLSplits = new();
        Explorer anExpF = new();

        for (anExpF.Init(myPipeShell, ShapeType.Face); anExpF.More(); anExpF.Next())
        {
            Face aF = ShapeCasts.Face(anExpF.Current());
            if (!aMapF.Add(aF))
            {
                continue;
            }

            ReduceVertexTolerance(aF);
            CheckSingularityAndAdd(aF, myFuzzyValue, aLF, aLSplits);
        }

        {
            NCollection_List<Shape>.Iterator anItrS = new(aLSplits);
            for (; anItrS.More(); anItrS.Next())
            {
                Face aF = ShapeCasts.Face(anItrS.Value());
                aLF.Append(aF);
            }

            // Split interfered faces
            PerformBoolean(aLF, ref myPipeShell);
        }

        aLF.Clear();
        aMapF.Clear();
        for (anExpF.Init(myPipeShell, ShapeType.Face); anExpF.More(); anExpF.Next())
        {
            Face aF = ShapeCasts.Face(anExpF.Current());
            if (!aMapF.Add(aF))
            {
                continue;
            }

            aLF.Append(aF);
        }

        if (!myTopBottom.IsNull)
        {
            Iterator anItLids = new(myTopBottom);
            for (; anItLids.More(); anItLids.Next())
            {
                Face aF = ShapeCasts.Face(anItLids.Value());
                aLF.Append(aF);
            }
        }

        BOPAlgo_MakerVolume aMV = new();
        aMV.SetArguments(aLF);
        aMV.SetFuzzyValue(myFuzzyValue);
        aMV.SetIntersect(true);
        aMV.SetRunParallel(myIsParallel);
        aMV.SetAvoidInternalShapes(true);
        aMV.Perform();

        if (aMV.HasErrors())
        {
            return;
        }

        myResult = aMV.InternalShape();

        RemoveExcessSolids(aLSplits, myResult, aLF, aMV);

        UnifyShape();
        RemoveInternalWires(myResult);

        myErrorStatus = BRepFill_AdvancedEvolved_OK;
    }

    //=================================================================================================

    private void UnifyShape()
    {
        ShapeUpgrade_UnifySameDomain aUnifier = new();

        aUnifier.Initialize(myResult, true, true, false);
        aUnifier.SetSafeInputMode(true);
        aUnifier.AllowInternalEdges(false);
        aUnifier.SetLinearTolerance(aPipeLinearTolerance);
        aUnifier.SetAngularTolerance(aPipeAngularTolerance);
        aUnifier.Build();

        myResult = aUnifier.InternalShape();
    }

    //=================================================================================================

    private void ExtractOuterSolid(ref Shape theShape, NCollection_List<Shape> theArgsList)
    {
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aMapS = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(theShape, ShapeType.Face, ShapeType.Solid, aMapS);

        // theArgsList.Clear();
        NCollection_List<Shape> aNewList = new();
        int aNbF = aMapS.Extent();
        for (int i = 1; i <= aNbF; ++i)
        {
            if (aMapS[i].Extent() == 1)
            {
                aNewList.Append(aMapS.FindKey(i));
            }
        }

        if (aNewList.IsEmpty())
        {
            return;
        }

        {
            NCollection_List<Shape>.Iterator anItrF = new();

            bool isRemoved = true;
            while (isRemoved)
            {
                isRemoved = false;
                for (anItrF.Initialize(theArgsList); anItrF.More(); anItrF.Next())
                {
                    Face aF = ShapeCasts.Face(anItrF.Value());
                    if (!ContainsInList(aNewList, aF))
                    {
                        theArgsList.Remove(aF);
                        isRemoved = true;
                        break;
                    }
                }
            }
        }

        BOPAlgo_MakerVolume aMV = new();
        aMV.SetArguments(aNewList);
        aMV.SetIntersect(true);
        aMV.SetRunParallel(myIsParallel);
        aMV.SetAvoidInternalShapes(true);
        aMV.Perform();

        if (aMV.HasErrors())
        {
            return;
        }

        theShape = aMV.InternalShape();
    }

    //=================================================================================================

    private void RemoveExcessSolids(NCollection_List<Shape> theLSplits,
                                    in Shape theShape,
                                    NCollection_List<Shape> theArgsList,
                                    BOPAlgo_MakerVolume theMV)
    {
        if (myErrorStatus != BRepFill_AdvancedEvolved_NotVolume)
        {
            return;
        }

        Shape aResShape = theShape;

        Explorer anExpSo = new();
        for (int i = 0; i < 2; i++)
        {
            anExpSo.Init(aResShape, ShapeType.Solid);
            if (!anExpSo.More())
            {
                // No any solids
                myResult = aResShape;
                return;
            }

            anExpSo.Next();
            if (!anExpSo.More())
            {
                // Only one solid has been generated
                // OCCT: the explorer is past its only solid, so Current() is the explored shape itself (the
                // compound), which TopoDS::Solid casts unchecked in native's release build: the result is that
                // compound. The port keeps it without the cast (which would throw Standard_TypeMismatch).
                myResult = anExpSo.Current();
                return;
            }

            if (i != 0)
            {
                break;
            }

            ExtractOuterSolid(ref aResShape, theArgsList);
        }

        NCollection_List<Shape> aSolidList = new();

        // Look for all solids containing lids
        {
            anExpSo.Init(aResShape, ShapeType.Solid);
            for (; anExpSo.More(); anExpSo.Next())
            {
                Solid aSol = ShapeCasts.Solid(anExpSo.Current());
                NCollection_IndexedMap<Shape> aMapF = new(ShapeHasher.Instance);
                Exploration.MapShapes(aSol, aMapF);

                bool areThereLids = false;
                Explorer anExpLids = new(myTopBottom, ShapeType.Face);
                for (; anExpLids.More(); anExpLids.Next())
                {
                    areThereLids = true;
                    Face aFLid = ShapeCasts.Face(anExpLids.Current());
                    int aFIdx = aMapF.FindIndex(aFLid);
                    if (aFIdx < 1)
                    {
                        continue;
                    }

                    Face aFSol = ShapeCasts.Face(aMapF.FindKey(aFIdx));

                    if (aFSol.IsEqual(aFLid))
                    {
                        aSolidList.Append(aSol);
                    }

                    break;
                }

                if (!areThereLids)
                {
                    break;
                }
            }

            if (aSolidList.Extent() < 1)
            {
                myResult = aResShape;
                return;
            }

            if (aSolidList.Extent() == 1)
            {
                myResult = aSolidList.First();
                return;
            }

            if (aSolidList.Extent() > 0)
            {
                BrepBuilder aBB = new();
                CompSolid aCompSol = new();
                aBB.MakeCompSolid(ref aCompSol);
                NCollection_List<Shape>.Iterator anItl = new(aSolidList);
                for (; anItl.More(); anItl.Next())
                {
                    Solid aSol = ShapeCasts.Solid(anItl.Value());
                    aBB.Add(aCompSol, aSol);
                }

                aResShape = aCompSol;
                aSolidList.Clear();
            }
        }

        {
            // Remove Split faces from the list of arguments
            NCollection_List<Shape>.Iterator anItl = new(theLSplits);
            for (; anItl.More(); anItl.Next())
            {
                Face aF = ShapeCasts.Face(anItl.Value());
                theArgsList.Remove(aF);
            }

            // Create a list of invalid faces. The face is invalid if
            // BOPAlgo_MakerVolume changes its orientation while creating solids.
            // Faces from theLSplits are not checked.
            NCollection_List<Shape> aListInvFaces = new();
            for (anItl.Initialize(theArgsList); anItl.More(); anItl.Next())
            {
                Face aF = ShapeCasts.Face(anItl.Value());
                foreach (Shape aMod in theMV.Modified(aF))
                {
                    Face aFM = ShapeCasts.Face(aMod);

                    if (aFM.Orientation() != aF.Orientation())
                    {
                        aListInvFaces.Append(aFM);
                    }
                }
            }

            for (anExpSo.Init(aResShape, ShapeType.Solid); anExpSo.More(); anExpSo.Next())
            {
                Solid aSo = ShapeCasts.Solid(anExpSo.Current());
                NCollection_IndexedMap<Shape> aMapF = new(ShapeHasher.Instance);
                Exploration.MapShapes(aSo, ShapeType.Face, aMapF);
                bool isToDelete = false;

                for (anItl.Initialize(aListInvFaces); anItl.More(); anItl.Next())
                {
                    Face aF = ShapeCasts.Face(anItl.Value());
                    if (aMapF.Contains(aF))
                    {
                        isToDelete = true;
                        break;
                    }
                }

                if (isToDelete)
                {
                    continue;
                }

                for (anItl.Initialize(theArgsList); anItl.More(); anItl.Next())
                {
                    Face aF = ShapeCasts.Face(anItl.Value());
                    int anIdx = aMapF.FindIndex(aF);
                    if (anIdx == 0)
                    {
                        continue;
                    }

                    Face aF1 = ShapeCasts.Face(aMapF.FindKey(anIdx));

                    // aF and aF1 are same shapes. Check if they are equal.

                    if (!aF.IsEqual(aF1))
                    {
                        isToDelete = true;
                        break;
                    }
                }

                if (isToDelete)
                {
                    continue;
                }

                aSolidList.Append(aSo);
            }
        }

        if (aSolidList.Extent() < 1)
        {
            myResult = aResShape;
            return;
        }

        if (aSolidList.Extent() == 1)
        {
            myResult = aSolidList.First();
            return;
        }

        {
            BrepBuilder aBB = new();
            CompSolid aCmpSol = new();
            aBB.MakeCompSolid(ref aCmpSol);

            for (NCollection_List<Shape>.Iterator anItl = new(aSolidList); anItl.More(); anItl.Next())
            {
                Solid aSo = ShapeCasts.Solid(anItl.Value());
                aBB.Add(aCmpSol, aSo);
            }

            myResult = aCmpSol;
        }
    }

    //=======================================================================
    // function : RebuildFaces
    // purpose  : Creates a wires from theEdges and puts it to the new face
    //            which is empty-copied from theSourceFace.
    //=======================================================================
    private static void RebuildFaces(NCollection_List<Shape> theLE,
                                     in Face theSourceFace,
                                     NCollection_List<Shape> theList)
    {
        // build new faces
        BOPAlgo_BuilderFace aBF = new();

        Face aF = ShapeCasts.Face(theSourceFace.Oriented(Forward));

        aBF.SetFace(aF);
        aBF.SetShapes(theLE);

        aBF.Perform();

        NCollection_List<Shape> aLFR = aBF.Areas();

        if (aLFR.IsEmpty())
        {
            theList.Append(theSourceFace);
            return;
        }

        NCollection_List<Shape>.Iterator aItFR = new(aLFR);
        for (; aItFR.More(); aItFR.Next())
        {
            Shape aFR = ShapeCasts.Face(aItFR.Value());
            theList.Append(aFR);
        }
    }

    //=======================================================================
    // function : MakeEdgeDegenerated
    // purpose  : Returns TRUE if degenerated edge has been created.
    //           Every degenerated edge (to split) must be added in theLEdges twice
    //           with different orientations. Moreover, Degenerated edges cannot be shared.
    //           Therefore, make copy of them before adding.
    //=======================================================================
    private static bool MakeEdgeDegenerated(in Vertex theV,
                                            in Face theFace,
                                            in Point2D thePf,
                                            in Point2D thePl,
                                            NCollection_List<Shape> theLEdges)
    {
        FaceSurfaceAdaptor anAS = new(theFace, false);

        double aTol = 2.0 * Tool.Tolerance(theV);
        double aTolU = anAS.UResolution(aTol), aTolV = anAS.VResolution(aTol);

        if ((Math.Abs(thePf.X() - thePl.X()) < aTolU) && (Math.Abs(thePf.Y() - thePl.Y()) < aTolV))
        {
            return false;
        }

        Vertex aVf = ShapeCasts.Vertex(theV.Oriented(Forward)),
                      aVl = ShapeCasts.Vertex(theV.Oriented(Reversed));

        Xy aV = thePl.XY() - thePf.XY();
        LineCurve2D aL1 = new(thePf, new Direction2D(aV));
        LineCurve2D aL2 = new(thePl, new Direction2D(aV.Reversed()));

        BrepBuilder aBB = new();
        Edge anEdegen1 = new(), anEdegen2 = new();
        aBB.MakeEdge(ref anEdegen1);
        aBB.MakeEdge(ref anEdegen2);

        aBB.UpdateEdge(anEdegen1, aL1, theFace, global::Kerf.Engine.Foundation.Precision.Confusion());
        aBB.UpdateEdge(anEdegen2, aL2, theFace, global::Kerf.Engine.Foundation.Precision.Confusion());

        anEdegen1.Orientation(Forward);
        anEdegen2.Orientation(Forward);

        aBB.Add(anEdegen1, aVf);
        aBB.Add(anEdegen1, aVl);
        aBB.Add(anEdegen2, aVf);
        aBB.Add(anEdegen2, aVl);

        aBB.Degenerated(anEdegen1, true);
        aBB.Degenerated(anEdegen2, true);

        double aLPar = aV.Modulus();
        aBB.Range(anEdegen1, 0.0, aLPar);
        aBB.Range(anEdegen2, 0.0, aLPar);

        theLEdges.Append(anEdegen1);
        theLEdges.Append(anEdegen2);

        return true;
    }

    //=================================================================================================

    private static void InsertEDegenerated(in Face theFace, NCollection_List<Shape> theLEdges)
    {
        BrepBuilder aBB = new();
        Wire aWir = new();
        aBB.MakeWire(ref aWir);

        NCollection_List<Shape>.Iterator anItr = new(theLEdges);
        for (; anItr.More(); anItr.Next())
        {
            Edge anE = ShapeCasts.Edge(anItr.Value());
            aBB.Add(aWir, anE);
        }

        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aMapVE = new(ShapeHasher.Instance);
        Exploration.MapShapesAndUniqueAncestors(aWir, ShapeType.Vertex, ShapeType.Edge, aMapVE);

        WireExplorer anExp = new(aWir, theFace);

        Edge anE1 = anExp.Current(), aFirstEdge, aLastEdge = new();

        if (anE1.IsNull)
        {
            // It is possible if aWir contains
            // only INTERNAL/EXTERNAL edges.

            return;
        }

        aFirstEdge = anE1;
        anExp.Next();

        // Map containing all vertices of degenerated edges
        NCollection_Map<Shape> aMapVofDE = new(ShapeHasher.Instance);

        {
            Explorer anExpDE = new(aWir, ShapeType.Edge);
            for (; anExpDE.More(); anExpDE.Next())
            {
                Edge anE = ShapeCasts.Edge(anExpDE.Current());
                if (!Tool.Degenerated(anE))
                {
                    continue;
                }

                Exploration.Vertices(anE, out Vertex aV1, out Vertex aV2);

                // aV1 and aV2 are SAME vertices

                aMapVofDE.Add(aV1);
            }
        }

        for (; anExp.More(); anExp.Next())
        {
            Edge anE2 = anExp.Current();
            aLastEdge = anE2;

            Vertex aVertCurr = anExp.CurrentVertex();

            if (aMapVofDE.Contains(aVertCurr))
            {
                // Necessary degenerated edge has already been created.
                anE1 = anE2;
                continue;
            }

            Curve2D aC1 = Tool.CurveOnSurface(anE1, theFace, out double aF, out double aL)!,
                         aC2 = Tool.CurveOnSurface(anE2, theFace, out aF, out aL)!;
            aF = Tool.Parameter(aVertCurr, anE1);
            aL = Tool.Parameter(aVertCurr, anE2);
            Point2D aPf = aC1.Value(aF), aPl = aC2.Value(aL);

            if (MakeEdgeDegenerated(aVertCurr, theFace, aPf, aPl, theLEdges))
            {
                aMapVofDE.Add(aVertCurr);
                anE1 = anE2;
                continue;
            }

            bool aFound = aMapVE.FindFromKey(aVertCurr, out NCollection_List<Shape> anEList);
            if (aFound && (anEList.Extent() <= 2))
            {
                anE1 = anE2;
                continue;
            }

            // Case like cone with apex. In 2D space all is OK
            // (therefore BRepTools_WireExplorer processes this case
            // correctly). But in 3D-space, we have several edges with
            // the same vertex. Cone apex must be plugged by degenerated edge.

            bool hasDegenerated = false;
            // OCCT dereferences the seek's result without a check.
            anItr.Initialize(anEList ?? throw new NullReferenceException());
            for (; anItr.More(); anItr.Next())
            {
                Edge anEdge = ShapeCasts.Edge(anItr.Value());
                if (Tool.Degenerated(anEdge))
                {
                    hasDegenerated = true;
                    break;
                }
            }

            if (hasDegenerated)
            {
                anE1 = anE2;
                continue;
            }

            // Look for the pair for anE1 and anE2 edges
            for (int i = 0; i < 2; i++)
            {
                Point2D aPoint = i != 0 ? aPl : aPf;
                anItr.Initialize(anEList);
                for (; anItr.More(); anItr.Next())
                {
                    Edge anEdge = ShapeCasts.Edge(anItr.Value());

                    if (anEdge.IsSame(anE1) || anEdge.IsSame(anE2))
                    {
                        continue;
                    }

                    Curve2D aC = Tool.CurveOnSurface(anEdge, theFace, out aF, out aL)!;
                    aF = Tool.Parameter(aVertCurr, anEdge);
                    Point2D aP = aC.Value(aF);

                    if (MakeEdgeDegenerated(aVertCurr, theFace, aPoint, aP, theLEdges))
                    {
                        aMapVofDE.Add(aVertCurr);
                        i = 2;
                        break;
                    }
                }
            }

            anE1 = anE2;
        }

        if (aFirstEdge.IsNull || aLastEdge.IsNull)
        {
            return;
        }

        // TopExp::CommonVertex(...) does not work
        // if edges have more than one pair of common vertex
        //(e.g. two halves of circle). Here, we process this case.
        Vertex[] aV = new Vertex[4];
        Exploration.Vertices(aFirstEdge, out aV[0], out aV[1]);
        if (!aV[0].IsNull && aV[0].IsSame(aV[1]))
        {
            // Possible reason is the NOT-CLOSED edge
            // has only single vertex and is covered by it.
            return;
        }

        Exploration.Vertices(aLastEdge, out aV[2], out aV[3]);
        if (!aV[2].IsNull && aV[2].IsSame(aV[3]))
        {
            // Possible reason is the NOT-CLOSED edge
            // has only single vertex and is covered by it.
            return;
        }

        for (int anIDFE = 0; anIDFE < 2; anIDFE++)
        {
            for (int anIDLE = 2; anIDLE < 4; anIDLE++)
            {
                if (!aV[anIDFE].IsSame(aV[anIDLE]))
                {
                    continue;
                }

                if (aMapVE.FindFromKey(aV[anIDFE], out NCollection_List<Shape> anEList) && (anEList.Extent() > 2))
                {
                    // Causes:
                    //  1. Non-manifold topology.
                    //  2. Case such as: two edges meeting a seam at the same vertex, which must not be joined.
                    continue;
                }

                Curve2D aC1 = Tool.CurveOnSurface(aFirstEdge, theFace, out double aF, out double aL)!,
                             aC2 = Tool.CurveOnSurface(aLastEdge, theFace, out aF, out aL)!;
                aF = Tool.Parameter(aV[anIDFE], aFirstEdge);
                aL = Tool.Parameter(aV[anIDLE], aLastEdge);
                Point2D aPf = aC1.Value(aF), aPl = aC2.Value(aL);

                MakeEdgeDegenerated(aV[anIDFE], theFace, aPf, aPl, theLEdges);
            }
        }
    }

    //=======================================================================
    // function : CheckSingularityAndAdd
    // purpose  : Returns TRUE if theF has been split
    //=======================================================================
    private bool CheckSingularityAndAdd(in Face theF,
                                        double theFuzzyToler,
                                        NCollection_List<Shape> theListOfFaces,
                                        NCollection_List<Shape> theListOfSplits)
    {
        FaceSurfaceAdaptor anAS = new(theF, false);
        SurfaceKind aSType = anAS.GetType();

        if (aSType == SurfaceKind.OffsetSurface)
        {
            aSType = anAS.BasisSurface()!.GetType();
        }

        if (aSType == Plane)
        {
            NCollection_Map<Shape> aME = new(ShapeHasher.Instance);
            NCollection_List<Shape> aLE = new();
            Explorer anExp = new(theF, ShapeType.Edge);
            for (; anExp.More(); anExp.Next())
            {
                Edge anE = ShapeCasts.Edge(anExp.Current());

                if (aME.Add(anE))
                {
                    aLE.Append(anE);
                }
            }

            // Split interfered edges
            BOPAlgo_PaveFiller aPF = new();
            aPF.SetArguments(aLE);
            aPF.SetRunParallel(myIsParallel);

            aPF.Perform();
            if (aPF.HasErrors())
            {
                theListOfFaces.Append(theF);
                return false;
            }

            BOPDS_DS aDS = aPF.DS();
            if (aDS.NbShapes() == aDS.NbSourceShapes())
            {
                // Interfered edges have not been detected
                theListOfFaces.Append(theF);
                return false;
            }

            BOPAlgo_Builder aBuilder = new();
            aBuilder.SetArguments(aLE);
            aBuilder.SetRunParallel(myIsParallel);
            aBuilder.PerformWithFiller(aPF);
            if (aBuilder.HasErrors())
            {
                theListOfFaces.Append(theF);
                return false;
            }

            Shape anEdges = aBuilder.InternalShape();

            BrepBuilder aBB = new();
            Compound aCompW = new(), aCompF = new();
            aBB.MakeCompound(ref aCompW);
            aBB.MakeCompound(ref aCompF);
            Shape aCompWS = aCompW, aCompFS = aCompF;
            BOPAlgo_Tools.EdgesToWires(anEdges, ref aCompWS, true);
            BOPAlgo_Tools.WiresToFaces(aCompWS, ref aCompFS);

            aME.Clear();
            anExp.Init(aCompFS, ShapeType.Face);
            for (; anExp.More(); anExp.Next())
            {
                Face aF = ShapeCasts.Face(anExp.Current());
                theListOfSplits.Append(aF);
            }

            return true;
        }

        if ((aSType != SurfaceKind.Cone) && (aSType != SurfaceKind.Sphere) && (aSType != SurfaceKind.BezierSurface)
            && (aSType != SurfaceKind.BSplineSurface) && (aSType != SurfaceOfRevolution))
        {
            theListOfFaces.Append(theF);
            return false;
        }

        BrepBuilder aBB2 = new();

        Compound aCWires = new();
        aBB2.MakeCompound(ref aCWires);

        bool isSplit = false;
        NCollection_List<Shape> aListEdges = new();

        Face aFace = ShapeCasts.Face(theF.Oriented(Forward));

        for (Iterator anExpW = new(aFace); anExpW.More(); anExpW.Next())
        {
            Wire aWir = ShapeCasts.Wire(anExpW.Value());

            NCollection_List<Shape> aLGF = new();
            Explorer anEExp = new(aWir, ShapeType.Edge);
            for (; anEExp.More(); anEExp.Next())
            {
                Edge anE = ShapeCasts.Edge(anEExp.Current());
                aLGF.Append(anE);
            }

            BOPAlgo_PaveFiller aPF = new();
            aPF.SetArguments(aLGF);
            aPF.SetFuzzyValue(theFuzzyToler);
            aPF.Perform();

            if (aPF.HasErrors())
            {
                continue;
            }

            BOPDS_DS aDS = aPF.DS();
            if (aDS.NbShapes() == aDS.NbSourceShapes())
            {
                // No new shapes have been created
                continue;
            }

            BOPAlgo_Builder aBuilder = new();
            aBuilder.SetArguments(aLGF);
            aBuilder.SetRunParallel(myIsParallel);
            aBuilder.SetNonDestructive(true);
            aBuilder.PerformWithFiller(aPF);
            if (aBuilder.HasErrors())
            {
                continue;
            }

            NCollection_List<Shape> aLE = new();
            NCollection_List<Shape>.Iterator aBItr = new(aLGF);
            for (; aBItr.More(); aBItr.Next())
            {
                Edge aSh = ShapeCasts.Edge(aBItr.Value());
                var aLM = aBuilder.Modified(aSh);
                if (aLM.Count == 0 || Tool.Degenerated(aSh))
                {
                    aLE.Append(aSh);
                    continue;
                }

                foreach (Shape aMod in aLM)
                {
                    Edge anEM = ShapeCasts.Edge(aMod);
                    aLE.Append(anEM);
                }
            }

            isSplit = true;
            InsertEDegenerated(aFace, aLE);
            aListEdges.Append(aLE);
        }

        if (!isSplit)
        {
            theListOfFaces.Append(theF);
            return false;
        }

        RebuildFaces(aListEdges, theF, theListOfSplits);

        NCollection_List<Shape>.Iterator anItrS = new(theListOfSplits);
        for (; anItrS.More(); anItrS.Next())
        {
            Face aF = ShapeCasts.Face(anItrS.Value());
            theListOfFaces.Append(aF.Oriented(theF.Orientation()));
        }

        return true;
    }

    //=================================================================================================

    private static bool ContainsInList(NCollection_List<Shape> theL, in Shape theObject)
    {
        NCollection_List<Shape>.Iterator anIt = new(theL);
        for (; anIt.More(); anIt.Next())
        {
            if (anIt.Value().IsSame(theObject))
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
    private static void FindInternals(in Shape theS, NCollection_List<Shape> theLInt)
    {
        Iterator itS = new(theS);
        for (; itS.More(); itS.Next())
        {
            Shape aSS = itS.Value();
            if (aSS.Orientation() == Internal)
            {
                theLInt.Append(aSS);
            }
            else
            {
                Iterator itSS = new(aSS);
                for (; itSS.More(); itSS.Next())
                {
                    if (itSS.Value().Orientation() == Internal)
                    {
                        theLInt.Append(aSS);
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
    private static void RemoveInternalWires(in Shape theShape)
    {
        Explorer anExpF = new(theShape, ShapeType.Face);
        for (; anExpF.More(); anExpF.Next())
        {
            Shape aF = anExpF.Current();
            NCollection_List<Shape> aLWToRemove = new();
            FindInternals(aF, aLWToRemove);
            if (aLWToRemove.Extent() != 0)
            {
                aF.Free(true);
                NCollection_List<Shape>.Iterator itR = new(aLWToRemove);
                for (; itR.More(); itR.Next())
                {
                    new BrepBuilder().Remove(aF, itR.Value());
                }
                aF.Free(false);
            }
        }
    }

    //=================================================================================================

    private static void ProcessVertex(in Vertex aV, NCollection_List<Shape> aLE, NCollection_List<Shape> aLF)
    {
        double aTol, aD2, aTolMax2, aTolE, aParam;
        Point3D aPC3D;
        Point2D aPC2D;
        Orientation anOrV;

        NCollection_List<Shape>.Iterator anIt = new();
        Explorer aVExp = new();

        //
        aTolMax2 = -1.0e6;
        //
        BrepTVertex TV = (BrepTVertex)aV.TShape()!;
        Point3D aPV3D = TV.Pnt();
        aTol = Tool.Tolerance(aV);
        //
        anIt.Initialize(aLE);
        for (; anIt.More(); anIt.Next())
        {
            Edge aE = ShapeCasts.Edge(anIt.Value());
            //
            BrepTEdge TE = (BrepTEdge)aE.TShape()!;
            Placement Eloc = aE.Location();
            //
            aVExp.Init(aE, ShapeType.Vertex);
            for (; aVExp.More(); aVExp.Next())
            {
                Vertex aVx = ShapeCasts.Vertex(aVExp.Current());
                //
                if (!aVx.IsSame(aV))
                {
                    continue;
                }
                //
                anOrV = aVx.Orientation();
                if (anOrV != Forward && anOrV != Reversed)
                {
                    continue;
                }
                //
                NCollection_List<CurveRepresentation> aLCR = TE.Curves();
                NCollection_List<CurveRepresentation>.Iterator itcr = new(aLCR);
                for (; itcr.More(); itcr.Next())
                {
                    CurveRepresentation cr = itcr.Value();
                    Placement loc = cr.Location();
                    Placement L = (Eloc * loc).Predivided(aV.Location());
                    //
                    // 3D-Curve
                    if (cr.IsCurve3D)
                    {
                        Curve3D? aC3D = cr.Curve3D();
                        //
                        if (aC3D is null)
                        {
                            continue;
                        }
                        // 3D-point treatment
                        aParam = Tool.Parameter(aVx, aE);
                        aPC3D = aC3D.Value(aParam);
                        aPC3D.Transform(L.Transformation());
                        aD2 = aPV3D.SquareDistance(aPC3D);
                        if (aD2 > aTolMax2)
                        {
                            aTolMax2 = aD2;
                        }
                        //
                    } // if (cr->IsCurve3D())
                    //
                    // 2D-Curve
                    else if (cr.IsCurveOnSurface())
                    {
                        Curve2D? aC2D = cr.PCurve();
                        if (aC2D is null)
                        {
                            continue;
                        }
                        // Surface
                        Surface aS = cr.Surface()!;
                        //
                        // 2D-point treatment
                        aParam = Tool.Parameter(aVx, aE, aS, L);
                        aPC2D = aC2D.Value(aParam);
                        aS.D0(aPC2D.X(), aPC2D.Y(), out aPC3D);
                        aPC3D.Transform(L.Transformation());
                        aD2 = aPV3D.SquareDistance(aPC3D);
                        if (aD2 > aTolMax2)
                        {
                            aTolMax2 = aD2;
                        }
                    } // if (cr->IsCurveOnSurface())

                } // for (; itcr.More(); itcr.Next())
            } // for (; aVExp.More(); aVExp.Next())
        } // for (; anIt.More(); anIt.Next())
        // #########################################################
        //
        //  Reducing
        if (aTolMax2 < 0.0)
        {
            return;
        }
        //
        aTolMax2 = Math.Sqrt(aTolMax2);
        if (aTolMax2 > aTol)
        {
            return;
        }
        //
        anIt.Initialize(aLE);
        for (; anIt.More(); anIt.Next())
        {
            Edge aE = ShapeCasts.Edge(anIt.Value());

            aTolE = Tool.Tolerance(aE);
            if (aTolMax2 < aTolE)
            {
                aTolMax2 = aTolE;
            }
        }
        //
        anIt.Initialize(aLF);
        for (; anIt.More(); anIt.Next())
        {
            Face aF = ShapeCasts.Face(anIt.Value());

            aTolE = Tool.Tolerance(aF);
            if (aTolMax2 < aTolE)
            {
                aTolMax2 = aTolE;
            }
        }
        //
        if (aTolMax2 > aTol)
        {
            return;
        }
        //
        // Update Tolerance
        // with a small margin
        TV.Tolerance(aTolMax2 + aTolMax2 * 0.0001);
    }

    //=================================================================================================

    private static void ReduceVertexTolerance(in Shape aS)
    {
        int i, aNbV;
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aVEMap = new(ShapeHasher.Instance),
            aVFMap = new(ShapeHasher.Instance);

        Exploration.MapShapesAndUniqueAncestors(aS, ShapeType.Vertex, ShapeType.Edge, aVEMap);
        Exploration.MapShapesAndUniqueAncestors(aS, ShapeType.Vertex, ShapeType.Face, aVFMap);

        aNbV = aVEMap.Extent();
        for (i = 1; i <= aNbV; i++)
        {
            Vertex aV = ShapeCasts.Vertex(aVEMap.FindKey(i));
            NCollection_List<Shape> aLE = aVEMap[i];
            NCollection_List<Shape> aLF = aVFMap.FindFromKey(aV);

            ProcessVertex(aV, aLE, aLF);
        }
    }
}
