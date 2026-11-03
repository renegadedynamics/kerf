// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_AdvancedEvolved.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_AdvancedEvolved.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2018-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.BRepExtrema_SupportType;
using static Occt.BRepFill_TransitionStyle;
using static Kerf.Geometry.SurfaceKind;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

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
    private TopoDS_Wire mySpine = new();
    private TopoDS_Wire myProfile = new();
    private TopoDS_Shape myPipeShell = new();
    private TopoDS_Compound myTopBottom = new(); // Lids can be split on several faces
    private TopoDS_Shape myResult = new();
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
    public TopoDS_Shape Shape()
    {
        myResult.TShape()?.FreezeGraph();
        return myResult;
    }

    /// <summary><see cref="Shape"/> without freezing, for ported code.</summary>
    internal TopoDS_Shape InternalShape() => myResult;

    /// <summary>OCCT's directory for the debug shapes; the port writes none, so it is ignored.</summary>
    public void SetTemporaryDirectory(string thePath)
    {
    }

    /// <summary>Sets or unsets the parallel mode of the Booleans.</summary>
    public void SetParallelMode(bool theVal) => myIsParallel = theVal;

    //=================================================================================================

    private bool PerformBoolean(NCollection_List<TopoDS_Shape> theArgsList, ref TopoDS_Shape theResult)
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

    private void GetSpineAndProfile(in TopoDS_Wire theSpine, in TopoDS_Wire theProfile)
    {
        mySpine = theSpine;
        myProfile = theProfile;

        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMVEP = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(theProfile, TopAbs_VERTEX, TopAbs_EDGE, aMVEP);

        Vector3D aN2 = new();
        Point3D aLoc = new();

        for (int i = 1; i <= aMVEP.Extent(); i++)
        {
            TopoDS_Vertex aVC = TopoDS.Vertex(aMVEP.FindKey(i));

            NCollection_List<TopoDS_Shape> aLE = aMVEP.FindFromIndex(i);

            if (aLE.Extent() < 2)
            {
                continue;
            }

            TopoDS_Edge anE1 = TopoDS.Edge(aLE.First());
            TopoDS_Edge anE2 = TopoDS.Edge(aLE.Last());

            BRepAdaptor_Curve anAC1 = new(anE1), anAC2 = new(anE2);

            double aPar1 = BRep_Tool.Parameter(aVC, anE1);
            double aPar2 = BRep_Tool.Parameter(aVC, anE2);

            anAC1.D1(aPar1, out Point3D aP, out Vector3D aT1);
            anAC1.D1(aPar2, out aP, out Vector3D aT2); // OCCT: anAC1, not anAC2

            aN2 = aT1.Crossed(aT2);

            if (aN2.SquareMagnitude() > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
            {
                aLoc = BRep_Tool.Pnt(aVC);
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
                    TopoDS_Vertex aV = TopoDS.Vertex(anExtr.SupportOnShape1(anIdxMin));
                    NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMVES =
                        new(TopTools_ShapeMapHasher.Instance);
                    TopExp.MapShapesAndAncestors(theSpine, TopAbs_VERTEX, TopAbs_EDGE, aMVES);

                    NCollection_List<TopoDS_Shape> aLE = aMVES.FindFromKey(aV);

                    TopoDS_Edge anE1 = TopoDS.Edge(aLE.First());
                    TopoDS_Edge anE2 = TopoDS.Edge(aLE.Last());

                    BRepAdaptor_Curve anAC1 = new(anE1), anAC2 = new(anE2);

                    double aPar1 = BRep_Tool.Parameter(aV, anE1);
                    double aPar2 = BRep_Tool.Parameter(aV, anE2);

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
                    TopoDS_Edge anE = TopoDS.Edge(anExtr.SupportOnShape1(anIdxMin));
                    BRepAdaptor_Curve anAC = new(anE);
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

                TopoDS_Face aFSpine = aMkFSpine.InternalFace();
                // OCCT dereferences the down-cast plane without a check.
                Geom_Plane aPlnSpine = BRep_Tool.Surface(aFSpine) as Geom_Plane ?? throw new NullReferenceException();
                Vector3D aN1 = new(aPlnSpine.Axis().Direction());
                Vector3D aTanV;

                if (anExtr.SupportTypeShape2(anIdxMin) == BRepExtrema_IsVertex)
                {
                    TopoDS_Vertex aV = TopoDS.Vertex(anExtr.SupportOnShape2(anIdxMin));
                    NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMVES =
                        new(TopTools_ShapeMapHasher.Instance);
                    TopExp.MapShapesAndAncestors(theProfile, TopAbs_VERTEX, TopAbs_EDGE, aMVES);

                    NCollection_List<TopoDS_Shape> aLE = aMVES.FindFromKey(aV);

                    TopoDS_Edge anE1 = TopoDS.Edge(aLE.First());
                    TopoDS_Edge anE2 = TopoDS.Edge(aLE.Last());

                    BRepAdaptor_Curve anAC1 = new(anE1), anAC2 = new(anE2);

                    double aPar1 = BRep_Tool.Parameter(aV, anE1);
                    double aPar2 = BRep_Tool.Parameter(aV, anE2);

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
                    TopoDS_Edge anE = TopoDS.Edge(anExtr.SupportOnShape2(anIdxMin));
                    BRepAdaptor_Curve anAC = new(anE);
                    anExtr.ParOnEdgeS2(anIdxMin, out double aPar);

                    anAC.D1(aPar, out Point3D aP, out aTanV);
                }

                // The point in the profile, which is the nearest to the spine
                Point3D aPnear = anExtr.PointOnShape2(anIdxMin);

                BRepClass_FaceClassifier aFClass = new(aFSpine, aPnear, global::Kerf.Engine.Foundation.Precision.Confusion());
                if (aFClass.State() != TopAbs_OUT)
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

    private bool IsLid(in TopoDS_Face theF, NCollection_IndexedMap<TopoDS_Shape> theMapOfLids)
    {
        if (theMapOfLids.IsEmpty())
        {
            return false;
        }

        if (BRep_Tool.Surface(theF) is not Geom_Plane aPlnF)
        {
            return false;
        }

        for (int i = 1; i <= theMapOfLids.Extent(); i++)
        {
            TopoDS_Face aF = TopoDS.Face(theMapOfLids[i]);
            Geom_Plane? aPlane = BRep_Tool.Surface(aF) as Geom_Plane;

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
    public void Perform(in TopoDS_Wire theSpine, in TopoDS_Wire theProfile, double theTolerance, bool theSolidReq = true)
    {
        myErrorStatus = BRepFill_AdvancedEvolved_Empty;

        if (myFuzzyValue < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            myFuzzyValue = theTolerance;
        }

        // OCCT.NET: frozen inputs are copied at the entry (decision 1047).
        BRepTools_CopyAtEntry aSpineEntry = new(theSpine);
        BRepTools_CopyAtEntry aProfileEntry = new(theProfile);

        GetSpineAndProfile(TopoDS.Wire(aSpineEntry.Shape()), TopoDS.Wire(aProfileEntry.Shape()));

        myPipeShell = new TopoDS_Shape();
        myTopBottom = new TopoDS_Compound();
        myResult = new TopoDS_Shape();

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

            TopoDS_Shell aShell = new();
            NCollection_IndexedMap<TopoDS_Shape> aMFLids = new(TopTools_ShapeMapHasher.Instance);
            TopExp.MapShapes(myTopBottom, TopAbs_FACE, aMFLids);

            for (TopExp_Explorer anExp = new(myResult, TopAbs_FACE); anExp.More(); anExp.Next())
            {
                BRep_Builder aBB = new();
                if (aShell.IsNull())
                {
                    aBB.MakeShell(ref aShell);
                }

                TopoDS_Face aF = TopoDS.Face(anExp.Current());
                if (IsLid(aF, aMFLids))
                {
                    continue;
                }

                aBB.Add(aShell, aF);
            }

            if (!aShell.IsNull())
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
        if (myPipeShell.IsNull())
        {
            return;
        }

        if (BRep_Tool.IsClosed(myProfile))
        {
            // No need in lids creation
            myErrorStatus = BRepFill_AdvancedEvolved_NotSolid;
            return;
        }

        myErrorStatus = BRepFill_AdvancedEvolved_NoLids;

        BRepLib_FindSurface aFS = new(mySpine, -1.0, true);
        if (aFS.Surface() is not Geom_Plane aSurf)
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

        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMapEF = new(TopTools_ShapeMapHasher.Instance);

        TopExp.MapShapesAndAncestors(myPipeShell, TopAbs_EDGE, TopAbs_FACE, aMapEF);

        NCollection_List<TopoDS_Shape> aLE = new();

        double aDPMax = 0.0;
        for (int i = 1; i <= aMapEF.Extent(); i++)
        {
            NCollection_List<TopoDS_Shape> aListF = aMapEF[i];

            if (aListF.Extent() != 1)
            {
                continue;
            }

            TopoDS_Edge anE = TopoDS.Edge(aMapEF.FindKey(i));

            BRepAdaptor_Curve anAC = new(anE);
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
        TopoDS_Shape aFreeEdges = new();
        if (!PerformBoolean(aLE, ref aFreeEdges))
        {
            myErrorStatus = BRepFill_AdvancedEvolved_NotPlanarSpine;
            return;
        }

        // Collect all free edges to wires and create planar
        // top and bottom lids from these wires.
        BRep_Builder aBB = new();
        TopoDS_Compound aCompW = new(), aCompF = new();
        aBB.MakeCompound(ref aCompW);
        aBB.MakeCompound(ref aCompF);
        aBB.MakeCompound(ref myTopBottom);
        double anAngTol = Math.Sqrt(aSqAnguarTol);
        TopoDS_Shape aCompWS = aCompW, aCompFS = aCompF;
        BOPAlgo_Tools.EdgesToWires(aFreeEdges, ref aCompWS, true, anAngTol);
        BOPAlgo_Tools.WiresToFaces(aCompWS, ref aCompFS, anAngTol);

        {
            // Check orientation

            NCollection_IndexedMap<TopoDS_Shape> aMapV = new(TopTools_ShapeMapHasher.Instance);
            TopExp.MapShapes(myPipeShell, TopAbs_VERTEX, aMapV);
            TopExp_Explorer anExp = new(aCompFS, TopAbs_FACE);
            for (; anExp.More(); anExp.Next())
            {
                TopoDS_Face aF = TopoDS.Face(anExp.Current());
                // OCCT dereferences the down-cast plane without a check.
                Geom_Plane aPln = BRep_Tool.Surface(aF) as Geom_Plane ?? throw new NullReferenceException();
                Xyz aNorm = aPln.Position().Direction().XYZ();
                Xyz aLocP = aPln.Position().Location().XYZ();

                bool isFound = false;

                for (int i = 1; i <= aMapV.Extent(); i++)
                {
                    TopoDS_Vertex aV = TopoDS.Vertex(aMapV.FindKey(i));
                    Xyz aP = BRep_Tool.Pnt(aV).XYZ();

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

        NCollection_Map<TopoDS_Shape> aMapF = new(TopTools_ShapeMapHasher.Instance);
        NCollection_List<TopoDS_Shape> aLF = new(), aLSplits = new();
        TopExp_Explorer anExpF = new();

        for (anExpF.Init(myPipeShell, TopAbs_FACE); anExpF.More(); anExpF.Next())
        {
            TopoDS_Face aF = TopoDS.Face(anExpF.Current());
            if (!aMapF.Add(aF))
            {
                continue;
            }

            ReduceVertexTolerance(aF);
            CheckSingularityAndAdd(aF, myFuzzyValue, aLF, aLSplits);
        }

        {
            NCollection_List<TopoDS_Shape>.Iterator anItrS = new(aLSplits);
            for (; anItrS.More(); anItrS.Next())
            {
                TopoDS_Face aF = TopoDS.Face(anItrS.Value());
                aLF.Append(aF);
            }

            // Split interfered faces
            PerformBoolean(aLF, ref myPipeShell);
        }

        aLF.Clear();
        aMapF.Clear();
        for (anExpF.Init(myPipeShell, TopAbs_FACE); anExpF.More(); anExpF.Next())
        {
            TopoDS_Face aF = TopoDS.Face(anExpF.Current());
            if (!aMapF.Add(aF))
            {
                continue;
            }

            aLF.Append(aF);
        }

        if (!myTopBottom.IsNull())
        {
            TopoDS_Iterator anItLids = new(myTopBottom);
            for (; anItLids.More(); anItLids.Next())
            {
                TopoDS_Face aF = TopoDS.Face(anItLids.Value());
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

    private void ExtractOuterSolid(ref TopoDS_Shape theShape, NCollection_List<TopoDS_Shape> theArgsList)
    {
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMapS = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(theShape, TopAbs_FACE, TopAbs_SOLID, aMapS);

        // theArgsList.Clear();
        NCollection_List<TopoDS_Shape> aNewList = new();
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
            NCollection_List<TopoDS_Shape>.Iterator anItrF = new();

            bool isRemoved = true;
            while (isRemoved)
            {
                isRemoved = false;
                for (anItrF.Initialize(theArgsList); anItrF.More(); anItrF.Next())
                {
                    TopoDS_Face aF = TopoDS.Face(anItrF.Value());
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

    private void RemoveExcessSolids(NCollection_List<TopoDS_Shape> theLSplits,
                                    in TopoDS_Shape theShape,
                                    NCollection_List<TopoDS_Shape> theArgsList,
                                    BOPAlgo_MakerVolume theMV)
    {
        if (myErrorStatus != BRepFill_AdvancedEvolved_NotVolume)
        {
            return;
        }

        TopoDS_Shape aResShape = theShape;

        TopExp_Explorer anExpSo = new();
        for (int i = 0; i < 2; i++)
        {
            anExpSo.Init(aResShape, TopAbs_SOLID);
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

        NCollection_List<TopoDS_Shape> aSolidList = new();

        // Look for all solids containing lids
        {
            anExpSo.Init(aResShape, TopAbs_SOLID);
            for (; anExpSo.More(); anExpSo.Next())
            {
                TopoDS_Solid aSol = TopoDS.Solid(anExpSo.Current());
                NCollection_IndexedMap<TopoDS_Shape> aMapF = new(TopTools_ShapeMapHasher.Instance);
                TopExp.MapShapes(aSol, aMapF);

                bool areThereLids = false;
                TopExp_Explorer anExpLids = new(myTopBottom, TopAbs_FACE);
                for (; anExpLids.More(); anExpLids.Next())
                {
                    areThereLids = true;
                    TopoDS_Face aFLid = TopoDS.Face(anExpLids.Current());
                    int aFIdx = aMapF.FindIndex(aFLid);
                    if (aFIdx < 1)
                    {
                        continue;
                    }

                    TopoDS_Face aFSol = TopoDS.Face(aMapF.FindKey(aFIdx));

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
                BRep_Builder aBB = new();
                TopoDS_CompSolid aCompSol = new();
                aBB.MakeCompSolid(ref aCompSol);
                NCollection_List<TopoDS_Shape>.Iterator anItl = new(aSolidList);
                for (; anItl.More(); anItl.Next())
                {
                    TopoDS_Solid aSol = TopoDS.Solid(anItl.Value());
                    aBB.Add(aCompSol, aSol);
                }

                aResShape = aCompSol;
                aSolidList.Clear();
            }
        }

        {
            // Remove Split faces from the list of arguments
            NCollection_List<TopoDS_Shape>.Iterator anItl = new(theLSplits);
            for (; anItl.More(); anItl.Next())
            {
                TopoDS_Face aF = TopoDS.Face(anItl.Value());
                theArgsList.Remove(aF);
            }

            // Create a list of invalid faces. The face is invalid if
            // BOPAlgo_MakerVolume changes its orientation while creating solids.
            // Faces from theLSplits are not checked.
            NCollection_List<TopoDS_Shape> aListInvFaces = new();
            for (anItl.Initialize(theArgsList); anItl.More(); anItl.Next())
            {
                TopoDS_Face aF = TopoDS.Face(anItl.Value());
                foreach (TopoDS_Shape aMod in theMV.Modified(aF))
                {
                    TopoDS_Face aFM = TopoDS.Face(aMod);

                    if (aFM.Orientation() != aF.Orientation())
                    {
                        aListInvFaces.Append(aFM);
                    }
                }
            }

            for (anExpSo.Init(aResShape, TopAbs_SOLID); anExpSo.More(); anExpSo.Next())
            {
                TopoDS_Solid aSo = TopoDS.Solid(anExpSo.Current());
                NCollection_IndexedMap<TopoDS_Shape> aMapF = new(TopTools_ShapeMapHasher.Instance);
                TopExp.MapShapes(aSo, TopAbs_FACE, aMapF);
                bool isToDelete = false;

                for (anItl.Initialize(aListInvFaces); anItl.More(); anItl.Next())
                {
                    TopoDS_Face aF = TopoDS.Face(anItl.Value());
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
                    TopoDS_Face aF = TopoDS.Face(anItl.Value());
                    int anIdx = aMapF.FindIndex(aF);
                    if (anIdx == 0)
                    {
                        continue;
                    }

                    TopoDS_Face aF1 = TopoDS.Face(aMapF.FindKey(anIdx));

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
            BRep_Builder aBB = new();
            TopoDS_CompSolid aCmpSol = new();
            aBB.MakeCompSolid(ref aCmpSol);

            for (NCollection_List<TopoDS_Shape>.Iterator anItl = new(aSolidList); anItl.More(); anItl.Next())
            {
                TopoDS_Solid aSo = TopoDS.Solid(anItl.Value());
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
    private static void RebuildFaces(NCollection_List<TopoDS_Shape> theLE,
                                     in TopoDS_Face theSourceFace,
                                     NCollection_List<TopoDS_Shape> theList)
    {
        // build new faces
        BOPAlgo_BuilderFace aBF = new();

        TopoDS_Face aF = TopoDS.Face(theSourceFace.Oriented(TopAbs_FORWARD));

        aBF.SetFace(aF);
        aBF.SetShapes(theLE);

        aBF.Perform();

        NCollection_List<TopoDS_Shape> aLFR = aBF.Areas();

        if (aLFR.IsEmpty())
        {
            theList.Append(theSourceFace);
            return;
        }

        NCollection_List<TopoDS_Shape>.Iterator aItFR = new(aLFR);
        for (; aItFR.More(); aItFR.Next())
        {
            TopoDS_Shape aFR = TopoDS.Face(aItFR.Value());
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
    private static bool MakeEdgeDegenerated(in TopoDS_Vertex theV,
                                            in TopoDS_Face theFace,
                                            in Point2D thePf,
                                            in Point2D thePl,
                                            NCollection_List<TopoDS_Shape> theLEdges)
    {
        BRepAdaptor_Surface anAS = new(theFace, false);

        double aTol = 2.0 * BRep_Tool.Tolerance(theV);
        double aTolU = anAS.UResolution(aTol), aTolV = anAS.VResolution(aTol);

        if ((Math.Abs(thePf.X() - thePl.X()) < aTolU) && (Math.Abs(thePf.Y() - thePl.Y()) < aTolV))
        {
            return false;
        }

        TopoDS_Vertex aVf = TopoDS.Vertex(theV.Oriented(TopAbs_FORWARD)),
                      aVl = TopoDS.Vertex(theV.Oriented(TopAbs_REVERSED));

        Xy aV = thePl.XY() - thePf.XY();
        Geom2d_Line aL1 = new(thePf, new Direction2D(aV));
        Geom2d_Line aL2 = new(thePl, new Direction2D(aV.Reversed()));

        BRep_Builder aBB = new();
        TopoDS_Edge anEdegen1 = new(), anEdegen2 = new();
        aBB.MakeEdge(ref anEdegen1);
        aBB.MakeEdge(ref anEdegen2);

        aBB.UpdateEdge(anEdegen1, aL1, theFace, global::Kerf.Engine.Foundation.Precision.Confusion());
        aBB.UpdateEdge(anEdegen2, aL2, theFace, global::Kerf.Engine.Foundation.Precision.Confusion());

        anEdegen1.Orientation(TopAbs_FORWARD);
        anEdegen2.Orientation(TopAbs_FORWARD);

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

    private static void InsertEDegenerated(in TopoDS_Face theFace, NCollection_List<TopoDS_Shape> theLEdges)
    {
        BRep_Builder aBB = new();
        TopoDS_Wire aWir = new();
        aBB.MakeWire(ref aWir);

        NCollection_List<TopoDS_Shape>.Iterator anItr = new(theLEdges);
        for (; anItr.More(); anItr.Next())
        {
            TopoDS_Edge anE = TopoDS.Edge(anItr.Value());
            aBB.Add(aWir, anE);
        }

        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMapVE = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndUniqueAncestors(aWir, TopAbs_VERTEX, TopAbs_EDGE, aMapVE);

        BRepTools_WireExplorer anExp = new(aWir, theFace);

        TopoDS_Edge anE1 = anExp.Current(), aFirstEdge, aLastEdge = new();

        if (anE1.IsNull())
        {
            // It is possible if aWir contains
            // only INTERNAL/EXTERNAL edges.

            return;
        }

        aFirstEdge = anE1;
        anExp.Next();

        // Map containing all vertices of degenerated edges
        NCollection_Map<TopoDS_Shape> aMapVofDE = new(TopTools_ShapeMapHasher.Instance);

        {
            TopExp_Explorer anExpDE = new(aWir, TopAbs_EDGE);
            for (; anExpDE.More(); anExpDE.Next())
            {
                TopoDS_Edge anE = TopoDS.Edge(anExpDE.Current());
                if (!BRep_Tool.Degenerated(anE))
                {
                    continue;
                }

                TopExp.Vertices(anE, out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);

                // aV1 and aV2 are SAME vertices

                aMapVofDE.Add(aV1);
            }
        }

        for (; anExp.More(); anExp.Next())
        {
            TopoDS_Edge anE2 = anExp.Current();
            aLastEdge = anE2;

            TopoDS_Vertex aVertCurr = anExp.CurrentVertex();

            if (aMapVofDE.Contains(aVertCurr))
            {
                // Necessary degenerated edge has already been created.
                anE1 = anE2;
                continue;
            }

            Geom2d_Curve aC1 = BRep_Tool.CurveOnSurface(anE1, theFace, out double aF, out double aL)!,
                         aC2 = BRep_Tool.CurveOnSurface(anE2, theFace, out aF, out aL)!;
            aF = BRep_Tool.Parameter(aVertCurr, anE1);
            aL = BRep_Tool.Parameter(aVertCurr, anE2);
            Point2D aPf = aC1.Value(aF), aPl = aC2.Value(aL);

            if (MakeEdgeDegenerated(aVertCurr, theFace, aPf, aPl, theLEdges))
            {
                aMapVofDE.Add(aVertCurr);
                anE1 = anE2;
                continue;
            }

            bool aFound = aMapVE.FindFromKey(aVertCurr, out NCollection_List<TopoDS_Shape> anEList);
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
                TopoDS_Edge anEdge = TopoDS.Edge(anItr.Value());
                if (BRep_Tool.Degenerated(anEdge))
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
                    TopoDS_Edge anEdge = TopoDS.Edge(anItr.Value());

                    if (anEdge.IsSame(anE1) || anEdge.IsSame(anE2))
                    {
                        continue;
                    }

                    Geom2d_Curve aC = BRep_Tool.CurveOnSurface(anEdge, theFace, out aF, out aL)!;
                    aF = BRep_Tool.Parameter(aVertCurr, anEdge);
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

        if (aFirstEdge.IsNull() || aLastEdge.IsNull())
        {
            return;
        }

        // TopExp::CommonVertex(...) does not work
        // if edges have more than one pair of common vertex
        //(e.g. two halves of circle). Here, we process this case.
        TopoDS_Vertex[] aV = new TopoDS_Vertex[4];
        TopExp.Vertices(aFirstEdge, out aV[0], out aV[1]);
        if (!aV[0].IsNull() && aV[0].IsSame(aV[1]))
        {
            // Possible reason is the NOT-CLOSED edge
            // has only single vertex and is covered by it.
            return;
        }

        TopExp.Vertices(aLastEdge, out aV[2], out aV[3]);
        if (!aV[2].IsNull() && aV[2].IsSame(aV[3]))
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

                if (aMapVE.FindFromKey(aV[anIDFE], out NCollection_List<TopoDS_Shape> anEList) && (anEList.Extent() > 2))
                {
                    // Causes:
                    //  1. Non-manifold topology.
                    //  2. Case such as: two edges meeting a seam at the same vertex, which must not be joined.
                    continue;
                }

                Geom2d_Curve aC1 = BRep_Tool.CurveOnSurface(aFirstEdge, theFace, out double aF, out double aL)!,
                             aC2 = BRep_Tool.CurveOnSurface(aLastEdge, theFace, out aF, out aL)!;
                aF = BRep_Tool.Parameter(aV[anIDFE], aFirstEdge);
                aL = BRep_Tool.Parameter(aV[anIDLE], aLastEdge);
                Point2D aPf = aC1.Value(aF), aPl = aC2.Value(aL);

                MakeEdgeDegenerated(aV[anIDFE], theFace, aPf, aPl, theLEdges);
            }
        }
    }

    //=======================================================================
    // function : CheckSingularityAndAdd
    // purpose  : Returns TRUE if theF has been split
    //=======================================================================
    private bool CheckSingularityAndAdd(in TopoDS_Face theF,
                                        double theFuzzyToler,
                                        NCollection_List<TopoDS_Shape> theListOfFaces,
                                        NCollection_List<TopoDS_Shape> theListOfSplits)
    {
        BRepAdaptor_Surface anAS = new(theF, false);
        SurfaceKind aSType = anAS.GetType();

        if (aSType == OffsetSurface)
        {
            aSType = anAS.BasisSurface()!.GetType();
        }

        if (aSType == Plane)
        {
            NCollection_Map<TopoDS_Shape> aME = new(TopTools_ShapeMapHasher.Instance);
            NCollection_List<TopoDS_Shape> aLE = new();
            TopExp_Explorer anExp = new(theF, TopAbs_EDGE);
            for (; anExp.More(); anExp.Next())
            {
                TopoDS_Edge anE = TopoDS.Edge(anExp.Current());

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

            TopoDS_Shape anEdges = aBuilder.InternalShape();

            BRep_Builder aBB = new();
            TopoDS_Compound aCompW = new(), aCompF = new();
            aBB.MakeCompound(ref aCompW);
            aBB.MakeCompound(ref aCompF);
            TopoDS_Shape aCompWS = aCompW, aCompFS = aCompF;
            BOPAlgo_Tools.EdgesToWires(anEdges, ref aCompWS, true);
            BOPAlgo_Tools.WiresToFaces(aCompWS, ref aCompFS);

            aME.Clear();
            anExp.Init(aCompFS, TopAbs_FACE);
            for (; anExp.More(); anExp.Next())
            {
                TopoDS_Face aF = TopoDS.Face(anExp.Current());
                theListOfSplits.Append(aF);
            }

            return true;
        }

        if ((aSType != SurfaceKind.Cone) && (aSType != SurfaceKind.Sphere) && (aSType != BezierSurface)
            && (aSType != BSplineSurface) && (aSType != SurfaceOfRevolution))
        {
            theListOfFaces.Append(theF);
            return false;
        }

        BRep_Builder aBB2 = new();

        TopoDS_Compound aCWires = new();
        aBB2.MakeCompound(ref aCWires);

        bool isSplit = false;
        NCollection_List<TopoDS_Shape> aListEdges = new();

        TopoDS_Face aFace = TopoDS.Face(theF.Oriented(TopAbs_FORWARD));

        for (TopoDS_Iterator anExpW = new(aFace); anExpW.More(); anExpW.Next())
        {
            TopoDS_Wire aWir = TopoDS.Wire(anExpW.Value());

            NCollection_List<TopoDS_Shape> aLGF = new();
            TopExp_Explorer anEExp = new(aWir, TopAbs_EDGE);
            for (; anEExp.More(); anEExp.Next())
            {
                TopoDS_Edge anE = TopoDS.Edge(anEExp.Current());
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

            NCollection_List<TopoDS_Shape> aLE = new();
            NCollection_List<TopoDS_Shape>.Iterator aBItr = new(aLGF);
            for (; aBItr.More(); aBItr.Next())
            {
                TopoDS_Edge aSh = TopoDS.Edge(aBItr.Value());
                var aLM = aBuilder.Modified(aSh);
                if (aLM.Count == 0 || BRep_Tool.Degenerated(aSh))
                {
                    aLE.Append(aSh);
                    continue;
                }

                foreach (TopoDS_Shape aMod in aLM)
                {
                    TopoDS_Edge anEM = TopoDS.Edge(aMod);
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

        NCollection_List<TopoDS_Shape>.Iterator anItrS = new(theListOfSplits);
        for (; anItrS.More(); anItrS.Next())
        {
            TopoDS_Face aF = TopoDS.Face(anItrS.Value());
            theListOfFaces.Append(aF.Oriented(theF.Orientation()));
        }

        return true;
    }

    //=================================================================================================

    private static bool ContainsInList(NCollection_List<TopoDS_Shape> theL, in TopoDS_Shape theObject)
    {
        NCollection_List<TopoDS_Shape>.Iterator anIt = new(theL);
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
    private static void FindInternals(in TopoDS_Shape theS, NCollection_List<TopoDS_Shape> theLInt)
    {
        TopoDS_Iterator itS = new(theS);
        for (; itS.More(); itS.Next())
        {
            TopoDS_Shape aSS = itS.Value();
            if (aSS.Orientation() == TopAbs_INTERNAL)
            {
                theLInt.Append(aSS);
            }
            else
            {
                TopoDS_Iterator itSS = new(aSS);
                for (; itSS.More(); itSS.Next())
                {
                    if (itSS.Value().Orientation() == TopAbs_INTERNAL)
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
    private static void RemoveInternalWires(in TopoDS_Shape theShape)
    {
        TopExp_Explorer anExpF = new(theShape, TopAbs_FACE);
        for (; anExpF.More(); anExpF.Next())
        {
            TopoDS_Shape aF = anExpF.Current();
            NCollection_List<TopoDS_Shape> aLWToRemove = new();
            FindInternals(aF, aLWToRemove);
            if (aLWToRemove.Extent() != 0)
            {
                aF.Free(true);
                NCollection_List<TopoDS_Shape>.Iterator itR = new(aLWToRemove);
                for (; itR.More(); itR.Next())
                {
                    new BRep_Builder().Remove(aF, itR.Value());
                }
                aF.Free(false);
            }
        }
    }

    //=================================================================================================

    private static void ProcessVertex(in TopoDS_Vertex aV, NCollection_List<TopoDS_Shape> aLE, NCollection_List<TopoDS_Shape> aLF)
    {
        double aTol, aD2, aTolMax2, aTolE, aParam;
        Point3D aPC3D;
        Point2D aPC2D;
        TopAbs_Orientation anOrV;

        NCollection_List<TopoDS_Shape>.Iterator anIt = new();
        TopExp_Explorer aVExp = new();

        //
        aTolMax2 = -1.0e6;
        //
        BRep_TVertex TV = (BRep_TVertex)aV.TShape()!;
        Point3D aPV3D = TV.Pnt();
        aTol = BRep_Tool.Tolerance(aV);
        //
        anIt.Initialize(aLE);
        for (; anIt.More(); anIt.Next())
        {
            TopoDS_Edge aE = TopoDS.Edge(anIt.Value());
            //
            BRep_TEdge TE = (BRep_TEdge)aE.TShape()!;
            Placement Eloc = aE.Location();
            //
            aVExp.Init(aE, TopAbs_VERTEX);
            for (; aVExp.More(); aVExp.Next())
            {
                TopoDS_Vertex aVx = TopoDS.Vertex(aVExp.Current());
                //
                if (!aVx.IsSame(aV))
                {
                    continue;
                }
                //
                anOrV = aVx.Orientation();
                if (anOrV != TopAbs_FORWARD && anOrV != TopAbs_REVERSED)
                {
                    continue;
                }
                //
                NCollection_List<BRep_CurveRepresentation> aLCR = TE.Curves();
                NCollection_List<BRep_CurveRepresentation>.Iterator itcr = new(aLCR);
                for (; itcr.More(); itcr.Next())
                {
                    BRep_CurveRepresentation cr = itcr.Value();
                    Placement loc = cr.Location();
                    Placement L = (Eloc * loc).Predivided(aV.Location());
                    //
                    // 3D-Curve
                    if (cr.IsCurve3D())
                    {
                        Geom_Curve? aC3D = cr.Curve3D();
                        //
                        if (aC3D is null)
                        {
                            continue;
                        }
                        // 3D-point treatment
                        aParam = BRep_Tool.Parameter(aVx, aE);
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
                        Geom2d_Curve? aC2D = cr.PCurve();
                        if (aC2D is null)
                        {
                            continue;
                        }
                        // Surface
                        Geom_Surface aS = cr.Surface()!;
                        //
                        // 2D-point treatment
                        aParam = BRep_Tool.Parameter(aVx, aE, aS, L);
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
            TopoDS_Edge aE = TopoDS.Edge(anIt.Value());

            aTolE = BRep_Tool.Tolerance(aE);
            if (aTolMax2 < aTolE)
            {
                aTolMax2 = aTolE;
            }
        }
        //
        anIt.Initialize(aLF);
        for (; anIt.More(); anIt.Next())
        {
            TopoDS_Face aF = TopoDS.Face(anIt.Value());

            aTolE = BRep_Tool.Tolerance(aF);
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

    private static void ReduceVertexTolerance(in TopoDS_Shape aS)
    {
        int i, aNbV;
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aVEMap = new(TopTools_ShapeMapHasher.Instance),
            aVFMap = new(TopTools_ShapeMapHasher.Instance);

        TopExp.MapShapesAndUniqueAncestors(aS, TopAbs_VERTEX, TopAbs_EDGE, aVEMap);
        TopExp.MapShapesAndUniqueAncestors(aS, TopAbs_VERTEX, TopAbs_FACE, aVFMap);

        aNbV = aVEMap.Extent();
        for (i = 1; i <= aNbV; i++)
        {
            TopoDS_Vertex aV = TopoDS.Vertex(aVEMap.FindKey(i));
            NCollection_List<TopoDS_Shape> aLE = aVEMap[i];
            NCollection_List<TopoDS_Shape> aLF = aVFMap.FindFromKey(aV);

            ProcessVertex(aV, aLE, aLF);
        }
    }
}
