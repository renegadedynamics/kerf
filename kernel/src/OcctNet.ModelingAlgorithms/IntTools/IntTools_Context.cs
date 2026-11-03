// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_Context.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_Context.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Extrema.ExtFlag;
using static Kerf.Geometry.CurveKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Occt;

/// <summary>
/// The intersection context: caches of the tools the Boolean component builds per shape (point classifiers,
/// projectors, solid classifiers, hatchers, surface adaptors, boxes), so that each is built once per algorithm run.
/// </summary>
/// <remarks>
/// <para>
/// A working object, not a value (plan section 6). Each thread of a parallel loop has its own context
/// (<see cref="BOPTools_Parallel"/>, decision 580), so a context is used by one thread at a time and needs no lock.
/// The tools it hands out are its own cached objects, as OCCT's references are: callers run them
/// (<c>ProjPS(F).Perform(P)</c>) and read their results before the next call on the same context.
/// </para>
/// <para>
/// The cache audit (plan section 2.4, decision 714): every cached tool but one gives the same answers whatever the
/// context computed before. The exception is OCCT's <see cref="SurfaceData(in Face)"/>, whose boxes
/// IntTools_BeanFaceIntersector keys by sample index and enlarges by the first edge's tolerance (occt-issues.md, unit
/// 6C). The port's intersector uses the internal keyed overload instead, so that it gives every run the result of a
/// fresh context (decision 753).
/// </para>
/// <para>
/// The maps are keyed by shape with <see cref="ShapeHasher"/> (orientation ignored), and by curve with
/// reference equality (OCCT's handle hasher). The allocator goes (translation rules, section 9).
/// </para>
/// </remarks>
public class IntTools_Context : Transient
{
    private readonly NCollection_DataMap<Shape, IntTools_FClass2d> myFClass2dMap = new(ShapeHasher.Instance);
    private readonly NCollection_DataMap<Shape, GeomAPI_ProjectPointOnSurf> myProjPSMap = new(ShapeHasher.Instance);
    private readonly NCollection_DataMap<Shape, GeomAPI_ProjectPointOnCurve> myProjPCMap = new(ShapeHasher.Instance);
    private readonly NCollection_DataMap<Shape, BRepClass3d_SolidClassifier> mySClassMap = new(ShapeHasher.Instance);
    private readonly NCollection_DataMap<Curve3D, GeomAPI_ProjectPointOnCurve> myProjPTMap = new(ReferenceEqualityComparer.Instance);
    private readonly NCollection_DataMap<Shape, Geom2dHatch_Hatcher> myHatcherMap = new(ShapeHasher.Instance);
    private readonly NCollection_DataMap<Shape, IntTools_SurfaceRangeLocalizeData> myProjSDataMap = new(ShapeHasher.Instance);
    // OCCT.NET (decision 753): the localisation data keyed by face and by what the intersector's boxes depend on
    private readonly NCollection_DataMap<Shape, Dictionary<SurfaceDataKey, IntTools_SurfaceRangeLocalizeData>> mySDataKeyedMap = new(ShapeHasher.Instance);
    private readonly NCollection_DataMap<Shape, Box> myBndBoxDataMap = new(ShapeHasher.Instance);
    private readonly NCollection_DataMap<Shape, FaceSurfaceAdaptor> mySurfAdaptorMap = new(ShapeHasher.Instance);
    private readonly NCollection_DataMap<Shape, OrientedBox> myOBBMap = new(ShapeHasher.Instance);
    private int myCreateFlag;
    private double myPOnSTolerance;

    /// <summary>An empty context.</summary>
    public IntTools_Context()
    {
        myCreateFlag = 0;
        myPOnSTolerance = 1.0e-12;
    }

    /// <summary>
    /// The bounding box of the shape (<see cref="BRepBndLib.Add"/>), computed on the first call. OCCT returns a
    /// reference into its cache; the port returns the box by value.
    /// </summary>
    public Box BndBox(in Shape aS)
    {
        if (!myBndBoxDataMap.Find(aS, out Box aBox))
        {
            aBox = new Box();
            BRepBndLib.Add(aS, ref aBox);
            myBndBoxDataMap.Bind(aS, aBox);
        }
        return aBox;
    }

    /// <summary>True if the face's bounding box is open in some direction (an infinite face).</summary>
    public bool IsInfiniteFace(in Face aFace)
    {
        Box aBox = BndBox(aFace);
        return aBox.IsOpenXmax || aBox.IsOpenXmin || aBox.IsOpenYmax || aBox.IsOpenYmin
               || aBox.IsOpenZmax || aBox.IsOpenZmin;
    }

    /// <summary>The 2D point classifier of the face (taken FORWARD, with the face's tolerance), built on the first call.</summary>
    public IntTools_FClass2d FClass2d(in Face aF)
    {
        if (!myFClass2dMap.Find(aF, out IntTools_FClass2d? pFClass2d))
        {
            double aTolF;
            Face aFF;
            //
            aFF = aF;
            aFF.Orientation(Forward);
            aTolF = Tool.Tolerance(aFF);
            //
            pFClass2d = new IntTools_FClass2d(aFF, aTolF);
            //
            myFClass2dMap.Bind(aFF, pFClass2d);
        }
        return pFClass2d!;
    }

    /// <summary>
    /// The point projector of the face's surface, bounded by the face's UV bounds, with the context's tolerance and
    /// the minimum-only flag, built on the first call.
    /// </summary>
    public GeomAPI_ProjectPointOnSurf ProjPS(in Face aF)
    {
        if (!myProjPSMap.Find(aF, out GeomAPI_ProjectPointOnSurf? pProjPS))
        {
            UVBounds(aF, out double Umin, out double Usup, out double Vmin, out double Vsup);
            Surface aS = Tool.Surface(aF)!;
            //
            pProjPS = new GeomAPI_ProjectPointOnSurf();
            pProjPS.Init(aS, Umin, Usup, Vmin, Vsup, myPOnSTolerance);
            pProjPS.SetExtremaFlag(ExtFlag_MIN);
            //
            myProjPSMap.Bind(aF, pProjPS);
        }
        return pProjPS!;
    }

    /// <summary>The point projector of the edge's 3D curve over the edge's range, built on the first call.</summary>
    public GeomAPI_ProjectPointOnCurve ProjPC(in Edge aE)
    {
        if (!myProjPCMap.Find(aE, out GeomAPI_ProjectPointOnCurve? pProjPC))
        {
            Curve3D? aC3D = Tool.Curve(aE, out double f, out double l);
            //
            pProjPC = new GeomAPI_ProjectPointOnCurve();
            pProjPC.Init(aC3D!, f, l);
            //
            myProjPCMap.Bind(aE, pProjPC);
        }
        return pProjPC!;
    }

    /// <summary>The point projector of the curve over its own range, built on the first call (keyed by the curve object).</summary>
    public GeomAPI_ProjectPointOnCurve ProjPT(Curve3D aC3D)
    {
        if (!myProjPTMap.Find(aC3D, out GeomAPI_ProjectPointOnCurve? pProjPT))
        {
            double f, l;
            f = aC3D.FirstParameter();
            l = aC3D.LastParameter();
            //
            pProjPT = new GeomAPI_ProjectPointOnCurve();
            pProjPT.Init(aC3D, f, l);
            //
            myProjPTMap.Bind(aC3D, pProjPT);
        }
        return pProjPT!;
    }

    /// <summary>The 3D point classifier of the solid, built on the first call.</summary>
    public BRepClass3d_SolidClassifier SolidClassifier(in Solid aSolid)
    {
        if (!mySClassMap.Find(aSolid, out BRepClass3d_SolidClassifier? pSC))
        {
            pSC = new BRepClass3d_SolidClassifier(aSolid);
            //
            mySClassMap.Bind(aSolid, pSC);
        }
        return pSC!;
    }

    /// <summary>The surface adaptor of the face (restricted to the face's UV bounds), built on the first call.</summary>
    public FaceSurfaceAdaptor SurfaceAdaptor(in Face theFace)
    {
        if (!mySurfAdaptorMap.Find(theFace, out FaceSurfaceAdaptor? pBAS))
        {
            //
            pBAS = new FaceSurfaceAdaptor(theFace, true);
            //
            mySurfAdaptorMap.Bind(theFace, pBAS);
        }
        return pBAS!;
    }

    /// <summary>
    /// The 2D hatcher of the face (taken FORWARD), loaded with the face's pcurves trimmed to their edges' ranges, built
    /// on the first call.
    /// </summary>
    public Geom2dHatch_Hatcher Hatcher(in Face aF)
    {
        if (!myHatcherMap.Find(aF, out Geom2dHatch_Hatcher? pHatcher))
        {
            double aTolArcIntr, aTolTangfIntr, aTolHatch2D, aTolHatch3D;
            double aEpsT;
            Orientation aOrE;
            Curve2D? aC2D;
            TrimmedCurve2D aCT2D;
            Face aFF;
            Explorer aExp = new();
            //
            aTolHatch2D = 1.0e-8;
            aTolHatch3D = 1.0e-8;
            aTolArcIntr = 1.0e-10;
            aTolTangfIntr = 1.0e-10;
            aEpsT = global::Kerf.Engine.Foundation.Precision.PConfusion();
            //
            Geom2dHatch_Intersector aIntr = new(aTolArcIntr, aTolTangfIntr);
            pHatcher = new Geom2dHatch_Hatcher(aIntr, aTolHatch2D, aTolHatch3D, true, false);
            //
            aFF = aF;
            aFF.Orientation(Forward);
            _ = Tool.Surface(aFF);

            aExp.Init(aFF, ShapeType.Edge);
            for (; aExp.More(); aExp.Next())
            {
                Edge aE = ShapeCasts.Edge(aExp.Current());
                aOrE = aE.Orientation();
                //
                aC2D = Tool.CurveOnSurface(aE, aFF, out double aU1, out double aU2);
                if (aC2D is null)
                {
                    continue;
                }
                if (Math.Abs(aU1 - aU2) < aEpsT)
                {
                    continue;
                }
                //
                aCT2D = new TrimmedCurve2D(aC2D, aU1, aU2);
                CurveAdaptor2D aGAC = new(aCT2D);
                pHatcher.AddElement(aGAC, aOrE);
            } // for (; aExp.More() ; aExp.Next()) {
            //
            myHatcherMap.Bind(aFF, pHatcher);
        }
        return pHatcher!;
    }

    /// <summary>
    /// The oriented bounding box of the shape (<see cref="BRepBndLib.AddOBB"/>) enlarged by the gap (the fuzzy value,
    /// <see cref="Precision.Confusion"/> by default), computed on the first call. As in OCCT, a box cached with one gap
    /// is returned for any later gap.
    /// </summary>
    public OrientedBox OBB(in Shape aS) => OBB(aS, global::Kerf.Engine.Foundation.Precision.Confusion());

    /// <inheritdoc cref="OBB(in Shape)"/>
    public OrientedBox OBB(in Shape aS, double theGap)
    {
        if (!myOBBMap.Find(aS, out OrientedBox aBox))
        {
            aBox = new OrientedBox();
            BRepBndLib.AddOBB(aS, ref aBox);
            aBox.Enlarge(theGap);
            myOBBMap.Bind(aS, aBox);
        }
        return aBox;
    }

    /// <summary>
    /// The surface localization data of the face (3 by 3 samples, minimal ranges 10 PConfusion), made on the first
    /// call: the cached object itself, which IntTools_BeanFaceIntersector fills and reads.
    /// </summary>
    public IntTools_SurfaceRangeLocalizeData SurfaceData(in Face aF)
    {
        if (!myProjSDataMap.Find(aF, out IntTools_SurfaceRangeLocalizeData? pSData))
        {
            pSData = new IntTools_SurfaceRangeLocalizeData(3,
                                                           3,
                                                           10.0 * global::Kerf.Engine.Foundation.Precision.PConfusion(),
                                                           10.0 * global::Kerf.Engine.Foundation.Precision.PConfusion());
            //
            myProjSDataMap.Bind(aF, pSData);
        }
        return pSData!;
    }

    /// <summary>
    /// OCCT.NET (decision 753): the surface localisation data of the face for one set of box parameters (the
    /// intersector's criteria, the face's tolerance and the surface's parameter ranges), made as
    /// <see cref="SurfaceData(in Face)"/> on the first call with these values. The boxes
    /// IntTools_BeanFaceIntersector caches in it are then the boxes it would compute, whichever edge computed them.
    /// </summary>
    internal IntTools_SurfaceRangeLocalizeData SurfaceData(in Face aF,
                                                           double theCriteria,
                                                           double theFaceTolerance,
                                                           double theUMin,
                                                           double theUMax,
                                                           double theVMin,
                                                           double theVMax)
    {
        if (!mySDataKeyedMap.Find(aF, out Dictionary<SurfaceDataKey, IntTools_SurfaceRangeLocalizeData>? aMap))
        {
            aMap = [];
            mySDataKeyedMap.Bind(aF, aMap);
        }
        SurfaceDataKey aKey = new(theCriteria, theFaceTolerance, theUMin, theUMax, theVMin, theVMax);
        if (!aMap!.TryGetValue(aKey, out IntTools_SurfaceRangeLocalizeData? pSData))
        {
            pSData = new IntTools_SurfaceRangeLocalizeData(3,
                                                           3,
                                                           10.0 * global::Kerf.Engine.Foundation.Precision.PConfusion(),
                                                           10.0 * global::Kerf.Engine.Foundation.Precision.PConfusion());
            aMap.Add(aKey, pSData);
        }
        return pSData;
    }

    // The key of the localisation data: doubles compared by double.Equals, as the options structs of decision 640.
    private readonly record struct SurfaceDataKey(double Criteria, double FaceTolerance, double UMin, double UMax,
                                                  double VMin, double VMax);

    /// <summary>
    /// Computes the parameter of the point on the edge. Returns 0 if the distance is within the sum of the point's
    /// and the edge's tolerances (or the point is near a vertex when the projection fails), -2 if the edge has no
    /// geometry, -3 if the projection fails and no vertex is near, -4 if the point is too far.
    /// </summary>
    public int ComputePE(in Point3D aP1, double aTolP1, in Edge aE2, ref double aT, ref double aDist)
    {
        if (!Tool.IsGeometric(aE2))
        {
            return -2;
        }
        double aTolE2, aTolSum;
        int aNbProj;
        //
        GeomAPI_ProjectPointOnCurve aProjector = ProjPC(aE2);
        aProjector.Perform(aP1);

        aNbProj = aProjector.NbPoints();
        if (aNbProj != 0)
        {
            // point falls on the curve
            aDist = aProjector.LowerDistance();
            //
            aTolE2 = Tool.Tolerance(aE2);
            aTolSum = aTolP1 + aTolE2 + global::Kerf.Engine.Foundation.Precision.Confusion();
            //
            aT = aProjector.LowerDistanceParameter();
            if (aDist > aTolSum)
            {
                return -4;
            }
        }
        else
        {
            // point falls out of the curve, check distance to vertices
            Edge aEFwd = ShapeCasts.Edge(aE2.Oriented(Forward));
            Iterator itV = new(aEFwd);
            aDist = RealLast();
            for (; itV.More(); itV.Next())
            {
                Vertex aV = ShapeCasts.Vertex(itV.Value());
                if (aV.Orientation() == Forward || aV.Orientation() == Reversed)
                {
                    Point3D aPV = Tool.Pnt(aV);
                    aTolSum = aTolP1 + Tool.Tolerance(aV) + global::Kerf.Engine.Foundation.Precision.Confusion();
                    double aDist1 = aP1.Distance(aPV);
                    if (aDist1 < aDist && aDist1 < aTolSum)
                    {
                        aDist = aDist1;
                        aT = Tool.Parameter(aV, aEFwd);
                    }
                }
            }
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(aDist))
            {
                return -3;
            }
        }
        return 0;
    }

    /// <summary>
    /// Computes the parameter of the vertex on the edge and the tolerance the vertex would need there. Returns 0 if
    /// the distance is within the sum of the tolerances and the fuzzy value, -1 for a degenerated edge, -2 for an edge
    /// without geometry, -3 if the projection fails, -4 if the vertex is too far.
    /// </summary>
    public int ComputeVE(in Vertex theV, in Edge theE, ref double theT, ref double theTol) =>
        ComputeVE(theV, theE, ref theT, ref theTol, global::Kerf.Engine.Foundation.Precision.Confusion());

    /// <inheritdoc cref="ComputeVE(in Vertex, in Edge, ref double, ref double)"/>
    public int ComputeVE(in Vertex theV, in Edge theE, ref double theT, ref double theTol, double theFuzz)
    {
        if (Tool.Degenerated(theE))
        {
            return -1;
        }
        if (!Tool.IsGeometric(theE))
        {
            return -2;
        }
        double aDist, aTolV, aTolE, aTolSum;
        int aNbProj;
        Point3D aP;
        //
        aP = Tool.Pnt(theV);
        //
        GeomAPI_ProjectPointOnCurve aProjector = ProjPC(theE);
        aProjector.Perform(aP);

        aNbProj = aProjector.NbPoints();
        if (aNbProj == 0)
        {
            return -3;
        }
        //
        aDist = aProjector.LowerDistance();
        //
        aTolV = Tool.Tolerance(theV);
        aTolE = Tool.Tolerance(theE);
        aTolSum = aTolV + aTolE + Max(theFuzz, global::Kerf.Engine.Foundation.Precision.Confusion());
        //
        theTol = aDist + aTolE;
        theT = aProjector.LowerDistanceParameter();
        if (aDist > aTolSum)
        {
            return -4;
        }
        return 0;
    }

    /// <summary>
    /// Computes the UV parameters of the vertex on the face and the tolerance the vertex would need there. Returns 0
    /// if the distance is within the sum of the tolerances and the fuzzy value and the projection is inside the face,
    /// -1 if the projection fails, -2 if the vertex is too far, -3 if the projection is out of or on the face's
    /// boundary.
    /// </summary>
    public int ComputeVF(in Vertex theVertex, in Face theFace, ref double theU, ref double theV, ref double theTol) =>
        ComputeVF(theVertex, theFace, ref theU, ref theV, ref theTol, global::Kerf.Engine.Foundation.Precision.Confusion());

    /// <inheritdoc cref="ComputeVF(in Vertex, in Face, ref double, ref double, ref double)"/>
    public int ComputeVF(in Vertex theVertex, in Face theFace, ref double theU, ref double theV, ref double theTol,
                         double theFuzz)
    {
        double aTolV, aTolF, aTolSum, aDist;
        Point3D aP;

        aP = Tool.Pnt(theVertex);
        //
        // 1. Check if the point is projectable on the surface
        GeomAPI_ProjectPointOnSurf aProjector = ProjPS(theFace);
        aProjector.Perform(aP);
        //
        if (!aProjector.IsDone())
        { // the point is not  projectable on the surface
            return -1;
        }
        //
        // 2. Check the distance between the projection point and
        //    the original point
        aDist = aProjector.LowerDistance();
        //
        aTolV = Tool.Tolerance(theVertex);
        aTolF = Tool.Tolerance(theFace);
        //
        aTolSum = aTolV + aTolF + Max(theFuzz, global::Kerf.Engine.Foundation.Precision.Confusion());
        theTol = aDist + aTolF;
        aProjector.LowerDistanceParameters(out theU, out theV);
        //
        if (aDist > aTolSum)
        {
            // the distance is too large
            return -2;
        }
        //
        Point2D aP2d = new(theU, theV);
        bool pri = IsPointInFace(theFace, aP2d);
        if (!pri)
        { //  the point lays on the surface but out of the face
            return -3;
        }
        return 0;
    }

    /// <summary>The state of the 2D point relative to the face (<see cref="IntTools_FClass2d.Perform"/>).</summary>
    public Containment StatePointFace(in Face aF, in Point2D aP2d)
    {
        Containment aState;
        IntTools_FClass2d aClass2d = FClass2d(aF);
        aState = aClass2d.Perform(aP2d);
        return aState;
    }

    /// <summary>True if the 2D point is inside the face's boundaries (not OUT, not ON).</summary>
    public bool IsPointInFace(in Face aF, in Point2D aP2d)
    {
        Containment aState = StatePointFace(aF, aP2d);
        return aState != Out && aState != On;
    }

    /// <summary>True if the point projects on the face's surface closer than the tolerance, inside the face.</summary>
    public bool IsPointInFace(in Point3D aP, in Face aF, double aTol)
    {
        bool bIn = false;
        double aDist;
        //
        GeomAPI_ProjectPointOnSurf aProjector = ProjPS(aF);
        aProjector.Perform(aP);
        //
        bool bDone = aProjector.IsDone();
        if (bDone)
        {
            aDist = aProjector.LowerDistance();
            if (aDist < aTol)
            {
                aProjector.LowerDistanceParameters(out double U, out double V);
                Point2D aP2D = new(U, V);
                bIn = IsPointInFace(aF, aP2D);
            }
        }
        //
        return bIn;
    }

    /// <summary>True if the 2D point is inside or on the face's boundaries.</summary>
    public bool IsPointInOnFace(in Face aF, in Point2D aP2d)
    {
        Containment aState = StatePointFace(aF, aP2d);
        return aState != Out;
    }

    /// <summary>
    /// True if the point projects on the face's surface within the tolerance, inside or on the face's boundaries.
    /// </summary>
    public bool IsValidPointForFace(in Point3D aP, in Face aF, double aTol)
    {
        bool bFlag;
        double Umin;

        GeomAPI_ProjectPointOnSurf aProjector = ProjPS(aF);
        aProjector.Perform(aP);

        bFlag = aProjector.IsDone();
        if (bFlag)
        {

            Umin = aProjector.LowerDistance();
            // if (Umin > 1.e-3) { // it was
            if (Umin > aTol)
            {
                return !bFlag;
            }
            //
            aProjector.LowerDistanceParameters(out double U, out double V);
            Point2D aP2D = new(U, V);
            bFlag = IsPointInOnFace(aF, aP2D);
        }
        return bFlag;
    }

    /// <summary>True if <see cref="IsValidPointForFace"/> holds for both faces.</summary>
    public bool IsValidPointForFaces(in Point3D aP, in Face aF1, in Face aF2, double aTol)
    {
        bool bFlag1, bFlag2;

        bFlag1 = IsValidPointForFace(aP, aF1, aTol);
        if (!bFlag1)
        {
            return bFlag1;
        }
        bFlag2 = IsValidPointForFace(aP, aF2, aTol);
        return bFlag2;
    }

    /// <summary>True if <see cref="IsValidPointForFace"/> holds for the curve's intermediate point between the parameters.</summary>
    public bool IsValidBlockForFace(double aT1, double aT2, in IntTools_Curve aC, in Face aF, double aTol)
    {
        bool bFlag;
        double aTInterm;
        Point3D aPInterm;

        aTInterm = IntTools_Tools.IntermediatePoint(aT1, aT2);

        Curve3D aC3D = aC.Curve()!;
        // point 3D
        aC3D.D0(aTInterm, out aPInterm);
        //
        bFlag = IsValidPointForFace(aPInterm, aF, aTol);
        return bFlag;
    }

    /// <summary>
    /// True if the curve's intermediate point between the parameters is valid for both faces: through the pcurve where
    /// the curve has one for the face, through projection otherwise.
    /// </summary>
    public bool IsValidBlockForFaces(double theT1, double theT2, in IntTools_Curve theC, in Face theF1, in Face theF2,
                                     double theTol)
    {
        const int aNbElem = 2;
        Curve2D? aPC1 = theC.FirstCurve2d();
        Curve2D? aPC2 = theC.SecondCurve2d();
        Curve3D aC3D = theC.Curve()!;

        Curve2D?[] anArrPC = [aPC1, aPC2];
        Face[] anArrF = [theF1, theF2];

        double aMidPar = IntTools_Tools.IntermediatePoint(theT1, theT2);
        Point3D aP = aC3D.Value(aMidPar);

        bool bFlag = true;
        Point2D aPnt2D;

        for (int i = 0; (i < aNbElem) && bFlag; ++i)
        {
            Curve2D? aPC = anArrPC[i];
            Face aF = anArrF[i];

            if (aPC is not null)
            {
                aPC.D0(aMidPar, out aPnt2D);
                bFlag = IsPointInOnFace(aF, aPnt2D);
            }
            else
            {
                bFlag = IsValidPointForFace(aP, aF, theTol);
            }
        }

        return bFlag;
    }

    /// <summary>
    /// Computes the parameter of the vertex on the curve. True if the distance is within the sum of the vertex's
    /// tolerance and <paramref name="aTolC"/> (doubled, at least 1e-5 on B-spline and Bézier curves, 1e-6 on others);
    /// otherwise, or if the projection fails, false (and <paramref name="aT"/> is meaningless).
    /// </summary>
    public bool IsVertexOnLine(in Vertex aV, in IntTools_Curve aC, double aTolC, ref double aT)
    {
        bool bRet;
        double aTolV;
        //
        aTolV = Tool.Tolerance(aV);
        bRet = IsVertexOnLine(aV, aTolV, aC, aTolC, ref aT);
        //
        return bRet;
    }

    /// <inheritdoc cref="IsVertexOnLine(in Vertex, in IntTools_Curve, double, ref double)"/>
    public bool IsVertexOnLine(in Vertex aV, double aTolV, in IntTools_Curve aC, double aTolC, ref double aT)
    {
        double aFirst, aLast, aDist, aTolSum;
        int aNbProj;
        Point3D aPv;

        aPv = Tool.Pnt(aV);

        Curve3D aC3D = aC.Curve()!;

        aTolSum = aTolV + aTolC;
        //
        CurveAdaptor aGAC = new(aC3D);
        CurveKind aType = aGAC.GetType();
        if (aType == BSplineCurve || aType == BezierCurve)
        {
            aTolSum = 2.0 * aTolSum;
            if (aTolSum < 1.0e-5)
            {
                aTolSum = 1.0e-5;
            }
        }
        else
        {
            aTolSum = 2.0 * aTolSum; // xft
            if (aTolSum < 1.0e-6)
            {
                aTolSum = 1.0e-6;
            }
        }
        //
        aFirst = aC3D.FirstParameter();
        aLast = aC3D.LastParameter();
        //
        // Checking extremities first
        // It is necessary to chose the closest bound to the point
        bool bFirstValid = false;
        double aFirstDist = global::Kerf.Engine.Foundation.Precision.Infinite();
        //
        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(aFirst))
        {
            Point3D aPCFirst = aC3D.Value(aFirst);
            aFirstDist = aPv.Distance(aPCFirst);
            if (aFirstDist < aTolSum)
            {
                bFirstValid = true;
                aT = aFirst;
                //
                if (aFirstDist > aTolV)
                {
                    LocateExtPC anExt = new(aPv, aGAC, aFirst, 1.0e-10);

                    if (anExt.IsDone)
                    {
                        POnCurv aPOncurve = anExt.Point();
                        aT = aPOncurve.Parameter();

                        if ((aT > (aLast + aFirst) * 0.5) || (aPv.Distance(aPOncurve.Value()) > aTolSum)
                            || (aPCFirst.Distance(aPOncurve.Value()) < global::Kerf.Engine.Foundation.Precision.Confusion()))
                        {
                            aT = aFirst;
                        }
                    }
                    else
                    {
                        // Local search may fail. Try to use more precise algo.
                        CurvePointExtrema anExt2 = new(aPv, aGAC, 1.0e-10);
                        double aMinDist = RealLast();
                        int aMinIdx = -1;
                        if (anExt2.IsDone)
                        {
                            for (int anIdx = 1; anIdx <= anExt2.ExtCount; anIdx++)
                            {
                                if (anExt2.IsMin(anIdx) && anExt2.SquareDistance(anIdx) < aMinDist)
                                {
                                    aMinDist = anExt2.SquareDistance(anIdx);
                                    aMinIdx = anIdx;
                                }
                            }
                        }
                        if (aMinIdx != -1)
                        {
                            POnCurv aPOncurve = anExt2.Point(aMinIdx);
                            aT = aPOncurve.Parameter();

                            if ((aT > (aLast + aFirst) * 0.5) || (aPv.Distance(aPOncurve.Value()) > aTolSum)
                                || (aPCFirst.Distance(aPOncurve.Value()) < global::Kerf.Engine.Foundation.Precision.Confusion()))
                            {
                                aT = aFirst;
                            }
                        }
                    }
                }
            }
        }
        //
        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(aLast))
        {
            Point3D aPCLast = aC3D.Value(aLast);
            aDist = aPv.Distance(aPCLast);
            if (bFirstValid && (aFirstDist < aDist))
            {
                return true;
            }
            //
            if (aDist < aTolSum)
            {
                aT = aLast;
                //
                if (aDist > aTolV)
                {
                    LocateExtPC anExt = new(aPv, aGAC, aLast, 1.0e-10);

                    if (anExt.IsDone)
                    {
                        POnCurv aPOncurve = anExt.Point();
                        aT = aPOncurve.Parameter();

                        if ((aT < (aLast + aFirst) * 0.5) || (aPv.Distance(aPOncurve.Value()) > aTolSum)
                            || (aPCLast.Distance(aPOncurve.Value()) < global::Kerf.Engine.Foundation.Precision.Confusion()))
                        {
                            aT = aLast;
                        }
                    }
                    else
                    {
                        // Local search may fail. Try to use more precise algo.
                        CurvePointExtrema anExt2 = new(aPv, aGAC, 1.0e-10);
                        double aMinDist = RealLast();
                        int aMinIdx = -1;
                        if (anExt2.IsDone)
                        {
                            for (int anIdx = 1; anIdx <= anExt2.ExtCount; anIdx++)
                            {
                                if (anExt2.IsMin(anIdx) && anExt2.SquareDistance(anIdx) < aMinDist)
                                {
                                    aMinDist = anExt2.SquareDistance(anIdx);
                                    aMinIdx = anIdx;
                                }
                            }
                        }
                        if (aMinIdx != -1)
                        {
                            POnCurv aPOncurve = anExt2.Point(aMinIdx);
                            aT = aPOncurve.Parameter();

                            if ((aT < (aLast + aFirst) * 0.5) || (aPv.Distance(aPOncurve.Value()) > aTolSum)
                                || (aPCLast.Distance(aPOncurve.Value()) < global::Kerf.Engine.Foundation.Precision.Confusion()))
                            {
                                aT = aLast;
                            }
                        }
                    }
                }
                //
                return true;
            }
        }
        else if (bFirstValid)
        {
            return true;
        }
        //
        GeomAPI_ProjectPointOnCurve aProjector = ProjPT(aC3D);
        aProjector.Perform(aPv);

        aNbProj = aProjector.NbPoints();
        if (aNbProj == 0)
        {
            if (aC3D is BoundedCurve3D aBC)
            {
                Point3D aPStart = aBC.StartPoint();
                Point3D aPEnd = aBC.EndPoint();

                aDist = aPv.Distance(aPStart);
                if (aDist < aTolSum)
                {
                    aT = aFirst;
                    return true;
                }

                aDist = aPv.Distance(aPEnd);
                if (aDist < aTolSum)
                {
                    aT = aLast;
                    return true;
                }
            }

            return false;
        }

        aDist = aProjector.LowerDistance();

        if (aDist > aTolSum)
        {
            return false;
        }

        aT = aProjector.LowerDistanceParameter();

        return true;
    }

    /// <summary>Computes the parameter of the point's projection on the edge; false if the projection fails.</summary>
    public bool ProjectPointOnEdge(in Point3D aP, in Edge anEdge, ref double aT)
    {
        int aNbPoints;

        GeomAPI_ProjectPointOnCurve aProjector = ProjPC(anEdge);
        aProjector.Perform(aP);

        aNbPoints = aProjector.NbPoints();
        if (aNbPoints != 0)
        {
            aT = aProjector.LowerDistanceParameter();
            return true;
        }
        return false;
    }

    /// <summary>
    /// Sets the tolerance used to project points on surfaces (1e-12 by default), and clears the cached surface
    /// projectors so that every later one uses it.
    /// </summary>
    public void SetPOnSProjectionTolerance(double theValue)
    {
        myPOnSTolerance = theValue;
        clearCachedPOnSProjectors();
    }

    /// <summary>Clears the map of cached surface projectors.</summary>
    private void clearCachedPOnSProjectors() => myProjPSMap.Clear();

    /// <summary>The UV bounds of the face, from its cached surface adaptor.</summary>
    public void UVBounds(in Face theFace, out double UMin, out double UMax, out double VMin, out double VMax)
    {
        FaceSurfaceAdaptor aBAS = SurfaceAdaptor(theFace);
        UMin = aBAS.FirstUParameter();
        UMax = aBAS.LastUParameter();
        VMin = aBAS.FirstVParameter();
        VMax = aBAS.LastVParameter();
    }

    // myCreateFlag tells OCCT whether the context was given its allocator; the allocator goes, the flag stays.
    internal int CreateFlag() => myCreateFlag;

    internal double POnSProjectionTolerance() => myPOnSTolerance;
}
