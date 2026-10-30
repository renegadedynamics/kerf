// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_Context.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_Context.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Extrema_ExtFlag;
using static Occt.GeomAbs_CurveType;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

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
/// The cache audit (plan section 2.4, decision 6401): every cached tool but one gives the same answers whatever the
/// context computed before. The exception is <see cref="SurfaceData"/>, whose boxes IntTools_BeanFaceIntersector
/// keys by sample index and enlarges by the first edge's tolerance (occt-issues.md, unit 6C).
/// </para>
/// <para>
/// The maps are keyed by shape with <see cref="TopTools_ShapeMapHasher"/> (orientation ignored), and by curve with
/// reference equality (OCCT's handle hasher). The allocator goes (translation rules, section 9).
/// </para>
/// </remarks>
public class IntTools_Context : Standard_Transient
{
    private readonly NCollection_DataMap<TopoDS_Shape, IntTools_FClass2d> myFClass2dMap = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_DataMap<TopoDS_Shape, GeomAPI_ProjectPointOnSurf> myProjPSMap = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_DataMap<TopoDS_Shape, GeomAPI_ProjectPointOnCurve> myProjPCMap = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_DataMap<TopoDS_Shape, BRepClass3d_SolidClassifier> mySClassMap = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_DataMap<Geom_Curve, GeomAPI_ProjectPointOnCurve> myProjPTMap = new(ReferenceEqualityComparer.Instance);
    private readonly NCollection_DataMap<TopoDS_Shape, Geom2dHatch_Hatcher> myHatcherMap = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_DataMap<TopoDS_Shape, IntTools_SurfaceRangeLocalizeData> myProjSDataMap = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_DataMap<TopoDS_Shape, Bnd_Box> myBndBoxDataMap = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_DataMap<TopoDS_Shape, BRepAdaptor_Surface> mySurfAdaptorMap = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_DataMap<TopoDS_Shape, Bnd_OBB> myOBBMap = new(TopTools_ShapeMapHasher.Instance);
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
    public Bnd_Box BndBox(in TopoDS_Shape aS)
    {
        if (!myBndBoxDataMap.Find(aS, out Bnd_Box aBox))
        {
            aBox = new Bnd_Box();
            BRepBndLib.Add(aS, ref aBox);
            myBndBoxDataMap.Bind(aS, aBox);
        }
        return aBox;
    }

    /// <summary>True if the face's bounding box is open in some direction (an infinite face).</summary>
    public bool IsInfiniteFace(in TopoDS_Face aFace)
    {
        Bnd_Box aBox = BndBox(aFace);
        return aBox.IsOpenXmax() || aBox.IsOpenXmin() || aBox.IsOpenYmax() || aBox.IsOpenYmin()
               || aBox.IsOpenZmax() || aBox.IsOpenZmin();
    }

    /// <summary>The 2D point classifier of the face (taken FORWARD, with the face's tolerance), built on the first call.</summary>
    public IntTools_FClass2d FClass2d(in TopoDS_Face aF)
    {
        if (!myFClass2dMap.Find(aF, out IntTools_FClass2d? pFClass2d))
        {
            double aTolF;
            TopoDS_Face aFF;
            //
            aFF = aF;
            aFF.Orientation(TopAbs_FORWARD);
            aTolF = BRep_Tool.Tolerance(aFF);
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
    public GeomAPI_ProjectPointOnSurf ProjPS(in TopoDS_Face aF)
    {
        if (!myProjPSMap.Find(aF, out GeomAPI_ProjectPointOnSurf? pProjPS))
        {
            UVBounds(aF, out double Umin, out double Usup, out double Vmin, out double Vsup);
            Geom_Surface aS = BRep_Tool.Surface(aF)!;
            //
            pProjPS = new GeomAPI_ProjectPointOnSurf();
            pProjPS.Init(aS, Umin, Usup, Vmin, Vsup, myPOnSTolerance);
            pProjPS.SetExtremaFlag(Extrema_ExtFlag_MIN);
            //
            myProjPSMap.Bind(aF, pProjPS);
        }
        return pProjPS!;
    }

    /// <summary>The point projector of the edge's 3D curve over the edge's range, built on the first call.</summary>
    public GeomAPI_ProjectPointOnCurve ProjPC(in TopoDS_Edge aE)
    {
        if (!myProjPCMap.Find(aE, out GeomAPI_ProjectPointOnCurve? pProjPC))
        {
            Geom_Curve? aC3D = BRep_Tool.Curve(aE, out double f, out double l);
            //
            pProjPC = new GeomAPI_ProjectPointOnCurve();
            pProjPC.Init(aC3D!, f, l);
            //
            myProjPCMap.Bind(aE, pProjPC);
        }
        return pProjPC!;
    }

    /// <summary>The point projector of the curve over its own range, built on the first call (keyed by the curve object).</summary>
    public GeomAPI_ProjectPointOnCurve ProjPT(Geom_Curve aC3D)
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
    public BRepClass3d_SolidClassifier SolidClassifier(in TopoDS_Solid aSolid)
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
    public BRepAdaptor_Surface SurfaceAdaptor(in TopoDS_Face theFace)
    {
        if (!mySurfAdaptorMap.Find(theFace, out BRepAdaptor_Surface? pBAS))
        {
            //
            pBAS = new BRepAdaptor_Surface(theFace, true);
            //
            mySurfAdaptorMap.Bind(theFace, pBAS);
        }
        return pBAS!;
    }

    /// <summary>
    /// The 2D hatcher of the face (taken FORWARD), loaded with the face's pcurves trimmed to their edges' ranges, built
    /// on the first call.
    /// </summary>
    public Geom2dHatch_Hatcher Hatcher(in TopoDS_Face aF)
    {
        if (!myHatcherMap.Find(aF, out Geom2dHatch_Hatcher? pHatcher))
        {
            double aTolArcIntr, aTolTangfIntr, aTolHatch2D, aTolHatch3D;
            double aEpsT;
            TopAbs_Orientation aOrE;
            Geom2d_Curve? aC2D;
            Geom2d_TrimmedCurve aCT2D;
            TopoDS_Face aFF;
            TopExp_Explorer aExp = new();
            //
            aTolHatch2D = 1.0e-8;
            aTolHatch3D = 1.0e-8;
            aTolArcIntr = 1.0e-10;
            aTolTangfIntr = 1.0e-10;
            aEpsT = Precision.PConfusion();
            //
            Geom2dHatch_Intersector aIntr = new(aTolArcIntr, aTolTangfIntr);
            pHatcher = new Geom2dHatch_Hatcher(aIntr, aTolHatch2D, aTolHatch3D, true, false);
            //
            aFF = aF;
            aFF.Orientation(TopAbs_FORWARD);
            _ = BRep_Tool.Surface(aFF);

            aExp.Init(aFF, TopAbs_EDGE);
            for (; aExp.More(); aExp.Next())
            {
                TopoDS_Edge aE = TopoDS.Edge(aExp.Current());
                aOrE = aE.Orientation();
                //
                aC2D = BRep_Tool.CurveOnSurface(aE, aFF, out double aU1, out double aU2);
                if (aC2D is null)
                {
                    continue;
                }
                if (Math.Abs(aU1 - aU2) < aEpsT)
                {
                    continue;
                }
                //
                aCT2D = new Geom2d_TrimmedCurve(aC2D, aU1, aU2);
                Geom2dAdaptor_Curve aGAC = new(aCT2D);
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
    public Bnd_OBB OBB(in TopoDS_Shape aS) => OBB(aS, Precision.Confusion());

    /// <inheritdoc cref="OBB(in TopoDS_Shape)"/>
    public Bnd_OBB OBB(in TopoDS_Shape aS, double theGap)
    {
        if (!myOBBMap.Find(aS, out Bnd_OBB aBox))
        {
            aBox = new Bnd_OBB();
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
    public IntTools_SurfaceRangeLocalizeData SurfaceData(in TopoDS_Face aF)
    {
        if (!myProjSDataMap.Find(aF, out IntTools_SurfaceRangeLocalizeData? pSData))
        {
            pSData = new IntTools_SurfaceRangeLocalizeData(3,
                                                           3,
                                                           10.0 * Precision.PConfusion(),
                                                           10.0 * Precision.PConfusion());
            //
            myProjSDataMap.Bind(aF, pSData);
        }
        return pSData!;
    }

    /// <summary>
    /// Computes the parameter of the point on the edge. Returns 0 if the distance is within the sum of the point's
    /// and the edge's tolerances (or the point is near a vertex when the projection fails), -2 if the edge has no
    /// geometry, -3 if the projection fails and no vertex is near, -4 if the point is too far.
    /// </summary>
    public int ComputePE(in gp_Pnt aP1, double aTolP1, in TopoDS_Edge aE2, ref double aT, ref double aDist)
    {
        if (!BRep_Tool.IsGeometric(aE2))
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
            aTolE2 = BRep_Tool.Tolerance(aE2);
            aTolSum = aTolP1 + aTolE2 + Precision.Confusion();
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
            TopoDS_Edge aEFwd = TopoDS.Edge(aE2.Oriented(TopAbs_FORWARD));
            TopoDS_Iterator itV = new(aEFwd);
            aDist = RealLast();
            for (; itV.More(); itV.Next())
            {
                TopoDS_Vertex aV = TopoDS.Vertex(itV.Value());
                if (aV.Orientation() == TopAbs_FORWARD || aV.Orientation() == TopAbs_REVERSED)
                {
                    gp_Pnt aPV = BRep_Tool.Pnt(aV);
                    aTolSum = aTolP1 + BRep_Tool.Tolerance(aV) + Precision.Confusion();
                    double aDist1 = aP1.Distance(aPV);
                    if (aDist1 < aDist && aDist1 < aTolSum)
                    {
                        aDist = aDist1;
                        aT = BRep_Tool.Parameter(aV, aEFwd);
                    }
                }
            }
            if (Precision.IsInfinite(aDist))
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
    public int ComputeVE(in TopoDS_Vertex theV, in TopoDS_Edge theE, ref double theT, ref double theTol) =>
        ComputeVE(theV, theE, ref theT, ref theTol, Precision.Confusion());

    /// <inheritdoc cref="ComputeVE(in TopoDS_Vertex, in TopoDS_Edge, ref double, ref double)"/>
    public int ComputeVE(in TopoDS_Vertex theV, in TopoDS_Edge theE, ref double theT, ref double theTol, double theFuzz)
    {
        if (BRep_Tool.Degenerated(theE))
        {
            return -1;
        }
        if (!BRep_Tool.IsGeometric(theE))
        {
            return -2;
        }
        double aDist, aTolV, aTolE, aTolSum;
        int aNbProj;
        gp_Pnt aP;
        //
        aP = BRep_Tool.Pnt(theV);
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
        aTolV = BRep_Tool.Tolerance(theV);
        aTolE = BRep_Tool.Tolerance(theE);
        aTolSum = aTolV + aTolE + Max(theFuzz, Precision.Confusion());
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
    public int ComputeVF(in TopoDS_Vertex theVertex, in TopoDS_Face theFace, ref double theU, ref double theV, ref double theTol) =>
        ComputeVF(theVertex, theFace, ref theU, ref theV, ref theTol, Precision.Confusion());

    /// <inheritdoc cref="ComputeVF(in TopoDS_Vertex, in TopoDS_Face, ref double, ref double, ref double)"/>
    public int ComputeVF(in TopoDS_Vertex theVertex, in TopoDS_Face theFace, ref double theU, ref double theV, ref double theTol,
                         double theFuzz)
    {
        double aTolV, aTolF, aTolSum, aDist;
        gp_Pnt aP;

        aP = BRep_Tool.Pnt(theVertex);
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
        aTolV = BRep_Tool.Tolerance(theVertex);
        aTolF = BRep_Tool.Tolerance(theFace);
        //
        aTolSum = aTolV + aTolF + Max(theFuzz, Precision.Confusion());
        theTol = aDist + aTolF;
        aProjector.LowerDistanceParameters(out theU, out theV);
        //
        if (aDist > aTolSum)
        {
            // the distance is too large
            return -2;
        }
        //
        gp_Pnt2d aP2d = new(theU, theV);
        bool pri = IsPointInFace(theFace, aP2d);
        if (!pri)
        { //  the point lays on the surface but out of the face
            return -3;
        }
        return 0;
    }

    /// <summary>The state of the 2D point relative to the face (<see cref="IntTools_FClass2d.Perform"/>).</summary>
    public TopAbs_State StatePointFace(in TopoDS_Face aF, in gp_Pnt2d aP2d)
    {
        TopAbs_State aState;
        IntTools_FClass2d aClass2d = FClass2d(aF);
        aState = aClass2d.Perform(aP2d);
        return aState;
    }

    /// <summary>True if the 2D point is inside the face's boundaries (not OUT, not ON).</summary>
    public bool IsPointInFace(in TopoDS_Face aF, in gp_Pnt2d aP2d)
    {
        TopAbs_State aState = StatePointFace(aF, aP2d);
        return aState != TopAbs_OUT && aState != TopAbs_ON;
    }

    /// <summary>True if the point projects on the face's surface closer than the tolerance, inside the face.</summary>
    public bool IsPointInFace(in gp_Pnt aP, in TopoDS_Face aF, double aTol)
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
                gp_Pnt2d aP2D = new(U, V);
                bIn = IsPointInFace(aF, aP2D);
            }
        }
        //
        return bIn;
    }

    /// <summary>True if the 2D point is inside or on the face's boundaries.</summary>
    public bool IsPointInOnFace(in TopoDS_Face aF, in gp_Pnt2d aP2d)
    {
        TopAbs_State aState = StatePointFace(aF, aP2d);
        return aState != TopAbs_OUT;
    }

    /// <summary>
    /// True if the point projects on the face's surface within the tolerance, inside or on the face's boundaries.
    /// </summary>
    public bool IsValidPointForFace(in gp_Pnt aP, in TopoDS_Face aF, double aTol)
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
            gp_Pnt2d aP2D = new(U, V);
            bFlag = IsPointInOnFace(aF, aP2D);
        }
        return bFlag;
    }

    /// <summary>True if <see cref="IsValidPointForFace"/> holds for both faces.</summary>
    public bool IsValidPointForFaces(in gp_Pnt aP, in TopoDS_Face aF1, in TopoDS_Face aF2, double aTol)
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
    public bool IsValidBlockForFace(double aT1, double aT2, in IntTools_Curve aC, in TopoDS_Face aF, double aTol)
    {
        bool bFlag;
        double aTInterm;
        gp_Pnt aPInterm;

        aTInterm = IntTools_Tools.IntermediatePoint(aT1, aT2);

        Geom_Curve aC3D = aC.Curve()!;
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
    public bool IsValidBlockForFaces(double theT1, double theT2, in IntTools_Curve theC, in TopoDS_Face theF1, in TopoDS_Face theF2,
                                     double theTol)
    {
        const int aNbElem = 2;
        Geom2d_Curve? aPC1 = theC.FirstCurve2d();
        Geom2d_Curve? aPC2 = theC.SecondCurve2d();
        Geom_Curve aC3D = theC.Curve()!;

        Geom2d_Curve?[] anArrPC = [aPC1, aPC2];
        TopoDS_Face[] anArrF = [theF1, theF2];

        double aMidPar = IntTools_Tools.IntermediatePoint(theT1, theT2);
        gp_Pnt aP = aC3D.Value(aMidPar);

        bool bFlag = true;
        gp_Pnt2d aPnt2D;

        for (int i = 0; (i < aNbElem) && bFlag; ++i)
        {
            Geom2d_Curve? aPC = anArrPC[i];
            TopoDS_Face aF = anArrF[i];

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
    public bool IsVertexOnLine(in TopoDS_Vertex aV, in IntTools_Curve aC, double aTolC, ref double aT)
    {
        bool bRet;
        double aTolV;
        //
        aTolV = BRep_Tool.Tolerance(aV);
        bRet = IsVertexOnLine(aV, aTolV, aC, aTolC, ref aT);
        //
        return bRet;
    }

    /// <inheritdoc cref="IsVertexOnLine(in TopoDS_Vertex, in IntTools_Curve, double, ref double)"/>
    public bool IsVertexOnLine(in TopoDS_Vertex aV, double aTolV, in IntTools_Curve aC, double aTolC, ref double aT)
    {
        double aFirst, aLast, aDist, aTolSum;
        int aNbProj;
        gp_Pnt aPv;

        aPv = BRep_Tool.Pnt(aV);

        Geom_Curve aC3D = aC.Curve()!;

        aTolSum = aTolV + aTolC;
        //
        GeomAdaptor_Curve aGAC = new(aC3D);
        GeomAbs_CurveType aType = aGAC.GetType();
        if (aType == GeomAbs_BSplineCurve || aType == GeomAbs_BezierCurve)
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
        double aFirstDist = Precision.Infinite();
        //
        if (!Precision.IsInfinite(aFirst))
        {
            gp_Pnt aPCFirst = aC3D.Value(aFirst);
            aFirstDist = aPv.Distance(aPCFirst);
            if (aFirstDist < aTolSum)
            {
                bFirstValid = true;
                aT = aFirst;
                //
                if (aFirstDist > aTolV)
                {
                    Extrema_LocateExtPC anExt = new(aPv, aGAC, aFirst, 1.0e-10);

                    if (anExt.IsDone())
                    {
                        Extrema_POnCurv aPOncurve = anExt.Point();
                        aT = aPOncurve.Parameter();

                        if ((aT > (aLast + aFirst) * 0.5) || (aPv.Distance(aPOncurve.Value()) > aTolSum)
                            || (aPCFirst.Distance(aPOncurve.Value()) < Precision.Confusion()))
                        {
                            aT = aFirst;
                        }
                    }
                    else
                    {
                        // Local search may fail. Try to use more precise algo.
                        Extrema_ExtPC anExt2 = new(aPv, aGAC, 1.0e-10);
                        double aMinDist = RealLast();
                        int aMinIdx = -1;
                        if (anExt2.IsDone())
                        {
                            for (int anIdx = 1; anIdx <= anExt2.NbExt(); anIdx++)
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
                            Extrema_POnCurv aPOncurve = anExt2.Point(aMinIdx);
                            aT = aPOncurve.Parameter();

                            if ((aT > (aLast + aFirst) * 0.5) || (aPv.Distance(aPOncurve.Value()) > aTolSum)
                                || (aPCFirst.Distance(aPOncurve.Value()) < Precision.Confusion()))
                            {
                                aT = aFirst;
                            }
                        }
                    }
                }
            }
        }
        //
        if (!Precision.IsInfinite(aLast))
        {
            gp_Pnt aPCLast = aC3D.Value(aLast);
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
                    Extrema_LocateExtPC anExt = new(aPv, aGAC, aLast, 1.0e-10);

                    if (anExt.IsDone())
                    {
                        Extrema_POnCurv aPOncurve = anExt.Point();
                        aT = aPOncurve.Parameter();

                        if ((aT < (aLast + aFirst) * 0.5) || (aPv.Distance(aPOncurve.Value()) > aTolSum)
                            || (aPCLast.Distance(aPOncurve.Value()) < Precision.Confusion()))
                        {
                            aT = aLast;
                        }
                    }
                    else
                    {
                        // Local search may fail. Try to use more precise algo.
                        Extrema_ExtPC anExt2 = new(aPv, aGAC, 1.0e-10);
                        double aMinDist = RealLast();
                        int aMinIdx = -1;
                        if (anExt2.IsDone())
                        {
                            for (int anIdx = 1; anIdx <= anExt2.NbExt(); anIdx++)
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
                            Extrema_POnCurv aPOncurve = anExt2.Point(aMinIdx);
                            aT = aPOncurve.Parameter();

                            if ((aT < (aLast + aFirst) * 0.5) || (aPv.Distance(aPOncurve.Value()) > aTolSum)
                                || (aPCLast.Distance(aPOncurve.Value()) < Precision.Confusion()))
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
            if (aC3D is Geom_BoundedCurve aBC)
            {
                gp_Pnt aPStart = aBC.StartPoint();
                gp_Pnt aPEnd = aBC.EndPoint();

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
    public bool ProjectPointOnEdge(in gp_Pnt aP, in TopoDS_Edge anEdge, ref double aT)
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
    public void UVBounds(in TopoDS_Face theFace, out double UMin, out double UMax, out double VMin, out double VMax)
    {
        BRepAdaptor_Surface aBAS = SurfaceAdaptor(theFace);
        UMin = aBAS.FirstUParameter();
        UMax = aBAS.LastUParameter();
        VMin = aBAS.FirstVParameter();
        VMax = aBAS.LastVParameter();
    }

    // myCreateFlag tells OCCT whether the context was given its allocator; the allocator goes, the flag stays.
    internal int CreateFlag() => myCreateFlag;

    internal double POnSProjectionTolerance() => myPOnSTolerance;
}
