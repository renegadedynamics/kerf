// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_EdgeFace.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_EdgeFace.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// The edge/face intersection algorithm: the common parts of an edge and a face in 3D space, vertices (intersection
/// or touching) or the edge (the edge lies in the face).
/// </summary>
/// <remarks>
/// A working object (plan section 6), derived from by the pave filler's <c>BOPAlgo_EdgeFace</c> (unit 6E). It uses
/// the context it is given (or a new one) for the face's adaptor, projector and classifier and for the localisation
/// data of <see cref="IntTools_BeanFaceIntersector"/>, which gives every run the result a fresh context would give
/// (decision 753). The common parts are the algorithm's own <see cref="IntTools_CommonPrt"/> objects.
/// </remarks>
public class IntTools_EdgeFace
{
    protected TopoDS_Edge myEdge = new();
    protected TopoDS_Face myFace = new();
    protected double myFuzzyValue;
    protected BRepAdaptor_Curve myC = new();
    protected BRepAdaptor_Surface myS = new();
    protected double myCriteria;
    protected bool myIsDone;
    protected int myErrorStatus;
    protected IntTools_Context? myContext;
    protected readonly NCollection_Sequence<IntTools_CommonPrt> mySeqOfCommonPrts = new();
    protected IntTools_Range myRange;
    protected bool myQuickCoincidenceCheck;

    /// <summary>The minimal distance found.</summary>
    protected double myMinDistance;

    /// <summary>An empty algorithm.</summary>
    public IntTools_EdgeFace()
    {
        myFuzzyValue = global::Kerf.Engine.Foundation.Precision.Confusion();
        myIsDone = false;
        myErrorStatus = 1;
        myQuickCoincidenceCheck = false;
        myMinDistance = RealLast();
    }

    /// <summary>Sets the edge.</summary>
    public void SetEdge(in TopoDS_Edge theEdge) => myEdge = theEdge;

    /// <summary>The edge.</summary>
    public TopoDS_Edge Edge() => myEdge;

    /// <summary>Sets the face.</summary>
    public void SetFace(in TopoDS_Face theFace) => myFace = theFace;

    /// <summary>The face.</summary>
    public TopoDS_Face Face() => myFace;

    /// <summary>Sets the range of the edge processed by the algorithm.</summary>
    public void SetRange(in IntTools_Range theRange) => myRange = theRange;

    /// <summary>Sets the range of the edge processed by the algorithm.</summary>
    public void SetRange(double theFirst, double theLast)
    {
        myRange.SetFirst(theFirst);
        myRange.SetLast(theLast);
    }

    /// <summary>The range of the edge.</summary>
    public IntTools_Range Range() => myRange;

    /// <summary>Sets the intersection context.</summary>
    public void SetContext(IntTools_Context? theContext) => myContext = theContext;

    /// <summary>The intersection context.</summary>
    public IntTools_Context? Context() => myContext;

    /// <summary>Sets the fuzzy value (at least <see cref="Precision.Confusion"/>).</summary>
    public void SetFuzzyValue(double theFuzz) => myFuzzyValue = Max(theFuzz, global::Kerf.Engine.Foundation.Precision.Confusion());

    /// <summary>The fuzzy value.</summary>
    public double FuzzyValue() => myFuzzyValue;

    /// <summary>
    /// Sets the flag of the quick coincidence check, safe only if the edge's vertices lie on the face and the edge
    /// doesn't cross the face's boundaries on the range.
    /// </summary>
    public void UseQuickCoincidenceCheck(bool theFlag) => myQuickCoincidenceCheck = theFlag;

    /// <summary>The flag of the quick coincidence check.</summary>
    public bool IsCoincidenceCheckedQuickly() => myQuickCoincidenceCheck;

    /// <summary>True if the computation succeeded.</summary>
    public bool IsDone() => myIsDone;

    /// <summary>
    /// The completion code: 0 success, 1 not started, 2 and 3 invalid data (degenerated or non-geometric edge), 4 a
    /// projection failed.
    /// </summary>
    public int ErrorStatus() => myErrorStatus;

    /// <summary>The common parts (the algorithm's own sequence; ported code only reads it).</summary>
    public NCollection_Sequence<IntTools_CommonPrt> CommonParts() => mySeqOfCommonPrts;

    /// <summary>The minimal distance found between the edge and the face.</summary>
    public double MinimalDistance() => myMinDistance;

    /// <summary>Checks if the edge really lies in the face.</summary>
    protected bool IsCoincident()
    {
        int i, iCnt;
        double dT, aT, aD, aT1, aT2, aU, aV;

        Point3D aP;
        TopAbs_State aState;
        Point2D aP2d = new();
        //
        GeomAPI_ProjectPointOnSurf aProjector = myContext!.ProjPS(myFace);

        int aNbSeg = 23;
        if (myC.GetType() == Line && myS.GetType() == Plane)
        {
            aNbSeg = 2; // Check only three points for Line/Plane intersection
        }

        double aTresh = 0.5;
        int aTreshIdxF = RealToInt((aNbSeg + 1) * 0.25),
            aTreshIdxL = RealToInt((aNbSeg + 1) * 0.75);

        aT1 = myRange.First();
        aT2 = myRange.Last();
        double aBndShift = 0.01 * (aT2 - aT1);
        // Shifting first and last curve points in order to avoid projection
        // on surface boundary and rejection projection point with minimal distance
        aT1 += aBndShift;
        aT2 -= aBndShift;
        dT = (aT2 - aT1) / aNbSeg;
        //
        bool isClassified = false;
        iCnt = 0;
        for (i = 0; i <= aNbSeg; ++i)
        {
            aT = aT1 + i * dT;
            aP = myC.Value(aT);
            //
            aProjector.Perform(aP);
            if (!aProjector.IsDone())
            {
                continue;
            }
            //

            aD = aProjector.LowerDistance();
            if (aD > myCriteria)
            {
                if (aD > 100.0 * myCriteria)
                {
                    return false;
                }
                else
                {
                    continue;
                }
            }
            //

            ++iCnt;

            // We classify only three points: in the begin, in the
            // end and in the middle of the edge.
            // However, exact middle point (when i == (aNbSeg + 1)/2)
            // can be unprojectable. Therefore, it will not be able to
            // be classified. Therefore, points with indexes in
            //[aTreshIdxF, aTreshIdxL] range are made available
            // for classification.
            // isClassified == TRUE if MIDDLE point has been chosen and
            // classified correctly.

            if (((0 < i) && (i < aTreshIdxF)) || ((aTreshIdxL < i) && (i < aNbSeg)))
            {
                continue;
            }

            if (isClassified && (i != aNbSeg))
            {
                continue;
            }

            aProjector.LowerDistanceParameters(out aU, out aV);
            aP2d.SetX(aU);
            aP2d.SetY(aV);

            IntTools_FClass2d aClass2d = myContext.FClass2d(myFace);
            aState = aClass2d.Perform(aP2d);

            if (aState == TopAbs_OUT)
            {
                return false;
            }

            if (i != 0)
            {
                isClassified = true;
            }
        }
        //
        double aCoeff = (double)iCnt / ((double)aNbSeg + 1);
        return aCoeff > aTresh;
    }

    /// <summary>Checks the data.</summary>
    protected void CheckData()
    {
        if (BRep_Tool.Degenerated(myEdge))
        {
            myErrorStatus = 2;
        }
        if (!BRep_Tool.IsGeometric(myEdge))
        {
            myErrorStatus = 3;
        }
    }

    /// <summary>True if the point of the edge at <paramref name="aT"/> projects onto the face within the criteria.</summary>
    protected bool IsProjectable(double aT)
    {
        bool bFlag;
        Point3D aPC;
        //
        myC.D0(aT, out aPC);
        bFlag = myContext!.IsValidPointForFace(aPC, myFace, myCriteria);
        //
        return bFlag;
    }

    /// <summary>The distance from the edge's point at <paramref name="t"/> to the face's surface, less the criteria.</summary>
    protected double DistanceFunction(double t)
    {
        double aD = 0.0;

        //
        Point3D P;
        myC.D0(t, out P);
        //
        bool bIsEqDistance;

        bIsEqDistance = IntTools_EdgeFace.IsEqDistance(P, myS, 1.0e-7, ref aD);
        if (bIsEqDistance)
        {
            aD = aD - myCriteria;
            return aD;
        }

        //
        bool bFlag = false;

        GeomAPI_ProjectPointOnSurf aLocProj = myContext!.ProjPS(myFace);
        aLocProj.Perform(P);
        bFlag = aLocProj.IsDone();

        if (bFlag)
        {
            aD = aLocProj.LowerDistance();
        }
        //

        if (!bFlag)
        {
            myErrorStatus = 4;
            return 99.0;
        }

        //
        //   aD=aProjector.LowerDistance();
        //
        aD = aD - myCriteria;
        return aD;
    }

    /// <summary>
    /// True, with the distance in <paramref name="aD"/>, if the point is on the axis of a cylinder or cone or on the
    /// spine of a torus (all points of the surface's circle are then at the same distance).
    /// </summary>
    protected static bool IsEqDistance(in Point3D aP, BRepAdaptor_Surface aBAS, double aTol, ref double aD)
    {
        bool bRetFlag = true;

        SurfaceKind aSurfType = aBAS.GetType();

        if (aSurfType == SurfaceKind.Cylinder)
        {
            Cylinder aCyl = aBAS.Cylinder();
            Axis3D anAx1 = aCyl.Axis();
            Line3D aLinAxis = new(anAx1);
            double aDC, aRadius = aCyl.Radius();
            aDC = aLinAxis.Distance(aP);
            if (aDC < aTol)
            {
                aD = aRadius;
                return bRetFlag;
            }
        }

        if (aSurfType == SurfaceKind.Cone)
        {
            Cone aCone = aBAS.Cone();
            Axis3D anAx1 = aCone.Axis();
            Line3D aLinAxis = new(anAx1);
            double aDC, aRadius, aDS, aSemiAngle;
            aDC = aLinAxis.Distance(aP);
            if (aDC < aTol)
            {
                Point3D anApex = aCone.Apex();
                aSemiAngle = aCone.SemiAngle();
                aDS = aP.Distance(anApex);

                aRadius = aDS * Math.Tan(aSemiAngle);
                aD = aRadius;
                return bRetFlag;
            }
        }

        if (aSurfType == SurfaceKind.Torus)
        {
            double aMajorRadius, aMinorRadius, aDC;

            Torus aTorus = aBAS.Torus();
            Point3D aPLoc = aTorus.Location();
            aMajorRadius = aTorus.MajorRadius();

            aDC = Math.Abs(aPLoc.Distance(aP) - aMajorRadius);
            if (aDC < aTol)
            {
                aMinorRadius = aTorus.MinorRadius();
                aD = aMinorRadius;
                return bRetFlag;
            }
        }
        return !bRetFlag;
    }

    /// <summary>Sets the type of the common part (edge or vertex) and its vertex parameter.</summary>
    protected int MakeType(IntTools_CommonPrt aCommonPrt)
    {
        double af1, al1;
        double df1, tm = 0.0;
        bool bAllNullFlag;
        //
        bAllNullFlag = aCommonPrt.AllNullFlag();
        if (bAllNullFlag)
        {
            aCommonPrt.SetType(TopAbs_EDGE);
            return 0;
        }
        //
        aCommonPrt.Range1(out af1, out al1);

        {
            Point3D aPF, aPL;
            myC.D0(af1, out aPF);
            myC.D0(al1, out aPL);
            df1 = aPF.Distance(aPL);
            bool isWholeRange = false;

            if ((Math.Abs(af1 - myRange.First()) < myC.Resolution(myCriteria))
                && (Math.Abs(al1 - myRange.Last()) < myC.Resolution(myCriteria)))
            {
                isWholeRange = true;
            }

            if ((df1 > myCriteria * 2.0) && isWholeRange)
            {
                aCommonPrt.SetType(TopAbs_EDGE);
            }
            else
            {
                if (isWholeRange)
                {
                    tm = (af1 + al1) * 0.5;

                    if (aPF.Distance(myC.Value(tm)) > myCriteria * 2.0)
                    {
                        aCommonPrt.SetType(TopAbs_EDGE);
                        return 0;
                    }
                }

                if (!CheckTouch(aCommonPrt, ref tm))
                {
                    tm = (af1 + al1) * 0.5;
                }
                aCommonPrt.SetType(TopAbs_VERTEX);
                aCommonPrt.SetVertexParameter1(tm);
                aCommonPrt.SetRange1(af1, al1);
            }
        }
        return 0;
    }

    /// <summary>True, with the touching parameter in <paramref name="aTx"/>, if the edge touches the face on the part.</summary>
    protected bool CheckTouch(IntTools_CommonPrt aCP, ref double aTx)
    {
        double aTF, aTL, Tol, U1f, U1l, V1f, V1l, aDist2, aMinDist2;
        bool theflag = false;
        int aNbExt, iLower;

        aCP.Range1(out aTF, out aTL);

        //
        double aCR;
        aCR = myC.Resolution(myCriteria);
        if ((Math.Abs(aTF - myRange.First()) < aCR) && (Math.Abs(aTL - myRange.Last()) < aCR))
        {
            return theflag; // EDGE
        }
        //

        Tol = global::Kerf.Engine.Foundation.Precision.PConfusion();

        Geom_Curve Curve = BRep_Tool.Curve(myC.Edge(), out _, out _)!;
        Geom_Surface Surface = BRep_Tool.Surface(myS.Face())!;
        //   Surface->Bounds(U1f,U1l,V1f,V1l);
        U1f = myS.FirstUParameter();
        U1l = myS.LastUParameter();
        V1f = myS.FirstVParameter();
        V1l = myS.LastVParameter();

        GeomAdaptor_Curve TheCurve = new(Curve, aTF, aTL);
        GeomAdaptor_Surface TheSurface = new(Surface, U1f, U1l, V1f, V1l);

        Extrema_ExtCS anExtrema = new(TheCurve, TheSurface, Tol, Tol);

        aDist2 = 1.0e100;

        if (anExtrema.IsDone())
        {
            aMinDist2 = aDist2;

            if (!anExtrema.IsParallel())
            {
                aNbExt = anExtrema.NbExt();

                if (aNbExt > 0)
                {
                    iLower = 1;
                    for (int i = 1; i <= aNbExt; i++)
                    {
                        aDist2 = anExtrema.SquareDistance(i);
                        if (aDist2 < aMinDist2)
                        {
                            aMinDist2 = aDist2;
                            iLower = i;
                        }
                    }
                    aDist2 = anExtrema.SquareDistance(iLower);
                    anExtrema.Points(iLower, out Extrema_POnCurv aPOnC, out _);
                    aTx = aPOnC.Parameter();
                }
                else
                {
                    // modified by NIZHNY-MKK  Thu Jul 21 11:35:32 2005.BEGIN
                    IntCurveSurface_HInter anExactIntersector = new();

                    // OCCT copies the adaptors into handles; they aren't used afterwards, so the port passes them.
                    anExactIntersector.Perform(TheCurve, TheSurface);

                    if (anExactIntersector.IsDone())
                    {
                        for (int i = 1; i <= anExactIntersector.NbPoints(); i++)
                        {
                            IntCurveSurface_IntersectionPoint aPoint = anExactIntersector.Point(i);

                            if ((aPoint.W() >= aTF) && (aPoint.W() <= aTL))
                            {
                                aDist2 = 0.0;
                                aTx = aPoint.W();
                            }
                        }
                    }
                    // modified by NIZHNY-MKK  Thu Jul 21 11:35:40 2005.END
                }
            }
            else
            {
                return theflag;
            }
        }

        double aBoundaryDist;

        aBoundaryDist = DistanceFunction(aTF) + myCriteria;
        if (aBoundaryDist * aBoundaryDist < aDist2)
        {
            aDist2 = aBoundaryDist * aBoundaryDist;
            aTx = aTF;
        }

        aBoundaryDist = DistanceFunction(aTL) + myCriteria;
        if (aBoundaryDist * aBoundaryDist < aDist2)
        {
            aDist2 = aBoundaryDist * aBoundaryDist;
            aTx = aTL;
        }

        double aParameter = (aTF + aTL) * 0.5;
        aBoundaryDist = DistanceFunction(aParameter) + myCriteria;
        if (aBoundaryDist * aBoundaryDist < aDist2)
        {
            aDist2 = aBoundaryDist * aBoundaryDist;
            aTx = aParameter;
        }

        if (aDist2 > myCriteria * myCriteria)
        {
            return theflag;
        }

        if (Math.Abs(aTx - aTF) < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return !theflag;
        }

        if (Math.Abs(aTx - aTL) < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return !theflag;
        }

        if (aTx > aTF && aTx < aTL)
        {
            return !theflag;
        }

        return theflag;
    }

    /// <summary>Launches the process.</summary>
    public void Perform()
    {
        int i, aNb;
        IntTools_CommonPrt aCommonPrt = new();
        //
        aCommonPrt.SetEdge1(myEdge);
        //
        myErrorStatus = 0;
        CheckData();
        if (myErrorStatus != 0)
        {
            return;
        }
        //
        myContext ??= new IntTools_Context();
        //
        myIsDone = false;
        myC.Initialize(myEdge);
        CurveKind aCurveType;
        aCurveType = myC.GetType();
        //
        // Prepare myCriteria
        double aFuzz = myFuzzyValue / 2.0;
        double aTolF = BRep_Tool.Tolerance(myFace) + aFuzz;
        double aTolE = BRep_Tool.Tolerance(myEdge) + aFuzz;
        if (aCurveType == BSplineCurve || aCurveType == BezierCurve)
        {
            //--- 5112
            double diff1 = (aTolE / aTolF);
            double diff2 = (aTolF / aTolE);
            if (diff1 > 100 || diff2 > 100)
            {
                myCriteria = Max(aTolE, aTolF);
            }
            else
            { //--- 5112
                myCriteria = 1.5 * aTolE + aTolF;
            }
        }
        else
        {
            myCriteria = aTolE + aTolF;
        }

        // OCCT copies the context's adaptor; the port shares it (adaptors are only read here).
        myS = myContext.SurfaceAdaptor(myFace);

        if (myQuickCoincidenceCheck)
        {
            if (IsCoincident())
            {
                aCommonPrt.SetType(TopAbs_EDGE);
                aCommonPrt.SetRange1(myRange.First(), myRange.Last());
                mySeqOfCommonPrts.Append(new IntTools_CommonPrt(aCommonPrt));
                myIsDone = true;
                return;
            }
        }
        //
        IntTools_BeanFaceIntersector anIntersector = new(myC, myS, aTolE, aTolF);
        anIntersector.SetBeanParameters(myRange.First(), myRange.Last());
        //
        anIntersector.SetContext(myContext);
        //
        anIntersector.Perform();

        if (anIntersector.MinimalSquareDistance() < RealLast())
        {
            myMinDistance = Math.Sqrt(anIntersector.MinimalSquareDistance());
        }

        if (!anIntersector.IsDone())
        {
            return;
        }

        for (int r = 1; r <= anIntersector.Result().Length(); r++)
        {
            IntTools_Range aRange = anIntersector.Result().Value(r);

            if (IsProjectable(IntTools_Tools.IntermediatePoint(aRange.First(), aRange.Last())))
            {
                aCommonPrt.SetRange1(aRange.First(), aRange.Last());
                mySeqOfCommonPrts.Append(new IntTools_CommonPrt(aCommonPrt));
            }
        }

        aNb = mySeqOfCommonPrts.Length();

        for (i = 1; i <= aNb; i++)
        {
            IntTools_CommonPrt aCP = mySeqOfCommonPrts.Value(i);
            //
            double aTx1, aTx2;
            Point3D aPx1, aPx2;
            //
            aCP.Range1(out aTx1, out aTx2);
            myC.D0(aTx1, out aPx1);
            myC.D0(aTx2, out aPx2);
            aCP.SetBoundingPoints(aPx1, aPx2);
            //
            MakeType(aCP);
        }
        {
            // Line\Cylinder's Common Parts treatment
            CurveKind aCType;
            SurfaceKind aSType;
            TopAbs_ShapeEnum aType;
            bool bIsTouch;
            double aTx = 0.0;

            aCType = myC.GetType();
            aSType = myS.GetType();

            if (aCType == Line && aSType == SurfaceKind.Cylinder)
            {
                for (i = 1; i <= aNb; i++)
                {
                    IntTools_CommonPrt aCP = mySeqOfCommonPrts.Value(i);
                    aType = aCP.Type();
                    if (aType == TopAbs_EDGE)
                    {
                        bIsTouch = CheckTouch(aCP, ref aTx);
                        if (bIsTouch)
                        {
                            aCP.SetType(TopAbs_VERTEX);
                            aCP.SetVertexParameter1(aTx);
                            // aCP.SetRange1 (aTx, aTx);
                        }
                    }
                    else if (aType == TopAbs_VERTEX)
                    {
                        bIsTouch = CheckTouchVertex(aCP, ref aTx);
                        if (bIsTouch)
                        {
                            aCP.SetVertexParameter1(aTx);
                            // aCP.SetRange1 (aTx, aTx);
                        }
                    }
                }
            }

            // Circle\Plane's Common Parts treatment

            if (aCType == Circle && aSType == Plane)
            {
                bool bIsCoplanar, bIsRadius;
                bIsCoplanar = IsCoplanar(myC, myS);
                bIsRadius = IsRadius(myC, myS, myCriteria);
                if (!bIsCoplanar && !bIsRadius)
                {
                    for (i = 1; i <= aNb; i++)
                    {
                        IntTools_CommonPrt aCP = mySeqOfCommonPrts.Value(i);
                        aType = aCP.Type();
                        if (aType == TopAbs_EDGE)
                        {
                            bIsTouch = CheckTouch(aCP, ref aTx);
                            if (bIsTouch)
                            {
                                aCP.SetType(TopAbs_VERTEX);
                                aCP.SetVertexParameter1(aTx);
                                // aCP.SetRange1 (aTx, aTx);
                            }
                        }
                        else if (aType == TopAbs_VERTEX)
                        {
                            bIsTouch = CheckTouchVertex(aCP, ref aTx);
                            if (bIsTouch)
                            {
                                aCP.SetVertexParameter1(aTx);
                                // aCP.SetRange1 (aTx, aTx);
                            }
                        }
                    }
                }
            }
        }
        myIsDone = true;
    }

    /// <summary>True, with the touching parameter in <paramref name="aTx"/>, if a vertex common part is a touch.</summary>
    protected bool CheckTouchVertex(IntTools_CommonPrt aCP, ref double aTx)
    {
        double aTF, aTL, Tol, U1f, U1l, V1f, V1l;
        double aEpsT, aDist2, aMinDist2, aTm, aDist2New;
        bool theflag = false;
        int aNbExt, i, iLower;
        CurveKind aType;
        //
        aCP.Range1(out aTF, out aTL);
        aType = myC.GetType();
        //
        aEpsT = 8.0e-5;
        if (aType == Line)
        {
            aEpsT = 9.0e-5;
        }
        //
        aTm = 0.5 * (aTF + aTL);
        aDist2 = DistanceFunction(aTm);
        aDist2 *= aDist2;

        Tol = global::Kerf.Engine.Foundation.Precision.PConfusion();

        Geom_Curve Curve = BRep_Tool.Curve(myC.Edge(), out _, out _)!;
        Geom_Surface Surface = BRep_Tool.Surface(myS.Face())!;

        Surface.Bounds(out U1f, out U1l, out V1f, out V1l);

        GeomAdaptor_Curve TheCurve = new(Curve, aTF, aTL);
        GeomAdaptor_Surface TheSurface = new(Surface, U1f, U1l, V1f, V1l);

        Extrema_ExtCS anExtrema = new(TheCurve, TheSurface, Tol, Tol);

        if (!anExtrema.IsDone())
        {
            return theflag;
        }
        if (anExtrema.IsParallel())
        {
            return theflag;
        }

        aNbExt = anExtrema.NbExt();
        if (aNbExt == 0)
        {
            return theflag;
        }

        iLower = 1;
        aMinDist2 = 1.0e100;
        for (i = 1; i <= aNbExt; ++i)
        {
            aDist2 = anExtrema.SquareDistance(i);
            if (aDist2 < aMinDist2)
            {
                aMinDist2 = aDist2;
                iLower = i;
            }
        }

        aDist2New = anExtrema.SquareDistance(iLower);

        if (aDist2New > aDist2)
        {
            aTx = aTm;
            return !theflag;
        }

        if (aDist2New > myCriteria * myCriteria)
        {
            return theflag;
        }

        anExtrema.Points(iLower, out Extrema_POnCurv aPOnC, out _);

        aTx = aPOnC.Parameter();
        //
        if (Math.Abs(aTx - aTF) < aEpsT)
        {
            return theflag;
        }

        if (Math.Abs(aTx - aTL) < aEpsT)
        {
            return theflag;
        }

        if (aTx > aTF && aTx < aTL)
        {
            return !theflag;
        }

        return theflag;
    }

    // ---- file-static functions of IntTools_EdgeFace.cxx

    private static bool IsCoplanar(BRepAdaptor_Curve aCurve, BRepAdaptor_Surface aSurface)
    {
        bool bFlag = false;

        CurveKind aCType;
        SurfaceKind aSType;

        aCType = aCurve.GetType();
        aSType = aSurface.GetType();

        if (aCType == Circle && aSType == Plane)
        {
            Circle3D aCirc = aCurve.Circle();
            Axis3D anAx1 = aCirc.Axis();
            Direction3D aDirAx1 = anAx1.Direction();

            Plane3D aPln = aSurface.Plane();
            Axis3D anAx = aPln.Axis();
            Direction3D aDirPln = anAx.Direction();

            bFlag = IntTools_Tools.IsDirsCoinside(aDirAx1, aDirPln);
        }
        return bFlag;
    }

    private static bool IsRadius(BRepAdaptor_Curve aCurve, BRepAdaptor_Surface aSurface, double aCriteria)
    {
        bool bFlag = false;

        CurveKind aCType;
        SurfaceKind aSType;

        aCType = aCurve.GetType();
        aSType = aSurface.GetType();

        if (aCType == Circle && aSType == Plane)
        {
            Circle3D aCirc = aCurve.Circle();
            Point3D aCenter = aCirc.Location();
            double aR = aCirc.Radius();
            Plane3D aPln = aSurface.Plane();
            double aD = aPln.Distance(aCenter);
            if (Math.Abs(aD - aR) < aCriteria)
            {
                return !bFlag;
            }
        }
        return bFlag;
    }

    /// <summary>
    /// OCCT's file-level <c>AdaptiveDiscret</c>: the number of discretisation points for an edge on a cylinder (at least
    /// <paramref name="iDiscret"/>). Not called in OCCT; ported for completeness.
    /// </summary>
    internal static int AdaptiveDiscret(int iDiscret, BRepAdaptor_Curve aCurve, BRepAdaptor_Surface aSurface)
    {
        int iDiscretNew;

        iDiscretNew = iDiscret;

        SurfaceKind aSType;

        aSType = aSurface.GetType();

        if (aSType == SurfaceKind.Cylinder)
        {
            double aELength, aRadius, dLR;

            aELength = IntTools.Length(aCurve.Edge());

            Cylinder aCylinder = aSurface.Cylinder();
            aRadius = aCylinder.Radius();
            dLR = 2 * aRadius;

            iDiscretNew = (int)(aELength / dLR);

            if (iDiscretNew < iDiscret)
            {
                iDiscretNew = iDiscret;
            }
        }
        return iDiscretNew;
    }
}
