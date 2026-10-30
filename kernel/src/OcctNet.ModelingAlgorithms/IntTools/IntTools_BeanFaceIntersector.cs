// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_BeanFaceIntersector.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_BeanFaceIntersector.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_SurfaceType;

namespace Occt;

/// <summary>
/// Computes the ranges of parameters on the curve of a bean (a part of an edge) that bound the parts of the bean lying
/// on the surface of a face, according to the edge's and the face's tolerances.
/// </summary>
/// <remarks>
/// <para>
/// The real boundaries of the face are not taken into account: most result parts lie only inside the region of the
/// surface, which includes the inside of the face, and parts out of this region can be excluded from the result.
/// </para>
/// <para>
/// A working object (plan section 6). On B-spline, Bezier and other surfaces it localises the solutions with boxes of
/// samples of the curve and the surface. OCCT caches the surface's sample boxes in the context
/// (<see cref="IntTools_Context.SurfaceData(in TopoDS_Face)"/>), keyed by sample only, so a run could reuse boxes another edge
/// enlarged by its own tolerance, and its result depended on what the context computed before. The port keys the
/// cached boxes by everything they are computed from and checks a box cached by an earlier run as a fresh one is
/// checked, so every run gives the result a fresh context gives (decision 6601; occt-issues.md, unit 6C).
/// </para>
/// <para>
/// The adaptors given to <see cref="Init(BRepAdaptor_Curve, BRepAdaptor_Surface, double, double)"/> are shared, not
/// copied as OCCT's value members are: the algorithm only evaluates them. <see cref="Init(in TopoDS_Edge, in
/// TopoDS_Face)"/> makes its own curve adaptor. OCCT's member <c>myProjector</c>, never used, is left out.
/// </para>
/// </remarks>
public class IntTools_BeanFaceIntersector
{
    private BRepAdaptor_Curve myCurve = new();
    private BRepAdaptor_Surface mySurface = new();
    private Geom_Surface? myTrsfSurface;
    private double myFirstParameter;
    private double myLastParameter;
    private double myUMinParameter;
    private double myUMaxParameter;
    private double myVMinParameter;
    private double myVMaxParameter;
    private double myBeanTolerance;
    private double myFaceTolerance;
    private double myCurveResolution;
    private double myCriteria;
    private readonly IntTools_MarkedRangeSet myRangeManager = new();
    private IntTools_Context? myContext;
    private readonly NCollection_Sequence<IntTools_Range> myResults = new();
    private bool myIsDone;
    private double myMinSqDistance;

    /// <summary>An empty algorithm.</summary>
    public IntTools_BeanFaceIntersector()
    {
        myFirstParameter = 0.0;
        myLastParameter = 0.0;
        myUMinParameter = 0.0;
        myUMaxParameter = 0.0;
        myVMinParameter = 0.0;
        myVMaxParameter = 0.0;
        myBeanTolerance = 0.0;
        myFaceTolerance = 0.0;
        myIsDone = false;
        myMinSqDistance = RealLast();
        myCriteria = Precision.Confusion();
        myCurveResolution = Precision.PConfusion();
    }

    /// <summary>
    /// The algorithm for the edge and the face. The parts of the edge on the face's surface that lie in a hole of the
    /// face are part of the result.
    /// </summary>
    public IntTools_BeanFaceIntersector(in TopoDS_Edge theEdge, in TopoDS_Face theFace)
        : this()
    {
        Init(theEdge, theFace);
    }

    /// <summary>The algorithm for the curve and the surface with the tolerances.</summary>
    public IntTools_BeanFaceIntersector(BRepAdaptor_Curve theCurve, BRepAdaptor_Surface theSurface,
                                        double theBeanTolerance, double theFaceTolerance)
        : this()
    {
        Init(theCurve, theSurface, theBeanTolerance, theFaceTolerance);
    }

    /// <summary>
    /// The algorithm for the curve and the surface with the tolerances, on the given parameter ranges (used for
    /// optimisation).
    /// </summary>
    public IntTools_BeanFaceIntersector(BRepAdaptor_Curve theCurve,
                                        BRepAdaptor_Surface theSurface,
                                        double theFirstParOnCurve,
                                        double theLastParOnCurve,
                                        double theUMinParameter,
                                        double theUMaxParameter,
                                        double theVMinParameter,
                                        double theVMaxParameter,
                                        double theBeanTolerance,
                                        double theFaceTolerance)
    {
        myFirstParameter = theFirstParOnCurve;
        myLastParameter = theLastParOnCurve;
        myUMinParameter = theUMinParameter;
        myUMaxParameter = theUMaxParameter;
        myVMinParameter = theVMinParameter;
        myVMaxParameter = theVMaxParameter;
        myBeanTolerance = theBeanTolerance;
        myFaceTolerance = theFaceTolerance;
        myIsDone = false;
        myMinSqDistance = RealLast();

        myCurve = theCurve;

        myCriteria = myBeanTolerance + myFaceTolerance;
        myCurveResolution = myCurve.Resolution(myCriteria);

        mySurface = theSurface;
        myTrsfSurface = mySurface.GeomSurfaceTransformed();
    }

    /// <summary>
    /// Initialises the algorithm for the edge and the face. The parts of the edge on the face's surface that lie in a
    /// hole of the face are part of the result.
    /// </summary>
    public void Init(in TopoDS_Edge theEdge, in TopoDS_Face theFace)
    {
        myContext ??= new IntTools_Context();
        //
        // OCCT re-initialises its own adaptor member; the port makes a new one, since the member may be a caller's.
        myCurve = new BRepAdaptor_Curve(theEdge);
        mySurface = myContext.SurfaceAdaptor(theFace);
        myTrsfSurface = mySurface.GeomSurfaceTransformed();
        myBeanTolerance = BRep_Tool.Tolerance(theEdge);
        myFaceTolerance = BRep_Tool.Tolerance(theFace);

        myCriteria = myBeanTolerance + myFaceTolerance + Precision.Confusion();
        myCurveResolution = myCurve.Resolution(myCriteria);

        SetSurfaceParameters(mySurface.FirstUParameter(),
                             mySurface.LastUParameter(),
                             mySurface.FirstVParameter(),
                             mySurface.LastVParameter());
        myResults.Clear();
    }

    /// <summary>Initialises the algorithm for the curve and the surface with the tolerances.</summary>
    public void Init(BRepAdaptor_Curve theCurve, BRepAdaptor_Surface theSurface, double theBeanTolerance,
                     double theFaceTolerance)
    {
        myCurve = theCurve;
        mySurface = theSurface;
        myTrsfSurface = mySurface.GeomSurfaceTransformed();
        myBeanTolerance = theBeanTolerance;
        myFaceTolerance = theFaceTolerance;

        myCriteria = myBeanTolerance + myFaceTolerance;
        myCurveResolution = myCurve.Resolution(myCriteria);

        SetSurfaceParameters(mySurface.FirstUParameter(),
                             mySurface.LastUParameter(),
                             mySurface.FirstVParameter(),
                             mySurface.LastVParameter());
        myResults.Clear();
    }

    /// <summary>
    /// Initialises the algorithm for the curve and the surface with the tolerances, on the given parameter ranges (used
    /// for optimisation).
    /// </summary>
    public void Init(BRepAdaptor_Curve theCurve,
                     BRepAdaptor_Surface theSurface,
                     double theFirstParOnCurve,
                     double theLastParOnCurve,
                     double theUMinParameter,
                     double theUMaxParameter,
                     double theVMinParameter,
                     double theVMaxParameter,
                     double theBeanTolerance,
                     double theFaceTolerance)
    {
        Init(theCurve, theSurface, theBeanTolerance, theFaceTolerance);
        SetBeanParameters(theFirstParOnCurve, theLastParOnCurve);
        SetSurfaceParameters(theUMinParameter, theUMaxParameter, theVMinParameter, theVMaxParameter);
    }

    /// <summary>Sets the intersection context.</summary>
    public void SetContext(IntTools_Context? theContext) => myContext = theContext;

    /// <summary>The intersection context.</summary>
    public IntTools_Context? Context() => myContext;

    /// <summary>Sets the range of the curve.</summary>
    public void SetBeanParameters(double theFirstParOnCurve, double theLastParOnCurve)
    {
        myFirstParameter = theFirstParOnCurve;
        myLastParameter = theLastParOnCurve;
    }

    /// <summary>Sets the ranges of the surface.</summary>
    public void SetSurfaceParameters(double theUMinParameter, double theUMaxParameter, double theVMinParameter,
                                     double theVMaxParameter)
    {
        myUMinParameter = theUMinParameter;
        myUMaxParameter = theUMaxParameter;
        myVMinParameter = theVMinParameter;
        myVMaxParameter = theVMaxParameter;
    }

    /// <summary>Launches the algorithm.</summary>
    public void Perform()
    {
        myIsDone = false;
        myResults.Clear();

        myContext ??= new IntTools_Context();

        // Fast computation of Line/Plane case
        if (myCurve.GetType() == GeomAbs_Line && mySurface.GetType() == GeomAbs_Plane)
        {
            ComputeLinePlane();
            return;
        }

        // Fast check on coincidence for analytic cases
        if (FastComputeAnalytic())
        {
            // no further computation is necessary
            myIsDone = true;
            return;
        }

        // Initialization of the range manager
        myRangeManager.SetBoundaries(myFirstParameter, myLastParameter, 0);

        // Check coincidence
        bool isCoincide = TestComputeCoinside();
        if (isCoincide)
        {
            myResults.Append(new IntTools_Range(myFirstParameter, myLastParameter));
            myIsDone = true;
            return;
        }

        // Perform intersection

        // try to find localized solution
        bool bLocalize = (!Precision.IsInfinite(myUMinParameter) && !Precision.IsInfinite(myUMaxParameter)
                          && !Precision.IsInfinite(myVMinParameter) && !Precision.IsInfinite(myVMaxParameter));
        bLocalize = bLocalize
                    && (mySurface.GetType() == GeomAbs_BezierSurface || mySurface.GetType() == GeomAbs_OtherSurface
                        || (mySurface.GetType() == GeomAbs_BSplineSurface
                            && (mySurface.UDegree() > 2 || mySurface.VDegree() > 2)
                            && (mySurface.NbUKnots() > 2 && mySurface.NbVKnots() > 2)));

        bool isLocalized = bLocalize && ComputeLocalized();

        // Perform real intersection
        if (!isLocalized)
        {
            ComputeAroundExactIntersection();

            ComputeUsingExtremum();

            ComputeNearRangeBoundaries();
        }

        myIsDone = true;

        // Treatment of the results
        for (int i = 1; i <= myRangeManager.Length(); i++)
        {
            if (myRangeManager.Flag(i) != 2)
            {
                continue;
            }

            IntTools_Range aRange = myRangeManager.Range(i);
            int iLastRange = myResults.Length();
            if (iLastRange > 0)
            {
                ref IntTools_Range aLastRange = ref myResults.ChangeValue(iLastRange);
                if (Math.Abs(aRange.First() - aLastRange.Last()) > Precision.PConfusion())
                {
                    myResults.Append(aRange);
                }
                else
                {
                    aLastRange.SetLast(aRange.Last());
                }
            }
            else
            {
                myResults.Append(aRange);
            }
        }
    }

    /// <summary>True if the algorithm succeeded.</summary>
    public bool IsDone() => myIsDone;

    /// <summary>The result ranges (the algorithm's own sequence; ported code only reads it).</summary>
    public NCollection_Sequence<IntTools_Range> Result() => myResults;

    /// <summary>Copies the result ranges into <paramref name="theResults"/>.</summary>
    public void Result(NCollection_Sequence<IntTools_Range> theResults) => theResults.Assign(myResults);

    /// <summary>The minimal square distance found between the edge and the face.</summary>
    public double MinimalSquareDistance() => myMinSqDistance;

    private double Distance(double theArg)
    {
        gp_Pnt aPoint = myCurve.Value(theArg);

        GeomAPI_ProjectPointOnSurf aProjector = myContext!.ProjPS(mySurface.Face());
        aProjector.Perform(aPoint);

        if (aProjector.IsDone() && aProjector.NbPoints() > 0)
        {
            return aProjector.LowerDistance();
        }
        //
        double aDistance = RealLast();

        for (int i = 0; i < 4; i++)
        {
            double anIsoParameter = (i == 0) ? myUMinParameter
                                    : ((i == 1) ? myUMaxParameter : ((i == 2) ? myVMinParameter : myVMaxParameter));
            double aMinParameter = (i < 2) ? myVMinParameter : myUMinParameter;
            double aMaxParameter = (i < 2) ? myVMaxParameter : myUMaxParameter;
            double aMidParameter = (aMinParameter + aMaxParameter) * 0.5;
            gp_Pnt aPointMin = (i < 2) ? mySurface.Value(anIsoParameter, aMinParameter)
                                       : mySurface.Value(aMinParameter, anIsoParameter);
            gp_Pnt aPointMax = (i < 2) ? mySurface.Value(anIsoParameter, aMaxParameter)
                                       : mySurface.Value(aMaxParameter, anIsoParameter);
            gp_Pnt aPointMid = (i < 2) ? mySurface.Value(anIsoParameter, aMidParameter)
                                       : mySurface.Value(aMidParameter, anIsoParameter);

            bool useMinMaxPoints = true;
            bool computeisoline = true;

            if (aPointMin.IsEqual(aPointMax, myCriteria) && aPointMin.IsEqual(aPointMid, myCriteria)
                && aPointMax.IsEqual(aPointMid, myCriteria))
            {
                computeisoline = false;
            }

            if (computeisoline)
            {
                Geom_Curve aCurve = (i < 2) ? myTrsfSurface!.UIso(anIsoParameter) : myTrsfSurface!.VIso(anIsoParameter);
                GeomAPI_ProjectPointOnCurve aProjectorOnCurve = new(aPoint, aCurve, aMinParameter, aMaxParameter);

                if (aProjectorOnCurve.NbPoints() > 0)
                {
                    useMinMaxPoints = false;

                    if (aDistance > aProjectorOnCurve.LowerDistance())
                    {
                        aDistance = aProjectorOnCurve.LowerDistance();
                    }
                }
            }

            if (useMinMaxPoints)
            {
                double aPPDistance = aPoint.Distance(aPointMin);
                aDistance = (aPPDistance < aDistance) ? aPPDistance : aDistance;
                aPPDistance = aPoint.Distance(aPointMax);
                aDistance = (aPPDistance < aDistance) ? aPPDistance : aDistance;
            }
        }
        return aDistance;
    }

    private double Distance(double theArg, ref double theUParameter, ref double theVParameter)
    {
        gp_Pnt aPoint = myCurve.Value(theArg);

        theUParameter = myUMinParameter;
        theVParameter = myVMinParameter;
        //
        double aDistance = RealLast();
        bool projectionfound = false;

        GeomAPI_ProjectPointOnSurf aProjector = myContext!.ProjPS(mySurface.Face());
        aProjector.Perform(aPoint);

        if (aProjector.IsDone() && aProjector.NbPoints() > 0)
        {
            aProjector.LowerDistanceParameters(out theUParameter, out theVParameter);
            aDistance = aProjector.LowerDistance();
            projectionfound = true;
        }

        if (!projectionfound)
        {
            //
            for (int i = 0; i < 4; i++)
            {
                double anIsoParameter = (i == 0) ? myUMinParameter
                                        : ((i == 1) ? myUMaxParameter : ((i == 2) ? myVMinParameter : myVMaxParameter));
                double aMinParameter = (i < 2) ? myVMinParameter : myUMinParameter;
                double aMaxParameter = (i < 2) ? myVMaxParameter : myUMaxParameter;
                double aMidParameter = (aMinParameter + aMaxParameter) * 0.5;
                gp_Pnt aPointMin = (i < 2) ? mySurface.Value(anIsoParameter, aMinParameter)
                                           : mySurface.Value(aMinParameter, anIsoParameter);
                gp_Pnt aPointMax = (i < 2) ? mySurface.Value(anIsoParameter, aMaxParameter)
                                           : mySurface.Value(aMaxParameter, anIsoParameter);
                gp_Pnt aPointMid = (i < 2) ? mySurface.Value(anIsoParameter, aMidParameter)
                                           : mySurface.Value(aMidParameter, anIsoParameter);

                bool useMinMaxPoints = true;
                bool computeisoline = true;

                if (aPointMin.IsEqual(aPointMax, myCriteria) && aPointMin.IsEqual(aPointMid, myCriteria)
                    && aPointMax.IsEqual(aPointMid, myCriteria))
                {
                    computeisoline = false;
                }

                if (computeisoline)
                {
                    Geom_Curve aCurve = (i < 2) ? myTrsfSurface!.UIso(anIsoParameter) : myTrsfSurface!.VIso(anIsoParameter);
                    GeomAPI_ProjectPointOnCurve aProjectorOnCurve = new(aPoint, aCurve, aMinParameter, aMaxParameter);

                    if (aProjectorOnCurve.NbPoints() > 0)
                    {
                        useMinMaxPoints = false;

                        if (aDistance > aProjectorOnCurve.LowerDistance())
                        {
                            theUParameter = (i <= 1) ? anIsoParameter : aProjectorOnCurve.LowerDistanceParameter();
                            theVParameter = (i >= 2) ? anIsoParameter : aProjectorOnCurve.LowerDistanceParameter();
                            aDistance = aProjectorOnCurve.LowerDistance();
                        }
                    }
                }

                if (useMinMaxPoints)
                {
                    double aPPDistance = aPoint.Distance(aPointMin);

                    if (aPPDistance < aDistance)
                    {
                        theUParameter = (i <= 1) ? anIsoParameter : aMinParameter;
                        theVParameter = (i >= 2) ? anIsoParameter : aMinParameter;
                        aDistance = aPPDistance;
                    }
                    aPPDistance = aPoint.Distance(aPointMax);

                    if (aPPDistance < aDistance)
                    {
                        theUParameter = (i <= 1) ? anIsoParameter : aMaxParameter;
                        theVParameter = (i >= 2) ? anIsoParameter : aMaxParameter;
                        aDistance = aPPDistance;
                    }
                }
            }
        }
        theUParameter = (myUMinParameter > theUParameter) ? myUMinParameter : theUParameter;
        theUParameter = (myUMaxParameter < theUParameter) ? myUMaxParameter : theUParameter;
        theVParameter = (myVMinParameter > theVParameter) ? myVMinParameter : theVParameter;
        theVParameter = (myVMaxParameter < theVParameter) ? myVMaxParameter : theVParameter;

        return aDistance;
    }

    private void ComputeAroundExactIntersection()
    {
        IntCurveSurface_HInter anExactIntersector = new();

        // OCCT copies the adaptors into new handles.
        Adaptor3d_Curve aCurve = myCurve.ShallowCopy();
        Adaptor3d_Surface aSurface = mySurface.ShallowCopy();

        anExactIntersector.Perform(aCurve, aSurface);

        if (anExactIntersector.IsDone())
        {
            int i = 0;

            if (anExactIntersector.NbPoints() > 1)
            {
                // To avoid unification of the intersection points in a single intersection
                // range, perform exact range search considering the lowest possible tolerance
                // for edge and face.
                myCriteria = 3 * Precision.Confusion();
                myCurveResolution = myCurve.Resolution(myCriteria);
            }

            for (i = 1; i <= anExactIntersector.NbPoints(); i++)
            {
                IntCurveSurface_IntersectionPoint aPoint = anExactIntersector.Point(i);

                if ((aPoint.W() >= myFirstParameter) && (aPoint.W() <= myLastParameter))
                {
                    bool UIsNotValid = ((myUMinParameter > aPoint.U()) || (aPoint.U() > myUMaxParameter));
                    bool VIsNotValid = ((myVMinParameter > aPoint.V()) || (aPoint.V() > myVMaxParameter));
                    bool solutionIsValid = !UIsNotValid && !VIsNotValid;
                    double U = aPoint.U();
                    double V = aPoint.V();

                    if (UIsNotValid || VIsNotValid)
                    {
                        bool bUCorrected = true;

                        if (UIsNotValid)
                        {
                            bUCorrected = false;
                            solutionIsValid = false;
                            //
                            if (mySurface.IsUPeriodic())
                            {
                                double aNewU, aUPeriod, aEps;
                                //
                                aUPeriod = mySurface.UPeriod();
                                aEps = Epsilon(aUPeriod);
                                //
                                GeomInt.AdjustPeriodic(U, myUMinParameter, myUMaxParameter, aUPeriod, out aNewU, out _, aEps);
                                solutionIsValid = true;
                                bUCorrected = true;
                                U = aNewU;
                            }
                        }
                        //   if(solutionIsValid && VIsNotValid) {
                        if (bUCorrected && VIsNotValid)
                        {
                            solutionIsValid = false;
                            //
                            if (mySurface.IsVPeriodic())
                            {
                                double aNewV, aVPeriod, aEps;
                                //
                                aVPeriod = mySurface.VPeriod();
                                aEps = Epsilon(aVPeriod);
                                //
                                GeomInt.AdjustPeriodic(V, myVMinParameter, myVMaxParameter, aVPeriod, out aNewV, out _, aEps);
                                solutionIsValid = true;
                                V = aNewV;
                            }
                        }
                    }

                    if (!solutionIsValid)
                    {
                        continue;
                    }

                    int aNbRanges = myRangeManager.Length();

                    ComputeRangeFromStartPoint(false, aPoint.W(), U, V);
                    ComputeRangeFromStartPoint(true, aPoint.W(), U, V);

                    if (aNbRanges == myRangeManager.Length())
                    {
                        SetEmptyResultRange(aPoint.W(), myRangeManager);
                    }
                    else
                    {
                        myMinSqDistance = 0.0;
                    }
                }
            }

            for (i = 1; i <= anExactIntersector.NbSegments(); i++)
            {
                IntCurveSurface_IntersectionSegment aSegment = anExactIntersector.Segment(i);
                IntCurveSurface_IntersectionPoint aPoint1, aPoint2;
                aSegment.Values(out aPoint1, out aPoint2);

                double aFirstParameter = (aPoint1.W() < myFirstParameter) ? myFirstParameter : aPoint1.W();
                double aLastParameter = (myLastParameter < aPoint2.W()) ? myLastParameter : aPoint2.W();

                myRangeManager.InsertRange(aFirstParameter, aLastParameter, 2);

                ComputeRangeFromStartPoint(false, aPoint1.W(), aPoint1.U(), aPoint1.V());
                ComputeRangeFromStartPoint(true, aPoint2.W(), aPoint2.U(), aPoint2.V());
                myMinSqDistance = 0.0;
            }
        }
    }

    /// <summary>
    /// The fast check of the edge's coincidence with the face when both are analytic; it also finds whether an
    /// intersection is possible. True if the computation is complete, false if it must go on.
    /// </summary>
    private bool FastComputeAnalytic()
    {
        GeomAbs_CurveType aCT = myCurve.GetType();
        if (aCT == GeomAbs_BezierCurve || aCT == GeomAbs_BSplineCurve || aCT == GeomAbs_OffsetCurve
            || aCT == GeomAbs_OtherCurve)
        {
            // not supported type
            return false;
        }

        bool isCoincide = false;
        bool hasIntersection = true;

        GeomAbs_SurfaceType aST = mySurface.GetType();

        // Plane - Circle/Ellipse/Hyperbola/Parabola
        if (aST == GeomAbs_Plane)
        {
            gp_Pln surfPlane = mySurface.Plane();

            gp_Dir aDir;
            gp_Pnt aPLoc;
            switch (aCT)
            {
                case GeomAbs_Circle:
                    {
                        aDir = myCurve.Circle().Axis().Direction();
                        aPLoc = myCurve.Circle().Location();
                        break;
                    }
                case GeomAbs_Ellipse:
                    {
                        aDir = myCurve.Ellipse().Axis().Direction();
                        aPLoc = myCurve.Ellipse().Location();
                        break;
                    }
                case GeomAbs_Hyperbola:
                    {
                        aDir = myCurve.Hyperbola().Axis().Direction();
                        aPLoc = myCurve.Hyperbola().Location();
                        break;
                    }
                case GeomAbs_Parabola:
                    {
                        aDir = myCurve.Parabola().Axis().Direction();
                        aPLoc = myCurve.Parabola().Location();
                        break;
                    }
                default:
                    return false;
            }

            double anAngle = aDir.Angle(surfPlane.Axis().Direction());
            if (anAngle > Precision.Angular())
            {
                return false;
            }

            hasIntersection = false;

            double aDist = surfPlane.Distance(aPLoc);
            isCoincide = aDist < myCriteria;
        }

        // Cylinder - Line/Circle
        else if (aST == GeomAbs_Cylinder)
        {
            gp_Cylinder aCylinder = mySurface.Cylinder();
            gp_Ax1 aCylAxis = aCylinder.Axis();
            gp_Dir aCylDir = aCylAxis.Direction();
            double aCylRadius = aCylinder.Radius();

            if (aCT == GeomAbs_Line)
            {
                gp_Lin aLin = myCurve.Line();
                if (!aLin.Direction().IsParallel(aCylDir, Precision.Angular()))
                {
                    return false;
                }

                hasIntersection = false;

                double aDist = Math.Abs(aLin.Distance(aCylAxis.Location()) - aCylRadius);
                isCoincide = (aDist < myCriteria);
            }

            else if (aCT == GeomAbs_Circle)
            {
                gp_Circ aCircle = myCurve.Circle();

                double anAngle = aCylDir.Angle(aCircle.Axis().Direction());
                if (anAngle > Precision.Angular())
                {
                    return false;
                }

                double aDistLoc = new gp_Lin(aCylAxis).Distance(aCircle.Location());
                double aDist = aDistLoc + Math.Abs(aCircle.Radius() - aCylRadius);
                isCoincide = (aDist < myCriteria);

                if (!isCoincide)
                {
                    hasIntersection = (aDistLoc - (aCircle.Radius() + aCylRadius)) < myCriteria
                                      && (Math.Abs(aCircle.Radius() - aCylRadius) - aDistLoc) < myCriteria;
                }
            }
        }

        // Sphere - Line
        else if (aST == GeomAbs_Sphere)
        {
            gp_Sphere aSph = mySurface.Sphere();
            gp_Pnt aSphLoc = aSph.Location();
            if (aCT == GeomAbs_Line)
            {
                gp_Lin aLin = myCurve.Line();
                double aDist = aLin.Distance(aSphLoc) - aSph.Radius();
                hasIntersection = aDist < myCriteria;
            }
        }

        // Check intermediate point
        if (isCoincide)
        {
            myResults.Append(new IntTools_Range(myFirstParameter, myLastParameter));
        }

        return isCoincide || !hasIntersection;
    }

    private void ComputeLinePlane()
    {
        double Tolang = 1.0e-9;
        gp_Pln P = mySurface.Plane();
        gp_Lin L = myCurve.Line();

        myIsDone = true;

        double A, B, C, D;
        double Al, Bl, Cl;
        double Dis, Direc;

        P.Coefficients(out A, out B, out C, out D);
        gp_Pnt Orig = L.Location();
        L.Direction().Coord(out Al, out Bl, out Cl);

        Direc = A * Al + B * Bl + C * Cl;
        Dis = A * Orig.X() + B * Orig.Y() + C * Orig.Z() + D;

        bool parallel = false, inplane = false;
        if (Math.Abs(Direc) < Tolang)
        {
            parallel = true;
            inplane = Math.Abs(Dis) < myCriteria;
        }
        else
        {
            gp_Pnt p1 = ElCLib.Value(myFirstParameter, L);
            gp_Pnt p2 = ElCLib.Value(myLastParameter, L);
            double d1 = A * p1.X() + B * p1.Y() + C * p1.Z() + D;
            if (d1 < 0)
            {
                d1 = -d1;
            }
            double d2 = A * p2.X() + B * p2.Y() + C * p2.Z() + D;
            if (d2 < 0)
            {
                d2 = -d2;
            }
            if (d1 <= myCriteria && d2 <= myCriteria)
            {
                inplane = true;
            }
        }

        if (inplane)
        {
            IntTools_Range aRange = new(myFirstParameter, myLastParameter);
            myResults.Append(aRange);
            return;
        }

        if (parallel)
        {
            return;
        }

        double t = -Dis / Direc;
        if (t < myFirstParameter || t > myLastParameter)
        {
            return;
        }

        gp_Pnt pint = new(Orig.X() + t * Al, Orig.Y() + t * Bl, Orig.Z() + t * Cl);

        double u, v;
        ElSLib.Parameters(P, pint, out u, out v);
        if (myUMinParameter > u || u > myUMaxParameter || myVMinParameter > v || v > myVMaxParameter)
        {
            return;
        }
        //
        // compute correct range on the edge
        double anAngle, aDt;
        gp_Dir aDL, aDP;
        //
        aDL = L.Position().Direction();
        aDP = P.Position().Direction();
        anAngle = Math.Abs(M_PI_2 - aDL.Angle(aDP));
        //
        aDt = IntTools_Tools.ComputeIntRange(myBeanTolerance, myFaceTolerance, anAngle);
        //
        double t1 = Max(myFirstParameter, t - aDt);
        double t2 = Min(myLastParameter, t + aDt);
        IntTools_Range aRangeV = new(t1, t2);
        myResults.Append(aRangeV);
    }

    private void ComputeUsingExtremum()
    {
        double Tol;
        Tol = Precision.PConfusion();
        Geom_Curve aCurve = BRep_Tool.Curve(myCurve.Edge(), out _, out _)!;

        for (int i = 1; i <= myRangeManager.Length(); i++)
        {

            if (myRangeManager.Flag(i) > 0)
            {
                continue;
            }

            IntTools_Range aParamRange = myRangeManager.Range(i);
            double anarg1 = aParamRange.First();
            double anarg2 = aParamRange.Last();

            if (anarg2 - anarg1 < Precision.PConfusion())
            {

                if (((i > 1) && (myRangeManager.Flag(i - 1) == 2))
                    || ((i < myRangeManager.Length()) && (myRangeManager.Flag(i + 1) == 2)))
                {
                    myRangeManager.SetFlag(i, 1);
                    continue;
                }
            }

            GeomAdaptor_Curve aGACurve = new(aCurve, anarg1, anarg2);
            Extrema_ExtCS anExtCS = new();
            anExtCS.Initialize(mySurface, myUMinParameter, myUMaxParameter, myVMinParameter, myVMaxParameter, Tol, Tol);
            double first = aCurve.FirstParameter(), last = aCurve.LastParameter();
            if (aCurve.IsPeriodic()
                || (anarg1 >= first - Precision.PConfusion() && anarg2 <= last + Precision.PConfusion()))
            {
                // Extrema_ExtCS anExtCS (aGACurve, aGASurface, Tol, Tol);
                anExtCS.Perform(aGACurve, anarg1, anarg2);
            }

            if (anExtCS.IsDone() && (anExtCS.NbExt() != 0 || anExtCS.IsParallel()))
            {
                int anOldNbRanges = myRangeManager.Length();

                if (anExtCS.IsParallel())
                {
                    double aSqDist = anExtCS.SquareDistance(1);
                    myMinSqDistance = Min(myMinSqDistance, aSqDist);
                    if (aSqDist < myCriteria * myCriteria)
                    {
                        double U1 = 0.0, V1 = 0.0, U2 = 0.0, V2 = 0.0;
                        double adistance1 = Distance(anarg1, ref U1, ref V1);
                        double adistance2 = Distance(anarg2, ref U2, ref V2);
                        bool validdistance1 = (adistance1 < myCriteria);
                        bool validdistance2 = (adistance2 < myCriteria);

                        if (validdistance1 && validdistance2)
                        {
                            myRangeManager.InsertRange(anarg1, anarg2, 2);
                            continue;
                        }
                        else
                        {
                            if (validdistance1)
                            {
                                ComputeRangeFromStartPoint(true, anarg1, U1, V1);
                            }
                            else
                            {
                                if (validdistance2)
                                {
                                    ComputeRangeFromStartPoint(false, anarg2, U2, V2);
                                }
                                else
                                {
                                    double a = anarg1;
                                    double b = anarg2;
                                    double da = adistance1;
                                    double db = adistance2;
                                    double asolution = a;
                                    bool found = false;

                                    while (((b - a) > myCurveResolution) && !found)
                                    {
                                        asolution = (a + b) * 0.5;
                                        double adist = Distance(asolution, ref U1, ref V1);

                                        if (adist < myCriteria)
                                        {
                                            found = true;
                                        }
                                        else
                                        {
                                            if (da < db)
                                            {
                                                b = asolution;
                                                db = adist;
                                            }
                                            else
                                            {
                                                a = asolution;
                                                da = adist;
                                            }
                                        }
                                    } // end while

                                    if (found)
                                    {
                                        ComputeRangeFromStartPoint(false, asolution, U1, V1);
                                        ComputeRangeFromStartPoint(true, asolution, U1, V1);
                                    }
                                    else
                                    {
                                        myRangeManager.SetFlag(i, 1);
                                    }
                                }
                            }
                        }
                    }
                    else
                    {
                        myRangeManager.SetFlag(i, 1);
                    }
                }
                else
                {
                    bool solutionfound = false;

                    for (int j = 1; j <= anExtCS.NbExt(); j++)
                    {
                        if (anExtCS.SquareDistance(j) < myCriteria * myCriteria)
                        {
                            anExtCS.Points(j, out Extrema_POnCurv p1, out Extrema_POnSurf p2);
                            double U, V;
                            p2.Parameter(out U, out V);

                            int aNbRanges = myRangeManager.Length();
                            ComputeRangeFromStartPoint(false, p1.Parameter(), U, V);
                            ComputeRangeFromStartPoint(true, p1.Parameter(), U, V);
                            solutionfound = true;

                            if (aNbRanges == myRangeManager.Length())
                            {
                                SetEmptyResultRange(p1.Parameter(), myRangeManager);
                            }
                        }

                        myMinSqDistance = Min(myMinSqDistance, anExtCS.SquareDistance(j));
                    }

                    if (!solutionfound)
                    {
                        myRangeManager.SetFlag(i, 1);
                    }
                }
                int adifference = myRangeManager.Length() - anOldNbRanges;

                if (adifference > 0)
                {
                    i += adifference;
                }
            }
        }
    }

    private void ComputeNearRangeBoundaries()
    {
        double U = myUMinParameter;
        double V = myVMinParameter;

        for (int i = 1; i <= myRangeManager.Length(); i++)
        {

            if (myRangeManager.Flag(i) > 0)
            {
                continue;
            }

            if ((i > 1) && (myRangeManager.Flag(i - 1) > 0))
            {
                continue;
            }

            IntTools_Range aParamRange = myRangeManager.Range(i);

            if (Distance(aParamRange.First(), ref U, ref V) < myCriteria)
            {
                int aNbRanges = myRangeManager.Length();

                if (i > 1)
                {
                    ComputeRangeFromStartPoint(false, aParamRange.First(), U, V, i - 1);
                }
                ComputeRangeFromStartPoint(true, aParamRange.First(), U, V, i + (myRangeManager.Length() - aNbRanges));

                if (aNbRanges == myRangeManager.Length())
                {
                    SetEmptyResultRange(aParamRange.First(), myRangeManager);
                }
            }
        }

        if (myRangeManager.Flag(myRangeManager.Length()) == 0)
        {
            IntTools_Range aParamRange = myRangeManager.Range(myRangeManager.Length());

            if (Distance(aParamRange.Last(), ref U, ref V) < myCriteria)
            {
                int aNbRanges = myRangeManager.Length();

                ComputeRangeFromStartPoint(false, aParamRange.Last(), U, V, myRangeManager.Length());

                if (aNbRanges == myRangeManager.Length())
                {
                    SetEmptyResultRange(aParamRange.Last(), myRangeManager);
                }
            }
        }
    }

    /// <summary>
    /// Computes a range from the start point at <paramref name="theParameter"/>, increasing the parameter on the curve
    /// if <paramref name="ToIncreaseParameter"/>, else decreasing it.
    /// </summary>
    private void ComputeRangeFromStartPoint(bool ToIncreaseParameter, double theParameter, double theUParameter,
                                            double theVParameter)
    {
        int aFoundIndex = myRangeManager.GetIndex(theParameter, ToIncreaseParameter);

        if (aFoundIndex == 0)
        {
            return;
        }

        ComputeRangeFromStartPoint(ToIncreaseParameter, theParameter, theUParameter, theVParameter, aFoundIndex);
    }

    /// <summary>
    /// Computes a range from the start point at <paramref name="theParameter"/>, which belongs to the range number
    /// <paramref name="theIndex"/>, increasing the parameter on the curve if <paramref name="ToIncreaseParameter"/>,
    /// else decreasing it.
    /// </summary>
    private void ComputeRangeFromStartPoint(bool ToIncreaseParameter, double theParameter, double theUParameter,
                                            double theVParameter, int theIndex)
    {
        if (myRangeManager.Flag(theIndex) > 0)
        {
            return;
        }

        int aValidIndex = theIndex;

        double aMinDelta = myCurveResolution * 0.5;
        double aDeltaRestrictor = 0.1 * (myLastParameter - myFirstParameter);

        if (aMinDelta > aDeltaRestrictor)
        {
            aMinDelta = aDeltaRestrictor * 0.5;
        }

        double tenOfMinDelta = aMinDelta * 10.0;
        double aDelta = myCurveResolution;

        double aCurPar = (ToIncreaseParameter) ? (theParameter + aDelta) : (theParameter - aDelta);
        double aPrevPar = theParameter;
        IntTools_Range aCurrentRange = myRangeManager.Range(aValidIndex);

        bool BoundaryCondition =
          (ToIncreaseParameter) ? (aCurPar > aCurrentRange.Last()) : (aCurPar < aCurrentRange.First());

        if (BoundaryCondition)
        {
            aCurPar = (ToIncreaseParameter) ? aCurrentRange.Last() : aCurrentRange.First();
            BoundaryCondition = false;
        }

        int loopcounter = 0; // necessary as infinite loop restricter
        double U = theUParameter;
        double V = theVParameter;
        bool anotherSolutionFound = false;

        bool isboundaryindex = false;
        bool isvalidindex = true;

        while ((aDelta >= aMinDelta) && (loopcounter <= 10))
        {
            bool pointfound = false;

            //
            gp_Pnt aPoint = myCurve.Value(aCurPar);
            Extrema_GenLocateExtPS anExtrema = new(mySurface, 1.0e-10, 1.0e-10);
            anExtrema.Perform(aPoint, U, V);

            if (anExtrema.IsDone())
            {
                if (anExtrema.SquareDistance() < myCriteria * myCriteria)
                {
                    Extrema_POnSurf aPOnSurf = anExtrema.Point();
                    aPOnSurf.Parameter(out U, out V);
                    pointfound = true;
                }
            }
            else
            {
                pointfound = (Distance(aCurPar) < myCriteria);
            }

            if (pointfound)
            {
                aPrevPar = aCurPar;
                anotherSolutionFound = true;

                if (BoundaryCondition && (isboundaryindex || !isvalidindex))
                {
                    break;
                }
            }
            else
            {
                aDeltaRestrictor = aDelta;
            }

            // if point found decide to increase aDelta using derivative of distance function
            //

            aDelta = (pointfound) ? (aDelta * 2.0) : (aDelta * 0.5);
            aDelta = (aDelta < aDeltaRestrictor) ? aDelta : aDeltaRestrictor;

            aCurPar = (ToIncreaseParameter) ? (aPrevPar + aDelta) : (aPrevPar - aDelta);

            // prevent infinite loop when (aPrevPar +/- aDelta) == aPrevPar == 0.
            //

            if (aCurPar == aPrevPar)
            {
                break;
            }

            BoundaryCondition =
              (ToIncreaseParameter) ? (aCurPar > aCurrentRange.Last()) : (aCurPar < aCurrentRange.First());

            isboundaryindex = false;
            isvalidindex = true;

            if (BoundaryCondition)
            {
                isboundaryindex = ((!ToIncreaseParameter && (aValidIndex == 1))
                                   || (ToIncreaseParameter && (aValidIndex == myRangeManager.Length())));

                if (!isboundaryindex)
                {

                    if (pointfound)
                    {
                        int aFlag = (ToIncreaseParameter) ? myRangeManager.Flag(aValidIndex + 1)
                                                          : myRangeManager.Flag(aValidIndex - 1);

                        if (aFlag == 0)
                        {
                            aValidIndex = (ToIncreaseParameter) ? (aValidIndex + 1) : (aValidIndex - 1);
                            aCurrentRange = myRangeManager.Range(aValidIndex);

                            if ((ToIncreaseParameter && (aCurPar > aCurrentRange.Last()))
                                || (!ToIncreaseParameter && (aCurPar < aCurrentRange.First())))
                            {
                                aCurPar = (aCurrentRange.First() + aCurrentRange.Last()) * 0.5;
                                aDelta *= 0.5;
                            }
                        }
                        else
                        {
                            isvalidindex = false;
                            aCurPar = (ToIncreaseParameter) ? aCurrentRange.Last() : aCurrentRange.First();
                        }
                    }
                }
                else
                {
                    aCurPar = (ToIncreaseParameter) ? aCurrentRange.Last() : aCurrentRange.First();
                }

                if (aDelta < tenOfMinDelta)
                {
                    loopcounter++;
                }
                else
                {
                    loopcounter = 0;
                }
            } // if(BoundaryCondition)
        }

        if (anotherSolutionFound)
        {
            if (ToIncreaseParameter)
            {
                myRangeManager.InsertRange(theParameter, aPrevPar, 2);
            }
            else
            {
                myRangeManager.InsertRange(aPrevPar, theParameter, 2);
            }
        }
    }

    private static bool SetEmptyResultRange(double theParameter, IntTools_MarkedRangeSet theMarkedRange)
    {

        NCollection_Sequence<int> anIndices = theMarkedRange.GetIndices(theParameter);
        bool add = (anIndices.Length() > 0);

        for (int k = 1; k <= anIndices.Length(); k++)
        {
            if (theMarkedRange.Flag(anIndices.Value(k)) == 2)
            {
                add = false;
                break;
            }
        }

        if (add)
        {
            theMarkedRange.InsertRange(theParameter, theParameter, 2);
        }

        return add;
    }

    private bool LocalizeSolutions(in IntTools_CurveRangeSample theCurveRange,
                                   in Bnd_Box theBoxCurve,
                                   in IntTools_SurfaceRangeSample theSurfaceRange,
                                   in Bnd_Box theBoxSurface,
                                   IntTools_CurveRangeLocalizeData theCurveData,
                                   IntTools_SurfaceRangeLocalizeData theSurfaceData,
                                   NCollection_List<IntTools_CurveRangeSample> theListCurveRange,
                                   NCollection_List<IntTools_SurfaceRangeSample> theListSurfaceRange)
    {
        int tIt = 0, uIt = 0, vIt = 0;

        //
        IntTools_CurveRangeSample aRootRangeC = new(0);
        aRootRangeC.SetDepth(0);
        IntTools_SurfaceRangeSample aRootRangeS = new(0, 0, 0, 0);

        Bnd_Box aMainBoxC = theBoxCurve;
        Bnd_Box aMainBoxS = theBoxSurface;
        bool bMainBoxFoundS = false;
        bool bMainBoxFoundC = false;
        //
        NCollection_List<IntTools_CurveRangeSample> aListCurveRangeFound = new();
        NCollection_List<IntTools_SurfaceRangeSample> aListSurfaceRangeFound = new();

        IntTools_Range aRangeC = theCurveRange.GetRange(myFirstParameter, myLastParameter, theCurveData.GetNbSample());
        double localdiffC = (aRangeC.Last() - aRangeC.First()) / theCurveData.GetNbSample();

        double aCurPar = aRangeC.First();
        double aPrevPar = aRangeC.First();
        int aCurIndexInit = theCurveRange.GetRangeIndexDeeper(theCurveData.GetNbSample());

        NCollection_List<int> aListCToAvoid = new();
        bool bGlobalCheckDone = false;
        //

        //

        int aCurIndexU = theSurfaceRange.GetRangeIndexUDeeper(theSurfaceData.GetNbSampleU());

        int aCurIndexVInit = theSurfaceRange.GetRangeIndexVDeeper(theSurfaceData.GetNbSampleV());
        IntTools_Range aRangeV = theSurfaceRange.GetRangeV(myVMinParameter, myVMaxParameter, theSurfaceData.GetNbSampleV());

        //
        IntTools_Range aRangeU = theSurfaceRange.GetRangeU(myUMinParameter, myUMaxParameter, theSurfaceData.GetNbSampleU());
        double aCurParU = aRangeU.First();
        double aLocalDiffU = (aRangeU.Last() - aRangeU.First()) / theSurfaceData.GetNbSampleU();

        double aPrevParU = aCurParU;
        double aLocalDiffV = (aRangeV.Last() - aRangeV.First()) / theSurfaceData.GetNbSampleV();

        // ranges check.begin
        bool bAllowSamplingC = true;
        bool bAllowSamplingU = true;
        bool bAllowSamplingV = true;

        // check
        CheckSampling(theCurveRange,
                      theSurfaceRange,
                      theCurveData,
                      theSurfaceData,
                      localdiffC,
                      aLocalDiffU,
                      aLocalDiffV,
                      out bAllowSamplingC,
                      out bAllowSamplingU,
                      out bAllowSamplingV);
        //

        if (!bAllowSamplingC && !bAllowSamplingU && !bAllowSamplingV)
        {
            theListCurveRange.Append(theCurveRange);
            theListSurfaceRange.Append(theSurfaceRange);
            return true;
        }
        // ranges check.end

        // init template. begin
        IntTools_CurveRangeSample aNewRangeCTemplate = new();

        if (!bAllowSamplingC)
        {
            aNewRangeCTemplate = theCurveRange;
            aCurIndexInit = theCurveRange.GetRangeIndex();
            localdiffC = (aRangeC.Last() - aRangeC.First());
        }
        else
        {
            aNewRangeCTemplate.SetDepth(theCurveRange.GetDepth() + 1);
            aNewRangeCTemplate.SetRangeIndex(aCurIndexInit);
        }

        IntTools_SurfaceRangeSample aNewRangeSTemplate = theSurfaceRange;

        if (bAllowSamplingU)
        {
            aNewRangeSTemplate.SetDepthU(theSurfaceRange.GetDepthU() + 1);
        }
        else
        {
            aCurIndexU = aNewRangeSTemplate.GetIndexU();
            aLocalDiffU = aRangeU.Last() - aRangeU.First();
        }

        if (bAllowSamplingV)
        {
            aNewRangeSTemplate.SetDepthV(theSurfaceRange.GetDepthV() + 1);
        }
        else
        {
            aCurIndexVInit = theSurfaceRange.GetIndexV();
            aLocalDiffV = aRangeV.Last() - aRangeV.First();
        }
        // init template. end

        bool bHasOut = false;
        int nbU = (bAllowSamplingU) ? theSurfaceData.GetNbSampleU() : 1;
        int nbV = (bAllowSamplingV) ? theSurfaceData.GetNbSampleV() : 1;
        int nbC = (bAllowSamplingC) ? theCurveData.GetNbSample() : 1;

        for (uIt = 1; uIt <= nbU; uIt++, aCurIndexU++, aPrevParU = aCurParU)
        {
            aCurParU += aLocalDiffU;

            double aCurParV = aRangeV.First();
            double aPrevParV = aCurParV;
            int aCurIndexV = aCurIndexVInit;

            bool bHasOutV = false;

            // ///////
            for (vIt = 1; vIt <= nbV; vIt++, aCurIndexV++, aPrevParV = aCurParV)
            {

                aCurParV += aLocalDiffV;

                // //////////////
                //
                IntTools_SurfaceRangeSample aNewRangeS = aNewRangeSTemplate;

                if (bAllowSamplingU)
                {
                    aNewRangeS.SetIndexU(aCurIndexU);
                }

                if (bAllowSamplingV)
                {
                    aNewRangeS.SetIndexV(aCurIndexV);
                }

                if (theSurfaceData.IsRangeOut(aNewRangeS))
                {
                    bHasOutV = true;
                    continue;
                }

                // ///////

                Bnd_Box aBoxS = new();

                if (!theSurfaceData.FindBox(aNewRangeS, ref aBoxS))
                {

                    if (mySurface.GetType() == GeomAbs_BSplineSurface)
                    {
                        aBoxS = GetSurfaceBox(mySurface, aPrevParU, aCurParU, aPrevParV, aCurParV, myCriteria, theSurfaceData);
                    }
                    else
                    {
                        BndLib_AddSurface.Add(mySurface, aPrevParU, aCurParU, aPrevParV, aCurParV, myCriteria, ref aBoxS);
                    }
                    // 	Bnd_Box aMainBoxC;

                    if (!bMainBoxFoundC && theCurveData.FindBox(aRootRangeC, ref aMainBoxC))
                    {
                        bMainBoxFoundC = true;
                    }

                    if (aBoxS.IsOut(aMainBoxC))
                    {
                        theSurfaceData.AddOutRange(aNewRangeS);
                        bHasOutV = true;
                        continue;
                    }
                    // 	}
                    theSurfaceData.AddBox(aNewRangeS, aBoxS);
                }
                else
                {
                    // OCCT.NET (decision 6601): a box cached by an earlier run is checked against this run's curve box as
                    // a fresh box is, so the run gives the result of a fresh context. OCCT skips the check here.
                    if (!bMainBoxFoundC && theCurveData.FindBox(aRootRangeC, ref aMainBoxC))
                    {
                        bMainBoxFoundC = true;
                    }

                    if (aBoxS.IsOut(aMainBoxC))
                    {
                        theSurfaceData.AddOutRange(aNewRangeS);
                        bHasOutV = true;
                        continue;
                    }
                }

                if (aBoxS.IsOut(theBoxCurve))
                {
                    bHasOutV = true;
                    continue;
                }

                NCollection_List<Bnd_Box> aListOfBox = new();
                NCollection_List<int> aListOfIndex = new();

                bool bHasOutC = false;
                int aCurIndex = aCurIndexInit;

                // ////////////////////////////
                aCurPar = aRangeC.First();
                aPrevPar = aRangeC.First();
                IntTools_CurveRangeSample aCurRangeC = aNewRangeCTemplate;

                for (tIt = 1; tIt <= nbC; tIt++, aCurIndex++, aPrevPar = aCurPar)
                {

                    aCurPar += localdiffC;

                    // ignore already computed. begin
                    bool bFound = false;
                    NCollection_List<int>.Iterator anItToAvoid = new(aListCToAvoid);

                    for (; anItToAvoid.More(); anItToAvoid.Next())
                    {
                        if (tIt == anItToAvoid.Value())
                        {
                            bFound = true;
                            break;
                        }
                    }

                    if (!bFound)
                    {
                        if (bAllowSamplingC)
                        {
                            aCurRangeC.SetRangeIndex(aCurIndex);
                        }
                        bFound = theCurveData.IsRangeOut(aCurRangeC);
                    }

                    if (bFound)
                    {
                        bHasOutC = true;
                        continue;
                    }
                    // ignore already computed. end

                    // compute Box
                    Bnd_Box aBoxC = new();

                    if (!theCurveData.FindBox(aCurRangeC, ref aBoxC))
                    {
                        BndLib_Add3dCurve.Add(myCurve, aPrevPar, aCurPar, myCriteria, ref aBoxC);

                        //   Bnd_Box aMainBoxS;

                        if (!bMainBoxFoundS && theSurfaceData.FindBox(aRootRangeS, ref aMainBoxS))
                        {
                            bMainBoxFoundS = true;
                        }
                        if (aBoxC.IsOut(aMainBoxS))
                        {
                            theCurveData.AddOutRange(aCurRangeC);
                            bHasOutC = true;
                            continue;
                        }
                        //   }
                        theCurveData.AddBox(aCurRangeC, aBoxC);
                    }

                    if (!bGlobalCheckDone && aBoxC.IsOut(theBoxSurface))
                    {
                        aListCToAvoid.Append(tIt);
                        bHasOutC = true;
                        continue;
                    }

                    if (aBoxC.IsOut(aBoxS))
                    {
                        bHasOutV = true;
                        bHasOutC = true;
                        continue;
                    }
                    //

                    aListOfIndex.Append(tIt);
                    aListOfBox.Append(aBoxC);
                } // end for(tIt...)

                bGlobalCheckDone = true;

                if (bHasOutC)
                {
                    bHasOutV = true;
                }

                // //////////////
                //

                IntTools_CurveRangeSample aNewRangeC = aNewRangeCTemplate;

                aCurIndex = aCurIndexInit;
                NCollection_List<int>.Iterator anItI = new(aListOfIndex);
                NCollection_List<Bnd_Box>.Iterator anItBox = new(aListOfBox);
                bool bUseOldC = false;
                bool bUseOldS = false;
                bool bCheckSize = !bHasOutC;

                for (; anItI.More() && anItBox.More(); anItI.Next(), anItBox.Next())
                {
                    aCurIndex = aCurIndexInit + anItI.Value() - 1;

                    bUseOldS = false;

                    if (bAllowSamplingC)
                    {
                        aNewRangeC.SetRangeIndex(aCurIndex);
                    }

                    if (bCheckSize)
                    {

                        if ((theCurveRange.GetDepth() == 0) || (theSurfaceRange.GetDepthU() == 0)
                            || (theSurfaceRange.GetDepthV() == 0))
                        {
                            bHasOutC = true;
                            bHasOutV = true;
                        }
                        else if ((theCurveRange.GetDepth() < 4) && (theSurfaceRange.GetDepthU() < 4)
                                 && (theSurfaceRange.GetDepthV() < 4))
                        {
                            Bnd_Box aBoxC = anItBox.Value();

                            if (!aBoxC.IsWhole() && !aBoxS.IsWhole())
                            {
                                double aDiagC = aBoxC.SquareExtent();
                                double aDiagS = aBoxS.SquareExtent();

                                if (aDiagC < aDiagS)
                                {
                                    if ((aDiagC * 10.0) < aDiagS)
                                    {
                                        bUseOldC = true;
                                        bHasOutC = true;
                                        bHasOutV = true;
                                        break;
                                    }
                                }
                                else
                                {
                                    if ((aDiagS * 10.0) < aDiagC)
                                    {
                                        bUseOldS = true;
                                        bHasOutC = true;
                                        bHasOutV = true;
                                    }
                                }
                            }
                        }
                    }

                    if (!bHasOutC)
                    {
                        aListCurveRangeFound.Append(aNewRangeC);
                        aListSurfaceRangeFound.Append(aNewRangeS);
                    }
                    else
                    {

                        //   if(bUseOldS || bAllowSamplingU || bAllowSamplingV) {
                        //     theSurfaceData.AddBox(aNewRangeS, aBoxS);
                        //   }

                        if (bUseOldS && aNewRangeC.IsEqual(theCurveRange))
                        {
                            return false;
                        }

                        if (!LocalizeSolutions(aNewRangeC,
                                               anItBox.Value(),
                                               ((bUseOldS) ? theSurfaceRange : aNewRangeS),
                                               ((bUseOldS) ? theBoxSurface : aBoxS),
                                               theCurveData,
                                               theSurfaceData,
                                               theListCurveRange,
                                               theListSurfaceRange))
                        {
                            return false;
                        }
                    }
                }
                // end (tIt...)
                aListOfIndex.Clear();
                aListOfBox.Clear();

                if (bHasOutV)
                {
                    // 	theSurfaceData.AddBox(aNewRangeS, aBoxS);

                    if (bUseOldC && bAllowSamplingC && (bAllowSamplingU || bAllowSamplingV))
                    {
                        if (!LocalizeSolutions(theCurveRange,
                                               theBoxCurve,
                                               aNewRangeS,
                                               aBoxS,
                                               theCurveData,
                                               theSurfaceData,
                                               theListCurveRange,
                                               theListSurfaceRange))
                        {
                            return false;
                        }
                    }
                }
            } // end for (vIt...)

            if (bHasOutV)
            {
                bHasOut = true;
            }
        }

        if (!bHasOut)
        {
            theListCurveRange.Append(theCurveRange);
            theListSurfaceRange.Append(theSurfaceRange);
        }
        else
        {
            NCollection_List<IntTools_CurveRangeSample>.Iterator anIt1 = new(aListCurveRangeFound);
            NCollection_List<IntTools_SurfaceRangeSample>.Iterator anIt2 = new(aListSurfaceRangeFound);

            for (; anIt1.More() && anIt2.More(); anIt1.Next(), anIt2.Next())
            {
                theListCurveRange.Append(anIt1.Value());
                theListSurfaceRange.Append(anIt2.Value());
            }
        }
        return true;
    }

    private bool ComputeLocalized()
    {
        double Tol = Precision.PConfusion();

        IntTools_SurfaceRangeSample aSurfaceRange = new(0, 0, 0, 0);
        double dMinU = 10.0 * Precision.PConfusion();
        double dMinV = dMinU;
        // (OCCT builds an unused IntTools_SurfaceRangeLocalizeData aSurfaceDataInit(3, 3, dMinU, dMinV) here.)
        //
        // OCCT.NET (decision 6601): the face's data keyed by what its boxes are computed from (OCCT: by the face only).
        IntTools_SurfaceRangeLocalizeData aSurfaceData =
          myContext!.SurfaceData(mySurface.Face(), myCriteria, myFaceTolerance, myUMinParameter, myUMaxParameter,
                                 myVMinParameter, myVMaxParameter);
        aSurfaceData.RemoveRangeOutAll();
        aSurfaceData.ClearGrid();

        Bnd_Box FBox = new();
        bool bFBoxFound = aSurfaceData.FindBox(aSurfaceRange, ref FBox);

        if (mySurface.GetType() == GeomAbs_BSplineSurface)
        {
            Geom_BSplineSurface aBsplineSurface = (Geom_BSplineSurface)myTrsfSurface!;
            ComputeGridPoints(mySurface,
                              aBsplineSurface,
                              myUMinParameter,
                              myUMaxParameter,
                              myVMinParameter,
                              myVMaxParameter,
                              myFaceTolerance,
                              aSurfaceData);

            if (!bFBoxFound)
            {
                FBox = GetSurfaceBox(mySurface,
                                     myUMinParameter,
                                     myUMaxParameter,
                                     myVMinParameter,
                                     myVMaxParameter,
                                     myCriteria,
                                     aSurfaceData);
                aSurfaceData.AddBox(aSurfaceRange, FBox);
            }
        }
        else if (!bFBoxFound)
        {

            BndLib_AddSurface.Add(mySurface,
                                  myUMinParameter,
                                  myUMaxParameter,
                                  myVMinParameter,
                                  myVMaxParameter,
                                  myFaceTolerance,
                                  ref FBox);
            aSurfaceData.AddBox(aSurfaceRange, FBox);
        }

        Bnd_Box EBox = new();

        BndLib_Add3dCurve.Add(myCurve.Trim(myFirstParameter, myLastParameter, Precision.PConfusion()),
                              myBeanTolerance,
                              ref EBox);

        if (EBox.IsOut(FBox))
        {
            for (int i = 1; i <= myRangeManager.Length(); i++)
            {
                myRangeManager.SetFlag(i, 1);
            }
            aSurfaceData.ClearGrid();

            return true;
        }

        NCollection_List<IntTools_CurveRangeSample> aListCurveRange = new();
        NCollection_List<IntTools_SurfaceRangeSample> aListSurfaceRange = new();

        IntTools_CurveRangeSample aCurveRange = new(0);
        aCurveRange.SetDepth(0);
        int nbSampleC = 3;
        int nbSampleU = aSurfaceData.GetNbSampleU();
        int nbSampleV = aSurfaceData.GetNbSampleV();
        double dMinC = 10.0 * myCurveResolution;
        NCollection_List<IntTools_CurveRangeSample> aListOut = new();

        // check
        bool bAllowSamplingC = true;
        bool bAllowSamplingU = true;
        bool bAllowSamplingV = true;
        IntTools_CurveRangeLocalizeData aCurveDataTmp = new(nbSampleC, dMinC);
        IntTools_SurfaceRangeLocalizeData aSurfaceDataTmp = new(nbSampleU, nbSampleV, dMinU, dMinV);

        CheckSampling(aCurveRange,
                      aSurfaceRange,
                      aCurveDataTmp,
                      aSurfaceDataTmp,
                      myLastParameter - myFirstParameter,
                      myUMaxParameter - myUMinParameter,
                      myVMaxParameter - myVMinParameter,
                      out bAllowSamplingC,
                      out bAllowSamplingU,
                      out bAllowSamplingV);

        {
            IntTools_CurveRangeLocalizeData aCurveData = new(nbSampleC, dMinC);

            aCurveData.AddBox(aCurveRange, EBox);

            if (!LocalizeSolutions(aCurveRange,
                                   EBox,
                                   aSurfaceRange,
                                   FBox,
                                   aCurveData,
                                   aSurfaceData,
                                   aListCurveRange,
                                   aListSurfaceRange))
            {
                aSurfaceData.ClearGrid();

                return false;
            }
            NCollection_List<IntTools_CurveRangeSample> aListCurveRangeSort = new();
            NCollection_List<IntTools_SurfaceRangeSample> aListSurfaceRangeSort = new();

            MergeSolutions(aListCurveRange, aListSurfaceRange, aListCurveRangeSort, aListSurfaceRangeSort);

            NCollection_List<IntTools_CurveRangeSample>.Iterator anItC = new(aListCurveRangeSort);
            NCollection_List<IntTools_SurfaceRangeSample>.Iterator anItS = new(aListSurfaceRangeSort);
            IntTools_SurfaceRangeSample aRangeSPrev = new();

            Extrema_GenExtCS anExtremaGen = new();

            for (; anItC.More() && anItS.More(); anItC.Next(), anItS.Next())
            {

                IntTools_Range aRangeC = new(myFirstParameter, myLastParameter);

                if (bAllowSamplingC)
                {
                    aRangeC = anItC.Value().GetRange(myFirstParameter, myLastParameter, nbSampleC);
                }

                IntTools_Range aRangeU = new(myUMinParameter, myUMaxParameter);

                if (bAllowSamplingU)
                {
                    aRangeU = anItS.Value().GetRangeU(myUMinParameter, myUMaxParameter, nbSampleU);
                }

                IntTools_Range aRangeV = new(myVMinParameter, myVMaxParameter);

                if (bAllowSamplingV)
                {
                    aRangeV = anItS.Value().GetRangeV(myVMinParameter, myVMaxParameter, nbSampleV);
                }

                double anarg1 = aRangeC.First(), anarg2 = aRangeC.Last();

                bool bFound = false;

                int nMinIndex = myRangeManager.Length();
                int nMaxIndex = -1;
                NCollection_Sequence<int> anInds1 = myRangeManager.GetIndices(anarg1);
                int indIt = 1;

                for (indIt = 1; indIt <= anInds1.Length(); indIt++)
                {
                    int nIndex = anInds1.Value(indIt);
                    nMinIndex = (nMinIndex > nIndex) ? nIndex : nMinIndex;
                    nMaxIndex = (nMaxIndex < nIndex) ? nIndex : nMaxIndex;
                }

                for (indIt = nMinIndex; indIt <= nMaxIndex; indIt++)
                {
                    if (myRangeManager.Flag(indIt) == 2)
                    {
                        bFound = true;
                        break;
                    }
                }

                if (bFound)
                {
                    continue;
                }
                nMinIndex = (nMaxIndex >= 0) ? nMaxIndex : nMinIndex;
                NCollection_Sequence<int> anInds2 = myRangeManager.GetIndices(anarg2);

                for (indIt = 1; indIt <= anInds2.Length(); indIt++)
                {
                    int nIndex = anInds2.Value(indIt);
                    nMinIndex = (nMinIndex > nIndex) ? nIndex : nMinIndex;
                    nMaxIndex = (nMaxIndex < nIndex) ? nIndex : nMaxIndex;
                }

                for (indIt = nMinIndex; indIt <= nMaxIndex; indIt++)
                {
                    if (myRangeManager.Flag(indIt) == 2)
                    {
                        bFound = true;
                        break;
                    }
                }

                if (bFound)
                {
                    continue;
                }

                double parUF = aRangeU.First(), parUL = aRangeU.Last();
                double parVF = aRangeV.First(), parVL = aRangeV.Last();

                if (aRangeSPrev.IsEqual(anItS.Value()))
                {
                    anExtremaGen.Perform(myCurve, 10, anarg1, anarg2, Tol);
                }
                else
                {
                    anExtremaGen.Initialize(mySurface, 10, 10, parUF, parUL, parVF, parVL, Tol);
                    anExtremaGen.Perform(myCurve, 10, anarg1, anarg2, Tol);
                }

                if (anExtremaGen.IsDone() && (anExtremaGen.NbExt() > 0))
                {

                    for (int j = 1; j <= anExtremaGen.NbExt(); j++)
                    {

                        if (anExtremaGen.SquareDistance(j) < myCriteria * myCriteria)
                        {

                            Extrema_POnCurv p1;
                            Extrema_POnSurf p2;
                            p1 = anExtremaGen.PointOnCurve(j);
                            p2 = anExtremaGen.PointOnSurface(j);
                            double U, V, T;
                            T = p1.Parameter();
                            p2.Parameter(out U, out V);

                            if (myCurve.IsPeriodic())
                            {
                                T = ElCLib.InPeriod(T, anarg1, anarg1 + myCurve.Period());
                            }
                            if (mySurface.IsUPeriodic())
                            {
                                U = ElCLib.InPeriod(U, parUF, parUF + mySurface.UPeriod());
                            }
                            if (mySurface.IsVPeriodic())
                            {
                                V = ElCLib.InPeriod(V, parVF, parVF + mySurface.VPeriod());
                            }

                            // To avoid occasional going out of boundaries because of numerical
                            // problem
                            if (U < myUMinParameter)
                            {
                                U = myUMinParameter;
                            }
                            if (U > myUMaxParameter)
                            {
                                U = myUMaxParameter;
                            }
                            if (V < myVMinParameter)
                            {
                                V = myVMinParameter;
                            }
                            if (V > myVMaxParameter)
                            {
                                V = myVMaxParameter;
                            }

                            int aNbRanges = myRangeManager.Length();
                            ComputeRangeFromStartPoint(false, T, U, V);
                            ComputeRangeFromStartPoint(true, T, U, V);

                            if (aNbRanges == myRangeManager.Length())
                            {
                                SetEmptyResultRange(T, myRangeManager);
                            }
                        }
                    } // end for
                }
                else
                {
                    myRangeManager.InsertRange(anarg1, anarg2, 0);
                }

                aRangeSPrev = anItS.Value();
            }

            //
            aCurveData.ListRangeOut(aListOut);
        }

        //
        if (bAllowSamplingC)
        {
            NCollection_List<IntTools_CurveRangeSample>.Iterator anItC = new(aListOut);

            for (; anItC.More(); anItC.Next())
            {
                IntTools_Range aRangeC = anItC.Value().GetRange(myFirstParameter, myLastParameter, nbSampleC);
                myRangeManager.InsertRange(aRangeC.First(), aRangeC.Last(), 1);
            }
        }
        ComputeNearRangeBoundaries();

        aSurfaceData.ClearGrid();

        return true;
    }

    private bool TestComputeCoinside()
    {
        double cfp = myFirstParameter, clp = myLastParameter;
        const int nbSeg = 23;
        double cdp = (clp - cfp) / (double)nbSeg;

        int i = 0;

        double U = 0.0, V = 0.0;

        if (Distance(cfp, ref U, ref V) > myCriteria)
        {
            return false;
        }

        //
        ComputeRangeFromStartPoint(true, cfp, U, V);
        //

        int aFoundIndex = myRangeManager.GetIndex(clp, false);

        if (aFoundIndex != 0)
        {
            if (myRangeManager.Flag(aFoundIndex) == 2)
            {
                return true;
            }
        }

        if (Distance(clp, ref U, ref V) > myCriteria)
        {
            return false;
        }

        //
        ComputeRangeFromStartPoint(false, clp, U, V);
        //

        for (i = 1; i < nbSeg; i++)
        {
            double aPar = (cfp + ((double)i) * cdp);

            if (Distance(aPar, ref U, ref V) > myCriteria)
            {
                return false;
            }

            int aNbRanges = myRangeManager.Length();
            ComputeRangeFromStartPoint(false, aPar, U, V);
            ComputeRangeFromStartPoint(true, aPar, U, V);

            if (aNbRanges == myRangeManager.Length())
            {
                SetEmptyResultRange(aPar, myRangeManager);
            }
        }

        return true;
    }

    //  Modified by skv - Wed Nov  2 15:21:11 2005 Optimization Begin

    private static Bnd_Box GetSurfaceBox(BRepAdaptor_Surface theSurf,
                                         double theFirstU,
                                         double theLastU,
                                         double theFirstV,
                                         double theLastV,
                                         double theTolerance,
                                         IntTools_SurfaceRangeLocalizeData theSurfaceData)
    {
        Bnd_Box aTotalBox = new();

        BuildBox(theSurf, theFirstU, theLastU, theFirstV, theLastV, theSurfaceData, ref aTotalBox);

        aTotalBox.Enlarge(theTolerance);
        return aTotalBox;
    }

    private static void ComputeGridPoints(BRepAdaptor_Surface theSurf,
                                          Geom_BSplineSurface theBsplSurf,
                                          double theFirstU,
                                          double theLastU,
                                          double theFirstV,
                                          double theLastV,
                                          double theTolerance,
                                          IntTools_SurfaceRangeLocalizeData theSurfaceData)
    {
        int i;
        int j;
        int k;
        Span<int> aNbSamples = [theSurf.UDegree(), theSurf.VDegree()];
        Span<int> aNbKnots = [theSurf.NbUKnots(), theSurf.NbVKnots()];
        NCollection_Array1<double> aKnotsU = theBsplSurf.InternalUKnots();
        NCollection_Array1<double> aKnotsV = theBsplSurf.InternalVKnots();

        int iLmI;
        Span<int> iMin = [-1, -1];
        Span<int> iMax = [-1, -1];
        Span<int> aNbGridPnts = stackalloc int[2];
        Span<double> aFPar = [theFirstU, theFirstV];
        Span<double> aLPar = [theLastU, theLastV];
        Span<double> aFpTol = [aFPar[0] + theTolerance, aFPar[1] + theTolerance];
        Span<double> aFmTol = [aFPar[0] - theTolerance, aFPar[1] - theTolerance];
        Span<double> aLpTol = [aLPar[0] + theTolerance, aLPar[1] + theTolerance];
        Span<double> aLmTol = [aLPar[0] - theTolerance, aLPar[1] - theTolerance];

        // Compute number of U and V grid points.
        for (j = 0; j < 2; j++)
        {
            NCollection_Array1<double> aKnots = (j == 0) ? aKnotsU : aKnotsV;

            for (i = 1; i <= aNbKnots[j] && (iMin[j] == -1 || iMax[j] == -1); i++)
            {
                if (iMin[j] == -1 && aFpTol[j] < aKnots.Value(i))
                {
                    iMin[j] = i - 1;
                }

                iLmI = aNbKnots[j] - i + 1;

                if (iMax[j] == -1 && aLmTol[j] > aKnots.Value(iLmI))
                {
                    iMax[j] = iLmI + 1;
                }
            }

            // If indices are not found, return.
            // if (iMin[j] == -1 || iMax[j] == -1)
            // return;
            if (iMin[j] == -1)
            {
                iMin[j] = 1;
            }

            if (iMax[j] == -1)
            {
                iMax[j] = aNbKnots[j];
            }

            if (iMin[j] == 0)
            {
                iMin[j] = 1;
            }

            if (iMax[j] > aNbKnots[j])
            {
                iMax[j] = aNbKnots[j];
            }

            if (iMax[j] < iMin[j])
            {
                return;
            }

            if (iMax[j] == iMin[j])
            {
                iMax[j]++;
                iMin[j]--;
                if (iMin[j] == 0)
                {
                    iMin[j] = 1;
                }
                if (iMax[j] > aNbKnots[j])
                {
                    iMax[j] = aNbKnots[j];
                }
            }

            aNbGridPnts[j] = (iMax[j] - iMin[j]) * aNbSamples[j] + 1;

            // Setting the number of grid points.
            if (j == 0)
            {
                theSurfaceData.SetRangeUGrid(aNbGridPnts[j]);
            }
            else
            { // j == 1
                theSurfaceData.SetRangeVGrid(aNbGridPnts[j]);
            }

            // Setting the first and last parameters.
            int iAbs = 1;
            double aMinPar;
            double aMaxPar = (j == 0) ? theLastU : theLastV;

            for (i = iMin[j]; i < iMax[j]; i++)
            {
                // Get the first parameter.
                if (i == iMin[j])
                {
                    // The first knot.
                    if (aFmTol[j] > aKnots.Value(iMin[j]))
                    {
                        aMinPar = aFPar[j];
                    }
                    else
                    {
                        aMinPar = aKnots.Value(iMin[j]);
                    }
                }
                else
                {
                    aMinPar = aKnots.Value(i);
                }

                // Get the last parameter.
                if (i == iMax[j] - 1)
                {
                    // The last knot.
                    if (aLpTol[j] < aKnots.Value(iMax[j]))
                    {
                        aMaxPar = aLPar[j];
                    }
                    else
                    {
                        aMaxPar = aKnots.Value(iMax[j]);
                    }
                }
                else
                {
                    aMaxPar = aKnots.Value(i + 1);
                }

                // Compute grid parameters.
                double aDelta = (aMaxPar - aMinPar) / aNbSamples[j];

                for (k = 0; k < aNbSamples[j]; k++, aMinPar += aDelta)
                {
                    if (j == 0)
                    {
                        theSurfaceData.SetUParam(iAbs++, aMinPar);
                    }
                    else
                    {
                        theSurfaceData.SetVParam(iAbs++, aMinPar);
                    }
                }
            }

            // Add the last parameter
            if (j == 0)
            {
                theSurfaceData.SetUParam(iAbs++, aMaxPar);
            }
            else
            {
                theSurfaceData.SetVParam(iAbs++, aMaxPar);
            }
        }

        // Compute of grid points.
        gp_Pnt aPnt;
        double aParU;
        double aParV;
        gp_Vec aDU = new(), aDV = new();
        double du = 0, dv = 0;
        bool isCalcDefl = aNbGridPnts[0] < 30 && aNbGridPnts[1] < 30;

        Bnd_Box aGridBox = new(), anExtBox = new();

        for (i = 1; i <= aNbGridPnts[0]; i++)
        {
            aParU = theSurfaceData.GetUParam(i);

            if (isCalcDefl && i < aNbGridPnts[0])
            {
                du = 0.5 * (theSurfaceData.GetUParam(i + 1) - aParU);
            }

            for (j = 1; j <= aNbGridPnts[1]; j++)
            {
                aParV = theSurfaceData.GetVParam(j);

                if (isCalcDefl)
                {
                    theSurf.D1(aParU, aParV, out aPnt, out aDU, out aDV);
                }
                else
                {
                    theSurf.D0(aParU, aParV, out aPnt);
                }

                theSurfaceData.SetGridPoint(i, j, aPnt);
                //
                if (isCalcDefl)
                {
                    aGridBox.Add(aPnt);
                    if (i < aNbGridPnts[0] && j < aNbGridPnts[1])
                    {
                        dv = 0.5 * (theSurfaceData.GetVParam(j + 1) - aParV);
                        gp_Vec aShift = du * aDU + dv * aDV;
                        aPnt.Translate(aShift);
                        anExtBox.Add(aPnt);
                    }
                }
            }
        }

        // Compute deflection.
        double aDef = 0.0;
        if (isCalcDefl)
        {
            double xmin, ymin, zmin, xmax, ymax, zmax;
            double xmin1, ymin1, zmin1, xmax1, ymax1, zmax1;
            aGridBox.Get(out xmin, out ymin, out zmin, out xmax, out ymax, out zmax);
            anExtBox.Get(out xmin1, out ymin1, out zmin1, out xmax1, out ymax1, out zmax1);
            int anExtCount = 0;
            if (xmin1 < xmin)
            {
                aDef = Max(xmin - xmin1, aDef);
                anExtCount++;
            }
            if (ymin1 < ymin)
            {
                aDef = Max(ymin - ymin1, aDef);
                anExtCount++;
            }
            if (zmin1 < zmin)
            {
                aDef = Max(zmin - zmin1, aDef);
                anExtCount++;
            }
            if (xmax1 > xmax)
            {
                aDef = Max(xmax1 - xmax, aDef);
                anExtCount++;
            }
            if (ymax1 > ymax)
            {
                aDef = Max(ymax1 - ymax, aDef);
                anExtCount++;
            }
            if (zmax1 > zmax)
            {
                aDef = Max(zmax1 - zmax, aDef);
                anExtCount++;
            }
            if (anExtCount < 3)
            {
                aDef /= 2.0;
            }
        }

        if (theTolerance > aDef)
        {
            aDef = 2.0 * theTolerance;
        }

        theSurfaceData.SetGridDeflection(aDef);
    }

    private static void BuildBox(BRepAdaptor_Surface theSurf,
                                 double theFirstU,
                                 double theLastU,
                                 double theFirstV,
                                 double theLastV,
                                 IntTools_SurfaceRangeLocalizeData theSurfaceData,
                                 ref Bnd_Box theBox)
    {
        int i;
        int j;
        int aNbUPnts;
        int aNbVPnts;
        double aParam;
        gp_Pnt aPnt;

        theSurfaceData.SetFrame(theFirstU, theLastU, theFirstV, theLastV);
        aNbUPnts = theSurfaceData.GetNBUPointsInFrame();
        aNbVPnts = theSurfaceData.GetNBVPointsInFrame();

        // Add corner points.
        theSurf.D0(theFirstU, theFirstV, out aPnt);
        theBox.Add(aPnt);
        theSurf.D0(theLastU, theFirstV, out aPnt);
        theBox.Add(aPnt);
        theSurf.D0(theFirstU, theLastV, out aPnt);
        theBox.Add(aPnt);
        theSurf.D0(theLastU, theLastV, out aPnt);
        theBox.Add(aPnt);

        for (i = 1; i <= aNbUPnts; i++)
        {
            // Add top and bottom points.
            aParam = theSurfaceData.GetUParamInFrame(i);
            theSurf.D0(aParam, theFirstV, out aPnt);
            theBox.Add(aPnt);
            theSurf.D0(aParam, theLastV, out aPnt);
            theBox.Add(aPnt);

            // Add internal points.
            for (j = 1; j <= aNbVPnts; j++)
            {
                gp_Pnt aGridPnt = theSurfaceData.GetPointInFrame(i, j);

                theBox.Add(aGridPnt);
            }
        }

        // Add left and right points.
        for (j = 1; j <= aNbVPnts; j++)
        {
            aParam = theSurfaceData.GetVParamInFrame(j);
            theSurf.D0(theFirstU, aParam, out aPnt);
            theBox.Add(aPnt);
            theSurf.D0(theLastU, aParam, out aPnt);
            theBox.Add(aPnt);
        }

        theBox.Enlarge(theSurfaceData.GetGridDeflection());
    }

    //  Modified by skv - Wed Nov  2 15:21:11 2005 Optimization End

    private static void MergeSolutions(NCollection_List<IntTools_CurveRangeSample> theListCurveRange,
                                       NCollection_List<IntTools_SurfaceRangeSample> theListSurfaceRange,
                                       NCollection_List<IntTools_CurveRangeSample> theListCurveRangeSort,
                                       NCollection_List<IntTools_SurfaceRangeSample> theListSurfaceRangeSort)
    {
        NCollection_IndexedMap<IntTools_SurfaceRangeSample> aMapToAvoid = new();

        NCollection_DataMap<int, NCollection_List<int>> aCurveIdMap = new();
        NCollection_LinearVector<IntTools_CurveRangeSample> aCurveRangeVector = new();
        aCurveRangeVector.Reserve(theListCurveRange.Size());

        NCollection_List<IntTools_CurveRangeSample>.Iterator anItC = new(theListCurveRange);
        NCollection_List<IntTools_SurfaceRangeSample>.Iterator anItS = new(theListSurfaceRange);

        int aCurveRangeId = 0;
        int aSurfRangeSize = 0;
        for (; anItS.More() && anItC.More(); anItS.Next(), anItC.Next(), ++aCurveRangeId)
        {
            aCurveRangeVector.Append(anItC.Value());
            int aSurfIndex = aMapToAvoid.Add(anItS.Value());
            if (aSurfIndex > aSurfRangeSize)
            {
                NCollection_List<int> aNewList = new();
                aCurveIdMap.Bind(aSurfIndex, aNewList);
                aNewList.Append(aCurveRangeId);
                ++aSurfRangeSize;
            }
            else
            {
                aCurveIdMap.Find(aSurfIndex).Append(aCurveRangeId);
            }
        }

        for (int i = 1; i <= aMapToAvoid.Extent(); i++)
        {
            IntTools_SurfaceRangeSample aSurfRange = aMapToAvoid.FindKey(i);
            NCollection_List<int> aCurveRangeList = aCurveIdMap.Find(i);
            for (NCollection_List<int>.Iterator anIter = new(aCurveRangeList); anIter.More(); anIter.Next())
            {
                IntTools_CurveRangeSample aCurveRange = aCurveRangeVector[anIter.Value()];

                theListSurfaceRangeSort.Append(aSurfRange);
                theListCurveRangeSort.Append(aCurveRange);
            }
        }
    }

    private static void CheckSampling(in IntTools_CurveRangeSample theCurveRange,
                                      in IntTools_SurfaceRangeSample theSurfaceRange,
                                      IntTools_CurveRangeLocalizeData theCurveData,
                                      IntTools_SurfaceRangeLocalizeData theSurfaceData,
                                      double DiffC,
                                      double DiffU,
                                      double DiffV,
                                      out bool bAllowSamplingC,
                                      out bool bAllowSamplingU,
                                      out bool bAllowSamplingV)
    {

        const double dLimit = 1000;
        bAllowSamplingC = true;
        bAllowSamplingU = true;
        bAllowSamplingV = true;

        int aSamplesNb = theCurveRange.GetDepth() == 0 ? 1 : theCurveData.GetNbSample();

        // check
        if ((Math.Pow((double)theCurveData.GetNbSample(), (double)(theCurveRange.GetDepth() + 1)) > dLimit)
            || ((DiffC / (double)aSamplesNb) < theCurveData.GetMinRange()))
        {
            bAllowSamplingC = false;
        }

        aSamplesNb = theSurfaceRange.GetDepthU() == 0 ? 1 : theSurfaceData.GetNbSampleU();

        if ((Math.Pow((double)theSurfaceData.GetNbSampleU(), (double)(theSurfaceRange.GetDepthU() + 1)) > dLimit)
            || ((DiffU / (double)aSamplesNb) < theSurfaceData.GetMinRangeU()))
        {
            bAllowSamplingU = false;
        }

        aSamplesNb = theSurfaceRange.GetDepthV() == 0 ? 1 : theSurfaceData.GetNbSampleV();

        if ((Math.Pow((double)theSurfaceData.GetNbSampleV(), (double)(theSurfaceRange.GetDepthV() + 1)) > dLimit)
            || ((DiffV / (double)aSamplesNb) < theSurfaceData.GetMinRangeV()))
        {
            bAllowSamplingV = false;
        }
    }
}
