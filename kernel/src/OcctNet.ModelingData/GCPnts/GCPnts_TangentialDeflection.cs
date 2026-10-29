// Port of: src/ModelingData/TKGeomBase/GCPnts/GCPnts_TangentialDeflection.hxx
// Port of: src/ModelingData/TKGeomBase/GCPnts/GCPnts_TangentialDeflection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// Computes points on a curve such that both the angle between the tangents at consecutive points (angular
/// deflection) and the distance between the curve and the chords (curvature deflection) stay below given values.
/// This is the discretisation used by meshing. 2D curves give points with Z = 0.
/// </summary>
/// <remarks>
/// A working object (decision 21): <c>Initialize</c> recomputes and <c>AddPoint</c> adds points, as in OCCT. The
/// templates over the curve type are ported per instantiation (decision 2002).
/// </remarks>
public sealed class GCPnts_TangentialDeflection
{
    private const double Us3 = 0.3333333333333333333333333333;

    private double myAngularDeflection;
    private double myCurvatureDeflection;
    private double myUTol;
    private int myMinNbPnts;
    private double myMinLen;
    private double myLastU;
    private double myFirstu;
    private readonly NCollection_Sequence<gp_Pnt> myPoints = new();
    private readonly NCollection_Sequence<double> myParameters = new();

    /// <summary>An empty algorithm; use <c>Initialize</c>.</summary>
    public GCPnts_TangentialDeflection()
    {
        myAngularDeflection = 0.0;
        myCurvatureDeflection = 0.0;
        myUTol = 0.0;
        myMinNbPnts = 0;
        myMinLen = 0.0;
        myLastU = 0.0;
        myFirstu = 0.0;
    }

    /// <summary>
    /// Points on the curve with the angular deflection theAngularDeflection (radians) and the curvature deflection
    /// theCurvatureDeflection (a length); at least theMinimumOfPoints points, parameters closer than theUTol merged,
    /// segments not shorter than theMinLen.
    /// </summary>
    /// <exception cref="Standard_ConstructionError">
    /// A deflection is below Precision::Confusion() (curvature) or Precision::Angular() (angle).
    /// </exception>
    public GCPnts_TangentialDeflection(Adaptor3d_Curve theC, double theAngularDeflection, double theCurvatureDeflection,
        int theMinimumOfPoints = 2, double theUTol = 1.0e-9, double theMinLen = 1.0e-7)
        : this()
    {
        Initialize(theC, theAngularDeflection, theCurvatureDeflection, theMinimumOfPoints, theUTol, theMinLen);
    }

    /// <summary>Points on the part theFirstParameter..theLastParameter of the curve.</summary>
    /// <exception cref="Standard_ConstructionError">A deflection is too small.</exception>
    public GCPnts_TangentialDeflection(Adaptor3d_Curve theC, double theFirstParameter, double theLastParameter,
        double theAngularDeflection, double theCurvatureDeflection, int theMinimumOfPoints = 2,
        double theUTol = 1.0e-9, double theMinLen = 1.0e-7)
        : this()
    {
        Initialize(theC, theFirstParameter, theLastParameter, theAngularDeflection, theCurvatureDeflection,
            theMinimumOfPoints, theUTol, theMinLen);
    }

    /// <summary>Points on the 2D curve.</summary>
    /// <exception cref="Standard_ConstructionError">A deflection is too small.</exception>
    public GCPnts_TangentialDeflection(Adaptor2d_Curve2d theC, double theAngularDeflection,
        double theCurvatureDeflection, int theMinimumOfPoints = 2, double theUTol = 1.0e-9, double theMinLen = 1.0e-7)
        : this()
    {
        Initialize(theC, theAngularDeflection, theCurvatureDeflection, theMinimumOfPoints, theUTol, theMinLen);
    }

    /// <summary>Points on the part theFirstParameter..theLastParameter of the 2D curve.</summary>
    /// <exception cref="Standard_ConstructionError">A deflection is too small.</exception>
    public GCPnts_TangentialDeflection(Adaptor2d_Curve2d theC, double theFirstParameter, double theLastParameter,
        double theAngularDeflection, double theCurvatureDeflection, int theMinimumOfPoints = 2,
        double theUTol = 1.0e-9, double theMinLen = 1.0e-7)
        : this()
    {
        Initialize(theC, theFirstParameter, theLastParameter, theAngularDeflection, theCurvatureDeflection,
            theMinimumOfPoints, theUTol, theMinLen);
    }

    /// <summary>Recomputes the points on the curve.</summary>
    public void Initialize(Adaptor3d_Curve theC, double theAngularDeflection, double theCurvatureDeflection,
        int theMinimumOfPoints = 2, double theUTol = 1.0e-9, double theMinLen = 1.0e-7)
    {
        Initialize(theC, theC.FirstParameter(), theC.LastParameter(), theAngularDeflection, theCurvatureDeflection,
            theMinimumOfPoints, theUTol, theMinLen);
    }

    /// <summary>Recomputes the points on the 2D curve.</summary>
    public void Initialize(Adaptor2d_Curve2d theC, double theAngularDeflection, double theCurvatureDeflection,
        int theMinimumOfPoints = 2, double theUTol = 1.0e-9, double theMinLen = 1.0e-7)
    {
        Initialize(theC, theC.FirstParameter(), theC.LastParameter(), theAngularDeflection, theCurvatureDeflection,
            theMinimumOfPoints, theUTol, theMinLen);
    }

    /// <summary>Recomputes the points on the part theFirstParameter..theLastParameter of the curve.</summary>
    public void Initialize(Adaptor3d_Curve theC, double theFirstParameter, double theLastParameter,
        double theAngularDeflection, double theCurvatureDeflection, int theMinimumOfPoints = 2,
        double theUTol = 1.0e-9, double theMinLen = 1.0e-7)
    {
        initialize(theC, theFirstParameter, theLastParameter, theAngularDeflection, theCurvatureDeflection,
            theMinimumOfPoints, theUTol, theMinLen);
    }

    /// <summary>Recomputes the points on the part theFirstParameter..theLastParameter of the 2D curve.</summary>
    public void Initialize(Adaptor2d_Curve2d theC, double theFirstParameter, double theLastParameter,
        double theAngularDeflection, double theCurvatureDeflection, int theMinimumOfPoints = 2,
        double theUTol = 1.0e-9, double theMinLen = 1.0e-7)
    {
        initialize(theC, theFirstParameter, theLastParameter, theAngularDeflection, theCurvatureDeflection,
            theMinimumOfPoints, theUTol, theMinLen);
    }

    /// <summary>
    /// Adds a point, keeping the parameters sorted; a point whose parameter is within Precision::PConfusion() of an
    /// existing one replaces it when theIsReplace is true. Returns the index of the point.
    /// </summary>
    public int AddPoint(in gp_Pnt thePnt, double theParam, bool theIsReplace = true)
    {
        double tol = Precision.PConfusion();
        int index = -1;
        int nb = myParameters.Length();
        for (int i = 1; index == -1 && i <= nb; i++)
        {
            double dist = myParameters.Value(i) - theParam;
            if (Math.Abs(dist) <= tol)
            {
                index = i;
                if (theIsReplace)
                {
                    myPoints.ChangeValue(i) = thePnt;
                    myParameters.ChangeValue(i) = theParam;
                }
            }
            else if (dist > tol)
            {
                myPoints.InsertBefore(i, thePnt);
                myParameters.InsertBefore(i, theParam);
                index = i;
            }
        }
        if (index == -1)
        {
            myPoints.Append(thePnt);
            myParameters.Append(theParam);
            index = myParameters.Length();
        }
        return index;
    }

    /// <summary>The number of points computed.</summary>
    public int NbPoints() => myParameters.Length();

    /// <summary>The parameter of the point of index I (1 to NbPoints).</summary>
    public double Parameter(int I) => myParameters.Value(I);

    /// <summary>The point of index I (1 to NbPoints).</summary>
    public gp_Pnt Value(int I) => myPoints.Value(I);

    /// <summary>
    /// The angular step for an arc of radius theRadius that satisfies the linear and angular deflections, without
    /// going below theMinLength.
    /// </summary>
    /// <exception cref="Standard_ConstructionError">theRadius is negative.</exception>
    public static double ArcAngularStep(double theRadius, double theLinearDeflection, double theAngularDeflection,
        double theMinLength)
    {
        Standard_ConstructionError_Raise_if(theRadius < 0.0, "Negative radius");

        double aPrecision = Precision.Confusion();

        double Du = 0.0, aMinSizeAng = 0.0;
        if (theRadius > aPrecision)
        {
            Du = Max(1.0 - (theLinearDeflection / theRadius), 0.0);

            // It is not suitable to consider min size greater than 1/4 arc len.
            if (theMinLength > aPrecision)
            {
                aMinSizeAng = Min(theMinLength / theRadius, M_PI_2);
            }
        }
        Du = 2.0 * Math.Acos(Du);
        Du = Max(Min(Du, theAngularDeflection), aMinSizeAng);
        return Du;
    }

    private static void D0(Adaptor3d_Curve C, double U, out gp_Pnt P)
    {
        C.D0(U, out P);
    }

    private static void D2(Adaptor3d_Curve C, double U, out gp_Pnt P, out gp_Vec V1, out gp_Vec V2)
    {
        C.D2(U, out P, out V1, out V2);
    }

    private static void D0(Adaptor2d_Curve2d C, double U, out gp_Pnt PP)
    {
        double X, Y;
        C.D0(U, out gp_Pnt2d P);
        P.Coord(out X, out Y);
        PP = new gp_Pnt();
        PP.SetCoord(X, Y, 0.0);
    }

    private static void D2(Adaptor2d_Curve2d C, double U, out gp_Pnt PP, out gp_Vec VV1, out gp_Vec VV2)
    {
        double X, Y;
        C.D2(U, out gp_Pnt2d P, out gp_Vec2d V1, out gp_Vec2d V2);
        PP = new gp_Pnt();
        VV1 = new gp_Vec();
        VV2 = new gp_Vec();
        P.Coord(out X, out Y);
        PP.SetCoord(X, Y, 0.0);
        V1.Coord(out X, out Y);
        VV1.SetCoord(X, Y, 0.0);
        V2.Coord(out X, out Y);
        VV2.SetCoord(X, Y, 0.0);
    }

    private static double EstimAngl(in gp_Pnt P1, in gp_Pnt Pm, in gp_Pnt P2)
    {
        gp_Vec V1 = new(P1, Pm), V2 = new(Pm, P2);
        double L = V1.Magnitude() * V2.Magnitude();
        if (L > gp.Resolution())
        {
            return V1.CrossMagnitude(V2) / L;
        }
        else
        {
            return 0.0;
        }
    }

    // Return number of interval of continuity on which theParam is located.
    // Last parameter is used to increase search speed.
    private static int getIntervalIdx(double theParam, NCollection_Array1<double> theIntervs, int thePreviousIdx)
    {
        int anIdx;
        for (anIdx = thePreviousIdx; anIdx < theIntervs.Upper(); anIdx++)
        {
            if (theParam >= theIntervs[anIdx]
                && theParam <= theIntervs[anIdx + 1]) // Inside of anIdx interval.
            {
                break;
            }
        }
        return anIdx;
    }

    // ---- Templates over the curve type (3D instantiation) ----

    // theDu and theNotDone are left unchanged unless a step is found, as in OCCT.
    private void EvaluateDu(Adaptor3d_Curve theC, double theU, out gp_Pnt theP, ref double theDu,
        ref bool theNotDone)
    {
        gp_Vec T, N;
        D2(theC, theU, out theP, out T, out N);
        double Lt = T.Magnitude();
        double LTol = Precision.Confusion();
        if (Lt > LTol && N.Magnitude() > LTol)
        {
            double Lc = N.CrossMagnitude(T);
            double Ln = Lc / Lt;
            if (Ln > LTol)
            {
                theDu = Math.Sqrt(8.0 * Max(myCurvatureDeflection, myMinLen) / Ln);
                theNotDone = false;
            }
        }
    }

    private void PerformLinear(Adaptor3d_Curve theC)
    {
        gp_Pnt P;
        D0(theC, myFirstu, out P);
        myParameters.Append(myFirstu);
        myPoints.Append(P);
        if (myMinNbPnts > 2)
        {
            double Du = (myLastU - myFirstu) / myMinNbPnts;
            double U = myFirstu + Du;
            for (int i = 2; i < myMinNbPnts; i++)
            {
                D0(theC, U, out P);
                myParameters.Append(U);
                myPoints.Append(P);
                U += Du;
            }
        }
        D0(theC, myLastU, out P);
        myParameters.Append(myLastU);
        myPoints.Append(P);
    }

    private void PerformCircular(Adaptor3d_Curve theC)
    {
        // akm 8/01/02 : check the radius before divide by it
        double dfR = theC.Circle().Radius();
        double Du = GCPnts_TangentialDeflection.ArcAngularStep(dfR,
                                                               myCurvatureDeflection,
                                                               myAngularDeflection,
                                                               myMinLen);

        double aDiff = myLastU - myFirstu;
        // Round up number of points to satisfy curvatureDeflection more precisely
        int NbPoints = (int)Min(Math.Ceiling(aDiff / Du), 1.0e+6);
        NbPoints = Max(NbPoints, myMinNbPnts - 1);
        Du = aDiff / NbPoints;

        gp_Pnt P;
        double U = myFirstu;
        for (int i = 1; i <= NbPoints; i++)
        {
            D0(theC, U, out P);
            myParameters.Append(U);
            myPoints.Append(P);
            U += Du;
        }
        D0(theC, myLastU, out P);
        myParameters.Append(myLastU);
        myPoints.Append(P);
    }

    private void initialize(Adaptor3d_Curve theC,
                            double theFirstParameter,
                            double theLastParameter,
                            double theAngularDeflection,
                            double theCurvatureDeflection,
                            int theMinimumOfPoints,
                            double theUTol,
                            double theMinLen)
    {
        Standard_ConstructionError_Raise_if(
            theCurvatureDeflection < Precision.Confusion() || theAngularDeflection < Precision.Angular(),
            "GCPnts_TangentialDeflection::Initialize - Zero Deflection");
        myParameters.Clear();
        myPoints.Clear();
        if (theFirstParameter < theLastParameter)
        {
            myFirstu = theFirstParameter;
            myLastU = theLastParameter;
        }
        else
        {
            myLastU = theFirstParameter;
            myFirstu = theLastParameter;
        }
        myUTol = theUTol;
        myAngularDeflection = theAngularDeflection;
        myCurvatureDeflection = theCurvatureDeflection;
        myMinNbPnts = Max(theMinimumOfPoints, 2);
        myMinLen = Max(theMinLen, Precision.Confusion());

        switch (theC.GetType())
        {
            case GeomAbs_Line:
            {
                PerformLinear(theC);
                break;
            }
            case GeomAbs_Circle:
            {
                PerformCircular(theC);
                break;
            }
            case GeomAbs_BSplineCurve:
            {
                Geom_BSplineCurve aBS = theC.BSpline();
                if (aBS.NbPoles() == 2)
                {
                    PerformLinear(theC);
                }
                else
                {
                    PerformCurve(theC);
                }
                break;
            }
            case GeomAbs_BezierCurve:
            {
                Geom_BezierCurve aBZ = theC.Bezier();
                if (aBZ.NbPoles() == 2)
                {
                    PerformLinear(theC);
                }
                else
                {
                    PerformCurve(theC);
                }
                break;
            }
            default:
            {
                PerformCurve(theC);
                break;
            }
        }
    }

    private void PerformCurve(Adaptor3d_Curve theC)
    {
        int i, j;
        gp_XYZ V1, V2;
        gp_Pnt MiddlePoint, CurrentPoint, LastPoint;
        double Du, Dusave, MiddleU, L1, L2;

        double U1 = myFirstu;
        double LTol = Precision.Confusion(); // protection against zero length
        double ATol = 1.0e-2 * myAngularDeflection;
        if (ATol > 1.0e-2)
        {
            ATol = 1.0e-2;
        }
        else if (ATol < 1.0e-7)
        {
            ATol = 1.0e-7;
        }

        D0(theC, myLastU, out LastPoint);

        // Initialization of the computation
        bool NotDone = true;
        Dusave = (myLastU - myFirstu) * Us3;
        Du = Dusave;
        EvaluateDu(theC, U1, out CurrentPoint, ref Du, ref NotDone);
        myParameters.Append(U1);
        myPoints.Append(CurrentPoint);

        // Used to detect "isLine" current bspline and in Du computation in general handling.
        int NbInterv = theC.NbIntervals(GeomAbs_CN);
        NCollection_Array1<double> Intervs = new(1, NbInterv + 1);
        theC.Intervals(Intervs, GeomAbs_CN);

        if (NotDone || Du > 5.0 * Dusave)
        {
            // It is either a line or a singularity:
            V1 = (LastPoint.XYZ() - CurrentPoint.XYZ());
            L1 = V1.Modulus();
            if (L1 > LTol)
            {
                // If it is a line, we verify by computing minNbPoints:
                bool IsLine = true;
                int NbPoints = (myMinNbPnts > 3) ? myMinNbPnts : 3;
                switch (theC.GetType())
                {
                    case GeomAbs_BSplineCurve:
                    {
                        Geom_BSplineCurve BS = theC.BSpline();
                        NbPoints = Max(BS.Degree() + 1, NbPoints);
                        break;
                    }
                    case GeomAbs_BezierCurve:
                    {
                        Geom_BezierCurve BZ = theC.Bezier();
                        NbPoints = Max(BZ.Degree() + 1, NbPoints);
                        break;
                    }
                    default:
                    {
                        break;
                    }
                }

                ////
                double param = 0.0;
                for (i = 1; i <= NbInterv && IsLine; ++i)
                {
                    // Avoid usage intervals out of [myFirstu, myLastU].
                    if ((Intervs[i + 1] < myFirstu) || (Intervs[i] > myLastU))
                    {
                        continue;
                    }

                    // Fix border points in applicable intervals, to avoid be out of target interval.
                    if ((Intervs[i] < myFirstu) && (Intervs[i + 1] > myFirstu))
                    {
                        Intervs[i] = myFirstu;
                    }
                    if ((Intervs[i] < myLastU) && (Intervs[i + 1] > myLastU))
                    {
                        Intervs[i + 1] = myLastU;
                    }

                    double delta = (Intervs[i + 1] - Intervs[i]) / NbPoints;
                    for (j = 1; j <= NbPoints && IsLine; ++j)
                    {
                        param = Intervs[i] + j * delta;
                        D0(theC, param, out MiddlePoint);
                        V2 = (MiddlePoint.XYZ() - CurrentPoint.XYZ());
                        L2 = V2.Modulus();
                        if (L2 > LTol)
                        {
                            double aAngle = V2.CrossMagnitude(V1) / (L1 * L2);
                            IsLine = (aAngle < ATol);
                        }
                    }
                }

                if (IsLine)
                {
                    myParameters.Clear();
                    myPoints.Clear();

                    PerformLinear(theC);
                    return;
                }
                else
                {
                    // It was a singularity, continue:
                    // Du = Dusave;
                    EvaluateDu(theC, param, out MiddlePoint, ref Du, ref NotDone);
                }
            }
            else
            {
                Du = (myLastU - myFirstu) / 2.1;
                MiddleU = myFirstu + Du;
                D0(theC, MiddleU, out MiddlePoint);
                V1 = (MiddlePoint.XYZ() - CurrentPoint.XYZ());
                L1 = V1.Modulus();
                if (L1 < LTol)
                {
                    // L1 < LTol: this is a zero-length curve, computation is finished:
                    // return a segment of 2 points (protection)
                    myParameters.Append(myLastU);
                    myPoints.Append(LastPoint);
                    return;
                }
            }
        }

        if (Du > Dusave)
        {
            Du = Dusave;
        }
        else
        {
            Dusave = Du;
        }

        if (Du < myUTol)
        {
            Du = myLastU - myFirstu;
            if (Du < myUTol)
            {
                myParameters.Append(myLastU);
                myPoints.Append(LastPoint);
                return;
            }
        }

        // Normal processing for a curve
        bool MorePoints = true;
        double U2 = myFirstu;
        double AngleMax = myAngularDeflection * 0.5; // because we take the midpoint

        // Indexes of intervals of U1 and U2, used to handle non-uniform case.
        Span<int> aIdx = [Intervs.Lower(), Intervs.Lower()];
        bool isNeedToCheck = false;
        gp_Pnt aPrevPoint = myPoints.Last();

        while (MorePoints)
        {
            aIdx[0] = getIntervalIdx(U1, Intervs, aIdx[0]);
            U2 += Du;

            if (U2 >= myLastU) // End of curve
            {
                U2 = myLastU;
                CurrentPoint = LastPoint;
                Du = U2 - U1;
                Dusave = Du;
            }
            else
            {
                D0(theC, U2, out CurrentPoint); // Next point
            }

            double Coef = 0.0, ACoef = 0.0, FCoef = 0.0;
            bool Correction, TooLarge, TooSmall;
            TooLarge = false;
            Correction = true;
            TooSmall = false;

            while (Correction) // Adjustment of Du
            {
                if (isNeedToCheck)
                {
                    aIdx[1] = getIntervalIdx(U2, Intervs, aIdx[0]);
                    if (aIdx[1] > aIdx[0]) // Jump to another polynom.
                    {
                        // Set Du to the smallest value and check deflection on it.
                        if (Du > (Intervs[aIdx[0] + 1] - Intervs[aIdx[0]]) * Us3)
                        {
                            Du = (Intervs[aIdx[0] + 1] - Intervs[aIdx[0]]) * Us3;
                            U2 = U1 + Du;
                            if (U2 > myLastU)
                            {
                                U2 = myLastU;
                            }
                            D0(theC, U2, out CurrentPoint);
                        }
                    }
                }
                MiddleU = (U1 + U2) * 0.5; // Verification at the midpoint
                D0(theC, MiddleU, out MiddlePoint);

                V1 = (CurrentPoint.XYZ() - aPrevPoint.XYZ()); // Deflection criterion
                V2 = (MiddlePoint.XYZ() - aPrevPoint.XYZ());
                L1 = V1.Modulus();

                FCoef = (L1 > myMinLen) ? V1.CrossMagnitude(V2) / (L1 * myCurvatureDeflection) : 0.0;

                V1 = (CurrentPoint.XYZ() - MiddlePoint.XYZ()); // Angular criterion
                L1 = V1.Modulus();
                L2 = V2.Modulus();
                if (L1 > myMinLen && L2 > myMinLen)
                {
                    double angg = V1.CrossMagnitude(V2) / (L1 * L2);
                    ACoef = angg / AngleMax;
                }
                else
                {
                    ACoef = 0.0;
                }

                // Keep the most penalizing coefficient
                Coef = Max(ACoef, FCoef);

                if (isNeedToCheck && Coef < 0.55)
                {
                    isNeedToCheck = false;
                    Du = Dusave;
                    U2 = U1 + Du;
                    if (U2 > myLastU)
                    {
                        U2 = myLastU;
                    }
                    D0(theC, U2, out CurrentPoint);
                    continue;
                }

                if (Coef <= 1.0)
                {
                    if (Math.Abs(myLastU - U2) < myUTol)
                    {
                        myParameters.Append(myLastU);
                        myPoints.Append(LastPoint);
                        MorePoints = false;
                        Correction = false;
                    }
                    else
                    {
                        if (Coef >= 0.55 || TooLarge)
                        {
                            myParameters.Append(U2);
                            myPoints.Append(CurrentPoint);
                            aPrevPoint = CurrentPoint;
                            Correction = false;
                            isNeedToCheck = true;
                        }
                        else if (TooSmall)
                        {
                            Correction = false;
                            aPrevPoint = CurrentPoint;
                        }
                        else
                        {
                            TooSmall = true;
                            // double UUU2 = U2;
                            Du += Min((U2 - U1) * (1.0 - Coef), Du * Us3);

                            U2 = U1 + Du;
                            if (U2 > myLastU)
                            {
                                U2 = myLastU;
                            }
                            D0(theC, U2, out CurrentPoint);
                        }
                    }
                }
                else
                {
                    if (Coef >= 1.5)
                    {
                        if (!aPrevPoint.IsEqual(myPoints.Last(), Precision.Confusion()))
                        {
                            myParameters.Append(U1);
                            myPoints.Append(aPrevPoint);
                        }
                        U2 = MiddleU;
                        Du = U2 - U1;
                        CurrentPoint = MiddlePoint;
                    }
                    else
                    {
                        Du *= 0.9;
                        U2 = U1 + Du;
                        D0(theC, U2, out CurrentPoint);
                        TooLarge = true;
                    }
                }
            }

            Du = U2 - U1;

            if (MorePoints)
            {
                if (U1 > myFirstu)
                {
                    if (FCoef > ACoef)
                    {
                        // Deflection is the splitting criterion
                        EvaluateDu(theC, U2, out CurrentPoint, ref Du, ref NotDone);
                        if (NotDone)
                        {
                            Du += (Du - Dusave) * (Du / Dusave);
                            if (Du > 1.5 * Dusave)
                            {
                                Du = 1.5 * Dusave;
                            }
                            if (Du < 0.75 * Dusave)
                            {
                                Du = 0.75 * Dusave;
                            }
                        }
                    }
                    else
                    {
                        // Angle is the splitting criterion
                        Du += (Du - Dusave) * (Du / Dusave);
                        if (Du > 1.5 * Dusave)
                        {
                            Du = 1.5 * Dusave;
                        }
                        if (Du < 0.75 * Dusave)
                        {
                            Du = 0.75 * Dusave;
                        }
                    }
                }

                if (Du < myUTol)
                {
                    Du = myLastU - U2;
                    if (Du < myUTol)
                    {
                        myParameters.Append(myLastU);
                        myPoints.Append(LastPoint);
                        MorePoints = false;
                    }
                    else if (Du * Us3 > myUTol)
                    {
                        Du *= Us3;
                    }
                }
                U1 = U2;
                Dusave = Du;
            }
        }

        // Readjustment of the second to last point:
        i = myPoints.Length() - 1;
        if (i >= 2)
        {
            MiddleU = myParameters[i - 1];
            MiddleU = (myLastU + MiddleU) * 0.5;
            D0(theC, MiddleU, out MiddlePoint);
            myParameters.SetValue(i, MiddleU);
            myPoints.SetValue(i, MiddlePoint);
        }

        // Add points at midpoints of segments if the minimum
        // number of points is not yet reached.
        int Nbp = myPoints.Length();

        while (Nbp < myMinNbPnts)
        {
            for (i = 2; i <= Nbp; i += 2)
            {
                MiddleU = (myParameters.Value(i - 1) + myParameters.Value(i)) * 0.5;
                D0(theC, MiddleU, out MiddlePoint);
                myParameters.InsertBefore(i, MiddleU);
                myPoints.InsertBefore(i, MiddlePoint);
                Nbp++;
            }
        }

        // Additional check for intervals
        double MinLen2 = myMinLen * myMinLen;
        int MaxNbp = 10 * Nbp;
        for (i = 1; i < Nbp; ++i)
        {
            U1 = myParameters[i];
            U2 = myParameters[i + 1];

            if (U2 - U1 <= myUTol)
            {
                continue;
            }

            // Check maximal deflection on interval;
            double dmax = 0.0;
            double umax = 0.0;
            double amax = 0.0;
            EstimDefl(theC, U1, U2, ref dmax, ref umax);
            gp_Pnt P1 = myPoints[i];
            gp_Pnt P2 = myPoints[i + 1];
            D0(theC, umax, out MiddlePoint);
            amax = EstimAngl(P1, MiddlePoint, P2);
            if (dmax > myCurvatureDeflection || amax > AngleMax)
            {
                if (umax - U1 > myUTol && U2 - umax > myUTol)
                {
                    if (P1.SquareDistance(MiddlePoint) > MinLen2 && P2.SquareDistance(MiddlePoint) > MinLen2)
                    {
                        myParameters.InsertAfter(i, umax);
                        myPoints.InsertAfter(i, MiddlePoint);
                        ++Nbp;
                        --i; // To compensate ++i in loop header: i must point to first part of split interval
                        if (Nbp > MaxNbp)
                        {
                            break;
                        }
                    }
                }
            }
        }
    }

    private void EstimDefl(Adaptor3d_Curve theC, double theU1, double theU2, ref double theMaxDefl,
        ref double theUMax)
    {
        double Du = (myLastU - myFirstu);
        //
        GCPnts_DistFunction aFunc = new(theC, theU1, theU2);
        //
        const int aNbIter = 100;
        double aRelTol = Max(1.0e-3, 2.0 * myUTol / (Math.Abs(theU1) + Math.Abs(theU2)));
        //
        math_BrentMinimum anOptLoc = new(aRelTol, aNbIter, myUTol);
        anOptLoc.Perform(aFunc, theU1, (theU1 + theU2) / 2.0, theU2);
        if (anOptLoc.IsDone())
        {
            theMaxDefl = Math.Sqrt(-anOptLoc.Minimum());
            theUMax = anOptLoc.Location();
            return;
        }
        //
        math_Vector aLowBorder = new(1, 1), aUppBorder = new(1, 1), aSteps = new(1, 1);
        aSteps[1] = Max(0.1 * Du, 100.0 * myUTol);
        int aNbParticles = Max(8, RealToInt(32 * (theU2 - theU1) / Du));
        aLowBorder[1] = theU1;
        aUppBorder[1] = theU2;
        //
        //
        double aValue = 0.0;
        math_Vector aT = new(1, 1);
        GCPnts_DistFunctionMV aFuncMV = new(aFunc);

        math_PSO aFinder = new(aFuncMV, aLowBorder, aUppBorder, aSteps, aNbParticles);
        aFinder.Perform(aSteps, out aValue, aT);
        //
        anOptLoc.Perform(aFunc,
                         Max(aT[1] - aSteps[1], theU1),
                         aT[1],
                         Min(aT[1] + aSteps[1], theU2));
        if (anOptLoc.IsDone())
        {
            theMaxDefl = Math.Sqrt(-anOptLoc.Minimum());
            theUMax = anOptLoc.Location();
            return;
        }
        theMaxDefl = Math.Sqrt(-aValue);
        theUMax = aT[1];
    }

    // ---- Templates over the curve type (2D instantiation) ----

    // theDu and theNotDone are left unchanged unless a step is found, as in OCCT.
    private void EvaluateDu(Adaptor2d_Curve2d theC, double theU, out gp_Pnt theP, ref double theDu,
        ref bool theNotDone)
    {
        gp_Vec T, N;
        D2(theC, theU, out theP, out T, out N);
        double Lt = T.Magnitude();
        double LTol = Precision.Confusion();
        if (Lt > LTol && N.Magnitude() > LTol)
        {
            double Lc = N.CrossMagnitude(T);
            double Ln = Lc / Lt;
            if (Ln > LTol)
            {
                theDu = Math.Sqrt(8.0 * Max(myCurvatureDeflection, myMinLen) / Ln);
                theNotDone = false;
            }
        }
    }

    private void PerformLinear(Adaptor2d_Curve2d theC)
    {
        gp_Pnt P;
        D0(theC, myFirstu, out P);
        myParameters.Append(myFirstu);
        myPoints.Append(P);
        if (myMinNbPnts > 2)
        {
            double Du = (myLastU - myFirstu) / myMinNbPnts;
            double U = myFirstu + Du;
            for (int i = 2; i < myMinNbPnts; i++)
            {
                D0(theC, U, out P);
                myParameters.Append(U);
                myPoints.Append(P);
                U += Du;
            }
        }
        D0(theC, myLastU, out P);
        myParameters.Append(myLastU);
        myPoints.Append(P);
    }

    private void PerformCircular(Adaptor2d_Curve2d theC)
    {
        // akm 8/01/02 : check the radius before divide by it
        double dfR = theC.Circle().Radius();
        double Du = GCPnts_TangentialDeflection.ArcAngularStep(dfR,
                                                               myCurvatureDeflection,
                                                               myAngularDeflection,
                                                               myMinLen);

        double aDiff = myLastU - myFirstu;
        // Round up number of points to satisfy curvatureDeflection more precisely
        int NbPoints = (int)Min(Math.Ceiling(aDiff / Du), 1.0e+6);
        NbPoints = Max(NbPoints, myMinNbPnts - 1);
        Du = aDiff / NbPoints;

        gp_Pnt P;
        double U = myFirstu;
        for (int i = 1; i <= NbPoints; i++)
        {
            D0(theC, U, out P);
            myParameters.Append(U);
            myPoints.Append(P);
            U += Du;
        }
        D0(theC, myLastU, out P);
        myParameters.Append(myLastU);
        myPoints.Append(P);
    }

    private void initialize(Adaptor2d_Curve2d theC,
                            double theFirstParameter,
                            double theLastParameter,
                            double theAngularDeflection,
                            double theCurvatureDeflection,
                            int theMinimumOfPoints,
                            double theUTol,
                            double theMinLen)
    {
        Standard_ConstructionError_Raise_if(
            theCurvatureDeflection < Precision.Confusion() || theAngularDeflection < Precision.Angular(),
            "GCPnts_TangentialDeflection::Initialize - Zero Deflection");
        myParameters.Clear();
        myPoints.Clear();
        if (theFirstParameter < theLastParameter)
        {
            myFirstu = theFirstParameter;
            myLastU = theLastParameter;
        }
        else
        {
            myLastU = theFirstParameter;
            myFirstu = theLastParameter;
        }
        myUTol = theUTol;
        myAngularDeflection = theAngularDeflection;
        myCurvatureDeflection = theCurvatureDeflection;
        myMinNbPnts = Max(theMinimumOfPoints, 2);
        myMinLen = Max(theMinLen, Precision.Confusion());

        switch (theC.GetType())
        {
            case GeomAbs_Line:
            {
                PerformLinear(theC);
                break;
            }
            case GeomAbs_Circle:
            {
                PerformCircular(theC);
                break;
            }
            case GeomAbs_BSplineCurve:
            {
                Geom2d_BSplineCurve aBS = theC.BSpline();
                if (aBS.NbPoles() == 2)
                {
                    PerformLinear(theC);
                }
                else
                {
                    PerformCurve(theC);
                }
                break;
            }
            case GeomAbs_BezierCurve:
            {
                Geom2d_BezierCurve aBZ = theC.Bezier();
                if (aBZ.NbPoles() == 2)
                {
                    PerformLinear(theC);
                }
                else
                {
                    PerformCurve(theC);
                }
                break;
            }
            default:
            {
                PerformCurve(theC);
                break;
            }
        }
    }

    private void PerformCurve(Adaptor2d_Curve2d theC)
    {
        int i, j;
        gp_XYZ V1, V2;
        gp_Pnt MiddlePoint, CurrentPoint, LastPoint;
        double Du, Dusave, MiddleU, L1, L2;

        double U1 = myFirstu;
        double LTol = Precision.Confusion(); // protection against zero length
        double ATol = 1.0e-2 * myAngularDeflection;
        if (ATol > 1.0e-2)
        {
            ATol = 1.0e-2;
        }
        else if (ATol < 1.0e-7)
        {
            ATol = 1.0e-7;
        }

        D0(theC, myLastU, out LastPoint);

        // Initialization of the computation
        bool NotDone = true;
        Dusave = (myLastU - myFirstu) * Us3;
        Du = Dusave;
        EvaluateDu(theC, U1, out CurrentPoint, ref Du, ref NotDone);
        myParameters.Append(U1);
        myPoints.Append(CurrentPoint);

        // Used to detect "isLine" current bspline and in Du computation in general handling.
        int NbInterv = theC.NbIntervals(GeomAbs_CN);
        NCollection_Array1<double> Intervs = new(1, NbInterv + 1);
        theC.Intervals(Intervs, GeomAbs_CN);

        if (NotDone || Du > 5.0 * Dusave)
        {
            // It is either a line or a singularity:
            V1 = (LastPoint.XYZ() - CurrentPoint.XYZ());
            L1 = V1.Modulus();
            if (L1 > LTol)
            {
                // If it is a line, we verify by computing minNbPoints:
                bool IsLine = true;
                int NbPoints = (myMinNbPnts > 3) ? myMinNbPnts : 3;
                switch (theC.GetType())
                {
                    case GeomAbs_BSplineCurve:
                    {
                        Geom2d_BSplineCurve BS = theC.BSpline();
                        NbPoints = Max(BS.Degree() + 1, NbPoints);
                        break;
                    }
                    case GeomAbs_BezierCurve:
                    {
                        Geom2d_BezierCurve BZ = theC.Bezier();
                        NbPoints = Max(BZ.Degree() + 1, NbPoints);
                        break;
                    }
                    default:
                    {
                        break;
                    }
                }

                ////
                double param = 0.0;
                for (i = 1; i <= NbInterv && IsLine; ++i)
                {
                    // Avoid usage intervals out of [myFirstu, myLastU].
                    if ((Intervs[i + 1] < myFirstu) || (Intervs[i] > myLastU))
                    {
                        continue;
                    }

                    // Fix border points in applicable intervals, to avoid be out of target interval.
                    if ((Intervs[i] < myFirstu) && (Intervs[i + 1] > myFirstu))
                    {
                        Intervs[i] = myFirstu;
                    }
                    if ((Intervs[i] < myLastU) && (Intervs[i + 1] > myLastU))
                    {
                        Intervs[i + 1] = myLastU;
                    }

                    double delta = (Intervs[i + 1] - Intervs[i]) / NbPoints;
                    for (j = 1; j <= NbPoints && IsLine; ++j)
                    {
                        param = Intervs[i] + j * delta;
                        D0(theC, param, out MiddlePoint);
                        V2 = (MiddlePoint.XYZ() - CurrentPoint.XYZ());
                        L2 = V2.Modulus();
                        if (L2 > LTol)
                        {
                            double aAngle = V2.CrossMagnitude(V1) / (L1 * L2);
                            IsLine = (aAngle < ATol);
                        }
                    }
                }

                if (IsLine)
                {
                    myParameters.Clear();
                    myPoints.Clear();

                    PerformLinear(theC);
                    return;
                }
                else
                {
                    // It was a singularity, continue:
                    // Du = Dusave;
                    EvaluateDu(theC, param, out MiddlePoint, ref Du, ref NotDone);
                }
            }
            else
            {
                Du = (myLastU - myFirstu) / 2.1;
                MiddleU = myFirstu + Du;
                D0(theC, MiddleU, out MiddlePoint);
                V1 = (MiddlePoint.XYZ() - CurrentPoint.XYZ());
                L1 = V1.Modulus();
                if (L1 < LTol)
                {
                    // L1 < LTol: this is a zero-length curve, computation is finished:
                    // return a segment of 2 points (protection)
                    myParameters.Append(myLastU);
                    myPoints.Append(LastPoint);
                    return;
                }
            }
        }

        if (Du > Dusave)
        {
            Du = Dusave;
        }
        else
        {
            Dusave = Du;
        }

        if (Du < myUTol)
        {
            Du = myLastU - myFirstu;
            if (Du < myUTol)
            {
                myParameters.Append(myLastU);
                myPoints.Append(LastPoint);
                return;
            }
        }

        // Normal processing for a curve
        bool MorePoints = true;
        double U2 = myFirstu;
        double AngleMax = myAngularDeflection * 0.5; // because we take the midpoint

        // Indexes of intervals of U1 and U2, used to handle non-uniform case.
        Span<int> aIdx = [Intervs.Lower(), Intervs.Lower()];
        bool isNeedToCheck = false;
        gp_Pnt aPrevPoint = myPoints.Last();

        while (MorePoints)
        {
            aIdx[0] = getIntervalIdx(U1, Intervs, aIdx[0]);
            U2 += Du;

            if (U2 >= myLastU) // End of curve
            {
                U2 = myLastU;
                CurrentPoint = LastPoint;
                Du = U2 - U1;
                Dusave = Du;
            }
            else
            {
                D0(theC, U2, out CurrentPoint); // Next point
            }

            double Coef = 0.0, ACoef = 0.0, FCoef = 0.0;
            bool Correction, TooLarge, TooSmall;
            TooLarge = false;
            Correction = true;
            TooSmall = false;

            while (Correction) // Adjustment of Du
            {
                if (isNeedToCheck)
                {
                    aIdx[1] = getIntervalIdx(U2, Intervs, aIdx[0]);
                    if (aIdx[1] > aIdx[0]) // Jump to another polynom.
                    {
                        // Set Du to the smallest value and check deflection on it.
                        if (Du > (Intervs[aIdx[0] + 1] - Intervs[aIdx[0]]) * Us3)
                        {
                            Du = (Intervs[aIdx[0] + 1] - Intervs[aIdx[0]]) * Us3;
                            U2 = U1 + Du;
                            if (U2 > myLastU)
                            {
                                U2 = myLastU;
                            }
                            D0(theC, U2, out CurrentPoint);
                        }
                    }
                }
                MiddleU = (U1 + U2) * 0.5; // Verification at the midpoint
                D0(theC, MiddleU, out MiddlePoint);

                V1 = (CurrentPoint.XYZ() - aPrevPoint.XYZ()); // Deflection criterion
                V2 = (MiddlePoint.XYZ() - aPrevPoint.XYZ());
                L1 = V1.Modulus();

                FCoef = (L1 > myMinLen) ? V1.CrossMagnitude(V2) / (L1 * myCurvatureDeflection) : 0.0;

                V1 = (CurrentPoint.XYZ() - MiddlePoint.XYZ()); // Angular criterion
                L1 = V1.Modulus();
                L2 = V2.Modulus();
                if (L1 > myMinLen && L2 > myMinLen)
                {
                    double angg = V1.CrossMagnitude(V2) / (L1 * L2);
                    ACoef = angg / AngleMax;
                }
                else
                {
                    ACoef = 0.0;
                }

                // Keep the most penalizing coefficient
                Coef = Max(ACoef, FCoef);

                if (isNeedToCheck && Coef < 0.55)
                {
                    isNeedToCheck = false;
                    Du = Dusave;
                    U2 = U1 + Du;
                    if (U2 > myLastU)
                    {
                        U2 = myLastU;
                    }
                    D0(theC, U2, out CurrentPoint);
                    continue;
                }

                if (Coef <= 1.0)
                {
                    if (Math.Abs(myLastU - U2) < myUTol)
                    {
                        myParameters.Append(myLastU);
                        myPoints.Append(LastPoint);
                        MorePoints = false;
                        Correction = false;
                    }
                    else
                    {
                        if (Coef >= 0.55 || TooLarge)
                        {
                            myParameters.Append(U2);
                            myPoints.Append(CurrentPoint);
                            aPrevPoint = CurrentPoint;
                            Correction = false;
                            isNeedToCheck = true;
                        }
                        else if (TooSmall)
                        {
                            Correction = false;
                            aPrevPoint = CurrentPoint;
                        }
                        else
                        {
                            TooSmall = true;
                            // double UUU2 = U2;
                            Du += Min((U2 - U1) * (1.0 - Coef), Du * Us3);

                            U2 = U1 + Du;
                            if (U2 > myLastU)
                            {
                                U2 = myLastU;
                            }
                            D0(theC, U2, out CurrentPoint);
                        }
                    }
                }
                else
                {
                    if (Coef >= 1.5)
                    {
                        if (!aPrevPoint.IsEqual(myPoints.Last(), Precision.Confusion()))
                        {
                            myParameters.Append(U1);
                            myPoints.Append(aPrevPoint);
                        }
                        U2 = MiddleU;
                        Du = U2 - U1;
                        CurrentPoint = MiddlePoint;
                    }
                    else
                    {
                        Du *= 0.9;
                        U2 = U1 + Du;
                        D0(theC, U2, out CurrentPoint);
                        TooLarge = true;
                    }
                }
            }

            Du = U2 - U1;

            if (MorePoints)
            {
                if (U1 > myFirstu)
                {
                    if (FCoef > ACoef)
                    {
                        // Deflection is the splitting criterion
                        EvaluateDu(theC, U2, out CurrentPoint, ref Du, ref NotDone);
                        if (NotDone)
                        {
                            Du += (Du - Dusave) * (Du / Dusave);
                            if (Du > 1.5 * Dusave)
                            {
                                Du = 1.5 * Dusave;
                            }
                            if (Du < 0.75 * Dusave)
                            {
                                Du = 0.75 * Dusave;
                            }
                        }
                    }
                    else
                    {
                        // Angle is the splitting criterion
                        Du += (Du - Dusave) * (Du / Dusave);
                        if (Du > 1.5 * Dusave)
                        {
                            Du = 1.5 * Dusave;
                        }
                        if (Du < 0.75 * Dusave)
                        {
                            Du = 0.75 * Dusave;
                        }
                    }
                }

                if (Du < myUTol)
                {
                    Du = myLastU - U2;
                    if (Du < myUTol)
                    {
                        myParameters.Append(myLastU);
                        myPoints.Append(LastPoint);
                        MorePoints = false;
                    }
                    else if (Du * Us3 > myUTol)
                    {
                        Du *= Us3;
                    }
                }
                U1 = U2;
                Dusave = Du;
            }
        }

        // Readjustment of the second to last point:
        i = myPoints.Length() - 1;
        if (i >= 2)
        {
            MiddleU = myParameters[i - 1];
            MiddleU = (myLastU + MiddleU) * 0.5;
            D0(theC, MiddleU, out MiddlePoint);
            myParameters.SetValue(i, MiddleU);
            myPoints.SetValue(i, MiddlePoint);
        }

        // Add points at midpoints of segments if the minimum
        // number of points is not yet reached.
        int Nbp = myPoints.Length();

        while (Nbp < myMinNbPnts)
        {
            for (i = 2; i <= Nbp; i += 2)
            {
                MiddleU = (myParameters.Value(i - 1) + myParameters.Value(i)) * 0.5;
                D0(theC, MiddleU, out MiddlePoint);
                myParameters.InsertBefore(i, MiddleU);
                myPoints.InsertBefore(i, MiddlePoint);
                Nbp++;
            }
        }

        // Additional check for intervals
        double MinLen2 = myMinLen * myMinLen;
        int MaxNbp = 10 * Nbp;
        for (i = 1; i < Nbp; ++i)
        {
            U1 = myParameters[i];
            U2 = myParameters[i + 1];

            if (U2 - U1 <= myUTol)
            {
                continue;
            }

            // Check maximal deflection on interval;
            double dmax = 0.0;
            double umax = 0.0;
            double amax = 0.0;
            EstimDefl(theC, U1, U2, ref dmax, ref umax);
            gp_Pnt P1 = myPoints[i];
            gp_Pnt P2 = myPoints[i + 1];
            D0(theC, umax, out MiddlePoint);
            amax = EstimAngl(P1, MiddlePoint, P2);
            if (dmax > myCurvatureDeflection || amax > AngleMax)
            {
                if (umax - U1 > myUTol && U2 - umax > myUTol)
                {
                    if (P1.SquareDistance(MiddlePoint) > MinLen2 && P2.SquareDistance(MiddlePoint) > MinLen2)
                    {
                        myParameters.InsertAfter(i, umax);
                        myPoints.InsertAfter(i, MiddlePoint);
                        ++Nbp;
                        --i; // To compensate ++i in loop header: i must point to first part of split interval
                        if (Nbp > MaxNbp)
                        {
                            break;
                        }
                    }
                }
            }
        }
    }

    private void EstimDefl(Adaptor2d_Curve2d theC, double theU1, double theU2, ref double theMaxDefl,
        ref double theUMax)
    {
        double Du = (myLastU - myFirstu);
        //
        GCPnts_DistFunction2d aFunc = new(theC, theU1, theU2);
        //
        const int aNbIter = 100;
        double aRelTol = Max(1.0e-3, 2.0 * myUTol / (Math.Abs(theU1) + Math.Abs(theU2)));
        //
        math_BrentMinimum anOptLoc = new(aRelTol, aNbIter, myUTol);
        anOptLoc.Perform(aFunc, theU1, (theU1 + theU2) / 2.0, theU2);
        if (anOptLoc.IsDone())
        {
            theMaxDefl = Math.Sqrt(-anOptLoc.Minimum());
            theUMax = anOptLoc.Location();
            return;
        }
        //
        math_Vector aLowBorder = new(1, 1), aUppBorder = new(1, 1), aSteps = new(1, 1);
        aSteps[1] = Max(0.1 * Du, 100.0 * myUTol);
        int aNbParticles = Max(8, RealToInt(32 * (theU2 - theU1) / Du));
        aLowBorder[1] = theU1;
        aUppBorder[1] = theU2;
        //
        //
        double aValue = 0.0;
        math_Vector aT = new(1, 1);
        GCPnts_DistFunction2dMV aFuncMV = new(aFunc);

        math_PSO aFinder = new(aFuncMV, aLowBorder, aUppBorder, aSteps, aNbParticles);
        aFinder.Perform(aSteps, out aValue, aT);
        //
        anOptLoc.Perform(aFunc,
                         Max(aT[1] - aSteps[1], theU1),
                         aT[1],
                         Min(aT[1] + aSteps[1], theU2));
        if (anOptLoc.IsDone())
        {
            theMaxDefl = Math.Sqrt(-anOptLoc.Minimum());
            theUMax = anOptLoc.Location();
            return;
        }
        theMaxDefl = Math.Sqrt(-aValue);
        theUMax = aT[1];
    }
}
