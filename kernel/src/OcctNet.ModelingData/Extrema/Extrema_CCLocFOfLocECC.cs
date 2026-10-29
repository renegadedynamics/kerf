// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GFuncExtCC.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_CCLocFOfLocECC.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;

namespace Occt;

/// <summary>
/// The system whose roots are the extrema of the distance between two 3D curves:
/// F1(u, v) = (C2(v) - C1(u)).C1'(u) / ||C1'(u)|| and F2(u, v) = (C2(v) - C1(u)).C2'(v) / ||C2'(v)||, with its
/// derivatives, for math_FunctionSetRoot. Each root found (GetStateNumber) whose functions are within the tolerance
/// is stored with its square distance.
/// </summary>
/// <remarks>
/// The 3D instantiation of OCCT's template Extrema_GFuncExtCC (Extrema_CCLocFOfLocECC). OCCT keeps the curves as
/// <c>void*</c>; the port keeps references. Where the first derivative of a B-spline, Bezier, offset or other curve
/// vanishes, it is replaced by the first non-zero higher derivative or by finite differences.
/// </remarks>
public class Extrema_CCLocFOfLocECC : math_FunctionSetWithDerivatives
{
    private const double THE_MIN_TOL = 1.0e-20;
    private const double THE_TOL_FACTOR = 1.0e-12;
    private const double THE_MIN_STEP = 1e-7;
    private const int THE_MAX_ORDER = 3;

    private Adaptor3d_Curve? myC1;
    private Adaptor3d_Curve? myC2;
    private double myTol;
    private double myU;
    private double myV;
    private gp_Pnt myP1 = new();
    private gp_Pnt myP2 = new();
    private gp_Vec myDu = new();
    private gp_Vec myDv = new();
    private readonly NCollection_Sequence<double> mySqDist = new();
    private readonly NCollection_Sequence<Extrema_POnCurv> myPoints = new();
    private double myTolC1;
    private double myTolC2;
    private int myMaxDerivOrderC1;
    private int myMaxDerivOrderC2;
    private double myUinfium;
    private double myUsupremum;
    private double myVinfium;
    private double myVsupremum;

    /// <summary>The function without curves (<see cref="SetCurve"/> sets them).</summary>
    public Extrema_CCLocFOfLocECC(double theTol = 1.0e-10)
    {
        myC1 = null;
        myC2 = null;
        myTol = theTol;
        math_Vector V1 = new(1, 2), V2 = new(1, 2);
        V1[1] = 0.0;
        V2[1] = 0.0;
        V1[2] = 0.0;
        V2[2] = 0.0;
        SubIntervalInitialize(V1, V2);
        myMaxDerivOrderC1 = 0;
        myTolC1 = THE_MIN_TOL;
        myMaxDerivOrderC2 = 0;
        myTolC2 = THE_MIN_TOL;
    }

    /// <summary>The function of the curves theC1 and theC2, on their whole ranges.</summary>
    public Extrema_CCLocFOfLocECC(Adaptor3d_Curve theC1, Adaptor3d_Curve theC2, double theTol = 1.0e-10)
    {
        myC1 = theC1;
        myC2 = theC2;
        myTol = theTol;
        math_Vector V1 = new(1, 2), V2 = new(1, 2);
        V1[1] = Extrema_CurveTool.FirstParameter(myC1);
        V2[1] = Extrema_CurveTool.LastParameter(myC1);
        V1[2] = Extrema_CurveTool.FirstParameter(myC2);
        V2[2] = Extrema_CurveTool.LastParameter(myC2);
        SubIntervalInitialize(V1, V2);

        switch (Extrema_CurveTool.GetType(myC1))
        {
            case GeomAbs_BezierCurve:
            case GeomAbs_BSplineCurve:
            case GeomAbs_OffsetCurve:
            case GeomAbs_OtherCurve:
                myMaxDerivOrderC1 = THE_MAX_ORDER;
                myTolC1 = SearchOfTolerance(theC1);
                break;
            default:
                myMaxDerivOrderC1 = 0;
                myTolC1 = THE_MIN_TOL;
                break;
        }

        switch (Extrema_CurveTool.GetType(myC2))
        {
            case GeomAbs_BezierCurve:
            case GeomAbs_BSplineCurve:
            case GeomAbs_OffsetCurve:
            case GeomAbs_OtherCurve:
                myMaxDerivOrderC2 = THE_MAX_ORDER;
                myTolC2 = SearchOfTolerance(theC2);
                break;
            default:
                myMaxDerivOrderC2 = 0;
                myTolC2 = THE_MIN_TOL;
                break;
        }
    }

    /// <summary>Sets the curve of rank theRank (1 or 2).</summary>
    /// <exception cref="Standard_OutOfRange">theRank is not 1 or 2.</exception>
    public void SetCurve(int theRank, Adaptor3d_Curve theC)
    {
        Standard_OutOfRange_Raise_if(theRank < 1 || theRank > 2, "Extrema_GFuncExtCC::SetCurve()");

        if (theRank == 1)
        {
            myC1 = theC;
            switch (theC.GetType())
            {
                case GeomAbs_BezierCurve:
                case GeomAbs_BSplineCurve:
                case GeomAbs_OffsetCurve:
                case GeomAbs_OtherCurve:
                    myMaxDerivOrderC1 = THE_MAX_ORDER;
                    myTolC1 = SearchOfTolerance(theC);
                    break;
                default:
                    myMaxDerivOrderC1 = 0;
                    myTolC1 = THE_MIN_TOL;
                    break;
            }
        }
        else if (theRank == 2)
        {
            myC2 = theC;
            switch (theC.GetType())
            {
                case GeomAbs_BezierCurve:
                case GeomAbs_BSplineCurve:
                case GeomAbs_OffsetCurve:
                case GeomAbs_OtherCurve:
                    myMaxDerivOrderC2 = THE_MAX_ORDER;
                    myTolC2 = SearchOfTolerance(theC);
                    break;
                default:
                    myMaxDerivOrderC2 = 0;
                    myTolC2 = THE_MIN_TOL;
                    break;
            }
        }
    }

    public void SetTolerance(double theTol) => myTol = theTol;

    public override int NbVariables() => 2;

    public override int NbEquations() => 2;

    /// <summary>F(u, v), in theF; false where a curve's derivative vanishes.</summary>
    public override bool Value(math_Vector theUV, math_Vector theF)
    {
        myU = theUV[1];
        myV = theUV[2];
        Extrema_CurveTool.D1(myC1!, myU, out myP1, out myDu);
        Extrema_CurveTool.D1(myC2!, myV, out myP2, out myDv);

        gp_Vec P1P2 = new(myP1, myP2);

        double Ndu = myDu.Magnitude();

        if (myMaxDerivOrderC1 != 0)
        {
            if (Ndu <= myTolC1)
            {
                const double DivisionFactor = 1.0e-3;
                double du;
                if ((myUsupremum >= RealLast()) || (myUinfium <= RealFirst()))
                {
                    du = 0.0;
                }
                else
                {
                    du = myUsupremum - myUinfium;
                }

                double aDelta = Max(du * DivisionFactor, THE_MIN_STEP);

                int n = 1;
                gp_Vec V;
                bool IsDeriveFound;

                do
                {
                    V = Extrema_CurveTool.DN(myC1!, myU, ++n);
                    Ndu = V.Magnitude();
                    IsDeriveFound = (Ndu > myTolC1);
                } while (!IsDeriveFound && n < myMaxDerivOrderC1);

                if (IsDeriveFound)
                {
                    double u;

                    if (myU - myUinfium < aDelta)
                    {
                        u = myU + aDelta;
                    }
                    else
                    {
                        u = myU - aDelta;
                    }

                    Extrema_CurveTool.D0(myC1!, Min(myU, u), out gp_Pnt P1);
                    Extrema_CurveTool.D0(myC1!, Max(myU, u), out gp_Pnt P2);

                    gp_Vec V1 = new(P1, P2);
                    double aDirFactor = V.Dot(V1);

                    if (aDirFactor < 0.0)
                    {
                        myDu = -V;
                    }
                    else
                    {
                        myDu = V;
                    }
                }
                else
                {
                    gp_Pnt Ptemp = new();
                    gp_Pnt P1, P2, P3;
                    bool IsParameterGrown;

                    if (myU - myUinfium < 2 * aDelta)
                    {
                        Extrema_CurveTool.D0(myC1!, myU, out P1);
                        Extrema_CurveTool.D0(myC1!, myU + aDelta, out P2);
                        Extrema_CurveTool.D0(myC1!, myU + 2 * aDelta, out P3);
                        IsParameterGrown = true;
                    }
                    else
                    {
                        Extrema_CurveTool.D0(myC1!, myU - 2 * aDelta, out P1);
                        Extrema_CurveTool.D0(myC1!, myU - aDelta, out P2);
                        Extrema_CurveTool.D0(myC1!, myU, out P3);
                        IsParameterGrown = false;
                    }

                    gp_Vec V1 = new(Ptemp, P1), V2 = new(Ptemp, P2), V3 = new(Ptemp, P3);

                    if (IsParameterGrown)
                    {
                        myDu = -3 * V1 + 4 * V2 - V3;
                    }
                    else
                    {
                        myDu = V1 - 4 * V2 + 3 * V3;
                    }
                }
                Ndu = myDu.Magnitude();
            }
        }

        if (Ndu <= THE_MIN_TOL)
        {
            return false;
        }

        double Ndv = myDv.Magnitude();

        if (myMaxDerivOrderC2 != 0)
        {
            if (Ndv <= myTolC2)
            {
                const double DivisionFactor = 1.0e-3;
                double dv;
                if ((myVsupremum >= RealLast()) || (myVinfium <= RealFirst()))
                {
                    dv = 0.0;
                }
                else
                {
                    dv = myVsupremum - myVinfium;
                }

                double aDelta = Max(dv * DivisionFactor, THE_MIN_STEP);

                int n = 1;
                gp_Vec V;
                bool IsDeriveFound;

                do
                {
                    V = Extrema_CurveTool.DN(myC2!, myV, ++n);
                    Ndv = V.Magnitude();
                    IsDeriveFound = (Ndv > myTolC2);
                } while (!IsDeriveFound && n < myMaxDerivOrderC2);

                if (IsDeriveFound)
                {
                    double v;

                    if (myV - myVinfium < aDelta)
                    {
                        v = myV + aDelta;
                    }
                    else
                    {
                        v = myV - aDelta;
                    }

                    Extrema_CurveTool.D0(myC2!, Min(myV, v), out gp_Pnt P1);
                    Extrema_CurveTool.D0(myC2!, Max(myV, v), out gp_Pnt P2);

                    gp_Vec V1 = new(P1, P2);
                    double aDirFactor = V.Dot(V1);

                    if (aDirFactor < 0.0)
                    {
                        myDv = -V;
                    }
                    else
                    {
                        myDv = V;
                    }
                }
                else
                {
                    gp_Pnt Ptemp = new();
                    gp_Pnt P1, P2, P3;
                    bool IsParameterGrown;

                    if (myV - myVinfium < 2 * aDelta)
                    {
                        Extrema_CurveTool.D0(myC2!, myV, out P1);
                        Extrema_CurveTool.D0(myC2!, myV + aDelta, out P2);
                        Extrema_CurveTool.D0(myC2!, myV + 2 * aDelta, out P3);
                        IsParameterGrown = true;
                    }
                    else
                    {
                        Extrema_CurveTool.D0(myC2!, myV - 2 * aDelta, out P1);
                        Extrema_CurveTool.D0(myC2!, myV - aDelta, out P2);
                        Extrema_CurveTool.D0(myC2!, myV, out P3);
                        IsParameterGrown = false;
                    }

                    gp_Vec V1 = new(Ptemp, P1), V2 = new(Ptemp, P2), V3 = new(Ptemp, P3);

                    if (IsParameterGrown)
                    {
                        myDv = -3 * V1 + 4 * V2 - V3;
                    }
                    else
                    {
                        myDv = V1 - 4 * V2 + 3 * V3;
                    }
                }

                Ndv = myDv.Magnitude();
            }
        }

        if (Ndv <= THE_MIN_TOL)
        {
            return false;
        }

        theF[1] = P1P2.Dot(myDu) / Ndu;
        theF[2] = P1P2.Dot(myDv) / Ndv;
        return true;
    }

    /// <summary>The derivatives of F at (u, v), in theDF.</summary>
    public override bool Derivatives(math_Vector theUV, math_Matrix theDF)
    {
        math_Vector F = new(1, 2);
        return Values(theUV, F, theDF);
    }

    /// <summary>F and its derivatives at (u, v).</summary>
    public override bool Values(math_Vector theUV, math_Vector theF, math_Matrix theDF)
    {
        myU = theUV[1];
        myV = theUV[2];

        if (Value(theUV, theF) == false)
        {
            return false;
        }

        Extrema_CurveTool.D2(myC1!, myU, out myP1, out gp_Vec Du, out gp_Vec Duu);
        Extrema_CurveTool.D2(myC2!, myV, out myP2, out gp_Vec Dv, out gp_Vec Dvv);

        double myU_old = myU, myV_old = myV;
        gp_Pnt myP1_old = myP1, myP2_old = myP2;
        gp_Vec myDu_old = myDu, myDv_old = myDv;

        const double DivisionFactor = 0.01;

        double du;
        if ((myUsupremum >= RealLast()) || (myUinfium <= RealFirst()))
        {
            du = 0.0;
        }
        else
        {
            du = myUsupremum - myUinfium;
        }

        double aDeltaU = Max(du * DivisionFactor, THE_MIN_STEP);

        double dv;
        if ((myVsupremum >= RealLast()) || (myVinfium <= RealFirst()))
        {
            dv = 0.0;
        }
        else
        {
            dv = myVsupremum - myVinfium;
        }

        double aDeltaV = Max(dv * DivisionFactor, THE_MIN_STEP);

        gp_Vec P1P2 = new(myP1, myP2);

        if ((myMaxDerivOrderC1 != 0) && (Du.Magnitude() <= myTolC1))
        {
            math_Vector FF1 = new(1, 2), FF2 = new(1, 2), FF3 = new(1, 2);
            double F1, F2, F3;

            if (myU - myUinfium < 2 * aDeltaU)
            {
                F1 = theF[1];
                math_Vector UV2 = new(1, 2), UV3 = new(1, 2);
                UV2[1] = myU + aDeltaU;
                UV2[2] = myV;
                UV3[1] = myU + 2 * aDeltaU;
                UV3[2] = myV;
                if (!((Value(UV2, FF2)) && (Value(UV3, FF3))))
                {
                    return false;
                }

                F2 = FF2[1];
                F3 = FF3[1];

                theDF[1, 1] = (-3 * F1 + 4 * F2 - F3) / (2.0 * aDeltaU);
            }
            else
            {
                F3 = theF[1];
                math_Vector UV2 = new(1, 2), UV1 = new(1, 2);
                UV2[1] = myU - aDeltaU;
                UV2[2] = myV;
                UV1[1] = myU - 2 * aDeltaU;
                UV1[2] = myV;

                if (!((Value(UV2, FF2)) && (Value(UV1, FF1))))
                {
                    return false;
                }

                F1 = FF1[1];
                F2 = FF2[1];

                theDF[1, 1] = (F1 - 4 * F2 + 3 * F3) / (2.0 * aDeltaU);
            }

            myU = myU_old;
            myV = myV_old;

            if (myV - myVinfium < 2 * aDeltaV)
            {
                F1 = theF[1];
                math_Vector UV2 = new(1, 2), UV3 = new(1, 2);
                UV2[1] = myU;
                UV2[2] = myV + aDeltaV;
                UV3[1] = myU;
                UV3[2] = myV + 2 * aDeltaV;

                if (!((Value(UV2, FF2)) && (Value(UV3, FF3))))
                {
                    return false;
                }
                F2 = FF2[1];
                F3 = FF3[1];

                theDF[1, 2] = (-3 * F1 + 4 * F2 - F3) / (2.0 * aDeltaV);
            }
            else
            {
                F3 = theF[1];
                math_Vector UV2 = new(1, 2), UV1 = new(1, 2);
                UV2[1] = myU;
                UV2[2] = myV - aDeltaV;
                UV1[1] = myU;
                UV1[2] = myV - 2 * aDeltaV;
                if (!((Value(UV2, FF2)) && (Value(UV1, FF1))))
                {
                    return false;
                }

                F1 = FF1[1];
                F2 = FF2[1];

                theDF[1, 2] = (F1 - 4 * F2 + 3 * F3) / (2.0 * aDeltaV);
            }

            myU = myU_old;
            myV = myV_old;
            myP1 = myP1_old;
            myP2 = myP2_old;
            myDu = myDu_old;
            myDv = myDv_old;
        }
        else
        {
            double Ndu = myDu.Magnitude();
            theDF[1, 1] = -Ndu + (P1P2.Dot(Duu) / Ndu) - theF[1] * (myDu.Dot(Duu) / (Ndu * Ndu));
            theDF[1, 2] = myDv.Dot(myDu) / Ndu;
        }

        if ((myMaxDerivOrderC2 != 0) && (Dv.Magnitude() <= myTolC2))
        {
            math_Vector FF1 = new(1, 2), FF2 = new(1, 2), FF3 = new(1, 2);
            double F1, F2, F3;

            if (myV - myVinfium < 2 * aDeltaV)
            {
                F1 = theF[2];
                math_Vector UV2 = new(1, 2), UV3 = new(1, 2);
                UV2[1] = myU;
                UV2[2] = myV + aDeltaV;
                UV3[1] = myU;
                UV3[2] = myV + 2 * aDeltaV;

                if (!((Value(UV2, FF2)) && (Value(UV3, FF3))))
                {
                    return false;
                }

                F2 = FF2[2];
                F3 = FF3[2];

                theDF[2, 2] = (-3 * F1 + 4 * F2 - F3) / (2.0 * aDeltaV);
            }
            else
            {
                F3 = theF[2];
                math_Vector UV2 = new(1, 2), UV1 = new(1, 2);
                UV2[1] = myU;
                UV2[2] = myV - aDeltaV;
                UV1[1] = myU;
                UV1[2] = myV - 2 * aDeltaV;

                if (!((Value(UV2, FF2)) && (Value(UV1, FF1))))
                {
                    return false;
                }

                F1 = FF1[2];
                F2 = FF2[2];

                theDF[2, 2] = (F1 - 4 * F2 + 3 * F3) / (2.0 * aDeltaV);
            }

            myU = myU_old;
            myV = myV_old;

            if (myU - myUinfium < 2 * aDeltaU)
            {
                F1 = theF[2];
                math_Vector UV2 = new(1, 2), UV3 = new(1, 2);
                UV2[1] = myU + aDeltaU;
                UV2[2] = myV;
                UV3[1] = myU + 2 * aDeltaU;
                UV3[2] = myV;
                if (!((Value(UV2, FF2)) && (Value(UV3, FF3))))
                {
                    return false;
                }

                F2 = FF2[2];
                F3 = FF3[2];

                theDF[2, 1] = (-3 * F1 + 4 * F2 - F3) / (2.0 * aDeltaU);
            }
            else
            {
                F3 = theF[2];
                math_Vector UV2 = new(1, 2), UV1 = new(1, 2);
                UV2[1] = myU - aDeltaU;
                UV2[2] = myV;
                UV1[1] = myU - 2 * aDeltaU;
                UV1[2] = myV;

                if (!((Value(UV2, FF2)) && (Value(UV1, FF1))))
                {
                    return false;
                }

                F1 = FF1[2];
                F2 = FF2[2];

                theDF[2, 1] = (F1 - 4 * F2 + 3 * F3) / (2.0 * aDeltaU);
            }

            myU = myU_old;
            myV = myV_old;
            myP1 = myP1_old;
            myP2 = myP2_old;
            myDu = myDu_old;
            myDv = myDv_old;
        }
        else
        {
            double Ndv = myDv.Magnitude();
            theDF[2, 2] = Ndv + (P1P2.Dot(Dvv) / Ndv) - theF[2] * (myDv.Dot(Dvv) / (Ndv * Ndv));
            theDF[2, 1] = -myDu.Dot(myDv) / Ndv;
        }

        return true;
    }

    /// <summary>Saves the root just found if both functions are within the tolerance there.</summary>
    public override int GetStateNumber()
    {
        gp_Vec Du = myDu, Dv = myDv;
        gp_Vec P1P2 = new(myP1, myP2);

        double mod = Du.Magnitude();
        if (mod > myTolC1)
        {
            Du /= mod;
        }
        mod = Dv.Magnitude();
        if (mod > myTolC2)
        {
            Dv /= mod;
        }

        if (Math.Abs(P1P2.Dot(Du)) <= myTol && Math.Abs(P1P2.Dot(Dv)) <= myTol)
        {
            mySqDist.Append(myP1.SquareDistance(myP2));
            myPoints.Append(new Extrema_POnCurv(myU, myP1));
            myPoints.Append(new Extrema_POnCurv(myV, myP2));
        }
        return 0;
    }

    /// <summary>The number of extrema found.</summary>
    public int NbExt() => mySqDist.Length();

    /// <summary>The Nth square distance.</summary>
    public double SquareDistance(int theN) => mySqDist.Value(theN);

    /// <summary>The points of the Nth extremum, on the first and on the second curve.</summary>
    public void Points(int theN, out Extrema_POnCurv theP1, out Extrema_POnCurv theP2)
    {
        theP1 = myPoints.Value(2 * theN - 1);
        theP2 = myPoints.Value(2 * theN);
    }

    /// <summary>The curve of rank theRank (OCCT's <c>void*</c>).</summary>
    /// <exception cref="Standard_OutOfRange">theRank is not 1 or 2.</exception>
    public Adaptor3d_Curve? CurvePtr(int theRank)
    {
        Standard_OutOfRange_Raise_if(theRank < 1 || theRank > 2, "Extrema_GFuncExtCC::CurvePtr()");
        return (theRank == 1 ? myC1 : myC2);
    }

    public double Tolerance() => myTol;

    /// <summary>The bounds of the root search, for the finite differences.</summary>
    public void SubIntervalInitialize(math_Vector theInfBound, math_Vector theSupBound)
    {
        myUinfium = theInfBound[1];
        myUsupremum = theSupBound[1];
        myVinfium = theInfBound[2];
        myVsupremum = theSupBound[2];
    }

    /// <summary>The tolerance below which the first derivative of theC (one of the two curves, by reference) is taken
    /// as zero: 1e-12 of its maximum on 11 samples, at least 1e-20.</summary>
    public double SearchOfTolerance(Adaptor3d_Curve? theC)
    {
        const int NPoint = 10;
        double aStartParam, anEndParam;

        if (theC == myC1)
        {
            aStartParam = myUinfium;
            anEndParam = myUsupremum;
        }
        else if (theC == myC2)
        {
            aStartParam = myVinfium;
            anEndParam = myVsupremum;
        }
        else
        {
            return THE_MIN_TOL;
        }

        double aStep = (anEndParam - aStartParam) / (double)NPoint;

        int aNum = 0;
        double aMax = -Precision.Infinite();

        do
        {
            double u = aStartParam + aNum * aStep;
            if (u > anEndParam)
            {
                u = anEndParam;
            }

            Extrema_CurveTool.D1(theC!, u, out _, out gp_Vec VDer);
            double vm = VDer.Magnitude();
            if (vm > aMax)
            {
                aMax = vm;
            }
        } while (++aNum < NPoint + 1);

        return Max(aMax * THE_TOL_FACTOR, THE_MIN_TOL);
    }
}
