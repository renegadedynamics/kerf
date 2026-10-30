// Port of: src/ModelingAlgorithms/TKGeomAlgo/LocalAnalysis/LocalAnalysis_CurveContinuity.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/LocalAnalysis/LocalAnalysis_CurveContinuity.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;
using static Occt.LocalAnalysis_StatusErrorType;

namespace Occt;

/// <summary>
/// The local continuity between a point on one curve and a point on another: the distance (C0), the angle and ratio
/// of the first and second derivatives (C1, C2), the angle of the tangents (G1), the angle of the normals and the
/// relative variation of curvature (G2).
/// </summary>
/// <remarks>
/// A working result (decision 21): the constructor computes, the queries read. The queries but <see cref="IsDone"/>
/// and <see cref="StatusError"/> throw <see cref="StdFail_NotDone"/> when the analysis failed. <see cref="StatusError"/>
/// of a successful analysis is uninitialised in OCCT; here it is <c>LocalAnalysis_NullFirstDerivative</c>.
/// </remarks>
public class LocalAnalysis_CurveContinuity
{
    private double myContC0;
    private double myContC1;
    private double myContC2;
    private double myContG1;
    private double myContG2;
    private double myCourbC1;
    private double myCourbC2;
    private double myG2Variation;
    private double myLambda1;
    private double myLambda2;
    private readonly GeomAbs_Shape myTypeCont;
    private readonly double myepsnul;
    private readonly double myepsC0;
    private readonly double myepsC1;
    private readonly double myepsC2;
    private readonly double myepsG1;
    private readonly double myepsG2;
    private readonly double myMaxLon;
    private readonly double myperce;
    private bool myIsDone;
    private LocalAnalysis_StatusErrorType myErrorStatus;

    private void CurvC0(GeomLProp_CLProps Curv1, GeomLProp_CLProps Curv2)
    {
        myContC0 = (Curv1.Value()).Distance(Curv2.Value());
    }

    private void CurvC1(GeomLProp_CLProps Curv1, GeomLProp_CLProps Curv2)
    {
        gp_Vec V1, V2;
        double ang;
        V1 = Curv1.D1();
        V2 = Curv2.D1();
        double norm1, norm2;
        norm1 = V1.Magnitude();
        norm2 = V2.Magnitude();

        if ((norm1 > myepsnul) && (norm2 > myepsnul))
        {
            if (norm1 >= norm2)
            {
                myLambda1 = norm2 / norm1;
            }
            else
            {
                myLambda1 = norm1 / norm2;
            }
            ang = V1.Angle(V2);
            if (ang > M_PI / 2)
            {
                myContC1 = M_PI - ang;
            }
            else
            {
                myContC1 = ang;
            }
        }
        else
        {
            myIsDone = false;
            myErrorStatus = LocalAnalysis_NullFirstDerivative;
        }
    }

    private void CurvC2(GeomLProp_CLProps Curv1, GeomLProp_CLProps Curv2)
    {
        gp_Vec V1, V2, V12, V22;
        double norm1, norm2, norm12, norm22, ang;
        V1 = Curv1.D1();
        V2 = Curv2.D1();
        V12 = Curv1.D2();
        V22 = Curv2.D2();
        norm1 = V1.Magnitude();
        norm2 = V2.Magnitude();
        norm12 = V12.Magnitude();
        norm22 = V22.Magnitude();

        if ((norm1 > myepsnul) && (norm2 > myepsnul))
        {
            if ((norm12 > myepsnul) && (norm22 > myepsnul))
            {
                if (norm1 >= norm2)
                {
                    myLambda1 = norm2 / norm1;
                    myLambda2 = norm22 / norm12;
                }
                else
                {
                    myLambda1 = norm1 / norm2;
                    myLambda2 = norm12 / norm22;
                }
                ang = V12.Angle(V22);
                if (ang > M_PI / 2)
                {
                    myContC2 = M_PI - ang;
                }
                else
                {
                    myContC2 = ang;
                }
            }
            else
            {
                myIsDone = false;
                myErrorStatus = LocalAnalysis_NullSecondDerivative;
            }
        }
        else
        {
            myIsDone = false;
            myErrorStatus = LocalAnalysis_NullFirstDerivative;
        }
    }

    private void CurvG1(GeomLProp_CLProps Curv1, GeomLProp_CLProps Curv2)
    {
        double ang;
        if (Curv1.IsTangentDefined() && Curv2.IsTangentDefined())
        {
            Curv1.Tangent(out gp_Dir Tang1);
            Curv2.Tangent(out gp_Dir Tang2);
            ang = Tang1.Angle(Tang2);
            if (ang > M_PI / 2)
            {
                myContG1 = M_PI - ang;
            }
            else
            {
                myContG1 = ang;
            }
        }
        else
        {
            myIsDone = false;
            myErrorStatus = LocalAnalysis_TangentNotDefined;
        }
    }

    private void CurvG2(GeomLProp_CLProps Curv1, GeomLProp_CLProps Curv2)
    {
        double ang;
        double epscrb = 8 * myepsC0 / (myMaxLon * myMaxLon);

        if (Curv1.IsTangentDefined() && Curv2.IsTangentDefined())
        {
            myCourbC1 = Curv1.Curvature();
            myCourbC2 = Curv2.Curvature();
            if ((Math.Abs(myCourbC1) > epscrb) && (Math.Abs(myCourbC2) > epscrb))
            {
                // OCCT reads the first derivatives here (V1, V2) and never uses them
                _ = Curv1.D1();
                _ = Curv2.D1();
                Curv1.Normal(out gp_Dir D1);
                Curv2.Normal(out gp_Dir D2);
                ang = D1.Angle(D2);
                if (ang > M_PI / 2)
                {
                    myContG2 = M_PI - ang;
                }
                else
                {
                    myContG2 = ang;
                }
                myCourbC1 = Curv1.Curvature();
                myCourbC2 = Curv2.Curvature();
                myG2Variation = Math.Abs(myCourbC1 - myCourbC2) / Math.Sqrt(myCourbC1 * myCourbC2);
            }
            else
            {
                myIsDone = false;
                myErrorStatus = LocalAnalysis_NormalNotDefined;
            }
        }
        else
        {
            myIsDone = false;
            myErrorStatus = LocalAnalysis_TangentNotDefined;
        }
    }

    /// <summary>
    /// Analyses the continuity of the order between <paramref name="Curv1"/> at <paramref name="u1"/> and
    /// <paramref name="Curv2"/> at <paramref name="u2"/>.
    /// </summary>
    /// <remarks>
    /// <paramref name="EpsNul"/> detects null vectors, <paramref name="EpsC0"/> confuses points; <paramref name="EpsC1"/>,
    /// <paramref name="EpsC2"/>, <paramref name="EpsG1"/> and <paramref name="EpsG2"/> are angular tolerances (radians)
    /// on the derivatives, tangents and normals; <paramref name="Percent"/> bounds the relative variation of curvature
    /// for G2; <paramref name="Maxlen"/> is the curves' greatest length, used to detect null curvature. Angles are
    /// between 0 and PI/2.
    /// </remarks>
    public LocalAnalysis_CurveContinuity(Geom_Curve Curv1, double u1, Geom_Curve Curv2, double u2, GeomAbs_Shape Order,
                                         double EpsNul = 0.001, double EpsC0 = 0.001, double EpsC1 = 0.001,
                                         double EpsC2 = 0.001, double EpsG1 = 0.001, double EpsG2 = 0.001,
                                         double Percent = 0.01, double Maxlen = 10000)
    {
        myContC0 = 0.0;
        myContC1 = 0.0;
        myContC2 = 0.0;
        myContG1 = 0.0;
        myContG2 = 0.0;
        myCourbC1 = 0.0;
        myCourbC2 = 0.0;
        myG2Variation = 0.0;
        myLambda1 = 0.0;
        myLambda2 = 0.0;

        myTypeCont = Order;
        myepsnul = EpsNul;
        myMaxLon = Maxlen;
        myepsC0 = EpsC0;
        myepsC1 = EpsC1;
        myepsC2 = EpsC2;
        myepsG1 = EpsG1;
        myepsG2 = EpsG2;
        myperce = Percent;

        myIsDone = true;

        switch (Order)
        {
            case GeomAbs_C0:
            {
                var Curve1 = new GeomLProp_CLProps(Curv1, u1, 0, myepsnul);
                var Curve2 = new GeomLProp_CLProps(Curv2, u2, 0, myepsnul);
                CurvC0(Curve1, Curve2);
                break;
            }
            case GeomAbs_C1:
            {
                var Curve1 = new GeomLProp_CLProps(Curv1, u1, 1, myepsnul);
                var Curve2 = new GeomLProp_CLProps(Curv2, u2, 1, myepsnul);
                CurvC0(Curve1, Curve2);
                CurvC1(Curve1, Curve2);
                break;
            }
            case GeomAbs_C2:
            {
                var Curve1 = new GeomLProp_CLProps(Curv1, u1, 2, myepsnul);
                var Curve2 = new GeomLProp_CLProps(Curv2, u2, 2, myepsnul);
                CurvC0(Curve1, Curve2);
                CurvC1(Curve1, Curve2);
                CurvC2(Curve1, Curve2);
                break;
            }
            case GeomAbs_G1:
            {
                var Curve1 = new GeomLProp_CLProps(Curv1, u1, 1, myepsnul);
                var Curve2 = new GeomLProp_CLProps(Curv2, u2, 1, myepsnul);
                CurvC0(Curve1, Curve2);
                CurvG1(Curve1, Curve2);
                break;
            }
            case GeomAbs_G2:
            {
                var Curve1 = new GeomLProp_CLProps(Curv1, u1, 2, myepsnul);
                var Curve2 = new GeomLProp_CLProps(Curv2, u2, 2, myepsnul);
                CurvC0(Curve1, Curve2);
                CurvG1(Curve1, Curve2);
                CurvG2(Curve1, Curve2);
                break;
            }
            default:
                break;
        }
    }

    private void CheckDone()
    {
        if (!myIsDone)
        {
            throw new StdFail_NotDone();
        }
    }

    public bool IsC0()
    {
        CheckDone();
        return myContC0 <= myepsC0;
    }

    public bool IsC1()
    {
        CheckDone();
        return IsC0() && ((myContC1 <= myepsC1) || (Math.Abs(myContC1 - M_PI) <= myepsC1));
    }

    public bool IsC2()
    {
        double epsil1, epsil2;

        CheckDone();
        if (IsC1())
        {
            if ((myContC2 <= myepsC2) || (Math.Abs(myContC2 - M_PI) <= myepsC2))
            {
                epsil1 = 0.5 * myepsC1 * myepsC1 * myLambda1;
                epsil2 = 0.5 * myepsC2 * myepsC2 * myLambda2;
                if ((Math.Abs(myLambda1 * myLambda1 - myLambda2)) <= (epsil1 * epsil1 + epsil2))
                {
                    return true;
                }
            }
            else
            {
                return false;
            }
        }
        return false;
    }

    public bool IsG1()
    {
        CheckDone();
        return IsC0() && ((myContG1 <= myepsG1 || (Math.Abs(myContG1 - M_PI) <= myepsG1)));
    }

    public bool IsG2()
    {
        double CRBINF, CRBNUL;
        int IETA1, IETA2;
        // the state of each curvature (IETA): 0 null, 1 finite, 2 infinite

        CheckDone();
        if (IsG1())
        {
            CRBINF = 1 / myepsC0;
            CRBNUL = 8 * myepsC0 / (myMaxLon * myMaxLon);

            if (myCourbC1 > CRBINF)
            {
                IETA1 = 2;
            }
            else if (myCourbC1 < CRBNUL)
            {
                IETA1 = 0;
            }
            else
            {
                IETA1 = 1;
            }
            if (myCourbC2 > CRBINF)
            {
                IETA2 = 2;
            }
            else if (myCourbC2 < CRBNUL)
            {
                IETA2 = 0;
            }
            else
            {
                IETA2 = 1;
            }
            if (IETA1 == IETA2)
            {
                if (IETA1 == 1)
                {
                    // RealPart is the fractional part: with the angle in [0, PI/2], this tests myContG2 < myepsG2
                    double eps = RealPart((myContG2 + myepsG2) / M_PI) * M_PI;
                    if (Math.Abs(eps - myepsG2) < myepsG2)
                    {
                        return myG2Variation < myperce;
                    }
                    else
                    {
                        return false;
                    }
                }
                else
                {
                    return true;
                }
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    public double C0Value()
    {
        CheckDone();
        return (myContC0);
    }

    public double C1Angle()
    {
        CheckDone();
        return (myContC1);
    }

    public double C2Angle()
    {
        CheckDone();
        return (myContC2);
    }

    public double G1Angle()
    {
        CheckDone();
        return (myContG1);
    }

    public double G2Angle()
    {
        CheckDone();
        return (myContG2);
    }

    public double C1Ratio()
    {
        CheckDone();
        return (myLambda1);
    }

    public double C2Ratio()
    {
        CheckDone();
        return (myLambda2);
    }

    public double G2CurvatureVariation()
    {
        CheckDone();
        return (myG2Variation);
    }

    public bool IsDone() => (myIsDone);

    public LocalAnalysis_StatusErrorType StatusError() => myErrorStatus;

    public GeomAbs_Shape ContinuityStatus()
    {
        CheckDone();
        return (myTypeCont);
    }
}
