// Port of: src/ModelingAlgorithms/TKGeomAlgo/LocalAnalysis/LocalAnalysis_SurfaceContinuity.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/LocalAnalysis/LocalAnalysis_SurfaceContinuity.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;
using static Occt.LocalAnalysis_StatusErrorType;

namespace Occt;

/// <summary>
/// The local continuity between a point on one surface and a point on another: the distance (C0), the angles and
/// ratios of the first and second derivatives in u and v (C1, C2), the angle of the normals (G1) and the gap between
/// the curvatures (G2).
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21): the constructors with surfaces compute; the constructor with tolerances only
/// leaves the analysis to <see cref="ComputeAnalysis"/>, which may be called again on the same object
/// (GeomPlate_BuildPlateSurface does, in a loop) and keeps what earlier calls set. The queries but
/// <see cref="IsDone"/> and <see cref="StatusError"/> throw <see cref="NotDoneException"/> when an analysis failed.
/// </para>
/// <para>
/// OCCT leaves <see cref="StatusError"/> uninitialised until an analysis fails, and <see cref="ContinuityStatus"/>
/// uninitialised before the first <see cref="ComputeAnalysis"/>; here they are <c>LocalAnalysis_NullFirstDerivative</c>
/// and <c>GeomAbs_C0</c>.
/// </para>
/// </remarks>
public class LocalAnalysis_SurfaceContinuity
{
    private double myContC0;
    private double myContC1U;
    private double myContC1V;
    private double myContC2U;
    private double myContC2V;
    private double myContG1;
    private double myLambda1U;
    private double myLambda2U;
    private double myLambda1V;
    private double myLambda2V;
    private double myETA1;
    private double myETA2;
    private double myETA;
    private double myZETA1;
    private double myZETA2;
    private double myZETA;
    private double myAlpha;
    private Continuity myTypeCont;
    private readonly double myepsC0;
    private readonly double myepsnul;
    private readonly double myepsC1;
    private readonly double myepsC2;
    private readonly double myepsG1;
    private readonly double myperce;
    private readonly double mymaxlen;
    private double myGap;
    private bool myIsDone;
    private LocalAnalysis_StatusErrorType myErrorStatus;

    private void SurfC0(GeomLProp_SLProps Surf1, GeomLProp_SLProps Surf2)
    {
        myContC0 = (Surf1.Value()).Distance(Surf2.Value());
    }

    private void SurfC1(GeomLProp_SLProps Surf1, GeomLProp_SLProps Surf2)
    {
        Vector3D V1u, V2u, V1v, V2v;
        double norm1u, norm2u, norm1v, norm2v, angu, angv;

        V1u = Surf1.D1U();
        V2u = Surf2.D1U();

        V1v = Surf1.D1V();
        V2v = Surf2.D1V();

        norm1u = V1u.Magnitude();
        norm2u = V2u.Magnitude();
        norm1v = V1v.Magnitude();
        norm2v = V2v.Magnitude();

        if ((norm1u > myepsnul) && (norm2u > myepsnul) && (norm1v > myepsnul) && (norm2v > myepsnul))
        {
            if (norm1u >= norm2u)
            {
                myLambda1U = norm2u / norm1u;
            }
            else
            {
                myLambda1U = norm1u / norm2u;
            }
            if (norm1v >= norm2v)
            {
                myLambda1V = norm2v / norm1v;
            }
            else
            {
                myLambda1V = norm1v / norm2v;
            }
            angu = V1u.Angle(V2u);
            if (angu > M_PI / 2)
            {
                myContC1U = M_PI - angu;
            }
            else
            {
                myContC1U = angu;
            }
            angv = V1v.Angle(V2v);
            if (angv > M_PI / 2)
            {
                myContC1V = M_PI - angv;
            }
            else
            {
                myContC1V = angv;
            }
        }
        else
        {
            myIsDone = false;
            myErrorStatus = LocalAnalysis_NullFirstDerivative;
        }
    }

    private void SurfC2(GeomLProp_SLProps Surf1, GeomLProp_SLProps Surf2)
    {
        Vector3D V11u, V12u, V21u, V22u, V11v, V12v, V21v, V22v;
        double norm11u, norm12u, norm21u, norm22u, norm11v, norm12v, norm21v, norm22v;
        double ang;
        V11u = Surf1.D1U();
        V12u = Surf2.D1U();
        V21u = Surf1.D2U();
        V22u = Surf2.D2U();
        norm11u = V11u.Magnitude();
        norm12u = V12u.Magnitude();
        norm21u = V21u.Magnitude();
        norm22u = V22u.Magnitude();

        if ((norm11u > myepsnul) && (norm12u > myepsnul))
        {
            if ((norm21u > myepsnul) && (norm22u > myepsnul))
            {
                if (norm11u >= norm12u)
                {
                    myLambda1U = norm12u / norm11u;
                    myLambda2U = norm22u / norm21u;
                }
                else
                {
                    myLambda1U = norm11u / norm12u;
                    myLambda2U = norm21u / norm22u;
                }
                ang = V21u.Angle(V22u);
                if (ang > M_PI / 2)
                {
                    myContC2U = M_PI - ang;
                }
                else
                {
                    myContC2U = ang;
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

        V11v = Surf1.D1V();
        V12v = Surf2.D1V();
        V21v = Surf1.D2V();
        V22v = Surf2.D2V();
        norm11v = V11v.Magnitude();
        norm12v = V12v.Magnitude();
        norm21v = V21v.Magnitude();
        norm22v = V22v.Magnitude();

        if ((norm11v > myepsnul) && (norm12v > myepsnul))
        {
            if ((norm21v > myepsnul) && (norm22v > myepsnul))
            {
                if (norm11v >= norm12v)
                {
                    myLambda1V = norm12v / norm11v;
                    myLambda2V = norm22v / norm21v;
                }
                else
                {
                    myLambda1V = norm11v / norm12v;
                    myLambda2V = norm21v / norm22v;
                }
                ang = V21v.Angle(V22v);
                if (ang > M_PI / 2)
                {
                    myContC2V = M_PI - ang;
                }
                else
                {
                    myContC2V = ang;
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

    private void SurfG1(GeomLProp_SLProps Surf1, GeomLProp_SLProps Surf2)
    {
        if (Surf1.IsNormalDefined() && Surf2.IsNormalDefined())
        {
            Direction3D D1 = Surf1.Normal();
            Direction3D D2 = Surf2.Normal();
            double ang = D1.Angle(D2);
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
            myErrorStatus = LocalAnalysis_NormalNotDefined;
        }
    }

    private void SurfG2(GeomLProp_SLProps Surf1, GeomLProp_SLProps Surf2)
    {
        double RMIN1, RMIN2, RMAX1, RMAX2;
        double x1, x2, y1, y2, z1, z2;

        if (Surf1.IsCurvatureDefined() && Surf2.IsCurvatureDefined())
        {
            // OCCT passes DMIN as CurvatureDirections' MaxD and DMAX as its MinD; MCD below is symmetric in them
            Surf1.CurvatureDirections(out Direction3D DMIN1, out Direction3D DMAX1);
            Surf2.CurvatureDirections(out Direction3D DMIN2, out Direction3D DMAX2);
            DMIN1.Coord(out x1, out y1, out z1);
            DMAX1.Coord(out x2, out y2, out z2);
            var MCD1 = new Direction3D((Math.Abs(x1) + Math.Abs(x2)) / 2,
                                  (Math.Abs(y1) + Math.Abs(y2)) / 2,
                                  (Math.Abs(z1) + Math.Abs(z2)) / 2);
            DMIN2.Coord(out x1, out y1, out z1);
            DMAX2.Coord(out x2, out y2, out z2);
            var MCD2 = new Direction3D((Math.Abs(x1) + Math.Abs(x2)) / 2,
                                  (Math.Abs(y1) + Math.Abs(y2)) / 2,
                                  (Math.Abs(z1) + Math.Abs(z2)) / 2);

            myAlpha = MCD1.Angle(MCD2);
            RMIN1 = Surf1.MinCurvature();
            RMAX1 = Surf1.MaxCurvature();
            RMIN2 = Surf2.MinCurvature();
            RMAX2 = Surf2.MaxCurvature();
            myETA1 = (RMIN1 + RMAX1) / 2;
            myETA2 = (RMIN2 + RMAX2) / 2;
            myETA = (myETA1 + myETA2) / 2;
            myZETA1 = (RMAX1 - RMIN1) / 2;
            myZETA2 = (RMAX2 - RMIN2) / 2;
            myZETA = (myZETA1 + myZETA2) / 2;
            double DETA, DZETA;
            DETA = (myETA1 - myETA2) / 2;
            DZETA = (myZETA1 - myZETA2) / 2;
            myGap = Math.Abs(DETA)
                    + Math.Sqrt(DZETA * DZETA * Math.Cos(myAlpha) * Math.Cos(myAlpha)
                                + myZETA * myZETA * Math.Sin(myAlpha) * Math.Sin(myAlpha));
        }
        else
        {
            myIsDone = false;
            myErrorStatus = LocalAnalysis_CurvatureNotDefined;
        }
    }

    /// <summary>The tolerances only; <see cref="ComputeAnalysis"/> then analyses two surfaces' local properties.</summary>
    public LocalAnalysis_SurfaceContinuity(double EpsNul = 0.001, double EpsC0 = 0.001, double EpsC1 = 0.001,
                                           double EpsC2 = 0.001, double EpsG1 = 0.001, double Percent = 0.01,
                                           double Maxlen = 10000)
    {
        myepsnul = EpsNul;
        myepsC0 = EpsC0;
        myepsC1 = EpsC1;
        myepsC2 = EpsC2;
        myepsG1 = EpsG1;
        myperce = Percent;
        mymaxlen = Maxlen;
        myIsDone = true;
    }

    /// <summary>Analyses the continuity of the order between the points of the two surfaces' local properties.</summary>
    /// <remarks>The properties must have been computed to the order's derivative (2 for C2 and G2, 1 for C1 and G1).</remarks>
    public void ComputeAnalysis(GeomLProp_SLProps Surf1, GeomLProp_SLProps Surf2, Continuity Order)
    {
        myTypeCont = Order;
        switch (Order)
        {
            case C0:
                SurfC0(Surf1, Surf2);
                break;
            case C1:
                SurfC0(Surf1, Surf2);
                SurfC1(Surf1, Surf2);
                break;
            case C2:
                SurfC0(Surf1, Surf2);
                SurfC1(Surf1, Surf2);
                SurfC2(Surf1, Surf2);
                break;
            case G1:
                SurfC0(Surf1, Surf2);
                SurfG1(Surf1, Surf2);
                break;
            case G2:
                SurfC0(Surf1, Surf2);
                SurfG1(Surf1, Surf2);
                SurfG2(Surf1, Surf2);
                break;
            default:
                break;
        }
    }

    /// <summary>
    /// Analyses the continuity of the order between <paramref name="Surf1"/> at (<paramref name="u1"/>,
    /// <paramref name="v1"/>) and <paramref name="Surf2"/> at (<paramref name="u2"/>, <paramref name="v2"/>).
    /// </summary>
    /// <remarks>
    /// <paramref name="EpsNul"/> detects null vectors, <paramref name="EpsC0"/> confuses points; <paramref name="EpsC1"/>,
    /// <paramref name="EpsC2"/> and <paramref name="EpsG1"/> are angular tolerances (radians); <paramref name="Percent"/>
    /// bounds the curvature gap for G2; <paramref name="Maxlen"/> is the surfaces' greatest length, used to detect null
    /// curvature.
    /// </remarks>
    public LocalAnalysis_SurfaceContinuity(Geom_Surface Surf1, double u1, double v1, Geom_Surface Surf2, double u2,
                                           double v2, Continuity Order, double EpsNul = 0.001, double EpsC0 = 0.001,
                                           double EpsC1 = 0.001, double EpsC2 = 0.001, double EpsG1 = 0.001,
                                           double Percent = 0.01, double Maxlen = 10000)
    {
        myTypeCont = Order;
        myepsnul = EpsNul;
        myepsC0 = EpsC0;
        myepsC1 = EpsC1;
        myepsC2 = EpsC2;
        myepsG1 = EpsG1;
        myperce = Percent;
        mymaxlen = Maxlen;
        myIsDone = true;
        Analyse(Surf1, u1, v1, Surf2, u2, v2, Order);
    }

    /// <summary>
    /// Analyses the continuity of the order between <paramref name="Surf1"/> at <paramref name="curv1"/>'s point of
    /// parameter <paramref name="U"/> and <paramref name="Surf2"/> at <paramref name="curv2"/>'s; not done when
    /// <paramref name="U"/> is outside either curve's range.
    /// </summary>
    public LocalAnalysis_SurfaceContinuity(Geom2d_Curve curv1, Geom2d_Curve curv2, double U, Geom_Surface Surf1,
                                           Geom_Surface Surf2, Continuity Order, double EpsNul = 0.001,
                                           double EpsC0 = 0.001, double EpsC1 = 0.001, double EpsC2 = 0.001,
                                           double EpsG1 = 0.001, double Percent = 0.01, double Maxlen = 10000)
    {
        double pard1, parf1, pard2, parf2;

        myTypeCont = Order;
        myepsnul = EpsNul;
        myepsC0 = EpsC0;
        myepsC1 = EpsC1;
        myepsC2 = EpsC2;
        myepsG1 = EpsG1;
        myperce = Percent;
        mymaxlen = Maxlen;
        myIsDone = true;

        pard1 = curv1.FirstParameter();
        pard2 = curv2.FirstParameter();
        parf1 = curv1.LastParameter();
        parf2 = curv2.LastParameter();

        if ((U > parf1) || (U < pard1) || (U > parf2) || (U < pard2))
        {
            myIsDone = false;
        }
        else
        {
            Point2D pt1 = curv1.Value(U);
            Point2D pt2 = curv2.Value(U);

            pt1.Coord(out double u1, out double v1);
            pt2.Coord(out double u2, out double v2);
            Analyse(Surf1, u1, v1, Surf2, u2, v2, Order);
        }
    }

    // The switch both constructors with surfaces repeat in OCCT (decision 742's pattern)
    private void Analyse(Geom_Surface Surf1, double u1, double v1, Geom_Surface Surf2, double u2, double v2, Continuity Ordre)
    {
        switch (Ordre)
        {
            case C0:
            {
                var Surfa1 = new GeomLProp_SLProps(Surf1, u1, v1, 0, myepsnul);
                var Surfa2 = new GeomLProp_SLProps(Surf2, u2, v2, 0, myepsnul);
                SurfC0(Surfa1, Surfa2);
                break;
            }
            case C1:
            {
                var Surfa1 = new GeomLProp_SLProps(Surf1, u1, v1, 1, myepsnul);
                var Surfa2 = new GeomLProp_SLProps(Surf2, u2, v2, 1, myepsnul);
                SurfC0(Surfa1, Surfa2);
                SurfC1(Surfa1, Surfa2);
                break;
            }
            case C2:
            {
                var Surfa1 = new GeomLProp_SLProps(Surf1, u1, v1, 2, myepsnul);
                var Surfa2 = new GeomLProp_SLProps(Surf2, u2, v2, 2, myepsnul);
                SurfC0(Surfa1, Surfa2);
                SurfC1(Surfa1, Surfa2);
                SurfC2(Surfa1, Surfa2);
                break;
            }
            case G1:
            {
                var Surfa1 = new GeomLProp_SLProps(Surf1, u1, v1, 1, myepsnul);
                var Surfa2 = new GeomLProp_SLProps(Surf2, u2, v2, 1, myepsnul);
                SurfC0(Surfa1, Surfa2);
                SurfG1(Surfa1, Surfa2);
                break;
            }
            case G2:
            {
                var Surfa1 = new GeomLProp_SLProps(Surf1, u1, v1, 2, myepsnul);
                var Surfa2 = new GeomLProp_SLProps(Surf2, u2, v2, 2, myepsnul);
                SurfC0(Surfa1, Surfa2);
                SurfG1(Surfa1, Surfa2);
                SurfG2(Surfa1, Surfa2);
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
            throw new NotDoneException();
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
        return IsC0() && (myContC1U <= myepsC1) && (myContC1V <= myepsC1);
    }

    public bool IsC2()
    {
        double eps1u, eps1v, eps2u, eps2v;

        CheckDone();
        if (IsC1())
        {
            eps1u = 0.5 * myepsC1 * myepsC1 * myLambda1U;
            eps1v = 0.5 * myepsC1 * myepsC1 * myLambda1V;
            eps2u = 0.5 * myepsC2 * myepsC2 * myLambda2U;
            eps2v = 0.5 * myepsC2 * myepsC2 * myLambda2V;
            if ((myContC2U < myepsC2) && (myContC2V < myepsC2))
            {
                if (Math.Abs(myLambda1U * myLambda1U - myLambda2U) <= (eps1u * eps1u + eps2u))
                {
                    if (Math.Abs(myLambda1V * myLambda1V - myLambda2V) <= (eps1v * eps1v + eps2v))
                    {
                        return true;
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

    public bool IsG1()
    {
        CheckDone();
        return IsC0() && (myContG1 <= myepsG1);
    }

    public bool IsG2()
    {
        double EPSNL;
        int itype;

        CheckDone();
        itype = 0;
        EPSNL = 8 * myepsC0 / (mymaxlen * mymaxlen);
        if (IsG1())
        {
            if ((Math.Abs(myETA) < EPSNL) && (Math.Abs(myZETA) < EPSNL))
            {
                return true;
            }
            if ((Math.Abs(myZETA1) < EPSNL) && (Math.Abs(myZETA2) < EPSNL))
            {
                itype = 1;
            }
            else if ((Math.Abs(myETA1) < EPSNL) && (Math.Abs(myETA2) < EPSNL))
            {
                itype = 1;
            }
            else if ((Math.Abs(Math.Abs(myZETA) - Math.Abs(myETA))) < EPSNL)
            {
                itype = 1;
            }
            else if ((myETA1 < myZETA1) && (myETA2 < myZETA2))
            {
                itype = 1;
            }
            else if ((myETA1 > myZETA1) && (myETA2 > myZETA2))
            {
                itype = 1;
            }
            if (itype == 1)
            {
                if ((myETA >= (2 * myZETA)) && (myGap <= (myperce * (myETA - myZETA))))
                {
                    return true;
                }
                if ((myZETA >= myETA) && (myGap <= (myperce * myZETA)))
                {
                    return true;
                }
                return (myZETA <= myETA) && (myETA <= (2 * myZETA)) && (myGap <= (myperce * myETA));
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

    public Continuity ContinuityStatus()
    {
        CheckDone();
        return (myTypeCont);
    }

    public double C0Value()
    {
        CheckDone();
        return (myContC0);
    }

    public double C1UAngle()
    {
        CheckDone();
        return (myContC1U);
    }

    public double C1VAngle()
    {
        CheckDone();
        return (myContC1V);
    }

    public double C2UAngle()
    {
        CheckDone();
        return (myContC2U);
    }

    public double C2VAngle()
    {
        CheckDone();
        return (myContC2V);
    }

    public double G1Angle()
    {
        CheckDone();
        return (myContG1);
    }

    public double C1URatio()
    {
        CheckDone();
        return (myLambda1U);
    }

    public double C2URatio()
    {
        CheckDone();
        return (myLambda2U);
    }

    public double C1VRatio()
    {
        CheckDone();
        return (myLambda1V);
    }

    public double C2VRatio()
    {
        CheckDone();
        return (myLambda2V);
    }

    public double G2CurvatureGap()
    {
        CheckDone();
        return (myGap);
    }

    public bool IsDone() => (myIsDone);

    public LocalAnalysis_StatusErrorType StatusError() => myErrorStatus;
}
