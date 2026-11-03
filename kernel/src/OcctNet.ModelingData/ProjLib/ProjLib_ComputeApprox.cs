// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_ComputeApprox.hxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_ComputeApprox.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// modified by NIZHNY-OFV  Thu Jan 20 11:04:19 2005

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Geometry;
using static Occt.AppParCurves_Constraint;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;

namespace Occt;

/// <summary>
/// Approximates the projection of a 3D curve on an analytical surface (plane, cylinder, cone, sphere or torus) by a
/// 2D curve. On a plane, a B-spline or Bezier curve is projected exactly, pole by pole; otherwise the parametric
/// image of the curve is approximated by a 2D B-spline.
/// </summary>
/// <remarks>
/// A working object (decision 21): the setters and <see cref="Perform"/> are public. It keeps no reference to the
/// adaptors after <see cref="Perform"/>. <see cref="BSpline"/> and <see cref="Bezier"/> freeze the stored curve and
/// return the same object at each call (decision 243).
/// </remarks>
public class ProjLib_ComputeApprox
{
    private Geom2d_BSplineCurve? myBSpline;
    private Geom2d_BezierCurve? myBezier;
    private double myTolerance;
    private int myDegMin;
    private int myDegMax;
    private int myMaxSegments;
    private AppParCurves_Constraint myBndPnt;

    /// <summary>
    /// OCCT's file-local PlaneProjector: optimized point-to-plane projection without gp_Trsf overhead. Pre-computes
    /// the plane basis for efficient batch projection of multiple points.
    /// </summary>
    private readonly struct PlaneProjector
    {
        private readonly double OX, OY, OZ;    // Plane origin
        private readonly double DXx, DXy, DXz; // X direction components
        private readonly double DYx, DYy, DYz; // Y direction components

        public PlaneProjector(in CoordinateSystem thePos)
        {
            Point3D aLoc = thePos.Location();
            OX = aLoc.X();
            OY = aLoc.Y();
            OZ = aLoc.Z();

            Direction3D aXDir = thePos.XDirection();
            DXx = aXDir.X();
            DXy = aXDir.Y();
            DXz = aXDir.Z();

            Direction3D aYDir = thePos.YDirection();
            DYx = aYDir.X();
            DYy = aYDir.Y();
            DYz = aYDir.Z();
        }

        /// <summary>The 2D point X = (P - Origin) . XDirection, Y = (P - Origin) . YDirection.</summary>
        public Point2D Project(in Point3D theP)
        {
            double dX = theP.X() - OX;
            double dY = theP.Y() - OY;
            double dZ = theP.Z() - OZ;
            return new Point2D(dX * DXx + dY * DXy + dZ * DXz, dX * DYx + dY * DYy + dZ * DYz);
        }
    }

    // OFV:
    private static bool IsEqual(double Check, double With, double Toler)
    {
        return (Math.Abs(Check - With) < Toler);
    }

    private static Point2D Function_Value(double U,
                                           Adaptor3d_Curve myCurve,
                                           Adaptor3d_Surface mySurface,
                                           double U1,
                                           double U2,
                                           double V1,
                                           double V2,
                                           bool UCouture,
                                           bool VCouture)
    {
        double S = 0.0, T = 0.0;

        Point3D P3d = myCurve.Value(U);
        SurfaceKind SType = mySurface.GetType();

        switch (SType)
        {
            case Plane:
            {
                Plane3D Plane = mySurface.Plane();
                global::Kerf.Engine.Geometry.ElSLib.Parameters(Plane, P3d, out S, out T);
                break;
            }
            case SurfaceKind.Cylinder:
            {
                Cylinder Cylinder = mySurface.Cylinder();
                global::Kerf.Engine.Geometry.ElSLib.Parameters(Cylinder, P3d, out S, out T);
                break;
            }
            case SurfaceKind.Cone:
            {
                Cone Cone = mySurface.Cone();
                global::Kerf.Engine.Geometry.ElSLib.Parameters(Cone, P3d, out S, out T);
                break;
            }
            case SurfaceKind.Sphere:
            {
                Sphere Sphere = mySurface.Sphere();
                global::Kerf.Engine.Geometry.ElSLib.Parameters(Sphere, P3d, out S, out T);
                break;
            }
            case SurfaceKind.Torus:
            {
                Torus Torus = mySurface.Torus();
                global::Kerf.Engine.Geometry.ElSLib.Parameters(Torus, P3d, out S, out T);
                break;
            }
            default:
                throw new NoSuchObjectException("ProjLib_ComputeApprox::Value");
        }

        if (UCouture)
        {
            if (S < U1 || S > U2)
            {
                S = global::Kerf.Engine.Geometry.ElCLib.InPeriod(S, U1, U2);
            }
        }

        if (VCouture)
        {
            if (SType == SurfaceKind.Sphere)
            {
                if (Math.Abs(S - U1) > M_PI)
                {
                    T = M_PI - T;
                    S = M_PI + S;
                }
                // OCCT: "S > U1 || S < U2" is true whenever U1 < U2 (it was meant to be "S < U1 || S > U2"); InPeriod
                // leaves a value already in [U1, U2[ unchanged, so only a value at U2 moves.
                if (S > U1 || S < U2)
                {
                    S = global::Kerf.Engine.Geometry.ElCLib.InPeriod(S, U1, U2);
                }
            }
            if (T < V1 || T > V2)
            {
                T = global::Kerf.Engine.Geometry.ElCLib.InPeriod(T, V1, V2);
            }
        }

        return new Point2D(S, T);
    }

    private static bool Function_D1(double U,
                                    out Point2D P,
                                    out Vector2D D,
                                    Adaptor3d_Curve myCurve,
                                    Adaptor3d_Surface mySurface,
                                    double U1,
                                    double U2,
                                    double V1,
                                    double V2,
                                    bool UCouture,
                                    bool VCouture)
    {
        Point3D P3d;
        double dU, dV;

        // D is left unset on failure in OCCT; the port gives a zero vector (decision 32).
        D = new Vector2D();
        P = Function_Value(U, myCurve, mySurface, U1, U2, V1, V2, UCouture, VCouture);

        SurfaceKind Type = mySurface.GetType();

        switch (Type)
        {
            case Plane:
            case SurfaceKind.Cone:
            case SurfaceKind.Cylinder:
            case SurfaceKind.Sphere:
            case SurfaceKind.Torus:
            {
                Vector3D D1U, D1V;
                Vector3D T;
                myCurve.D1(U, out P3d, out T);
                mySurface.D1(P.X(), P.Y(), out P3d, out D1U, out D1V);

                dU = T.Dot(D1U);
                dV = T.Dot(D1V);
                double Nu = D1U.SquareMagnitude();
                double Nv = D1V.SquareMagnitude();

                if (Nu < Epsilon(1.0) || Nv < Epsilon(1.0))
                {
                    return false;
                }

                dU /= Nu;
                dV /= Nv;
                D = new Vector2D(dU, dV);
            }
            break;

            default:
                return false;
        }

        return true;
    }

    private static double Function_ComputeStep(Adaptor3d_Curve myCurve, double R)
    {
        double Step0 = .1;
        double W1, W2;
        W1 = myCurve.FirstParameter();
        W2 = myCurve.LastParameter();
        double L = GCPnts_AbscissaPoint.Length(myCurve);
        int nbp = RealToInt(L / (R * M_PI_4)) + 1;
        nbp = Max(nbp, 3);
        double Step = (W2 - W1) / (nbp - 1);
        if (Step > Step0)
        {
            Step = Step0;
            nbp = RealToInt((W2 - W1) / Step) + 1;
            nbp = Max(nbp, 3);
            Step = (W2 - W1) / (nbp - 1);
        }

        return Step;
    }

    private static void Function_SetUVBounds(ref double myU1,
                                             ref double myU2,
                                             ref double myV1,
                                             ref double myV2,
                                             ref bool UCouture,
                                             ref bool VCouture,
                                             Adaptor3d_Curve myCurve,
                                             Adaptor3d_Surface mySurface)
    {
        double W1, W2, W;
        Point3D P1, P2, P;
        //
        W1 = myCurve.FirstParameter();
        W2 = myCurve.LastParameter();
        W = 0.5 * (W1 + W2);
        // open the interval
        // W1 += 1.0e-9;
        // W2 -= 1.0e-9;
        P1 = myCurve.Value(W1);
        P2 = myCurve.Value(W2);
        P = myCurve.Value(W);

        switch (mySurface.GetType())
        {
            case SurfaceKind.Cone:
            {
                double tol = Epsilon(1.0);
                double ptol = global::Kerf.Engine.Foundation.Precision.PConfusion();
                Cone Cone = mySurface.Cone();
                VCouture = false;
                // Calculation of cone parameters for P == ConeApex often produces wrong
                // values of U
                Point3D ConeApex = Cone.Apex();
                if (ConeApex.XYZ().IsEqual(P1.XYZ(), tol))
                {
                    W1 += ptol;
                    P1 = myCurve.Value(W1);
                }
                if (ConeApex.XYZ().IsEqual(P2.XYZ(), tol))
                {
                    W2 -= ptol;
                    P2 = myCurve.Value(W2);
                }
                if (ConeApex.XYZ().IsEqual(P.XYZ(), tol))
                {
                    W += ptol;
                    P = myCurve.Value(W);
                }

                switch (myCurve.GetType())
                {
                    case Parabola:
                    case Hyperbola:
                    case Ellipse:
                    {
                        double U1, U2, V1, V2, U, V;
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(Cone, P1, out U1, out V1);
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(Cone, P2, out U2, out V2);
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(Cone, P, out U, out V);
                        myU1 = Min(U1, U2);
                        myU2 = Max(U1, U2);
                        if ((U1 < U && U < U2) && !myCurve.IsClosed())
                        {
                            UCouture = false;
                        }
                        else
                        {
                            UCouture = true;
                            myU2 = myU1 + 2 * M_PI;
                        }
                    }
                    break;
                    default:
                    {
                        double U1, V1, U, V, Delta = 0.0, d = 0.0, pmin = W1, pmax = W1, dmax = 0.0, Uf, Ul;
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(Cone, P1, out U1, out V1);
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(Cone, P2, out Ul, out V1);
                        Axis3D anAx1 = Cone.Axis();
                        Line3D aLin = new(anAx1);
                        double R = (aLin.Distance(P1) + aLin.Distance(P2) + aLin.Distance(P)) / 3.0;
                        double Step;
                        myU1 = U1;
                        myU2 = U1;
                        Uf = U1;
                        if (myCurve.GetType() == Line)
                        {
                            int nbp = 3;
                            Step = (W2 - W1) / (nbp - 1);
                        }
                        else
                        {
                            Step = Function_ComputeStep(myCurve, R);
                        }
                        //
                        bool isclandper = (!(myCurve.IsClosed()) && !(myCurve.IsPeriodic()));
                        bool isFirst = true;
                        for (double par = W1 + Step; par <= W2; par += Step)
                        {
                            if (!isclandper)
                            {
                                par += Step;
                            }
                            P = myCurve.Value(par);
                            global::Kerf.Engine.Geometry.ElSLib.Parameters(Cone, P, out U, out V);
                            U += Delta;
                            d = U - U1;
                            if (d > M_PI)
                            {
                                if (((IsEqual(U, (2 * M_PI), 1.0e-10) && (U1 >= 0.0 && U1 <= M_PI))
                                     && (IsEqual(U, Ul, 1.0e-10) && !IsEqual(Uf, 0.0, 1.0e-10)))
                                    && isclandper)
                                {
                                    U = 0.0;
                                }
                                else
                                {
                                    // Protection against first-last point on seam.
                                    if (isFirst)
                                    {
                                        U1 = 2 * M_PI;
                                    }
                                    else if (par + Step >= W2)
                                    {
                                        U = 0.0;
                                    }
                                    else
                                    {
                                        Delta -= 2 * M_PI;
                                    }
                                }
                                U += Delta;
                                d = U - U1;
                            }
                            else if (d < -M_PI)
                            {
                                if (((IsEqual(U, 0.0, 1.0e-10) && (U1 >= M_PI && U1 <= (2 * M_PI)))
                                     && (IsEqual(U, Ul, 1.0e-10) && !IsEqual(Uf, (2 * M_PI), 1.0e-10)))
                                    && isclandper)
                                {
                                    U = 2 * M_PI;
                                }
                                else
                                {
                                    // Protection against first-last point on seam.
                                    if (isFirst)
                                    {
                                        U1 = 0.0;
                                    }
                                    else if (par + Step >= W2)
                                    {
                                        U = 2 * M_PI;
                                    }
                                    else
                                    {
                                        Delta += 2 * M_PI;
                                    }
                                }
                                U += Delta;
                                d = U - U1;
                            }
                            dmax = Max(dmax, Math.Abs(d));
                            if (U < myU1)
                            {
                                myU1 = U;
                                pmin = par;
                            }
                            if (U > myU2)
                            {
                                myU2 = U;
                                pmax = par;
                            }
                            U1 = U;
                            isFirst = false;
                        } // for(double par = W1 + Step; par <= W2; par += Step)

                        if (Math.Abs(pmin - W1) > global::Kerf.Engine.Foundation.Precision.PConfusion()
                            && Math.Abs(pmin - W2) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                        {
                            myU1 -= dmax * .5;
                        }
                        if (Math.Abs(pmax - W1) > global::Kerf.Engine.Foundation.Precision.PConfusion()
                            && Math.Abs(pmax - W2) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                        {
                            myU2 += dmax * .5;
                        }

                        if ((myU1 >= 0.0 && myU1 <= 2 * M_PI) && (myU2 >= 0.0 && myU2 <= 2 * M_PI))
                        {
                            UCouture = false;
                        }
                        else
                        {
                            U = (myU1 + myU2) / 2.0;
                            myU1 = U - M_PI;
                            myU2 = U + M_PI;
                            UCouture = true;
                        }
                    }
                    break;
                } // switch curve type
            } // case Cone
            break;

            case SurfaceKind.Cylinder:
            {
                Cylinder Cylinder = mySurface.Cylinder();
                VCouture = false;

                if (myCurve.GetType() == Ellipse)
                {
                    double U1, U2, V1, V2, U, V;
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(Cylinder, P1, out U1, out V1);
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(Cylinder, P2, out U2, out V2);
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(Cylinder, P, out U, out V);
                    myU1 = Min(U1, U2);
                    myU2 = Max(U1, U2);

                    if (!myCurve.IsClosed())
                    {
                        if (myU1 < U && U < myU2)
                        {
                            U = (myU1 + myU2) / 2.0;
                            myU1 = U - M_PI;
                            myU2 = U + M_PI;
                        }
                        else
                        {
                            U = (myU1 + myU2) / 2.0;
                            if (myU1 < U)
                            {
                                myU1 = U - 2 * M_PI;
                                myU2 = U;
                            }
                            else
                            {
                                myU1 = U;
                                myU2 = U + 2 * M_PI;
                            }
                        }
                        UCouture = true;
                    }
                    else
                    {
                        Vector3D D1U, D1V;
                        Vector3D T;
                        Point3D P3d;
                        myCurve.D1(W1, out P3d, out T);
                        // OCCT: evaluates the surface at (U1, U2), two U parameters, where (U1, V1) was meant; on a
                        // cylinder D1U depends only on U, so only the sign test below is affected, through U1.
                        mySurface.D1(U1, U2, out P3d, out D1U, out D1V);
                        double dU = T.Dot(D1U);

                        UCouture = true;
                        if (dU > 0.0)
                        {
                            myU2 = myU1 + 2 * M_PI;
                        }
                        else
                        {
                            myU2 = myU1;
                            myU1 -= 2 * M_PI;
                        }
                    }
                }
                else
                {
                    double U1, V1, U, V;
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(Cylinder, P1, out U1, out V1);
                    double R = Cylinder.Radius();
                    double Delta = 0.0, Step;
                    double eps = M_PI, dmax = 0.0, d = 0.0;
                    Step = Function_ComputeStep(myCurve, R);
                    myU1 = U1;
                    myU2 = U1;
                    double pmin = W1, pmax = W1, plim = W2 + .1 * Step;
                    for (double par = W1 + Step; par <= plim; par += Step)
                    {
                        P = myCurve.Value(par);
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(Cylinder, P, out U, out V);
                        U += Delta;
                        d = U - U1;
                        if (d > eps)
                        {
                            U -= Delta;
                            Delta -= 2 * M_PI;
                            U += Delta;
                            d = U - U1;
                        }
                        else if (d < -eps)
                        {
                            U -= Delta;
                            Delta += 2 * M_PI;
                            U += Delta;
                            d = U - U1;
                        }
                        dmax = Max(dmax, Math.Abs(d));
                        if (U < myU1)
                        {
                            myU1 = U;
                            pmin = par;
                        }
                        if (U > myU2)
                        {
                            myU2 = U;
                            pmax = par;
                        }
                        U1 = U;
                    }

                    if (Math.Abs(pmin - W1) > global::Kerf.Engine.Foundation.Precision.PConfusion()
                        && Math.Abs(pmin - W2) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        myU1 -= dmax * .5;
                    }
                    if (Math.Abs(pmax - W1) > global::Kerf.Engine.Foundation.Precision.PConfusion()
                        && Math.Abs(pmax - W2) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        myU2 += dmax * .5;
                    }

                    if ((myU1 >= 0.0 && myU1 <= 2 * M_PI) && (myU2 >= 0.0 && myU2 <= 2 * M_PI))
                    {
                        UCouture = false;
                    }
                    else
                    {
                        U = (myU1 + myU2) / 2.0;
                        myU1 = U - M_PI;
                        myU2 = U + M_PI;
                        UCouture = true;
                    }
                }
            }
            break;
            //
            case SurfaceKind.Sphere:
            {
                VCouture = false;
                Sphere SP = mySurface.Sphere();
                if (myCurve.GetType() == Circle)
                {
                    UCouture = true;

                    // we seek to know how many times the seam is
                    // crossed.
                    // if 0 or 2 times: the PCurve is closed and in the interval
                    //                  [Uc-PI, Uc+PI] (Uc: U of circle center)
                    // if 1 time      : the PCurve is open and in the interval
                    //                  [U1, U1 +/- 2*PI]

                    // to determine the number of solutions, we solve the system
                    // x^2 + y^2 + z^2     = R^2  (1)
                    // A x + B y + C z + D = 0    (2)
                    // x > 0                      (3)
                    // y = 0                      (4)
                    // REM : (1) (2)     : equation of the circle
                    //       (1) (3) (4) : equation of the seam.
                    int NbSolutions = 0;
                    double A, B, C, D, R, Tol = 1.0e-10;
                    double U1, U2, V1, V2;
                    Transform3D Trsf = new();
                    //
                    Circle3D Circle = myCurve.Circle();
                    Trsf.SetTransformation(SP.Position());
                    Circle.Transform(Trsf);
                    //
                    R = SP.Radius();
                    Plane3D Plane = new(new CoordinateSystem(Circle.Position()));
                    Plane.Coefficients(out A, out B, out C, out D);
                    //
                    if (Math.Abs(C) < Tol)
                    {
                        if (Math.Abs(A) > Tol)
                        {
                            if ((D / A) < 0.0)
                            {
                                if ((R - Math.Abs(D / A)) > Tol)
                                {
                                    NbSolutions = 2;
                                }
                                else if (Math.Abs(R - Math.Abs(D / A)) < Tol)
                                {
                                    NbSolutions = 1;
                                }
                                else
                                {
                                    NbSolutions = 0;
                                }
                            }
                        }
                    }
                    else
                    {
                        double delta = R * R * (A * A + C * C) - D * D;
                        delta *= C * C;
                        if (Math.Abs(delta) < Tol * Tol)
                        {
                            if (A * D > 0.0)
                            {
                                NbSolutions = 1;
                            }
                        }
                        else if (delta > 0)
                        {
                            double xx;
                            delta = Math.Sqrt(delta);
                            xx = -A * D + delta;
                            //
                            if (xx > Tol)
                            {
                                NbSolutions++;
                            }
                            xx = -A * D - delta;
                            //
                            if (xx > Tol)
                            {
                                NbSolutions++;
                            }
                        }
                    }
                    //

                    // box+sphere >>
                    double UU = 0.0;
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(SP, P1, out U1, out V1);
                    double eps = 10.0 * Epsilon(1.0);
                    double dt = Max(global::Kerf.Engine.Foundation.Precision.PConfusion(), 0.01 * (W2 - W1));
                    if (Math.Abs(U1) < eps)
                    {
                        // May be U1 must be equal 2*PI?
                        Point3D Pd = myCurve.Value(W1 + dt);
                        double ud, vd;
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(SP, Pd, out ud, out vd);
                        if (Math.Abs(U1 - ud) > M_PI)
                        {
                            U1 = 2.0 * M_PI;
                        }
                    }
                    else if (Math.Abs(2.0 * M_PI - U1) < eps)
                    {
                        // maybe U1 = 0.?
                        Point3D Pd = myCurve.Value(W1 + dt);
                        double ud, vd;
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(SP, Pd, out ud, out vd);
                        if (Math.Abs(U1 - ud) > M_PI)
                        {
                            U1 = 0.0;
                        }
                    }
                    //
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(SP, P2, out U2, out V1);
                    if (Math.Abs(U2) < eps)
                    {
                        // May be U2 must be equal 2*PI?
                        Point3D Pd = myCurve.Value(W2 - dt);
                        double ud, vd;
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(SP, Pd, out ud, out vd);
                        if (Math.Abs(U2 - ud) > M_PI)
                        {
                            U2 = 2.0 * M_PI;
                        }
                    }
                    else if (Math.Abs(2.0 * M_PI - U2) < eps)
                    {
                        // maybe U2 = 0.?
                        Point3D Pd = myCurve.Value(W2 - dt);
                        double ud, vd;
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(SP, Pd, out ud, out vd);
                        if (Math.Abs(U2 - ud) > M_PI)
                        {
                            U2 = 0.0;
                        }
                    }
                    //
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(SP, P, out UU, out V1);
                    // OCCT: U2, adjusted to the seam just above, is overwritten here with the U of the point at
                    // W1 + PI/8 (a curve parameter offset, whatever the curve's parameterisation).
                    P2 = myCurve.Value(W1 + M_PI / 8);
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(SP, P2, out U2, out V2);
                    //
                    if (NbSolutions == 1)
                    {
                        if (Math.Abs(U1 - U2) > M_PI)
                        { // crossing the seam
                            if (U1 > M_PI)
                            {
                                myU1 = U1;
                                myU2 = U1 + 2 * M_PI;
                            }
                            else
                            {
                                myU2 = U1;
                                myU1 = U1 - 2 * M_PI;
                            }
                        }
                        else
                        { // not crossing the seam
                            if (U1 > U2)
                            {
                                myU2 = U1;
                                myU1 = U1 - 2 * M_PI;
                            }
                            else
                            {
                                myU1 = U1;
                                myU2 = U1 + 2 * M_PI;
                            }
                        }
                    }
                    else
                    { // 0 or 2 solutions
                        Point3D Center = Circle.Location();
                        double U, V;
                        global::Kerf.Engine.Geometry.ElSLib.SphereParameters(new CoordinateSystem(Constants.XOY()), 1, Center, out U, out V);
                        myU1 = U - M_PI;
                        myU2 = U + M_PI;
                    }
                    //
                    // eval the VCouture.
                    if ((C == 0) || Math.Abs(Math.Abs(D / C) - R) > 1.0e-10)
                    {
                        VCouture = false;
                    }
                    else
                    {
                        VCouture = true;
                        UCouture = true;

                        if (D / C < 0.0)
                        {
                            myV1 = -M_PI / 2.0;
                            myV2 = 3 * M_PI / 2.0;
                        }
                        else
                        {
                            myV1 = -3 * M_PI / 2.0;
                            myV2 = M_PI / 2.0;
                        }

                        // if P1.Z() equals +/- R we are at the vertex: not significant.
                        Point3D pp = P1.Transformed(Trsf);

                        if (Math.Abs(pp.X() * pp.X() + pp.Y() * pp.Y() + pp.Z() * pp.Z() - R * R) < Tol)
                        {
                            Point3D Center = Circle.Location();
                            double U, V;
                            global::Kerf.Engine.Geometry.ElSLib.SphereParameters(new CoordinateSystem(Constants.XOY()), 1, Center, out U, out V);
                            myU1 = U - M_PI;
                            myU2 = U + M_PI;
                            VCouture = false;
                        }
                    }

                    // box+sphere >>
                    myV1 = -1.0e+100;
                    myV2 = 1.0e+100;
                    // box+sphere <<
                } // if ( myCurve->GetType() == GeomAbs_Circle)

                else
                {
                    double U1, V1, U, V;
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(SP, P1, out U1, out V1);
                    double R = SP.Radius();
                    double Delta = 0.0, Step;
                    double eps = M_PI, dmax = 0.0, d = 0.0;
                    Step = Function_ComputeStep(myCurve, R);
                    myU1 = U1;
                    myU2 = U1;
                    double pmin = W1, pmax = W1, plim = W2 + .1 * Step;
                    for (double par = W1 + Step; par <= plim; par += Step)
                    {
                        P = myCurve.Value(par);
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(SP, P, out U, out V);
                        U += Delta;
                        d = U - U1;
                        if (d > eps)
                        {
                            U -= Delta;
                            Delta -= 2 * M_PI;
                            U += Delta;
                            d = U - U1;
                        }
                        else if (d < -eps)
                        {
                            U -= Delta;
                            Delta += 2 * M_PI;
                            U += Delta;
                            d = U - U1;
                        }
                        dmax = Max(dmax, Math.Abs(d));
                        if (U < myU1)
                        {
                            myU1 = U;
                            pmin = par;
                        }
                        if (U > myU2)
                        {
                            myU2 = U;
                            pmax = par;
                        }
                        U1 = U;
                    }

                    if (Math.Abs(pmin - W1) > global::Kerf.Engine.Foundation.Precision.PConfusion()
                        && Math.Abs(pmin - W2) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        myU1 -= dmax * .5;
                    }
                    if (Math.Abs(pmax - W1) > global::Kerf.Engine.Foundation.Precision.PConfusion()
                        && Math.Abs(pmax - W2) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        myU2 += dmax * .5;
                    }

                    if ((myU1 >= 0.0 && myU1 <= 2 * M_PI) && (myU2 >= 0.0 && myU2 <= 2 * M_PI))
                    {
                        myU1 = 0.0;
                        myU2 = 2.0 * M_PI;
                        UCouture = false;
                    }
                    else
                    {
                        U = (myU1 + myU2) / 2.0;
                        myU1 = U - M_PI;
                        myU2 = U + M_PI;
                        UCouture = true;
                    }

                    VCouture = false;
                }
            }
            break;
            //
            case SurfaceKind.Torus:
            {
                Torus TR = mySurface.Torus();
                double U1, V1, U, V, dU, dV;
                global::Kerf.Engine.Geometry.ElSLib.Parameters(TR, P1, out U1, out V1);
                double R = TR.MinorRadius();
                double DeltaU = 0.0, DeltaV = 0.0, Step;
                double eps = M_PI, dmaxU = 0.0, dmaxV = 0.0;
                Step = Function_ComputeStep(myCurve, R);
                myU1 = U1;
                myU2 = U1;
                myV1 = V1;
                myV2 = V1;
                double pminU = W1, pmaxU = W1, pminV = W1, pmaxV = W1, plim = W2 + .1 * Step;
                for (double par = W1 + Step; par <= plim; par += Step)
                {
                    P = myCurve.Value(par);
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(TR, P, out U, out V);
                    U += DeltaU;
                    V += DeltaV;
                    dU = U - U1;
                    dV = V - V1;
                    if (dU > eps)
                    {
                        U -= DeltaU;
                        DeltaU -= 2 * M_PI;
                        U += DeltaU;
                        dU = U - U1;
                    }
                    else if (dU < -eps)
                    {
                        U -= DeltaU;
                        DeltaU += 2 * M_PI;
                        U += DeltaU;
                        dU = U - U1;
                    }
                    if (dV > eps)
                    {
                        V -= DeltaV;
                        DeltaV -= 2 * M_PI;
                        V += DeltaV;
                        dV = V - V1;
                    }
                    else if (dV < -eps)
                    {
                        V -= DeltaV;
                        DeltaV += 2 * M_PI;
                        V += DeltaV;
                        dV = V - V1;
                    }
                    dmaxU = Max(dmaxU, Math.Abs(dU));
                    dmaxV = Max(dmaxV, Math.Abs(dV));
                    if (U < myU1)
                    {
                        myU1 = U;
                        pminU = par;
                    }
                    if (U > myU2)
                    {
                        myU2 = U;
                        pmaxU = par;
                    }
                    if (V < myV1)
                    {
                        myV1 = V;
                        pminV = par;
                    }
                    if (V > myV2)
                    {
                        myV2 = V;
                        pmaxV = par;
                    }
                    U1 = U;
                    V1 = V;
                }

                if (Math.Abs(pminU - W1) > global::Kerf.Engine.Foundation.Precision.PConfusion()
                    && Math.Abs(pminU - W2) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    myU1 -= dmaxU * .5;
                }
                if (Math.Abs(pmaxU - W1) > global::Kerf.Engine.Foundation.Precision.PConfusion()
                    && Math.Abs(pmaxU - W2) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    myU2 += dmaxU * .5;
                }
                if (Math.Abs(pminV - W1) > global::Kerf.Engine.Foundation.Precision.PConfusion()
                    && Math.Abs(pminV - W2) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    myV1 -= dmaxV * .5;
                }
                if (Math.Abs(pmaxV - W1) > global::Kerf.Engine.Foundation.Precision.PConfusion()
                    && Math.Abs(pmaxV - W2) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    myV2 += dmaxV * .5;
                }

                if ((myU1 >= 0.0 && myU1 <= 2 * M_PI) && (myU2 >= 0.0 && myU2 <= 2 * M_PI))
                {
                    myU1 = 0.0;
                    myU2 = 2.0 * M_PI;
                    UCouture = false;
                }
                else
                {
                    U = (myU1 + myU2) / 2.0;
                    myU1 = U - M_PI;
                    myU2 = U + M_PI;
                    UCouture = true;
                }
                if ((myV1 >= 0.0 && myV1 <= 2 * M_PI) && (myV2 >= 0.0 && myV2 <= 2 * M_PI))
                {
                    VCouture = false;
                }
                else
                {
                    V = (myV1 + myV2) / 2.0;
                    myV1 = V - M_PI;
                    myV2 = V + M_PI;
                    VCouture = true;
                }
            }
            break;

            default:
            {
                UCouture = false;
                VCouture = false;
            }
            break;
        }
    }

    /// <summary>OCCT's file-local class ProjLib_Function: the 2D point of the projection at a parameter.</summary>
    private sealed class ProjLib_Function : AppCont_Function
    {
        private readonly Adaptor3d_Curve myCurve;
        private readonly Adaptor3d_Surface mySurface;
        private readonly bool[] myIsPeriodic = new bool[2];
        private readonly double[] myPeriod = new double[2];

        public double myU1, myU2, myV1, myV2;
        public bool UCouture, VCouture;

        public ProjLib_Function(Adaptor3d_Curve C, Adaptor3d_Surface S)
        {
            myCurve = C;
            mySurface = S;
            myU1 = 0.0;
            myU2 = 0.0;
            myV1 = 0.0;
            myV2 = 0.0;
            UCouture = false;
            VCouture = false;
            myNbPnt = 0;
            myNbPnt2d = 1;
            Function_SetUVBounds(ref myU1, ref myU2, ref myV1, ref myV2, ref UCouture, ref VCouture, myCurve, mySurface);
            myIsPeriodic[0] = mySurface.IsUPeriodic();
            myIsPeriodic[1] = mySurface.IsVPeriodic();

            if (myIsPeriodic[0])
            {
                myPeriod[0] = mySurface.UPeriod();
            }
            else
            {
                myPeriod[0] = 0.0;
            }

            if (myIsPeriodic[1])
            {
                myPeriod[1] = mySurface.VPeriod();
            }
            else
            {
                myPeriod[1] = 0.0;
            }
        }

        public override void PeriodInformation(int theDimIdx, out bool IsPeriodic, out double thePeriod)
        {
            IsPeriodic = myIsPeriodic[theDimIdx - 1];
            thePeriod = myPeriod[theDimIdx - 1];
        }

        public override double FirstParameter() => (myCurve.FirstParameter());

        public override double LastParameter() => (myCurve.LastParameter());

        public override bool Value(double theT, NCollection_Array1<Point2D> thePnt2d, NCollection_Array1<Point3D> thePnt)
        {
            thePnt2d[1] =
                Function_Value(theT, myCurve, mySurface, myU1, myU2, myV1, myV2, UCouture, VCouture);
            return true;
        }

        public Point2D Value(double theT)
        {
            return Function_Value(theT, myCurve, mySurface, myU1, myU2, myV1, myV2, UCouture, VCouture);
        }

        public override bool D1(double theT, NCollection_Array1<Vector2D> theVec2d, NCollection_Array1<Vector3D> theVec)
        {
            Point2D aPnt2d;
            Vector2D aVec2d;
            bool isOk = Function_D1(theT,
                                    out aPnt2d,
                                    out aVec2d,
                                    myCurve,
                                    mySurface,
                                    myU1,
                                    myU2,
                                    myV1,
                                    myV2,
                                    UCouture,
                                    VCouture);
            theVec2d[1] = aVec2d;
            return isOk;
        }
    }

    private static double ComputeTolU(Adaptor3d_Surface theSurf, double theTolerance)
    {
        double aTolU = theSurf.UResolution(theTolerance);
        if (theSurf.IsUPeriodic())
        {
            aTolU = Min(aTolU, 0.01 * theSurf.UPeriod());
        }

        return aTolU;
    }

    private static double ComputeTolV(Adaptor3d_Surface theSurf, double theTolerance)
    {
        double aTolV = theSurf.VResolution(theTolerance);
        if (theSurf.IsVPeriodic())
        {
            aTolV = Min(aTolV, 0.01 * theSurf.VPeriod());
        }

        return aTolV;
    }

    /// <summary>An empty algorithm with the tolerance Precision::PApproximation().</summary>
    public ProjLib_ComputeApprox()
    {
        myTolerance = global::Kerf.Engine.Foundation.Precision.PApproximation();
        myDegMin = -1;
        myDegMax = -1;
        myMaxSegments = -1;
        myBndPnt = AppParCurves_TangencyPoint;
    }

    /// <summary>
    /// Approximates the projection of <paramref name="C"/> on <paramref name="S"/> within the 3D tolerance
    /// <paramref name="Tol"/> (at least Precision::PApproximation()).
    /// </summary>
    /// <exception cref="NoSuchObjectException">The surface is not analytical and the case is not a B-spline or Bezier
    /// curve on a plane.</exception>
    public ProjLib_ComputeApprox(Adaptor3d_Curve C, Adaptor3d_Surface S, double Tol)
    {
        myTolerance = Max(Tol, global::Kerf.Engine.Foundation.Precision.PApproximation());
        myDegMin = -1;
        myDegMax = -1;
        myMaxSegments = -1;
        myBndPnt = AppParCurves_TangencyPoint;
        Perform(C, S);
    }

    /// <summary>
    /// Computes the projection. If the surface is a plane and the curve a B-spline or a Bezier curve, the poles are
    /// projected (no approximation).
    /// </summary>
    /// <exception cref="NoSuchObjectException">The surface is not analytical and the case is not a B-spline or Bezier
    /// curve on a plane.</exception>
    public void Perform(Adaptor3d_Curve C, Adaptor3d_Surface S)
    {
        // if the surface is a plane and the curve a BSpline or a BezierCurve,
        // don`t make an Approx but only the projection of the poles.

        int NbKnots, NbPoles;
        CurveKind CType = C.GetType();
        SurfaceKind SType = S.GetType();

        bool SurfIsAnal = ProjLib.IsAnaSurf(S);

        bool CurvIsAnal = (CType != BSplineCurve) && (CType != BezierCurve)
                          && (CType != OffsetCurve) && (CType != OtherCurve);

        bool simplecase = SurfIsAnal && CurvIsAnal;
        if (CType == BSplineCurve || CType == BezierCurve)
        {
            int aNbKnots = 1;
            if (CType == BSplineCurve)
            {
                aNbKnots = C.NbKnots();
            }
            simplecase = simplecase && C.Degree() <= 2 && aNbKnots <= 2;
        }

        if (CType == BSplineCurve && SType == Plane)
        {
            // get the poles and eventually the weights
            Geom_BSplineCurve BS = C.BSpline();
            NbPoles = BS.NbPoles();
            NCollection_Array1<Point3D> P3d = BS.InternalPoles();
            NCollection_Array1<Point2D> Poles = new(1, NbPoles);

            // Project poles onto plane using optimized projector (avoids gp_Trsf per point)
            PlaneProjector aProj = new(S.Plane().Position());
            for (int i = 1; i <= NbPoles; i++)
            {
                Poles.SetValue(i, aProj.Project(P3d[i]));
            }
            NCollection_Array1<double> Knots = BS.InternalKnots();
            NCollection_Array1<int> Mults = BS.InternalMultiplicities();
            // get the knots and mults if BSplineCurve
            if (BS.IsRational())
            {
                myBSpline = new Geom2d_BSplineCurve(Poles,
                                                    BS.InternalWeightsArray(),
                                                    Knots,
                                                    Mults,
                                                    BS.Degree(),
                                                    BS.IsPeriodic());
            }
            else
            {
                myBSpline = new Geom2d_BSplineCurve(Poles, Knots, Mults, BS.Degree(), BS.IsPeriodic());
            }
        }
        else if (CType == BezierCurve && SType == Plane)
        {
            // get the poles and eventually the weights
            Geom_BezierCurve BezierCurvePtr = C.Bezier();
            NbPoles = BezierCurvePtr.NbPoles();
            NCollection_Array1<Point3D> P3d = BezierCurvePtr.InternalPoles();
            NCollection_Array1<Point2D> Poles = new(1, NbPoles);

            // Project poles onto plane using optimized projector (avoids gp_Trsf per point)
            PlaneProjector aProj = new(S.Plane().Position());
            for (int i = 1; i <= NbPoles; i++)
            {
                Poles.SetValue(i, aProj.Project(P3d[i]));
            }
            if (BezierCurvePtr.IsRational())
            {
                myBezier = new Geom2d_BezierCurve(Poles, BezierCurvePtr.InternalWeightsArray());
            }
            else
            {
                myBezier = new Geom2d_BezierCurve(Poles);
            }
        }
        else
        {
            ProjLib_Function F = new(C, S);

            //-----------
            int Deg1 = 5, Deg2;
            if (simplecase)
            {
                Deg2 = 8;
            }
            else
            {
                Deg2 = 10;
            }
            if (myDegMin > 0)
            {
                Deg1 = myDegMin;
            }
            //
            if (myDegMax > 0)
            {
                Deg2 = myDegMax;
            }
            //
            int aMaxSegments = 1000;
            if (myMaxSegments > 0)
            {
                aMaxSegments = myMaxSegments;
            }
            AppParCurves_Constraint aFistC = AppParCurves_TangencyPoint,
                                    aLastC = AppParCurves_TangencyPoint;
            if (myBndPnt != AppParCurves_TangencyPoint)
            {
                aFistC = myBndPnt;
                aLastC = myBndPnt;
            }

            //-------------
            double aTolU = ComputeTolU(S, myTolerance);
            double aTolV = ComputeTolV(S, myTolerance);
            double aTol2d =
                Max(Math.Sqrt(aTolU * aTolU + aTolV * aTolV), global::Kerf.Engine.Foundation.Precision.PConfusion());

            Approx_FitAndDivide2d Fit = new(Deg1, Deg2, myTolerance, aTol2d, true, aFistC, aLastC);
            Fit.SetMaxSegments(aMaxSegments);
            if (simplecase)
            {
                Fit.SetHangChecking(false);
            }
            Fit.Perform(F);

            double aNewTol2d = 0;
            if (Fit.IsAllApproximated())
            {
                int i;
                int NbCurves = Fit.NbMultiCurves();

                // try to make the curve at least C1
                CompBezierCurves2dToBSplineCurve2d Conv = new();

                double Tol3d, Tol2d;
                for (i = 1; i <= NbCurves; i++)
                {
                    Fit.Error(i, out Tol3d, out Tol2d);
                    aNewTol2d = Max(aNewTol2d, Tol2d);
                    AppParCurves_MultiCurve MC = Fit.Value(i);               // Load the i-th Curve
                    NCollection_Array1<Point2D> Poles2d = new(1, MC.Degree() + 1); // Retrieve the poles
                    MC.Curve(1, Poles2d);
                    Conv.AddCurve(Poles2d);
                }

                // update the fields of ProjLib_Approx
                Conv.Perform();
                NbPoles = Conv.PoleCount;
                NbKnots = Conv.KnotCount;

                if (NbPoles <= 0 || NbPoles > 100000)
                {
                    return;
                }
                if (NbKnots <= 0 || NbKnots > 100000)
                {
                    return;
                }

                NCollection_Array1<Point2D> NewPoles = new(1, NbPoles);
                NCollection_Array1<double> NewKnots = new(1, NbKnots);
                NCollection_Array1<int> NewMults = new(1, NbKnots);

                Conv.KnotsAndMults(NewKnots, NewMults);
                Conv.Poles(NewPoles);

                global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(C.FirstParameter(), C.LastParameter(), NewKnots);

                // Set NewKnots(NbKnots) exactly C->LastParameter()
                // to avoid problems if trim is used.
                NewKnots[NbKnots] = C.LastParameter();

                // The start and end poles need to be adjusted:
                // (Due to seam issues, we had to open the definition interval
                // of the curve.)
                // We choose to compute these poles by extending the approximated
                // curve.
                myBSpline = new Geom2d_BSplineCurve(NewPoles, NewKnots, NewMults, Conv.Degree());

                if (aFistC == AppParCurves_PassPoint || aLastC == AppParCurves_PassPoint)
                {
                    // try to smoother the Curve GeomAbs_C1.
                    int aDeg = myBSpline.Degree();
                    bool OK = true;
                    double aSmoothTol = Max(global::Kerf.Engine.Foundation.Precision.Confusion(), aNewTol2d);
                    for (int ij = 2; ij < NbKnots; ij++)
                    {
                        OK = OK && myBSpline.RemoveKnot(ij, aDeg - 1, aSmoothTol);
                    }
                }
            }
            else
            {
                int NbCurves = Fit.NbMultiCurves();
                if (NbCurves != 0)
                {
                    double Tol3d, Tol2d;
                    Fit.Error(NbCurves, out Tol3d, out Tol2d);
                    aNewTol2d = Tol2d;
                }
            }

            // restore tolerance 3d from 2d

            // Here we consider that
            //    aTolU(new)/aTolV(new) = aTolU(old)/aTolV(old)
            //(it is assumption indeed).
            // Then,
            //   Tol3D(new)/Tol3D(old) = Tol2D(new)/Tol2D(old).
            myTolerance *= (aNewTol2d / aTol2d);

            // Return curve home
            double UFirst = F.FirstParameter();
            double ULast = F.LastParameter();
            double Umid = (UFirst + ULast) / 2;
            Point3D P3d = C.Value(Umid);
            double u = 0.0, v = 0.0;
            switch (SType)
            {
                case Plane:
                {
                    Plane3D Plane = S.Plane();
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(Plane, P3d, out u, out v);
                    break;
                }
                case SurfaceKind.Cylinder:
                {
                    Cylinder Cylinder = S.Cylinder();
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(Cylinder, P3d, out u, out v);
                    break;
                }
                case SurfaceKind.Cone:
                {
                    Cone Cone = S.Cone();
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(Cone, P3d, out u, out v);
                    break;
                }
                case SurfaceKind.Sphere:
                {
                    Sphere Sphere = S.Sphere();
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(Sphere, P3d, out u, out v);
                    break;
                }
                case SurfaceKind.Torus:
                {
                    Torus Torus = S.Torus();
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(Torus, P3d, out u, out v);
                    break;
                }
                default:
                    throw new NoSuchObjectException("ProjLib_ComputeApprox::Value");
            }
            bool ToMirror = false;
            double du = 0.0, dv = 0.0;
            int number;
            if (F.VCouture)
            {
                if (SType == SurfaceKind.Sphere && Math.Abs(u - F.myU1) > M_PI)
                {
                    ToMirror = true;
                    dv = -M_PI;
                    v = M_PI - v;
                }
                double newV = global::Kerf.Engine.Geometry.ElCLib.InPeriod(v, F.myV1, F.myV2);
                number = (int)(Math.Floor((newV - v) / (F.myV2 - F.myV1)));
                dv -= number * (F.myV2 - F.myV1);
            }
            if (F.UCouture || (F.VCouture && SType == SurfaceKind.Sphere))
            {
                double aNbPer;
                Point2D P2d = F.Value(Umid);
                du = u - P2d.X();
                du = (du < 0) ? (du - global::Kerf.Engine.Foundation.Precision.PConfusion()) : (du + global::Kerf.Engine.Foundation.Precision.PConfusion());
                aNbPer = Math.Truncate(du / M_PI); // modf's integral part
                number = (int)aNbPer;
                du = number * M_PI;
            }

            if (myBSpline is not null)
            {
                if (du != 0.0 || dv != 0.0)
                {
                    myBSpline.Translate(new Vector2D(du, dv));
                }
                if (ToMirror)
                {
                    Axis2D Axe = new(new Point2D(0.0, 0.0), new Direction2D(Direction2D.D.X));
                    myBSpline.Mirror(Axe);
                }
            }
        }
    }

    /// <summary>Sets the 3D tolerance of the approximation.</summary>
    public void SetTolerance(double theTolerance)
    {
        myTolerance = theTolerance;
    }

    /// <summary>Sets the minimal and maximal degrees of the approximation (a value of 0 or less keeps the
    /// default).</summary>
    public void SetDegree(int theDegMin, int theDegMax)
    {
        myDegMin = theDegMin;
        myDegMax = theDegMax;
    }

    /// <summary>Sets the maximal number of segments of the approximation (0 or less keeps the default, 1000).</summary>
    public void SetMaxSegments(int theMaxSegments)
    {
        myMaxSegments = theMaxSegments;
    }

    /// <summary>Sets the constraint at the ends of the approximation (by default AppParCurves_TangencyPoint).</summary>
    public void SetBndPnt(AppParCurves_Constraint theBndPnt)
    {
        myBndPnt = theBndPnt;
    }

    /// <summary>The B-spline result, frozen (the same object at each call), or null.</summary>
    public Geom2d_BSplineCurve? BSpline()
    {
        myBSpline?.Freeze();
        return myBSpline;
    }

    /// <summary>The B-spline result, unfrozen, for ported code that discards the algorithm.</summary>
    internal Geom2d_BSplineCurve? InternalBSpline() => myBSpline;

    /// <summary>The Bezier result (a Bezier curve on a plane), frozen (the same object at each call), or null.</summary>
    public Geom2d_BezierCurve? Bezier()
    {
        myBezier?.Freeze();
        return myBezier;
    }

    /// <summary>The Bezier result, unfrozen, for ported code that discards the algorithm.</summary>
    internal Geom2d_BezierCurve? InternalBezier() => myBezier;

    /// <summary>The reached 3D tolerance.</summary>
    public double Tolerance() => myTolerance;
}
