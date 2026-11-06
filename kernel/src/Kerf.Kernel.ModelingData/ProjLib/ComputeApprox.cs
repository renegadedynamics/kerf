// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_ComputeApprox.hxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_ComputeApprox.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// modified by NIZHNY-OFV  Thu Jan 20 11:04:19 2005

using Kerf.Engine.Approximation;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Measurement;
using Kerf.Geometry;
using static Kerf.Engine.Approximation.Constraint;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Geometry.Projection;

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
internal class ComputeApprox
{
    private BSplineCurve2D? _bSpline;
    private BezierCurve2D? _bezier;
    private double _tolerance;
    private int _degMin;
    private int _degMax;
    private int _maxSegments;
    private Constraint _bndPnt;

    /// <summary>
    /// OCCT's file-local PlaneProjector: optimized point-to-plane projection without gp_Trsf overhead. Pre-computes
    /// the plane basis for efficient batch projection of multiple points.
    /// </summary>
    private readonly struct PlaneProjector
    {
        private readonly double _ox, _oy, _oz;    // Plane origin
        private readonly double _dXx, _dXy, _dXz; // X direction components
        private readonly double _dYx, _dYy, _dYz; // Y direction components

        public PlaneProjector(in CoordinateSystem pos)
        {
            Point3D loc = pos.Location;
            _ox = loc.X;
            _oy = loc.Y;
            _oz = loc.Z;

            Direction3D xDir = pos.XDirection;
            _dXx = xDir.X;
            _dXy = xDir.Y;
            _dXz = xDir.Z;

            Direction3D yDir = pos.YDirection;
            _dYx = yDir.X;
            _dYy = yDir.Y;
            _dYz = yDir.Z;
        }

        /// <summary>The 2D point X = (P - Origin) . XDirection, Y = (P - Origin) . YDirection.</summary>
        public Point2D Project(in Point3D p)
        {
            double dX = p.X - _ox;
            double dY = p.Y - _oy;
            double dZ = p.Z - _oz;
            return new Point2D(dX * _dXx + dY * _dXy + dZ * _dXz, dX * _dYx + dY * _dYy + dZ * _dYz);
        }
    }

    // OFV:
    private static bool IsEqual(double check, double with, double toler)
    {
        return (Math.Abs(check - with) < toler);
    }

    private static Point2D Function_Value(double u,
                                           CurveAdaptorBase myCurve,
                                           SurfaceAdaptorBase mySurface,
                                           double u1,
                                           double u2,
                                           double v1,
                                           double v2,
                                           bool uCouture,
                                           bool vCouture,
                                           Winding? uWinding = null,
                                           Winding? vWinding = null)
    {
        double s = 0.0, t = 0.0;

        Point3D p3d = myCurve.Value(u);
        SurfaceKind sType = mySurface.GetType();

        switch (sType)
        {
            case Plane:
            {
                Plane3D plane = mySurface.Plane();
                global::Kerf.Engine.Geometry.ElSLib.Parameters(plane, p3d, out s, out t);
                break;
            }
            case SurfaceKind.Cylinder:
            {
                Cylinder3D cylinder = mySurface.Cylinder();
                global::Kerf.Engine.Geometry.ElSLib.Parameters(cylinder, p3d, out s, out t);
                break;
            }
            case SurfaceKind.Cone:
            {
                Cone3D cone = mySurface.Cone();
                global::Kerf.Engine.Geometry.ElSLib.Parameters(cone, p3d, out s, out t);
                break;
            }
            case SurfaceKind.Sphere:
            {
                Sphere3D sphere = mySurface.Sphere();
                global::Kerf.Engine.Geometry.ElSLib.Parameters(sphere, p3d, out s, out t);
                break;
            }
            case SurfaceKind.Torus:
            {
                Torus3D torus = mySurface.Torus();
                global::Kerf.Engine.Geometry.ElSLib.Parameters(torus, p3d, out s, out t);
                break;
            }
            default:
                throw new NoSuchObjectException("ProjLib_ComputeApprox::Value");
        }

        if (uCouture)
        {
            if (uWinding is not null)
            {
                s = uWinding.Unwrap(u, s);
            }
            else if (s < u1 || s > u2)
            {
                s = global::Kerf.Engine.Geometry.ElCLib.InPeriod(s, u1, u2);
            }
        }

        if (vCouture)
        {
            if (sType == SurfaceKind.Sphere)
            {
                if (Math.Abs(s - u1) > M_PI)
                {
                    t = M_PI - t;
                    s = M_PI + s;
                }
                // OCCT: "S > U1 || S < U2" is true whenever U1 < U2 (it was meant to be "S < U1 || S > U2"); InPeriod
                // leaves a value already in [U1, U2[ unchanged, so only a value at U2 moves.
                if (s > u1 || s < u2)
                {
                    s = global::Kerf.Engine.Geometry.ElCLib.InPeriod(s, u1, u2);
                }
            }
            if (vWinding is not null)
            {
                t = vWinding.Unwrap(u, t);
            }
            else if (t < v1 || t > v2)
            {
                t = global::Kerf.Engine.Geometry.ElCLib.InPeriod(t, v1, v2);
            }
        }

        return new Point2D(s, t);
    }

    private static bool Function_D1(double u,
                                    out Point2D p,
                                    out Vector2D d,
                                    CurveAdaptorBase myCurve,
                                    SurfaceAdaptorBase mySurface,
                                    double u1,
                                    double u2,
                                    double v1,
                                    double v2,
                                    bool uCouture,
                                    bool vCouture,
                                    Winding? uWinding,
                                    Winding? vWinding)
    {
        Point3D p3d;
        double dU, dV;

        // D is left unset on failure in OCCT; the port gives a zero vector (decision 32).
        d = new Vector2D();
        p = Function_Value(u, myCurve, mySurface, u1, u2, v1, v2, uCouture, vCouture, uWinding, vWinding);

        SurfaceKind type = mySurface.GetType();

        switch (type)
        {
            case Plane:
            case SurfaceKind.Cone:
            case SurfaceKind.Cylinder:
            case SurfaceKind.Sphere:
            case SurfaceKind.Torus:
            {
                Vector3D d1U, d1V;
                Vector3D t;
                myCurve.D1(u, out p3d, out t);
                mySurface.D1(p.X, p.Y, out p3d, out d1U, out d1V);

                dU = t.Dot(d1U);
                dV = t.Dot(d1V);
                double nu = d1U.SquareMagnitude;
                double nv = d1V.SquareMagnitude;

                if (nu < Epsilon(1.0) || nv < Epsilon(1.0))
                {
                    return false;
                }

                dU /= nu;
                dV /= nv;
                d = new Vector2D(dU, dV);
            }
            break;

            default:
                return false;
        }

        return true;
    }

    private static double Function_ComputeStep(CurveAdaptorBase myCurve, double r)
    {
        double step0 = .1;
        double w1, w2;
        w1 = myCurve.FirstParameter();
        w2 = myCurve.LastParameter();
        double l = AbscissaPoint.Length(myCurve);
        int nbp = RealToInt(l / (r * M_PI_4)) + 1;
        nbp = Max(nbp, 3);
        double step = (w2 - w1) / (nbp - 1);
        if (step > step0)
        {
            step = step0;
            nbp = RealToInt((w2 - w1) / step) + 1;
            nbp = Max(nbp, 3);
            step = (w2 - w1) / (nbp - 1);
        }

        return step;
    }

    private static void Function_SetUVBounds(ref double myU1,
                                             ref double myU2,
                                             ref double myV1,
                                             ref double myV2,
                                             ref bool uCouture,
                                             ref bool vCouture,
                                             CurveAdaptorBase myCurve,
                                             SurfaceAdaptorBase mySurface,
                                             out Winding? uWinding,
                                             out Winding? vWinding)
    {
        uWinding = null;
        vWinding = null;
        double w1, w2, w;
        Point3D p1, p2, p;
        //
        w1 = myCurve.FirstParameter();
        w2 = myCurve.LastParameter();
        w = 0.5 * (w1 + w2);
        // open the interval
        // W1 += 1.0e-9;
        // W2 -= 1.0e-9;
        p1 = myCurve.Value(w1);
        p2 = myCurve.Value(w2);
        p = myCurve.Value(w);

        switch (mySurface.GetType())
        {
            case SurfaceKind.Cone:
            {
                double tol = Epsilon(1.0);
                double ptol = global::Kerf.Engine.Foundation.Precision.PConfusion();
                Cone3D cone = mySurface.Cone();
                vCouture = false;
                // Calculation of cone parameters for P == ConeApex often produces wrong
                // values of U
                Point3D coneApex = cone.Apex;
                if (coneApex.XYZ().IsEqual(p1.XYZ(), tol))
                {
                    w1 += ptol;
                    p1 = myCurve.Value(w1);
                }
                if (coneApex.XYZ().IsEqual(p2.XYZ(), tol))
                {
                    w2 -= ptol;
                    p2 = myCurve.Value(w2);
                }
                if (coneApex.XYZ().IsEqual(p.XYZ(), tol))
                {
                    w += ptol;
                    p = myCurve.Value(w);
                }

                switch (myCurve.GetType())
                {
                    case Parabola:
                    case Hyperbola:
                    case Ellipse:
                    {
                        double u1, u2, v1, v2, u, v;
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(cone, p1, out u1, out v1);
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(cone, p2, out u2, out v2);
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(cone, p, out u, out v);
                        myU1 = Min(u1, u2);
                        myU2 = Max(u1, u2);
                        if ((u1 < u && u < u2) && !myCurve.IsClosed)
                        {
                            uCouture = false;
                        }
                        else
                        {
                            uCouture = true;
                            myU2 = myU1 + 2 * M_PI;
                        }
                    }
                    break;
                    default:
                    {
                        double u1, v1, u, v, Delta = 0.0, d = 0.0, pmin = w1, pmax = w1, dmax = 0.0, uf, ul;
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(cone, p1, out u1, out v1);
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(cone, p2, out ul, out v1);
                        Axis3D ax1 = cone.Axis;
                        Line3D lin = new(ax1);
                        double r = (lin.DistanceTo(p1) + lin.DistanceTo(p2) + lin.DistanceTo(p)) / 3.0;
                        double step;
                        myU1 = u1;
                        myU2 = u1;
                        uf = u1;
                        if (myCurve.GetType() == Line)
                        {
                            int nbp = 3;
                            step = (w2 - w1) / (nbp - 1);
                        }
                        else
                        {
                            step = Function_ComputeStep(myCurve, r);
                        }
                        //
                        bool isclandper = (!(myCurve.IsClosed) && !(myCurve.IsPeriodic));
                        bool isFirst = true;
                        var aUTrack = new Winding(w1, u1);
                        for (double par = w1 + step; par <= w2; par += step)
                        {
                            if (!isclandper)
                            {
                                par += step;
                            }
                            p = myCurve.Value(par);
                            global::Kerf.Engine.Geometry.ElSLib.Parameters(cone, p, out u, out v);
                            u += Delta;
                            d = u - u1;
                            if (d > M_PI)
                            {
                                if (((IsEqual(u, (2 * M_PI), 1.0e-10) && (u1 >= 0.0 && u1 <= M_PI))
                                     && (IsEqual(u, ul, 1.0e-10) && !IsEqual(uf, 0.0, 1.0e-10)))
                                    && isclandper)
                                {
                                    u = 0.0;
                                }
                                else
                                {
                                    // Protection against first-last point on seam.
                                    if (isFirst)
                                    {
                                        u1 = 2 * M_PI;
                                    }
                                    else if (par + step >= w2)
                                    {
                                        u = 0.0;
                                    }
                                    else
                                    {
                                        // the new turn replaces the previous one (OCCT added both, so a second
                                        // crossing of the seam unwrapped wrongly: decision 1459)
                                        u -= Delta;
                                        Delta -= 2 * M_PI;
                                    }
                                }
                                u += Delta;
                                d = u - u1;
                            }
                            else if (d < -M_PI)
                            {
                                if (((IsEqual(u, 0.0, 1.0e-10) && (u1 >= M_PI && u1 <= (2 * M_PI)))
                                     && (IsEqual(u, ul, 1.0e-10) && !IsEqual(uf, (2 * M_PI), 1.0e-10)))
                                    && isclandper)
                                {
                                    u = 2 * M_PI;
                                }
                                else
                                {
                                    // Protection against first-last point on seam.
                                    if (isFirst)
                                    {
                                        u1 = 0.0;
                                    }
                                    else if (par + step >= w2)
                                    {
                                        u = 2 * M_PI;
                                    }
                                    else
                                    {
                                        // the new turn replaces the previous one (OCCT added both, so a second
                                        // crossing of the seam unwrapped wrongly: decision 1459)
                                        u -= Delta;
                                        Delta += 2 * M_PI;
                                    }
                                }
                                u += Delta;
                                d = u - u1;
                            }
                            dmax = Max(dmax, Math.Abs(d));
                            if (u < myU1)
                            {
                                myU1 = u;
                                pmin = par;
                            }
                            if (u > myU2)
                            {
                                myU2 = u;
                                pmax = par;
                            }
                            if (isFirst)
                            {
                                aUTrack.ResetFirst(u1);
                            }
                            aUTrack.Add(par, u);
                            u1 = u;
                            isFirst = false;
                        } // for(double par = W1 + Step; par <= W2; par += Step)

                        if (Math.Abs(pmin - w1) > global::Kerf.Engine.Foundation.Precision.PConfusion()
                            && Math.Abs(pmin - w2) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                        {
                            myU1 -= dmax * .5;
                        }
                        if (Math.Abs(pmax - w1) > global::Kerf.Engine.Foundation.Precision.PConfusion()
                            && Math.Abs(pmax - w2) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                        {
                            myU2 += dmax * .5;
                        }

                        if (aUTrack.IsMoreThanATurn(dmax))
                        {
                            uWinding = aUTrack;
                            uCouture = true;
                        }
                        else if ((myU1 >= 0.0 && myU1 <= 2 * M_PI) && (myU2 >= 0.0 && myU2 <= 2 * M_PI))
                        {
                            uCouture = false;
                        }
                        else
                        {
                            u = (myU1 + myU2) / 2.0;
                            myU1 = u - M_PI;
                            myU2 = u + M_PI;
                            uCouture = true;
                        }
                    }
                    break;
                } // switch curve type
            } // case Cone
            break;

            case SurfaceKind.Cylinder:
            {
                Cylinder3D cylinder = mySurface.Cylinder();
                vCouture = false;

                if (myCurve.GetType() == Ellipse)
                {
                    double u1, u2, v1, v2, u, v;
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(cylinder, p1, out u1, out v1);
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(cylinder, p2, out u2, out v2);
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(cylinder, p, out u, out v);
                    myU1 = Min(u1, u2);
                    myU2 = Max(u1, u2);

                    if (!myCurve.IsClosed)
                    {
                        if (myU1 < u && u < myU2)
                        {
                            u = (myU1 + myU2) / 2.0;
                            myU1 = u - M_PI;
                            myU2 = u + M_PI;
                        }
                        else
                        {
                            u = (myU1 + myU2) / 2.0;
                            if (myU1 < u)
                            {
                                myU1 = u - 2 * M_PI;
                                myU2 = u;
                            }
                            else
                            {
                                myU1 = u;
                                myU2 = u + 2 * M_PI;
                            }
                        }
                        uCouture = true;
                    }
                    else
                    {
                        Vector3D d1U, d1V;
                        Vector3D t;
                        Point3D p3d;
                        myCurve.D1(w1, out p3d, out t);
                        // OCCT: evaluates the surface at (U1, U2), two U parameters, where (U1, V1) was meant; on a
                        // cylinder D1U depends only on U, so only the sign test below is affected, through U1.
                        mySurface.D1(u1, u2, out p3d, out d1U, out d1V);
                        double dU = t.Dot(d1U);

                        uCouture = true;
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
                    double u1, v1, u, v;
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(cylinder, p1, out u1, out v1);
                    double r = cylinder.Radius;
                    double Delta = 0.0, step;
                    double eps = M_PI, dmax = 0.0, d = 0.0;
                    step = Function_ComputeStep(myCurve, r);
                    myU1 = u1;
                    myU2 = u1;
                    double pmin = w1, pmax = w1, plim = w2 + .1 * step;
                    var aUTrack = new Winding(w1, u1);
                    for (double par = w1 + step; par <= plim; par += step)
                    {
                        p = myCurve.Value(par);
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(cylinder, p, out u, out v);
                        u += Delta;
                        d = u - u1;
                        if (d > eps)
                        {
                            u -= Delta;
                            Delta -= 2 * M_PI;
                            u += Delta;
                            d = u - u1;
                        }
                        else if (d < -eps)
                        {
                            u -= Delta;
                            Delta += 2 * M_PI;
                            u += Delta;
                            d = u - u1;
                        }
                        dmax = Max(dmax, Math.Abs(d));
                        if (u < myU1)
                        {
                            myU1 = u;
                            pmin = par;
                        }
                        if (u > myU2)
                        {
                            myU2 = u;
                            pmax = par;
                        }
                        aUTrack.Add(par, u);
                        u1 = u;
                    }

                    if (Math.Abs(pmin - w1) > global::Kerf.Engine.Foundation.Precision.PConfusion()
                        && Math.Abs(pmin - w2) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        myU1 -= dmax * .5;
                    }
                    if (Math.Abs(pmax - w1) > global::Kerf.Engine.Foundation.Precision.PConfusion()
                        && Math.Abs(pmax - w2) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        myU2 += dmax * .5;
                    }

                    if (aUTrack.IsMoreThanATurn(dmax))
                    {
                        uWinding = aUTrack;
                        uCouture = true;
                    }
                    else if ((myU1 >= 0.0 && myU1 <= 2 * M_PI) && (myU2 >= 0.0 && myU2 <= 2 * M_PI))
                    {
                        uCouture = false;
                    }
                    else
                    {
                        u = (myU1 + myU2) / 2.0;
                        myU1 = u - M_PI;
                        myU2 = u + M_PI;
                        uCouture = true;
                    }
                }
            }
            break;
            //
            case SurfaceKind.Sphere:
            {
                vCouture = false;
                Sphere3D sp = mySurface.Sphere();
                if (myCurve.GetType() == Circle)
                {
                    uCouture = true;

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
                    int nbSolutions = 0;
                    double a, b, c, D, r, Tol = 1.0e-10;
                    double u1, u2, v1, v2;
                    Transform3D trsf = new();
                    //
                    Circle3D circle = myCurve.Circle();
                    trsf = trsf.WithTransformation(sp.Position);
                    circle = circle.Transformed(trsf);
                    //
                    r = sp.Radius;
                    Plane3D plane = new(new CoordinateSystem(circle.Position));
                    plane.Coefficients(out a, out b, out c, out D);
                    //
                    if (Math.Abs(c) < Tol)
                    {
                        if (Math.Abs(a) > Tol)
                        {
                            if ((D / a) < 0.0)
                            {
                                if ((r - Math.Abs(D / a)) > Tol)
                                {
                                    nbSolutions = 2;
                                }
                                else if (Math.Abs(r - Math.Abs(D / a)) < Tol)
                                {
                                    nbSolutions = 1;
                                }
                                else
                                {
                                    nbSolutions = 0;
                                }
                            }
                        }
                    }
                    else
                    {
                        double delta = r * r * (a * a + c * c) - D * D;
                        delta *= c * c;
                        if (Math.Abs(delta) < Tol * Tol)
                        {
                            if (a * D > 0.0)
                            {
                                nbSolutions = 1;
                            }
                        }
                        else if (delta > 0)
                        {
                            double xx;
                            delta = Math.Sqrt(delta);
                            xx = -a * D + delta;
                            //
                            if (xx > Tol)
                            {
                                nbSolutions++;
                            }
                            xx = -a * D - delta;
                            //
                            if (xx > Tol)
                            {
                                nbSolutions++;
                            }
                        }
                    }
                    //

                    // box+sphere >>
                    double uu = 0.0;
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(sp, p1, out u1, out v1);
                    double eps = 10.0 * Epsilon(1.0);
                    double dt = Max(global::Kerf.Engine.Foundation.Precision.PConfusion(), 0.01 * (w2 - w1));
                    if (Math.Abs(u1) < eps)
                    {
                        // May be U1 must be equal 2*PI?
                        Point3D pd = myCurve.Value(w1 + dt);
                        double ud, vd;
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(sp, pd, out ud, out vd);
                        if (Math.Abs(u1 - ud) > M_PI)
                        {
                            u1 = 2.0 * M_PI;
                        }
                    }
                    else if (Math.Abs(2.0 * M_PI - u1) < eps)
                    {
                        // maybe U1 = 0.?
                        Point3D pd = myCurve.Value(w1 + dt);
                        double ud, vd;
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(sp, pd, out ud, out vd);
                        if (Math.Abs(u1 - ud) > M_PI)
                        {
                            u1 = 0.0;
                        }
                    }
                    //
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(sp, p2, out u2, out v1);
                    if (Math.Abs(u2) < eps)
                    {
                        // May be U2 must be equal 2*PI?
                        Point3D pd = myCurve.Value(w2 - dt);
                        double ud, vd;
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(sp, pd, out ud, out vd);
                        if (Math.Abs(u2 - ud) > M_PI)
                        {
                            u2 = 2.0 * M_PI;
                        }
                    }
                    else if (Math.Abs(2.0 * M_PI - u2) < eps)
                    {
                        // maybe U2 = 0.?
                        Point3D pd = myCurve.Value(w2 - dt);
                        double ud, vd;
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(sp, pd, out ud, out vd);
                        if (Math.Abs(u2 - ud) > M_PI)
                        {
                            u2 = 0.0;
                        }
                    }
                    //
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(sp, p, out uu, out v1);
                    // OCCT: U2, adjusted to the seam just above, is overwritten here with the U of the point at
                    // W1 + PI/8 (a curve parameter offset, whatever the curve's parameterisation).
                    p2 = myCurve.Value(w1 + M_PI / 8);
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(sp, p2, out u2, out v2);
                    //
                    if (nbSolutions == 1)
                    {
                        if (Math.Abs(u1 - u2) > M_PI)
                        { // crossing the seam
                            if (u1 > M_PI)
                            {
                                myU1 = u1;
                                myU2 = u1 + 2 * M_PI;
                            }
                            else
                            {
                                myU2 = u1;
                                myU1 = u1 - 2 * M_PI;
                            }
                        }
                        else
                        { // not crossing the seam
                            if (u1 > u2)
                            {
                                myU2 = u1;
                                myU1 = u1 - 2 * M_PI;
                            }
                            else
                            {
                                myU1 = u1;
                                myU2 = u1 + 2 * M_PI;
                            }
                        }
                    }
                    else
                    { // 0 or 2 solutions
                        Point3D center = circle.Location;
                        double u, v;
                        global::Kerf.Engine.Geometry.ElSLib.SphereParameters(new CoordinateSystem(Constants.XOY()), 1, center, out u, out v);
                        myU1 = u - M_PI;
                        myU2 = u + M_PI;
                    }
                    //
                    // eval the VCouture.
                    if ((c == 0) || Math.Abs(Math.Abs(D / c) - r) > 1.0e-10)
                    {
                        vCouture = false;
                    }
                    else
                    {
                        vCouture = true;
                        uCouture = true;

                        if (D / c < 0.0)
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
                        Point3D pp = p1.Transformed(trsf);

                        if (Math.Abs(pp.X * pp.X + pp.Y * pp.Y + pp.Z * pp.Z - r * r) < Tol)
                        {
                            Point3D center = circle.Location;
                            double u, v;
                            global::Kerf.Engine.Geometry.ElSLib.SphereParameters(new CoordinateSystem(Constants.XOY()), 1, center, out u, out v);
                            myU1 = u - M_PI;
                            myU2 = u + M_PI;
                            vCouture = false;
                        }
                    }

                    // box+sphere >>
                    myV1 = -1.0e+100;
                    myV2 = 1.0e+100;
                    // box+sphere <<
                } // if ( myCurve->GetType() == GeomAbs_Circle)

                else
                {
                    double u1, v1, u, v;
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(sp, p1, out u1, out v1);
                    double r = sp.Radius;
                    double Delta = 0.0, step;
                    double eps = M_PI, dmax = 0.0, d = 0.0;
                    step = Function_ComputeStep(myCurve, r);
                    myU1 = u1;
                    myU2 = u1;
                    double pmin = w1, pmax = w1, plim = w2 + .1 * step;
                    var aUTrack = new Winding(w1, u1);
                    for (double par = w1 + step; par <= plim; par += step)
                    {
                        p = myCurve.Value(par);
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(sp, p, out u, out v);
                        u += Delta;
                        d = u - u1;
                        if (d > eps)
                        {
                            u -= Delta;
                            Delta -= 2 * M_PI;
                            u += Delta;
                            d = u - u1;
                        }
                        else if (d < -eps)
                        {
                            u -= Delta;
                            Delta += 2 * M_PI;
                            u += Delta;
                            d = u - u1;
                        }
                        dmax = Max(dmax, Math.Abs(d));
                        if (u < myU1)
                        {
                            myU1 = u;
                            pmin = par;
                        }
                        if (u > myU2)
                        {
                            myU2 = u;
                            pmax = par;
                        }
                        aUTrack.Add(par, u);
                        u1 = u;
                    }

                    if (Math.Abs(pmin - w1) > global::Kerf.Engine.Foundation.Precision.PConfusion()
                        && Math.Abs(pmin - w2) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        myU1 -= dmax * .5;
                    }
                    if (Math.Abs(pmax - w1) > global::Kerf.Engine.Foundation.Precision.PConfusion()
                        && Math.Abs(pmax - w2) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        myU2 += dmax * .5;
                    }

                    if (aUTrack.IsMoreThanATurn(dmax))
                    {
                        uWinding = aUTrack;
                        uCouture = true;
                    }
                    else if ((myU1 >= 0.0 && myU1 <= 2 * M_PI) && (myU2 >= 0.0 && myU2 <= 2 * M_PI))
                    {
                        myU1 = 0.0;
                        myU2 = 2.0 * M_PI;
                        uCouture = false;
                    }
                    else
                    {
                        u = (myU1 + myU2) / 2.0;
                        myU1 = u - M_PI;
                        myU2 = u + M_PI;
                        uCouture = true;
                    }

                    vCouture = false;
                }
            }
            break;
            //
            case SurfaceKind.Torus:
            {
                Torus3D tr = mySurface.Torus();
                double u1, v1, u, v, dU, dV;
                global::Kerf.Engine.Geometry.ElSLib.Parameters(tr, p1, out u1, out v1);
                double r = tr.MinorRadius;
                double deltaU = 0.0, deltaV = 0.0, step;
                double eps = M_PI, dmaxU = 0.0, dmaxV = 0.0;
                step = Function_ComputeStep(myCurve, r);
                myU1 = u1;
                myU2 = u1;
                myV1 = v1;
                myV2 = v1;
                double pminU = w1, pmaxU = w1, pminV = w1, pmaxV = w1, plim = w2 + .1 * step;
                var aUTrack = new Winding(w1, u1);
                var aVTrack = new Winding(w1, v1);
                for (double par = w1 + step; par <= plim; par += step)
                {
                    p = myCurve.Value(par);
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(tr, p, out u, out v);
                    u += deltaU;
                    v += deltaV;
                    dU = u - u1;
                    dV = v - v1;
                    if (dU > eps)
                    {
                        u -= deltaU;
                        deltaU -= 2 * M_PI;
                        u += deltaU;
                        dU = u - u1;
                    }
                    else if (dU < -eps)
                    {
                        u -= deltaU;
                        deltaU += 2 * M_PI;
                        u += deltaU;
                        dU = u - u1;
                    }
                    if (dV > eps)
                    {
                        v -= deltaV;
                        deltaV -= 2 * M_PI;
                        v += deltaV;
                        dV = v - v1;
                    }
                    else if (dV < -eps)
                    {
                        v -= deltaV;
                        deltaV += 2 * M_PI;
                        v += deltaV;
                        dV = v - v1;
                    }
                    dmaxU = Max(dmaxU, Math.Abs(dU));
                    dmaxV = Max(dmaxV, Math.Abs(dV));
                    if (u < myU1)
                    {
                        myU1 = u;
                        pminU = par;
                    }
                    if (u > myU2)
                    {
                        myU2 = u;
                        pmaxU = par;
                    }
                    if (v < myV1)
                    {
                        myV1 = v;
                        pminV = par;
                    }
                    if (v > myV2)
                    {
                        myV2 = v;
                        pmaxV = par;
                    }
                    aUTrack.Add(par, u);
                    aVTrack.Add(par, v);
                    u1 = u;
                    v1 = v;
                }

                if (Math.Abs(pminU - w1) > global::Kerf.Engine.Foundation.Precision.PConfusion()
                    && Math.Abs(pminU - w2) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    myU1 -= dmaxU * .5;
                }
                if (Math.Abs(pmaxU - w1) > global::Kerf.Engine.Foundation.Precision.PConfusion()
                    && Math.Abs(pmaxU - w2) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    myU2 += dmaxU * .5;
                }
                if (Math.Abs(pminV - w1) > global::Kerf.Engine.Foundation.Precision.PConfusion()
                    && Math.Abs(pminV - w2) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    myV1 -= dmaxV * .5;
                }
                if (Math.Abs(pmaxV - w1) > global::Kerf.Engine.Foundation.Precision.PConfusion()
                    && Math.Abs(pmaxV - w2) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    myV2 += dmaxV * .5;
                }

                if (aUTrack.IsMoreThanATurn(dmaxU))
                {
                    uWinding = aUTrack;
                    uCouture = true;
                }
                else if ((myU1 >= 0.0 && myU1 <= 2 * M_PI) && (myU2 >= 0.0 && myU2 <= 2 * M_PI))
                {
                    myU1 = 0.0;
                    myU2 = 2.0 * M_PI;
                    uCouture = false;
                }
                else
                {
                    u = (myU1 + myU2) / 2.0;
                    myU1 = u - M_PI;
                    myU2 = u + M_PI;
                    uCouture = true;
                }
                if (aVTrack.IsMoreThanATurn(dmaxV))
                {
                    vWinding = aVTrack;
                    vCouture = true;
                }
                else if ((myV1 >= 0.0 && myV1 <= 2 * M_PI) && (myV2 >= 0.0 && myV2 <= 2 * M_PI))
                {
                    vCouture = false;
                }
                else
                {
                    v = (myV1 + myV2) / 2.0;
                    myV1 = v - M_PI;
                    myV2 = v + M_PI;
                    vCouture = true;
                }
            }
            break;

            default:
            {
                uCouture = false;
                vCouture = false;
            }
            break;
        }
    }

    /// <summary>
    /// The angular parameter of a curve winding more than a period around the surface (a helix of several turns on a
    /// cylinder, a coil on a torus), from Function_SetUVBounds's samples, unwrapped. A point's parameter, known modulo
    /// 2π, is taken at the turn the samples predict (decision 1459). OCCT kept a window of 2π around the middle of
    /// the range, so the curve's image wrapped inside it and the approximation bridged the jump.
    /// </summary>
    private sealed class Winding
    {
        private readonly List<double> _params = [];
        private readonly List<double> _values = [];

        public Winding(double theParam, double theValue)
        {
            Add(theParam, theValue);
        }

        public void ResetFirst(double theValue)
        {
            _values[0] = theValue;
        }

        public void Add(double theParam, double theValue)
        {
            _params.Add(theParam);
            _values.Add(theValue);
        }

        // True if the samples span more than a turn, by more than half the largest step between samples (a closed
        // curve's samples span a turn, and the last can overshoot the end by a tenth of a step)
        public bool IsMoreThanATurn(double theMaxStep)
            => _values.Max() - _values.Min() > 2 * M_PI + 0.5 * theMaxStep;

        public double Unwrap(double theParam, double theValue)
        {
            int i = _params.BinarySearch(theParam);
            double aPredicted;
            if (i >= 0)
            {
                aPredicted = _values[i];
            }
            else
            {
                int j = Math.Clamp(~i, 1, _params.Count - 1);
                double aT = (theParam - _params[j - 1]) / (_params[j] - _params[j - 1]);
                aPredicted = _values[j - 1] + aT * (_values[j] - _values[j - 1]);
            }
            return theValue + 2 * M_PI * Math.Round((aPredicted - theValue) / (2 * M_PI));
        }
    }

    /// <summary>OCCT's file-local class ProjLib_Function: the 2D point of the projection at a parameter.</summary>
    private sealed class ProjLib_Function : Function
    {
        private readonly CurveAdaptorBase _curve;
        private readonly SurfaceAdaptorBase _surface;
        private readonly bool[] _isPeriodic = new bool[2];
        private readonly double[] _period = new double[2];

        public double myU1, myU2, myV1, myV2;
        public bool UCouture, VCouture;
        private readonly Winding? _uWinding, _vWinding;

        public ProjLib_Function(CurveAdaptorBase c, SurfaceAdaptorBase s)
        {
            _curve = c;
            _surface = s;
            myU1 = 0.0;
            myU2 = 0.0;
            myV1 = 0.0;
            myV2 = 0.0;
            UCouture = false;
            VCouture = false;
            _nbPnt = 0;
            _nbPnt2d = 1;
            Function_SetUVBounds(ref myU1, ref myU2, ref myV1, ref myV2, ref UCouture, ref VCouture, _curve, _surface, out _uWinding, out _vWinding);
            _isPeriodic[0] = _surface.IsUPeriodic;
            _isPeriodic[1] = _surface.IsVPeriodic;

            if (_isPeriodic[0])
            {
                _period[0] = _surface.UPeriod();
            }
            else
            {
                _period[0] = 0.0;
            }

            if (_isPeriodic[1])
            {
                _period[1] = _surface.VPeriod();
            }
            else
            {
                _period[1] = 0.0;
            }
        }

        public override void PeriodInformation(int dimIdx, out bool isPeriodic, out double period)
        {
            isPeriodic = _isPeriodic[dimIdx - 1];
            period = _period[dimIdx - 1];
        }

        public override double FirstParameter() => (_curve.FirstParameter());

        public override double LastParameter() => (_curve.LastParameter());

        public override bool Value(double t, NCollection_Array1<Point2D> pnt2d, NCollection_Array1<Point3D> pnt)
        {
            pnt2d[1] =
                Function_Value(t, _curve, _surface, myU1, myU2, myV1, myV2, UCouture, VCouture, _uWinding, _vWinding);
            return true;
        }

        public Point2D Value(double t)
        {
            return Function_Value(t, _curve, _surface, myU1, myU2, myV1, myV2, UCouture, VCouture, _uWinding, _vWinding);
        }

        public override bool D1(double t, NCollection_Array1<Vector2D> vec2d, NCollection_Array1<Vector3D> vec)
        {
            Point2D pnt2d;
            Vector2D aVec2d;
            bool isOk = Function_D1(t,
                                    out pnt2d,
                                    out aVec2d,
                                    _curve,
                                    _surface,
                                    myU1,
                                    myU2,
                                    myV1,
                                    myV2,
                                    UCouture,
                                    VCouture,
                                    _uWinding,
                                    _vWinding);
            vec2d[1] = aVec2d;
            return isOk;
        }
    }

    private static double ComputeTolU(SurfaceAdaptorBase surf, double tolerance)
    {
        double tolU = surf.UResolution(tolerance);
        if (surf.IsUPeriodic)
        {
            tolU = Min(tolU, 0.01 * surf.UPeriod());
        }

        return tolU;
    }

    private static double ComputeTolV(SurfaceAdaptorBase surf, double tolerance)
    {
        double tolV = surf.VResolution(tolerance);
        if (surf.IsVPeriodic)
        {
            tolV = Min(tolV, 0.01 * surf.VPeriod());
        }

        return tolV;
    }

    /// <summary>An empty algorithm with the tolerance Precision::PApproximation().</summary>
    public ComputeApprox()
    {
        _tolerance = global::Kerf.Engine.Foundation.Precision.PApproximation();
        _degMin = -1;
        _degMax = -1;
        _maxSegments = -1;
        _bndPnt = TangencyPoint;
    }

    /// <summary>
    /// Approximates the projection of <paramref name="c"/> on <paramref name="s"/> within the 3D tolerance
    /// <paramref name="tol"/> (at least Precision::PApproximation()).
    /// </summary>
    /// <exception cref="NoSuchObjectException">The surface is not analytical and the case is not a B-spline or Bezier
    /// curve on a plane.</exception>
    public ComputeApprox(CurveAdaptorBase c, SurfaceAdaptorBase s, double tol)
    {
        _tolerance = Max(tol, global::Kerf.Engine.Foundation.Precision.PApproximation());
        _degMin = -1;
        _degMax = -1;
        _maxSegments = -1;
        _bndPnt = TangencyPoint;
        Perform(c, s);
    }

    /// <summary>
    /// Computes the projection. If the surface is a plane and the curve a B-spline or a Bezier curve, the poles are
    /// projected (no approximation).
    /// </summary>
    /// <exception cref="NoSuchObjectException">The surface is not analytical and the case is not a B-spline or Bezier
    /// curve on a plane.</exception>
    public void Perform(CurveAdaptorBase c, SurfaceAdaptorBase s)
    {
        // if the surface is a plane and the curve a BSpline or a BezierCurve,
        // don`t make an Approx but only the projection of the poles.

        int NbKnots, nbPoles;
        CurveKind cType = c.GetType();
        SurfaceKind sType = s.GetType();

        bool surfIsAnal = global::Kerf.Engine.Geometry.Projection.ProjLib.IsAnaSurf(s);

        bool curvIsAnal = (cType != BSplineCurve) && (cType != BezierCurve)
                          && (cType != OffsetCurve) && (cType != OtherCurve);

        bool simplecase = surfIsAnal && curvIsAnal;
        if (cType == BSplineCurve || cType == BezierCurve)
        {
            int nbKnots = 1;
            if (cType == BSplineCurve)
            {
                nbKnots = c.KnotCount;
            }
            simplecase = simplecase && c.Degree() <= 2 && nbKnots <= 2;
        }

        if (cType == BSplineCurve && sType == Plane)
        {
            // get the poles and eventually the weights
            BSplineCurve3D bs = c.BSpline();
            nbPoles = bs.PoleCount;
            NCollection_Array1<Point3D> p3d = bs.InternalPoles();
            NCollection_Array1<Point2D> poles = new(1, nbPoles);

            // Project poles onto plane using optimized projector (avoids gp_Trsf per point)
            PlaneProjector proj = new(s.Plane().Position);
            for (int i = 1; i <= nbPoles; i++)
            {
                poles.SetValue(i, proj.Project(p3d[i]));
            }
            NCollection_Array1<double> knots = bs.InternalKnots();
            NCollection_Array1<int> mults = bs.InternalMultiplicities();
            // get the knots and mults if BSplineCurve
            if (bs.IsRational)
            {
                _bSpline = new BSplineCurve2D(poles,
                                                    bs.InternalWeightsArray(),
                                                    knots,
                                                    mults,
                                                    bs.Degree,
                                                    bs.IsPeriodic);
            }
            else
            {
                _bSpline = new BSplineCurve2D(poles, knots, mults, bs.Degree, bs.IsPeriodic);
            }
        }
        else if (cType == BezierCurve && sType == Plane)
        {
            // get the poles and eventually the weights
            BezierCurve3D bezierCurvePtr = c.Bezier();
            nbPoles = bezierCurvePtr.PoleCount;
            NCollection_Array1<Point3D> p3d = bezierCurvePtr.InternalPoles();
            NCollection_Array1<Point2D> poles = new(1, nbPoles);

            // Project poles onto plane using optimized projector (avoids gp_Trsf per point)
            PlaneProjector proj = new(s.Plane().Position);
            for (int i = 1; i <= nbPoles; i++)
            {
                poles.SetValue(i, proj.Project(p3d[i]));
            }
            if (bezierCurvePtr.IsRational)
            {
                _bezier = new BezierCurve2D(poles, bezierCurvePtr.InternalWeightsArray());
            }
            else
            {
                _bezier = new BezierCurve2D(poles);
            }
        }
        else
        {
            ProjLib_Function f = new(c, s);

            //-----------
            int deg1 = 5, deg2;
            if (simplecase)
            {
                deg2 = 8;
            }
            else
            {
                deg2 = 10;
            }
            if (_degMin > 0)
            {
                deg1 = _degMin;
            }
            //
            if (_degMax > 0)
            {
                deg2 = _degMax;
            }
            //
            int maxSegments = 1000;
            if (_maxSegments > 0)
            {
                maxSegments = _maxSegments;
            }
            Constraint fistC = TangencyPoint,
                                    lastC = TangencyPoint;
            if (_bndPnt != TangencyPoint)
            {
                fistC = _bndPnt;
                lastC = _bndPnt;
            }

            //-------------
            double tolU = ComputeTolU(s, _tolerance);
            double tolV = ComputeTolV(s, _tolerance);
            double aTol2d =
                Max(Math.Sqrt(tolU * tolU + tolV * tolV), global::Kerf.Engine.Foundation.Precision.PConfusion());

            FitAndDivide2d fit = new(deg1, deg2, _tolerance, aTol2d, true, fistC, lastC);
            fit.SetMaxSegments(maxSegments);
            if (simplecase)
            {
                fit.SetHangChecking(false);
            }
            fit.Perform(f);

            double newTol2d = 0;
            if (fit.IsAllApproximated)
            {
                int i;
                int nbCurves = fit.MultiCurveCount;

                // try to make the curve at least C1
                CompBezierCurves2dToBSplineCurve2d conv = new();

                double tol3d, tol2d;
                for (i = 1; i <= nbCurves; i++)
                {
                    fit.Error(i, out tol3d, out tol2d);
                    newTol2d = Max(newTol2d, tol2d);
                    MultiCurve mc = fit.Value(i);               // Load the i-th Curve
                    NCollection_Array1<Point2D> poles2d = new(1, mc.Degree() + 1); // Retrieve the poles
                    mc.Curve(1, poles2d);
                    conv.AddCurve(poles2d);
                }

                // update the fields of ProjLib_Approx
                conv.Perform();
                nbPoles = conv.PoleCount;
                NbKnots = conv.KnotCount;

                if (nbPoles <= 0 || nbPoles > 100000)
                {
                    return;
                }
                if (NbKnots <= 0 || NbKnots > 100000)
                {
                    return;
                }

                NCollection_Array1<Point2D> newPoles = new(1, nbPoles);
                NCollection_Array1<double> newKnots = new(1, NbKnots);
                NCollection_Array1<int> newMults = new(1, NbKnots);

                conv.KnotsAndMults(newKnots, newMults);
                conv.Poles(newPoles);

                global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(c.FirstParameter(), c.LastParameter(), newKnots);

                // Set NewKnots(NbKnots) exactly C->LastParameter()
                // to avoid problems if trim is used.
                newKnots[NbKnots] = c.LastParameter();

                // The start and end poles need to be adjusted:
                // (Due to seam issues, we had to open the definition interval
                // of the curve.)
                // We choose to compute these poles by extending the approximated
                // curve.
                _bSpline = new BSplineCurve2D(newPoles, newKnots, newMults, conv.Degree());

                if (fistC == PassPoint || lastC == PassPoint)
                {
                    // try to smoother the Curve GeomAbs_C1.
                    int deg = _bSpline.Degree;
                    bool ok = true;
                    double smoothTol = Max(global::Kerf.Engine.Foundation.Precision.Confusion(), newTol2d);
                    for (int ij = 2; ij < NbKnots; ij++)
                    {
                        ok = ok && _bSpline.RemoveKnot(ij, deg - 1, smoothTol);
                    }
                }
            }
            else
            {
                int nbCurves = fit.MultiCurveCount;
                if (nbCurves != 0)
                {
                    double tol3d, tol2d;
                    fit.Error(nbCurves, out tol3d, out tol2d);
                    newTol2d = tol2d;
                }
            }

            // restore tolerance 3d from 2d

            // Here we consider that
            //    aTolU(new)/aTolV(new) = aTolU(old)/aTolV(old)
            //(it is assumption indeed).
            // Then,
            //   Tol3D(new)/Tol3D(old) = Tol2D(new)/Tol2D(old).
            _tolerance *= (newTol2d / aTol2d);

            // Return curve home
            double uFirst = f.FirstParameter();
            double uLast = f.LastParameter();
            double umid = (uFirst + uLast) / 2;
            Point3D p3d = c.Value(umid);
            double u = 0.0, v = 0.0;
            switch (sType)
            {
                case Plane:
                {
                    Plane3D plane = s.Plane();
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(plane, p3d, out u, out v);
                    break;
                }
                case SurfaceKind.Cylinder:
                {
                    Cylinder3D cylinder = s.Cylinder();
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(cylinder, p3d, out u, out v);
                    break;
                }
                case SurfaceKind.Cone:
                {
                    Cone3D cone = s.Cone();
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(cone, p3d, out u, out v);
                    break;
                }
                case SurfaceKind.Sphere:
                {
                    Sphere3D sphere = s.Sphere();
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(sphere, p3d, out u, out v);
                    break;
                }
                case SurfaceKind.Torus:
                {
                    Torus3D torus = s.Torus();
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(torus, p3d, out u, out v);
                    break;
                }
                default:
                    throw new NoSuchObjectException("ProjLib_ComputeApprox::Value");
            }
            bool toMirror = false;
            double du = 0.0, dv = 0.0;
            int number;
            if (f.VCouture)
            {
                if (sType == SurfaceKind.Sphere && Math.Abs(u - f.myU1) > M_PI)
                {
                    toMirror = true;
                    dv = -M_PI;
                    v = M_PI - v;
                }
                double newV = global::Kerf.Engine.Geometry.ElCLib.InPeriod(v, f.myV1, f.myV2);
                number = (int)(Math.Floor((newV - v) / (f.myV2 - f.myV1)));
                dv -= number * (f.myV2 - f.myV1);
            }
            if (f.UCouture || (f.VCouture && sType == SurfaceKind.Sphere))
            {
                double nbPer;
                Point2D p2d = f.Value(umid);
                du = u - p2d.X;
                du = (du < 0) ? (du - global::Kerf.Engine.Foundation.Precision.PConfusion()) : (du + global::Kerf.Engine.Foundation.Precision.PConfusion());
                nbPer = Math.Truncate(du / M_PI); // modf's integral part
                number = (int)nbPer;
                du = number * M_PI;
            }

            if (_bSpline is not null)
            {
                if (du != 0.0 || dv != 0.0)
                {
                    _bSpline.Translate(new Vector2D(du, dv));
                }
                if (toMirror)
                {
                    Axis2D axe = new(new Point2D(0.0, 0.0), new Direction2D(Direction2D.D.X));
                    _bSpline.Mirror(axe);
                }
            }
        }
    }

    /// <summary>Sets the 3D tolerance of the approximation.</summary>
    public void SetTolerance(double tolerance)
    {
        _tolerance = tolerance;
    }

    /// <summary>Sets the minimal and maximal degrees of the approximation (a value of 0 or less keeps the
    /// default).</summary>
    public void SetDegree(int degMin, int degMax)
    {
        _degMin = degMin;
        _degMax = degMax;
    }

    /// <summary>Sets the maximal number of segments of the approximation (0 or less keeps the default, 1000).</summary>
    public void SetMaxSegments(int maxSegments)
    {
        _maxSegments = maxSegments;
    }

    /// <summary>Sets the constraint at the ends of the approximation (by default AppParCurves_TangencyPoint).</summary>
    public void SetBndPnt(Constraint bndPnt)
    {
        _bndPnt = bndPnt;
    }

    /// <summary>The B-spline result, frozen (the same object at each call), or null.</summary>
    public BSplineCurve2D? BSpline()
    {
        _bSpline?.Freeze();
        return _bSpline;
    }

    /// <summary>The B-spline result, unfrozen, for ported code that discards the algorithm.</summary>
    internal BSplineCurve2D? InternalBSpline() => _bSpline;

    /// <summary>The Bezier result (a Bezier curve on a plane), frozen (the same object at each call), or null.</summary>
    public BezierCurve2D? Bezier()
    {
        _bezier?.Freeze();
        return _bezier;
    }

    /// <summary>The Bezier result, unfrozen, for ported code that discards the algorithm.</summary>
    internal BezierCurve2D? InternalBezier() => _bezier;

    /// <summary>The reached 3D tolerance.</summary>
    public double Tolerance() => _tolerance;
}
