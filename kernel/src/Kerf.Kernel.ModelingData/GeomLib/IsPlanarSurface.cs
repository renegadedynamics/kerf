// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_IsPlanarSurface.hxx
// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_IsPlanarSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Geometry.Library;

/// <summary>
/// Finds whether a surface is planar within a tolerance, and its plane. The test is made by the constructor.
/// </summary>
internal sealed class IsPlanarSurface
{
    private readonly Plane3D _plan = new();
    private readonly bool _isPlan;

    /// <summary>Tests whether S is planar within Tol.</summary>
    public IsPlanarSurface(Surface s, double tol = 1.0e-7)
    {
        SurfaceAdaptor AS = new(s);
        SurfaceKind type;

        type = AS.GetType();

        switch (type)
        {
            case Plane:
            {
                _isPlan = true;
                _plan = AS.Plane();
                break;
            }
            case SurfaceKind.Cylinder:
            case SurfaceKind.Cone:
            case SurfaceKind.Sphere:
            case SurfaceKind.Torus:
            {
                _isPlan = false;
                break;
            }

            case SurfaceOfRevolution:
            {
                bool essai = true;
                Point3D p;
                Vector3D DU, DV, dn;
                Direction3D dir = AS.AxeOfRevolution().Direction;
                double umin, umax, vmin, vmax;
                s.ParameterBounds(out umin, out umax, out vmin, out vmax);
                s.D1((umin + umax) / 2, (vmin + vmax) / 2, out p, out DU, out DV);
                if (DU.Magnitude <= Constants.Resolution() || DV.Magnitude <= Constants.Resolution())
                {
                    double newU = (umin + umax) / 2 + (umax - umin) * 0.1;
                    double newV = (vmin + vmax) / 2 + (vmax - vmin) * 0.1;
                    s.D1(newU, newV, out p, out DU, out DV);
                }
                dn = DU ^ DV;
                if (dn.Magnitude > 1.0e-7)
                {
                    double angle = dir.AngleTo(new Direction3D(dn));
                    if (angle > M_PI / 2)
                    {
                        angle = M_PI - angle;
                        dir = dir.Reversed();
                    }
                    essai = (angle < 0.1);
                }

                if (essai)
                {
                    CoordinateSystem axe = new(p, dir);
                    axe = axe.WithXDirection(new Direction3D(DU));
                    _plan = _plan.WithPosition(axe);
                    _plan = _plan.WithLocation(p);
                    Curve3D c;
                    c = s.UIso(umin);
                    _isPlan = Controle(c, _plan, tol);
                }
                else
                {
                    _isPlan = false;
                }

                break;
            }
            case SurfaceOfExtrusion:
            {
                bool essai = false;
                double umin, umax, vmin, vmax;
                double norm;
                Vector3D Du, Dv, dn;
                Point3D p;

                s.ParameterBounds(out umin, out umax, out vmin, out vmax);
                s.D1((umin + umax) / 2, (vmin + vmax) / 2, out p, out Du, out Dv);
                if (Du.Magnitude <= Constants.Resolution() || Dv.Magnitude <= Constants.Resolution())
                {
                    double newU = (umin + umax) / 2 + (umax - umin) * 0.1;
                    double newV = (vmin + vmax) / 2 + (vmax - vmin) * 0.1;
                    s.D1(newU, newV, out p, out Du, out Dv);
                }
                dn = Du ^ Dv;
                norm = dn.Magnitude;
                if (norm > 1.0e-15)
                {
                    dn /= norm;
                    double angmax = tol / (vmax - vmin);
                    Direction3D d = new(dn);
                    essai = (d.IsNormal(AS.Direction(), angmax));
                }
                if (essai)
                {
                    CoordinateSystem axe = new(p, new Direction3D(dn), new Direction3D(Du));
                    _plan = _plan.WithPosition(axe);
                    _plan = _plan.WithLocation(p);
                    Curve3D c;
                    c = s.VIso((vmin + vmax) / 2);
                    _isPlan = Controle(c, _plan, tol);
                }
                else
                {
                    _isPlan = false;
                }
                break;
            }

            default:
            {
                int nbU, nbV, ii, jj, kk;
                nbU = 8 + 3 * AS.NbUIntervals(CN);
                nbV = 8 + 3 * AS.NbVIntervals(CN);
                double umin, umax, vmin, vmax, du, dv, u, v;
                s.ParameterBounds(out umin, out umax, out vmin, out vmax);
                du = (umax - umin) / (nbU - 1);
                dv = (vmax - vmin) / (nbV - 1);
                NCollection_Array1<Point3D> pnts = new(1, nbU * nbV);
                for (ii = 0, kk = 1; ii < nbU; ii++)
                {
                    u = umin + du * ii;
                    for (jj = 0; jj < nbV; jj++, kk++)
                    {
                        v = vmin + dv * jj;
                        s.D0(u, v, out pnts[kk]);
                    }
                }

                _isPlan = Controle(pnts, tol, s, ref _plan);
                break;
            }
        }
    }

    /// <summary>Whether the surface is planar.</summary>
    public bool IsPlanar => _isPlan;

    /// <summary>The plane of the surface; throws StdFail_NotDone if it is not planar.</summary>
    public Plane3D Plan()
    {
        if (!_isPlan)
        {
            throw new NotDoneException(" GeomLib_IsPlanarSurface");
        }
        return _plan;
    }

    private static bool Controle(NCollection_Array1<Point3D> poles, double tol, Surface s, ref Plane3D plan)
    {
        bool isPlan = false;
        double gx, gy, gz;
        Point3D bary;
        Direction3D dx, dy;
        double tolSingular = global::Kerf.Engine.Foundation.Precision.Confusion();

        global::Kerf.Engine.Geometry.Library.GeomLib.Inertia(poles, out bary, out dx, out dy, out gx, out gy, out gz);
        if (gz < tol && gy > tolSingular)
        {
            Point3D p;
            Vector3D DU, dv;
            double umin, umax, vmin, vmax;
            s.ParameterBounds(out umin, out umax, out vmin, out vmax);
            s.D1((umin + umax) / 2, (vmin + vmax) / 2, out p, out DU, out dv);

            if (DU.SquareMagnitude > Constants.Resolution() && dv.SquareMagnitude > Constants.Resolution())
            {
                // Choose DX as close as possible to DU
                Direction3D du = new(DU);
                double angle1 = du.AngleTo(dx);
                double angle2 = du.AngleTo(dy);
                if (angle1 > M_PI / 2)
                {
                    angle1 = M_PI - angle1;
                }
                if (angle2 > M_PI / 2)
                {
                    angle2 = M_PI - angle2;
                }
                if (angle2 < angle1)
                {
                    du = dy;
                    dy = dx;
                    dx = du;
                }
                if (dx.AngleTo(new Direction3D(DU)) > M_PI / 2)
                {
                    dx = dx.Reversed();
                }
                if (dy.AngleTo(new Direction3D(dv)) > M_PI / 2)
                {
                    dy = dy.Reversed();
                }

                CoordinateSystem axe = new(bary, dx ^ dy, dx);
                plan = plan.WithPosition(axe);
                plan = plan.WithLocation(bary);
                isPlan = true;
            }
        }
        return isPlan;
    }

    private static bool Controle(Curve3D c, in Plane3D plan, double tol)
    {
        bool b = true;
        int ii, nb;
        CurveKind type;
        CurveAdaptor ac = new(c);
        type = ac.GetType();

        switch (type)
        {
            case Line:
            {
                nb = 2;
                break;
            }
            case Circle:
            {
                nb = 3;
                break;
            }

            case Ellipse:
            case Hyperbola:
            case Parabola:
            {
                nb = 5;
                break;
            }
            case BezierCurve:
            {
                nb = ac.PoleCount;
                break;
            }
            case BSplineCurve:
            {
                nb = ac.PoleCount;
                break;
            }
            default:
            {
                nb = 8 + 3 * ac.NbIntervals(CN);
                break;
            }
        }

        double u, du, f, l, d;
        f = ac.FirstParameter();
        l = ac.LastParameter();
        du = (l - f) / (nb - 1);
        for (ii = 1; ii <= nb && b; ii++)
        {
            u = (ii - 1) * du + f;
            d = plan.DistanceTo(c.Value(u));
            b = d < tol;
        }

        return b;
    }
}
