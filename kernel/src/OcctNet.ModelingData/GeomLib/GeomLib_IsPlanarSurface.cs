// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_IsPlanarSurface.hxx
// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_IsPlanarSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;

namespace Occt;

/// <summary>
/// Finds whether a surface is planar within a tolerance, and its plane. The test is made by the constructor.
/// </summary>
public sealed class GeomLib_IsPlanarSurface
{
    private readonly Plane3D myPlan = new();
    private readonly bool IsPlan;

    /// <summary>Tests whether S is planar within Tol.</summary>
    public GeomLib_IsPlanarSurface(Geom_Surface S, double Tol = 1.0e-7)
    {
        GeomAdaptor_Surface AS = new(S);
        SurfaceKind Type;

        Type = AS.GetType();

        switch (Type)
        {
            case Plane:
            {
                IsPlan = true;
                myPlan = AS.Plane();
                break;
            }
            case SurfaceKind.Cylinder:
            case SurfaceKind.Cone:
            case SurfaceKind.Sphere:
            case SurfaceKind.Torus:
            {
                IsPlan = false;
                break;
            }

            case SurfaceOfRevolution:
            {
                bool Essai = true;
                Point3D P;
                Vector3D DU, DV, Dn;
                Direction3D Dir = AS.AxeOfRevolution().Direction();
                double Umin, Umax, Vmin, Vmax;
                S.Bounds(out Umin, out Umax, out Vmin, out Vmax);
                S.D1((Umin + Umax) / 2, (Vmin + Vmax) / 2, out P, out DU, out DV);
                if (DU.Magnitude() <= Constants.Resolution() || DV.Magnitude() <= Constants.Resolution())
                {
                    double NewU = (Umin + Umax) / 2 + (Umax - Umin) * 0.1;
                    double NewV = (Vmin + Vmax) / 2 + (Vmax - Vmin) * 0.1;
                    S.D1(NewU, NewV, out P, out DU, out DV);
                }
                Dn = DU ^ DV;
                if (Dn.Magnitude() > 1.0e-7)
                {
                    double angle = Dir.Angle(new Direction3D(Dn));
                    if (angle > M_PI / 2)
                    {
                        angle = M_PI - angle;
                        Dir.Reverse();
                    }
                    Essai = (angle < 0.1);
                }

                if (Essai)
                {
                    CoordinateSystem axe = new(P, Dir);
                    axe.SetXDirection(new Direction3D(DU));
                    myPlan.SetPosition(axe);
                    myPlan.SetLocation(P);
                    Geom_Curve C;
                    C = S.UIso(Umin);
                    IsPlan = Controle(C, myPlan, Tol);
                }
                else
                {
                    IsPlan = false;
                }

                break;
            }
            case SurfaceOfExtrusion:
            {
                bool Essai = false;
                double Umin, Umax, Vmin, Vmax;
                double norm;
                Vector3D Du, Dv, Dn;
                Point3D P;

                S.Bounds(out Umin, out Umax, out Vmin, out Vmax);
                S.D1((Umin + Umax) / 2, (Vmin + Vmax) / 2, out P, out Du, out Dv);
                if (Du.Magnitude() <= Constants.Resolution() || Dv.Magnitude() <= Constants.Resolution())
                {
                    double NewU = (Umin + Umax) / 2 + (Umax - Umin) * 0.1;
                    double NewV = (Vmin + Vmax) / 2 + (Vmax - Vmin) * 0.1;
                    S.D1(NewU, NewV, out P, out Du, out Dv);
                }
                Dn = Du ^ Dv;
                norm = Dn.Magnitude();
                if (norm > 1.0e-15)
                {
                    Dn /= norm;
                    double angmax = Tol / (Vmax - Vmin);
                    Direction3D D = new(Dn);
                    Essai = (D.IsNormal(AS.Direction(), angmax));
                }
                if (Essai)
                {
                    CoordinateSystem axe = new(P, new Direction3D(Dn), new Direction3D(Du));
                    myPlan.SetPosition(axe);
                    myPlan.SetLocation(P);
                    Geom_Curve C;
                    C = S.VIso((Vmin + Vmax) / 2);
                    IsPlan = Controle(C, myPlan, Tol);
                }
                else
                {
                    IsPlan = false;
                }
                break;
            }

            default:
            {
                int NbU, NbV, ii, jj, kk;
                NbU = 8 + 3 * AS.NbUIntervals(CN);
                NbV = 8 + 3 * AS.NbVIntervals(CN);
                double Umin, Umax, Vmin, Vmax, du, dv, U, V;
                S.Bounds(out Umin, out Umax, out Vmin, out Vmax);
                du = (Umax - Umin) / (NbU - 1);
                dv = (Vmax - Vmin) / (NbV - 1);
                NCollection_Array1<Point3D> Pnts = new(1, NbU * NbV);
                for (ii = 0, kk = 1; ii < NbU; ii++)
                {
                    U = Umin + du * ii;
                    for (jj = 0; jj < NbV; jj++, kk++)
                    {
                        V = Vmin + dv * jj;
                        S.D0(U, V, out Pnts[kk]);
                    }
                }

                IsPlan = Controle(Pnts, Tol, S, ref myPlan);
                break;
            }
        }
    }

    /// <summary>Whether the surface is planar.</summary>
    public bool IsPlanar() => IsPlan;

    /// <summary>The plane of the surface; throws StdFail_NotDone if it is not planar.</summary>
    public Plane3D Plan()
    {
        if (!IsPlan)
        {
            throw new NotDoneException(" GeomLib_IsPlanarSurface");
        }
        return myPlan;
    }

    private static bool Controle(NCollection_Array1<Point3D> Poles, double Tol, Geom_Surface S, ref Plane3D Plan)
    {
        bool IsPlan = false;
        double gx, gy, gz;
        Point3D Bary;
        Direction3D DX, DY;
        double aTolSingular = global::Kerf.Engine.Foundation.Precision.Confusion();

        GeomLib.Inertia(Poles, out Bary, out DX, out DY, out gx, out gy, out gz);
        if (gz < Tol && gy > aTolSingular)
        {
            Point3D P;
            Vector3D DU, DV;
            double umin, umax, vmin, vmax;
            S.Bounds(out umin, out umax, out vmin, out vmax);
            S.D1((umin + umax) / 2, (vmin + vmax) / 2, out P, out DU, out DV);

            if (DU.SquareMagnitude() > Constants.Resolution() && DV.SquareMagnitude() > Constants.Resolution())
            {
                // Choose DX as close as possible to DU
                Direction3D du = new(DU);
                double Angle1 = du.Angle(DX);
                double Angle2 = du.Angle(DY);
                if (Angle1 > M_PI / 2)
                {
                    Angle1 = M_PI - Angle1;
                }
                if (Angle2 > M_PI / 2)
                {
                    Angle2 = M_PI - Angle2;
                }
                if (Angle2 < Angle1)
                {
                    du = DY;
                    DY = DX;
                    DX = du;
                }
                if (DX.Angle(new Direction3D(DU)) > M_PI / 2)
                {
                    DX.Reverse();
                }
                if (DY.Angle(new Direction3D(DV)) > M_PI / 2)
                {
                    DY.Reverse();
                }

                CoordinateSystem axe = new(Bary, DX ^ DY, DX);
                Plan.SetPosition(axe);
                Plan.SetLocation(Bary);
                IsPlan = true;
            }
        }
        return IsPlan;
    }

    private static bool Controle(Geom_Curve C, in Plane3D Plan, double Tol)
    {
        bool B = true;
        int ii, Nb;
        CurveKind Type;
        GeomAdaptor_Curve AC = new(C);
        Type = AC.GetType();

        switch (Type)
        {
            case Line:
            {
                Nb = 2;
                break;
            }
            case Circle:
            {
                Nb = 3;
                break;
            }

            case Ellipse:
            case Hyperbola:
            case Parabola:
            {
                Nb = 5;
                break;
            }
            case BezierCurve:
            {
                Nb = AC.NbPoles();
                break;
            }
            case BSplineCurve:
            {
                Nb = AC.NbPoles();
                break;
            }
            default:
            {
                Nb = 8 + 3 * AC.NbIntervals(CN);
                break;
            }
        }

        double u, du, f, l, d;
        f = AC.FirstParameter();
        l = AC.LastParameter();
        du = (l - f) / (Nb - 1);
        for (ii = 1; ii <= Nb && B; ii++)
        {
            u = (ii - 1) * du + f;
            d = Plan.Distance(C.Value(u));
            B = d < Tol;
        }

        return B;
    }
}
