// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_SurfPointConstRadInv.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_SurfPointConstRadInv.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The inverse of the surface/restriction constant-radius function at an extremity: a fillet passing through a
/// given point, on a curve (the guide), tangent to a surface. The variables are the parameter on the curve and (U, V)
/// on the surface.
/// </summary>
public class BRepBlend_SurfPointConstRadInv : Blend_SurfPointFuncInv
{
    private readonly SurfaceAdaptorBase surf;
    private readonly CurveAdaptorBase curv;
    private Point3D point;
    private double ray;
    private int choix;

    public BRepBlend_SurfPointConstRadInv(SurfaceAdaptorBase S, CurveAdaptorBase C)
    {
        surf = S;
        curv = C;
        ray = 0.0;
        choix = 0;
    }

    /// <summary>Sets the radius and the "quadrant" <paramref name="Choix"/> (1 to 4).</summary>
    public void Set(double R, int Choix)
    {
        choix = Choix;
        switch (choix)
        {
            case 1:
            case 2:
            {
                ray = -Math.Abs(R);
            }
            break;
            case 3:
            case 4:
            {
                ray = Math.Abs(R);
            }
            break;
            default:
            {
                ray = -Math.Abs(R);
            }
            break;
        }
    }

    public override int NbEquations()
    {
        return 3;
    }

    public override bool Value(math_Vector X, math_Vector F)
    {
        double theD, norm, unsurnorm;
        Point3D ptcur, pts;
        Vector3D d1cur, d1u, d1v;
        Xyz nplan, ns, @ref;
        curv.D1(X[1], out ptcur, out d1cur);
        nplan = d1cur.Normalized().XYZ();
        //  theD = -(nplan.Dot(ptcur.XYZ()));
        Xyz ptcurXYZ = ptcur.XYZ();
        theD = nplan.Dot(ptcurXYZ);
        theD = theD * (-1.0);

        surf.D1(X[2], X[3], out pts, out d1u, out d1v);
        F[1] = nplan.Dot(point.XYZ()) + theD;
        F[2] = nplan.Dot(pts.XYZ()) + theD;
        ns = d1u.Crossed(d1v).XYZ();
        norm = nplan.Crossed(ns).Modulus();
        unsurnorm = 1.0 / norm;
        ns.SetLinearForm(nplan.Dot(ns), nplan, -1.0, ns);
        ns.Multiply(unsurnorm);
        @ref = pts.XYZ() - point.XYZ();
        @ref.SetLinearForm(ray, ns, @ref);
        F[3] = @ref.SquareModulus() - ray * ray;
        return true;
    }

    public override bool Derivatives(math_Vector X, Matrix D)
    {
        Point3D ptcur, pts;
        Vector3D d1cur, d2cur, nplan, dnplan = new(), d1u, d1v, d2u, d2v, duv;
        double theD, dtheD, normd1cur, unsurnormd1cur;

        curv.D2(X[1], out ptcur, out d1cur, out d2cur);
        normd1cur = d1cur.Magnitude();
        unsurnormd1cur = 1.0 / normd1cur;
        nplan = unsurnormd1cur * d1cur;
        //  theD = -(nplan.XYZ().Dot(ptcur.XYZ()));
        Xyz nplanXYZ = nplan.XYZ();
        Xyz ptcurXYZ = ptcur.XYZ();
        theD = nplanXYZ.Dot(ptcurXYZ);
        theD = theD * (-1.0);

        dnplan.SetLinearForm(-nplan.Dot(d2cur), nplan, d2cur);
        dnplan.Multiply(unsurnormd1cur);
        dtheD = -nplan.XYZ().Dot(d1cur.XYZ()) - dnplan.XYZ().Dot(ptcur.XYZ());
        D[1, 1] = dnplan.XYZ().Dot(point.XYZ()) + dtheD;
        D[1, 2] = D[1, 3] = 0.0;
        surf.D2(X[2], X[3], out pts, out d1u, out d1v, out d2u, out d2v, out duv);
        D[2, 1] = dnplan.XYZ().Dot(pts.XYZ()) + dtheD;
        D[2, 2] = nplan.Dot(d1u);
        D[2, 3] = nplan.Dot(d1v);

        Vector3D nsurf = d1u.Crossed(d1v);
        Vector3D dunsurf = d2u.Crossed(d1v).Added(d1u.Crossed(duv));
        Vector3D dvnsurf = d1u.Crossed(d2v).Added(duv.Crossed(d1v));

        Vector3D nplancrosnsurf = nplan.Crossed(nsurf);
        Vector3D dwnplancrosnsurf = dnplan.Crossed(nsurf);
        Vector3D dunplancrosnsurf = nplan.Crossed(dunsurf);
        Vector3D dvnplancrosnsurf = nplan.Crossed(dvnsurf);

        double norm2 = nplancrosnsurf.SquareMagnitude();
        double norm = Math.Sqrt(norm2);
        double unsurnorm = 1.0 / norm;
        double raysurnorm = ray * unsurnorm;
        double unsurnorm2 = unsurnorm * unsurnorm;
        double raysurnorm2 = ray * unsurnorm2;
        double dwnorm = unsurnorm * nplancrosnsurf.Dot(dwnplancrosnsurf);
        double dunorm = unsurnorm * nplancrosnsurf.Dot(dunplancrosnsurf);
        double dvnorm = unsurnorm * nplancrosnsurf.Dot(dvnplancrosnsurf);

        double nplandotnsurf = nplan.Dot(nsurf);
        double dwnplandotnsurf = dnplan.Dot(nsurf);
        double dunplandotnsurf = nplan.Dot(dunsurf);
        double dvnplandotnsurf = nplan.Dot(dvnsurf);

        Vector3D temp = new(), dwtemp = new(), dutemp = new(), dvtemp = new();
        temp.SetLinearForm(nplandotnsurf, nplan, -1.0, nsurf);
        dwtemp.SetLinearForm(nplandotnsurf, dnplan, dwnplandotnsurf, nplan);
        dutemp.SetLinearForm(dunplandotnsurf, nplan, -1.0, dunsurf);
        dvtemp.SetLinearForm(dvnplandotnsurf, nplan, -1.0, dvnsurf);

        Vector3D @ref = new(), dwref = new(), duref = new(), dvref = new(), corde = new(point, pts);
        @ref.SetLinearForm(raysurnorm, temp, corde);
        dwref.SetLinearForm(raysurnorm, dwtemp, -raysurnorm2 * dwnorm, temp);
        duref.SetLinearForm(raysurnorm, dutemp, -raysurnorm2 * dunorm, temp, d1u);
        dvref.SetLinearForm(raysurnorm, dvtemp, -raysurnorm2 * dvnorm, temp, d1v);

        @ref.Add(@ref);
        D[3, 1] = @ref.Dot(dwref);
        D[3, 2] = @ref.Dot(duref);
        D[3, 3] = @ref.Dot(dvref);

        return true;
    }

    public override bool Values(math_Vector X, math_Vector F, Matrix D)
    {
        Point3D ptcur, pts;
        Vector3D d1cur, d2cur, nplan, dnplan = new(), d1u, d1v, d2u, d2v, duv;
        double theD, dtheD, normd1cur, unsurnormd1cur;

        curv.D2(X[1], out ptcur, out d1cur, out d2cur);
        surf.D2(X[2], X[3], out pts, out d1u, out d1v, out d2u, out d2v, out duv);
        normd1cur = d1cur.Magnitude();
        unsurnormd1cur = 1.0 / normd1cur;
        nplan = unsurnormd1cur * d1cur;
        //  theD = -(nplan.XYZ().Dot(ptcur.XYZ()));
        Xyz nplanXYZ = nplan.XYZ();
        Xyz ptcurXYZ = ptcur.XYZ();
        theD = nplanXYZ.Dot(ptcurXYZ);
        theD = theD * (-1.0);

        F[1] = nplan.XYZ().Dot(point.XYZ()) + theD;
        F[2] = nplan.XYZ().Dot(pts.XYZ()) + theD;

        dnplan.SetLinearForm(-nplan.Dot(d2cur), nplan, d2cur);
        dnplan.Multiply(unsurnormd1cur);
        dtheD = -nplan.XYZ().Dot(d1cur.XYZ()) - dnplan.XYZ().Dot(ptcur.XYZ());
        D[1, 1] = dnplan.XYZ().Dot(point.XYZ()) + dtheD;
        D[1, 2] = D[1, 3] = 0.0;
        D[2, 1] = dnplan.XYZ().Dot(pts.XYZ()) + dtheD;
        D[2, 2] = nplan.Dot(d1u);
        D[2, 3] = nplan.Dot(d1v);

        Vector3D nsurf = d1u.Crossed(d1v);
        Vector3D dunsurf = d2u.Crossed(d1v).Added(d1u.Crossed(duv));
        Vector3D dvnsurf = d1u.Crossed(d2v).Added(duv.Crossed(d1v));

        Vector3D nplancrosnsurf = nplan.Crossed(nsurf);
        Vector3D dwnplancrosnsurf = dnplan.Crossed(nsurf);
        Vector3D dunplancrosnsurf = nplan.Crossed(dunsurf);
        Vector3D dvnplancrosnsurf = nplan.Crossed(dvnsurf);

        double norm2 = nplancrosnsurf.SquareMagnitude();
        double norm = Math.Sqrt(norm2);
        double unsurnorm = 1.0 / norm;
        double raysurnorm = ray * unsurnorm;
        double unsurnorm2 = unsurnorm * unsurnorm;
        double raysurnorm2 = ray * unsurnorm2;
        double dwnorm = unsurnorm * nplancrosnsurf.Dot(dwnplancrosnsurf);
        double dunorm = unsurnorm * nplancrosnsurf.Dot(dunplancrosnsurf);
        double dvnorm = unsurnorm * nplancrosnsurf.Dot(dvnplancrosnsurf);

        double nplandotnsurf = nplan.Dot(nsurf);
        double dwnplandotnsurf = dnplan.Dot(nsurf);
        double dunplandotnsurf = nplan.Dot(dunsurf);
        double dvnplandotnsurf = nplan.Dot(dvnsurf);

        Vector3D temp = new(), dwtemp = new(), dutemp = new(), dvtemp = new();
        temp.SetLinearForm(nplandotnsurf, nplan, -1.0, nsurf);
        dwtemp.SetLinearForm(nplandotnsurf, dnplan, dwnplandotnsurf, nplan);
        dutemp.SetLinearForm(dunplandotnsurf, nplan, -1.0, dunsurf);
        dvtemp.SetLinearForm(dvnplandotnsurf, nplan, -1.0, dvnsurf);

        Vector3D @ref = new(), dwref = new(), duref = new(), dvref = new(), corde = new(point, pts);
        @ref.SetLinearForm(raysurnorm, temp, corde);
        F[3] = @ref.SquareMagnitude() - ray * ray;
        dwref.SetLinearForm(raysurnorm, dwtemp, -raysurnorm2 * dwnorm, temp);
        duref.SetLinearForm(raysurnorm, dutemp, -raysurnorm2 * dunorm, temp, d1u);
        dvref.SetLinearForm(raysurnorm, dvtemp, -raysurnorm2 * dvnorm, temp, d1v);

        @ref.Add(@ref);
        D[3, 1] = @ref.Dot(dwref);
        D[3, 2] = @ref.Dot(duref);
        D[3, 3] = @ref.Dot(dvref);

        return true;
    }

    public override void Set(in Point3D P)
    {
        point = P;
    }

    public override void GetTolerance(math_Vector Tolerance, double Tol)
    {
        Tolerance[1] = curv.Resolution(Tol);
        Tolerance[2] = surf.UResolution(Tol);
        Tolerance[3] = surf.VResolution(Tol);
    }

    public override void GetBounds(math_Vector InfBound, math_Vector SupBound)
    {
        InfBound[1] = curv.FirstParameter();
        SupBound[1] = curv.LastParameter();
        InfBound[2] = surf.FirstUParameter();
        SupBound[2] = surf.LastUParameter();
        InfBound[3] = surf.FirstVParameter();
        SupBound[3] = surf.LastVParameter();
    }

    public override bool IsSolution(math_Vector Sol, double Tol)
    {
        math_Vector valsol = new(1, 3);
        Value(Sol, valsol);
        return Math.Abs(valsol[1]) <= Tol && Math.Abs(valsol[2]) <= Tol
               && Math.Abs(valsol[3]) <= 2 * Tol * Math.Abs(ray);
    }
}
