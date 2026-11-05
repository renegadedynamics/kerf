// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_SurfPointEvolRadInv.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_SurfPointEvolRadInv.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Laws;
using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Fillets.Blend;

/// <summary>
/// The inverse of the surface/restriction evolving-radius function at an extremity: a fillet passing through a
/// given point, on a curve (the guide), tangent to a surface. The variables are the parameter on the curve and (U, V)
/// on the surface.
/// </summary>
internal class SurfPointEvolRadInv : SurfPointFuncInv
{
    private readonly SurfaceAdaptorBase _surf;
    private readonly CurveAdaptorBase _curv;
    private Point3D _point;
    private double _ray;
    private int _choix;
    private readonly LawsFunction _tevol;
    private double _sg1;

    public SurfPointEvolRadInv(SurfaceAdaptorBase s, CurveAdaptorBase c, LawsFunction evol)
    {
        _surf = s;
        _curv = c;
        _tevol = evol;
    }

    /// <summary>Sets the "quadrant" <paramref name="choix"/> (1 to 4).</summary>
    public void Set(int choix)
    {
        _choix = choix;
        switch (_choix)
        {
            case 1:
            case 2:
                _sg1 = -1;
                break;
            case 3:
            case 4:
                _sg1 = 1;
                break;
            default:
                _sg1 = -1;
                break;
        }
    }

    public override int NbEquations()
    {
        return 3;
    }

    public override bool Value(math_Vector x, math_Vector f)
    {
        double theD, norm, unsurnorm;
        Point3D ptcur, pts;
        Vector3D d1cur, d1u, d1v;
        Xyz nplan, ns, @ref;
        _curv.D1(x[1], out ptcur, out d1cur);
        _ray = _sg1 * _tevol.Value(x[1]);
        nplan = d1cur.Normalized().XYZ();
        theD = -(nplan.Dot(ptcur.XYZ()));
        _surf.D1(x[2], x[3], out pts, out d1u, out d1v);
        f[1] = nplan.Dot(_point.XYZ()) + theD;
        f[2] = nplan.Dot(pts.XYZ()) + theD;
        ns = d1u.Crossed(d1v).XYZ();
        norm = nplan.Crossed(ns).Modulus();
        unsurnorm = 1.0 / norm;
        ns.SetLinearForm(nplan.Dot(ns), nplan, -1.0, ns);
        ns.Multiply(unsurnorm);
        @ref = pts.XYZ() - _point.XYZ();
        @ref.SetLinearForm(_ray, ns, @ref);
        f[3] = @ref.SquareModulus() - _ray * _ray;
        return true;
    }

    public override bool Derivatives(math_Vector x, Matrix d)
    {
        Point3D ptcur, pts;
        Vector3D d1cur, d2cur, nplan, dnplan = new(), d1u, d1v, d2u, d2v, duv;
        double dtheD, normd1cur, unsurnormd1cur, dray;

        _curv.D2(x[1], out ptcur, out d1cur, out d2cur);
        _tevol.D1(x[1], out _ray, out dray);
        _ray = _sg1 * _ray;
        dray = _sg1 * dray;
        normd1cur = d1cur.Magnitude;
        unsurnormd1cur = 1.0 / normd1cur;
        nplan = unsurnormd1cur * d1cur;

        dnplan = Vector3D.LinearForm(-nplan.Dot(d2cur), nplan, d2cur);
        dnplan = dnplan.Multiplied(unsurnormd1cur);
        dtheD = -nplan.XYZ().Dot(d1cur.XYZ()) - dnplan.XYZ().Dot(ptcur.XYZ());
        d[1, 1] = dnplan.XYZ().Dot(_point.XYZ()) + dtheD;
        d[1, 2] = d[1, 3] = 0.0;
        _surf.D2(x[2], x[3], out pts, out d1u, out d1v, out d2u, out d2v, out duv);
        d[2, 1] = dnplan.XYZ().Dot(pts.XYZ()) + dtheD;
        d[2, 2] = nplan.Dot(d1u);
        d[2, 3] = nplan.Dot(d1v);

        Vector3D nsurf = d1u.Crossed(d1v);
        Vector3D dunsurf = d2u.Crossed(d1v).Added(d1u.Crossed(duv));
        Vector3D dvnsurf = d1u.Crossed(d2v).Added(duv.Crossed(d1v));

        Vector3D nplancrosnsurf = nplan.Crossed(nsurf);
        Vector3D dwnplancrosnsurf = dnplan.Crossed(nsurf);
        Vector3D dunplancrosnsurf = nplan.Crossed(dunsurf);
        Vector3D dvnplancrosnsurf = nplan.Crossed(dvnsurf);

        double norm2 = nplancrosnsurf.SquareMagnitude;
        double norm = Math.Sqrt(norm2);
        double unsurnorm = 1.0 / norm;
        double raysurnorm = _ray * unsurnorm;
        double unsurnorm2 = unsurnorm * unsurnorm;
        double raysurnorm2 = _ray * unsurnorm2;
        double dwnorm = unsurnorm * nplancrosnsurf.Dot(dwnplancrosnsurf);
        double dunorm = unsurnorm * nplancrosnsurf.Dot(dunplancrosnsurf);
        double dvnorm = unsurnorm * nplancrosnsurf.Dot(dvnplancrosnsurf);

        double nplandotnsurf = nplan.Dot(nsurf);
        double dwnplandotnsurf = dnplan.Dot(nsurf);
        double dunplandotnsurf = nplan.Dot(dunsurf);
        double dvnplandotnsurf = nplan.Dot(dvnsurf);

        Vector3D temp = new(), dwtemp = new(), dutemp = new(), dvtemp = new();
        temp = Vector3D.LinearForm(nplandotnsurf, nplan, -1.0, nsurf);
        dwtemp = Vector3D.LinearForm(nplandotnsurf, dnplan, dwnplandotnsurf, nplan);
        dutemp = Vector3D.LinearForm(dunplandotnsurf, nplan, -1.0, dunsurf);
        dvtemp = Vector3D.LinearForm(dvnplandotnsurf, nplan, -1.0, dvnsurf);

        Vector3D @ref = new(), dwref = new(), duref = new(), dvref = new(), corde = new(_point, pts);
        @ref = Vector3D.LinearForm(raysurnorm, temp, corde);
        dwref = Vector3D.LinearForm(raysurnorm, dwtemp, -raysurnorm2 * dwnorm, temp);
        dwref = Vector3D.LinearForm(1.0, dwref, dray * unsurnorm, temp);
        duref = Vector3D.LinearForm(raysurnorm, dutemp, -raysurnorm2 * dunorm, temp, d1u);
        dvref = Vector3D.LinearForm(raysurnorm, dvtemp, -raysurnorm2 * dvnorm, temp, d1v);

        @ref = @ref.Added(@ref);
        d[3, 1] = @ref.Dot(dwref) - 2.0 * dray * _ray;
        d[3, 2] = @ref.Dot(duref);
        d[3, 3] = @ref.Dot(dvref);

        return true;
    }

    public override bool Values(math_Vector x, math_Vector f, Matrix d)
    {
        Point3D ptcur, pts;
        Vector3D d1cur, d2cur, nplan, dnplan = new(), d1u, d1v, d2u, d2v, duv;
        double theD, dtheD, normd1cur, unsurnormd1cur, dray;

        _curv.D2(x[1], out ptcur, out d1cur, out d2cur);
        _tevol.D1(x[1], out _ray, out dray);
        _ray = _sg1 * _ray;
        dray = _sg1 * dray;
        _surf.D2(x[2], x[3], out pts, out d1u, out d1v, out d2u, out d2v, out duv);
        normd1cur = d1cur.Magnitude;
        unsurnormd1cur = 1.0 / normd1cur;
        nplan = unsurnormd1cur * d1cur;
        theD = -(nplan.XYZ().Dot(ptcur.XYZ()));
        f[1] = nplan.XYZ().Dot(_point.XYZ()) + theD;
        f[2] = nplan.XYZ().Dot(pts.XYZ()) + theD;

        dnplan = Vector3D.LinearForm(-nplan.Dot(d2cur), nplan, d2cur);
        dnplan = dnplan.Multiplied(unsurnormd1cur);
        dtheD = -nplan.XYZ().Dot(d1cur.XYZ()) - dnplan.XYZ().Dot(ptcur.XYZ());
        d[1, 1] = dnplan.XYZ().Dot(_point.XYZ()) + dtheD;
        d[1, 2] = d[1, 3] = 0.0;
        d[2, 1] = dnplan.XYZ().Dot(pts.XYZ()) + dtheD;
        d[2, 2] = nplan.Dot(d1u);
        d[2, 3] = nplan.Dot(d1v);

        Vector3D nsurf = d1u.Crossed(d1v);
        Vector3D dunsurf = d2u.Crossed(d1v).Added(d1u.Crossed(duv));
        Vector3D dvnsurf = d1u.Crossed(d2v).Added(duv.Crossed(d1v));

        Vector3D nplancrosnsurf = nplan.Crossed(nsurf);
        Vector3D dwnplancrosnsurf = dnplan.Crossed(nsurf);
        Vector3D dunplancrosnsurf = nplan.Crossed(dunsurf);
        Vector3D dvnplancrosnsurf = nplan.Crossed(dvnsurf);

        double norm2 = nplancrosnsurf.SquareMagnitude;
        double norm = Math.Sqrt(norm2);
        double unsurnorm = 1.0 / norm;
        double raysurnorm = _ray * unsurnorm;
        double unsurnorm2 = unsurnorm * unsurnorm;
        double raysurnorm2 = _ray * unsurnorm2;
        double dwnorm = unsurnorm * nplancrosnsurf.Dot(dwnplancrosnsurf);
        double dunorm = unsurnorm * nplancrosnsurf.Dot(dunplancrosnsurf);
        double dvnorm = unsurnorm * nplancrosnsurf.Dot(dvnplancrosnsurf);

        double nplandotnsurf = nplan.Dot(nsurf);
        double dwnplandotnsurf = dnplan.Dot(nsurf);
        double dunplandotnsurf = nplan.Dot(dunsurf);
        double dvnplandotnsurf = nplan.Dot(dvnsurf);

        Vector3D temp = new(), dwtemp = new(), dutemp = new(), dvtemp = new();
        temp = Vector3D.LinearForm(nplandotnsurf, nplan, -1.0, nsurf);
        dwtemp = Vector3D.LinearForm(nplandotnsurf, dnplan, dwnplandotnsurf, nplan);
        dutemp = Vector3D.LinearForm(dunplandotnsurf, nplan, -1.0, dunsurf);
        dvtemp = Vector3D.LinearForm(dvnplandotnsurf, nplan, -1.0, dvnsurf);

        Vector3D @ref = new(), dwref = new(), duref = new(), dvref = new(), corde = new(_point, pts);
        @ref = Vector3D.LinearForm(raysurnorm, temp, corde);
        f[3] = @ref.SquareMagnitude - _ray * _ray;
        dwref = Vector3D.LinearForm(raysurnorm, dwtemp, -raysurnorm2 * dwnorm, temp);
        dwref = Vector3D.LinearForm(1.0, dwref, dray * unsurnorm, temp);
        duref = Vector3D.LinearForm(raysurnorm, dutemp, -raysurnorm2 * dunorm, temp, d1u);
        dvref = Vector3D.LinearForm(raysurnorm, dvtemp, -raysurnorm2 * dvnorm, temp, d1v);

        @ref = @ref.Added(@ref);
        d[3, 1] = @ref.Dot(dwref) - 2.0 * dray * _ray;
        d[3, 2] = @ref.Dot(duref);
        d[3, 3] = @ref.Dot(dvref);

        return true;
    }

    public override void Set(in Point3D p)
    {
        _point = p;
    }

    public override void GetTolerance(math_Vector tolerance, double tol)
    {
        tolerance[1] = _curv.Resolution(tol);
        tolerance[2] = _surf.UResolution(tol);
        tolerance[3] = _surf.VResolution(tol);
    }

    public override void GetBounds(math_Vector infBound, math_Vector supBound)
    {
        infBound[1] = _curv.FirstParameter();
        supBound[1] = _curv.LastParameter();
        infBound[2] = _surf.FirstUParameter();
        supBound[2] = _surf.LastUParameter();
        infBound[3] = _surf.FirstVParameter();
        supBound[3] = _surf.LastVParameter();
    }

    public override bool IsSolution(math_Vector sol, double tol)
    {
        math_Vector valsol = new(1, 3);
        Value(sol, valsol);
        return Math.Abs(valsol[1]) <= tol && Math.Abs(valsol[2]) <= tol
               && Math.Abs(valsol[3]) <= 2 * tol * Math.Abs(_ray);
    }
}
