// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_SurfPointEvolRadInv.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_SurfPointEvolRadInv.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Laws;
using Kerf.Engine.Fillets.BlendFunctions;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;

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

    // Kerf (decision 2216): the rolling ball's equations, as SurfPointConstRadInv's (decision 2158), of the radius the
    // law gives at the guide's parameter: the centre at the radius from the surface's point along its own unit normal,
    // as far from the point as the radius and square there to the restriction's tangent (SetTangent; OCCT has the point
    // in the guide's plane instead); the guide's plane through the two points' midpoint.
    private Vector3D _tangent;

    /// <summary>The restriction's tangent at the point <see cref="Set(in Point3D)"/> gives, which the ball is square to.</summary>
    public void SetTangent(in Vector3D tangent)
    {
        _tangent = tangent.Magnitude > 0 ? tangent.Normalized() : tangent;
    }

    private void Ball(math_Vector x, int order, math_Vector f, Matrix? d)
    {
        _curv.D2(x[1], out Point3D ptcur, out Vector3D d1cur, out Vector3D d2cur);
        double normd1cur = d1cur.Magnitude;
        Vector3D nplan = d1cur / normd1cur;
        double theD = -(nplan.XYZ().Dot(ptcur.XYZ()));
        _tevol.D1(x[1], out double ray, out double dray);
        _ray = _sg1 * ray;
        dray = _sg1 * dray;

        _surf.D2(x[2], x[3], out Point3D pts, out Vector3D d1u, out Vector3D d1v, out Vector3D d2u, out Vector3D d2v,
                 out Vector3D duv);
        Vector3D nsurf = d1u.Crossed(d1v);
        double invnorm = ConstRad.InverseNorm(nsurf);
        Vector3D nn = invnorm * nsurf;
        Point3D center = pts.Translated(-_ray * nn);
        Vector3D corde = new(_point, center);

        f[1] = (nplan.XYZ().Dot(pts.XYZ()) + nplan.XYZ().Dot(_point.XYZ())) / 2 + theD;
        f[2] = corde.Dot(_tangent);
        f[3] = corde.SquareMagnitude - _ray * _ray;
        if (order == 0 || d is null)
        {
            return;
        }

        // In the guide's parameter: the plane, and the radius, the centre moving by -dray along the unit normal.
        Vector3D dnplan = (d2cur - nplan.Dot(d2cur) * nplan) / normd1cur;
        Vector3D mid = new((pts.XYZ() + _point.XYZ()) / 2 - ptcur.XYZ());
        d[1, 1] = dnplan.Dot(mid) - normd1cur;
        d[2, 1] = -dray * nn.Dot(_tangent);
        d[3, 1] = -2 * dray * corde.Dot(nn) - 2 * _ray * dray;

        // In u and v: the surface's point and its reversed unit normal.
        Vector3D dcu = d1u - _ray * ConstRad.UnitNormalD1(nn, invnorm, d2u.Crossed(d1v).Added(d1u.Crossed(duv)));
        Vector3D dcv = d1v - _ray * ConstRad.UnitNormalD1(nn, invnorm, duv.Crossed(d1v).Added(d1u.Crossed(d2v)));
        d[1, 2] = nplan.Dot(d1u) / 2;
        d[1, 3] = nplan.Dot(d1v) / 2;
        d[2, 2] = dcu.Dot(_tangent);
        d[2, 3] = dcv.Dot(_tangent);
        d[3, 2] = 2 * corde.Dot(dcu);
        d[3, 3] = 2 * corde.Dot(dcv);
    }

    public override bool Value(math_Vector x, math_Vector f)
    {
        Ball(x, 0, f, null);
        return true;
    }

    public override bool Derivatives(math_Vector x, Matrix d)
    {
        Ball(x, 1, new math_Vector(1, 3), d);
        return true;
    }

    public override bool Values(math_Vector x, math_Vector f, Matrix d)
    {
        Ball(x, 1, f, d);
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
