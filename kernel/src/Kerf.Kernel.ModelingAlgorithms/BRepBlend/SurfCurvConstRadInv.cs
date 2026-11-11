// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_SurfCurvConstRadInv.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_SurfCurvConstRadInv.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Fillets.BlendFunctions;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Kerf.Engine.Fillets.Blend;

/// <summary>
/// The inverse of the surface/restriction constant-radius function on a restriction of the surface: a fillet
/// between a curve and the surface, its point on the surface on a 2D arc. The variables are the guide's parameter,
/// the parameter on the curve and the parameter on the arc.
/// </summary>
internal class SurfCurvConstRadInv : SurfCurvFuncInv
{
    private readonly SurfaceAdaptorBase _surf;
    private readonly CurveAdaptorBase _curv;
    private readonly CurveAdaptorBase _guide;
    private Curve2DAdaptorBase? _rst;
    private double _ray;
    private int _choix;

    public SurfCurvConstRadInv(SurfaceAdaptorBase s, CurveAdaptorBase c, CurveAdaptorBase cg)
    {
        _surf = s;
        _curv = c;
        _guide = cg;
        _ray = 0.0;
        _choix = 0;
    }

    /// <summary>Sets the radius and the "quadrant" <paramref name="choix"/> (1 to 4).</summary>
    public void Set(double r, int choix)
    {
        _choix = choix;
        switch (_choix)
        {
            case 1:
            case 2:
            {
                _ray = -Math.Abs(r);
            }
            break;
            case 3:
            case 4:
            {
                _ray = Math.Abs(r);
            }
            break;
            default:
            {
                _ray = -Math.Abs(r);
            }
            break;
        }
    }

    public override int NbEquations()
    {
        return 3;
    }

    // Kerf (decision 2158): the rolling ball's equations, as SurfRstConstRad's: the centre at the radius from the
    // surface's point along its own unit normal, square to the curve at its point and as far from it as the radius; the
    // guide's plane through the two points' midpoint.
    private void Ball(math_Vector x, int order, math_Vector f, Matrix? d)
    {
        Point3D ptgui;
        Vector3D d1gui, d2gui;
        _guide.D2(x[1], out ptgui, out d1gui, out d2gui);
        double normd1gui = d1gui.Magnitude;
        Vector3D nplan = d1gui / normd1gui;
        double theD = -(nplan.XYZ().Dot(ptgui.XYZ()));

        _curv.D2(x[2], out Point3D ptcur, out Vector3D d1cur, out Vector3D d2cur);
        double invnormcur = ConstRad.InverseNorm(d1cur);
        Vector3D tcur = invnormcur * d1cur;

        _rst!.D1(x[3], out Point2D p2drst, out Vector2D d1rst);
        _surf.D2(p2drst.X, p2drst.Y, out Point3D pts, out Vector3D d1u, out Vector3D d1v, out Vector3D d2u, out Vector3D d2v,
                 out Vector3D duv);
        Vector3D nsurf = d1u.Crossed(d1v);
        double invnorm = ConstRad.InverseNorm(nsurf);
        Vector3D nn = invnorm * nsurf;
        Point3D center = pts.Translated(-_ray * nn);
        Vector3D corde = new(ptcur, center);

        f[1] = (nplan.XYZ().Dot(pts.XYZ()) + nplan.XYZ().Dot(ptcur.XYZ())) / 2 + theD;
        f[2] = corde.Dot(tcur);
        f[3] = corde.SquareMagnitude - _ray * _ray;
        if (order == 0 || d is null)
        {
            return;
        }

        Vector3D dnplan = (d2gui - nplan.Dot(d2gui) * nplan) / normd1gui;
        Vector3D mid = new((pts.XYZ() + ptcur.XYZ()) / 2 - ptgui.XYZ());

        // In the guide's parameter: only the plane moves.
        d[1, 1] = dnplan.Dot(mid) - normd1gui;
        d[2, 1] = 0.0;
        d[3, 1] = 0.0;

        // In the curve's parameter.
        d[1, 2] = nplan.Dot(d1cur) / 2;
        d[2, 2] = corde.Dot(ConstRad.UnitNormalD1(tcur, invnormcur, d2cur)) - d1cur.Dot(tcur);
        d[3, 2] = -2 * corde.Dot(d1cur);

        // In the arc's parameter: the surface's point and its reversed unit normal along the arc.
        Vector3D dwrstpts = Vector3D.LinearForm(d1rst.X, d1u, d1rst.Y, d1v);
        Vector3D dwrstnsurf = Vector3D.LinearForm(d1rst.X, d2u.Crossed(d1v).Added(d1u.Crossed(duv)), d1rst.Y,
                                                  d1u.Crossed(d2v).Added(duv.Crossed(d1v)));
        Vector3D dcenter = dwrstpts - _ray * ConstRad.UnitNormalD1(nn, invnorm, dwrstnsurf);
        d[1, 3] = nplan.Dot(dwrstpts) / 2;
        d[2, 3] = dcenter.Dot(tcur);
        d[3, 3] = 2 * corde.Dot(dcenter);
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

    public override void Set(Curve2DAdaptorBase rst)
    {
        _rst = rst;
    }

    public override void GetTolerance(math_Vector tolerance, double tol)
    {
        tolerance[1] = _guide.Resolution(tol);
        tolerance[2] = _curv.Resolution(tol);
        double ru, rv;
        ru = _surf.UResolution(tol);
        rv = _surf.VResolution(tol);
        tolerance[3] = _rst!.Resolution(Min(ru, rv));
    }

    public override void GetBounds(math_Vector infBound, math_Vector supBound)
    {
        infBound[1] = _guide.FirstParameter();
        supBound[1] = _guide.LastParameter();
        infBound[2] = _curv.FirstParameter();
        supBound[2] = _curv.LastParameter();
        infBound[3] = _rst!.FirstParameter();
        supBound[3] = _rst.LastParameter();
    }

    public override bool IsSolution(math_Vector sol, double tol)
    {
        math_Vector valsol = new(1, 3);
        Value(sol, valsol);
        return Math.Abs(valsol[1]) <= tol && Math.Abs(valsol[2]) <= tol
               && Math.Abs(valsol[3]) <= 2 * tol * Math.Abs(_ray);
    }
}
