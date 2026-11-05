// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_SurfCurvEvolRadInv.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_SurfCurvEvolRadInv.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Laws;
using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Fillets.Blend;

/// <summary>
/// The inverse of the surface/restriction evolving-radius function on a restriction of the surface: a fillet
/// between a curve and the surface, its point on the surface on a 2D arc. The variables are the guide's parameter,
/// the parameter on the curve and the parameter on the arc.
/// </summary>
internal class SurfCurvEvolRadInv : SurfCurvFuncInv
{
    private readonly SurfaceAdaptorBase _surf;
    private readonly CurveAdaptorBase _curv;
    private readonly CurveAdaptorBase _guide;
    private Curve2DAdaptorBase? _rst;
    private double _ray;
    private int _choix;
    private readonly LawsFunction _tevol;
    private double _sg1;

    public SurfCurvEvolRadInv(SurfaceAdaptorBase s, CurveAdaptorBase c, CurveAdaptorBase cg, LawsFunction evol)
    {
        _surf = s;
        _curv = c;
        _guide = cg;
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
        Point3D ptgui;
        Vector3D d1gui;
        _guide.D1(x[1], out ptgui, out d1gui);
        _ray = _sg1 * _tevol.Value(x[1]);
        Vector3D nplan = d1gui.Normalized();
        //  double theD = -(nplan.XYZ().Dot(ptgui.XYZ()));
        Xyz nplanXYZ = nplan.XYZ();
        Xyz ptguiXYZ = ptgui.XYZ();
        double theD = nplanXYZ.Dot(ptguiXYZ);
        theD = theD * (-1.0);

        Point3D ptcur = _curv.Value(x[2]);
        f[1] = nplan.XYZ().Dot(ptcur.XYZ()) + theD;
        Point2D p2drst = _rst!.Value(x[3]);
        Point3D pts;
        Vector3D du, dv;
        _surf.D1(p2drst.X, p2drst.Y, out pts, out du, out dv);
        f[2] = nplan.XYZ().Dot(pts.XYZ()) + theD;
        Vector3D ns = du.Crossed(dv);
        double norm = nplan.Crossed(ns).Magnitude;
        double unsurnorm = 1.0 / norm;
        ns = Vector3D.LinearForm(nplan.Dot(ns), nplan, -1.0, ns);
        ns = ns.Multiplied(unsurnorm);
        Vector3D @ref = new(ptcur, pts);
        @ref = Vector3D.LinearForm(_ray, ns, @ref);
        f[3] = @ref.SquareMagnitude - _ray * _ray;
        return true;
    }

    public override bool Derivatives(math_Vector x, Matrix d)
    {
        Point3D ptgui;
        Vector3D d1gui, d2gui;
        _guide.D2(x[1], out ptgui, out d1gui, out d2gui);
        double normd1gui = d1gui.Magnitude, dray;
        double unsurnormd1gui = 1.0 / normd1gui;
        _tevol.D1(x[1], out _ray, out dray);
        _ray = _sg1 * _ray;
        dray = _sg1 * dray;
        Vector3D nplan = d1gui.Multiplied(unsurnormd1gui);
        Vector3D dnplan = new();
        dnplan = Vector3D.LinearForm(-nplan.Dot(d2gui), nplan, d2gui);
        dnplan = dnplan.Multiplied(unsurnormd1gui);
        double dtheD = -nplan.XYZ().Dot(d1gui.XYZ()) - dnplan.XYZ().Dot(ptgui.XYZ());
        Point3D ptcur;
        Vector3D d1cur;
        _curv.D1(x[2], out ptcur, out d1cur);
        d[1, 1] = dnplan.XYZ().Dot(ptcur.XYZ()) + dtheD;
        d[1, 2] = nplan.XYZ().Dot(d1cur.XYZ());
        d[1, 3] = 0.0;

        Point2D p2drst;
        Vector2D d1rst;
        _rst!.D1(x[3], out p2drst, out d1rst);
        Point3D pts;
        Vector3D d1u, d1v, d2u, d2v, duv;
        _surf.D2(p2drst.X, p2drst.Y, out pts, out d1u, out d1v, out d2u, out d2v, out duv);
        d[2, 1] = dnplan.XYZ().Dot(pts.XYZ()) + dtheD;
        d[2, 2] = 0.0;
        Vector3D dwrstpts = new();
        dwrstpts = Vector3D.LinearForm(d1rst.X, d1u, d1rst.Y, d1v);
        d[2, 3] = nplan.XYZ().Dot(dwrstpts.XYZ());

        Vector3D nsurf = d1u.Crossed(d1v);
        Vector3D dunsurf = d2u.Crossed(d1v).Added(d1u.Crossed(duv));
        Vector3D dvnsurf = d1u.Crossed(d2v).Added(duv.Crossed(d1v));
        Vector3D dwrstnsurf = new();
        dwrstnsurf = Vector3D.LinearForm(d1rst.X, dunsurf, d1rst.Y, dvnsurf);

        Vector3D nplancrosnsurf = nplan.Crossed(nsurf);
        Vector3D dwguinplancrosnsurf = dnplan.Crossed(nsurf);
        Vector3D dwrstnplancrosnsurf = nplan.Crossed(dwrstnsurf);

        double norm2 = nplancrosnsurf.SquareMagnitude;
        double norm = Math.Sqrt(norm2);
        double unsurnorm = 1.0 / norm;
        double raysurnorm = _ray * unsurnorm;
        double unsurnorm2 = unsurnorm * unsurnorm;
        double raysurnorm2 = _ray * unsurnorm2;
        double dwguinorm = unsurnorm * nplancrosnsurf.Dot(dwguinplancrosnsurf);
        double dwrstnorm = unsurnorm * nplancrosnsurf.Dot(dwrstnplancrosnsurf);

        double nplandotnsurf = nplan.Dot(nsurf);
        double dwguinplandotnsurf = dnplan.Dot(nsurf);
        double dwrstnplandotnsurf = nplan.Dot(dwrstnsurf);

        Vector3D temp = new(), dwguitemp = new(), dwrsttemp = new();
        temp = Vector3D.LinearForm(nplandotnsurf, nplan, -1.0, nsurf);
        dwguitemp = Vector3D.LinearForm(nplandotnsurf, dnplan, dwguinplandotnsurf, nplan);
        dwrsttemp = Vector3D.LinearForm(dwrstnplandotnsurf, nplan, -1.0, dwrstnsurf);

        Vector3D corde = new(ptcur, pts);
        Vector3D @ref = new(), dwguiref = new(), dwrstref = new();
        @ref = Vector3D.LinearForm(raysurnorm, temp, corde);
        dwguiref = Vector3D.LinearForm(raysurnorm, dwguitemp, -raysurnorm2 * dwguinorm, temp);
        dwguiref = Vector3D.LinearForm(1.0, dwguiref, dray * unsurnorm, temp);
        dwrstref = Vector3D.LinearForm(raysurnorm, dwrsttemp, -raysurnorm2 * dwrstnorm, temp, dwrstpts);

        @ref = @ref.Added(@ref);
        d[3, 1] = @ref.Dot(dwguiref) - 2.0 * dray * _ray;
        d[3, 2] = -@ref.Dot(d1cur);
        d[3, 3] = @ref.Dot(dwrstref);

        return true;
    }

    public override bool Values(math_Vector x, math_Vector f, Matrix d)
    {
        Point3D ptgui;
        Vector3D d1gui, d2gui;
        _guide.D2(x[1], out ptgui, out d1gui, out d2gui);
        double dray;
        _tevol.D1(x[1], out _ray, out dray);
        _ray = _sg1 * _ray;
        dray = _sg1 * dray;
        double normd1gui = d1gui.Magnitude;
        double unsurnormd1gui = 1.0 / normd1gui;
        Vector3D nplan = d1gui.Multiplied(unsurnormd1gui);
        //  double theD = -(nplan.XYZ().Dot(ptgui.XYZ()));
        Xyz nplanXYZ = nplan.XYZ();
        Xyz ptcurXYZ = ptgui.XYZ();
        double theD = nplanXYZ.Dot(ptcurXYZ);
        theD = theD * (-1.0);

        Vector3D dnplan = new();
        dnplan = Vector3D.LinearForm(-nplan.Dot(d2gui), nplan, d2gui);
        dnplan = dnplan.Multiplied(unsurnormd1gui);
        double dtheD = -nplan.XYZ().Dot(d1gui.XYZ()) - dnplan.XYZ().Dot(ptgui.XYZ());
        Point3D ptcur;
        Vector3D d1cur;
        _curv.D1(x[2], out ptcur, out d1cur);
        f[1] = nplan.XYZ().Dot(ptcur.XYZ()) + theD;
        d[1, 1] = dnplan.XYZ().Dot(ptcur.XYZ()) + dtheD;
        d[1, 2] = nplan.XYZ().Dot(d1cur.XYZ());
        d[1, 3] = 0.0;

        Point2D p2drst;
        Vector2D d1rst;
        _rst!.D1(x[3], out p2drst, out d1rst);
        Point3D pts;
        Vector3D d1u, d1v, d2u, d2v, duv;
        _surf.D2(p2drst.X, p2drst.Y, out pts, out d1u, out d1v, out d2u, out d2v, out duv);
        f[2] = nplan.XYZ().Dot(pts.XYZ()) + theD;
        d[2, 1] = dnplan.XYZ().Dot(pts.XYZ()) + dtheD;
        d[2, 2] = 0.0;
        Vector3D dwrstpts = new();
        dwrstpts = Vector3D.LinearForm(d1rst.X, d1u, d1rst.Y, d1v);
        d[2, 3] = nplan.XYZ().Dot(dwrstpts.XYZ());

        Vector3D nsurf = d1u.Crossed(d1v);
        Vector3D dunsurf = d2u.Crossed(d1v).Added(d1u.Crossed(duv));
        Vector3D dvnsurf = d1u.Crossed(d2v).Added(duv.Crossed(d1v));
        Vector3D dwrstnsurf = new();
        dwrstnsurf = Vector3D.LinearForm(d1rst.X, dunsurf, d1rst.Y, dvnsurf);

        Vector3D nplancrosnsurf = nplan.Crossed(nsurf);
        Vector3D dwguinplancrosnsurf = dnplan.Crossed(nsurf);
        Vector3D dwrstnplancrosnsurf = nplan.Crossed(dwrstnsurf);

        double norm2 = nplancrosnsurf.SquareMagnitude;
        double norm = Math.Sqrt(norm2);
        double unsurnorm = 1.0 / norm;
        double raysurnorm = _ray * unsurnorm;
        double unsurnorm2 = unsurnorm * unsurnorm;
        double raysurnorm2 = _ray * unsurnorm2;
        double dwguinorm = unsurnorm * nplancrosnsurf.Dot(dwguinplancrosnsurf);
        double dwrstnorm = unsurnorm * nplancrosnsurf.Dot(dwrstnplancrosnsurf);

        double nplandotnsurf = nplan.Dot(nsurf);
        double dwguinplandotnsurf = dnplan.Dot(nsurf);
        double dwrstnplandotnsurf = nplan.Dot(dwrstnsurf);

        Vector3D temp = new(), dwguitemp = new(), dwrsttemp = new();
        temp = Vector3D.LinearForm(nplandotnsurf, nplan, -1.0, nsurf);
        dwguitemp = Vector3D.LinearForm(nplandotnsurf, dnplan, dwguinplandotnsurf, nplan);
        dwrsttemp = Vector3D.LinearForm(dwrstnplandotnsurf, nplan, -1.0, dwrstnsurf);

        Vector3D corde = new(ptcur, pts);
        Vector3D @ref = new(), dwguiref = new(), dwrstref = new();
        @ref = Vector3D.LinearForm(raysurnorm, temp, corde);
        f[3] = @ref.SquareMagnitude - _ray * _ray;
        dwguiref = Vector3D.LinearForm(raysurnorm, dwguitemp, -raysurnorm2 * dwguinorm, temp);
        dwguiref = Vector3D.LinearForm(1.0, dwguiref, dray * unsurnorm, temp);
        dwrstref = Vector3D.LinearForm(raysurnorm, dwrsttemp, -raysurnorm2 * dwrstnorm, temp, dwrstpts);

        @ref = @ref.Added(@ref);
        d[3, 1] = @ref.Dot(dwguiref) - 2.0 * dray * _ray;
        d[3, 2] = -@ref.Dot(d1cur);
        d[3, 3] = @ref.Dot(dwrstref);
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
