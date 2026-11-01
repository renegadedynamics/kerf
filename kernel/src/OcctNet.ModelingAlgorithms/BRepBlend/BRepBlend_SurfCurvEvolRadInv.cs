// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_SurfCurvEvolRadInv.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_SurfCurvEvolRadInv.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The inverse of the surface/restriction evolving-radius function on a restriction of the surface: a fillet
/// between a curve and the surface, its point on the surface on a 2D arc. The variables are the guide's parameter,
/// the parameter on the curve and the parameter on the arc.
/// </summary>
public class BRepBlend_SurfCurvEvolRadInv : Blend_SurfCurvFuncInv
{
    private readonly Adaptor3d_Surface surf;
    private readonly Adaptor3d_Curve curv;
    private readonly Adaptor3d_Curve guide;
    private Adaptor2d_Curve2d? rst;
    private double ray;
    private int choix;
    private readonly Law_Function tevol;
    private double sg1;

    public BRepBlend_SurfCurvEvolRadInv(Adaptor3d_Surface S, Adaptor3d_Curve C, Adaptor3d_Curve Cg, Law_Function Evol)
    {
        surf = S;
        curv = C;
        guide = Cg;
        tevol = Evol;
    }

    /// <summary>Sets the "quadrant" <paramref name="Choix"/> (1 to 4).</summary>
    public void Set(int Choix)
    {
        choix = Choix;
        switch (choix)
        {
            case 1:
            case 2:
                sg1 = -1;
                break;
            case 3:
            case 4:
                sg1 = 1;
                break;
            default:
                sg1 = -1;
                break;
        }
    }

    public override int NbEquations()
    {
        return 3;
    }

    public override bool Value(math_Vector X, math_Vector F)
    {
        gp_Pnt ptgui;
        gp_Vec d1gui;
        guide.D1(X[1], out ptgui, out d1gui);
        ray = sg1 * tevol.Value(X[1]);
        gp_Vec nplan = d1gui.Normalized();
        //  double theD = -(nplan.XYZ().Dot(ptgui.XYZ()));
        gp_XYZ nplanXYZ = nplan.XYZ();
        gp_XYZ ptguiXYZ = ptgui.XYZ();
        double theD = nplanXYZ.Dot(ptguiXYZ);
        theD = theD * (-1.0);

        gp_Pnt ptcur = curv.Value(X[2]);
        F[1] = nplan.XYZ().Dot(ptcur.XYZ()) + theD;
        gp_Pnt2d p2drst = rst!.Value(X[3]);
        gp_Pnt pts;
        gp_Vec du, dv;
        surf.D1(p2drst.X(), p2drst.Y(), out pts, out du, out dv);
        F[2] = nplan.XYZ().Dot(pts.XYZ()) + theD;
        gp_Vec ns = du.Crossed(dv);
        double norm = nplan.Crossed(ns).Magnitude();
        double unsurnorm = 1.0 / norm;
        ns.SetLinearForm(nplan.Dot(ns), nplan, -1.0, ns);
        ns.Multiply(unsurnorm);
        gp_Vec @ref = new(ptcur, pts);
        @ref.SetLinearForm(ray, ns, @ref);
        F[3] = @ref.SquareMagnitude() - ray * ray;
        return true;
    }

    public override bool Derivatives(math_Vector X, math_Matrix D)
    {
        gp_Pnt ptgui;
        gp_Vec d1gui, d2gui;
        guide.D2(X[1], out ptgui, out d1gui, out d2gui);
        double normd1gui = d1gui.Magnitude(), dray;
        double unsurnormd1gui = 1.0 / normd1gui;
        tevol.D1(X[1], out ray, out dray);
        ray = sg1 * ray;
        dray = sg1 * dray;
        gp_Vec nplan = d1gui.Multiplied(unsurnormd1gui);
        gp_Vec dnplan = new();
        dnplan.SetLinearForm(-nplan.Dot(d2gui), nplan, d2gui);
        dnplan.Multiply(unsurnormd1gui);
        double dtheD = -nplan.XYZ().Dot(d1gui.XYZ()) - dnplan.XYZ().Dot(ptgui.XYZ());
        gp_Pnt ptcur;
        gp_Vec d1cur;
        curv.D1(X[2], out ptcur, out d1cur);
        D[1, 1] = dnplan.XYZ().Dot(ptcur.XYZ()) + dtheD;
        D[1, 2] = nplan.XYZ().Dot(d1cur.XYZ());
        D[1, 3] = 0.0;

        gp_Pnt2d p2drst;
        gp_Vec2d d1rst;
        rst!.D1(X[3], out p2drst, out d1rst);
        gp_Pnt pts;
        gp_Vec d1u, d1v, d2u, d2v, duv;
        surf.D2(p2drst.X(), p2drst.Y(), out pts, out d1u, out d1v, out d2u, out d2v, out duv);
        D[2, 1] = dnplan.XYZ().Dot(pts.XYZ()) + dtheD;
        D[2, 2] = 0.0;
        gp_Vec dwrstpts = new();
        dwrstpts.SetLinearForm(d1rst.X(), d1u, d1rst.Y(), d1v);
        D[2, 3] = nplan.XYZ().Dot(dwrstpts.XYZ());

        gp_Vec nsurf = d1u.Crossed(d1v);
        gp_Vec dunsurf = d2u.Crossed(d1v).Added(d1u.Crossed(duv));
        gp_Vec dvnsurf = d1u.Crossed(d2v).Added(duv.Crossed(d1v));
        gp_Vec dwrstnsurf = new();
        dwrstnsurf.SetLinearForm(d1rst.X(), dunsurf, d1rst.Y(), dvnsurf);

        gp_Vec nplancrosnsurf = nplan.Crossed(nsurf);
        gp_Vec dwguinplancrosnsurf = dnplan.Crossed(nsurf);
        gp_Vec dwrstnplancrosnsurf = nplan.Crossed(dwrstnsurf);

        double norm2 = nplancrosnsurf.SquareMagnitude();
        double norm = Math.Sqrt(norm2);
        double unsurnorm = 1.0 / norm;
        double raysurnorm = ray * unsurnorm;
        double unsurnorm2 = unsurnorm * unsurnorm;
        double raysurnorm2 = ray * unsurnorm2;
        double dwguinorm = unsurnorm * nplancrosnsurf.Dot(dwguinplancrosnsurf);
        double dwrstnorm = unsurnorm * nplancrosnsurf.Dot(dwrstnplancrosnsurf);

        double nplandotnsurf = nplan.Dot(nsurf);
        double dwguinplandotnsurf = dnplan.Dot(nsurf);
        double dwrstnplandotnsurf = nplan.Dot(dwrstnsurf);

        gp_Vec temp = new(), dwguitemp = new(), dwrsttemp = new();
        temp.SetLinearForm(nplandotnsurf, nplan, -1.0, nsurf);
        dwguitemp.SetLinearForm(nplandotnsurf, dnplan, dwguinplandotnsurf, nplan);
        dwrsttemp.SetLinearForm(dwrstnplandotnsurf, nplan, -1.0, dwrstnsurf);

        gp_Vec corde = new(ptcur, pts);
        gp_Vec @ref = new(), dwguiref = new(), dwrstref = new();
        @ref.SetLinearForm(raysurnorm, temp, corde);
        dwguiref.SetLinearForm(raysurnorm, dwguitemp, -raysurnorm2 * dwguinorm, temp);
        dwguiref.SetLinearForm(1.0, dwguiref, dray * unsurnorm, temp);
        dwrstref.SetLinearForm(raysurnorm, dwrsttemp, -raysurnorm2 * dwrstnorm, temp, dwrstpts);

        @ref.Add(@ref);
        D[3, 1] = @ref.Dot(dwguiref) - 2.0 * dray * ray;
        D[3, 2] = -@ref.Dot(d1cur);
        D[3, 3] = @ref.Dot(dwrstref);

        return true;
    }

    public override bool Values(math_Vector X, math_Vector F, math_Matrix D)
    {
        gp_Pnt ptgui;
        gp_Vec d1gui, d2gui;
        guide.D2(X[1], out ptgui, out d1gui, out d2gui);
        double dray;
        tevol.D1(X[1], out ray, out dray);
        ray = sg1 * ray;
        dray = sg1 * dray;
        double normd1gui = d1gui.Magnitude();
        double unsurnormd1gui = 1.0 / normd1gui;
        gp_Vec nplan = d1gui.Multiplied(unsurnormd1gui);
        //  double theD = -(nplan.XYZ().Dot(ptgui.XYZ()));
        gp_XYZ nplanXYZ = nplan.XYZ();
        gp_XYZ ptcurXYZ = ptgui.XYZ();
        double theD = nplanXYZ.Dot(ptcurXYZ);
        theD = theD * (-1.0);

        gp_Vec dnplan = new();
        dnplan.SetLinearForm(-nplan.Dot(d2gui), nplan, d2gui);
        dnplan.Multiply(unsurnormd1gui);
        double dtheD = -nplan.XYZ().Dot(d1gui.XYZ()) - dnplan.XYZ().Dot(ptgui.XYZ());
        gp_Pnt ptcur;
        gp_Vec d1cur;
        curv.D1(X[2], out ptcur, out d1cur);
        F[1] = nplan.XYZ().Dot(ptcur.XYZ()) + theD;
        D[1, 1] = dnplan.XYZ().Dot(ptcur.XYZ()) + dtheD;
        D[1, 2] = nplan.XYZ().Dot(d1cur.XYZ());
        D[1, 3] = 0.0;

        gp_Pnt2d p2drst;
        gp_Vec2d d1rst;
        rst!.D1(X[3], out p2drst, out d1rst);
        gp_Pnt pts;
        gp_Vec d1u, d1v, d2u, d2v, duv;
        surf.D2(p2drst.X(), p2drst.Y(), out pts, out d1u, out d1v, out d2u, out d2v, out duv);
        F[2] = nplan.XYZ().Dot(pts.XYZ()) + theD;
        D[2, 1] = dnplan.XYZ().Dot(pts.XYZ()) + dtheD;
        D[2, 2] = 0.0;
        gp_Vec dwrstpts = new();
        dwrstpts.SetLinearForm(d1rst.X(), d1u, d1rst.Y(), d1v);
        D[2, 3] = nplan.XYZ().Dot(dwrstpts.XYZ());

        gp_Vec nsurf = d1u.Crossed(d1v);
        gp_Vec dunsurf = d2u.Crossed(d1v).Added(d1u.Crossed(duv));
        gp_Vec dvnsurf = d1u.Crossed(d2v).Added(duv.Crossed(d1v));
        gp_Vec dwrstnsurf = new();
        dwrstnsurf.SetLinearForm(d1rst.X(), dunsurf, d1rst.Y(), dvnsurf);

        gp_Vec nplancrosnsurf = nplan.Crossed(nsurf);
        gp_Vec dwguinplancrosnsurf = dnplan.Crossed(nsurf);
        gp_Vec dwrstnplancrosnsurf = nplan.Crossed(dwrstnsurf);

        double norm2 = nplancrosnsurf.SquareMagnitude();
        double norm = Math.Sqrt(norm2);
        double unsurnorm = 1.0 / norm;
        double raysurnorm = ray * unsurnorm;
        double unsurnorm2 = unsurnorm * unsurnorm;
        double raysurnorm2 = ray * unsurnorm2;
        double dwguinorm = unsurnorm * nplancrosnsurf.Dot(dwguinplancrosnsurf);
        double dwrstnorm = unsurnorm * nplancrosnsurf.Dot(dwrstnplancrosnsurf);

        double nplandotnsurf = nplan.Dot(nsurf);
        double dwguinplandotnsurf = dnplan.Dot(nsurf);
        double dwrstnplandotnsurf = nplan.Dot(dwrstnsurf);

        gp_Vec temp = new(), dwguitemp = new(), dwrsttemp = new();
        temp.SetLinearForm(nplandotnsurf, nplan, -1.0, nsurf);
        dwguitemp.SetLinearForm(nplandotnsurf, dnplan, dwguinplandotnsurf, nplan);
        dwrsttemp.SetLinearForm(dwrstnplandotnsurf, nplan, -1.0, dwrstnsurf);

        gp_Vec corde = new(ptcur, pts);
        gp_Vec @ref = new(), dwguiref = new(), dwrstref = new();
        @ref.SetLinearForm(raysurnorm, temp, corde);
        F[3] = @ref.SquareMagnitude() - ray * ray;
        dwguiref.SetLinearForm(raysurnorm, dwguitemp, -raysurnorm2 * dwguinorm, temp);
        dwguiref.SetLinearForm(1.0, dwguiref, dray * unsurnorm, temp);
        dwrstref.SetLinearForm(raysurnorm, dwrsttemp, -raysurnorm2 * dwrstnorm, temp, dwrstpts);

        @ref.Add(@ref);
        D[3, 1] = @ref.Dot(dwguiref) - 2.0 * dray * ray;
        D[3, 2] = -@ref.Dot(d1cur);
        D[3, 3] = @ref.Dot(dwrstref);
        return true;
    }

    public override void Set(Adaptor2d_Curve2d Rst)
    {
        rst = Rst;
    }

    public override void GetTolerance(math_Vector Tolerance, double Tol)
    {
        Tolerance[1] = guide.Resolution(Tol);
        Tolerance[2] = curv.Resolution(Tol);
        double ru, rv;
        ru = surf.UResolution(Tol);
        rv = surf.VResolution(Tol);
        Tolerance[3] = rst!.Resolution(Min(ru, rv));
    }

    public override void GetBounds(math_Vector InfBound, math_Vector SupBound)
    {
        InfBound[1] = guide.FirstParameter();
        SupBound[1] = guide.LastParameter();
        InfBound[2] = curv.FirstParameter();
        SupBound[2] = curv.LastParameter();
        InfBound[3] = rst!.FirstParameter();
        SupBound[3] = rst.LastParameter();
    }

    public override bool IsSolution(math_Vector Sol, double Tol)
    {
        math_Vector valsol = new(1, 3);
        Value(Sol, valsol);
        return Math.Abs(valsol[1]) <= Tol && Math.Abs(valsol[2]) <= Tol
               && Math.Abs(valsol[3]) <= 2 * Tol * Math.Abs(ray);
    }
}
