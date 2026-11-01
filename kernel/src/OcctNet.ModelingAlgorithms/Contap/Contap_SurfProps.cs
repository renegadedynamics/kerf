// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_SurfProps.hxx
// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_SurfProps.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_SurfaceType;

namespace Occt;

/// <summary>
/// The point, the (non-normalized) normal and its derivatives of a surface adaptor, in closed form on the elementary
/// surfaces and from the derivatives otherwise: what the contour functions evaluate.
/// </summary>
public static class Contap_SurfProps
{
    /// <summary>Computes the point <paramref name="P"/> and the normal <paramref name="Norm"/> of <paramref name="S"/> at (U, V).</summary>
    public static void Normale(Adaptor3d_Surface S, double U, double V, out gp_Pnt P, out gp_Vec Norm)
    {
        Norm = new gp_Vec();
        GeomAbs_SurfaceType typS = Adaptor3d_HSurfaceTool.GetType(S);
        switch (typS)
        {
            case GeomAbs_Plane:
            {
                gp_Pln pl = Adaptor3d_HSurfaceTool.Plane(S);
                Norm = new gp_Vec(pl.Axis().Direction());
                P = ElSLib.Value(U, V, pl);
                if (!pl.Direct())
                {
                    Norm.Reverse();
                }
            }
            break;

            case GeomAbs_Sphere:
            {
                gp_Sphere sp = Adaptor3d_HSurfaceTool.Sphere(S);
                P = ElSLib.Value(U, V, sp);
                Norm = new gp_Vec(sp.Location(), P);
                if (sp.Direct())
                {
                    Norm.Divide(sp.Radius());
                }
                else
                {
                    Norm.Divide(-sp.Radius());
                }
            }
            break;

            case GeomAbs_Cylinder:
            {
                gp_Cylinder cy = Adaptor3d_HSurfaceTool.Cylinder(S);
                P = ElSLib.Value(U, V, cy);
                Norm.SetLinearForm(Math.Cos(U), new gp_Vec(cy.XAxis().Direction()), Math.Sin(U),
                                   new gp_Vec(cy.YAxis().Direction()));
                if (!cy.Direct())
                {
                    Norm.Reverse();
                }
            }
            break;

            case GeomAbs_Cone:
            {
                gp_Cone co = Adaptor3d_HSurfaceTool.Cone(S);
                P = ElSLib.Value(U, V, co);
                double Angle = co.SemiAngle();
                double Sina = Math.Sin(Angle);
                double Cosa = Math.Cos(Angle);
                double Rad = co.RefRadius();

                double Vcalc = V;
                if (Math.Abs(V * Sina + Rad) <= 1e-12)
                { // on est a l'apex
                    Norm.SetCoord(0, 0, 0);
                    return;
                }

                if (Rad + Vcalc * Sina < 0.0)
                {
                    Norm.SetLinearForm(Sina, new gp_Vec(co.Axis().Direction()), Cosa * Math.Cos(U),
                                       new gp_Vec(co.XAxis().Direction()), Cosa * Math.Sin(U),
                                       new gp_Vec(co.YAxis().Direction()));
                }
                else
                {
                    Norm.SetLinearForm(-Sina, new gp_Vec(co.Axis().Direction()), Cosa * Math.Cos(U),
                                       new gp_Vec(co.XAxis().Direction()), Cosa * Math.Sin(U),
                                       new gp_Vec(co.YAxis().Direction()));
                }
                if (!co.Direct())
                {
                    Norm.Reverse();
                }
            }
            break;
            default:
            {
                Adaptor3d_HSurfaceTool.D1(S, U, V, out P, out gp_Vec d1u, out gp_Vec d1v);
                Norm = d1u.Crossed(d1v);
            }
            break;
        }
    }

    /// <summary>Computes the point, the first derivatives and the normal of <paramref name="S"/> at (U, V).</summary>
    public static void DerivAndNorm(Adaptor3d_Surface S, double U, double V, out gp_Pnt P, out gp_Vec d1u,
                                    out gp_Vec d1v, out gp_Vec Norm)
    {
        Norm = new gp_Vec();
        GeomAbs_SurfaceType typS = Adaptor3d_HSurfaceTool.GetType(S);
        switch (typS)
        {
            case GeomAbs_Plane:
            {
                gp_Pln pl = Adaptor3d_HSurfaceTool.Plane(S);
                Norm = new gp_Vec(pl.Axis().Direction());
                ElSLib.D1(U, V, pl, out P, out d1u, out d1v);
                if (!pl.Direct())
                {
                    Norm.Reverse();
                }
            }
            break;

            case GeomAbs_Sphere:
            {
                gp_Sphere sp = Adaptor3d_HSurfaceTool.Sphere(S);
                ElSLib.D1(U, V, sp, out P, out d1u, out d1v);
                Norm = new gp_Vec(sp.Location(), P);
                if (sp.Direct())
                {
                    Norm.Divide(sp.Radius());
                }
                else
                {
                    Norm.Divide(-sp.Radius());
                }
            }
            break;

            case GeomAbs_Cylinder:
            {
                gp_Cylinder cy = Adaptor3d_HSurfaceTool.Cylinder(S);
                ElSLib.D1(U, V, cy, out P, out d1u, out d1v);
                Norm.SetLinearForm(Math.Cos(U), new gp_Vec(cy.XAxis().Direction()), Math.Sin(U),
                                   new gp_Vec(cy.YAxis().Direction()));
                if (!cy.Direct())
                {
                    Norm.Reverse();
                }
            }
            break;

            case GeomAbs_Cone:
            {
                gp_Cone co = Adaptor3d_HSurfaceTool.Cone(S);
                ElSLib.D1(U, V, co, out P, out d1u, out d1v);
                double Angle = co.SemiAngle();
                double Sina = Math.Sin(Angle);
                double Cosa = Math.Cos(Angle);
                double Rad = co.RefRadius();

                double Vcalc = V;
                if (Math.Abs(V * Sina + Rad) <= RealEpsilon())
                { // on est a l'apex
                    double Vfi = Adaptor3d_HSurfaceTool.FirstVParameter(S);
                    if (Vfi < -Rad / Sina)
                    { // partie valide pour V < Vapex
                        Vcalc = V - 1;
                    }
                    else
                    {
                        Vcalc = V + 1.0;
                    }
                }

                if (Rad + Vcalc * Sina < 0.0)
                {
                    Norm.SetLinearForm(Sina, new gp_Vec(co.Axis().Direction()), Cosa * Math.Cos(U),
                                       new gp_Vec(co.XAxis().Direction()), Cosa * Math.Sin(U),
                                       new gp_Vec(co.YAxis().Direction()));
                }
                else
                {
                    Norm.SetLinearForm(-Sina, new gp_Vec(co.Axis().Direction()), Cosa * Math.Cos(U),
                                       new gp_Vec(co.XAxis().Direction()), Cosa * Math.Sin(U),
                                       new gp_Vec(co.YAxis().Direction()));
                }
                if (!co.Direct())
                {
                    Norm.Reverse();
                }
            }
            break;
            default:
            {
                Adaptor3d_HSurfaceTool.D1(S, U, V, out P, out d1u, out d1v);
                Norm = d1u.Crossed(d1v);
            }
            break;
        }
    }

    /// <summary>Computes the point, the normal and its derivatives of <paramref name="S"/> at (U, V).</summary>
    public static void NormAndDn(Adaptor3d_Surface S, double U, double V, out gp_Pnt P, out gp_Vec Norm,
                                 out gp_Vec Dnu, out gp_Vec Dnv)
    {
        Norm = new gp_Vec();
        Dnu = new gp_Vec();
        Dnv = new gp_Vec();
        GeomAbs_SurfaceType typS = Adaptor3d_HSurfaceTool.GetType(S);
        switch (typS)
        {
            case GeomAbs_Plane:
            {
                gp_Pln pl = Adaptor3d_HSurfaceTool.Plane(S);
                P = ElSLib.Value(U, V, pl);
                Norm = new gp_Vec(pl.Axis().Direction());
                if (!pl.Direct())
                {
                    Norm.Reverse();
                }
                Dnu = Dnv = new gp_Vec(0.0, 0.0, 0.0);
            }
            break;

            case GeomAbs_Sphere:
            {
                gp_Sphere sp = Adaptor3d_HSurfaceTool.Sphere(S);
                ElSLib.D1(U, V, sp, out P, out Dnu, out Dnv);
                Norm = new gp_Vec(sp.Location(), P);
                double Rad = sp.Radius();
                if (!sp.Direct())
                {
                    Rad = -Rad;
                }
                Norm.Divide(Rad);
                Dnu.Divide(Rad);
                Dnv.Divide(Rad);
            }
            break;

            case GeomAbs_Cylinder:
            {
                gp_Cylinder cy = Adaptor3d_HSurfaceTool.Cylinder(S);
                P = ElSLib.Value(U, V, cy);
                Norm.SetLinearForm(Math.Cos(U), new gp_Vec(cy.XAxis().Direction()), Math.Sin(U),
                                   new gp_Vec(cy.YAxis().Direction()));
                Dnu.SetLinearForm(-Math.Sin(U), new gp_Vec(cy.XAxis().Direction()), Math.Cos(U),
                                  new gp_Vec(cy.YAxis().Direction()));
                if (!cy.Direct())
                {
                    Norm.Reverse();
                    Dnu.Reverse();
                }
                Dnv = new gp_Vec(0.0, 0.0, 0.0);
            }
            break;

            case GeomAbs_Cone:
            {
                gp_Cone co = Adaptor3d_HSurfaceTool.Cone(S);
                P = ElSLib.Value(U, V, co);
                double Angle = co.SemiAngle();
                double Sina = Math.Sin(Angle);
                double Cosa = Math.Cos(Angle);
                double Rad = co.RefRadius();
                double Vcalc = V;
                if (Math.Abs(V * Sina + Rad) <= RealEpsilon())
                { // on est a l'apex
                    double Vfi = Adaptor3d_HSurfaceTool.FirstVParameter(S);
                    if (Vfi < -Rad / Sina)
                    { // partie valide pour V < Vapex
                        Vcalc = V - 1;
                    }
                    else
                    {
                        Vcalc = V + 1.0;
                    }
                }

                if (Rad + Vcalc * Sina < 0.0)
                {
                    Norm.SetLinearForm(Sina, new gp_Vec(co.Axis().Direction()), Cosa * Math.Cos(U),
                                       new gp_Vec(co.XAxis().Direction()), Cosa * Math.Sin(U),
                                       new gp_Vec(co.YAxis().Direction()));
                }
                else
                {
                    Norm.SetLinearForm(-Sina, new gp_Vec(co.Axis().Direction()), Cosa * Math.Cos(U),
                                       new gp_Vec(co.XAxis().Direction()), Cosa * Math.Sin(U),
                                       new gp_Vec(co.YAxis().Direction()));
                }
                Dnu.SetLinearForm(-Cosa * Math.Sin(U), new gp_Vec(co.XAxis().Direction()), Cosa * Math.Cos(U),
                                  new gp_Vec(co.YAxis().Direction()));
                if (!co.Direct())
                {
                    Norm.Reverse();
                    Dnu.Reverse();
                }
                Dnv = new gp_Vec(0.0, 0.0, 0.0);
            }
            break;

            default:
            {
                Adaptor3d_HSurfaceTool.D2(S, U, V, out P, out gp_Vec d1u, out gp_Vec d1v, out gp_Vec d2u,
                                          out gp_Vec d2v, out gp_Vec d2uv);
                Norm = d1u.Crossed(d1v);
                Dnu = d2u.Crossed(d1v) + d1u.Crossed(d2uv);
                Dnv = d2uv.Crossed(d1v) + d1u.Crossed(d2v);
            }
            break;
        }
    }
}
