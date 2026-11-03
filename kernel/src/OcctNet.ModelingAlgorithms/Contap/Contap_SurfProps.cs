// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_SurfProps.hxx
// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_SurfProps.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Kerf.Geometry.SurfaceKind;

namespace Occt;

/// <summary>
/// The point, the (non-normalized) normal and its derivatives of a surface adaptor, in closed form on the elementary
/// surfaces and from the derivatives otherwise: what the contour functions evaluate.
/// </summary>
public static class Contap_SurfProps
{
    /// <summary>Computes the point <paramref name="P"/> and the normal <paramref name="Norm"/> of <paramref name="S"/> at (U, V).</summary>
    public static void Normale(SurfaceAdaptorBase S, double U, double V, out Point3D P, out Vector3D Norm)
    {
        Norm = new Vector3D();
        SurfaceKind typS = HSurfaceTool.GetType(S);
        switch (typS)
        {
            case Plane:
            {
                Plane3D pl = HSurfaceTool.Plane(S);
                Norm = new Vector3D(pl.Axis().Direction());
                P = global::Kerf.Engine.Geometry.ElSLib.Value(U, V, pl);
                if (!pl.Direct())
                {
                    Norm.Reverse();
                }
            }
            break;

            case SurfaceKind.Sphere:
            {
                Sphere sp = HSurfaceTool.Sphere(S);
                P = global::Kerf.Engine.Geometry.ElSLib.Value(U, V, sp);
                Norm = new Vector3D(sp.Location(), P);
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

            case SurfaceKind.Cylinder:
            {
                Cylinder cy = HSurfaceTool.Cylinder(S);
                P = global::Kerf.Engine.Geometry.ElSLib.Value(U, V, cy);
                Norm.SetLinearForm(Math.Cos(U), new Vector3D(cy.XAxis().Direction()), Math.Sin(U),
                                   new Vector3D(cy.YAxis().Direction()));
                if (!cy.Direct())
                {
                    Norm.Reverse();
                }
            }
            break;

            case SurfaceKind.Cone:
            {
                Cone co = HSurfaceTool.Cone(S);
                P = global::Kerf.Engine.Geometry.ElSLib.Value(U, V, co);
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
                    Norm.SetLinearForm(Sina, new Vector3D(co.Axis().Direction()), Cosa * Math.Cos(U),
                                       new Vector3D(co.XAxis().Direction()), Cosa * Math.Sin(U),
                                       new Vector3D(co.YAxis().Direction()));
                }
                else
                {
                    Norm.SetLinearForm(-Sina, new Vector3D(co.Axis().Direction()), Cosa * Math.Cos(U),
                                       new Vector3D(co.XAxis().Direction()), Cosa * Math.Sin(U),
                                       new Vector3D(co.YAxis().Direction()));
                }
                if (!co.Direct())
                {
                    Norm.Reverse();
                }
            }
            break;
            default:
            {
                HSurfaceTool.D1(S, U, V, out P, out Vector3D d1u, out Vector3D d1v);
                Norm = d1u.Crossed(d1v);
            }
            break;
        }
    }

    /// <summary>Computes the point, the first derivatives and the normal of <paramref name="S"/> at (U, V).</summary>
    public static void DerivAndNorm(SurfaceAdaptorBase S, double U, double V, out Point3D P, out Vector3D d1u,
                                    out Vector3D d1v, out Vector3D Norm)
    {
        Norm = new Vector3D();
        SurfaceKind typS = HSurfaceTool.GetType(S);
        switch (typS)
        {
            case Plane:
            {
                Plane3D pl = HSurfaceTool.Plane(S);
                Norm = new Vector3D(pl.Axis().Direction());
                global::Kerf.Engine.Geometry.ElSLib.D1(U, V, pl, out P, out d1u, out d1v);
                if (!pl.Direct())
                {
                    Norm.Reverse();
                }
            }
            break;

            case SurfaceKind.Sphere:
            {
                Sphere sp = HSurfaceTool.Sphere(S);
                global::Kerf.Engine.Geometry.ElSLib.D1(U, V, sp, out P, out d1u, out d1v);
                Norm = new Vector3D(sp.Location(), P);
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

            case SurfaceKind.Cylinder:
            {
                Cylinder cy = HSurfaceTool.Cylinder(S);
                global::Kerf.Engine.Geometry.ElSLib.D1(U, V, cy, out P, out d1u, out d1v);
                Norm.SetLinearForm(Math.Cos(U), new Vector3D(cy.XAxis().Direction()), Math.Sin(U),
                                   new Vector3D(cy.YAxis().Direction()));
                if (!cy.Direct())
                {
                    Norm.Reverse();
                }
            }
            break;

            case SurfaceKind.Cone:
            {
                Cone co = HSurfaceTool.Cone(S);
                global::Kerf.Engine.Geometry.ElSLib.D1(U, V, co, out P, out d1u, out d1v);
                double Angle = co.SemiAngle();
                double Sina = Math.Sin(Angle);
                double Cosa = Math.Cos(Angle);
                double Rad = co.RefRadius();

                double Vcalc = V;
                if (Math.Abs(V * Sina + Rad) <= RealEpsilon())
                { // on est a l'apex
                    double Vfi = HSurfaceTool.FirstVParameter(S);
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
                    Norm.SetLinearForm(Sina, new Vector3D(co.Axis().Direction()), Cosa * Math.Cos(U),
                                       new Vector3D(co.XAxis().Direction()), Cosa * Math.Sin(U),
                                       new Vector3D(co.YAxis().Direction()));
                }
                else
                {
                    Norm.SetLinearForm(-Sina, new Vector3D(co.Axis().Direction()), Cosa * Math.Cos(U),
                                       new Vector3D(co.XAxis().Direction()), Cosa * Math.Sin(U),
                                       new Vector3D(co.YAxis().Direction()));
                }
                if (!co.Direct())
                {
                    Norm.Reverse();
                }
            }
            break;
            default:
            {
                HSurfaceTool.D1(S, U, V, out P, out d1u, out d1v);
                Norm = d1u.Crossed(d1v);
            }
            break;
        }
    }

    /// <summary>Computes the point, the normal and its derivatives of <paramref name="S"/> at (U, V).</summary>
    public static void NormAndDn(SurfaceAdaptorBase S, double U, double V, out Point3D P, out Vector3D Norm,
                                 out Vector3D Dnu, out Vector3D Dnv)
    {
        Norm = new Vector3D();
        Dnu = new Vector3D();
        Dnv = new Vector3D();
        SurfaceKind typS = HSurfaceTool.GetType(S);
        switch (typS)
        {
            case Plane:
            {
                Plane3D pl = HSurfaceTool.Plane(S);
                P = global::Kerf.Engine.Geometry.ElSLib.Value(U, V, pl);
                Norm = new Vector3D(pl.Axis().Direction());
                if (!pl.Direct())
                {
                    Norm.Reverse();
                }
                Dnu = Dnv = new Vector3D(0.0, 0.0, 0.0);
            }
            break;

            case SurfaceKind.Sphere:
            {
                Sphere sp = HSurfaceTool.Sphere(S);
                global::Kerf.Engine.Geometry.ElSLib.D1(U, V, sp, out P, out Dnu, out Dnv);
                Norm = new Vector3D(sp.Location(), P);
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

            case SurfaceKind.Cylinder:
            {
                Cylinder cy = HSurfaceTool.Cylinder(S);
                P = global::Kerf.Engine.Geometry.ElSLib.Value(U, V, cy);
                Norm.SetLinearForm(Math.Cos(U), new Vector3D(cy.XAxis().Direction()), Math.Sin(U),
                                   new Vector3D(cy.YAxis().Direction()));
                Dnu.SetLinearForm(-Math.Sin(U), new Vector3D(cy.XAxis().Direction()), Math.Cos(U),
                                  new Vector3D(cy.YAxis().Direction()));
                if (!cy.Direct())
                {
                    Norm.Reverse();
                    Dnu.Reverse();
                }
                Dnv = new Vector3D(0.0, 0.0, 0.0);
            }
            break;

            case SurfaceKind.Cone:
            {
                Cone co = HSurfaceTool.Cone(S);
                P = global::Kerf.Engine.Geometry.ElSLib.Value(U, V, co);
                double Angle = co.SemiAngle();
                double Sina = Math.Sin(Angle);
                double Cosa = Math.Cos(Angle);
                double Rad = co.RefRadius();
                double Vcalc = V;
                if (Math.Abs(V * Sina + Rad) <= RealEpsilon())
                { // on est a l'apex
                    double Vfi = HSurfaceTool.FirstVParameter(S);
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
                    Norm.SetLinearForm(Sina, new Vector3D(co.Axis().Direction()), Cosa * Math.Cos(U),
                                       new Vector3D(co.XAxis().Direction()), Cosa * Math.Sin(U),
                                       new Vector3D(co.YAxis().Direction()));
                }
                else
                {
                    Norm.SetLinearForm(-Sina, new Vector3D(co.Axis().Direction()), Cosa * Math.Cos(U),
                                       new Vector3D(co.XAxis().Direction()), Cosa * Math.Sin(U),
                                       new Vector3D(co.YAxis().Direction()));
                }
                Dnu.SetLinearForm(-Cosa * Math.Sin(U), new Vector3D(co.XAxis().Direction()), Cosa * Math.Cos(U),
                                  new Vector3D(co.YAxis().Direction()));
                if (!co.Direct())
                {
                    Norm.Reverse();
                    Dnu.Reverse();
                }
                Dnv = new Vector3D(0.0, 0.0, 0.0);
            }
            break;

            default:
            {
                HSurfaceTool.D2(S, U, V, out P, out Vector3D d1u, out Vector3D d1v, out Vector3D d2u,
                                          out Vector3D d2v, out Vector3D d2uv);
                Norm = d1u.Crossed(d1v);
                Dnu = d2u.Crossed(d1v) + d1u.Crossed(d2uv);
                Dnv = d2uv.Crossed(d1v) + d1u.Crossed(d2v);
            }
            break;
        }
    }
}
