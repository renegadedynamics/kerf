// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_SurfProps.hxx
// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_SurfProps.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.HiddenLines.Contours;

/// <summary>
/// The point, the (non-normalized) normal and its derivatives of a surface adaptor, in closed form on the elementary
/// surfaces and from the derivatives otherwise: what the contour functions evaluate.
/// </summary>
public static class SurfProps
{
    /// <summary>Computes the point <paramref name="p"/> and the normal <paramref name="norm"/> of <paramref name="s"/> at (U, V).</summary>
    public static void Normale(SurfaceAdaptorBase s, double u, double v, out Point3D p, out Vector3D norm)
    {
        norm = new Vector3D();
        SurfaceKind typS = HSurfaceTool.GetType(s);
        switch (typS)
        {
            case Plane:
            {
                Plane3D pl = HSurfaceTool.Plane(s);
                norm = new Vector3D(pl.Axis().Direction());
                p = global::Kerf.Engine.Geometry.ElSLib.Value(u, v, pl);
                if (!pl.Direct())
                {
                    norm = norm.Reversed();
                }
            }
            break;

            case SurfaceKind.Sphere:
            {
                Sphere sp = HSurfaceTool.Sphere(s);
                p = global::Kerf.Engine.Geometry.ElSLib.Value(u, v, sp);
                norm = new Vector3D(sp.Location(), p);
                if (sp.Direct())
                {
                    norm = norm.Divided(sp.Radius());
                }
                else
                {
                    norm = norm.Divided(-sp.Radius());
                }
            }
            break;

            case SurfaceKind.Cylinder:
            {
                Cylinder cy = HSurfaceTool.Cylinder(s);
                p = global::Kerf.Engine.Geometry.ElSLib.Value(u, v, cy);
                norm = Vector3D.LinearForm(Math.Cos(u), new Vector3D(cy.XAxis().Direction()), Math.Sin(u),
                                   new Vector3D(cy.YAxis().Direction()));
                if (!cy.Direct())
                {
                    norm = norm.Reversed();
                }
            }
            break;

            case SurfaceKind.Cone:
            {
                Cone co = HSurfaceTool.Cone(s);
                p = global::Kerf.Engine.Geometry.ElSLib.Value(u, v, co);
                double angle = co.SemiAngle();
                double sina = Math.Sin(angle);
                double cosa = Math.Cos(angle);
                double rad = co.RefRadius();

                double vcalc = v;
                if (Math.Abs(v * sina + rad) <= 1e-12)
                { // on est a l'apex
                    norm = norm.WithCoord(0, 0, 0);
                    return;
                }

                if (rad + vcalc * sina < 0.0)
                {
                    norm = Vector3D.LinearForm(sina, new Vector3D(co.Axis().Direction()), cosa * Math.Cos(u),
                                       new Vector3D(co.XAxis().Direction()), cosa * Math.Sin(u),
                                       new Vector3D(co.YAxis().Direction()));
                }
                else
                {
                    norm = Vector3D.LinearForm(-sina, new Vector3D(co.Axis().Direction()), cosa * Math.Cos(u),
                                       new Vector3D(co.XAxis().Direction()), cosa * Math.Sin(u),
                                       new Vector3D(co.YAxis().Direction()));
                }
                if (!co.Direct())
                {
                    norm = norm.Reversed();
                }
            }
            break;
            default:
            {
                HSurfaceTool.D1(s, u, v, out p, out Vector3D d1u, out Vector3D d1v);
                norm = d1u.Crossed(d1v);
            }
            break;
        }
    }

    /// <summary>Computes the point, the first derivatives and the normal of <paramref name="s"/> at (U, V).</summary>
    public static void DerivAndNorm(SurfaceAdaptorBase s, double u, double v, out Point3D p, out Vector3D d1u,
                                    out Vector3D d1v, out Vector3D norm)
    {
        norm = new Vector3D();
        SurfaceKind typS = HSurfaceTool.GetType(s);
        switch (typS)
        {
            case Plane:
            {
                Plane3D pl = HSurfaceTool.Plane(s);
                norm = new Vector3D(pl.Axis().Direction());
                global::Kerf.Engine.Geometry.ElSLib.D1(u, v, pl, out p, out d1u, out d1v);
                if (!pl.Direct())
                {
                    norm = norm.Reversed();
                }
            }
            break;

            case SurfaceKind.Sphere:
            {
                Sphere sp = HSurfaceTool.Sphere(s);
                global::Kerf.Engine.Geometry.ElSLib.D1(u, v, sp, out p, out d1u, out d1v);
                norm = new Vector3D(sp.Location(), p);
                if (sp.Direct())
                {
                    norm = norm.Divided(sp.Radius());
                }
                else
                {
                    norm = norm.Divided(-sp.Radius());
                }
            }
            break;

            case SurfaceKind.Cylinder:
            {
                Cylinder cy = HSurfaceTool.Cylinder(s);
                global::Kerf.Engine.Geometry.ElSLib.D1(u, v, cy, out p, out d1u, out d1v);
                norm = Vector3D.LinearForm(Math.Cos(u), new Vector3D(cy.XAxis().Direction()), Math.Sin(u),
                                   new Vector3D(cy.YAxis().Direction()));
                if (!cy.Direct())
                {
                    norm = norm.Reversed();
                }
            }
            break;

            case SurfaceKind.Cone:
            {
                Cone co = HSurfaceTool.Cone(s);
                global::Kerf.Engine.Geometry.ElSLib.D1(u, v, co, out p, out d1u, out d1v);
                double angle = co.SemiAngle();
                double sina = Math.Sin(angle);
                double cosa = Math.Cos(angle);
                double rad = co.RefRadius();

                double vcalc = v;
                if (Math.Abs(v * sina + rad) <= RealEpsilon())
                { // on est a l'apex
                    double vfi = HSurfaceTool.FirstVParameter(s);
                    if (vfi < -rad / sina)
                    { // partie valide pour V < Vapex
                        vcalc = v - 1;
                    }
                    else
                    {
                        vcalc = v + 1.0;
                    }
                }

                if (rad + vcalc * sina < 0.0)
                {
                    norm = Vector3D.LinearForm(sina, new Vector3D(co.Axis().Direction()), cosa * Math.Cos(u),
                                       new Vector3D(co.XAxis().Direction()), cosa * Math.Sin(u),
                                       new Vector3D(co.YAxis().Direction()));
                }
                else
                {
                    norm = Vector3D.LinearForm(-sina, new Vector3D(co.Axis().Direction()), cosa * Math.Cos(u),
                                       new Vector3D(co.XAxis().Direction()), cosa * Math.Sin(u),
                                       new Vector3D(co.YAxis().Direction()));
                }
                if (!co.Direct())
                {
                    norm = norm.Reversed();
                }
            }
            break;
            default:
            {
                HSurfaceTool.D1(s, u, v, out p, out d1u, out d1v);
                norm = d1u.Crossed(d1v);
            }
            break;
        }
    }

    /// <summary>Computes the point, the normal and its derivatives of <paramref name="s"/> at (U, V).</summary>
    public static void NormAndDn(SurfaceAdaptorBase s, double u, double v, out Point3D p, out Vector3D norm,
                                 out Vector3D dnu, out Vector3D dnv)
    {
        norm = new Vector3D();
        dnu = new Vector3D();
        dnv = new Vector3D();
        SurfaceKind typS = HSurfaceTool.GetType(s);
        switch (typS)
        {
            case Plane:
            {
                Plane3D pl = HSurfaceTool.Plane(s);
                p = global::Kerf.Engine.Geometry.ElSLib.Value(u, v, pl);
                norm = new Vector3D(pl.Axis().Direction());
                if (!pl.Direct())
                {
                    norm = norm.Reversed();
                }
                dnu = dnv = new Vector3D(0.0, 0.0, 0.0);
            }
            break;

            case SurfaceKind.Sphere:
            {
                Sphere sp = HSurfaceTool.Sphere(s);
                global::Kerf.Engine.Geometry.ElSLib.D1(u, v, sp, out p, out dnu, out dnv);
                norm = new Vector3D(sp.Location(), p);
                double rad = sp.Radius();
                if (!sp.Direct())
                {
                    rad = -rad;
                }
                norm = norm.Divided(rad);
                dnu = dnu.Divided(rad);
                dnv = dnv.Divided(rad);
            }
            break;

            case SurfaceKind.Cylinder:
            {
                Cylinder cy = HSurfaceTool.Cylinder(s);
                p = global::Kerf.Engine.Geometry.ElSLib.Value(u, v, cy);
                norm = Vector3D.LinearForm(Math.Cos(u), new Vector3D(cy.XAxis().Direction()), Math.Sin(u),
                                   new Vector3D(cy.YAxis().Direction()));
                dnu = Vector3D.LinearForm(-Math.Sin(u), new Vector3D(cy.XAxis().Direction()), Math.Cos(u),
                                  new Vector3D(cy.YAxis().Direction()));
                if (!cy.Direct())
                {
                    norm = norm.Reversed();
                    dnu = dnu.Reversed();
                }
                dnv = new Vector3D(0.0, 0.0, 0.0);
            }
            break;

            case SurfaceKind.Cone:
            {
                Cone co = HSurfaceTool.Cone(s);
                p = global::Kerf.Engine.Geometry.ElSLib.Value(u, v, co);
                double angle = co.SemiAngle();
                double sina = Math.Sin(angle);
                double cosa = Math.Cos(angle);
                double rad = co.RefRadius();
                double vcalc = v;
                if (Math.Abs(v * sina + rad) <= RealEpsilon())
                { // on est a l'apex
                    double vfi = HSurfaceTool.FirstVParameter(s);
                    if (vfi < -rad / sina)
                    { // partie valide pour V < Vapex
                        vcalc = v - 1;
                    }
                    else
                    {
                        vcalc = v + 1.0;
                    }
                }

                if (rad + vcalc * sina < 0.0)
                {
                    norm = Vector3D.LinearForm(sina, new Vector3D(co.Axis().Direction()), cosa * Math.Cos(u),
                                       new Vector3D(co.XAxis().Direction()), cosa * Math.Sin(u),
                                       new Vector3D(co.YAxis().Direction()));
                }
                else
                {
                    norm = Vector3D.LinearForm(-sina, new Vector3D(co.Axis().Direction()), cosa * Math.Cos(u),
                                       new Vector3D(co.XAxis().Direction()), cosa * Math.Sin(u),
                                       new Vector3D(co.YAxis().Direction()));
                }
                dnu = Vector3D.LinearForm(-cosa * Math.Sin(u), new Vector3D(co.XAxis().Direction()), cosa * Math.Cos(u),
                                  new Vector3D(co.YAxis().Direction()));
                if (!co.Direct())
                {
                    norm = norm.Reversed();
                    dnu = dnu.Reversed();
                }
                dnv = new Vector3D(0.0, 0.0, 0.0);
            }
            break;

            default:
            {
                HSurfaceTool.D2(s, u, v, out p, out Vector3D d1u, out Vector3D d1v, out Vector3D d2u,
                                          out Vector3D d2v, out Vector3D d2uv);
                norm = d1u.Crossed(d1v);
                dnu = d2u.Crossed(d1v) + d1u.Crossed(d2uv);
                dnv = d2uv.Crossed(d1v) + d1u.Crossed(d2v);
            }
            break;
        }
    }
}
