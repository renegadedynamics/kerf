// Port of: src/ModelingData/TKG3d/Adaptor3d/Adaptor3d_HSurfaceTool.hxx
// Port of: src/ModelingData/TKG3d/Adaptor3d/Adaptor3d_HSurfaceTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Geometry.Adaptors;

/// <summary>
/// Static access to a surface adaptor's services, as the generic algorithms (intersections, extrema) call them,
/// plus sampling counts and a G1 test.
/// </summary>
internal static class HSurfaceTool
{
    public static double FirstUParameter(SurfaceAdaptorBase surf) => surf.FirstUParameter();

    public static double FirstVParameter(SurfaceAdaptorBase surf) => surf.FirstVParameter();

    public static double LastUParameter(SurfaceAdaptorBase surf) => surf.LastUParameter();

    public static double LastVParameter(SurfaceAdaptorBase surf) => surf.LastVParameter();

    public static int NbUIntervals(SurfaceAdaptorBase surf, Continuity sh) => surf.NbUIntervals(sh);

    public static int NbVIntervals(SurfaceAdaptorBase surf, Continuity sh) => surf.NbVIntervals(sh);

    public static void UIntervals(SurfaceAdaptorBase surf, NCollection_Array1<double> tab, Continuity sh) =>
        surf.UIntervals(tab, sh);

    public static void VIntervals(SurfaceAdaptorBase surf, NCollection_Array1<double> tab, Continuity sh) =>
        surf.VIntervals(tab, sh);

    public static SurfaceAdaptorBase UTrim(SurfaceAdaptorBase surf, double first, double last, double tol) =>
        surf.UTrim(first, last, tol);

    public static SurfaceAdaptorBase VTrim(SurfaceAdaptorBase surf, double first, double last, double tol) =>
        surf.VTrim(first, last, tol);

    public static bool IsUClosed(SurfaceAdaptorBase surf) => surf.IsUClosed;

    public static bool IsVClosed(SurfaceAdaptorBase surf) => surf.IsVClosed;

    public static bool IsUPeriodic(SurfaceAdaptorBase surf) => surf.IsUPeriodic;

    public static double UPeriod(SurfaceAdaptorBase surf) => surf.UPeriod();

    public static bool IsVPeriodic(SurfaceAdaptorBase surf) => surf.IsVPeriodic;

    public static double VPeriod(SurfaceAdaptorBase surf) => surf.VPeriod();

    public static Point3D Value(SurfaceAdaptorBase surf, double u, double v) => surf.Value(u, v);

    public static void D0(SurfaceAdaptorBase surf, double u, double v, out Point3D pnt) =>
        surf.D0(u, v, out pnt);

    public static void D1(SurfaceAdaptorBase surf, double u, double v, out Point3D pnt, out Vector3D d1U,
                          out Vector3D d1V) =>
        surf.D1(u, v, out pnt, out d1U, out d1V);

    public static void D2(SurfaceAdaptorBase surf, double u, double v, out Point3D pnt, out Vector3D d1U,
                          out Vector3D d1V, out Vector3D d2U, out Vector3D d2V, out Vector3D d2UV) =>
        surf.D2(u, v, out pnt, out d1U, out d1V, out d2U, out d2V, out d2UV);

    public static void D3(SurfaceAdaptorBase surf, double u, double v, out Point3D pnt, out Vector3D d1U,
                          out Vector3D d1V, out Vector3D d2U, out Vector3D d2V, out Vector3D d2UV,
                          out Vector3D d3U, out Vector3D d3V, out Vector3D d3UUV, out Vector3D d3UVV) =>
        surf.D3(u,
                   v,
                   out pnt,
                   out d1U,
                   out d1V,
                   out d2U,
                   out d2V,
                   out d2UV,
                   out d3U,
                   out d3V,
                   out d3UUV,
                   out d3UVV);

    public static Vector3D DN(SurfaceAdaptorBase surf, double u, double v, int nu, int nv) =>
        surf.DN(u, v, nu, nv);

    public static double UResolution(SurfaceAdaptorBase surf, double r3d) => surf.UResolution(r3d);

    public static double VResolution(SurfaceAdaptorBase surf, double r3d) => surf.VResolution(r3d);

    public static SurfaceKind GetType(SurfaceAdaptorBase surf) => surf.GetType();

    public static Plane3D Plane(SurfaceAdaptorBase surf) => surf.Plane();

    public static Cylinder3D Cylinder(SurfaceAdaptorBase surf) => surf.Cylinder();

    public static Cone3D Cone(SurfaceAdaptorBase surf) => surf.Cone();

    public static Torus3D Torus(SurfaceAdaptorBase surf) => surf.Torus();

    public static Sphere3D Sphere(SurfaceAdaptorBase surf) => surf.Sphere();

    public static global::Kerf.Geometry.BezierSurface Bezier(SurfaceAdaptorBase surf) => surf.Bezier();

    public static global::Kerf.Geometry.BSplineSurface BSpline(SurfaceAdaptorBase surf) => surf.BSpline();

    public static Axis3D AxeOfRevolution(SurfaceAdaptorBase surf) => surf.AxeOfRevolution();

    public static Direction3D Direction(SurfaceAdaptorBase surf) => surf.Direction();

    public static CurveAdaptorBase BasisCurve(SurfaceAdaptorBase surf) => surf.BasisCurve();

    public static SurfaceAdaptorBase BasisSurface(SurfaceAdaptorBase surf) => surf.BasisSurface();

    public static double OffsetValue(SurfaceAdaptorBase surf) => surf.OffsetValue();

    /// <summary>
    /// True if the surface is G1 along U (theAlongU) or V: tested on the B-spline curve behind it (an iso-curve of a
    /// B-spline surface, the basis curve of a swept surface in its direction, through an offset surface or curve);
    /// false for any other surface.
    /// </summary>
    public static bool IsSurfG1(SurfaceAdaptorBase surf, bool alongU, double angTol = Precision_Angular)
    {
        double uf, ul, vf, vl;
        uf = surf.FirstUParameter();
        ul = surf.LastUParameter();
        vf = surf.FirstVParameter();
        vl = surf.LastVParameter();

        SurfaceAdaptorBase s = surf;
        CurveAdaptorBase? c = null;

        global::Kerf.Geometry.BSplineSurface bs;
        BSplineCurve3D? bc = null;

        if (s.GetType() == SurfaceKind.OffsetSurface)
        {
            s = s.BasisSurface();
        }

        if (s.GetType() == SurfaceOfRevolution || s.GetType() == SurfaceOfExtrusion)
        {
            c = s.BasisCurve();
        }

        if (c is not null)
        {
            if (c.GetType() == OffsetCurve)
            {
                OffsetCurve3D oc = c.OffsetCurve();
                c = new CurveAdaptor(oc.BasisCurve);
            }

            if (c.GetType() == BSplineCurve)
            {
                if ((alongU && s.GetType() == SurfaceOfExtrusion)
                    || (!alongU && s.GetType() == SurfaceOfRevolution))
                {
                    bc = c.BSpline();
                }
            }
        }

        if (s.GetType() == SurfaceKind.BSplineSurface)
        {
            bs = s.BSpline();

            if (alongU)
            {
                double isoPar = (vf + vl) / 2.0;
                bc = bs.VIso(isoPar) as BSplineCurve3D;
            }
            else
            {
                double isoPar = (uf + ul) / 2.0;
                bc = bs.UIso(isoPar) as BSplineCurve3D;
            }
        }

        if (bc is not null)
        {
            if (alongU)
            {
                return bc.IsG1(uf, ul, angTol);
            }
            else
            {
                return bc.IsG1(vf, vl, angTol);
            }
        }

        return false;
    }

    // Precision::Angular(), as a constant for the default argument (decision 129).
    private const double Precision_Angular = 1.0e-12;

    /// <summary>The number of U samples for the surface's type.</summary>
    public static int NbSamplesU(SurfaceAdaptorBase s)
    {
        switch (s.GetType())
        {
            case global::Kerf.Geometry.SurfaceKind.Plane:
                return 2;
            case SurfaceKind.BezierSurface:
                return (3 + s.UPoleCount);
            case SurfaceKind.BSplineSurface:
            {
                int nbs = s.UKnotCount * s.UDegree();
                return (nbs < 2 ? 2 : nbs);
            }
            case global::Kerf.Geometry.SurfaceKind.Torus:
                return 20;
            default:
                break;
        }
        return 10;
    }

    /// <summary>The number of V samples for the surface's type.</summary>
    public static int NbSamplesV(SurfaceAdaptorBase s)
    {
        switch (s.GetType())
        {
            case global::Kerf.Geometry.SurfaceKind.Plane:
                return 2;
            case SurfaceKind.BezierSurface:
                return (3 + s.VPoleCount);
            case SurfaceKind.BSplineSurface:
            {
                int nbs = s.VKnotCount * s.VDegree();
                return (nbs < 2 ? 2 : nbs);
            }
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
            case global::Kerf.Geometry.SurfaceKind.Cone:
            case global::Kerf.Geometry.SurfaceKind.Sphere:
            case global::Kerf.Geometry.SurfaceKind.Torus:
            case SurfaceOfRevolution:
            case SurfaceOfExtrusion:
                return 15;
            default:
                break;
        }
        return 10;
    }

    /// <summary>The number of U samples on [u1, u2], scaled from <see cref="NbSamplesU(SurfaceAdaptorBase)"/>.</summary>
    public static int NbSamplesU(SurfaceAdaptorBase s, double u1, double u2)
    {
        int nbs = NbSamplesU(s);
        int n = nbs;
        if (nbs > 10)
        {
            double uf = FirstUParameter(s);
            double ul = LastUParameter(s);
            n *= (int)((u2 - u1) / (ul - uf));
            if (n > nbs || n > 50)
            {
                n = nbs;
            }
            if (n < 5)
            {
                n = 5;
            }
        }
        return n;
    }

    /// <summary>The number of V samples on [v1, v2], scaled from <see cref="NbSamplesV(SurfaceAdaptorBase)"/>.</summary>
    public static int NbSamplesV(SurfaceAdaptorBase s, double v1, double v2)
    {
        int nbs = NbSamplesV(s);
        int n = nbs;
        if (nbs > 10)
        {
            double vf = FirstVParameter(s);
            double vl = LastVParameter(s);
            n *= (int)((v2 - v1) / (vl - vf));
            if (n > nbs || n > 50)
            {
                n = nbs;
            }
            if (n < 5)
            {
                n = 5;
            }
        }
        return n;
    }
}
