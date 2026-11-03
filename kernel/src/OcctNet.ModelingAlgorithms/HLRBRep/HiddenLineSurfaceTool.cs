// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_SurfaceTool.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_SurfaceTool.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_SurfaceTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.HiddenLines;

/// <summary>The tool over HLRBRep's surfaces that the curve/surface intersection of the hiding uses.</summary>
/// <remarks>
/// A struct without state, so it can be a type argument of the generic intersections (decision 338). OCCT's surface
/// type is <c>HLRBRep_Surface*</c>; the port's type argument is the class itself (decision 1091). Some members go
/// to the surface (its type, bounds and quadrics), others straight to its adaptor (<c>Value</c>, the derivatives,
/// the intervals, the resolutions), as OCCT's. OCCT's ratios in the bounded <c>NbSamplesU</c> and <c>NbSamplesV</c>
/// divide by <c>first - last</c>, so they are negative and the counts above 10 end at 5; reproduced.
/// </remarks>
public readonly struct HiddenLineSurfaceTool : IIntersectionSurfaceTool<HlrSurface>
{
    public static double FirstUParameter(in HlrSurface surf) =>
        surf.FirstUParameter();

    public static double FirstVParameter(in HlrSurface surf) =>
        surf.FirstVParameter();

    public static double LastUParameter(in HlrSurface surf) =>
        surf.LastUParameter();

    public static double LastVParameter(in HlrSurface surf) =>
        surf.LastVParameter();

    public static int NbUIntervals(in HlrSurface surf, Continuity sh) =>
        surf.NbUIntervals(sh);

    public static int NbVIntervals(in HlrSurface surf, Continuity sh) =>
        surf.NbVIntervals(sh);

    public static void UIntervals(in HlrSurface surf, NCollection_Array1<double> tab, Continuity sh)
    {
        surf.Surface().UIntervals(tab, sh);
    }

    public static void VIntervals(in HlrSurface surf, NCollection_Array1<double> tab, Continuity sh)
    {
        surf.Surface().VIntervals(tab, sh);
    }

    public static SurfaceAdaptorBase UTrim(in HlrSurface surf, double first, double last, double tol) =>
        surf.Surface().UTrim(first, last, tol);

    public static SurfaceAdaptorBase VTrim(in HlrSurface surf, double first, double last, double tol) =>
        surf.Surface().VTrim(first, last, tol);

    public static bool IsUClosed(in HlrSurface surf) =>
        surf.IsUClosed;

    public static bool IsVClosed(in HlrSurface surf) =>
        surf.IsVClosed;

    public static bool IsUPeriodic(in HlrSurface surf) =>
        surf.IsUPeriodic;

    public static double UPeriod(in HlrSurface surf) =>
        surf.UPeriod();

    public static bool IsVPeriodic(in HlrSurface surf) =>
        surf.IsVPeriodic;

    public static double VPeriod(in HlrSurface surf) =>
        surf.VPeriod();

    public static Point3D Value(in HlrSurface surf, double u, double v) =>
        surf.Surface().Value(u, v);

    public static void D0(in HlrSurface surf, double u, double v, out Point3D p)
    {
        surf.Surface().D0(u, v, out p);
    }

    public static void D1(in HlrSurface surf, double u, double v, out Point3D p, out Vector3D d1U,
                          out Vector3D d1V)
    {
        surf.Surface().D1(u, v, out p, out d1U, out d1V);
    }

    public static void D2(in HlrSurface surf, double u, double v, out Point3D p, out Vector3D d1U,
                          out Vector3D d1V, out Vector3D d2U, out Vector3D d2V, out Vector3D d2UV)
    {
        surf.Surface().D2(u, v, out p, out d1U, out d1V, out d2U, out d2V, out d2UV);
    }

    public static void D3(in HlrSurface surf, double u, double v, out Point3D p, out Vector3D d1U,
                          out Vector3D d1V, out Vector3D d2U, out Vector3D d2V, out Vector3D d2UV, out Vector3D d3U,
                          out Vector3D d3V, out Vector3D d3UUV, out Vector3D d3UVV)
    {
        surf.Surface().D3(u, v, out p, out d1U, out d1V, out d2U, out d2V, out d2UV,
                             out d3U, out d3V, out d3UUV, out d3UVV);
    }

    public static Vector3D DN(in HlrSurface surf, double u, double v, int nu, int nv) =>
        surf.Surface().DN(u, v, nu, nv);

    public static double UResolution(in HlrSurface surf, double r3d) =>
        surf.Surface().UResolution(r3d);

    public static double VResolution(in HlrSurface surf, double r3d) =>
        surf.Surface().VResolution(r3d);

    public static SurfaceKind GetType(in HlrSurface surf) =>
        surf.GetType();

    public static Plane3D Plane(in HlrSurface surf) =>
        surf.Plane();

    public static Cylinder Cylinder(in HlrSurface surf) =>
        surf.Cylinder();

    public static Cone Cone(in HlrSurface surf) =>
        surf.Cone();

    public static Sphere Sphere(in HlrSurface surf) =>
        surf.Sphere();

    public static Torus Torus(in HlrSurface surf) =>
        surf.Torus();

    public static global::Kerf.Geometry.BezierSurface Bezier(in HlrSurface surf) =>
        surf.Surface().Bezier();

    public static global::Kerf.Geometry.BSplineSurface BSpline(in HlrSurface surf) =>
        surf.Surface().BSpline();

    public static Axis3D AxeOfRevolution(in HlrSurface surf) =>
        surf.Axis();

    public static Direction3D Direction(in HlrSurface surf) =>
        surf.Surface().Direction();

    public static CurveAdaptorBase BasisCurve(in HlrSurface surf) =>
        surf.Surface().BasisCurve();

    public static SurfaceAdaptorBase BasisSurface(in HlrSurface surf) =>
        surf.Surface().BasisSurface();

    public static double OffsetValue(in HlrSurface surf) =>
        surf.Surface().OffsetValue();

    /// <summary>The number of samples in U on the whole range.</summary>
    public static int NbSamplesU(in HlrSurface surf)
    {
        int nbs;
        SurfaceKind typS = surf.GetType();
        switch (typS)
        {
            case global::Kerf.Geometry.SurfaceKind.Plane:
                nbs = 2;
                break;
            case SurfaceKind.BezierSurface:
                nbs = 3 + surf.UPoleCount;
                break;
            case SurfaceKind.BSplineSurface:
                nbs = surf.UKnotCount;
                nbs *= surf.UDegree();
                if (nbs < 2)
                {
                    nbs = 2;
                }
                break;
            case global::Kerf.Geometry.SurfaceKind.Torus:
                nbs = 20;
                break;
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
            case global::Kerf.Geometry.SurfaceKind.Cone:
            case global::Kerf.Geometry.SurfaceKind.Sphere:
            case SurfaceOfRevolution:
            case SurfaceOfExtrusion:
                nbs = 10;
                break;
            default:
                nbs = 10;
                break;
        }
        return nbs;
    }

    /// <summary>The number of samples in V on the whole range.</summary>
    public static int NbSamplesV(in HlrSurface surf)
    {
        int nbs;
        SurfaceKind typS = surf.GetType();
        switch (typS)
        {
            case global::Kerf.Geometry.SurfaceKind.Plane:
                nbs = 2;
                break;
            case SurfaceKind.BezierSurface:
                nbs = 3 + surf.VPoleCount;
                break;
            case SurfaceKind.BSplineSurface:
                nbs = surf.VKnotCount;
                nbs *= surf.VDegree();
                if (nbs < 2)
                {
                    nbs = 2;
                }
                break;
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
            case global::Kerf.Geometry.SurfaceKind.Cone:
            case global::Kerf.Geometry.SurfaceKind.Sphere:
            case global::Kerf.Geometry.SurfaceKind.Torus:
            case SurfaceOfRevolution:
            case SurfaceOfExtrusion:
                nbs = 15;
                break;
            default:
                nbs = 10;
                break;
        }
        return nbs;
    }

    /// <summary>The number of samples in U on [theU1, theU2].</summary>
    public static int NbSamplesU(in HlrSurface surf, double u1, double u2)
    {
        int nbs = NbSamplesU(surf);
        int n = nbs;
        if (nbs > 10)
        {
            double uf = FirstUParameter(surf);
            double ul = LastUParameter(surf);
            n *= (int)((u2 - u1) / (uf - ul)); // OCCT
            if (n > nbs)
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

    /// <summary>The number of samples in V on [theV1, theV2].</summary>
    public static int NbSamplesV(in HlrSurface surf, double v1, double v2)
    {
        int nbs = NbSamplesV(surf);
        int n = nbs;
        if (nbs > 10)
        {
            double vf = FirstVParameter(surf);
            double vl = LastVParameter(surf);
            n *= (int)((v2 - v1) / (vf - vl)); // OCCT
            if (n > nbs)
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
