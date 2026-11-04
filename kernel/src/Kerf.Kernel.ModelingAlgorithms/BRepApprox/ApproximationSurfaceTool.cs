// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_SurfaceTool.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_SurfaceTool.lxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_SurfaceTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Intersection.Approximation;

/// <summary>The static tool over a <see cref="FaceSurfaceAdaptor"/>: every member forwards to the adaptor.</summary>
/// <remarks>
/// OCCT's class of static functions is an empty struct implementing the static members of
/// <see cref="IApproxSurfaceTool{ThePSurface}"/>, so that it can be the ThePSurfaceTool type argument of
/// ApproxInt's and IntImp's generics (decision 607).
/// </remarks>
internal readonly struct ApproximationSurfaceTool : IApproxSurfaceTool<FaceSurfaceAdaptor>
{
    public static double FirstUParameter(in FaceSurfaceAdaptor surf) => surf.FirstUParameter();

    public static double FirstVParameter(in FaceSurfaceAdaptor surf) => surf.FirstVParameter();

    public static double LastUParameter(in FaceSurfaceAdaptor surf) => surf.LastUParameter();

    public static double LastVParameter(in FaceSurfaceAdaptor surf) => surf.LastVParameter();

    public static int NbUIntervals(in FaceSurfaceAdaptor surf, Continuity s) => surf.NbUIntervals(s);

    public static int NbVIntervals(in FaceSurfaceAdaptor surf, Continuity s) => surf.NbVIntervals(s);

    public static void UIntervals(in FaceSurfaceAdaptor surf, NCollection_Array1<double> tab, Continuity s)
    {
        surf.UIntervals(tab, s);
    }

    public static void VIntervals(in FaceSurfaceAdaptor surf, NCollection_Array1<double> tab, Continuity s)
    {
        surf.VIntervals(tab, s);
    }

    public static SurfaceAdaptorBase UTrim(in FaceSurfaceAdaptor surf, double f, double l, double tol) =>
        surf.UTrim(f, l, tol);

    public static SurfaceAdaptorBase VTrim(in FaceSurfaceAdaptor surf, double f, double l, double tol) =>
        surf.VTrim(f, l, tol);

    public static bool IsUClosed(in FaceSurfaceAdaptor s) => s.IsUClosed;

    public static bool IsVClosed(in FaceSurfaceAdaptor s) => s.IsVClosed;

    public static bool IsUPeriodic(in FaceSurfaceAdaptor s) => s.IsUPeriodic;

    public static double UPeriod(in FaceSurfaceAdaptor s) => s.UPeriod();

    public static bool IsVPeriodic(in FaceSurfaceAdaptor s) => s.IsVPeriodic;

    public static double VPeriod(in FaceSurfaceAdaptor s) => s.VPeriod();

    public static Point3D Value(in FaceSurfaceAdaptor s, double u, double v) => s.Value(u, v);

    public static void D0(in FaceSurfaceAdaptor s, double u, double v, out Point3D p)
    {
        s.D0(u, v, out p);
    }

    public static void D1(in FaceSurfaceAdaptor s, double u, double v, out Point3D p, out Vector3D d1U, out Vector3D d1V)
    {
        s.D1(u, v, out p, out d1U, out d1V);
    }

    public static void D2(in FaceSurfaceAdaptor s, double u, double v, out Point3D p, out Vector3D d1U, out Vector3D d1V,
                          out Vector3D d2U, out Vector3D d2V, out Vector3D d2UV)
    {
        s.D2(u, v, out p, out d1U, out d1V, out d2U, out d2V, out d2UV);
    }

    public static void D3(in FaceSurfaceAdaptor s, double u, double v, out Point3D p, out Vector3D d1U, out Vector3D d1V,
                          out Vector3D d2U, out Vector3D d2V, out Vector3D d2UV, out Vector3D d3U, out Vector3D d3V,
                          out Vector3D d3UUV, out Vector3D d3UVV)
    {
        s.D3(u, v, out p, out d1U, out d1V, out d2U, out d2V, out d2UV, out d3U, out d3V, out d3UUV, out d3UVV);
    }

    public static Vector3D DN(in FaceSurfaceAdaptor s, double u, double v, int nu, int nv) => s.DN(u, v, nu, nv);

    public static double UResolution(in FaceSurfaceAdaptor s, double r3d) => s.UResolution(r3d);

    public static double VResolution(in FaceSurfaceAdaptor s, double r3d) => s.VResolution(r3d);

    public static SurfaceKind GetType(in FaceSurfaceAdaptor s) => s.GetType();

    public static Plane3D Plane(in FaceSurfaceAdaptor s) => s.Plane();

    public static Cylinder Cylinder(in FaceSurfaceAdaptor s) => s.Cylinder();

    public static Cone Cone(in FaceSurfaceAdaptor s) => s.Cone();

    public static Torus Torus(in FaceSurfaceAdaptor s) => s.Torus();

    public static Sphere Sphere(in FaceSurfaceAdaptor s) => s.Sphere();

    public static BezierSurface Bezier(in FaceSurfaceAdaptor s) => (s.Bezier());

    public static BSplineSurface BSpline(in FaceSurfaceAdaptor s) => (s.BSpline());

    public static Axis3D AxeOfRevolution(in FaceSurfaceAdaptor s) => (s.AxeOfRevolution());

    public static Direction3D Direction(in FaceSurfaceAdaptor s) => (s.Direction());

    public static CurveAdaptorBase BasisCurve(in FaceSurfaceAdaptor s) => (s.BasisCurve());

    public static int NbSamplesU(in FaceSurfaceAdaptor s) => (10);

    public static int NbSamplesV(in FaceSurfaceAdaptor s) => (10);

    public static int NbSamplesU(in FaceSurfaceAdaptor s, double u1, double u2) => (10);

    public static int NbSamplesV(in FaceSurfaceAdaptor s, double v1, double v2) => (10);
}
