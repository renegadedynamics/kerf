// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Approximation;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Intersection.CurveSurface;

/// <summary>
/// Adaptor3d_HSurfaceTool as the tool of the generic curve/surface algorithms
/// (<see cref="IIntersectionSurfaceTool{TheSurface}"/>, and through it IntImp's surface tools): every member
/// forwards to <see cref="HSurfaceTool"/>.
/// </summary>
/// <remarks>
/// Not an OCCT class. OCCT passes Adaptor3d_HSurfaceTool itself as the template argument; the port's
/// Adaptor3d_HSurfaceTool is a static class of Kerf.Kernel.ModelingData, which can't implement the interfaces of this
/// assembly, so this struct stands in for it (decision 338). IntPatch and the other instantiations over
/// Adaptor3d_HSurfaceTool use it too, and so do GeomInt's instantiations of ApproxInt's generics
/// (<see cref="IApproxSurfaceTool{ThePSurface}"/>).
/// </remarks>
internal readonly struct TheHSurfaceTool
    : IIntersectionSurfaceTool<SurfaceAdaptorBase>, IApproxSurfaceTool<SurfaceAdaptorBase>
{
    public static double FirstUParameter(in SurfaceAdaptorBase s) => HSurfaceTool.FirstUParameter(s);

    public static double FirstVParameter(in SurfaceAdaptorBase s) => HSurfaceTool.FirstVParameter(s);

    public static double LastUParameter(in SurfaceAdaptorBase s) => HSurfaceTool.LastUParameter(s);

    public static double LastVParameter(in SurfaceAdaptorBase s) => HSurfaceTool.LastVParameter(s);

    public static int NbUIntervals(in SurfaceAdaptorBase s, Continuity sh) => HSurfaceTool.NbUIntervals(s, sh);

    public static int NbVIntervals(in SurfaceAdaptorBase s, Continuity sh) => HSurfaceTool.NbVIntervals(s, sh);

    public static void UIntervals(in SurfaceAdaptorBase s, NCollection_Array1<double> t, Continuity sh) =>
        HSurfaceTool.UIntervals(s, t, sh);

    public static void VIntervals(in SurfaceAdaptorBase s, NCollection_Array1<double> t, Continuity sh) =>
        HSurfaceTool.VIntervals(s, t, sh);

    public static SurfaceAdaptorBase UTrim(in SurfaceAdaptorBase s, double first, double last, double tol) =>
        HSurfaceTool.UTrim(s, first, last, tol);

    public static SurfaceAdaptorBase VTrim(in SurfaceAdaptorBase s, double first, double last, double tol) =>
        HSurfaceTool.VTrim(s, first, last, tol);

    public static bool IsUClosed(in SurfaceAdaptorBase s) => HSurfaceTool.IsUClosed(s);

    public static bool IsVClosed(in SurfaceAdaptorBase s) => HSurfaceTool.IsVClosed(s);

    public static bool IsUPeriodic(in SurfaceAdaptorBase s) => HSurfaceTool.IsUPeriodic(s);

    public static double UPeriod(in SurfaceAdaptorBase s) => HSurfaceTool.UPeriod(s);

    public static bool IsVPeriodic(in SurfaceAdaptorBase s) => HSurfaceTool.IsVPeriodic(s);

    public static double VPeriod(in SurfaceAdaptorBase s) => HSurfaceTool.VPeriod(s);

    public static Point3D Value(in SurfaceAdaptorBase s, double u, double v) => HSurfaceTool.Value(s, u, v);

    public static void D0(in SurfaceAdaptorBase s, double u, double v, out Point3D p) =>
        HSurfaceTool.D0(s, u, v, out p);

    public static void D1(in SurfaceAdaptorBase s, double u, double v, out Point3D p, out Vector3D d1U, out Vector3D d1V) =>
        HSurfaceTool.D1(s, u, v, out p, out d1U, out d1V);

    public static double UResolution(in SurfaceAdaptorBase s, double r3d) => HSurfaceTool.UResolution(s, r3d);

    public static double VResolution(in SurfaceAdaptorBase s, double r3d) => HSurfaceTool.VResolution(s, r3d);

    public static SurfaceKind GetType(in SurfaceAdaptorBase s) => HSurfaceTool.GetType(s);

    public static Plane3D Plane(in SurfaceAdaptorBase s) => HSurfaceTool.Plane(s);

    public static Cylinder Cylinder(in SurfaceAdaptorBase s) => HSurfaceTool.Cylinder(s);

    public static Cone Cone(in SurfaceAdaptorBase s) => HSurfaceTool.Cone(s);

    public static Torus Torus(in SurfaceAdaptorBase s) => HSurfaceTool.Torus(s);

    public static Sphere Sphere(in SurfaceAdaptorBase s) => HSurfaceTool.Sphere(s);

    public static Axis3D AxeOfRevolution(in SurfaceAdaptorBase s) => HSurfaceTool.AxeOfRevolution(s);

    public static Direction3D Direction(in SurfaceAdaptorBase s) => HSurfaceTool.Direction(s);

    public static CurveAdaptorBase BasisCurve(in SurfaceAdaptorBase s) => HSurfaceTool.BasisCurve(s);

    public static SurfaceAdaptorBase BasisSurface(in SurfaceAdaptorBase s) => HSurfaceTool.BasisSurface(s);

    public static double OffsetValue(in SurfaceAdaptorBase s) => HSurfaceTool.OffsetValue(s);

    public static int NbSamplesU(in SurfaceAdaptorBase s, double u1, double u2) =>
        HSurfaceTool.NbSamplesU(s, u1, u2);

    public static int NbSamplesV(in SurfaceAdaptorBase s, double v1, double v2) =>
        HSurfaceTool.NbSamplesV(s, v1, v2);
}
