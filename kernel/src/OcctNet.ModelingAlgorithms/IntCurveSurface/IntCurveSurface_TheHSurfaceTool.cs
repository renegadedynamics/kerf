// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Adaptor3d_HSurfaceTool as the tool of the generic curve/surface algorithms
/// (<see cref="IIntCurveSurface_SurfaceTool{TheSurface}"/>, and through it IntImp's surface tools): every member
/// forwards to <see cref="HSurfaceTool"/>.
/// </summary>
/// <remarks>
/// Not an OCCT class. OCCT passes Adaptor3d_HSurfaceTool itself as the template argument; the port's
/// Adaptor3d_HSurfaceTool is a static class of OcctNet.ModelingData, which can't implement the interfaces of this
/// assembly, so this struct stands in for it (decision 338). IntPatch and the other instantiations over
/// Adaptor3d_HSurfaceTool use it too, and so do GeomInt's instantiations of ApproxInt's generics
/// (<see cref="IApproxInt_SurfaceTool{ThePSurface}"/>).
/// </remarks>
public readonly struct IntCurveSurface_TheHSurfaceTool
    : IIntCurveSurface_SurfaceTool<SurfaceAdaptorBase>, IApproxInt_SurfaceTool<SurfaceAdaptorBase>
{
    public static double FirstUParameter(in SurfaceAdaptorBase S) => HSurfaceTool.FirstUParameter(S);

    public static double FirstVParameter(in SurfaceAdaptorBase S) => HSurfaceTool.FirstVParameter(S);

    public static double LastUParameter(in SurfaceAdaptorBase S) => HSurfaceTool.LastUParameter(S);

    public static double LastVParameter(in SurfaceAdaptorBase S) => HSurfaceTool.LastVParameter(S);

    public static int NbUIntervals(in SurfaceAdaptorBase S, Continuity Sh) => HSurfaceTool.NbUIntervals(S, Sh);

    public static int NbVIntervals(in SurfaceAdaptorBase S, Continuity Sh) => HSurfaceTool.NbVIntervals(S, Sh);

    public static void UIntervals(in SurfaceAdaptorBase S, NCollection_Array1<double> T, Continuity Sh) =>
        HSurfaceTool.UIntervals(S, T, Sh);

    public static void VIntervals(in SurfaceAdaptorBase S, NCollection_Array1<double> T, Continuity Sh) =>
        HSurfaceTool.VIntervals(S, T, Sh);

    public static SurfaceAdaptorBase UTrim(in SurfaceAdaptorBase S, double First, double Last, double Tol) =>
        HSurfaceTool.UTrim(S, First, Last, Tol);

    public static SurfaceAdaptorBase VTrim(in SurfaceAdaptorBase S, double First, double Last, double Tol) =>
        HSurfaceTool.VTrim(S, First, Last, Tol);

    public static bool IsUClosed(in SurfaceAdaptorBase S) => HSurfaceTool.IsUClosed(S);

    public static bool IsVClosed(in SurfaceAdaptorBase S) => HSurfaceTool.IsVClosed(S);

    public static bool IsUPeriodic(in SurfaceAdaptorBase S) => HSurfaceTool.IsUPeriodic(S);

    public static double UPeriod(in SurfaceAdaptorBase S) => HSurfaceTool.UPeriod(S);

    public static bool IsVPeriodic(in SurfaceAdaptorBase S) => HSurfaceTool.IsVPeriodic(S);

    public static double VPeriod(in SurfaceAdaptorBase S) => HSurfaceTool.VPeriod(S);

    public static Point3D Value(in SurfaceAdaptorBase S, double U, double V) => HSurfaceTool.Value(S, U, V);

    public static void D0(in SurfaceAdaptorBase S, double U, double V, out Point3D P) =>
        HSurfaceTool.D0(S, U, V, out P);

    public static void D1(in SurfaceAdaptorBase S, double U, double V, out Point3D P, out Vector3D D1U, out Vector3D D1V) =>
        HSurfaceTool.D1(S, U, V, out P, out D1U, out D1V);

    public static double UResolution(in SurfaceAdaptorBase S, double R3d) => HSurfaceTool.UResolution(S, R3d);

    public static double VResolution(in SurfaceAdaptorBase S, double R3d) => HSurfaceTool.VResolution(S, R3d);

    public static SurfaceKind GetType(in SurfaceAdaptorBase S) => HSurfaceTool.GetType(S);

    public static Plane3D Plane(in SurfaceAdaptorBase S) => HSurfaceTool.Plane(S);

    public static Cylinder Cylinder(in SurfaceAdaptorBase S) => HSurfaceTool.Cylinder(S);

    public static Cone Cone(in SurfaceAdaptorBase S) => HSurfaceTool.Cone(S);

    public static Torus Torus(in SurfaceAdaptorBase S) => HSurfaceTool.Torus(S);

    public static Sphere Sphere(in SurfaceAdaptorBase S) => HSurfaceTool.Sphere(S);

    public static Axis3D AxeOfRevolution(in SurfaceAdaptorBase S) => HSurfaceTool.AxeOfRevolution(S);

    public static Direction3D Direction(in SurfaceAdaptorBase S) => HSurfaceTool.Direction(S);

    public static CurveAdaptorBase BasisCurve(in SurfaceAdaptorBase S) => HSurfaceTool.BasisCurve(S);

    public static SurfaceAdaptorBase BasisSurface(in SurfaceAdaptorBase S) => HSurfaceTool.BasisSurface(S);

    public static double OffsetValue(in SurfaceAdaptorBase S) => HSurfaceTool.OffsetValue(S);

    public static int NbSamplesU(in SurfaceAdaptorBase S, double u1, double u2) =>
        HSurfaceTool.NbSamplesU(S, u1, u2);

    public static int NbSamplesV(in SurfaceAdaptorBase S, double v1, double v2) =>
        HSurfaceTool.NbSamplesV(S, v1, v2);
}
