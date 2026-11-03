// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Adaptor3d_HSurfaceTool as the tool of the generic curve/surface algorithms
/// (<see cref="IIntCurveSurface_SurfaceTool{TheSurface}"/>, and through it IntImp's surface tools): every member
/// forwards to <see cref="Adaptor3d_HSurfaceTool"/>.
/// </summary>
/// <remarks>
/// Not an OCCT class. OCCT passes Adaptor3d_HSurfaceTool itself as the template argument; the port's
/// Adaptor3d_HSurfaceTool is a static class of OcctNet.ModelingData, which can't implement the interfaces of this
/// assembly, so this struct stands in for it (decision 338). IntPatch and the other instantiations over
/// Adaptor3d_HSurfaceTool use it too, and so do GeomInt's instantiations of ApproxInt's generics
/// (<see cref="IApproxInt_SurfaceTool{ThePSurface}"/>).
/// </remarks>
public readonly struct IntCurveSurface_TheHSurfaceTool
    : IIntCurveSurface_SurfaceTool<Adaptor3d_Surface>, IApproxInt_SurfaceTool<Adaptor3d_Surface>
{
    public static double FirstUParameter(in Adaptor3d_Surface S) => Adaptor3d_HSurfaceTool.FirstUParameter(S);

    public static double FirstVParameter(in Adaptor3d_Surface S) => Adaptor3d_HSurfaceTool.FirstVParameter(S);

    public static double LastUParameter(in Adaptor3d_Surface S) => Adaptor3d_HSurfaceTool.LastUParameter(S);

    public static double LastVParameter(in Adaptor3d_Surface S) => Adaptor3d_HSurfaceTool.LastVParameter(S);

    public static int NbUIntervals(in Adaptor3d_Surface S, Continuity Sh) => Adaptor3d_HSurfaceTool.NbUIntervals(S, Sh);

    public static int NbVIntervals(in Adaptor3d_Surface S, Continuity Sh) => Adaptor3d_HSurfaceTool.NbVIntervals(S, Sh);

    public static void UIntervals(in Adaptor3d_Surface S, NCollection_Array1<double> T, Continuity Sh) =>
        Adaptor3d_HSurfaceTool.UIntervals(S, T, Sh);

    public static void VIntervals(in Adaptor3d_Surface S, NCollection_Array1<double> T, Continuity Sh) =>
        Adaptor3d_HSurfaceTool.VIntervals(S, T, Sh);

    public static Adaptor3d_Surface UTrim(in Adaptor3d_Surface S, double First, double Last, double Tol) =>
        Adaptor3d_HSurfaceTool.UTrim(S, First, Last, Tol);

    public static Adaptor3d_Surface VTrim(in Adaptor3d_Surface S, double First, double Last, double Tol) =>
        Adaptor3d_HSurfaceTool.VTrim(S, First, Last, Tol);

    public static bool IsUClosed(in Adaptor3d_Surface S) => Adaptor3d_HSurfaceTool.IsUClosed(S);

    public static bool IsVClosed(in Adaptor3d_Surface S) => Adaptor3d_HSurfaceTool.IsVClosed(S);

    public static bool IsUPeriodic(in Adaptor3d_Surface S) => Adaptor3d_HSurfaceTool.IsUPeriodic(S);

    public static double UPeriod(in Adaptor3d_Surface S) => Adaptor3d_HSurfaceTool.UPeriod(S);

    public static bool IsVPeriodic(in Adaptor3d_Surface S) => Adaptor3d_HSurfaceTool.IsVPeriodic(S);

    public static double VPeriod(in Adaptor3d_Surface S) => Adaptor3d_HSurfaceTool.VPeriod(S);

    public static Point3D Value(in Adaptor3d_Surface S, double U, double V) => Adaptor3d_HSurfaceTool.Value(S, U, V);

    public static void D0(in Adaptor3d_Surface S, double U, double V, out Point3D P) =>
        Adaptor3d_HSurfaceTool.D0(S, U, V, out P);

    public static void D1(in Adaptor3d_Surface S, double U, double V, out Point3D P, out Vector3D D1U, out Vector3D D1V) =>
        Adaptor3d_HSurfaceTool.D1(S, U, V, out P, out D1U, out D1V);

    public static double UResolution(in Adaptor3d_Surface S, double R3d) => Adaptor3d_HSurfaceTool.UResolution(S, R3d);

    public static double VResolution(in Adaptor3d_Surface S, double R3d) => Adaptor3d_HSurfaceTool.VResolution(S, R3d);

    public static SurfaceKind GetType(in Adaptor3d_Surface S) => Adaptor3d_HSurfaceTool.GetType(S);

    public static Plane3D Plane(in Adaptor3d_Surface S) => Adaptor3d_HSurfaceTool.Plane(S);

    public static Cylinder Cylinder(in Adaptor3d_Surface S) => Adaptor3d_HSurfaceTool.Cylinder(S);

    public static Cone Cone(in Adaptor3d_Surface S) => Adaptor3d_HSurfaceTool.Cone(S);

    public static Torus Torus(in Adaptor3d_Surface S) => Adaptor3d_HSurfaceTool.Torus(S);

    public static Sphere Sphere(in Adaptor3d_Surface S) => Adaptor3d_HSurfaceTool.Sphere(S);

    public static Axis3D AxeOfRevolution(in Adaptor3d_Surface S) => Adaptor3d_HSurfaceTool.AxeOfRevolution(S);

    public static Direction3D Direction(in Adaptor3d_Surface S) => Adaptor3d_HSurfaceTool.Direction(S);

    public static Adaptor3d_Curve BasisCurve(in Adaptor3d_Surface S) => Adaptor3d_HSurfaceTool.BasisCurve(S);

    public static Adaptor3d_Surface BasisSurface(in Adaptor3d_Surface S) => Adaptor3d_HSurfaceTool.BasisSurface(S);

    public static double OffsetValue(in Adaptor3d_Surface S) => Adaptor3d_HSurfaceTool.OffsetValue(S);

    public static int NbSamplesU(in Adaptor3d_Surface S, double u1, double u2) =>
        Adaptor3d_HSurfaceTool.NbSamplesU(S, u1, u2);

    public static int NbSamplesV(in Adaptor3d_Surface S, double v1, double v2) =>
        Adaptor3d_HSurfaceTool.NbSamplesV(S, v1, v2);
}
