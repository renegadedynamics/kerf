// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_SurfaceTool.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_SurfaceTool.lxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_SurfaceTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>The static tool over a <see cref="BRepAdaptor_Surface"/>: every member forwards to the adaptor.</summary>
/// <remarks>
/// OCCT's class of static functions is an empty struct implementing the static members of
/// <see cref="IApproxInt_SurfaceTool{ThePSurface}"/>, so that it can be the ThePSurfaceTool type argument of
/// ApproxInt's and IntImp's generics (decision 607).
/// </remarks>
public readonly struct BRepApprox_SurfaceTool : IApproxInt_SurfaceTool<BRepAdaptor_Surface>
{
    public static double FirstUParameter(in BRepAdaptor_Surface Surf) => Surf.FirstUParameter();

    public static double FirstVParameter(in BRepAdaptor_Surface Surf) => Surf.FirstVParameter();

    public static double LastUParameter(in BRepAdaptor_Surface Surf) => Surf.LastUParameter();

    public static double LastVParameter(in BRepAdaptor_Surface Surf) => Surf.LastVParameter();

    public static int NbUIntervals(in BRepAdaptor_Surface Surf, Continuity S) => Surf.NbUIntervals(S);

    public static int NbVIntervals(in BRepAdaptor_Surface Surf, Continuity S) => Surf.NbVIntervals(S);

    public static void UIntervals(in BRepAdaptor_Surface Surf, NCollection_Array1<double> Tab, Continuity S)
    {
        Surf.UIntervals(Tab, S);
    }

    public static void VIntervals(in BRepAdaptor_Surface Surf, NCollection_Array1<double> Tab, Continuity S)
    {
        Surf.VIntervals(Tab, S);
    }

    public static Adaptor3d_Surface UTrim(in BRepAdaptor_Surface Surf, double F, double L, double Tol) =>
        Surf.UTrim(F, L, Tol);

    public static Adaptor3d_Surface VTrim(in BRepAdaptor_Surface Surf, double F, double L, double Tol) =>
        Surf.VTrim(F, L, Tol);

    public static bool IsUClosed(in BRepAdaptor_Surface S) => S.IsUClosed();

    public static bool IsVClosed(in BRepAdaptor_Surface S) => S.IsVClosed();

    public static bool IsUPeriodic(in BRepAdaptor_Surface S) => S.IsUPeriodic();

    public static double UPeriod(in BRepAdaptor_Surface S) => S.UPeriod();

    public static bool IsVPeriodic(in BRepAdaptor_Surface S) => S.IsVPeriodic();

    public static double VPeriod(in BRepAdaptor_Surface S) => S.VPeriod();

    public static Point3D Value(in BRepAdaptor_Surface S, double U, double V) => S.Value(U, V);

    public static void D0(in BRepAdaptor_Surface S, double U, double V, out Point3D P)
    {
        S.D0(U, V, out P);
    }

    public static void D1(in BRepAdaptor_Surface S, double U, double V, out Point3D P, out Vector3D D1U, out Vector3D D1V)
    {
        S.D1(U, V, out P, out D1U, out D1V);
    }

    public static void D2(in BRepAdaptor_Surface S, double U, double V, out Point3D P, out Vector3D D1U, out Vector3D D1V,
                          out Vector3D D2U, out Vector3D D2V, out Vector3D D2UV)
    {
        S.D2(U, V, out P, out D1U, out D1V, out D2U, out D2V, out D2UV);
    }

    public static void D3(in BRepAdaptor_Surface S, double U, double V, out Point3D P, out Vector3D D1U, out Vector3D D1V,
                          out Vector3D D2U, out Vector3D D2V, out Vector3D D2UV, out Vector3D D3U, out Vector3D D3V,
                          out Vector3D D3UUV, out Vector3D D3UVV)
    {
        S.D3(U, V, out P, out D1U, out D1V, out D2U, out D2V, out D2UV, out D3U, out D3V, out D3UUV, out D3UVV);
    }

    public static Vector3D DN(in BRepAdaptor_Surface S, double U, double V, int Nu, int Nv) => S.DN(U, V, Nu, Nv);

    public static double UResolution(in BRepAdaptor_Surface S, double R3d) => S.UResolution(R3d);

    public static double VResolution(in BRepAdaptor_Surface S, double R3d) => S.VResolution(R3d);

    public static SurfaceKind GetType(in BRepAdaptor_Surface S) => S.GetType();

    public static Plane3D Plane(in BRepAdaptor_Surface S) => S.Plane();

    public static Cylinder Cylinder(in BRepAdaptor_Surface S) => S.Cylinder();

    public static Cone Cone(in BRepAdaptor_Surface S) => S.Cone();

    public static Torus Torus(in BRepAdaptor_Surface S) => S.Torus();

    public static Sphere Sphere(in BRepAdaptor_Surface S) => S.Sphere();

    public static Geom_BezierSurface Bezier(in BRepAdaptor_Surface S) => (S.Bezier());

    public static Geom_BSplineSurface BSpline(in BRepAdaptor_Surface S) => (S.BSpline());

    public static Axis3D AxeOfRevolution(in BRepAdaptor_Surface S) => (S.AxeOfRevolution());

    public static Direction3D Direction(in BRepAdaptor_Surface S) => (S.Direction());

    public static Adaptor3d_Curve BasisCurve(in BRepAdaptor_Surface S) => (S.BasisCurve());

    public static int NbSamplesU(in BRepAdaptor_Surface S) => (10);

    public static int NbSamplesV(in BRepAdaptor_Surface S) => (10);

    public static int NbSamplesU(in BRepAdaptor_Surface S, double u1, double u2) => (10);

    public static int NbSamplesV(in BRepAdaptor_Surface S, double v1, double v2) => (10);
}
