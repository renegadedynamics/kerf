// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_SurfaceTool.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_SurfaceTool.lxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_SurfaceTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;

namespace Occt;

/// <summary>The static tool over a <see cref="FaceSurfaceAdaptor"/>: every member forwards to the adaptor.</summary>
/// <remarks>
/// OCCT's class of static functions is an empty struct implementing the static members of
/// <see cref="IApproxInt_SurfaceTool{ThePSurface}"/>, so that it can be the ThePSurfaceTool type argument of
/// ApproxInt's and IntImp's generics (decision 607).
/// </remarks>
public readonly struct BRepApprox_SurfaceTool : IApproxInt_SurfaceTool<FaceSurfaceAdaptor>
{
    public static double FirstUParameter(in FaceSurfaceAdaptor Surf) => Surf.FirstUParameter();

    public static double FirstVParameter(in FaceSurfaceAdaptor Surf) => Surf.FirstVParameter();

    public static double LastUParameter(in FaceSurfaceAdaptor Surf) => Surf.LastUParameter();

    public static double LastVParameter(in FaceSurfaceAdaptor Surf) => Surf.LastVParameter();

    public static int NbUIntervals(in FaceSurfaceAdaptor Surf, Continuity S) => Surf.NbUIntervals(S);

    public static int NbVIntervals(in FaceSurfaceAdaptor Surf, Continuity S) => Surf.NbVIntervals(S);

    public static void UIntervals(in FaceSurfaceAdaptor Surf, NCollection_Array1<double> Tab, Continuity S)
    {
        Surf.UIntervals(Tab, S);
    }

    public static void VIntervals(in FaceSurfaceAdaptor Surf, NCollection_Array1<double> Tab, Continuity S)
    {
        Surf.VIntervals(Tab, S);
    }

    public static SurfaceAdaptorBase UTrim(in FaceSurfaceAdaptor Surf, double F, double L, double Tol) =>
        Surf.UTrim(F, L, Tol);

    public static SurfaceAdaptorBase VTrim(in FaceSurfaceAdaptor Surf, double F, double L, double Tol) =>
        Surf.VTrim(F, L, Tol);

    public static bool IsUClosed(in FaceSurfaceAdaptor S) => S.IsUClosed;

    public static bool IsVClosed(in FaceSurfaceAdaptor S) => S.IsVClosed;

    public static bool IsUPeriodic(in FaceSurfaceAdaptor S) => S.IsUPeriodic;

    public static double UPeriod(in FaceSurfaceAdaptor S) => S.UPeriod();

    public static bool IsVPeriodic(in FaceSurfaceAdaptor S) => S.IsVPeriodic;

    public static double VPeriod(in FaceSurfaceAdaptor S) => S.VPeriod();

    public static Point3D Value(in FaceSurfaceAdaptor S, double U, double V) => S.Value(U, V);

    public static void D0(in FaceSurfaceAdaptor S, double U, double V, out Point3D P)
    {
        S.D0(U, V, out P);
    }

    public static void D1(in FaceSurfaceAdaptor S, double U, double V, out Point3D P, out Vector3D D1U, out Vector3D D1V)
    {
        S.D1(U, V, out P, out D1U, out D1V);
    }

    public static void D2(in FaceSurfaceAdaptor S, double U, double V, out Point3D P, out Vector3D D1U, out Vector3D D1V,
                          out Vector3D D2U, out Vector3D D2V, out Vector3D D2UV)
    {
        S.D2(U, V, out P, out D1U, out D1V, out D2U, out D2V, out D2UV);
    }

    public static void D3(in FaceSurfaceAdaptor S, double U, double V, out Point3D P, out Vector3D D1U, out Vector3D D1V,
                          out Vector3D D2U, out Vector3D D2V, out Vector3D D2UV, out Vector3D D3U, out Vector3D D3V,
                          out Vector3D D3UUV, out Vector3D D3UVV)
    {
        S.D3(U, V, out P, out D1U, out D1V, out D2U, out D2V, out D2UV, out D3U, out D3V, out D3UUV, out D3UVV);
    }

    public static Vector3D DN(in FaceSurfaceAdaptor S, double U, double V, int Nu, int Nv) => S.DN(U, V, Nu, Nv);

    public static double UResolution(in FaceSurfaceAdaptor S, double R3d) => S.UResolution(R3d);

    public static double VResolution(in FaceSurfaceAdaptor S, double R3d) => S.VResolution(R3d);

    public static SurfaceKind GetType(in FaceSurfaceAdaptor S) => S.GetType();

    public static Plane3D Plane(in FaceSurfaceAdaptor S) => S.Plane();

    public static Cylinder Cylinder(in FaceSurfaceAdaptor S) => S.Cylinder();

    public static Cone Cone(in FaceSurfaceAdaptor S) => S.Cone();

    public static Torus Torus(in FaceSurfaceAdaptor S) => S.Torus();

    public static Sphere Sphere(in FaceSurfaceAdaptor S) => S.Sphere();

    public static BezierSurface Bezier(in FaceSurfaceAdaptor S) => (S.Bezier());

    public static BSplineSurface BSpline(in FaceSurfaceAdaptor S) => (S.BSpline());

    public static Axis3D AxeOfRevolution(in FaceSurfaceAdaptor S) => (S.AxeOfRevolution());

    public static Direction3D Direction(in FaceSurfaceAdaptor S) => (S.Direction());

    public static CurveAdaptorBase BasisCurve(in FaceSurfaceAdaptor S) => (S.BasisCurve());

    public static int NbSamplesU(in FaceSurfaceAdaptor S) => (10);

    public static int NbSamplesV(in FaceSurfaceAdaptor S) => (10);

    public static int NbSamplesU(in FaceSurfaceAdaptor S, double u1, double u2) => (10);

    public static int NbSamplesV(in FaceSurfaceAdaptor S, double v1, double v2) => (10);
}
