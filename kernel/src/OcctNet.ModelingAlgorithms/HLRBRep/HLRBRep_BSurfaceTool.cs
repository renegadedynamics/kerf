// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_BSurfaceTool.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_BSurfaceTool.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_BSurfaceTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using static Kerf.Geometry.SurfaceKind;

namespace Occt;

/// <summary>The static tool over the surface of a face (BRepAdaptor_Surface) that HLRBRep_Surface reads through.</summary>
public static class HLRBRep_BSurfaceTool
{
    public static double FirstUParameter(FaceSurfaceAdaptor S) => S.FirstUParameter();

    public static double FirstVParameter(FaceSurfaceAdaptor S) => S.FirstVParameter();

    public static double LastUParameter(FaceSurfaceAdaptor S) => S.LastUParameter();

    public static double LastVParameter(FaceSurfaceAdaptor S) => S.LastVParameter();

    public static bool IsUClosed(FaceSurfaceAdaptor S) => S.IsUClosed;

    public static bool IsVClosed(FaceSurfaceAdaptor S) => S.IsVClosed;

    public static bool IsUPeriodic(FaceSurfaceAdaptor S) => S.IsUPeriodic;

    public static double UPeriod(FaceSurfaceAdaptor S) => S.UPeriod();

    public static bool IsVPeriodic(FaceSurfaceAdaptor S) => S.IsVPeriodic;

    public static double VPeriod(FaceSurfaceAdaptor S) => S.VPeriod();

    public static SurfaceKind GetType(FaceSurfaceAdaptor S) => S.GetType();

    public static Plane3D Plane(FaceSurfaceAdaptor S) => S.Plane();

    public static Cylinder Cylinder(FaceSurfaceAdaptor S) => S.Cylinder();

    public static Cone Cone(FaceSurfaceAdaptor S) => S.Cone();

    public static Sphere Sphere(FaceSurfaceAdaptor S) => S.Sphere();

    public static Torus Torus(FaceSurfaceAdaptor S) => S.Torus();

    public static global::Kerf.Geometry.BezierSurface Bezier(FaceSurfaceAdaptor S) => S.Bezier();

    public static global::Kerf.Geometry.BSplineSurface BSpline(FaceSurfaceAdaptor S) => S.BSpline();

    public static Axis3D AxeOfRevolution(FaceSurfaceAdaptor S) => S.AxeOfRevolution();

    public static Direction3D Direction(FaceSurfaceAdaptor S) => S.Direction();

    public static CurveAdaptorBase BasisCurve(FaceSurfaceAdaptor S) => S.BasisCurve();

    public static Continuity UContinuity(FaceSurfaceAdaptor S) => S.UContinuity();

    public static Continuity VContinuity(FaceSurfaceAdaptor S) => S.VContinuity();

    public static int UDegree(FaceSurfaceAdaptor S) => S.UDegree();

    public static int NbUPoles(FaceSurfaceAdaptor S) => S.UPoleCount;

    public static int NbUKnots(FaceSurfaceAdaptor S) => S.UKnotCount;

    public static bool IsURational(FaceSurfaceAdaptor S) => S.IsURational;

    public static int VDegree(FaceSurfaceAdaptor S) => S.VDegree();

    public static int NbVPoles(FaceSurfaceAdaptor S) => S.VPoleCount;

    public static int NbVKnots(FaceSurfaceAdaptor S) => S.VKnotCount;

    public static bool IsVRational(FaceSurfaceAdaptor S) => S.IsVRational;

    public static int NbUIntervals(FaceSurfaceAdaptor Surf, Continuity S) => Surf.NbUIntervals(S);

    public static int NbVIntervals(FaceSurfaceAdaptor Surf, Continuity S) => Surf.NbVIntervals(S);

    public static void UIntervals(FaceSurfaceAdaptor Surf, NCollection_Array1<double> Tab, Continuity S)
    {
        Surf.UIntervals(Tab, S);
    }

    public static void VIntervals(FaceSurfaceAdaptor Surf, NCollection_Array1<double> Tab, Continuity S)
    {
        Surf.VIntervals(Tab, S);
    }

    public static SurfaceAdaptorBase UTrim(FaceSurfaceAdaptor Surf, double F, double L, double Tol) =>
        Surf.UTrim(F, L, Tol);

    public static SurfaceAdaptorBase VTrim(FaceSurfaceAdaptor Surf, double F, double L, double Tol) =>
        Surf.VTrim(F, L, Tol);

    public static Point3D Value(FaceSurfaceAdaptor S, double U, double V) => S.Value(U, V);

    public static void D0(FaceSurfaceAdaptor S, double U, double V, out Point3D P)
    {
        S.D0(U, V, out P);
    }

    public static void D1(FaceSurfaceAdaptor S, double U, double V, out Point3D P, out Vector3D D1U, out Vector3D D1V)
    {
        S.D1(U, V, out P, out D1U, out D1V);
    }

    public static void D2(FaceSurfaceAdaptor S, double U, double V, out Point3D P, out Vector3D D1U, out Vector3D D1V,
                          out Vector3D D2U, out Vector3D D2V, out Vector3D D2UV)
    {
        S.D2(U, V, out P, out D1U, out D1V, out D2U, out D2V, out D2UV);
    }

    public static void D3(FaceSurfaceAdaptor S, double U, double V, out Point3D P, out Vector3D D1U, out Vector3D D1V,
                          out Vector3D D2U, out Vector3D D2V, out Vector3D D2UV, out Vector3D D3U, out Vector3D D3V,
                          out Vector3D D3UUV, out Vector3D D3UVV)
    {
        S.D3(U, V, out P, out D1U, out D1V, out D2U, out D2V, out D2UV, out D3U, out D3V, out D3UUV, out D3UVV);
    }

    public static Vector3D DN(FaceSurfaceAdaptor S, double U, double V, int Nu, int Nv) => S.DN(U, V, Nu, Nv);

    public static double UResolution(FaceSurfaceAdaptor S, double R3d) => S.UResolution(R3d);

    public static double VResolution(FaceSurfaceAdaptor S, double R3d) => S.VResolution(R3d);

    /// <summary>The number of samples of the surface in U on its whole range.</summary>
    public static int NbSamplesU(FaceSurfaceAdaptor S)
    {
        int nbs;
        SurfaceKind typS = S.GetType();
        switch (typS)
        {
            case global::Kerf.Geometry.SurfaceKind.Plane:
                nbs = 2;
                break;
            case SurfaceKind.BezierSurface:
                nbs = 3 + S.UPoleCount;
                break;
            case SurfaceKind.BSplineSurface:
                nbs = S.UKnotCount;
                nbs *= S.UDegree();
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
        return (nbs);
    }

    /// <summary>The number of samples of the surface in V on its whole range.</summary>
    public static int NbSamplesV(FaceSurfaceAdaptor S)
    {
        int nbs;
        SurfaceKind typS = S.GetType();
        switch (typS)
        {
            case global::Kerf.Geometry.SurfaceKind.Plane:
                nbs = 2;
                break;
            case SurfaceKind.BezierSurface:
                nbs = 3 + S.VPoleCount;
                break;
            case SurfaceKind.BSplineSurface:
                nbs = S.VKnotCount;
                nbs *= S.VDegree();
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
        return (nbs);
    }

    /// <summary>The number of samples of the surface in U on [u1, u2].</summary>
    public static int NbSamplesU(FaceSurfaceAdaptor S, double u1, double u2)
    {
        int nbs = NbSamplesU(S);
        int n = nbs;
        if (nbs > 10)
        {
            double uf = FirstUParameter(S);
            double ul = LastUParameter(S);
            n *= (int)((u2 - u1) / (uf - ul)); // OCCT: the ratio is negative, so n ends at 5
            if (n > nbs)
            {
                n = nbs;
            }
            if (n < 5)
            {
                n = 5;
            }
        }
        return (n);
    }

    /// <summary>The number of samples of the surface in V on [v1, v2].</summary>
    public static int NbSamplesV(FaceSurfaceAdaptor S, double v1, double v2)
    {
        int nbs = NbSamplesV(S);
        int n = nbs;
        if (nbs > 10)
        {
            double vf = FirstVParameter(S);
            double vl = LastVParameter(S);
            n *= (int)((v2 - v1) / (vf - vl)); // OCCT: the ratio is negative, so n ends at 5
            if (n > nbs)
            {
                n = nbs;
            }
            if (n < 5)
            {
                n = 5;
            }
        }
        return (n);
    }
}
