// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_BSurfaceTool.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_BSurfaceTool.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_BSurfaceTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.HiddenLines;

/// <summary>The static tool over the surface of a face (BRepAdaptor_Surface) that HLRBRep_Surface reads through.</summary>
internal static class BSurfaceTool
{
    public static double FirstUParameter(FaceSurfaceAdaptor s) => s.FirstUParameter();

    public static double FirstVParameter(FaceSurfaceAdaptor s) => s.FirstVParameter();

    public static double LastUParameter(FaceSurfaceAdaptor s) => s.LastUParameter();

    public static double LastVParameter(FaceSurfaceAdaptor s) => s.LastVParameter();

    public static bool IsUClosed(FaceSurfaceAdaptor s) => s.IsUClosed;

    public static bool IsVClosed(FaceSurfaceAdaptor s) => s.IsVClosed;

    public static bool IsUPeriodic(FaceSurfaceAdaptor s) => s.IsUPeriodic;

    public static double UPeriod(FaceSurfaceAdaptor s) => s.UPeriod();

    public static bool IsVPeriodic(FaceSurfaceAdaptor s) => s.IsVPeriodic;

    public static double VPeriod(FaceSurfaceAdaptor s) => s.VPeriod();

    public static SurfaceKind GetType(FaceSurfaceAdaptor s) => s.GetType();

    public static Plane3D Plane(FaceSurfaceAdaptor s) => s.Plane();

    public static Cylinder3D Cylinder(FaceSurfaceAdaptor s) => s.Cylinder();

    public static Cone3D Cone(FaceSurfaceAdaptor s) => s.Cone();

    public static Sphere3D Sphere(FaceSurfaceAdaptor s) => s.Sphere();

    public static Torus3D Torus(FaceSurfaceAdaptor s) => s.Torus();

    public static global::Kerf.Geometry.BezierSurface Bezier(FaceSurfaceAdaptor s) => s.Bezier();

    public static global::Kerf.Geometry.BSplineSurface BSpline(FaceSurfaceAdaptor s) => s.BSpline();

    public static Axis3D AxeOfRevolution(FaceSurfaceAdaptor s) => s.AxeOfRevolution();

    public static Direction3D Direction(FaceSurfaceAdaptor s) => s.Direction();

    public static CurveAdaptorBase BasisCurve(FaceSurfaceAdaptor s) => s.BasisCurve();

    public static Continuity UContinuity(FaceSurfaceAdaptor s) => s.UContinuity();

    public static Continuity VContinuity(FaceSurfaceAdaptor s) => s.VContinuity();

    public static int UDegree(FaceSurfaceAdaptor s) => s.UDegree();

    public static int NbUPoles(FaceSurfaceAdaptor s) => s.UPoleCount;

    public static int NbUKnots(FaceSurfaceAdaptor s) => s.UKnotCount;

    public static bool IsURational(FaceSurfaceAdaptor s) => s.IsURational;

    public static int VDegree(FaceSurfaceAdaptor s) => s.VDegree();

    public static int NbVPoles(FaceSurfaceAdaptor s) => s.VPoleCount;

    public static int NbVKnots(FaceSurfaceAdaptor s) => s.VKnotCount;

    public static bool IsVRational(FaceSurfaceAdaptor s) => s.IsVRational;

    public static int NbUIntervals(FaceSurfaceAdaptor surf, Continuity s) => surf.NbUIntervals(s);

    public static int NbVIntervals(FaceSurfaceAdaptor surf, Continuity s) => surf.NbVIntervals(s);

    public static void UIntervals(FaceSurfaceAdaptor surf, NCollection_Array1<double> tab, Continuity s)
    {
        surf.UIntervals(tab, s);
    }

    public static void VIntervals(FaceSurfaceAdaptor surf, NCollection_Array1<double> tab, Continuity s)
    {
        surf.VIntervals(tab, s);
    }

    public static SurfaceAdaptorBase UTrim(FaceSurfaceAdaptor surf, double f, double l, double tol) =>
        surf.UTrim(f, l, tol);

    public static SurfaceAdaptorBase VTrim(FaceSurfaceAdaptor surf, double f, double l, double tol) =>
        surf.VTrim(f, l, tol);

    public static Point3D Value(FaceSurfaceAdaptor s, double u, double v) => s.Value(u, v);

    public static void D0(FaceSurfaceAdaptor s, double u, double v, out Point3D p)
    {
        s.D0(u, v, out p);
    }

    public static void D1(FaceSurfaceAdaptor s, double u, double v, out Point3D p, out Vector3D d1U, out Vector3D d1V)
    {
        s.D1(u, v, out p, out d1U, out d1V);
    }

    public static void D2(FaceSurfaceAdaptor s, double u, double v, out Point3D p, out Vector3D d1U, out Vector3D d1V,
                          out Vector3D d2U, out Vector3D d2V, out Vector3D d2UV)
    {
        s.D2(u, v, out p, out d1U, out d1V, out d2U, out d2V, out d2UV);
    }

    public static void D3(FaceSurfaceAdaptor s, double u, double v, out Point3D p, out Vector3D d1U, out Vector3D d1V,
                          out Vector3D d2U, out Vector3D d2V, out Vector3D d2UV, out Vector3D d3U, out Vector3D d3V,
                          out Vector3D d3UUV, out Vector3D d3UVV)
    {
        s.D3(u, v, out p, out d1U, out d1V, out d2U, out d2V, out d2UV, out d3U, out d3V, out d3UUV, out d3UVV);
    }

    public static Vector3D DN(FaceSurfaceAdaptor s, double u, double v, int nu, int nv) => s.DN(u, v, nu, nv);

    public static double UResolution(FaceSurfaceAdaptor s, double r3d) => s.UResolution(r3d);

    public static double VResolution(FaceSurfaceAdaptor s, double r3d) => s.VResolution(r3d);

    /// <summary>The number of samples of the surface in U on its whole range.</summary>
    public static int NbSamplesU(FaceSurfaceAdaptor s)
    {
        int nbs;
        SurfaceKind typS = s.GetType();
        switch (typS)
        {
            case global::Kerf.Geometry.SurfaceKind.Plane:
                nbs = 2;
                break;
            case SurfaceKind.BezierSurface:
                nbs = 3 + s.UPoleCount;
                break;
            case SurfaceKind.BSplineSurface:
                nbs = s.UKnotCount;
                nbs *= s.UDegree();
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
    public static int NbSamplesV(FaceSurfaceAdaptor s)
    {
        int nbs;
        SurfaceKind typS = s.GetType();
        switch (typS)
        {
            case global::Kerf.Geometry.SurfaceKind.Plane:
                nbs = 2;
                break;
            case SurfaceKind.BezierSurface:
                nbs = 3 + s.VPoleCount;
                break;
            case SurfaceKind.BSplineSurface:
                nbs = s.VKnotCount;
                nbs *= s.VDegree();
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
    public static int NbSamplesU(FaceSurfaceAdaptor s, double u1, double u2)
    {
        int nbs = NbSamplesU(s);
        int n = nbs;
        if (nbs > 10)
        {
            double uf = FirstUParameter(s);
            double ul = LastUParameter(s);
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
    public static int NbSamplesV(FaceSurfaceAdaptor s, double v1, double v2)
    {
        int nbs = NbSamplesV(s);
        int n = nbs;
        if (nbs > 10)
        {
            double vf = FirstVParameter(s);
            double vl = LastVParameter(s);
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
