// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_BSurfaceTool.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_BSurfaceTool.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_BSurfaceTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Geometry.SurfaceKind;

namespace Occt;

/// <summary>The static tool over the surface of a face (BRepAdaptor_Surface) that HLRBRep_Surface reads through.</summary>
public static class HLRBRep_BSurfaceTool
{
    public static double FirstUParameter(BRepAdaptor_Surface S) => S.FirstUParameter();

    public static double FirstVParameter(BRepAdaptor_Surface S) => S.FirstVParameter();

    public static double LastUParameter(BRepAdaptor_Surface S) => S.LastUParameter();

    public static double LastVParameter(BRepAdaptor_Surface S) => S.LastVParameter();

    public static bool IsUClosed(BRepAdaptor_Surface S) => S.IsUClosed();

    public static bool IsVClosed(BRepAdaptor_Surface S) => S.IsVClosed();

    public static bool IsUPeriodic(BRepAdaptor_Surface S) => S.IsUPeriodic();

    public static double UPeriod(BRepAdaptor_Surface S) => S.UPeriod();

    public static bool IsVPeriodic(BRepAdaptor_Surface S) => S.IsVPeriodic();

    public static double VPeriod(BRepAdaptor_Surface S) => S.VPeriod();

    public static SurfaceKind GetType(BRepAdaptor_Surface S) => S.GetType();

    public static Plane3D Plane(BRepAdaptor_Surface S) => S.Plane();

    public static Cylinder Cylinder(BRepAdaptor_Surface S) => S.Cylinder();

    public static Cone Cone(BRepAdaptor_Surface S) => S.Cone();

    public static Sphere Sphere(BRepAdaptor_Surface S) => S.Sphere();

    public static Torus Torus(BRepAdaptor_Surface S) => S.Torus();

    public static Geom_BezierSurface Bezier(BRepAdaptor_Surface S) => S.Bezier();

    public static Geom_BSplineSurface BSpline(BRepAdaptor_Surface S) => S.BSpline();

    public static Axis3D AxeOfRevolution(BRepAdaptor_Surface S) => S.AxeOfRevolution();

    public static Direction3D Direction(BRepAdaptor_Surface S) => S.Direction();

    public static Adaptor3d_Curve BasisCurve(BRepAdaptor_Surface S) => S.BasisCurve();

    public static Continuity UContinuity(BRepAdaptor_Surface S) => S.UContinuity();

    public static Continuity VContinuity(BRepAdaptor_Surface S) => S.VContinuity();

    public static int UDegree(BRepAdaptor_Surface S) => S.UDegree();

    public static int NbUPoles(BRepAdaptor_Surface S) => S.NbUPoles();

    public static int NbUKnots(BRepAdaptor_Surface S) => S.NbUKnots();

    public static bool IsURational(BRepAdaptor_Surface S) => S.IsURational();

    public static int VDegree(BRepAdaptor_Surface S) => S.VDegree();

    public static int NbVPoles(BRepAdaptor_Surface S) => S.NbVPoles();

    public static int NbVKnots(BRepAdaptor_Surface S) => S.NbVKnots();

    public static bool IsVRational(BRepAdaptor_Surface S) => S.IsVRational();

    public static int NbUIntervals(BRepAdaptor_Surface Surf, Continuity S) => Surf.NbUIntervals(S);

    public static int NbVIntervals(BRepAdaptor_Surface Surf, Continuity S) => Surf.NbVIntervals(S);

    public static void UIntervals(BRepAdaptor_Surface Surf, NCollection_Array1<double> Tab, Continuity S)
    {
        Surf.UIntervals(Tab, S);
    }

    public static void VIntervals(BRepAdaptor_Surface Surf, NCollection_Array1<double> Tab, Continuity S)
    {
        Surf.VIntervals(Tab, S);
    }

    public static Adaptor3d_Surface UTrim(BRepAdaptor_Surface Surf, double F, double L, double Tol) =>
        Surf.UTrim(F, L, Tol);

    public static Adaptor3d_Surface VTrim(BRepAdaptor_Surface Surf, double F, double L, double Tol) =>
        Surf.VTrim(F, L, Tol);

    public static Point3D Value(BRepAdaptor_Surface S, double U, double V) => S.Value(U, V);

    public static void D0(BRepAdaptor_Surface S, double U, double V, out Point3D P)
    {
        S.D0(U, V, out P);
    }

    public static void D1(BRepAdaptor_Surface S, double U, double V, out Point3D P, out Vector3D D1U, out Vector3D D1V)
    {
        S.D1(U, V, out P, out D1U, out D1V);
    }

    public static void D2(BRepAdaptor_Surface S, double U, double V, out Point3D P, out Vector3D D1U, out Vector3D D1V,
                          out Vector3D D2U, out Vector3D D2V, out Vector3D D2UV)
    {
        S.D2(U, V, out P, out D1U, out D1V, out D2U, out D2V, out D2UV);
    }

    public static void D3(BRepAdaptor_Surface S, double U, double V, out Point3D P, out Vector3D D1U, out Vector3D D1V,
                          out Vector3D D2U, out Vector3D D2V, out Vector3D D2UV, out Vector3D D3U, out Vector3D D3V,
                          out Vector3D D3UUV, out Vector3D D3UVV)
    {
        S.D3(U, V, out P, out D1U, out D1V, out D2U, out D2V, out D2UV, out D3U, out D3V, out D3UUV, out D3UVV);
    }

    public static Vector3D DN(BRepAdaptor_Surface S, double U, double V, int Nu, int Nv) => S.DN(U, V, Nu, Nv);

    public static double UResolution(BRepAdaptor_Surface S, double R3d) => S.UResolution(R3d);

    public static double VResolution(BRepAdaptor_Surface S, double R3d) => S.VResolution(R3d);

    /// <summary>The number of samples of the surface in U on its whole range.</summary>
    public static int NbSamplesU(BRepAdaptor_Surface S)
    {
        int nbs;
        SurfaceKind typS = S.GetType();
        switch (typS)
        {
            case global::Kerf.Geometry.SurfaceKind.Plane:
                nbs = 2;
                break;
            case BezierSurface:
                nbs = 3 + S.NbUPoles();
                break;
            case BSplineSurface:
                nbs = S.NbUKnots();
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
    public static int NbSamplesV(BRepAdaptor_Surface S)
    {
        int nbs;
        SurfaceKind typS = S.GetType();
        switch (typS)
        {
            case global::Kerf.Geometry.SurfaceKind.Plane:
                nbs = 2;
                break;
            case BezierSurface:
                nbs = 3 + S.NbVPoles();
                break;
            case BSplineSurface:
                nbs = S.NbVKnots();
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
    public static int NbSamplesU(BRepAdaptor_Surface S, double u1, double u2)
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
    public static int NbSamplesV(BRepAdaptor_Surface S, double v1, double v2)
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
