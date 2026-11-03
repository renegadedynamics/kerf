// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_SurfaceTool.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_SurfaceTool.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_SurfaceTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Geometry.SurfaceKind;

namespace Occt;

/// <summary>The tool over HLRBRep's surfaces that the curve/surface intersection of the hiding uses.</summary>
/// <remarks>
/// A struct without state, so it can be a type argument of the generic intersections (decision 338). OCCT's surface
/// type is <c>HLRBRep_Surface*</c>; the port's type argument is the class itself (decision 1091). Some members go
/// to the surface (its type, bounds and quadrics), others straight to its adaptor (<c>Value</c>, the derivatives,
/// the intervals, the resolutions), as OCCT's. OCCT's ratios in the bounded <c>NbSamplesU</c> and <c>NbSamplesV</c>
/// divide by <c>first - last</c>, so they are negative and the counts above 10 end at 5; reproduced.
/// </remarks>
public readonly struct HLRBRep_SurfaceTool : IIntCurveSurface_SurfaceTool<HLRBRep_Surface>
{
    public static double FirstUParameter(in HLRBRep_Surface theSurf) =>
        theSurf.FirstUParameter();

    public static double FirstVParameter(in HLRBRep_Surface theSurf) =>
        theSurf.FirstVParameter();

    public static double LastUParameter(in HLRBRep_Surface theSurf) =>
        theSurf.LastUParameter();

    public static double LastVParameter(in HLRBRep_Surface theSurf) =>
        theSurf.LastVParameter();

    public static int NbUIntervals(in HLRBRep_Surface theSurf, Continuity theSh) =>
        theSurf.NbUIntervals(theSh);

    public static int NbVIntervals(in HLRBRep_Surface theSurf, Continuity theSh) =>
        theSurf.NbVIntervals(theSh);

    public static void UIntervals(in HLRBRep_Surface theSurf, NCollection_Array1<double> theTab, Continuity theSh)
    {
        theSurf.Surface().UIntervals(theTab, theSh);
    }

    public static void VIntervals(in HLRBRep_Surface theSurf, NCollection_Array1<double> theTab, Continuity theSh)
    {
        theSurf.Surface().VIntervals(theTab, theSh);
    }

    public static Adaptor3d_Surface UTrim(in HLRBRep_Surface theSurf, double theFirst, double theLast, double theTol) =>
        theSurf.Surface().UTrim(theFirst, theLast, theTol);

    public static Adaptor3d_Surface VTrim(in HLRBRep_Surface theSurf, double theFirst, double theLast, double theTol) =>
        theSurf.Surface().VTrim(theFirst, theLast, theTol);

    public static bool IsUClosed(in HLRBRep_Surface theSurf) =>
        theSurf.IsUClosed();

    public static bool IsVClosed(in HLRBRep_Surface theSurf) =>
        theSurf.IsVClosed();

    public static bool IsUPeriodic(in HLRBRep_Surface theSurf) =>
        theSurf.IsUPeriodic();

    public static double UPeriod(in HLRBRep_Surface theSurf) =>
        theSurf.UPeriod();

    public static bool IsVPeriodic(in HLRBRep_Surface theSurf) =>
        theSurf.IsVPeriodic();

    public static double VPeriod(in HLRBRep_Surface theSurf) =>
        theSurf.VPeriod();

    public static Point3D Value(in HLRBRep_Surface theSurf, double theU, double theV) =>
        theSurf.Surface().Value(theU, theV);

    public static void D0(in HLRBRep_Surface theSurf, double theU, double theV, out Point3D theP)
    {
        theSurf.Surface().D0(theU, theV, out theP);
    }

    public static void D1(in HLRBRep_Surface theSurf, double theU, double theV, out Point3D theP, out Vector3D theD1U,
                          out Vector3D theD1V)
    {
        theSurf.Surface().D1(theU, theV, out theP, out theD1U, out theD1V);
    }

    public static void D2(in HLRBRep_Surface theSurf, double theU, double theV, out Point3D theP, out Vector3D theD1U,
                          out Vector3D theD1V, out Vector3D theD2U, out Vector3D theD2V, out Vector3D theD2UV)
    {
        theSurf.Surface().D2(theU, theV, out theP, out theD1U, out theD1V, out theD2U, out theD2V, out theD2UV);
    }

    public static void D3(in HLRBRep_Surface theSurf, double theU, double theV, out Point3D theP, out Vector3D theD1U,
                          out Vector3D theD1V, out Vector3D theD2U, out Vector3D theD2V, out Vector3D theD2UV, out Vector3D theD3U,
                          out Vector3D theD3V, out Vector3D theD3UUV, out Vector3D theD3UVV)
    {
        theSurf.Surface().D3(theU, theV, out theP, out theD1U, out theD1V, out theD2U, out theD2V, out theD2UV,
                             out theD3U, out theD3V, out theD3UUV, out theD3UVV);
    }

    public static Vector3D DN(in HLRBRep_Surface theSurf, double theU, double theV, int theNu, int theNv) =>
        theSurf.Surface().DN(theU, theV, theNu, theNv);

    public static double UResolution(in HLRBRep_Surface theSurf, double theR3d) =>
        theSurf.Surface().UResolution(theR3d);

    public static double VResolution(in HLRBRep_Surface theSurf, double theR3d) =>
        theSurf.Surface().VResolution(theR3d);

    public static SurfaceKind GetType(in HLRBRep_Surface theSurf) =>
        theSurf.GetType();

    public static Plane3D Plane(in HLRBRep_Surface theSurf) =>
        theSurf.Plane();

    public static Cylinder Cylinder(in HLRBRep_Surface theSurf) =>
        theSurf.Cylinder();

    public static Cone Cone(in HLRBRep_Surface theSurf) =>
        theSurf.Cone();

    public static Sphere Sphere(in HLRBRep_Surface theSurf) =>
        theSurf.Sphere();

    public static Torus Torus(in HLRBRep_Surface theSurf) =>
        theSurf.Torus();

    public static Geom_BezierSurface Bezier(in HLRBRep_Surface theSurf) =>
        theSurf.Surface().Bezier();

    public static Geom_BSplineSurface BSpline(in HLRBRep_Surface theSurf) =>
        theSurf.Surface().BSpline();

    public static Axis3D AxeOfRevolution(in HLRBRep_Surface theSurf) =>
        theSurf.Axis();

    public static Direction3D Direction(in HLRBRep_Surface theSurf) =>
        theSurf.Surface().Direction();

    public static Adaptor3d_Curve BasisCurve(in HLRBRep_Surface theSurf) =>
        theSurf.Surface().BasisCurve();

    public static Adaptor3d_Surface BasisSurface(in HLRBRep_Surface theSurf) =>
        theSurf.Surface().BasisSurface();

    public static double OffsetValue(in HLRBRep_Surface theSurf) =>
        theSurf.Surface().OffsetValue();

    /// <summary>The number of samples in U on the whole range.</summary>
    public static int NbSamplesU(in HLRBRep_Surface theSurf)
    {
        int nbs;
        SurfaceKind typS = theSurf.GetType();
        switch (typS)
        {
            case global::Kerf.Geometry.SurfaceKind.Plane:
                nbs = 2;
                break;
            case BezierSurface:
                nbs = 3 + theSurf.NbUPoles();
                break;
            case BSplineSurface:
                nbs = theSurf.NbUKnots();
                nbs *= theSurf.UDegree();
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
    public static int NbSamplesV(in HLRBRep_Surface theSurf)
    {
        int nbs;
        SurfaceKind typS = theSurf.GetType();
        switch (typS)
        {
            case global::Kerf.Geometry.SurfaceKind.Plane:
                nbs = 2;
                break;
            case BezierSurface:
                nbs = 3 + theSurf.NbVPoles();
                break;
            case BSplineSurface:
                nbs = theSurf.NbVKnots();
                nbs *= theSurf.VDegree();
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
    public static int NbSamplesU(in HLRBRep_Surface theSurf, double theU1, double theU2)
    {
        int nbs = NbSamplesU(theSurf);
        int n = nbs;
        if (nbs > 10)
        {
            double uf = FirstUParameter(theSurf);
            double ul = LastUParameter(theSurf);
            n *= (int)((theU2 - theU1) / (uf - ul)); // OCCT
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
    public static int NbSamplesV(in HLRBRep_Surface theSurf, double theV1, double theV2)
    {
        int nbs = NbSamplesV(theSurf);
        int n = nbs;
        if (nbs > 10)
        {
            double vf = FirstVParameter(theSurf);
            double vl = LastVParameter(theSurf);
            n *= (int)((theV2 - theV1) / (vf - vl)); // OCCT
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
