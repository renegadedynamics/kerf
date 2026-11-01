// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_SurfaceTool.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_SurfaceTool.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_SurfaceTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_SurfaceType;

namespace Occt;

/// <summary>The tool over HLRBRep's surfaces that the curve/surface intersection of the hiding uses.</summary>
/// <remarks>
/// A struct without state, so it can be a type argument of the generic intersections (decision 338). OCCT's surface
/// type is <c>HLRBRep_Surface*</c>; the port's type argument is the class itself (decision 10601). Some members go
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

    public static int NbUIntervals(in HLRBRep_Surface theSurf, GeomAbs_Shape theSh) =>
        theSurf.NbUIntervals(theSh);

    public static int NbVIntervals(in HLRBRep_Surface theSurf, GeomAbs_Shape theSh) =>
        theSurf.NbVIntervals(theSh);

    public static void UIntervals(in HLRBRep_Surface theSurf, NCollection_Array1<double> theTab, GeomAbs_Shape theSh)
    {
        theSurf.Surface().UIntervals(theTab, theSh);
    }

    public static void VIntervals(in HLRBRep_Surface theSurf, NCollection_Array1<double> theTab, GeomAbs_Shape theSh)
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

    public static gp_Pnt Value(in HLRBRep_Surface theSurf, double theU, double theV) =>
        theSurf.Surface().Value(theU, theV);

    public static void D0(in HLRBRep_Surface theSurf, double theU, double theV, out gp_Pnt theP)
    {
        theSurf.Surface().D0(theU, theV, out theP);
    }

    public static void D1(in HLRBRep_Surface theSurf, double theU, double theV, out gp_Pnt theP, out gp_Vec theD1U,
                          out gp_Vec theD1V)
    {
        theSurf.Surface().D1(theU, theV, out theP, out theD1U, out theD1V);
    }

    public static void D2(in HLRBRep_Surface theSurf, double theU, double theV, out gp_Pnt theP, out gp_Vec theD1U,
                          out gp_Vec theD1V, out gp_Vec theD2U, out gp_Vec theD2V, out gp_Vec theD2UV)
    {
        theSurf.Surface().D2(theU, theV, out theP, out theD1U, out theD1V, out theD2U, out theD2V, out theD2UV);
    }

    public static void D3(in HLRBRep_Surface theSurf, double theU, double theV, out gp_Pnt theP, out gp_Vec theD1U,
                          out gp_Vec theD1V, out gp_Vec theD2U, out gp_Vec theD2V, out gp_Vec theD2UV, out gp_Vec theD3U,
                          out gp_Vec theD3V, out gp_Vec theD3UUV, out gp_Vec theD3UVV)
    {
        theSurf.Surface().D3(theU, theV, out theP, out theD1U, out theD1V, out theD2U, out theD2V, out theD2UV,
                             out theD3U, out theD3V, out theD3UUV, out theD3UVV);
    }

    public static gp_Vec DN(in HLRBRep_Surface theSurf, double theU, double theV, int theNu, int theNv) =>
        theSurf.Surface().DN(theU, theV, theNu, theNv);

    public static double UResolution(in HLRBRep_Surface theSurf, double theR3d) =>
        theSurf.Surface().UResolution(theR3d);

    public static double VResolution(in HLRBRep_Surface theSurf, double theR3d) =>
        theSurf.Surface().VResolution(theR3d);

    public static GeomAbs_SurfaceType GetType(in HLRBRep_Surface theSurf) =>
        theSurf.GetType();

    public static gp_Pln Plane(in HLRBRep_Surface theSurf) =>
        theSurf.Plane();

    public static gp_Cylinder Cylinder(in HLRBRep_Surface theSurf) =>
        theSurf.Cylinder();

    public static gp_Cone Cone(in HLRBRep_Surface theSurf) =>
        theSurf.Cone();

    public static gp_Sphere Sphere(in HLRBRep_Surface theSurf) =>
        theSurf.Sphere();

    public static gp_Torus Torus(in HLRBRep_Surface theSurf) =>
        theSurf.Torus();

    public static Geom_BezierSurface Bezier(in HLRBRep_Surface theSurf) =>
        theSurf.Surface().Bezier();

    public static Geom_BSplineSurface BSpline(in HLRBRep_Surface theSurf) =>
        theSurf.Surface().BSpline();

    public static gp_Ax1 AxeOfRevolution(in HLRBRep_Surface theSurf) =>
        theSurf.Axis();

    public static gp_Dir Direction(in HLRBRep_Surface theSurf) =>
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
        GeomAbs_SurfaceType typS = theSurf.GetType();
        switch (typS)
        {
            case GeomAbs_Plane:
                nbs = 2;
                break;
            case GeomAbs_BezierSurface:
                nbs = 3 + theSurf.NbUPoles();
                break;
            case GeomAbs_BSplineSurface:
                nbs = theSurf.NbUKnots();
                nbs *= theSurf.UDegree();
                if (nbs < 2)
                {
                    nbs = 2;
                }
                break;
            case GeomAbs_Torus:
                nbs = 20;
                break;
            case GeomAbs_Cylinder:
            case GeomAbs_Cone:
            case GeomAbs_Sphere:
            case GeomAbs_SurfaceOfRevolution:
            case GeomAbs_SurfaceOfExtrusion:
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
        GeomAbs_SurfaceType typS = theSurf.GetType();
        switch (typS)
        {
            case GeomAbs_Plane:
                nbs = 2;
                break;
            case GeomAbs_BezierSurface:
                nbs = 3 + theSurf.NbVPoles();
                break;
            case GeomAbs_BSplineSurface:
                nbs = theSurf.NbVKnots();
                nbs *= theSurf.VDegree();
                if (nbs < 2)
                {
                    nbs = 2;
                }
                break;
            case GeomAbs_Cylinder:
            case GeomAbs_Cone:
            case GeomAbs_Sphere:
            case GeomAbs_Torus:
            case GeomAbs_SurfaceOfRevolution:
            case GeomAbs_SurfaceOfExtrusion:
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
