// Port of: src/ModelingData/TKG3d/Adaptor3d/Adaptor3d_HSurfaceTool.hxx
// Port of: src/ModelingData/TKG3d/Adaptor3d/Adaptor3d_HSurfaceTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_SurfaceType;

namespace Occt;

/// <summary>
/// Static access to a surface adaptor's services, as the generic algorithms (intersections, extrema) call them,
/// plus sampling counts and a G1 test.
/// </summary>
public static class Adaptor3d_HSurfaceTool
{
    public static double FirstUParameter(Adaptor3d_Surface theSurf) => theSurf.FirstUParameter();

    public static double FirstVParameter(Adaptor3d_Surface theSurf) => theSurf.FirstVParameter();

    public static double LastUParameter(Adaptor3d_Surface theSurf) => theSurf.LastUParameter();

    public static double LastVParameter(Adaptor3d_Surface theSurf) => theSurf.LastVParameter();

    public static int NbUIntervals(Adaptor3d_Surface theSurf, GeomAbs_Shape theSh) => theSurf.NbUIntervals(theSh);

    public static int NbVIntervals(Adaptor3d_Surface theSurf, GeomAbs_Shape theSh) => theSurf.NbVIntervals(theSh);

    public static void UIntervals(Adaptor3d_Surface theSurf, NCollection_Array1<double> theTab, GeomAbs_Shape theSh) =>
        theSurf.UIntervals(theTab, theSh);

    public static void VIntervals(Adaptor3d_Surface theSurf, NCollection_Array1<double> theTab, GeomAbs_Shape theSh) =>
        theSurf.VIntervals(theTab, theSh);

    public static Adaptor3d_Surface UTrim(Adaptor3d_Surface theSurf, double theFirst, double theLast, double theTol) =>
        theSurf.UTrim(theFirst, theLast, theTol);

    public static Adaptor3d_Surface VTrim(Adaptor3d_Surface theSurf, double theFirst, double theLast, double theTol) =>
        theSurf.VTrim(theFirst, theLast, theTol);

    public static bool IsUClosed(Adaptor3d_Surface theSurf) => theSurf.IsUClosed();

    public static bool IsVClosed(Adaptor3d_Surface theSurf) => theSurf.IsVClosed();

    public static bool IsUPeriodic(Adaptor3d_Surface theSurf) => theSurf.IsUPeriodic();

    public static double UPeriod(Adaptor3d_Surface theSurf) => theSurf.UPeriod();

    public static bool IsVPeriodic(Adaptor3d_Surface theSurf) => theSurf.IsVPeriodic();

    public static double VPeriod(Adaptor3d_Surface theSurf) => theSurf.VPeriod();

    public static gp_Pnt Value(Adaptor3d_Surface theSurf, double theU, double theV) => theSurf.Value(theU, theV);

    public static void D0(Adaptor3d_Surface theSurf, double theU, double theV, out gp_Pnt thePnt) =>
        theSurf.D0(theU, theV, out thePnt);

    public static void D1(Adaptor3d_Surface theSurf, double theU, double theV, out gp_Pnt thePnt, out gp_Vec theD1U,
                          out gp_Vec theD1V) =>
        theSurf.D1(theU, theV, out thePnt, out theD1U, out theD1V);

    public static void D2(Adaptor3d_Surface theSurf, double theU, double theV, out gp_Pnt thePnt, out gp_Vec theD1U,
                          out gp_Vec theD1V, out gp_Vec theD2U, out gp_Vec theD2V, out gp_Vec theD2UV) =>
        theSurf.D2(theU, theV, out thePnt, out theD1U, out theD1V, out theD2U, out theD2V, out theD2UV);

    public static void D3(Adaptor3d_Surface theSurf, double theU, double theV, out gp_Pnt thePnt, out gp_Vec theD1U,
                          out gp_Vec theD1V, out gp_Vec theD2U, out gp_Vec theD2V, out gp_Vec theD2UV,
                          out gp_Vec theD3U, out gp_Vec theD3V, out gp_Vec theD3UUV, out gp_Vec theD3UVV) =>
        theSurf.D3(theU,
                   theV,
                   out thePnt,
                   out theD1U,
                   out theD1V,
                   out theD2U,
                   out theD2V,
                   out theD2UV,
                   out theD3U,
                   out theD3V,
                   out theD3UUV,
                   out theD3UVV);

    public static gp_Vec DN(Adaptor3d_Surface theSurf, double theU, double theV, int theNU, int theNV) =>
        theSurf.DN(theU, theV, theNU, theNV);

    public static double UResolution(Adaptor3d_Surface theSurf, double theR3d) => theSurf.UResolution(theR3d);

    public static double VResolution(Adaptor3d_Surface theSurf, double theR3d) => theSurf.VResolution(theR3d);

    public static GeomAbs_SurfaceType GetType(Adaptor3d_Surface theSurf) => theSurf.GetType();

    public static gp_Pln Plane(Adaptor3d_Surface theSurf) => theSurf.Plane();

    public static gp_Cylinder Cylinder(Adaptor3d_Surface theSurf) => theSurf.Cylinder();

    public static gp_Cone Cone(Adaptor3d_Surface theSurf) => theSurf.Cone();

    public static gp_Torus Torus(Adaptor3d_Surface theSurf) => theSurf.Torus();

    public static gp_Sphere Sphere(Adaptor3d_Surface theSurf) => theSurf.Sphere();

    public static Geom_BezierSurface Bezier(Adaptor3d_Surface theSurf) => theSurf.Bezier();

    public static Geom_BSplineSurface BSpline(Adaptor3d_Surface theSurf) => theSurf.BSpline();

    public static gp_Ax1 AxeOfRevolution(Adaptor3d_Surface theSurf) => theSurf.AxeOfRevolution();

    public static gp_Dir Direction(Adaptor3d_Surface theSurf) => theSurf.Direction();

    public static Adaptor3d_Curve BasisCurve(Adaptor3d_Surface theSurf) => theSurf.BasisCurve();

    public static Adaptor3d_Surface BasisSurface(Adaptor3d_Surface theSurf) => theSurf.BasisSurface();

    public static double OffsetValue(Adaptor3d_Surface theSurf) => theSurf.OffsetValue();

    /// <summary>
    /// True if the surface is G1 along U (theAlongU) or V: tested on the B-spline curve behind it (an iso-curve of a
    /// B-spline surface, the basis curve of a swept surface in its direction, through an offset surface or curve);
    /// false for any other surface.
    /// </summary>
    public static bool IsSurfG1(Adaptor3d_Surface theSurf, bool theAlongU, double theAngTol = Precision_Angular)
    {
        double aUf, aUl, aVf, aVl;
        aUf = theSurf.FirstUParameter();
        aUl = theSurf.LastUParameter();
        aVf = theSurf.FirstVParameter();
        aVl = theSurf.LastVParameter();

        Adaptor3d_Surface aS = theSurf;
        Adaptor3d_Curve? aC = null;

        Geom_BSplineSurface aBS;
        Geom_BSplineCurve? aBC = null;

        if (aS.GetType() == GeomAbs_OffsetSurface)
        {
            aS = aS.BasisSurface();
        }

        if (aS.GetType() == GeomAbs_SurfaceOfRevolution || aS.GetType() == GeomAbs_SurfaceOfExtrusion)
        {
            aC = aS.BasisCurve();
        }

        if (aC is not null)
        {
            if (aC.GetType() == GeomAbs_OffsetCurve)
            {
                Geom_OffsetCurve aOC = aC.OffsetCurve();
                aC = new GeomAdaptor_Curve(aOC.BasisCurve());
            }

            if (aC.GetType() == GeomAbs_BSplineCurve)
            {
                if ((theAlongU && aS.GetType() == GeomAbs_SurfaceOfExtrusion)
                    || (!theAlongU && aS.GetType() == GeomAbs_SurfaceOfRevolution))
                {
                    aBC = aC.BSpline();
                }
            }
        }

        if (aS.GetType() == GeomAbs_BSplineSurface)
        {
            aBS = aS.BSpline();

            if (theAlongU)
            {
                double anIsoPar = (aVf + aVl) / 2.0;
                aBC = aBS.VIso(anIsoPar) as Geom_BSplineCurve;
            }
            else
            {
                double anIsoPar = (aUf + aUl) / 2.0;
                aBC = aBS.UIso(anIsoPar) as Geom_BSplineCurve;
            }
        }

        if (aBC is not null)
        {
            if (theAlongU)
            {
                return aBC.IsG1(aUf, aUl, theAngTol);
            }
            else
            {
                return aBC.IsG1(aVf, aVl, theAngTol);
            }
        }

        return false;
    }

    // Precision::Angular(), as a constant for the default argument (decision 129).
    private const double Precision_Angular = 1.0e-12;

    /// <summary>The number of U samples for the surface's type.</summary>
    public static int NbSamplesU(Adaptor3d_Surface S)
    {
        switch (S.GetType())
        {
            case GeomAbs_Plane:
                return 2;
            case GeomAbs_BezierSurface:
                return (3 + S.NbUPoles());
            case GeomAbs_BSplineSurface:
            {
                int nbs = S.NbUKnots() * S.UDegree();
                return (nbs < 2 ? 2 : nbs);
            }
            case GeomAbs_Torus:
                return 20;
            default:
                break;
        }
        return 10;
    }

    /// <summary>The number of V samples for the surface's type.</summary>
    public static int NbSamplesV(Adaptor3d_Surface S)
    {
        switch (S.GetType())
        {
            case GeomAbs_Plane:
                return 2;
            case GeomAbs_BezierSurface:
                return (3 + S.NbVPoles());
            case GeomAbs_BSplineSurface:
            {
                int nbs = S.NbVKnots() * S.VDegree();
                return (nbs < 2 ? 2 : nbs);
            }
            case GeomAbs_Cylinder:
            case GeomAbs_Cone:
            case GeomAbs_Sphere:
            case GeomAbs_Torus:
            case GeomAbs_SurfaceOfRevolution:
            case GeomAbs_SurfaceOfExtrusion:
                return 15;
            default:
                break;
        }
        return 10;
    }

    /// <summary>The number of U samples on [u1, u2], scaled from <see cref="NbSamplesU(Adaptor3d_Surface)"/>.</summary>
    public static int NbSamplesU(Adaptor3d_Surface S, double u1, double u2)
    {
        int nbs = NbSamplesU(S);
        int n = nbs;
        if (nbs > 10)
        {
            double uf = FirstUParameter(S);
            double ul = LastUParameter(S);
            n *= (int)((u2 - u1) / (ul - uf));
            if (n > nbs || n > 50)
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

    /// <summary>The number of V samples on [v1, v2], scaled from <see cref="NbSamplesV(Adaptor3d_Surface)"/>.</summary>
    public static int NbSamplesV(Adaptor3d_Surface S, double v1, double v2)
    {
        int nbs = NbSamplesV(S);
        int n = nbs;
        if (nbs > 10)
        {
            double vf = FirstVParameter(S);
            double vl = LastVParameter(S);
            n *= (int)((v2 - v1) / (vl - vf));
            if (n > nbs || n > 50)
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
