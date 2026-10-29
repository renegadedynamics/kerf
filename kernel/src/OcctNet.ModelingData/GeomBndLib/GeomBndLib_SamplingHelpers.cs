// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_SamplingHelpers.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_SurfaceType;

namespace Occt;

/// <summary>Sample counts for curves and surfaces (OCCT's namespace GeomBndLib_SamplingHelpers).</summary>
/// <remarks>
/// OCCT's template <c>ComputeNbSamplesT</c>, which only chooses between the 3D and 2D functions, is the pair of
/// overloads of the same name.
/// </remarks>
internal static class GeomBndLib_SamplingHelpers
{
    /// <summary>Compute number of samples for a 3D curve.</summary>
    internal static int ComputeNbSamples(Adaptor3d_Curve theCurve, double theUMin, double theUMax)
    {
        int N;
        GeomAbs_CurveType aType = theCurve.GetType();
        switch (aType)
        {
            case GeomAbs_BezierCurve:
            {
                N = 2 * theCurve.NbPoles();
                double du = theUMax - theUMin;
                if (du < .9)
                {
                    N = RealToInt(du * N) + 1;
                    N = Max(N, 5);
                }
                break;
            }
            case GeomAbs_BSplineCurve:
            {
                Geom_BSplineCurve aBC = theCurve.BSpline();
                N = 2 * (aBC.Degree() + 1) * (aBC.NbKnots() - 1);
                double umin = aBC.FirstParameter(), umax = aBC.LastParameter();
                double du = (theUMax - theUMin) / (umax - umin);
                if (du < .9)
                {
                    N = RealToInt(du * N) + 1;
                    N = Max(N, 5);
                }
                break;
            }
            default:
                N = 33;
                break;
        }
        return Min(500, N);
    }

    /// <summary>Compute number of samples for a 2D curve.</summary>
    internal static int ComputeNbSamples2d(Adaptor2d_Curve2d theCurve, double theUMin, double theUMax)
    {
        int N;
        GeomAbs_CurveType aType = theCurve.GetType();
        switch (aType)
        {
            case GeomAbs_BezierCurve:
            {
                N = 2 * theCurve.NbPoles();
                double du = theUMax - theUMin;
                if (du < .9)
                {
                    N = RealToInt(du * N) + 1;
                    N = Max(N, 5);
                }
                break;
            }
            case GeomAbs_BSplineCurve:
            {
                Geom2d_BSplineCurve aBC = theCurve.BSpline();
                N = 2 * (aBC.Degree() + 1) * (aBC.NbKnots() - 1);
                double umin = aBC.FirstParameter(), umax = aBC.LastParameter();
                double du = (theUMax - theUMin) / (umax - umin);
                if (du < .9)
                {
                    N = RealToInt(du * N) + 1;
                    N = Max(N, 5);
                }
                break;
            }
            default:
                N = 17;
                break;
        }
        return Min(23, N);
    }

    /// <summary>Compute number of U-samples for a surface.</summary>
    internal static int ComputeNbUSamples(Adaptor3d_Surface theSurf, double theUMin, double theUMax)
    {
        int N;
        GeomAbs_SurfaceType aType = theSurf.GetType();
        switch (aType)
        {
            case GeomAbs_BezierSurface:
            {
                N = 2 * theSurf.NbUPoles();
                double du = theUMax - theUMin;
                if (du < .9)
                {
                    N = RealToInt(du * N) + 1;
                    N = Max(N, 5);
                }
                break;
            }
            case GeomAbs_BSplineSurface:
            {
                Geom_BSplineSurface aBS = theSurf.BSpline();
                N = 2 * (aBS.UDegree() + 1) * (aBS.NbUKnots() - 1);
                aBS.Bounds(out double umin, out double umax, out _, out _);
                double du = (theUMax - theUMin) / (umax - umin);
                if (du < .9)
                {
                    N = RealToInt(du * N) + 1;
                    N = Max(N, 5);
                }
                break;
            }
            default:
                N = 33;
                break;
        }
        return Min(50, N);
    }

    /// <summary>Compute number of V-samples for a surface.</summary>
    internal static int ComputeNbVSamples(Adaptor3d_Surface theSurf, double theVMin, double theVMax)
    {
        int N;
        GeomAbs_SurfaceType aType = theSurf.GetType();
        switch (aType)
        {
            case GeomAbs_BezierSurface:
            {
                N = 2 * theSurf.NbVPoles();
                double dv = theVMax - theVMin;
                if (dv < .9)
                {
                    N = RealToInt(dv * N) + 1;
                    N = Max(N, 5);
                }
                break;
            }
            case GeomAbs_BSplineSurface:
            {
                Geom_BSplineSurface aBS = theSurf.BSpline();
                N = 2 * (aBS.VDegree() + 1) * (aBS.NbVKnots() - 1);
                aBS.Bounds(out _, out _, out double vmin, out double vmax);
                double dv = (theVMax - theVMin) / (vmax - vmin);
                if (dv < .9)
                {
                    N = RealToInt(dv * N) + 1;
                    N = Max(N, 5);
                }
                break;
            }
            default:
                N = 33;
                break;
        }
        return Min(50, N);
    }

    /// <summary>Compute number of U-samples for a surface (full range variant).</summary>
    internal static int ComputeNbUSamples(Adaptor3d_Surface theSurf)
    {
        int N;
        GeomAbs_SurfaceType aType = theSurf.GetType();
        switch (aType)
        {
            case GeomAbs_BezierSurface:
                N = 2 * theSurf.NbUPoles();
                break;
            case GeomAbs_BSplineSurface:
            {
                Geom_BSplineSurface aBS = theSurf.BSpline();
                N = 2 * (aBS.UDegree() + 1) * (aBS.NbUKnots() - 1);
                break;
            }
            default:
                N = 33;
                break;
        }
        return Min(50, N);
    }

    /// <summary>Compute number of V-samples for a surface (full range variant).</summary>
    internal static int ComputeNbVSamples(Adaptor3d_Surface theSurf)
    {
        int N;
        GeomAbs_SurfaceType aType = theSurf.GetType();
        switch (aType)
        {
            case GeomAbs_BezierSurface:
                N = 2 * theSurf.NbVPoles();
                break;
            case GeomAbs_BSplineSurface:
            {
                Geom_BSplineSurface aBS = theSurf.BSpline();
                N = 2 * (aBS.VDegree() + 1) * (aBS.NbVKnots() - 1);
                break;
            }
            default:
                N = 33;
                break;
        }
        return Min(50, N);
    }

    /// <summary>OCCT's <c>ComputeNbSamplesT</c> over a 3D curve.</summary>
    internal static int ComputeNbSamplesT(Adaptor3d_Curve theCurve, double theU1, double theU2)
        => ComputeNbSamples(theCurve, theU1, theU2);

    /// <summary>OCCT's <c>ComputeNbSamplesT</c> over a 2D curve.</summary>
    internal static int ComputeNbSamplesT(Adaptor2d_Curve2d theCurve, double theU1, double theU2)
        => ComputeNbSamples2d(theCurve, theU1, theU2);
}
