// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_SamplingHelpers.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Geometry.Bounds.Computation;

/// <summary>Sample counts for curves and surfaces (OCCT's namespace GeomBndLib_SamplingHelpers).</summary>
/// <remarks>
/// OCCT's template <c>ComputeNbSamplesT</c>, which only chooses between the 3D and 2D functions, is the pair of
/// overloads of the same name.
/// </remarks>
internal static class SamplingHelpers
{
    /// <summary>Compute number of samples for a 3D curve.</summary>
    internal static int ComputeNbSamples(CurveAdaptorBase curve, double uMin, double uMax)
    {
        int n;
        CurveKind type = curve.GetType();
        switch (type)
        {
            case BezierCurve:
            {
                n = 2 * curve.PoleCount;
                double du = uMax - uMin;
                if (du < .9)
                {
                    n = RealToInt(du * n) + 1;
                    n = Max(n, 5);
                }
                break;
            }
            case BSplineCurve:
            {
                BSplineCurve3D bc = curve.BSpline();
                n = 2 * (bc.Degree + 1) * (bc.KnotCount - 1);
                double umin = bc.FirstParameter, umax = bc.LastParameter;
                double du = (uMax - uMin) / (umax - umin);
                if (du < .9)
                {
                    n = RealToInt(du * n) + 1;
                    n = Max(n, 5);
                }
                break;
            }
            default:
                n = 33;
                break;
        }
        return Min(500, n);
    }

    /// <summary>Compute number of samples for a 2D curve.</summary>
    internal static int ComputeNbSamples2d(Curve2DAdaptorBase curve, double uMin, double uMax)
    {
        int n;
        CurveKind type = curve.GetType();
        switch (type)
        {
            case BezierCurve:
            {
                n = 2 * curve.PoleCount;
                double du = uMax - uMin;
                if (du < .9)
                {
                    n = RealToInt(du * n) + 1;
                    n = Max(n, 5);
                }
                break;
            }
            case BSplineCurve:
            {
                BSplineCurve2D bc = curve.BSpline();
                n = 2 * (bc.Degree + 1) * (bc.KnotCount - 1);
                double umin = bc.FirstParameter, umax = bc.LastParameter;
                double du = (uMax - uMin) / (umax - umin);
                if (du < .9)
                {
                    n = RealToInt(du * n) + 1;
                    n = Max(n, 5);
                }
                break;
            }
            default:
                n = 17;
                break;
        }
        return Min(23, n);
    }

    /// <summary>Compute number of U-samples for a surface.</summary>
    internal static void AddCellCentres(NCollection_Array2<Point3D> fineGrid,
                                        NCollection_Array2<Xyz> pnts,
                                        int nbUSamples,
                                        int nbVSamples,
                                        Span<double> coordMin,
                                        Span<double> coordMax,
                                        Span<double> deflMax)
    {
        // Kerf: OCCT looks at each cell's corners and edges' midpoints only, so an extreme at a cell's centre is out of
        // reach of its search (OCCT #1604, decision 20203)
        for (int i = 1; i < nbUSamples; ++i)
        {
            for (int j = 1; j < nbVSamples; ++j)
            {
                Point3D centre = fineGrid.Value(2 * i, 2 * j);
                Xyz mean = 0.25 * (pnts[i, j] + pnts[i + 1, j] + pnts[i, j + 1] + pnts[i + 1, j + 1]);
                for (int k = 0; k < 3; ++k)
                {
                    double c = centre.Coord(k + 1);
                    coordMin[k] = Min(coordMin[k], c);
                    coordMax[k] = Max(coordMax[k], c);
                    deflMax[k] = Max(deflMax[k], Math.Abs(c - mean.Coord(k + 1)));
                }
            }
        }
    }

    /// <summary>
    /// The lowest and highest of a coordinate over a sample and the fine grid's points around it: what says whether the
    /// search should look near the sample. The sample alone can lie further from an extreme than the deflection.
    /// </summary>
    internal static void NeighbourhoodRange(NCollection_Array2<Point3D> fineGrid,
                                            int nbUSamples,
                                            int nbVSamples,
                                            int i,
                                            int j,
                                            int coord,
                                            out double lowest,
                                            out double highest)
    {
        lowest = double.PositiveInfinity;
        highest = double.NegativeInfinity;
        int iFine = 2 * i - 1, jFine = 2 * j - 1;
        int iLast = Min(iFine + 1, 2 * nbUSamples - 1), jLast = Min(jFine + 1, 2 * nbVSamples - 1);
        for (int a = Max(iFine - 1, 1); a <= iLast; ++a)
        {
            for (int b = Max(jFine - 1, 1); b <= jLast; ++b)
            {
                double c = fineGrid.Value(a, b).Coord(coord);
                lowest = Min(lowest, c);
                highest = Max(highest, c);
            }
        }
    }

    internal static int ComputeNbUSamples(SurfaceAdaptorBase surf, double uMin, double uMax)
    {
        int n;
        SurfaceKind type = surf.GetType();
        switch (type)
        {
            case SurfaceKind.BezierSurface:
            {
                n = 2 * surf.UPoleCount;
                double du = uMax - uMin;
                if (du < .9)
                {
                    n = RealToInt(du * n) + 1;
                    n = Max(n, 5);
                }
                break;
            }
            case SurfaceKind.BSplineSurface:
            {
                global::Kerf.Geometry.BSplineSurface bs = surf.BSpline();
                n = 2 * (bs.UDegree + 1) * (bs.UKnotCount - 1);
                bs.ParameterBounds(out double umin, out double umax, out _, out _);
                double du = (uMax - uMin) / (umax - umin);
                if (du < .9)
                {
                    n = RealToInt(du * n) + 1;
                    n = Max(n, 5);
                }
                break;
            }
            default:
                n = 33;
                break;
        }
        return Min(50, n);
    }

    /// <summary>Compute number of V-samples for a surface.</summary>
    internal static int ComputeNbVSamples(SurfaceAdaptorBase surf, double vMin, double vMax)
    {
        int n;
        SurfaceKind type = surf.GetType();
        switch (type)
        {
            case SurfaceKind.BezierSurface:
            {
                n = 2 * surf.VPoleCount;
                double dv = vMax - vMin;
                if (dv < .9)
                {
                    n = RealToInt(dv * n) + 1;
                    n = Max(n, 5);
                }
                break;
            }
            case SurfaceKind.BSplineSurface:
            {
                global::Kerf.Geometry.BSplineSurface bs = surf.BSpline();
                n = 2 * (bs.VDegree + 1) * (bs.VKnotCount - 1);
                bs.ParameterBounds(out _, out _, out double vmin, out double vmax);
                double dv = (vMax - vMin) / (vmax - vmin);
                if (dv < .9)
                {
                    n = RealToInt(dv * n) + 1;
                    n = Max(n, 5);
                }
                break;
            }
            default:
                n = 33;
                break;
        }
        return Min(50, n);
    }

    /// <summary>Compute number of U-samples for a surface (full range variant).</summary>
    internal static int ComputeNbUSamples(SurfaceAdaptorBase surf)
    {
        int n;
        SurfaceKind type = surf.GetType();
        switch (type)
        {
            case SurfaceKind.BezierSurface:
                n = 2 * surf.UPoleCount;
                break;
            case SurfaceKind.BSplineSurface:
            {
                global::Kerf.Geometry.BSplineSurface bs = surf.BSpline();
                n = 2 * (bs.UDegree + 1) * (bs.UKnotCount - 1);
                break;
            }
            default:
                n = 33;
                break;
        }
        return Min(50, n);
    }

    /// <summary>Compute number of V-samples for a surface (full range variant).</summary>
    internal static int ComputeNbVSamples(SurfaceAdaptorBase surf)
    {
        int n;
        SurfaceKind type = surf.GetType();
        switch (type)
        {
            case SurfaceKind.BezierSurface:
                n = 2 * surf.VPoleCount;
                break;
            case SurfaceKind.BSplineSurface:
            {
                global::Kerf.Geometry.BSplineSurface bs = surf.BSpline();
                n = 2 * (bs.VDegree + 1) * (bs.VKnotCount - 1);
                break;
            }
            default:
                n = 33;
                break;
        }
        return Min(50, n);
    }

    /// <summary>OCCT's <c>ComputeNbSamplesT</c> over a 3D curve.</summary>
    internal static int ComputeNbSamplesT(CurveAdaptorBase curve, double u1, double u2)
        => ComputeNbSamples(curve, u1, u2);

    /// <summary>OCCT's <c>ComputeNbSamplesT</c> over a 2D curve.</summary>
    internal static int ComputeNbSamplesT(Curve2DAdaptorBase curve, double u1, double u2)
        => ComputeNbSamples2d(curve, u1, u2);
}
