// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_BezierSurface.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_BezierSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;

using Kerf.Engine.Geometry.Evaluation.Grid;
using Kerf.Geometry;

namespace Kerf.Engine.Geometry.Bounds.Computation;

/// <summary>
/// Computes bounding box for a Bezier surface (Geom_BezierSurface): the poles' convex hull for the full surface, grid
/// sampling for trimmed patches, sampling + PSO/Powell optimization for <c>BoxOptimal</c>.
/// </summary>
internal sealed class BezierSurfaceBounds : SurfaceBounds.ISurfaceEvaluator
{
    private readonly BezierSurface _geom;

    public BezierSurfaceBounds(BezierSurface surf)
    {
        _geom = surf;
    }

    public BezierSurface Geometry() => _geom;

    /// <summary>Compute bounding box for full surface.</summary>
    public Box Box(double tol)
    {
        Box box = new();

        // For full Bezier surface, use convex hull of all poles.
        NCollection_Array2<Point3D> poles = _geom.InternalPoles();
        for (int i = poles.LowerRow(); i <= poles.UpperRow(); i++)
        {
            for (int j = poles.LowerCol(); j <= poles.UpperCol(); j++)
            {
                box.Add(poles.Value(i, j));
            }
        }
        box.Enlarge(tol);
        return box;
    }

    /// <summary>Compute bounding box for surface patch [theUMin, theUMax] x [theVMin, theVMax].</summary>
    public Box Box(double uMin, double uMax, double vMin, double vMax, double tol)
    {
        Box box = new();
        double pTol = global::Kerf.Engine.Foundation.Precision.Parametric(global::Kerf.Engine.Foundation.Precision.Confusion());

        // Check if the parameters match the full surface bounds.
        _geom.ParameterBounds(out double u1, out double u2, out double v1, out double v2);
        if (Math.Abs(uMin - u1) <= pTol && Math.Abs(vMin - v1) <= pTol
            && Math.Abs(uMax - u2) <= pTol && Math.Abs(vMax - v2) <= pTol)
        {
            // Full surface: use convex hull of all poles.
            return Box(tol);
        }

        // Trimmed Bezier: fall back to grid sampling.
        // OCCT samples the patch without enlarging the box by a deflection estimate (occt-issues.md).
        SurfaceAdaptor gaSurf = new(_geom);
        int nbUSamples = SamplingHelpers.ComputeNbUSamples(gaSurf);
        int nbVSamples = SamplingHelpers.ComputeNbVSamples(gaSurf);

        NCollection_Array1<double> uParams = new(1, nbUSamples);
        NCollection_Array1<double> vParams = new(1, nbVSamples);
        for (int i = 1; i <= nbUSamples; i++)
        {
            uParams.SetValue(i, uMin + ((uMax - uMin) * (i - 1) / (nbUSamples - 1)));
        }
        for (int j = 1; j <= nbVSamples; j++)
        {
            vParams.SetValue(j, vMin + ((vMax - vMin) * (j - 1) / (nbVSamples - 1)));
        }

        SurfaceGrid evaluator = new(gaSurf);
        NCollection_Array2<Point3D> grid = evaluator.EvaluateGrid(uParams, vParams);
        for (int i = grid.LowerRow(); i <= grid.UpperRow(); i++)
        {
            for (int j = grid.LowerCol(); j <= grid.UpperCol(); j++)
            {
                box.Add(grid.Value(i, j));
            }
        }
        box.Enlarge(tol);
        return box;
    }

    /// <summary>Compute precise bounding box for full surface.</summary>
    public Box BoxOptimal(double tol)
    {
        _geom.ParameterBounds(out double u1, out double u2, out double v1, out double v2);
        return BoxOptimal(u1, u2, v1, v2, tol);
    }

    /// <summary>Compute precise bounding box using numerical optimization.</summary>
    public Box BoxOptimal(double uMin, double uMax, double vMin, double vMax, double tol)
    {
        SurfaceAdaptor gaSurf = new(_geom);
        return SurfaceBoxOptimal(gaSurf, uMin, uMax, vMin, vMax, tol);
    }

    /// <summary>
    /// The body of OCCT's GeomBndLib_BezierSurface::BoxOptimal after its adaptor is built; GeomBndLib_BSplineSurface's
    /// is the same text, and calls it.
    /// </summary>
    internal static Box SurfaceBoxOptimal(SurfaceAdaptor aGASurf,
                                              double uMin,
                                              double uMax,
                                              double vMin,
                                              double vMax,
                                              double tol)
    {
        Box box = new();
        int nbUSamples = SamplingHelpers.ComputeNbUSamples(aGASurf, uMin, uMax);
        int nbVSamples = SamplingHelpers.ComputeNbVSamples(aGASurf, vMin, vMax);

        Span<double> coordMin = [RealLast(), RealLast(), RealLast()];
        Span<double> coordMax = [-RealLast(), -RealLast(), -RealLast()];
        Span<double> deflMax = [-RealLast(), -RealLast(), -RealLast()];

        double du = (uMax - uMin) / (nbUSamples - 1);
        double du2 = du / 2.0;
        double dv = (vMax - vMin) / (nbVSamples - 1);
        double dv2 = dv / 2.0;

        // Use batch grid evaluation with finer grid to include midpoints.
        int nbUSamplesFine = 2 * nbUSamples - 1;
        int nbVSamplesFine = 2 * nbVSamples - 1;
        NCollection_Array1<double> uParams = new(1, nbUSamplesFine);
        NCollection_Array1<double> vParams = new(1, nbVSamplesFine);
        for (int i = 1; i <= nbUSamplesFine; i++)
        {
            uParams.SetValue(i, uMin + (i - 1) * du2);
        }
        for (int j = 1; j <= nbVSamplesFine; j++)
        {
            vParams.SetValue(j, vMin + (j - 1) * dv2);
        }

        SurfaceGrid evaluator = new(aGASurf);
        NCollection_Array2<Point3D> fineGrid = evaluator.EvaluateGrid(uParams, vParams);

        NCollection_Array2<Xyz> pnts = new(1, nbUSamples, 1, nbVSamples);
        for (int i = 1; i <= nbUSamples; i++)
        {
            int iFine = 2 * i - 1;
            for (int j = 1; j <= nbVSamples; j++)
            {
                int jFine = 2 * j - 1;
                Point3D p = fineGrid.Value(iFine, jFine);
                pnts[i, j] = p.XYZ();
                for (int k = 0; k < 3; ++k)
                {
                    if (coordMin[k] > p.Coord(k + 1))
                    {
                        coordMin[k] = p.Coord(k + 1);
                    }
                    if (coordMax[k] < p.Coord(k + 1))
                    {
                        coordMax[k] = p.Coord(k + 1);
                    }
                }
                // U-midpoint deflection.
                if (i > 1)
                {
                    Xyz aPm = 0.5 * (pnts[i - 1, j] + pnts[i, j]);
                    Point3D aPM = fineGrid.Value(iFine - 1, jFine);
                    Xyz d = (aPM.XYZ() - aPm);
                    for (int k = 0; k < 3; ++k)
                    {
                        if (coordMin[k] > aPM.Coord(k + 1))
                        {
                            coordMin[k] = aPM.Coord(k + 1);
                        }
                        if (coordMax[k] < aPM.Coord(k + 1))
                        {
                            coordMax[k] = aPM.Coord(k + 1);
                        }
                        double diff = Math.Abs(d.Coord(k + 1));
                        if (deflMax[k] < diff)
                        {
                            deflMax[k] = diff;
                        }
                    }
                }
                // V-midpoint deflection.
                if (j > 1)
                {
                    Xyz aPm = 0.5 * (pnts[i, j - 1] + pnts[i, j]);
                    Point3D pm = fineGrid.Value(iFine, jFine - 1);
                    Xyz d = (pm.XYZ() - aPm);
                    for (int k = 0; k < 3; ++k)
                    {
                        if (coordMin[k] > pm.Coord(k + 1))
                        {
                            coordMin[k] = pm.Coord(k + 1);
                        }
                        if (coordMax[k] < pm.Coord(k + 1))
                        {
                            coordMax[k] = pm.Coord(k + 1);
                        }
                        double diff = Math.Abs(d.Coord(k + 1));
                        if (deflMax[k] < diff)
                        {
                            deflMax[k] = diff;
                        }
                    }
                }
            }
        }

        SamplingHelpers.AddCellCentres(fineGrid, pnts, nbUSamples, nbVSamples, coordMin, coordMax, deflMax);

        // Adjust minmax using PSO + Powell optimization.
        global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOSwarm keptSwarm = new();
        double eps = Max(tol, global::Kerf.Engine.Foundation.Precision.Confusion());
        for (int k = 0; k < 3; ++k)
        {
            double diff = deflMax[k];
            if (diff <= eps)
            {
                continue;
            }
            double cMin = coordMin[k];
            double cMax = coordMax[k];
            for (int i = 1; i <= nbUSamples; ++i)
            {
                for (int j = 1; j <= nbVSamples; ++j)
                {
                    SamplingHelpers.NeighbourhoodRange(fineGrid, nbUSamples, nbVSamples, i, j, k + 1, out double reachMin, out double reachMax);
                    if (reachMin - cMin < diff)
                    {
                        double anUMin = uMin + Max(0, i - 2) * du;
                        double anUMax = uMin + Min(nbUSamples - 1, i) * du;
                        double aVMin = vMin + Max(0, j - 2) * dv;
                        double aVMax = vMin + Min(nbVSamples - 1, j) * dv;
                        double locMin = OptimizationHelpers.AdjustExtrSurf(aGASurf, anUMin, anUMax, aVMin, aVMax, cMin, k + 1, eps, true, keptSwarm);
                        if (locMin < cMin)
                        {
                            cMin = locMin;
                        }
                    }
                    else if (cMax - reachMax < diff)
                    {
                        double anUMin = uMin + Max(0, i - 2) * du;
                        double anUMax = uMin + Min(nbUSamples - 1, i) * du;
                        double aVMin = vMin + Max(0, j - 2) * dv;
                        double aVMax = vMin + Min(nbVSamples - 1, j) * dv;
                        double locMax = OptimizationHelpers.AdjustExtrSurf(aGASurf, anUMin, anUMax, aVMin, aVMax, cMax, k + 1, eps, false, keptSwarm);
                        if (locMax > cMax)
                        {
                            cMax = locMax;
                        }
                    }
                }
            }
            coordMin[k] = cMin;
            coordMax[k] = cMax;
        }

        box.Add(new Point3D(coordMin[0], coordMin[1], coordMin[2]));
        box.Add(new Point3D(coordMax[0], coordMax[1], coordMax[2]));
        box.Enlarge(eps);
        return box;
    }
}
