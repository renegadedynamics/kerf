// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_OtherSurface.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_OtherSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;

using Kerf.Engine.Geometry.Evaluation.Grid;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Bounds.Computation;

/// <summary>
/// Computes bounding box for a general surface through its adaptor: grid sampling (GeomGridEval_Surface) for
/// <c>Box</c>, sampling + PSO/Powell numerical optimization for <c>BoxOptimal</c>.
/// </summary>
/// <remarks>Keeps a reference to the adaptor, as OCCT does (its <c>std::reference_wrapper</c>).</remarks>
public sealed class OtherSurfaceBounds : SurfaceBounds.ISurfaceEvaluator
{
    private readonly SurfaceAdaptorBase _surf;

    public OtherSurfaceBounds(SurfaceAdaptorBase surf)
    {
        _surf = surf;
    }

    /// <summary>Compute bounding box for full surface.</summary>
    public Box Box(double tol)
        => Box(_surf.FirstUParameter(), _surf.LastUParameter(), _surf.FirstVParameter(), _surf.LastVParameter(), tol);

    /// <summary>Compute bounding box for surface patch [theUMin, theUMax] x [theVMin, theVMax].</summary>
    public Box Box(double uMin, double uMax, double vMin, double vMax, double tol)
    {
        Box box = new();
        SurfaceAdaptorBase surf = _surf;
        int nu = SamplingHelpers.ComputeNbUSamples(surf, uMin, uMax);
        int nv = SamplingHelpers.ComputeNbVSamples(surf, vMin, vMax);

        NCollection_Array1<double> uParams = new(1, nu);
        NCollection_Array1<double> vParams = new(1, nv);
        for (int i = 1; i <= nu; i++)
        {
            uParams.SetValue(i, uMin + ((uMax - uMin) * (i - 1) / (nu - 1)));
        }
        for (int j = 1; j <= nv; j++)
        {
            vParams.SetValue(j, vMin + ((vMax - vMin) * (j - 1) / (nv - 1)));
        }

        // OCCT samples the grid without enlarging the box by a deflection estimate (occt-issues.md).
        SurfaceGrid evaluator = new(surf);
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
        => BoxOptimal(_surf.FirstUParameter(), _surf.LastUParameter(), _surf.FirstVParameter(), _surf.LastVParameter(), tol);

    /// <summary>Compute precise bounding box using PSO + Powell optimization.</summary>
    public Box BoxOptimal(double uMin, double uMax, double vMin, double vMax, double tol)
    {
        Box box = new();
        SurfaceAdaptorBase surf = _surf;
        int nu = SamplingHelpers.ComputeNbUSamples(surf, uMin, uMax);
        int nv = SamplingHelpers.ComputeNbVSamples(surf, vMin, vMax);

        Span<double> coordMin = [RealLast(), RealLast(), RealLast()];
        Span<double> coordMax = [-RealLast(), -RealLast(), -RealLast()];
        Span<double> deflMax = [-RealLast(), -RealLast(), -RealLast()];

        double du = (uMax - uMin) / (nu - 1);
        double du2 = du / 2.0;
        double dv = (vMax - vMin) / (nv - 1);
        double dv2 = dv / 2.0;

        // Use batch grid evaluation with finer grid to include midpoints.
        int nuFine = 2 * nu - 1;
        int nvFine = 2 * nv - 1;
        NCollection_Array1<double> uParams = new(1, nuFine);
        NCollection_Array1<double> vParams = new(1, nvFine);
        for (int i = 1; i <= nuFine; i++)
        {
            uParams.SetValue(i, uMin + (i - 1) * du2);
        }
        for (int j = 1; j <= nvFine; j++)
        {
            vParams.SetValue(j, vMin + (j - 1) * dv2);
        }

        SurfaceGrid evaluator = new(surf);
        NCollection_Array2<Point3D> fineGrid = evaluator.EvaluateGrid(uParams, vParams);

        NCollection_Array2<Xyz> pnts = new(1, nu, 1, nv);
        for (int i = 1; i <= nu; i++)
        {
            int iFine = 2 * i - 1;
            for (int j = 1; j <= nv; j++)
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
                    Point3D PM = fineGrid.Value(iFine - 1, jFine);
                    Xyz aD = (PM.XYZ() - aPm);
                    for (int k = 0; k < 3; ++k)
                    {
                        if (coordMin[k] > PM.Coord(k + 1))
                        {
                            coordMin[k] = PM.Coord(k + 1);
                        }
                        if (coordMax[k] < PM.Coord(k + 1))
                        {
                            coordMax[k] = PM.Coord(k + 1);
                        }
                        double d = Math.Abs(aD.Coord(k + 1));
                        if (deflMax[k] < d)
                        {
                            deflMax[k] = d;
                        }
                    }
                }
                // V-midpoint deflection.
                if (j > 1)
                {
                    Xyz aPm = 0.5 * (pnts[i, j - 1] + pnts[i, j]);
                    Point3D pm = fineGrid.Value(iFine, jFine - 1);
                    Xyz aD = (pm.XYZ() - aPm);
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
                        double d = Math.Abs(aD.Coord(k + 1));
                        if (deflMax[k] < d)
                        {
                            deflMax[k] = d;
                        }
                    }
                }
            }
        }

        // Adjust minmax using PSO + Powell optimization.
        double eps = Max(tol, global::Kerf.Engine.Foundation.Precision.Confusion());
        for (int k = 0; k < 3; ++k)
        {
            double d = deflMax[k];
            if (d <= eps)
            {
                continue;
            }
            double cMin = coordMin[k];
            double cMax = coordMax[k];
            for (int i = 1; i <= nu; ++i)
            {
                for (int j = 1; j <= nv; ++j)
                {
                    if (pnts[i, j].Coord(k + 1) - cMin < d)
                    {
                        double umin = uMin + Max(0, i - 2) * du;
                        double umax = uMin + Min(nu - 1, i) * du;
                        double vmin = vMin + Max(0, j - 2) * dv;
                        double vmax = vMin + Min(nv - 1, j) * dv;
                        double cmin = OptimizationHelpers.AdjustExtrSurf(surf, umin, umax, vmin, vmax, cMin, k + 1, eps, true);
                        if (cmin < cMin)
                        {
                            cMin = cmin;
                        }
                    }
                    else if (cMax - pnts[i, j].Coord(k + 1) < d)
                    {
                        double umin = uMin + Max(0, i - 2) * du;
                        double umax = uMin + Min(nu - 1, i) * du;
                        double vmin = vMin + Max(0, j - 2) * dv;
                        double vmax = vMin + Min(nv - 1, j) * dv;
                        double cmax = OptimizationHelpers.AdjustExtrSurf(surf, umin, umax, vmin, vmax, cMax, k + 1, eps, false);
                        if (cmax > cMax)
                        {
                            cMax = cmax;
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
