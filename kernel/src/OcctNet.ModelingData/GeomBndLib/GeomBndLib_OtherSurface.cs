// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_OtherSurface.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_OtherSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Computes bounding box for a general surface through its adaptor: grid sampling (GeomGridEval_Surface) for
/// <c>Box</c>, sampling + PSO/Powell numerical optimization for <c>BoxOptimal</c>.
/// </summary>
/// <remarks>Keeps a reference to the adaptor, as OCCT does (its <c>std::reference_wrapper</c>).</remarks>
public sealed class GeomBndLib_OtherSurface : GeomBndLib_Surface.ISurfaceEvaluator
{
    private readonly Adaptor3d_Surface mySurf;

    public GeomBndLib_OtherSurface(Adaptor3d_Surface theSurf)
    {
        mySurf = theSurf;
    }

    /// <summary>Compute bounding box for full surface.</summary>
    public Bnd_Box Box(double theTol)
        => Box(mySurf.FirstUParameter(), mySurf.LastUParameter(), mySurf.FirstVParameter(), mySurf.LastVParameter(), theTol);

    /// <summary>Compute bounding box for surface patch [theUMin, theUMax] x [theVMin, theVMax].</summary>
    public Bnd_Box Box(double theUMin, double theUMax, double theVMin, double theVMax, double theTol)
    {
        Bnd_Box aBox = new();
        Adaptor3d_Surface aSurf = mySurf;
        int Nu = GeomBndLib_SamplingHelpers.ComputeNbUSamples(aSurf, theUMin, theUMax);
        int Nv = GeomBndLib_SamplingHelpers.ComputeNbVSamples(aSurf, theVMin, theVMax);

        NCollection_Array1<double> aUParams = new(1, Nu);
        NCollection_Array1<double> aVParams = new(1, Nv);
        for (int i = 1; i <= Nu; i++)
        {
            aUParams.SetValue(i, theUMin + ((theUMax - theUMin) * (i - 1) / (Nu - 1)));
        }
        for (int j = 1; j <= Nv; j++)
        {
            aVParams.SetValue(j, theVMin + ((theVMax - theVMin) * (j - 1) / (Nv - 1)));
        }

        // OCCT samples the grid without enlarging the box by a deflection estimate (occt-issues.md).
        GeomGridEval_Surface anEvaluator = new(aSurf);
        NCollection_Array2<gp_Pnt> aGrid = anEvaluator.EvaluateGrid(aUParams, aVParams);
        for (int i = aGrid.LowerRow(); i <= aGrid.UpperRow(); i++)
        {
            for (int j = aGrid.LowerCol(); j <= aGrid.UpperCol(); j++)
            {
                aBox.Add(aGrid.Value(i, j));
            }
        }
        aBox.Enlarge(theTol);
        return aBox;
    }

    /// <summary>Compute precise bounding box for full surface.</summary>
    public Bnd_Box BoxOptimal(double theTol)
        => BoxOptimal(mySurf.FirstUParameter(), mySurf.LastUParameter(), mySurf.FirstVParameter(), mySurf.LastVParameter(), theTol);

    /// <summary>Compute precise bounding box using PSO + Powell optimization.</summary>
    public Bnd_Box BoxOptimal(double theUMin, double theUMax, double theVMin, double theVMax, double theTol)
    {
        Bnd_Box aBox = new();
        Adaptor3d_Surface aSurf = mySurf;
        int Nu = GeomBndLib_SamplingHelpers.ComputeNbUSamples(aSurf, theUMin, theUMax);
        int Nv = GeomBndLib_SamplingHelpers.ComputeNbVSamples(aSurf, theVMin, theVMax);

        Span<double> CoordMin = [RealLast(), RealLast(), RealLast()];
        Span<double> CoordMax = [-RealLast(), -RealLast(), -RealLast()];
        Span<double> DeflMax = [-RealLast(), -RealLast(), -RealLast()];

        double du = (theUMax - theUMin) / (Nu - 1);
        double du2 = du / 2.0;
        double dv = (theVMax - theVMin) / (Nv - 1);
        double dv2 = dv / 2.0;

        // Use batch grid evaluation with finer grid to include midpoints.
        int NuFine = 2 * Nu - 1;
        int NvFine = 2 * Nv - 1;
        NCollection_Array1<double> aUParams = new(1, NuFine);
        NCollection_Array1<double> aVParams = new(1, NvFine);
        for (int i = 1; i <= NuFine; i++)
        {
            aUParams.SetValue(i, theUMin + (i - 1) * du2);
        }
        for (int j = 1; j <= NvFine; j++)
        {
            aVParams.SetValue(j, theVMin + (j - 1) * dv2);
        }

        GeomGridEval_Surface anEvaluator = new(aSurf);
        NCollection_Array2<gp_Pnt> aFineGrid = anEvaluator.EvaluateGrid(aUParams, aVParams);

        NCollection_Array2<gp_XYZ> aPnts = new(1, Nu, 1, Nv);
        for (int i = 1; i <= Nu; i++)
        {
            int iFine = 2 * i - 1;
            for (int j = 1; j <= Nv; j++)
            {
                int jFine = 2 * j - 1;
                gp_Pnt P = aFineGrid.Value(iFine, jFine);
                aPnts[i, j] = P.XYZ();
                for (int k = 0; k < 3; ++k)
                {
                    if (CoordMin[k] > P.Coord(k + 1))
                    {
                        CoordMin[k] = P.Coord(k + 1);
                    }
                    if (CoordMax[k] < P.Coord(k + 1))
                    {
                        CoordMax[k] = P.Coord(k + 1);
                    }
                }
                // U-midpoint deflection.
                if (i > 1)
                {
                    gp_XYZ aPm = 0.5 * (aPnts[i - 1, j] + aPnts[i, j]);
                    gp_Pnt PM = aFineGrid.Value(iFine - 1, jFine);
                    gp_XYZ aD = (PM.XYZ() - aPm);
                    for (int k = 0; k < 3; ++k)
                    {
                        if (CoordMin[k] > PM.Coord(k + 1))
                        {
                            CoordMin[k] = PM.Coord(k + 1);
                        }
                        if (CoordMax[k] < PM.Coord(k + 1))
                        {
                            CoordMax[k] = PM.Coord(k + 1);
                        }
                        double d = Math.Abs(aD.Coord(k + 1));
                        if (DeflMax[k] < d)
                        {
                            DeflMax[k] = d;
                        }
                    }
                }
                // V-midpoint deflection.
                if (j > 1)
                {
                    gp_XYZ aPm = 0.5 * (aPnts[i, j - 1] + aPnts[i, j]);
                    gp_Pnt PM = aFineGrid.Value(iFine, jFine - 1);
                    gp_XYZ aD = (PM.XYZ() - aPm);
                    for (int k = 0; k < 3; ++k)
                    {
                        if (CoordMin[k] > PM.Coord(k + 1))
                        {
                            CoordMin[k] = PM.Coord(k + 1);
                        }
                        if (CoordMax[k] < PM.Coord(k + 1))
                        {
                            CoordMax[k] = PM.Coord(k + 1);
                        }
                        double d = Math.Abs(aD.Coord(k + 1));
                        if (DeflMax[k] < d)
                        {
                            DeflMax[k] = d;
                        }
                    }
                }
            }
        }

        // Adjust minmax using PSO + Powell optimization.
        double eps = Max(theTol, Precision.Confusion());
        for (int k = 0; k < 3; ++k)
        {
            double d = DeflMax[k];
            if (d <= eps)
            {
                continue;
            }
            double CMin = CoordMin[k];
            double CMax = CoordMax[k];
            for (int i = 1; i <= Nu; ++i)
            {
                for (int j = 1; j <= Nv; ++j)
                {
                    if (aPnts[i, j].Coord(k + 1) - CMin < d)
                    {
                        double umin = theUMin + Max(0, i - 2) * du;
                        double umax = theUMin + Min(Nu - 1, i) * du;
                        double vmin = theVMin + Max(0, j - 2) * dv;
                        double vmax = theVMin + Min(Nv - 1, j) * dv;
                        double cmin = GeomBndLib_OptimizationHelpers.AdjustExtrSurf(aSurf, umin, umax, vmin, vmax, CMin, k + 1, eps, true);
                        if (cmin < CMin)
                        {
                            CMin = cmin;
                        }
                    }
                    else if (CMax - aPnts[i, j].Coord(k + 1) < d)
                    {
                        double umin = theUMin + Max(0, i - 2) * du;
                        double umax = theUMin + Min(Nu - 1, i) * du;
                        double vmin = theVMin + Max(0, j - 2) * dv;
                        double vmax = theVMin + Min(Nv - 1, j) * dv;
                        double cmax = GeomBndLib_OptimizationHelpers.AdjustExtrSurf(aSurf, umin, umax, vmin, vmax, CMax, k + 1, eps, false);
                        if (cmax > CMax)
                        {
                            CMax = cmax;
                        }
                    }
                }
            }
            CoordMin[k] = CMin;
            CoordMax[k] = CMax;
        }

        aBox.Add(new gp_Pnt(CoordMin[0], CoordMin[1], CoordMin[2]));
        aBox.Add(new gp_Pnt(CoordMax[0], CoordMax[1], CoordMax[2]));
        aBox.Enlarge(eps);
        return aBox;
    }
}
