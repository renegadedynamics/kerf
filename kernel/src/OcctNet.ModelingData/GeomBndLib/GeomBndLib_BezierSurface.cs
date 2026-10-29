// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_BezierSurface.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_BezierSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Computes bounding box for a Bezier surface (Geom_BezierSurface): the poles' convex hull for the full surface, grid
/// sampling for trimmed patches, sampling + PSO/Powell optimization for <c>BoxOptimal</c>.
/// </summary>
public sealed class GeomBndLib_BezierSurface : GeomBndLib_Surface.ISurfaceEvaluator
{
    private readonly Geom_BezierSurface myGeom;

    public GeomBndLib_BezierSurface(Geom_BezierSurface theSurf)
    {
        myGeom = theSurf;
    }

    public Geom_BezierSurface Geometry() => myGeom;

    /// <summary>Compute bounding box for full surface.</summary>
    public Bnd_Box Box(double theTol)
    {
        Bnd_Box aBox = new();

        // For full Bezier surface, use convex hull of all poles.
        NCollection_Array2<gp_Pnt> aPoles = myGeom.InternalPoles();
        for (int anI = aPoles.LowerRow(); anI <= aPoles.UpperRow(); anI++)
        {
            for (int aJ = aPoles.LowerCol(); aJ <= aPoles.UpperCol(); aJ++)
            {
                aBox.Add(aPoles.Value(anI, aJ));
            }
        }
        aBox.Enlarge(theTol);
        return aBox;
    }

    /// <summary>Compute bounding box for surface patch [theUMin, theUMax] x [theVMin, theVMax].</summary>
    public Bnd_Box Box(double theUMin, double theUMax, double theVMin, double theVMax, double theTol)
    {
        Bnd_Box aBox = new();
        double PTol = Precision.Parametric(Precision.Confusion());

        // Check if the parameters match the full surface bounds.
        myGeom.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        if (Math.Abs(theUMin - aU1) <= PTol && Math.Abs(theVMin - aV1) <= PTol
            && Math.Abs(theUMax - aU2) <= PTol && Math.Abs(theVMax - aV2) <= PTol)
        {
            // Full surface: use convex hull of all poles.
            return Box(theTol);
        }

        // Trimmed Bezier: fall back to grid sampling.
        // OCCT samples the patch without enlarging the box by a deflection estimate (occt-issues.md).
        GeomAdaptor_Surface aGASurf = new(myGeom);
        int aNbUSamples = GeomBndLib_SamplingHelpers.ComputeNbUSamples(aGASurf);
        int aNbVSamples = GeomBndLib_SamplingHelpers.ComputeNbVSamples(aGASurf);

        NCollection_Array1<double> aUParams = new(1, aNbUSamples);
        NCollection_Array1<double> aVParams = new(1, aNbVSamples);
        for (int anI = 1; anI <= aNbUSamples; anI++)
        {
            aUParams.SetValue(anI, theUMin + ((theUMax - theUMin) * (anI - 1) / (aNbUSamples - 1)));
        }
        for (int aJ = 1; aJ <= aNbVSamples; aJ++)
        {
            aVParams.SetValue(aJ, theVMin + ((theVMax - theVMin) * (aJ - 1) / (aNbVSamples - 1)));
        }

        GeomGridEval_Surface anEvaluator = new(aGASurf);
        NCollection_Array2<gp_Pnt> aGrid = anEvaluator.EvaluateGrid(aUParams, aVParams);
        for (int anI = aGrid.LowerRow(); anI <= aGrid.UpperRow(); anI++)
        {
            for (int aJ = aGrid.LowerCol(); aJ <= aGrid.UpperCol(); aJ++)
            {
                aBox.Add(aGrid.Value(anI, aJ));
            }
        }
        aBox.Enlarge(theTol);
        return aBox;
    }

    /// <summary>Compute precise bounding box for full surface.</summary>
    public Bnd_Box BoxOptimal(double theTol)
    {
        myGeom.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        return BoxOptimal(aU1, aU2, aV1, aV2, theTol);
    }

    /// <summary>Compute precise bounding box using numerical optimization.</summary>
    public Bnd_Box BoxOptimal(double theUMin, double theUMax, double theVMin, double theVMax, double theTol)
    {
        GeomAdaptor_Surface aGASurf = new(myGeom);
        return SurfaceBoxOptimal(aGASurf, theUMin, theUMax, theVMin, theVMax, theTol);
    }

    /// <summary>
    /// The body of OCCT's GeomBndLib_BezierSurface::BoxOptimal after its adaptor is built; GeomBndLib_BSplineSurface's
    /// is the same text, and calls it.
    /// </summary>
    internal static Bnd_Box SurfaceBoxOptimal(GeomAdaptor_Surface aGASurf,
                                              double theUMin,
                                              double theUMax,
                                              double theVMin,
                                              double theVMax,
                                              double theTol)
    {
        Bnd_Box aBox = new();
        int aNbUSamples = GeomBndLib_SamplingHelpers.ComputeNbUSamples(aGASurf, theUMin, theUMax);
        int aNbVSamples = GeomBndLib_SamplingHelpers.ComputeNbVSamples(aGASurf, theVMin, theVMax);

        Span<double> aCoordMin = [RealLast(), RealLast(), RealLast()];
        Span<double> aCoordMax = [-RealLast(), -RealLast(), -RealLast()];
        Span<double> aDeflMax = [-RealLast(), -RealLast(), -RealLast()];

        double aDU = (theUMax - theUMin) / (aNbUSamples - 1);
        double aDU2 = aDU / 2.0;
        double aDV = (theVMax - theVMin) / (aNbVSamples - 1);
        double aDV2 = aDV / 2.0;

        // Use batch grid evaluation with finer grid to include midpoints.
        int aNbUSamplesFine = 2 * aNbUSamples - 1;
        int aNbVSamplesFine = 2 * aNbVSamples - 1;
        NCollection_Array1<double> aUParams = new(1, aNbUSamplesFine);
        NCollection_Array1<double> aVParams = new(1, aNbVSamplesFine);
        for (int anI = 1; anI <= aNbUSamplesFine; anI++)
        {
            aUParams.SetValue(anI, theUMin + (anI - 1) * aDU2);
        }
        for (int aJ = 1; aJ <= aNbVSamplesFine; aJ++)
        {
            aVParams.SetValue(aJ, theVMin + (aJ - 1) * aDV2);
        }

        GeomGridEval_Surface anEvaluator = new(aGASurf);
        NCollection_Array2<gp_Pnt> aFineGrid = anEvaluator.EvaluateGrid(aUParams, aVParams);

        NCollection_Array2<gp_XYZ> aPnts = new(1, aNbUSamples, 1, aNbVSamples);
        for (int anI = 1; anI <= aNbUSamples; anI++)
        {
            int anIFine = 2 * anI - 1;
            for (int aJ = 1; aJ <= aNbVSamples; aJ++)
            {
                int aJFine = 2 * aJ - 1;
                gp_Pnt aP = aFineGrid.Value(anIFine, aJFine);
                aPnts[anI, aJ] = aP.XYZ();
                for (int aK = 0; aK < 3; ++aK)
                {
                    if (aCoordMin[aK] > aP.Coord(aK + 1))
                    {
                        aCoordMin[aK] = aP.Coord(aK + 1);
                    }
                    if (aCoordMax[aK] < aP.Coord(aK + 1))
                    {
                        aCoordMax[aK] = aP.Coord(aK + 1);
                    }
                }
                // U-midpoint deflection.
                if (anI > 1)
                {
                    gp_XYZ aPm = 0.5 * (aPnts[anI - 1, aJ] + aPnts[anI, aJ]);
                    gp_Pnt aPM = aFineGrid.Value(anIFine - 1, aJFine);
                    gp_XYZ aD = (aPM.XYZ() - aPm);
                    for (int aK = 0; aK < 3; ++aK)
                    {
                        if (aCoordMin[aK] > aPM.Coord(aK + 1))
                        {
                            aCoordMin[aK] = aPM.Coord(aK + 1);
                        }
                        if (aCoordMax[aK] < aPM.Coord(aK + 1))
                        {
                            aCoordMax[aK] = aPM.Coord(aK + 1);
                        }
                        double aDiff = Math.Abs(aD.Coord(aK + 1));
                        if (aDeflMax[aK] < aDiff)
                        {
                            aDeflMax[aK] = aDiff;
                        }
                    }
                }
                // V-midpoint deflection.
                if (aJ > 1)
                {
                    gp_XYZ aPm = 0.5 * (aPnts[anI, aJ - 1] + aPnts[anI, aJ]);
                    gp_Pnt aPM = aFineGrid.Value(anIFine, aJFine - 1);
                    gp_XYZ aD = (aPM.XYZ() - aPm);
                    for (int aK = 0; aK < 3; ++aK)
                    {
                        if (aCoordMin[aK] > aPM.Coord(aK + 1))
                        {
                            aCoordMin[aK] = aPM.Coord(aK + 1);
                        }
                        if (aCoordMax[aK] < aPM.Coord(aK + 1))
                        {
                            aCoordMax[aK] = aPM.Coord(aK + 1);
                        }
                        double aDiff = Math.Abs(aD.Coord(aK + 1));
                        if (aDeflMax[aK] < aDiff)
                        {
                            aDeflMax[aK] = aDiff;
                        }
                    }
                }
            }
        }

        // Adjust minmax using PSO + Powell optimization.
        double anEps = Max(theTol, Precision.Confusion());
        for (int aK = 0; aK < 3; ++aK)
        {
            double aDiff = aDeflMax[aK];
            if (aDiff <= anEps)
            {
                continue;
            }
            double aCMin = aCoordMin[aK];
            double aCMax = aCoordMax[aK];
            for (int anI = 1; anI <= aNbUSamples; ++anI)
            {
                for (int aJ = 1; aJ <= aNbVSamples; ++aJ)
                {
                    if (aPnts[anI, aJ].Coord(aK + 1) - aCMin < aDiff)
                    {
                        double anUMin = theUMin + Max(0, anI - 2) * aDU;
                        double anUMax = theUMin + Min(aNbUSamples - 1, anI) * aDU;
                        double aVMin = theVMin + Max(0, aJ - 2) * aDV;
                        double aVMax = theVMin + Min(aNbVSamples - 1, aJ) * aDV;
                        double aLocMin = GeomBndLib_OptimizationHelpers.AdjustExtrSurf(aGASurf, anUMin, anUMax, aVMin, aVMax, aCMin, aK + 1, anEps, true);
                        if (aLocMin < aCMin)
                        {
                            aCMin = aLocMin;
                        }
                    }
                    else if (aCMax - aPnts[anI, aJ].Coord(aK + 1) < aDiff)
                    {
                        double anUMin = theUMin + Max(0, anI - 2) * aDU;
                        double anUMax = theUMin + Min(aNbUSamples - 1, anI) * aDU;
                        double aVMin = theVMin + Max(0, aJ - 2) * aDV;
                        double aVMax = theVMin + Min(aNbVSamples - 1, aJ) * aDV;
                        double aLocMax = GeomBndLib_OptimizationHelpers.AdjustExtrSurf(aGASurf, anUMin, anUMax, aVMin, aVMax, aCMax, aK + 1, anEps, false);
                        if (aLocMax > aCMax)
                        {
                            aCMax = aLocMax;
                        }
                    }
                }
            }
            aCoordMin[aK] = aCMin;
            aCoordMax[aK] = aCMax;
        }

        aBox.Add(new gp_Pnt(aCoordMin[0], aCoordMin[1], aCoordMin[2]));
        aBox.Add(new gp_Pnt(aCoordMax[0], aCoordMax[1], aCoordMax[2]));
        aBox.Enlarge(anEps);
        return aBox;
    }
}
