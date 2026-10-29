// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_SplineHelpers.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Sampling and pole-hull helpers for spline bounding boxes (OCCT's namespace GeomBndLib_SplineHelpers).</summary>
/// <remarks>
/// OCCT's templates are ported once per instantiation it makes (rules, section 9): <c>FillBox</c> over
/// (Bnd_Box, GeomAdaptor_Curve, gp_Pnt), <c>ReduceSplineBox</c> over (gp_Pnt, Bnd_Box), and <c>CurveBoxOptimal</c>
/// over (GeomAdaptor_Curve, Bnd_Box, gp_Pnt) and (Geom2dAdaptor_Curve, Bnd_Box2d, gp_Pnt2d). The templates
/// <c>BezierCurveBox</c>, <c>BSplineCurveBox</c> and the pole-range overload of <c>ReduceSplineBox</c>, and the
/// function <c>ComputePoleIndexRange</c> that only they call, are never instantiated or called in OCCT V8_0_1 and are
/// not ported; <c>PointOps</c> is written out where it is used.
/// </remarks>
internal static class GeomBndLib_SplineHelpers
{
    /// <summary>Fill box with sampled points and return maximum deflection.</summary>
    internal static double FillBox(ref Bnd_Box theBox, GeomAdaptor_Curve theCurve, double theFirst, double theLast, int theN)
    {
        theBox.Add(theCurve.EvalD0(theFirst));
        double aP = theFirst;
        double aTol = 0.0;
        double aDP = theLast - theFirst;
        if (Math.Abs(aDP) > Precision.PConfusion())
        {
            gp_Pnt aP1 = theCurve.EvalD0(theFirst);
            double aStep = aDP / (2 * theN);
            for (int anIdx = 1; anIdx <= theN; ++anIdx)
            {
                aP += aStep;
                gp_Pnt aP2 = theCurve.EvalD0(aP);
                theBox.Add(aP2);
                aP += aStep;
                gp_Pnt aP3 = theCurve.EvalD0(aP);
                theBox.Add(aP3);
                // PointOps<gp_Pnt>::Mid
                gp_Pnt aPc = new((aP1.XYZ() + aP3.XYZ()) / 2.0);
                aTol = Max(aTol, aPc.Distance(aP2));
                aP1 = aP3;
            }
        }
        else
        {
            theBox.Add(theCurve.EvalD0(theLast));
        }
        return aTol;
    }

    /// <summary>Reduce box using poles convex hull.</summary>
    internal static void ReduceSplineBox(NCollection_Array1<gp_Pnt> thePoles, in Bnd_Box theOrigBox, ref Bnd_Box theReducedBox)
    {
        Bnd_Box aPolesBox = new();
        for (int anIdx = thePoles.Lower(); anIdx <= thePoles.Upper(); ++anIdx)
        {
            aPolesBox.Add(thePoles.Value(anIdx));
        }

        (double aXMin, double aXMax, double aYMin, double aYMax, double aZMin, double aZMax) = theOrigBox.Get();
        if (!aPolesBox.IsVoid())
        {
            (double aPXMin, double aPXMax, double aPYMin, double aPYMax, double aPZMin, double aPZMax) = aPolesBox.Get();
            theReducedBox.Update(Max(aXMin, aPXMin),
                                 Max(aYMin, aPYMin),
                                 Max(aZMin, aPZMin),
                                 Min(aXMax, aPXMax),
                                 Min(aYMax, aPYMax),
                                 Min(aZMax, aPZMax));
        }
        else
        {
            theReducedBox.Update(aXMin, aYMin, aZMin, aXMax, aYMax, aZMax);
        }
    }

    /// <summary>Compute precise bounding box for a 3D curve using sampling + numerical optimization.</summary>
    internal static Bnd_Box CurveBoxOptimal(GeomAdaptor_Curve theAdaptor, double theU1, double theU2, double theTol)
    {
        const int NDim = 3;
        int aNbSamples = GeomBndLib_SamplingHelpers.ComputeNbSamplesT(theAdaptor, theU1, theU2);

        Span<double> aCoordMin = stackalloc double[NDim];
        Span<double> aCoordMax = stackalloc double[NDim];
        Span<double> aDeflMax = stackalloc double[NDim];
        for (int aK = 0; aK < NDim; ++aK)
        {
            aCoordMin[aK] = RealLast();
            aCoordMax[aK] = -RealLast();
            aDeflMax[aK] = -RealLast();
        }

        double aDU = (theU2 - theU1) / (aNbSamples - 1);
        double aDU2 = aDU / 2.0;
        NCollection_Array1<gp_XYZ> aPnts = new(1, aNbSamples);
        for (int anI = 1, aJ = 0; anI <= aNbSamples; anI++, aJ++)
        {
            double aU = theU1 + aJ * aDU;
            gp_Pnt aP = theAdaptor.EvalD0(aU);
            aPnts[anI] = aP.XYZ();
            for (int aK = 0; aK < NDim; ++aK)
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
            if (anI > 1)
            {
                gp_XYZ aPm = 0.5 * (aPnts[anI - 1] + aPnts[anI]);
                gp_Pnt aPMid = theAdaptor.EvalD0(aU - aDU2);
                gp_XYZ aD = aPMid.XYZ() - aPm;
                for (int aK = 0; aK < NDim; ++aK)
                {
                    if (aCoordMin[aK] > aPMid.Coord(aK + 1))
                    {
                        aCoordMin[aK] = aPMid.Coord(aK + 1);
                    }
                    if (aCoordMax[aK] < aPMid.Coord(aK + 1))
                    {
                        aCoordMax[aK] = aPMid.Coord(aK + 1);
                    }
                    double aDiff = Math.Abs(aD.Coord(aK + 1));
                    if (aDeflMax[aK] < aDiff)
                    {
                        aDeflMax[aK] = aDiff;
                    }
                }
            }
        }

        double anEps = Max(theTol, Precision.Confusion());
        for (int aK = 0; aK < NDim; ++aK)
        {
            double aDiff = aDeflMax[aK];
            if (aDiff <= anEps)
            {
                continue;
            }
            double aCMin = aCoordMin[aK];
            double aCMax = aCoordMax[aK];
            for (int anI = 1; anI <= aNbSamples; ++anI)
            {
                if (aPnts[anI].Coord(aK + 1) - aCMin < aDiff)
                {
                    double anUMin = theU1 + Max(0, anI - 2) * aDU;
                    double anUMax = theU1 + Min(aNbSamples - 1, anI) * aDU;
                    double aLocMin = GeomBndLib_OptimizationHelpers.AdjustExtrT(theAdaptor, anUMin, anUMax, aCMin, aK + 1, anEps, true);
                    if (aLocMin < aCMin)
                    {
                        aCMin = aLocMin;
                    }
                }
                else if (aCMax - aPnts[anI].Coord(aK + 1) < aDiff)
                {
                    double anUMin = theU1 + Max(0, anI - 2) * aDU;
                    double anUMax = theU1 + Min(aNbSamples - 1, anI) * aDU;
                    double aLocMax = GeomBndLib_OptimizationHelpers.AdjustExtrT(theAdaptor, anUMin, anUMax, aCMax, aK + 1, anEps, false);
                    if (aLocMax > aCMax)
                    {
                        aCMax = aLocMax;
                    }
                }
            }
            aCoordMin[aK] = aCMin;
            aCoordMax[aK] = aCMax;
        }

        Bnd_Box aBox = new();
        aBox.Add(new gp_Pnt(aCoordMin[0], aCoordMin[1], aCoordMin[2]));
        aBox.Add(new gp_Pnt(aCoordMax[0], aCoordMax[1], aCoordMax[2]));
        aBox.Enlarge(anEps);
        return aBox;
    }

    /// <summary>Compute precise bounding box for a 2D curve using sampling + numerical optimization.</summary>
    internal static Bnd_Box2d CurveBoxOptimal(Geom2dAdaptor_Curve theAdaptor, double theU1, double theU2, double theTol)
    {
        const int NDim = 2;
        int aNbSamples = GeomBndLib_SamplingHelpers.ComputeNbSamplesT(theAdaptor, theU1, theU2);

        Span<double> aCoordMin = stackalloc double[NDim];
        Span<double> aCoordMax = stackalloc double[NDim];
        Span<double> aDeflMax = stackalloc double[NDim];
        for (int aK = 0; aK < NDim; ++aK)
        {
            aCoordMin[aK] = RealLast();
            aCoordMax[aK] = -RealLast();
            aDeflMax[aK] = -RealLast();
        }

        double aDU = (theU2 - theU1) / (aNbSamples - 1);
        double aDU2 = aDU / 2.0;
        NCollection_Array1<gp_XY> aPnts = new(1, aNbSamples);
        for (int anI = 1, aJ = 0; anI <= aNbSamples; anI++, aJ++)
        {
            double aU = theU1 + aJ * aDU;
            gp_Pnt2d aP = theAdaptor.EvalD0(aU);
            aPnts[anI] = aP.XY();
            for (int aK = 0; aK < NDim; ++aK)
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
            if (anI > 1)
            {
                gp_XY aPm = 0.5 * (aPnts[anI - 1] + aPnts[anI]);
                gp_Pnt2d aPMid = theAdaptor.EvalD0(aU - aDU2);
                gp_XY aD = aPMid.XY() - aPm;
                for (int aK = 0; aK < NDim; ++aK)
                {
                    if (aCoordMin[aK] > aPMid.Coord(aK + 1))
                    {
                        aCoordMin[aK] = aPMid.Coord(aK + 1);
                    }
                    if (aCoordMax[aK] < aPMid.Coord(aK + 1))
                    {
                        aCoordMax[aK] = aPMid.Coord(aK + 1);
                    }
                    double aDiff = Math.Abs(aD.Coord(aK + 1));
                    if (aDeflMax[aK] < aDiff)
                    {
                        aDeflMax[aK] = aDiff;
                    }
                }
            }
        }

        double anEps = Max(theTol, Precision.Confusion());
        for (int aK = 0; aK < NDim; ++aK)
        {
            double aDiff = aDeflMax[aK];
            if (aDiff <= anEps)
            {
                continue;
            }
            double aCMin = aCoordMin[aK];
            double aCMax = aCoordMax[aK];
            for (int anI = 1; anI <= aNbSamples; ++anI)
            {
                if (aPnts[anI].Coord(aK + 1) - aCMin < aDiff)
                {
                    double anUMin = theU1 + Max(0, anI - 2) * aDU;
                    double anUMax = theU1 + Min(aNbSamples - 1, anI) * aDU;
                    double aLocMin = GeomBndLib_OptimizationHelpers.AdjustExtrT(theAdaptor, anUMin, anUMax, aCMin, aK + 1, anEps, true);
                    if (aLocMin < aCMin)
                    {
                        aCMin = aLocMin;
                    }
                }
                else if (aCMax - aPnts[anI].Coord(aK + 1) < aDiff)
                {
                    double anUMin = theU1 + Max(0, anI - 2) * aDU;
                    double anUMax = theU1 + Min(aNbSamples - 1, anI) * aDU;
                    double aLocMax = GeomBndLib_OptimizationHelpers.AdjustExtrT(theAdaptor, anUMin, anUMax, aCMax, aK + 1, anEps, false);
                    if (aLocMax > aCMax)
                    {
                        aCMax = aLocMax;
                    }
                }
            }
            aCoordMin[aK] = aCMin;
            aCoordMax[aK] = aCMax;
        }

        Bnd_Box2d aBox = new();
        aBox.Add(new gp_Pnt2d(aCoordMin[0], aCoordMin[1]));
        aBox.Add(new gp_Pnt2d(aCoordMax[0], aCoordMax[1]));
        aBox.Enlarge(anEps);
        return aBox;
    }
}
