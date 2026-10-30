// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The package statics of GeomInt (surface/surface intersection on Geom surfaces).</summary>
/// <remarks>
/// The package has one function. Unit 6B ports it, ahead of unit 5G's classes, because IntTools_FClass2d and
/// BOPTools_AlgoTools2D call it (decision 715).
/// </remarks>
public static partial class GeomInt
{
    /// <summary>
    /// Adjusts the periodic parameter <paramref name="thePar"/> to the range [<paramref name="theParMin"/>,
    /// <paramref name="theParMax"/>] by a whole number of periods (<paramref name="theNewPar"/>, offset
    /// <paramref name="theOffset"/>), if it is out of the range by more than <paramref name="theEps"/>. True if the
    /// offset is positive.
    /// </summary>
    public static bool AdjustPeriodic(double thePar,
                                      double theParMin,
                                      double theParMax,
                                      double thePeriod,
                                      out double theNewPar,
                                      out double theOffset,
                                      double theEps = 0.0)
    {
        bool bMin, bMax;
        //
        theOffset = 0.0;
        theNewPar = thePar;
        bMin = theParMin - thePar > theEps;
        bMax = thePar - theParMax > theEps;
        //
        if (bMin || bMax)
        {
            double dp, aNbPer;
            //
            dp = (bMin) ? (theParMax - thePar) : (theParMin - thePar);
            aNbPer = Math.Truncate(dp / thePeriod); // modf
            //
            theOffset = aNbPer * thePeriod;
            theNewPar += theOffset;
        }
        //
        return (theOffset > 0.0);
    }
}
