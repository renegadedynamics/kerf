// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Intersection.Surfaces;

/// <summary>The package statics of GeomInt (surface/surface intersection on Geom surfaces).</summary>
/// <remarks>
/// The package has one function. Unit 6B ports it, ahead of unit 5G's classes, because IntTools_FClass2d and
/// BOPTools_AlgoTools2D call it (decision 715).
/// </remarks>
internal static partial class GeomInt
{
    /// <summary>
    /// Adjusts the periodic parameter <paramref name="par"/> to the range [<paramref name="parMin"/>,
    /// <paramref name="parMax"/>] by a whole number of periods (<paramref name="newPar"/>, offset
    /// <paramref name="offset"/>), if it is out of the range by more than <paramref name="eps"/>. True if the
    /// offset is positive.
    /// </summary>
    public static bool AdjustPeriodic(double par,
                                      double parMin,
                                      double parMax,
                                      double period,
                                      out double newPar,
                                      out double offset,
                                      double eps = 0.0)
    {
        bool bMin, bMax;
        //
        offset = 0.0;
        newPar = par;
        bMin = parMin - par > eps;
        bMax = par - parMax > eps;
        //
        if (bMin || bMax)
        {
            double dp, nbPer;
            //
            dp = (bMin) ? (parMax - par) : (parMin - par);
            nbPer = Math.Truncate(dp / period); // modf
            //
            offset = nbPer * period;
            newPar += offset;
        }
        //
        return (offset > 0.0);
    }
}
