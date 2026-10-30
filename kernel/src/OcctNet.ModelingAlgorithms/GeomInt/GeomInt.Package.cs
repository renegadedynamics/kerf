// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The GeomInt package's tools: intersections between Geom surfaces (<see cref="GeomInt_IntSS"/>).</summary>
public static partial class GeomInt
{
    /// <summary>
    /// Brings thePar into [theParMin, theParMax] (with the tolerance theEps) by whole periods of thePeriod:
    /// theNewPar is the adjusted parameter and theOffset the shift. Returns true if the shift is positive.
    /// </summary>
    public static bool AdjustPeriodic(double thePar, double theParMin, double theParMax, double thePeriod,
                                      out double theNewPar, out double theOffset, double theEps = 0.0)
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
            aNbPer = Math.Truncate(dp / thePeriod); // modf's integral part
            //
            theOffset = aNbPer * thePeriod;
            theNewPar += theOffset;
        }
        //
        return (theOffset > 0.0);
    }
}
