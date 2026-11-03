// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GCurveLocator.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TheCurveLocatorOfTheProjPCurOfCInter.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The sample of a 2D curve nearest a point: OCCT's template Extrema_GCurveLocator instantiated over
/// HLRBRep_Curve and HLRBRep_CurveTool.
/// </summary>
public static class HLRBRep_TheCurveLocatorOfTheProjPCurOfCInter
{
    /// <summary>
    /// Among the points C(ui), i = 1 to <paramref name="theNbU"/> - 1, evenly spaced from the first parameter of
    /// the curve, the one nearest <paramref name="theP"/>.
    /// </summary>
    /// <exception cref="OutOfRangeException"><paramref name="theNbU"/> is less than 2.</exception>
    public static void Locate(in Point2D theP, HLRBRep_Curve theC, int theNbU, out POnCurv2d thePapp)
    {
        if (theNbU < 2)
        {
            throw new OutOfRangeException();
        }

        double aU = HLRBRep_CurveTool.FirstParameter(theC);
        double aPasU = (HLRBRep_CurveTool.LastParameter(theC) - aU) / (theNbU - 1);
        double aDist2Min = RealLast();
        double aUMin = 0;
        Point2D aPntMin = new();
        double aDist2;
        Point2D aPt;

        for (int aNoSample = 1; aNoSample < theNbU; aNoSample++, aU += aPasU)
        {
            aPt = HLRBRep_CurveTool.Value(theC, aU);
            aDist2 = aPt.SquareDistance(theP);
            if (aDist2 < aDist2Min)
            {
                aDist2Min = aDist2;
                aUMin = aU;
                aPntMin = aPt;
            }
        }
        thePapp = new POnCurv2d(aUMin, aPntMin);
    }

    /// <summary>
    /// As the other overload, sampling between <paramref name="theUmin"/> and <paramref name="theUsup"/> (clamped to
    /// the curve's range).
    /// </summary>
    /// <exception cref="OutOfRangeException"><paramref name="theNbU"/> is less than 2.</exception>
    public static void Locate(in Point2D theP, HLRBRep_Curve theC, int theNbU, double theUmin, double theUsup,
                              out POnCurv2d thePapp)
    {
        if (theNbU < 2)
        {
            throw new OutOfRangeException();
        }

        double aUinf = HLRBRep_CurveTool.FirstParameter(theC);
        double aUlast = HLRBRep_CurveTool.LastParameter(theC);

        double aU1 = Min(aUinf, aUlast);
        double aU2 = Max(aUinf, aUlast);
        double aU11 = Min(theUmin, theUsup);
        double aU12 = Max(theUmin, theUsup);

        if (aU11 < aU1 - RealEpsilon())
        {
            aU11 = aU1;
        }
        if (aU12 > aU2 + RealEpsilon())
        {
            aU12 = aU2;
        }

        double aU = aU11;
        double aPasU = (aU12 - aU) / (theNbU - 1);
        double aDist2Min = RealLast();
        double aUMin = 0;
        Point2D aPntMin = new();
        double aDist2;
        Point2D aPt;

        for (int aNoSample = 1; aNoSample < theNbU; aNoSample++, aU += aPasU)
        {
            aPt = HLRBRep_CurveTool.Value(theC, aU);
            aDist2 = aPt.SquareDistance(theP);
            if (aDist2 < aDist2Min)
            {
                aDist2Min = aDist2;
                aUMin = aU;
                aPntMin = aPt;
            }
        }
        thePapp = new POnCurv2d(aUMin, aPntMin);
    }
}
