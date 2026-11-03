// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GCurveLocator.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TheCurveLocatorOfTheProjPCurOfCInter.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.HiddenLines;

/// <summary>
/// The sample of a 2D curve nearest a point: OCCT's template Extrema_GCurveLocator instantiated over
/// HLRBRep_Curve and HLRBRep_CurveTool.
/// </summary>
public static class TheCurveLocatorOfTheProjPCurOfCInter
{
    /// <summary>
    /// Among the points C(ui), i = 1 to <paramref name="nbU"/> - 1, evenly spaced from the first parameter of
    /// the curve, the one nearest <paramref name="p"/>.
    /// </summary>
    /// <exception cref="OutOfRangeException"><paramref name="nbU"/> is less than 2.</exception>
    public static void Locate(in Point2D p, HiddenLineCurve c, int nbU, out POnCurv2d papp)
    {
        if (nbU < 2)
        {
            throw new OutOfRangeException();
        }

        double u = HiddenLineCurveTool.FirstParameter(c);
        double pasU = (HiddenLineCurveTool.LastParameter(c) - u) / (nbU - 1);
        double dist2Min = RealLast();
        double uMin = 0;
        Point2D pntMin = new();
        double dist2;
        Point2D pt;

        for (int noSample = 1; noSample < nbU; noSample++, u += pasU)
        {
            pt = HiddenLineCurveTool.Value(c, u);
            dist2 = pt.SquareDistanceTo(p);
            if (dist2 < dist2Min)
            {
                dist2Min = dist2;
                uMin = u;
                pntMin = pt;
            }
        }
        papp = new POnCurv2d(uMin, pntMin);
    }

    /// <summary>
    /// As the other overload, sampling between <paramref name="umin"/> and <paramref name="usup"/> (clamped to
    /// the curve's range).
    /// </summary>
    /// <exception cref="OutOfRangeException"><paramref name="nbU"/> is less than 2.</exception>
    public static void Locate(in Point2D p, HiddenLineCurve c, int nbU, double umin, double usup,
                              out POnCurv2d papp)
    {
        if (nbU < 2)
        {
            throw new OutOfRangeException();
        }

        double uinf = HiddenLineCurveTool.FirstParameter(c);
        double ulast = HiddenLineCurveTool.LastParameter(c);

        double u1 = Min(uinf, ulast);
        double u2 = Max(uinf, ulast);
        double u11 = Min(umin, usup);
        double u12 = Max(umin, usup);

        if (u11 < u1 - RealEpsilon())
        {
            u11 = u1;
        }
        if (u12 > u2 + RealEpsilon())
        {
            u12 = u2;
        }

        double u = u11;
        double pasU = (u12 - u) / (nbU - 1);
        double dist2Min = RealLast();
        double uMin = 0;
        Point2D pntMin = new();
        double dist2;
        Point2D pt;

        for (int noSample = 1; noSample < nbU; noSample++, u += pasU)
        {
            pt = HiddenLineCurveTool.Value(c, u);
            dist2 = pt.SquareDistanceTo(p);
            if (dist2 < dist2Min)
            {
                dist2Min = dist2;
                uMin = u;
                pntMin = pt;
            }
        }
        papp = new POnCurv2d(uMin, pntMin);
    }
}
