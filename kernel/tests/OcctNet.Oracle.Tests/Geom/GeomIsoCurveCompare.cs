#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using N = Build123d.Occt;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// Compares a native curve returned by a surface's UIso or VIso with the port's: its class, its definition (axes,
/// radius, poles, weights, knots, trim parameters, recursively for a trimmed curve's basis), its parameters,
/// closure, periodicity and continuity, and its evaluation (D0 to D3, DN up to 3).
/// </summary>
internal static class GeomIsoCurveCompare
{
    private static string NativeKind(N.Geom_Curve theN)
    {
        using (var aC = N.Geom_Line.DownCast(theN))
        {
            if (aC is not null)
            {
                return nameof(Geom_Line);
            }
        }
        using (var aC = N.Geom_Circle.DownCast(theN))
        {
            if (aC is not null)
            {
                return nameof(Geom_Circle);
            }
        }
        using (var aC = N.Geom_TrimmedCurve.DownCast(theN))
        {
            if (aC is not null)
            {
                return nameof(Geom_TrimmedCurve);
            }
        }
        using (var aC = N.Geom_BezierCurve.DownCast(theN))
        {
            if (aC is not null)
            {
                return nameof(Geom_BezierCurve);
            }
        }
        using (var aC = N.Geom_BSplineCurve.DownCast(theN))
        {
            if (aC is not null)
            {
                return nameof(Geom_BSplineCurve);
            }
        }
        return "another curve";
    }

    private static void SameDir(Oracle theOracle, N.gp_Dir theN, in gp_Dir theP, string theWhat)
    {
        theOracle.Same(theN.X(), theP.X(), theWhat + " X");
        theOracle.Same(theN.Y(), theP.Y(), theWhat + " Y");
        theOracle.Same(theN.Z(), theP.Z(), theWhat + " Z");
    }

    /// <summary>The class and the definition of the curve.</summary>
    private static void SameDefinition(Oracle theOracle, N.Geom_Curve theN, Geom_Curve theP, string theWhat)
    {
        Assert.True(NativeKind(theN) == theP.GetType().Name, $"{theWhat}: native {NativeKind(theN)}, port {theP.GetType().Name}");
        switch (theP)
        {
            case Geom_Line aLine:
            {
                using var aNLine = N.Geom_Line.DownCast(theN)!;
                using N.gp_Ax1 aNPos = aNLine.Position();
                using N.gp_Pnt aNLoc = aNPos.Location();
                using N.gp_Dir aNDir = aNPos.Direction();
                theOracle.Same(aNLoc, aLine.Position().Location(), theWhat + " line location");
                SameDir(theOracle, aNDir, aLine.Position().Direction(), theWhat + " line direction");
                break;
            }
            case Geom_Circle aCircle:
            {
                using var aNCircle = N.Geom_Circle.DownCast(theN)!;
                using N.gp_Ax2 aNPos = aNCircle.Position();
                using N.gp_Pnt aNLoc = aNPos.Location();
                using N.gp_Dir aNDir = aNPos.Direction();
                using N.gp_Dir aNXDir = aNPos.XDirection();
                theOracle.Same(aNCircle.Radius(), aCircle.Radius(), theWhat + " circle radius");
                theOracle.Same(aNLoc, aCircle.Position().Location(), theWhat + " circle location");
                SameDir(theOracle, aNDir, aCircle.Position().Direction(), theWhat + " circle direction");
                SameDir(theOracle, aNXDir, aCircle.Position().XDirection(), theWhat + " circle X direction");
                break;
            }
            case Geom_TrimmedCurve aTrimmed:
            {
                using var aNTrimmed = N.Geom_TrimmedCurve.DownCast(theN)!;
                using N.Geom_Curve aNBasis = aNTrimmed.BasisCurve()!;
                SameDefinition(theOracle, aNBasis, aTrimmed.BasisCurve(), theWhat + " basis");
                break;
            }
            case Geom_BezierCurve aBezier:
            {
                using var aNBezier = N.Geom_BezierCurve.DownCast(theN)!;
                theOracle.Same(aNBezier.Degree(), aBezier.Degree(), theWhat + " Bezier degree");
                theOracle.Same(aNBezier.IsRational(), aBezier.IsRational(), theWhat + " Bezier IsRational");
                for (int i = 1; i <= aBezier.NbPoles(); i++)
                {
                    using N.gp_Pnt aNPole = aNBezier.Pole(i);
                    theOracle.Same(aNPole, aBezier.Pole(i), theWhat + " Bezier pole");
                    theOracle.Same(aNBezier.Weight(i), aBezier.Weight(i), theWhat + " Bezier weight");
                }
                break;
            }
            case Geom_BSplineCurve aBSpline:
            {
                using var aNBSpline = N.Geom_BSplineCurve.DownCast(theN)!;
                theOracle.Same(aNBSpline.Degree(), aBSpline.Degree(), theWhat + " B-spline degree");
                theOracle.Same(aNBSpline.IsRational(), aBSpline.IsRational(), theWhat + " B-spline IsRational");
                theOracle.Same(aNBSpline.NbPoles(), aBSpline.NbPoles(), theWhat + " B-spline NbPoles");
                theOracle.Same(aNBSpline.NbKnots(), aBSpline.NbKnots(), theWhat + " B-spline NbKnots");
                for (int i = 1; i <= aBSpline.NbPoles(); i++)
                {
                    using N.gp_Pnt aNPole = aNBSpline.Pole(i);
                    theOracle.Same(aNPole, aBSpline.Pole(i), theWhat + " B-spline pole");
                    theOracle.Same(aNBSpline.Weight(i), aBSpline.Weight(i), theWhat + " B-spline weight");
                }
                for (int i = 1; i <= aBSpline.NbKnots(); i++)
                {
                    theOracle.Same(aNBSpline.Knot(i), aBSpline.Knot(i), theWhat + " B-spline knot");
                    theOracle.Same(aNBSpline.Multiplicity(i), aBSpline.Multiplicity(i), theWhat + " B-spline multiplicity");
                }
                break;
            }
        }
    }

    private static Exception? Catch(Action theAction)
    {
        try
        {
            theAction();
            return null;
        }
        catch (Exception anE)
        {
            return anE;
        }
    }

    /// <summary>D0 to D3 and DN (1 to 3) at U; both must throw, or neither.</summary>
    private static void SameEvaluation(Oracle theOracle, N.Geom_Curve theN, Geom_Curve theP, double U, string theWhat)
    {
        using var aNP = new N.gp_Pnt();
        using var aNV1 = new N.gp_Vec();
        using var aNV2 = new N.gp_Vec();
        using var aNV3 = new N.gp_Vec();
        gp_Pnt P = new();
        gp_Vec V1 = new(), V2 = new(), V3 = new();
        Exception? aNErr = Catch(() => theN.D3(U, aNP, aNV1, aNV2, aNV3));
        Exception? aPErr = Catch(() => theP.D3(U, out P, out V1, out V2, out V3));
        Assert.True((aNErr is null) == (aPErr is null), $"{theWhat} D3({U}): native {aNErr?.Message ?? "ok"}, port {aPErr?.ToString() ?? "ok"}");
        if (aNErr is not null)
        {
            return;
        }
        theOracle.Same(aNP, P, theWhat + " D3 P");
        theOracle.Same(aNV1, V1, theWhat + " D3 V1");
        theOracle.Same(aNV2, V2, theWhat + " D3 V2");
        theOracle.Same(aNV3, V3, theWhat + " D3 V3");

        using (N.gp_Pnt aNP0 = theN.EvalD0(U))
        {
            theOracle.Same(aNP0, theP.Value(U), theWhat + " D0");
        }
        theN.D1(U, aNP, aNV1);
        theP.D1(U, out P, out V1);
        theOracle.Same(aNP, P, theWhat + " D1 P");
        theOracle.Same(aNV1, V1, theWhat + " D1 V1");
        theN.D2(U, aNP, aNV1, aNV2);
        theP.D2(U, out P, out V1, out V2);
        theOracle.Same(aNP, P, theWhat + " D2 P");
        theOracle.Same(aNV1, V1, theWhat + " D2 V1");
        theOracle.Same(aNV2, V2, theWhat + " D2 V2");
        for (int k = 1; k <= 3; k++)
        {
            using N.gp_Vec aNVN = theN.DN(U, k);
            theOracle.Same(aNVN, theP.DN(U, k), $"{theWhat} DN({k})");
        }
    }

    /// <summary>
    /// The iso-curves at theParam of the two surfaces (UIso if theU), compared as the class header says; both
    /// surfaces must throw, or neither.
    /// </summary>
    internal static void SameIso(Oracle theOracle, N.Geom_Surface theN, Geom_Surface theP, bool theU, double theParam,
                                 string theWhat)
    {
        theWhat += (theU ? " UIso" : " VIso") + "(" + theParam.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + ")";
        N.Geom_Curve? aN = null;
        Geom_Curve? aP = null;
        Exception? aNErr = Catch(() => aN = theU ? theN.UIso(theParam) : theN.VIso(theParam));
        Exception? aPErr = Catch(() => aP = theU ? theP.UIso(theParam) : theP.VIso(theParam));
        Assert.True((aNErr is null) == (aPErr is null), $"{theWhat}: native {aNErr?.Message ?? "ok"}, port {aPErr?.ToString() ?? "ok"}");
        if (aNErr is not null)
        {
            return;
        }
        using (aN)
        {
            SameCurve(theOracle, aN!, aP!, theWhat);
        }
    }

    /// <summary>Two iso-curves, compared as the class header says.</summary>
    internal static void SameCurve(Oracle theOracle, N.Geom_Curve theN, Geom_Curve theP, string theWhat)
    {
        SameDefinition(theOracle, theN, theP, theWhat);

        double aF = theP.FirstParameter(), aL = theP.LastParameter();
        theOracle.Same(theN.FirstParameter(), aF, theWhat + " FirstParameter");
        theOracle.Same(theN.LastParameter(), aL, theWhat + " LastParameter");
        theOracle.Same(theN.IsClosed(), theP.IsClosed(), theWhat + " IsClosed");
        theOracle.Same(theN.IsPeriodic(), theP.IsPeriodic(), theWhat + " IsPeriodic");
        if (theP.IsPeriodic())
        {
            theOracle.Same(theN.Period(), theP.Period(), theWhat + " Period");
        }
        theOracle.Same((int)theN.Continuity(), (int)theP.Continuity(), theWhat + " Continuity");
        for (int k = 0; k <= 3; k++)
        {
            theOracle.Same(theN.IsCN(k), theP.IsCN(k), theWhat + " IsCN");
        }

        if (Precision.IsInfinite(aF) || Precision.IsInfinite(aL))
        {
            aF = -20;
            aL = 20;
        }
        SameEvaluation(theOracle, theN, theP, aF, theWhat);
        SameEvaluation(theOracle, theN, theP, aL, theWhat);
        for (int k = 0; k < 3; k++)
        {
            SameEvaluation(theOracle, theN, theP, theOracle.Real(aF, aL), theWhat);
        }
    }

    /// <summary>
    /// UIso and VIso at the bounds of a finite surface, and at random parameters inside the bounds (and outside
    /// the first period of a periodic direction).
    /// </summary>
    internal static void SameIsos(Oracle theOracle, N.Geom_Surface theN, Geom_Surface theP, string theWhat)
    {
        theP.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        foreach (bool aU in new[] { true, false })
        {
            double aF = aU ? aU1 : aV1, aL = aU ? aU2 : aV2;
            var aParams = new List<double>();
            if (Precision.IsInfinite(aF) || Precision.IsInfinite(aL))
            {
                aParams.Add(theOracle.Real(-20, 20));
                aParams.Add(theOracle.Real(-20, 20));
            }
            else
            {
                aParams.Add(aF);
                aParams.Add(aL);
                aParams.Add(theOracle.Real(aF, aL));
                aParams.Add(theOracle.Real(aF, aL));
                if (aU ? theP.IsUPeriodic() : theP.IsVPeriodic())
                {
                    aParams.Add(theOracle.Real(aF, aL) + 2 * (aL - aF));
                }
            }
            foreach (double aParam in aParams)
            {
                SameIso(theOracle, theN, theP, aU, aParam, theWhat);
            }
        }
    }
}
#endif
