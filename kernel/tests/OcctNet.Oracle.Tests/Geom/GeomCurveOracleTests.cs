#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;
using N = Build123d.Occt;
using static OcctNet.Oracle.Tests.Geom.GeomOracleInputs;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// The Geom curves (unit 1A) against native OCCT, on random inputs: evaluation (D0 to D3, DN), parameters,
/// periodicity, continuity, reversal, transformation, and the Bezier and B-spline modifications.
/// </summary>
/// <remarks>
/// The native curves are changed in place with OCCT's mutators; the port's through its public non-mutating
/// twins (<c>KnotInserted</c>, <c>WithPole</c>, <c>Transformed</c>, ...), so the twins are checked too. Where
/// native OCCT throws, the port must throw as well. Directions, axes and transformations are built on both sides
/// from the same raw numbers (<see cref="Geom.GeomOracleInputs"/>, decision 175): the last-bit differences first
/// seen after rotations were differences of the inputs, so transformed curves are compared at the default
/// tolerance like the rest, and so is the closure of infinite curves.
/// </remarks>
public class GeomCurveOracleTests(ITestOutputHelper output)
{
    private const int Cases = 300;

    // ---------------------------------------------------------------------------------------------
    // Conversions and comparisons
    // ---------------------------------------------------------------------------------------------

    private static gp_XYZ FromNative(N.gp_Vec theV) => new(theV.X(), theV.Y(), theV.Z());

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

    /// <summary>Both threw, or neither did; true if neither did.</summary>
    private static bool SameOutcome(Exception? theNative, Exception? thePort, string theWhat)
    {
        Assert.True((theNative is null) == (thePort is null),
                    $"{theWhat}: native {(theNative is null ? "succeeded" : "threw " + theNative.Message)}, "
                    + $"port {(thePort is null ? "succeeded" : "threw " + thePort.GetType().Name + " " + thePort.Message)}");
        return theNative is null;
    }

    /// <summary>D0 to D3 and DN (1 to theMaxN) at theU.</summary>
    private static void SameEval(Oracle theO, N.Geom_Curve theN, Geom_Curve theP, double theU, int theMaxN, string theWhat,
                                 double theTol = Oracle.Tolerance)
    {
        using var aNP = new N.gp_Pnt();
        using var aNV1 = new N.gp_Vec();
        using var aNV2 = new N.gp_Vec();
        using var aNV3 = new N.gp_Vec();
        gp_Pnt aP = new();
        gp_Vec aV1 = new(), aV2 = new(), aV3 = new();

        if (SameOutcome(Catch(() => theN.D0(theU, aNP)), Catch(() => theP.D0(theU, out aP)), $"{theWhat} D0({theU})"))
        {
            theO.Same(Oracle.FromNative(aNP).XYZ(), aP.XYZ(), $"{theWhat} D0({theU})", theTol);
        }
        if (SameOutcome(Catch(() => theN.D1(theU, aNP, aNV1)), Catch(() => theP.D1(theU, out aP, out aV1)), $"{theWhat} D1({theU})"))
        {
            theO.Same(Oracle.FromNative(aNP).XYZ(), aP.XYZ(), $"{theWhat} D1({theU}) P", theTol);
            theO.Same(FromNative(aNV1), aV1.XYZ(), $"{theWhat} D1({theU}) V1", theTol);
        }
        if (SameOutcome(Catch(() => theN.D2(theU, aNP, aNV1, aNV2)), Catch(() => theP.D2(theU, out aP, out aV1, out aV2)),
                        $"{theWhat} D2({theU})"))
        {
            theO.Same(Oracle.FromNative(aNP).XYZ(), aP.XYZ(), $"{theWhat} D2({theU}) P", theTol);
            theO.Same(FromNative(aNV1), aV1.XYZ(), $"{theWhat} D2({theU}) V1", theTol);
            theO.Same(FromNative(aNV2), aV2.XYZ(), $"{theWhat} D2({theU}) V2", theTol);
        }
        if (SameOutcome(Catch(() => theN.D3(theU, aNP, aNV1, aNV2, aNV3)),
                        Catch(() => theP.D3(theU, out aP, out aV1, out aV2, out aV3)), $"{theWhat} D3({theU})"))
        {
            theO.Same(Oracle.FromNative(aNP).XYZ(), aP.XYZ(), $"{theWhat} D3({theU}) P", theTol);
            theO.Same(FromNative(aNV1), aV1.XYZ(), $"{theWhat} D3({theU}) V1", theTol);
            theO.Same(FromNative(aNV2), aV2.XYZ(), $"{theWhat} D3({theU}) V2", theTol);
            theO.Same(FromNative(aNV3), aV3.XYZ(), $"{theWhat} D3({theU}) V3", theTol);
        }
        for (int k = 1; k <= theMaxN; k++)
        {
            N.gp_Vec? aNVN = null;
            gp_Vec aVN = new();
            if (SameOutcome(Catch(() => aNVN = theN.DN(theU, k)), Catch(() => aVN = theP.DN(theU, k)), $"{theWhat} DN({theU}, {k})"))
            {
                theO.Same(FromNative(aNVN!), aVN.XYZ(), $"{theWhat} DN({theU}, {k})", theTol);
            }
            aNVN?.Dispose();
        }
    }

    /// <summary>Bounds, closure, periodicity, continuity.</summary>
    private static void SameProperties(Oracle theO, N.Geom_Curve theN, Geom_Curve theP, string theWhat)
    {
        theO.Same(theN.FirstParameter(), theP.FirstParameter(), $"{theWhat} FirstParameter");
        theO.Same(theN.LastParameter(), theP.LastParameter(), $"{theWhat} LastParameter");
        theO.Same(theN.IsClosed(), theP.IsClosed(), $"{theWhat} IsClosed");
        theO.Same(theN.IsPeriodic(), theP.IsPeriodic(), $"{theWhat} IsPeriodic");
        Assert.Equal((int)theN.Continuity(), (int)theP.Continuity());
        for (int k = 0; k <= 5; k++)
        {
            theO.Same(theN.IsCN(k), theP.IsCN(k), $"{theWhat} IsCN({k})");
        }
        double aNPer = 0, aPPer = 0;
        if (SameOutcome(Catch(() => aNPer = theN.Period()), Catch(() => aPPer = theP.Period()), $"{theWhat} Period"))
        {
            theO.Same(aNPer, aPPer, $"{theWhat} Period");
        }
    }

    /// <summary>Parameters to evaluate at: random ones inside the bounds (or theFallback), a few outside, and the bounds.</summary>
    private static List<double> Params(Oracle theO, Geom_Curve theP, double theFallbackMin, double theFallbackMax, bool theOutside = true)
    {
        double aF = theP.FirstParameter(), aL = theP.LastParameter();
        if (Precision.IsInfinite(aF) || Precision.IsInfinite(aL))
        {
            aF = theFallbackMin;
            aL = theFallbackMax;
        }
        else
        {
            var aB = new List<double> { aF, aL };
            for (int i = 0; i < 5; i++)
            {
                aB.Add(theO.Real(aF, aL));
            }
            if (theOutside)
            {
                aB.Add(aF - theO.Real(0, 1) * (aL - aF) * 0.3);
                aB.Add(aL + theO.Real(0, 1) * (aL - aF) * 0.3);
            }
            return aB;
        }
        var aR = new List<double>();
        for (int i = 0; i < 6; i++)
        {
            aR.Add(theO.Real(aF, aL));
        }
        return aR;
    }

    /// <summary>A random transformation (rotation, translation, scale, mirror or a composition), native and port.</summary>
    private static (N.gp_Trsf Native, gp_Trsf Port) RandomTrsf(Oracle theO)
    {
        var aNative = new N.gp_Trsf();
        switch (theO.Int(0, 4))
        {
            case 0:
            {
                (N.gp_Ax1 aNAxis, gp_Ax1 anAxis) = Ax1s(theO);
                using var aNAxisOwner = aNAxis;
                double anAngle = theO.Angle();
                aNative.SetRotation(aNAxis, anAngle);
                return (aNative, gp_Trsf.Rotation(anAxis, anAngle));
            }
            case 1:
            {
                gp_Vec aV = theO.Vec();
                using var aNV = Oracle.ToNative(aV);
                aNative.SetTranslation(aNV);
                return (aNative, gp_Trsf.Translation(aV));
            }
            case 2:
            {
                gp_Pnt aP = theO.Pnt();
                double aS = theO.Real(0.2, 3) * (theO.Int(0, 1) == 0 ? 1 : -1);
                using var aNP = Oracle.ToNative(aP);
                aNative.SetScale(aNP, aS);
                return (aNative, gp_Trsf.Scale(aP, aS));
            }
            case 3:
            {
                (N.gp_Ax2 aNAx, gp_Ax2 anAx) = Ax2sFromDirection(theO);
                using var aNAxOwner = aNAx;
                aNative.SetMirror(aNAx);
                return (aNative, gp_Trsf.Mirror(anAx));
            }
            default:
            {
                (N.gp_Ax1 aNAxis, gp_Ax1 anAxis) = Ax1s(theO);
                using var aNAxisOwner = aNAxis;
                double anAngle = theO.Angle();
                gp_Pnt aP = theO.Pnt();
                double aS = theO.Real(0.2, 3);
                using var aNP = Oracle.ToNative(aP);
                using var aNS = new N.gp_Trsf();
                aNative.SetRotation(aNAxis, anAngle);
                aNS.SetScale(aNP, aS);
                aNative.Multiply(aNS);
                return (aNative, gp_Trsf.Rotation(anAxis, anAngle).Multiplied(gp_Trsf.Scale(aP, aS)));
            }
        }
    }

    /// <summary>Reversal and transformation, compared by evaluation.</summary>
    private static void SameReversedAndTransformed(Oracle theO, N.Geom_Curve theN, Geom_Curve theP, List<double> theParams,
                                                   int theMaxN, string theWhat, double theTol = Oracle.Tolerance)
    {
        foreach (double u in theParams)
        {
            theO.Same(theN.ReversedParameter(u), theP.ReversedParameter(u), $"{theWhat} ReversedParameter");
        }

        using (var aNR = theN.Reversed()!)
        {
            Geom_Curve aR = theP.Reversed();
            SameProperties(theO, aNR, aR, $"{theWhat} reversed");
            foreach (double u in theParams)
            {
                SameEval(theO, aNR, aR, theP.ReversedParameter(u), theMaxN, $"{theWhat} reversed", theTol);
            }
        }

        (N.gp_Trsf aNT, gp_Trsf aT) = RandomTrsf(theO);
        using (aNT)
        {
            theO.Same(theN.ParametricTransformation(aNT), theP.ParametricTransformation(aT), $"{theWhat} ParametricTransformation");
            using var aNG = theN.Transformed(aNT);
            using var aNC = N.Geom_Curve.DownCast(aNG)!;
            var aC = (Geom_Curve)theP.Transformed(aT);
            SameProperties(theO, aNC, aC, $"{theWhat} transformed");
            // What Transform changes, at the default tolerance: the poles, or the conic's or line's axes
            switch (aC)
            {
                case Geom_BezierCurve aBz:
                {
                    using var aNBz = N.Geom_BezierCurve.DownCast(aNC)!;
                    for (int i = 1; i <= aBz.NbPoles(); i++)
                    {
                        using var aNPole = aNBz.Pole(i);
                        theO.Same(Oracle.FromNative(aNPole).XYZ(), aBz.Pole(i).XYZ(), $"{theWhat} transformed Pole({i})");
                    }
                    break;
                }
                case Geom_BSplineCurve aBs:
                {
                    using var aNBs = N.Geom_BSplineCurve.DownCast(aNC)!;
                    for (int i = 1; i <= aBs.NbPoles(); i++)
                    {
                        using var aNPole = aNBs.Pole(i);
                        theO.Same(Oracle.FromNative(aNPole).XYZ(), aBs.Pole(i).XYZ(), $"{theWhat} transformed Pole({i})");
                    }
                    break;
                }
                case Geom_Conic aCn:
                {
                    using var aNCn = N.Geom_Conic.DownCast(aNC)!;
                    using var aNPos = aNCn.Position();
                    using var aNLoc = aNPos.Location();
                    using var aNX = aNPos.XDirection();
                    using var aNXX = aNX.XYZ();
                    using var aNZ = aNPos.Direction();
                    using var aNZX = aNZ.XYZ();
                    theO.Same(Oracle.FromNative(aNLoc).XYZ(), aCn.Position().Location().XYZ(), $"{theWhat} transformed Location");
                    theO.Same(Oracle.FromNative(aNXX), aCn.Position().XDirection().XYZ(), $"{theWhat} transformed XDirection");
                    theO.Same(Oracle.FromNative(aNZX), aCn.Position().Direction().XYZ(), $"{theWhat} transformed Direction");
                    break;
                }
            }
            foreach (double u in theParams)
            {
                double aNU = theN.TransformedParameter(u, aNT);
                double aU = theP.TransformedParameter(u, aT);
                theO.Same(aNU, aU, $"{theWhat} TransformedParameter");
                SameEval(theO, aNC, aC, aU, theMaxN, $"{theWhat} transformed", theTol);
            }
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Random curves
    // ---------------------------------------------------------------------------------------------

    private sealed record BSplineData(NCollection_Array1<gp_Pnt> Poles,
                                      NCollection_Array1<double>? Weights,
                                      NCollection_Array1<double> Knots,
                                      NCollection_Array1<int> Mults,
                                      int Degree,
                                      bool Periodic)
    {
        public Geom_BSplineCurve Port() => Weights is null
            ? new Geom_BSplineCurve(Poles, Knots, Mults, Degree, Periodic)
            : new Geom_BSplineCurve(Poles, Weights, Knots, Mults, Degree, Periodic);

        public N.Geom_BSplineCurve Native() => Oracle.ToNativeCurve(Poles, Weights, Knots, Mults, Degree, Periodic);
    }

    /// <summary>
    /// A random B-spline: degree in [theMinDegree, theMaxDegree], 2 to 7 knots, inner multiplicities up to
    /// Degree - theSmoothness, periodic about one time in three (unless forced), rational half the time.
    /// </summary>
    private static BSplineData RandomBSpline(Oracle theO, bool? thePeriodic = null, int theMinDegree = 1, int theMaxDegree = 6,
                                             int theSmoothness = 0)
    {
        while (true)
        {
            int aDeg = theO.Int(theMinDegree, theMaxDegree);
            bool aPer = thePeriodic ?? theO.Int(0, 2) == 0;
            int aNbKnots = theO.Int(2, 7);
            var aKnots = new NCollection_Array1<double>(1, aNbKnots);
            var aMults = new NCollection_Array1<int>(1, aNbKnots);
            aKnots[1] = theO.Real(-10, 10);
            for (int i = 2; i <= aNbKnots; i++)
            {
                aKnots[i] = aKnots[i - 1] + theO.Real(0.1, 3);
            }
            int aMaxInner = Math.Max(1, aDeg - theSmoothness);
            for (int i = 2; i < aNbKnots; i++)
            {
                aMults[i] = theO.Int(1, aMaxInner);
            }
            if (aPer)
            {
                aMults[1] = aMults[aNbKnots] = theO.Int(1, aMaxInner);
            }
            else
            {
                aMults[1] = aMults[aNbKnots] = aDeg + 1;
            }
            int aNbPoles = BSplCLib.NbPoles(aDeg, aPer, aMults);
            if (aNbPoles < 2 || (aPer && aNbPoles <= aDeg))
            {
                continue;
            }
            var aPoles = new NCollection_Array1<gp_Pnt>(1, aNbPoles);
            for (int i = 1; i <= aNbPoles; i++)
            {
                aPoles[i] = theO.Pnt();
            }
            NCollection_Array1<double>? aWeights = null;
            if (theO.Int(0, 1) == 0)
            {
                aWeights = new NCollection_Array1<double>(1, aNbPoles);
                for (int i = 1; i <= aNbPoles; i++)
                {
                    aWeights[i] = theO.Real(0.5, 2);
                }
            }
            return new BSplineData(aPoles, aWeights, aKnots, aMults, aDeg, aPer);
        }
    }

    private static (N.Geom_BezierCurve Native, Geom_BezierCurve Port) RandomBezier(Oracle theO, int theMaxDegree = 10)
    {
        int aNbPoles = theO.Int(2, theMaxDegree + 1);
        var aPoles = new NCollection_Array1<gp_Pnt>(1, aNbPoles);
        for (int i = 1; i <= aNbPoles; i++)
        {
            aPoles[i] = theO.Pnt();
        }
        using var aNPoles = Oracle.ToNative(aPoles);
        if (theO.Int(0, 1) == 0)
        {
            return (new N.Geom_BezierCurve(aNPoles), new Geom_BezierCurve(aPoles));
        }
        var aWeights = new NCollection_Array1<double>(1, aNbPoles);
        for (int i = 1; i <= aNbPoles; i++)
        {
            aWeights[i] = theO.Real(0.5, 2);
        }
        using var aNWeights = Oracle.ToNative(aWeights);
        return (new N.Geom_BezierCurve(aNPoles, aNWeights), new Geom_BezierCurve(aPoles, aWeights));
    }

    /// <summary>A random conic or line; the second value is a sensible parameter range for unbounded curves.</summary>
    private static (N.Geom_Curve Native, Geom_Curve Port, double Range, string Kind) RandomElementary(Oracle theO, bool theNoLine = false)
    {
        (N.gp_Ax2 aNAx2, gp_Ax2 anAx2) = Ax2sFromDirection(theO);
        using var aNAx2Owner = aNAx2;
        switch (theO.Int(theNoLine ? 1 : 0, 4))
        {
            case 0:
            {
                (N.gp_Ax1 aNAx1, gp_Ax1 anAx1) = Ax1s(theO);
                using var aNAx1Owner = aNAx1;
                return (new N.Geom_Line(aNAx1), new Geom_Line(anAx1), 100, "line");
            }
            case 1:
            {
                double aR = theO.Real(0.1, 50);
                return (new N.Geom_Circle(aNAx2, aR), new Geom_Circle(anAx2, aR), 10, "circle");
            }
            case 2:
            {
                double aMaj = theO.Real(0.1, 50), aMin = theO.Real(0.05, aMaj);
                return (new N.Geom_Ellipse(aNAx2, aMaj, aMin), new Geom_Ellipse(anAx2, aMaj, aMin), 10, "ellipse");
            }
            case 3:
            {
                double aMaj = theO.Real(0.1, 50), aMin = theO.Real(0.1, 50);
                return (new N.Geom_Hyperbola(aNAx2, aMaj, aMin), new Geom_Hyperbola(anAx2, aMaj, aMin), 3, "hyperbola");
            }
            default:
            {
                double aF = theO.Real(0.01, 20);
                return (new N.Geom_Parabola(aNAx2, aF), new Geom_Parabola(anAx2, aF), 50, "parabola");
            }
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Tests
    // ---------------------------------------------------------------------------------------------

    [Fact]
    public void Line_and_conics()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            (N.Geom_Curve aN, Geom_Curve aP, double aRange, string aKind) = RandomElementary(anOracle);
            using (aN)
            {
                SameProperties(anOracle, aN, aP, aKind);
                List<double> aParams = Params(anOracle, aP, -aRange, aRange);
                foreach (double u in aParams)
                {
                    SameEval(anOracle, aN, aP, u, 5, aKind);
                }
                SameReversedAndTransformed(anOracle, aN, aP, aParams, 4, aKind);

                if (aP is Geom_Conic aConic)
                {
                    using var aNConic = N.Geom_Conic.DownCast(aN)!;
                    double aNE = 0, aPE = 0;
                    if (SameOutcome(Catch(() => aNE = aNConic.Eccentricity()), Catch(() => aPE = aConic.Eccentricity()), $"{aKind} Eccentricity"))
                    {
                        anOracle.Same(aNE, aPE, $"{aKind} Eccentricity");
                    }
                }
                switch (aP)
                {
                    case Geom_Ellipse anE:
                    {
                        using var aNE = N.Geom_Ellipse.DownCast(aN)!;
                        using var aF1 = aNE.Focus1();
                        using var aF2 = aNE.Focus2();
                        anOracle.Same(Oracle.FromNative(aF1).XYZ(), anE.Focus1().XYZ(), "ellipse Focus1");
                        anOracle.Same(Oracle.FromNative(aF2).XYZ(), anE.Focus2().XYZ(), "ellipse Focus2");
                        anOracle.Same(aNE.Focal(), anE.Focal(), "ellipse Focal");
                        anOracle.Same(aNE.Parameter(), anE.Parameter(), "ellipse Parameter");
                        break;
                    }
                    case Geom_Hyperbola aH:
                    {
                        using var aNH = N.Geom_Hyperbola.DownCast(aN)!;
                        using var aF1 = aNH.Focus1();
                        anOracle.Same(Oracle.FromNative(aF1).XYZ(), aH.Focus1().XYZ(), "hyperbola Focus1");
                        anOracle.Same(aNH.Focal(), aH.Focal(), "hyperbola Focal");
                        anOracle.Same(aNH.Parameter(), aH.Parameter(), "hyperbola Parameter");
                        break;
                    }
                    case Geom_Parabola aPb:
                    {
                        using var aNPb = N.Geom_Parabola.DownCast(aN)!;
                        using var aF = aNPb.Focus();
                        anOracle.Same(Oracle.FromNative(aF).XYZ(), aPb.Focus().XYZ(), "parabola Focus");
                        anOracle.Same(aNPb.Parameter(), aPb.Parameter(), "parabola Parameter");
                        break;
                    }
                }
            }
        }

        anOracle.Report("Geom line and conics");
    }

    [Fact]
    public void Trimmed_curves()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            N.Geom_Curve aNB;
            Geom_Curve aPB;
            double aRange;
            if (anOracle.Int(0, 2) == 0)
            {
                BSplineData aData = RandomBSpline(anOracle);
                aNB = aData.Native();
                aPB = aData.Port();
                aRange = 0;
            }
            else
            {
                (aNB, aPB, aRange, _) = RandomElementary(anOracle);
            }

            using (aNB)
            {
                double aF = aPB.FirstParameter(), aL = aPB.LastParameter();
                double aU1, aU2;
                if (aPB.IsPeriodic())
                {
                    aU1 = anOracle.Real(-10, 10);
                    aU2 = anOracle.Real(-10, 10);
                }
                else if (Precision.IsInfinite(aF))
                {
                    aU1 = anOracle.Real(-aRange, aRange);
                    aU2 = anOracle.Real(-aRange, aRange);
                }
                else
                {
                    aU1 = anOracle.Real(aF, aL);
                    aU2 = anOracle.Real(aF, aL);
                }
                bool aSense = anOracle.Int(0, 1) == 0;
                bool anAdjust = anOracle.Int(0, 3) != 0;

                using var aNT = new N.Geom_TrimmedCurve(aNB, aU1, aU2, aSense, anAdjust);
                var aPT = new Geom_TrimmedCurve(aPB, aU1, aU2, aSense, anAdjust);
                SameProperties(anOracle, aNT, aPT, "trimmed");
                using (var aNS = aNT.StartPoint())
                using (var aNE = aNT.EndPoint())
                {
                    anOracle.Same(Oracle.FromNative(aNS).XYZ(), aPT.StartPoint().XYZ(), "trimmed StartPoint");
                    anOracle.Same(Oracle.FromNative(aNE).XYZ(), aPT.EndPoint().XYZ(), "trimmed EndPoint");
                }
                List<double> aParams = Params(anOracle, aPT, 0, 0, theOutside: false);
                foreach (double u in aParams)
                {
                    SameEval(anOracle, aNT, aPT, u, 3, "trimmed");
                }
                SameReversedAndTransformed(anOracle, aNT, aPT, aParams, 3, "trimmed");

                // SetTrim on the trimmed curve, through WithTrim
                double aV1 = anOracle.Real(aPT.FirstParameter(), aPT.LastParameter());
                double aV2 = anOracle.Real(aPT.FirstParameter(), aPT.LastParameter());
                Geom_TrimmedCurve? aPT2 = null;
                if (SameOutcome(Catch(() => aNT.SetTrim(aV1, aV2, aSense, anAdjust)),
                                Catch(() => aPT2 = aPT.WithTrim(aV1, aV2, aSense, anAdjust)), "SetTrim"))
                {
                    SameProperties(anOracle, aNT, aPT2!, "SetTrim");
                    foreach (double u in Params(anOracle, aPT2!, 0, 0, theOutside: false))
                    {
                        SameEval(anOracle, aNT, aPT2!, u, 2, "SetTrim");
                    }
                }
            }
        }

        anOracle.Report("Geom_TrimmedCurve");
    }

    [Fact]
    public void Bezier_evaluation()
    {
        var anOracle = new Oracle(output);
        var aDirect = new Oracle(output, 1);
        for (int i = 0; i < Cases; i++)
        {
            (N.Geom_BezierCurve aN, Geom_BezierCurve aP) = RandomBezier(anOracle, 15);
            using (aN)
            {
                SameProperties(anOracle, aN, aP, "Bezier");
                anOracle.Same(aN.IsRational(), aP.IsRational(), "Bezier IsRational");
                List<double> aParams = Params(anOracle, aP, 0, 1);
                foreach (double u in aParams)
                {
                    SameEval(aDirect, aN, aP, u, aP.Degree() + 1, "Bezier");
                }
                SameReversedAndTransformed(anOracle, aN, aP, aParams, 3, "Bezier");

                double aNTol = 0;
                aN.Resolution(0.01, ref aNTol);
                aP.Resolution(0.01, out double aPTol);
                anOracle.Same(aNTol, aPTol, "Bezier Resolution");
            }
        }

        aDirect.Report("Geom_BezierCurve D0 to DN");
        anOracle.Report("Geom_BezierCurve evaluation (the rest)");
    }

    private static void SameBezier(Oracle theO, N.Geom_BezierCurve theN, Geom_BezierCurve theP, string theWhat)
    {
        Assert.Equal(theN.NbPoles(), theP.NbPoles());
        theO.Same(theN.IsRational(), theP.IsRational(), $"{theWhat} IsRational");
        if (theN.IsClosed() != theP.IsClosed())
        {
            using var aN1 = theN.Value(theN.FirstParameter());
            using var aN2 = theN.Value(theN.LastParameter());
            throw new Xunit.Sdk.XunitException($"{theWhat} IsClosed {theP.GetType().Name} {((object)theP as Geom_OffsetCurve)?.BasisCurve().GetType().Name}: native {Oracle.FromNative(aN1).SquareDistance(Oracle.FromNative(aN2)):R} port {theP.Value(theP.FirstParameter()).SquareDistance(theP.Value(theP.LastParameter())):R} np1 {Oracle.FromNative(aN1)} pp1 {theP.Value(theP.FirstParameter())} np2 {Oracle.FromNative(aN2)} pp2 {theP.Value(theP.LastParameter())}");
        }
        theO.Same(theN.IsClosed(), theP.IsClosed(), $"{theWhat} IsClosed");
        for (int i = 1; i <= theP.NbPoles(); i++)
        {
            using var aNP = theN.Pole(i);
            theO.Same(Oracle.FromNative(aNP).XYZ(), theP.Pole(i).XYZ(), $"{theWhat} Pole({i})");
            theO.Same(theN.Weight(i), theP.Weight(i), $"{theWhat} Weight({i})");
        }
        foreach (double u in new[] { 0.0, 0.3, 0.77, 1.0 })
        {
            SameEval(theO, theN, theP, u, 2, theWhat);
        }
    }

    [Fact]
    public void Bezier_modifications()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            (N.Geom_BezierCurve aN, Geom_BezierCurve aP) = RandomBezier(anOracle, 12);
            using (aN)
            {
                for (int aStep = 0; aStep < 4; aStep++)
                {
                    int aNb = aP.NbPoles();
                    Geom_BezierCurve? aNext = null;
                    string aWhat;
                    Exception? aNE, aPE;
                    switch (anOracle.Int(0, 7))
                    {
                        case 0:
                        {
                            int aDeg = Math.Min(Geom_BezierCurve.MaxDegree(), aP.Degree() + anOracle.Int(0, 3));
                            aWhat = $"Increase({aDeg})";
                            aNE = Catch(() => aN.Increase(aDeg));
                            aPE = Catch(() => aNext = aP.Increased(aDeg));
                            break;
                        }
                        case 1:
                        {
                            int anIndex = anOracle.Int(0, aNb);
                            gp_Pnt aPnt = anOracle.Pnt();
                            using var aNPnt = Oracle.ToNative(aPnt);
                            if (anOracle.Int(0, 1) == 0)
                            {
                                aWhat = $"InsertPoleAfter({anIndex})";
                                aNE = Catch(() => aN.InsertPoleAfter(anIndex, aNPnt));
                                aPE = Catch(() => aNext = aP.PoleInsertedAfter(anIndex, aPnt));
                            }
                            else
                            {
                                double aW = anOracle.Real(0.5, 2);
                                aWhat = $"InsertPoleAfter({anIndex}, w)";
                                aNE = Catch(() => aN.InsertPoleAfter(anIndex, aNPnt, aW));
                                aPE = Catch(() => aNext = aP.PoleInsertedAfter(anIndex, aPnt, aW));
                            }
                            break;
                        }
                        case 2:
                        {
                            int anIndex = anOracle.Int(1, aNb);
                            gp_Pnt aPnt = anOracle.Pnt();
                            double aW = anOracle.Real(0.5, 2);
                            using var aNPnt = Oracle.ToNative(aPnt);
                            aWhat = $"InsertPoleBefore({anIndex}, w)";
                            aNE = Catch(() => aN.InsertPoleBefore(anIndex, aNPnt, aW));
                            aPE = Catch(() => aNext = aP.PoleInsertedBefore(anIndex, aPnt, aW));
                            break;
                        }
                        case 3:
                        {
                            if (aNb <= 2)
                            {
                                continue;
                            }
                            int anIndex = anOracle.Int(1, aNb);
                            aWhat = $"RemovePole({anIndex})";
                            aNE = Catch(() => aN.RemovePole(anIndex));
                            aPE = Catch(() => aNext = aP.PoleRemoved(anIndex));
                            break;
                        }
                        case 4:
                        {
                            double aU1 = anOracle.Real(-0.5, 1.5), aU2 = anOracle.Real(-0.5, 1.5);
                            aWhat = $"Segment({aU1}, {aU2})";
                            aNE = Catch(() => aN.Segment(aU1, aU2));
                            aPE = Catch(() => aNext = aP.Segmented(aU1, aU2));
                            break;
                        }
                        case 5:
                        {
                            int anIndex = anOracle.Int(1, aNb);
                            gp_Pnt aPnt = anOracle.Pnt();
                            using var aNPnt = Oracle.ToNative(aPnt);
                            aWhat = $"SetPole({anIndex})";
                            aNE = Catch(() => aN.SetPole(anIndex, aNPnt));
                            aPE = Catch(() => aNext = aP.WithPole(anIndex, aPnt));
                            break;
                        }
                        case 6:
                        {
                            int anIndex = anOracle.Int(1, aNb);
                            double aW = anOracle.Int(0, 2) == 0 ? 1.0 : anOracle.Real(0.5, 2);
                            aWhat = $"SetWeight({anIndex}, {aW})";
                            aNE = Catch(() => aN.SetWeight(anIndex, aW));
                            aPE = Catch(() => aNext = aP.WithWeight(anIndex, aW));
                            break;
                        }
                        default:
                        {
                            aWhat = "Reverse";
                            aNE = Catch(() => aN.Reverse());
                            aPE = Catch(() => aNext = (Geom_BezierCurve)aP.Reversed());
                            break;
                        }
                    }
                    if (SameOutcome(aNE, aPE, aWhat))
                    {
                        aP = aNext!;
                        SameBezier(anOracle, aN, aP, aWhat);
                    }
                }
            }
        }

        anOracle.Report("Geom_BezierCurve modifications");
    }

    private static void SameBSpline(Oracle theO, N.Geom_BSplineCurve theN, Geom_BSplineCurve theP, string theWhat)
    {
        Assert.Equal(theN.Degree(), theP.Degree());
        Assert.Equal(theN.NbPoles(), theP.NbPoles());
        Assert.Equal(theN.NbKnots(), theP.NbKnots());
        theO.Same(theN.IsRational(), theP.IsRational(), $"{theWhat} IsRational");
        theO.Same(theN.IsPeriodic(), theP.IsPeriodic(), $"{theWhat} IsPeriodic");
        Assert.Equal((int)theN.KnotDistribution(), (int)theP.KnotDistribution());
        Assert.Equal(theN.FirstUKnotIndex(), theP.FirstUKnotIndex());
        Assert.Equal(theN.LastUKnotIndex(), theP.LastUKnotIndex());
        for (int i = 1; i <= theP.NbPoles(); i++)
        {
            using var aNP = theN.Pole(i);
            theO.Same(Oracle.FromNative(aNP).XYZ(), theP.Pole(i).XYZ(), $"{theWhat} Pole({i})");
            theO.Same(theN.Weight(i), theP.Weight(i), $"{theWhat} Weight({i})");
        }
        for (int i = 1; i <= theP.NbKnots(); i++)
        {
            theO.Same(theN.Knot(i), theP.Knot(i), $"{theWhat} Knot({i})");
            Assert.Equal(theN.Multiplicity(i), theP.Multiplicity(i));
        }
        using (var aNSeq = theN.KnotSequence())
        {
            double[] aNS = aNSeq.ToArray();
            ReadOnlySpan<double> aPS = theP.KnotSequence().AsSpan();
            Assert.Equal(aNS.Length, aPS.Length);
            for (int i = 0; i < aNS.Length; i++)
            {
                theO.Same(aNS[i], aPS[i], $"{theWhat} KnotSequence[{i}]");
            }
        }
        SameProperties(theO, theN, theP, theWhat);
    }

    /// <summary>The bounds, the knots and random parameters.</summary>
    private static List<double> BSplineParams(Oracle theO, Geom_BSplineCurve theP)
    {
        List<double> aParams = Params(theO, theP, 0, 0);
        for (int i = 1; i <= theP.NbKnots(); i++)
        {
            aParams.Add(theP.Knot(i));
        }
        if (theP.IsPeriodic())
        {
            double aPer = theP.Period();
            aParams.Add(theO.Real(theP.FirstParameter() - 3 * aPer, theP.LastParameter() + 3 * aPer));
        }
        return aParams;
    }

    [Fact]
    public void BSpline_evaluation()
    {
        var anOracle = new Oracle(output);
        var aDirect = new Oracle(output, 1);
        for (int i = 0; i < Cases; i++)
        {
            BSplineData aData = RandomBSpline(anOracle);
            using N.Geom_BSplineCurve aN = aData.Native();
            Geom_BSplineCurve aP = aData.Port();
            SameBSpline(anOracle, aN, aP, "BSpline");
            using (var aNS = aN.StartPoint())
            using (var aNE = aN.EndPoint())
            {
                anOracle.Same(Oracle.FromNative(aNS).XYZ(), aP.StartPoint().XYZ(), "BSpline StartPoint");
                anOracle.Same(Oracle.FromNative(aNE).XYZ(), aP.EndPoint().XYZ(), "BSpline EndPoint");
            }

            List<double> aParams = BSplineParams(anOracle, aP);
            foreach (double u in aParams)
            {
                SameEval(aDirect, aN, aP, u, aP.Degree() + 1, "BSpline");

                double aNU = u, aPU = u;
                aN.PeriodicNormalization(ref aNU);
                aP.PeriodicNormalization(ref aPU);
                anOracle.Same(aNU, aPU, "PeriodicNormalization");

                double aTol = anOracle.Int(0, 1) == 0 ? 0.0 : 1e-3;
                bool aRep = anOracle.Int(0, 1) == 0;
                int aNI1 = 0, aNI2 = 0;
                aN.LocateU(u, aTol, ref aNI1, ref aNI2, aRep);
                aP.LocateU(u, aTol, out int aPI1, out int aPI2, aRep);
                Assert.Equal((aNI1, aNI2), (aPI1, aPI2));
            }

            // Local evaluation on each span
            for (int k = aP.FirstUKnotIndex(); k < aP.LastUKnotIndex(); k++)
            {
                double u = anOracle.Real(aP.Knot(k), aP.Knot(k + 1));
                using var aNP = aN.LocalValue(u, k, k + 1);
                anOracle.Same(Oracle.FromNative(aNP).XYZ(), aP.LocalValue(u, k, k + 1).XYZ(), "LocalValue");
                using var aNP3 = new N.gp_Pnt();
                using var aNV1 = new N.gp_Vec();
                using var aNV2 = new N.gp_Vec();
                using var aNV3 = new N.gp_Vec();
                aN.LocalD3(u, k, k + 1, aNP3, aNV1, aNV2, aNV3);
                aP.LocalD3(u, k, k + 1, out gp_Pnt aP3, out gp_Vec aV1, out gp_Vec aV2, out gp_Vec aV3);
                anOracle.Same(Oracle.FromNative(aNP3).XYZ(), aP3.XYZ(), "LocalD3 P");
                anOracle.Same(FromNative(aNV1), aV1.XYZ(), "LocalD3 V1");
                anOracle.Same(FromNative(aNV2), aV2.XYZ(), "LocalD3 V2");
                anOracle.Same(FromNative(aNV3), aV3.XYZ(), "LocalD3 V3");
                int aN4 = Math.Min(4, aP.Degree() + 1);
                using var aNVN = aN.LocalDN(u, k, k + 1, aN4);
                anOracle.Same(FromNative(aNVN), aP.LocalDN(u, k, k + 1, aN4).XYZ(), "LocalDN");
            }

            double aNTol = 0;
            aN.Resolution(0.01, ref aNTol);
            aP.Resolution(0.01, out double aPTol);
            anOracle.Same(aNTol, aPTol, "BSpline Resolution");

            double aTf = anOracle.Real(aP.FirstParameter(), aP.LastParameter());
            double aTl = anOracle.Real(aTf, aP.LastParameter());
            anOracle.Same(aN.IsG1(aTf, aTl, 0.1), aP.IsG1(aTf, aTl, 0.1), "IsG1");

            SameReversedAndTransformed(anOracle, aN, aP, aParams, 3, "BSpline");
        }

        aDirect.Report("Geom_BSplineCurve D0 to DN");
        anOracle.Report("Geom_BSplineCurve evaluation (the rest)");
    }

    [Fact]
    public void BSpline_modifications()
    {
        var anOracle = new Oracle(output);
        int aFailures = 0;
        for (int i = 0; i < Cases; i++)
        {
            BSplineData aData = RandomBSpline(anOracle, theMaxDegree: 5);
            using N.Geom_BSplineCurve aN = aData.Native();
            Geom_BSplineCurve aP = aData.Port();
            for (int aStep = 0; aStep < 4; aStep++)
            {
                Geom_BSplineCurve? aNext = null;
                string aWhat;
                Exception? aNE, aPE;
                double aF = aP.FirstParameter(), aL = aP.LastParameter();
                int aNbKnots = aP.NbKnots(), aNbPoles = aP.NbPoles(), aDeg = aP.Degree();
                switch (anOracle.Int(0, 12))
                {
                    case 0:
                    {
                        double u = anOracle.Int(0, 3) == 0 ? aP.Knot(anOracle.Int(1, aNbKnots)) : anOracle.Real(aF, aL);
                        int aM = anOracle.Int(1, aDeg);
                        double aTol = anOracle.Int(0, 1) == 0 ? 0.0 : 1e-3;
                        bool anAdd = anOracle.Int(0, 1) == 0;
                        aWhat = $"InsertKnot({u}, {aM}, {aTol}, {anAdd})";
                        aNE = Catch(() => aN.InsertKnot(u, aM, aTol, anAdd));
                        aPE = Catch(() => aNext = aP.KnotInserted(u, aM, aTol, anAdd));
                        break;
                    }
                    case 1:
                    {
                        double u1 = anOracle.Real(aF, aL), u2 = anOracle.Real(aF, aL);
                        var aK = NCollection_Array1<double>.From([Math.Min(u1, u2), Math.Max(u1, u2)]);
                        var aM = NCollection_Array1<int>.From([anOracle.Int(1, aDeg), anOracle.Int(1, aDeg)]);
                        bool anAdd = anOracle.Int(0, 1) == 0;
                        using var aNK = Oracle.ToNative(aK);
                        using var aNM = Oracle.ToNative(aM);
                        aWhat = $"InsertKnots(.., {anAdd})";
                        aNE = Catch(() => aN.InsertKnots(aNK, aNM, 0.0, anAdd));
                        aPE = Catch(() => aNext = aP.KnotsInserted(aK, aM, 0.0, anAdd));
                        break;
                    }
                    case 2:
                    {
                        int anIndex = anOracle.Int(1, aNbKnots);
                        int aM = anOracle.Int(0, aDeg);
                        double aTol = new[] { 1e-7, 1e-2, 10.0, 1000.0 }[anOracle.Int(0, 3)];
                        aWhat = $"RemoveKnot({anIndex}, {aM}, {aTol})";
                        bool aNR = false;
                        Geom_BSplineCurve? aPR = null;
                        aNE = Catch(() => aNR = aN.RemoveKnot(anIndex, aM, aTol));
                        aPE = Catch(() => aPR = aP.KnotRemoved(anIndex, aM, aTol));
                        if (aNE is null && aPE is null)
                        {
                            anOracle.Same(aNR, aPR is not null, aWhat);
                            aNext = aPR ?? aP;
                        }
                        break;
                    }
                    case 3:
                    {
                        int aNewDeg = Math.Min(Geom_BSplineCurve.MaxDegree(), aDeg + anOracle.Int(0, 3));
                        aWhat = $"IncreaseDegree({aNewDeg})";
                        aNE = Catch(() => aN.IncreaseDegree(aNewDeg));
                        aPE = Catch(() => aNext = aP.DegreeIncreased(aNewDeg));
                        break;
                    }
                    case 4:
                    {
                        double u1 = anOracle.Real(aF, aL), u2 = anOracle.Real(aF, aL);
                        double aU1 = Math.Min(u1, u2), aU2 = Math.Max(u1, u2);
                        if (aP.IsPeriodic() && anOracle.Int(0, 1) == 0)
                        {
                            aU1 += aP.Period() * anOracle.Int(-2, 2);
                            aU2 = aU1 + anOracle.Real(0.1, 1) * aP.Period();
                        }
                        aWhat = $"Segment({aU1}, {aU2})";
                        aNE = Catch(() => aN.Segment(aU1, aU2));
                        aPE = Catch(() => aNext = aP.Segmented(aU1, aU2));
                        break;
                    }
                    case 5:
                    {
                        int anIndex = anOracle.Int(1, aNbPoles);
                        gp_Pnt aPnt = anOracle.Pnt();
                        using var aNPnt = Oracle.ToNative(aPnt);
                        if (anOracle.Int(0, 1) == 0)
                        {
                            aWhat = $"SetPole({anIndex})";
                            aNE = Catch(() => aN.SetPole(anIndex, aNPnt));
                            aPE = Catch(() => aNext = aP.WithPole(anIndex, aPnt));
                        }
                        else
                        {
                            double aW = anOracle.Real(0.5, 2);
                            aWhat = $"SetPole({anIndex}, {aW})";
                            aNE = Catch(() => aN.SetPole(anIndex, aNPnt, aW));
                            aPE = Catch(() => aNext = aP.WithPole(anIndex, aPnt, aW));
                        }
                        break;
                    }
                    case 6:
                    {
                        int anIndex = anOracle.Int(1, aNbPoles);
                        double aW = anOracle.Int(0, 2) == 0 ? 1.0 : anOracle.Real(0.5, 2);
                        aWhat = $"SetWeight({anIndex}, {aW})";
                        aNE = Catch(() => aN.SetWeight(anIndex, aW));
                        aPE = Catch(() => aNext = aP.WithWeight(anIndex, aW));
                        break;
                    }
                    case 7:
                    {
                        double u = anOracle.Real(aF, aL);
                        int aI1 = anOracle.Int(1, aNbPoles), aI2 = anOracle.Int(aI1, aNbPoles);
                        gp_Pnt aPnt = anOracle.Pnt();
                        using var aNPnt = Oracle.ToNative(aPnt);
                        int aNF = 0, aNL = 0, aPF = 0, aPL = 0;
                        aWhat = $"MovePoint({u}, {aI1}, {aI2})";
                        aNE = Catch(() => aN.MovePoint(u, aNPnt, aI1, aI2, ref aNF, ref aNL));
                        aPE = Catch(() => aNext = aP.PointMoved(u, aPnt, aI1, aI2, out aPF, out aPL));
                        if (aNE is null && aPE is null)
                        {
                            Assert.Equal((aNF, aNL), (aPF, aPL));
                        }
                        break;
                    }
                    case 8:
                    {
                        if (aDeg < 2)
                        {
                            // Degree 1 makes BSplCLib::MovePointAndTangent's 2 x 2 system singular for most
                            // inputs: native OCCT (release, no argument checks) inverts it anyway, the port's
                            // gp_Mat2d::Invert throws (decisions 9 and 84).
                            continue;
                        }
                        double u = anOracle.Real(aF, aL);
                        gp_Pnt aPnt = anOracle.Pnt();
                        gp_Vec aTan = anOracle.Vec();
                        int aStart = anOracle.Int(-1, 1), anEnd = anOracle.Int(-1, 1);
                        using var aNPnt = Oracle.ToNative(aPnt);
                        using var aNTan = Oracle.ToNative(aTan);
                        int aNErr = 0, aPErr = 0;
                        aWhat = $"MovePointAndTangent({u}, {aStart}, {anEnd})";
                        aNE = Catch(() => aN.MovePointAndTangent(u, aNPnt, aNTan, 1e-6, aStart, anEnd, ref aNErr));
                        aPE = Catch(() => aNext = aP.PointAndTangentMoved(u, aPnt, aTan, 1e-6, aStart, anEnd, out aPErr));
                        if (aNE is null && aPE is null)
                        {
                            Assert.Equal(aNErr, aPErr);
                        }
                        break;
                    }
                    case 9:
                    {
                        if (aP.IsPeriodic())
                        {
                            switch (anOracle.Int(0, 2))
                            {
                                case 0:
                                    aWhat = "SetNotPeriodic";
                                    aNE = Catch(() => aN.SetNotPeriodic());
                                    aPE = Catch(() => aNext = aP.WithNotPeriodic());
                                    break;
                                case 1:
                                {
                                    int anIndex = anOracle.Int(aP.FirstUKnotIndex(), aP.LastUKnotIndex());
                                    aWhat = $"SetOrigin({anIndex})";
                                    aNE = Catch(() => aN.SetOrigin(anIndex));
                                    aPE = Catch(() => aNext = aP.WithOrigin(anIndex));
                                    break;
                                }
                                default:
                                {
                                    double u = anOracle.Real(aF - aP.Period(), aL + aP.Period());
                                    aWhat = $"SetOrigin({u}, 1e-7)";
                                    aNE = Catch(() => aN.SetOrigin(u, 1e-7));
                                    aPE = Catch(() => aNext = aP.WithOrigin(u, 1e-7));
                                    break;
                                }
                            }
                        }
                        else
                        {
                            aWhat = "SetPeriodic";
                            aNE = Catch(() => aN.SetPeriodic());
                            aPE = Catch(() => aNext = aP.WithPeriodic());
                        }
                        break;
                    }
                    case 10:
                    {
                        int anIndex = anOracle.Int(1, aNbKnots);
                        int aM = anOracle.Int(1, aDeg);
                        if (anOracle.Int(0, 1) == 0)
                        {
                            aWhat = $"IncreaseMultiplicity({anIndex}, {aM})";
                            aNE = Catch(() => aN.IncreaseMultiplicity(anIndex, aM));
                            aPE = Catch(() => aNext = aP.MultiplicityIncreased(anIndex, aM));
                        }
                        else
                        {
                            int aI2 = anOracle.Int(anIndex, aNbKnots);
                            int aStepM = anOracle.Int(1, 2);
                            aWhat = $"IncrementMultiplicity({anIndex}, {aI2}, {aStepM})";
                            aNE = Catch(() => aN.IncrementMultiplicity(anIndex, aI2, aStepM));
                            aPE = Catch(() => aNext = aP.MultiplicityIncremented(anIndex, aI2, aStepM));
                        }
                        break;
                    }
                    case 11:
                    {
                        int anIndex = anOracle.Int(1, aNbKnots);
                        double aLo = anIndex > 1 ? aP.Knot(anIndex - 1) : aP.Knot(anIndex) - 1;
                        double aHi = anIndex < aNbKnots ? aP.Knot(anIndex + 1) : aP.Knot(anIndex) + 1;
                        double aK = anOracle.Real(aLo, aHi);
                        aWhat = $"SetKnot({anIndex}, {aK})";
                        aNE = Catch(() => aN.SetKnot(anIndex, aK));
                        aPE = Catch(() => aNext = aP.WithKnot(anIndex, aK));
                        break;
                    }
                    default:
                    {
                        aWhat = "Reverse";
                        aNE = Catch(() => aN.Reverse());
                        aPE = Catch(() => aNext = (Geom_BSplineCurve)aP.Reversed());
                        break;
                    }
                }

                if (SameOutcome(aNE, aPE, aWhat))
                {
                    aP = aNext!;
                    SameBSpline(anOracle, aN, aP, aWhat);
                    foreach (double u in BSplineParams(anOracle, aP))
                    {
                        SameEval(anOracle, aN, aP, u, 2, aWhat);
                    }
                }
                else
                {
                    aFailures++;
                    break;
                }
            }
        }

        output.WriteLine($"{aFailures} modifications failed in both (invalid random arguments)");
        anOracle.Report("Geom_BSplineCurve modifications");
    }

    [Fact]
    public void Offset_curves()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            N.Geom_Curve aNB;
            Geom_Curve aPB;
            switch (anOracle.Int(0, 3))
            {
                case 0:
                {
                    BSplineData aData = RandomBSpline(anOracle, theMinDegree: 3, theSmoothness: 2);
                    aNB = aData.Native();
                    aPB = aData.Port();
                    break;
                }
                case 1:
                {
                    (N.Geom_BezierCurve aNBz, Geom_BezierCurve aPBz) = RandomBezier(anOracle, 8);
                    aNB = aNBz;
                    aPB = aPBz;
                    break;
                }
                default:
                    (aNB, aPB, _, _) = RandomElementary(anOracle, theNoLine: true);
                    break;
            }

            using (aNB)
            {
                N.Geom_Curve aNBasis = aNB;
                Geom_Curve aPBasis = aPB;
                N.Geom_TrimmedCurve? aNTrim = null;
                if (anOracle.Int(0, 3) == 0 && !Precision.IsInfinite(aPB.FirstParameter()))
                {
                    // offset of a trimmed curve: the trimmed curve is kept around a copy of its basis
                    double aF = aPB.FirstParameter(), aL = aPB.LastParameter();
                    double aU1 = anOracle.Real(aF, aF + 0.4 * (aL - aF)), aU2 = anOracle.Real(aF + 0.6 * (aL - aF), aL);
                    aNTrim = new N.Geom_TrimmedCurve(aNB, aU1, aU2);
                    aNBasis = aNTrim;
                    aPBasis = new Geom_TrimmedCurve(aPB, aU1, aU2);
                }
                using (aNTrim)
                {
                    (N.gp_Dir aNDir, gp_Dir aDir) = Dirs(anOracle);
                    using var aNDirOwner = aNDir;
                    double anOffset = anOracle.Real(0.1, 20) * (anOracle.Int(0, 1) == 0 ? 1 : -1);
                    using var aNO = new N.Geom_OffsetCurve(aNBasis, anOffset, aNDir);
                    var aPO = new Geom_OffsetCurve(aPBasis, anOffset, aDir);

                    // nested: an offset of the offset folds into one offset of the basis
                    using var aNDir2 = new N.gp_Dir(aNDir);
                    double anOffset2 = anOracle.Real(-5, 5);
                    using var aNO2 = new N.Geom_OffsetCurve(aNO, anOffset2, aNDir2);
                    var aPO2 = new Geom_OffsetCurve(aPO, anOffset2, aDir);

                    foreach ((N.Geom_OffsetCurve aN, Geom_OffsetCurve aP, string aWhat) in new[] { (aNO, aPO, "offset"), (aNO2, aPO2, "offset of offset") })
                    {
                        anOracle.Same(aN.Offset(), aP.Offset(), $"{aWhat} Offset");
                        using (var aND = aN.Direction())
                        using (var aNDXYZ = aND.XYZ())
                        {
                            anOracle.Same(Oracle.FromNative(aNDXYZ), aP.Direction().XYZ(), $"{aWhat} Direction");
                        }
                        Assert.Equal((int)aN.GetBasisCurveContinuity(), (int)aP.GetBasisCurveContinuity());
                        SameProperties(anOracle, aN, aP, aWhat);
                        List<double> aParams = Params(anOracle, aP, -5, 5, theOutside: false);
                        foreach (double u in aParams)
                        {
                            SameEval(anOracle, aN, aP, u, 4, aWhat, 1e-10);
                        }
                        SameReversedAndTransformed(anOracle, aN, aP, aParams, 3, aWhat, 1e-10);
                    }

                    // the setters, through the twins
                    double anOffset3 = anOracle.Real(-10, 10);
                    (N.gp_Dir aNDir3, gp_Dir aDir3) = Dirs(anOracle);
                    using var aNDir3Owner = aNDir3;
                    aNO.SetOffsetValue(anOffset3);
                    aNO.SetDirection(aNDir3);
                    Geom_OffsetCurve aPO3 = aPO.WithOffsetValue(anOffset3).WithDirection(aDir3);
                    foreach (double u in Params(anOracle, aPO3, -5, 5, theOutside: false))
                    {
                        SameEval(anOracle, aNO, aPO3, u, 3, "offset setters", 1e-10);
                    }
                }
            }
        }

        anOracle.Report("Geom_OffsetCurve");
    }

    [Fact]
    public void Offset_at_singular_points()
    {
        // A cubic Bezier with a cusp (null first derivative at u = 0.5): the offset adjusts the derivatives there.
        var anOracle = new Oracle(output);
        for (int i = 0; i < 50; i++)
        {
            double a = anOracle.Real(1, 10), h = anOracle.Real(1, 10);
            var aPoles = NCollection_Array1<gp_Pnt>.From([new gp_Pnt(0, 0, 0), new gp_Pnt(a, h, 0), new gp_Pnt(0, h, 0), new gp_Pnt(a, 0, 0)]);
            using var aNPoles = Oracle.ToNative(aPoles);
            using var aNB = new N.Geom_BezierCurve(aNPoles);
            var aPB = new Geom_BezierCurve(aPoles);
            gp_Dir aDir = new(0, 0, 1);
            using var aNDir = Oracle.ToNative(aDir);
            double anOffset = anOracle.Real(-3, 3);
            using var aNO = new N.Geom_OffsetCurve(aNB, anOffset, aNDir, true);
            var aPO = new Geom_OffsetCurve(aPB, anOffset, aDir, true);
            foreach (double u in new[] { 0.0, 0.25, 0.5 - 1e-9, 0.5, 0.75, 1.0, anOracle.Real(0, 1) })
            {
                SameEval(anOracle, aNO, aPO, u, 4, "cusp offset", 1e-9);
            }
        }

        anOracle.Report("Geom_OffsetCurve at singular points");
    }
}
#endif
