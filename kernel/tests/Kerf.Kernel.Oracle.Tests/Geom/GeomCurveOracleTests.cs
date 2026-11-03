// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Xunit.Abstractions;
using static Kerf.Kernel.Oracle.Tests.GeomOracleInputs;

namespace Kerf.Kernel.Oracle.Tests;

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
    private static readonly int Cases = Tape.Cases(300, 34);

    // ---------------------------------------------------------------------------------------------
    // Conversions and comparisons
    // ---------------------------------------------------------------------------------------------

    private static Xyz FromNative(N.gp_Vec theV) => new(theV.X(), theV.Y(), theV.Z());

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
                    + $"port {(thePort is null ? "succeeded" : "threw " + global::Kerf.Engine.Foundation.OcctTypeNames.Of(thePort.GetType()) + " " + thePort.Message)}");
        return theNative is null;
    }

    /// <summary>
    /// D0 to D3 and DN (1 to theMaxN) at theU. Plan 3.2's third level for an offset curve (occt-issues.md, the offset
    /// curves' CalculateD3 and EvaluateDN; Kerf decision 1487): OCCT's D3 gives a D2 other than D2's and a D3 that
    /// isn't its derivative, where the basis's speed varies. There the port's D3 is compared with its own D2 (V2) and
    /// its DN(3) with D3's V3 (the unit tests check V3 against a finite difference); native's values are read as
    /// recorded.
    /// </summary>
    private static void SameEval(Oracle theO, N.Geom_Curve theN, Curve3D theP, double theU, int theMaxN, string theWhat,
                                 double theTol = Oracle.Tolerance)
    {
        bool anOffset = theP is OffsetCurve3D;
        Vector3D aD2V2 = new();
        using var aNP = new N.gp_Pnt();
        using var aNV1 = new N.gp_Vec();
        using var aNV2 = new N.gp_Vec();
        using var aNV3 = new N.gp_Vec();
        Point3D aP = new();
        Vector3D aV1 = new(), aV2 = new(), aV3 = new();

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
            aD2V2 = aV2;
        }
        bool aD3Done = false;
        if (SameOutcome(Catch(() => theN.D3(theU, aNP, aNV1, aNV2, aNV3)),
                        Catch(() => theP.D3(theU, out aP, out aV1, out aV2, out aV3)), $"{theWhat} D3({theU})"))
        {
            aD3Done = true;
            theO.Same(Oracle.FromNative(aNP).XYZ(), aP.XYZ(), $"{theWhat} D3({theU}) P", theTol);
            theO.Same(FromNative(aNV1), aV1.XYZ(), $"{theWhat} D3({theU}) V1", theTol);
            if (anOffset)
            {
                _ = FromNative(aNV2);
                _ = FromNative(aNV3);
                if (((OffsetCurve3D)theP).BasisCurve.DN(theU, 1).Magnitude > 1e-6)
                {
                    // (beside a cusp of the basis the two are as ill-conditioned as OCCT's)
                    theO.Same(aD2V2.XYZ(), aV2.XYZ(), $"{theWhat} D3({theU}) V2 against D2's", theTol);
                }
            }
            else
            {
                theO.Same(FromNative(aNV2), aV2.XYZ(), $"{theWhat} D3({theU}) V2", theTol);
                theO.Same(FromNative(aNV3), aV3.XYZ(), $"{theWhat} D3({theU}) V3", theTol);
            }
        }
        for (int k = 1; k <= theMaxN; k++)
        {
            N.gp_Vec? aNVN = null;
            Vector3D aVN = new();
            if (SameOutcome(Catch(() => aNVN = theN.DN(theU, k)), Catch(() => aVN = theP.DN(theU, k)), $"{theWhat} DN({theU}, {k})"))
            {
                if (anOffset && k == 3)
                {
                    _ = FromNative(aNVN!);
                    if (aD3Done)
                    {
                        theO.Same(aV3.XYZ(), aVN.XYZ(), $"{theWhat} DN({theU}, 3) against D3's", theTol);
                    }
                }
                else
                {
                    theO.Same(FromNative(aNVN!), aVN.XYZ(), $"{theWhat} DN({theU}, {k})", theTol);
                }
            }
            aNVN?.Dispose();
        }
    }

    /// <summary>Bounds, closure, periodicity, continuity.</summary>
    private static void SameProperties(Oracle theO, N.Geom_Curve theN, Curve3D theP, string theWhat)
    {
        theO.Same(theN.FirstParameter(), theP.FirstParameter, $"{theWhat} FirstParameter");
        theO.Same(theN.LastParameter(), theP.LastParameter, $"{theWhat} LastParameter");
        theO.Same(theN.IsClosed(), theP.IsClosed, $"{theWhat} IsClosed");
        theO.Same(theN.IsPeriodic(), theP.IsPeriodic, $"{theWhat} IsPeriodic");
        Assert.Equal((int)theN.Continuity(), (int)theP.Continuity);
        for (int k = 0; k <= 5; k++)
        {
            theO.Same(theN.IsCN(k), theP.IsCN(k), $"{theWhat} IsCN({k})");
        }
        double aNPer = 0, aPPer = 0;
        if (SameOutcome(Catch(() => aNPer = theN.Period()), Catch(() => aPPer = theP.Period), $"{theWhat} Period"))
        {
            theO.Same(aNPer, aPPer, $"{theWhat} Period");
        }
    }

    /// <summary>Parameters to evaluate at: random ones inside the bounds (or theFallback), a few outside, and the bounds.</summary>
    private static List<double> Params(Oracle theO, Curve3D theP, double theFallbackMin, double theFallbackMax, bool theOutside = true)
    {
        double aF = theP.FirstParameter, aL = theP.LastParameter;
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(aF) || global::Kerf.Engine.Foundation.Precision.IsInfinite(aL))
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
    private static (N.gp_Trsf Native, Transform3D Port) RandomTrsf(Oracle theO)
    {
        var aNative = new N.gp_Trsf();
        switch (theO.Int(0, 4))
        {
            case 0:
            {
                (N.gp_Ax1 aNAxis, Axis3D anAxis) = Ax1s(theO);
                using var aNAxisOwner = aNAxis;
                double anAngle = theO.Angle();
                aNative.SetRotation(aNAxis, anAngle);
                return (aNative, Transform3D.Rotation(anAxis, anAngle));
            }
            case 1:
            {
                Vector3D aV = theO.Vec();
                using var aNV = Oracle.ToNative(aV);
                aNative.SetTranslation(aNV);
                return (aNative, Transform3D.Translation(aV));
            }
            case 2:
            {
                Point3D aP = theO.Pnt();
                double aS = theO.Real(0.2, 3) * (theO.Int(0, 1) == 0 ? 1 : -1);
                using var aNP = Oracle.ToNative(aP);
                aNative.SetScale(aNP, aS);
                return (aNative, Transform3D.Scale(aP, aS));
            }
            case 3:
            {
                (N.gp_Ax2 aNAx, Frame3D anAx) = Ax2sFromDirection(theO);
                using var aNAxOwner = aNAx;
                aNative.SetMirror(aNAx);
                return (aNative, Transform3D.Mirror(anAx));
            }
            default:
            {
                (N.gp_Ax1 aNAxis, Axis3D anAxis) = Ax1s(theO);
                using var aNAxisOwner = aNAxis;
                double anAngle = theO.Angle();
                Point3D aP = theO.Pnt();
                double aS = theO.Real(0.2, 3);
                using var aNP = Oracle.ToNative(aP);
                using var aNS = new N.gp_Trsf();
                aNative.SetRotation(aNAxis, anAngle);
                aNS.SetScale(aNP, aS);
                aNative.Multiply(aNS);
                return (aNative, Transform3D.Rotation(anAxis, anAngle).Multiplied(Transform3D.Scale(aP, aS)));
            }
        }
    }

    /// <summary>Reversal and transformation, compared by evaluation.</summary>
    private static void SameReversedAndTransformed(Oracle theO, N.Geom_Curve theN, Curve3D theP, List<double> theParams,
                                                   int theMaxN, string theWhat, double theTol = Oracle.Tolerance)
    {
        foreach (double u in theParams)
        {
            theO.Same(theN.ReversedParameter(u), theP.ReversedParameter(u), $"{theWhat} ReversedParameter");
        }

        using (var aNR = theN.Reversed()!)
        {
            Curve3D aR = theP.Reversed();
            SameProperties(theO, aNR, aR, $"{theWhat} reversed");
            foreach (double u in theParams)
            {
                SameEval(theO, aNR, aR, theP.ReversedParameter(u), theMaxN, $"{theWhat} reversed", theTol);
            }
        }

        (N.gp_Trsf aNT, Transform3D aT) = RandomTrsf(theO);
        using (aNT)
        {
            theO.Same(theN.ParametricTransformation(aNT), theP.ParametricTransformation(aT), $"{theWhat} ParametricTransformation");
            using var aNG = theN.Transformed(aNT);
            using var aNC = N.Geom_Curve.DownCast(aNG)!;
            var aC = (Curve3D)theP.Transformed(aT);
            SameProperties(theO, aNC, aC, $"{theWhat} transformed");
            // What Transform changes, at the default tolerance: the poles, or the conic's or line's axes
            switch (aC)
            {
                case BezierCurve3D aBz:
                {
                    using var aNBz = N.Geom_BezierCurve.DownCast(aNC)!;
                    for (int i = 1; i <= aBz.PoleCount; i++)
                    {
                        using var aNPole = aNBz.Pole(i);
                        theO.Same(Oracle.FromNative(aNPole).XYZ(), aBz.Pole(i).XYZ(), $"{theWhat} transformed Pole({i})");
                    }
                    break;
                }
                case BSplineCurve3D aBs:
                {
                    using var aNBs = N.Geom_BSplineCurve.DownCast(aNC)!;
                    for (int i = 1; i <= aBs.PoleCount; i++)
                    {
                        using var aNPole = aNBs.Pole(i);
                        theO.Same(Oracle.FromNative(aNPole).XYZ(), aBs.Pole(i).XYZ(), $"{theWhat} transformed Pole({i})");
                    }
                    break;
                }
                case ConicCurve3D aCn:
                {
                    using var aNCn = N.Geom_Conic.DownCast(aNC)!;
                    using var aNPos = aNCn.Position();
                    using var aNLoc = aNPos.Location();
                    using var aNX = aNPos.XDirection();
                    using var aNXX = aNX.XYZ();
                    using var aNZ = aNPos.Direction();
                    using var aNZX = aNZ.XYZ();
                    theO.Same(Oracle.FromNative(aNLoc).XYZ(), aCn.Position.Location.XYZ(), $"{theWhat} transformed Location");
                    theO.Same(Oracle.FromNative(aNXX), aCn.Position.XDirection.XYZ(), $"{theWhat} transformed XDirection");
                    theO.Same(Oracle.FromNative(aNZX), aCn.Position.Direction.XYZ(), $"{theWhat} transformed Direction");
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

    private sealed record BSplineData(NCollection_Array1<Point3D> Poles,
                                      NCollection_Array1<double>? Weights,
                                      NCollection_Array1<double> Knots,
                                      NCollection_Array1<int> Mults,
                                      int Degree,
                                      bool Periodic)
    {
        public BSplineCurve3D Port() => Weights is null
            ? new BSplineCurve3D(Poles, Knots, Mults, Degree, Periodic)
            : new BSplineCurve3D(Poles, Weights, Knots, Mults, Degree, Periodic);

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
            int aNbPoles = global::Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(aDeg, aPer, aMults);
            if (aNbPoles < 2 || (aPer && aNbPoles <= aDeg))
            {
                continue;
            }
            var aPoles = new NCollection_Array1<Point3D>(1, aNbPoles);
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

    private static (N.Geom_BezierCurve Native, BezierCurve3D Port) RandomBezier(Oracle theO, int theMaxDegree = 10)
    {
        int aNbPoles = theO.Int(2, theMaxDegree + 1);
        var aPoles = new NCollection_Array1<Point3D>(1, aNbPoles);
        for (int i = 1; i <= aNbPoles; i++)
        {
            aPoles[i] = theO.Pnt();
        }
        using var aNPoles = Oracle.ToNative(aPoles);
        if (theO.Int(0, 1) == 0)
        {
            return (new N.Geom_BezierCurve(aNPoles), new BezierCurve3D(aPoles));
        }
        var aWeights = new NCollection_Array1<double>(1, aNbPoles);
        for (int i = 1; i <= aNbPoles; i++)
        {
            aWeights[i] = theO.Real(0.5, 2);
        }
        using var aNWeights = Oracle.ToNative(aWeights);
        return (new N.Geom_BezierCurve(aNPoles, aNWeights), new BezierCurve3D(aPoles, aWeights));
    }

    /// <summary>A random conic or line; the second value is a sensible parameter range for unbounded curves.</summary>
    private static (N.Geom_Curve Native, Curve3D Port, double Range, string Kind) RandomElementary(Oracle theO, bool theNoLine = false)
    {
        (N.gp_Ax2 aNAx2, Frame3D anAx2) = Ax2sFromDirection(theO);
        using var aNAx2Owner = aNAx2;
        switch (theO.Int(theNoLine ? 1 : 0, 4))
        {
            case 0:
            {
                (N.gp_Ax1 aNAx1, Axis3D anAx1) = Ax1s(theO);
                using var aNAx1Owner = aNAx1;
                return (new N.Geom_Line(aNAx1), new LineCurve3D(anAx1), 100, "line");
            }
            case 1:
            {
                double aR = theO.Real(0.1, 50);
                return (new N.Geom_Circle(aNAx2, aR), new CircleCurve3D(anAx2, aR), 10, "circle");
            }
            case 2:
            {
                double aMaj = theO.Real(0.1, 50), aMin = theO.Real(0.05, aMaj);
                return (new N.Geom_Ellipse(aNAx2, aMaj, aMin), new EllipseCurve3D(anAx2, aMaj, aMin), 10, "ellipse");
            }
            case 3:
            {
                double aMaj = theO.Real(0.1, 50), aMin = theO.Real(0.1, 50);
                return (new N.Geom_Hyperbola(aNAx2, aMaj, aMin), new HyperbolaCurve3D(anAx2, aMaj, aMin), 3, "hyperbola");
            }
            default:
            {
                double aF = theO.Real(0.01, 20);
                return (new N.Geom_Parabola(aNAx2, aF), new ParabolaCurve3D(anAx2, aF), 50, "parabola");
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
            (N.Geom_Curve aN, Curve3D aP, double aRange, string aKind) = RandomElementary(anOracle);
            using (aN)
            {
                SameProperties(anOracle, aN, aP, aKind);
                List<double> aParams = Params(anOracle, aP, -aRange, aRange);
                foreach (double u in aParams)
                {
                    SameEval(anOracle, aN, aP, u, 5, aKind);
                }
                SameReversedAndTransformed(anOracle, aN, aP, aParams, 4, aKind);

                if (aP is ConicCurve3D aConic)
                {
                    using var aNConic = N.Geom_Conic.DownCast(aN)!;
                    double aNE = 0, aPE = 0;
                    if (SameOutcome(Catch(() => aNE = aNConic.Eccentricity()), Catch(() => aPE = aConic.Eccentricity), $"{aKind} Eccentricity"))
                    {
                        anOracle.Same(aNE, aPE, $"{aKind} Eccentricity");
                    }
                }
                switch (aP)
                {
                    case EllipseCurve3D anE:
                    {
                        using var aNE = N.Geom_Ellipse.DownCast(aN)!;
                        using var aF1 = aNE.Focus1();
                        using var aF2 = aNE.Focus2();
                        anOracle.Same(Oracle.FromNative(aF1).XYZ(), anE.Focus1.XYZ(), "ellipse Focus1");
                        anOracle.Same(Oracle.FromNative(aF2).XYZ(), anE.Focus2.XYZ(), "ellipse Focus2");
                        anOracle.Same(aNE.Focal(), anE.Focal, "ellipse Focal");
                        anOracle.Same(aNE.Parameter(), anE.Parameter, "ellipse Parameter");
                        break;
                    }
                    case HyperbolaCurve3D aH:
                    {
                        using var aNH = N.Geom_Hyperbola.DownCast(aN)!;
                        using var aF1 = aNH.Focus1();
                        anOracle.Same(Oracle.FromNative(aF1).XYZ(), aH.Focus1.XYZ(), "hyperbola Focus1");
                        anOracle.Same(aNH.Focal(), aH.Focal, "hyperbola Focal");
                        anOracle.Same(aNH.Parameter(), aH.Parameter, "hyperbola Parameter");
                        break;
                    }
                    case ParabolaCurve3D aPb:
                    {
                        using var aNPb = N.Geom_Parabola.DownCast(aN)!;
                        using var aF = aNPb.Focus();
                        anOracle.Same(Oracle.FromNative(aF).XYZ(), aPb.Focus.XYZ(), "parabola Focus");
                        anOracle.Same(aNPb.Parameter(), aPb.Parameter, "parabola Parameter");
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
            Curve3D aPB;
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
                double aF = aPB.FirstParameter, aL = aPB.LastParameter;
                double aU1, aU2;
                if (aPB.IsPeriodic)
                {
                    aU1 = anOracle.Real(-10, 10);
                    aU2 = anOracle.Real(-10, 10);
                }
                else if (global::Kerf.Engine.Foundation.Precision.IsInfinite(aF))
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
                var aPT = new TrimmedCurve3D(aPB, aU1, aU2, aSense, anAdjust);
                SameProperties(anOracle, aNT, aPT, "trimmed");
                using (var aNS = aNT.StartPoint())
                using (var aNE = aNT.EndPoint())
                {
                    anOracle.Same(Oracle.FromNative(aNS).XYZ(), aPT.StartPoint.XYZ(), "trimmed StartPoint");
                    anOracle.Same(Oracle.FromNative(aNE).XYZ(), aPT.EndPoint.XYZ(), "trimmed EndPoint");
                }
                List<double> aParams = Params(anOracle, aPT, 0, 0, theOutside: false);
                foreach (double u in aParams)
                {
                    SameEval(anOracle, aNT, aPT, u, 3, "trimmed");
                }
                SameReversedAndTransformed(anOracle, aNT, aPT, aParams, 3, "trimmed");

                // SetTrim on the trimmed curve, through WithTrim
                double aV1 = anOracle.Real(aPT.FirstParameter, aPT.LastParameter);
                double aV2 = anOracle.Real(aPT.FirstParameter, aPT.LastParameter);
                TrimmedCurve3D? aPT2 = null;
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
            (N.Geom_BezierCurve aN, BezierCurve3D aP) = RandomBezier(anOracle, 15);
            using (aN)
            {
                SameProperties(anOracle, aN, aP, "Bezier");
                anOracle.Same(aN.IsRational(), aP.IsRational, "Bezier IsRational");
                List<double> aParams = Params(anOracle, aP, 0, 1);
                foreach (double u in aParams)
                {
                    SameEval(aDirect, aN, aP, u, aP.Degree + 1, "Bezier");
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

    private static void SameBezier(Oracle theO, N.Geom_BezierCurve theN, BezierCurve3D theP, string theWhat)
    {
        Assert.Equal(theN.NbPoles(), theP.PoleCount);
        theO.Same(theN.IsRational(), theP.IsRational, $"{theWhat} IsRational");
        if (theN.IsClosed() != theP.IsClosed)
        {
            using var aN1 = theN.Value(theN.FirstParameter());
            using var aN2 = theN.Value(theN.LastParameter());
            throw new Xunit.Sdk.XunitException($"{theWhat} IsClosed {Kerf.Engine.Foundation.OcctTypeNames.Of(theP.GetType())} {((object)theP as OffsetCurve3D)?.BasisCurve.GetType().Name}: native {Oracle.FromNative(aN1).SquareDistanceTo(Oracle.FromNative(aN2)):R} port {theP.Value(theP.FirstParameter).SquareDistanceTo(theP.Value(theP.LastParameter)):R} np1 {Oracle.FromNative(aN1)} pp1 {theP.Value(theP.FirstParameter)} np2 {Oracle.FromNative(aN2)} pp2 {theP.Value(theP.LastParameter)}");
        }
        theO.Same(theN.IsClosed(), theP.IsClosed, $"{theWhat} IsClosed");
        for (int i = 1; i <= theP.PoleCount; i++)
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
            (N.Geom_BezierCurve aN, BezierCurve3D aP) = RandomBezier(anOracle, 12);
            using (aN)
            {
                for (int aStep = 0; aStep < 4; aStep++)
                {
                    int aNb = aP.PoleCount;
                    BezierCurve3D? aNext = null;
                    string aWhat;
                    Exception? aNE, aPE;
                    switch (anOracle.Int(0, 7))
                    {
                        case 0:
                        {
                            int aDeg = Math.Min(BezierCurve3D.MaxDegree, aP.Degree + anOracle.Int(0, 3));
                            aWhat = $"Increase({aDeg})";
                            aNE = Catch(() => aN.Increase(aDeg));
                            aPE = Catch(() => aNext = aP.Increased(aDeg));
                            break;
                        }
                        case 1:
                        {
                            int anIndex = anOracle.Int(0, aNb);
                            Point3D aPnt = anOracle.Pnt();
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
                            Point3D aPnt = anOracle.Pnt();
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
                            Point3D aPnt = anOracle.Pnt();
                            using var aNPnt = Oracle.ToNative(aPnt);
                            aWhat = $"SetPole({anIndex})";
                            aNE = Catch(() => aN.SetPole(anIndex, aNPnt));
                            aPE = Catch(() => aNext = aP.WithPoleOneBased(anIndex, aPnt));
                            break;
                        }
                        case 6:
                        {
                            int anIndex = anOracle.Int(1, aNb);
                            double aW = anOracle.Int(0, 2) == 0 ? 1.0 : anOracle.Real(0.5, 2);
                            aWhat = $"SetWeight({anIndex}, {aW})";
                            aNE = Catch(() => aN.SetWeight(anIndex, aW));
                            aPE = Catch(() => aNext = aP.WithWeightOneBased(anIndex, aW));
                            break;
                        }
                        default:
                        {
                            aWhat = "Reverse";
                            aNE = Catch(() => aN.Reverse());
                            aPE = Catch(() => aNext = (BezierCurve3D)aP.Reversed());
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

    private static void SameBSpline(Oracle theO, N.Geom_BSplineCurve theN, BSplineCurve3D theP, string theWhat)
    {
        Assert.Equal(theN.Degree(), theP.Degree);
        Assert.Equal(theN.NbPoles(), theP.PoleCount);
        Assert.Equal(theN.NbKnots(), theP.KnotCount);
        theO.Same(theN.IsRational(), theP.IsRational, $"{theWhat} IsRational");
        theO.Same(theN.IsPeriodic(), theP.IsPeriodic, $"{theWhat} IsPeriodic");
        Assert.Equal((int)theN.KnotDistribution(), (int)theP.KnotDistribution);
        Assert.Equal(theN.FirstUKnotIndex(), theP.FirstUKnotIndex());
        Assert.Equal(theN.LastUKnotIndex(), theP.LastUKnotIndex());
        for (int i = 1; i <= theP.PoleCount; i++)
        {
            using var aNP = theN.Pole(i);
            theO.Same(Oracle.FromNative(aNP).XYZ(), theP.Pole(i).XYZ(), $"{theWhat} Pole({i})");
            theO.Same(theN.Weight(i), theP.Weight(i), $"{theWhat} Weight({i})");
        }
        for (int i = 1; i <= theP.KnotCount; i++)
        {
            theO.Same(theN.Knot(i), theP.Knot(i), $"{theWhat} Knot({i})");
            Assert.Equal(theN.Multiplicity(i), theP.Multiplicity(i));
        }
        using (var aNSeq = theN.KnotSequence())
        {
            double[] aNS = aNSeq.ToArray();
            ReadOnlySpan<double> aPS = theP.KnotSequence.ToArray();
            Assert.Equal(aNS.Length, aPS.Length);
            for (int i = 0; i < aNS.Length; i++)
            {
                theO.Same(aNS[i], aPS[i], $"{theWhat} KnotSequence[{i}]");
            }
        }
        SameProperties(theO, theN, theP, theWhat);
    }

    /// <summary>The bounds, the knots and random parameters.</summary>
    private static List<double> BSplineParams(Oracle theO, BSplineCurve3D theP)
    {
        List<double> aParams = Params(theO, theP, 0, 0);
        for (int i = 1; i <= theP.KnotCount; i++)
        {
            aParams.Add(theP.Knot(i));
        }
        if (theP.IsPeriodic)
        {
            double aPer = theP.Period;
            aParams.Add(theO.Real(theP.FirstParameter - 3 * aPer, theP.LastParameter + 3 * aPer));
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
            BSplineCurve3D aP = aData.Port();
            SameBSpline(anOracle, aN, aP, "BSpline");
            using (var aNS = aN.StartPoint())
            using (var aNE = aN.EndPoint())
            {
                anOracle.Same(Oracle.FromNative(aNS).XYZ(), aP.StartPoint.XYZ(), "BSpline StartPoint");
                anOracle.Same(Oracle.FromNative(aNE).XYZ(), aP.EndPoint.XYZ(), "BSpline EndPoint");
            }

            List<double> aParams = BSplineParams(anOracle, aP);
            foreach (double u in aParams)
            {
                SameEval(aDirect, aN, aP, u, aP.Degree + 1, "BSpline");

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
                aP.LocalD3(u, k, k + 1, out Point3D aP3, out Vector3D aV1, out Vector3D aV2, out Vector3D aV3);
                anOracle.Same(Oracle.FromNative(aNP3).XYZ(), aP3.XYZ(), "LocalD3 P");
                anOracle.Same(FromNative(aNV1), aV1.XYZ(), "LocalD3 V1");
                anOracle.Same(FromNative(aNV2), aV2.XYZ(), "LocalD3 V2");
                anOracle.Same(FromNative(aNV3), aV3.XYZ(), "LocalD3 V3");
                int aN4 = Math.Min(4, aP.Degree + 1);
                using var aNVN = aN.LocalDN(u, k, k + 1, aN4);
                anOracle.Same(FromNative(aNVN), aP.LocalDN(u, k, k + 1, aN4).XYZ(), "LocalDN");
            }

            double aNTol = 0;
            aN.Resolution(0.01, ref aNTol);
            aP.Resolution(0.01, out double aPTol);
            anOracle.Same(aNTol, aPTol, "BSpline Resolution");

            double aTf = anOracle.Real(aP.FirstParameter, aP.LastParameter);
            double aTl = anOracle.Real(aTf, aP.LastParameter);
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
            BSplineCurve3D aP = aData.Port();
            for (int aStep = 0; aStep < 4; aStep++)
            {
                BSplineCurve3D? aNext = null;
                string aWhat;
                Exception? aNE, aPE;
                double aF = aP.FirstParameter, aL = aP.LastParameter;
                int aNbKnots = aP.KnotCount, aNbPoles = aP.PoleCount, aDeg = aP.Degree;
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
                        BSplineCurve3D? aPR = null;
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
                        int aNewDeg = Math.Min(BSplineCurve3D.MaxDegree, aDeg + anOracle.Int(0, 3));
                        aWhat = $"IncreaseDegree({aNewDeg})";
                        aNE = Catch(() => aN.IncreaseDegree(aNewDeg));
                        aPE = Catch(() => aNext = aP.DegreeIncreased(aNewDeg));
                        break;
                    }
                    case 4:
                    {
                        double u1 = anOracle.Real(aF, aL), u2 = anOracle.Real(aF, aL);
                        double aU1 = Math.Min(u1, u2), aU2 = Math.Max(u1, u2);
                        if (aP.IsPeriodic && anOracle.Int(0, 1) == 0)
                        {
                            aU1 += aP.Period * anOracle.Int(-2, 2);
                            aU2 = aU1 + anOracle.Real(0.1, 1) * aP.Period;
                        }
                        aWhat = $"Segment({aU1}, {aU2})";
                        aNE = Catch(() => aN.Segment(aU1, aU2));
                        aPE = Catch(() => aNext = aP.Segmented(aU1, aU2));
                        break;
                    }
                    case 5:
                    {
                        int anIndex = anOracle.Int(1, aNbPoles);
                        Point3D aPnt = anOracle.Pnt();
                        using var aNPnt = Oracle.ToNative(aPnt);
                        if (anOracle.Int(0, 1) == 0)
                        {
                            aWhat = $"SetPole({anIndex})";
                            aNE = Catch(() => aN.SetPole(anIndex, aNPnt));
                            aPE = Catch(() => aNext = aP.WithPoleOneBased(anIndex, aPnt));
                        }
                        else
                        {
                            double aW = anOracle.Real(0.5, 2);
                            aWhat = $"SetPole({anIndex}, {aW})";
                            aNE = Catch(() => aN.SetPole(anIndex, aNPnt, aW));
                            aPE = Catch(() => aNext = aP.WithPoleOneBased(anIndex, aPnt, aW));
                        }
                        break;
                    }
                    case 6:
                    {
                        int anIndex = anOracle.Int(1, aNbPoles);
                        double aW = anOracle.Int(0, 2) == 0 ? 1.0 : anOracle.Real(0.5, 2);
                        aWhat = $"SetWeight({anIndex}, {aW})";
                        aNE = Catch(() => aN.SetWeight(anIndex, aW));
                        aPE = Catch(() => aNext = aP.WithWeightOneBased(anIndex, aW));
                        break;
                    }
                    case 7:
                    {
                        double u = anOracle.Real(aF, aL);
                        int aI1 = anOracle.Int(1, aNbPoles), aI2 = anOracle.Int(aI1, aNbPoles);
                        Point3D aPnt = anOracle.Pnt();
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
                        Point3D aPnt = anOracle.Pnt();
                        Vector3D aTan = anOracle.Vec();
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
                        if (aP.IsPeriodic)
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
                                    double u = anOracle.Real(aF - aP.Period, aL + aP.Period);
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
                        aPE = Catch(() => aNext = aP.WithKnotOneBased(anIndex, aK));
                        break;
                    }
                    default:
                    {
                        aWhat = "Reverse";
                        aNE = Catch(() => aN.Reverse());
                        aPE = Catch(() => aNext = (BSplineCurve3D)aP.Reversed());
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
            Curve3D aPB;
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
                    (N.Geom_BezierCurve aNBz, BezierCurve3D aPBz) = RandomBezier(anOracle, 8);
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
                Curve3D aPBasis = aPB;
                N.Geom_TrimmedCurve? aNTrim = null;
                if (anOracle.Int(0, 3) == 0 && !global::Kerf.Engine.Foundation.Precision.IsInfinite(aPB.FirstParameter))
                {
                    // offset of a trimmed curve: the trimmed curve is kept around a copy of its basis
                    double aF = aPB.FirstParameter, aL = aPB.LastParameter;
                    double aU1 = anOracle.Real(aF, aF + 0.4 * (aL - aF)), aU2 = anOracle.Real(aF + 0.6 * (aL - aF), aL);
                    aNTrim = new N.Geom_TrimmedCurve(aNB, aU1, aU2);
                    aNBasis = aNTrim;
                    aPBasis = new TrimmedCurve3D(aPB, aU1, aU2);
                }
                using (aNTrim)
                {
                    (N.gp_Dir aNDir, Direction3D aDir) = Dirs(anOracle);
                    using var aNDirOwner = aNDir;
                    double anOffset = anOracle.Real(0.1, 20) * (anOracle.Int(0, 1) == 0 ? 1 : -1);
                    using var aNO = new N.Geom_OffsetCurve(aNBasis, anOffset, aNDir);
                    var aPO = new OffsetCurve3D(aPBasis, anOffset, aDir);

                    // nested: an offset of the offset folds into one offset of the basis
                    using var aNDir2 = new N.gp_Dir(aNDir);
                    double anOffset2 = anOracle.Real(-5, 5);
                    using var aNO2 = new N.Geom_OffsetCurve(aNO, anOffset2, aNDir2);
                    var aPO2 = new OffsetCurve3D(aPO, anOffset2, aDir);

                    foreach ((N.Geom_OffsetCurve aN, OffsetCurve3D aP, string aWhat) in new[] { (aNO, aPO, "offset"), (aNO2, aPO2, "offset of offset") })
                    {
                        anOracle.Same(aN.Offset(), aP.Offset, $"{aWhat} Offset");
                        using (var aND = aN.Direction())
                        using (var aNDXYZ = aND.XYZ())
                        {
                            anOracle.Same(Oracle.FromNative(aNDXYZ), aP.Direction.XYZ(), $"{aWhat} Direction");
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
                    (N.gp_Dir aNDir3, Direction3D aDir3) = Dirs(anOracle);
                    using var aNDir3Owner = aNDir3;
                    aNO.SetOffsetValue(anOffset3);
                    aNO.SetDirection(aNDir3);
                    OffsetCurve3D aPO3 = aPO.WithOffsetValue(anOffset3).WithDirection(aDir3);
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
            var aPoles = NCollection_Array1<Point3D>.From([new Point3D(0, 0, 0), new Point3D(a, h, 0), new Point3D(0, h, 0), new Point3D(a, 0, 0)]);
            using var aNPoles = Oracle.ToNative(aPoles);
            using var aNB = new N.Geom_BezierCurve(aNPoles);
            var aPB = new BezierCurve3D(aPoles);
            Direction3D aDir = new(0, 0, 1);
            using var aNDir = Oracle.ToNative(aDir);
            double anOffset = anOracle.Real(-3, 3);
            using var aNO = new N.Geom_OffsetCurve(aNB, anOffset, aNDir, true);
            var aPO = new OffsetCurve3D(aPB, anOffset, aDir, true);
            foreach (double u in new[] { 0.0, 0.25, 0.5 - 1e-9, 0.5, 0.75, 1.0, anOracle.Real(0, 1) })
            {
                SameEval(anOracle, aNO, aPO, u, 4, "cusp offset", 1e-9);
            }
        }

        anOracle.Report("Geom_OffsetCurve at singular points");
    }
}
