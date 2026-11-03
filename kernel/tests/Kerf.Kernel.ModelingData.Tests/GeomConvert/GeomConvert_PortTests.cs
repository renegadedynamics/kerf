// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Geometry.Continuity;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// OCCT.NET's own tests of GeomConvert's statics, GeomConvert_ApproxCurve, GeomConvert_ApproxSurface,
/// GeomConvert_BSplineCurveKnotSplitting, GeomConvert_BSplineCurveToBezierCurve and the freezing rules of
/// GeomConvert_CompCurveToBSplineCurve (OCCT's GTests cover little of them).
/// </summary>
public class GeomConvert_PortTests
{
    private const double Tol = 1e-9;

    private static Point3D P(double theX, double theY, double theZ = 0.0) => new(theX, theY, theZ);

    private static BSplineCurve3D Cubic()
    {
        // A smooth cubic with three spans
        NCollection_Array1<Point3D> aPoles = NCollection_Array1<Point3D>.From([
            P(0, 0, 0), P(1, 2, 1), P(3, 3, 0), P(5, 1, -1), P(7, 2, 0), P(8, 0, 1)]);
        return new BSplineCurve3D(aPoles, NCollection_Array1<double>.From([0.0, 1.0, 2.0, 3.0]),
                                     NCollection_Array1<int>.From([4, 1, 1, 4]), 3);
    }

    private static BSplineCurve3D Segment(Point3D theP1, Point3D theP2) =>
        new(NCollection_Array1<Point3D>.From([theP1, theP2]), NCollection_Array1<double>.From([0.0, 1.0]),
            NCollection_Array1<int>.From([2, 2]), 1);

    private static void AssertSamePoints(Curve3D theExpected, double theU1, double theU2, Curve3D theActual,
                                         double theV1, double theV2, double theTol)
    {
        for (int i = 0; i <= 20; i++)
        {
            Point3D aP = theExpected.Value(theU1 + (theU2 - theU1) * i / 20.0);
            Point3D aQ = theActual.Value(theV1 + (theV2 - theV1) * i / 20.0);
            Assert.True(aP.Distance(aQ) < theTol, $"point {i}: {aP} vs {aQ}");
        }
    }

    [Fact]
    public void CurveToBSplineCurve_WholeCircle_IsPeriodic_AndLongRationalC1Arc_IsJoined()
    {
        CircleCurve3D aCircle = new(new Frame3D(P(1, 2, 3), new Direction3D(0, 1, 1), new Direction3D(1, 0, 0)), 4.0);
        BSplineCurve3D aWhole = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(aCircle);
        Assert.True(aWhole.IsPeriodic);
        Assert.False(aWhole.IsFrozen);
        Assert.True(aWhole.StartPoint().Distance(aCircle.Value(0.0)) < Tol);
        for (int i = 0; i <= 20; i++)
        {
            Assert.Equal(4.0, aWhole.Value(i * 0.3).Distance(P(1, 2, 3)), 1e-12);
        }

        // An arc of 6.2 radians with RationalC1: two halves joined by GeomConvert_CompCurveToBSplineCurve
        BSplineCurve3D aLong = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(new TrimmedCurve3D(aCircle, 0.0, 6.2), RationalC1);
        Assert.True(aLong.StartPoint().Distance(aCircle.Value(0.0)) < Tol);
        Assert.True(aLong.EndPoint().Distance(aCircle.Value(6.2)) < Tol);
        Assert.True(aLong.Continuity() >= C1);
    }

    [Fact]
    public void CurveToBSplineCurve_FrozenTrimmedPeriodicBSpline_IsCopied()
    {
        BSplineCurve3D aPeriodic = Cubic().KnotInserted(1.5, 1, 0.0, false);
        aPeriodic = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(new TrimmedCurve3D(aPeriodic, 0.0, 3.0));
        aPeriodic.Freeze();
        TrimmedCurve3D aTrimmed = new(aPeriodic, 0.5, 2.5);
        BSplineCurve3D aResult = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(aTrimmed);
        Assert.False(aResult.IsFrozen);
        AssertSamePoints(aPeriodic, 0.5, 2.5, aResult, 0.5, 2.5, Tol);
        Assert.Equal(3.0, aPeriodic.LastParameter());
    }

    [Fact]
    public void CurveToBSplineCurve_OffsetCurve_IsApproximated_AndUnboundedCurvesThrow()
    {
        OffsetCurve3D anOffset = new(Cubic(), 0.5, new Direction3D(0, 0, 1));
        BSplineCurve3D aResult = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(anOffset);
        Assert.False(aResult.IsFrozen);
        for (int i = 0; i <= 10; i++)
        {
            double aU = 0.3 * i;
            Assert.True(aResult.Value(aU).Distance(anOffset.Value(aU)) < 1e-3);
        }

        Assert.ThrowsAny<DomainErrorException>(() => global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(new LineCurve3D(P(0, 0), new Direction3D(1, 0, 0))));
        Assert.ThrowsAny<DomainErrorException>(
            () => global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(new ParabolaCurve3D(new Frame3D(P(0, 0), new Direction3D(0, 0, 1)), 1.0)));
    }

    [Fact]
    public void SplitBSplineCurve_ByKnotsAndParameters()
    {
        BSplineCurve3D aC = Cubic();
        BSplineCurve3D aPart = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SplitBSplineCurve(aC, 2, 3);
        Assert.Equal(1.0, aPart.FirstParameter());
        Assert.Equal(2.0, aPart.LastParameter());
        AssertSamePoints(aC, 1.0, 2.0, aPart, 1.0, 2.0, Tol);

        BSplineCurve3D aReversed = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SplitBSplineCurve(aC, 3, 2);
        Assert.True(aReversed.StartPoint().Distance(aC.Value(2.0)) < Tol);
        Assert.True(aReversed.EndPoint().Distance(aC.Value(1.0)) < Tol);

        BSplineCurve3D aByParams = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SplitBSplineCurve(aC, 2.5, 0.5, 1e-9);
        Assert.True(aByParams.StartPoint().Distance(aC.Value(2.5)) < Tol);
        Assert.True(aByParams.EndPoint().Distance(aC.Value(0.5)) < Tol);

        // GeomConvert throws Standard_DomainError for both (Geom2dConvert: Standard_OutOfRange for the second)
        Assert.ThrowsAny<DomainErrorException>(() => global::Kerf.Engine.Geometry.Conversion.GeomConvert.SplitBSplineCurve(aC, 2, 2));
        Assert.ThrowsAny<DomainErrorException>(() => global::Kerf.Engine.Geometry.Conversion.GeomConvert.SplitBSplineCurve(aC, 0, 2));
    }

    private static BSplineSurface BiCubic()
    {
        NCollection_Array2<Point3D> aPoles = new(1, 5, 1, 4);
        for (int i = 1; i <= 5; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                aPoles[i, j] = P(i, j, Math.Sin(i + 2.0 * j));
            }
        }

        return new BSplineSurface(aPoles, NCollection_Array1<double>.From([0.0, 1.0, 3.0]),
                                       NCollection_Array1<double>.From([0.0, 2.0]), NCollection_Array1<int>.From([4, 1, 4]),
                                       NCollection_Array1<int>.From([4, 4]), 3, 3);
    }

    [Fact]
    public void SplitBSplineSurface_AllOverloads()
    {
        BSplineSurface aS = BiCubic();
        BSplineSurface aKnots = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SplitBSplineSurface(aS, 3, 2, 1, 2);
        aKnots.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        Assert.Equal((1.0, 3.0, 0.0, 2.0), (aU1, aU2, aV1, aV2));
        // reversed in U: the parameter u of the result is 4 - u on S
        Assert.True(aKnots.Value(1.5, 0.5).Distance(aS.Value(2.5, 0.5)) < Tol);

        BSplineSurface aU = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SplitBSplineSurface(aS, 1, 2, true);
        aU.Bounds(out aU1, out aU2, out _, out _);
        Assert.Equal((0.0, 1.0), (aU1, aU2));

        BSplineSurface aParams = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SplitBSplineSurface(aS, 0.5, 2.5, 1.5, 0.5, 1e-9);
        Assert.True(aParams.Value(1.0, 1.0).Distance(aS.Value(1.0, 1.0)) < Tol);
        aParams.Bounds(out _, out _, out aV1, out aV2);
        // reversed in V
        Assert.True(aParams.Value(1.0, aV1).Distance(aS.Value(1.0, 1.5)) < Tol);

        Assert.ThrowsAny<DomainErrorException>(() => global::Kerf.Engine.Geometry.Conversion.GeomConvert.SplitBSplineSurface(aS, 1, 1, 1, 2));
        Assert.ThrowsAny<DomainErrorException>(() => global::Kerf.Engine.Geometry.Conversion.GeomConvert.SplitBSplineSurface(aS, 1, 4, false));
        Assert.ThrowsAny<DomainErrorException>(() => global::Kerf.Engine.Geometry.Conversion.GeomConvert.SplitBSplineSurface(aS, 1.0, 1.0, true, 1e-9));
    }

    [Fact]
    public void SplitBSplineSurface_InV_ReversesV()
    {
        // A split by parameters in V with FromParam1 > ToParam2 reverses V, as the U split reverses U (OCCT reversed
        // U here: occt-issues.md, GeomConvert::SplitBSplineSurface; Kerf decision 1490)
        BSplineSurface aS = BiCubic();
        BSplineSurface aSplit = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SplitBSplineSurface(aS, 1.5, 0.5, false, 1e-9);
        aSplit.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        Assert.Equal((0.0, 3.0), (aU1, aU2));
        Assert.True(aSplit.Value(aU1, aV1).Distance(aS.Value(0.0, 1.5)) < Tol);
        Assert.True(aSplit.Value(aU2, aV2).Distance(aS.Value(3.0, 0.5)) < Tol);
        Assert.True(aSplit.Value(1.2, aV1 + 0.25).Distance(aS.Value(1.2, 1.25)) < Tol);
    }

    [Fact]
    public void BSplineCurveKnotSplitting()
    {
        // Knot 2 with multiplicity 3 (C0), knot 3 with multiplicity 2 (C1)
        BSplineCurve3D aC = Cubic().KnotInserted(1.0, 3, 0.0, false).KnotInserted(2.0, 2, 0.0, false);
        global::Kerf.Engine.Geometry.Conversion.BSplineCurveKnotSplitting aSplit0 = new(aC, 0);
        Assert.Equal(2, aSplit0.SplitCount);
        global::Kerf.Engine.Geometry.Conversion.BSplineCurveKnotSplitting aSplit1 = new(aC, 1);
        Assert.Equal(3, aSplit1.SplitCount);
        Assert.Equal(1, aSplit1.SplitValue(1));
        Assert.Equal(2, aSplit1.SplitValue(2));
        Assert.Equal(4, aSplit1.SplitValue(3));
        global::Kerf.Engine.Geometry.Conversion.BSplineCurveKnotSplitting aSplit2 = new(aC, 2);
        NCollection_Array1<int> aValues = new(1, aSplit2.SplitCount);
        aSplit2.Splitting(aValues);
        Assert.Equal([1, 2, 3, 4], aValues.AsSpan().ToArray());
        Assert.ThrowsAny<RangeErrorException>(() => new global::Kerf.Engine.Geometry.Conversion.BSplineCurveKnotSplitting(aC, -1));
        Assert.ThrowsAny<RangeErrorException>(() => aSplit2.SplitValue(5));
    }

    [Fact]
    public void BSplineCurveToBezierCurve()
    {
        BSplineCurve3D aC = Cubic();
        aC.Freeze();
        global::Kerf.Engine.Geometry.Conversion.BSplineCurveToBezierCurve aConv = new(aC);
        Assert.Equal(3, aConv.ArcCount);
        NCollection_Array1<BezierCurve3D> anArcs = new(1, 3);
        aConv.Arcs(anArcs);
        NCollection_Array1<double> aKnots = new(1, 4);
        aConv.Knots(aKnots);
        Assert.Equal([0.0, 1.0, 2.0, 3.0], aKnots.AsSpan().ToArray());
        for (int i = 1; i <= 3; i++)
        {
            Assert.Equal(3, anArcs[i].Degree());
            AssertSamePoints(aC, i - 1.0, i, anArcs[i], 0.0, 1.0, Tol);
            Assert.NotSame(anArcs[i], aConv.Arc(i));
            Assert.False(anArcs[i].IsFrozen);
        }

        Assert.ThrowsAny<OutOfRangeException>(() => aConv.Arc(4));

        // A part, with bounds snapped onto knots within half the tolerance
        global::Kerf.Engine.Geometry.Conversion.BSplineCurveToBezierCurve aPart = new(aC, 0.5, 2.0 + 1e-4, 1e-3);
        Assert.Equal(2, aPart.ArcCount);
        AssertSamePoints(aC, 0.5, 1.0, aPart.Arc(1), 0.0, 1.0, Tol);
        Assert.ThrowsAny<DomainErrorException>(() => new global::Kerf.Engine.Geometry.Conversion.BSplineCurveToBezierCurve(aC, 1.0, 1.0, 1e-3));

        // Rational
        BSplineCurve3D aRational = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(
            new TrimmedCurve3D(new CircleCurve3D(new Frame3D(P(0, 0), new Direction3D(0, 0, 1)), 2.0), 0.0, 3.0));
        global::Kerf.Engine.Geometry.Conversion.BSplineCurveToBezierCurve aRConv = new(aRational);
        for (int i = 1; i <= aRConv.ArcCount; i++)
        {
            BezierCurve3D anArc = aRConv.Arc(i);
            Assert.True(anArc.IsRational);
            Assert.Equal(2.0, anArc.Value(0.37).Distance(P(0, 0)), 1e-12);
        }
    }

    [Fact]
    public void BSplineCurveToBezierCurve_Periodic()
    {
        // GeomConvert keeps a periodic curve periodic (Geom2dConvert makes it non-periodic first, OCC7372)
        NCollection_Array1<Point3D> aPoles = NCollection_Array1<Point3D>.From([P(0, 0), P(4, 0, 1), P(4, 4), P(0, 4, -1), P(-1, 2)]);
        BSplineCurve3D aPeriodic = new(aPoles, NCollection_Array1<double>.From([0.0, 1.0, 2.0, 3.0, 4.0, 5.0]),
                                          NCollection_Array1<int>.From([1, 1, 1, 1, 1, 1]), 3, true);
        global::Kerf.Engine.Geometry.Conversion.BSplineCurveToBezierCurve aConv = new(aPeriodic);
        Assert.Equal(5, aConv.ArcCount);
        for (int i = 1; i <= 5; i++)
        {
            AssertSamePoints(aPeriodic, i - 1.0, i, aConv.Arc(i), 0.0, 1.0, Tol);
        }
    }

    [Fact]
    public void CompCurveToBSplineCurve_KeepsHandedOutCurves_AndOptions()
    {
        BSplineCurve3D aC = Cubic();
        BSplineCurve3D aMiddle = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SplitBSplineCurve(aC, 2, 3);
        BSplineCurve3D aFirst = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SplitBSplineCurve(aC, 1, 2);
        BSplineCurve3D aLast = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SplitBSplineCurve(aC, 3, 4);
        aFirst.Freeze();
        global::Kerf.Engine.Geometry.Conversion.CompCurveToBSplineCurve aComp = new(aMiddle);
        BSplineCurve3D? aHandedOut = aComp.BSplineCurve();
        Assert.NotNull(aHandedOut);
        Assert.True(aHandedOut.IsFrozen);
        Assert.Same(aHandedOut, aComp.BSplineCurve());
        Assert.NotSame(aMiddle, aHandedOut);

        Assert.True(aComp.Add(aLast, 1e-7, true));
        Assert.True(aComp.Add(aFirst, 1e-7));
        Assert.Equal(1.0, aHandedOut.FirstParameter());
        Assert.Equal(2.0, aHandedOut.LastParameter());

        BSplineCurve3D aResult = aComp.BSplineCurve()!;
        Assert.True(aResult.StartPoint().Distance(aC.StartPoint()) < Tol);
        Assert.True(aResult.EndPoint().Distance(aC.EndPoint()) < Tol);
        // The C2 junctions are merged back: the original knots
        Assert.Equal(4, aResult.KnotCount);

        // MinM keeps the joint knot's multiplicity
        global::Kerf.Engine.Geometry.Conversion.CompCurveToBSplineCurve aKept = new(aFirst);
        Assert.True(aKept.Add(aMiddle, 1e-7, true, true, 2));
        Assert.Equal(2, aKept.BSplineCurve()!.Multiplicity(2));

        // A lower degree is raised; WithRatio false keeps the added curve's parametrisation length
        TrimmedCurve3D aLine = new(new LineCurve3D(aC.EndPoint(), new Direction3D(1, 0, 0)), 0.0, 10.0);
        global::Kerf.Engine.Geometry.Conversion.CompCurveToBSplineCurve aWithLine = new(aC);
        Assert.True(aWithLine.Add(aLine, 1e-7, true, false));
        BSplineCurve3D aJoined = aWithLine.BSplineCurve()!;
        Assert.Equal(3, aJoined.Degree());
        Assert.Equal(13.0, aJoined.LastParameter(), 1e-12);
        Assert.True(aJoined.EndPoint().Distance(aLine.EndPoint()) < Tol);
    }

    [Fact]
    public void ConcatG1_OpenAndClosed()
    {
        BSplineCurve3D aC = Cubic();
        NCollection_Array1<BSplineCurve3D> aCurves = new(0, 2);
        aCurves[0] = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SplitBSplineCurve(aC, 1, 2);
        aCurves[1] = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SplitBSplineCurve(aC, 2, 3);
        aCurves[2] = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SplitBSplineCurve(aC, 3, 4);
        foreach (BSplineCurve3D aCurve in new[] { aCurves[0], aCurves[1], aCurves[2] })
        {
            aCurve.Freeze();
        }

        NCollection_Array1<double> aTols = NCollection_Array1<double>.From([1e-7, 1e-7], 0);
        bool isClosed = false;
        global::Kerf.Engine.Geometry.Conversion.GeomConvert.ConcatG1(aCurves, aTols, out NCollection_HArray1<BSplineCurve3D> aResult, ref isClosed, 1e-7);
        Assert.Equal(1, aResult.Length());
        AssertSamePoints(aC, 0.0, 3.0, aResult[0], 0.0, 3.0, 1e-9);

        // A closed G1 contour: four quarter circles, as rational B-splines
        CircleCurve3D aCircle = new(new Frame3D(P(0, 0, 1), new Direction3D(1, 1, 1)), 2.0);
        NCollection_Array1<BSplineCurve3D> aQuarters = new(0, 3);
        for (int i = 0; i < 4; i++)
        {
            aQuarters[i] = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(new TrimmedCurve3D(aCircle, i * M_PI / 2, (i + 1) * M_PI / 2));
            aQuarters[i].Freeze();
        }

        isClosed = true;
        global::Kerf.Engine.Geometry.Conversion.GeomConvert.ConcatG1(aQuarters, NCollection_Array1<double>.From([1e-7, 1e-7, 1e-7], 0), out aResult, ref isClosed, 1e-7);
        Assert.Equal(1, aResult.Length());
        Assert.True(aResult[0].IsPeriodic);
        for (int i = 0; i <= 40; i++)
        {
            double aU = aResult[0].FirstParameter() + (aResult[0].LastParameter() - aResult[0].FirstParameter()) * i / 40.0;
            Assert.Equal(2.0, aResult[0].Value(aU).Distance(P(0, 0, 1)), 1e-7);
        }
    }

    [Fact]
    public void ConcatG1_SingleClosedCurve_DoesNotChangeTheCaller()
    {
        CircleCurve3D aCircle = new(new Frame3D(P(0, 0), new Direction3D(0, 0, 1)), 2.0);
        BSplineCurve3D aWhole = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(new TrimmedCurve3D(aCircle, 0.0, 2 * M_PI));
        aWhole.Freeze();
        NCollection_Array1<BSplineCurve3D> aCurves = new(0, 0);
        aCurves[0] = aWhole;
        bool isClosed = true;
        global::Kerf.Engine.Geometry.Conversion.GeomConvert.ConcatG1(aCurves, new NCollection_Array1<double>(0, -1), out NCollection_HArray1<BSplineCurve3D> aResult,
                             ref isClosed, 1e-7);
        Assert.True(aResult[0].IsPeriodic);
        Assert.False(aWhole.IsPeriodic);
        Assert.NotSame(aWhole, aResult[0]);
    }

    [Fact]
    public void ConcatG1_CurvesThatDontJoin_Throw()
    {
        BSplineCurve3D aC = Cubic();
        NCollection_Array1<BSplineCurve3D> aCurves = new(0, 1);
        aCurves[0] = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SplitBSplineCurve(aC, 1, 2);
        aCurves[1] = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SplitBSplineCurve(aC, 3, 4);
        bool isClosed = false;
        // GeomLProp::Continuity: "Courbes non jointives"
        Assert.ThrowsAny<FailureException>(() => global::Kerf.Engine.Geometry.Conversion.GeomConvert.ConcatG1(aCurves, NCollection_Array1<double>.From([1e-7], 0),
                                                                       out _, ref isClosed, 1e-7));
    }

    [Fact]
    public void ConcatG1_ClosedWire_ReadsTheCallersTolerances_OCCT()
    {
        // As in Geom2dConvert (occt-issues.md): after reordering a closed wire, ConcatG1 merges each group with the
        // caller's tolerances, not the reordered ones, so s3 and s0 are merged with the tolerance of the junction s1-s2
        // instead of ClosedTolerance. (ConcatC1 reads the reordered ones in 3D.)
        BSplineCurve3D[] aSegments =
        [
            Segment(P(0, 0), P(1, 0)),
            Segment(P(1, 0), P(2, 0)),
            Segment(P(2, 0), P(2, 1)),
            Segment(P(2, 1), P(1e-5, 0)),
        ];

        NCollection_Array1<BSplineCurve3D> aCurves = NCollection_Array1<BSplineCurve3D>.From(aSegments, 0);
        bool isClosed = true;
        Assert.ThrowsAny<ConstructionErrorException>(() => global::Kerf.Engine.Geometry.Conversion.GeomConvert.ConcatG1(
            aCurves, NCollection_Array1<double>.From([1e-7, 1e-12, 1e-7], 0), out _, ref isClosed, 1e-4));

        aCurves = NCollection_Array1<BSplineCurve3D>.From(aSegments, 0);
        isClosed = true;
        global::Kerf.Engine.Geometry.Conversion.GeomConvert.ConcatC1(aCurves, NCollection_Array1<double>.From([1e-7, 1e-12, 1e-7], 0), out _,
                             out NCollection_HArray1<BSplineCurve3D> aResult, ref isClosed, 1e-4);
        Assert.Equal(2, aResult.Length());
        Assert.True(aResult[1].StartPoint().Distance(P(2, 1)) < Tol);
        Assert.True(aResult[1].EndPoint().Distance(P(2, 0)) < Tol);
    }

    [Fact]
    public void ConcatC1_GroupsAndIndices()
    {
        // A polyline with a corner after the second segment: two groups
        Point3D[] aPts = [P(0, 0), P(1, 0), P(2, 0), P(2, 1)];
        NCollection_Array1<BSplineCurve3D> aCurves = new(0, 2);
        for (int i = 0; i < 3; i++)
        {
            aCurves[i] = Segment(aPts[i], aPts[i + 1]);
        }

        bool isClosed = false;
        global::Kerf.Engine.Geometry.Conversion.GeomConvert.ConcatC1(aCurves, NCollection_Array1<double>.From([1e-7, 1e-7], 0), out NCollection_HArray1<int> anIndices,
                             out NCollection_HArray1<BSplineCurve3D> aResult, ref isClosed, 1e-7);
        Assert.Equal(2, aResult.Length());
        Assert.Equal(3, anIndices.Length());
        Assert.Equal(0, anIndices[0]);
        Assert.Equal(2, anIndices[1]);
        Assert.Equal(0, anIndices[2]);
        Assert.True(aResult[0].StartPoint().Distance(aPts[0]) < Tol);
        Assert.True(aResult[0].EndPoint().Distance(aPts[2]) < Tol);
        Assert.Same(aCurves[2], aResult[1]);
    }

    [Fact]
    public void ConcatC1_RationalPieces_AreNormalisedCopies()
    {
        // Two rational quartics joining with G1 continuity, whose two first and two last weights are 2: Pretreatment
        // divides their weights by 2, in copies that replace them in the array; then Hermit makes them C1
        Point3D[][] aPoles =
        [
            [P(0, 0), P(1, 0), P(2, 1, 1), P(3, 1), P(4, 1)],
            [P(4, 1), P(5, 1), P(6, 2, -1), P(7, 0), P(8, 0)],
        ];
        NCollection_Array1<BSplineCurve3D> aCurves = new(0, 1);
        BSplineCurve3D[] anOriginals = new BSplineCurve3D[2];
        for (int i = 0; i < 2; i++)
        {
            anOriginals[i] = new BSplineCurve3D(NCollection_Array1<Point3D>.From(aPoles[i]),
                                                   NCollection_Array1<double>.From([2.0, 2.0, 1.0, 2.0, 2.0]),
                                                   NCollection_Array1<double>.From([0.0, 1.0]),
                                                   NCollection_Array1<int>.From([5, 5]), 4);
            anOriginals[i].Freeze();
            aCurves[i] = anOriginals[i];
        }

        bool isClosed = false;
        global::Kerf.Engine.Geometry.Conversion.GeomConvert.ConcatC1(aCurves, NCollection_Array1<double>.From([1e-7], 0), out _,
                             out NCollection_HArray1<BSplineCurve3D> aResult, ref isClosed, 1e-7);
        Assert.Equal(1, aResult.Length());
        Assert.NotSame(anOriginals[0], aCurves[0]);
        Assert.Equal(2.0, anOriginals[0].Weight(1));
        Assert.Equal(1.0, aCurves[0].Weight(1));
        Assert.Equal(0.5, aCurves[0].Weight(3));
        Assert.True(aResult[0].StartPoint().Distance(P(0, 0)) < Tol);
        Assert.True(aResult[0].EndPoint().Distance(P(8, 0)) < Tol);
    }

    [Fact]
    public void C0BSplineToC1BSplineCurve_RemovesG1Knots()
    {
        BSplineCurve3D aC = Cubic();
        // Structurally C0 at knot 1, geometrically smooth
        BSplineCurve3D aC0 = aC.KnotInserted(1.0, 3, 0.0, false);
        Assert.Equal(3, aC0.Multiplicity(2));
        aC0.Freeze();
        BSplineCurve3D aResult = aC0;
        global::Kerf.Engine.Geometry.Conversion.GeomConvert.C0BSplineToC1BSplineCurve(ref aResult, 1e-7);
        Assert.NotSame(aC0, aResult);
        Assert.False(aResult.IsFrozen);
        Assert.Equal(3, aC0.Multiplicity(2));
        Assert.True(aResult.Continuity() >= C1);
        Assert.True(aResult.StartPoint().Distance(aC.StartPoint()) < Tol);
        Assert.True(aResult.EndPoint().Distance(aC.EndPoint()) < Tol);

        // Nothing to do: still a new curve (GeomConvert, unlike Geom2dConvert, always goes through the concatenation)
        BSplineCurve3D aSmooth = aC;
        global::Kerf.Engine.Geometry.Conversion.GeomConvert.C0BSplineToC1BSplineCurve(ref aSmooth, 1e-7);
        Assert.NotSame(aC, aSmooth);
        AssertSamePoints(aC, 0.0, 3.0, aSmooth, 0.0, 3.0, Tol);
    }

    [Fact]
    public void C0BSplineToArrayOfC1BSplineCurve_SplitsAtCorners()
    {
        BSplineCurve3D aPolyline = new(NCollection_Array1<Point3D>.From([P(0, 0), P(1, 0), P(1, 1)]),
                                          NCollection_Array1<double>.From([0.0, 1.0, 2.0]), NCollection_Array1<int>.From([2, 1, 2]), 1);
        global::Kerf.Engine.Geometry.Conversion.GeomConvert.C0BSplineToArrayOfC1BSplineCurve(aPolyline, out NCollection_HArray1<BSplineCurve3D> aTab, 1e-7);
        Assert.Equal(2, aTab.Length());
        Assert.True(aTab[0].EndPoint().Distance(P(1, 0)) < Tol);
        Assert.True(aTab[1].StartPoint().Distance(P(1, 0)) < Tol);

        BSplineCurve3D aC = Cubic();
        global::Kerf.Engine.Geometry.Conversion.GeomConvert.C0BSplineToArrayOfC1BSplineCurve(aC, out aTab, 1e-7);
        Assert.Equal(1, aTab.Length());
        Assert.Same(aC, aTab[0]);
    }

    [Fact]
    public void ApproxCurve_OfAnOffsetCurve_AndThroughAnAdaptor()
    {
        OffsetCurve3D anOffset = new(Cubic(), -0.3, new Direction3D(0, 0, 1));
        global::Kerf.Engine.Geometry.Conversion.ApproxCurve anApprox = new(anOffset, 1e-5, C2, 20, 9);
        Assert.True(anApprox.HasResult);
        Assert.True(anApprox.IsDone);
        Assert.True(anApprox.MaxError() < 1e-5);
        BSplineCurve3D aCurve = anApprox.Curve()!;
        Assert.True(aCurve.IsFrozen);
        Assert.Same(aCurve, anApprox.Curve());
        for (int i = 0; i <= 30; i++)
        {
            double aU = 0.1 * i;
            Assert.True(aCurve.Value(aU).Distance(anOffset.Value(aU)) < 1e-5);
        }

        global::Kerf.Engine.Geometry.Conversion.ApproxCurve aPart = new(new CurveAdaptor(anOffset, 0.5, 2.0), 1e-5, C1, 20, 9);
        Assert.True(aPart.HasResult);
        Assert.Equal(0.5, aPart.Curve()!.FirstParameter());
        Assert.Equal(2.0, aPart.Curve()!.LastParameter());
        Assert.StartsWith("******* Dump of ApproxCurve *******\n*******Error   ", aPart.ToString());
    }

    [Fact]
    public void ApproxSurface_OfAnOffsetSurface()
    {
        BSplineSurface aBasis = BiCubic();
        OffsetSurface anOffset = new(aBasis, 0.2);
        global::Kerf.Engine.Geometry.Conversion.ApproxSurface anApprox = new(anOffset, 1e-4, C1, C1, 9, 9, 50, 1);
        Assert.True(anApprox.HasResult);
        BSplineSurface aSurface = anApprox.Surface()!;
        Assert.True(aSurface.IsFrozen);
        Assert.True(anApprox.MaxError() >= 0.0);
        for (int i = 0; i <= 6; i++)
        {
            for (int j = 0; j <= 4; j++)
            {
                double aU = 0.5 * i, aV = 0.5 * j;
                Assert.True(aSurface.Value(aU, aV).Distance(anOffset.Value(aU, aV)) < 1e-3);
            }
        }

        Assert.Contains("Result max error :", anApprox.ToString());

        // Through SurfaceToBSplineSurface: an unfrozen copy for the caller
        BSplineSurface aConverted = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SurfaceToBSplineSurface(anOffset);
        Assert.False(aConverted.IsFrozen);
        Assert.True(aConverted.Value(1.0, 1.0).Distance(anOffset.Value(1.0, 1.0)) < 1e-3);
    }

    [Fact]
    public void SurfaceToBSplineSurface_SweptAndElementary()
    {
        // A trimmed surface of revolution of a circle arc (a torus patch) and an extrusion of a B-spline curve
        CircleCurve3D aMeridian = new(new Frame3D(P(5, 0, 0), new Direction3D(0, 1, 0)), 1.0);
        RevolutionSurface aRevol = new(aMeridian, new Axis3D(P(0, 0, 0), new Direction3D(0, 0, 1)));
        TrimmedSurface aRevolPatch = new(aRevol, 0.0, 4.0, 0.5, 2.0);
        BSplineSurface aR = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SurfaceToBSplineSurface(aRevolPatch);
        ExtrusionSurface anExtru = new(Cubic(), new Direction3D(0, 1, 1));
        BSplineSurface anE = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SurfaceToBSplineSurface(new TrimmedSurface(anExtru, 0.5, 2.5, -1.0, 2.0));
        for (int i = 0; i <= 8; i++)
        {
            double aT = i / 8.0;
            Point3D aOnR = aR.Value(4.0 * aT, 0.5 + 1.5 * aT);
            Assert.Equal(1.0, Math.Sqrt(Math.Pow(Math.Sqrt(aOnR.X() * aOnR.X() + aOnR.Y() * aOnR.Y()) - 5.0, 2) + aOnR.Z() * aOnR.Z()), 1e-12);
            Assert.True(anE.Value(0.5 + 2 * aT, -1 + 3 * aT).Distance(anExtru.Value(0.5 + 2 * aT, -1 + 3 * aT)) < Tol);
        }

        Assert.ThrowsAny<DomainErrorException>(
            () => global::Kerf.Engine.Geometry.Conversion.GeomConvert.SurfaceToBSplineSurface(new CylindricalSurface(new CoordinateSystem(Constants.XOY()), 1.0)));
        BSplineSurface aSphere = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SurfaceToBSplineSurface(new SphericalSurface(new CoordinateSystem(Constants.XOY()), 3.0));
        Assert.True(aSphere.IsUPeriodic);
        Assert.Equal(3.0, aSphere.Value(1.0, 0.3).Distance(P(0, 0)), 1e-12);
    }
}
