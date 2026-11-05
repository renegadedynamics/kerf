// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Geometry;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Geometry.Continuity;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>OCCT.NET's own tests of Geom2dConvert (OCCT has GTests only for two of its classes).</summary>
public class Geom2dConvertTests
{
    private const double Tol = 1e-9;

    private static BSplineCurve2D Cubic()
    {
        // A smooth cubic with three spans
        NCollection_Array1<Point2D> aPoles = NCollection_Array1<Point2D>.From([
            new Point2D(0, 0), new Point2D(1, 2), new Point2D(3, 3), new Point2D(5, 1), new Point2D(7, 2), new Point2D(8, 0)]);
        return new BSplineCurve2D(aPoles, NCollection_Array1<double>.From([0.0, 1.0, 2.0, 3.0]),
                                       NCollection_Array1<int>.From([4, 1, 1, 4]), 3);
    }

    private static void AssertSamePoints(Curve2D theExpected, double theU1, double theU2, Curve2D theActual,
                                         double theV1, double theV2, double theTol)
    {
        for (int i = 0; i <= 20; i++)
        {
            Point2D aP = theExpected.Value(theU1 + (theU2 - theU1) * i / 20.0);
            Point2D aQ = theActual.Value(theV1 + (theV2 - theV1) * i / 20.0);
            Assert.True(aP.DistanceTo(aQ) < theTol, $"point {i}: {aP} vs {aQ}");
        }
    }

    private static double MinDistance(Curve2D theCurve, Point2D theP)
    {
        double aMin = double.MaxValue;
        double aF = theCurve.FirstParameter, aL = theCurve.LastParameter;
        for (int i = 0; i <= 2000; i++)
        {
            aMin = Math.Min(aMin, theCurve.Value(aF + (aL - aF) * i / 2000.0).DistanceTo(theP));
        }

        return aMin;
    }

    [Fact]
    public void CurveToBSplineCurve_WholeCircle_IsPeriodic()
    {
        CircleCurve2D aCircle = new(new Axis2D(new Point2D(1, 2), new Direction2D(1, 1)), 3.0);
        foreach (ParameterisationType aParam in new[] { TgtThetaOver2, QuasiAngular, RationalC1, Polynomial })
        {
            BSplineCurve2D aBS = Geom2dConvert2D.CurveToBSplineCurve(aCircle, aParam);
            Assert.True(aBS.IsPeriodic);
            Assert.False(aBS.IsFrozen);
            for (int i = 0; i < 16; i++)
            {
                double aU = aBS.FirstParameter + (aBS.LastParameter - aBS.FirstParameter) * i / 16.0;
                Assert.Equal(3.0, aBS.Value(aU).DistanceTo(new Point2D(1, 2)), aParam == Polynomial ? 1e-4 : Tol);
            }
        }
    }

    [Fact]
    public void CurveToBSplineCurve_LeftHandedEllipse()
    {
        EllipseCurve2D anEllipse = new(new Frame2D(new Point2D(0, 0), new Direction2D(0, 1), false), 4.0, 2.0);
        TrimmedCurve2D aTrimmed = new(anEllipse, 0.3, 2.5);
        BSplineCurve2D aBS = Geom2dConvert2D.CurveToBSplineCurve(aTrimmed);
        Assert.True(aBS.StartPoint.DistanceTo(aTrimmed.StartPoint) < Tol);
        Assert.True(aBS.EndPoint.DistanceTo(aTrimmed.EndPoint) < Tol);
        Assert.True(MinDistance(aTrimmed, aBS.Value(0.5 * (aBS.FirstParameter + aBS.LastParameter))) < 1e-3);
    }

    [Fact]
    public void CurveToBSplineCurve_LongRationalC1Arc_JoinsTwoHalves()
    {
        CircleCurve2D aCircle = new(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 2.0);
        TrimmedCurve2D anArc = new(aCircle, 0.1, 6.2);
        BSplineCurve2D aBS = Geom2dConvert2D.CurveToBSplineCurve(anArc, RationalC1);
        Assert.False(aBS.IsFrozen);
        Assert.True(aBS.StartPoint.DistanceTo(anArc.StartPoint) < Tol);
        Assert.True(aBS.EndPoint.DistanceTo(anArc.EndPoint) < Tol);
        for (int i = 0; i <= 50; i++)
        {
            double aU = aBS.FirstParameter + (aBS.LastParameter - aBS.FirstParameter) * i / 50.0;
            Assert.Equal(2.0, aBS.Value(aU).DistanceTo(new Point2D(0, 0)), Tol);
        }
    }

    [Fact]
    public void CurveToBSplineCurve_TrimmedLineHyperbolaParabolaBezier()
    {
        LineCurve2D aLine = new(new Point2D(1, 1), new Direction2D(1, 2));
        BSplineCurve2D aBS = Geom2dConvert2D.CurveToBSplineCurve(new TrimmedCurve2D(aLine, -2.0, 3.0));
        Assert.Equal(1, aBS.Degree);
        Assert.Equal(2, aBS.PoleCount);
        AssertSamePoints(aLine, -2.0, 3.0, aBS, -2.0, 3.0, Tol);

        HyperbolaCurve2D aHyp = new(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 3.0, 2.0);
        TrimmedCurve2D aHypTrimmed = new(aHyp, -1.0, 1.5);
        aBS = Geom2dConvert2D.CurveToBSplineCurve(aHypTrimmed);
        Assert.True(aBS.StartPoint.DistanceTo(aHypTrimmed.StartPoint) < Tol);
        Assert.True(aBS.EndPoint.DistanceTo(aHypTrimmed.EndPoint) < Tol);

        ParabolaCurve2D aParab = new(new Axis2D(new Point2D(0, 0), new Direction2D(0, 1)), 1.5);
        TrimmedCurve2D aParabTrimmed = new(aParab, -2.0, 4.0);
        aBS = Geom2dConvert2D.CurveToBSplineCurve(aParabTrimmed);
        Assert.Equal(2, aBS.Degree);
        AssertSamePoints(aParabTrimmed, -2.0, 4.0, aBS, aBS.FirstParameter, aBS.LastParameter, Tol);

        BezierCurve2D aBez = new(NCollection_Array1<Point2D>.From([new Point2D(0, 0), new Point2D(1, 3), new Point2D(4, 2)]),
                                      NCollection_Array1<double>.From([1.0, 2.0, 1.0]));
        aBS = Geom2dConvert2D.CurveToBSplineCurve(aBez);
        Assert.True(aBS.IsRational);
        AssertSamePoints(aBez, 0.0, 1.0, aBS, 0.0, 1.0, Tol);
        aBS = Geom2dConvert2D.CurveToBSplineCurve(new TrimmedCurve2D(aBez, 0.2, 0.7));
        AssertSamePoints(aBez, 0.2, 0.7, aBS, 0.0, 1.0, Tol);
    }

    [Fact]
    public void CurveToBSplineCurve_FrozenTrimmedBSpline_IsCopied()
    {
        BSplineCurve2D aCubic = Cubic();
        aCubic.Freeze();
        TrimmedCurve2D aTrimmed = new(aCubic, 0.5, 2.5);
        BSplineCurve2D aBS = Geom2dConvert2D.CurveToBSplineCurve(aTrimmed);
        Assert.False(aBS.IsFrozen);
        Assert.Equal(0.5, aBS.FirstParameter);
        Assert.Equal(2.5, aBS.LastParameter);
        AssertSamePoints(aCubic, 0.5, 2.5, aBS, 0.5, 2.5, Tol);
        Assert.Equal(0.0, aCubic.FirstParameter);
    }

    [Fact]
    public void CurveToBSplineCurve_OffsetCurve_Approximated()
    {
        CircleCurve2D aCircle = new(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 2.0);
        OffsetCurve2D anOffset = new(new TrimmedCurve2D(aCircle, 0.0, 2.0), 0.5);
        BSplineCurve2D aBS = Geom2dConvert2D.CurveToBSplineCurve(anOffset);
        Assert.False(aBS.IsFrozen);
        for (int i = 0; i <= 20; i++)
        {
            double aU = aBS.FirstParameter + (aBS.LastParameter - aBS.FirstParameter) * i / 20.0;
            Assert.Equal(2.5, aBS.Value(aU).DistanceTo(new Point2D(0, 0)), 1e-4);
        }
    }

    [Fact]
    public void CurveToBSplineCurve_UnboundedCurves_Throw()
    {
        EngineAssert.ThrowsAny<DomainErrorException>(() => Geom2dConvert2D.CurveToBSplineCurve(new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0))));
        EngineAssert.ThrowsAny<DomainErrorException>(() => Geom2dConvert2D.CurveToBSplineCurve(
            new ParabolaCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 1.0)));
    }

    [Fact]
    public void SplitBSplineCurve_ByKnotsAndParameters()
    {
        BSplineCurve2D aC = Cubic();
        BSplineCurve2D aPart = Geom2dConvert2D.SplitBSplineCurve(aC, 2, 3);
        Assert.Equal(1.0, aPart.FirstParameter);
        Assert.Equal(2.0, aPart.LastParameter);
        AssertSamePoints(aC, 1.0, 2.0, aPart, 1.0, 2.0, Tol);

        BSplineCurve2D aReversed = Geom2dConvert2D.SplitBSplineCurve(aC, 3, 2);
        Assert.True(aReversed.StartPoint.DistanceTo(aC.Value(2.0)) < Tol);
        Assert.True(aReversed.EndPoint.DistanceTo(aC.Value(1.0)) < Tol);

        BSplineCurve2D aByParams = Geom2dConvert2D.SplitBSplineCurve(aC, 2.5, 0.5, 1e-9);
        Assert.True(aByParams.StartPoint.DistanceTo(aC.Value(2.5)) < Tol);
        Assert.True(aByParams.EndPoint.DistanceTo(aC.Value(0.5)) < Tol);

        EngineAssert.ThrowsAny<DomainErrorException>(() => Geom2dConvert2D.SplitBSplineCurve(aC, 2, 2));
        EngineAssert.ThrowsAny<OutOfRangeException>(() => Geom2dConvert2D.SplitBSplineCurve(aC, 0, 2));
    }

    [Fact]
    public void BSplineCurveKnotSplitting()
    {
        // Knot 2 with multiplicity 3 (C0), knot 3 with multiplicity 2 (C1)
        BSplineCurve2D aC = Cubic().KnotInserted(1.0, 3).KnotInserted(2.0, 2);
        BSplineCurveKnotSplitting2D aSplit0 = new(aC, 0);
        Assert.Equal(2, aSplit0.SplitCount);
        BSplineCurveKnotSplitting2D aSplit1 = new(aC, 1);
        Assert.Equal(3, aSplit1.SplitCount);
        Assert.Equal(1, aSplit1.SplitValue(1));
        Assert.Equal(2, aSplit1.SplitValue(2));
        Assert.Equal(4, aSplit1.SplitValue(3));
        BSplineCurveKnotSplitting2D aSplit2 = new(aC, 2);
        NCollection_Array1<int> aValues = new(1, aSplit2.SplitCount);
        aSplit2.Splitting(aValues);
        Assert.Equal([1, 2, 3, 4], aValues.AsSpan().ToArray());
        EngineAssert.ThrowsAny<RangeErrorException>(() => new BSplineCurveKnotSplitting2D(aC, -1));
        EngineAssert.ThrowsAny<RangeErrorException>(() => aSplit2.SplitValue(5));
    }

    [Fact]
    public void BSplineCurveToBezierCurve()
    {
        BSplineCurve2D aC = Cubic();
        BSplineCurveToBezierCurve2D aConv = new(aC);
        Assert.Equal(3, aConv.ArcCount);
        NCollection_Array1<BezierCurve2D> anArcs = new(1, 3);
        aConv.Arcs(anArcs);
        NCollection_Array1<double> aKnots = new(1, 4);
        aConv.Knots(aKnots);
        Assert.Equal([0.0, 1.0, 2.0, 3.0], aKnots.AsSpan().ToArray());
        for (int i = 1; i <= 3; i++)
        {
            Assert.Equal(3, anArcs[i].Degree);
            AssertSamePoints(aC, i - 1.0, i, anArcs[i], 0.0, 1.0, Tol);
            Assert.NotSame(anArcs[i], aConv.Arc(i));
        }

        EngineAssert.ThrowsAny<OutOfRangeException>(() => aConv.Arc(4));

        // A part, with bounds snapped onto knots within half the tolerance
        BSplineCurveToBezierCurve2D aPart = new(aC, 0.5, 2.0 + 1e-4, 1e-3);
        Assert.Equal(2, aPart.ArcCount);
        AssertSamePoints(aC, 0.5, 1.0, aPart.Arc(1), 0.0, 1.0, Tol);
        EngineAssert.ThrowsAny<DomainErrorException>(() => new BSplineCurveToBezierCurve2D(aC, 1.0, 1.0, 1e-3));
    }

    [Fact]
    public void CompCurveToBSplineCurve_AddsBeforeAndAfter_AndKeepsHandedOutCurves()
    {
        BSplineCurve2D aC = Cubic();
        BSplineCurve2D aMiddle = Geom2dConvert2D.SplitBSplineCurve(aC, 2, 3);
        BSplineCurve2D aFirst = Geom2dConvert2D.SplitBSplineCurve(aC, 1, 2);
        BSplineCurve2D aLast = Geom2dConvert2D.SplitBSplineCurve(aC, 3, 4);
        CompCurveToBSplineCurve2D aComp = new(aMiddle);
        BSplineCurve2D? aHandedOut = aComp.BSplineCurve();
        Assert.NotNull(aHandedOut);
        Assert.True(aHandedOut.IsFrozen);
        Assert.Same(aHandedOut, aComp.BSplineCurve());
        Assert.NotSame(aMiddle, aHandedOut);

        Assert.True(aComp.Add(aLast, 1e-7, true));
        Assert.True(aComp.Add(aFirst, 1e-7));
        Assert.Equal(1.0, aHandedOut.FirstParameter);
        Assert.Equal(2.0, aHandedOut.LastParameter);

        BSplineCurve2D aResult = aComp.BSplineCurve()!;
        Assert.True(aResult.StartPoint.DistanceTo(aC.StartPoint) < Tol);
        Assert.True(aResult.EndPoint.DistanceTo(aC.EndPoint) < Tol);
        // The C2 junctions are merged back: the original knots
        Assert.Equal(4, aResult.KnotCount);

        // A curve that touches neither end
        LineCurve2D aLine = new(new Point2D(100, 100), new Direction2D(1, 0));
        Assert.False(aComp.Add(new TrimmedCurve2D(aLine, 0.0, 1.0), 1e-7));

        aComp.Clear();
        Assert.Null(aComp.BSplineCurve());
        Assert.True(aComp.Add(aLast, 1e-7));
        Assert.Equal(aLast.PoleCount, aComp.BSplineCurve()!.PoleCount);
    }

    [Fact]
    public void ConcatG1_OpenAndClosed()
    {
        BSplineCurve2D aC = Cubic();
        NCollection_Array1<BSplineCurve2D> aCurves = new(0, 2);
        aCurves[0] = Geom2dConvert2D.SplitBSplineCurve(aC, 1, 2);
        aCurves[1] = Geom2dConvert2D.SplitBSplineCurve(aC, 2, 3);
        aCurves[2] = Geom2dConvert2D.SplitBSplineCurve(aC, 3, 4);
        foreach (BSplineCurve2D aCurve in new[] { aCurves[0], aCurves[1], aCurves[2] })
        {
            aCurve.Freeze();
        }

        NCollection_Array1<double> aTols = NCollection_Array1<double>.From([1e-7, 1e-7], 0);
        bool isClosed = false;
        Geom2dConvert2D.ConcatG1(aCurves, aTols, out NCollection_HArray1<BSplineCurve2D> aResult, ref isClosed, 1e-7);
        Assert.Equal(1, aResult.Length());
        AssertSamePoints(aC, 0.0, 3.0, aResult[0], 0.0, 3.0, 1e-9);

        // A closed G1 contour: four quarter circles, as rational B-splines
        CircleCurve2D aCircle = new(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 2.0);
        NCollection_Array1<BSplineCurve2D> aQuarters = new(0, 3);
        for (int i = 0; i < 4; i++)
        {
            aQuarters[i] = Geom2dConvert2D.CurveToBSplineCurve(new TrimmedCurve2D(aCircle, i * M_PI / 2, (i + 1) * M_PI / 2));
            aQuarters[i].Freeze();
        }

        isClosed = true;
        Geom2dConvert2D.ConcatG1(aQuarters, NCollection_Array1<double>.From([1e-7, 1e-7, 1e-7], 0), out aResult, ref isClosed, 1e-7);
        Assert.Equal(1, aResult.Length());
        Assert.True(aResult[0].IsPeriodic);
        for (int i = 0; i <= 40; i++)
        {
            double aU = aResult[0].FirstParameter + (aResult[0].LastParameter - aResult[0].FirstParameter) * i / 40.0;
            Assert.Equal(2.0, aResult[0].Value(aU).DistanceTo(new Point2D(0, 0)), 1e-7);
        }
    }

    [Fact]
    public void ConcatG1_SingleClosedCurve_DoesNotChangeTheCaller()
    {
        CircleCurve2D aCircle = new(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 2.0);
        BSplineCurve2D aWhole = Geom2dConvert2D.CurveToBSplineCurve(new TrimmedCurve2D(aCircle, 0.0, 2 * M_PI));
        aWhole.Freeze();
        NCollection_Array1<BSplineCurve2D> aCurves = new(0, 0);
        aCurves[0] = aWhole;
        bool isClosed = true;
        Geom2dConvert2D.ConcatG1(aCurves, new NCollection_Array1<double>(0, -1), out NCollection_HArray1<BSplineCurve2D> aResult, ref isClosed, 1e-7);
        Assert.True(aResult[0].IsPeriodic);
        Assert.False(aWhole.IsPeriodic);
        Assert.NotSame(aWhole, aResult[0]);
    }

    [Fact]
    public void ConcatG1_CurvesThatDontJoin_Throw()
    {
        BSplineCurve2D aC = Cubic();
        NCollection_Array1<BSplineCurve2D> aCurves = new(0, 1);
        aCurves[0] = Geom2dConvert2D.SplitBSplineCurve(aC, 1, 2);
        aCurves[1] = Geom2dConvert2D.SplitBSplineCurve(aC, 3, 4);
        bool isClosed = false;
        EngineAssert.ThrowsAny<FailureException>(() => Geom2dConvert2D.ConcatG1(aCurves, NCollection_Array1<double>.From([1e-7], 0),
                                                                         out _, ref isClosed, 1e-7));
    }

    private static BSplineCurve2D Segment(Point2D theP1, Point2D theP2) =>
        new(NCollection_Array1<Point2D>.From([theP1, theP2]), NCollection_Array1<double>.From([0.0, 1.0]),
            NCollection_Array1<int>.From([2, 2]), 1);

    [Fact]
    public void ConcatG1_ClosedWire_ReadsTheCallersTolerances_OCCT()
    {
        // A closed wire of four segments: s0-s1 collinear (one G1 group), corners at s1-s2 and s2-s3, and a gap of
        // 1e-5 at the closure s3-s0, within ClosedTolerance. ConcatG1 reorders the curves to s2, s3, s0, s1 (the
        // closure made G1) and its local tolerances with them, but merges with the caller's array: s3 and s0 are
        // merged with the tolerance of the junction s1-s2 instead of ClosedTolerance (occt-issues.md).
        BSplineCurve2D[] aSegments =
        [
            Segment(new Point2D(0, 0), new Point2D(1, 0)),
            Segment(new Point2D(1, 0), new Point2D(2, 0)),
            Segment(new Point2D(2, 0), new Point2D(2, 1)),
            Segment(new Point2D(2, 1), new Point2D(1e-5, 0)),
        ];

        NCollection_Array1<BSplineCurve2D> aCurves = NCollection_Array1<BSplineCurve2D>.From(aSegments, 0);
        bool isClosed = true;
        EngineAssert.ThrowsAny<ConstructionErrorException>(() => Geom2dConvert2D.ConcatG1(
            aCurves, NCollection_Array1<double>.From([1e-7, 1e-12, 1e-7], 0), out _, ref isClosed, 1e-4));

        // With the junction s1-s2 given the closure's tolerance, the closure merges
        aCurves = NCollection_Array1<BSplineCurve2D>.From(aSegments, 0);
        isClosed = true;
        Geom2dConvert2D.ConcatG1(aCurves, NCollection_Array1<double>.From([1e-7, 1e-4, 1e-7], 0),
                               out NCollection_HArray1<BSplineCurve2D> aResult, ref isClosed, 1e-4);
        Assert.Equal(2, aResult.Length());
        Assert.Same(aSegments[2], aResult[0]);
        Assert.True(aResult[1].StartPoint.DistanceTo(new Point2D(2, 1)) < Tol);
        Assert.True(aResult[1].EndPoint.DistanceTo(new Point2D(2, 0)) < Tol);
    }

    [Fact]
    public void ConcatC1_GroupsAndIndices()
    {
        // A polyline with a corner after the second segment: two groups
        NCollection_Array1<BSplineCurve2D> aCurves = new(0, 2);
        Point2D[] aPts = [new(0, 0), new(1, 0), new(2, 0), new(2, 1)];
        for (int i = 0; i < 3; i++)
        {
            aCurves[i] = new BSplineCurve2D(NCollection_Array1<Point2D>.From([aPts[i], aPts[i + 1]]),
                                                 NCollection_Array1<double>.From([0.0, 1.0]), NCollection_Array1<int>.From([2, 2]), 1);
        }

        bool isClosed = false;
        Geom2dConvert2D.ConcatC1(aCurves, NCollection_Array1<double>.From([1e-7, 1e-7], 0), out NCollection_HArray1<int> anIndices,
                               out NCollection_HArray1<BSplineCurve2D> aResult, ref isClosed, 1e-7);
        Assert.Equal(2, aResult.Length());
        Assert.Equal(3, anIndices.Length());
        Assert.Equal(0, anIndices[0]);
        Assert.Equal(2, anIndices[1]);
        Assert.Equal(0, anIndices[2]);
        Assert.True(aResult[0].StartPoint.DistanceTo(aPts[0]) < Tol);
        Assert.True(aResult[0].EndPoint.DistanceTo(aPts[2]) < Tol);
        Assert.Same(aCurves[2], aResult[1]);
    }

    [Fact]
    public void ConcatC1_RationalPieces_AreNormalisedCopies()
    {
        // Two rational quartics joining with G1 continuity, whose two first and two last weights are 2:
        // Pretreatment divides their weights by 2, in copies that replace them in the array
        Point2D[][] aPoles =
        [
            [new(0, 0), new(1, 0), new(2, 1), new(3, 1), new(4, 1)],
            [new(4, 1), new(5, 1), new(6, 2), new(7, 0), new(8, 0)],
        ];
        NCollection_Array1<BSplineCurve2D> aCurves = new(0, 1);
        BSplineCurve2D[] anOriginals = new BSplineCurve2D[2];
        for (int i = 0; i < 2; i++)
        {
            anOriginals[i] = new BSplineCurve2D(NCollection_Array1<Point2D>.From(aPoles[i]),
                                                     NCollection_Array1<double>.From([2.0, 2.0, 1.0, 2.0, 2.0]),
                                                     NCollection_Array1<double>.From([0.0, 1.0]),
                                                     NCollection_Array1<int>.From([5, 5]), 4);
            anOriginals[i].Freeze();
            aCurves[i] = anOriginals[i];
        }

        bool isClosed = false;
        Geom2dConvert2D.ConcatC1(aCurves, NCollection_Array1<double>.From([1e-7], 0), out _,
                               out NCollection_HArray1<BSplineCurve2D> aResult, ref isClosed, 1e-7);
        Assert.Equal(1, aResult.Length());
        Assert.NotSame(anOriginals[0], aCurves[0]);
        Assert.Equal(2.0, anOriginals[0].Weight(1));
        Assert.Equal(1.0, aCurves[0].Weight(1));
        Assert.Equal(0.5, aCurves[0].Weight(3));
        Assert.True(aResult[0].StartPoint.DistanceTo(new Point2D(0, 0)) < Tol);
        Assert.True(aResult[0].EndPoint.DistanceTo(new Point2D(8, 0)) < Tol);
        Assert.True(MinDistance(aResult[0], anOriginals[0].Value(0.5)) < 1e-2);
        Assert.True(MinDistance(aResult[0], anOriginals[1].Value(0.5)) < 1e-2);
    }

    [Fact]
    public void C0BSplineToC1BSplineCurve_RemovesG1Knots()
    {
        BSplineCurve2D aC = Cubic();
        // Structurally C0 at knot 1, geometrically smooth
        BSplineCurve2D aC0 = aC.KnotInserted(1.0, 3);
        Assert.Equal(3, aC0.Multiplicity(2));
        aC0.Freeze();
        BSplineCurve2D aResult = aC0;
        Geom2dConvert2D.C0BSplineToC1BSplineCurve(ref aResult, 1e-7);
        Assert.NotSame(aC0, aResult);
        Assert.Equal(3, aC0.Multiplicity(2));
        Assert.True(aResult.Continuity >= C1);
        Assert.True(aResult.StartPoint.DistanceTo(aC.StartPoint) < Tol);
        Assert.True(aResult.EndPoint.DistanceTo(aC.EndPoint) < Tol);

        // Nothing to do
        BSplineCurve2D aSmooth = aC;
        Geom2dConvert2D.C0BSplineToC1BSplineCurve(ref aSmooth, 1e-7);
        Assert.Same(aC, aSmooth);
    }

    [Fact]
    public void C0BSplineToArrayOfC1BSplineCurve_SplitsAtCorners()
    {
        BSplineCurve2D aPolyline = new(NCollection_Array1<Point2D>.From([new Point2D(0, 0), new Point2D(1, 0), new Point2D(1, 1)]),
                                            NCollection_Array1<double>.From([0.0, 1.0, 2.0]), NCollection_Array1<int>.From([2, 1, 2]), 1);
        Geom2dConvert2D.C0BSplineToArrayOfC1BSplineCurve(aPolyline, out NCollection_HArray1<BSplineCurve2D> aTab, 1e-7);
        Assert.Equal(2, aTab.Length());
        Assert.True(aTab[0].EndPoint.DistanceTo(new Point2D(1, 0)) < Tol);
        Assert.True(aTab[1].StartPoint.DistanceTo(new Point2D(1, 0)) < Tol);

        BSplineCurve2D aC = Cubic();
        Geom2dConvert2D.C0BSplineToArrayOfC1BSplineCurve(aC, out aTab, 1e-7);
        Assert.Equal(1, aTab.Length());
        Assert.Same(aC, aTab[0]);
    }

    [Fact]
    public void ApproxCurve_OfAnOffsetCurve()
    {
        BSplineCurve2D aC = Cubic();
        OffsetCurve2D anOffset = new(aC, 0.2);
        ApproxCurve2D anApprox = new(anOffset, 1e-5, C2, 50, 9);
        Assert.True(anApprox.HasResult);
        Assert.True(anApprox.IsDone);
        Assert.True(anApprox.MaxError() < 1e-5);
        BSplineCurve2D aResult = anApprox.Curve()!;
        Assert.True(aResult.IsFrozen);
        Assert.Same(aResult, anApprox.Curve());
        Assert.True(aResult.Degree <= 9);
        AssertSamePoints(anOffset, 0.0, 3.0, aResult, aResult.FirstParameter, aResult.LastParameter, 1e-4);
        Assert.Contains("Dump of ApproxCurve", anApprox.ToString());

        // Through an adaptor on part of the curve
        ApproxCurve2D aPart = new(new CurveAdaptor2D(anOffset, 0.5, 2.5), 1e-5, C1, 50, 9);
        Assert.True(aPart.HasResult);
        Assert.Equal(0.5, aPart.Curve()!.FirstParameter);
        Assert.Equal(2.5, aPart.Curve()!.LastParameter);
    }

    [Fact]
    public void ApproxArcsSegments_LineAndCircle()
    {
        LineCurve2D aLine = new(new Point2D(0, 0), new Direction2D(1, 1));
        ApproxArcsSegments2D aLineApprox = new(new CurveAdaptor2D(aLine, 0.0, 5.0), 1e-3, 0.01);
        IReadOnlyList<Curve2D> aResult = aLineApprox.GetResult();
        Assert.Single(aResult);
        Assert.True(aResult[0].IsFrozen);
        Assert.True(aResult[0].Value(aResult[0].LastParameter).DistanceTo(aLine.Value(5.0)) < 1e-9);

        CircleCurve2D aCircle = new(new Axis2D(new Point2D(1, 1), new Direction2D(1, 0)), 2.0);
        ApproxArcsSegments2D aCircleApprox = new(new CurveAdaptor2D(aCircle, 0.0, 3.0), 1e-3, 0.01);
        aResult = aCircleApprox.GetResult();
        Assert.Equal(2, aResult.Count);
        foreach (Curve2D aCurve in aResult)
        {
            CircleCurve2D aBasis = (CircleCurve2D)((TrimmedCurve2D)aCurve).BasisCurve;
            Assert.Equal(2.0, aBasis.Radius, 1e-9);
        }
    }

    [Fact]
    public void ApproxArcsSegments_FreeForm()
    {
        BSplineCurve2D aC = Cubic();
        const double aTol = 1e-2;
        ApproxArcsSegments2D anApprox = new(new CurveAdaptor2D(aC), aTol, 0.1);
        IReadOnlyList<Curve2D> aResult = anApprox.GetResult();
        Assert.True(aResult.Count > 1);
        Assert.True(aResult[0].Value(aResult[0].FirstParameter).DistanceTo(aC.StartPoint) < 1e-7);
        Curve2D aLastCurve = aResult[aResult.Count - 1];
        Assert.True(aLastCurve.Value(aLastCurve.LastParameter).DistanceTo(aC.EndPoint) < 1e-7);
        for (int i = 0; i < aResult.Count; i++)
        {
            Curve2D aCurve = aResult[i];
            Assert.True(aCurve is TrimmedCurve2D { } aTrimmed && (aTrimmed.BasisCurve is LineCurve2D or CircleCurve2D));
            if (i > 0)
            {
                Curve2D aPrev = aResult[i - 1];
                Assert.True(aPrev.Value(aPrev.LastParameter).DistanceTo(aCurve.Value(aCurve.FirstParameter)) < 1e-6);
            }

            // Every piece lies within the tolerance of the curve
            for (int k = 0; k <= 10; k++)
            {
                Point2D aP = aCurve.Value(aCurve.FirstParameter + (aCurve.LastParameter - aCurve.FirstParameter) * k / 10.0);
                Assert.True(MinDistance(aC, aP) < aTol + 2e-3, $"piece {i} point {k}");
            }
        }
    }

    [Fact]
    public void PPoint()
    {
        PPoint2D anEmpty = new();
        Assert.Equal(RealLast(), anEmpty.Parameter());
        PPoint2D aP = new(1.0, new Xy(1, 2), new Xy(0, 1));
        PPoint2D aQ = new(1.0 + 1e-12, new Xy(5, 5), new Xy(1, 0));
        Assert.True(aP.IsEqual(aQ));
        Assert.False(aP == aQ);
        Assert.Equal(new Xy(3, 3), aP.WithD1(new Xy(3, 3)).D1());
        Assert.Equal(Math.Sqrt(9 + 16), aP.Dist(new PPoint2D(0.0, new Xy(4, 6), new Xy(0, 1))), 1e-15);

        CircleCurve2D aCircle = new(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 2.0);
        PPoint2D anOnCurve = new(M_PI / 2, new CurveAdaptor2D(aCircle));
        Assert.True(new Point2D(anOnCurve.Point()).DistanceTo(new Point2D(0, 2)) < 1e-15);
        Assert.True(new Vector2D(anOnCurve.D1()).IsParallel(new Vector2D(-1, 0), 1e-12));
    }
}
