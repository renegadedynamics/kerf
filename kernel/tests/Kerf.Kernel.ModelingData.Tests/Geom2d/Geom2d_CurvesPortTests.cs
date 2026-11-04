// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Evaluation;
using Kerf.Geometry;
using Xunit.Abstractions;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// OCCT.NET's own tests of the Geom2d curves: freezing and the non-mutating twins, the shared basis curves of
/// trimmed and offset curves (decision 132), the PackedList accessors (decision 131), reproductions of OCCT
/// issues the port keeps (occt-issues.md), and finite-difference checks of the Bezier and B-spline derivatives.
/// </summary>
public class Geom2d_CurvesPortTest
{
    private readonly ITestOutputHelper myOutput;

    public Geom2d_CurvesPortTest(ITestOutputHelper theOutput) => myOutput = theOutput;

    private static NCollection_Array1<T> Arr<T>(params T[] theValues) => NCollection_Array1<T>.From(theValues);

    private static Transform2D Translation(double theX, double theY)
    {
        Transform2D aTrsf = new Transform2D().WithTranslation(new Vector2D(theX, theY));
        return aTrsf;
    }

    private static BSplineCurve2D CubicBSpline()
    {
        // 6 poles, uniform interior knots, clamped ends: degree 3, knots 0..3
        return new BSplineCurve2D(Arr(new Point2D(0, 0), new Point2D(1, 2), new Point2D(2, -1),
                                           new Point2D(3, 3), new Point2D(4, 0), new Point2D(5, 1)),
                                       Arr(0.0, 1.0, 2.0, 3.0),
                                       Arr(4, 1, 1, 4),
                                       3);
    }

    private static BSplineCurve2D RationalBSpline()
    {
        return new BSplineCurve2D(Arr(new Point2D(0, 0), new Point2D(1, 2), new Point2D(2, -1),
                                           new Point2D(3, 3), new Point2D(4, 0), new Point2D(5, 1)),
                                       Arr(1.0, 2.0, 0.5, 1.5, 1.0, 3.0),
                                       Arr(0.0, 1.0, 2.0, 3.0),
                                       Arr(4, 1, 1, 4),
                                       3);
    }

    private static BezierCurve2D CubicBezier() =>
        new BezierCurve2D(Arr(new Point2D(0, 0), new Point2D(1, 1), new Point2D(2, 1), new Point2D(3, 0)));

    private static void AssertSamePoint(in Point2D theExpected, in Point2D theActual, double theTol = 1e-12)
    {
        Assert.Equal(theExpected.X, theActual.X, theTol);
        Assert.Equal(theExpected.Y, theActual.Y, theTol);
    }

    private static void AssertSameVec(in Vector2D theExpected, in Vector2D theActual, double theTol)
    {
        Assert.Equal(theExpected.X, theActual.X, theTol);
        Assert.Equal(theExpected.Y, theActual.Y, theTol);
    }

    // (1) Freezing and the non-mutating twins

    [Fact]
    public void Frozen_ConicMutators_Throw()
    {
        CircleCurve2D aCircle = new CircleCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 2.0);
        aCircle.Freeze();
        EngineAssert.Throws<ImmutableObjectException>(() => aCircle.SetRadius(3.0));
        EngineAssert.Throws<ImmutableObjectException>(() => aCircle.Transform(Translation(1, 0)));
        EngineAssert.Throws<ImmutableObjectException>(() => aCircle.Reverse());
        EngineAssert.Throws<ImmutableObjectException>(() => aCircle.SetLocation(new Point2D(1, 1)));

        EllipseCurve2D anEllipse = new EllipseCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 3.0, 1.0);
        anEllipse.Freeze();
        EngineAssert.Throws<ImmutableObjectException>(() => anEllipse.SetMajorRadius(4.0));
        EngineAssert.Throws<ImmutableObjectException>(() => anEllipse.SetMinorRadius(0.5));

        HyperbolaCurve2D aHyperbola = new HyperbolaCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 3.0, 1.0);
        aHyperbola.Freeze();
        EngineAssert.Throws<ImmutableObjectException>(() => aHyperbola.SetMajorRadius(4.0));

        ParabolaCurve2D aParabola = new ParabolaCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 2.0);
        aParabola.Freeze();
        EngineAssert.Throws<ImmutableObjectException>(() => aParabola.SetFocal(3.0));

        LineCurve2D aLine = new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0));
        aLine.Freeze();
        EngineAssert.Throws<ImmutableObjectException>(() => aLine.SetDirection(new Direction2D(0, 1)));
        EngineAssert.Throws<ImmutableObjectException>(() => aLine.Mirror(new Point2D(1, 1)));
    }

    [Fact]
    public void Frozen_BoundedCurveMutators_Throw()
    {
        BSplineCurve2D aBSpline = CubicBSpline();
        aBSpline.Freeze();
        EngineAssert.Throws<ImmutableObjectException>(() => aBSpline.SetPole(2, new Point2D(9, 9)));
        EngineAssert.Throws<ImmutableObjectException>(() => aBSpline.InsertKnot(0.5));
        EngineAssert.Throws<ImmutableObjectException>(() => aBSpline.Segment(0.5, 1.5));
        EngineAssert.Throws<ImmutableObjectException>(() => aBSpline.IncreaseDegree(4));
        EngineAssert.Throws<ImmutableObjectException>(() => aBSpline.SetPeriodic());
        EngineAssert.Throws<ImmutableObjectException>(() => aBSpline.RemoveKnot(2, 0, 1e-6));
        EngineAssert.Throws<ImmutableObjectException>(
            () => aBSpline.SetEvalRepresentation(new RepCurveDesc2D.Full { Representation = CubicBSpline() }));

        BezierCurve2D aBezier = CubicBezier();
        aBezier.Freeze();
        EngineAssert.Throws<ImmutableObjectException>(() => aBezier.SetPole(2, new Point2D(9, 9)));
        EngineAssert.Throws<ImmutableObjectException>(() => aBezier.Increase(4));
        EngineAssert.Throws<ImmutableObjectException>(() => aBezier.Segment(0.2, 0.8));
        EngineAssert.Throws<ImmutableObjectException>(() => aBezier.SetWeight(2, 2.0));

        TrimmedCurve2D aTrim = new TrimmedCurve2D(CubicBSpline(), 0.5, 2.5);
        aTrim.Freeze();
        EngineAssert.Throws<ImmutableObjectException>(() => aTrim.SetTrim(1.0, 2.0));
        EngineAssert.Throws<ImmutableObjectException>(() => aTrim.Reverse());

        OffsetCurve2D anOffset = new OffsetCurve2D(CubicBSpline(), 0.5);
        anOffset.Freeze();
        EngineAssert.Throws<ImmutableObjectException>(() => anOffset.SetOffsetValue(1.0));
        EngineAssert.Throws<ImmutableObjectException>(() => anOffset.Transform(Translation(1, 0)));
    }

    [Fact]
    public void Copy_IsUnfrozen()
    {
        Geometry2D[] aCurves =
        [
            new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0)),
            new CircleCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 2.0),
            new EllipseCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 3.0, 1.0),
            new HyperbolaCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 3.0, 1.0),
            new ParabolaCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 2.0),
            CubicBezier(),
            CubicBSpline(),
            new TrimmedCurve2D(CubicBSpline(), 0.5, 2.5),
            new OffsetCurve2D(CubicBSpline(), 0.5),
        ];
        foreach (Geometry2D aCurve in aCurves)
        {
            aCurve.Freeze();
            Geometry2D aCopy = aCurve.Copy();
            Assert.False(aCopy.IsFrozen);
            Assert.Equal(aCurve.GetType(), aCopy.GetType());
            // The transformed copy is a new object, and the frozen original is unchanged
            Curve2D aMoved = (Curve2D)aCurve.Translated(new Vector2D(1, 2));
            Curve2D anOrig = (Curve2D)aCurve;
            double aU = Math.Max(anOrig.FirstParameter, -1.0) + 0.1;
            Point2D aP = anOrig.Value(aU);
            Point2D aQ = aMoved.Value(aMoved.TransformedParameter(aU, Translation(1, 2)));
            AssertSamePoint(new Point2D(aP.X + 1, aP.Y + 2), aQ, 1e-12);
        }
    }

    [Fact]
    public void Twins_DontChangeOriginal_Conics()
    {
        CircleCurve2D aCircle = new CircleCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 2.0);
        CircleCurve2D aBig = aCircle.WithRadius(5.0);
        Assert.Equal(2.0, aCircle.Radius);
        Assert.Equal(5.0, aBig.Radius);
        EngineAssert.Throws<ConstructionErrorException>(() => aCircle.WithRadius(-1.0));

        EllipseCurve2D anEllipse = new EllipseCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 3.0, 1.0);
        Assert.Equal(4.0, anEllipse.WithMajorRadius(4.0).MajorRadius);
        Assert.Equal(0.5, anEllipse.WithMinorRadius(0.5).MinorRadius);
        Assert.Equal(3.0, anEllipse.MajorRadius);
        Assert.Equal(1.0, anEllipse.MinorRadius);

        ParabolaCurve2D aParabola = new ParabolaCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 2.0);
        Assert.Equal(7.0, aParabola.WithFocal(7.0).Focal);
        Assert.Equal(2.0, aParabola.Focal);

        LineCurve2D aLine = new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0));
        LineCurve2D aMoved = aLine.WithLocation(new Point2D(3, 4));
        AssertSamePoint(new Point2D(0, 0), aLine.Location);
        AssertSamePoint(new Point2D(3, 4), aMoved.Location);

        ConicCurve2D aConic = aCircle.WithLocation(new Point2D(1, 1));
        AssertSamePoint(new Point2D(1, 1), aConic.Location);
        AssertSamePoint(new Point2D(0, 0), aCircle.Location);

        Curve2D aReversed = aCircle.Reversed();
        Assert.False(aReversed.IsFrozen);
        Assert.True(aCircle.Position.YDirection.IsOpposite(((CircleCurve2D)aReversed).Position.YDirection, 1e-12));
        Assert.True(aCircle.Position.YDirection.IsEqual(new Direction2D(0, 1), 1e-12));
    }

    [Fact]
    public void Twins_DontChangeOriginal_BSpline()
    {
        BSplineCurve2D aCurve = CubicBSpline();
        aCurve.Freeze();
        Point2D aMid = aCurve.Value(1.5);

        // KnotsInserted: Add defaults to false, as Geom2d_BSplineCurve::InsertKnots
        BSplineCurve2D anInserted = aCurve.KnotsInserted(Arr(0.5, 1.0), Arr(1, 2));
        Assert.Equal(4, aCurve.KnotCount);
        Assert.Equal(5, anInserted.KnotCount);
        // Knot 1.0 existed with multiplicity 1: raised TO 2 (not by 2)
        Assert.Equal(2, anInserted.Multiplicity(3));
        AssertSamePoint(aMid, anInserted.Value(1.5), 1e-12);
        BSplineCurve2D anAdded = aCurve.KnotsInserted(Arr(1.0), Arr(1), 0.0, true);
        Assert.Equal(2, anAdded.Multiplicity(2));

        BSplineCurve2D aOneInserted = aCurve.KnotInserted(0.5, 2);
        Assert.Equal(5, aOneInserted.KnotCount);
        Assert.Equal(2, aOneInserted.Multiplicity(2));

        // KnotRemoved: possible when the knot was just inserted, null when the curve would change
        BSplineCurve2D? aRemoved = aOneInserted.KnotRemoved(2, 0, 1e-9);
        Assert.NotNull(aRemoved);
        Assert.Equal(4, aRemoved.KnotCount);
        Assert.Null(aCurve.KnotRemoved(2, 0, 1e-9));
        Assert.Equal(4, aCurve.KnotCount);

        BSplineCurve2D aSegment = aCurve.Segmented(0.5, 2.5);
        Assert.Equal(0.5, aSegment.FirstParameter, 1e-12);
        Assert.Equal(2.5, aSegment.LastParameter, 1e-12);
        Assert.Equal(0.0, aCurve.FirstParameter);
        Assert.Equal(3.0, aCurve.LastParameter);

        BSplineCurve2D aPeriodic = aCurve.WithPeriodic();
        Assert.True(aPeriodic.IsPeriodic);
        Assert.False(aCurve.IsPeriodic);
        BSplineCurve2D aNotPeriodic = aPeriodic.WithNotPeriodic();
        Assert.False(aNotPeriodic.IsPeriodic);
        Assert.True(aPeriodic.IsPeriodic);
        AssertSamePoint(aPeriodic.Value(1.3), aNotPeriodic.Value(1.3), 1e-10);

        BSplineCurve2D aRaised = aCurve.DegreeIncreased(5);
        Assert.Equal(5, aRaised.Degree);
        Assert.Equal(3, aCurve.Degree);
        AssertSamePoint(aMid, aRaised.Value(1.5), 1e-12);

        BSplineCurve2D aWeighted = aCurve.WithWeightOneBased(3, 2.0);
        Assert.True(aWeighted.IsRational);
        Assert.False(aCurve.IsRational);

        BSplineCurve2D aPoled = aCurve.WithPoleOneBased(2, new Point2D(9, 9));
        AssertSamePoint(new Point2D(9, 9), aPoled.Pole(2));
        AssertSamePoint(new Point2D(1, 2), aCurve.Pole(2));

        BSplineCurve2D aMovedPoint = aCurve.PointMoved(1.5, new Point2D(2.5, 2.0), 1, aCurve.PoleCount, out int aFirst, out _);
        Assert.True(aFirst > 0);
        AssertSamePoint(new Point2D(2.5, 2.0), aMovedPoint.Value(1.5), 1e-9);
        AssertSamePoint(aMid, aCurve.Value(1.5), 0.0);

        Assert.False(aCurve.Reversed().IsFrozen);
        Assert.True(aCurve.IsFrozen);
    }

    [Fact]
    public void Twins_DontChangeOriginal_Bezier()
    {
        BezierCurve2D aCurve = CubicBezier();
        aCurve.Freeze();
        Point2D aMid = aCurve.Value(0.5);

        BezierCurve2D anIncreased = aCurve.Increased(5);
        Assert.Equal(5, anIncreased.Degree);
        Assert.Equal(3, aCurve.Degree);
        AssertSamePoint(aMid, anIncreased.Value(0.5), 1e-12);

        BezierCurve2D aSegment = aCurve.Segmented(0.25, 0.75);
        AssertSamePoint(aMid, aSegment.Value(0.5), 1e-12);
        AssertSamePoint(new Point2D(0, 0), aCurve.StartPoint);

        Assert.Equal(5, aCurve.PoleInsertedAfter(2, new Point2D(1.5, 2)).PoleCount);
        Assert.Equal(3, aCurve.PoleRemoved(2).PoleCount);
        Assert.Equal(4, aCurve.PoleCount);
        Assert.True(aCurve.WithWeightOneBased(2, 3.0).IsRational);
        Assert.False(aCurve.IsRational);
    }

    [Fact]
    public void Twins_TrimmedAndOffset()
    {
        TrimmedCurve2D aTrim = new TrimmedCurve2D(CubicBSpline(), 0.5, 2.5);
        TrimmedCurve2D aNarrow = aTrim.WithTrim(1.0, 2.0);
        Assert.Equal(1.0, aNarrow.FirstParameter);
        Assert.Equal(0.5, aTrim.FirstParameter);

        OffsetCurve2D anOffset = new OffsetCurve2D(CubicBSpline(), 0.5);
        OffsetCurve2D aFar = anOffset.WithOffsetValue(2.0);
        Assert.Equal(2.0, aFar.Offset);
        Assert.Equal(0.5, anOffset.Offset);
    }

    // (2) Decision 132: shared basis curves are frozen; their owner copies on write

    [Fact]
    public void TrimmedCurve_BasisCurve_FrozenAndCopyOnWrite()
    {
        BSplineCurve2D aCaller = CubicBSpline();
        TrimmedCurve2D aTrim = new TrimmedCurve2D(aCaller, 0.5, 2.5);
        // The constructor copies the basis: the caller's curve isn't shared nor frozen
        Assert.False(aCaller.IsFrozen);

        Curve2D aBasis = aTrim.BasisCurve;
        Assert.True(aBasis.IsFrozen);
        Assert.NotSame(aCaller, aBasis);
        Assert.Same(aBasis, aTrim.BasisCurve);
        Point2D aBasisPoint = aBasis.Value(1.0);

        // The trimmed curve is still mutable: it copies its basis first
        aTrim.Transform(Translation(10, 0));
        AssertSamePoint(new Point2D(aBasisPoint.X + 10, aBasisPoint.Y), aTrim.Value(1.0));
        AssertSamePoint(aBasisPoint, aBasis.Value(1.0), 0.0);
        Assert.NotSame(aBasis, aTrim.BasisCurve);

        Curve2D aBasis2 = aTrim.BasisCurve;
        Point2D aStart = aTrim.StartPoint;
        aTrim.Reverse();
        AssertSamePoint(aStart, aTrim.EndPoint, 1e-12);
        AssertSamePoint(new Point2D(aBasisPoint.X + 10, aBasisPoint.Y), aBasis2.Value(1.0));

        // A trimmed curve of a trimmed curve doesn't freeze the inner basis
        TrimmedCurve2D anInner = new TrimmedCurve2D(CubicBSpline(), 0.5, 2.5);
        TrimmedCurve2D anOuter = new TrimmedCurve2D(anInner, 1.0, 2.0);
        Assert.IsType<BSplineCurve2D>(anOuter.BasisCurve);
        anInner.Transform(Translation(1, 1)); // would throw if the constructor had frozen its basis
    }

    [Fact]
    public void OffsetCurve_BasisCurve_FrozenAndCopyOnWrite()
    {
        BSplineCurve2D aCaller = CubicBSpline();
        OffsetCurve2D anOffset = new OffsetCurve2D(aCaller, 0.5);

        // As in OCCT the offset curve shares the caller's curve, which is therefore frozen
        Assert.True(aCaller.IsFrozen);
        Assert.Same(aCaller, anOffset.BasisCurve);
        EngineAssert.Throws<ImmutableObjectException>(() => aCaller.SetPole(2, new Point2D(0, 9)));

        Point2D aBasisPoint = aCaller.Value(1.0);
        Point2D anOffsetPoint = anOffset.Value(1.0);

        anOffset.Transform(Translation(0, 5));
        AssertSamePoint(new Point2D(anOffsetPoint.X, anOffsetPoint.Y + 5), anOffset.Value(1.0), 1e-12);
        AssertSamePoint(aBasisPoint, aCaller.Value(1.0), 0.0);
        Assert.NotSame(aCaller, anOffset.BasisCurve);

        Curve2D aBasis2 = anOffset.BasisCurve;
        Assert.True(aBasis2.IsFrozen);
        anOffset.Reverse();
        Assert.Equal(-0.5, anOffset.Offset);
        AssertSamePoint(new Point2D(aBasisPoint.X, aBasisPoint.Y + 5), aBasis2.Value(1.0), 0.0);

        // An offset of a trimmed curve rebuilds the trimmed curve (a copy): the caller's isn't frozen
        TrimmedCurve2D aTrim = new TrimmedCurve2D(CubicBSpline(), 0.5, 2.5);
        OffsetCurve2D anOffsetOfTrim = new OffsetCurve2D(aTrim, 0.5);
        Assert.False(aTrim.IsFrozen);
        Assert.NotSame(aTrim, anOffsetOfTrim.BasisCurve);

        // An offset of an offset adds the offsets and shares the inner basis
        OffsetCurve2D anOffsetOfOffset = new OffsetCurve2D(anOffset, 0.25);
        Assert.Equal(-0.25, anOffsetOfOffset.Offset, 1e-15);
    }

    // (3) Decision 131: returned arrays are copies

    [Fact]
    public void BSpline_ArrayAccessors_AreCopies()
    {
        BSplineCurve2D aCurve = RationalBSpline();

        Assert.Equal(aCurve.InternalPoles().AsSpan().ToArray(), aCurve.Poles.ToArray());
        Assert.Equal(aCurve.InternalKnots().AsSpan().ToArray(), aCurve.Knots.ToArray());
        Assert.Equal(aCurve.InternalMultiplicities().AsSpan().ToArray(), aCurve.Multiplicities.ToArray());
        Assert.Equal(aCurve.InternalKnotSequence().AsSpan().ToArray(), aCurve.KnotSequence.ToArray());
        IReadOnlyList<double>? aWeights = aCurve.WeightsOrNull();
        Assert.NotNull(aWeights);
        Assert.Equal(aCurve.InternalWeights()!.AsSpan().ToArray(), aWeights.ToArray());
        Assert.Equal(aCurve.InternalWeightsArray().AsSpan().ToArray(), aCurve.Weights.ToArray());

        // A later internal change doesn't reach a list handed out before it
        IReadOnlyList<Point2D> aPoles = aCurve.Poles;
        IReadOnlyList<double> aKnots = aCurve.Knots;
        aCurve.SetPole(2, new Point2D(42, 42));
        aCurve.SetKnot(2, 1.25);
        Assert.Equal(new Point2D(1, 2), aPoles[1]);
        Assert.Equal(1.0, aKnots[1]);
        Assert.Equal(new Point2D(42, 42), aCurve.Poles[1]);

        // Non-rational: Weights() is null, WeightsArray() is ones
        BSplineCurve2D aPlain = CubicBSpline();
        Assert.Null(aPlain.WeightsOrNull());
        Assert.Null(aPlain.InternalWeights());
        Assert.All(aPlain.Weights.ToArray(), w => Assert.Equal(1.0, w));
        Assert.Equal(aPlain.PoleCount, aPlain.Weights.Count);
    }

    [Fact]
    public void Bezier_ArrayAccessors_AreCopies()
    {
        BezierCurve2D aCurve = new BezierCurve2D(Arr(new Point2D(0, 0), new Point2D(1, 1), new Point2D(2, 0)),
                                                           Arr(1.0, 2.0, 1.0));
        Assert.Equal(aCurve.InternalPoles().AsSpan().ToArray(), aCurve.Poles.ToArray());
        Assert.Equal(new[] { 0.0, 1.0 }, aCurve.Knots.ToArray());
        Assert.Equal(new[] { 3, 3 }, aCurve.Multiplicities.ToArray());
        Assert.Equal(new[] { 0.0, 0.0, 0.0, 1.0, 1.0, 1.0 }, aCurve.KnotSequence.ToArray());
        Assert.Equal(new[] { 1.0, 2.0, 1.0 }, aCurve.WeightsOrNull()!.ToArray());
        Assert.Equal(new[] { 1.0, 2.0, 1.0 }, aCurve.Weights.ToArray());

        IReadOnlyList<Point2D> aPoles = aCurve.Poles;
        aCurve.SetPole(2, new Point2D(5, 5));
        Assert.Equal(new Point2D(1, 1), aPoles[1]);

        // The shared unit weights are never written: making a non-rational curve rational copies them
        BezierCurve2D aPlain = CubicBezier();
        Assert.Null(aPlain.WeightsOrNull());
        aPlain.SetWeight(2, 3.0);
        Assert.Equal(1.0, global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(4)[2]);
        Assert.Null(CubicBezier().WeightsOrNull());
        Assert.Equal(1.0, CubicBezier().Weights[1]);
    }

    // (4) Reproductions of OCCT issues

    /// <summary>
    /// occt-issues.md, Geom2d_OffsetCurveUtils::CalculateD3 and EvaluateDN (Kerf decision 1487): OCCT's EvalD3
    /// disagreed with EvalD2 (and with the derivative of EvalD2) wherever the basis curve's speed varies. Now its D2 is
    /// EvalD2's and its D3 the derivative of D2, through the curve, Adaptor2d_OffsetCurve and the adaptor's DN(3).
    /// </summary>
    [Fact]
    public void OffsetCurveEvalD3_IsTheDerivativeOfEvalD2()
    {
        EllipseCurve2D anEllipse = new EllipseCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 3.0, 1.0);
        OffsetCurve2D anOffset = new OffsetCurve2D(anEllipse, 0.5);
        double aU = 0.7, h = 1e-4;

        Curve2D.ResD2 aD2 = anOffset.EvalD2(aU);
        Curve2D.ResD3 aD3 = anOffset.EvalD3(aU);
        Vector2D aFdD3 = (anOffset.EvalD2(aU + h).D2 - anOffset.EvalD2(aU - h).D2) / (2.0 * h);
        Vector2D aFdD2 = (anOffset.EvalD1(aU + h).D1 - anOffset.EvalD1(aU - h).D1) / (2.0 * h);
        myOutput.WriteLine($"ellipse: EvalD2.D2 = {aD2.D2}, EvalD3.D2 = {aD3.D2}, FD(D1) = {aFdD2}");
        myOutput.WriteLine($"ellipse: EvalD3.D3 = {aD3.D3}, FD(EvalD2.D2) = {aFdD3}");

        // EvalD2's D2 is right (it matches the derivative of D1); point and D1 agree between EvalD2 and EvalD3
        AssertSameVec(aFdD2, aD2.D2, 1e-6);
        AssertSamePoint(aD2.Point, aD3.Point, 1e-15);
        AssertSameVec(aD2.D1, aD3.D1, 1e-15);
        // EvalD3's D2 is EvalD2's, its D3 the finite difference of D2
        AssertSameVec(aD2.D2, aD3.D2, 1e-12);
        AssertSameVec(aFdD3, aD3.D3, 1e-6);
        AssertSameVec(aD3.D3, new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D(anOffset).EvalDN(aU, 3), 1e-12);

        // On a circle they agree
        CircleCurve2D aCircle = new CircleCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 3.0);
        OffsetCurve2D aCircleOffset = new OffsetCurve2D(aCircle, 0.5);
        Curve2D.ResD2 aCD2 = aCircleOffset.EvalD2(aU);
        Curve2D.ResD3 aCD3 = aCircleOffset.EvalD3(aU);
        Vector2D aCFdD3 = (aCircleOffset.EvalD2(aU + h).D2 - aCircleOffset.EvalD2(aU - h).D2) / (2.0 * h);
        myOutput.WriteLine($"circle: EvalD2.D2 = {aCD2.D2}, EvalD3.D2 = {aCD3.D2}, EvalD3.D3 = {aCD3.D3}, FD = {aCFdD3}");
        AssertSameVec(aCD2.D2, aCD3.D2, 1e-12);
        AssertSameVec(aCFdD3, aCD3.D3, 1e-6);

        // Adaptor2d_OffsetCurve uses the same ones
        OffsetCurveAdaptor2D anAdaptor = new OffsetCurveAdaptor2D(new TestCurveAdaptor(anEllipse), 0.5);
        anAdaptor.D3(aU, out Point2D aAP, out Vector2D aAV1, out Vector2D aAV2, out Vector2D aAV3);
        AssertSamePoint(aD3.Point, aAP, 1e-15);
        AssertSameVec(aD3.D2, aAV2, 1e-15);
        AssertSameVec(aD3.D3, aAV3, 1e-15);
    }

    /// <summary>
    /// occt-issues.md: Geom2d_BSplineCurve::IncreaseDegree sizes the new poles as NbPoles + Step * (LastUKnotIndex -
    /// FirstUKnotIndex), which counts only the knots inside the parameter range. On a non-periodic curve whose end
    /// knots have multiplicity below Degree + 1 (unclamped), BSplCLib::IncreaseDegree raises every multiplicity, so
    /// the pole count no longer matches the multiplicities. Native OCCT gives the same curve (oracle tests).
    /// </summary>
    [Fact]
    public void OcctIssue_BSplineIncreaseDegree_UnclampedCurveInconsistent()
    {
        // Clamped at the start, multiplicity 2 at the end: 8 poles
        NCollection_Array1<Point2D> aPoles = Arr(new Point2D(0, 0), new Point2D(1, 2), new Point2D(2, -1),
                                                  new Point2D(3, 2), new Point2D(4, 0), new Point2D(5, 1),
                                                  new Point2D(6, -1), new Point2D(7, 0));
        NCollection_Array1<double> aKnots = Arr(0.0, 1.0, 2.0, 3.0, 4.0);
        NCollection_Array1<int> aMults = Arr(4, 2, 2, 2, 2);
        BSplineCurve2D aCurve = new BSplineCurve2D(aPoles, aKnots, aMults, 3);
        Assert.Equal(global::Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(3, false, aCurve.InternalMultiplicities()), aCurve.PoleCount);

        aCurve.IncreaseDegree(4);
        int anExpected = global::Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(aCurve.Degree, false, aCurve.InternalMultiplicities());
        myOutput.WriteLine($"after IncreaseDegree(4): {aCurve.PoleCount} poles, multiplicities "
                           + $"[{string.Join(", ", aCurve.Multiplicities)}] need {anExpected}");
        // OCCT: the pole count doesn't match the multiplicities
        Assert.NotEqual(anExpected, aCurve.PoleCount);

        // Unclamped at both ends, BSplCLib::IncreaseDegree writes past the new poles (native OCCT, whose
        // NCollection range checks are on, throws Standard_OutOfRange too)
        BSplineCurve2D anUnclamped = new BSplineCurve2D(
            Arr(new Point2D(0, 0), new Point2D(1, 2), new Point2D(2, -1), new Point2D(3, 2), new Point2D(4, 0)),
            Arr(0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0),
            Arr(1, 1, 1, 1, 1, 1, 1, 1, 1),
            3);
        EngineAssert.ThrowsAny<OutOfRangeException>(() => anUnclamped.IncreaseDegree(4));
    }

    /// <summary>
    /// occt-issues.md: Geom2d_BSplineCurve never drops an attached evaluation representation, so after Transform
    /// (or SetPole, ...) it still evaluates through the old representation. Geom_BSplineCurve drops it.
    /// </summary>
    [Fact]
    public void OcctIssue_BSplineKeepsEvalRepresentationAfterModification()
    {
        BSplineCurve2D aCurve = CubicBSpline();
        Point2D aBefore = aCurve.Value(1.5);
        aCurve.SetEvalRepresentation(new RepCurveDesc2D.Full { Representation = (Curve2D)aCurve.Copy() });
        Assert.True(aCurve.HasEvalRepresentation);

        aCurve.Transform(Translation(10, 0));
        Assert.True(aCurve.HasEvalRepresentation);
        // OCCT: still the untransformed point
        AssertSamePoint(aBefore, aCurve.EvalD0(1.5), 0.0);
        // ... although the poles did move
        AssertSamePoint(new Point2D(10, 0), aCurve.Pole(1), 0.0);

        aCurve.SetPole(3, new Point2D(50, 50));
        Assert.True(aCurve.HasEvalRepresentation);
        AssertSamePoint(aBefore, aCurve.EvalD0(1.5), 0.0);

        // Without the representation the curve evaluates its own (changed) poles
        BSplineCurve2D aCleared = aCurve.WithoutEvalRepresentation();
        Assert.False(aCleared.HasEvalRepresentation);
        Assert.True(aCleared.EvalD0(1.5).DistanceTo(aBefore) > 1.0);
    }

    /// <summary>
    /// occt-issues.md: Geom2d_BezierCurve drops an attached evaluation representation in Transform (and when its
    /// poles are replaced), but keeps it through SetPole, SetWeight, Reverse and Segment.
    /// </summary>
    [Fact]
    public void OcctIssue_BezierKeepsEvalRepresentationExceptOnTransform()
    {
        BezierCurve2D aCurve = CubicBezier();
        Point2D aBefore = aCurve.Value(0.3);
        aCurve.SetEvalRepresentation(new RepCurveDesc2D.Full { Representation = (Curve2D)aCurve.Copy() });

        aCurve.SetPole(2, new Point2D(1, 5));
        Assert.True(aCurve.HasEvalRepresentation);
        AssertSamePoint(aBefore, aCurve.EvalD0(0.3), 0.0);

        aCurve.Reverse();
        Assert.True(aCurve.HasEvalRepresentation);
        AssertSamePoint(aBefore, aCurve.EvalD0(0.3), 0.0);

        aCurve.Transform(Translation(10, 0));
        Assert.False(aCurve.HasEvalRepresentation);
        Assert.True(aCurve.EvalD0(0.3).X > 9.0);

        // Increase rebuilds the poles (init), which also drops it
        BezierCurve2D aSecond = CubicBezier();
        aSecond.SetEvalRepresentation(new RepCurveDesc2D.Full { Representation = (Curve2D)aSecond.Copy() });
        aSecond.Increase(4);
        Assert.False(aSecond.HasEvalRepresentation);
    }

    [Fact]
    public void EvalRepresentation_Twins_CopyTheDescriptor()
    {
        BSplineCurve2D aCurve = CubicBSpline();
        BSplineCurve2D aRep = CubicBSpline().WithPoleOneBased(3, new Point2D(2, 7));
        RepCurveDesc2D.Full aDesc = new RepCurveDesc2D.Full { Representation = aRep };
        BSplineCurve2D aWith = aCurve.WithEvalRepresentation(aDesc);
        Assert.False(aCurve.HasEvalRepresentation);
        Assert.True(aWith.HasEvalRepresentation);
        AssertSamePoint(aRep.Value(1.2), aWith.EvalD0(1.2), 0.0);
        // Changing the caller's descriptor later doesn't reach the curve
        aDesc.Representation = CubicBSpline();
        AssertSamePoint(aRep.Value(1.2), aWith.EvalD0(1.2), 0.0);
        // A descriptor whose representation is the curve itself is rejected
        EngineAssert.Throws<ProgramErrorException>(
            () => aCurve.WithEvalRepresentation(new RepCurveDesc2D.Full { Representation = aCurve }));
    }

    // (5) Finite-difference checks

    private static void CheckDerivatives(Curve2D theCurve, double theU)
    {
        const double h = 1e-5;
        Curve2D.ResD2 aD2 = theCurve.EvalD2(theU);
        Vector2D aFd1 = new Vector2D(theCurve.EvalD0(theU - h), theCurve.EvalD0(theU + h)) / (2.0 * h);
        Vector2D aFd2 = (theCurve.EvalD1(theU + h).D1 - theCurve.EvalD1(theU - h).D1) / (2.0 * h);
        double aScale1 = Math.Max(1.0, aD2.D1.Magnitude);
        double aScale2 = Math.Max(1.0, aD2.D2.Magnitude);
        Assert.True((aFd1 - aD2.D1).Magnitude < 1e-6 * aScale1, $"D1 at {theU}: {aD2.D1} vs {aFd1}");
        Assert.True((aFd2 - aD2.D2).Magnitude < 1e-5 * aScale2, $"D2 at {theU}: {aD2.D2} vs {aFd2}");
        Curve2D.ResD3 aD3 = theCurve.EvalD3(theU);
        AssertSamePoint(aD2.Point, aD3.Point, 1e-12);
        AssertSameVec(aD2.D1, theCurve.EvalDN(theU, 1), 1e-9 * aScale1);
        AssertSameVec(aD2.D2, theCurve.EvalDN(theU, 2), 1e-9 * aScale2);
    }

    [Fact]
    public void FiniteDifferences_BSplineAndBezier()
    {
        Random aRandom = new Random(20261029);
        for (int aTrial = 0; aTrial < 20; ++aTrial)
        {
            int aNbPoles = 4 + aRandom.Next(5);
            Point2D[] aPoles = new Point2D[aNbPoles];
            double[] aWeights = new double[aNbPoles];
            for (int i = 0; i < aNbPoles; ++i)
            {
                aPoles[i] = new Point2D(aRandom.NextDouble() * 10 - 5, aRandom.NextDouble() * 10 - 5);
                aWeights[i] = 0.5 + aRandom.NextDouble() * 2;
            }

            BezierCurve2D aBezier = new BezierCurve2D(Arr(aPoles), Arr(aWeights));
            BezierCurve2D aPlainBezier = new BezierCurve2D(Arr(aPoles));

            // Cubic B-spline with uniform interior knots
            int aDeg = 3;
            int aNbKnots = aNbPoles - aDeg + 1;
            double[] aKnots = new double[aNbKnots];
            int[] aMults = new int[aNbKnots];
            for (int i = 0; i < aNbKnots; ++i)
            {
                aKnots[i] = i;
                aMults[i] = 1;
            }
            aMults[0] = aMults[aNbKnots - 1] = aDeg + 1;
            BSplineCurve2D aBSpline = new BSplineCurve2D(Arr(aPoles), Arr(aWeights), Arr(aKnots), Arr(aMults), aDeg);

            for (int k = 0; k < 5; ++k)
            {
                double t = 0.05 + 0.9 * aRandom.NextDouble();
                CheckDerivatives(aBezier, t);
                CheckDerivatives(aPlainBezier, t);
                double u = t * (aNbKnots - 1);
                // stay away from the knots, where the B-spline is only C2
                if (Math.Abs(u - Math.Round(u)) > 1e-3)
                {
                    CheckDerivatives(aBSpline, u);
                }
            }
        }
    }

    /// <summary>A minimal adaptor over a Geom2d curve (Geom2dAdaptor_Curve isn't ported yet).</summary>
    private sealed class TestCurveAdaptor : Curve2DAdaptorBase
    {
        private readonly Curve2D myCurve;

        public TestCurveAdaptor(Curve2D theCurve) => myCurve = theCurve;

        public override double FirstParameter() => myCurve.FirstParameter;

        public override double LastParameter() => myCurve.LastParameter;

        public override Point2D Value(double U) => myCurve.Value(U);

        public override void D0(double U, out Point2D P) => myCurve.D0(U, out P);

        public override void D1(double U, out Point2D P, out Vector2D V) => myCurve.D1(U, out P, out V);

        public override void D2(double U, out Point2D P, out Vector2D V1, out Vector2D V2) =>
            myCurve.D2(U, out P, out V1, out V2);

        public override void D3(double U, out Point2D P, out Vector2D V1, out Vector2D V2, out Vector2D V3) =>
            myCurve.D3(U, out P, out V1, out V2, out V3);

        public override Vector2D DN(double U, int N) => myCurve.DN(U, N);
    }
}
