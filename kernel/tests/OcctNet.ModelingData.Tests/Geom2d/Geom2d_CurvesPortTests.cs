// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// OCCT.NET's own tests of the Geom2d curves: freezing and the non-mutating twins, the shared basis curves of
/// trimmed and offset curves (decision 702), the PackedList accessors (decision 701), reproductions of OCCT
/// issues the port keeps (occt-issues.md), and finite-difference checks of the Bezier and B-spline derivatives.
/// </summary>
public class Geom2d_CurvesPortTest
{
    private readonly ITestOutputHelper myOutput;

    public Geom2d_CurvesPortTest(ITestOutputHelper theOutput) => myOutput = theOutput;

    private static NCollection_Array1<T> Arr<T>(params T[] theValues) => NCollection_Array1<T>.From(theValues);

    private static gp_Trsf2d Translation(double theX, double theY)
    {
        gp_Trsf2d aTrsf = new gp_Trsf2d();
        aTrsf.SetTranslation(new gp_Vec2d(theX, theY));
        return aTrsf;
    }

    private static Geom2d_BSplineCurve CubicBSpline()
    {
        // 6 poles, uniform interior knots, clamped ends: degree 3, knots 0..3
        return new Geom2d_BSplineCurve(Arr(new gp_Pnt2d(0, 0), new gp_Pnt2d(1, 2), new gp_Pnt2d(2, -1),
                                           new gp_Pnt2d(3, 3), new gp_Pnt2d(4, 0), new gp_Pnt2d(5, 1)),
                                       Arr(0.0, 1.0, 2.0, 3.0),
                                       Arr(4, 1, 1, 4),
                                       3);
    }

    private static Geom2d_BSplineCurve RationalBSpline()
    {
        return new Geom2d_BSplineCurve(Arr(new gp_Pnt2d(0, 0), new gp_Pnt2d(1, 2), new gp_Pnt2d(2, -1),
                                           new gp_Pnt2d(3, 3), new gp_Pnt2d(4, 0), new gp_Pnt2d(5, 1)),
                                       Arr(1.0, 2.0, 0.5, 1.5, 1.0, 3.0),
                                       Arr(0.0, 1.0, 2.0, 3.0),
                                       Arr(4, 1, 1, 4),
                                       3);
    }

    private static Geom2d_BezierCurve CubicBezier() =>
        new Geom2d_BezierCurve(Arr(new gp_Pnt2d(0, 0), new gp_Pnt2d(1, 1), new gp_Pnt2d(2, 1), new gp_Pnt2d(3, 0)));

    private static void AssertSamePoint(in gp_Pnt2d theExpected, in gp_Pnt2d theActual, double theTol = 1e-12)
    {
        Assert.Equal(theExpected.X(), theActual.X(), theTol);
        Assert.Equal(theExpected.Y(), theActual.Y(), theTol);
    }

    private static void AssertSameVec(in gp_Vec2d theExpected, in gp_Vec2d theActual, double theTol)
    {
        Assert.Equal(theExpected.X(), theActual.X(), theTol);
        Assert.Equal(theExpected.Y(), theActual.Y(), theTol);
    }

    // (1) Freezing and the non-mutating twins

    [Fact]
    public void Frozen_ConicMutators_Throw()
    {
        Geom2d_Circle aCircle = new Geom2d_Circle(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 2.0);
        aCircle.Freeze();
        Assert.Throws<Standard_ImmutableObject>(() => aCircle.SetRadius(3.0));
        Assert.Throws<Standard_ImmutableObject>(() => aCircle.Transform(Translation(1, 0)));
        Assert.Throws<Standard_ImmutableObject>(() => aCircle.Reverse());
        Assert.Throws<Standard_ImmutableObject>(() => aCircle.SetLocation(new gp_Pnt2d(1, 1)));

        Geom2d_Ellipse anEllipse = new Geom2d_Ellipse(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 3.0, 1.0);
        anEllipse.Freeze();
        Assert.Throws<Standard_ImmutableObject>(() => anEllipse.SetMajorRadius(4.0));
        Assert.Throws<Standard_ImmutableObject>(() => anEllipse.SetMinorRadius(0.5));

        Geom2d_Hyperbola aHyperbola = new Geom2d_Hyperbola(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 3.0, 1.0);
        aHyperbola.Freeze();
        Assert.Throws<Standard_ImmutableObject>(() => aHyperbola.SetMajorRadius(4.0));

        Geom2d_Parabola aParabola = new Geom2d_Parabola(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 2.0);
        aParabola.Freeze();
        Assert.Throws<Standard_ImmutableObject>(() => aParabola.SetFocal(3.0));

        Geom2d_Line aLine = new Geom2d_Line(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0));
        aLine.Freeze();
        Assert.Throws<Standard_ImmutableObject>(() => aLine.SetDirection(new gp_Dir2d(0, 1)));
        Assert.Throws<Standard_ImmutableObject>(() => aLine.Mirror(new gp_Pnt2d(1, 1)));
    }

    [Fact]
    public void Frozen_BoundedCurveMutators_Throw()
    {
        Geom2d_BSplineCurve aBSpline = CubicBSpline();
        aBSpline.Freeze();
        Assert.Throws<Standard_ImmutableObject>(() => aBSpline.SetPole(2, new gp_Pnt2d(9, 9)));
        Assert.Throws<Standard_ImmutableObject>(() => aBSpline.InsertKnot(0.5));
        Assert.Throws<Standard_ImmutableObject>(() => aBSpline.Segment(0.5, 1.5));
        Assert.Throws<Standard_ImmutableObject>(() => aBSpline.IncreaseDegree(4));
        Assert.Throws<Standard_ImmutableObject>(() => aBSpline.SetPeriodic());
        Assert.Throws<Standard_ImmutableObject>(() => aBSpline.RemoveKnot(2, 0, 1e-6));
        Assert.Throws<Standard_ImmutableObject>(
            () => aBSpline.SetEvalRepresentation(new Geom2dEval_RepCurveDesc.Full { Representation = CubicBSpline() }));

        Geom2d_BezierCurve aBezier = CubicBezier();
        aBezier.Freeze();
        Assert.Throws<Standard_ImmutableObject>(() => aBezier.SetPole(2, new gp_Pnt2d(9, 9)));
        Assert.Throws<Standard_ImmutableObject>(() => aBezier.Increase(4));
        Assert.Throws<Standard_ImmutableObject>(() => aBezier.Segment(0.2, 0.8));
        Assert.Throws<Standard_ImmutableObject>(() => aBezier.SetWeight(2, 2.0));

        Geom2d_TrimmedCurve aTrim = new Geom2d_TrimmedCurve(CubicBSpline(), 0.5, 2.5);
        aTrim.Freeze();
        Assert.Throws<Standard_ImmutableObject>(() => aTrim.SetTrim(1.0, 2.0));
        Assert.Throws<Standard_ImmutableObject>(() => aTrim.Reverse());

        Geom2d_OffsetCurve anOffset = new Geom2d_OffsetCurve(CubicBSpline(), 0.5);
        anOffset.Freeze();
        Assert.Throws<Standard_ImmutableObject>(() => anOffset.SetOffsetValue(1.0));
        Assert.Throws<Standard_ImmutableObject>(() => anOffset.Transform(Translation(1, 0)));
    }

    [Fact]
    public void Copy_IsUnfrozen()
    {
        Geom2d_Geometry[] aCurves =
        [
            new Geom2d_Line(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)),
            new Geom2d_Circle(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 2.0),
            new Geom2d_Ellipse(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 3.0, 1.0),
            new Geom2d_Hyperbola(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 3.0, 1.0),
            new Geom2d_Parabola(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 2.0),
            CubicBezier(),
            CubicBSpline(),
            new Geom2d_TrimmedCurve(CubicBSpline(), 0.5, 2.5),
            new Geom2d_OffsetCurve(CubicBSpline(), 0.5),
        ];
        foreach (Geom2d_Geometry aCurve in aCurves)
        {
            aCurve.Freeze();
            Geom2d_Geometry aCopy = aCurve.Copy();
            Assert.False(aCopy.IsFrozen());
            Assert.Equal(aCurve.GetType(), aCopy.GetType());
            // The transformed copy is a new object, and the frozen original is unchanged
            Geom2d_Curve aMoved = (Geom2d_Curve)aCurve.Translated(new gp_Vec2d(1, 2));
            Geom2d_Curve anOrig = (Geom2d_Curve)aCurve;
            double aU = Math.Max(anOrig.FirstParameter(), -1.0) + 0.1;
            gp_Pnt2d aP = anOrig.Value(aU);
            gp_Pnt2d aQ = aMoved.Value(aMoved.TransformedParameter(aU, Translation(1, 2)));
            AssertSamePoint(new gp_Pnt2d(aP.X() + 1, aP.Y() + 2), aQ, 1e-12);
        }
    }

    [Fact]
    public void Twins_DontChangeOriginal_Conics()
    {
        Geom2d_Circle aCircle = new Geom2d_Circle(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 2.0);
        Geom2d_Circle aBig = aCircle.WithRadius(5.0);
        Assert.Equal(2.0, aCircle.Radius());
        Assert.Equal(5.0, aBig.Radius());
        Assert.Throws<Standard_ConstructionError>(() => aCircle.WithRadius(-1.0));

        Geom2d_Ellipse anEllipse = new Geom2d_Ellipse(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 3.0, 1.0);
        Assert.Equal(4.0, anEllipse.WithMajorRadius(4.0).MajorRadius());
        Assert.Equal(0.5, anEllipse.WithMinorRadius(0.5).MinorRadius());
        Assert.Equal(3.0, anEllipse.MajorRadius());
        Assert.Equal(1.0, anEllipse.MinorRadius());

        Geom2d_Parabola aParabola = new Geom2d_Parabola(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 2.0);
        Assert.Equal(7.0, aParabola.WithFocal(7.0).Focal());
        Assert.Equal(2.0, aParabola.Focal());

        Geom2d_Line aLine = new Geom2d_Line(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0));
        Geom2d_Line aMoved = aLine.WithLocation(new gp_Pnt2d(3, 4));
        AssertSamePoint(new gp_Pnt2d(0, 0), aLine.Location());
        AssertSamePoint(new gp_Pnt2d(3, 4), aMoved.Location());

        Geom2d_Conic aConic = aCircle.WithLocation(new gp_Pnt2d(1, 1));
        AssertSamePoint(new gp_Pnt2d(1, 1), aConic.Location());
        AssertSamePoint(new gp_Pnt2d(0, 0), aCircle.Location());

        Geom2d_Curve aReversed = aCircle.Reversed();
        Assert.False(aReversed.IsFrozen());
        Assert.True(aCircle.Position().YDirection().IsOpposite(((Geom2d_Circle)aReversed).Position().YDirection(), 1e-12));
        Assert.True(aCircle.Position().YDirection().IsEqual(new gp_Dir2d(0, 1), 1e-12));
    }

    [Fact]
    public void Twins_DontChangeOriginal_BSpline()
    {
        Geom2d_BSplineCurve aCurve = CubicBSpline();
        aCurve.Freeze();
        gp_Pnt2d aMid = aCurve.Value(1.5);

        // KnotsInserted: Add defaults to false, as Geom2d_BSplineCurve::InsertKnots
        Geom2d_BSplineCurve anInserted = aCurve.KnotsInserted(Arr(0.5, 1.0), Arr(1, 2));
        Assert.Equal(4, aCurve.NbKnots());
        Assert.Equal(5, anInserted.NbKnots());
        // Knot 1.0 existed with multiplicity 1: raised TO 2 (not by 2)
        Assert.Equal(2, anInserted.Multiplicity(3));
        AssertSamePoint(aMid, anInserted.Value(1.5), 1e-12);
        Geom2d_BSplineCurve anAdded = aCurve.KnotsInserted(Arr(1.0), Arr(1), 0.0, true);
        Assert.Equal(2, anAdded.Multiplicity(2));

        Geom2d_BSplineCurve aOneInserted = aCurve.KnotInserted(0.5, 2);
        Assert.Equal(5, aOneInserted.NbKnots());
        Assert.Equal(2, aOneInserted.Multiplicity(2));

        // KnotRemoved: possible when the knot was just inserted, null when the curve would change
        Geom2d_BSplineCurve? aRemoved = aOneInserted.KnotRemoved(2, 0, 1e-9);
        Assert.NotNull(aRemoved);
        Assert.Equal(4, aRemoved.NbKnots());
        Assert.Null(aCurve.KnotRemoved(2, 0, 1e-9));
        Assert.Equal(4, aCurve.NbKnots());

        Geom2d_BSplineCurve aSegment = aCurve.Segmented(0.5, 2.5);
        Assert.Equal(0.5, aSegment.FirstParameter(), 1e-12);
        Assert.Equal(2.5, aSegment.LastParameter(), 1e-12);
        Assert.Equal(0.0, aCurve.FirstParameter());
        Assert.Equal(3.0, aCurve.LastParameter());

        Geom2d_BSplineCurve aPeriodic = aCurve.WithPeriodic();
        Assert.True(aPeriodic.IsPeriodic());
        Assert.False(aCurve.IsPeriodic());
        Geom2d_BSplineCurve aNotPeriodic = aPeriodic.WithNotPeriodic();
        Assert.False(aNotPeriodic.IsPeriodic());
        Assert.True(aPeriodic.IsPeriodic());
        AssertSamePoint(aPeriodic.Value(1.3), aNotPeriodic.Value(1.3), 1e-10);

        Geom2d_BSplineCurve aRaised = aCurve.DegreeIncreased(5);
        Assert.Equal(5, aRaised.Degree());
        Assert.Equal(3, aCurve.Degree());
        AssertSamePoint(aMid, aRaised.Value(1.5), 1e-12);

        Geom2d_BSplineCurve aWeighted = aCurve.WithWeight(3, 2.0);
        Assert.True(aWeighted.IsRational());
        Assert.False(aCurve.IsRational());

        Geom2d_BSplineCurve aPoled = aCurve.WithPole(2, new gp_Pnt2d(9, 9));
        AssertSamePoint(new gp_Pnt2d(9, 9), aPoled.Pole(2));
        AssertSamePoint(new gp_Pnt2d(1, 2), aCurve.Pole(2));

        Geom2d_BSplineCurve aMovedPoint = aCurve.PointMoved(1.5, new gp_Pnt2d(2.5, 2.0), 1, aCurve.NbPoles(), out int aFirst, out _);
        Assert.True(aFirst > 0);
        AssertSamePoint(new gp_Pnt2d(2.5, 2.0), aMovedPoint.Value(1.5), 1e-9);
        AssertSamePoint(aMid, aCurve.Value(1.5), 0.0);

        Assert.False(aCurve.Reversed().IsFrozen());
        Assert.True(aCurve.IsFrozen());
    }

    [Fact]
    public void Twins_DontChangeOriginal_Bezier()
    {
        Geom2d_BezierCurve aCurve = CubicBezier();
        aCurve.Freeze();
        gp_Pnt2d aMid = aCurve.Value(0.5);

        Geom2d_BezierCurve anIncreased = aCurve.Increased(5);
        Assert.Equal(5, anIncreased.Degree());
        Assert.Equal(3, aCurve.Degree());
        AssertSamePoint(aMid, anIncreased.Value(0.5), 1e-12);

        Geom2d_BezierCurve aSegment = aCurve.Segmented(0.25, 0.75);
        AssertSamePoint(aMid, aSegment.Value(0.5), 1e-12);
        AssertSamePoint(new gp_Pnt2d(0, 0), aCurve.StartPoint());

        Assert.Equal(5, aCurve.PoleInsertedAfter(2, new gp_Pnt2d(1.5, 2)).NbPoles());
        Assert.Equal(3, aCurve.PoleRemoved(2).NbPoles());
        Assert.Equal(4, aCurve.NbPoles());
        Assert.True(aCurve.WithWeight(2, 3.0).IsRational());
        Assert.False(aCurve.IsRational());
    }

    [Fact]
    public void Twins_TrimmedAndOffset()
    {
        Geom2d_TrimmedCurve aTrim = new Geom2d_TrimmedCurve(CubicBSpline(), 0.5, 2.5);
        Geom2d_TrimmedCurve aNarrow = aTrim.WithTrim(1.0, 2.0);
        Assert.Equal(1.0, aNarrow.FirstParameter());
        Assert.Equal(0.5, aTrim.FirstParameter());

        Geom2d_OffsetCurve anOffset = new Geom2d_OffsetCurve(CubicBSpline(), 0.5);
        Geom2d_OffsetCurve aFar = anOffset.WithOffsetValue(2.0);
        Assert.Equal(2.0, aFar.Offset());
        Assert.Equal(0.5, anOffset.Offset());
    }

    // (2) Decision 702: shared basis curves are frozen; their owner copies on write

    [Fact]
    public void TrimmedCurve_BasisCurve_FrozenAndCopyOnWrite()
    {
        Geom2d_BSplineCurve aCaller = CubicBSpline();
        Geom2d_TrimmedCurve aTrim = new Geom2d_TrimmedCurve(aCaller, 0.5, 2.5);
        // The constructor copies the basis: the caller's curve isn't shared nor frozen
        Assert.False(aCaller.IsFrozen());

        Geom2d_Curve aBasis = aTrim.BasisCurve();
        Assert.True(aBasis.IsFrozen());
        Assert.NotSame(aCaller, aBasis);
        Assert.Same(aBasis, aTrim.BasisCurve());
        gp_Pnt2d aBasisPoint = aBasis.Value(1.0);

        // The trimmed curve is still mutable: it copies its basis first
        aTrim.Transform(Translation(10, 0));
        AssertSamePoint(new gp_Pnt2d(aBasisPoint.X() + 10, aBasisPoint.Y()), aTrim.Value(1.0));
        AssertSamePoint(aBasisPoint, aBasis.Value(1.0), 0.0);
        Assert.NotSame(aBasis, aTrim.BasisCurve());

        Geom2d_Curve aBasis2 = aTrim.BasisCurve();
        gp_Pnt2d aStart = aTrim.StartPoint();
        aTrim.Reverse();
        AssertSamePoint(aStart, aTrim.EndPoint(), 1e-12);
        AssertSamePoint(new gp_Pnt2d(aBasisPoint.X() + 10, aBasisPoint.Y()), aBasis2.Value(1.0));

        // A trimmed curve of a trimmed curve doesn't freeze the inner basis
        Geom2d_TrimmedCurve anInner = new Geom2d_TrimmedCurve(CubicBSpline(), 0.5, 2.5);
        Geom2d_TrimmedCurve anOuter = new Geom2d_TrimmedCurve(anInner, 1.0, 2.0);
        Assert.IsType<Geom2d_BSplineCurve>(anOuter.BasisCurve());
        anInner.Transform(Translation(1, 1)); // would throw if the constructor had frozen its basis
    }

    [Fact]
    public void OffsetCurve_BasisCurve_FrozenAndCopyOnWrite()
    {
        Geom2d_BSplineCurve aCaller = CubicBSpline();
        Geom2d_OffsetCurve anOffset = new Geom2d_OffsetCurve(aCaller, 0.5);

        // As in OCCT the offset curve shares the caller's curve, which is therefore frozen
        Assert.True(aCaller.IsFrozen());
        Assert.Same(aCaller, anOffset.BasisCurve());
        Assert.Throws<Standard_ImmutableObject>(() => aCaller.SetPole(2, new gp_Pnt2d(0, 9)));

        gp_Pnt2d aBasisPoint = aCaller.Value(1.0);
        gp_Pnt2d anOffsetPoint = anOffset.Value(1.0);

        anOffset.Transform(Translation(0, 5));
        AssertSamePoint(new gp_Pnt2d(anOffsetPoint.X(), anOffsetPoint.Y() + 5), anOffset.Value(1.0), 1e-12);
        AssertSamePoint(aBasisPoint, aCaller.Value(1.0), 0.0);
        Assert.NotSame(aCaller, anOffset.BasisCurve());

        Geom2d_Curve aBasis2 = anOffset.BasisCurve();
        Assert.True(aBasis2.IsFrozen());
        anOffset.Reverse();
        Assert.Equal(-0.5, anOffset.Offset());
        AssertSamePoint(new gp_Pnt2d(aBasisPoint.X(), aBasisPoint.Y() + 5), aBasis2.Value(1.0), 0.0);

        // An offset of a trimmed curve rebuilds the trimmed curve (a copy): the caller's isn't frozen
        Geom2d_TrimmedCurve aTrim = new Geom2d_TrimmedCurve(CubicBSpline(), 0.5, 2.5);
        Geom2d_OffsetCurve anOffsetOfTrim = new Geom2d_OffsetCurve(aTrim, 0.5);
        Assert.False(aTrim.IsFrozen());
        Assert.NotSame(aTrim, anOffsetOfTrim.BasisCurve());

        // An offset of an offset adds the offsets and shares the inner basis
        Geom2d_OffsetCurve anOffsetOfOffset = new Geom2d_OffsetCurve(anOffset, 0.25);
        Assert.Equal(-0.25, anOffsetOfOffset.Offset(), 1e-15);
    }

    // (3) Decision 701: returned arrays are copies

    [Fact]
    public void BSpline_ArrayAccessors_AreCopies()
    {
        Geom2d_BSplineCurve aCurve = RationalBSpline();

        Assert.Equal(aCurve.PolesRef().AsSpan().ToArray(), aCurve.Poles().AsSpan().ToArray());
        Assert.Equal(aCurve.KnotsRef().AsSpan().ToArray(), aCurve.Knots().AsSpan().ToArray());
        Assert.Equal(aCurve.MultiplicitiesRef().AsSpan().ToArray(), aCurve.Multiplicities().AsSpan().ToArray());
        Assert.Equal(aCurve.KnotSequenceRef().AsSpan().ToArray(), aCurve.KnotSequence().AsSpan().ToArray());
        PackedList<double>? aWeights = aCurve.Weights();
        Assert.NotNull(aWeights);
        Assert.Equal(aCurve.WeightsRef()!.AsSpan().ToArray(), aWeights.AsSpan().ToArray());
        Assert.Equal(aCurve.WeightsArrayRef().AsSpan().ToArray(), aCurve.WeightsArray().AsSpan().ToArray());

        // A later internal change doesn't reach a list handed out before it
        PackedList<gp_Pnt2d> aPoles = aCurve.Poles();
        PackedList<double> aKnots = aCurve.Knots();
        aCurve.SetPole(2, new gp_Pnt2d(42, 42));
        aCurve.SetKnot(2, 1.25);
        Assert.Equal(new gp_Pnt2d(1, 2), aPoles[1]);
        Assert.Equal(1.0, aKnots[1]);
        Assert.Equal(new gp_Pnt2d(42, 42), aCurve.Poles()[1]);

        // Non-rational: Weights() is null, WeightsArray() is ones
        Geom2d_BSplineCurve aPlain = CubicBSpline();
        Assert.Null(aPlain.Weights());
        Assert.Null(aPlain.WeightsRef());
        Assert.All(aPlain.WeightsArray().AsSpan().ToArray(), w => Assert.Equal(1.0, w));
        Assert.Equal(aPlain.NbPoles(), aPlain.WeightsArray().Count);
    }

    [Fact]
    public void Bezier_ArrayAccessors_AreCopies()
    {
        Geom2d_BezierCurve aCurve = new Geom2d_BezierCurve(Arr(new gp_Pnt2d(0, 0), new gp_Pnt2d(1, 1), new gp_Pnt2d(2, 0)),
                                                           Arr(1.0, 2.0, 1.0));
        Assert.Equal(aCurve.PolesRef().AsSpan().ToArray(), aCurve.Poles().AsSpan().ToArray());
        Assert.Equal(new[] { 0.0, 1.0 }, aCurve.Knots().AsSpan().ToArray());
        Assert.Equal(new[] { 3, 3 }, aCurve.Multiplicities().AsSpan().ToArray());
        Assert.Equal(new[] { 0.0, 0.0, 0.0, 1.0, 1.0, 1.0 }, aCurve.KnotSequence().AsSpan().ToArray());
        Assert.Equal(new[] { 1.0, 2.0, 1.0 }, aCurve.Weights()!.AsSpan().ToArray());
        Assert.Equal(new[] { 1.0, 2.0, 1.0 }, aCurve.WeightsArray().AsSpan().ToArray());

        PackedList<gp_Pnt2d> aPoles = aCurve.Poles();
        aCurve.SetPole(2, new gp_Pnt2d(5, 5));
        Assert.Equal(new gp_Pnt2d(1, 1), aPoles[1]);

        // The shared unit weights are never written: making a non-rational curve rational copies them
        Geom2d_BezierCurve aPlain = CubicBezier();
        Assert.Null(aPlain.Weights());
        aPlain.SetWeight(2, 3.0);
        Assert.Equal(1.0, BSplCLib.UnitWeights(4)[2]);
        Assert.Null(CubicBezier().Weights());
        Assert.Equal(1.0, CubicBezier().WeightsArray()[1]);
    }

    // (4) Reproductions of OCCT issues

    /// <summary>
    /// occt-issues.md, Geom2d_OffsetCurveUtils::CalculateD3: the D2 it returns subtracts the Ndir term that
    /// CalculateD2 adds, and its D3 has wrong DNdir and Ndir coefficients, so EvalD3 disagrees with EvalD2 (and with
    /// the derivative of EvalD2) wherever the basis curve's speed varies. On a circle (constant speed) they agree.
    /// </summary>
    [Fact]
    public void OcctIssue_OffsetCurveEvalD3_WrongWhereBasisSpeedVaries()
    {
        Geom2d_Ellipse anEllipse = new Geom2d_Ellipse(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 3.0, 1.0);
        Geom2d_OffsetCurve anOffset = new Geom2d_OffsetCurve(anEllipse, 0.5);
        double aU = 0.7, h = 1e-4;

        Geom2d_Curve.ResD2 aD2 = anOffset.EvalD2(aU);
        Geom2d_Curve.ResD3 aD3 = anOffset.EvalD3(aU);
        gp_Vec2d aFdD3 = (anOffset.EvalD2(aU + h).D2 - anOffset.EvalD2(aU - h).D2) / (2.0 * h);
        gp_Vec2d aFdD2 = (anOffset.EvalD1(aU + h).D1 - anOffset.EvalD1(aU - h).D1) / (2.0 * h);
        myOutput.WriteLine($"ellipse: EvalD2.D2 = {aD2.D2}, EvalD3.D2 = {aD3.D2}, FD(D1) = {aFdD2}");
        myOutput.WriteLine($"ellipse: EvalD3.D3 = {aD3.D3}, FD(EvalD2.D2) = {aFdD3}");

        // EvalD2's D2 is right (it matches the derivative of D1); point and D1 agree between EvalD2 and EvalD3
        AssertSameVec(aFdD2, aD2.D2, 1e-6);
        AssertSamePoint(aD2.Point, aD3.Point, 1e-15);
        AssertSameVec(aD2.D1, aD3.D1, 1e-15);
        // OCCT: EvalD3's D2 and D3 are wrong
        Assert.True((aD3.D2 - aD2.D2).Magnitude() > 1e-3);
        Assert.True((aD3.D3 - aFdD3).Magnitude() > 1e-3);

        // On a circle they agree
        Geom2d_Circle aCircle = new Geom2d_Circle(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 3.0);
        Geom2d_OffsetCurve aCircleOffset = new Geom2d_OffsetCurve(aCircle, 0.5);
        Geom2d_Curve.ResD2 aCD2 = aCircleOffset.EvalD2(aU);
        Geom2d_Curve.ResD3 aCD3 = aCircleOffset.EvalD3(aU);
        gp_Vec2d aCFdD3 = (aCircleOffset.EvalD2(aU + h).D2 - aCircleOffset.EvalD2(aU - h).D2) / (2.0 * h);
        myOutput.WriteLine($"circle: EvalD2.D2 = {aCD2.D2}, EvalD3.D2 = {aCD3.D2}, EvalD3.D3 = {aCD3.D3}, FD = {aCFdD3}");
        AssertSameVec(aCD2.D2, aCD3.D2, 1e-12);
        AssertSameVec(aCFdD3, aCD3.D3, 1e-6);

        // Adaptor2d_OffsetCurve uses the same formulas
        Adaptor2d_OffsetCurve anAdaptor = new Adaptor2d_OffsetCurve(new TestCurveAdaptor(anEllipse), 0.5);
        anAdaptor.D3(aU, out gp_Pnt2d aAP, out gp_Vec2d aAV1, out gp_Vec2d aAV2, out gp_Vec2d aAV3);
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
        NCollection_Array1<gp_Pnt2d> aPoles = Arr(new gp_Pnt2d(0, 0), new gp_Pnt2d(1, 2), new gp_Pnt2d(2, -1),
                                                  new gp_Pnt2d(3, 2), new gp_Pnt2d(4, 0), new gp_Pnt2d(5, 1),
                                                  new gp_Pnt2d(6, -1), new gp_Pnt2d(7, 0));
        NCollection_Array1<double> aKnots = Arr(0.0, 1.0, 2.0, 3.0, 4.0);
        NCollection_Array1<int> aMults = Arr(4, 2, 2, 2, 2);
        Geom2d_BSplineCurve aCurve = new Geom2d_BSplineCurve(aPoles, aKnots, aMults, 3);
        Assert.Equal(BSplCLib.NbPoles(3, false, aCurve.MultiplicitiesRef()), aCurve.NbPoles());

        aCurve.IncreaseDegree(4);
        int anExpected = BSplCLib.NbPoles(aCurve.Degree(), false, aCurve.MultiplicitiesRef());
        myOutput.WriteLine($"after IncreaseDegree(4): {aCurve.NbPoles()} poles, multiplicities "
                           + $"[{string.Join(", ", aCurve.Multiplicities())}] need {anExpected}");
        // OCCT: the pole count doesn't match the multiplicities
        Assert.NotEqual(anExpected, aCurve.NbPoles());

        // Unclamped at both ends, BSplCLib::IncreaseDegree writes past the new poles (native OCCT, whose
        // NCollection range checks are on, throws Standard_OutOfRange too)
        Geom2d_BSplineCurve anUnclamped = new Geom2d_BSplineCurve(
            Arr(new gp_Pnt2d(0, 0), new gp_Pnt2d(1, 2), new gp_Pnt2d(2, -1), new gp_Pnt2d(3, 2), new gp_Pnt2d(4, 0)),
            Arr(0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0),
            Arr(1, 1, 1, 1, 1, 1, 1, 1, 1),
            3);
        Assert.ThrowsAny<Standard_OutOfRange>(() => anUnclamped.IncreaseDegree(4));
    }

    /// <summary>
    /// occt-issues.md: Geom2d_BSplineCurve never drops an attached evaluation representation, so after Transform
    /// (or SetPole, ...) it still evaluates through the old representation. Geom_BSplineCurve drops it.
    /// </summary>
    [Fact]
    public void OcctIssue_BSplineKeepsEvalRepresentationAfterModification()
    {
        Geom2d_BSplineCurve aCurve = CubicBSpline();
        gp_Pnt2d aBefore = aCurve.Value(1.5);
        aCurve.SetEvalRepresentation(new Geom2dEval_RepCurveDesc.Full { Representation = (Geom2d_Curve)aCurve.Copy() });
        Assert.True(aCurve.HasEvalRepresentation());

        aCurve.Transform(Translation(10, 0));
        Assert.True(aCurve.HasEvalRepresentation());
        // OCCT: still the untransformed point
        AssertSamePoint(aBefore, aCurve.EvalD0(1.5), 0.0);
        // ... although the poles did move
        AssertSamePoint(new gp_Pnt2d(10, 0), aCurve.Pole(1), 0.0);

        aCurve.SetPole(3, new gp_Pnt2d(50, 50));
        Assert.True(aCurve.HasEvalRepresentation());
        AssertSamePoint(aBefore, aCurve.EvalD0(1.5), 0.0);

        // Without the representation the curve evaluates its own (changed) poles
        Geom2d_BSplineCurve aCleared = aCurve.WithoutEvalRepresentation();
        Assert.False(aCleared.HasEvalRepresentation());
        Assert.True(aCleared.EvalD0(1.5).Distance(aBefore) > 1.0);
    }

    /// <summary>
    /// occt-issues.md: Geom2d_BezierCurve drops an attached evaluation representation in Transform (and when its
    /// poles are replaced), but keeps it through SetPole, SetWeight, Reverse and Segment.
    /// </summary>
    [Fact]
    public void OcctIssue_BezierKeepsEvalRepresentationExceptOnTransform()
    {
        Geom2d_BezierCurve aCurve = CubicBezier();
        gp_Pnt2d aBefore = aCurve.Value(0.3);
        aCurve.SetEvalRepresentation(new Geom2dEval_RepCurveDesc.Full { Representation = (Geom2d_Curve)aCurve.Copy() });

        aCurve.SetPole(2, new gp_Pnt2d(1, 5));
        Assert.True(aCurve.HasEvalRepresentation());
        AssertSamePoint(aBefore, aCurve.EvalD0(0.3), 0.0);

        aCurve.Reverse();
        Assert.True(aCurve.HasEvalRepresentation());
        AssertSamePoint(aBefore, aCurve.EvalD0(0.3), 0.0);

        aCurve.Transform(Translation(10, 0));
        Assert.False(aCurve.HasEvalRepresentation());
        Assert.True(aCurve.EvalD0(0.3).X() > 9.0);

        // Increase rebuilds the poles (init), which also drops it
        Geom2d_BezierCurve aSecond = CubicBezier();
        aSecond.SetEvalRepresentation(new Geom2dEval_RepCurveDesc.Full { Representation = (Geom2d_Curve)aSecond.Copy() });
        aSecond.Increase(4);
        Assert.False(aSecond.HasEvalRepresentation());
    }

    [Fact]
    public void EvalRepresentation_Twins_CopyTheDescriptor()
    {
        Geom2d_BSplineCurve aCurve = CubicBSpline();
        Geom2d_BSplineCurve aRep = CubicBSpline().WithPole(3, new gp_Pnt2d(2, 7));
        Geom2dEval_RepCurveDesc.Full aDesc = new Geom2dEval_RepCurveDesc.Full { Representation = aRep };
        Geom2d_BSplineCurve aWith = aCurve.WithEvalRepresentation(aDesc);
        Assert.False(aCurve.HasEvalRepresentation());
        Assert.True(aWith.HasEvalRepresentation());
        AssertSamePoint(aRep.Value(1.2), aWith.EvalD0(1.2), 0.0);
        // Changing the caller's descriptor later doesn't reach the curve
        aDesc.Representation = CubicBSpline();
        AssertSamePoint(aRep.Value(1.2), aWith.EvalD0(1.2), 0.0);
        // A descriptor whose representation is the curve itself is rejected
        Assert.Throws<Standard_ProgramError>(
            () => aCurve.WithEvalRepresentation(new Geom2dEval_RepCurveDesc.Full { Representation = aCurve }));
    }

    // (5) Finite-difference checks

    private static void CheckDerivatives(Geom2d_Curve theCurve, double theU)
    {
        const double h = 1e-5;
        Geom2d_Curve.ResD2 aD2 = theCurve.EvalD2(theU);
        gp_Vec2d aFd1 = new gp_Vec2d(theCurve.EvalD0(theU - h), theCurve.EvalD0(theU + h)) / (2.0 * h);
        gp_Vec2d aFd2 = (theCurve.EvalD1(theU + h).D1 - theCurve.EvalD1(theU - h).D1) / (2.0 * h);
        double aScale1 = Math.Max(1.0, aD2.D1.Magnitude());
        double aScale2 = Math.Max(1.0, aD2.D2.Magnitude());
        Assert.True((aFd1 - aD2.D1).Magnitude() < 1e-6 * aScale1, $"D1 at {theU}: {aD2.D1} vs {aFd1}");
        Assert.True((aFd2 - aD2.D2).Magnitude() < 1e-5 * aScale2, $"D2 at {theU}: {aD2.D2} vs {aFd2}");
        Geom2d_Curve.ResD3 aD3 = theCurve.EvalD3(theU);
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
            gp_Pnt2d[] aPoles = new gp_Pnt2d[aNbPoles];
            double[] aWeights = new double[aNbPoles];
            for (int i = 0; i < aNbPoles; ++i)
            {
                aPoles[i] = new gp_Pnt2d(aRandom.NextDouble() * 10 - 5, aRandom.NextDouble() * 10 - 5);
                aWeights[i] = 0.5 + aRandom.NextDouble() * 2;
            }

            Geom2d_BezierCurve aBezier = new Geom2d_BezierCurve(Arr(aPoles), Arr(aWeights));
            Geom2d_BezierCurve aPlainBezier = new Geom2d_BezierCurve(Arr(aPoles));

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
            Geom2d_BSplineCurve aBSpline = new Geom2d_BSplineCurve(Arr(aPoles), Arr(aWeights), Arr(aKnots), Arr(aMults), aDeg);

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
    private sealed class TestCurveAdaptor : Adaptor2d_Curve2d
    {
        private readonly Geom2d_Curve myCurve;

        public TestCurveAdaptor(Geom2d_Curve theCurve) => myCurve = theCurve;

        public override double FirstParameter() => myCurve.FirstParameter();

        public override double LastParameter() => myCurve.LastParameter();

        public override gp_Pnt2d Value(double U) => myCurve.Value(U);

        public override void D0(double U, out gp_Pnt2d P) => myCurve.D0(U, out P);

        public override void D1(double U, out gp_Pnt2d P, out gp_Vec2d V) => myCurve.D1(U, out P, out V);

        public override void D2(double U, out gp_Pnt2d P, out gp_Vec2d V1, out gp_Vec2d V2) =>
            myCurve.D2(U, out P, out V1, out V2);

        public override void D3(double U, out gp_Pnt2d P, out gp_Vec2d V1, out gp_Vec2d V2, out gp_Vec2d V3) =>
            myCurve.D3(U, out P, out V1, out V2, out V3);

        public override gp_Vec2d DN(double U, int N) => myCurve.DN(U, N);
    }
}
