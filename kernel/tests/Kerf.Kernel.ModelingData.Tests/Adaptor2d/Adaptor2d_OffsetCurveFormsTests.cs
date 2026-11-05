// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// Adaptor2d_OffsetCurve on curves beyond the line (unit 11I): the offset's values against the known offset of a circle,
/// its derivatives against differences of its values, and what it says of a basis's kind, continuity and intervals.
/// </summary>
public class Adaptor2d_OffsetCurveFormsTests
{
    private const double Tol = 1e-9;

    private static readonly Frame2D XY = new(new Point2D(0, 0), new Direction2D(1, 0), new Direction2D(0, 1));

    private static CurveAdaptor2D Circle5() => new(new CircleCurve2D(new Circle2D(XY, 5)));

    private static BSplineCurve2D TwoSpans()
        => new([new Point2D(0, 0), new Point2D(1, 2), new Point2D(3, 2), new Point2D(4, 0)],
               [0.0, 1.0, 2.0], [3, 1, 3], 2);

    private static void Near(Point2D theExpected, Point2D theActual)
        => Assert.True(theExpected.IsEqual(theActual, Tol), $"expected {theExpected.X} {theExpected.Y}, got {theActual.X} {theActual.Y}");

    [Fact]
    public void The_offset_of_a_circle_is_a_circle_of_the_radius_moved_outwards()
    {
        OffsetCurveAdaptor2D anOffset = new(Circle5(), 2);
        Assert.Equal(CurveKind.Circle, anOffset.GetType());
        Near(new Point2D(7, 0), anOffset.Value(0));
        anOffset.D0(Math.PI / 2, out Point2D aTop);
        Near(new Point2D(0, 7), aTop);
        Assert.Equal(7.0, anOffset.Circle().Radius, Tol);
        Assert.True(anOffset.IsClosed);
        Assert.True(anOffset.IsPeriodic);
        Assert.Equal(2 * Math.PI, anOffset.Period(), Tol);
        Assert.False(anOffset.IsRational);
    }

    [Fact]
    public void An_offset_past_the_centre_turns_the_circle_inside_out_and_to_it_leaves_no_circle()
    {
        // 5 - 7: the point at u = 0 has crossed to (-2, 0), and the circle's frame turns to keep it at u = 0
        OffsetCurveAdaptor2D aPast = new(Circle5(), -7);
        Circle2D aCircle = aPast.Circle();
        Assert.Equal(2.0, aCircle.Radius, Tol);
        Near(aPast.Value(0), new CircleCurve2D(aCircle).Value(0));
        EngineAssert.Throws<NoSuchObjectException>(() => new OffsetCurveAdaptor2D(Circle5(), -5).Circle());

        // a zero offset is the basis itself
        Assert.Equal(5.0, new OffsetCurveAdaptor2D(Circle5(), 0).Circle().Radius);
    }

    [Theory]
    [InlineData(0.3)]
    [InlineData(2.0)]
    public void The_derivatives_of_an_offset_ellipse_are_differences_of_its_values(double theU)
    {
        OffsetCurveAdaptor2D anOffset = new(new CurveAdaptor2D(new EllipseCurve2D(new Ellipse2D(XY, 5, 3))), 1);
        Assert.Equal(CurveKind.OffsetCurve, anOffset.GetType());
        const double H = 1e-4;

        anOffset.D1(theU, out Point2D aP, out Vector2D aV1);
        Near(anOffset.Value(theU), aP);
        Vector2D aDiff1 = new(anOffset.Value(theU - H), anOffset.Value(theU + H));
        Assert.True((aDiff1 / (2 * H)).IsEqual(aV1, 1e-5, 1e-3));

        anOffset.D2(theU, out aP, out Vector2D aV1b, out Vector2D aV2);
        Assert.True(aV1.IsEqual(aV1b, 1e-12, 1e-9));
        anOffset.D1(theU - H, out _, out Vector2D aBefore);
        anOffset.D1(theU + H, out _, out Vector2D anAfter);
        Vector2D aDiff2 = (anAfter - aBefore) / (2 * H);
        Assert.Equal(aDiff2.X, aV2.X, 1e-4);
        Assert.Equal(aDiff2.Y, aV2.Y, 1e-4);

        anOffset.D3(theU, out aP, out _, out Vector2D aV2b, out Vector2D aV3);
        Assert.Equal(aV2.X, aV2b.X, 1e-9);
        anOffset.D2(theU - H, out _, out _, out Vector2D aBefore2);
        anOffset.D2(theU + H, out _, out _, out Vector2D anAfter2);
        Vector2D aDiff3 = (anAfter2 - aBefore2) / (2 * H);
        Assert.Equal(aDiff3.X, aV3.X, 1e-3);
        Assert.Equal(aDiff3.Y, aV3.Y, 1e-3);

        Assert.True(anOffset.Resolution(1) > 0);
        EngineAssert.Throws<MethodNotImplementedException>(() => anOffset.DN(theU, 1));
    }

    [Fact]
    public void What_a_nonzero_offset_leaves_of_the_bases_conic()
    {
        OffsetCurveAdaptor2D anEllipse = new(new CurveAdaptor2D(new EllipseCurve2D(new Ellipse2D(XY, 5, 3))), 1);
        EngineAssert.Throws<NoSuchObjectException>(() => anEllipse.Ellipse());
        EngineAssert.Throws<NoSuchObjectException>(() => anEllipse.Line());
        EngineAssert.Throws<NoSuchObjectException>(() => anEllipse.Circle());
        EngineAssert.Throws<NoSuchObjectException>(() => anEllipse.Hyperbola());
        EngineAssert.Throws<NoSuchObjectException>(() => anEllipse.Parabola());
        EngineAssert.Throws<NoSuchObjectException>(() => anEllipse.Degree());
        EngineAssert.Throws<NoSuchObjectException>(() => _ = anEllipse.PoleCount);
        EngineAssert.Throws<NoSuchObjectException>(() => _ = anEllipse.KnotCount);
        EngineAssert.Throws<NoSuchObjectException>(() => anEllipse.Bezier());
        EngineAssert.Throws<NoSuchObjectException>(() => anEllipse.BSpline());
    }

    [Fact]
    public void A_zero_offset_passes_the_bases_kind_through()
    {
        Assert.Equal(3.0, new OffsetCurveAdaptor2D(new CurveAdaptor2D(new EllipseCurve2D(new Ellipse2D(XY, 5, 3))), 0).Ellipse().MinorRadius);
        Assert.Equal(3.0, new OffsetCurveAdaptor2D(new CurveAdaptor2D(new HyperbolaCurve2D(new Hyperbola2D(XY, 5, 3)), -1, 1), 0).Hyperbola().MinorRadius);
        Assert.Equal(2.0, new OffsetCurveAdaptor2D(new CurveAdaptor2D(new ParabolaCurve2D(new Parabola2D(XY, 2)), -1, 1), 0).Parabola().Focal);

        BSplineCurve2D aSpline = TwoSpans();
        OffsetCurveAdaptor2D aZero = new(new CurveAdaptor2D(aSpline), 0);
        Assert.Equal(2, aZero.Degree());
        Assert.Equal(4, aZero.PoleCount);
        Assert.Equal(3, aZero.KnotCount);
        Assert.Same(aSpline, aZero.BSpline());
        Assert.False(aZero.IsRational);

        BezierCurve2D aBezier = new([new Point2D(0, 0), new Point2D(1, 2), new Point2D(2, 0)]);
        OffsetCurveAdaptor2D aZeroBezier = new(new CurveAdaptor2D(aBezier), 0);
        Assert.Same(aBezier, aZeroBezier.Bezier());
        Assert.True(aZero.SampleCount > 0);
    }

    [Theory]
    [InlineData(Continuity.CN, Continuity.CN)]
    [InlineData(Continuity.C3, Continuity.C2)]
    [InlineData(Continuity.C2, Continuity.G2)]
    [InlineData(Continuity.G2, Continuity.C1)]
    [InlineData(Continuity.C1, Continuity.G1)]
    [InlineData(Continuity.G1, Continuity.C0)]
    public void An_offset_loses_a_step_of_continuity(Continuity theBasis, Continuity theOffset)
        => Assert.Equal(theOffset, new OffsetCurveAdaptor2D(new BasisOfContinuity(theBasis), 1, 0, 1).Continuity());

    [Fact]
    public void A_C0_basis_has_no_offset_continuity_and_isnt_closed()
    {
        OffsetCurveAdaptor2D anOffset = new(new BasisOfContinuity(Continuity.C0), 1, 0, 1);
        EngineAssert.Throws<TypeMismatchException>(() => anOffset.Continuity());
        Assert.False(anOffset.IsClosed);
    }

    [Fact]
    public void The_intervals_of_an_offset_are_of_two_more_steps_on_the_basis()
    {
        // the basis is C1 in the middle: enough for an offset C0 (two steps, GeomAbs_C1), not C1 (GeomAbs_C2)
        OffsetCurveAdaptor2D anOffset = new(new CurveAdaptor2D(TwoSpans()), 0.1, 0, 2);
        Assert.Equal(1, anOffset.NbIntervals(Continuity.C0));
        Assert.Equal(2, anOffset.NbIntervals(Continuity.C1));
        NCollection_Array1<double> aKnots = new(1, 3);
        anOffset.Intervals(aKnots, Continuity.C1);
        Assert.Equal((0.0, 1.0, 2.0), (aKnots[1], aKnots[2], aKnots[3]));

        // trimmed to [0.5, 1.5], the interval's ends are the offset's
        OffsetCurveAdaptor2D aTrimmed = new(new CurveAdaptor2D(TwoSpans()), 0.1, 0.5, 1.5);
        Assert.Equal(2, aTrimmed.NbIntervals(Continuity.C1));
        aTrimmed.Intervals(aKnots, Continuity.C1);
        Assert.Equal((0.5, 1.0, 1.5), (aKnots[1], aKnots[2], aKnots[3]));

        // a single interval
        OffsetCurveAdaptor2D aSmooth = new(Circle5(), 1);
        Assert.Equal(1, aSmooth.NbIntervals(Continuity.C2));
        NCollection_Array1<double> aTwo = new(1, 2);
        aSmooth.Intervals(aTwo, Continuity.C2);
        Assert.Equal((aSmooth.FirstParameter(), aSmooth.LastParameter()), (aTwo[1], aTwo[2]));
    }

    [Fact]
    public void A_closed_basis_turning_at_its_seam_has_an_open_offset()
    {
        // a closed polyline-like spline with a kink where it closes
        BSplineCurve2D aClosed = new([new Point2D(0, 0), new Point2D(2, 3), new Point2D(4, 0), new Point2D(0, 0)], [0.0, 1.0], [4, 4], 3);
        Assert.False(new OffsetCurveAdaptor2D(new CurveAdaptor2D(aClosed), 1).IsClosed);
        Assert.False(new OffsetCurveAdaptor2D(new CurveAdaptor2D(new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0)), 0, 1), 1).IsClosed);
        Assert.True(new OffsetCurveAdaptor2D(new CurveAdaptor2D(aClosed), 0).IsClosed);
    }

    /// <summary>Reports a chosen continuity; Adaptor2d_Curve2d's defaults answer the rest.</summary>
    private sealed class BasisOfContinuity(Continuity theContinuity) : Curve2DAdaptorBase
    {
        public override Continuity Continuity() => theContinuity;

        public override bool IsClosed => true;
    }

    [Fact]
    public void A_curve_adaptor_that_overrides_nothing_says_so_for_each_member()
    {
        Curve2DAdaptorBase aBase = new Bare();
        void Refused(Action theAction) => EngineAssert.Throws<MethodNotImplementedException>(theAction);
        Refused(() => aBase.ShallowCopy());
        Refused(() => aBase.FirstParameter());
        Refused(() => aBase.LastParameter());
        Refused(() => aBase.Continuity());
        Refused(() => aBase.NbIntervals(Continuity.C0));
        Refused(() => aBase.Intervals(new NCollection_Array1<double>(1, 2), Continuity.C0));
        Refused(() => aBase.Trim(0, 1, 0));
        Refused(() => _ = aBase.IsClosed);
        Refused(() => _ = aBase.IsPeriodic);
        Refused(() => aBase.Period());
        Refused(() => aBase.Value(0));
        Refused(() => aBase.D0(0, out _));
        Refused(() => aBase.D1(0, out _, out _));
        Refused(() => aBase.D2(0, out _, out _, out _));
        Refused(() => aBase.D3(0, out _, out _, out _, out _));
        Refused(() => aBase.DN(0, 1));
        Refused(() => aBase.Resolution(1));
        Refused(() => aBase.GetType());
        Refused(() => aBase.Line());
        Refused(() => aBase.Circle());
        Refused(() => aBase.Ellipse());
        Refused(() => aBase.Hyperbola());
        Refused(() => aBase.Parabola());
        Refused(() => aBase.Degree());
        Refused(() => _ = aBase.IsRational);
        Refused(() => _ = aBase.PoleCount);
        Refused(() => _ = aBase.KnotCount);
        Refused(() => aBase.Bezier());
        Refused(() => aBase.BSpline());
        Assert.Equal(20, aBase.SampleCount);
    }

    private sealed class Bare : Curve2DAdaptorBase;

    [Fact]
    public void A_line_adaptor_has_no_other_kind()
    {
        Line2d aLine = new(new Point2D(0, 0), new Direction2D(1, 0), 0, 1);
        Assert.Equal(new Direction2D(1, 0), aLine.Line().Direction);
        EngineAssert.Throws<NoSuchObjectException>(() => aLine.Circle());
        EngineAssert.Throws<NoSuchObjectException>(() => aLine.Ellipse());
        EngineAssert.Throws<NoSuchObjectException>(() => aLine.Hyperbola());
        EngineAssert.Throws<NoSuchObjectException>(() => aLine.Parabola());
        EngineAssert.Throws<NoSuchObjectException>(() => aLine.Degree());
        EngineAssert.Throws<NoSuchObjectException>(() => _ = aLine.IsRational);
        EngineAssert.Throws<NoSuchObjectException>(() => _ = aLine.PoleCount);
        EngineAssert.Throws<NoSuchObjectException>(() => _ = aLine.KnotCount);
        EngineAssert.Throws<NoSuchObjectException>(() => aLine.Bezier());
        EngineAssert.Throws<NoSuchObjectException>(() => aLine.BSpline());
    }
}
