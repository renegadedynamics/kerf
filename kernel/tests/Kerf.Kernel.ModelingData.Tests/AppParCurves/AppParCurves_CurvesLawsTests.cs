// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Engine.Approximation.Constraint;

namespace Kerf.Kernel.ModelingData.Tests;

using Gradient = global::Kerf.Engine.Approximation.Gradient<AppParCurves_TestLine, AppParCurves_TestLineTool>;

/// <summary>
/// The approximation's curves by their laws (unit 12Q, decision 1753): a cubic B-spline's second derivative is the
/// exact central difference of its first, which is quadratic on a span, and its first the difference of its points; each
/// curve is asked for in its own dimension only; a constraint is its point and its kind.
/// </summary>
public class AppParCurves_CurvesLawsTests
{
    // A cubic B-spline MultiCurve with an interior knot at 0.4: 5 poles, one 3d curve and one 2d curve
    private static MultiBSpCurve CubicBSpline()
    {
        Point3D[] aP3d = [new(0, 0, 0), new(1, 2, 0.5), new(2, 2.5, -1), new(3, 0, 1), new(4, 1, 0)];
        Point2D[] aP2d = [new(0, 1), new(1, 3), new(2, -1), new(3, 0.5), new(5, 1)];
        NCollection_Array1<MultiPoint> aPoles = new(1, 5);
        for (int i = 1; i <= 5; i++)
        {
            MultiPoint aMP = new(1, 1);
            aMP.SetPoint(1, aP3d[i - 1]);
            aMP.SetPoint2d(2, aP2d[i - 1]);
            aPoles[i] = aMP;
        }
        return new MultiBSpCurve(aPoles, NCollection_Array1<double>.From([0.0, 0.4, 1.0]), NCollection_Array1<int>.From([4, 1, 4]));
    }

    [Theory]
    [InlineData(0.15)]
    [InlineData(0.3)]
    [InlineData(0.55)]
    [InlineData(0.9)]
    public void A_cubic_bspline_s_derivatives_are_the_differences_of_its_lower_ones(double u)
    {
        MultiBSpCurve aCurve = CubicBSpline();
        Assert.Equal(3, aCurve.Degree());
        const double h = 1.0e-3;
        // 3d: D1 is quadratic on the span, so its central difference is D2 exactly; the points' five-point difference,
        // exact to the fourth degree, is D1
        aCurve.D2(1, u, out Point3D aP, out Vector3D aV1, out Vector3D aV2);
        aCurve.D1(1, u + h, out Point3D _, out Vector3D aV1Plus);
        aCurve.D1(1, u - h, out Point3D _, out Vector3D aV1Minus);
        aCurve.Value(1, u + h, out Point3D aPPlus);
        aCurve.Value(1, u - h, out Point3D aPMinus);
        aCurve.Value(1, u, out Point3D aPAt);
        Assert.Equal(0, aP.DistanceTo(aPAt), 14);
        Assert.Equal(0, (aV2 - (aV1Plus - aV1Minus) / (2 * h)).Magnitude, 7);
        aCurve.Value(1, u + 2 * h, out Point3D aPPlus2);
        aCurve.Value(1, u - 2 * h, out Point3D aPMinus2);
        Assert.Equal(0, (aV1 - (8 * new Vector3D(aPMinus, aPPlus) - new Vector3D(aPMinus2, aPPlus2)) / (12 * h)).Magnitude, 7);

        aCurve.D2(2, u, out Point2D aQ, out Vector2D aW1, out Vector2D aW2);
        aCurve.D1(2, u + h, out Point2D _, out Vector2D aW1Plus);
        aCurve.D1(2, u - h, out Point2D _, out Vector2D aW1Minus);
        aCurve.Value(2, u + h, out Point2D aQPlus);
        aCurve.Value(2, u - h, out Point2D aQMinus);
        aCurve.Value(2, u, out Point2D aQAt);
        Assert.Equal(0, aQ.DistanceTo(aQAt), 14);
        Assert.Equal(0, (aW2 - (aW1Plus - aW1Minus) / (2 * h)).Magnitude, 7);
        aCurve.Value(2, u + 2 * h, out Point2D aQPlus2);
        aCurve.Value(2, u - 2 * h, out Point2D aQMinus2);
        Assert.Equal(0, (aW1 - (8 * new Vector2D(aQMinus, aQPlus) - new Vector2D(aQMinus2, aQPlus2)) / (12 * h)).Magnitude, 7);
    }

    [Fact]
    public void Each_curve_is_asked_for_in_its_own_dimension_only()
    {
        // curve 1 is 3d and curve 2 is 2d: the other dimension's evaluators, and an index past them, are refused
        MultiBSpCurve aBSpline = CubicBSpline();
        Assert.Throws<OutOfRangeException>(() => aBSpline.Value(2, 0.5, out Point3D _));
        Assert.Throws<OutOfRangeException>(() => aBSpline.D1(2, 0.5, out Point3D _, out Vector3D _));
        Assert.Throws<OutOfRangeException>(() => aBSpline.D2(2, 0.5, out Point3D _, out Vector3D _, out Vector3D _));
        Assert.Throws<OutOfRangeException>(() => aBSpline.Value(1, 0.5, out Point2D _));
        Assert.Throws<OutOfRangeException>(() => aBSpline.D1(1, 0.5, out Point2D _, out Vector2D _));
        Assert.Throws<OutOfRangeException>(() => aBSpline.D2(1, 0.5, out Point2D _, out Vector2D _, out Vector2D _));
        Assert.Throws<OutOfRangeException>(() => aBSpline.Dimension(3));
        Assert.Throws<OutOfRangeException>(() => aBSpline.Dimension(0));

        // the same of a Bezier MultiCurve, made pole by pole
        MultiCurve aBezier = new(3);
        Assert.Equal(3, aBezier.NbPoles());
        for (int i = 1; i <= 3; i++)
        {
            MultiPoint aMP = new(1, 1);
            aMP.SetPoint(1, new Point3D(i, i * i, 0));
            aMP.SetPoint2d(2, new Point2D(i, -i));
            aBezier.SetValue(i, aMP);
        }
        Assert.Equal(2, aBezier.NbCurves());
        Assert.Equal(3, aBezier.Dimension(1));
        Assert.Equal(2, aBezier.Dimension(2));
        Assert.Throws<OutOfRangeException>(() => aBezier.Value(2, 0.5, out Point3D _));
        Assert.Throws<OutOfRangeException>(() => aBezier.D1(2, 0.5, out Point3D _, out Vector3D _));
        Assert.Throws<OutOfRangeException>(() => aBezier.D2(2, 0.5, out Point3D _, out Vector3D _, out Vector3D _));
        Assert.Throws<OutOfRangeException>(() => aBezier.Value(1, 0.5, out Point2D _));
        Assert.Throws<OutOfRangeException>(() => aBezier.D1(1, 0.5, out Point2D _, out Vector2D _));
        Assert.Throws<OutOfRangeException>(() => aBezier.D2(1, 0.5, out Point2D _, out Vector2D _, out Vector2D _));
        Assert.Throws<OutOfRangeException>(() => aBezier.Transform(2, 0, 1, 0, 1, 0, 1));
        Assert.Throws<OutOfRangeException>(() => aBezier.Transform2d(1, 0, 1, 0, 1));
        Assert.Throws<OutOfRangeException>(() => aBezier.Curve(0, new NCollection_Array1<Point3D>(1, 3)));
        Assert.Throws<OutOfRangeException>(() => aBezier.Curve(0, new NCollection_Array1<Point2D>(1, 3)));
        Assert.Throws<OutOfRangeException>(() => aBezier.Pole(0, 0));
        Assert.Throws<OutOfRangeException>(() => aBezier.Pole2d(0, 0));
        Assert.Throws<OutOfRangeException>(() => aBezier.Value(4));
        Assert.Throws<OutOfRangeException>(() => aBezier.SetValue(0, new MultiPoint(1, 1)));
        // a Bezier of 3 poles is a parabola: its middle is the average of a quarter of each end and half the middle pole
        aBezier.Value(1, 0.5, out Point3D aMid);
        Assert.Equal(0, aMid.DistanceTo(new Point3D(2, (1 + 2 * 4 + 9) / 4.0, 0)), 14);
        Assert.Equal(0, aBezier.Pole(1, 3).DistanceTo(new Point3D(3, 9, 0)), 15);

        // new, empty poles; and with none, no poles
        aBezier.SetNbPoles(5);
        Assert.Equal(5, aBezier.NbPoles());
        Assert.Equal(0, aBezier.Value(5).PointCount);
        Assert.Equal(0, new MultiCurve().NbPoles());
        Assert.Equal(0, new MultiCurve().NbCurves());
    }

    [Fact]
    public void A_constraint_is_its_point_and_its_kind()
    {
        var aCouple = new ConstraintCouple(3, TangencyPoint);
        Assert.True(aCouple == new ConstraintCouple(3, TangencyPoint));
        Assert.True(aCouple.Equals((object)new ConstraintCouple(3, TangencyPoint)));
        Assert.Equal(aCouple.GetHashCode(), new ConstraintCouple(3, TangencyPoint).GetHashCode());
        Assert.True(aCouple != aCouple.WithIndex(4));
        Assert.True(aCouple != aCouple.WithConstraint(PassPoint));
        Assert.Equal(PassPoint, aCouple.WithConstraint(PassPoint).Constraint());
        Assert.Equal("AppParCurves_ConstraintCouple(3, TangencyPoint)", aCouple.ToString());
    }

    [Fact]
    public void A_parabola_in_the_plane_is_fitted_by_a_parabola_whatever_parameters_it_starts_from()
    {
        // points of y = x² on x in [-1, 1], 2d only, at parameters off the true ones: the optimiser moves them until
        // the three-pole Bezier it fits is the parabola, so every point of the curve is on it, and it passes the vertex
        const int n = 15;
        AppParCurves_TestLine aLine = new(n, 0, 1);
        math_Vector U = new(1, n);
        for (int i = 0; i < n; i++)
        {
            double t = (double)i / (n - 1);
            aLine.P2d[i][0] = new Point2D(2 * t - 1, (2 * t - 1) * (2 * t - 1));
            U[i + 1] = t + (i == 0 || i == n - 1 ? 0 : 0.03 * Math.Sin(Math.PI * t));
        }
        NCollection_HArray1<ConstraintCouple> aConstraints = new(1, 3);
        aConstraints[1] = new ConstraintCouple(1, PassPoint);
        aConstraints[2] = new ConstraintCouple(8, PassPoint); // the vertex, (0, 0), held as the parameters move
        aConstraints[3] = new ConstraintCouple(n, PassPoint);
        Gradient aGrad = new(aLine, 1, n, aConstraints, U, 3, 1.0e-6, 1.0e-6, 200);
        Assert.True(aGrad.MaxError2d() < 1.0e-4, $"MaxError2d {aGrad.MaxError2d()}");
        MultiCurve aCurve = aGrad.Value();
        Assert.Equal(1, aCurve.NbCurves());
        for (int k = 0; k <= 10; k++)
        {
            aCurve.Value(1, k / 10.0, out Point2D aP);
            Assert.Equal(aP.X * aP.X, aP.Y, 3);
        }
        aCurve.Value(1, U[8], out Point2D aVertex);
        Assert.Equal(0, aVertex.DistanceTo(new Point2D(0, 0)), 6);
        aCurve.Value(1, 0.0, out Point2D aStart);
        Assert.Equal(0, aStart.DistanceTo(new Point2D(-1, 1)), 9);
    }

    [Fact]
    public void An_ellipse_in_the_plane_approximated_from_chord_lengths_stays_on_the_ellipse()
    {
        // 2d only, through AppDef's line: chord lengths aren't the ellipse's parameters, so the approximation moves them
        // (the gradient over 2d points); every point of every Bezier is on the ellipse to the tolerance
        NCollection_Array1<Point2D> aPnts = new(1, 30);
        for (int i = 1; i <= 30; i++)
        {
            double t = 2.0 * (i - 1) / 29;
            aPnts[i] = new Point2D(3.0 * Math.Cos(t), 2.0 * Math.Sin(t));
        }
        Compute aComputer = new(new MultiLine(aPnts), 3, 6, 1.0e-6, 1.0e-6, 5, true, global::Kerf.Geometry.ParametrizationType.ChordLength, false);
        Assert.True(aComputer.IsAllApproximated);
        for (int c = 1; c <= aComputer.MultiCurveCount; c++)
        {
            aComputer.Error(c, out double aTol3d, out double aTol2d);
            Assert.Equal(0.0, aTol3d);
            Assert.True(aTol2d <= 1.0e-6, $"{aTol2d}");
            MultiCurve aCurve = aComputer.Value(c);
            for (int k = 0; k <= 20; k++)
            {
                aCurve.Value(1, k / 20.0, out Point2D aP);
                Assert.Equal(1.0, aP.X * aP.X / 9 + aP.Y * aP.Y / 4, 4);
            }
        }
    }
}
