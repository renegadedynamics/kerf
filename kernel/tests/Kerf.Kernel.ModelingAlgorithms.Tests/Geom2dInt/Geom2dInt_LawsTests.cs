// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The 2D intersector of a conic with a parametric curve by what an intersection is (unit 12Q, decision 1753): a point on
/// both, at the parameters it gives on each, where the two equations meet. The curve is the Bezier of y = x^2 on
/// [-2, 2] (x = 4t - 2), so each conic's crossings solve a polynomial in x.
/// </summary>
public class Geom2dInt_LawsTests
{
    private const double Tol = 1e-7;

    private static CurveAdaptor2D Bowl()
    {
        var aPoles = new NCollection_Array1<Point2D>(1, 3);
        aPoles[1] = new Point2D(-2, 4);
        aPoles[2] = new Point2D(0, -4);
        aPoles[3] = new Point2D(2, 4);
        return new CurveAdaptor2D(new BezierCurve2D(aPoles));
    }

    private static TwoDDomain Domain(Func<double, Point2D> theValue, double theFirst, double theLast) =>
        new(theValue(theFirst), theFirst, Tol, theValue(theLast), theLast, Tol);

    private static TwoDDomain BowlDomain(CurveAdaptor2D theBowl) => Domain(theBowl.Value, 0, 1);

    // The crossings found, each on both curves at its parameters, by x
    private static double[] Crossings(Intersection2D theInter, Func<double, Point2D> theConic, CurveAdaptor2D theBowl)
    {
        Assert.True(theInter.IsDone);
        var aXs = new List<double>();
        for (int i = 1; i <= theInter.PointCount; i++)
        {
            IntersectionPoint2D aP = theInter.Point(i);
            Assert.True(aP.Value().DistanceTo(theConic(aP.ParamOnFirst())) < 1e-6, "not on the conic at its parameter");
            Assert.True(aP.Value().DistanceTo(theBowl.Value(aP.ParamOnSecond())) < 1e-6, "not on the curve at its parameter");
            // on y = x^2
            Assert.Equal(aP.Value().X * aP.Value().X, aP.Value().Y, 6);
            aXs.Add(aP.Value().X);
        }
        aXs.Sort();
        return [.. aXs];
    }

    private static void AssertPair(double[] theXs, double theX)
    {
        Assert.Equal(2, theXs.Length);
        Assert.Equal(-theX, theXs[0], 6);
        Assert.Equal(theX, theXs[1], 6);
    }

    [Fact]
    public void A_line_across_the_bowl_meets_it_where_x_squared_is_its_height()
    {
        var aLine = new Line2D(new Point2D(-5, 1), new Direction2D(1, 0));
        CurveAdaptor2D aBowl = Bowl();
        var anInter = new IntConicCurveOfGInter(aLine, Domain(u => ElCLib.Value(u, aLine), 0, 10), aBowl, BowlDomain(aBowl), Tol, Tol);
        AssertPair(Crossings(anInter, u => ElCLib.Value(u, aLine), aBowl), 1);
    }

    [Fact]
    public void A_circle_meets_the_bowl_where_x_squared_plus_x_to_the_fourth_is_one()
    {
        var aCircle = new Circle2D(new Frame2D(new Point2D(0, 0), new Direction2D(1, 0)), 1);
        CurveAdaptor2D aBowl = Bowl();
        var anInter = new IntConicCurveOfGInter(aCircle, Domain(u => ElCLib.Value(u, aCircle), 0, Math.PI), aBowl, BowlDomain(aBowl), Tol, Tol);
        AssertPair(Crossings(anInter, u => ElCLib.Value(u, aCircle), aBowl), Math.Sqrt((Math.Sqrt(5) - 1) / 2));
    }

    [Fact]
    public void An_ellipse_meets_the_bowl_where_its_equation_does()
    {
        // x^2 / 4 + y^2 = 1 with y = x^2: x^4 + x^2 / 4 - 1 = 0
        var anEllipse = new Ellipse2D(new Frame2D(new Point2D(0, 0), new Direction2D(1, 0)), 2, 1);
        CurveAdaptor2D aBowl = Bowl();
        var anInter = new IntConicCurveOfGInter(anEllipse, Domain(u => ElCLib.Value(u, anEllipse), 0, Math.PI), aBowl, BowlDomain(aBowl), Tol, Tol);
        AssertPair(Crossings(anInter, u => ElCLib.Value(u, anEllipse), aBowl), Math.Sqrt((-0.25 + Math.Sqrt(0.0625 + 4)) / 2));
    }

    [Fact]
    public void A_parabola_opening_down_meets_the_bowl_at_its_half_height()
    {
        // y = 2 - x^2: its vertex at (0, 2), opening down, focal 1/4
        var aParab = new Parabola2D(new Axis2D(new Point2D(0, 2), new Direction2D(0, -1)), 0.25);
        CurveAdaptor2D aBowl = Bowl();
        var anInter = new IntConicCurveOfGInter(aParab, Domain(u => ElCLib.Value(u, aParab), -3, 3), aBowl, BowlDomain(aBowl), Tol, Tol);
        AssertPair(Crossings(anInter, u => ElCLib.Value(u, aParab), aBowl), 1);
    }

    [Fact]
    public void A_hyperbola_above_meets_the_bowl_at_the_golden_ratio()
    {
        // y^2 - x^2 = 1, its upper branch: x^4 = 1 + x^2, so x^2 is the golden ratio
        var aHypr = new Hyperbola2D(new Axis2D(new Point2D(0, 0), new Direction2D(0, 1)), 1, 1);
        CurveAdaptor2D aBowl = Bowl();
        var anInter = new IntConicCurveOfGInter(aHypr, Domain(u => ElCLib.Value(u, aHypr), -3, 3), aBowl, BowlDomain(aBowl), Tol, Tol);
        AssertPair(Crossings(anInter, u => ElCLib.Value(u, aHypr), aBowl), Math.Sqrt((1 + Math.Sqrt(5)) / 2));
    }

    [Fact]
    public void Two_curves_meet_alike_with_a_domain_on_either_or_neither()
    {
        // a circle of radius 1 as a curve, and the bowl
        var aCircleCurve = new CurveAdaptor2D(new CircleCurve2D(new Circle2D(new Frame2D(new Point2D(0, 0), new Direction2D(1, 0)), 1)));
        CurveAdaptor2D aBowl = Bowl();
        double aX = Math.Sqrt((Math.Sqrt(5) - 1) / 2);
        TwoDDomain aCircleDomain = Domain(aCircleCurve.Value, 0, 2 * Math.PI), aBowlDomain = BowlDomain(aBowl);
        aCircleDomain.SetEquivalentParameters(0, 2 * Math.PI);
        foreach (GInter anInter in new[]
        {
            new GInter(aCircleCurve, aBowl, Tol, Tol),
            new GInter(aCircleCurve, aCircleDomain, aBowl, Tol, Tol),
            new GInter(aCircleCurve, aBowl, aBowlDomain, Tol, Tol),
            new GInter(aCircleCurve, aCircleDomain, aBowl, aBowlDomain, Tol, Tol),
        })
        {
            AssertPair(Crossings(anInter, aCircleCurve.Value, aBowl), aX);
        }
    }

    [Fact]
    public void The_locator_gives_the_sample_nearest_the_point()
    {
        // the bowl sampled at t = 0, 0.25, 0.5, 0.75 (OCCT's nbU - 1 samples): (1, 1) is a sample
        CurveAdaptor2D aBowl = Bowl();
        TheCurveLocatorOfTheProjPCurOfGInter.Locate(new Point2D(1.2, 0.9), aBowl, 5, out POnCurv2d aNear);
        Assert.Equal(0.75, aNear.Parameter(), 12);
        Assert.True(aNear.Value().DistanceTo(new Point2D(1, 1)) < 1e-12);
        // between 0.5 and 1 in 3 samples, 0.5, 0.75 and 1; and clamped to the curve's range
        TheCurveLocatorOfTheProjPCurOfGInter.Locate(new Point2D(2.1, 4.2), aBowl, 3, 0.5, 1, out POnCurv2d aRanged);
        Assert.Equal(0.75, aRanged.Parameter(), 12);
        TheCurveLocatorOfTheProjPCurOfGInter.Locate(new Point2D(-2, 4), aBowl, 3, -1, 2, out POnCurv2d aClamped);
        Assert.Equal(0, aClamped.Parameter(), 12);
        Assert.Throws<global::Kerf.Engine.Foundation.OutOfRangeException>(() =>
            TheCurveLocatorOfTheProjPCurOfGInter.Locate(new Point2D(0, 0), aBowl, 1, out _));
    }

    [Fact]
    public void A_curve_of_many_intervals_is_met_without_running_out_of_stack()
    {
        // OCCT recursed once for each interval of the second curve: OCCT #245's section met a pcurve of 9,364 intervals
        // and ended the process (decision 28612). A zigzag of 50,000 straight spans, crossed once by a vertical line
        const int Spans = 50000;
        var aPoles = new NCollection_Array1<Point2D>(1, Spans + 1);
        var aKnots = new NCollection_Array1<double>(1, Spans + 1);
        var aMults = new NCollection_Array1<int>(1, Spans + 1);
        for (int i = 1; i <= Spans + 1; i++)
        {
            aPoles[i] = new Point2D(i - 1, (i % 2) * 0.1);
            aKnots[i] = i - 1;
            aMults[i] = i == 1 || i == Spans + 1 ? 2 : 1;
        }
        var aZigzag = new CurveAdaptor2D(new BSplineCurve2D(aPoles, aKnots, aMults, 1));
        Assert.Equal(Spans, aZigzag.NbIntervals(Kerf.Geometry.Continuity.C2));
        var aLine = new CurveAdaptor2D(new LineCurve2D(new Point2D(12345.5, -1), new Direction2D(0, 1)));
        var anInter = new GInter(aLine, aZigzag, Tol, Tol);
        Assert.True(anInter.IsDone);
        IntersectionPoint2D aP = Assert.Single(Enumerable.Range(1, anInter.PointCount).Select(anInter.Point));
        Assert.Equal(12345.5, aP.Value().X, 9);
        Assert.Equal(0.05, aP.Value().Y, 9);
    }
}
