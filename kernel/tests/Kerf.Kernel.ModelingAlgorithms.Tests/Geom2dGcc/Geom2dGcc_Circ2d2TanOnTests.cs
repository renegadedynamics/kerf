// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Constraints;
using Kerf.Engine.Construction.Constraints.Analytic;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Geom2dGcc_Circ2d2TanOn and its Geo and Iter solvers: the analytic dispatch gives GccAna's results, a free-form
/// centre curve goes through the bisectors (Geo), free-form arguments through the iteration (Iter), and the accessors
/// throw as OCCT's.
/// </summary>
public class Geom2dGcc_Circ2d2TanOnTests
{
    private static Circle2D Circ(double x, double y, double r) => new(new Axis2D(new Point2D(x, y), new Direction2D(1, 0)), r);

    private static Line2D Lin(double x, double y, double dx, double dy) => new(new Point2D(x, y), new Direction2D(dx, dy));

    private static QualifiedCurve Qualified(Curve2D theCurve, Qualifier thePos)
        => new(new CurveAdaptor2D(theCurve), thePos);

    [Fact]
    public void A_circle_and_a_line_on_a_line_give_GccAna_results()
    {
        // (GccAna's circle/circle constructors trip on OCCT's own range bugs too often for a sanity test: decision 778.)
        Circle2D c1 = Circ(0.5, 0.25, 1);
        Line2D l2 = Lin(0, -3, 1, 0), on = Lin(0, 4, 1, 0.1);
        var aGcc = new Circ2d2TanOn(Qualified(new CircleCurve2D(c1), Qualifier.Unqualified),
                                              Qualified(new LineCurve2D(l2), Qualifier.Unqualified),
                                              new CurveAdaptor2D(new LineCurve2D(on)), 1.0e-7, 0, 0, 0);
        var anAna = new AnalyticCirc2d2TanOn(new QualifiedCirc(c1, Qualifier.Unqualified),
                                            new QualifiedLin(l2, Qualifier.Unqualified), on, 1.0e-7);
        Assert.True(aGcc.IsDone);
        Assert.Equal(anAna.SolutionCount, aGcc.SolutionCount);
        Assert.True(aGcc.SolutionCount > 0);
        for (int i = 1; i <= aGcc.SolutionCount; i++)
        {
            Assert.Equal(anAna.ThisSolution(i), aGcc.ThisSolution(i));
            aGcc.WhichQualifier(i, out Qualifier q1, out Qualifier q2);
            anAna.WhichQualifier(i, out Qualifier a1, out Qualifier a2);
            Assert.Equal((a1, a2), (q1, q2));
            aGcc.Tangency1(i, out double p1, out double a1p, out Point2D t1);
            anAna.Tangency1(i, out double r1, out double b1p, out Point2D u1);
            Assert.Equal((r1, b1p, u1), (p1, a1p, t1));
            aGcc.CenterOn3(i, out double pc, out Point2D cc);
            anAna.CenterOn3(i, out double rc, out Point2D uc);
            Assert.Equal((rc, uc), (pc, cc));
        }
    }

    [Fact]
    public void A_line_and_a_circle_on_a_circle_are_inverted_back()
    {
        // A line first and a circle second: GccAna takes the circle first, and the dispatch swaps the answers back.
        Line2D l1 = Lin(0, -1, 1, 0);
        Circle2D c2 = Circ(2, 3, 1), on = Circ(0, 2, 4);
        var aGcc = new Circ2d2TanOn(Qualified(new LineCurve2D(l1), Qualifier.Unqualified),
                                              Qualified(new CircleCurve2D(c2), Qualifier.Outside),
                                              new CurveAdaptor2D(new CircleCurve2D(on)), 1.0e-7, 0, 0, 0);
        var anAna = new AnalyticCirc2d2TanOn(new QualifiedCirc(c2, Qualifier.Outside),
                                            new QualifiedLin(l1, Qualifier.Unqualified), on, 1.0e-7);
        Assert.True(aGcc.IsDone);
        Assert.Equal(anAna.SolutionCount, aGcc.SolutionCount);
        for (int i = 1; i <= aGcc.SolutionCount; i++)
        {
            Assert.Equal(anAna.ThisSolution(i), aGcc.ThisSolution(i));
            aGcc.WhichQualifier(i, out Qualifier q1, out Qualifier q2);
            anAna.WhichQualifier(i, out Qualifier a1, out Qualifier a2);
            Assert.Equal((a2, a1), (q1, q2));
            aGcc.Tangency1(i, out _, out _, out Point2D t1);
            anAna.Tangency2(i, out _, out _, out Point2D u2);
            Assert.Equal(u2, t1);
        }
    }

    [Fact]
    public void A_circle_and_a_point_give_GccAna_results()
    {
        Circle2D c1 = Circ(1, 1, 2), on = Circ(0, 0, 6);
        Point2D p = new(5, 1);
        var aGcc = new Circ2d2TanOn(Qualified(new CircleCurve2D(c1), Qualifier.Unqualified), new CartesianPoint2(p),
                                              new CurveAdaptor2D(new CircleCurve2D(on)), 1.0e-7, 0, 0);
        var anAna = new AnalyticCirc2d2TanOn(new QualifiedCirc(c1, Qualifier.Unqualified), p, on, 1.0e-7);
        Assert.Equal(anAna.SolutionCount, aGcc.SolutionCount);
        for (int i = 1; i <= aGcc.SolutionCount; i++)
        {
            Assert.Equal(anAna.ThisSolution(i), aGcc.ThisSolution(i));
        }
    }

    [Fact]
    public void Two_circles_on_an_ellipse_go_through_the_bisectors()
    {
        Circle2D c1 = Circ(-3, 0, 1), c2 = Circ(3, 0, 1);
        var anEllipse = new EllipseCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 5, 2);
        var aGcc = new Circ2d2TanOn(Qualified(new CircleCurve2D(c1), Qualifier.Outside),
                                              Qualified(new CircleCurve2D(c2), Qualifier.Outside),
                                              new CurveAdaptor2D(anEllipse), 1.0e-7, 0, 0, 0);
        Assert.True(aGcc.IsDone);
        // The bisector of two equal circles is the y axis; it meets the ellipse at (0, 2) and (0, -2).
        Assert.Equal(2, aGcc.SolutionCount);
        for (int i = 1; i <= 2; i++)
        {
            Circle2D aSol = aGcc.ThisSolution(i);
            Assert.Equal(0, aSol.Location.X, 1.0e-9);
            Assert.Equal(2, Math.Abs(aSol.Location.Y), 1.0e-9);
            Assert.Equal(Math.Sqrt(13) - 1, aSol.Radius, 1.0e-9);
            aGcc.CenterOn3(i, out double aPar, out Point2D aCen);
            Assert.True(anEllipse.Value(aPar).DistanceTo(aCen) < 1.0e-9);
            aGcc.Tangency1(i, out _, out _, out Point2D t1);
            Assert.Equal(1, t1.DistanceTo(c1.Location), 1.0e-9);
            aGcc.WhichQualifier(i, out Qualifier q1, out Qualifier q2);
            Assert.Equal((Qualifier.Outside, Qualifier.Outside), (q1, q2));
        }
    }

    [Fact]
    public void An_ellipse_and_a_circle_on_a_line_iterate_to_one_circle()
    {
        // Ellipse x^2/16 + y^2/4 = 1 and the circle of radius 1 at (10, 0); centre on the x axis: the circle centred
        // at (6.5, 0) of radius 2.5 touches the ellipse at (4, 0) and the circle at (9, 0).
        var anEllipse = new EllipseCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 4, 2);
        var aGcc = new Circ2d2TanOn(Qualified(anEllipse, Qualifier.Unqualified),
                                              Qualified(new CircleCurve2D(Circ(10, 0, 1)), Qualifier.Unqualified),
                                              new CurveAdaptor2D(new LineCurve2D(Lin(0, 0, 1, 0))), 1.0e-7,
                                              0.05, Math.PI - 0.05, 6.4);
        Assert.True(aGcc.IsDone);
        Assert.Equal(1, aGcc.SolutionCount);
        Circle2D aSol = aGcc.ThisSolution(1);
        Assert.Equal(6.5, aSol.Location.X, 1.0e-6);
        Assert.Equal(0, aSol.Location.Y, 1.0e-6);
        Assert.Equal(2.5, aSol.Radius, 1.0e-6);
        // the Iter's parameters and the centre, copied (OCCT returned 0 and left the centre the origin: decision 1545)
        aGcc.Tangency1(1, out double aParSol, out double aParArg, out Point2D t1);
        Assert.True(t1.DistanceTo(new Point2D(4, 0)) < 1.0e-6);
        Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aParSol, aSol).DistanceTo(t1) < 1.0e-6);
        Assert.True(anEllipse.Value(aParArg).DistanceTo(t1) < 1.0e-6);
        aGcc.CenterOn3(1, out double aParCen, out Point2D aCen);
        Assert.True(aCen.DistanceTo(aSol.Location) < 1.0e-9);
        Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aParCen, Lin(0, 0, 1, 0)).DistanceTo(aSol.Location) < 1.0e-6);
    }

    [Fact]
    public void A_free_form_curve_and_a_point_on_a_line()
    {
        // The circle through (10, 0) tangent to the ellipse x^2/16 + y^2/4 = 1, centred on the x axis: (7, 0), radius 3.
        // OCCT gave math_FunctionSetRoot 3-vectors for a 4-variable function and threw (decision 1545).
        var anEllipse = new EllipseCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 4, 2);
        var aGcc = new Circ2d2TanOn(Qualified(anEllipse, Qualifier.Unqualified), new CartesianPoint2(new Point2D(10, 0)),
                                    new CurveAdaptor2D(new LineCurve2D(Lin(0, 0, 1, 0))), 1.0e-7, 0, 6);
        Assert.True(aGcc.IsDone);
        Assert.Equal(1, aGcc.SolutionCount);
        Circle2D aSol = aGcc.ThisSolution(1);
        Assert.Equal(7.0, aSol.Location.X, 1.0e-6);
        Assert.Equal(0.0, aSol.Location.Y, 1.0e-6);
        Assert.Equal(3.0, aSol.Radius, 1.0e-6);
        aGcc.Tangency1(1, out double aParSol, out double aParArg, out Point2D t1);
        Assert.True(t1.DistanceTo(new Point2D(4, 0)) < 1.0e-6);
        Assert.True(anEllipse.Value(aParArg).DistanceTo(t1) < 1.0e-6);
        Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aParSol, aSol).DistanceTo(t1) < 1.0e-6);
        aGcc.Tangency2(1, out _, out _, out Point2D t2);
        Assert.True(t2.DistanceTo(new Point2D(10, 0)) < 1.0e-9);
    }

    [Fact]
    public void Accessors_throw_as_OCCT()
    {
        Circle2D c1 = Circ(0, 0, 1), c2 = Circ(3, 0, 1);
        var aGcc = new Circ2d2TanOn(Qualified(new CircleCurve2D(c1), Qualifier.Outside),
                                              Qualified(new CircleCurve2D(c2), Qualifier.Outside),
                                              new CurveAdaptor2D(new LineCurve2D(Lin(1.5, 0, 0, 1))), 1.0e-7, 0, 0, 0);
        Assert.True(aGcc.IsDone);
        EngineAssert.Throws<OutOfRangeException>(() => aGcc.ThisSolution(0));
        EngineAssert.Throws<OutOfRangeException>(() => aGcc.ThisSolution(aGcc.SolutionCount + 1));
        EngineAssert.Throws<OutOfRangeException>(() => aGcc.CenterOn3(0, out _, out _));

        // An iteration that fails: IsDone is false and the accessors throw StdFail_NotDone.
        var anIter = new Circ2d2TanOnIter(new QCurve(new CurveAdaptor2D(new CircleCurve2D(c1)), Qualifier.Enclosed),
                                                    new QCurve(new CurveAdaptor2D(new CircleCurve2D(Circ(30, 0, 1))), Qualifier.Enclosed),
                                                    Lin(0, 0, 1, 0), 0, 0, 0, 1.0e-7);
        if (!anIter.IsDone)
        {
            EngineAssert.Throws<NotDoneException>(() => anIter.WhichQualifier(out _, out _));
            EngineAssert.Throws<NotDoneException>(() => anIter.IsTheSame1());
        }
    }

    [Fact]
    public void Bad_qualifiers_throw()
    {
        var anEllipse = new EllipseCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 4, 2);
        // An enclosing line is meaningless.
        EngineAssert.Throws<BadQualifierException>(
            () => new Circ2d2TanOnGeo(new QualifiedLin(Lin(0, 0, 1, 0), Qualifier.Enclosing),
                                                new QualifiedLin(Lin(0, 1, 1, 0), Qualifier.Outside),
                                                new CurveAdaptor2D(anEllipse), 1.0e-7));
        EngineAssert.Throws<BadQualifierException>(
            () => new Circ2d2TanOnIter(new QualifiedLin(Lin(0, 0, 1, 0), Qualifier.Enclosing),
                                                 new QCurve(new CurveAdaptor2D(anEllipse), Qualifier.Outside),
                                                 Lin(0, 1, 1, 0), 0, 0, 0, 1.0e-7));
    }
}
