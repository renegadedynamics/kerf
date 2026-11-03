// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.GccEnt_Position;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Geom2dGcc_Circ2d2TanOn and its Geo and Iter solvers: the analytic dispatch gives GccAna's results, a free-form
/// centre curve goes through the bisectors (Geo), free-form arguments through the iteration (Iter), and the accessors
/// throw as OCCT's.
/// </summary>
public class Geom2dGcc_Circ2d2TanOnTests
{
    private static Circle2D Circ(double x, double y, double r) => new(new Axis2D(new Point2D(x, y), new Direction2D(1, 0)), r);

    private static Line2D Lin(double x, double y, double dx, double dy) => new(new Point2D(x, y), new Direction2D(dx, dy));

    private static Geom2dGcc_QualifiedCurve Qualified(Geom2d_Curve theCurve, GccEnt_Position thePos)
        => new(new Geom2dAdaptor_Curve(theCurve), thePos);

    [Fact]
    public void A_circle_and_a_line_on_a_line_give_GccAna_results()
    {
        // (GccAna's circle/circle constructors trip on OCCT's own range bugs too often for a sanity test: decision 778.)
        Circle2D c1 = Circ(0.5, 0.25, 1);
        Line2D l2 = Lin(0, -3, 1, 0), on = Lin(0, 4, 1, 0.1);
        var aGcc = new Geom2dGcc_Circ2d2TanOn(Qualified(new Geom2d_Circle(c1), GccEnt_unqualified),
                                              Qualified(new Geom2d_Line(l2), GccEnt_unqualified),
                                              new Geom2dAdaptor_Curve(new Geom2d_Line(on)), 1.0e-7, 0, 0, 0);
        var anAna = new GccAna_Circ2d2TanOn(new GccEnt_QualifiedCirc(c1, GccEnt_unqualified),
                                            new GccEnt_QualifiedLin(l2, GccEnt_unqualified), on, 1.0e-7);
        Assert.True(aGcc.IsDone());
        Assert.Equal(anAna.NbSolutions(), aGcc.NbSolutions());
        Assert.True(aGcc.NbSolutions() > 0);
        for (int i = 1; i <= aGcc.NbSolutions(); i++)
        {
            Assert.Equal(anAna.ThisSolution(i), aGcc.ThisSolution(i));
            aGcc.WhichQualifier(i, out GccEnt_Position q1, out GccEnt_Position q2);
            anAna.WhichQualifier(i, out GccEnt_Position a1, out GccEnt_Position a2);
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
        var aGcc = new Geom2dGcc_Circ2d2TanOn(Qualified(new Geom2d_Line(l1), GccEnt_unqualified),
                                              Qualified(new Geom2d_Circle(c2), GccEnt_outside),
                                              new Geom2dAdaptor_Curve(new Geom2d_Circle(on)), 1.0e-7, 0, 0, 0);
        var anAna = new GccAna_Circ2d2TanOn(new GccEnt_QualifiedCirc(c2, GccEnt_outside),
                                            new GccEnt_QualifiedLin(l1, GccEnt_unqualified), on, 1.0e-7);
        Assert.True(aGcc.IsDone());
        Assert.Equal(anAna.NbSolutions(), aGcc.NbSolutions());
        for (int i = 1; i <= aGcc.NbSolutions(); i++)
        {
            Assert.Equal(anAna.ThisSolution(i), aGcc.ThisSolution(i));
            aGcc.WhichQualifier(i, out GccEnt_Position q1, out GccEnt_Position q2);
            anAna.WhichQualifier(i, out GccEnt_Position a1, out GccEnt_Position a2);
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
        var aGcc = new Geom2dGcc_Circ2d2TanOn(Qualified(new Geom2d_Circle(c1), GccEnt_unqualified), new Geom2d_CartesianPoint(p),
                                              new Geom2dAdaptor_Curve(new Geom2d_Circle(on)), 1.0e-7, 0, 0);
        var anAna = new GccAna_Circ2d2TanOn(new GccEnt_QualifiedCirc(c1, GccEnt_unqualified), p, on, 1.0e-7);
        Assert.Equal(anAna.NbSolutions(), aGcc.NbSolutions());
        for (int i = 1; i <= aGcc.NbSolutions(); i++)
        {
            Assert.Equal(anAna.ThisSolution(i), aGcc.ThisSolution(i));
        }
    }

    [Fact]
    public void Two_circles_on_an_ellipse_go_through_the_bisectors()
    {
        Circle2D c1 = Circ(-3, 0, 1), c2 = Circ(3, 0, 1);
        var anEllipse = new Geom2d_Ellipse(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 5, 2);
        var aGcc = new Geom2dGcc_Circ2d2TanOn(Qualified(new Geom2d_Circle(c1), GccEnt_outside),
                                              Qualified(new Geom2d_Circle(c2), GccEnt_outside),
                                              new Geom2dAdaptor_Curve(anEllipse), 1.0e-7, 0, 0, 0);
        Assert.True(aGcc.IsDone());
        // The bisector of two equal circles is the y axis; it meets the ellipse at (0, 2) and (0, -2).
        Assert.Equal(2, aGcc.NbSolutions());
        for (int i = 1; i <= 2; i++)
        {
            Circle2D aSol = aGcc.ThisSolution(i);
            Assert.Equal(0, aSol.Location().X(), 1.0e-9);
            Assert.Equal(2, Math.Abs(aSol.Location().Y()), 1.0e-9);
            Assert.Equal(Math.Sqrt(13) - 1, aSol.Radius(), 1.0e-9);
            aGcc.CenterOn3(i, out double aPar, out Point2D aCen);
            Assert.True(anEllipse.Value(aPar).Distance(aCen) < 1.0e-9);
            aGcc.Tangency1(i, out _, out _, out Point2D t1);
            Assert.Equal(1, t1.Distance(c1.Location()), 1.0e-9);
            aGcc.WhichQualifier(i, out GccEnt_Position q1, out GccEnt_Position q2);
            Assert.Equal((GccEnt_outside, GccEnt_outside), (q1, q2));
        }
    }

    [Fact]
    public void An_ellipse_and_a_circle_on_a_line_iterate_to_one_circle()
    {
        // Ellipse x^2/16 + y^2/4 = 1 and the circle of radius 1 at (10, 0); centre on the x axis: the circle centred
        // at (6.5, 0) of radius 2.5 touches the ellipse at (4, 0) and the circle at (9, 0).
        var anEllipse = new Geom2d_Ellipse(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 4, 2);
        var aGcc = new Geom2dGcc_Circ2d2TanOn(Qualified(anEllipse, GccEnt_unqualified),
                                              Qualified(new Geom2d_Circle(Circ(10, 0, 1)), GccEnt_unqualified),
                                              new Geom2dAdaptor_Curve(new Geom2d_Line(Lin(0, 0, 1, 0))), 1.0e-7,
                                              0.05, Math.PI - 0.05, 6.4);
        Assert.True(aGcc.IsDone());
        Assert.Equal(1, aGcc.NbSolutions());
        Circle2D aSol = aGcc.ThisSolution(1);
        Assert.Equal(6.5, aSol.Location().X(), 1.0e-6);
        Assert.Equal(0, aSol.Location().Y(), 1.0e-6);
        Assert.Equal(2.5, aSol.Radius(), 1.0e-6);
        // OCCT's Iter returns 0 for the parameters, and the dispatch doesn't copy the centre's.
        aGcc.Tangency1(1, out double aParSol, out double aParArg, out Point2D t1);
        Assert.Equal((0.0, 0.0), (aParSol, aParArg));
        Assert.True(t1.Distance(new Point2D(4, 0)) < 1.0e-6);
        aGcc.CenterOn3(1, out double aParCen, out Point2D aCen);
        Assert.Equal(0.0, aParCen);
        Assert.Equal(new Point2D(), aCen);
    }

    [Fact]
    public void A_free_form_curve_and_a_point_throw_as_OCCT()
    {
        // OCCT gives math_FunctionSetRoot 3-vectors for the 4-variable function: Standard_OutOfRange, natively too.
        var anEllipse = new Geom2d_Ellipse(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 4, 2);
        Assert.ThrowsAny<OutOfRangeException>(
            () => new Geom2dGcc_Circ2d2TanOn(Qualified(anEllipse, GccEnt_unqualified), new Geom2d_CartesianPoint(new Point2D(10, 0)),
                                             new Geom2dAdaptor_Curve(new Geom2d_Line(Lin(0, 0, 1, 0))), 1.0e-7, 0, 6));
    }

    [Fact]
    public void Accessors_throw_as_OCCT()
    {
        Circle2D c1 = Circ(0, 0, 1), c2 = Circ(3, 0, 1);
        var aGcc = new Geom2dGcc_Circ2d2TanOn(Qualified(new Geom2d_Circle(c1), GccEnt_outside),
                                              Qualified(new Geom2d_Circle(c2), GccEnt_outside),
                                              new Geom2dAdaptor_Curve(new Geom2d_Line(Lin(1.5, 0, 0, 1))), 1.0e-7, 0, 0, 0);
        Assert.True(aGcc.IsDone());
        Assert.Throws<OutOfRangeException>(() => aGcc.ThisSolution(0));
        Assert.Throws<OutOfRangeException>(() => aGcc.ThisSolution(aGcc.NbSolutions() + 1));
        Assert.Throws<OutOfRangeException>(() => aGcc.CenterOn3(0, out _, out _));

        // An iteration that fails: IsDone is false and the accessors throw StdFail_NotDone.
        var anIter = new Geom2dGcc_Circ2d2TanOnIter(new Geom2dGcc_QCurve(new Geom2dAdaptor_Curve(new Geom2d_Circle(c1)), GccEnt_enclosed),
                                                    new Geom2dGcc_QCurve(new Geom2dAdaptor_Curve(new Geom2d_Circle(Circ(30, 0, 1))), GccEnt_enclosed),
                                                    Lin(0, 0, 1, 0), 0, 0, 0, 1.0e-7);
        if (!anIter.IsDone())
        {
            Assert.Throws<NotDoneException>(() => anIter.WhichQualifier(out _, out _));
            Assert.Throws<NotDoneException>(() => anIter.IsTheSame1());
        }
    }

    [Fact]
    public void Bad_qualifiers_throw()
    {
        var anEllipse = new Geom2d_Ellipse(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 4, 2);
        // An enclosing line is meaningless.
        Assert.Throws<GccEnt_BadQualifier>(
            () => new Geom2dGcc_Circ2d2TanOnGeo(new GccEnt_QualifiedLin(Lin(0, 0, 1, 0), GccEnt_enclosing),
                                                new GccEnt_QualifiedLin(Lin(0, 1, 1, 0), GccEnt_outside),
                                                new Geom2dAdaptor_Curve(anEllipse), 1.0e-7));
        Assert.Throws<GccEnt_BadQualifier>(
            () => new Geom2dGcc_Circ2d2TanOnIter(new GccEnt_QualifiedLin(Lin(0, 0, 1, 0), GccEnt_enclosing),
                                                 new Geom2dGcc_QCurve(new Geom2dAdaptor_Curve(anEllipse), GccEnt_outside),
                                                 Lin(0, 1, 1, 0), 0, 0, 0, 1.0e-7));
    }
}
