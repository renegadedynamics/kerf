// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using Kerf.Geometry;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>
/// Kerf.Geometry's <see cref="Tangents2D"/> (docs/api/geometry.md; unit 10L5): circles and lines constrained by 2D curves
/// and points, checked by the laws a solution must satisfy: each contact lies on the solution and on its target, and
/// the solution's tangent there is the target's.
/// </summary>
public class Tangents2DTests
{
    private const double Tol = 1e-7;

    private static CircleCurve2D Circle(double x, double y, double r) => new(new Axis2D(new Point2D(x, y), Direction2D.UnitX), r);

    private static LineCurve2D Line(double x, double y, double dx, double dy) => new(new Point2D(x, y), new Direction2D(dx, dy));

    private static double DistanceToCircle(Circle2D c, Point2D p) => Math.Abs(c.Location.DistanceTo(p) - c.Radius);

    private static double DistanceToLine(Line2D l, Point2D p) => l.DistanceTo(p);

    // A contact on a curve target: on the target at its parameter, and the solution's tangent parallel to the target's
    private static void AssertTangent(Curve2D target, TangentContact2D contact, Vector2D solutionTangent)
    {
        Assert.False(contact.IsCoincident);
        Assert.True(target.Evaluate(contact.TargetParameter).DistanceTo(contact.Point) < 1e-6);
        Vector2D targetTangent = target.Derivatives(contact.TargetParameter, 1).D1;
        Assert.True(Math.Abs(solutionTangent.Crossed(targetTangent)) < 1e-6 * solutionTangent.Magnitude * targetTangent.Magnitude);
    }

    private static Vector2D TangentOf(Circle2D c, Point2D p) => new(-(p.Y - c.Location.Y), p.X - c.Location.X);

    [Fact]
    public void Two_circles_have_four_common_tangent_lines()
    {
        CircleCurve2D a = Circle(0, 0, 1), b = Circle(10, 0, 2);
        ImmutableArray<TangentLine2D> lines = Tangents2D.Lines(a, b).Value;
        Assert.Equal(4, lines.Length);
        foreach (TangentLine2D line in lines)
        {
            Assert.Equal(2, line.Contacts.Length);
            AssertTangent(a, line.Contacts[0], new Vector2D(line.Line.Direction));
            AssertTangent(b, line.Contacts[1], new Vector2D(line.Line.Direction));
            Assert.True(DistanceToLine(line.Line, line.Contacts[0].Point) < Tol);
        }

        // Qualified: outside both (the solution on the right of its direction)
        ImmutableArray<TangentLine2D> outside = Tangents2D.Lines(TangentTarget2D.Tangent(a, Tangency.Outside),
                                                                 TangentTarget2D.Tangent(b, Tangency.Outside)).Value;
        Assert.Single(outside);
        Assert.All(outside[0].Contacts, c => Assert.Equal(Tangency.Outside, c.Tangency));
    }

    [Fact]
    public void A_line_through_a_point_keeps_its_targets_order()
    {
        CircleCurve2D circle = Circle(0, 0, 1);
        ImmutableArray<TangentLine2D> lines = Tangents2D.Lines(new Point2D(5, 0), circle).Value;
        Assert.Equal(2, lines.Length);
        foreach (TangentLine2D line in lines)
        {
            Assert.True(line.Contacts[0].Point.DistanceTo(new Point2D(5, 0)) < Tol);   // the point first, as given
            AssertTangent(circle, line.Contacts[1], new Vector2D(line.Line.Direction));
        }
    }

    [Fact]
    public void Circles_of_a_radius_touch_two_lines_and_pass_through_points()
    {
        LineCurve2D x = Line(0, 0, 1, 0), y = Line(0, 0, 0, 1);
        ImmutableArray<TangentCircle2D> circles = Tangents2D.Circles(x, y, 2).Value;
        Assert.Equal(4, circles.Length);
        foreach (TangentCircle2D c in circles)
        {
            Assert.Equal(2, c.Circle.Radius, 9);
            AssertTangent(x, c.Contacts[0], TangentOf(c.Circle, c.Contacts[0].Point));
            AssertTangent(y, c.Contacts[1], TangentOf(c.Circle, c.Contacts[1].Point));
        }

        ImmutableArray<TangentCircle2D> enclosed = Tangents2D.Circles(TangentTarget2D.Tangent(x, Tangency.Enclosed),
                                                                      TangentTarget2D.Tangent(y, Tangency.Outside), 2).Value;
        TangentCircle2D one = Assert.Single(enclosed);
        Assert.True(one.Circle.Location.DistanceTo(new Point2D(2, 2)) < Tol);

        ImmutableArray<TangentCircle2D> throughTwo = Tangents2D.Circles(new Point2D(-1, 0), new Point2D(1, 0), 1).Value;
        Assert.Single(throughTwo);   // the two points a diameter apart
    }

    [Fact]
    public void Circles_touch_three_targets()
    {
        // through three points: the circumcircle
        TangentCircle2D through = Assert.Single(Tangents2D.Circles(new Point2D(1, 0), new Point2D(0, 1), new Point2D(-1, 0)).Value);
        Assert.True(through.Circle.Location.DistanceTo(Point2D.Origin) < Tol);
        Assert.Equal(1, through.Circle.Radius, 9);

        // three lines: the incircle and three excircles of the triangle
        LineCurve2D a = Line(0, 0, 1, 0), b = Line(0, 0, 0, 1), c = Line(4, 0, -4, 3);
        ImmutableArray<TangentCircle2D> circles = Tangents2D.Circles(a, b, c).Value;
        Assert.Equal(4, circles.Length);
        Assert.Contains(circles, s => Math.Abs(s.Circle.Radius - 1) < 1e-9);   // the 3-4-5 triangle's inradius
        foreach (TangentCircle2D s in circles)
        {
            for (int k = 0; k < 3; k++)
            {
                AssertTangent(new[] { a, b, c }[k], s.Contacts[k], TangentOf(s.Circle, s.Contacts[k].Point));
            }
        }

        // two circles and a point, the point given first
        CircleCurve2D c1 = Circle(-5, 0, 1), c2 = Circle(5, 0, 1);
        ImmutableArray<TangentCircle2D> mixed = Tangents2D.Circles(new Point2D(0, 3), c1, c2).Value;
        Assert.NotEmpty(mixed);
        Assert.All(mixed, s =>
        {
            Assert.True(DistanceToCircle(s.Circle, new Point2D(0, 3)) < 1e-6);
            AssertTangent(c1, s.Contacts[1], TangentOf(s.Circle, s.Contacts[1].Point));
            AssertTangent(c2, s.Contacts[2], TangentOf(s.Circle, s.Contacts[2].Point));
        });
    }

    [Fact]
    public void A_free_form_curve_is_solved_iteratively()
    {
        var bezier = new BezierCurve2D([new Point2D(0, 0), new Point2D(50, 50), new Point2D(0, 100)]);
        LineCurve2D line = Line(100, 0, -1, 0);   // OCCT's BUC60897
        ImmutableArray<TangentCircle2D> circles = Tangents2D.Circles(TangentTarget2D.Tangent(line, Tangency.Outside),
                                                                     TangentTarget2D.Tangent(bezier, Tangency.Outside), 10).Value;
        Assert.NotEmpty(circles);
        Assert.All(circles, s =>
        {
            Assert.True(DistanceToCircle(s.Circle, s.Contacts[1].Point) < 1e-6);
            AssertTangent(bezier, s.Contacts[1], TangentOf(s.Circle, s.Contacts[1].Point));
        });
    }

    [Fact]
    public void Circles_about_a_centre_touch_a_circle_twice()
    {
        CircleCurve2D circle = Circle(0, 0, 2);
        ImmutableArray<TangentCircle2D> circles = Tangents2D.CirclesCentredAt(circle, new Point2D(5, 0)).Value;
        Assert.Equal([3.0, 7.0], circles.Select(c => Math.Round(c.Circle.Radius, 9)).Order());
        Assert.Empty(Tangents2D.CirclesCentredAt(TangentTarget2D.Tangent(circle, Tangency.Enclosed), new Point2D(5, 0)).Value);
    }

    [Fact]
    public void Misuse_throws()
    {
        LineCurve2D line = Line(0, 0, 1, 0);
        KernelAssert.Throws<ArgumentException>(() => Tangents2D.Lines(new Point2D(0, 0), new Point2D(1, 1)));
        KernelAssert.Throws<ArgumentException>(() => Tangents2D.Circles(TangentTarget2D.Tangent(line, Tangency.Enclosing), Circle(0, 3, 1), 1));
        KernelAssert.Throws<ArgumentOutOfRangeException>(() => Tangents2D.Circles(line, Circle(0, 3, 1), 0));
        KernelAssert.Throws<ArgumentException>(() => Tangents2D.CirclesCentredAt(new Point2D(1, 1), Point2D.Origin));
        KernelAssert.Throws<ArgumentNullException>(() => TangentTarget2D.Tangent(null!));
    }
}
