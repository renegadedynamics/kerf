// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Constraints;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The geometric solvers (decision 19805), by what every solution must satisfy: its centre on what was asked, its
/// distance from each argument what tangency requires, its tangency points on it. A qualifier can only narrow the answers.
/// </summary>
public class Geom2dGcc_GeoLawsTests
{
    private const double Tol = 1e-6;

    private static Circle2D Circle(double x, double y, double r) => new(new Frame2D(new Point2D(x, y), new Direction2D(1, 0)), r);

    private static Line2D Line(double x, double y, double dx, double dy) => new(new Point2D(x, y), new Direction2D(dx, dy));

    private static CurveAdaptor2D Bezier(params double[] theXY)
    {
        var aPoles = new NCollection_Array1<Point2D>(1, theXY.Length / 2);
        for (int i = 0; i < theXY.Length / 2; i++)
        {
            aPoles[i + 1] = new Point2D(theXY[2 * i], theXY[(2 * i) + 1]);
        }
        return new CurveAdaptor2D(new BezierCurve2D(aPoles));
    }

    // The centre runs along this: y = 0.1 x^2, from x = -10 to 10
    private static CurveAdaptor2D Track() => Bezier(-10, 10, 0, -10, 10, 10);

    private static double OffTrack(in Point2D theP) => theP.Y - (0.1 * theP.X * theP.X);

    private static readonly Qualifier[] AllQualifiers = [Qualifier.Outside, Qualifier.Enclosed, Qualifier.Enclosing];

    private static double DistanceToLine(in Point2D theP, in Line2D theLine)
    {
        double dx = theP.X - theLine.Location.X, dy = theP.Y - theLine.Location.Y;
        return Math.Abs((dx * theLine.Direction.Y) - (dy * theLine.Direction.X));
    }

    private static void AssertTangentToCircle(in Circle2D theSolution, in Circle2D theArgument)
    {
        double aD = theSolution.Location.DistanceTo(theArgument.Location);
        bool anExternal = Math.Abs(aD - theSolution.Radius - theArgument.Radius) < Tol;
        bool anInternal = Math.Abs(aD - Math.Abs(theSolution.Radius - theArgument.Radius)) < Tol;
        Assert.True(anExternal || anInternal, $"{aD} from a circle of {theArgument.Radius}, itself of {theSolution.Radius}");
    }

    private static void AssertTangentToLine(in Circle2D theSolution, in Line2D theArgument) =>
        Assert.Equal(theSolution.Radius, DistanceToLine(theSolution.Location, theArgument), Tol);

    private static void AssertThrough(in Circle2D theSolution, in Point2D theP) =>
        Assert.Equal(theSolution.Radius, theSolution.Location.DistanceTo(theP), Tol);

    private static void AssertOn(Circ2d2TanOnGeo theSolver, Action<Circle2D> theCheck, int theAtLeast)
    {
        if (theAtLeast == 0 && !theSolver.IsDone)
        {
            return; // a qualifier that leaves nothing can leave it not done
        }
        Assert.True(theSolver.IsDone);
        Assert.True(theSolver.SolutionCount >= theAtLeast, theSolver.SolutionCount.ToString());
        for (int i = 1; i <= theSolver.SolutionCount; i++)
        {
            Circle2D aSolution = theSolver.ThisSolution(i);
            Assert.Equal(0, OffTrack(aSolution.Location), 1e-4);
            theCheck(aSolution);
            theSolver.CenterOn3(i, out _, out Point2D aCentre);
            Assert.True(aCentre.DistanceTo(aSolution.Location) < Tol);
            if (!theSolver.IsTheSame1(i))
            {
                theSolver.Tangency1(i, out double aSol, out _, out Point2D aP);
                Assert.True(ElCLib.Value(aSol, aSolution).DistanceTo(aP) < Tol);
            }
            if (!theSolver.IsTheSame2(i))
            {
                theSolver.Tangency2(i, out double aSol, out _, out Point2D aP);
                Assert.True(ElCLib.Value(aSol, aSolution).DistanceTo(aP) < Tol);
            }
            theSolver.WhichQualifier(i, out _, out _);
        }
        Assert.Throws<Kerf.Engine.Foundation.OutOfRangeException>(() => theSolver.ThisSolution(theSolver.SolutionCount + 1));
    }

    [Fact]
    public void A_circle_and_a_line_with_the_centre_on_a_curve()
    {
        Circle2D aCircle = Circle(0, 8, 1);
        Line2D aLine = Line(-20, -1, 1, 0);
        void Check(Circle2D s)
        {
            AssertTangentToCircle(s, aCircle);
            AssertTangentToLine(s, aLine);
        }
        var aFree = new Circ2d2TanOnGeo(new QualifiedCirc(aCircle, Qualifier.Unqualified), new QualifiedLin(aLine, Qualifier.Unqualified), Track(), 1e-7);
        AssertOn(aFree, Check, 1);
        foreach (Qualifier aQ in AllQualifiers)
        {
            var aSolver = new Circ2d2TanOnGeo(new QualifiedCirc(aCircle, aQ), new QualifiedLin(aLine, aQ == Qualifier.Enclosing ? Qualifier.Outside : aQ), Track(), 1e-7);
            AssertOn(aSolver, Check, 0);
            Assert.True(!aSolver.IsDone || aSolver.SolutionCount <= aFree.SolutionCount);
        }
    }

    [Fact]
    public void Two_lines_with_the_centre_on_a_curve()
    {
        Line2D aFirst = Line(-20, -1, 1, 0), aSecond = Line(-6, 0, 0, 1);
        void Check(Circle2D s)
        {
            AssertTangentToLine(s, aFirst);
            AssertTangentToLine(s, aSecond);
        }
        var aFree = new Circ2d2TanOnGeo(new QualifiedLin(aFirst, Qualifier.Unqualified), new QualifiedLin(aSecond, Qualifier.Unqualified), Track(), 1e-7);
        AssertOn(aFree, Check, 1);
        foreach (Qualifier aQ in new[] { Qualifier.Outside, Qualifier.Enclosed })
        {
            var aSolver = new Circ2d2TanOnGeo(new QualifiedLin(aFirst, aQ), new QualifiedLin(aSecond, aQ), Track(), 1e-7);
            AssertOn(aSolver, Check, 0);
            Assert.True(!aSolver.IsDone || aSolver.SolutionCount <= aFree.SolutionCount);
        }
    }

    [Fact]
    public void A_circle_and_a_point_with_the_centre_on_a_curve()
    {
        Circle2D aCircle = Circle(0, 8, 1);
        var aPoint = new Point2D(2, -2);
        void Check(Circle2D s)
        {
            AssertTangentToCircle(s, aCircle);
            AssertThrough(s, aPoint);
        }
        var aFree = new Circ2d2TanOnGeo(new QualifiedCirc(aCircle, Qualifier.Unqualified), aPoint, Track(), 1e-7);
        AssertOn(aFree, Check, 1);
        foreach (Qualifier aQ in AllQualifiers)
        {
            var aSolver = new Circ2d2TanOnGeo(new QualifiedCirc(aCircle, aQ), aPoint, Track(), 1e-7);
            AssertOn(aSolver, Check, 0);
            Assert.True(!aSolver.IsDone || aSolver.SolutionCount <= aFree.SolutionCount);
        }
    }

    [Fact]
    public void A_line_and_a_point_with_the_centre_on_a_curve()
    {
        Line2D aLine = Line(-20, -1, 1, 0);
        var aPoint = new Point2D(2, -0.2);
        void Check(Circle2D s)
        {
            AssertTangentToLine(s, aLine);
            AssertThrough(s, aPoint);
        }
        var aFree = new Circ2d2TanOnGeo(new QualifiedLin(aLine, Qualifier.Unqualified), aPoint, Track(), 1e-7);
        AssertOn(aFree, Check, 1);
        foreach (Qualifier aQ in new[] { Qualifier.Outside, Qualifier.Enclosed })
        {
            var aSolver = new Circ2d2TanOnGeo(new QualifiedLin(aLine, aQ), aPoint, Track(), 1e-7);
            AssertOn(aSolver, Check, 0);
            Assert.True(!aSolver.IsDone || aSolver.SolutionCount <= aFree.SolutionCount);
        }
    }

    [Fact]
    public void Two_points_with_the_centre_on_a_curve_have_it_on_their_bisector()
    {
        Point2D aOne = new(-3, 5), aTwo = new(3, 5);
        var aSolver = new Circ2d2TanOnGeo(aOne, aTwo, Track(), 1e-7);
        AssertOn(aSolver, s =>
        {
            AssertThrough(s, aOne);
            AssertThrough(s, aTwo);
            Assert.Equal(0, s.Location.X, 1e-5);
        }, 1);
        // the same point twice has no bisector
        var aSame = new Circ2d2TanOnGeo(aOne, aOne, Track(), 1e-7);
        Assert.Equal(0, aSame.IsDone ? aSame.SolutionCount : 0);
    }

    [Fact]
    public void Two_circles_with_the_centre_on_a_curve()
    {
        Circle2D aFirst = Circle(-5, 8, 1), aSecond = Circle(5, 8, 1);
        void Check(Circle2D s)
        {
            AssertTangentToCircle(s, aFirst);
            AssertTangentToCircle(s, aSecond);
        }
        var aFree = new Circ2d2TanOnGeo(new QualifiedCirc(aFirst, Qualifier.Unqualified), new QualifiedCirc(aSecond, Qualifier.Unqualified), Track(), 1e-7);
        AssertOn(aFree, Check, 1);
        foreach (Qualifier aQ in AllQualifiers)
        {
            var aSolver = new Circ2d2TanOnGeo(new QualifiedCirc(aFirst, aQ), new QualifiedCirc(aSecond, aQ), Track(), 1e-7);
            AssertOn(aSolver, Check, 0);
            Assert.True(!aSolver.IsDone || aSolver.SolutionCount <= aFree.SolutionCount);
        }
    }

    // Circles of a given radius tangent to a curve and something else

    private static CurveAdaptor2D Hump() => Bezier(-10, 0, 0, 8, 10, 0); // highest at (0, 4)

    private static void AssertRad(Circ2d2TanRadGeo theSolver, double theRadius, Action<Circle2D> theCheck, int theAtLeast)
    {
        if (theAtLeast == 0 && !theSolver.IsDone)
        {
            return;
        }
        Assert.True(theSolver.IsDone);
        Assert.True(theSolver.SolutionCount >= theAtLeast, theSolver.SolutionCount.ToString());
        for (int i = 1; i <= theSolver.SolutionCount; i++)
        {
            Circle2D aSolution = theSolver.ThisSolution(i);
            Assert.Equal(theRadius, aSolution.Radius, Tol);
            theCheck(aSolution);
            if (!theSolver.IsTheSame2(i))
            {
                theSolver.Tangency2(i, out double aSol, out _, out Point2D aP);
                Assert.True(ElCLib.Value(aSol, aSolution).DistanceTo(aP) < 1e-5);
            }
            if (!theSolver.IsTheSame1(i))
            {
                theSolver.Tangency1(i, out double aSol, out _, out Point2D aP);
                Assert.True(ElCLib.Value(aSol, aSolution).DistanceTo(aP) < 1e-5);
            }
            theSolver.WhichQualifier(i, out _, out _);
        }
    }

    [Fact]
    public void A_radius_tangent_to_a_line_and_a_curve()
    {
        Line2D aLine = Line(-20, 5, 1, 0);
        var aFree = new Circ2d2TanRadGeo(new QualifiedLin(aLine, Qualifier.Unqualified), new QCurve(Hump(), Qualifier.Unqualified), 1, 1e-7);
        AssertRad(aFree, 1, s => AssertTangentToLine(s, aLine), 1);
        foreach (Qualifier aQ in new[] { Qualifier.Outside, Qualifier.Enclosed })
        {
            var aSolver = new Circ2d2TanRadGeo(new QualifiedLin(aLine, aQ), new QCurve(Hump(), aQ), 1, 1e-7);
            AssertRad(aSolver, 1, s => AssertTangentToLine(s, aLine), 0);
            Assert.True(!aSolver.IsDone || aSolver.SolutionCount <= aFree.SolutionCount);
        }
    }

    [Fact]
    public void A_radius_tangent_to_a_circle_and_a_curve()
    {
        Circle2D aCircle = Circle(0, 5.5, 1);
        var aFree = new Circ2d2TanRadGeo(new QualifiedCirc(aCircle, Qualifier.Unqualified), new QCurve(Hump(), Qualifier.Unqualified), 0.5, 1e-7);
        AssertRad(aFree, 0.5, s => AssertTangentToCircle(s, aCircle), 1);
        foreach (Qualifier aQ in AllQualifiers)
        {
            var aSolver = new Circ2d2TanRadGeo(new QualifiedCirc(aCircle, aQ), new QCurve(Hump(), aQ), 0.5, 1e-7);
            AssertRad(aSolver, 0.5, s => AssertTangentToCircle(s, aCircle), 0);
            Assert.True(!aSolver.IsDone || aSolver.SolutionCount <= aFree.SolutionCount);
        }
    }

    [Fact]
    public void A_radius_tangent_to_a_curve_through_a_point()
    {
        var aPoint = new Point2D(1, 5);
        var aFree = new Circ2d2TanRadGeo(new QCurve(Hump(), Qualifier.Unqualified), aPoint, 2, 1e-7);
        AssertRad(aFree, 2, s => AssertThrough(s, aPoint), 1);
        foreach (Qualifier aQ in AllQualifiers)
        {
            var aSolver = new Circ2d2TanRadGeo(new QCurve(Hump(), aQ), aPoint, 2, 1e-7);
            AssertRad(aSolver, 2, s => AssertThrough(s, aPoint), 0);
            Assert.True(!aSolver.IsDone || aSolver.SolutionCount <= aFree.SolutionCount);
        }
    }

    [Fact]
    public void A_radius_tangent_to_two_curves()
    {
        CurveAdaptor2D aDip = Bezier(-10, 12, 0, 4, 10, 12); // lowest at (0, 8)
        var aFree = new Circ2d2TanRadGeo(new QCurve(Hump(), Qualifier.Unqualified), new QCurve(aDip, Qualifier.Unqualified), 3, 1e-7);
        AssertRad(aFree, 3, _ => { }, 1);
        foreach (Qualifier aQ in new[] { Qualifier.Outside, Qualifier.Enclosed })
        {
            var aSolver = new Circ2d2TanRadGeo(new QCurve(Hump(), aQ), new QCurve(aDip, aQ), 3, 1e-7);
            AssertRad(aSolver, 3, _ => { }, 0);
            Assert.True(!aSolver.IsDone || aSolver.SolutionCount <= aFree.SolutionCount);
        }
    }

    [Fact]
    public void A_negative_radius_or_no_qualifier_is_refused()
    {
        Line2D aLine = Line(-20, 5, 1, 0);
        Circle2D aCircle = Circle(0, 8, 1);
        var aFine = new QCurve(Hump(), Qualifier.Unqualified);
        var aNone = new QCurve(Hump(), Qualifier.Noqualifier);
        Assert.Throws<Kerf.Engine.Foundation.NegativeValueException>(() => new Circ2d2TanRadGeo(new QualifiedLin(aLine, Qualifier.Unqualified), aFine, -1, 1e-7));
        Assert.Throws<Kerf.Engine.Foundation.NegativeValueException>(() => new Circ2d2TanRadGeo(new QualifiedCirc(aCircle, Qualifier.Unqualified), aFine, -1, 1e-7));
        Assert.Throws<Kerf.Engine.Foundation.NegativeValueException>(() => new Circ2d2TanRadGeo(aFine, new Point2D(0, 0), -1, 1e-7));
        Assert.Throws<Kerf.Engine.Foundation.NegativeValueException>(() => new Circ2d2TanRadGeo(aFine, aFine, -1, 1e-7));
        Assert.Throws<BadQualifierException>(() => new Circ2d2TanRadGeo(new QualifiedLin(aLine, Qualifier.Unqualified), aNone, 1, 1e-7));
        Assert.Throws<BadQualifierException>(() => new Circ2d2TanRadGeo(new QualifiedCirc(aCircle, Qualifier.Unqualified), aNone, 1, 1e-7));
        Assert.Throws<BadQualifierException>(() => new Circ2d2TanRadGeo(aNone, new Point2D(0, 0), 1, 1e-7));
        Assert.Throws<BadQualifierException>(() => new Circ2d2TanRadGeo(aFine, aNone, 1, 1e-7));
    }
}
