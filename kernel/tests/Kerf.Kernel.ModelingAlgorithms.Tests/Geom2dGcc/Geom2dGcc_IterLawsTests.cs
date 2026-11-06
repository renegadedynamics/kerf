// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Constraints;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The iterative solvers, one test for each kind of argument (decision 19805). The answer is known: everything is built
/// tangent to the circle of radius 2 about the origin, or through it. Started at that answer, the solver must keep it,
/// give its tangency points, and refuse a qualifier the answer doesn't satisfy; started near it, it must come back.
/// </summary>
public class Geom2dGcc_IterLawsTests
{
    private const double Tol = 1e-7;
    private const double Root = 1.4142135623730951;

    // Arguments tangent to the answer from outside it

    private static readonly Line2D Above = new(new Point2D(0, 2), new Direction2D(1, 0)); // at parameter 0
    private static readonly Line2D Left = new(new Point2D(-2, 0), new Direction2D(0, 1)); // at parameter 0
    private static readonly Circle2D Right = new(new Frame2D(new Point2D(5, 0), new Direction2D(1, 0)), 3); // at angle pi
    private static readonly Circle2D Below = new(new Frame2D(new Point2D(0, -5), new Direction2D(1, 0)), 3); // at angle pi / 2

    // Parabolas curving away from the answer, each touching it at its middle: parameter 0.5
    private static CurveAdaptor2D Bow(double theX0, double theY0, double theX1, double theY1, double theX2, double theY2)
    {
        var aPoles = new NCollection_Array1<Point2D>(1, 3);
        aPoles[1] = new Point2D(theX0, theY0);
        aPoles[2] = new Point2D(theX1, theY1);
        aPoles[3] = new Point2D(theX2, theY2);
        return new CurveAdaptor2D(new BezierCurve2D(aPoles));
    }

    private static CurveAdaptor2D BowLeft() => Bow(-3, -2, -1, 0, -3, 2);

    private static CurveAdaptor2D BowBelow() => Bow(-2, -3, 0, -1, 2, -3);

    private static CurveAdaptor2D BowRight() => Bow(3, -2, 1, 0, 3, 2);

    private static CurveAdaptor2D BowAbove() => Bow(-2, 3, 0, 1, 2, 3);

    // Points on the answer. They're below its centre on purpose: a point is taken as a circle of no radius, whose parameter
    // stays at pi, and the solver checks the tangent there, (0, -1), against the way to the centre (occt-issues.md)
    private static readonly Point2D OnArcA = new(Root, -Root);
    private static readonly Point2D OnArcB = new(-Root, -Root);

    private static QCurve Q(CurveAdaptor2D theCurve, Qualifier theQ) => new(theCurve, theQ);

    private static QualifiedCirc Q(in Circle2D theCircle, Qualifier theQ) => new(theCircle, theQ);

    private static QualifiedLin Q(in Line2D theLine, Qualifier theQ) => new(theLine, theQ);

    private static void AssertAnswer(in Circle2D theSolution)
    {
        Assert.Equal(0, theSolution.Location.X, Tol);
        Assert.Equal(0, theSolution.Location.Y, Tol);
        Assert.Equal(2, theSolution.Radius, Tol);
    }

    private static void AssertTangency(in Circle2D theSolution, double theParSol, in Point2D thePoint)
    {
        // the point is on the answer, at the parameter it says
        Assert.True(ElCLib.Value(theParSol, theSolution).DistanceTo(thePoint) < Tol);
        Assert.Equal(theSolution.Radius, theSolution.Location.DistanceTo(thePoint), Tol);
    }

    private static void AssertSolved(Circ2d3TanIter theSolver)
    {
        Assert.True(theSolver.IsDone);
        Circle2D aSolution = theSolver.ThisSolution();
        AssertAnswer(aSolution);
        theSolver.Tangency1(out double aSol1, out _, out Point2D aP1);
        AssertTangency(aSolution, aSol1, aP1);
        theSolver.Tangency2(out double aSol2, out _, out Point2D aP2);
        AssertTangency(aSolution, aSol2, aP2);
        theSolver.Tangency3(out double aSol3, out _, out Point2D aP3);
        AssertTangency(aSolution, aSol3, aP3);
        theSolver.WhichQualifier(out Qualifier aQ1, out _, out _); // a point has no qualifier
        Assert.NotEqual(Qualifier.Noqualifier, aQ1);
        Assert.False(theSolver.IsTheSame1());
        Assert.False(theSolver.IsTheSame2());
    }

    public static TheoryData<int> ThreeTangentForms() => new() { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };

    private static Circ2d3TanIter Three(int theForm, Qualifier theQ, double theStartOff = 0)
    {
        double d = theStartOff;
        return theForm switch
        {
            0 => new Circ2d3TanIter(Q(BowLeft(), theQ), Q(BowBelow(), theQ), Q(BowRight(), theQ), 0.5 + d, 0.5 + d, 0.5 + d, 1e-9),
            1 => new Circ2d3TanIter(Q(Right, theQ), Q(BowBelow(), theQ), Q(BowLeft(), theQ), Math.PI + d, 0.5 + d, 0.5 + d, 1e-9),
            2 => new Circ2d3TanIter(Q(Right, theQ), Q(Below, theQ), Q(BowLeft(), theQ), Math.PI + d, Math.PI / 2 + d, 0.5 + d, 1e-9),
            3 => new Circ2d3TanIter(Q(Above, theQ), Q(BowBelow(), theQ), Q(BowLeft(), theQ), 0 + d, 0.5 + d, 0.5 + d, 1e-9),
            4 => new Circ2d3TanIter(Q(Above, theQ), Q(Left, theQ), Q(BowBelow(), theQ), 0 + d, 0 + d, 0.5 + d, 1e-9),
            5 => new Circ2d3TanIter(Q(BowLeft(), theQ), Q(BowAbove(), theQ), OnArcA, 0.5 + d, 0.5 + d, 1e-9),
            6 => new Circ2d3TanIter(Q(BowAbove(), theQ), OnArcB, OnArcA, 0.5 + d, 1e-9),
            7 => new Circ2d3TanIter(Q(Above, theQ), Q(BowLeft(), theQ), OnArcA, 0 + d, 0.5 + d, 1e-9),
            8 => new Circ2d3TanIter(Q(Right, theQ), Q(Above, theQ), Q(BowLeft(), theQ), Math.PI + d, 0 + d, 0.5 + d, 1e-9),
            _ => new Circ2d3TanIter(Q(Right, theQ), Q(BowAbove(), theQ), OnArcB, Math.PI + d, 0.5 + d, 1e-9),
        };
    }

    [Theory]
    [MemberData(nameof(ThreeTangentForms))]
    public void Three_tangents_keep_the_answer_they_start_on(int theForm)
    {
        AssertSolved(Three(theForm, Qualifier.Unqualified));
    }

    [Theory]
    [MemberData(nameof(ThreeTangentForms))]
    public void Three_tangents_come_back_from_nearby(int theForm)
    {
        // nearer than it started, though only as near as its stopping rule asks, not to the tolerance
        Circ2d3TanIter aSolver = Three(theForm, Qualifier.Unqualified, 0.01);
        Assert.True(aSolver.IsDone);
        Circle2D aSolution = aSolver.ThisSolution();
        Assert.Equal(0, aSolution.Location.X, 0.01);
        Assert.Equal(0, aSolution.Location.Y, 0.01);
        Assert.Equal(2, aSolution.Radius, 0.01);
    }

    [Theory]
    [MemberData(nameof(ThreeTangentForms))]
    public void Three_tangents_accept_a_qualifier_the_answer_satisfies_or_refuse_it(int theForm)
    {
        // the answer has every argument outside it; the arguments curve so that their inner side faces it, whichever
        // way the qualifiers read, one of the three is refused
        int aDone = 0;
        foreach (Qualifier aQ in new[] { Qualifier.Outside, Qualifier.Enclosed, Qualifier.Enclosing })
        {
            Circ2d3TanIter aSolver;
            try
            {
                aSolver = Three(theForm, aQ);
            }
            catch (BadQualifierException)
            {
                continue; // a line can't be enclosing
            }
            if (aSolver.IsDone)
            {
                aDone++;
                AssertAnswer(aSolver.ThisSolution());
            }
            else
            {
                Assert.Throws<Kerf.Engine.Foundation.NotDoneException>(() => aSolver.Tangency1(out _, out _, out _));
                Assert.Throws<Kerf.Engine.Foundation.NotDoneException>(() => aSolver.Tangency2(out _, out _, out _));
                Assert.Throws<Kerf.Engine.Foundation.NotDoneException>(() => aSolver.Tangency3(out _, out _, out _));
                Assert.Throws<Kerf.Engine.Foundation.NotDoneException>(() => aSolver.WhichQualifier(out _, out _, out _));
                Assert.Throws<Kerf.Engine.Foundation.NotDoneException>(() => aSolver.IsTheSame1());
                Assert.Throws<Kerf.Engine.Foundation.NotDoneException>(() => aSolver.IsTheSame2());
            }
        }
        Assert.True(aDone < 3);
    }

    [Fact]
    public void Three_tangents_refuse_a_qualifier_that_is_none()
    {
        Assert.Throws<BadQualifierException>(() => Three(0, Qualifier.Noqualifier));
        Assert.Throws<BadQualifierException>(() => Three(1, Qualifier.Noqualifier));
        Assert.Throws<BadQualifierException>(() => Three(3, Qualifier.Noqualifier));
        Assert.Throws<BadQualifierException>(() => Three(5, Qualifier.Noqualifier));
    }

    // Two tangents with the centre on something, always the origin: on the line y = x at parameter 0, the circle of radius
    // 3 about (3, 0) at angle pi, or a Bezier curve along y = x at parameter 0.5

    private static readonly Line2D Diagonal = new(new Point2D(0, 0), new Direction2D(1, 1));
    private static readonly Circle2D Through = new(new Frame2D(new Point2D(3, 0), new Direction2D(1, 0)), 3);

    private static CurveAdaptor2D CurveOn() => Bow(-3, -3, 0, 0, 3, 3);

    public static TheoryData<int> TwoTangentForms() => new() { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

    private static Circ2d2TanOnIter TwoOn(int theForm, Qualifier theQ, double theStartOff = 0)
    {
        double d = theStartOff;
        return theForm switch
        {
            0 => new Circ2d2TanOnIter(Q(Above, theQ), Q(BowBelow(), theQ), Diagonal, 0 + d, 0.5 + d, 0 + d, 1e-9),
            1 => new Circ2d2TanOnIter(Q(BowLeft(), theQ), Q(BowBelow(), theQ), Diagonal, 0.5 + d, 0.5 + d, 0 + d, 1e-9),
            2 => new Circ2d2TanOnIter(Q(BowLeft(), theQ), OnArcA, Diagonal, 0.5 + d, 0 + d, 1e-9),
            3 => new Circ2d2TanOnIter(Q(Right, theQ), Q(BowBelow(), theQ), Diagonal, Math.PI + d, 0.5 + d, 0 + d, 1e-9),
            4 => new Circ2d2TanOnIter(Q(Right, theQ), Q(BowBelow(), theQ), Through, Math.PI + d, 0.5 + d, Math.PI + d, 1e-9),
            5 => new Circ2d2TanOnIter(Q(Above, theQ), Q(BowBelow(), theQ), Through, 0 + d, 0.5 + d, Math.PI + d, 1e-9),
            6 => new Circ2d2TanOnIter(Q(BowLeft(), theQ), Q(BowBelow(), theQ), Through, 0.5 + d, 0.5 + d, Math.PI + d, 1e-9),
            7 => new Circ2d2TanOnIter(Q(BowLeft(), theQ), OnArcA, Through, 0.5 + d, Math.PI + d, 1e-9),
            8 => new Circ2d2TanOnIter(Q(BowLeft(), theQ), Q(BowBelow(), theQ), CurveOn(), 0.5 + d, 0.5 + d, 0.5 + d, 1e-9),
            9 => new Circ2d2TanOnIter(Q(Right, theQ), Q(BowBelow(), theQ), CurveOn(), Math.PI + d, 0.5 + d, 0.5 + d, 1e-9),
            10 => new Circ2d2TanOnIter(Q(Above, theQ), Q(BowBelow(), theQ), CurveOn(), 0 + d, 0.5 + d, 0.5 + d, 1e-9),
            _ => new Circ2d2TanOnIter(Q(BowLeft(), theQ), OnArcA, CurveOn(), 0.5 + d, 0.5 + d, 1e-9),
        };
    }

    private static void AssertSolved(Circ2d2TanOnIter theSolver)
    {
        Assert.True(theSolver.IsDone);
        Circle2D aSolution = theSolver.ThisSolution();
        AssertAnswer(aSolution);
        theSolver.Tangency1(out double aSol1, out _, out Point2D aP1);
        AssertTangency(aSolution, aSol1, aP1);
        theSolver.Tangency2(out double aSol2, out _, out Point2D aP2);
        AssertTangency(aSolution, aSol2, aP2);
        theSolver.CenterOn3(out _, out Point2D aCentre);
        Assert.True(aCentre.DistanceTo(aSolution.Location) < Tol);
        Assert.False(theSolver.IsTheSame1());
    }

    [Theory]
    [MemberData(nameof(TwoTangentForms))]
    public void Two_tangents_and_a_centre_keep_the_answer_they_start_on(int theForm)
    {
        AssertSolved(TwoOn(theForm, Qualifier.Unqualified));
    }

    [Theory]
    [MemberData(nameof(TwoTangentForms))]
    public void Two_tangents_and_a_centre_come_back_from_nearby(int theForm)
    {
        Circ2d2TanOnIter aSolver = TwoOn(theForm, Qualifier.Unqualified, 0.01);
        Assert.True(aSolver.IsDone);
        Circle2D aSolution = aSolver.ThisSolution();
        Assert.Equal(0, aSolution.Location.X, 0.1);
        Assert.Equal(0, aSolution.Location.Y, 0.1);
        Assert.Equal(2, aSolution.Radius, 0.1);
    }

    [Theory]
    [MemberData(nameof(TwoTangentForms))]
    public void Two_tangents_and_a_centre_read_their_qualifiers(int theForm)
    {
        foreach (Qualifier aQ in new[] { Qualifier.Outside, Qualifier.Enclosed })
        {
            Circ2d2TanOnIter aSolver = TwoOn(theForm, aQ);
            if (aSolver.IsDone)
            {
                AssertAnswer(aSolver.ThisSolution());
                aSolver.WhichQualifier(out Qualifier aQ1, out _);
                Assert.Equal(aQ, aQ1);
            }
            else
            {
                Assert.Throws<Kerf.Engine.Foundation.NotDoneException>(() => aSolver.Tangency1(out _, out _, out _));
                Assert.Throws<Kerf.Engine.Foundation.NotDoneException>(() => aSolver.Tangency2(out _, out _, out _));
                Assert.Throws<Kerf.Engine.Foundation.NotDoneException>(() => aSolver.CenterOn3(out _, out _));
                Assert.Throws<Kerf.Engine.Foundation.NotDoneException>(() => aSolver.IsTheSame1());
            }
        }
    }

    [Fact]
    public void Two_tangents_and_a_centre_refuse_a_qualifier_that_is_none()
    {
        foreach (int aForm in new[] { 0, 1, 2, 3, 4, 5, 8, 9, 10, 11 })
        {
            Assert.Throws<BadQualifierException>(() => TwoOn(aForm, Qualifier.Noqualifier));
        }
    }

    // Lines tangent to two things: y = 1, over a circle of radius 1 about the origin and two parabolas

    private static readonly Circle2D UnitCircle = new(new Frame2D(new Point2D(0, 0), new Direction2D(1, 0)), 1);

    private static CurveAdaptor2D Dip(double theX) => Bow(theX - 2, 2, theX, 0, theX + 2, 2);

    private static void AssertLevelLine(Lin2d2TanIter theSolver)
    {
        Assert.True(theSolver.IsDone);
        Line2D aLine = theSolver.ThisSolution();
        Assert.Equal(0, aLine.Direction.Y, 1e-6);
        Assert.Equal(1, aLine.Location.Y, 1e-6);
        theSolver.Tangency1(out double aParSol, out _, out Point2D aP1);
        Assert.True(ElCLib.Value(aParSol, aLine).DistanceTo(aP1) < 1e-6);
        theSolver.Tangency2(out double aParSol2, out _, out Point2D aP2);
        // on the line; OCCT gives the parameter unsigned, whichever way the line runs (occt-issues.md)
        Assert.True(ElCLib.Value(ElCLib.Parameter(aLine, aP2), aLine).DistanceTo(aP2) < 1e-6);
        Assert.Equal(aLine.Location.DistanceTo(aP2), Math.Abs(aParSol2), 1e-6);
        theSolver.WhichQualifier(out Qualifier aQ1, out Qualifier aQ2);
        Assert.NotEqual(Qualifier.Noqualifier, aQ1);
        Assert.NotEqual(Qualifier.Noqualifier, aQ2);
    }

    [Fact]
    public void A_line_tangent_to_a_circle_and_a_curve()
    {
        AssertLevelLine(new Lin2d2TanIter(Q(UnitCircle, Qualifier.Unqualified), Q(Dip(5), Qualifier.Unqualified), 0.5, 1e-9));
        AssertLevelLine(new Lin2d2TanIter(Q(UnitCircle, Qualifier.Unqualified), Q(Dip(5), Qualifier.Unqualified), 0.51, 1e-9));
        foreach (Qualifier aQ in new[] { Qualifier.Outside, Qualifier.Enclosing })
        {
            var aSolver = new Lin2d2TanIter(Q(UnitCircle, aQ), Q(Dip(5), aQ), 0.5, 1e-9);
            if (aSolver.IsDone)
            {
                AssertLevelLine(aSolver);
            }
        }
    }

    [Fact]
    public void A_line_tangent_to_two_curves()
    {
        AssertLevelLine(new Lin2d2TanIter(Q(Dip(-5), Qualifier.Unqualified), Q(Dip(5), Qualifier.Unqualified), 0.5, 0.5, 1e-9));
        Assert.Throws<BadQualifierException>(() => new Lin2d2TanIter(Q(Dip(-5), Qualifier.Noqualifier), Q(Dip(5), Qualifier.Unqualified), 0.5, 0.5, 1e-9));
    }

    [Fact]
    public void A_line_from_a_point_tangent_to_a_curve()
    {
        AssertLevelLine2(new Lin2d2TanIter(Q(Dip(5), Qualifier.Unqualified), new Point2D(-5, 1), 0.5, 1e-9));
        Assert.Throws<BadQualifierException>(() => new Lin2d2TanIter(Q(Dip(5), Qualifier.Noqualifier), new Point2D(-5, 1), 0.5, 1e-9));
    }

    private static void AssertLevelLine2(Lin2d2TanIter theSolver)
    {
        Assert.True(theSolver.IsDone);
        Line2D aLine = theSolver.ThisSolution();
        Assert.Equal(0, aLine.Direction.Y, 1e-6);
        Assert.Equal(1, aLine.Location.Y, 1e-6);
    }

    // Circles of a given radius, their centre on something, tangent to a curve

    private static void AssertTangentAtRadius(Circ2dTanOnRadGeo theSolver, double theRadius, Func<Point2D, double> theOn)
    {
        Assert.True(theSolver.IsDone);
        Assert.True(theSolver.SolutionCount >= 1);
        for (int i = 1; i <= theSolver.SolutionCount; i++)
        {
            Circle2D aCircle = theSolver.ThisSolution(i);
            Assert.Equal(theRadius, aCircle.Radius, Tol);
            Assert.Equal(0, theOn(aCircle.Location), 1e-5);
            if (!theSolver.IsTheSame1(i))
            {
                theSolver.Tangency1(i, out double aParSol, out _, out Point2D aP);
                Assert.True(ElCLib.Value(aParSol, aCircle).DistanceTo(aP) < 1e-5);
            }
            theSolver.CenterOn3(i, out _, out Point2D aCentre);
            Assert.True(aCentre.DistanceTo(aCircle.Location) < Tol);
            theSolver.WhichQualifier(i, out _);
        }
        Assert.Throws<Kerf.Engine.Foundation.OutOfRangeException>(() => theSolver.ThisSolution(theSolver.SolutionCount + 1));
    }

    [Theory]
    [InlineData("Unqualified")]
    [InlineData("Outside")]
    [InlineData("Enclosed")]
    public void Tangent_to_a_curve_with_its_centre_on_a_line(string theQualifier)
    {
        Qualifier theQ = Enum.Parse<Qualifier>(theQualifier);
        // the parabola's lowest point is at (0, 0.. ): the line is the y axis
        var aLine = new Line2D(new Point2D(0, -10), new Direction2D(0, 1));
        var aSolver = new Circ2dTanOnRadGeo(Q(BowBelow(), theQ), aLine, 0.5, 1e-7);
        if (aSolver.IsDone && aSolver.SolutionCount > 0)
        {
            AssertTangentAtRadius(aSolver, 0.5, p => p.X);
        }
        Assert.Equal(theQ == Qualifier.Unqualified, aSolver.IsDone && aSolver.SolutionCount >= 2);
    }

    [Theory]
    [InlineData("Unqualified")]
    [InlineData("Outside")]
    public void Tangent_to_a_curve_with_its_centre_on_a_circle(string theQualifier)
    {
        Qualifier theQ = Enum.Parse<Qualifier>(theQualifier);
        var aSolver = new Circ2dTanOnRadGeo(Q(BowBelow(), theQ), UnitCircle, 0.5, 1e-7);
        if (aSolver.IsDone && aSolver.SolutionCount > 0)
        {
            AssertTangentAtRadius(aSolver, 0.5, p => new Point2D(0, 0).DistanceTo(p) - 1);
        }
        Assert.True(aSolver.IsDone);
    }

    [Fact]
    public void Tangent_to_a_line_circle_curve_or_point_with_its_centre_on_a_curve()
    {
        CurveAdaptor2D anOn = Bow(-5, 0, 0, 0, 5, 0);
        double OnIt(Point2D p) => p.Y;
        AssertTangentAtRadius(new Circ2dTanOnRadGeo(Q(UnitCircle, Qualifier.Unqualified), anOn, 0.5, 1e-7), 0.5, OnIt);
        AssertTangentAtRadius(new Circ2dTanOnRadGeo(Q(new Line2D(new Point2D(1, 0), new Direction2D(0, 1)), Qualifier.Unqualified), anOn, 0.5, 1e-7), 0.5, OnIt);
        AssertTangentAtRadius(new Circ2dTanOnRadGeo(Q(BowRight(), Qualifier.Unqualified), anOn, 0.5, 1e-7), 0.5, OnIt);
        AssertTangentAtRadius(new Circ2dTanOnRadGeo(new Point2D(1, 0), anOn, 0.5, 1e-7), 0.5, OnIt);
    }

    [Fact]
    public void A_negative_radius_or_a_missing_qualifier_is_refused()
    {
        var aLine = new Line2D(new Point2D(0, -10), new Direction2D(0, 1));
        Assert.Throws<BadQualifierException>(() => new Circ2dTanOnRadGeo(Q(BowBelow(), Qualifier.Noqualifier), aLine, 0.5, 1e-7));
        Assert.Throws<BadQualifierException>(() => new Circ2dTanOnRadGeo(Q(BowBelow(), Qualifier.Noqualifier), UnitCircle, 0.5, 1e-7));
        Assert.Throws<BadQualifierException>(() => new Circ2dTanOnRadGeo(Q(BowBelow(), Qualifier.Noqualifier), BowLeft(), 0.5, 1e-7));
        Assert.Throws<BadQualifierException>(() => new Circ2dTanOnRadGeo(Q(UnitCircle, Qualifier.Noqualifier), BowLeft(), 0.5, 1e-7));
        Assert.Throws<BadQualifierException>(() => new Circ2dTanOnRadGeo(Q(aLine, Qualifier.Noqualifier), BowLeft(), 0.5, 1e-7));
        Assert.Throws<Kerf.Engine.Foundation.NegativeValueException>(() => new Circ2dTanOnRadGeo(Q(BowBelow(), Qualifier.Unqualified), aLine, -0.5, 1e-7));
        Assert.Throws<Kerf.Engine.Foundation.NegativeValueException>(() => new Circ2dTanOnRadGeo(Q(BowBelow(), Qualifier.Unqualified), UnitCircle, -0.5, 1e-7));
    }

    // The function solved when a circle is tangent to a curve

    [Theory]
    [InlineData(0.1)]
    [InlineData(0.5)]
    [InlineData(0.8)]
    public void The_function_of_a_circle_and_a_curve_has_the_derivative_it_says(double theX)
    {
        var aFunction = new FunctionTanCirCu(Right, BowBelow());
        const double h = 1e-6;
        Assert.True(aFunction.Value(theX, out double aF));
        Assert.True(aFunction.Value(theX + h, out double aFp));
        Assert.True(aFunction.Value(theX - h, out double aFm));
        Assert.True(aFunction.Derivative(theX, out double aD));
        Assert.Equal((aFp - aFm) / (2 * h), aD, 1e-5);
        Assert.True(aFunction.Values(theX, out double aF2, out double aD2));
        Assert.Equal(aF, aF2, 1e-12);
        Assert.Equal(aD, aD2, 1e-12);
    }

    [Fact]
    public void Qualified_curves_are_values_of_their_curve_and_qualifier()
    {
        CurveAdaptor2D aCurve = BowLeft();
        var aOne = new QCurve(aCurve, Qualifier.Outside);
        var aTwo = new QCurve(aCurve, Qualifier.Outside);
        Assert.True(aOne == aTwo);
        Assert.False(aOne != aTwo);
        Assert.Equal(aOne.GetHashCode(), aTwo.GetHashCode());
        Assert.True(aOne != new QCurve(aCurve, Qualifier.Enclosed));
        Assert.True(aOne != new QCurve(BowRight(), Qualifier.Outside));
        Assert.False(aOne.Equals("no"));
        Assert.Contains("Outside", aOne.ToString(), StringComparison.OrdinalIgnoreCase);
        Assert.Equal(Qualifier.Outside, aOne.Qualifier());

        QualifiedCurve aQ1 = Geom2dGcc.Outside(aCurve), aQ2 = Geom2dGcc.Outside(aCurve);
        Assert.True(aQ1 == aQ2);
        Assert.False(aQ1 != aQ2);
        Assert.Equal(aQ1.GetHashCode(), aQ2.GetHashCode());
        Assert.True(aQ1 != Geom2dGcc.Enclosed(aCurve));
        Assert.True(aQ1 != Geom2dGcc.Enclosing(aCurve));
        Assert.True(aQ1 != Geom2dGcc.Unqualified(aCurve));
        Assert.False(aQ1.Equals("no"));
        Assert.Contains("Outside", aQ1.ToString(), StringComparison.OrdinalIgnoreCase);
        Assert.Equal(Qualifier.Outside, aQ1.Qualifier());
        Assert.True(aQ1.IsOutside);
    }
}
