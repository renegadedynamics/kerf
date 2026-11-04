// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Constraints.Analytic;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// GccAna_Circ2d2TanOn: circles tangent to two lines, circles or points with their centre on a line or a circle. Each
/// constructor on configurations whose solutions are known, then every solution checked: the centre on the third
/// curve, tangency to (or passage through) each argument, the tangency points and their parameters.
/// </summary>
public class GccAna_Circ2d2TanOnTests
{
    private const double Tol = 1.0e-7;

    private static Line2D Lin(double x, double y, double dx, double dy) => new(new Point2D(x, y), new Direction2D(dx, dy));

    private static Circle2D Circ(double x, double y, double r) => new(new Axis2D(new Point2D(x, y), new Direction2D(1, 0)), r);

    // The solution touches the argument (a line, a circle) or passes through it (a point).
    private static void AssertTouches(object theArg, in Circle2D theSol)
    {
        Point2D c = theSol.Location;
        double r = theSol.Radius;
        switch (theArg)
        {
            case Line2D l:
                Assert.Equal(r, l.DistanceTo(c), 1.0e-6);
                break;
            case Circle2D ci:
                double d = c.DistanceTo(ci.Location);
                double aGap = Math.Min(Math.Abs(d - (r + ci.Radius)), Math.Abs(d - Math.Abs(r - ci.Radius)));
                Assert.True(aGap < 1.0e-6, $"solution {theSol} not tangent to {ci}: gap {aGap}");
                break;
            case Point2D p:
                Assert.Equal(r, c.DistanceTo(p), 1.0e-6);
                break;
        }
    }

    // The tangency point is on the solution and on the argument, and the parameters evaluate to it.
    private static void AssertTangencyPoint(object theArg, in Circle2D theSol, double theParSol, double theParArg,
                                            in Point2D thePnt, bool theCheckParArg = true)
    {
        Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(theParSol, theSol).DistanceTo(thePnt) < Tol, "ParSol");
        switch (theArg)
        {
            case Line2D l:
                Assert.True(l.DistanceTo(thePnt) < Tol, "point on the line");
                if (theCheckParArg)
                {
                    Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(theParArg, l).DistanceTo(thePnt) < Tol, "ParArg");
                }
                break;
            case Circle2D ci:
                Assert.True(Math.Abs(ci.Location.DistanceTo(thePnt) - ci.Radius) < Tol, "point on the circle");
                if (theCheckParArg)
                {
                    Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(theParArg, ci).DistanceTo(thePnt) < Tol, "ParArg");
                }
                break;
            case Point2D p:
                Assert.True(p.DistanceTo(thePnt) < Tol, "the point itself");
                Assert.Equal(0.0, theParArg);
                break;
        }
    }

    // Every solution: centre on the third curve (through CenterOn3's parameter), tangent to both arguments, and
    // the tangency points, unless the solution is the argument itself.
    private static void CheckAll(AnalyticCirc2d2TanOn theAlgo, object theArg1, object theArg2, object theOn,
                                 bool theCheckParArg1 = true, bool theCheckPnt1 = true)
    {
        Assert.True(theAlgo.IsDone);
        for (int i = 1; i <= theAlgo.SolutionCount; i++)
        {
            Circle2D aSol = theAlgo.ThisSolution(i);
            AssertTouches(theArg1, aSol);
            AssertTouches(theArg2, aSol);

            theAlgo.CenterOn3(i, out double aParOn, out Point2D aPntOn);
            Point2D aCenter = aSol.Location;
            // CenterOn3 gives the centre (OCCT gave the first tangency point: decision 1541)
            Assert.True(aPntOn.DistanceTo(aCenter) < 1.0e-9, "CenterOn3 point");
            switch (theOn)
            {
                case Line2D l:
                    Assert.True(l.DistanceTo(aCenter) < 1.0e-6, "centre on the line");
                    Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aParOn, l).DistanceTo(aCenter) < 1.0e-6, "CenterOn3 parameter");
                    break;
                case Circle2D c:
                    Assert.True(Math.Abs(c.Location.DistanceTo(aCenter) - c.Radius) < 1.0e-6, "centre on the circle");
                    Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aParOn, c).DistanceTo(aCenter) < 1.0e-6, "CenterOn3 parameter");
                    break;
            }

            if (!theAlgo.IsTheSame1(i))
            {
                theAlgo.Tangency1(i, out double aParSol, out double aParArg, out Point2D aPnt);
                if (theCheckPnt1)
                {
                    AssertTangencyPoint(theArg1, aSol, aParSol, aParArg, aPnt, theCheckParArg1);
                }
            }
            else
            {
                EngineAssert.Throws<NotDoneException>(() => theAlgo.Tangency1(i, out _, out _, out _));
            }
            if (!theAlgo.IsTheSame2(i))
            {
                theAlgo.Tangency2(i, out double aParSol, out double aParArg, out Point2D aPnt);
                AssertTangencyPoint(theArg2, aSol, aParSol, aParArg, aPnt);
            }
            theAlgo.WhichQualifier(i, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ1, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ2);
            Assert.NotEqual(global::Kerf.Engine.Construction.Constraints.Qualifier.Unqualified, aQ1);
            Assert.NotEqual(global::Kerf.Engine.Construction.Constraints.Qualifier.Unqualified, aQ2);
        }
    }

    // The qualifier of a solution relative to a line: outside if the line's origin is on the left of the solution's
    // centre (the centre on the line's right), enclosed otherwise.
    private static global::Kerf.Engine.Construction.Constraints.Qualifier SideOf(in Line2D theLin, in Point2D theCenter)
    {
        Direction2D d = theLin.Direction;
        double aCross = (theLin.Location.X - theCenter.X) * (-d.Y) + (theLin.Location.Y - theCenter.Y) * d.X;
        return aCross > 0.0 ? global::Kerf.Engine.Construction.Constraints.Qualifier.Outside : global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed;
    }

    // ===== Centre on a line =====

    [Fact]
    public void TwoPoints_OnLine_OneSolution()
    {
        Point2D p1 = new(0, 0), p2 = new(2, 0);
        Line2D on = Lin(0, 0, 1, 0);
        AnalyticCirc2d2TanOn a = new(p1, p2, on, Tol);
        Assert.True(a.IsDone);
        Assert.Equal(1, a.SolutionCount);
        Assert.True(a.ThisSolution(1).Location.DistanceTo(new Point2D(1, 0)) < Tol);
        Assert.Equal(1.0, a.ThisSolution(1).Radius, 1.0e-12);
        a.WhichQualifier(1, out global::Kerf.Engine.Construction.Constraints.Qualifier q1, out global::Kerf.Engine.Construction.Constraints.Qualifier q2);
        Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Noqualifier, q1);
        Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Noqualifier, q2);
        CheckAll(a, p1, p2, on);
    }

    [Fact]
    public void TwoPoints_OnLine_ParallelToBisector_NoSolution()
    {
        AnalyticCirc2d2TanOn a = new(new Point2D(0, 0), new Point2D(2, 0), Lin(5, 0, 0, 1), Tol);
        Assert.True(a.IsDone);
        Assert.Equal(0, a.SolutionCount);
    }

    [Fact]
    public void TwoPoints_Coincident_NoSolution()
    {
        AnalyticCirc2d2TanOn a = new(new Point2D(1, 1), new Point2D(1, 1), Lin(0, 0, 1, 0), Tol);
        Assert.True(a.IsDone);
        Assert.Equal(0, a.SolutionCount);
    }

    [Fact]
    public void TwoSecantLines_OnLine_TwoSolutionsAndQualifiers()
    {
        Line2D l1 = Lin(0, 0, 1, 0); // inside: y > 0
        Line2D l2 = Lin(0, 0, 0, 1); // inside: x < 0
        Line2D on = Lin(0, 4, 1, 0);
        AnalyticCirc2d2TanOn a = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(l1), global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(l2), on, Tol);
        Assert.Equal(2, a.SolutionCount);
        for (int i = 1; i <= 2; i++)
        {
            Assert.Equal(4.0, a.ThisSolution(i).Radius, 1.0e-9);
            a.WhichQualifier(i, out global::Kerf.Engine.Construction.Constraints.Qualifier q1, out global::Kerf.Engine.Construction.Constraints.Qualifier q2);
            Assert.Equal(SideOf(l1, a.ThisSolution(i).Location), q1);
            Assert.Equal(SideOf(l2, a.ThisSolution(i).Location), q2);
        }
        CheckAll(a, l1, l2, on);

        // enclosed by both: only the centre (-4, 4)
        AnalyticCirc2d2TanOn e = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosed(l1), global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosed(l2), on, Tol);
        Assert.Equal(1, e.SolutionCount);
        Assert.True(e.ThisSolution(1).Location.DistanceTo(new Point2D(-4, 4)) < 1.0e-9);
        CheckAll(e, l1, l2, on);

        // outside the second line: the centre (4, 4)
        AnalyticCirc2d2TanOn o = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosed(l1), global::Kerf.Engine.Construction.Constraints.GccEnt.Outside(l2), on, Tol);
        Assert.Equal(1, o.SolutionCount);
        Assert.True(o.ThisSolution(1).Location.DistanceTo(new Point2D(4, 4)) < 1.0e-9);

        // outside both: the centres are above the first line, so none
        AnalyticCirc2d2TanOn n = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Outside(l1), global::Kerf.Engine.Construction.Constraints.GccEnt.Outside(l2), on, Tol);
        Assert.Equal(0, n.SolutionCount);
    }

    [Fact]
    public void TwoParallelLines_OnTransverseLine_OneSolution()
    {
        Line2D l1 = Lin(0, 0, 1, 0);
        Line2D l2 = Lin(0, 2, 1, 0);
        Line2D on = Lin(3, 0, 0, 1);
        AnalyticCirc2d2TanOn a = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(l1), global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(l2), on, Tol);
        Assert.Equal(1, a.SolutionCount);
        Assert.True(a.ThisSolution(1).Location.DistanceTo(new Point2D(3, 1)) < 1.0e-9);
        Assert.Equal(1.0, a.ThisSolution(1).Radius, 1.0e-9);
        a.WhichQualifier(1, out global::Kerf.Engine.Construction.Constraints.Qualifier q1, out global::Kerf.Engine.Construction.Constraints.Qualifier q2);
        Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed, q1);
        Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, q2);
        CheckAll(a, l1, l2, on);

        // the solution is enclosed by the first line and outside the second
        Assert.Equal(0, new AnalyticCirc2d2TanOn(global::Kerf.Engine.Construction.Constraints.GccEnt.Outside(l1), global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(l2), on, Tol).SolutionCount);
        Assert.Equal(1, new AnalyticCirc2d2TanOn(global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosed(l1), global::Kerf.Engine.Construction.Constraints.GccEnt.Outside(l2), on, Tol).SolutionCount);
    }

    [Fact]
    public void LinePoint_OnLine_Parabola()
    {
        // focus (0, 2), directrix y = 0: x^2 = 4 (y - 1); on x = 2 the centre is (2, 2) with radius 2
        Line2D l1 = Lin(0, 0, 1, 0);
        Point2D p = new(0, 2);
        Line2D on = Lin(2, 0, 0, 1);
        AnalyticCirc2d2TanOn a = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(l1), p, on, Tol);
        Assert.Equal(1, a.SolutionCount);
        Assert.True(a.ThisSolution(1).Location.DistanceTo(new Point2D(2, 2)) < 1.0e-9);
        Assert.Equal(2.0, a.ThisSolution(1).Radius, 1.0e-9);
        a.WhichQualifier(1, out global::Kerf.Engine.Construction.Constraints.Qualifier q1, out global::Kerf.Engine.Construction.Constraints.Qualifier q2);
        Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed, q1);
        Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Noqualifier, q2);
        CheckAll(a, l1, p, on);
        Assert.Equal(0, new AnalyticCirc2d2TanOn(global::Kerf.Engine.Construction.Constraints.GccEnt.Outside(l1), p, on, Tol).SolutionCount);
        Assert.Equal(1, new AnalyticCirc2d2TanOn(global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosed(l1), p, on, Tol).SolutionCount);
    }

    [Fact]
    public void LinePoint_OnLine_TwoCrossings()
    {
        // the parabola x^2 = 4 (y - 1) crossed by the line y = 5: x = +-4, radius 5
        Line2D l1 = Lin(0, 0, 1, 0);
        Point2D p = new(0, 2);
        Line2D on = Lin(0, 5, 1, 0);
        AnalyticCirc2d2TanOn a = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(l1), p, on, Tol);
        Assert.Equal(2, a.SolutionCount);
        for (int i = 1; i <= 2; i++)
        {
            Assert.Equal(5.0, a.ThisSolution(i).Radius, 1.0e-9);
        }
        CheckAll(a, l1, p, on);
    }

    [Fact]
    public void CirclePoint_OnLine()
    {
        Circle2D c1 = Circ(0, 0, 1);
        Point2D p = new(3, 0);
        Line2D on = Lin(0, 2, 1, 0);
        // unqualified: every solution passes through the point
        AnalyticCirc2d2TanOn u = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(c1), p, on, Tol);
        Assert.True(u.SolutionCount >= 2);
        // OCCT stored the parameter of the first tangency point on C1 at the bisector's index (decision 1541)
        CheckAll(u, c1, p, on);
        foreach (global::Kerf.Engine.Construction.Constraints.QualifiedCirc q in new[] { global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(c1), global::Kerf.Engine.Construction.Constraints.GccEnt.Outside(c1), global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosing(c1) })
        {
            AnalyticCirc2d2TanOn a = new(q, p, on, Tol);
            Assert.True(a.SolutionCount >= 1, q.ToString());
            // OCCT took the radius from the qualifier on both branches of the bisector, so only some of the solutions
            // passed through the point; all do now (decision 1541)
            int aNbThrough = 0;
            for (int i = 1; i <= a.SolutionCount; i++)
            {
                Circle2D s = a.ThisSolution(i);
                AssertTouches(c1, s);
                Assert.True(Math.Abs(on.DistanceTo(s.Location)) < 1.0e-6);
                aNbThrough += Math.Abs(s.Location.DistanceTo(p) - s.Radius) < 1.0e-6 ? 1 : 0;
            }
            Assert.Equal(a.SolutionCount, aNbThrough);
            for (int i = 1; i <= a.SolutionCount; i++)
            {
                a.WhichQualifier(i, out global::Kerf.Engine.Construction.Constraints.Qualifier q1, out global::Kerf.Engine.Construction.Constraints.Qualifier q2);
                Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Noqualifier, q2);
                double d = a.ThisSolution(i).Location.DistanceTo(c1.Location);
                double r = a.ThisSolution(i).Radius;
                if (q1 == global::Kerf.Engine.Construction.Constraints.Qualifier.Outside)
                {
                    Assert.Equal(r + 1.0, d, 1.0e-6);
                }
                else if (q1 == global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing)
                {
                    Assert.Equal(r - 1.0, d, 1.0e-6);
                }
                if (!q.IsUnqualified)
                {
                    Assert.Equal(q.Qualifier(), q1);
                }
            }
        }
        // the point is outside the circle: no solution inside it
        Assert.Equal(0, new AnalyticCirc2d2TanOn(global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosed(c1), p, on, Tol).SolutionCount);
    }

    [Fact]
    public void CircleLine_OnLine()
    {
        Circle2D c1 = Circ(0, 3, 1);
        Line2D l2 = Lin(0, 0, 1, 0);
        Line2D on = Lin(2, 0, 0, 1);
        AnalyticCirc2d2TanOn a = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(c1), global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(l2), on, Tol);
        Assert.True(a.SolutionCount >= 2);
        CheckAll(a, c1, l2, on);
        int aNbOut = 0;
        for (int i = 1; i <= a.SolutionCount; i++)
        {
            a.WhichQualifier(i, out global::Kerf.Engine.Construction.Constraints.Qualifier q1, out global::Kerf.Engine.Construction.Constraints.Qualifier q2);
            Assert.Equal(SideOf(l2, a.ThisSolution(i).Location), q2);
            aNbOut += q1 == global::Kerf.Engine.Construction.Constraints.Qualifier.Outside ? 1 : 0;
        }
        AnalyticCirc2d2TanOn o = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Outside(c1), global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosed(l2), on, Tol);
        Assert.Equal(aNbOut, o.SolutionCount);
        CheckAll(o, c1, l2, on);
    }

    [Fact]
    public void TwoCircles_OnLine()
    {
        Circle2D c1 = Circ(-3, 0, 1);
        Circle2D c2 = Circ(3, 0, 2);
        // outside both: d2 - d1 = 1, so on x = -1 (two centres, symmetric about the x axis)
        Line2D on = Lin(-1, 0, 0, 1);
        AnalyticCirc2d2TanOn a = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Outside(c1), global::Kerf.Engine.Construction.Constraints.GccEnt.Outside(c2), on, Tol);
        Assert.True(a.SolutionCount >= 1);
        CheckAll(a, c1, c2, on, theCheckParArg1: false);
        for (int i = 1; i <= a.SolutionCount; i++)
        {
            Circle2D s = a.ThisSolution(i);
            Assert.Equal(s.Radius + 1.0, s.Location.DistanceTo(c1.Location), 1.0e-6);
            Assert.Equal(s.Radius + 2.0, s.Location.DistanceTo(c2.Location), 1.0e-6);
        }
        // enclosing both: d1 - d2 = 1, so on x = 1
        Line2D on2 = Lin(1, 0, 0, 1);
        AnalyticCirc2d2TanOn e = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosing(c1), global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosing(c2), on2, Tol);
        Assert.True(e.SolutionCount >= 1);
        CheckAll(e, c1, c2, on2, theCheckParArg1: false);
        for (int i = 1; i <= e.SolutionCount; i++)
        {
            Circle2D s = e.ThisSolution(i);
            Assert.Equal(s.Radius - 1.0, s.Location.DistanceTo(c1.Location), 1.0e-6);
            Assert.Equal(s.Radius - 2.0, s.Location.DistanceTo(c2.Location), 1.0e-6);
        }
    }

    [Fact]
    public void TwoCircles_OnLine_SymmetricBoundaryCase()
    {
        // equal circles symmetric about the y axis, centre line the x axis through both centres: the boundary case
        Circle2D c1 = Circ(-3, 0, 1);
        Circle2D c2 = Circ(3, 0, 1);
        Line2D on = Lin(0, -5, 0, 1);
        AnalyticCirc2d2TanOn a = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Outside(c1), global::Kerf.Engine.Construction.Constraints.GccEnt.Outside(c2), on, Tol);
        Assert.True(a.SolutionCount >= 1);
        CheckAll(a, c1, c2, on);
    }

    // ===== Centre on a circle =====

    [Fact]
    public void TwoPoints_OnCircle_TwoSolutions()
    {
        Point2D p1 = new(-1, 0), p2 = new(1, 0);
        Circle2D on = Circ(0, 0, 2);
        AnalyticCirc2d2TanOn a = new(p1, p2, on, Tol);
        Assert.Equal(2, a.SolutionCount);
        for (int i = 1; i <= 2; i++)
        {
            Assert.Equal(Math.Sqrt(5.0), a.ThisSolution(i).Radius, 1.0e-9);
        }
        CheckAll(a, p1, p2, on);
    }

    [Fact]
    public void TwoPoints_OnCircle_Missing_NoSolution()
    {
        AnalyticCirc2d2TanOn a = new(new Point2D(-1, 10), new Point2D(1, 10), Circ(10, 0, 2), Tol);
        Assert.True(a.IsDone);
        Assert.Equal(0, a.SolutionCount);
    }

    [Fact]
    public void TwoLines_OnCircle_Unqualified()
    {
        // two secant lines turning counter-clockwise (OCCT's signed angle test sends the clockwise ones to its
        // parallel branch)
        Line2D l1 = Lin(0, 0, 1, 0);
        Line2D l2 = Lin(0, 0, 0, 1);
        Circle2D on = Circ(0, 0, 3);
        AnalyticCirc2d2TanOn a = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(l1), global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(l2), on, Tol);
        Assert.True(a.IsDone);
        Assert.True(a.SolutionCount >= 1);
        for (int i = 1; i <= a.SolutionCount; i++)
        {
            Circle2D s = a.ThisSolution(i);
            Assert.True(Math.Abs(s.Location.DistanceTo(on.Location) - 3.0) < 1.0e-6, "centre on the circle");
            a.WhichQualifier(i, out global::Kerf.Engine.Construction.Constraints.Qualifier q1, out global::Kerf.Engine.Construction.Constraints.Qualifier q2);
            Assert.Equal(SideOf(l1, s.Location), q1);
            Assert.Equal(SideOf(l2, s.Location), q2);
        }
    }

    [Fact]
    public void TwoLines_OnCircle_ClockwiseFindsBothBisectors()
    {
        // OCCT's signed angle test sent lines turning clockwise to its parallel branch, the centres on the first
        // bisector only (2 solutions); both bisectors give centres now (decision 1541)
        Line2D l1 = Lin(0, 0, 0, 1);
        Line2D l2 = Lin(0, 0, 1, 0);
        Circle2D on = Circ(0, 0, 3);
        AnalyticCirc2d2TanOn a = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(l1), global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(l2), on, Tol);
        Assert.True(a.IsDone);
        Assert.Equal(4, a.SolutionCount);
        for (int i = 1; i <= 4; i++)
        {
            Circle2D s = a.ThisSolution(i);
            Assert.Equal(Math.Abs(s.Location.X), Math.Abs(s.Location.Y), 1.0e-9); // on a bisector y = x or y = -x
        }
        CheckAll(a, l1, l2, on);
    }

    [Fact]
    public void TwoParallelLines_OnCircle()
    {
        Line2D l1 = Lin(0, -1, 1, 0);
        Line2D l2 = Lin(0, 1, 1, 0);
        Circle2D on = Circ(0, 0, 2);
        AnalyticCirc2d2TanOn a = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(l1), global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(l2), on, Tol);
        Assert.Equal(2, a.SolutionCount);
        for (int i = 1; i <= 2; i++)
        {
            Assert.Equal(1.0, a.ThisSolution(i).Radius, 1.0e-9);
        }
        CheckAll(a, l1, l2, on);
    }

    [Fact]
    public void LinePoint_OnCircle()
    {
        // the parabola x^2 = 4 (y - 1) crossed by the circle of radius 5 about the origin: y = -2 + sqrt(33)
        Line2D l1 = Lin(0, 0, 1, 0);
        Point2D p = new(0, 2);
        Circle2D on = Circ(0, 0, 5);
        AnalyticCirc2d2TanOn a = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(l1), p, on, Tol);
        Assert.True(a.SolutionCount >= 2);
        for (int i = 1; i <= a.SolutionCount; i++)
        {
            Circle2D s = a.ThisSolution(i);
            AssertTouches(l1, s);
            AssertTouches(p, s);
            Assert.True(Math.Abs(s.Location.DistanceTo(on.Location) - 5.0) < 1.0e-6);
            Assert.Equal(-2.0 + Math.Sqrt(33.0), s.Location.Y, 1.0e-9);
            a.CenterOn3(i, out double aPar, out _);
            Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aPar, on).DistanceTo(s.Location) < 1.0e-6);
            a.Tangency2(i, out double aParSol2, out _, out Point2D aPnt2);
            Assert.True(aPnt2.DistanceTo(p) < Tol);
            Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aParSol2, s).DistanceTo(p) < Tol);
        }
        Assert.Equal(0, new AnalyticCirc2d2TanOn(global::Kerf.Engine.Construction.Constraints.GccEnt.Outside(l1), p, on, Tol).SolutionCount);
    }

    [Fact]
    public void LinePoint_OnCircle_PointAtCentre()
    {
        // OCCT's boundary case took the direction from the point to OnCirc's centre, null here, and threw; every
        // centre on OnCirc is 5 from the point, and the two at height 5 touch the line (decision 1541)
        Line2D l1 = Lin(0, 0, 1, 0);
        Point2D p = new(0, 2);
        Circle2D on = Circ(0, 2, 5);
        AnalyticCirc2d2TanOn a = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(l1), p, on, Tol);
        Assert.Equal(2, a.SolutionCount);
        for (int i = 1; i <= 2; i++)
        {
            Assert.Equal(5.0, a.ThisSolution(i).Radius, 1.0e-9);
            Assert.Equal(4.0, Math.Abs(a.ThisSolution(i).Location.X), 1.0e-9);
        }
        CheckAll(a, l1, p, on);
    }

    [Fact]
    public void CirclePoint_OnCircle()
    {
        Circle2D c1 = Circ(0, 0, 1);
        Point2D p = new(4, 0);
        Circle2D on = Circ(2, 0, 2.5);
        // OCCT gave each centre on the bisector both radii (|R1 - d| and R1 + d) when C1 was unqualified, and the
        // qualifier's radius when it was qualified, whichever branch the centre was on, so only some solutions passed
        // through the point; all do now (decision 1541)
        foreach (global::Kerf.Engine.Construction.Constraints.QualifiedCirc q in new[] { global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(c1), global::Kerf.Engine.Construction.Constraints.GccEnt.Outside(c1), global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosing(c1) })
        {
            AnalyticCirc2d2TanOn a = new(q, p, on, Tol);
            Assert.True(a.SolutionCount >= 1, q.ToString());
            int aNbThrough = 0;
            for (int i = 1; i <= a.SolutionCount; i++)
            {
                Circle2D s = a.ThisSolution(i);
                AssertTouches(c1, s);
                Assert.True(Math.Abs(s.Location.DistanceTo(on.Location) - 2.5) < 1.0e-6, "centre on the circle");
                a.CenterOn3(i, out double aPar, out _);
                Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aPar, on).DistanceTo(s.Location) < 1.0e-6);
                a.Tangency1(i, out double aParSol, out double aParArg, out Point2D aPnt);
                AssertTangencyPoint(c1, s, aParSol, aParArg, aPnt);
                aNbThrough += Math.Abs(s.Location.DistanceTo(p) - s.Radius) < 1.0e-6 ? 1 : 0;
                if (q.IsOutside)
                {
                    Assert.Equal(s.Radius + 1.0, s.Location.DistanceTo(c1.Location), 1.0e-6);
                }
            }
            Assert.Equal(a.SolutionCount, aNbThrough);
        }
    }

    [Fact]
    public void CircleLine_OnCircle()
    {
        Circle2D c1 = Circ(0, 4, 1);
        Line2D l2 = Lin(0, 0, 1, 0);
        Circle2D on = Circ(0, 0, 3);
        AnalyticCirc2d2TanOn a = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Outside(c1), global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosed(l2), on, Tol);
        Assert.True(a.SolutionCount >= 1);
        CheckAll(a, c1, l2, on);
        for (int i = 1; i <= a.SolutionCount; i++)
        {
            Circle2D s = a.ThisSolution(i);
            Assert.Equal(s.Radius + 1.0, s.Location.DistanceTo(c1.Location), 1.0e-6);
            Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed, SideOf(l2, s.Location));
        }
    }

    [Fact]
    public void TwoCircles_OnCircle()
    {
        Circle2D c1 = Circ(-3, 0, 1);
        Circle2D c2 = Circ(3, 0, 1.5);
        Circle2D on = Circ(0, 0, 4);
        AnalyticCirc2d2TanOn a = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Outside(c1), global::Kerf.Engine.Construction.Constraints.GccEnt.Outside(c2), on, Tol);
        Assert.True(a.SolutionCount >= 1);
        CheckAll(a, c1, c2, on);
        for (int i = 1; i <= a.SolutionCount; i++)
        {
            Circle2D s = a.ThisSolution(i);
            Assert.Equal(s.Radius + 1.0, s.Location.DistanceTo(c1.Location), 1.0e-6);
            Assert.Equal(s.Radius + 1.5, s.Location.DistanceTo(c2.Location), 1.0e-6);
            a.WhichQualifier(i, out global::Kerf.Engine.Construction.Constraints.Qualifier q1, out global::Kerf.Engine.Construction.Constraints.Qualifier q2);
            Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, q1);
            Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, q2);
        }
    }

    // ===== Errors =====

    [Fact]
    public void BadQualifier_EnclosingLine()
    {
        Line2D l = Lin(0, 0, 1, 0);
        global::Kerf.Engine.Construction.Constraints.QualifiedLin aBad = new(l, global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing);
        EngineAssert.Throws<global::Kerf.Engine.Construction.Constraints.BadQualifierException>(() => new AnalyticCirc2d2TanOn(aBad, global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(l), Lin(0, 1, 0, 1), Tol));
        EngineAssert.Throws<global::Kerf.Engine.Construction.Constraints.BadQualifierException>(() => new AnalyticCirc2d2TanOn(aBad, new Point2D(0, 1), Lin(0, 1, 0, 1), Tol));
        EngineAssert.Throws<global::Kerf.Engine.Construction.Constraints.BadQualifierException>(() => new AnalyticCirc2d2TanOn(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(Circ(0, 2, 1)), aBad,
                                                                          Circ(0, 0, 3), Tol));
        global::Kerf.Engine.Construction.Constraints.QualifiedCirc aBadC = new(Circ(0, 2, 1), global::Kerf.Engine.Construction.Constraints.Qualifier.Noqualifier);
        EngineAssert.Throws<global::Kerf.Engine.Construction.Constraints.BadQualifierException>(() => new AnalyticCirc2d2TanOn(aBadC, new Point2D(0, 1), Circ(0, 0, 3), Tol));
    }

    [Fact]
    public void Accessors_OutOfRange()
    {
        AnalyticCirc2d2TanOn a = new(new Point2D(0, 0), new Point2D(2, 0), Lin(0, 0, 1, 0), Tol);
        EngineAssert.Throws<OutOfRangeException>(() => a.WhichQualifier(2, out _, out _));
        EngineAssert.Throws<OutOfRangeException>(() => a.Tangency1(0, out _, out _, out _));
        EngineAssert.Throws<OutOfRangeException>(() => a.Tangency2(2, out _, out _, out _));
        EngineAssert.Throws<OutOfRangeException>(() => a.CenterOn3(2, out _, out _));
        EngineAssert.Throws<OutOfRangeException>(() => a.IsTheSame1(2));
        EngineAssert.Throws<OutOfRangeException>(() => a.IsTheSame2(0));
        // OCCT checked nothing in ThisSolution (decision 1541)
        EngineAssert.Throws<OutOfRangeException>(() => a.ThisSolution(2));
    }

    [Fact]
    public void LinePoint_OnLine_LineIsOnLine_NoSolution()
    {
        // the tangent line and the centre line coincide
        AnalyticCirc2d2TanOn a = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(Lin(0, 0, 1, 0)), new Point2D(0, 2), Lin(1, 0, 1, 0), Tol);
        Assert.False(a.IsDone);
        Assert.Equal(0, a.SolutionCount);
        EngineAssert.Throws<NotDoneException>(() => a.WhichQualifier(1, out _, out _));
    }
}
