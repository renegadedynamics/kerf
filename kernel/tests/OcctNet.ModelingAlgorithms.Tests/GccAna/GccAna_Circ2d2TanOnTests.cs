// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.GccEnt_Position;

namespace OcctNet.ModelingAlgorithms.Tests;

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
        Point2D c = theSol.Location();
        double r = theSol.Radius();
        switch (theArg)
        {
            case Line2D l:
                Assert.Equal(r, l.Distance(c), 1.0e-6);
                break;
            case Circle2D ci:
                double d = c.Distance(ci.Location());
                double aGap = Math.Min(Math.Abs(d - (r + ci.Radius())), Math.Abs(d - Math.Abs(r - ci.Radius())));
                Assert.True(aGap < 1.0e-6, $"solution {theSol} not tangent to {ci}: gap {aGap}");
                break;
            case Point2D p:
                Assert.Equal(r, c.Distance(p), 1.0e-6);
                break;
        }
    }

    // The tangency point is on the solution and on the argument, and the parameters evaluate to it.
    private static void AssertTangencyPoint(object theArg, in Circle2D theSol, double theParSol, double theParArg,
                                            in Point2D thePnt, bool theCheckParArg = true)
    {
        Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(theParSol, theSol).Distance(thePnt) < Tol, "ParSol");
        switch (theArg)
        {
            case Line2D l:
                Assert.True(l.Distance(thePnt) < Tol, "point on the line");
                if (theCheckParArg)
                {
                    Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(theParArg, l).Distance(thePnt) < Tol, "ParArg");
                }
                break;
            case Circle2D ci:
                Assert.True(Math.Abs(ci.Location().Distance(thePnt) - ci.Radius()) < Tol, "point on the circle");
                if (theCheckParArg)
                {
                    Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(theParArg, ci).Distance(thePnt) < Tol, "ParArg");
                }
                break;
            case Point2D p:
                Assert.True(p.Distance(thePnt) < Tol, "the point itself");
                Assert.Equal(0.0, theParArg);
                break;
        }
    }

    // Every solution: centre on the third curve (through CenterOn3's parameter), tangent to both arguments, and
    // the tangency points, unless the solution is the argument itself.
    private static void CheckAll(GccAna_Circ2d2TanOn theAlgo, object theArg1, object theArg2, object theOn,
                                 bool theCheckParArg1 = true, bool theCheckPnt1 = true)
    {
        Assert.True(theAlgo.IsDone());
        for (int i = 1; i <= theAlgo.NbSolutions(); i++)
        {
            Circle2D aSol = theAlgo.ThisSolution(i);
            AssertTouches(theArg1, aSol);
            AssertTouches(theArg2, aSol);

            theAlgo.CenterOn3(i, out double aParOn, out Point2D aPntOn);
            Point2D aCenter = aSol.Location();
            switch (theOn)
            {
                case Line2D l:
                    Assert.True(l.Distance(aCenter) < 1.0e-6, "centre on the line");
                    Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aParOn, l).Distance(aCenter) < 1.0e-6, "CenterOn3 parameter");
                    break;
                case Circle2D c:
                    Assert.True(Math.Abs(c.Location().Distance(aCenter) - c.Radius()) < 1.0e-6, "centre on the circle");
                    Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aParOn, c).Distance(aCenter) < 1.0e-6, "CenterOn3 parameter");
                    break;
            }

            if (!theAlgo.IsTheSame1(i))
            {
                theAlgo.Tangency1(i, out double aParSol, out double aParArg, out Point2D aPnt);
                if (theCheckPnt1)
                {
                    AssertTangencyPoint(theArg1, aSol, aParSol, aParArg, aPnt, theCheckParArg1);
                    // OCCT: CenterOn3 returns the first tangency point, not the centre
                    Assert.Equal(aPnt, aPntOn);
                }
            }
            else
            {
                Assert.Throws<NotDoneException>(() => theAlgo.Tangency1(i, out _, out _, out _));
            }
            if (!theAlgo.IsTheSame2(i))
            {
                theAlgo.Tangency2(i, out double aParSol, out double aParArg, out Point2D aPnt);
                AssertTangencyPoint(theArg2, aSol, aParSol, aParArg, aPnt);
            }
            theAlgo.WhichQualifier(i, out GccEnt_Position aQ1, out GccEnt_Position aQ2);
            Assert.NotEqual(GccEnt_unqualified, aQ1);
            Assert.NotEqual(GccEnt_unqualified, aQ2);
        }
    }

    // The qualifier of a solution relative to a line: outside if the line's origin is on the left of the solution's
    // centre (the centre on the line's right), enclosed otherwise.
    private static GccEnt_Position SideOf(in Line2D theLin, in Point2D theCenter)
    {
        Direction2D d = theLin.Direction();
        double aCross = (theLin.Location().X() - theCenter.X()) * (-d.Y()) + (theLin.Location().Y() - theCenter.Y()) * d.X();
        return aCross > 0.0 ? GccEnt_outside : GccEnt_enclosed;
    }

    // ===== Centre on a line =====

    [Fact]
    public void TwoPoints_OnLine_OneSolution()
    {
        Point2D p1 = new(0, 0), p2 = new(2, 0);
        Line2D on = Lin(0, 0, 1, 0);
        GccAna_Circ2d2TanOn a = new(p1, p2, on, Tol);
        Assert.True(a.IsDone());
        Assert.Equal(1, a.NbSolutions());
        Assert.True(a.ThisSolution(1).Location().Distance(new Point2D(1, 0)) < Tol);
        Assert.Equal(1.0, a.ThisSolution(1).Radius(), 1.0e-12);
        a.WhichQualifier(1, out GccEnt_Position q1, out GccEnt_Position q2);
        Assert.Equal(GccEnt_noqualifier, q1);
        Assert.Equal(GccEnt_noqualifier, q2);
        CheckAll(a, p1, p2, on);
    }

    [Fact]
    public void TwoPoints_OnLine_ParallelToBisector_NoSolution()
    {
        GccAna_Circ2d2TanOn a = new(new Point2D(0, 0), new Point2D(2, 0), Lin(5, 0, 0, 1), Tol);
        Assert.True(a.IsDone());
        Assert.Equal(0, a.NbSolutions());
    }

    [Fact]
    public void TwoPoints_Coincident_NoSolution()
    {
        GccAna_Circ2d2TanOn a = new(new Point2D(1, 1), new Point2D(1, 1), Lin(0, 0, 1, 0), Tol);
        Assert.True(a.IsDone());
        Assert.Equal(0, a.NbSolutions());
    }

    [Fact]
    public void TwoSecantLines_OnLine_TwoSolutionsAndQualifiers()
    {
        Line2D l1 = Lin(0, 0, 1, 0); // inside: y > 0
        Line2D l2 = Lin(0, 0, 0, 1); // inside: x < 0
        Line2D on = Lin(0, 4, 1, 0);
        GccAna_Circ2d2TanOn a = new(GccEnt.Unqualified(l1), GccEnt.Unqualified(l2), on, Tol);
        Assert.Equal(2, a.NbSolutions());
        for (int i = 1; i <= 2; i++)
        {
            Assert.Equal(4.0, a.ThisSolution(i).Radius(), 1.0e-9);
            a.WhichQualifier(i, out GccEnt_Position q1, out GccEnt_Position q2);
            Assert.Equal(SideOf(l1, a.ThisSolution(i).Location()), q1);
            Assert.Equal(SideOf(l2, a.ThisSolution(i).Location()), q2);
        }
        CheckAll(a, l1, l2, on);

        // enclosed by both: only the centre (-4, 4)
        GccAna_Circ2d2TanOn e = new(GccEnt.Enclosed(l1), GccEnt.Enclosed(l2), on, Tol);
        Assert.Equal(1, e.NbSolutions());
        Assert.True(e.ThisSolution(1).Location().Distance(new Point2D(-4, 4)) < 1.0e-9);
        CheckAll(e, l1, l2, on);

        // outside the second line: the centre (4, 4)
        GccAna_Circ2d2TanOn o = new(GccEnt.Enclosed(l1), GccEnt.Outside(l2), on, Tol);
        Assert.Equal(1, o.NbSolutions());
        Assert.True(o.ThisSolution(1).Location().Distance(new Point2D(4, 4)) < 1.0e-9);

        // outside both: the centres are above the first line, so none
        GccAna_Circ2d2TanOn n = new(GccEnt.Outside(l1), GccEnt.Outside(l2), on, Tol);
        Assert.Equal(0, n.NbSolutions());
    }

    [Fact]
    public void TwoParallelLines_OnTransverseLine_OneSolution()
    {
        Line2D l1 = Lin(0, 0, 1, 0);
        Line2D l2 = Lin(0, 2, 1, 0);
        Line2D on = Lin(3, 0, 0, 1);
        GccAna_Circ2d2TanOn a = new(GccEnt.Unqualified(l1), GccEnt.Unqualified(l2), on, Tol);
        Assert.Equal(1, a.NbSolutions());
        Assert.True(a.ThisSolution(1).Location().Distance(new Point2D(3, 1)) < 1.0e-9);
        Assert.Equal(1.0, a.ThisSolution(1).Radius(), 1.0e-9);
        a.WhichQualifier(1, out GccEnt_Position q1, out GccEnt_Position q2);
        Assert.Equal(GccEnt_enclosed, q1);
        Assert.Equal(GccEnt_outside, q2);
        CheckAll(a, l1, l2, on);

        // the solution is enclosed by the first line and outside the second
        Assert.Equal(0, new GccAna_Circ2d2TanOn(GccEnt.Outside(l1), GccEnt.Unqualified(l2), on, Tol).NbSolutions());
        Assert.Equal(1, new GccAna_Circ2d2TanOn(GccEnt.Enclosed(l1), GccEnt.Outside(l2), on, Tol).NbSolutions());
    }

    [Fact]
    public void LinePoint_OnLine_Parabola()
    {
        // focus (0, 2), directrix y = 0: x^2 = 4 (y - 1); on x = 2 the centre is (2, 2) with radius 2
        Line2D l1 = Lin(0, 0, 1, 0);
        Point2D p = new(0, 2);
        Line2D on = Lin(2, 0, 0, 1);
        GccAna_Circ2d2TanOn a = new(GccEnt.Unqualified(l1), p, on, Tol);
        Assert.Equal(1, a.NbSolutions());
        Assert.True(a.ThisSolution(1).Location().Distance(new Point2D(2, 2)) < 1.0e-9);
        Assert.Equal(2.0, a.ThisSolution(1).Radius(), 1.0e-9);
        a.WhichQualifier(1, out GccEnt_Position q1, out GccEnt_Position q2);
        Assert.Equal(GccEnt_enclosed, q1);
        Assert.Equal(GccEnt_noqualifier, q2);
        CheckAll(a, l1, p, on);
        Assert.Equal(0, new GccAna_Circ2d2TanOn(GccEnt.Outside(l1), p, on, Tol).NbSolutions());
        Assert.Equal(1, new GccAna_Circ2d2TanOn(GccEnt.Enclosed(l1), p, on, Tol).NbSolutions());
    }

    [Fact]
    public void LinePoint_OnLine_TwoCrossings()
    {
        // the parabola x^2 = 4 (y - 1) crossed by the line y = 5: x = +-4, radius 5
        Line2D l1 = Lin(0, 0, 1, 0);
        Point2D p = new(0, 2);
        Line2D on = Lin(0, 5, 1, 0);
        GccAna_Circ2d2TanOn a = new(GccEnt.Unqualified(l1), p, on, Tol);
        Assert.Equal(2, a.NbSolutions());
        for (int i = 1; i <= 2; i++)
        {
            Assert.Equal(5.0, a.ThisSolution(i).Radius(), 1.0e-9);
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
        GccAna_Circ2d2TanOn u = new(GccEnt.Unqualified(c1), p, on, Tol);
        Assert.True(u.NbSolutions() >= 2);
        // OCCT: the parameter of the first tangency point on C1 is stored at the bisector's index
        CheckAll(u, c1, p, on, theCheckParArg1: false);
        foreach (GccEnt_QualifiedCirc q in new[] { GccEnt.Unqualified(c1), GccEnt.Outside(c1), GccEnt.Enclosing(c1) })
        {
            GccAna_Circ2d2TanOn a = new(q, p, on, Tol);
            Assert.True(a.NbSolutions() >= 1, q.ToString());
            // OCCT: a qualified circle takes the centres on both branches of the bisector, so only some of the
            // solutions pass through the point
            int aNbThrough = 0;
            for (int i = 1; i <= a.NbSolutions(); i++)
            {
                Circle2D s = a.ThisSolution(i);
                AssertTouches(c1, s);
                Assert.True(Math.Abs(on.Distance(s.Location())) < 1.0e-6);
                aNbThrough += Math.Abs(s.Location().Distance(p) - s.Radius()) < 1.0e-6 ? 1 : 0;
            }
            Assert.True(aNbThrough >= 1, q.ToString());
            for (int i = 1; i <= a.NbSolutions(); i++)
            {
                a.WhichQualifier(i, out GccEnt_Position q1, out GccEnt_Position q2);
                Assert.Equal(GccEnt_noqualifier, q2);
                double d = a.ThisSolution(i).Location().Distance(c1.Location());
                double r = a.ThisSolution(i).Radius();
                if (q1 == GccEnt_outside)
                {
                    Assert.Equal(r + 1.0, d, 1.0e-6);
                }
                else if (q1 == GccEnt_enclosing)
                {
                    Assert.Equal(r - 1.0, d, 1.0e-6);
                }
                if (!q.IsUnqualified())
                {
                    Assert.Equal(q.Qualifier(), q1);
                }
            }
        }
        // the point is outside the circle: no solution inside it
        Assert.Equal(0, new GccAna_Circ2d2TanOn(GccEnt.Enclosed(c1), p, on, Tol).NbSolutions());
    }

    [Fact]
    public void CircleLine_OnLine()
    {
        Circle2D c1 = Circ(0, 3, 1);
        Line2D l2 = Lin(0, 0, 1, 0);
        Line2D on = Lin(2, 0, 0, 1);
        GccAna_Circ2d2TanOn a = new(GccEnt.Unqualified(c1), GccEnt.Unqualified(l2), on, Tol);
        Assert.True(a.NbSolutions() >= 2);
        CheckAll(a, c1, l2, on);
        int aNbOut = 0;
        for (int i = 1; i <= a.NbSolutions(); i++)
        {
            a.WhichQualifier(i, out GccEnt_Position q1, out GccEnt_Position q2);
            Assert.Equal(SideOf(l2, a.ThisSolution(i).Location()), q2);
            aNbOut += q1 == GccEnt_outside ? 1 : 0;
        }
        GccAna_Circ2d2TanOn o = new(GccEnt.Outside(c1), GccEnt.Enclosed(l2), on, Tol);
        Assert.Equal(aNbOut, o.NbSolutions());
        CheckAll(o, c1, l2, on);
    }

    [Fact]
    public void TwoCircles_OnLine()
    {
        Circle2D c1 = Circ(-3, 0, 1);
        Circle2D c2 = Circ(3, 0, 2);
        // outside both: d2 - d1 = 1, so on x = -1 (two centres, symmetric about the x axis)
        Line2D on = Lin(-1, 0, 0, 1);
        GccAna_Circ2d2TanOn a = new(GccEnt.Outside(c1), GccEnt.Outside(c2), on, Tol);
        Assert.True(a.NbSolutions() >= 1);
        CheckAll(a, c1, c2, on, theCheckParArg1: false);
        for (int i = 1; i <= a.NbSolutions(); i++)
        {
            Circle2D s = a.ThisSolution(i);
            Assert.Equal(s.Radius() + 1.0, s.Location().Distance(c1.Location()), 1.0e-6);
            Assert.Equal(s.Radius() + 2.0, s.Location().Distance(c2.Location()), 1.0e-6);
        }
        // enclosing both: d1 - d2 = 1, so on x = 1
        Line2D on2 = Lin(1, 0, 0, 1);
        GccAna_Circ2d2TanOn e = new(GccEnt.Enclosing(c1), GccEnt.Enclosing(c2), on2, Tol);
        Assert.True(e.NbSolutions() >= 1);
        CheckAll(e, c1, c2, on2, theCheckParArg1: false);
        for (int i = 1; i <= e.NbSolutions(); i++)
        {
            Circle2D s = e.ThisSolution(i);
            Assert.Equal(s.Radius() - 1.0, s.Location().Distance(c1.Location()), 1.0e-6);
            Assert.Equal(s.Radius() - 2.0, s.Location().Distance(c2.Location()), 1.0e-6);
        }
    }

    [Fact]
    public void TwoCircles_OnLine_SymmetricBoundaryCase()
    {
        // equal circles symmetric about the y axis, centre line the x axis through both centres: the boundary case
        Circle2D c1 = Circ(-3, 0, 1);
        Circle2D c2 = Circ(3, 0, 1);
        Line2D on = Lin(0, -5, 0, 1);
        GccAna_Circ2d2TanOn a = new(GccEnt.Outside(c1), GccEnt.Outside(c2), on, Tol);
        Assert.True(a.NbSolutions() >= 1);
        CheckAll(a, c1, c2, on);
    }

    // ===== Centre on a circle =====

    [Fact]
    public void TwoPoints_OnCircle_TwoSolutions()
    {
        Point2D p1 = new(-1, 0), p2 = new(1, 0);
        Circle2D on = Circ(0, 0, 2);
        GccAna_Circ2d2TanOn a = new(p1, p2, on, Tol);
        Assert.Equal(2, a.NbSolutions());
        for (int i = 1; i <= 2; i++)
        {
            Assert.Equal(Math.Sqrt(5.0), a.ThisSolution(i).Radius(), 1.0e-9);
        }
        CheckAll(a, p1, p2, on);
    }

    [Fact]
    public void TwoPoints_OnCircle_Missing_NoSolution()
    {
        GccAna_Circ2d2TanOn a = new(new Point2D(-1, 10), new Point2D(1, 10), Circ(10, 0, 2), Tol);
        Assert.True(a.IsDone());
        Assert.Equal(0, a.NbSolutions());
    }

    [Fact]
    public void TwoLines_OnCircle_Unqualified()
    {
        // two secant lines turning counter-clockwise (OCCT's signed angle test sends the clockwise ones to its
        // parallel branch)
        Line2D l1 = Lin(0, 0, 1, 0);
        Line2D l2 = Lin(0, 0, 0, 1);
        Circle2D on = Circ(0, 0, 3);
        GccAna_Circ2d2TanOn a = new(GccEnt.Unqualified(l1), GccEnt.Unqualified(l2), on, Tol);
        Assert.True(a.IsDone());
        Assert.True(a.NbSolutions() >= 1);
        for (int i = 1; i <= a.NbSolutions(); i++)
        {
            Circle2D s = a.ThisSolution(i);
            Assert.True(Math.Abs(s.Location().Distance(on.Location()) - 3.0) < 1.0e-6, "centre on the circle");
            a.WhichQualifier(i, out GccEnt_Position q1, out GccEnt_Position q2);
            Assert.Equal(SideOf(l1, s.Location()), q1);
            Assert.Equal(SideOf(l2, s.Location()), q2);
        }
    }

    [Fact]
    public void TwoLines_OnCircle_ClockwiseTakesTheParallelBranch()
    {
        // OCCT: D1.Angle(D2) <= Resolution is meant for parallel lines but is signed: lines turning clockwise get the
        // parallel case's treatment, the centres on the first (inside) bisector only
        Line2D l1 = Lin(0, 0, 0, 1);
        Line2D l2 = Lin(0, 0, 1, 0);
        Circle2D on = Circ(0, 0, 3);
        GccAna_Circ2d2TanOn a = new(GccEnt.Unqualified(l1), GccEnt.Unqualified(l2), on, Tol);
        Assert.True(a.IsDone());
        Assert.Equal(2, a.NbSolutions());
        for (int i = 1; i <= 2; i++)
        {
            Circle2D s = a.ThisSolution(i);
            Assert.Equal(s.Location().X(), s.Location().Y(), 1.0e-9); // on the bisector y = x
            CheckAll(a, l1, l2, on);
        }
    }

    [Fact]
    public void TwoParallelLines_OnCircle()
    {
        Line2D l1 = Lin(0, -1, 1, 0);
        Line2D l2 = Lin(0, 1, 1, 0);
        Circle2D on = Circ(0, 0, 2);
        GccAna_Circ2d2TanOn a = new(GccEnt.Unqualified(l1), GccEnt.Unqualified(l2), on, Tol);
        Assert.Equal(2, a.NbSolutions());
        for (int i = 1; i <= 2; i++)
        {
            Assert.Equal(1.0, a.ThisSolution(i).Radius(), 1.0e-9);
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
        GccAna_Circ2d2TanOn a = new(GccEnt.Unqualified(l1), p, on, Tol);
        Assert.True(a.NbSolutions() >= 2);
        for (int i = 1; i <= a.NbSolutions(); i++)
        {
            Circle2D s = a.ThisSolution(i);
            AssertTouches(l1, s);
            AssertTouches(p, s);
            Assert.True(Math.Abs(s.Location().Distance(on.Location()) - 5.0) < 1.0e-6);
            Assert.Equal(-2.0 + Math.Sqrt(33.0), s.Location().Y(), 1.0e-9);
            a.CenterOn3(i, out double aPar, out _);
            Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aPar, on).Distance(s.Location()) < 1.0e-6);
            // OCCT: the tangency point on the line is placed with the boundary case's distance, not the radius
            a.Tangency2(i, out double aParSol2, out _, out Point2D aPnt2);
            Assert.True(aPnt2.Distance(p) < Tol);
            Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aParSol2, s).Distance(p) < Tol);
        }
        Assert.Equal(0, new GccAna_Circ2d2TanOn(GccEnt.Outside(l1), p, on, Tol).NbSolutions());
    }

    [Fact]
    public void LinePoint_OnCircle_PointAtCentre_Throws()
    {
        // the direction from the point to OnCirc's centre is null: gp_Dir2d throws (decision 9)
        Assert.Throws<ConstructionErrorException>(
            () => new GccAna_Circ2d2TanOn(GccEnt.Unqualified(Lin(0, 0, 1, 0)), new Point2D(0, 2), Circ(0, 2, 5), Tol));
    }

    [Fact]
    public void CirclePoint_OnCircle()
    {
        Circle2D c1 = Circ(0, 0, 1);
        Point2D p = new(4, 0);
        Circle2D on = Circ(2, 0, 2.5);
        // OCCT: each centre on the bisector gets both radii (|R1 - d| and R1 + d) when C1 is unqualified, and the
        // qualifier's radius when it is qualified, whichever branch the centre is on: every solution is tangent to
        // C1 with its centre on OnCirc, but only some pass through the point
        foreach (GccEnt_QualifiedCirc q in new[] { GccEnt.Unqualified(c1), GccEnt.Outside(c1), GccEnt.Enclosing(c1) })
        {
            GccAna_Circ2d2TanOn a = new(q, p, on, Tol);
            Assert.True(a.NbSolutions() >= 1, q.ToString());
            int aNbThrough = 0;
            for (int i = 1; i <= a.NbSolutions(); i++)
            {
                Circle2D s = a.ThisSolution(i);
                AssertTouches(c1, s);
                Assert.True(Math.Abs(s.Location().Distance(on.Location()) - 2.5) < 1.0e-6, "centre on the circle");
                a.CenterOn3(i, out double aPar, out _);
                Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aPar, on).Distance(s.Location()) < 1.0e-6);
                a.Tangency1(i, out double aParSol, out double aParArg, out Point2D aPnt);
                AssertTangencyPoint(c1, s, aParSol, aParArg, aPnt);
                aNbThrough += Math.Abs(s.Location().Distance(p) - s.Radius()) < 1.0e-6 ? 1 : 0;
                if (q.IsOutside())
                {
                    Assert.Equal(s.Radius() + 1.0, s.Location().Distance(c1.Location()), 1.0e-6);
                }
            }
            Assert.True(aNbThrough >= 1, q.ToString());
        }
    }

    [Fact]
    public void CircleLine_OnCircle()
    {
        Circle2D c1 = Circ(0, 4, 1);
        Line2D l2 = Lin(0, 0, 1, 0);
        Circle2D on = Circ(0, 0, 3);
        GccAna_Circ2d2TanOn a = new(GccEnt.Outside(c1), GccEnt.Enclosed(l2), on, Tol);
        Assert.True(a.NbSolutions() >= 1);
        CheckAll(a, c1, l2, on);
        for (int i = 1; i <= a.NbSolutions(); i++)
        {
            Circle2D s = a.ThisSolution(i);
            Assert.Equal(s.Radius() + 1.0, s.Location().Distance(c1.Location()), 1.0e-6);
            Assert.Equal(GccEnt_enclosed, SideOf(l2, s.Location()));
        }
    }

    [Fact]
    public void TwoCircles_OnCircle()
    {
        Circle2D c1 = Circ(-3, 0, 1);
        Circle2D c2 = Circ(3, 0, 1.5);
        Circle2D on = Circ(0, 0, 4);
        GccAna_Circ2d2TanOn a = new(GccEnt.Outside(c1), GccEnt.Outside(c2), on, Tol);
        Assert.True(a.NbSolutions() >= 1);
        CheckAll(a, c1, c2, on);
        for (int i = 1; i <= a.NbSolutions(); i++)
        {
            Circle2D s = a.ThisSolution(i);
            Assert.Equal(s.Radius() + 1.0, s.Location().Distance(c1.Location()), 1.0e-6);
            Assert.Equal(s.Radius() + 1.5, s.Location().Distance(c2.Location()), 1.0e-6);
            a.WhichQualifier(i, out GccEnt_Position q1, out GccEnt_Position q2);
            Assert.Equal(GccEnt_outside, q1);
            Assert.Equal(GccEnt_outside, q2);
        }
    }

    // ===== Errors =====

    [Fact]
    public void BadQualifier_EnclosingLine()
    {
        Line2D l = Lin(0, 0, 1, 0);
        GccEnt_QualifiedLin aBad = new(l, GccEnt_enclosing);
        Assert.Throws<GccEnt_BadQualifier>(() => new GccAna_Circ2d2TanOn(aBad, GccEnt.Unqualified(l), Lin(0, 1, 0, 1), Tol));
        Assert.Throws<GccEnt_BadQualifier>(() => new GccAna_Circ2d2TanOn(aBad, new Point2D(0, 1), Lin(0, 1, 0, 1), Tol));
        Assert.Throws<GccEnt_BadQualifier>(() => new GccAna_Circ2d2TanOn(GccEnt.Unqualified(Circ(0, 2, 1)), aBad,
                                                                          Circ(0, 0, 3), Tol));
        GccEnt_QualifiedCirc aBadC = new(Circ(0, 2, 1), GccEnt_noqualifier);
        Assert.Throws<GccEnt_BadQualifier>(() => new GccAna_Circ2d2TanOn(aBadC, new Point2D(0, 1), Circ(0, 0, 3), Tol));
    }

    [Fact]
    public void Accessors_OutOfRange()
    {
        GccAna_Circ2d2TanOn a = new(new Point2D(0, 0), new Point2D(2, 0), Lin(0, 0, 1, 0), Tol);
        Assert.Throws<OutOfRangeException>(() => a.WhichQualifier(2, out _, out _));
        Assert.Throws<OutOfRangeException>(() => a.Tangency1(0, out _, out _, out _));
        Assert.Throws<OutOfRangeException>(() => a.Tangency2(2, out _, out _, out _));
        Assert.Throws<OutOfRangeException>(() => a.CenterOn3(2, out _, out _));
        Assert.Throws<OutOfRangeException>(() => a.IsTheSame1(2));
        Assert.Throws<OutOfRangeException>(() => a.IsTheSame2(0));
        // OCCT checks nothing in ThisSolution: past the arrays the port throws
        Assert.Throws<OutOfRangeException>(() => a.ThisSolution(3));
    }

    [Fact]
    public void LinePoint_OnLine_LineIsOnLine_NoSolution()
    {
        // the tangent line and the centre line coincide
        GccAna_Circ2d2TanOn a = new(GccEnt.Unqualified(Lin(0, 0, 1, 0)), new Point2D(0, 2), Lin(1, 0, 1, 0), Tol);
        Assert.False(a.IsDone());
        Assert.Equal(0, a.NbSolutions());
        Assert.Throws<NotDoneException>(() => a.WhichQualifier(1, out _, out _));
    }
}
