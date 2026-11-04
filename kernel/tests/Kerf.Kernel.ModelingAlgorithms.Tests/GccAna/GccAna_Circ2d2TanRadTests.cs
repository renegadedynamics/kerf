// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Constraints.Analytic;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// GccAna_Circ2d2TanRad on OCCT's documented configurations: solution counts, qualifiers, and on every solution the
/// radius, the tangencies and the tangency points.
/// </summary>
public class GccAna_Circ2d2TanRadTests
{
    private const double Tol = 1.0e-9;
    private static readonly Direction2D DX = new(Direction2D.D.X);

    private static Circle2D Circ(double x, double y, double r) => new(new Axis2D(new Point2D(x, y), DX), r);

    private static Line2D Lin(double x, double y, double dx, double dy) => new(new Point2D(x, y), new Direction2D(dx, dy));

    // The qualifier a circle argument has for a solution, from the geometry
    private static global::Kerf.Engine.Construction.Constraints.Qualifier CircQualifier(in Circle2D theArg, in Circle2D theSol)
    {
        double d = theArg.Location.DistanceTo(theSol.Location);
        if (Math.Abs(d + theSol.Radius - theArg.Radius) < 1.0e-7)
        {
            return global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed;
        }
        if (Math.Abs(d - theSol.Radius - theArg.Radius) < 1.0e-7)
        {
            return global::Kerf.Engine.Construction.Constraints.Qualifier.Outside;
        }
        Assert.Equal(theSol.Radius, d + theArg.Radius, 1.0e-7);
        return global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing;
    }

    // Enclosed: the centre on the left of the line; outside: on the right
    private static global::Kerf.Engine.Construction.Constraints.Qualifier LinQualifier(in Line2D theArg, in Circle2D theSol)
    {
        Vector2D aV = new(theArg.Location, theSol.Location);
        return new Vector2D(theArg.Direction).Crossed(aV) > 0.0 ? global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed : global::Kerf.Engine.Construction.Constraints.Qualifier.Outside;
    }

    private static void AssertTangentCirc(in Circle2D theArg, in Circle2D theSol)
    {
        double d = theArg.Location.DistanceTo(theSol.Location);
        double aGap = Math.Min(Math.Abs(d - (theArg.Radius + theSol.Radius)), Math.Abs(d - Math.Abs(theArg.Radius - theSol.Radius)));
        Assert.True(aGap < 1.0e-7, $"not tangent: {aGap}");
    }

    private static void AssertNear(in Point2D theP, in Point2D theQ) => Assert.True(theP.DistanceTo(theQ) < 1.0e-7, $"{theP} != {theQ}");

    private static void CheckTangency1(AnalyticCirc2d2TanRad theAlgo, int theIndex, in Circle2D theArg)
    {
        Circle2D aSol = theAlgo.ThisSolution(theIndex);
        AssertTangentCirc(theArg, aSol);
        if (theAlgo.IsTheSame1(theIndex))
        {
            return;
        }
        theAlgo.Tangency1(theIndex, out double aParSol, out double aParArg, out Point2D aPnt);
        AssertNear(global::Kerf.Engine.Geometry.ElCLib.Value(aParSol, aSol), aPnt);
        AssertNear(global::Kerf.Engine.Geometry.ElCLib.Value(aParArg, theArg), aPnt);
    }

    private static void CheckTangency2(AnalyticCirc2d2TanRad theAlgo, int theIndex, in Circle2D theArg)
    {
        Circle2D aSol = theAlgo.ThisSolution(theIndex);
        AssertTangentCirc(theArg, aSol);
        if (theAlgo.IsTheSame2(theIndex))
        {
            return;
        }
        theAlgo.Tangency2(theIndex, out double aParSol, out double aParArg, out Point2D aPnt);
        AssertNear(global::Kerf.Engine.Geometry.ElCLib.Value(aParSol, aSol), aPnt);
        AssertNear(global::Kerf.Engine.Geometry.ElCLib.Value(aParArg, theArg), aPnt);
    }

    private static void CheckTangencyLin(AnalyticCirc2d2TanRad theAlgo, int theIndex, in Line2D theArg, bool theFirst)
    {
        Circle2D aSol = theAlgo.ThisSolution(theIndex);
        Assert.Equal(aSol.Radius, theArg.DistanceTo(aSol.Location), 1.0e-7);
        double aParSol, aParArg;
        Point2D aPnt;
        if (theFirst)
        {
            theAlgo.Tangency1(theIndex, out aParSol, out aParArg, out aPnt);
        }
        else
        {
            theAlgo.Tangency2(theIndex, out aParSol, out aParArg, out aPnt);
        }
        AssertNear(global::Kerf.Engine.Geometry.ElCLib.Value(aParSol, aSol), aPnt);
        AssertNear(global::Kerf.Engine.Geometry.ElCLib.Value(aParArg, theArg), aPnt);
    }

    private static void CheckThrough(AnalyticCirc2d2TanRad theAlgo, int theIndex, in Point2D thePnt, bool theFirst)
    {
        Circle2D aSol = theAlgo.ThisSolution(theIndex);
        Assert.Equal(aSol.Radius, aSol.Location.DistanceTo(thePnt), 1.0e-7);
        double aParSol, aParArg;
        Point2D aPnt;
        if (theFirst)
        {
            theAlgo.Tangency1(theIndex, out aParSol, out aParArg, out aPnt);
        }
        else
        {
            theAlgo.Tangency2(theIndex, out aParSol, out aParArg, out aPnt);
        }
        AssertNear(aPnt, thePnt);
        AssertNear(global::Kerf.Engine.Geometry.ElCLib.Value(aParSol, aSol), aPnt);
    }

    [Theory]
    [InlineData(2.0, 2)]
    [InlineData(1.0, 1)]
    [InlineData(0.5, 0)]
    public void TwoPoints_Counts(double theRadius, int theNb)
    {
        Point2D aP1 = new(0, 0), aP2 = new(2, 0);
        AnalyticCirc2d2TanRad anAlgo = new(aP1, aP2, theRadius, Tol);
        Assert.True(anAlgo.IsDone);
        Assert.Equal(theNb, anAlgo.SolutionCount);
        for (int i = 1; i <= theNb; i++)
        {
            Assert.Equal(theRadius, anAlgo.ThisSolution(i).Radius);
            CheckThrough(anAlgo, i, aP1, true);
            CheckThrough(anAlgo, i, aP2, false);
            anAlgo.WhichQualifier(i, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ1, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ2);
            Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Noqualifier, aQ1);
            Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Noqualifier, aQ2);
        }
    }

    [Fact]
    public void TwoSecantLines_FourCirclesInTheQuadrants()
    {
        Line2D aL1 = Lin(0, 0, 1, 0), aL2 = Lin(0, 0, 0, 1);
        AnalyticCirc2d2TanRad anAlgo = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aL1), global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aL2), 1.0, Tol);
        Assert.True(anAlgo.IsDone);
        Assert.Equal(4, anAlgo.SolutionCount);
        HashSet<(double, double)> aCentres = [];
        for (int i = 1; i <= 4; i++)
        {
            Circle2D aSol = anAlgo.ThisSolution(i);
            Assert.Equal(1.0, aSol.Radius);
            CheckTangencyLin(anAlgo, i, aL1, true);
            CheckTangencyLin(anAlgo, i, aL2, false);
            aCentres.Add((Math.Round(aSol.Location.X, 9), Math.Round(aSol.Location.Y, 9)));
        }
        Assert.Equal([(1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0)], aCentres);
    }

    [Theory]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed, global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed, -1.0, 1.0)]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed, global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, 1.0, 1.0)]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed, -1.0, -1.0)]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, 1.0, -1.0)]
    public void TwoSecantLines_Qualified(object theQ1Value, object theQ2Value, double theX, double theY)
    {
        var theQ1 = (global::Kerf.Engine.Construction.Constraints.Qualifier)theQ1Value; // the theory's data as object: the engine type is internal
        var theQ2 = (global::Kerf.Engine.Construction.Constraints.Qualifier)theQ2Value; // the theory's data as object: the engine type is internal
        // The inside of the X axis (along +X) is y > 0, the inside of the Y axis (along +Y) x < 0
        Line2D aL1 = Lin(0, 0, 1, 0), aL2 = Lin(0, 0, 0, 1);
        AnalyticCirc2d2TanRad anAlgo = new(new global::Kerf.Engine.Construction.Constraints.QualifiedLin(aL1, theQ1), new global::Kerf.Engine.Construction.Constraints.QualifiedLin(aL2, theQ2), 1.0, Tol);
        Assert.True(anAlgo.IsDone);
        Assert.Equal(1, anAlgo.SolutionCount);
        AssertNear(anAlgo.ThisSolution(1).Location, new Point2D(theX, theY));
        CheckTangencyLin(anAlgo, 1, aL1, true);
        CheckTangencyLin(anAlgo, 1, aL2, false);
        anAlgo.WhichQualifier(1, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ1, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ2);
        Assert.Equal(theQ1, aQ1);
        Assert.Equal(theQ2, aQ2);
        Assert.Equal(theQ1, LinQualifier(aL1, anAlgo.ThisSolution(1)));
        Assert.Equal(theQ2, LinQualifier(aL2, anAlgo.ThisSolution(1)));
    }

    [Fact]
    public void ParallelLines_NoSolution()
    {
        AnalyticCirc2d2TanRad anAlgo = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(Lin(0, 0, 1, 0)), global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(Lin(0, 2, 1, 0)), 1.0, Tol);
        Assert.True(anAlgo.IsDone);
        Assert.Equal(0, anAlgo.SolutionCount);
    }

    [Fact]
    public void LineEnclosingQualifier_Throws()
    {
        Line2D aL = Lin(0, 0, 1, 0);
        global::Kerf.Engine.Construction.Constraints.QualifiedLin aBad = new(aL, global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing);
        Assert.Throws<global::Kerf.Engine.Construction.Constraints.BadQualifierException>(() => new AnalyticCirc2d2TanRad(aBad, global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aL), 1.0, Tol));
        Assert.Throws<global::Kerf.Engine.Construction.Constraints.BadQualifierException>(() => new AnalyticCirc2d2TanRad(aBad, new Point2D(0, 1), 1.0, Tol));
        Assert.Throws<global::Kerf.Engine.Construction.Constraints.BadQualifierException>(() => new AnalyticCirc2d2TanRad(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(Circ(0, 0, 1)), aBad, 1.0, Tol));
        global::Kerf.Engine.Construction.Constraints.QualifiedCirc aNoQual = new(Circ(0, 0, 1), global::Kerf.Engine.Construction.Constraints.Qualifier.Noqualifier);
        Assert.Throws<global::Kerf.Engine.Construction.Constraints.BadQualifierException>(() => new AnalyticCirc2d2TanRad(aNoQual, global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(Circ(4, 0, 1)), 1.0, Tol));
    }

    [Fact]
    public void NegativeRadius_Throws()
    {
        Line2D aL1 = Lin(0, 0, 1, 0), aL2 = Lin(0, 0, 0, 1);
        Circle2D aC1 = Circ(0, 0, 1), aC2 = Circ(4, 0, 1);
        Assert.Throws<NegativeValueException>(() => new AnalyticCirc2d2TanRad(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aL1), global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aL2), -1.0, Tol));
        Assert.Throws<NegativeValueException>(() => new AnalyticCirc2d2TanRad(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aC1), global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aC2), -1.0, Tol));
        Assert.Throws<NegativeValueException>(() => new AnalyticCirc2d2TanRad(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aC1), global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aL1), -1.0, Tol));
        Assert.Throws<NegativeValueException>(() => new AnalyticCirc2d2TanRad(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aC1), new Point2D(3, 0), -1.0, Tol));
        Assert.Throws<NegativeValueException>(() => new AnalyticCirc2d2TanRad(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aL1), new Point2D(3, 1), -1.0, Tol));
        Assert.Throws<NegativeValueException>(() => new AnalyticCirc2d2TanRad(new Point2D(0, 0), new Point2D(1, 0), -1.0, Tol));
    }

    [Fact]
    public void Accessors_OutOfRange()
    {
        AnalyticCirc2d2TanRad anAlgo = new(new Point2D(0, 0), new Point2D(2, 0), 2.0, Tol);
        Assert.Throws<OutOfRangeException>(() => anAlgo.ThisSolution(0));
        Assert.Throws<OutOfRangeException>(() => anAlgo.ThisSolution(3));
        Assert.Throws<OutOfRangeException>(() => anAlgo.WhichQualifier(3, out _, out _));
        Assert.Throws<OutOfRangeException>(() => anAlgo.Tangency1(3, out _, out _, out _));
        Assert.Throws<OutOfRangeException>(() => anAlgo.Tangency2(0, out _, out _, out _));
        Assert.Throws<OutOfRangeException>(() => anAlgo.IsTheSame1(3));
        Assert.Throws<OutOfRangeException>(() => anAlgo.IsTheSame2(3));
    }

    // C1 at the origin, C2 at (4, 0), both of radius 1, and solutions of radius 2.5: outside both (2), enclosing one
    // and outside the other (2 + 2); none encloses both, and none is enclosed (the radius is larger).
    [Theory]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Unqualified, global::Kerf.Engine.Construction.Constraints.Qualifier.Unqualified, 6)]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, 2)]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing, global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, 2)]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing, 2)]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing, global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing, 0)]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed, global::Kerf.Engine.Construction.Constraints.Qualifier.Unqualified, 0)]
    public void TwoCircles(object theQ1Value, object theQ2Value, int theNb)
    {
        var theQ1 = (global::Kerf.Engine.Construction.Constraints.Qualifier)theQ1Value; // the theory's data as object: the engine type is internal
        var theQ2 = (global::Kerf.Engine.Construction.Constraints.Qualifier)theQ2Value; // the theory's data as object: the engine type is internal
        Circle2D aC1 = Circ(0, 0, 1), aC2 = Circ(4, 0, 1);
        AnalyticCirc2d2TanRad anAlgo = new(new global::Kerf.Engine.Construction.Constraints.QualifiedCirc(aC1, theQ1), new global::Kerf.Engine.Construction.Constraints.QualifiedCirc(aC2, theQ2), 2.5, Tol);
        Assert.True(anAlgo.IsDone);
        Assert.Equal(theNb, anAlgo.SolutionCount);
        for (int i = 1; i <= theNb; i++)
        {
            Circle2D aSol = anAlgo.ThisSolution(i);
            Assert.Equal(2.5, aSol.Radius);
            CheckTangency1(anAlgo, i, aC1);
            CheckTangency2(anAlgo, i, aC2);
            anAlgo.WhichQualifier(i, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ1, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ2);
            Assert.Equal(CircQualifier(aC1, aSol), aQ1);
            Assert.Equal(CircQualifier(aC2, aSol), aQ2);
            if (theQ1 != global::Kerf.Engine.Construction.Constraints.Qualifier.Unqualified)
            {
                Assert.Equal(theQ1, aQ1);
            }
            if (theQ2 != global::Kerf.Engine.Construction.Constraints.Qualifier.Unqualified)
            {
                Assert.Equal(theQ2, aQ2);
            }
        }
    }

    // With one circle enclosing and the other unqualified, OCCT's boundary test ("> 0.0" for "<= Tol") always held and it
    // returned one circle not tangent to the second; the true answer is the 2 circles enclosing the first circle and
    // outside the other (decision 1542)
    [Theory]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing, global::Kerf.Engine.Construction.Constraints.Qualifier.Unqualified)]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Unqualified, global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing)]
    public void TwoCircles_EnclosingUnqualified(object theQ1Value, object theQ2Value)
    {
        var theQ1 = (global::Kerf.Engine.Construction.Constraints.Qualifier)theQ1Value; // the theory's data as object: the engine type is internal
        var theQ2 = (global::Kerf.Engine.Construction.Constraints.Qualifier)theQ2Value; // the theory's data as object: the engine type is internal
        Circle2D aC1 = Circ(0, 0, 1), aC2 = Circ(4, 0, 1);
        AnalyticCirc2d2TanRad anAlgo = new(new global::Kerf.Engine.Construction.Constraints.QualifiedCirc(aC1, theQ1), new global::Kerf.Engine.Construction.Constraints.QualifiedCirc(aC2, theQ2), 2.5, Tol);
        Assert.True(anAlgo.IsDone);
        Assert.Equal(2, anAlgo.SolutionCount);
        for (int i = 1; i <= 2; i++)
        {
            CheckTangency1(anAlgo, i, aC1);
            CheckTangency2(anAlgo, i, aC2);
            anAlgo.WhichQualifier(i, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ1, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ2);
            Assert.Equal(theQ1 == global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing ? theQ1 : global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, aQ1);
            Assert.Equal(theQ2 == global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing ? theQ2 : global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, aQ2);
        }
    }

    // With one circle outside and the other unqualified, OCCT centred the second pair of offset circles on the same
    // point and missed the 2 circles enclosing the unqualified circle: 4 solutions (decision 1542)
    [Fact]
    public void TwoCircles_UnqualifiedOutside()
    {
        Circle2D aC1 = Circ(0, 0, 1), aC2 = Circ(4, 0, 1);
        AnalyticCirc2d2TanRad anAlgo = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aC1), global::Kerf.Engine.Construction.Constraints.GccEnt.Outside(aC2), 2.5, Tol);
        Assert.True(anAlgo.IsDone);
        Assert.Equal(4, anAlgo.SolutionCount);
        int aNbEnclosing = 0;
        for (int i = 1; i <= 4; i++)
        {
            CheckTangency1(anAlgo, i, aC1);
            CheckTangency2(anAlgo, i, aC2);
            anAlgo.WhichQualifier(i, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ1, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ2);
            aNbEnclosing += aQ1 == global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing ? 1 : 0;
            Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, aQ2);
        }
        Assert.Equal(2, aNbEnclosing);
    }

    // Two circles of radius 2 whose centres are 1 apart: solutions of radius 0.5 inside both, or inside one and
    // outside the other
    [Fact]
    public void TwoOverlappingCircles_Enclosed()
    {
        Circle2D aC1 = Circ(0, 0, 2), aC2 = Circ(1, 0, 2);
        AnalyticCirc2d2TanRad anAlgo = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosed(aC1), global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosed(aC2), 0.5, Tol);
        Assert.True(anAlgo.IsDone);
        Assert.Equal(2, anAlgo.SolutionCount);
        for (int i = 1; i <= 2; i++)
        {
            CheckTangency1(anAlgo, i, aC1);
            CheckTangency2(anAlgo, i, aC2);
            Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed, CircQualifier(aC1, anAlgo.ThisSolution(i)));
            Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed, CircQualifier(aC2, anAlgo.ThisSolution(i)));
        }
    }

    [Fact]
    public void ConcentricCircles_NoSolution()
    {
        AnalyticCirc2d2TanRad anAlgo = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(Circ(0, 0, 1)), global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(Circ(0, 0, 3)), 1.0, Tol);
        Assert.True(anAlgo.IsDone);
        Assert.Equal(0, anAlgo.SolutionCount);
    }

    // A circle at (0, 0.5) of radius 1 cutting the X axis, solutions of radius 0.3: above the axis, outside the
    // circle (2) or inside it (2); below the axis, outside the circle (2).
    [Theory]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Unqualified, global::Kerf.Engine.Construction.Constraints.Qualifier.Unqualified, 6)]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, global::Kerf.Engine.Construction.Constraints.Qualifier.Unqualified, 4)]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed, global::Kerf.Engine.Construction.Constraints.Qualifier.Unqualified, 2)]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Unqualified, global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed, 4)]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Unqualified, global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, 2)]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, 2)]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing, global::Kerf.Engine.Construction.Constraints.Qualifier.Unqualified, 0)]
    public void CircleAndLine(object theQ1Value, object theQ2Value, int theNb)
    {
        var theQ1 = (global::Kerf.Engine.Construction.Constraints.Qualifier)theQ1Value; // the theory's data as object: the engine type is internal
        var theQ2 = (global::Kerf.Engine.Construction.Constraints.Qualifier)theQ2Value; // the theory's data as object: the engine type is internal
        Circle2D aC1 = Circ(0, 0.5, 1);
        Line2D aL2 = Lin(0, 0, 1, 0);
        AnalyticCirc2d2TanRad anAlgo = new(new global::Kerf.Engine.Construction.Constraints.QualifiedCirc(aC1, theQ1), new global::Kerf.Engine.Construction.Constraints.QualifiedLin(aL2, theQ2), 0.3, Tol);
        Assert.True(anAlgo.IsDone);
        Assert.Equal(theNb, anAlgo.SolutionCount);
        for (int i = 1; i <= theNb; i++)
        {
            Circle2D aSol = anAlgo.ThisSolution(i);
            Assert.Equal(0.3, aSol.Radius);
            CheckTangency1(anAlgo, i, aC1);
            CheckTangencyLin(anAlgo, i, aL2, false);
            anAlgo.WhichQualifier(i, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ1, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ2);
            Assert.Equal(CircQualifier(aC1, aSol), aQ1);
            Assert.Equal(LinQualifier(aL2, aSol), aQ2);
            Assert.False(anAlgo.IsTheSame1(i));
            Assert.False(anAlgo.IsTheSame2(i));
        }
    }

    // A unit circle at the origin and a point outside it, at (3, 0): solutions of radius 1.5 outside the circle (2);
    // a point inside it, at (0.5, 0): solutions of radius 0.5 inside the circle (2).
    [Theory]
    [InlineData(3.0, 1.5, global::Kerf.Engine.Construction.Constraints.Qualifier.Unqualified, 2)]
    [InlineData(3.0, 1.5, global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, 2)]
    [InlineData(3.0, 1.5, global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing, 0)]
    [InlineData(3.0, 1.5, global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed, 0)]
    [InlineData(0.5, 0.5, global::Kerf.Engine.Construction.Constraints.Qualifier.Unqualified, 2)]
    [InlineData(0.5, 0.5, global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed, 2)]
    [InlineData(0.5, 0.5, global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, 0)]
    public void CircleAndPoint(double theX, double theRadius, object theQ1Value, int theNb)
    {
        var theQ1 = (global::Kerf.Engine.Construction.Constraints.Qualifier)theQ1Value; // the theory's data as object: the engine type is internal
        Circle2D aC1 = Circ(0, 0, 1);
        Point2D aP2 = new(theX, 0);
        AnalyticCirc2d2TanRad anAlgo = new(new global::Kerf.Engine.Construction.Constraints.QualifiedCirc(aC1, theQ1), aP2, theRadius, Tol);
        Assert.True(anAlgo.IsDone);
        Assert.Equal(theNb, anAlgo.SolutionCount);
        for (int i = 1; i <= theNb; i++)
        {
            Circle2D aSol = anAlgo.ThisSolution(i);
            Assert.Equal(theRadius, aSol.Radius);
            CheckTangency1(anAlgo, i, aC1);
            CheckThrough(anAlgo, i, aP2, false);
            anAlgo.WhichQualifier(i, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ1, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ2);
            Assert.Equal(CircQualifier(aC1, aSol), aQ1);
            Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Noqualifier, aQ2);
        }
    }

    // The X axis and the point (0, 0.5): solutions of radius 1 above the axis (2)
    [Theory]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Unqualified, 2)]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed, 2)]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, 0)]
    public void LineAndPoint(object theQ1Value, int theNb)
    {
        var theQ1 = (global::Kerf.Engine.Construction.Constraints.Qualifier)theQ1Value; // the theory's data as object: the engine type is internal
        Line2D aL1 = Lin(0, 0, 1, 0);
        Point2D aP2 = new(0, 0.5);
        AnalyticCirc2d2TanRad anAlgo = new(new global::Kerf.Engine.Construction.Constraints.QualifiedLin(aL1, theQ1), aP2, 1.0, Tol);
        Assert.True(anAlgo.IsDone);
        Assert.Equal(theNb, anAlgo.SolutionCount);
        for (int i = 1; i <= theNb; i++)
        {
            Circle2D aSol = anAlgo.ThisSolution(i);
            Assert.Equal(1.0, aSol.Radius);
            CheckTangencyLin(anAlgo, i, aL1, true);
            CheckThrough(anAlgo, i, aP2, false);
            anAlgo.WhichQualifier(i, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ1, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ2);
            Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed, aQ1);
            Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Noqualifier, aQ2);
        }
    }

    // A solution equal to an argument: the circle of radius 1 through a point on it. OCCT leaves that solution's
    // tangency point at the default (0, 0) and still asks for its parameter on the solution, so the circle is off the
    // origin here (see CircleAtTheOriginAndPointOnIt_Throws)
    [Fact]
    public void CircleAndPointOnIt_SolutionIsTheArgument()
    {
        Circle2D aC1 = Circ(1, 2, 1);
        AnalyticCirc2d2TanRad anAlgo = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aC1), new Point2D(2, 2), 1.0, Tol);
        Assert.True(anAlgo.IsDone);
        Assert.True(anAlgo.SolutionCount >= 1);
        Assert.True(anAlgo.IsTheSame1(1));
        Assert.Equal(aC1.Location.X, anAlgo.ThisSolution(1).Location.X);
        Assert.Throws<NotDoneException>(() => anAlgo.Tangency1(1, out _, out _, out _));
        for (int i = 2; i <= anAlgo.SolutionCount; i++)
        {
            CheckTangency1(anAlgo, i, aC1);
            CheckThrough(anAlgo, i, new Point2D(2, 2), false);
        }
    }

    // As above with the circle centred at the origin: OCCT took the parameter of the solution's unset tangency point
    // (0, 0), its centre, and threw Standard_ConstructionError; the solution that is the argument has no tangency point
    // (decision 1542)
    [Fact]
    public void CircleAtTheOriginAndPointOnIt_SolutionIsTheArgument()
    {
        AnalyticCirc2d2TanRad anAlgo = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(Circ(0, 0, 1)), new Point2D(1, 0), 1.0, Tol);
        Assert.True(anAlgo.IsDone);
        Assert.True(anAlgo.SolutionCount >= 1);
        Assert.True(anAlgo.IsTheSame1(1));
        Assert.Throws<NotDoneException>(() => anAlgo.Tangency1(1, out _, out _, out _));
    }
}
