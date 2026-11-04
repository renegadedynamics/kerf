// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Constraints.Analytic;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// GccAna_Circ2dTanOnRad on OCCT's documented configurations: solution counts, qualifiers, and on every solution the
/// radius, the centre on the On-curve, the tangency and the tangency point. Where OCCT's results were wrong
/// (docs/occt-issues.md), the tests check the corrected ones (decision 17104).
/// </summary>
public class GccAna_Circ2dTanOnRadTests
{
    private const double Tol = 1.0e-9;
    private static readonly Direction2D DX = new(Direction2D.D.X);

    private static Circle2D Circ(double x, double y, double r) => new(new Axis2D(new Point2D(x, y), DX), r);

    private static Line2D Lin(double x, double y, double dx, double dy) => new(new Point2D(x, y), new Direction2D(dx, dy));

    private static void AssertNear(in Point2D theP, in Point2D theQ) => Assert.True(theP.DistanceTo(theQ) < 1.0e-7, $"{theP} != {theQ}");

    private static void CheckCentreOnLine(AnalyticCirc2dTanOnRad theAlgo, int theIndex, in Line2D theOn)
    {
        Circle2D aSol = theAlgo.ThisSolution(theIndex);
        Assert.True(theOn.DistanceTo(aSol.Location) < 1.0e-7);
        theAlgo.CenterOn3(theIndex, out double aParArg, out _);
        AssertNear(global::Kerf.Engine.Geometry.ElCLib.Value(aParArg, theOn), aSol.Location);
    }

    private static void CheckCentreOnCirc(AnalyticCirc2dTanOnRad theAlgo, int theIndex, in Circle2D theOn)
    {
        Circle2D aSol = theAlgo.ThisSolution(theIndex);
        Assert.True(theOn.DistanceTo(aSol.Location) < 1.0e-7);
        theAlgo.CenterOn3(theIndex, out double aParArg, out _);
        AssertNear(global::Kerf.Engine.Geometry.ElCLib.Value(aParArg, theOn), aSol.Location);
    }

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

    private static void CheckTangentCirc(AnalyticCirc2dTanOnRad theAlgo, int theIndex, in Circle2D theArg)
    {
        Circle2D aSol = theAlgo.ThisSolution(theIndex);
        global::Kerf.Engine.Construction.Constraints.Qualifier aQual = CircQualifier(theArg, aSol);
        theAlgo.WhichQualifier(theIndex, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ1);
        Assert.Equal(aQual, aQ1);
        theAlgo.Tangency1(theIndex, out double aParSol, out double aParArg, out Point2D aPnt);
        AssertNear(global::Kerf.Engine.Geometry.ElCLib.Value(aParSol, aSol), aPnt);
        AssertNear(global::Kerf.Engine.Geometry.ElCLib.Value(aParArg, theArg), aPnt);
    }

    // Unit circle at the origin, centres on the X axis, radius 0.4: outside at x = +-1.4, inside at x = +-0.6. OCCT put
    // the tangency points at the solutions' far ends (the sign of dir1cen was swapped); they are where the circles touch
    // (decision 17104).
    [Theory]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Unqualified, 4)]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, 2)]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed, 2)]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing, 0)]
    public void CircleCentreOnLine(global::Kerf.Engine.Construction.Constraints.Qualifier theQ, int theNb)
    {
        Circle2D aC = Circ(0, 0, 1);
        Line2D anOn = Lin(0, 0, 1, 0);
        AnalyticCirc2dTanOnRad anAlgo = new(new global::Kerf.Engine.Construction.Constraints.QualifiedCirc(aC, theQ), anOn, 0.4, Tol);
        Assert.True(anAlgo.IsDone);
        Assert.Equal(theNb, anAlgo.SolutionCount);
        for (int i = 1; i <= theNb; i++)
        {
            Assert.Equal(0.4, anAlgo.ThisSolution(i).Radius);
            Assert.False(anAlgo.IsTheSame1(i));
            CheckCentreOnLine(anAlgo, i, anOn);
            CheckTangentCirc(anAlgo, i, aC);
        }
    }

    // Enclosed in the unit circle, radius 0.5, centres on the X axis: (+-0.5, 0), touching it at (+-1, 0). OCCT put the
    // tangency point at the argument's centre and threw taking its parameter (decision 17104)
    [Fact]
    public void CircleCentreOnLine_EnclosedHalfRadius()
    {
        Circle2D aC = Circ(0, 0, 1);
        Line2D anOn = Lin(0, 0, 1, 0);
        AnalyticCirc2dTanOnRad anAlgo = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosed(aC), anOn, 0.5, Tol);
        Assert.Equal(2, anAlgo.SolutionCount);
        for (int i = 1; i <= 2; i++)
        {
            CheckCentreOnLine(anAlgo, i, anOn);
            CheckTangentCirc(anAlgo, i, aC);
            anAlgo.Tangency1(i, out _, out _, out Point2D aPnt);
            AssertNear(aPnt, new Point2D(2 * anAlgo.ThisSolution(i).Location.X, 0));
        }
    }

    // CenterOn3 gives the centre (OCCT gave the tangency point: decision 17104)
    [Fact]
    public void CenterOn3_ReturnsTheCentre()
    {
        AnalyticCirc2dTanOnRad anAlgo = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Outside(Circ(0, 0, 1)), Lin(0, 0, 1, 0), 0.5, Tol);
        Assert.Equal(2, anAlgo.SolutionCount);
        for (int i = 1; i <= 2; i++)
        {
            anAlgo.CenterOn3(i, out _, out Point2D aPnt);
            Assert.Equal(anAlgo.ThisSolution(i).Location, aPnt);
        }
    }

    // The X axis, centres on the Y axis, radius 1: (0, 1) inside (left of +X), (0, -1) outside
    [Theory]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Unqualified, 2)]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed, 1)]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, 1)]
    public void LineCentreOnLine(global::Kerf.Engine.Construction.Constraints.Qualifier theQ, int theNb)
    {
        Line2D aL = Lin(0, 0, 1, 0);
        Line2D anOn = Lin(0, 0, 0, 1);
        AnalyticCirc2dTanOnRad anAlgo = new(new global::Kerf.Engine.Construction.Constraints.QualifiedLin(aL, theQ), anOn, 1.0, Tol);
        Assert.True(anAlgo.IsDone);
        Assert.Equal(theNb, anAlgo.SolutionCount);
        for (int i = 1; i <= theNb; i++)
        {
            Circle2D aSol = anAlgo.ThisSolution(i);
            Assert.Equal(1.0, aL.DistanceTo(aSol.Location), 1.0e-9);
            Assert.True(anOn.DistanceTo(aSol.Location) < 1.0e-9);
            anAlgo.WhichQualifier(i, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ1);
            Assert.Equal(aSol.Location.Y > 0 ? global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed : global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, aQ1);
            anAlgo.Tangency1(i, out double aParSol, out double aParArg, out Point2D aPnt);
            AssertNear(global::Kerf.Engine.Geometry.ElCLib.Value(aParSol, aSol), aPnt);
            AssertNear(global::Kerf.Engine.Geometry.ElCLib.Value(aParArg, aL), aPnt);
        }
        if (theQ == global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed)
        {
            AssertNear(anAlgo.ThisSolution(1).Location, new Point2D(0, 1));
        }
    }

    // Each centre has its own parameter on the line (OCCT gave the second solution the first one's: decision 17104)
    [Fact]
    public void LineCentreOnLine_EachCentreItsParameter()
    {
        Line2D anOn = Lin(0, 0, 0, 1);
        AnalyticCirc2dTanOnRad anAlgo = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(Lin(0, 0, 1, 0)), anOn, 1.0, Tol);
        Assert.Equal(2, anAlgo.SolutionCount);
        CheckCentreOnLine(anAlgo, 1, anOn);
        CheckCentreOnLine(anAlgo, 2, anOn);
        anAlgo.CenterOn3(1, out double aPar1, out _);
        anAlgo.CenterOn3(2, out double aPar2, out _);
        Assert.Equal(2.0, Math.Abs(aPar1 - aPar2), 1.0e-9);
    }

    [Fact]
    public void ParallelLines_NoSolution()
    {
        AnalyticCirc2dTanOnRad anAlgo = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(Lin(0, 0, 1, 0)), Lin(0, 1, 1, 0), 1.0, Tol);
        Assert.True(anAlgo.IsDone);
        Assert.Equal(0, anAlgo.SolutionCount);
    }

    // The point (1, 0), centres on the Y axis: radius 2 at (0, +-sqrt(3)), radius 1 at the origin, radius 0.5 none
    [Theory]
    [InlineData(2.0, 2)]
    [InlineData(1.0, 1)]
    [InlineData(0.5, 0)]
    public void PointCentreOnLine(double theRadius, int theNb)
    {
        Point2D aP = new(1, 0);
        Line2D anOn = Lin(0, 0, 0, 1);
        AnalyticCirc2dTanOnRad anAlgo = new(aP, anOn, theRadius, Tol);
        Assert.True(anAlgo.IsDone);
        Assert.Equal(theNb, anAlgo.SolutionCount);
        for (int i = 1; i <= theNb; i++)
        {
            Circle2D aSol = anAlgo.ThisSolution(i);
            Assert.Equal(theRadius, aSol.Radius);
            Assert.Equal(theRadius, aSol.Location.DistanceTo(aP), 1.0e-9);
            CheckCentreOnLine(anAlgo, i, anOn);
            anAlgo.WhichQualifier(i, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ1);
            Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Noqualifier, aQ1);
            anAlgo.Tangency1(i, out double aParSol, out _, out Point2D aPnt);
            AssertNear(aPnt, aP);
            AssertNear(global::Kerf.Engine.Geometry.ElCLib.Value(aParSol, aSol), aPnt);
        }
    }

    // The point on the line: two solutions (OCCT found them but left IsDone false: decision 17104)
    [Fact]
    public void PointOnTheCentreLine()
    {
        Line2D anOn = Lin(0, 0, 0, 1);
        AnalyticCirc2dTanOnRad anAlgo = new(new Point2D(0, 3), anOn, 1.0, Tol);
        Assert.True(anAlgo.IsDone);
        Assert.Equal(2, anAlgo.SolutionCount);
        for (int i = 1; i <= 2; i++)
        {
            Assert.Equal(1.0, Math.Abs(anAlgo.ThisSolution(i).Location.Y - 3), 1.0e-9);
            CheckCentreOnLine(anAlgo, i, anOn);
            anAlgo.Tangency1(i, out _, out _, out Point2D aPnt);
            AssertNear(aPnt, new Point2D(0, 3));
        }
    }

    // Unit circle at the origin, centres on the circle at (3, 0) of radius 1, radius 1.5: outside the unit circle at
    // distance 2.5 (2); none encloses or is enclosed by it.
    [Theory]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Unqualified, 2)]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, 2)]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing, 0)]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed, 0)]
    public void CircleCentreOnCircle(global::Kerf.Engine.Construction.Constraints.Qualifier theQ, int theNb)
    {
        Circle2D aC = Circ(0, 0, 1);
        Circle2D anOn = Circ(3, 0, 1);
        AnalyticCirc2dTanOnRad anAlgo = new(new global::Kerf.Engine.Construction.Constraints.QualifiedCirc(aC, theQ), anOn, 1.5, Tol);
        Assert.True(anAlgo.IsDone);
        Assert.Equal(theNb, anAlgo.SolutionCount);
        for (int i = 1; i <= theNb; i++)
        {
            Assert.Equal(1.5, anAlgo.ThisSolution(i).Radius);
            CheckCentreOnCirc(anAlgo, i, anOn);
            CheckTangentCirc(anAlgo, i, aC);
        }
    }

    // Unit circles at the origin and at (1, 0), radius just over 3 (within the tolerance): the circle centred at (2, 0)
    // encloses the first. OCCT wrote the qualifier at index 0 and threw Standard_OutOfRange (decision 17104).
    [Fact]
    public void CircleCentreOnCircle_EnclosingAll()
    {
        Circle2D aC = Circ(0, 0, 1);
        Circle2D anOn = Circ(1, 0, 1);
        AnalyticCirc2dTanOnRad anAlgo = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aC), anOn, 3.0 + 5.0e-10, Tol);
        Assert.Equal(1, anAlgo.SolutionCount);
        AssertNear(anAlgo.ThisSolution(1).Location, new Point2D(2, 0));
        anAlgo.WhichQualifier(1, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ1);
        Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing, aQ1);
        CheckCentreOnCirc(anAlgo, 1, anOn);
    }

    // The X axis, centres on the circle of radius 2 at the origin, radius 1: (+-sqrt(3), +-1). OCCT truncated a dot
    // product into its side (0), put the tangency point at the centre and threw taking its parameter (decision 17104)
    [Theory]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Unqualified, 4)]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed, 2)]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, 2)]
    public void LineCentreOnCircle(global::Kerf.Engine.Construction.Constraints.Qualifier theQ, int theNb)
    {
        Line2D aL = Lin(0, 0, 1, 0);
        Circle2D anOn = Circ(0, 0, 2);
        AnalyticCirc2dTanOnRad anAlgo = new(new global::Kerf.Engine.Construction.Constraints.QualifiedLin(aL, theQ), anOn, 1.0, Tol);
        Assert.Equal(theNb, anAlgo.SolutionCount);
        for (int i = 1; i <= theNb; i++)
        {
            Circle2D aSol = anAlgo.ThisSolution(i);
            Assert.Equal(Math.Sqrt(3.0), Math.Abs(aSol.Location.X), 1.0e-9);
            CheckCentreOnCirc(anAlgo, i, anOn);
            anAlgo.WhichQualifier(i, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ1);
            Assert.Equal(aSol.Location.Y > 0 ? global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed : global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, aQ1);
            anAlgo.Tangency1(i, out double aParSol, out double aParArg, out Point2D aPnt);
            AssertNear(aPnt, new Point2D(aSol.Location.X, 0));
            AssertNear(global::Kerf.Engine.Geometry.ElCLib.Value(aParSol, aSol), aPnt);
            AssertNear(global::Kerf.Engine.Geometry.ElCLib.Value(aParArg, aL), aPnt);
        }
    }



    // The point (1, 1), centres on the circle at (3, 0) of radius 1, radius 2.5: two circles
    [Fact]
    public void PointCentreOnCircle()
    {
        Point2D aP = new(1, 1);
        Circle2D anOn = Circ(3, 0, 1);
        AnalyticCirc2dTanOnRad anAlgo = new(aP, anOn, 2.5, Tol);
        Assert.True(anAlgo.IsDone);
        Assert.Equal(2, anAlgo.SolutionCount);
        for (int i = 1; i <= 2; i++)
        {
            Circle2D aSol = anAlgo.ThisSolution(i);
            Assert.Equal(2.5, aSol.Radius);
            Assert.Equal(2.5, aSol.Location.DistanceTo(aP), 1.0e-9);
            Assert.True(anOn.DistanceTo(aSol.Location) < 1.0e-9);
        }
        // each solution's tangency data (OCCT filled the first's only: decision 17104)
        for (int i = 1; i <= 2; i++)
        {
            anAlgo.Tangency1(i, out double aParSol, out _, out Point2D aPnt);
            AssertNear(aPnt, aP);
            AssertNear(global::Kerf.Engine.Geometry.ElCLib.Value(aParSol, anAlgo.ThisSolution(i)), aP);
        }
    }

    // The origin, centres on the circle at (3, 0) of radius 1, radius 2: the one centre at (2, 0)
    [Fact]
    public void PointCentreOnCircle_Tangent()
    {
        AnalyticCirc2dTanOnRad anAlgo = new(new Point2D(0, 0), Circ(3, 0, 1), 2.0, Tol);
        Assert.True(anAlgo.IsDone);
        Assert.Equal(1, anAlgo.SolutionCount);
        AssertNear(anAlgo.ThisSolution(1).Location, new Point2D(2, 0));
    }

    [Fact]
    public void Errors()
    {
        Line2D aL = Lin(0, 0, 1, 0);
        Circle2D aC = Circ(0, 0, 1);
        Assert.Throws<NegativeValueException>(() => new AnalyticCirc2dTanOnRad(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aC), aL, -1.0, Tol));
        Assert.Throws<NegativeValueException>(() => new AnalyticCirc2dTanOnRad(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aL), Lin(0, 0, 0, 1), -1.0, Tol));
        Assert.Throws<NegativeValueException>(() => new AnalyticCirc2dTanOnRad(new Point2D(1, 0), aL, -1.0, Tol));
        Assert.Throws<NegativeValueException>(() => new AnalyticCirc2dTanOnRad(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aC), Circ(3, 0, 1), -1.0, Tol));
        Assert.Throws<NegativeValueException>(() => new AnalyticCirc2dTanOnRad(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aL), aC, -1.0, Tol));
        Assert.Throws<NegativeValueException>(() => new AnalyticCirc2dTanOnRad(new Point2D(1, 0), Circ(3, 0, 1), -1.0, Tol));
        global::Kerf.Engine.Construction.Constraints.QualifiedLin aBad = new(aL, global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing);
        Assert.Throws<global::Kerf.Engine.Construction.Constraints.BadQualifierException>(() => new AnalyticCirc2dTanOnRad(aBad, Lin(0, 0, 0, 1), 1.0, Tol));
        Assert.Throws<global::Kerf.Engine.Construction.Constraints.BadQualifierException>(() => new AnalyticCirc2dTanOnRad(aBad, aC, 1.0, Tol));
        global::Kerf.Engine.Construction.Constraints.QualifiedCirc aNoQual = new(aC, global::Kerf.Engine.Construction.Constraints.Qualifier.Noqualifier);
        Assert.Throws<global::Kerf.Engine.Construction.Constraints.BadQualifierException>(() => new AnalyticCirc2dTanOnRad(aNoQual, aL, 1.0, Tol));
        Assert.Throws<global::Kerf.Engine.Construction.Constraints.BadQualifierException>(() => new AnalyticCirc2dTanOnRad(aNoQual, Circ(3, 0, 1), 1.0, Tol));

        AnalyticCirc2dTanOnRad anAlgo = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aC), aL, 0.4, Tol);
        Assert.Throws<OutOfRangeException>(() => anAlgo.ThisSolution(0));
        Assert.Throws<OutOfRangeException>(() => anAlgo.ThisSolution(5));
        Assert.Throws<OutOfRangeException>(() => anAlgo.WhichQualifier(5, out _));
        Assert.Throws<OutOfRangeException>(() => anAlgo.Tangency1(5, out _, out _, out _));
        Assert.Throws<OutOfRangeException>(() => anAlgo.CenterOn3(0, out _, out _));
        Assert.Throws<OutOfRangeException>(() => anAlgo.IsTheSame1(5));
    }

    // The unit circle at (1, 0), enclosed, centres on the unit circle at the origin, radius 1: the argument itself
    [Fact]
    public void CircleCentreOnCircle_SolutionIsTheArgument()
    {
        Circle2D aC = Circ(1, 0, 1);
        AnalyticCirc2dTanOnRad anAlgo = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosed(aC), Circ(0, 0, 1), 1.0, Tol);
        Assert.True(anAlgo.IsDone);
        Assert.Equal(1, anAlgo.SolutionCount);
        Assert.True(anAlgo.IsTheSame1(1));
        Assert.Equal(aC, anAlgo.ThisSolution(1));
    }
}
