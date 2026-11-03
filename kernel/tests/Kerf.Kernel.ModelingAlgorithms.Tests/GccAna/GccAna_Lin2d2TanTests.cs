// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Constraints.Analytic;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// GccAna_Lin2d2Tan on OCCT's documented configurations: solution counts, qualifiers, and on every solution the
/// tangencies (or pass-through) and the tangency points with their parameters.
/// </summary>
public class GccAna_Lin2d2TanTests
{
    private const double Tol = 1.0e-7;
    private static readonly Direction2D DX = new(Direction2D.D.X);

    private static Circle2D Circ(double x, double y, double r) => new(new Axis2D(new Point2D(x, y), DX), r);

    private static void AssertNear(in Point2D theP, in Point2D theQ) =>
        Assert.True(theP.DistanceTo(theQ) < Tol, $"{theP} != {theQ}");

    // The side of the line the point is on: the left (inside) is positive
    private static double Side(in Line2D theL, in Point2D theP) =>
        new Vector2D(theL.Direction).Crossed(new Vector2D(theL.Location, theP));

    private static void CheckTangentToCircle(in Line2D theSol, in Circle2D theC, double theParSol, double theParArg,
                                             in Point2D thePnt)
    {
        Assert.Equal(theC.Radius, theSol.DistanceTo(theC.Location), Tol);
        Assert.True(theSol.DistanceTo(thePnt) < Tol);
        Assert.Equal(theC.Radius, thePnt.DistanceTo(theC.Location), Tol);
        AssertNear(global::Kerf.Engine.Geometry.ElCLib.Value(theParSol, theSol), thePnt);
        AssertNear(global::Kerf.Engine.Geometry.ElCLib.Value(theParArg, theC), thePnt);
    }

    private static void CheckTwoCircles(AnalyticLin2d2Tan theAlgo, in Circle2D theC1, in Circle2D theC2)
    {
        for (int i = 1; i <= theAlgo.SolutionCount; i++)
        {
            Line2D aSol = theAlgo.ThisSolution(i);
            theAlgo.Tangency1(i, out double aPs1, out double aPa1, out Point2D aP1);
            theAlgo.Tangency2(i, out double aPs2, out double aPa2, out Point2D aP2);
            CheckTangentToCircle(aSol, theC1, aPs1, aPa1, aP1);
            CheckTangentToCircle(aSol, theC2, aPs2, aPa2, aP2);
        }
    }

    [Fact]
    public void TwoPoints_OneLine()
    {
        Point2D aP1 = new(1, 1), aP2 = new(4, 5);
        AnalyticLin2d2Tan anAlgo = new(aP1, aP2, 1.0e-6);
        Assert.True(anAlgo.IsDone);
        Assert.Equal(1, anAlgo.SolutionCount);
        Line2D aSol = anAlgo.ThisSolution(1);
        Assert.True(aSol.DistanceTo(aP1) < Tol && aSol.DistanceTo(aP2) < Tol);
        anAlgo.Tangency1(1, out double aPs1, out double aPa1, out Point2D aT1);
        anAlgo.Tangency2(1, out double aPs2, out double aPa2, out Point2D aT2);
        AssertNear(aT1, aP1);
        AssertNear(aT2, aP2);
        Assert.Equal(0.0, aPs1, Tol);
        Assert.Equal(5.0, aPs2, Tol);
        Assert.Equal(0.0, aPa1);
        Assert.Equal(0.0, aPa2);
        anAlgo.WhichQualifier(1, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ1, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ2);
        Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Noqualifier, aQ1);
        Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Noqualifier, aQ2);
    }

    [Fact]
    public void TwoPoints_Coincident_NotDone()
    {
        AnalyticLin2d2Tan anAlgo = new(new Point2D(1, 1), new Point2D(1, 1 + 1.0e-9), 1.0e-6);
        Assert.False(anAlgo.IsDone);
        Assert.Equal(0, anAlgo.SolutionCount);
        Assert.Throws<NotDoneException>(() => anAlgo.Tangency1(1, out _, out _, out _));
    }

    [Fact]
    public void CirclePoint_Unqualified_TwoTangents()
    {
        Circle2D aC = Circ(0, 0, 1);
        Point2D aP = new(3, 1);
        AnalyticLin2d2Tan anAlgo = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aC), aP, 1.0e-6);
        Assert.True(anAlgo.IsDone);
        Assert.Equal(2, anAlgo.SolutionCount);
        for (int i = 1; i <= 2; i++)
        {
            Line2D aSol = anAlgo.ThisSolution(i);
            anAlgo.Tangency1(i, out double aPs, out double aPa, out Point2D aT);
            CheckTangentToCircle(aSol, aC, aPs, aPa, aT);
            anAlgo.Tangency2(i, out double aPs2, out double aPa2, out Point2D aT2);
            AssertNear(aT2, aP);
            AssertNear(global::Kerf.Engine.Geometry.ElCLib.Value(aPs2, aSol), aP);
            Assert.Equal(0.0, aPa2);
            // OCCT returns the qualifier given, not a computed one
            anAlgo.WhichQualifier(i, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ1, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ2);
            Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Unqualified, aQ1);
            Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Noqualifier, aQ2);
        }
        Assert.False(anAlgo.ThisSolution(1).Direction.IsParallel(anAlgo.ThisSolution(2).Direction, 1.0e-6));
    }

    [Fact]
    public void CirclePoint_Qualified_OneTangentEach()
    {
        Circle2D aC = Circ(0, 0, 1);
        Point2D aP = new(3, 1);
        AnalyticLin2d2Tan anEnc = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosing(aC), aP, 1.0e-6);
        AnalyticLin2d2Tan anOut = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Outside(aC), aP, 1.0e-6);
        Assert.Equal(1, anEnc.SolutionCount);
        Assert.Equal(1, anOut.SolutionCount);
        // The circle is on the left (inside) of an enclosing solution and on the right of an outside one
        Assert.True(Side(anEnc.ThisSolution(1), aC.Location) > 0.0);
        Assert.True(Side(anOut.ThisSolution(1), aC.Location) < 0.0);
        anEnc.WhichQualifier(1, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ, out _);
        Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing, aQ);
        anOut.WhichQualifier(1, out aQ, out _);
        Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, aQ);
        for (int k = 0; k < 2; k++)
        {
            AnalyticLin2d2Tan anAlgo = k == 0 ? anEnc : anOut;
            anAlgo.Tangency1(1, out double aPs, out double aPa, out Point2D aT);
            CheckTangentToCircle(anAlgo.ThisSolution(1), aC, aPs, aPa, aT);
            Assert.True(anAlgo.ThisSolution(1).DistanceTo(aP) < Tol);
        }
    }

    [Fact]
    public void CirclePoint_PointInside_NoSolution()
    {
        AnalyticLin2d2Tan anAlgo = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(Circ(0, 0, 2)), new Point2D(0.5, 0.5), 1.0e-6);
        Assert.True(anAlgo.IsDone);
        Assert.Equal(0, anAlgo.SolutionCount);
    }

    [Fact]
    public void CirclePoint_PointOnCircle_OneTangent()
    {
        Circle2D aC = Circ(1, 1, 2);
        Point2D aP = new(3, 1);
        AnalyticLin2d2Tan anAlgo = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aC), aP, 1.0e-6);
        Assert.Equal(1, anAlgo.SolutionCount);
        anAlgo.Tangency1(1, out double aPs, out double aPa, out Point2D aT);
        CheckTangentToCircle(anAlgo.ThisSolution(1), aC, aPs, aPa, aT);
        AssertNear(aT, aP);
    }

    [Fact]
    public void BadQualifiers_Throw()
    {
        Circle2D aC = Circ(0, 0, 1);
        Assert.Throws<global::Kerf.Engine.Construction.Constraints.BadQualifierException>(() => new AnalyticLin2d2Tan(global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosed(aC), new Point2D(3, 0), 1.0e-6));
        Assert.Throws<global::Kerf.Engine.Construction.Constraints.BadQualifierException>(
            () => new AnalyticLin2d2Tan(new global::Kerf.Engine.Construction.Constraints.QualifiedCirc(aC, global::Kerf.Engine.Construction.Constraints.Qualifier.Noqualifier), new Point2D(3, 0), 1.0e-6));
        Assert.Throws<global::Kerf.Engine.Construction.Constraints.BadQualifierException>(
            () => new AnalyticLin2d2Tan(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aC), global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosed(Circ(5, 0, 1)), 1.0e-6));
    }

    [Fact]
    public void TwoExternalCircles_Unqualified_FourTangents()
    {
        Circle2D aC1 = Circ(0, 0, 1), aC2 = Circ(5, 1, 1.5);
        AnalyticLin2d2Tan anAlgo = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aC1), global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aC2), 1.0e-6);
        Assert.True(anAlgo.IsDone);
        Assert.Equal(4, anAlgo.SolutionCount);
        CheckTwoCircles(anAlgo, aC1, aC2);
        // Two external tangents (both centres on one side) and two internal ones
        int anInternal = 0;
        for (int i = 1; i <= 4; i++)
        {
            Line2D aSol = anAlgo.ThisSolution(i);
            if (Side(aSol, aC1.Location) * Side(aSol, aC2.Location) < 0.0)
            {
                anInternal++;
            }
        }
        Assert.Equal(2, anInternal);
    }

    [Theory]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing, global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing)]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing, global::Kerf.Engine.Construction.Constraints.Qualifier.Outside)]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing)]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, global::Kerf.Engine.Construction.Constraints.Qualifier.Outside)]
    public void TwoExternalCircles_Qualified_OneTangent(global::Kerf.Engine.Construction.Constraints.Qualifier theQ1, global::Kerf.Engine.Construction.Constraints.Qualifier theQ2)
    {
        Circle2D aC1 = Circ(0, 0, 1), aC2 = Circ(5, 1, 1.5);
        AnalyticLin2d2Tan anAlgo = new(new global::Kerf.Engine.Construction.Constraints.QualifiedCirc(aC1, theQ1), new global::Kerf.Engine.Construction.Constraints.QualifiedCirc(aC2, theQ2), 1.0e-6);
        Assert.True(anAlgo.IsDone);
        Assert.Equal(1, anAlgo.SolutionCount);
        CheckTwoCircles(anAlgo, aC1, aC2);
        Line2D aSol = anAlgo.ThisSolution(1);
        Assert.Equal(theQ1 == global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing, Side(aSol, aC1.Location) > 0.0);
        Assert.Equal(theQ2 == global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing, Side(aSol, aC2.Location) > 0.0);
        // OCCT: the second qualifier is the first argument's
        anAlgo.WhichQualifier(1, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ1, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ2);
        Assert.Equal(theQ1, aQ1);
        Assert.Equal(theQ1, aQ2);
    }

    [Fact]
    public void SecantCircles_Unqualified_TwoTangents()
    {
        Circle2D aC1 = Circ(0, 0, 2), aC2 = Circ(3, 0, 2);
        AnalyticLin2d2Tan anAlgo = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aC1), global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aC2), 1.0e-6);
        Assert.Equal(2, anAlgo.SolutionCount);
        CheckTwoCircles(anAlgo, aC1, aC2);
    }

    [Fact]
    public void NestedCircles_NoSolution()
    {
        AnalyticLin2d2Tan anAlgo = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(Circ(0, 0, 5)), global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(Circ(1, 0, 1)), 1.0e-6);
        Assert.True(anAlgo.IsDone);
        Assert.Equal(0, anAlgo.SolutionCount);
    }

    [Fact]
    public void InternallyTangentCircles_Enclosing_OneTangent()
    {
        Circle2D aC1 = Circ(0, 0, 3), aC2 = Circ(1, 0, 2);
        AnalyticLin2d2Tan anAlgo = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosing(aC1), global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosing(aC2), 1.0e-6);
        Assert.Equal(1, anAlgo.SolutionCount);
        CheckTwoCircles(anAlgo, aC1, aC2);
        anAlgo.Tangency1(1, out _, out _, out Point2D aT1);
        AssertNear(aT1, new Point2D(3, 0));
    }

    [Fact]
    public void Mixed_EnclosingUnqualified_TwoTangents_SecondTangencyPointIsTheFirsts()
    {
        Circle2D aC1 = Circ(0, 0, 1), aC2 = Circ(5, 1, 1.5);
        AnalyticLin2d2Tan anAlgo = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosing(aC1), global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aC2), 1.0e-6);
        Assert.Equal(2, anAlgo.SolutionCount);
        for (int i = 1; i <= 2; i++)
        {
            Line2D aSol = anAlgo.ThisSolution(i);
            Assert.Equal(aC1.Radius, aSol.DistanceTo(aC1.Location), Tol);
            Assert.Equal(aC2.Radius, aSol.DistanceTo(aC2.Location), Tol);
            Assert.True(Side(aSol, aC1.Location) > 0.0);
            anAlgo.Tangency2(i, out double aPs2, out double aPa2, out Point2D aT2);
            CheckTangentToCircle(aSol, aC2, aPs2, aPa2, aT2);
        }
        // OCCT: the second solution's first tangency point is the first solution's
        anAlgo.Tangency1(1, out _, out _, out Point2D aT11);
        anAlgo.Tangency1(2, out _, out _, out Point2D aT21);
        Assert.Equal(aT11, aT21);
        Assert.False(anAlgo.ThisSolution(2).DistanceTo(aT21) < Tol);
    }

    [Fact]
    public void Accessors_OutOfRange()
    {
        AnalyticLin2d2Tan anAlgo = new(new Point2D(0, 0), new Point2D(1, 0), 1.0e-6);
        Assert.Throws<OutOfRangeException>(() => anAlgo.ThisSolution(2));
        Assert.Throws<OutOfRangeException>(() => anAlgo.ThisSolution(0));
        Assert.Throws<OutOfRangeException>(() => anAlgo.WhichQualifier(2, out _, out _));
        Assert.Throws<OutOfRangeException>(() => anAlgo.Tangency1(2, out _, out _, out _));
        Assert.Throws<OutOfRangeException>(() => anAlgo.Tangency2(2, out _, out _, out _));
    }
}
