// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Constraints.Analytic;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>GccAna_Lin2dTanObl: lines through a point or tangent to a circle, at an angle to a line.</summary>
public class GccAna_Lin2dTanOblTests
{
    private const double Tol = 1.0e-9;
    private static readonly Direction2D DX = new(Direction2D.D.X);

    private static Circle2D Circ(double x, double y, double r) => new(new Axis2D(new Point2D(x, y), DX), r);

    private static void CheckIntersection(AnalyticLin2dTanObl theAlgo, int theIndex, in Line2D theLine)
    {
        Line2D aSol = theAlgo.ThisSolution(theIndex);
        theAlgo.Intersection2(theIndex, out double aPs, out double aPa, out Point2D aP);
        Assert.True(aSol.Distance(aP) < Tol);
        Assert.True(theLine.Distance(aP) < Tol);
        Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aPs, aSol).Distance(aP) < Tol);
        Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aPa, theLine).Distance(aP) < Tol);
    }

    [Fact]
    public void ThroughPoint_AtAngle()
    {
        Line2D aL = new(new Point2D(1, 0), new Direction2D(1, 1));
        Point2D aP = new(0, 3);
        double anAngle = 0.4;
        AnalyticLin2dTanObl anAlgo = new(aP, aL, anAngle);
        Assert.True(anAlgo.IsDone);
        Assert.Equal(1, anAlgo.SolutionCount);
        Line2D aSol = anAlgo.ThisSolution(1);
        Assert.Equal(anAngle, aL.Direction().Angle(aSol.Direction()), 1.0e-12);
        Assert.True(aSol.Distance(aP) < Tol);
        anAlgo.Tangency1(1, out double aPs, out double aPa, out Point2D aT);
        Assert.Equal(aP, aT);
        Assert.Equal(0.0, aPs, Tol);
        Assert.Equal(0.0, aPa);
        CheckIntersection(anAlgo, 1, aL);
        anAlgo.WhichQualifier(1, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ);
        Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Noqualifier, aQ);
    }

    [Fact]
    public void ThroughPoint_Parallel_IntersectionIsTheOrigin()
    {
        Line2D aL = new(new Point2D(1, 0), DX);
        AnalyticLin2dTanObl anAlgo = new(new Point2D(0, 3), aL, 0.0);
        Assert.Equal(1, anAlgo.SolutionCount);
        // OCCT: no intersection, so the intersection data are the default point's
        anAlgo.Intersection2(1, out _, out _, out Point2D aP);
        Assert.Equal(new Point2D(), aP);
    }

    [Fact]
    public void TangentToCircle_Unqualified_Two_SecondIntersectionIsTheFirsts()
    {
        Line2D aL = new(new Point2D(0, -5), new Direction2D(1, 0.2));
        Circle2D aC = Circ(1, 2, 1.5);
        double anAngle = 1.0;
        AnalyticLin2dTanObl anAlgo = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aC), aL, anAngle);
        Assert.Equal(2, anAlgo.SolutionCount);
        for (int i = 1; i <= 2; i++)
        {
            Line2D aSol = anAlgo.ThisSolution(i);
            Assert.Equal(anAngle, aL.Direction().Angle(aSol.Direction()), 1.0e-12);
            Assert.Equal(aC.Radius(), aSol.Distance(aC.Location()), Tol);
            anAlgo.Tangency1(i, out double aPs, out double aPa, out Point2D aT);
            Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aPs, aSol).Distance(aT) < Tol);
            Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aPa, aC).Distance(aT) < Tol);
        }
        anAlgo.WhichQualifier(1, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ1);
        anAlgo.WhichQualifier(2, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ2);
        Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing, aQ1);
        Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, aQ2);
        CheckIntersection(anAlgo, 1, aL);
        // OCCT: the second solution's intersection is computed with the first solution
        anAlgo.Intersection2(1, out _, out _, out Point2D aP1);
        anAlgo.Intersection2(2, out _, out _, out Point2D aP2);
        Assert.Equal(aP1, aP2);
        Assert.False(anAlgo.ThisSolution(2).Distance(aP2) < Tol);
    }

    [Fact]
    public void TangentToCircle_Qualified_One()
    {
        Line2D aL = new(new Point2D(0, -5), DX);
        Circle2D aC = Circ(0, 0, 1);
        AnalyticLin2dTanObl anEnc = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosing(aC), aL, M_PI / 2.0);
        AnalyticLin2dTanObl anOut = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Outside(aC), aL, M_PI / 2.0);
        Assert.Equal(1, anEnc.SolutionCount);
        Assert.Equal(1, anOut.SolutionCount);
        // Vertical solutions going up: the circle is on the left of the enclosing one (x = 1)
        Assert.Equal(1.0, anEnc.ThisSolution(1).Location().X(), Tol);
        Assert.Equal(-1.0, anOut.ThisSolution(1).Location().X(), Tol);
        CheckIntersection(anEnc, 1, aL);
        CheckIntersection(anOut, 1, aL);
    }

    [Fact]
    public void BadQualifiers_Throw()
    {
        Line2D aL = new(new Point2D(0, 0), DX);
        Assert.Throws<global::Kerf.Engine.Construction.Constraints.BadQualifierException>(() => new AnalyticLin2dTanObl(global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosed(Circ(0, 3, 1)), aL, 0.5));
        Assert.Throws<global::Kerf.Engine.Construction.Constraints.BadQualifierException>(
            () => new AnalyticLin2dTanObl(new global::Kerf.Engine.Construction.Constraints.QualifiedCirc(Circ(0, 3, 1), global::Kerf.Engine.Construction.Constraints.Qualifier.Noqualifier), aL, 0.5));
    }

    [Fact]
    public void Accessors_OutOfRange()
    {
        AnalyticLin2dTanObl anAlgo = new(new Point2D(1, 1), new Line2D(new Point2D(0, 0), DX), 0.3);
        Assert.Throws<OutOfRangeException>(() => anAlgo.ThisSolution(2));
        Assert.Throws<OutOfRangeException>(() => anAlgo.WhichQualifier(0, out _));
        Assert.Throws<OutOfRangeException>(() => anAlgo.Tangency1(2, out _, out _, out _));
        Assert.Throws<OutOfRangeException>(() => anAlgo.Intersection2(2, out _, out _, out _));
    }
}
