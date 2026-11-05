// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Constraints.Analytic;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>GccAna_Lin2dTanPar: lines parallel to a line, through a point or tangent to a circle.</summary>
public class GccAna_Lin2dTanParTests
{
    private const double Tol = 1.0e-9;
    private static readonly Direction2D DX = new(Direction2D.D.X);

    private static Circle2D Circ(double x, double y, double r) => new(new Axis2D(new Point2D(x, y), DX), r);

    private static double Side(in Line2D theL, in Point2D theP) =>
        new Vector2D(theL.Direction).Crossed(new Vector2D(theL.Location, theP));

    [Fact]
    public void ThroughPoint_OneParallel()
    {
        Line2D aL = new(new Point2D(0, 0), new Direction2D(1, 2));
        Point2D aP = new(3, -1);
        Lin2dTanPar anAlgo = new(aP, aL);
        Assert.True(anAlgo.IsDone);
        Assert.Equal(1, anAlgo.SolutionCount);
        Line2D aSol = anAlgo.ThisSolution(1);
        Assert.True(aSol.Direction.IsEqual(aL.Direction, 1.0e-12));
        Assert.True(aSol.DistanceTo(aP) < Tol);
        anAlgo.Tangency1(1, out double aPs, out double aPa, out Point2D aT);
        Assert.Equal(aP, aT);
        Assert.Equal(0.0, aPs);
        Assert.Equal(0.0, aPa);
        anAlgo.WhichQualifier(1, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ);
        Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Noqualifier, aQ);
    }

    [Fact]
    public void TangentToCircle_Unqualified_Two()
    {
        Line2D aL = new(new Point2D(0, 0), new Direction2D(1, 1));
        Circle2D aC = Circ(2, -1, 1.5);
        Lin2dTanPar anAlgo = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aC), aL);
        Assert.Equal(2, anAlgo.SolutionCount);
        for (int i = 1; i <= 2; i++)
        {
            Line2D aSol = anAlgo.ThisSolution(i);
            Assert.True(aSol.Direction.IsEqual(aL.Direction, 1.0e-12));
            Assert.Equal(aC.Radius, aSol.DistanceTo(aC.Location), Tol);
            anAlgo.Tangency1(i, out double aPs, out double aPa, out Point2D aT);
            Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aPs, aSol).DistanceTo(aT) < Tol);
            Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aPa, aC).DistanceTo(aT) < Tol);
            anAlgo.WhichQualifier(i, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ);
            // enclosing: the circle on the left (inside) of the solution
            Assert.Equal(aQ == global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing, Side(aSol, aC.Location) > 0.0);
        }
        anAlgo.WhichQualifier(1, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ1);
        anAlgo.WhichQualifier(2, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ2);
        Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, aQ1);
        Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing, aQ2);
    }

    [Fact]
    public void TangentToCircle_Qualified_One()
    {
        Line2D aL = new(new Point2D(0, 0), new Direction2D(1, 0));
        Circle2D aC = Circ(0, 3, 1);
        Lin2dTanPar anEnc = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosing(aC), aL);
        Lin2dTanPar anOut = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Outside(aC), aL);
        Assert.Equal(1, anEnc.SolutionCount);
        Assert.Equal(1, anOut.SolutionCount);
        Assert.Equal(2.0, anEnc.ThisSolution(1).Location.Y, Tol);
        Assert.Equal(4.0, anOut.ThisSolution(1).Location.Y, Tol);
    }

    [Fact]
    public void BadQualifiers_Throw()
    {
        Line2D aL = new(new Point2D(0, 0), new Direction2D(1, 0));
        EngineAssert.Throws<global::Kerf.Engine.Construction.Constraints.BadQualifierException>(() => new Lin2dTanPar(global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosed(Circ(0, 3, 1)), aL));
        EngineAssert.Throws<global::Kerf.Engine.Construction.Constraints.BadQualifierException>(
            () => new Lin2dTanPar(new global::Kerf.Engine.Construction.Constraints.QualifiedCirc(Circ(0, 3, 1), global::Kerf.Engine.Construction.Constraints.Qualifier.Noqualifier), aL));
    }

    [Fact]
    public void Accessors_OutOfRange()
    {
        Lin2dTanPar anAlgo = new(new Point2D(1, 1), new Line2D(new Point2D(0, 0), DX));
        EngineAssert.Throws<OutOfRangeException>(() => anAlgo.ThisSolution(2));
        EngineAssert.Throws<OutOfRangeException>(() => anAlgo.WhichQualifier(0, out _));
        EngineAssert.Throws<OutOfRangeException>(() => anAlgo.Tangency1(2, out _, out _, out _));
    }
}
