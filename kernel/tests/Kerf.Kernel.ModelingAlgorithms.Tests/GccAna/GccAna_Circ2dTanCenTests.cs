// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Constraints.Analytic;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>GccAna_Circ2dTanCen: circles with a given centre, tangent to a circle or a line, or through a point.</summary>
public class GccAna_Circ2dTanCenTests
{
    private const double Tol = 1.0e-9;
    private static readonly Direction2D DX = new(Direction2D.D.X);

    private static Circle2D Circ(double x, double y, double r) => new(new Axis2D(new Point2D(x, y), DX), r);

    private static void CheckCircle(AnalyticCirc2dTanCen theAlgo, int theIndex, in Circle2D theArg, in Point2D theCentre)
    {
        Circle2D aSol = theAlgo.ThisSolution(theIndex);
        Assert.Equal(theCentre, aSol.Location);
        double d = theArg.Location.DistanceTo(theCentre);
        theAlgo.WhichQualifier(theIndex, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ);
        double anExpected = aQ switch
        {
            global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed => theArg.Radius - d,
            global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing => theArg.Radius + d,
            _ => d - theArg.Radius,
        };
        Assert.Equal(anExpected, aSol.Radius, Tol);
        theAlgo.Tangency1(theIndex, out double aPs, out double aPa, out Point2D aT);
        Assert.Equal(theArg.Radius, aT.DistanceTo(theArg.Location), Tol);
        Assert.Equal(aSol.Radius, aT.DistanceTo(theCentre), Tol);
        Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aPs, aSol).DistanceTo(aT) < Tol);
        Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aPa, theArg).DistanceTo(aT) < Tol);
    }

    [Fact]
    public void Circle_CentreOutside_Unqualified_Two()
    {
        Circle2D aC = Circ(0, 0, 1);
        Point2D aP = new(3, 4);
        AnalyticCirc2dTanCen anAlgo = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aC), aP, 1.0e-9);
        Assert.True(anAlgo.IsDone);
        Assert.Equal(2, anAlgo.SolutionCount);
        CheckCircle(anAlgo, 1, aC, aP);
        CheckCircle(anAlgo, 2, aC, aP);
        Assert.Equal(4.0, anAlgo.ThisSolution(1).Radius, Tol);
        Assert.Equal(6.0, anAlgo.ThisSolution(2).Radius, Tol);
        anAlgo.WhichQualifier(1, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ1);
        anAlgo.WhichQualifier(2, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ2);
        Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, aQ1);
        Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing, aQ2);
        Assert.False(anAlgo.IsTheSame1(1));
    }

    [Fact]
    public void Circle_CentreInside_Unqualified_Two()
    {
        Circle2D aC = Circ(1, 1, 5);
        Point2D aP = new(2, 1);
        AnalyticCirc2dTanCen anAlgo = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aC), aP, 1.0e-9);
        Assert.Equal(2, anAlgo.SolutionCount);
        CheckCircle(anAlgo, 1, aC, aP);
        CheckCircle(anAlgo, 2, aC, aP);
        anAlgo.WhichQualifier(1, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ1);
        anAlgo.WhichQualifier(2, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ2);
        Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed, aQ1);
        Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing, aQ2);
    }

    [Theory]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed, 0.5, 1)]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed, 3.0, 0)]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing, 0.5, 1)]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing, 3.0, 1)]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, 0.5, 0)]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, 3.0, 1)]
    public void Circle_Qualified(object theQValue, double theX, int theNb)
    {
        var theQ = (global::Kerf.Engine.Construction.Constraints.Qualifier)theQValue; // the theory's data as object: the engine type is internal
        Circle2D aC = Circ(0, 0, 2);
        Point2D aP = new(theX, 0);
        AnalyticCirc2dTanCen anAlgo = new(new global::Kerf.Engine.Construction.Constraints.QualifiedCirc(aC, theQ), aP, 1.0e-9);
        Assert.True(anAlgo.IsDone);
        Assert.Equal(theNb, anAlgo.SolutionCount);
        for (int i = 1; i <= theNb; i++)
        {
            anAlgo.WhichQualifier(i, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ);
            Assert.Equal(theQ, aQ);
            CheckCircle(anAlgo, i, aC, aP);
        }
    }

    [Fact]
    public void Circle_Concentric_Unqualified_IsTheSame()
    {
        Circle2D aC = Circ(1, 2, 3);
        AnalyticCirc2dTanCen anAlgo = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aC), new Point2D(1, 2), 1.0e-9);
        Assert.Equal(1, anAlgo.SolutionCount);
        Assert.Equal(aC, anAlgo.ThisSolution(1));
        Assert.True(anAlgo.IsTheSame1(1));
        Assert.Throws<NotDoneException>(() => anAlgo.Tangency1(1, out _, out _, out _));
    }

    [Fact]
    public void Circle_Concentric_Enclosing_IsTheSame()
    {
        Circle2D aC = Circ(1, 2, 3);
        AnalyticCirc2dTanCen anAlgo = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosing(aC), new Point2D(1, 2), 1.0e-9);
        Assert.Equal(1, anAlgo.SolutionCount);
        Assert.Equal(3.0, anAlgo.ThisSolution(1).Radius);
        Assert.True(anAlgo.IsTheSame1(1));
    }

    // The centre on the circle: a solution of radius 0, whose tangency point is its centre. ElCLib's parameter there
    // normalises a null vector: native OCCT throws Standard_ConstructionError, as the port does
    [Fact]
    public void Circle_CentreOnTheCircle_Throws()
    {
        Circle2D aC = Circ(0, 0, 2);
        Assert.Throws<ConstructionErrorException>(() => new AnalyticCirc2dTanCen(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aC), new Point2D(2, 0), 1.0e-9));
    }

    [Fact]
    public void Line_Tangent()
    {
        Line2D aL = new(new Point2D(0, 1), new Direction2D(1, 1));
        foreach (Point2D aP in new[] { new Point2D(3, 0), new Point2D(-2, 5) })
        {
            AnalyticCirc2dTanCen anAlgo = new(aL, aP);
            Assert.True(anAlgo.IsDone);
            Assert.Equal(1, anAlgo.SolutionCount);
            Circle2D aSol = anAlgo.ThisSolution(1);
            Assert.Equal(aL.DistanceTo(aP), aSol.Radius, Tol);
            anAlgo.Tangency1(1, out double aPs, out double aPa, out Point2D aT);
            Assert.True(aL.DistanceTo(aT) < Tol);
            Assert.Equal(aSol.Radius, aT.DistanceTo(aP), Tol);
            Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aPs, aSol).DistanceTo(aT) < Tol);
            Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aPa, aL).DistanceTo(aT) < Tol);
            anAlgo.WhichQualifier(1, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ);
            Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Noqualifier, aQ);
        }
    }

    // The centre at the line's location: OCCT's side test normalises a null vector, and native OCCT throws
    // Standard_ConstructionError, as the port does
    [Fact]
    public void Line_CentreAtTheLinesLocation_Throws()
    {
        Line2D aL = new(new Point2D(1, 1), DX);
        Assert.Throws<ConstructionErrorException>(() => new AnalyticCirc2dTanCen(aL, new Point2D(1, 1)));
    }

    [Fact]
    public void Point_Through()
    {
        AnalyticCirc2dTanCen anAlgo = new(new Point2D(4, 5), new Point2D(1, 1));
        Assert.Equal(1, anAlgo.SolutionCount);
        Assert.Equal(5.0, anAlgo.ThisSolution(1).Radius, Tol);
        anAlgo.Tangency1(1, out double aPs, out double aPa, out Point2D aT);
        Assert.Equal(new Point2D(4, 5), aT);
        Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aPs, anAlgo.ThisSolution(1)).DistanceTo(aT) < Tol);
        Assert.Equal(0.0, aPa);
    }

    [Fact]
    public void Errors()
    {
        Assert.Throws<global::Kerf.Engine.Construction.Constraints.BadQualifierException>(
            () => new AnalyticCirc2dTanCen(new global::Kerf.Engine.Construction.Constraints.QualifiedCirc(Circ(0, 0, 1), global::Kerf.Engine.Construction.Constraints.Qualifier.Noqualifier), new Point2D(3, 0), 1.0e-9));
        AnalyticCirc2dTanCen anAlgo = new(new Point2D(4, 5), new Point2D(1, 1));
        Assert.Throws<OutOfRangeException>(() => anAlgo.ThisSolution(2));
        Assert.Throws<OutOfRangeException>(() => anAlgo.WhichQualifier(0, out _));
        Assert.Throws<OutOfRangeException>(() => anAlgo.Tangency1(2, out _, out _, out _));
        Assert.Throws<OutOfRangeException>(() => anAlgo.IsTheSame1(2));
    }
}
