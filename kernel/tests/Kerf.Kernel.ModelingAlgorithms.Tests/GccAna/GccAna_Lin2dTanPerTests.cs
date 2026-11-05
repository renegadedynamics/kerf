// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Constraints.Analytic;
using Kerf.Engine.Foundation;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// GccAna_Lin2dTanPer: lines through a point or tangent to a circle, perpendicular to a line or a circle.
/// </summary>
public class GccAna_Lin2dTanPerTests
{
    private const double Tol = 1.0e-9;
    private static readonly Direction2D DX = new(Direction2D.D.X);

    private static Circle2D Circ(double x, double y, double r) => new(new Axis2D(new Point2D(x, y), DX), r);

    private static void CheckTangency(Lin2dTanPer theAlgo, int theIndex, in Circle2D theC)
    {
        Line2D aSol = theAlgo.ThisSolution(theIndex);
        Assert.Equal(theC.Radius, aSol.DistanceTo(theC.Location), Tol);
        theAlgo.Tangency1(theIndex, out double aPs, out double aPa, out Point2D aT);
        Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aPs, aSol).DistanceTo(aT) < Tol);
        Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aPa, theC).DistanceTo(aT) < Tol);
    }

    [Fact]
    public void PointLine_Perpendicular()
    {
        Line2D aL = new(new Point2D(0, 0), new Direction2D(2, 1));
        Point2D aP = new(1, 4);
        Lin2dTanPer anAlgo = new(aP, aL);
        Assert.True(anAlgo.IsDone);
        Assert.Equal(1, anAlgo.SolutionCount);
        Line2D aSol = anAlgo.ThisSolution(1);
        Assert.Equal(0.0, aSol.Direction.Dot(aL.Direction), 1.0e-12);
        Assert.True(aSol.DistanceTo(aP) < Tol);
        anAlgo.Intersection2(1, out double aPs, out double aPa, out Point2D aI);
        Assert.True(aL.DistanceTo(aI) < Tol && aSol.DistanceTo(aI) < Tol);
        Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aPs, aSol).DistanceTo(aI) < Tol);
        Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aPa, aL).DistanceTo(aI) < Tol);
        anAlgo.Tangency1(1, out double aPs1, out double aPa1, out Point2D aT);
        Assert.Equal(aP, aT);
        Assert.Equal(0.0, aPs1, Tol);
        Assert.Equal(0.0, aPa1);
    }

    [Fact]
    public void PointCircle_ThroughTheCentre_NearestIntersection()
    {
        Circle2D aC = Circ(3, 0, 1);
        Point2D aP = new(0, 0);
        Lin2dTanPer anAlgo = new(aP, aC);
        Assert.Equal(1, anAlgo.SolutionCount);
        Line2D aSol = anAlgo.ThisSolution(1);
        Assert.True(aSol.DistanceTo(aC.Location) < Tol);
        anAlgo.Intersection2(1, out double aPs, out double aPa, out Point2D aI);
        Assert.Equal(aC.Radius, aI.DistanceTo(aC.Location), Tol);
        Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aPs, aSol).DistanceTo(aI) < Tol);
        Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aPa, aC).DistanceTo(aI) < Tol);
        // the nearer intersection (OCCT kept the last IntAna2d returned: decision 1544)
        Assert.True(aI.DistanceTo(new Point2D(2, 0)) < Tol);
    }

    [Fact]
    public void CircleLine_Unqualified_Two()
    {
        Line2D aL = new(new Point2D(0, -3), new Direction2D(1, 0.5));
        Circle2D aC = Circ(1, 1, 2);
        Lin2dTanPer anAlgo = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aC), aL);
        Assert.Equal(2, anAlgo.SolutionCount);
        for (int i = 1; i <= 2; i++)
        {
            Assert.Equal(0.0, anAlgo.ThisSolution(i).Direction.Dot(aL.Direction), 1.0e-12);
            CheckTangency(anAlgo, i, aC);
            anAlgo.Intersection2(i, out _, out _, out Point2D aI);
            Assert.True(aL.DistanceTo(aI) < Tol && anAlgo.ThisSolution(i).DistanceTo(aI) < Tol);
            // the side the circle is on (OCCT never set it: decision 1544)
            anAlgo.WhichQualifier(i, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ);
            Line2D aSol = anAlgo.ThisSolution(i);
            double aSide = aSol.Direction.XY() ^ (aC.Location.XY() - aSol.Location.XY());
            Assert.Equal(aSide > 0.0 ? global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing : global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, aQ);
        }
    }

    [Fact]
    public void CircleLine_Qualified_One()
    {
        Line2D aL = new(new Point2D(0, -3), DX);
        Circle2D aC = Circ(0, 0, 1);
        Lin2dTanPer anEnc = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosing(aC), aL);
        Lin2dTanPer anOut = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Outside(aC), aL);
        Assert.Equal(1, anEnc.SolutionCount);
        Assert.Equal(1, anOut.SolutionCount);
        CheckTangency(anEnc, 1, aC);
        CheckTangency(anOut, 1, aC);
        Assert.Equal(1.0, anEnc.ThisSolution(1).Location.X, Tol);
        Assert.Equal(-1.0, anOut.ThisSolution(1).Location.X, Tol);
    }

    [Fact]
    public void CircleCircle_ThroughTheCentre()
    {
        // A line perpendicular to a circle passes through its centre: the tangents to C1 from C2's centre, one with C1
        // on its left and one on its right (OCCT gave two lines perpendicular to the line of centres, whatever the
        // qualifier, missing C2's centre: decision 1544)
        Circle2D aC1 = Circ(0, 0, 1);
        Circle2D aC2 = Circ(4, 3, 6);
        foreach ((global::Kerf.Engine.Construction.Constraints.QualifiedCirc aQ, int aNb) in new[] { (global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aC1), 2), (global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosing(aC1), 1), (global::Kerf.Engine.Construction.Constraints.GccEnt.Outside(aC1), 1) })
        {
            Lin2dTanPer anAlgo = new(aQ, aC2);
            Assert.Equal(aNb, anAlgo.SolutionCount);
            for (int i = 1; i <= aNb; i++)
            {
                Line2D aSol = anAlgo.ThisSolution(i);
                Assert.True(aSol.DistanceTo(aC2.Location) < Tol);
                CheckTangency(anAlgo, i, aC1);
                anAlgo.Intersection2(i, out double aPs, out double aPa, out Point2D aI);
                Assert.Equal(aC2.Radius, aI.DistanceTo(aC2.Location), Tol);
                Assert.True(aSol.DistanceTo(aI) < Tol);
                Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aPa, aC2).DistanceTo(aI) < Tol);
                Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aPs, aSol).DistanceTo(aI) < Tol);
                anAlgo.WhichQualifier(i, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ1);
                double aSide = aSol.Direction.XY() ^ (aC1.Location.XY() - aSol.Location.XY());
                Assert.Equal(aSide > 0.0 ? global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing : global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, aQ1);
                if (!aQ.IsUnqualified)
                {
                    Assert.Equal(aQ.Qualifier(), aQ1);
                }
            }
        }
    }

    [Fact]
    public void BadQualifiers_Throw()
    {
        EngineAssert.Throws<global::Kerf.Engine.Construction.Constraints.BadQualifierException>(
            () => new Lin2dTanPer(global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosed(Circ(0, 0, 1)), new Line2D(new Point2D(0, 0), DX)));
        EngineAssert.Throws<global::Kerf.Engine.Construction.Constraints.BadQualifierException>(() => new Lin2dTanPer(global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosed(Circ(0, 0, 1)), Circ(4, 0, 1)));
    }

    [Fact]
    public void Accessors_OutOfRange()
    {
        Lin2dTanPer anAlgo = new(new Point2D(1, 1), new Line2D(new Point2D(0, 0), DX));
        // OCCT throws Standard_RangeError (OutOfRange's base) from ThisSolution
        EngineAssert.ThrowsAny<RangeErrorException>(() => anAlgo.ThisSolution(2));
        EngineAssert.Throws<OutOfRangeException>(() => anAlgo.WhichQualifier(0, out _));
        EngineAssert.Throws<OutOfRangeException>(() => anAlgo.Tangency1(2, out _, out _, out _));
        EngineAssert.Throws<OutOfRangeException>(() => anAlgo.Intersection2(2, out _, out _, out _));
    }
}
