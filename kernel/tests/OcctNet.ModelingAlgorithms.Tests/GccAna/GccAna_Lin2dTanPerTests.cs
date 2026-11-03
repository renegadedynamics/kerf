// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.GccEnt_Position;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// GccAna_Lin2dTanPer: lines through a point or tangent to a circle, perpendicular to a line or a circle.
/// </summary>
public class GccAna_Lin2dTanPerTests
{
    private const double Tol = 1.0e-9;
    private static readonly Direction2D DX = new(Direction2D.D.X);

    private static Circle2D Circ(double x, double y, double r) => new(new Axis2D(new Point2D(x, y), DX), r);

    private static void CheckTangency(GccAna_Lin2dTanPer theAlgo, int theIndex, in Circle2D theC)
    {
        Line2D aSol = theAlgo.ThisSolution(theIndex);
        Assert.Equal(theC.Radius(), aSol.Distance(theC.Location()), Tol);
        theAlgo.Tangency1(theIndex, out double aPs, out double aPa, out Point2D aT);
        Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aPs, aSol).Distance(aT) < Tol);
        Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aPa, theC).Distance(aT) < Tol);
    }

    [Fact]
    public void PointLine_Perpendicular()
    {
        Line2D aL = new(new Point2D(0, 0), new Direction2D(2, 1));
        Point2D aP = new(1, 4);
        GccAna_Lin2dTanPer anAlgo = new(aP, aL);
        Assert.True(anAlgo.IsDone());
        Assert.Equal(1, anAlgo.NbSolutions());
        Line2D aSol = anAlgo.ThisSolution(1);
        Assert.Equal(0.0, aSol.Direction().Dot(aL.Direction()), 1.0e-12);
        Assert.True(aSol.Distance(aP) < Tol);
        anAlgo.Intersection2(1, out double aPs, out double aPa, out Point2D aI);
        Assert.True(aL.Distance(aI) < Tol && aSol.Distance(aI) < Tol);
        Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aPs, aSol).Distance(aI) < Tol);
        Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aPa, aL).Distance(aI) < Tol);
        anAlgo.Tangency1(1, out double aPs1, out double aPa1, out Point2D aT);
        Assert.Equal(aP, aT);
        Assert.Equal(0.0, aPs1, Tol);
        Assert.Equal(0.0, aPa1);
    }

    [Fact]
    public void PointCircle_ThroughTheCentre_LastIntersection()
    {
        Circle2D aC = Circ(3, 0, 1);
        Point2D aP = new(0, 0);
        GccAna_Lin2dTanPer anAlgo = new(aP, aC);
        Assert.Equal(1, anAlgo.NbSolutions());
        Line2D aSol = anAlgo.ThisSolution(1);
        Assert.True(aSol.Distance(aC.Location()) < Tol);
        anAlgo.Intersection2(1, out double aPs, out double aPa, out Point2D aI);
        Assert.Equal(aC.Radius(), aI.Distance(aC.Location()), Tol);
        Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aPs, aSol).Distance(aI) < Tol);
        Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aPa, aC).Distance(aI) < Tol);
        // OCCT keeps the last intersection point IntAna2d returns, whatever its distance
        IntAna2d_AnaIntersection anInt = new(aSol, aC);
        Assert.Equal(anInt.Point(anInt.NbPoints()).Value(), aI);
    }

    [Fact]
    public void CircleLine_Unqualified_Two()
    {
        Line2D aL = new(new Point2D(0, -3), new Direction2D(1, 0.5));
        Circle2D aC = Circ(1, 1, 2);
        GccAna_Lin2dTanPer anAlgo = new(GccEnt.Unqualified(aC), aL);
        Assert.Equal(2, anAlgo.NbSolutions());
        for (int i = 1; i <= 2; i++)
        {
            Assert.Equal(0.0, anAlgo.ThisSolution(i).Direction().Dot(aL.Direction()), 1.0e-12);
            CheckTangency(anAlgo, i, aC);
            anAlgo.Intersection2(i, out _, out _, out Point2D aI);
            Assert.True(aL.Distance(aI) < Tol && anAlgo.ThisSolution(i).Distance(aI) < Tol);
            // OCCT: this constructor never sets the qualifier
            anAlgo.WhichQualifier(i, out GccEnt_Position aQ);
            Assert.Equal(GccEnt_unqualified, aQ);
        }
    }

    [Fact]
    public void CircleLine_Qualified_One()
    {
        Line2D aL = new(new Point2D(0, -3), DX);
        Circle2D aC = Circ(0, 0, 1);
        GccAna_Lin2dTanPer anEnc = new(GccEnt.Enclosing(aC), aL);
        GccAna_Lin2dTanPer anOut = new(GccEnt.Outside(aC), aL);
        Assert.Equal(1, anEnc.NbSolutions());
        Assert.Equal(1, anOut.NbSolutions());
        CheckTangency(anEnc, 1, aC);
        CheckTangency(anOut, 1, aC);
        Assert.Equal(1.0, anEnc.ThisSolution(1).Location().X(), Tol);
        Assert.Equal(-1.0, anOut.ThisSolution(1).Location().X(), Tol);
    }

    [Fact]
    public void CircleCircle_AlwaysTwo_PerpendicularToTheLineOfCentres()
    {
        Circle2D aC1 = Circ(0, 0, 1);
        Circle2D aC2 = Circ(4, 3, 6);
        foreach (GccEnt_QualifiedCirc aQ in new[] { GccEnt.Unqualified(aC1), GccEnt.Enclosing(aC1), GccEnt.Outside(aC1) })
        {
            GccAna_Lin2dTanPer anAlgo = new(aQ, aC2);
            // OCCT: two solutions whatever the qualifier
            Assert.Equal(2, anAlgo.NbSolutions());
            Direction2D aD = new(new Vector2D(aC1.Location(), aC2.Location()));
            for (int i = 1; i <= 2; i++)
            {
                Line2D aSol = anAlgo.ThisSolution(i);
                Assert.Equal(0.0, aSol.Direction().Dot(aD), 1.0e-12);
                CheckTangency(anAlgo, i, aC1);
                anAlgo.Intersection2(i, out double aPs, out double aPa, out Point2D aI);
                Assert.Equal(aC2.Radius(), aI.Distance(aC2.Location()), Tol);
                Assert.True(aSol.Distance(aI) < Tol);
                Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aPa, aC2).Distance(aI) < Tol);
                Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aPs, aSol).Distance(aI) < Tol);
            }
            anAlgo.WhichQualifier(1, out GccEnt_Position aQ1);
            Assert.Equal(aQ.IsOutside() ? GccEnt_outside : GccEnt_enclosing, aQ1);
        }
    }

    [Fact]
    public void BadQualifiers_Throw()
    {
        Assert.Throws<GccEnt_BadQualifier>(
            () => new GccAna_Lin2dTanPer(GccEnt.Enclosed(Circ(0, 0, 1)), new Line2D(new Point2D(0, 0), DX)));
        Assert.Throws<GccEnt_BadQualifier>(() => new GccAna_Lin2dTanPer(GccEnt.Enclosed(Circ(0, 0, 1)), Circ(4, 0, 1)));
    }

    [Fact]
    public void Accessors_OutOfRange()
    {
        GccAna_Lin2dTanPer anAlgo = new(new Point2D(1, 1), new Line2D(new Point2D(0, 0), DX));
        // OCCT throws Standard_RangeError (OutOfRange's base) from ThisSolution
        Assert.ThrowsAny<RangeErrorException>(() => anAlgo.ThisSolution(2));
        Assert.Throws<OutOfRangeException>(() => anAlgo.WhichQualifier(0, out _));
        Assert.Throws<OutOfRangeException>(() => anAlgo.Tangency1(2, out _, out _, out _));
        Assert.Throws<OutOfRangeException>(() => anAlgo.Intersection2(2, out _, out _, out _));
    }
}
