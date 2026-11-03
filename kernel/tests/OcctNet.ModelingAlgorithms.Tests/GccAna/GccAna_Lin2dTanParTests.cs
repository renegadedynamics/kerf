// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.GccEnt_Position;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>GccAna_Lin2dTanPar: lines parallel to a line, through a point or tangent to a circle.</summary>
public class GccAna_Lin2dTanParTests
{
    private const double Tol = 1.0e-9;
    private static readonly Direction2D DX = new(Direction2D.D.X);

    private static Circle2D Circ(double x, double y, double r) => new(new Axis2D(new Point2D(x, y), DX), r);

    private static double Side(in Line2D theL, in Point2D theP) =>
        new Vector2D(theL.Direction()).Crossed(new Vector2D(theL.Location(), theP));

    [Fact]
    public void ThroughPoint_OneParallel()
    {
        Line2D aL = new(new Point2D(0, 0), new Direction2D(1, 2));
        Point2D aP = new(3, -1);
        GccAna_Lin2dTanPar anAlgo = new(aP, aL);
        Assert.True(anAlgo.IsDone());
        Assert.Equal(1, anAlgo.NbSolutions());
        Line2D aSol = anAlgo.ThisSolution(1);
        Assert.True(aSol.Direction().IsEqual(aL.Direction(), 1.0e-12));
        Assert.True(aSol.Distance(aP) < Tol);
        anAlgo.Tangency1(1, out double aPs, out double aPa, out Point2D aT);
        Assert.Equal(aP, aT);
        Assert.Equal(0.0, aPs);
        Assert.Equal(0.0, aPa);
        anAlgo.WhichQualifier(1, out GccEnt_Position aQ);
        Assert.Equal(GccEnt_noqualifier, aQ);
    }

    [Fact]
    public void TangentToCircle_Unqualified_Two()
    {
        Line2D aL = new(new Point2D(0, 0), new Direction2D(1, 1));
        Circle2D aC = Circ(2, -1, 1.5);
        GccAna_Lin2dTanPar anAlgo = new(GccEnt.Unqualified(aC), aL);
        Assert.Equal(2, anAlgo.NbSolutions());
        for (int i = 1; i <= 2; i++)
        {
            Line2D aSol = anAlgo.ThisSolution(i);
            Assert.True(aSol.Direction().IsEqual(aL.Direction(), 1.0e-12));
            Assert.Equal(aC.Radius(), aSol.Distance(aC.Location()), Tol);
            anAlgo.Tangency1(i, out double aPs, out double aPa, out Point2D aT);
            Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aPs, aSol).Distance(aT) < Tol);
            Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aPa, aC).Distance(aT) < Tol);
            anAlgo.WhichQualifier(i, out GccEnt_Position aQ);
            // enclosing: the circle on the left (inside) of the solution
            Assert.Equal(aQ == GccEnt_enclosing, Side(aSol, aC.Location()) > 0.0);
        }
        anAlgo.WhichQualifier(1, out GccEnt_Position aQ1);
        anAlgo.WhichQualifier(2, out GccEnt_Position aQ2);
        Assert.Equal(GccEnt_outside, aQ1);
        Assert.Equal(GccEnt_enclosing, aQ2);
    }

    [Fact]
    public void TangentToCircle_Qualified_One()
    {
        Line2D aL = new(new Point2D(0, 0), new Direction2D(1, 0));
        Circle2D aC = Circ(0, 3, 1);
        GccAna_Lin2dTanPar anEnc = new(GccEnt.Enclosing(aC), aL);
        GccAna_Lin2dTanPar anOut = new(GccEnt.Outside(aC), aL);
        Assert.Equal(1, anEnc.NbSolutions());
        Assert.Equal(1, anOut.NbSolutions());
        Assert.Equal(2.0, anEnc.ThisSolution(1).Location().Y(), Tol);
        Assert.Equal(4.0, anOut.ThisSolution(1).Location().Y(), Tol);
    }

    [Fact]
    public void BadQualifiers_Throw()
    {
        Line2D aL = new(new Point2D(0, 0), new Direction2D(1, 0));
        Assert.Throws<GccEnt_BadQualifier>(() => new GccAna_Lin2dTanPar(GccEnt.Enclosed(Circ(0, 3, 1)), aL));
        Assert.Throws<GccEnt_BadQualifier>(
            () => new GccAna_Lin2dTanPar(new GccEnt_QualifiedCirc(Circ(0, 3, 1), GccEnt_noqualifier), aL));
    }

    [Fact]
    public void Accessors_OutOfRange()
    {
        GccAna_Lin2dTanPar anAlgo = new(new Point2D(1, 1), new Line2D(new Point2D(0, 0), DX));
        Assert.Throws<OutOfRangeException>(() => anAlgo.ThisSolution(2));
        Assert.Throws<OutOfRangeException>(() => anAlgo.WhichQualifier(0, out _));
        Assert.Throws<OutOfRangeException>(() => anAlgo.Tangency1(2, out _, out _, out _));
    }
}
