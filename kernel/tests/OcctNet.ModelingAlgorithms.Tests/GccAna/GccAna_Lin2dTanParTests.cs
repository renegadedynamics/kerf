// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GccEnt_Position;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>GccAna_Lin2dTanPar: lines parallel to a line, through a point or tangent to a circle.</summary>
public class GccAna_Lin2dTanParTests
{
    private const double Tol = 1.0e-9;
    private static readonly gp_Dir2d DX = new(gp_Dir2d.D.X);

    private static gp_Circ2d Circ(double x, double y, double r) => new(new gp_Ax2d(new gp_Pnt2d(x, y), DX), r);

    private static double Side(in gp_Lin2d theL, in gp_Pnt2d theP) =>
        new gp_Vec2d(theL.Direction()).Crossed(new gp_Vec2d(theL.Location(), theP));

    [Fact]
    public void ThroughPoint_OneParallel()
    {
        gp_Lin2d aL = new(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 2));
        gp_Pnt2d aP = new(3, -1);
        GccAna_Lin2dTanPar anAlgo = new(aP, aL);
        Assert.True(anAlgo.IsDone());
        Assert.Equal(1, anAlgo.NbSolutions());
        gp_Lin2d aSol = anAlgo.ThisSolution(1);
        Assert.True(aSol.Direction().IsEqual(aL.Direction(), 1.0e-12));
        Assert.True(aSol.Distance(aP) < Tol);
        anAlgo.Tangency1(1, out double aPs, out double aPa, out gp_Pnt2d aT);
        Assert.Equal(aP, aT);
        Assert.Equal(0.0, aPs);
        Assert.Equal(0.0, aPa);
        anAlgo.WhichQualifier(1, out GccEnt_Position aQ);
        Assert.Equal(GccEnt_noqualifier, aQ);
    }

    [Fact]
    public void TangentToCircle_Unqualified_Two()
    {
        gp_Lin2d aL = new(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 1));
        gp_Circ2d aC = Circ(2, -1, 1.5);
        GccAna_Lin2dTanPar anAlgo = new(GccEnt.Unqualified(aC), aL);
        Assert.Equal(2, anAlgo.NbSolutions());
        for (int i = 1; i <= 2; i++)
        {
            gp_Lin2d aSol = anAlgo.ThisSolution(i);
            Assert.True(aSol.Direction().IsEqual(aL.Direction(), 1.0e-12));
            Assert.Equal(aC.Radius(), aSol.Distance(aC.Location()), Tol);
            anAlgo.Tangency1(i, out double aPs, out double aPa, out gp_Pnt2d aT);
            Assert.True(ElCLib.Value(aPs, aSol).Distance(aT) < Tol);
            Assert.True(ElCLib.Value(aPa, aC).Distance(aT) < Tol);
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
        gp_Lin2d aL = new(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0));
        gp_Circ2d aC = Circ(0, 3, 1);
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
        gp_Lin2d aL = new(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0));
        Assert.Throws<GccEnt_BadQualifier>(() => new GccAna_Lin2dTanPar(GccEnt.Enclosed(Circ(0, 3, 1)), aL));
        Assert.Throws<GccEnt_BadQualifier>(
            () => new GccAna_Lin2dTanPar(new GccEnt_QualifiedCirc(Circ(0, 3, 1), GccEnt_noqualifier), aL));
    }

    [Fact]
    public void Accessors_OutOfRange()
    {
        GccAna_Lin2dTanPar anAlgo = new(new gp_Pnt2d(1, 1), new gp_Lin2d(new gp_Pnt2d(0, 0), DX));
        Assert.Throws<Standard_OutOfRange>(() => anAlgo.ThisSolution(2));
        Assert.Throws<Standard_OutOfRange>(() => anAlgo.WhichQualifier(0, out _));
        Assert.Throws<Standard_OutOfRange>(() => anAlgo.Tangency1(2, out _, out _, out _));
    }
}
