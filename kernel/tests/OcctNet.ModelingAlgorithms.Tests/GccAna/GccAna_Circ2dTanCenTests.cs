// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GccEnt_Position;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>GccAna_Circ2dTanCen: circles with a given centre, tangent to a circle or a line, or through a point.</summary>
public class GccAna_Circ2dTanCenTests
{
    private const double Tol = 1.0e-9;
    private static readonly gp_Dir2d DX = new(gp_Dir2d.D.X);

    private static gp_Circ2d Circ(double x, double y, double r) => new(new gp_Ax2d(new gp_Pnt2d(x, y), DX), r);

    private static void CheckCircle(GccAna_Circ2dTanCen theAlgo, int theIndex, in gp_Circ2d theArg, in gp_Pnt2d theCentre)
    {
        gp_Circ2d aSol = theAlgo.ThisSolution(theIndex);
        Assert.Equal(theCentre, aSol.Location());
        double d = theArg.Location().Distance(theCentre);
        theAlgo.WhichQualifier(theIndex, out GccEnt_Position aQ);
        double anExpected = aQ switch
        {
            GccEnt_enclosed => theArg.Radius() - d,
            GccEnt_enclosing => theArg.Radius() + d,
            _ => d - theArg.Radius(),
        };
        Assert.Equal(anExpected, aSol.Radius(), Tol);
        theAlgo.Tangency1(theIndex, out double aPs, out double aPa, out gp_Pnt2d aT);
        Assert.Equal(theArg.Radius(), aT.Distance(theArg.Location()), Tol);
        Assert.Equal(aSol.Radius(), aT.Distance(theCentre), Tol);
        Assert.True(ElCLib.Value(aPs, aSol).Distance(aT) < Tol);
        Assert.True(ElCLib.Value(aPa, theArg).Distance(aT) < Tol);
    }

    [Fact]
    public void Circle_CentreOutside_Unqualified_Two()
    {
        gp_Circ2d aC = Circ(0, 0, 1);
        gp_Pnt2d aP = new(3, 4);
        GccAna_Circ2dTanCen anAlgo = new(GccEnt.Unqualified(aC), aP, 1.0e-9);
        Assert.True(anAlgo.IsDone());
        Assert.Equal(2, anAlgo.NbSolutions());
        CheckCircle(anAlgo, 1, aC, aP);
        CheckCircle(anAlgo, 2, aC, aP);
        Assert.Equal(4.0, anAlgo.ThisSolution(1).Radius(), Tol);
        Assert.Equal(6.0, anAlgo.ThisSolution(2).Radius(), Tol);
        anAlgo.WhichQualifier(1, out GccEnt_Position aQ1);
        anAlgo.WhichQualifier(2, out GccEnt_Position aQ2);
        Assert.Equal(GccEnt_outside, aQ1);
        Assert.Equal(GccEnt_enclosing, aQ2);
        Assert.False(anAlgo.IsTheSame1(1));
    }

    [Fact]
    public void Circle_CentreInside_Unqualified_Two()
    {
        gp_Circ2d aC = Circ(1, 1, 5);
        gp_Pnt2d aP = new(2, 1);
        GccAna_Circ2dTanCen anAlgo = new(GccEnt.Unqualified(aC), aP, 1.0e-9);
        Assert.Equal(2, anAlgo.NbSolutions());
        CheckCircle(anAlgo, 1, aC, aP);
        CheckCircle(anAlgo, 2, aC, aP);
        anAlgo.WhichQualifier(1, out GccEnt_Position aQ1);
        anAlgo.WhichQualifier(2, out GccEnt_Position aQ2);
        Assert.Equal(GccEnt_enclosed, aQ1);
        Assert.Equal(GccEnt_enclosing, aQ2);
    }

    [Theory]
    [InlineData(GccEnt_enclosed, 0.5, 1)]
    [InlineData(GccEnt_enclosed, 3.0, 0)]
    [InlineData(GccEnt_enclosing, 0.5, 1)]
    [InlineData(GccEnt_enclosing, 3.0, 1)]
    [InlineData(GccEnt_outside, 0.5, 0)]
    [InlineData(GccEnt_outside, 3.0, 1)]
    public void Circle_Qualified(GccEnt_Position theQ, double theX, int theNb)
    {
        gp_Circ2d aC = Circ(0, 0, 2);
        gp_Pnt2d aP = new(theX, 0);
        GccAna_Circ2dTanCen anAlgo = new(new GccEnt_QualifiedCirc(aC, theQ), aP, 1.0e-9);
        Assert.True(anAlgo.IsDone());
        Assert.Equal(theNb, anAlgo.NbSolutions());
        for (int i = 1; i <= theNb; i++)
        {
            anAlgo.WhichQualifier(i, out GccEnt_Position aQ);
            Assert.Equal(theQ, aQ);
            CheckCircle(anAlgo, i, aC, aP);
        }
    }

    [Fact]
    public void Circle_Concentric_Unqualified_IsTheSame()
    {
        gp_Circ2d aC = Circ(1, 2, 3);
        GccAna_Circ2dTanCen anAlgo = new(GccEnt.Unqualified(aC), new gp_Pnt2d(1, 2), 1.0e-9);
        Assert.Equal(1, anAlgo.NbSolutions());
        Assert.Equal(aC, anAlgo.ThisSolution(1));
        Assert.True(anAlgo.IsTheSame1(1));
        Assert.Throws<StdFail_NotDone>(() => anAlgo.Tangency1(1, out _, out _, out _));
    }

    [Fact]
    public void Circle_Concentric_Enclosing_IsTheSame()
    {
        gp_Circ2d aC = Circ(1, 2, 3);
        GccAna_Circ2dTanCen anAlgo = new(GccEnt.Enclosing(aC), new gp_Pnt2d(1, 2), 1.0e-9);
        Assert.Equal(1, anAlgo.NbSolutions());
        Assert.Equal(3.0, anAlgo.ThisSolution(1).Radius());
        Assert.True(anAlgo.IsTheSame1(1));
    }

    [Fact]
    public void Circle_CentreOnTheCircle_ZeroRadius_NaNParameter()
    {
        gp_Circ2d aC = Circ(0, 0, 2);
        GccAna_Circ2dTanCen anAlgo = new(GccEnt.Unqualified(aC), new gp_Pnt2d(2, 0), 1.0e-9);
        Assert.Equal(2, anAlgo.NbSolutions());
        Assert.Equal(0.0, anAlgo.ThisSolution(1).Radius());
        // The tangency point is the zero-radius solution's centre: OCCT's release build gives a NaN parameter there
        anAlgo.Tangency1(1, out double aPs, out double aPa, out gp_Pnt2d aT);
        Assert.Equal(new gp_Pnt2d(2, 0), aT);
        Assert.True(double.IsNaN(aPs));
        Assert.Equal(0.0, aPa, Tol);
        CheckCircle(anAlgo, 2, aC, new gp_Pnt2d(2, 0));
    }

    [Fact]
    public void Line_Tangent()
    {
        gp_Lin2d aL = new(new gp_Pnt2d(0, 1), new gp_Dir2d(1, 1));
        foreach (gp_Pnt2d aP in new[] { new gp_Pnt2d(3, 0), new gp_Pnt2d(-2, 5) })
        {
            GccAna_Circ2dTanCen anAlgo = new(aL, aP);
            Assert.True(anAlgo.IsDone());
            Assert.Equal(1, anAlgo.NbSolutions());
            gp_Circ2d aSol = anAlgo.ThisSolution(1);
            Assert.Equal(aL.Distance(aP), aSol.Radius(), Tol);
            anAlgo.Tangency1(1, out double aPs, out double aPa, out gp_Pnt2d aT);
            Assert.True(aL.Distance(aT) < Tol);
            Assert.Equal(aSol.Radius(), aT.Distance(aP), Tol);
            Assert.True(ElCLib.Value(aPs, aSol).Distance(aT) < Tol);
            Assert.True(ElCLib.Value(aPa, aL).Distance(aT) < Tol);
            anAlgo.WhichQualifier(1, out GccEnt_Position aQ);
            Assert.Equal(GccEnt_noqualifier, aQ);
        }
    }

    [Fact]
    public void Line_CentreAtTheLinesLocation_ZeroRadius()
    {
        // OCCT normalises a null vector here; its release build takes the second branch, as the port does
        gp_Lin2d aL = new(new gp_Pnt2d(1, 1), DX);
        GccAna_Circ2dTanCen anAlgo = new(aL, new gp_Pnt2d(1, 1));
        Assert.Equal(1, anAlgo.NbSolutions());
        Assert.Equal(0.0, anAlgo.ThisSolution(1).Radius());
        anAlgo.Tangency1(1, out double aPs, out double aPa, out gp_Pnt2d aT);
        Assert.Equal(new gp_Pnt2d(1, 1), aT);
        Assert.True(double.IsNaN(aPs));
        Assert.Equal(0.0, aPa);
    }

    [Fact]
    public void Point_Through()
    {
        GccAna_Circ2dTanCen anAlgo = new(new gp_Pnt2d(4, 5), new gp_Pnt2d(1, 1));
        Assert.Equal(1, anAlgo.NbSolutions());
        Assert.Equal(5.0, anAlgo.ThisSolution(1).Radius(), Tol);
        anAlgo.Tangency1(1, out double aPs, out double aPa, out gp_Pnt2d aT);
        Assert.Equal(new gp_Pnt2d(4, 5), aT);
        Assert.True(ElCLib.Value(aPs, anAlgo.ThisSolution(1)).Distance(aT) < Tol);
        Assert.Equal(0.0, aPa);
    }

    [Fact]
    public void Errors()
    {
        Assert.Throws<GccEnt_BadQualifier>(
            () => new GccAna_Circ2dTanCen(new GccEnt_QualifiedCirc(Circ(0, 0, 1), GccEnt_noqualifier), new gp_Pnt2d(3, 0), 1.0e-9));
        GccAna_Circ2dTanCen anAlgo = new(new gp_Pnt2d(4, 5), new gp_Pnt2d(1, 1));
        Assert.Throws<Standard_OutOfRange>(() => anAlgo.ThisSolution(2));
        Assert.Throws<Standard_OutOfRange>(() => anAlgo.WhichQualifier(0, out _));
        Assert.Throws<Standard_OutOfRange>(() => anAlgo.Tangency1(2, out _, out _, out _));
        Assert.Throws<Standard_OutOfRange>(() => anAlgo.IsTheSame1(2));
    }
}
