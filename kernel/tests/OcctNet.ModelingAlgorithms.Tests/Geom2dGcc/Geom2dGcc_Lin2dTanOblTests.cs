// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GccEnt_Position;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Geom2dGcc_Lin2dTanObl: the analytic dispatch against GccAna_Lin2dTanObl, the iterative solver on an ellipse, OCCT's
/// parallel case, and the accessors' exceptions. OCCT has no GTest for it.
/// </summary>
public class Geom2dGcc_Lin2dTanOblTests
{
    private static readonly gp_Dir2d DX = new(gp_Dir2d.D.X);

    [Fact]
    public void Circle_GivesGccAnasSolutions()
    {
        gp_Circ2d             c1    = new(new gp_Ax2d(new gp_Pnt2d(1, 2), DX), 3);
        gp_Lin2d              aLine = new(new gp_Pnt2d(0, -4), new gp_Dir2d(1, 0.5));
        Geom2dGcc_Lin2dTanObl aLin  = new(Geom2dGcc.Unqualified(new Geom2dAdaptor_Curve(new Geom2d_Circle(c1))), aLine,
                                          1e-9, 0.7);
        GccAna_Lin2dTanObl    anAna = new(GccEnt.Unqualified(c1), aLine, 0.7);
        Assert.True(aLin.IsDone());
        Assert.Equal(2, aLin.NbSolutions());
        for (int i = 1; i <= 2; i++)
        {
            Assert.Equal(anAna.ThisSolution(i), aLin.ThisSolution(i));
            aLin.Tangency1(i, out double p1, out double a1, out gp_Pnt2d t1);
            anAna.Tangency1(i, out double q1, out double b1, out gp_Pnt2d u1);
            Assert.Equal((q1, b1, u1), (p1, a1, t1));
            aLin.Intersection2(i, out double p2, out double a2, out gp_Pnt2d t2);
            anAna.Intersection2(i, out double q2, out double b2, out gp_Pnt2d u2);
            Assert.Equal((q2, b2, u2), (p2, a2, t2));
            aLin.WhichQualifier(i, out GccEnt_Position aQ);
            anAna.WhichQualifier(i, out GccEnt_Position aR);
            Assert.Equal(aR, aQ);
        }
    }

    [Fact]
    public void Ellipse_GivesTangentLinesAtTheAngle()
    {
        Geom2dAdaptor_Curve   anEll = new(new Geom2d_Ellipse(new gp_Ax2d(new gp_Pnt2d(0, 0), DX), 3, 1));
        gp_Lin2d              aLine = new(new gp_Pnt2d(0, -4), DX);
        Geom2dGcc_Lin2dTanObl aLin  = new(Geom2dGcc.Unqualified(anEll), aLine, 1e-6, M_PI / 4);
        Assert.True(aLin.IsDone());
        Assert.Equal(2, aLin.NbSolutions());
        for (int i = 1; i <= 2; i++)
        {
            gp_Lin2d aSol = aLin.ThisSolution(i);
            Assert.Equal(M_PI / 4, Math.Abs(DX.Angle(aSol.Direction())), 1e-9);
            aLin.Tangency1(i, out _, out double a1, out gp_Pnt2d t1);
            Geom2dGcc_Lin2d2TanTests.CheckTangent(aSol, anEll, a1, t1, 1e-6);
            aLin.Intersection2(i, out double p2, out double a2, out gp_Pnt2d t2);
            Assert.True(aLine.Distance(t2) < 1e-9);
            Assert.True(aSol.Distance(t2) < 1e-9);
            Assert.True(ElCLib.Value(a2, aLine).Distance(t2) < 1e-9);
            Assert.True(ElCLib.Value(p2, aSol).Distance(t2) < 1e-9);
        }
    }

    [Fact]
    public void Ellipse_ParallelToTheLine_ThrowsIsParallel()
    {
        // OCCT: with an angle of 0 (or pi) the iterative solver's Intersection2 throws, and Add calls it on the first
        // solution found, so the constructor throws
        Geom2dAdaptor_Curve anEll = new(new Geom2d_Ellipse(new gp_Ax2d(new gp_Pnt2d(0, 0), DX), 3, 1));
        gp_Lin2d            aLine = new(new gp_Pnt2d(0, -4), DX);
        Assert.Throws<Geom2dGcc_IsParallel>(() => new Geom2dGcc_Lin2dTanObl(Geom2dGcc.Unqualified(anEll), aLine, 1e-6, 0.0));
    }

    [Fact]
    public void Accessors_ThrowAsOCCT()
    {
        Geom2dAdaptor_Curve   anEll = new(new Geom2d_Ellipse(new gp_Ax2d(new gp_Pnt2d(0, 0), DX), 3, 1));
        gp_Lin2d              aLine = new(new gp_Pnt2d(0, -4), DX);
        Geom2dGcc_Lin2dTanObl aLin  = new(Geom2dGcc.Unqualified(anEll), aLine, 1e-6, M_PI / 4);
        Assert.Throws<Standard_OutOfRange>(() => aLin.ThisSolution(3));
        Assert.Throws<Standard_OutOfRange>(() => aLin.Tangency1(0, out _, out _, out _));
        Assert.Throws<Standard_OutOfRange>(() => aLin.Intersection2(3, out _, out _, out _));
        Assert.Throws<Standard_OutOfRange>(() => aLin.WhichQualifier(3, out _));
        Assert.Throws<GccEnt_BadQualifier>(
            () => new Geom2dGcc_Lin2dTanObl(new Geom2dGcc_QualifiedCurve(anEll, GccEnt_noqualifier), aLine, 1e-6, M_PI / 4));
    }
}
