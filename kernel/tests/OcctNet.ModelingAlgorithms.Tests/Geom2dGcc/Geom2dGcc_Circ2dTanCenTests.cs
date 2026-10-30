// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GccEnt_Position;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Geom2dGcc_Circ2dTanCen: the analytic dispatch against GccAna_Circ2dTanCen, the geometric solver on an ellipse, and
/// the accessors' exceptions. OCCT has no GTest for it.
/// </summary>
public class Geom2dGcc_Circ2dTanCenTests
{
    private static readonly gp_Dir2d DX = new(gp_Dir2d.D.X);

    [Fact]
    public void Circle_GivesGccAnasSolutions()
    {
        gp_Circ2d              c1      = new(new gp_Ax2d(new gp_Pnt2d(1, 2), DX), 3);
        gp_Pnt2d               aCenter = new(2, 2.5);
        Geom2dGcc_Circ2dTanCen aCirc   = new(Geom2dGcc.Unqualified(new Geom2dAdaptor_Curve(new Geom2d_Circle(c1))),
                                             new Geom2d_CartesianPoint(aCenter), 1e-9);
        GccAna_Circ2dTanCen    anAna   = new(GccEnt.Unqualified(c1), aCenter, 1e-9);
        Assert.True(aCirc.IsDone());
        Assert.Equal(anAna.NbSolutions(), aCirc.NbSolutions());
        Assert.Equal(2, aCirc.NbSolutions());
        for (int i = 1; i <= aCirc.NbSolutions(); i++)
        {
            Assert.Equal(anAna.ThisSolution(i), aCirc.ThisSolution(i));
            Assert.Equal(anAna.IsTheSame1(i), aCirc.IsTheSame1(i));
            aCirc.Tangency1(i, out double p1, out double a1, out gp_Pnt2d t1);
            anAna.Tangency1(i, out double q1, out double b1, out gp_Pnt2d u1);
            Assert.Equal((q1, b1, u1), (p1, a1, t1));
            aCirc.WhichQualifier(i, out GccEnt_Position aQ);
            anAna.WhichQualifier(i, out GccEnt_Position aR);
            Assert.Equal(aR, aQ);
        }
    }

    [Fact]
    public void Line_GivesGccAnasSolution()
    {
        gp_Lin2d               l1      = new(new gp_Pnt2d(0, 1), new gp_Dir2d(1, 1));
        gp_Pnt2d               aCenter = new(3, -1);
        Geom2dGcc_Circ2dTanCen aCirc   = new(Geom2dGcc.Unqualified(new Geom2dAdaptor_Curve(new Geom2d_Line(l1))),
                                             new Geom2d_CartesianPoint(aCenter), 1e-9);
        GccAna_Circ2dTanCen    anAna   = new(l1, aCenter);
        Assert.Equal(1, aCirc.NbSolutions());
        Assert.Equal(anAna.ThisSolution(1), aCirc.ThisSolution(1));
    }

    [Fact]
    public void Ellipse_GivesTheNearestTangentCircle()
    {
        // From the ellipse's centre, the nearest extrema are at the minor radius. OCCT compares two uninitialised
        // indices to decide whether to add the farthest one; the port's are 0, so it keeps the nearest only.
        Geom2dAdaptor_Curve    anEll = new(new Geom2d_Ellipse(new gp_Ax2d(new gp_Pnt2d(1, 1), DX), 3, 1));
        Geom2dGcc_Circ2dTanCen aCirc = new(Geom2dGcc.Unqualified(anEll), new Geom2d_CartesianPoint(1, 1), 1e-9);
        Assert.True(aCirc.IsDone());
        Assert.Equal(1, aCirc.NbSolutions());
        Assert.Equal(1.0, aCirc.ThisSolution(1).Radius(), 1e-9);
        Assert.False(aCirc.IsTheSame1(1));
        aCirc.Tangency1(1, out double aParSol, out double aParArg, out gp_Pnt2d aPnt);
        Assert.Equal(0.0, aParSol);
        Assert.True(anEll.Value(aParArg).Distance(aPnt) < 1e-9);
        Assert.Equal(1.0, aPnt.Distance(new gp_Pnt2d(1, 1)), 1e-9);
        aCirc.WhichQualifier(1, out GccEnt_Position aQ);
        Assert.Equal(GccEnt_unqualified, aQ);
    }

    [Fact]
    public void Accessors_ThrowAsOCCT()
    {
        Geom2dAdaptor_Curve    anEll = new(new Geom2d_Ellipse(new gp_Ax2d(new gp_Pnt2d(1, 1), DX), 3, 1));
        Geom2dGcc_Circ2dTanCen aCirc = new(Geom2dGcc.Unqualified(anEll), new Geom2d_CartesianPoint(1, 1), 1e-9);
        Assert.Throws<Standard_OutOfRange>(() => aCirc.ThisSolution(0));
        Assert.Throws<Standard_OutOfRange>(() => aCirc.ThisSolution(2));
        Assert.Throws<Standard_OutOfRange>(() => aCirc.Tangency1(2, out _, out _, out _));
        Assert.Throws<Standard_OutOfRange>(() => aCirc.WhichQualifier(-1, out _));
        Assert.Throws<Standard_OutOfRange>(() => aCirc.IsTheSame1(2));
    }
}
