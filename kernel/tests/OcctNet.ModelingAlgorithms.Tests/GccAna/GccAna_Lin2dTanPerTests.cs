// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GccEnt_Position;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// GccAna_Lin2dTanPer: lines through a point or tangent to a circle, perpendicular to a line or a circle.
/// </summary>
public class GccAna_Lin2dTanPerTests
{
    private const double Tol = 1.0e-9;
    private static readonly gp_Dir2d DX = new(gp_Dir2d.D.X);

    private static gp_Circ2d Circ(double x, double y, double r) => new(new gp_Ax2d(new gp_Pnt2d(x, y), DX), r);

    private static void CheckTangency(GccAna_Lin2dTanPer theAlgo, int theIndex, in gp_Circ2d theC)
    {
        gp_Lin2d aSol = theAlgo.ThisSolution(theIndex);
        Assert.Equal(theC.Radius(), aSol.Distance(theC.Location()), Tol);
        theAlgo.Tangency1(theIndex, out double aPs, out double aPa, out gp_Pnt2d aT);
        Assert.True(ElCLib.Value(aPs, aSol).Distance(aT) < Tol);
        Assert.True(ElCLib.Value(aPa, theC).Distance(aT) < Tol);
    }

    [Fact]
    public void PointLine_Perpendicular()
    {
        gp_Lin2d aL = new(new gp_Pnt2d(0, 0), new gp_Dir2d(2, 1));
        gp_Pnt2d aP = new(1, 4);
        GccAna_Lin2dTanPer anAlgo = new(aP, aL);
        Assert.True(anAlgo.IsDone());
        Assert.Equal(1, anAlgo.NbSolutions());
        gp_Lin2d aSol = anAlgo.ThisSolution(1);
        Assert.Equal(0.0, aSol.Direction().Dot(aL.Direction()), 1.0e-12);
        Assert.True(aSol.Distance(aP) < Tol);
        anAlgo.Intersection2(1, out double aPs, out double aPa, out gp_Pnt2d aI);
        Assert.True(aL.Distance(aI) < Tol && aSol.Distance(aI) < Tol);
        Assert.True(ElCLib.Value(aPs, aSol).Distance(aI) < Tol);
        Assert.True(ElCLib.Value(aPa, aL).Distance(aI) < Tol);
        anAlgo.Tangency1(1, out double aPs1, out double aPa1, out gp_Pnt2d aT);
        Assert.Equal(aP, aT);
        Assert.Equal(0.0, aPs1, Tol);
        Assert.Equal(0.0, aPa1);
    }

    [Fact]
    public void PointCircle_ThroughTheCentre_LastIntersection()
    {
        gp_Circ2d aC = Circ(3, 0, 1);
        gp_Pnt2d aP = new(0, 0);
        GccAna_Lin2dTanPer anAlgo = new(aP, aC);
        Assert.Equal(1, anAlgo.NbSolutions());
        gp_Lin2d aSol = anAlgo.ThisSolution(1);
        Assert.True(aSol.Distance(aC.Location()) < Tol);
        anAlgo.Intersection2(1, out double aPs, out double aPa, out gp_Pnt2d aI);
        Assert.Equal(aC.Radius(), aI.Distance(aC.Location()), Tol);
        Assert.True(ElCLib.Value(aPs, aSol).Distance(aI) < Tol);
        Assert.True(ElCLib.Value(aPa, aC).Distance(aI) < Tol);
        // OCCT keeps the last intersection point IntAna2d returns, whatever its distance
        IntAna2d_AnaIntersection anInt = new(aSol, aC);
        Assert.Equal(anInt.Point(anInt.NbPoints()).Value(), aI);
    }

    [Fact]
    public void CircleLine_Unqualified_Two()
    {
        gp_Lin2d aL = new(new gp_Pnt2d(0, -3), new gp_Dir2d(1, 0.5));
        gp_Circ2d aC = Circ(1, 1, 2);
        GccAna_Lin2dTanPer anAlgo = new(GccEnt.Unqualified(aC), aL);
        Assert.Equal(2, anAlgo.NbSolutions());
        for (int i = 1; i <= 2; i++)
        {
            Assert.Equal(0.0, anAlgo.ThisSolution(i).Direction().Dot(aL.Direction()), 1.0e-12);
            CheckTangency(anAlgo, i, aC);
            anAlgo.Intersection2(i, out _, out _, out gp_Pnt2d aI);
            Assert.True(aL.Distance(aI) < Tol && anAlgo.ThisSolution(i).Distance(aI) < Tol);
            // OCCT: this constructor never sets the qualifier
            anAlgo.WhichQualifier(i, out GccEnt_Position aQ);
            Assert.Equal(GccEnt_unqualified, aQ);
        }
    }

    [Fact]
    public void CircleLine_Qualified_One()
    {
        gp_Lin2d aL = new(new gp_Pnt2d(0, -3), DX);
        gp_Circ2d aC = Circ(0, 0, 1);
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
        gp_Circ2d aC1 = Circ(0, 0, 1);
        gp_Circ2d aC2 = Circ(4, 3, 6);
        foreach (GccEnt_QualifiedCirc aQ in new[] { GccEnt.Unqualified(aC1), GccEnt.Enclosing(aC1), GccEnt.Outside(aC1) })
        {
            GccAna_Lin2dTanPer anAlgo = new(aQ, aC2);
            // OCCT: two solutions whatever the qualifier
            Assert.Equal(2, anAlgo.NbSolutions());
            gp_Dir2d aD = new(new gp_Vec2d(aC1.Location(), aC2.Location()));
            for (int i = 1; i <= 2; i++)
            {
                gp_Lin2d aSol = anAlgo.ThisSolution(i);
                Assert.Equal(0.0, aSol.Direction().Dot(aD), 1.0e-12);
                CheckTangency(anAlgo, i, aC1);
                anAlgo.Intersection2(i, out double aPs, out double aPa, out gp_Pnt2d aI);
                Assert.Equal(aC2.Radius(), aI.Distance(aC2.Location()), Tol);
                Assert.True(aSol.Distance(aI) < Tol);
                Assert.True(ElCLib.Value(aPa, aC2).Distance(aI) < Tol);
                Assert.True(ElCLib.Value(aPs, aSol).Distance(aI) < Tol);
            }
            anAlgo.WhichQualifier(1, out GccEnt_Position aQ1);
            Assert.Equal(aQ.IsOutside() ? GccEnt_outside : GccEnt_enclosing, aQ1);
        }
    }

    [Fact]
    public void BadQualifiers_Throw()
    {
        Assert.Throws<GccEnt_BadQualifier>(
            () => new GccAna_Lin2dTanPer(GccEnt.Enclosed(Circ(0, 0, 1)), new gp_Lin2d(new gp_Pnt2d(0, 0), DX)));
        Assert.Throws<GccEnt_BadQualifier>(() => new GccAna_Lin2dTanPer(GccEnt.Enclosed(Circ(0, 0, 1)), Circ(4, 0, 1)));
    }

    [Fact]
    public void Accessors_OutOfRange()
    {
        GccAna_Lin2dTanPer anAlgo = new(new gp_Pnt2d(1, 1), new gp_Lin2d(new gp_Pnt2d(0, 0), DX));
        // OCCT throws Standard_RangeError (OutOfRange's base) from ThisSolution
        Assert.ThrowsAny<Standard_RangeError>(() => anAlgo.ThisSolution(2));
        Assert.Throws<Standard_OutOfRange>(() => anAlgo.WhichQualifier(0, out _));
        Assert.Throws<Standard_OutOfRange>(() => anAlgo.Tangency1(2, out _, out _, out _));
        Assert.Throws<Standard_OutOfRange>(() => anAlgo.Intersection2(2, out _, out _, out _));
    }
}
