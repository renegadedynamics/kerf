// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GccEnt_Position;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// GccAna_Lin2d2Tan on OCCT's documented configurations: solution counts, qualifiers, and on every solution the
/// tangencies (or pass-through) and the tangency points with their parameters.
/// </summary>
public class GccAna_Lin2d2TanTests
{
    private const double Tol = 1.0e-7;
    private static readonly gp_Dir2d DX = new(gp_Dir2d.D.X);

    private static gp_Circ2d Circ(double x, double y, double r) => new(new gp_Ax2d(new gp_Pnt2d(x, y), DX), r);

    private static void AssertNear(in gp_Pnt2d theP, in gp_Pnt2d theQ) =>
        Assert.True(theP.Distance(theQ) < Tol, $"{theP} != {theQ}");

    // The side of the line the point is on: the left (inside) is positive
    private static double Side(in gp_Lin2d theL, in gp_Pnt2d theP) =>
        new gp_Vec2d(theL.Direction()).Crossed(new gp_Vec2d(theL.Location(), theP));

    private static void CheckTangentToCircle(in gp_Lin2d theSol, in gp_Circ2d theC, double theParSol, double theParArg,
                                             in gp_Pnt2d thePnt)
    {
        Assert.Equal(theC.Radius(), theSol.Distance(theC.Location()), Tol);
        Assert.True(theSol.Distance(thePnt) < Tol);
        Assert.Equal(theC.Radius(), thePnt.Distance(theC.Location()), Tol);
        AssertNear(ElCLib.Value(theParSol, theSol), thePnt);
        AssertNear(ElCLib.Value(theParArg, theC), thePnt);
    }

    private static void CheckTwoCircles(GccAna_Lin2d2Tan theAlgo, in gp_Circ2d theC1, in gp_Circ2d theC2)
    {
        for (int i = 1; i <= theAlgo.NbSolutions(); i++)
        {
            gp_Lin2d aSol = theAlgo.ThisSolution(i);
            theAlgo.Tangency1(i, out double aPs1, out double aPa1, out gp_Pnt2d aP1);
            theAlgo.Tangency2(i, out double aPs2, out double aPa2, out gp_Pnt2d aP2);
            CheckTangentToCircle(aSol, theC1, aPs1, aPa1, aP1);
            CheckTangentToCircle(aSol, theC2, aPs2, aPa2, aP2);
        }
    }

    [Fact]
    public void TwoPoints_OneLine()
    {
        gp_Pnt2d aP1 = new(1, 1), aP2 = new(4, 5);
        GccAna_Lin2d2Tan anAlgo = new(aP1, aP2, 1.0e-6);
        Assert.True(anAlgo.IsDone());
        Assert.Equal(1, anAlgo.NbSolutions());
        gp_Lin2d aSol = anAlgo.ThisSolution(1);
        Assert.True(aSol.Distance(aP1) < Tol && aSol.Distance(aP2) < Tol);
        anAlgo.Tangency1(1, out double aPs1, out double aPa1, out gp_Pnt2d aT1);
        anAlgo.Tangency2(1, out double aPs2, out double aPa2, out gp_Pnt2d aT2);
        AssertNear(aT1, aP1);
        AssertNear(aT2, aP2);
        Assert.Equal(0.0, aPs1, Tol);
        Assert.Equal(5.0, aPs2, Tol);
        Assert.Equal(0.0, aPa1);
        Assert.Equal(0.0, aPa2);
        anAlgo.WhichQualifier(1, out GccEnt_Position aQ1, out GccEnt_Position aQ2);
        Assert.Equal(GccEnt_noqualifier, aQ1);
        Assert.Equal(GccEnt_noqualifier, aQ2);
    }

    [Fact]
    public void TwoPoints_Coincident_NotDone()
    {
        GccAna_Lin2d2Tan anAlgo = new(new gp_Pnt2d(1, 1), new gp_Pnt2d(1, 1 + 1.0e-9), 1.0e-6);
        Assert.False(anAlgo.IsDone());
        Assert.Equal(0, anAlgo.NbSolutions());
        Assert.Throws<StdFail_NotDone>(() => anAlgo.Tangency1(1, out _, out _, out _));
    }

    [Fact]
    public void CirclePoint_Unqualified_TwoTangents()
    {
        gp_Circ2d aC = Circ(0, 0, 1);
        gp_Pnt2d aP = new(3, 1);
        GccAna_Lin2d2Tan anAlgo = new(GccEnt.Unqualified(aC), aP, 1.0e-6);
        Assert.True(anAlgo.IsDone());
        Assert.Equal(2, anAlgo.NbSolutions());
        for (int i = 1; i <= 2; i++)
        {
            gp_Lin2d aSol = anAlgo.ThisSolution(i);
            anAlgo.Tangency1(i, out double aPs, out double aPa, out gp_Pnt2d aT);
            CheckTangentToCircle(aSol, aC, aPs, aPa, aT);
            anAlgo.Tangency2(i, out double aPs2, out double aPa2, out gp_Pnt2d aT2);
            AssertNear(aT2, aP);
            AssertNear(ElCLib.Value(aPs2, aSol), aP);
            Assert.Equal(0.0, aPa2);
            // OCCT returns the qualifier given, not a computed one
            anAlgo.WhichQualifier(i, out GccEnt_Position aQ1, out GccEnt_Position aQ2);
            Assert.Equal(GccEnt_unqualified, aQ1);
            Assert.Equal(GccEnt_noqualifier, aQ2);
        }
        Assert.False(anAlgo.ThisSolution(1).Direction().IsParallel(anAlgo.ThisSolution(2).Direction(), 1.0e-6));
    }

    [Fact]
    public void CirclePoint_Qualified_OneTangentEach()
    {
        gp_Circ2d aC = Circ(0, 0, 1);
        gp_Pnt2d aP = new(3, 1);
        GccAna_Lin2d2Tan anEnc = new(GccEnt.Enclosing(aC), aP, 1.0e-6);
        GccAna_Lin2d2Tan anOut = new(GccEnt.Outside(aC), aP, 1.0e-6);
        Assert.Equal(1, anEnc.NbSolutions());
        Assert.Equal(1, anOut.NbSolutions());
        // The circle is on the left (inside) of an enclosing solution and on the right of an outside one
        Assert.True(Side(anEnc.ThisSolution(1), aC.Location()) > 0.0);
        Assert.True(Side(anOut.ThisSolution(1), aC.Location()) < 0.0);
        anEnc.WhichQualifier(1, out GccEnt_Position aQ, out _);
        Assert.Equal(GccEnt_enclosing, aQ);
        anOut.WhichQualifier(1, out aQ, out _);
        Assert.Equal(GccEnt_outside, aQ);
        for (int k = 0; k < 2; k++)
        {
            GccAna_Lin2d2Tan anAlgo = k == 0 ? anEnc : anOut;
            anAlgo.Tangency1(1, out double aPs, out double aPa, out gp_Pnt2d aT);
            CheckTangentToCircle(anAlgo.ThisSolution(1), aC, aPs, aPa, aT);
            Assert.True(anAlgo.ThisSolution(1).Distance(aP) < Tol);
        }
    }

    [Fact]
    public void CirclePoint_PointInside_NoSolution()
    {
        GccAna_Lin2d2Tan anAlgo = new(GccEnt.Unqualified(Circ(0, 0, 2)), new gp_Pnt2d(0.5, 0.5), 1.0e-6);
        Assert.True(anAlgo.IsDone());
        Assert.Equal(0, anAlgo.NbSolutions());
    }

    [Fact]
    public void CirclePoint_PointOnCircle_OneTangent()
    {
        gp_Circ2d aC = Circ(1, 1, 2);
        gp_Pnt2d aP = new(3, 1);
        GccAna_Lin2d2Tan anAlgo = new(GccEnt.Unqualified(aC), aP, 1.0e-6);
        Assert.Equal(1, anAlgo.NbSolutions());
        anAlgo.Tangency1(1, out double aPs, out double aPa, out gp_Pnt2d aT);
        CheckTangentToCircle(anAlgo.ThisSolution(1), aC, aPs, aPa, aT);
        AssertNear(aT, aP);
    }

    [Fact]
    public void BadQualifiers_Throw()
    {
        gp_Circ2d aC = Circ(0, 0, 1);
        Assert.Throws<GccEnt_BadQualifier>(() => new GccAna_Lin2d2Tan(GccEnt.Enclosed(aC), new gp_Pnt2d(3, 0), 1.0e-6));
        Assert.Throws<GccEnt_BadQualifier>(
            () => new GccAna_Lin2d2Tan(new GccEnt_QualifiedCirc(aC, GccEnt_noqualifier), new gp_Pnt2d(3, 0), 1.0e-6));
        Assert.Throws<GccEnt_BadQualifier>(
            () => new GccAna_Lin2d2Tan(GccEnt.Unqualified(aC), GccEnt.Enclosed(Circ(5, 0, 1)), 1.0e-6));
    }

    [Fact]
    public void TwoExternalCircles_Unqualified_FourTangents()
    {
        gp_Circ2d aC1 = Circ(0, 0, 1), aC2 = Circ(5, 1, 1.5);
        GccAna_Lin2d2Tan anAlgo = new(GccEnt.Unqualified(aC1), GccEnt.Unqualified(aC2), 1.0e-6);
        Assert.True(anAlgo.IsDone());
        Assert.Equal(4, anAlgo.NbSolutions());
        CheckTwoCircles(anAlgo, aC1, aC2);
        // Two external tangents (both centres on one side) and two internal ones
        int anInternal = 0;
        for (int i = 1; i <= 4; i++)
        {
            gp_Lin2d aSol = anAlgo.ThisSolution(i);
            if (Side(aSol, aC1.Location()) * Side(aSol, aC2.Location()) < 0.0)
            {
                anInternal++;
            }
        }
        Assert.Equal(2, anInternal);
    }

    [Theory]
    [InlineData(GccEnt_enclosing, GccEnt_enclosing)]
    [InlineData(GccEnt_enclosing, GccEnt_outside)]
    [InlineData(GccEnt_outside, GccEnt_enclosing)]
    [InlineData(GccEnt_outside, GccEnt_outside)]
    public void TwoExternalCircles_Qualified_OneTangent(GccEnt_Position theQ1, GccEnt_Position theQ2)
    {
        gp_Circ2d aC1 = Circ(0, 0, 1), aC2 = Circ(5, 1, 1.5);
        GccAna_Lin2d2Tan anAlgo = new(new GccEnt_QualifiedCirc(aC1, theQ1), new GccEnt_QualifiedCirc(aC2, theQ2), 1.0e-6);
        Assert.True(anAlgo.IsDone());
        Assert.Equal(1, anAlgo.NbSolutions());
        CheckTwoCircles(anAlgo, aC1, aC2);
        gp_Lin2d aSol = anAlgo.ThisSolution(1);
        Assert.Equal(theQ1 == GccEnt_enclosing, Side(aSol, aC1.Location()) > 0.0);
        Assert.Equal(theQ2 == GccEnt_enclosing, Side(aSol, aC2.Location()) > 0.0);
        // OCCT: the second qualifier is the first argument's
        anAlgo.WhichQualifier(1, out GccEnt_Position aQ1, out GccEnt_Position aQ2);
        Assert.Equal(theQ1, aQ1);
        Assert.Equal(theQ1, aQ2);
    }

    [Fact]
    public void SecantCircles_Unqualified_TwoTangents()
    {
        gp_Circ2d aC1 = Circ(0, 0, 2), aC2 = Circ(3, 0, 2);
        GccAna_Lin2d2Tan anAlgo = new(GccEnt.Unqualified(aC1), GccEnt.Unqualified(aC2), 1.0e-6);
        Assert.Equal(2, anAlgo.NbSolutions());
        CheckTwoCircles(anAlgo, aC1, aC2);
    }

    [Fact]
    public void NestedCircles_NoSolution()
    {
        GccAna_Lin2d2Tan anAlgo = new(GccEnt.Unqualified(Circ(0, 0, 5)), GccEnt.Unqualified(Circ(1, 0, 1)), 1.0e-6);
        Assert.True(anAlgo.IsDone());
        Assert.Equal(0, anAlgo.NbSolutions());
    }

    [Fact]
    public void InternallyTangentCircles_Enclosing_OneTangent()
    {
        gp_Circ2d aC1 = Circ(0, 0, 3), aC2 = Circ(1, 0, 2);
        GccAna_Lin2d2Tan anAlgo = new(GccEnt.Enclosing(aC1), GccEnt.Enclosing(aC2), 1.0e-6);
        Assert.Equal(1, anAlgo.NbSolutions());
        CheckTwoCircles(anAlgo, aC1, aC2);
        anAlgo.Tangency1(1, out _, out _, out gp_Pnt2d aT1);
        AssertNear(aT1, new gp_Pnt2d(3, 0));
    }

    [Fact]
    public void Mixed_EnclosingUnqualified_TwoTangents_SecondTangencyPointIsTheFirsts()
    {
        gp_Circ2d aC1 = Circ(0, 0, 1), aC2 = Circ(5, 1, 1.5);
        GccAna_Lin2d2Tan anAlgo = new(GccEnt.Enclosing(aC1), GccEnt.Unqualified(aC2), 1.0e-6);
        Assert.Equal(2, anAlgo.NbSolutions());
        for (int i = 1; i <= 2; i++)
        {
            gp_Lin2d aSol = anAlgo.ThisSolution(i);
            Assert.Equal(aC1.Radius(), aSol.Distance(aC1.Location()), Tol);
            Assert.Equal(aC2.Radius(), aSol.Distance(aC2.Location()), Tol);
            Assert.True(Side(aSol, aC1.Location()) > 0.0);
            anAlgo.Tangency2(i, out double aPs2, out double aPa2, out gp_Pnt2d aT2);
            CheckTangentToCircle(aSol, aC2, aPs2, aPa2, aT2);
        }
        // OCCT: the second solution's first tangency point is the first solution's
        anAlgo.Tangency1(1, out _, out _, out gp_Pnt2d aT11);
        anAlgo.Tangency1(2, out _, out _, out gp_Pnt2d aT21);
        Assert.Equal(aT11, aT21);
        Assert.False(anAlgo.ThisSolution(2).Distance(aT21) < Tol);
    }

    [Fact]
    public void Accessors_OutOfRange()
    {
        GccAna_Lin2d2Tan anAlgo = new(new gp_Pnt2d(0, 0), new gp_Pnt2d(1, 0), 1.0e-6);
        Assert.Throws<Standard_OutOfRange>(() => anAlgo.ThisSolution(2));
        Assert.Throws<Standard_OutOfRange>(() => anAlgo.ThisSolution(0));
        Assert.Throws<Standard_OutOfRange>(() => anAlgo.WhichQualifier(2, out _, out _));
        Assert.Throws<Standard_OutOfRange>(() => anAlgo.Tangency1(2, out _, out _, out _));
        Assert.Throws<Standard_OutOfRange>(() => anAlgo.Tangency2(2, out _, out _, out _));
    }
}
