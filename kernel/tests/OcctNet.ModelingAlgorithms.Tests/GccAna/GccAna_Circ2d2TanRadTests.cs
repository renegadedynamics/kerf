// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GccEnt_Position;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// GccAna_Circ2d2TanRad on OCCT's documented configurations: solution counts, qualifiers, and on every solution the
/// radius, the tangencies and the tangency points.
/// </summary>
public class GccAna_Circ2d2TanRadTests
{
    private const double Tol = 1.0e-9;
    private static readonly gp_Dir2d DX = new(gp_Dir2d.D.X);

    private static gp_Circ2d Circ(double x, double y, double r) => new(new gp_Ax2d(new gp_Pnt2d(x, y), DX), r);

    private static gp_Lin2d Lin(double x, double y, double dx, double dy) => new(new gp_Pnt2d(x, y), new gp_Dir2d(dx, dy));

    // The qualifier a circle argument has for a solution, from the geometry
    private static GccEnt_Position CircQualifier(in gp_Circ2d theArg, in gp_Circ2d theSol)
    {
        double d = theArg.Location().Distance(theSol.Location());
        if (Math.Abs(d + theSol.Radius() - theArg.Radius()) < 1.0e-7)
        {
            return GccEnt_enclosed;
        }
        if (Math.Abs(d - theSol.Radius() - theArg.Radius()) < 1.0e-7)
        {
            return GccEnt_outside;
        }
        Assert.Equal(theSol.Radius(), d + theArg.Radius(), 1.0e-7);
        return GccEnt_enclosing;
    }

    // Enclosed: the centre on the left of the line; outside: on the right
    private static GccEnt_Position LinQualifier(in gp_Lin2d theArg, in gp_Circ2d theSol)
    {
        gp_Vec2d aV = new(theArg.Location(), theSol.Location());
        return new gp_Vec2d(theArg.Direction()).Crossed(aV) > 0.0 ? GccEnt_enclosed : GccEnt_outside;
    }

    private static void AssertTangentCirc(in gp_Circ2d theArg, in gp_Circ2d theSol)
    {
        double d = theArg.Location().Distance(theSol.Location());
        double aGap = Math.Min(Math.Abs(d - (theArg.Radius() + theSol.Radius())), Math.Abs(d - Math.Abs(theArg.Radius() - theSol.Radius())));
        Assert.True(aGap < 1.0e-7, $"not tangent: {aGap}");
    }

    private static void AssertNear(in gp_Pnt2d theP, in gp_Pnt2d theQ) => Assert.True(theP.Distance(theQ) < 1.0e-7, $"{theP} != {theQ}");

    private static void CheckTangency1(GccAna_Circ2d2TanRad theAlgo, int theIndex, in gp_Circ2d theArg)
    {
        gp_Circ2d aSol = theAlgo.ThisSolution(theIndex);
        AssertTangentCirc(theArg, aSol);
        if (theAlgo.IsTheSame1(theIndex))
        {
            return;
        }
        theAlgo.Tangency1(theIndex, out double aParSol, out double aParArg, out gp_Pnt2d aPnt);
        AssertNear(ElCLib.Value(aParSol, aSol), aPnt);
        AssertNear(ElCLib.Value(aParArg, theArg), aPnt);
    }

    private static void CheckTangency2(GccAna_Circ2d2TanRad theAlgo, int theIndex, in gp_Circ2d theArg)
    {
        gp_Circ2d aSol = theAlgo.ThisSolution(theIndex);
        AssertTangentCirc(theArg, aSol);
        if (theAlgo.IsTheSame2(theIndex))
        {
            return;
        }
        theAlgo.Tangency2(theIndex, out double aParSol, out double aParArg, out gp_Pnt2d aPnt);
        AssertNear(ElCLib.Value(aParSol, aSol), aPnt);
        AssertNear(ElCLib.Value(aParArg, theArg), aPnt);
    }

    private static void CheckTangencyLin(GccAna_Circ2d2TanRad theAlgo, int theIndex, in gp_Lin2d theArg, bool theFirst)
    {
        gp_Circ2d aSol = theAlgo.ThisSolution(theIndex);
        Assert.Equal(aSol.Radius(), theArg.Distance(aSol.Location()), 1.0e-7);
        double aParSol, aParArg;
        gp_Pnt2d aPnt;
        if (theFirst)
        {
            theAlgo.Tangency1(theIndex, out aParSol, out aParArg, out aPnt);
        }
        else
        {
            theAlgo.Tangency2(theIndex, out aParSol, out aParArg, out aPnt);
        }
        AssertNear(ElCLib.Value(aParSol, aSol), aPnt);
        AssertNear(ElCLib.Value(aParArg, theArg), aPnt);
    }

    private static void CheckThrough(GccAna_Circ2d2TanRad theAlgo, int theIndex, in gp_Pnt2d thePnt, bool theFirst)
    {
        gp_Circ2d aSol = theAlgo.ThisSolution(theIndex);
        Assert.Equal(aSol.Radius(), aSol.Location().Distance(thePnt), 1.0e-7);
        double aParSol, aParArg;
        gp_Pnt2d aPnt;
        if (theFirst)
        {
            theAlgo.Tangency1(theIndex, out aParSol, out aParArg, out aPnt);
        }
        else
        {
            theAlgo.Tangency2(theIndex, out aParSol, out aParArg, out aPnt);
        }
        AssertNear(aPnt, thePnt);
        AssertNear(ElCLib.Value(aParSol, aSol), aPnt);
    }

    [Theory]
    [InlineData(2.0, 2)]
    [InlineData(1.0, 1)]
    [InlineData(0.5, 0)]
    public void TwoPoints_Counts(double theRadius, int theNb)
    {
        gp_Pnt2d aP1 = new(0, 0), aP2 = new(2, 0);
        GccAna_Circ2d2TanRad anAlgo = new(aP1, aP2, theRadius, Tol);
        Assert.True(anAlgo.IsDone());
        Assert.Equal(theNb, anAlgo.NbSolutions());
        for (int i = 1; i <= theNb; i++)
        {
            Assert.Equal(theRadius, anAlgo.ThisSolution(i).Radius());
            CheckThrough(anAlgo, i, aP1, true);
            CheckThrough(anAlgo, i, aP2, false);
            anAlgo.WhichQualifier(i, out GccEnt_Position aQ1, out GccEnt_Position aQ2);
            Assert.Equal(GccEnt_noqualifier, aQ1);
            Assert.Equal(GccEnt_noqualifier, aQ2);
        }
    }

    [Fact]
    public void TwoSecantLines_FourCirclesInTheQuadrants()
    {
        gp_Lin2d aL1 = Lin(0, 0, 1, 0), aL2 = Lin(0, 0, 0, 1);
        GccAna_Circ2d2TanRad anAlgo = new(GccEnt.Unqualified(aL1), GccEnt.Unqualified(aL2), 1.0, Tol);
        Assert.True(anAlgo.IsDone());
        Assert.Equal(4, anAlgo.NbSolutions());
        HashSet<(double, double)> aCentres = [];
        for (int i = 1; i <= 4; i++)
        {
            gp_Circ2d aSol = anAlgo.ThisSolution(i);
            Assert.Equal(1.0, aSol.Radius());
            CheckTangencyLin(anAlgo, i, aL1, true);
            CheckTangencyLin(anAlgo, i, aL2, false);
            aCentres.Add((Math.Round(aSol.Location().X(), 9), Math.Round(aSol.Location().Y(), 9)));
        }
        Assert.Equal([(1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0)], aCentres);
    }

    [Theory]
    [InlineData(GccEnt_enclosed, GccEnt_enclosed, -1.0, 1.0)]
    [InlineData(GccEnt_enclosed, GccEnt_outside, 1.0, 1.0)]
    [InlineData(GccEnt_outside, GccEnt_enclosed, -1.0, -1.0)]
    [InlineData(GccEnt_outside, GccEnt_outside, 1.0, -1.0)]
    public void TwoSecantLines_Qualified(GccEnt_Position theQ1, GccEnt_Position theQ2, double theX, double theY)
    {
        // The inside of the X axis (along +X) is y > 0, the inside of the Y axis (along +Y) x < 0
        gp_Lin2d aL1 = Lin(0, 0, 1, 0), aL2 = Lin(0, 0, 0, 1);
        GccAna_Circ2d2TanRad anAlgo = new(new GccEnt_QualifiedLin(aL1, theQ1), new GccEnt_QualifiedLin(aL2, theQ2), 1.0, Tol);
        Assert.True(anAlgo.IsDone());
        Assert.Equal(1, anAlgo.NbSolutions());
        AssertNear(anAlgo.ThisSolution(1).Location(), new gp_Pnt2d(theX, theY));
        CheckTangencyLin(anAlgo, 1, aL1, true);
        CheckTangencyLin(anAlgo, 1, aL2, false);
        anAlgo.WhichQualifier(1, out GccEnt_Position aQ1, out GccEnt_Position aQ2);
        Assert.Equal(theQ1, aQ1);
        Assert.Equal(theQ2, aQ2);
        Assert.Equal(theQ1, LinQualifier(aL1, anAlgo.ThisSolution(1)));
        Assert.Equal(theQ2, LinQualifier(aL2, anAlgo.ThisSolution(1)));
    }

    [Fact]
    public void ParallelLines_NoSolution()
    {
        GccAna_Circ2d2TanRad anAlgo = new(GccEnt.Unqualified(Lin(0, 0, 1, 0)), GccEnt.Unqualified(Lin(0, 2, 1, 0)), 1.0, Tol);
        Assert.True(anAlgo.IsDone());
        Assert.Equal(0, anAlgo.NbSolutions());
    }

    [Fact]
    public void LineEnclosingQualifier_Throws()
    {
        gp_Lin2d aL = Lin(0, 0, 1, 0);
        GccEnt_QualifiedLin aBad = new(aL, GccEnt_enclosing);
        Assert.Throws<GccEnt_BadQualifier>(() => new GccAna_Circ2d2TanRad(aBad, GccEnt.Unqualified(aL), 1.0, Tol));
        Assert.Throws<GccEnt_BadQualifier>(() => new GccAna_Circ2d2TanRad(aBad, new gp_Pnt2d(0, 1), 1.0, Tol));
        Assert.Throws<GccEnt_BadQualifier>(() => new GccAna_Circ2d2TanRad(GccEnt.Unqualified(Circ(0, 0, 1)), aBad, 1.0, Tol));
        GccEnt_QualifiedCirc aNoQual = new(Circ(0, 0, 1), GccEnt_noqualifier);
        Assert.Throws<GccEnt_BadQualifier>(() => new GccAna_Circ2d2TanRad(aNoQual, GccEnt.Unqualified(Circ(4, 0, 1)), 1.0, Tol));
    }

    [Fact]
    public void NegativeRadius_Throws()
    {
        gp_Lin2d aL1 = Lin(0, 0, 1, 0), aL2 = Lin(0, 0, 0, 1);
        gp_Circ2d aC1 = Circ(0, 0, 1), aC2 = Circ(4, 0, 1);
        Assert.Throws<Standard_NegativeValue>(() => new GccAna_Circ2d2TanRad(GccEnt.Unqualified(aL1), GccEnt.Unqualified(aL2), -1.0, Tol));
        Assert.Throws<Standard_NegativeValue>(() => new GccAna_Circ2d2TanRad(GccEnt.Unqualified(aC1), GccEnt.Unqualified(aC2), -1.0, Tol));
        Assert.Throws<Standard_NegativeValue>(() => new GccAna_Circ2d2TanRad(GccEnt.Unqualified(aC1), GccEnt.Unqualified(aL1), -1.0, Tol));
        Assert.Throws<Standard_NegativeValue>(() => new GccAna_Circ2d2TanRad(GccEnt.Unqualified(aC1), new gp_Pnt2d(3, 0), -1.0, Tol));
        Assert.Throws<Standard_NegativeValue>(() => new GccAna_Circ2d2TanRad(GccEnt.Unqualified(aL1), new gp_Pnt2d(3, 1), -1.0, Tol));
        Assert.Throws<Standard_NegativeValue>(() => new GccAna_Circ2d2TanRad(new gp_Pnt2d(0, 0), new gp_Pnt2d(1, 0), -1.0, Tol));
    }

    [Fact]
    public void Accessors_OutOfRange()
    {
        GccAna_Circ2d2TanRad anAlgo = new(new gp_Pnt2d(0, 0), new gp_Pnt2d(2, 0), 2.0, Tol);
        Assert.Throws<Standard_OutOfRange>(() => anAlgo.ThisSolution(0));
        Assert.Throws<Standard_OutOfRange>(() => anAlgo.ThisSolution(3));
        Assert.Throws<Standard_OutOfRange>(() => anAlgo.WhichQualifier(3, out _, out _));
        Assert.Throws<Standard_OutOfRange>(() => anAlgo.Tangency1(3, out _, out _, out _));
        Assert.Throws<Standard_OutOfRange>(() => anAlgo.Tangency2(0, out _, out _, out _));
        Assert.Throws<Standard_OutOfRange>(() => anAlgo.IsTheSame1(3));
        Assert.Throws<Standard_OutOfRange>(() => anAlgo.IsTheSame2(3));
    }

    // C1 at the origin, C2 at (4, 0), both of radius 1, and solutions of radius 2.5: outside both (2), enclosing one
    // and outside the other (2 + 2); none encloses both, and none is enclosed (the radius is larger).
    [Theory]
    [InlineData(GccEnt_unqualified, GccEnt_unqualified, 6)]
    [InlineData(GccEnt_outside, GccEnt_outside, 2)]
    [InlineData(GccEnt_enclosing, GccEnt_outside, 2)]
    [InlineData(GccEnt_outside, GccEnt_enclosing, 2)]
    [InlineData(GccEnt_enclosing, GccEnt_enclosing, 0)]
    [InlineData(GccEnt_enclosed, GccEnt_unqualified, 0)]
    public void TwoCircles(GccEnt_Position theQ1, GccEnt_Position theQ2, int theNb)
    {
        gp_Circ2d aC1 = Circ(0, 0, 1), aC2 = Circ(4, 0, 1);
        GccAna_Circ2d2TanRad anAlgo = new(new GccEnt_QualifiedCirc(aC1, theQ1), new GccEnt_QualifiedCirc(aC2, theQ2), 2.5, Tol);
        Assert.True(anAlgo.IsDone());
        Assert.Equal(theNb, anAlgo.NbSolutions());
        for (int i = 1; i <= theNb; i++)
        {
            gp_Circ2d aSol = anAlgo.ThisSolution(i);
            Assert.Equal(2.5, aSol.Radius());
            CheckTangency1(anAlgo, i, aC1);
            CheckTangency2(anAlgo, i, aC2);
            anAlgo.WhichQualifier(i, out GccEnt_Position aQ1, out GccEnt_Position aQ2);
            Assert.Equal(CircQualifier(aC1, aSol), aQ1);
            Assert.Equal(CircQualifier(aC2, aSol), aQ2);
            if (theQ1 != GccEnt_unqualified)
            {
                Assert.Equal(theQ1, aQ1);
            }
            if (theQ2 != GccEnt_unqualified)
            {
                Assert.Equal(theQ2, aQ2);
            }
        }
    }

    // OCCT: with one circle enclosing and the other unqualified, a test meant for the boundary case ("> 0.0" for
    // "<= Tol") always holds, and the one circle returned, centred at center1 + (Radius - R1) dir1, encloses the first
    // circle but isn't tangent to the second (the true answer here is the 2 circles enclosing C1, outside C2).
    [Theory]
    [InlineData(GccEnt_enclosing, GccEnt_unqualified, 1.5)]
    [InlineData(GccEnt_unqualified, GccEnt_enclosing, 2.5)]
    public void TwoCircles_EnclosingUnqualified(GccEnt_Position theQ1, GccEnt_Position theQ2, double theX)
    {
        gp_Circ2d aC1 = Circ(0, 0, 1), aC2 = Circ(4, 0, 1);
        GccAna_Circ2d2TanRad anAlgo = new(new GccEnt_QualifiedCirc(aC1, theQ1), new GccEnt_QualifiedCirc(aC2, theQ2), 2.5, Tol);
        Assert.True(anAlgo.IsDone());
        Assert.Equal(1, anAlgo.NbSolutions());
        AssertNear(anAlgo.ThisSolution(1).Location(), new gp_Pnt2d(theX, 0));
    }

    // OCCT: with one circle outside and the other unqualified, the second pair of offset circles is centred on the
    // same point (C(1) for C(2)), so the 2 circles enclosing the unqualified circle are missed
    [Fact]
    public void TwoCircles_UnqualifiedOutside()
    {
        gp_Circ2d aC1 = Circ(0, 0, 1), aC2 = Circ(4, 0, 1);
        GccAna_Circ2d2TanRad anAlgo = new(GccEnt.Unqualified(aC1), GccEnt.Outside(aC2), 2.5, Tol);
        Assert.True(anAlgo.IsDone());
        Assert.Equal(2, anAlgo.NbSolutions());
        for (int i = 1; i <= 2; i++)
        {
            CheckTangency1(anAlgo, i, aC1);
            CheckTangency2(anAlgo, i, aC2);
            anAlgo.WhichQualifier(i, out GccEnt_Position aQ1, out GccEnt_Position aQ2);
            Assert.Equal(GccEnt_outside, aQ1);
            Assert.Equal(GccEnt_outside, aQ2);
        }
    }

    // Two circles of radius 2 whose centres are 1 apart: solutions of radius 0.5 inside both, or inside one and
    // outside the other
    [Fact]
    public void TwoOverlappingCircles_Enclosed()
    {
        gp_Circ2d aC1 = Circ(0, 0, 2), aC2 = Circ(1, 0, 2);
        GccAna_Circ2d2TanRad anAlgo = new(GccEnt.Enclosed(aC1), GccEnt.Enclosed(aC2), 0.5, Tol);
        Assert.True(anAlgo.IsDone());
        Assert.Equal(2, anAlgo.NbSolutions());
        for (int i = 1; i <= 2; i++)
        {
            CheckTangency1(anAlgo, i, aC1);
            CheckTangency2(anAlgo, i, aC2);
            Assert.Equal(GccEnt_enclosed, CircQualifier(aC1, anAlgo.ThisSolution(i)));
            Assert.Equal(GccEnt_enclosed, CircQualifier(aC2, anAlgo.ThisSolution(i)));
        }
    }

    [Fact]
    public void ConcentricCircles_NoSolution()
    {
        GccAna_Circ2d2TanRad anAlgo = new(GccEnt.Unqualified(Circ(0, 0, 1)), GccEnt.Unqualified(Circ(0, 0, 3)), 1.0, Tol);
        Assert.True(anAlgo.IsDone());
        Assert.Equal(0, anAlgo.NbSolutions());
    }

    // A circle at (0, 0.5) of radius 1 cutting the X axis, solutions of radius 0.3: above the axis, outside the
    // circle (2) or inside it (2); below the axis, outside the circle (2).
    [Theory]
    [InlineData(GccEnt_unqualified, GccEnt_unqualified, 6)]
    [InlineData(GccEnt_outside, GccEnt_unqualified, 4)]
    [InlineData(GccEnt_enclosed, GccEnt_unqualified, 2)]
    [InlineData(GccEnt_unqualified, GccEnt_enclosed, 4)]
    [InlineData(GccEnt_unqualified, GccEnt_outside, 2)]
    [InlineData(GccEnt_outside, GccEnt_outside, 2)]
    [InlineData(GccEnt_enclosing, GccEnt_unqualified, 0)]
    public void CircleAndLine(GccEnt_Position theQ1, GccEnt_Position theQ2, int theNb)
    {
        gp_Circ2d aC1 = Circ(0, 0.5, 1);
        gp_Lin2d aL2 = Lin(0, 0, 1, 0);
        GccAna_Circ2d2TanRad anAlgo = new(new GccEnt_QualifiedCirc(aC1, theQ1), new GccEnt_QualifiedLin(aL2, theQ2), 0.3, Tol);
        Assert.True(anAlgo.IsDone());
        Assert.Equal(theNb, anAlgo.NbSolutions());
        for (int i = 1; i <= theNb; i++)
        {
            gp_Circ2d aSol = anAlgo.ThisSolution(i);
            Assert.Equal(0.3, aSol.Radius());
            CheckTangency1(anAlgo, i, aC1);
            CheckTangencyLin(anAlgo, i, aL2, false);
            anAlgo.WhichQualifier(i, out GccEnt_Position aQ1, out GccEnt_Position aQ2);
            Assert.Equal(CircQualifier(aC1, aSol), aQ1);
            Assert.Equal(LinQualifier(aL2, aSol), aQ2);
            Assert.False(anAlgo.IsTheSame1(i));
            Assert.False(anAlgo.IsTheSame2(i));
        }
    }

    // A unit circle at the origin and a point outside it, at (3, 0): solutions of radius 1.5 outside the circle (2);
    // a point inside it, at (0.5, 0): solutions of radius 0.5 inside the circle (2).
    [Theory]
    [InlineData(3.0, 1.5, GccEnt_unqualified, 2)]
    [InlineData(3.0, 1.5, GccEnt_outside, 2)]
    [InlineData(3.0, 1.5, GccEnt_enclosing, 0)]
    [InlineData(3.0, 1.5, GccEnt_enclosed, 0)]
    [InlineData(0.5, 0.5, GccEnt_unqualified, 2)]
    [InlineData(0.5, 0.5, GccEnt_enclosed, 2)]
    [InlineData(0.5, 0.5, GccEnt_outside, 0)]
    public void CircleAndPoint(double theX, double theRadius, GccEnt_Position theQ1, int theNb)
    {
        gp_Circ2d aC1 = Circ(0, 0, 1);
        gp_Pnt2d aP2 = new(theX, 0);
        GccAna_Circ2d2TanRad anAlgo = new(new GccEnt_QualifiedCirc(aC1, theQ1), aP2, theRadius, Tol);
        Assert.True(anAlgo.IsDone());
        Assert.Equal(theNb, anAlgo.NbSolutions());
        for (int i = 1; i <= theNb; i++)
        {
            gp_Circ2d aSol = anAlgo.ThisSolution(i);
            Assert.Equal(theRadius, aSol.Radius());
            CheckTangency1(anAlgo, i, aC1);
            CheckThrough(anAlgo, i, aP2, false);
            anAlgo.WhichQualifier(i, out GccEnt_Position aQ1, out GccEnt_Position aQ2);
            Assert.Equal(CircQualifier(aC1, aSol), aQ1);
            Assert.Equal(GccEnt_noqualifier, aQ2);
        }
    }

    // The X axis and the point (0, 0.5): solutions of radius 1 above the axis (2)
    [Theory]
    [InlineData(GccEnt_unqualified, 2)]
    [InlineData(GccEnt_enclosed, 2)]
    [InlineData(GccEnt_outside, 0)]
    public void LineAndPoint(GccEnt_Position theQ1, int theNb)
    {
        gp_Lin2d aL1 = Lin(0, 0, 1, 0);
        gp_Pnt2d aP2 = new(0, 0.5);
        GccAna_Circ2d2TanRad anAlgo = new(new GccEnt_QualifiedLin(aL1, theQ1), aP2, 1.0, Tol);
        Assert.True(anAlgo.IsDone());
        Assert.Equal(theNb, anAlgo.NbSolutions());
        for (int i = 1; i <= theNb; i++)
        {
            gp_Circ2d aSol = anAlgo.ThisSolution(i);
            Assert.Equal(1.0, aSol.Radius());
            CheckTangencyLin(anAlgo, i, aL1, true);
            CheckThrough(anAlgo, i, aP2, false);
            anAlgo.WhichQualifier(i, out GccEnt_Position aQ1, out GccEnt_Position aQ2);
            Assert.Equal(GccEnt_enclosed, aQ1);
            Assert.Equal(GccEnt_noqualifier, aQ2);
        }
    }

    // A solution equal to an argument: the circle of radius 1 through a point on it
    [Fact]
    public void CircleAndPointOnIt_SolutionIsTheArgument()
    {
        gp_Circ2d aC1 = Circ(0, 0, 1);
        GccAna_Circ2d2TanRad anAlgo = new(GccEnt.Unqualified(aC1), new gp_Pnt2d(1, 0), 1.0, Tol);
        Assert.True(anAlgo.IsDone());
        Assert.True(anAlgo.NbSolutions() >= 1);
        Assert.True(anAlgo.IsTheSame1(1));
        Assert.Equal(aC1.Location().X(), anAlgo.ThisSolution(1).Location().X());
        Assert.Throws<StdFail_NotDone>(() => anAlgo.Tangency1(1, out _, out _, out _));
        for (int i = 2; i <= anAlgo.NbSolutions(); i++)
        {
            CheckTangency1(anAlgo, i, aC1);
            CheckThrough(anAlgo, i, new gp_Pnt2d(1, 0), false);
        }
    }
}
