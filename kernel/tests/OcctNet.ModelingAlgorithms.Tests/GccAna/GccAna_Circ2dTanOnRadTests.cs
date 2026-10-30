// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GccEnt_Position;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// GccAna_Circ2dTanOnRad on OCCT's documented configurations: solution counts, qualifiers, and on every solution the
/// radius, the centre on the On-curve, the tangency and the tangency point. Where OCCT's own results are wrong
/// (docs/occt-issues.md), the tests check what OCCT computes.
/// </summary>
public class GccAna_Circ2dTanOnRadTests
{
    private const double Tol = 1.0e-9;
    private static readonly gp_Dir2d DX = new(gp_Dir2d.D.X);

    private static gp_Circ2d Circ(double x, double y, double r) => new(new gp_Ax2d(new gp_Pnt2d(x, y), DX), r);

    private static gp_Lin2d Lin(double x, double y, double dx, double dy) => new(new gp_Pnt2d(x, y), new gp_Dir2d(dx, dy));

    private static void AssertNear(in gp_Pnt2d theP, in gp_Pnt2d theQ) => Assert.True(theP.Distance(theQ) < 1.0e-7, $"{theP} != {theQ}");

    private static void CheckCentreOnLine(GccAna_Circ2dTanOnRad theAlgo, int theIndex, in gp_Lin2d theOn)
    {
        gp_Circ2d aSol = theAlgo.ThisSolution(theIndex);
        Assert.True(theOn.Distance(aSol.Location()) < 1.0e-7);
        theAlgo.CenterOn3(theIndex, out double aParArg, out _);
        AssertNear(ElCLib.Value(aParArg, theOn), aSol.Location());
    }

    private static void CheckCentreOnCirc(GccAna_Circ2dTanOnRad theAlgo, int theIndex, in gp_Circ2d theOn)
    {
        gp_Circ2d aSol = theAlgo.ThisSolution(theIndex);
        Assert.True(theOn.Distance(aSol.Location()) < 1.0e-7);
        theAlgo.CenterOn3(theIndex, out double aParArg, out _);
        AssertNear(ElCLib.Value(aParArg, theOn), aSol.Location());
    }

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

    private static void CheckTangentCirc(GccAna_Circ2dTanOnRad theAlgo, int theIndex, in gp_Circ2d theArg)
    {
        gp_Circ2d aSol = theAlgo.ThisSolution(theIndex);
        GccEnt_Position aQual = CircQualifier(theArg, aSol);
        theAlgo.WhichQualifier(theIndex, out GccEnt_Position aQ1);
        Assert.Equal(aQual, aQ1);
        theAlgo.Tangency1(theIndex, out double aParSol, out double aParArg, out gp_Pnt2d aPnt);
        AssertNear(ElCLib.Value(aParSol, aSol), aPnt);
        AssertNear(ElCLib.Value(aParArg, theArg), aPnt);
    }

    // Unit circle at the origin, centres on the X axis, radius 0.4: outside at x = +-1.4, inside at x = +-0.6.
    // OCCT: the tangency points are the points of the solutions opposite the true ones (the sign of dir1cen is
    // swapped), so they are checked on the solution, at the diameter's other end.
    [Theory]
    [InlineData(GccEnt_unqualified, 4)]
    [InlineData(GccEnt_outside, 2)]
    [InlineData(GccEnt_enclosed, 2)]
    [InlineData(GccEnt_enclosing, 0)]
    public void CircleCentreOnLine(GccEnt_Position theQ, int theNb)
    {
        gp_Circ2d aC = Circ(0, 0, 1);
        gp_Lin2d anOn = Lin(0, 0, 1, 0);
        GccAna_Circ2dTanOnRad anAlgo = new(new GccEnt_QualifiedCirc(aC, theQ), anOn, 0.4, Tol);
        Assert.True(anAlgo.IsDone());
        Assert.Equal(theNb, anAlgo.NbSolutions());
        for (int i = 1; i <= theNb; i++)
        {
            gp_Circ2d aSol = anAlgo.ThisSolution(i);
            Assert.Equal(0.4, aSol.Radius());
            Assert.False(anAlgo.IsTheSame1(i));
            CheckCentreOnLine(anAlgo, i, anOn);
            anAlgo.WhichQualifier(i, out GccEnt_Position aQ1);
            Assert.Equal(CircQualifier(aC, aSol), aQ1);
            anAlgo.Tangency1(i, out double aParSol, out _, out gp_Pnt2d aPnt);
            AssertNear(ElCLib.Value(aParSol, aSol), aPnt);
            gp_Pnt2d aTrue = new(aSol.Location().XY() - (aPnt.XY() - aSol.Location().XY()));
            Assert.Equal(1.0, aTrue.Distance(aC.Location()), 1.0e-9);
        }
    }

    // OCCT puts the tangency point at the centre of the argument (the sign is swapped), and ElCLib's parameter of a
    // circle's centre normalises a null vector: native OCCT throws Standard_ConstructionError, as the port does
    [Fact]
    public void CircleCentreOnLine_TangencyPointAtTheCentre_Throws()
    {
        Assert.Throws<Standard_ConstructionError>(
            () => new GccAna_Circ2dTanOnRad(GccEnt.Enclosed(Circ(0, 0, 1)), Lin(0, 0, 1, 0), 0.5, Tol));
    }

    // OCCT: CenterOn3 returns the tangency point, not the centre
    [Fact]
    public void CenterOn3_ReturnsTheTangencyPoint()
    {
        GccAna_Circ2dTanOnRad anAlgo = new(GccEnt.Outside(Circ(0, 0, 1)), Lin(0, 0, 1, 0), 0.5, Tol);
        Assert.Equal(2, anAlgo.NbSolutions());
        anAlgo.CenterOn3(1, out _, out gp_Pnt2d aPnt);
        anAlgo.Tangency1(1, out _, out _, out gp_Pnt2d aTan);
        Assert.Equal(aTan, aPnt);
    }

    // The X axis, centres on the Y axis, radius 1: (0, 1) inside (left of +X), (0, -1) outside
    [Theory]
    [InlineData(GccEnt_unqualified, 2)]
    [InlineData(GccEnt_enclosed, 1)]
    [InlineData(GccEnt_outside, 1)]
    public void LineCentreOnLine(GccEnt_Position theQ, int theNb)
    {
        gp_Lin2d aL = Lin(0, 0, 1, 0);
        gp_Lin2d anOn = Lin(0, 0, 0, 1);
        GccAna_Circ2dTanOnRad anAlgo = new(new GccEnt_QualifiedLin(aL, theQ), anOn, 1.0, Tol);
        Assert.True(anAlgo.IsDone());
        Assert.Equal(theNb, anAlgo.NbSolutions());
        for (int i = 1; i <= theNb; i++)
        {
            gp_Circ2d aSol = anAlgo.ThisSolution(i);
            Assert.Equal(1.0, aL.Distance(aSol.Location()), 1.0e-9);
            Assert.True(anOn.Distance(aSol.Location()) < 1.0e-9);
            anAlgo.WhichQualifier(i, out GccEnt_Position aQ1);
            Assert.Equal(aSol.Location().Y() > 0 ? GccEnt_enclosed : GccEnt_outside, aQ1);
            anAlgo.Tangency1(i, out double aParSol, out double aParArg, out gp_Pnt2d aPnt);
            AssertNear(ElCLib.Value(aParSol, aSol), aPnt);
            AssertNear(ElCLib.Value(aParArg, aL), aPnt);
        }
        if (theQ == GccEnt_enclosed)
        {
            AssertNear(anAlgo.ThisSolution(1).Location(), new gp_Pnt2d(0, 1));
        }
    }

    // OCCT: the second solution's centre parameter is that of the first (pntcen3 from cirsol(1))
    [Fact]
    public void LineCentreOnLine_SecondCentreParameterIsTheFirst()
    {
        GccAna_Circ2dTanOnRad anAlgo = new(GccEnt.Unqualified(Lin(0, 0, 1, 0)), Lin(0, 0, 0, 1), 1.0, Tol);
        Assert.Equal(2, anAlgo.NbSolutions());
        anAlgo.CenterOn3(1, out double aPar1, out _);
        anAlgo.CenterOn3(2, out double aPar2, out _);
        Assert.Equal(aPar1, aPar2);
    }

    [Fact]
    public void ParallelLines_NoSolution()
    {
        GccAna_Circ2dTanOnRad anAlgo = new(GccEnt.Unqualified(Lin(0, 0, 1, 0)), Lin(0, 1, 1, 0), 1.0, Tol);
        Assert.True(anAlgo.IsDone());
        Assert.Equal(0, anAlgo.NbSolutions());
    }

    // The point (1, 0), centres on the Y axis: radius 2 at (0, +-sqrt(3)), radius 1 at the origin, radius 0.5 none
    [Theory]
    [InlineData(2.0, 2)]
    [InlineData(1.0, 1)]
    [InlineData(0.5, 0)]
    public void PointCentreOnLine(double theRadius, int theNb)
    {
        gp_Pnt2d aP = new(1, 0);
        gp_Lin2d anOn = Lin(0, 0, 0, 1);
        GccAna_Circ2dTanOnRad anAlgo = new(aP, anOn, theRadius, Tol);
        Assert.True(anAlgo.IsDone());
        Assert.Equal(theNb, anAlgo.NbSolutions());
        for (int i = 1; i <= theNb; i++)
        {
            gp_Circ2d aSol = anAlgo.ThisSolution(i);
            Assert.Equal(theRadius, aSol.Radius());
            Assert.Equal(theRadius, aSol.Location().Distance(aP), 1.0e-9);
            CheckCentreOnLine(anAlgo, i, anOn);
            anAlgo.WhichQualifier(i, out GccEnt_Position aQ1);
            Assert.Equal(GccEnt_noqualifier, aQ1);
            anAlgo.Tangency1(i, out double aParSol, out _, out gp_Pnt2d aPnt);
            AssertNear(aPnt, aP);
            AssertNear(ElCLib.Value(aParSol, aSol), aPnt);
        }
    }

    // OCCT: with the point on the line, the two solutions are found but IsDone stays false
    [Fact]
    public void PointOnTheCentreLine_NotDone()
    {
        GccAna_Circ2dTanOnRad anAlgo = new(new gp_Pnt2d(0, 3), Lin(0, 0, 0, 1), 1.0, Tol);
        Assert.False(anAlgo.IsDone());
        Assert.Equal(2, anAlgo.NbSolutions());
        Assert.Throws<StdFail_NotDone>(() => anAlgo.Tangency1(1, out _, out _, out _));
        AssertNear(anAlgo.ThisSolution(1).Location(), new gp_Pnt2d(0, 4));
        AssertNear(anAlgo.ThisSolution(2).Location(), new gp_Pnt2d(0, 2));
    }

    // Unit circle at the origin, centres on the circle at (3, 0) of radius 1, radius 1.5: outside the unit circle at
    // distance 2.5 (2); none encloses or is enclosed by it.
    [Theory]
    [InlineData(GccEnt_unqualified, 2)]
    [InlineData(GccEnt_outside, 2)]
    [InlineData(GccEnt_enclosing, 0)]
    [InlineData(GccEnt_enclosed, 0)]
    public void CircleCentreOnCircle(GccEnt_Position theQ, int theNb)
    {
        gp_Circ2d aC = Circ(0, 0, 1);
        gp_Circ2d anOn = Circ(3, 0, 1);
        GccAna_Circ2dTanOnRad anAlgo = new(new GccEnt_QualifiedCirc(aC, theQ), anOn, 1.5, Tol);
        Assert.True(anAlgo.IsDone());
        Assert.Equal(theNb, anAlgo.NbSolutions());
        for (int i = 1; i <= theNb; i++)
        {
            Assert.Equal(1.5, anAlgo.ThisSolution(i).Radius());
            CheckCentreOnCirc(anAlgo, i, anOn);
            CheckTangentCirc(anAlgo, i, aC);
        }
    }

    // Unit circles at the origin and at (1, 0), radius just over 3 (within the tolerance): the circle centred at
    // (2, 0) encloses the first. OCCT's "sign" branch writes the qualifier at index 0, outside its array: native OCCT's
    // range check throws Standard_OutOfRange (tools/gccrec's hand-picked cases), and so does the port.
    [Fact]
    public void CircleCentreOnCircle_EnclosingAll_Throws()
    {
        gp_Circ2d aC = Circ(0, 0, 1);
        gp_Circ2d anOn = Circ(1, 0, 1);
        Assert.Throws<Standard_OutOfRange>(() => new GccAna_Circ2dTanOnRad(GccEnt.Unqualified(aC), anOn, 3.0 + 5.0e-10, Tol));
    }

    // The X axis, centres on the circle of radius 2 at the origin, radius 1. OCCT truncates a dot product into its side
    // (0), so the tangency point is the solution's centre and ElCLib's parameter there normalises a null vector: native
    // OCCT throws Standard_ConstructionError on every such case (tools/gccrec), as the port does
    [Theory]
    [InlineData(GccEnt_unqualified)]
    [InlineData(GccEnt_enclosed)]
    [InlineData(GccEnt_outside)]
    public void LineCentreOnCircle_Throws(GccEnt_Position theQ)
    {
        gp_Lin2d aL = Lin(0, 0, 1, 0);
        gp_Circ2d anOn = Circ(0, 0, 2);
        Assert.Throws<Standard_ConstructionError>(() => new GccAna_Circ2dTanOnRad(new GccEnt_QualifiedLin(aL, theQ), anOn, 1.0, Tol));
    }



    // The point (1, 1), centres on the circle at (3, 0) of radius 1, radius 2.5: two circles
    [Fact]
    public void PointCentreOnCircle()
    {
        gp_Pnt2d aP = new(1, 1);
        gp_Circ2d anOn = Circ(3, 0, 1);
        GccAna_Circ2dTanOnRad anAlgo = new(aP, anOn, 2.5, Tol);
        Assert.True(anAlgo.IsDone());
        Assert.Equal(2, anAlgo.NbSolutions());
        for (int i = 1; i <= 2; i++)
        {
            gp_Circ2d aSol = anAlgo.ThisSolution(i);
            Assert.Equal(2.5, aSol.Radius());
            Assert.Equal(2.5, aSol.Location().Distance(aP), 1.0e-9);
            Assert.True(anOn.Distance(aSol.Location()) < 1.0e-9);
        }
        // OCCT fills the tangency data of the first solution only
        anAlgo.Tangency1(1, out double aParSol, out _, out gp_Pnt2d aPnt);
        AssertNear(aPnt, aP);
        AssertNear(ElCLib.Value(aParSol, anAlgo.ThisSolution(1)), aP);
    }

    // The origin, centres on the circle at (3, 0) of radius 1, radius 2: the one centre at (2, 0)
    [Fact]
    public void PointCentreOnCircle_Tangent()
    {
        GccAna_Circ2dTanOnRad anAlgo = new(new gp_Pnt2d(0, 0), Circ(3, 0, 1), 2.0, Tol);
        Assert.True(anAlgo.IsDone());
        Assert.Equal(1, anAlgo.NbSolutions());
        AssertNear(anAlgo.ThisSolution(1).Location(), new gp_Pnt2d(2, 0));
    }

    [Fact]
    public void Errors()
    {
        gp_Lin2d aL = Lin(0, 0, 1, 0);
        gp_Circ2d aC = Circ(0, 0, 1);
        Assert.Throws<Standard_NegativeValue>(() => new GccAna_Circ2dTanOnRad(GccEnt.Unqualified(aC), aL, -1.0, Tol));
        Assert.Throws<Standard_NegativeValue>(() => new GccAna_Circ2dTanOnRad(GccEnt.Unqualified(aL), Lin(0, 0, 0, 1), -1.0, Tol));
        Assert.Throws<Standard_NegativeValue>(() => new GccAna_Circ2dTanOnRad(new gp_Pnt2d(1, 0), aL, -1.0, Tol));
        Assert.Throws<Standard_NegativeValue>(() => new GccAna_Circ2dTanOnRad(GccEnt.Unqualified(aC), Circ(3, 0, 1), -1.0, Tol));
        Assert.Throws<Standard_NegativeValue>(() => new GccAna_Circ2dTanOnRad(GccEnt.Unqualified(aL), aC, -1.0, Tol));
        Assert.Throws<Standard_NegativeValue>(() => new GccAna_Circ2dTanOnRad(new gp_Pnt2d(1, 0), Circ(3, 0, 1), -1.0, Tol));
        GccEnt_QualifiedLin aBad = new(aL, GccEnt_enclosing);
        Assert.Throws<GccEnt_BadQualifier>(() => new GccAna_Circ2dTanOnRad(aBad, Lin(0, 0, 0, 1), 1.0, Tol));
        Assert.Throws<GccEnt_BadQualifier>(() => new GccAna_Circ2dTanOnRad(aBad, aC, 1.0, Tol));
        GccEnt_QualifiedCirc aNoQual = new(aC, GccEnt_noqualifier);
        Assert.Throws<GccEnt_BadQualifier>(() => new GccAna_Circ2dTanOnRad(aNoQual, aL, 1.0, Tol));
        Assert.Throws<GccEnt_BadQualifier>(() => new GccAna_Circ2dTanOnRad(aNoQual, Circ(3, 0, 1), 1.0, Tol));

        GccAna_Circ2dTanOnRad anAlgo = new(GccEnt.Unqualified(aC), aL, 0.4, Tol);
        Assert.Throws<Standard_OutOfRange>(() => anAlgo.ThisSolution(0));
        Assert.Throws<Standard_OutOfRange>(() => anAlgo.ThisSolution(5));
        Assert.Throws<Standard_OutOfRange>(() => anAlgo.WhichQualifier(5, out _));
        Assert.Throws<Standard_OutOfRange>(() => anAlgo.Tangency1(5, out _, out _, out _));
        Assert.Throws<Standard_OutOfRange>(() => anAlgo.CenterOn3(0, out _, out _));
        Assert.Throws<Standard_OutOfRange>(() => anAlgo.IsTheSame1(5));
    }

    // The unit circle at (1, 0), enclosed, centres on the unit circle at the origin, radius 1: the argument itself
    [Fact]
    public void CircleCentreOnCircle_SolutionIsTheArgument()
    {
        gp_Circ2d aC = Circ(1, 0, 1);
        GccAna_Circ2dTanOnRad anAlgo = new(GccEnt.Enclosed(aC), Circ(0, 0, 1), 1.0, Tol);
        Assert.True(anAlgo.IsDone());
        Assert.Equal(1, anAlgo.NbSolutions());
        Assert.True(anAlgo.IsTheSame1(1));
        Assert.Equal(aC, anAlgo.ThisSolution(1));
    }
}
