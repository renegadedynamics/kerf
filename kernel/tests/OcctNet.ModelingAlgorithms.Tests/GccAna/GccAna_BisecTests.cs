// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.GccInt_IType;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// GccAna's bisectors (Pnt2dBisec, LinPnt2dBisec, Lin2dBisec, CircLin2dBisec, CircPnt2dBisec, Circ2dBisec) on OCCT's
/// documented configurations: the kind and number of solutions, and every solution's points equidistant from both
/// arguments.
/// </summary>
public class GccAna_BisecTests
{
    private const double Tol = 1.0e-9;
    private static readonly Direction2D DX = new(Direction2D.D.X);

    private static Circle2D Circ(double x, double y, double r) => new(new Axis2D(new Point2D(x, y), DX), r);

    private static double DistToCirc(in Circle2D theC, in Point2D theP) => Math.Abs(theC.Location().Distance(theP) - theC.Radius());

    /// <summary>
    /// True if theP is equidistant from the two circles in one of the senses GccAna's bisectors take: the distances to
    /// the near or the far side of each circle (|d - R| or d + R), as the centres of the circles tangent to both are.
    /// </summary>
    private static bool Equidistant(in Circle2D theC1, in Circle2D theC2, in Point2D theP, double theTol)
    {
        double d1 = theC1.Location().Distance(theP), d2 = theC2.Location().Distance(theP);
        foreach (double a in new[] { Math.Abs(d1 - theC1.Radius()), d1 + theC1.Radius() })
        {
            foreach (double b in new[] { Math.Abs(d2 - theC2.Radius()), d2 + theC2.Radius() })
            {
                if (Math.Abs(a - b) < theTol)
                {
                    return true;
                }
            }
        }
        return false;
    }

    /// <summary>Points along the bisecting curve, through ElCLib.</summary>
    private static IEnumerable<Point2D> Samples(GccInt_Bisec theB)
    {
        double[] aParams = [-2.0, -0.7, 0.0, 0.3, 1.1, 2.5];
        foreach (double u in aParams)
        {
            yield return theB.ArcType() switch
            {
                GccInt_Lin => global::Kerf.Engine.Geometry.ElCLib.Value(u, theB.Line()),
                GccInt_Cir => global::Kerf.Engine.Geometry.ElCLib.Value(u, theB.Circle()),
                GccInt_Ell => global::Kerf.Engine.Geometry.ElCLib.Value(u, theB.Ellipse()),
                GccInt_Hpr => global::Kerf.Engine.Geometry.ElCLib.Value(u, theB.Hyperbola()),
                GccInt_Par => global::Kerf.Engine.Geometry.ElCLib.Value(u, theB.Parabola()),
                _ => theB.Point(),
            };
        }
    }

    [Fact]
    public void Pnt2dBisec_PerpendicularBisector()
    {
        Point2D aP1 = new(1, 2), aP2 = new(4, -2);
        GccAna_Pnt2dBisec anAlgo = new(aP1, aP2);
        Assert.True(anAlgo.IsDone());
        Assert.True(anAlgo.HasSolution());
        Line2D aL = anAlgo.ThisSolution();
        Assert.Equal(0.0, aL.Distance(new Point2D(2.5, 0.0)), Tol);
        Assert.Equal(0.0, aL.Direction().Dot(new Direction2D(aP2.XY() - aP1.XY())), Tol);
        foreach (Point2D aP in Samples(new GccInt_BLine(aL)))
        {
            Assert.Equal(aP.Distance(aP1), aP.Distance(aP2), Tol);
        }
    }

    [Fact]
    public void Pnt2dBisec_CoincidentPoints_NoSolution()
    {
        GccAna_Pnt2dBisec anAlgo = new(new Point2D(1, 1), new Point2D(1, 1));
        Assert.True(anAlgo.IsDone());
        Assert.False(anAlgo.HasSolution());
        Assert.Throws<GccAna_NoSolution>(() => anAlgo.ThisSolution());
    }

    [Fact]
    public void LinPnt2dBisec_Parabola()
    {
        Line2D aL = new(new Point2D(0, 0), new Direction2D(1, 1));
        foreach (Point2D aP in new[] { new Point2D(-1, 2), new Point2D(3, -1) })
        {
            GccAna_LinPnt2dBisec anAlgo = new(aL, aP);
            Assert.True(anAlgo.IsDone());
            GccInt_Bisec aB = anAlgo.ThisSolution();
            Assert.Equal(GccInt_Par, aB.ArcType());
            Assert.Equal(0.0, aB.Parabola().Focus().Distance(aP), Tol);
            foreach (Point2D aQ in Samples(aB))
            {
                Assert.Equal(aQ.Distance(aP), aL.Distance(aQ), Tol);
            }
        }
    }

    [Fact]
    public void LinPnt2dBisec_PointOnTheLine_Line()
    {
        Line2D aL = new(new Point2D(0, 0), DX);
        GccAna_LinPnt2dBisec anAlgo = new(aL, new Point2D(2, 0));
        GccInt_Bisec aB = anAlgo.ThisSolution();
        Assert.Equal(GccInt_Lin, aB.ArcType());
        Assert.Equal(0.0, aB.Line().Direction().Dot(DX), Tol);
        Assert.Equal(0.0, aB.Line().Distance(new Point2D(2, 0)), Tol);
        Assert.Throws<MethodNotImplementedException>(() => aB.Parabola());
    }

    [Fact]
    public void Lin2dBisec_Secant_InsideThenOutside()
    {
        Line2D aL1 = new(new Point2D(0, 0), DX);
        Line2D aL2 = new(new Point2D(1, 0), new Direction2D(1, 1));
        GccAna_Lin2dBisec anAlgo = new(aL1, aL2);
        Assert.True(anAlgo.IsDone());
        Assert.Equal(2, anAlgo.NbSolutions());
        // The first along D1 + D2
        Assert.True(anAlgo.ThisSolution(1).Direction().IsParallel(new Direction2D(DX.XY() + aL2.Direction().XY()), 1.0e-12));
        for (int i = 1; i <= 2; i++)
        {
            Line2D aSol = anAlgo.ThisSolution(i);
            foreach (Point2D aP in Samples(new GccInt_BLine(aSol)))
            {
                Assert.Equal(aL1.Distance(aP), aL2.Distance(aP), Tol);
            }
            anAlgo.Intersection1(i, out double aPs, out double aPa, out Point2D aPnt);
            Assert.Equal(0.0, aPnt.Distance(new Point2D(1, 0)), Tol);
            Assert.Equal(0.0, global::Kerf.Engine.Geometry.ElCLib.Value(aPs, aSol).Distance(aPnt), Tol);
            Assert.Equal(0.0, global::Kerf.Engine.Geometry.ElCLib.Value(aPa, aL1).Distance(aPnt), Tol);
            anAlgo.Intersection2(i, out aPs, out aPa, out aPnt);
            Assert.Equal(0.0, global::Kerf.Engine.Geometry.ElCLib.Value(aPa, aL2).Distance(aPnt), Tol);
        }
        Assert.Equal(0.0, anAlgo.ThisSolution(1).Direction().Dot(anAlgo.ThisSolution(2).Direction()), Tol);
        Assert.Throws<OutOfRangeException>(() => anAlgo.ThisSolution(3));
    }

    [Fact]
    public void Lin2dBisec_Parallel_OneMidline()
    {
        Line2D aL1 = new(new Point2D(0, 0), DX);
        Line2D aL2 = new(new Point2D(5, 2), new Direction2D(-1, 0));
        GccAna_Lin2dBisec anAlgo = new(aL1, aL2);
        Assert.Equal(1, anAlgo.NbSolutions());
        Assert.Equal(1.0, anAlgo.ThisSolution(1).Location().Y(), Tol);
        Assert.Equal(0.0, anAlgo.ThisSolution(1).Direction().Crossed(DX), Tol);
    }

    [Fact]
    public void Lin2dBisec_Identical_TheFirstLine()
    {
        Line2D aL1 = new(new Point2D(0, 1), new Direction2D(1, 2));
        GccAna_Lin2dBisec anAlgo = new(aL1, new Line2D(new Point2D(1, 3), new Direction2D(1, 2)));
        Assert.Equal(1, anAlgo.NbSolutions());
        Assert.Equal(aL1, anAlgo.ThisSolution(1));
    }

    [Fact]
    public void CircLin2dBisec_TwoParabolas()
    {
        Circle2D aC = Circ(0, 3, 1);
        Line2D aL = new(new Point2D(0, 0), DX);
        GccAna_CircLin2dBisec anAlgo = new(aC, aL);
        Assert.True(anAlgo.IsDone());
        Assert.Equal(2, anAlgo.NbSolutions());
        for (int i = 1; i <= 2; i++)
        {
            GccInt_Bisec aB = anAlgo.ThisSolution(i);
            Assert.Equal(GccInt_Par, aB.ArcType());
            foreach (Point2D aP in Samples(aB))
            {
                // the first parabola is equidistant from the line and the circle's near side, the second its far side
                double aDc = aC.Location().Distance(aP);
                Assert.Equal(aL.Distance(aP), i == 1 ? Math.Abs(aDc - aC.Radius()) : aDc + aC.Radius(), Tol);
            }
        }
        Assert.Throws<OutOfRangeException>(() => anAlgo.ThisSolution(3));
    }

    [Fact]
    public void CircLin2dBisec_TangentLine_LineAndParabola()
    {
        Circle2D aC = Circ(0, 1, 1);
        GccAna_CircLin2dBisec anAlgo = new(aC, new Line2D(new Point2D(0, 0), DX));
        Assert.Equal(GccInt_Lin, anAlgo.ThisSolution(1).ArcType());
        Assert.Equal(GccInt_Par, anAlgo.ThisSolution(2).ArcType());
    }

    [Theory]
    [InlineData(5.0, 1.0, GccInt_Hpr, 2)] // outside: the two branches of a hyperbola
    [InlineData(1.0, 0.5, GccInt_Ell, 1)] // inside: an ellipse
    [InlineData(2.0, 0.0, GccInt_Lin, 1)] // on the circle: a line
    [InlineData(0.0, 0.0, GccInt_Cir, 1)] // the centre: a circle
    public void CircPnt2dBisec(double theX, double theY, GccInt_IType theType, int theNb)
    {
        Circle2D aC = Circ(0, 0, 2);
        Point2D aP = new(theX, theY);
        GccAna_CircPnt2dBisec anAlgo = new(aC, aP);
        Assert.True(anAlgo.IsDone());
        Assert.Equal(theNb, anAlgo.NbSolutions());
        for (int i = 1; i <= theNb; i++)
        {
            GccInt_Bisec aB = anAlgo.ThisSolution(i);
            Assert.Equal(theType, aB.ArcType());
            foreach (Point2D aQ in Samples(aB))
            {
                Assert.True(Equidistant(aC, new Circle2D(new Axis2D(aP, DX), 0.0), aQ, 1.0e-8), $"solution {i} at {aQ}");
            }
        }
    }

    [Fact]
    public void CircPnt2dBisec_Tolerance_PutsANearPointOnTheCircle()
    {
        Circle2D aC = Circ(0, 0, 2);
        Point2D aP = new(2.001, 0.0);
        Assert.Equal(2, new GccAna_CircPnt2dBisec(aC, aP).NbSolutions());
        GccAna_CircPnt2dBisec anAlgo = new(aC, aP, 0.01);
        Assert.Equal(1, anAlgo.NbSolutions());
        Assert.Equal(GccInt_Lin, anAlgo.ThisSolution(1).ArcType());
    }

    // OCCT's cases by the relative position of the circles (circle1 the larger): counts and kinds
    [Theory]
    [InlineData(0.5, 3.0, 1.0, 2, new[] { GccInt_Ell, GccInt_Ell })] // the second strictly inside
    [InlineData(0.0, 3.0, 1.0, 2, new[] { GccInt_Cir, GccInt_Cir })] // concentric
    [InlineData(2.0, 3.0, 1.0, 2, new[] { GccInt_Ell, GccInt_Lin })] // internally tangent
    [InlineData(3.0, 3.0, 1.0, 3, new[] { GccInt_Hpr, GccInt_Hpr, GccInt_Ell })] // secant
    [InlineData(3.0, 2.0, 2.0, 2, new[] { GccInt_Lin, GccInt_Ell })] // secant, same radius
    [InlineData(4.0, 3.0, 1.0, 3, new[] { GccInt_Lin, GccInt_Hpr, GccInt_Hpr })] // externally tangent
    [InlineData(6.0, 3.0, 1.0, 4, new[] { GccInt_Hpr, GccInt_Hpr, GccInt_Hpr, GccInt_Hpr })] // apart
    [InlineData(6.0, 2.0, 2.0, 3, new[] { GccInt_Lin, GccInt_Hpr, GccInt_Hpr })] // apart, same radius
    public void Circ2dBisec(double theDist, double theR1, double theR2, int theNb, GccInt_IType[] theTypes)
    {
        Circle2D aC1 = Circ(0, 0, theR1);
        Circle2D aC2 = Circ(theDist, 0, theR2);
        GccAna_Circ2dBisec anAlgo = new(aC1, aC2);
        Assert.True(anAlgo.IsDone());
        Assert.Equal(theNb, anAlgo.NbSolutions());
        for (int i = 1; i <= theNb; i++)
        {
            GccInt_Bisec aB = anAlgo.ThisSolution(i);
            Assert.Equal(theTypes[i - 1], aB.ArcType());
            foreach (Point2D aP in Samples(aB))
            {
                Assert.True(Equidistant(aC1, aC2, aP, 1.0e-8), $"solution {i} at {aP}");
            }
        }
    }

    [Fact]
    public void Circ2dBisec_SameCircleTangent_NoSolution()
    {
        // Internally tangent with the same radius is the same circle: no bisector
        GccAna_Circ2dBisec anAlgo = new(Circ(1, 1, 2), Circ(1, 1, 2));
        Assert.True(anAlgo.IsDone());
        Assert.Equal(0, anAlgo.NbSolutions());
    }

    [Fact]
    public void GccInt_Kinds()
    {
        Point2D aP = new(1, 2);
        GccInt_Bisec[] aBs =
        [
            new GccInt_BPoint(aP),
            new GccInt_BLine(new Line2D(aP, DX)),
            new GccInt_BCirc(Circ(1, 2, 3)),
            new GccInt_BElips(new Ellipse2D(new Axis2D(aP, DX), 3, 2)),
            new GccInt_BHyper(new Hyperbola2D(new Axis2D(aP, DX), 3, 2)),
            new GccInt_BParab(new Parabola2D(new Axis2D(aP, DX), 2)),
        ];
        GccInt_IType[] aTypes = [GccInt_Pnt, GccInt_Lin, GccInt_Cir, GccInt_Ell, GccInt_Hpr, GccInt_Par];
        for (int i = 0; i < aBs.Length; i++)
        {
            Assert.Equal(aTypes[i], aBs[i].ArcType());
        }
        Assert.Equal(aP, aBs[0].Point());
        Assert.Equal(3.0, aBs[2].Circle().Radius());
        Assert.Equal(2.0, aBs[3].Ellipse().MinorRadius());
        Assert.Equal(3.0, aBs[4].Hyperbola().MajorRadius());
        Assert.Equal(2.0, aBs[5].Parabola().Focal());
        Assert.Throws<MethodNotImplementedException>(() => aBs[0].Line());
        Assert.Throws<MethodNotImplementedException>(() => aBs[1].Circle());
        Assert.Throws<MethodNotImplementedException>(() => aBs[2].Ellipse());
        Assert.Throws<MethodNotImplementedException>(() => aBs[3].Hyperbola());
        Assert.Throws<MethodNotImplementedException>(() => aBs[4].Parabola());
        Assert.Throws<MethodNotImplementedException>(() => aBs[5].Point());
    }
}
