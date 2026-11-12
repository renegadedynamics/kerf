// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The bound that tells a piece of a curve beside a piece of a surface from one that may touch it (unit 15Y2, decision
/// 32801): it never calls apart two pieces that come within the clearance, it does tell an edge beside a narrow strip
/// from the strip at once, and it says nothing of what it can't bound.
/// </summary>
public class SampleSeparationTests
{
    private static readonly Vector3D Along = new(0.6, 0.5, Math.Sqrt(0.39));
    private static readonly Vector3D Across = new Vector3D(0.5, -0.6, 0).Normalized();
    private static readonly Vector3D Up = Along.Crossed(Across).Normalized();

    // Askew to the axes, as the boxes it replaces are worst at
    private static Point3D At(double theAlong, double theAcross, double theUp)
    {
        Vector3D aP = (Along * theAlong) + (Across * theAcross) + (Up * theUp);
        return new Point3D(aP.X, aP.Y, aP.Z);
    }

    // A cubic surface of 4 by 5 spans with waves 0.3 high, about 6 by 7
    private static BSplineSurface Wavy()
    {
        Point3D[] aPoles = new Point3D[56];
        for (int i = 0; i < 7; i++)
        {
            for (int j = 0; j < 8; j++)
            {
                aPoles[(i * 8) + j] = At(j, i, 0.3 * Math.Sin(1.9 * i) * Math.Cos(1.3 * j));
            }
        }
        return new BSplineSurface(new Grid<Point3D>(aPoles, 7, 8), [0.0, 1, 2, 3, 4], [0.0, 1, 2, 4, 7, 8], [4, 1, 1, 1, 4], [4, 1, 1, 1, 1, 4], 3, 3);
    }

    // Of degrees 5 and 4 with simple knots: smooth enough at them that an adaptor's intervals of continuity leave them out
    private static BSplineSurface Smooth()
    {
        Point3D[] aPoles = new Point3D[48];
        for (int i = 0; i < 8; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                aPoles[(i * 6) + j] = At(1.2 * j, 0.8 * i, 0.4 * Math.Cos(1.1 * i) * Math.Sin(1.7 * j));
            }
        }
        return new BSplineSurface(new Grid<Point3D>(aPoles, 8, 6), [0.0, 1, 2, 3], [0.0, 2, 3], [6, 1, 1, 6], [5, 1, 5], 5, 4);
    }

    private static global::Kerf.Topology.Edge Wave(double theHeight)
    {
        Point3D[] aPoles = new Point3D[9];
        for (int i = 0; i < 9; i++)
        {
            aPoles[i] = At(0.8 * i, 1 + (0.5 * i) + Math.Sin(2.1 * i), theHeight + (0.2 * Math.Cos(1.7 * i)));
        }
        return new MakeEdge(new BSplineCurve3D(aPoles, [0.0, 1, 2, 3, 4], [6, 1, 1, 1, 6], 5)).Edge();
    }

    private static SampleSeparation Of(global::Kerf.Topology.Edge theEdge, Surface theSurface, out EdgeCurveAdaptor theCurve,
                                       out FaceSurfaceAdaptor theFace)
    {
        theCurve = new EdgeCurveAdaptor(theEdge);
        theFace = new FaceSurfaceAdaptor(new MakeFace(theSurface, 1e-7).Face());
        return new SampleSeparation(theCurve, theFace);
    }

    // The least distance between samples of the two pieces: no less than the pieces' own
    private static double Sampled(EdgeCurveAdaptor theCurve, FaceSurfaceAdaptor theFace, double t0, double t1, double u0, double u1,
                                  double v0, double v1)
    {
        const int Across = 16, OnCurve = 32;
        Point3D[] aGrid = new Point3D[(Across + 1) * (Across + 1)];
        for (int i = 0; i <= Across; i++)
        {
            for (int j = 0; j <= Across; j++)
            {
                aGrid[(i * (Across + 1)) + j] = theFace.EvalD0(u0 + ((u1 - u0) * i / Across), v0 + ((v1 - v0) * j / Across));
            }
        }
        double aLeast = double.MaxValue;
        for (int k = 0; k <= OnCurve; k++)
        {
            Point3D aP = theCurve.EvalD0(t0 + ((t1 - t0) * k / OnCurve));
            foreach (Point3D aQ in aGrid)
            {
                aLeast = Math.Min(aLeast, aP.SquareDistanceTo(aQ));
            }
        }
        return Math.Sqrt(aLeast);
    }

    // Pieces as the localisation makes them, a third of a third and so on, of curves that run through, over and beside the
    // waves: none called apart comes within the clearance, and most of those that are far are called apart
    [Fact]
    public void Apart_IsNeverSaidOfPiecesWithinTheClearance()
    {
        Random aRandom = new(15_2);
        int aTold = 0, aFar = 0, aTried = 0;
        foreach (BSplineSurface aSurface in new[] { Wavy(), Smooth() })
        {
            List<global::Kerf.Topology.Edge> anEdges = [];
            foreach (double aHeight in new[] { -0.2, 0.0, 0.05, 0.15, 0.31, 0.6 })
            {
                anEdges.Add(new MakeEdge(At(-0.5, 0.3 + aHeight, aHeight), At(7.5, 5.5 - aHeight, aHeight + 0.1)).Edge());
                anEdges.Add(new MakeEdge(new Circle3D(new Frame3D(At(3.5, 3, aHeight), new Direction3D(Up + (Across * 0.3))), 2.5)).Edge());
                anEdges.Add(new MakeEdge(new Ellipse3D(new Frame3D(At(3, 3, aHeight), new Direction3D(Up + (Along * 0.2))), 3.0, 1.2)).Edge());
                anEdges.Add(Wave(aHeight));
            }
            foreach (global::Kerf.Topology.Edge anEdge in anEdges)
            {
                SampleSeparation aSeparation = Of(anEdge, aSurface, out EdgeCurveAdaptor aCurve, out FaceSurfaceAdaptor aFace);
                double aFirst = aCurve.FirstParameter(), aLast = aCurve.LastParameter();
                for (int aTry = 0; aTry < 150; aTry++)
                {
                    (double t0, double t1) = Piece(aRandom, aFirst, aLast);
                    (double u0, double u1) = Piece(aRandom, aFace.FirstUParameter(), aFace.LastUParameter());
                    (double v0, double v1) = Piece(aRandom, aFace.FirstVParameter(), aFace.LastVParameter());
                    double aClearance = aTry % 2 == 0 ? 3e-7 : 1e-3;
                    bool isApart = aSeparation.Apart(t0, t1, u0, u1, v0, v1, aClearance);
                    double aLeast = Sampled(aCurve, aFace, t0, t1, u0, u1, v0, v1);
                    Assert.False(isApart && aLeast <= aClearance, $"apart at {aLeast}: t {t0}..{t1} u {u0}..{u1} v {v0}..{v1}");
                    aTried++;
                    if (aLeast > 0.25)
                    {
                        aFar++;
                        aTold += isApart ? 1 : 0;
                    }
                }
            }
        }
        Assert.True(aFar > aTried / 4, $"{aFar} far of {aTried}");
        Assert.True(aTold > aFar / 2, $"{aTold} told apart of {aFar} far");
    }

    private static (double, double) Piece(Random theRandom, double theFirst, double theLast)
    {
        int aCount = (int)Math.Pow(3, theRandom.Next(0, 5));
        int anIndex = theRandom.Next(0, aCount);
        double aStep = (theLast - theFirst) / aCount;
        return (theFirst + (anIndex * aStep), theFirst + ((anIndex + 1) * aStep));
    }

    // A piece of a curve that touches a flat patch at its middle, its ends off the patch by what it bends: it isn't apart,
    // which only the bound on the curve's bulge can say, since its chord is
    [Fact]
    public void Apart_AllowsForWhatACurveBulgesFromItsChord()
    {
        global::Kerf.Topology.Edge[] anEdges =
        [
            new MakeEdge(new Circle3D(new Frame3D(At(3.5, 3, 0.2), new Direction3D(Up + (Across * 0.3))), 2.5)).Edge(),
            new MakeEdge(new Ellipse3D(new Frame3D(At(3, 3, 0.1), new Direction3D(Up + (Along * 0.2))), 3.0, 1.2)).Edge(),
            Wave(0.3),
        ];
        int aBent = 0;
        foreach (global::Kerf.Topology.Edge anEdge in anEdges)
        {
            EdgeCurveAdaptor aCurve = new(anEdge);
            double aFirst = aCurve.FirstParameter(), aRange = aCurve.LastParameter() - aFirst;
            for (int k = 1; k < 40; k++)
            {
                double aMiddle = aFirst + (aRange * k / 40);
                Curve3D.ResD2 at = aCurve.EvalD2(aMiddle);
                Vector3D aTangent = at.D1.Normalized();
                Vector3D aBend = at.D2 - (aTangent * at.D2.Dot(aTangent));
                if (aBend.Magnitude < 1e-3)
                {
                    continue;
                }
                Vector3D aSide = aTangent.Crossed(aBend.Normalized());
                Point3D[] aPoles = new Point3D[16];
                for (int i = 0; i < 4; i++)
                {
                    for (int j = 0; j < 4; j++)
                    {
                        aPoles[(i * 4) + j] = at.Point + (aTangent * ((i / 3.0) - 0.5)) + (aSide * ((j / 3.0) - 0.5));
                    }
                }
                BSplineSurface aFlat = new(new Grid<Point3D>(aPoles, 4, 4), [0.0, 1], [0.0, 1], [4, 4], [4, 4], 3, 3);
                SampleSeparation aSeparation = Of(anEdge, aFlat, out _, out _);
                foreach (double aHalf in new[] { aRange / 20, aRange / 200, aRange / 2000 })
                {
                    if (aMiddle - aHalf < aFirst || aMiddle + aHalf > aFirst + aRange)
                    {
                        continue;
                    }
                    Assert.False(aSeparation.Apart(aMiddle - aHalf, aMiddle + aHalf, 0, 1, 0, 1, 3e-7), $"apart at its middle: {aMiddle} by {aHalf}");
                    aBent++;
                }
            }
        }
        Assert.True(aBent > 200, $"{aBent}");
    }

    // A line that touches a piece of a wavy surface at its middle isn't apart from it, though the piece's corners may all lie
    // to one side of the line: only the bound on the surface's bulge can say so
    [Fact]
    public void Apart_AllowsForWhatASurfaceBulgesFromItsCorners()
    {
        int aTried = 0;
        foreach (BSplineSurface aSurface in new[] { Wavy(), Smooth() })
        {
            FaceSurfaceAdaptor aFace = new(new MakeFace(aSurface, 1e-7).Face());
            double aU = aFace.LastUParameter(), aV = aFace.LastVParameter();
            foreach (int aCount in new[] { 3, 9, 27 })
            {
                for (int i = 0; i < aCount; i++)
                {
                    for (int j = 0; j < aCount; j++)
                    {
                        double u0 = aU * i / aCount, u1 = aU * (i + 1) / aCount, v0 = aV * j / aCount, v1 = aV * (j + 1) / aCount;
                        Surface.ResD1 at = aFace.EvalD1((u0 + u1) / 2, (v0 + v1) / 2);
                        foreach (Vector3D aWay in new[] { at.D1U.Normalized(), at.D1V.Normalized(), (at.D1U + at.D1V).Normalized() })
                        {
                            global::Kerf.Topology.Edge aLine = new MakeEdge(at.Point - (aWay * 0.05), at.Point + (aWay * 0.05)).Edge();
                            EdgeCurveAdaptor aCurve = new(aLine);
                            Assert.False(new SampleSeparation(aCurve, aFace).Apart(aCurve.FirstParameter(), aCurve.LastParameter(), u0, u1, v0, v1, 3e-7),
                                         $"apart at its middle: u {u0}..{u1} v {v0}..{v1}");
                            aTried++;
                        }
                    }
                }
            }
        }
        Assert.True(aTried > 4000, $"{aTried}");
    }

    // Unit 15Y's case: an edge 9.4 long beside a strip 0.1 wide and 10 long, 0.007 above it. Their boxes overlap down to
    // samples the size of the gap; here every ninth of the edge is told from every ninth by ninth of the strip, though not by
    // more than they are apart. Of an edge that crosses the strip all are told apart but those about the crossing
    [Fact]
    public void Apart_TellsAnEdgeBesideANarrowStripByItsNinths()
    {
        const double Gap = 0.007, Width = 0.1;
        Point3D[] aPoles = new Point3D[36];
        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                aPoles[(i * 6) + j] = At(2.0 * j, Width * i / 5, 0.002 * Math.Sin(1.3 * i) * Math.Cos(0.7 * j));
            }
        }
        BSplineSurface aStrip = new(new Grid<Point3D>(aPoles, 6, 6), [0.0, 1, 2, 3], [0.0, 1, 2, 3], [4, 1, 1, 4], [4, 1, 1, 4], 3, 3);

        int Ninths(global::Kerf.Topology.Edge theEdge, double theClearance, out bool theMiddleIsApart)
        {
            SampleSeparation aSeparation = Of(theEdge, aStrip, out EdgeCurveAdaptor aCurve, out _);
            double aFirst = aCurve.FirstParameter(), aNinth = (aCurve.LastParameter() - aFirst) / 9;
            int aCount = 0;
            for (int k = 0; k < 9; k++)
            {
                for (int i = 0; i < 9; i++)
                {
                    for (int j = 0; j < 9; j++)
                    {
                        aCount += aSeparation.Apart(aFirst + (k * aNinth), aFirst + ((k + 1) * aNinth), i / 3.0, (i + 1) / 3.0, j / 3.0,
                                                    (j + 1) / 3.0, theClearance) ? 1 : 0;
                    }
                }
            }
            theMiddleIsApart = aSeparation.Apart(aFirst + (4 * aNinth), aFirst + (5 * aNinth), 4 / 3.0, 5 / 3.0, 4 / 3.0, 5 / 3.0, theClearance);
            return aCount;
        }

        global::Kerf.Topology.Edge aBeside = new MakeEdge(At(0.3, Width / 2, Gap), At(9.7, Width / 2, Gap)).Edge();
        Assert.Equal(729, Ninths(aBeside, 3e-7, out _));
        Ninths(aBeside, 0.01, out bool isApart); // the strip's waves are 0.002 high, so the two are within 0.009 there
        Assert.False(isApart);

        global::Kerf.Topology.Edge aCrossing = new MakeEdge(At(0.3, Width / 2, Gap), At(9.7, Width / 2, -Gap)).Edge();
        Assert.InRange(Ninths(aCrossing, 3e-7, out isApart), 700, 728);
        Assert.False(isApart);
    }

    // A cone as a Bezier patch, one side its apex, and its axis. A sample by the apex, a 729th each way, bulges next to
    // nothing, but its span's second derivative is the base's, a thousand times its own: by the span's bound the axis a
    // millionth below the apex isn't told from it, by the sample's own nodes it is. The axis through the apex never is
    [Fact]
    public void Apart_BoundsASmallPieceByItsOwnNodes()
    {
        const double K = 0.5522847498;
        Point3D anApex = new(100, 100, 40);
        Point3D[] aBase = [new(130, 100, -20), new(130, 100 + (30 * K), -20), new(100 + (30 * K), 130, -20), new(100, 130, -20)];
        Point3D[] aPoles = new Point3D[8];
        for (int i = 0; i < 4; i++)
        {
            aPoles[2 * i] = aBase[i];
            aPoles[(2 * i) + 1] = anApex;
        }
        BezierSurface aCone = new(new Grid<Point3D>(aPoles, 4, 2));
        global::Kerf.Topology.Edge anAxis = new MakeEdge(new Point3D(100, 100, 0), new Point3D(100, 100, 40)).Edge();
        SampleSeparation aSeparation = Of(anAxis, aCone, out EdgeCurveAdaptor aCurve, out _);
        double aLast = aCurve.LastParameter();
        for (int i = 0; i < 729; i += 91)
        {
            double u0 = i / 729.0, u1 = (i + 1) / 729.0, v0 = 728 / 729.0;
            Assert.True(aSeparation.Apart(aLast - 0.05, aLast - 3e-6, u0, u1, v0, 1, 6e-7), $"at {i}");
            Assert.False(aSeparation.Apart(aLast - 0.05, aLast, u0, u1, v0, 1, 6e-7), $"at {i}");
            Assert.False(aSeparation.Apart(aLast - 0.05, aLast - 3e-6, u0, u1, 0, 1, 6e-7), $"at {i}"); // all the way to the base
        }
    }

    // What isn't bounded isn't called apart, however far: a surface that isn't a polynomial spline (a rational one, an
    // offset, a cylinder), a rational or a periodic curve, and a piece with a knot inside it where the surface folds
    [Fact]
    public void Apart_SaysNothingOfWhatItCannotBound()
    {
        global::Kerf.Topology.Edge aFar = new MakeEdge(At(0, 0, 50), At(7, 6, 50)).Edge();
        BSplineSurface aWavy = Wavy();
        SampleSeparation aSeparation = Of(aFar, aWavy, out EdgeCurveAdaptor aCurve, out _);
        double aFirst = aCurve.FirstParameter(), aLast = aCurve.LastParameter();
        Assert.True(aSeparation.Apart(aFirst, aLast, 0, 1, 0, 1, 1e-3));

        double[] aWeights = new double[56];
        Array.Fill(aWeights, 1.0);
        aWeights[20] = 2.0;
        BSplineSurface aRational = new(aWavy.Poles, [0.0, 1, 2, 3, 4], [0.0, 1, 2, 4, 7, 8], [4, 1, 1, 1, 4], [4, 1, 1, 1, 1, 4], 3, 3,
                                       false, false, new Grid<double>(aWeights, 7, 8));
        Assert.False(Of(aFar, aRational, out _, out _).Apart(aFirst, aLast, 0, 1, 0, 1, 1e-3));
        Assert.False(Of(aFar, new OffsetSurface(aWavy, 0.01), out _, out _).Apart(aFirst, aLast, 0, 1, 0, 1, 1e-3));
        Assert.False(Of(aFar, new CylindricalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 1.0), out _, out _)
                         .Apart(aFirst, aLast, 0, 1, 0, 1, 1e-3));

        // a fold along u = 1: a knot of the degree's multiplicity
        Point3D[] aPoles = new Point3D[28];
        for (int i = 0; i < 7; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                aPoles[(i * 4) + j] = At(j, i, Math.Abs(i - 3));
            }
        }
        BSplineSurface aFolded = new(new Grid<Point3D>(aPoles, 7, 4), [0.0, 1, 2], [0.0, 1], [4, 3, 4], [4, 4], 3, 3);
        aSeparation = Of(aFar, aFolded, out _, out _);
        Assert.True(aSeparation.Apart(aFirst, aLast, 0, 1, 0, 1, 1e-3));
        Assert.True(aSeparation.Apart(aFirst, aLast, 1, 2, 0, 1, 1e-3));
        Assert.False(aSeparation.Apart(aFirst, aLast, 0.5, 1.5, 0, 1, 1e-3));

        Point3D[] aRing = [At(0, 0, 50), At(4, 0, 50), At(4, 4, 51), At(0, 4, 50)];
        global::Kerf.Topology.Edge aPeriodic = new MakeEdge(new BSplineCurve3D(aRing, [0.0, 1, 2, 3, 4], [1, 1, 1, 1, 1], 3, true)).Edge();
        aSeparation = Of(aPeriodic, aWavy, out aCurve, out _);
        Assert.False(aSeparation.Apart(aCurve.FirstParameter(), aCurve.FirstParameter() + 0.5, 0, 1, 0, 1, 1e-3));
    }
}
