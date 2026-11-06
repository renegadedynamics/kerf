// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Kernel.ModelingData.Tests;
using Kerf.Topology;
using static Kerf.Engine.Healing.Extend.ExtendStatus;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// ShapeFix_Wire's gap fixes (<c>FixGap3d</c>, <c>FixGap2d</c>) over every kind of curve, in both orientations, by range
/// and by conversion. Two edges meet at a right angle with a gap between their curves. What must hold whichever way the
/// fix goes: the caller's wire doesn't change; a fix never widens the gap, and a conversion closes it; the far end of
/// each edge doesn't move; and a fix by range only moves the ends along the same curve.
/// </summary>
public class ShapeFix_WireGapTests
{
    private const double Conf = 1e-7;
    private const double Gap = 1e-3;

    public enum Kind
    {
        Line, Circle, Ellipse, Parabola, Hyperbola, BSpline, BSpline1, Bezier, TrimmedLine, OffsetLine, TrimmedBSpline,
    }

    public static TheoryData<Kind> Kinds()
    {
        var aData = new TheoryData<Kind>();
        foreach (Kind k in Enum.GetValues<Kind>())
        {
            aData.Add(k);
        }
        return aData;
    }

    // A curve through theP with direction theT there, on the plane: ending there if theArrive, else starting there
    private static (Curve3D C, double F, double L) Make3d(Kind theKind, Point3D theP, double theTx, double theTy, bool theArrive)
    {
        var aT = new Direction3D(theTx, theTy, 0);
        var aX = new Direction3D(theTy, -theTx, 0);
        var aZ = new Direction3D(0, 0, 1);
        Point3D Along(double aS, double aO = 0) =>
            new(theP.X + aS * theTx + aO * theTy, theP.Y + aS * theTy - aO * theTx, 0);
        double aSign = theArrive ? -1 : 1;
        (double F, double L) Range(double aLen) => theArrive ? (-aLen, 0) : (0, aLen);
        switch (theKind)
        {
            case Kind.Line:
            {
                (double f, double l) = Range(10);
                return (new LineCurve3D(theP, aT), f, l);
            }
            case Kind.TrimmedLine:
            {
                (double f, double l) = Range(10);
                return (new TrimmedCurve3D(new LineCurve3D(theP, aT), f, l), f, l);
            }
            case Kind.OffsetLine:
            {
                (double f, double l) = Range(10);
                return (new OffsetCurve3D(new LineCurve3D(Along(0, -2), aT), 2, aZ), f, l);
            }
            case Kind.Circle:
            {
                var aFrame = new Frame3D(Along(0, -5), aZ, aX);
                return theArrive ? (new CircleCurve3D(aFrame, 5), 1.5 * Math.PI, 2 * Math.PI) : (new CircleCurve3D(aFrame, 5), 0, Math.PI / 2);
            }
            case Kind.Ellipse:
            {
                var aFrame = new Frame3D(Along(0, -6), aZ, aX);
                return theArrive ? (new EllipseCurve3D(aFrame, 6, 3), 1.5 * Math.PI, 2 * Math.PI) : (new EllipseCurve3D(aFrame, 6, 3), 0, Math.PI / 2);
            }
            case Kind.Parabola:
            {
                (double f, double l) = Range(4);
                return (new ParabolaCurve3D(new Frame3D(theP, aZ, aX), 2), f, l);
            }
            case Kind.Hyperbola:
            {
                (double f, double l) = Range(1);
                return (new HyperbolaCurve3D(new Frame3D(Along(0, -5), aZ, aX), 5, 3), f, l);
            }
            case Kind.Bezier:
            {
                return (new BezierCurve3D(new[] { Along(aSign * 10, 1), Along(aSign * 5, 2), theP }.OrderedFor(theArrive)), 0, 1);
            }
            case Kind.BSpline1:
            {
                Point3D[] aPoles = new[] { Along(aSign * 10, 2), Along(aSign * 5, 1), theP }.OrderedFor(theArrive);
                return (new BSplineCurve3D(aPoles, [0.0, 1.0, 2.0], [2, 1, 2], 1), 0, 2);
            }
            case Kind.TrimmedBSpline:
            {
                Point3D[] aPoles = new[] { Along(aSign * 10, 1), Along(aSign * 7, 2), Along(aSign * 3, 1), theP }.OrderedFor(theArrive);
                return (new TrimmedCurve3D(new BSplineCurve3D(aPoles, [0.0, 1.0], [4, 4], 3), 0, 1), 0, 1);
            }
            default:
            {
                Point3D[] aPoles = new[] { Along(aSign * 10, 1), Along(aSign * 7, 2), Along(aSign * 3, 1), theP }.OrderedFor(theArrive);
                return (new BSplineCurve3D(aPoles, [0.0, 1.0], [4, 4], 3), 0, 1);
            }
        }
    }

    private static (Curve2D C, double F, double L) Make2d(Kind theKind, Point2D theP, double theTx, double theTy, bool theArrive)
    {
        var aT = new Direction2D(theTx, theTy);
        var aX = new Direction2D(theTy, -theTx);
        Point2D Along(double aS, double aO = 0) =>
            new(theP.X + aS * theTx + aO * theTy, theP.Y + aS * theTy - aO * theTx);
        double aSign = theArrive ? -1 : 1;
        (double F, double L) Range(double aLen) => theArrive ? (-aLen, 0) : (0, aLen);
        switch (theKind)
        {
            case Kind.Line:
            {
                (double f, double l) = Range(10);
                return (new LineCurve2D(theP, aT), f, l);
            }
            case Kind.TrimmedLine:
            {
                (double f, double l) = Range(10);
                return (new TrimmedCurve2D(new LineCurve2D(theP, aT), f, l), f, l);
            }
            case Kind.OffsetLine:
            {
                (double f, double l) = Range(10);
                return (new OffsetCurve2D(new LineCurve2D(Along(0, -2), aT), 2), f, l);
            }
            case Kind.Circle:
            {
                var anAxis = new Axis2D(Along(0, -5), aX);
                return theArrive ? (new CircleCurve2D(anAxis, 5), 1.5 * Math.PI, 2 * Math.PI) : (new CircleCurve2D(anAxis, 5), 0, Math.PI / 2);
            }
            case Kind.Ellipse:
            {
                var anAxis = new Axis2D(Along(0, -6), aX);
                return theArrive ? (new EllipseCurve2D(anAxis, 6, 3), 1.5 * Math.PI, 2 * Math.PI) : (new EllipseCurve2D(anAxis, 6, 3), 0, Math.PI / 2);
            }
            case Kind.Parabola:
            {
                (double f, double l) = Range(4);
                return (new ParabolaCurve2D(new Axis2D(theP, aX), 2), f, l);
            }
            case Kind.Hyperbola:
            {
                (double f, double l) = Range(1);
                return (new HyperbolaCurve2D(new Axis2D(Along(0, -5), aX), 5, 3), f, l);
            }
            case Kind.Bezier:
            {
                return (new BezierCurve2D(new[] { Along(aSign * 10, 1), Along(aSign * 5, 2), theP }.OrderedFor(theArrive)), 0, 1);
            }
            case Kind.BSpline1:
            {
                Point2D[] aPoles = new[] { Along(aSign * 10, 2), Along(aSign * 5, 1), theP }.OrderedFor(theArrive);
                return (new BSplineCurve2D(aPoles, [0.0, 1.0, 2.0], [2, 1, 2], 1), 0, 2);
            }
            case Kind.TrimmedBSpline:
            {
                Point2D[] aPoles = new[] { Along(aSign * 10, 1), Along(aSign * 7, 2), Along(aSign * 3, 1), theP }.OrderedFor(theArrive);
                return (new TrimmedCurve2D(new BSplineCurve2D(aPoles, [0.0, 1.0], [4, 4], 3), 0, 1), 0, 1);
            }
            default:
            {
                Point2D[] aPoles = new[] { Along(aSign * 10, 1), Along(aSign * 7, 2), Along(aSign * 3, 1), theP }.OrderedFor(theArrive);
                return (new BSplineCurve2D(aPoles, [0.0, 1.0], [4, 4], 3), 0, 1);
            }
        }
    }

    private static Wire MakeWire(params Edge[] theEdges)
    {
        var aB = new BrepBuilder();
        Wire aW = new();
        aB.MakeWire(ref aW);
        foreach (Edge anE in theEdges)
        {
            aB.Add(aW, anE);
        }
        return aW;
    }

    private static List<Edge> Edges(in Shape theShape)
    {
        var aList = new List<Edge>();
        for (var anIt = new Iterator(theShape); anIt.More(); anIt.Next())
        {
            aList.Add(ShapeCasts.Edge(anIt.Value()));
        }
        return aList;
    }

    private static Vertex TolerantVertex(in Point3D theP)
    {
        var aB = new BrepBuilder();
        Vertex aV = new();
        aB.MakeVertex(ref aV, theP, 1e-2);
        return aV;
    }

    private static Edge Oriented(Curve3D theC, double theF, double theL, in Vertex theStart, in Vertex theEnd, bool theReversed) =>
        theReversed
            ? ShapeCasts.Edge(TestShapes.Edge(theC, theF, theL, theEnd, theStart).Reversed())
            : TestShapes.Edge(theC, theF, theL, theStart, theEnd);

    // the distance from a point to the curve, over its range and well beyond
    private static double DistanceToCarrier(in Point3D theP, Curve3D theC, double theF, double theL)
    {
        if (theC is TrimmedCurve3D aTrimmed)
        {
            theC = aTrimmed.BasisCurve;
        }
        bool aPeriodic = CurveAnalysis.IsPeriodic(theC);
        double aF = aPeriodic ? theC.FirstParameter : Math.Max(theC.FirstParameter, theF - 2 * (theL - theF));
        double aL = aPeriodic ? theC.LastParameter : Math.Min(theC.LastParameter, theL + 2 * (theL - theF));
        var aProj = new ProjectPointOnCurve();
        aProj.Init(theP, theC, aF, aL);
        double aBest = Math.Min(theC.Value(aF).DistanceTo(theP), theC.Value(aL).DistanceTo(theP));
        return aProj.NbPoints() == 0 ? aBest : Math.Min(aBest, aProj.LowerDistance());
    }

    private static double DistanceToCarrier(in Point2D theP, Curve2D theC, double theF, double theL)
    {
        if (theC is TrimmedCurve2D aTrimmed)
        {
            theC = aTrimmed.BasisCurve;
        }
        bool aPeriodic = CurveAnalysis.IsPeriodic(theC);
        double aF = aPeriodic ? theC.FirstParameter : Math.Max(theC.FirstParameter, theF - 2 * (theL - theF));
        double aL = aPeriodic ? theC.LastParameter : Math.Min(theC.LastParameter, theL + 2 * (theL - theF));
        var aProj = new ProjectPointOnCurve2D();
        aProj.Init(theP, theC, aF, aL);
        double aBest = Math.Min(theC.Value(aF).DistanceTo(theP), theC.Value(aL).DistanceTo(theP));
        return aProj.NbPoints() == 0 ? aBest : Math.Min(aBest, aProj.LowerDistance());
    }

    // One pair of 3d curves through FixGap3d, its laws checked. True if it fixed the gap.
    private static bool Gap3d(Kind theKind1, Kind theKind2, bool theRev1, bool theRev2, bool theConvert, bool theShared)
    {
        var aCorner1 = new Point3D(10, 0, 0);
        var aCorner2 = new Point3D(10, Gap, 0);
        // a reversed edge runs along its curve backwards: it leaves theP against its direction of travel
        (Curve3D C1, double F1, double L1) = theRev1 ? Make3d(theKind1, aCorner1, -1, 0, false) : Make3d(theKind1, aCorner1, 1, 0, true);
        (Curve3D C2, double F2, double L2) = theRev2 ? Make3d(theKind2, aCorner2, 0, -1, true) : Make3d(theKind2, aCorner2, 0, 1, false);
        Point3D aFar1 = C1.Value(theRev1 ? L1 : F1), aFar2 = C2.Value(theRev2 ? F2 : L2);
        Vertex aV1 = TolerantVertex(aCorner1), aV2 = TolerantVertex(aCorner2);
        if (theShared)
        {
            aV1 = aV2 = TolerantVertex(new Point3D(10, Gap / 2, 0));
        }
        Edge anE1 = Oriented(C1, F1, L1, TestShapes.Vertex(aFar1), aV1, theRev1);
        Edge anE2 = Oriented(C2, F2, L2, aV2, TestShapes.Vertex(aFar2), theRev2);
        Wire aW = TestShapes.Frozen(MakeWire(anE1, anE2));
        var aSnap = ShapeSnapshot.Of(aW);

        var sfw = new WireFixer(aW, new Face(), 1e-5);
        bool aDone = sfw.FixGap3d(2, theConvert);
        Wire aRes = sfw.Wire();
        aSnap.AssertUnchanged(aW);

        List<Edge> aResEdges = Edges(aRes);
        Assert.Equal(2, aResEdges.Count);
        var sae = new EdgeAnalysis();
        Assert.True(sae.Curve3d(aResEdges[0], out Curve3D? aN1, out double aNf1, out double aNl1));
        Assert.True(sae.Curve3d(aResEdges[1], out Curve3D? aN2, out double aNf2, out double aNl2));
        Point3D anEnd1 = aN1!.Value(aNl1), aStart2 = aN2!.Value(aNf2);
        double aGap = anEnd1.DistanceTo(aStart2);
        string aCase = $"{theKind1} {theKind2} rev {theRev1} {theRev2} convert {theConvert} shared {theShared}";
        Assert.Equal(aDone, sfw.LastFixStatus(DONE1));
        if (!aDone)
        {
            Assert.True(Math.Abs(aGap - Gap) < 1e-9, aCase);
            return false;
        }
        Assert.True(aGap <= Gap + 1e-9, $"{aCase}: gap {aGap}");
        // the far ends stay: exactly on the curve's own range, or as the conversion approximates it
        Assert.True(aN1.Value(aNf1).DistanceTo(aFar1) < 1e-4, $"{aCase}: far end 1 moved to {aN1.Value(aNf1)}");
        Assert.True(aN2.Value(aNl2).DistanceTo(aFar2) < 1e-4, $"{aCase}: far end 2 moved to {aN2.Value(aNl2)}");
        if (theConvert)
        {
            // the end poles are moved to one point
            Assert.True(aGap < 1e-9, $"{aCase}: gap {aGap}");
            Assert.True(anEnd1.DistanceTo(new Point3D(10, Gap / 2, 0)) < 1e-9, aCase);
            Assert.IsType<BSplineCurve3D>(aN1);
            Assert.IsType<BSplineCurve3D>(aN2);
        }
        else
        {
            Assert.True(DistanceToCarrier(anEnd1, C1, F1, L1) < 1e-6, $"{aCase}: left curve 1");
            Assert.True(DistanceToCarrier(aStart2, C2, F2, L2) < 1e-6, $"{aCase}: left curve 2");
        }
        return true;
    }

    private static Face PlaneFace(Surface? theSurface = null)
    {
        var aB = new BrepBuilder();
        Face aF = new();
        aB.MakeFace(ref aF, theSurface ?? new PlaneSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0))), Conf);
        return TestShapes.Frozen(aF);
    }

    // An edge with a straight 3d curve between the pcurve's ends, on the face, for the wire's orientation
    private static Edge OnFace(in Face theFace, Curve2D theC, double theF, double theL, bool theReversed)
    {
        Point2D aA = theC.Value(theF), aB2 = theC.Value(theL);
        var aP1 = new Point3D(aA.X, aA.Y, 0);
        var aP2 = new Point3D(aB2.X, aB2.Y, 0);
        Vertex aV1 = TolerantVertex(aP1), aV2 = TolerantVertex(aP2);
        Edge anE = TestShapes.Edge(aV1, aV2);
        var aB = new BrepBuilder();
        aB.UpdateEdge(anE, theC, theFace, 1e-2);
        aB.Range(anE, theFace, theF, theL);
        return theReversed ? ShapeCasts.Edge(anE.Reversed()) : anE;
    }

    private static bool Gap2d(Kind theKind1, Kind theKind2, bool theRev1, bool theRev2, bool theConvert, Face? theFace = null)
    {
        var aCorner1 = new Point2D(10, 0);
        var aCorner2 = new Point2D(10, Gap);
        (Curve2D C1, double F1, double L1) = theRev1 ? Make2d(theKind1, aCorner1, -1, 0, false) : Make2d(theKind1, aCorner1, 1, 0, true);
        (Curve2D C2, double F2, double L2) = theRev2 ? Make2d(theKind2, aCorner2, 0, -1, true) : Make2d(theKind2, aCorner2, 0, 1, false);
        Point2D aFar1 = C1.Value(theRev1 ? L1 : F1), aFar2 = C2.Value(theRev2 ? F2 : L2);
        Face aF = theFace ?? PlaneFace();
        Wire aW = TestShapes.Frozen(MakeWire(OnFace(aF, C1, F1, L1, theRev1), OnFace(aF, C2, F2, L2, theRev2)));
        var aSnap = ShapeSnapshot.Of(aW);

        var sfw = new WireFixer(aW, aF, 1e-5);
        bool aDone = sfw.FixGap2d(2, theConvert);
        Wire aRes = sfw.Wire();
        aSnap.AssertUnchanged(aW);

        List<Edge> aResEdges = Edges(aRes);
        Assert.Equal(2, aResEdges.Count);
        var sae = new EdgeAnalysis();
        Assert.True(sae.PCurve(aResEdges[0], aF, out Curve2D? aN1, out double aNf1, out double aNl1));
        Assert.True(sae.PCurve(aResEdges[1], aF, out Curve2D? aN2, out double aNf2, out double aNl2));
        Point2D anEnd1 = aN1!.Value(aNl1), aStart2 = aN2!.Value(aNf2);
        double aGap = anEnd1.DistanceTo(aStart2);
        string aCase = $"{theKind1} {theKind2} rev {theRev1} {theRev2} convert {theConvert}";
        Assert.Equal(aDone, sfw.LastFixStatus(DONE1));
        if (!aDone)
        {
            Assert.True(Math.Abs(aGap - Gap) < 1e-9, aCase);
            return false;
        }
        Assert.True(aGap <= Gap + 1e-9, $"{aCase}: gap {aGap}");
        Assert.True(aN1.Value(aNf1).DistanceTo(aFar1) < 1e-4, $"{aCase}: far end 1 moved to {aN1.Value(aNf1)}");
        Assert.True(aN2.Value(aNl2).DistanceTo(aFar2) < 1e-4, $"{aCase}: far end 2 moved to {aN2.Value(aNl2)}");
        if (theConvert)
        {
            Assert.True(aGap < 1e-9, $"{aCase}: gap {aGap}");
            Assert.IsType<BSplineCurve2D>(aN1);
            Assert.IsType<BSplineCurve2D>(aN2);
        }
        else
        {
            Assert.True(DistanceToCarrier(anEnd1, C1, F1, L1) < 1e-6, $"{aCase}: left curve 1");
            Assert.True(DistanceToCarrier(aStart2, C2, F2, L2) < 1e-6, $"{aCase}: left curve 2");
        }
        return true;
    }

    private static readonly bool[] Both = [false, true];

    [Theory]
    [MemberData(nameof(Kinds))]
    public void FixGap3d_KeepsItsLaws_ForEveryPairOfCurves(Kind theKind)
    {
        foreach (Kind aOther in Enum.GetValues<Kind>())
        {
            foreach (bool aRev1 in Both)
            {
                foreach (bool aRev2 in Both)
                {
                    foreach (bool aShared in Both)
                    {
                        Gap3d(theKind, aOther, aRev1, aRev2, false, aShared);
                        Gap3d(theKind, aOther, aRev1, aRev2, true, aShared);
                    }
                }
            }
        }
    }

    [Theory]
    [MemberData(nameof(Kinds))]
    public void FixGap2d_KeepsItsLaws_ForEveryPairOfCurves(Kind theKind)
    {
        foreach (Kind aOther in Enum.GetValues<Kind>())
        {
            foreach (bool aRev1 in Both)
            {
                foreach (bool aRev2 in Both)
                {
                    Gap2d(theKind, aOther, aRev1, aRev2, false);
                    Gap2d(theKind, aOther, aRev1, aRev2, true);
                }
            }
        }
    }

    [Theory]
    [MemberData(nameof(Kinds))]
    public void FixGap3d_ByConversion_ClosesTheGap_WithAnyCurve(Kind theKind)
    {
        Assert.True(Gap3d(theKind, Kind.Line, false, false, true, true));
        Assert.True(Gap3d(Kind.Line, theKind, false, false, true, true));
    }

    [Theory]
    [MemberData(nameof(Kinds))]
    public void FixGap2d_ByConversion_ClosesTheGap_WithAnyCurve(Kind theKind)
    {
        Assert.True(Gap2d(theKind, Kind.Line, false, false, true));
        Assert.True(Gap2d(Kind.Line, theKind, false, false, true));
    }

    [Fact]
    public void FixGap_ByRange_MeetsAtTheLinesIntersection()
    {
        // the two lines cross at the first one's end, so the second is cut back there and the first doesn't change
        Assert.True(Gap3d(Kind.Line, Kind.Line, false, false, false, false));
        Assert.True(Gap2d(Kind.Line, Kind.Line, false, false, false));
        Assert.True(Gap3d(Kind.Line, Kind.Line, true, true, false, false));
        Assert.True(Gap2d(Kind.Line, Kind.Line, true, true, false));
    }

    private static (Curve3D C, double F, double L) Result3d(in Edge theE)
    {
        Assert.True(new EdgeAnalysis().Curve3d(theE, out Curve3D? aC, out double aF, out double aL));
        return (aC!, aF, aL);
    }

    private static (Curve2D C, double F, double L) Result2d(in Edge theE, in Face theF)
    {
        Assert.True(new EdgeAnalysis().PCurve(theE, theF, out Curve2D? aC, out double aF, out double aL));
        return (aC!, aF, aL);
    }

    [Fact]
    public void FixGap3d_OnOneEdge_ClosesACircleByItsWholePeriod()
    {
        // an arc nearly round, the wire's only edge: by range it becomes the whole circle
        var aFrame = new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0));
        var aCircle = new CircleCurve3D(aFrame, 5);
        Vertex aV = TolerantVertex(new Point3D(5, 0, 0));
        Wire aW = TestShapes.Frozen(MakeWire(TestShapes.Edge(aCircle, 0.001, 2 * Math.PI - 0.001, aV, aV)));

        var sfw = new WireFixer(aW, new Face(), 1e-5);
        Assert.True(sfw.FixGap3d(1));
        (Curve3D aC, double aF, double aL) = Result3d(Edges(sfw.Wire())[0]);
        Assert.Equal(2 * Math.PI, aL - aF, 9);
        Assert.True(aC.Value(aF).DistanceTo(aC.Value(aL)) < 1e-9);
    }

    [Fact]
    public void FixGap3d_OnOneEdge_ByConversion_ClosesTheCurveAtOneEnd()
    {
        var aPoles = new[] { new Point3D(5, 0.01, 0), new Point3D(10, 5, 0), new Point3D(0, 10, 0), new Point3D(5, -0.01, 0) };
        var aCurve = new BSplineCurve3D(aPoles, [0.0, 1.0], [4, 4], 3);
        Vertex aV = TolerantVertex(new Point3D(5, 0, 0));
        Wire aW = TestShapes.Frozen(MakeWire(TestShapes.Edge(aCurve, 0, 1, aV, aV)));

        var sfw = new WireFixer(aW, new Face(), 1e-5);
        Assert.True(sfw.FixGap3d(1, true));
        (Curve3D aC, double aF, double aL) = Result3d(Edges(sfw.Wire())[0]);
        // as OCCT, with one curve the "second" conversion is missing, so both poles go to the curve's start, not the vertex
        Assert.True(aC.Value(aF).DistanceTo(new Point3D(5, 0.01, 0)) < 1e-9);
        Assert.True(aC.Value(aL).DistanceTo(new Point3D(5, 0.01, 0)) < 1e-9);
        // the caller's curve still has its poles
        Assert.Equal(0.01, aCurve.Value(0).Y, 12);
    }

    [Fact]
    public void FixGap2d_OnOneEdge_ClosesACircleByItsWholePeriod()
    {
        Face aF = PlaneFace();
        var aCircle = new CircleCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 5);
        Wire aW = TestShapes.Frozen(MakeWire(OnFace(aF, aCircle, 0.001, 2 * Math.PI - 0.001, false)));

        var sfw = new WireFixer(aW, aF, 1e-5);
        Assert.True(sfw.FixGap2d(1));
        (Curve2D aC, double aFirst, double aLast) = Result2d(Edges(sfw.Wire())[0], aF);
        Assert.Equal(2 * Math.PI, aLast - aFirst, 9);
        Assert.True(aC.Value(aFirst).DistanceTo(aC.Value(aLast)) < 1e-9);
    }

    [Fact]
    public void FixGap_ByConversion_OfAPartOfASpline_KeepsToThatPart()
    {
        // the edges use the middle of their splines: the converted curves are that segment, not the whole spline
        var aPoles = new[] { new Point3D(-5, 0, 0), new Point3D(0, 0, 0), new Point3D(10, 0, 0), new Point3D(15, 0, 0) };
        var aS1 = new BSplineCurve3D(aPoles, [0.0, 1.0], [4, 4], 3);
        var aPoles2 = new[] { new Point3D(10, -5, 0), new Point3D(10, 0, 0), new Point3D(10, 10, 0), new Point3D(10, 15, 0) };
        var aS2 = new BSplineCurve3D(aPoles2, [0.0, 1.0], [4, 4], 3);
        Point3D aEnd1 = aS1.Value(0.75), aStart2 = aS2.Value(0.25);
        Vertex aV = TolerantVertex(new Point3D((aEnd1.X + aStart2.X) / 2, (aEnd1.Y + aStart2.Y) / 2, 0));
        Wire aW = TestShapes.Frozen(MakeWire(
            TestShapes.Edge(aS1, 0.25, 0.75, TestShapes.Vertex(aS1.Value(0.25)), aV),
            TestShapes.Edge(aS2, 0.25, 0.75, aV, TestShapes.Vertex(aS2.Value(0.75)))));

        var sfw = new WireFixer(aW, new Face(), 1e-5);
        Assert.True(sfw.FixGap3d(2, true));
        List<Edge> aRes = Edges(sfw.Wire());
        (Curve3D aC1, double aF1, double aL1) = Result3d(aRes[0]);
        (Curve3D aC2, double aF2, double aL2) = Result3d(aRes[1]);
        Assert.True(aC1.Value(aL1).DistanceTo(aC2.Value(aF2)) < 1e-9);
        Assert.True(aC1.Value(aF1).DistanceTo(aS1.Value(0.25)) < 1e-9);
        Assert.True(aC2.Value(aL2).DistanceTo(aS2.Value(0.75)) < 1e-9);
        // the segment's own range, no longer the whole spline's end
        Assert.True(aC1.Value(aF1).DistanceTo(aS1.Value(0)) > 1);
    }

    private static Edge WithInternalVertex(Edge theE, double theParam)
    {
        var aB = new BrepBuilder();
        Vertex aV = TolerantVertex(Tool.Curve(theE, out _, out _)!.Value(theParam));
        aB.Add(theE, aV.Oriented(Orientation.Internal));
        aB.UpdateVertex(ShapeCasts.Vertex(aV.Oriented(Orientation.Internal)), theParam, theE, 1e-2);
        return theE;
    }

    private static int InternalVertices(in Edge theE)
    {
        int aCount = 0;
        for (var anIt = new Iterator(theE, false); anIt.More(); anIt.Next())
        {
            aCount += anIt.Value().Orientation == Orientation.Internal ? 1 : 0;
        }
        return aCount;
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FixGap3d_KeepsAnEdgesInternalVertices(bool theConvert)
    {
        // two edges overrunning each other, each with a vertex inside it which has to survive the edge's replacement
        Vertex aV = TolerantVertex(new Point3D(10, 0, 0));
        Edge anE1 = TestShapes.Edge(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), 0, 10.5, TestShapes.Vertex(new Point3D(0, 0, 0)), aV);
        Edge anE2 = TestShapes.Edge(new LineCurve3D(new Point3D(10, -0.5, 0), new Direction3D(0, 1, 0)), 0, 10, aV, TestShapes.Vertex(new Point3D(10, 9.5, 0)));
        Wire aW = TestShapes.Frozen(MakeWire(WithInternalVertex(anE1, 5), WithInternalVertex(anE2, 5)));

        var sfw = new WireFixer(aW, new Face(), 1e-5);
        Assert.True(sfw.FixGap3d(2, theConvert));
        List<Edge> aRes = Edges(sfw.Wire());
        Assert.Equal(1, InternalVertices(aRes[0]));
        Assert.Equal(1, InternalVertices(aRes[1]));
        (Curve3D aC1, _, double aL1) = Result3d(aRes[0]);
        (Curve3D aC2, double aF2, _) = Result3d(aRes[1]);
        Assert.True(aC1.Value(aL1).DistanceTo(aC2.Value(aF2)) < 1e-9);
        Assert.True(aC1.Value(aL1).DistanceTo(new Point3D(10, 0, 0)) < 1e-9);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FixGap2d_KeepsAnEdgesInternalVertices(bool theConvert)
    {
        Face aF = PlaneFace();
        Edge anE1 = OnFace(aF, new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0)), 0, 10.5, false);
        Edge anE2 = OnFace(aF, new LineCurve2D(new Point2D(10, -0.5), new Direction2D(0, 1)), 0, 10, false);
        Wire aW = TestShapes.Frozen(MakeWire(WithInternalVertex(anE1, 5), WithInternalVertex(anE2, 5)));

        var sfw = new WireFixer(aW, aF, 1e-5);
        Assert.True(sfw.FixGap2d(2, theConvert));
        List<Edge> aRes = Edges(sfw.Wire());
        Assert.Equal(1, InternalVertices(aRes[0]));
        Assert.Equal(1, InternalVertices(aRes[1]));
        (Curve2D aC1, _, double aL1) = Result2d(aRes[0], aF);
        (Curve2D aC2, double aF2, _) = Result2d(aRes[1], aF);
        Assert.True(aC1.Value(aL1).DistanceTo(aC2.Value(aF2)) < 1e-9);
    }

    [Fact]
    public void FixGaps_ByRangesMode_ClosesEachCorner_OfAClosedWire()
    {
        // a triangle whose lines don't quite meet at their ends: each corner is the intersection of two lines
        Point3D aA = new(0, 0, 0), aB = new(10, 0, 0), aC = new(0, 10, 0);
        double aGap = 1e-3;
        var aV = new[] { TolerantVertex(aA), TolerantVertex(aB), TolerantVertex(aC) };
        Edge Side(Point3D aFrom, Point3D aTo, Vertex aV1, Vertex aV2)
        {
            var aDir = new Direction3D(new Vector3D(aFrom, aTo));
            var aOrigin = new Point3D(aFrom.X + aGap * aDir.X, aFrom.Y + aGap * aDir.Y, 0);
            return TestShapes.Edge(new LineCurve3D(aOrigin, aDir), 0, aFrom.DistanceTo(aTo) - 2 * aGap, aV1, aV2);
        }
        Wire aW = TestShapes.Frozen(MakeWire(Side(aA, aB, aV[0], aV[1]), Side(aB, aC, aV[1], aV[2]), Side(aC, aA, aV[2], aV[0])));

        var sfw = new WireFixer(aW, new Face(), 1e-5);
        sfw.FixGapsByRangesMode() = true;
        Assert.True(sfw.FixGaps3d());
        Assert.True(sfw.StatusGaps3d(DONE1));
        List<Edge> aRes = Edges(sfw.Wire());
        Assert.Equal(3, aRes.Count);
        for (int i = 0; i < 3; i++)
        {
            (Curve3D aC1, _, double aL1) = Result3d(aRes[i]);
            (Curve3D aC2, double aF2, _) = Result3d(aRes[(i + 1) % 3]);
            Assert.True(aC1.Value(aL1).DistanceTo(aC2.Value(aF2)) < 1e-5);
        }
    }

    [Fact]
    public void FixGaps2d_ByRangesMode_ClosesEachCorner_OfAClosedWire()
    {
        Face aF = PlaneFace();
        const double G = 1e-3;
        Wire aW = TestShapes.Frozen(MakeWire(
            OnFace(aF, new LineCurve2D(new Point2D(G, 0), new Direction2D(1, 0)), 0, 10 - 2 * G, false),
            OnFace(aF, new LineCurve2D(new Point2D(10, G), new Direction2D(0, 1)), 0, 10 - 2 * G, false),
            OnFace(aF, new LineCurve2D(new Point2D(10 - G, 10), new Direction2D(-1, 0)), 0, 10 - 2 * G, false),
            OnFace(aF, new LineCurve2D(new Point2D(0, 10 - G), new Direction2D(0, -1)), 0, 10 - 2 * G, false)));

        var sfw = new WireFixer(aW, aF, 1e-5);
        sfw.FixGapsByRangesMode() = true;
        Assert.True(sfw.FixGaps2d());
        Assert.True(sfw.StatusGaps2d(DONE1));
        List<Edge> aRes = Edges(sfw.Wire());
        for (int i = 0; i < 4; i++)
        {
            (Curve2D aC1, _, double aL1) = Result2d(aRes[i], aF);
            (Curve2D aC2, double aF2, _) = Result2d(aRes[(i + 1) % 4], aF);
            Assert.True(aC1.Value(aL1).DistanceTo(aC2.Value(aF2)) < 1e-7);
        }
    }

    [Fact]
    public void FixGap3d_WithAVertexFarFromBoth_FallsBackOnTheCurvesCrossings()
    {
        // the vertex is nowhere near the corner, so its projections don't help; the arc and the chord cross twice,
        // and the fix ends both curves at one of the crossings
        var aFrame = new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0));
        var aCircle = new CircleCurve3D(aFrame, 10);
        var aLine = new LineCurve3D(new Point3D(9.9, -2, 0), new Direction3D(0, 1, 0));
        Vertex aV = TolerantVertex(new Point3D(3, 7, 3));
        Wire aW = TestShapes.Frozen(MakeWire(
            TestShapes.Edge(aCircle, 2 * Math.PI - 0.2, 2 * Math.PI + 0.2, TestShapes.Vertex(aCircle.Value(-0.2)), aV),
            TestShapes.Edge(aLine, 0, 6, aV, TestShapes.Vertex(aLine.Value(6)))));

        var sfw = new WireFixer(aW, new Face(), 1e-5);
        Assert.True(sfw.FixGap3d(2));
        List<Edge> aRes = Edges(sfw.Wire());
        (Curve3D aC1, _, double aL1) = Result3d(aRes[0]);
        (Curve3D aC2, double aF2, _) = Result3d(aRes[1]);
        Point3D aEnd = aC1.Value(aL1);
        Assert.True(aEnd.DistanceTo(aC2.Value(aF2)) < 1e-7);
        Assert.Equal(9.9, aEnd.X, 7);
        Assert.Equal(10, Math.Sqrt(aEnd.X * aEnd.X + aEnd.Y * aEnd.Y), 7);
    }

    [Fact]
    public void FixGap3d_WithSkewLines_EndsBothAtTheirClosestPoints()
    {
        // lines that don't meet: the closest points are nearer than the ends, so both ranges change
        Vertex aV = TolerantVertex(new Point3D(3, 7, 3));
        var aL1 = new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        var aL2 = new LineCurve3D(new Point3D(10, -0.5, 1e-3), new Direction3D(0, 1, 0));
        Wire aW = TestShapes.Frozen(MakeWire(
            TestShapes.Edge(aL1, 0, 10.5, TestShapes.Vertex(new Point3D(0, 0, 0)), aV),
            TestShapes.Edge(aL2, 0, 10, aV, TestShapes.Vertex(aL2.Value(10)))));

        var sfw = new WireFixer(aW, new Face(), 1e-5);
        Assert.True(sfw.FixGap3d(2));
        List<Edge> aRes = Edges(sfw.Wire());
        (Curve3D aC1, _, double aEnd1) = Result3d(aRes[0]);
        (Curve3D aC2, double aStart2, _) = Result3d(aRes[1]);
        Assert.Equal(10, aC1.Value(aEnd1).X, 7);
        Assert.Equal(0, aC2.Value(aStart2).Y, 7);
        Assert.Equal(1e-3, aC1.Value(aEnd1).DistanceTo(aC2.Value(aStart2)), 9);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void FixGap2d_WithOverlappingLines_MeetsWithinTheOverlap(bool theRev1, bool theRev2)
    {
        // collinear lines overlapping by 0.01: the intersection is a segment, and the fix meets at one of its ends
        Face aF = PlaneFace();
        var aL1 = new LineCurve2D(new Point2D(theRev1 ? 10 : 0, 0), new Direction2D(theRev1 ? -1 : 1, 0));
        var aL2 = new LineCurve2D(new Point2D(theRev2 ? 19.99 : 9.99, 0), new Direction2D(theRev2 ? -1 : 1, 0));
        Wire aW = TestShapes.Frozen(MakeWire(OnFace(aF, aL1, 0, 10, theRev1), OnFace(aF, aL2, 0, 10, theRev2)));

        var sfw = new WireFixer(aW, aF, 1e-5);
        sfw.FixGap2d(2);
        List<Edge> aRes = Edges(sfw.Wire());
        (Curve2D aC1, double aF1, double aEnd1) = Result2d(aRes[0], aF);
        (Curve2D aC2, double aStart2, double aL2End) = Result2d(aRes[1], aF);
        Assert.True(aC1.Value(aEnd1).DistanceTo(aC2.Value(aStart2)) <= 0.01 + 1e-9);
        Assert.Equal(0, aC1.Value(aEnd1).Y, 9);
        // the far ends stay
        Assert.True(aC1.Value(aF1).DistanceTo(new Point2D(0, 0)) < 1e-9);
        Assert.True(aC2.Value(aL2End).DistanceTo(new Point2D(19.99, 0)) < 1e-9);
    }

    [Fact]
    public void FixGap2d_OnABoundedSurface_StaysInsideTheBounds()
    {
        // the lines meet outside the surface's U range: the extension stops at the surface's edge
        var aPlane = new PlaneSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)));
        Face aF = PlaneFace(new TrimmedSurface(aPlane, 0, 9.95, 0, 20));
        Edge anE1 = OnFace(aF, new LineCurve2D(new Point2D(0, 0.5), new Direction2D(1, 0.01)), 0, 9.9, false);
        Edge anE2 = OnFace(aF, new LineCurve2D(new Point2D(9.9, 0.7), new Direction2D(0.01, 1)), 0, 10, false);
        Wire aW = TestShapes.Frozen(MakeWire(anE1, anE2));

        var sfw = new WireFixer(aW, aF, 1e-5);
        bool aDone = sfw.FixGap2d(2);
        List<Edge> aRes = Edges(sfw.Wire());
        (Curve2D aC1, _, double aEnd1) = Result2d(aRes[0], aF);
        (Curve2D aC2, double aStart2, _) = Result2d(aRes[1], aF);
        Assert.True(aDone);
        Assert.InRange(aC1.Value(aEnd1).X, 0, 9.95 + 1e-9);
        Assert.InRange(aC2.Value(aStart2).X, 0, 9.95 + 1e-9);
    }

    [Fact]
    public void FixGap2d_OnASeam_FailsAndChangesNothing()
    {
        // a cylinder's seam is listed twice in the wire: the fix leaves it to FixSeam
        Face aF = TestShapes.CylinderFace(5, 10);
        Wire aOuter = ShapeAnalysisAnalysis.OuterWire(aF);
        var sfw = new WireFixer(aOuter, aF, 1e-5);
        int aSeam = 0;
        for (int i = 1; i <= sfw.NbEdges(); i++)
        {
            if (sfw.WireData()!.IsSeam(i))
            {
                aSeam = i;
                break;
            }
        }
        Assert.NotEqual(0, aSeam);
        Assert.False(sfw.FixGap2d(aSeam));
        Assert.True(sfw.LastFixStatus(FAIL1));
    }

    [Fact]
    public void FixGap3d_WithoutCurves_Fails()
    {
        // an edge with no 3d curve at all
        var aB = new BrepBuilder();
        Edge aNoCurve = new();
        aB.MakeEdge(ref aNoCurve);
        Wire aW = TestShapes.Frozen(MakeWire(aNoCurve, TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(1, 0, 0))));
        var sfw = new WireFixer(aW, new Face(), 1e-5);
        Assert.False(sfw.FixGap3d(2));
        Assert.True(sfw.LastFixStatus(FAIL1));
    }

    [Fact]
    public void FixGap2d_ExtendedRanges_OnCirclesPastTheirSeam_AdjustByAPeriod()
    {
        // pcurves on a circle whose range crosses 0: the meeting point's parameter comes back into the edge's range
        Face aF = PlaneFace();
        var aCircle = new CircleCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(0, -1)), 5);
        // from just before the circle's own start, 2 pi on, to the point (5, 0), which it reaches at pi / 2
        var aLine = new LineCurve2D(new Point2D(5.001, 0), new Direction2D(0, 1));
        Wire aW = TestShapes.Frozen(MakeWire(OnFace(aF, aCircle, 2 * Math.PI - 1, 2 * Math.PI + Math.PI / 2, false), OnFace(aF, aLine, 0, 5, false)));

        var sfw = new WireFixer(aW, aF, 1e-5);
        Assert.True(sfw.FixGap2d(2));
        List<Edge> aRes = Edges(sfw.Wire());
        (Curve2D aC1, double aFirst, double aEnd1) = Result2d(aRes[0], aF);
        (Curve2D aC2, double aStart2, _) = Result2d(aRes[1], aF);
        Assert.True(aC1.Value(aEnd1).DistanceTo(aC2.Value(aStart2)) <= 1e-3 + 1e-9);
        // still the same stretch of circle, its start unmoved
        Assert.Equal(2 * Math.PI - 1, aFirst, 9);
        Assert.True(aEnd1 > aFirst);
    }

    [Theory]
    [InlineData(1, 1, false)]
    [InlineData(-1, 1, false)]
    [InlineData(1, -1, false)]
    [InlineData(-1, -1, false)]
    [InlineData(1, 1, true)]
    [InlineData(-1, 1, true)]
    [InlineData(1, -1, true)]
    [InlineData(-1, -1, true)]
    public void FixGap2d_MeetingOutsideTheSurface_CutsTheExtensionAtItsEdge(int theSignX, int theSignY, bool theSwap)
    {
        // the lines would meet at (10, 0.6), past the surface's edge at 9.95. The first is extended only as far as the
        // edge; the second crosses the edge further along, so it's left where it started. Mirrored and swapped, the
        // same case meets every side of the surface.
        Point2D At(double aX, double aY)
        {
            double aA = theSignX * aX, aB = theSignY * aY;
            return theSwap ? new Point2D(aB, aA) : new Point2D(aA, aB);
        }
        Direction2D Towards(double aX, double aY)
        {
            Point2D aO = At(0, 0), aE = At(aX, aY);
            return new Direction2D(aE.X - aO.X, aE.Y - aO.Y);
        }
        double aA1 = theSignX > 0 ? -1 : -9.95, aA2 = theSignX > 0 ? 9.95 : 1;
        double aB1 = theSignY > 0 ? -1 : -20, aB2 = theSignY > 0 ? 20 : 1;
        var aPlane = new PlaneSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)));
        Face aF = PlaneFace(theSwap ? new TrimmedSurface(aPlane, aB1, aB2, aA1, aA2) : new TrimmedSurface(aPlane, aA1, aA2, aB1, aB2));
        Edge anE1 = OnFace(aF, new LineCurve2D(At(0, 0.6), Towards(1, 0)), 0, 9.9, false);
        Point2D aStart2 = At(10.05, 0.7);
        Edge anE2 = OnFace(aF, new LineCurve2D(aStart2, Towards(-0.5, 1)), 0, 5, false);
        Wire aW = TestShapes.Frozen(MakeWire(anE1, anE2));

        var sfw = new WireFixer(aW, aF, 1e-5);
        Assert.True(sfw.FixGap2d(2));
        List<Edge> aRes = Edges(sfw.Wire());
        (Curve2D aC1, double aF1, double aEnd1) = Result2d(aRes[0], aF);
        (Curve2D aC2, double aF2, _) = Result2d(aRes[1], aF);
        Assert.True(aC1.Value(aEnd1).DistanceTo(At(9.95, 0.6)) < 1e-7, $"ended at {aC1.Value(aEnd1)}");
        Assert.True(aC1.Value(aF1).DistanceTo(At(0, 0.6)) < 1e-9);
        Assert.True(aC2.Value(aF2).DistanceTo(aStart2) < 1e-9, $"second started at {aC2.Value(aF2)}");
    }

    [Fact]
    public void FixGap2d_APolylineAlongTheSurfacesEdge_StopsWhereItLeavesIt()
    {
        // the first pcurve runs along the surface's edge, then out; the meeting point is outside. Against the edge
        // its intersection is a segment, and the extension stops at the segment's end.
        var aPlane = new PlaneSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)));
        Face aF = PlaneFace(new TrimmedSurface(aPlane, -1, 10, -1, 20));
        var aPolyline = new BSplineCurve2D(new[] { new Point2D(10, 0), new Point2D(10, 5), new Point2D(12, 7) }, [0.0, 1.0, 2.0], [2, 1, 2], 1);
        double aOnFirstEdge = 0.5;
        Edge anE1 = OnFace(aF, aPolyline, 0, 1 + aOnFirstEdge, false);
        Edge anE2 = OnFace(aF, new LineCurve2D(new Point2D(11.5, 5.8), new Direction2D(1, -1)), 0, 5, false);
        Wire aW = TestShapes.Frozen(MakeWire(anE1, anE2));

        var sfw = new WireFixer(aW, aF, 1e-5);
        sfw.FixGap2d(2);
        List<Edge> aRes = Edges(sfw.Wire());
        (Curve2D aC1, double aF1, double aEnd1) = Result2d(aRes[0], aF);
        Assert.True(aC1.Value(aF1).DistanceTo(new Point2D(10, 0)) < 1e-9);
        Assert.True(aEnd1 > aF1);
    }
}

/// <summary>The poles of a curve, given from its far end, in the order of its parameter.</summary>
file static class PoleOrder
{
    public static T[] OrderedFor<T>(this T[] thePoles, bool theArrive)
    {
        if (!theArrive)
        {
            Array.Reverse(thePoles);
        }
        return thePoles;
    }
}
