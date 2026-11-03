// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using static Kerf.Engine.Intersection.TwoD.Position2D;
using static Kerf.Engine.Intersection.TwoD.Situation;
using static Kerf.Engine.Intersection.TwoD.TypeTrans;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>IntRes2d's value types and the result merging of IntRes2d_Intersection (no GTests in OCCT).</summary>
public class IntRes2dTests
{
    /// <summary>Exposes IntRes2d_Intersection's protected members, as an algorithm deriving from it would use them.</summary>
    private sealed class TestIntersection : Intersection2D
    {
        public void Done() => _done = true;

        public void DoInsert(in IntersectionPoint2D P) => Insert(P);

        public void DoAppend(in IntersectionSegment2D S) => Append(S);

        public void DoAppend(TestIntersection Other, double F1, double L1, double F2, double L2)
            => Append(Other, F1, L1, F2, L2);

        public void DoSetValues(TestIntersection Other) => SetValues(Other);

        public void DoReset() => ResetFields();
    }

    private static IntersectionPoint2D Pnt(double u1, double u2, TypeTrans t1 = TypeTrans.In,
                                                  TypeTrans t2 = TypeTrans.Out,
                                                  Position2D pos = Position2D.Middle)
        => new(new Point2D(u1, u2), u1, u2, new Transition(false, pos, t1),
               new Transition(false, pos, t2), false);

    [Fact]
    public void Transition_DefaultIsUndecidedMiddle()
    {
        var aT = new Transition();
        Assert.Equal(Position2D.Middle, aT.PositionOnCurve());
        Assert.Equal(TypeTrans.Undecided, aT.TransitionType());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.DomainErrorException>(() => aT.IsTangent);
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.DomainErrorException>(() => aT.Situation());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.DomainErrorException>(() => aT.IsOpposite);
        Assert.NotEqual(aT, default);
    }

    [Fact]
    public void Transition_InOutTouch()
    {
        var anIn = new Transition(false, Position2D.Head, TypeTrans.In);
        Assert.Equal(TypeTrans.In, anIn.TransitionType());
        Assert.False(anIn.IsTangent);
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.DomainErrorException>(() => anIn.Situation());

        var aTouch = new Transition(true, Position2D.End, Situation.Outside, true);
        Assert.Equal(TypeTrans.Touch, aTouch.TransitionType());
        Assert.Equal(Situation.Outside, aTouch.Situation());
        Assert.True(aTouch.IsOpposite);
        Assert.True(aTouch.IsTangent);

        // SetValue of IN/OUT keeps the situation and opposite flag of a previous TOUCH, as in OCCT
        aTouch.SetValue(false, Position2D.Middle, TypeTrans.Out);
        Assert.Equal(TypeTrans.Out, aTouch.TransitionType());
        aTouch.SetValue(true, Position2D.Middle, Situation.Inside, false);
        Assert.Equal(Situation.Inside, aTouch.Situation());

        var anUndecided = new Transition(Position2D.End);
        Assert.Equal(TypeTrans.Undecided, anUndecided.TransitionType());
        Assert.Equal(Position2D.Head, anUndecided.WithPosition(Position2D.Head).PositionOnCurve());
        Assert.Equal(Position2D.End, anUndecided.PositionOnCurve());
    }

    [Fact]
    public void IntersectionPoint_ReversedFlagSwapsCurves()
    {
        var aT1 = new Transition(false, Position2D.Head, TypeTrans.In);
        var aT2 = new Transition(false, Position2D.End, TypeTrans.Out);
        var aP = new IntersectionPoint2D(new Point2D(1, 2), 0.25, 0.75, aT1, aT2, false);
        Assert.Equal(0.25, aP.ParamOnFirst());
        Assert.Equal(0.75, aP.ParamOnSecond());
        Assert.Equal(aT1, aP.TransitionOfFirst());

        var aR = new IntersectionPoint2D(new Point2D(1, 2), 0.25, 0.75, aT1, aT2, true);
        Assert.Equal(0.75, aR.ParamOnFirst());
        Assert.Equal(0.25, aR.ParamOnSecond());
        Assert.Equal(aT2, aR.TransitionOfFirst());
        Assert.Equal(aT1, aR.TransitionOfSecond());

        aP.SetValues(new Point2D(1, 2), 0.25, 0.75, aT1, aT2, true);
        Assert.Equal(aR, aP);

        var aDefault = new IntersectionPoint2D();
        Assert.Equal(RealLast(), aDefault.ParamOnFirst());
        Assert.Equal(TypeTrans.Undecided, aDefault.TransitionOfFirst().TransitionType());
    }

    [Fact]
    public void IntersectionSegment_Constructors()
    {
        IntersectionPoint2D aP1 = Pnt(0.1, 0.2), aP2 = Pnt(0.3, 0.4);

        var aSeg = new IntersectionSegment2D(aP1, aP2, true, false);
        Assert.Equal(aP1, aSeg.FirstPoint());
        Assert.Equal(aP2, aSeg.LastPoint());
        // reversed and opposite: the points are swapped
        var aRev = new IntersectionSegment2D(aP1, aP2, true, true);
        Assert.Equal(aP2, aRev.FirstPoint());
        // reversed but not opposite: unchanged
        Assert.Equal(aP1, new IntersectionSegment2D(aP1, aP2, false, true).FirstPoint());

        var aHalf = new IntersectionSegment2D(aP1, true, true, true);
        Assert.False(aHalf.HasFirstPoint);
        Assert.True(aHalf.HasLastPoint);
        Assert.Equal(aP1, aHalf.LastPoint());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.DomainErrorException>(() => aHalf.FirstPoint());

        var anInfinite = new IntersectionSegment2D(false);
        Assert.False(anInfinite.HasFirstPoint || anInfinite.HasLastPoint);
        Assert.Equal(new IntersectionSegment2D(), anInfinite);
    }

    [Fact]
    public void Domain_BoundsAndClosure()
    {
        var anInfinite = new TwoDDomain();
        Assert.False(anInfinite.HasFirstPoint);
        Assert.False(anInfinite.HasLastPoint);
        Assert.Equal(anInfinite, default);
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.DomainErrorException>(() => anInfinite.FirstParameter());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.DomainErrorException>(() => anInfinite.WithEquivalentParameters(0, 1));

        var aBounded = new TwoDDomain(new Point2D(1, 0), 0, 1e-7, new Point2D(1, 0), 2 * Math.PI, 1e-7);
        Assert.True(aBounded.HasFirstPoint && aBounded.HasLastPoint);
        Assert.False(aBounded.IsClosed);
        var aClosed = aBounded.WithEquivalentParameters(0, 2 * Math.PI);
        Assert.True(aClosed.IsClosed);
        aClosed.EquivalentParameters(out double aZero, out double aPeriod);
        Assert.Equal(0, aZero);
        Assert.Equal(2 * Math.PI, aPeriod);

        // semi-infinite, with infinite values limited to Precision::Infinite()
        var aSemi = new TwoDDomain(new Point2D(1e300, -1e300), -1e300, 1e-7, false);
        Assert.False(aSemi.HasFirstPoint);
        Assert.Equal(-global::Kerf.Engine.Foundation.Precision.Infinite(), aSemi.LastParameter());
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Infinite(), aSemi.LastPoint().X);
        Assert.Equal(-global::Kerf.Engine.Foundation.Precision.Infinite(), aSemi.LastPoint().Y);
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.DomainErrorException>(() => aSemi.FirstPoint());
    }

    [Fact]
    public void Intersection_NotDoneThrows()
    {
        var anInter = new TestIntersection();
        Assert.False(anInter.IsDone);
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.NotDoneException>(() => anInter.PointCount);
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.NotDoneException>(() => anInter.IsEmpty);
        anInter.Done();
        Assert.True(anInter.IsEmpty);
    }

    [Fact]
    public void Intersection_InsertSortsAndSkipsDuplicates()
    {
        var anInter = new TestIntersection();
        anInter.DoInsert(Pnt(0.5, 1));
        anInter.DoInsert(Pnt(0.2, 2));
        anInter.DoInsert(Pnt(0.9, 3));
        anInter.DoInsert(Pnt(0.5 + 1e-9, 1 + 1e-9)); // equal within 1e-8, same transitions: skipped
        anInter.DoInsert(Pnt(0.5, 1, TypeTrans.Out, TypeTrans.In)); // other transitions: kept, before the equal one
        anInter.Done();

        Assert.Equal(4, anInter.PointCount);
        Assert.Equal(0.2, anInter.Point(1).ParamOnFirst());
        Assert.Equal(TypeTrans.Out, anInter.Point(2).TransitionOfFirst().TransitionType());
        Assert.Equal(TypeTrans.In, anInter.Point(3).TransitionOfFirst().TransitionType());
        Assert.Equal(0.9, anInter.Point(4).ParamOnFirst());

        var aCopy = new TestIntersection();
        aCopy.DoSetValues(anInter);
        Assert.Equal(4, aCopy.PointCount);
        anInter.DoReset();
        Assert.False(anInter.IsDone);
        Assert.Equal(4, aCopy.PointCount);
    }

    [Fact]
    public void Intersection_AppendMergesSegmentsAndFixesPositions()
    {
        // Two parts of composite curves: the first meets on [0, 1] x [0, 1], the second on [1, 2] x [1, 2].
        var aPart1 = new TestIntersection();
        aPart1.DoAppend(new IntersectionSegment2D(Pnt(0, 0, pos: Position2D.Head), Pnt(1, 1, pos: Position2D.End),
                                                         false, false));
        aPart1.Done();
        var aPart2 = new TestIntersection();
        aPart2.DoAppend(new IntersectionSegment2D(Pnt(1, 1, pos: Position2D.Head), Pnt(2, 2, pos: Position2D.End),
                                                         false, false));
        // a point at an end of the part but inside the composite curves: its positions become Middle
        aPart2.DoInsert(Pnt(1.5, 3, pos: Position2D.Head));
        aPart2.Done();

        var aResult = new TestIntersection();
        aResult.DoAppend(aPart1, 0, 3, 0, 4);
        aResult.DoAppend(aPart2, 0, 3, 0, 4);

        Assert.True(aResult.IsDone);
        Assert.Equal(1, aResult.SegmentCount);
        Assert.Equal(0, aResult.Segment(1).FirstPoint().ParamOnFirst());
        Assert.Equal(2, aResult.Segment(1).LastPoint().ParamOnFirst());
        Assert.Equal(1, aResult.PointCount);
        Assert.Equal(Position2D.Middle, aResult.Point(1).TransitionOfFirst().PositionOnCurve());
        Assert.Equal(Position2D.Middle, aResult.Point(1).TransitionOfSecond().PositionOnCurve());
        // the segment's ends are at the composite curves' first parameters, so they keep their positions
        Assert.Equal(Position2D.Head, aResult.Segment(1).FirstPoint().TransitionOfFirst().PositionOnCurve());
    }

    [Fact]
    public void Intersection_AppendRemovesPointsInsideMergedSegment()
    {
        var aPart1 = new TestIntersection();
        aPart1.DoInsert(Pnt(1.5, 1.5));
        aPart1.DoAppend(new IntersectionSegment2D(Pnt(0, 0), Pnt(1, 1), false, false));
        aPart1.Done();
        var aPart2 = new TestIntersection();
        aPart2.DoAppend(new IntersectionSegment2D(Pnt(1, 1), Pnt(2, 2), false, false));
        aPart2.Done();

        var aResult = new TestIntersection();
        aResult.DoAppend(aPart1, 0, 2, 0, 2);
        Assert.Equal(1, aResult.PointCount);
        aResult.DoAppend(aPart2, 0, 2, 0, 2);
        Assert.Equal(1, aResult.SegmentCount);
        Assert.Equal(0, aResult.PointCount);
    }
}
