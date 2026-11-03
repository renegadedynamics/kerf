// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Occt.IntRes2d_Position;
using static Occt.IntRes2d_Situation;
using static Occt.IntRes2d_TypeTrans;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>IntRes2d's value types and the result merging of IntRes2d_Intersection (no GTests in OCCT).</summary>
public class IntRes2dTests
{
    /// <summary>Exposes IntRes2d_Intersection's protected members, as an algorithm deriving from it would use them.</summary>
    private sealed class TestIntersection : IntRes2d_Intersection
    {
        public void Done() => done = true;

        public void DoInsert(in IntRes2d_IntersectionPoint P) => Insert(P);

        public void DoAppend(in IntRes2d_IntersectionSegment S) => Append(S);

        public void DoAppend(TestIntersection Other, double F1, double L1, double F2, double L2)
            => Append(Other, F1, L1, F2, L2);

        public void DoSetValues(TestIntersection Other) => SetValues(Other);

        public void DoReset() => ResetFields();
    }

    private static IntRes2d_IntersectionPoint Pnt(double u1, double u2, IntRes2d_TypeTrans t1 = IntRes2d_In,
                                                  IntRes2d_TypeTrans t2 = IntRes2d_Out,
                                                  IntRes2d_Position pos = IntRes2d_Middle)
        => new(new Point2D(u1, u2), u1, u2, new IntRes2d_Transition(false, pos, t1),
               new IntRes2d_Transition(false, pos, t2), false);

    [Fact]
    public void Transition_DefaultIsUndecidedMiddle()
    {
        var aT = new IntRes2d_Transition();
        Assert.Equal(IntRes2d_Middle, aT.PositionOnCurve());
        Assert.Equal(IntRes2d_Undecided, aT.TransitionType());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.DomainErrorException>(() => aT.IsTangent());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.DomainErrorException>(() => aT.Situation());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.DomainErrorException>(() => aT.IsOpposite());
        Assert.NotEqual(aT, default);
    }

    [Fact]
    public void Transition_InOutTouch()
    {
        var anIn = new IntRes2d_Transition(false, IntRes2d_Head, IntRes2d_In);
        Assert.Equal(IntRes2d_In, anIn.TransitionType());
        Assert.False(anIn.IsTangent());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.DomainErrorException>(() => anIn.Situation());

        var aTouch = new IntRes2d_Transition(true, IntRes2d_End, IntRes2d_Outside, true);
        Assert.Equal(IntRes2d_Touch, aTouch.TransitionType());
        Assert.Equal(IntRes2d_Outside, aTouch.Situation());
        Assert.True(aTouch.IsOpposite());
        Assert.True(aTouch.IsTangent());

        // SetValue of IN/OUT keeps the situation and opposite flag of a previous TOUCH, as in OCCT
        aTouch.SetValue(false, IntRes2d_Middle, IntRes2d_Out);
        Assert.Equal(IntRes2d_Out, aTouch.TransitionType());
        aTouch.SetValue(true, IntRes2d_Middle, IntRes2d_Inside, false);
        Assert.Equal(IntRes2d_Inside, aTouch.Situation());

        var anUndecided = new IntRes2d_Transition(IntRes2d_End);
        Assert.Equal(IntRes2d_Undecided, anUndecided.TransitionType());
        Assert.Equal(IntRes2d_Head, anUndecided.WithPosition(IntRes2d_Head).PositionOnCurve());
        Assert.Equal(IntRes2d_End, anUndecided.PositionOnCurve());
    }

    [Fact]
    public void IntersectionPoint_ReversedFlagSwapsCurves()
    {
        var aT1 = new IntRes2d_Transition(false, IntRes2d_Head, IntRes2d_In);
        var aT2 = new IntRes2d_Transition(false, IntRes2d_End, IntRes2d_Out);
        var aP = new IntRes2d_IntersectionPoint(new Point2D(1, 2), 0.25, 0.75, aT1, aT2, false);
        Assert.Equal(0.25, aP.ParamOnFirst());
        Assert.Equal(0.75, aP.ParamOnSecond());
        Assert.Equal(aT1, aP.TransitionOfFirst());

        var aR = new IntRes2d_IntersectionPoint(new Point2D(1, 2), 0.25, 0.75, aT1, aT2, true);
        Assert.Equal(0.75, aR.ParamOnFirst());
        Assert.Equal(0.25, aR.ParamOnSecond());
        Assert.Equal(aT2, aR.TransitionOfFirst());
        Assert.Equal(aT1, aR.TransitionOfSecond());

        aP.SetValues(new Point2D(1, 2), 0.25, 0.75, aT1, aT2, true);
        Assert.Equal(aR, aP);

        var aDefault = new IntRes2d_IntersectionPoint();
        Assert.Equal(RealLast(), aDefault.ParamOnFirst());
        Assert.Equal(IntRes2d_Undecided, aDefault.TransitionOfFirst().TransitionType());
    }

    [Fact]
    public void IntersectionSegment_Constructors()
    {
        IntRes2d_IntersectionPoint aP1 = Pnt(0.1, 0.2), aP2 = Pnt(0.3, 0.4);

        var aSeg = new IntRes2d_IntersectionSegment(aP1, aP2, true, false);
        Assert.Equal(aP1, aSeg.FirstPoint());
        Assert.Equal(aP2, aSeg.LastPoint());
        // reversed and opposite: the points are swapped
        var aRev = new IntRes2d_IntersectionSegment(aP1, aP2, true, true);
        Assert.Equal(aP2, aRev.FirstPoint());
        // reversed but not opposite: unchanged
        Assert.Equal(aP1, new IntRes2d_IntersectionSegment(aP1, aP2, false, true).FirstPoint());

        var aHalf = new IntRes2d_IntersectionSegment(aP1, true, true, true);
        Assert.False(aHalf.HasFirstPoint());
        Assert.True(aHalf.HasLastPoint());
        Assert.Equal(aP1, aHalf.LastPoint());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.DomainErrorException>(() => aHalf.FirstPoint());

        var anInfinite = new IntRes2d_IntersectionSegment(false);
        Assert.False(anInfinite.HasFirstPoint() || anInfinite.HasLastPoint());
        Assert.Equal(new IntRes2d_IntersectionSegment(), anInfinite);
    }

    [Fact]
    public void Domain_BoundsAndClosure()
    {
        var anInfinite = new IntRes2d_Domain();
        Assert.False(anInfinite.HasFirstPoint());
        Assert.False(anInfinite.HasLastPoint());
        Assert.Equal(anInfinite, default);
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.DomainErrorException>(() => anInfinite.FirstParameter());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.DomainErrorException>(() => anInfinite.WithEquivalentParameters(0, 1));

        var aBounded = new IntRes2d_Domain(new Point2D(1, 0), 0, 1e-7, new Point2D(1, 0), 2 * Math.PI, 1e-7);
        Assert.True(aBounded.HasFirstPoint() && aBounded.HasLastPoint());
        Assert.False(aBounded.IsClosed());
        var aClosed = aBounded.WithEquivalentParameters(0, 2 * Math.PI);
        Assert.True(aClosed.IsClosed());
        aClosed.EquivalentParameters(out double aZero, out double aPeriod);
        Assert.Equal(0, aZero);
        Assert.Equal(2 * Math.PI, aPeriod);

        // semi-infinite, with infinite values limited to Precision::Infinite()
        var aSemi = new IntRes2d_Domain(new Point2D(1e300, -1e300), -1e300, 1e-7, false);
        Assert.False(aSemi.HasFirstPoint());
        Assert.Equal(-global::Kerf.Engine.Foundation.Precision.Infinite(), aSemi.LastParameter());
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Infinite(), aSemi.LastPoint().X());
        Assert.Equal(-global::Kerf.Engine.Foundation.Precision.Infinite(), aSemi.LastPoint().Y());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.DomainErrorException>(() => aSemi.FirstPoint());
    }

    [Fact]
    public void Intersection_NotDoneThrows()
    {
        var anInter = new TestIntersection();
        Assert.False(anInter.IsDone());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.NotDoneException>(() => anInter.NbPoints());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.NotDoneException>(() => anInter.IsEmpty());
        anInter.Done();
        Assert.True(anInter.IsEmpty());
    }

    [Fact]
    public void Intersection_InsertSortsAndSkipsDuplicates()
    {
        var anInter = new TestIntersection();
        anInter.DoInsert(Pnt(0.5, 1));
        anInter.DoInsert(Pnt(0.2, 2));
        anInter.DoInsert(Pnt(0.9, 3));
        anInter.DoInsert(Pnt(0.5 + 1e-9, 1 + 1e-9)); // equal within 1e-8, same transitions: skipped
        anInter.DoInsert(Pnt(0.5, 1, IntRes2d_Out, IntRes2d_In)); // other transitions: kept, before the equal one
        anInter.Done();

        Assert.Equal(4, anInter.NbPoints());
        Assert.Equal(0.2, anInter.Point(1).ParamOnFirst());
        Assert.Equal(IntRes2d_Out, anInter.Point(2).TransitionOfFirst().TransitionType());
        Assert.Equal(IntRes2d_In, anInter.Point(3).TransitionOfFirst().TransitionType());
        Assert.Equal(0.9, anInter.Point(4).ParamOnFirst());

        var aCopy = new TestIntersection();
        aCopy.DoSetValues(anInter);
        Assert.Equal(4, aCopy.NbPoints());
        anInter.DoReset();
        Assert.False(anInter.IsDone());
        Assert.Equal(4, aCopy.NbPoints());
    }

    [Fact]
    public void Intersection_AppendMergesSegmentsAndFixesPositions()
    {
        // Two parts of composite curves: the first meets on [0, 1] x [0, 1], the second on [1, 2] x [1, 2].
        var aPart1 = new TestIntersection();
        aPart1.DoAppend(new IntRes2d_IntersectionSegment(Pnt(0, 0, pos: IntRes2d_Head), Pnt(1, 1, pos: IntRes2d_End),
                                                         false, false));
        aPart1.Done();
        var aPart2 = new TestIntersection();
        aPart2.DoAppend(new IntRes2d_IntersectionSegment(Pnt(1, 1, pos: IntRes2d_Head), Pnt(2, 2, pos: IntRes2d_End),
                                                         false, false));
        // a point at an end of the part but inside the composite curves: its positions become Middle
        aPart2.DoInsert(Pnt(1.5, 3, pos: IntRes2d_Head));
        aPart2.Done();

        var aResult = new TestIntersection();
        aResult.DoAppend(aPart1, 0, 3, 0, 4);
        aResult.DoAppend(aPart2, 0, 3, 0, 4);

        Assert.True(aResult.IsDone());
        Assert.Equal(1, aResult.NbSegments());
        Assert.Equal(0, aResult.Segment(1).FirstPoint().ParamOnFirst());
        Assert.Equal(2, aResult.Segment(1).LastPoint().ParamOnFirst());
        Assert.Equal(1, aResult.NbPoints());
        Assert.Equal(IntRes2d_Middle, aResult.Point(1).TransitionOfFirst().PositionOnCurve());
        Assert.Equal(IntRes2d_Middle, aResult.Point(1).TransitionOfSecond().PositionOnCurve());
        // the segment's ends are at the composite curves' first parameters, so they keep their positions
        Assert.Equal(IntRes2d_Head, aResult.Segment(1).FirstPoint().TransitionOfFirst().PositionOnCurve());
    }

    [Fact]
    public void Intersection_AppendRemovesPointsInsideMergedSegment()
    {
        var aPart1 = new TestIntersection();
        aPart1.DoInsert(Pnt(1.5, 1.5));
        aPart1.DoAppend(new IntRes2d_IntersectionSegment(Pnt(0, 0), Pnt(1, 1), false, false));
        aPart1.Done();
        var aPart2 = new TestIntersection();
        aPart2.DoAppend(new IntRes2d_IntersectionSegment(Pnt(1, 1), Pnt(2, 2), false, false));
        aPart2.Done();

        var aResult = new TestIntersection();
        aResult.DoAppend(aPart1, 0, 2, 0, 2);
        Assert.Equal(1, aResult.NbPoints());
        aResult.DoAppend(aPart2, 0, 2, 0, 2);
        Assert.Equal(1, aResult.NbSegments());
        Assert.Equal(0, aResult.NbPoints());
    }
}
