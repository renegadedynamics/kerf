// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification.Hatching;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The hatcher's parts by what they must do (decision 1753): an element is its curve and orientation, a probe from a
/// point reaches the elements however far they run, a hatching's points stay in order, and the classifier finds a
/// point inside a square to be in it.
/// </summary>
public class Geom2dHatch_PartsLawsTests
{
    private static readonly LineCurve2D XAxis = new(new Point2D(0, 0), new Direction2D(1, 0));

    private static CurveAdaptor2D Segment(double x1, double y1, double x2, double y2)
    {
        Point2D aP1 = new(x1, y1), aP2 = new(x2, y2);
        LineCurve2D aLine = new(aP1, new Direction2D(new Vector2D(aP1, aP2)));
        return new CurveAdaptor2D(new TrimmedCurve2D(aLine, 0.0, aP1.DistanceTo(aP2)));
    }

    [Fact]
    public void An_element_is_its_curve_and_its_orientation()
    {
        var aCurve = Segment(0, 0, 1, 0);
        var anElement = new Element2D(aCurve, Forward);
        Element2D aSame = anElement;
        Element2D aReversed = anElement.WithOrientation(Reversed);
        Assert.True(anElement == aSame);
        Assert.True(anElement.Equals((object)aSame));
        Assert.Equal(anElement.GetHashCode(), aSame.GetHashCode());
        Assert.True(anElement != aReversed);
        Assert.Equal(Reversed, aReversed.Orientation());
        Assert.StartsWith("Geom2dHatch_Element(", aReversed.ToString());
        Assert.EndsWith(", Reversed)", aReversed.ToString());
        // another element on an equal curve is another element: the curve is held, not compared
        Assert.True(anElement != new Element2D(Segment(0, 0, 1, 0), Forward));

        var anEmpty = new Element2D();
        Assert.Equal(Forward, anEmpty.Orientation());
        Assert.Equal("Geom2dHatch_Element(no curve, Forward)", anEmpty.ToString());
    }

    [Theory]
    [InlineData(-1.0e101, 1.0e101, -1.0, 1.0)] // a whole line: probed between -1 and 1
    [InlineData(-1.0e101, 3.0, 2.0, 3.0)] // from minus infinity to 3: in the last unit
    [InlineData(2.0, 1.0e101, 2.0, 3.0)] // from 2 to infinity: in the first unit
    public void A_probe_from_a_point_reaches_an_element_that_runs_to_infinity(double theFirst, double theLast, double theFrom, double theTo)
    {
        var anElements = new Elements2D();
        anElements.Bind(1, new Element2D(new CurveAdaptor2D(XAxis, theFirst, theLast), Forward));
        Assert.True(anElements.IsBound(1));
        Assert.Equal(Forward, anElements[1].Orientation());
        Point2D aP = new(0.5, 5);
        Assert.True(anElements.Segment(aP, out Line2D aLine, out double aPar));
        // the half-line starts at the point and meets the axis at its parameter, within the stretch probed
        Assert.Equal(0, aLine.Location.DistanceTo(aP), 15);
        Point2D aHit = new(aP.X + aPar * aLine.Direction.X, aP.Y + aPar * aLine.Direction.Y);
        Assert.Equal(0, aHit.Y, 9);
        Assert.InRange(aHit.X, theFrom - 1.0e-9, theTo + 1.0e-9);

        anElements.Clear();
        Assert.False(anElements.IsBound(1));
    }

    [Fact]
    public void A_hatching_keeps_its_points_in_order_and_its_domains_until_a_point_is_added()
    {
        var aHatching = new Hatching2D(Segment(0, 0, 10, 0));
        foreach (double aParameter in new[] { 7.0, 2.0, 5.0 })
        {
            var aPoint = new PointOnHatching();
            aPoint.SetParameter(aParameter);
            aHatching.AddPoint(aPoint, 1.0e-9);
        }
        Assert.Equal(3, aHatching.PointCount);
        Assert.Equal([2.0, 5.0, 7.0], Enumerable.Range(1, 3).Select(i => aHatching.Point(i).Parameter()));

        aHatching.AddDomain(new HatchingDomain(aHatching.Point(1), aHatching.Point(2)));
        aHatching.AddDomain(new HatchingDomain(aHatching.Point(2), aHatching.Point(3)));
        aHatching.RemDomain(1);
        Assert.Equal(1, aHatching.DomainCount);
        Assert.Equal(5.0, aHatching.Domain(1).FirstPoint().Parameter());
        aHatching.IsDone(true);
        var aMore = new PointOnHatching();
        aMore.SetParameter(9.0);
        aHatching.AddPoint(aMore, 1.0e-9);
        // a new point makes the domains out of date: they're gone
        Assert.Equal(0, aHatching.DomainCount);
        Assert.False(aHatching.IsDone());

        Assert.Equal(HatchingErrorStatus.NoProblem, aHatching.Status());
        aHatching.TrimFailed(true);
        Assert.True(aHatching.TrimFailed());
        Assert.Equal(HatchingErrorStatus.TrimFailure, aHatching.Status());
    }

    [Fact]
    public void A_hatching_is_classified_at_its_end_or_where_it_starts_or_at_the_origin_of_a_whole_line()
    {
        Assert.Equal(0, new Hatching2D(Segment(1, 1, 4, 5)).ClassificationPoint().DistanceTo(new Point2D(4, 5)), 12);
        Assert.Equal(0, new Hatching2D(new CurveAdaptor2D(XAxis, 2.0, 1.0e101)).ClassificationPoint().DistanceTo(new Point2D(2, 0)), 12);
        Assert.Equal(0, new Hatching2D(new CurveAdaptor2D(XAxis)).ClassificationPoint().DistanceTo(new Point2D(0, 0)), 12);
    }

    [Fact]
    public void A_line_without_an_end_meets_a_crossing_segment_where_it_crosses()
    {
        var anIntersector = new Intersector2D();
        Assert.Equal(0, anIntersector.TangencyTolerance());
        anIntersector.SetConfusionTolerance(1.0e-9);
        anIntersector.SetTangencyTolerance(1.0e-9);
        Assert.Equal(1.0e-9, anIntersector.ConfusionTolerance());
        Assert.Equal(1.0e-9, anIntersector.TangencyTolerance());

        // the half-line from (1, 1) along X with no end meets the segment x = 6 at 5 from its start
        anIntersector.Perform(new Line2D(new Point2D(1, 1), new Direction2D(1, 0)), double.MaxValue, 1.0e-9, Segment(6, -3, 6, 3));
        Assert.True(anIntersector.IsDone);
        Assert.Equal(1, anIntersector.PointCount);
        Assert.Equal(5.0, anIntersector.Point(1).ParamOnFirst(), 9);
        Assert.Equal(0, anIntersector.Point(1).Value().DistanceTo(new Point2D(6, 1)), 9);
        // bounded to 4, it doesn't reach
        anIntersector.Perform(new Line2D(new Point2D(1, 1), new Direction2D(1, 0)), 4.0, 1.0e-9, Segment(6, -3, 6, 3));
        Assert.Equal(0, anIntersector.PointCount);
    }

    [Fact]
    public void A_point_inside_a_square_is_in_it_and_one_outside_is_out()
    {
        var anElements = new Elements2D();
        anElements.Bind(1, new Element2D(Segment(0, 0, 10, 0), Forward));
        anElements.Bind(2, new Element2D(Segment(10, 0, 10, 10), Forward));
        anElements.Bind(3, new Element2D(Segment(10, 10, 0, 10), Forward));
        anElements.Bind(4, new Element2D(Segment(0, 10, 0, 0), Forward));
        var aClassifier = new Classifier2D();
        Assert.True(aClassifier.NoWires());
        Assert.False(aClassifier.Rejected());
        aClassifier.Perform(anElements, new Point2D(3, 4), 1.0e-9);
        Assert.Equal(Containment.In, aClassifier.State());
        aClassifier.Perform(anElements, new Point2D(13, 4), 1.0e-9);
        Assert.Equal(Containment.Out, aClassifier.State());
        Assert.Equal(Containment.In, new Classifier2D(anElements, new Point2D(9.5, 0.5), 1.0e-9).State());
    }
}
