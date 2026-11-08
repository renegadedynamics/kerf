// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification.Hatching;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The hatcher by what a hatch must do (decision 1753): its domains are the stretches of the hatching line inside the
/// polygon, whichever way the line meets the boundary, and with segments kept a run along an edge belongs to them.
/// </summary>
public class Geom2dHatch_LawsTests
{
    private static Hatcher2D NewHatcher(bool theKeepPoints = true, bool theKeepSegments = false) =>
        new(new Intersector2D(1.0e-10, 1.0e-10), 1.0e-8, 1.0e-8, theKeepPoints, theKeepSegments);

    private static CurveAdaptor2D Segment(double x1, double y1, double x2, double y2)
    {
        Point2D aP1 = new(x1, y1), aP2 = new(x2, y2);
        LineCurve2D aLine = new(aP1, new Direction2D(new Vector2D(aP1, aP2)));
        return new CurveAdaptor2D(new TrimmedCurve2D(aLine, 0.0, aP1.DistanceTo(aP2)));
    }

    private static void AddPolygon(Hatcher2D theHatcher, Orientation theOr, params double[] theXY)
    {
        int aNb = theXY.Length / 2;
        for (int i = 0; i < aNb; i++)
        {
            int j = (i + 1) % aNb;
            theHatcher.AddElement(Segment(theXY[2 * i], theXY[2 * i + 1], theXY[2 * j], theXY[2 * j + 1]), theOr);
        }
    }

    // Parameter along a horizontal or sloping line from (x, y) is the distance from there
    private static CurveAdaptor2D Chord(double x, double y, double dx, double dy, double theLength = 100) =>
        new(new TrimmedCurve2D(new LineCurve2D(new Point2D(x, y), new Direction2D(dx, dy)), 0.0, theLength));

    private static (double First, double Second)[] Domains(Hatcher2D theHatcher, int theIndH)
    {
        theHatcher.Trim(theIndH);
        Assert.True(theHatcher.TrimDone(theIndH));
        theHatcher.ComputeDomains(theIndH);
        Assert.True(theHatcher.IsDone(theIndH));
        var aResult = new (double, double)[theHatcher.NbDomains(theIndH)];
        for (int i = 1; i <= aResult.Length; i++)
        {
            HatchingDomain aDomain = theHatcher.Domain(theIndH, i);
            aResult[i - 1] = (aDomain.HasFirstPoint ? aDomain.FirstPoint().Parameter() : double.NegativeInfinity,
                              aDomain.HasSecondPoint ? aDomain.SecondPoint().Parameter() : double.PositiveInfinity);
        }
        return aResult;
    }

    private static void AssertDomains((double First, double Second)[] theDomains, params double[] theEnds)
    {
        Assert.Equal(theEnds.Length / 2, theDomains.Length);
        for (int i = 0; i < theDomains.Length; i++)
        {
            Assert.Equal(theEnds[2 * i], theDomains[i].First, 1e-6);
            Assert.Equal(theEnds[2 * i + 1], theDomains[i].Second, 1e-6);
        }
    }

    [Fact]
    public void Each_stretch_inside_a_comb_is_a_domain()
    {
        // a comb with its teeth down: the line y = 1 crosses all three
        Hatcher2D aHatcher = NewHatcher();
        AddPolygon(aHatcher, Forward, 0, 2, 0, 0, 1, 0, 1, 1.5, 2, 1.5, 2, 0, 3, 0, 3, 1.5, 4, 1.5, 4, 0, 5, 0, 5, 2);
        int aH = aHatcher.AddHatching(Chord(-1, 1, 1, 0));
        AssertDomains(Domains(aHatcher, aH), 1, 2, 3, 4, 5, 6);
    }

    [Fact]
    public void A_hatching_through_a_corner_of_a_diamond_touches_and_stays_out()
    {
        Hatcher2D aHatcher = NewHatcher();
        AddPolygon(aHatcher, Forward, 0, -1, 1, 0, 0, 1, -1, 0);
        // along y = 1: touches at the top corner, so its only point is a domain of no length
        int aTouching = aHatcher.AddHatching(Chord(-5, 1, 1, 0));
        AssertDomains(Domains(aHatcher, aTouching), 5, 5);
        // through the left and right corners: inside between them
        int aThrough = aHatcher.AddHatching(Chord(-5, 0, 1, 0));
        AssertDomains(Domains(aHatcher, aThrough), 4, 6);
    }

    [Fact]
    public void A_hatching_along_an_edge_with_segments_kept_has_that_edge_as_its_domain()
    {
        Hatcher2D aHatcher = NewHatcher(true, true);
        AddPolygon(aHatcher, Forward, 0, 0, 1, 0, 1, 1, 0, 1);
        int aH = aHatcher.AddHatching(Chord(-1, 0, 1, 0));
        AssertDomains(Domains(aHatcher, aH), 1, 2);
    }

    [Fact]
    public void A_hatching_along_an_edge_without_segments_has_no_domain()
    {
        Hatcher2D aHatcher = NewHatcher(true, false);
        AddPolygon(aHatcher, Forward, 0, 0, 1, 0, 1, 1, 0, 1);
        int aH = aHatcher.AddHatching(Chord(-1, 0, 1, 0));
        Assert.Empty(Domains(aHatcher, aH));
    }

    [Fact]
    public void A_hatching_along_an_edge_of_a_notch_passes_through_its_length()
    {
        // an L: the line y = 1 runs in from the left along the inner edge's level and out at x = 1
        Hatcher2D aHatcher = NewHatcher(true, true);
        AddPolygon(aHatcher, Forward, 0, 0, 2, 0, 2, 1, 1, 1, 1, 2, 0, 2);
        int aH = aHatcher.AddHatching(Chord(-1, 1, 1, 0));
        (double First, double Second)[] aDomains = Domains(aHatcher, aH);
        Assert.NotEmpty(aDomains);
        Assert.Equal(1, aDomains[0].First, 1e-6);
        Assert.Equal(3, aDomains[^1].Second, 1e-6);
    }

    [Fact]
    public void Segments_with_opposite_elements_cancel_out()
    {
        // two squares sharing an edge, each with its own copy: the line along the shared edge
        Hatcher2D aHatcher = NewHatcher(true, true);
        AddPolygon(aHatcher, Forward, 0, 0, 1, 0, 1, 1, 0, 1);
        AddPolygon(aHatcher, Forward, 0, -1, 1, -1, 1, 0, 0, 0);
        int aH = aHatcher.AddHatching(Chord(-1, 0, 1, 0));
        (double First, double Second)[] aDomains = Domains(aHatcher, aH);
        Assert.Single(aDomains);
        Assert.Equal(1, aDomains[0].First, 1e-6);
        Assert.Equal(2, aDomains[0].Second, 1e-6);
    }

    [Fact]
    public void A_hatching_ending_inside_opens_a_domain_it_cannot_close()
    {
        Hatcher2D aHatcher = NewHatcher();
        AddPolygon(aHatcher, Forward, 0, 0, 10, 0, 10, 10, 0, 10);
        // from outside, stopping inside
        int anIn = aHatcher.AddHatching(new CurveAdaptor2D(new TrimmedCurve2D(new LineCurve2D(new Point2D(-1, 5), new Direction2D(1, 0)), 0, 5)));
        (double First, double Second)[] aDomains = Domains(aHatcher, anIn);
        Assert.Single(aDomains);
        Assert.Equal(1, aDomains[0].First, 1e-6);
        // from inside, leaving
        int anOut = aHatcher.AddHatching(new CurveAdaptor2D(new TrimmedCurve2D(new LineCurve2D(new Point2D(5, 5), new Direction2D(1, 0)), 0, 20)));
        aDomains = Domains(aHatcher, anOut);
        Assert.Single(aDomains);
        Assert.Equal(5, aDomains[0].Second, 1e-6);
        // wholly inside
        int aWithin = aHatcher.AddHatching(new CurveAdaptor2D(new TrimmedCurve2D(new LineCurve2D(new Point2D(2, 5), new Direction2D(1, 0)), 0, 3)));
        aDomains = Domains(aHatcher, aWithin);
        Assert.Single(aDomains);
    }

    [Fact]
    public void A_circle_tangent_to_a_hatching_touches_and_a_chord_is_inside()
    {
        Hatcher2D aHatcher = NewHatcher();
        var aCircle = new CircleCurve2D(new Circle2D(new Frame2D(new Point2D(0, 0), new Direction2D(1, 0)), 2));
        aHatcher.AddElement(new CurveAdaptor2D(aCircle), Forward);
        int aTangent = aHatcher.AddHatching(Chord(-5, 2, 1, 0));
        AssertDomains(Domains(aHatcher, aTangent), 5, 5);
        int aChord = aHatcher.AddHatching(Chord(-5, 0, 1, 0));
        AssertDomains(Domains(aHatcher, aChord), 3, 7);
    }

    private static Hatcher2D SquareWith(params (double X, Orientation Or)[] theWalls)
    {
        Hatcher2D aHatcher = NewHatcher();
        AddPolygon(aHatcher, Forward, 0, 0, 10, 0, 10, 10, 0, 10);
        foreach ((double aX, Orientation anOr) in theWalls)
        {
            aHatcher.AddElement(Segment(aX, 0, aX, 10), anOr);
        }
        return aHatcher;
    }

    [Fact]
    public void An_internal_wall_keeps_both_sides_in_and_splits_the_domain_there()
    {
        Hatcher2D aHatcher = SquareWith((5, Internal));
        int aH = aHatcher.AddHatching(Chord(-1, 5, 1, 0));
        AssertDomains(Domains(aHatcher, aH), 1, 6, 6, 11);
        Assert.Equal(Containment.In, aHatcher.Point(aH, 2).StateBefore());
        Assert.Equal(Containment.In, aHatcher.Point(aH, 2).StateAfter());
    }

    [Theory]
    [InlineData(Forward, 1, 6)] // the wall's forward side has the material on its left: stops here
    [InlineData(Reversed, 6, 11)] // and with the other side, starts here
    public void A_wall_inside_decides_which_side_is_in_by_its_orientation(Orientation theOr, double theFirst, double theSecond)
    {
        Hatcher2D aHatcher = SquareWith((5, theOr));
        int aH = aHatcher.AddHatching(Chord(-1, 5, 1, 0));
        AssertDomains(Domains(aHatcher, aH), theFirst, theSecond);
    }

    [Fact]
    public void An_external_wall_inside_has_no_coherent_parity()
    {
        Hatcher2D aHatcher = SquareWith((7, External));
        int aH = aHatcher.AddHatching(Chord(-1, 5, 1, 0));
        aHatcher.Trim(aH);
        aHatcher.ComputeDomains(aH);
        Assert.Equal(HatchingErrorStatus.IncoherentParity, aHatcher.Status(aH));
        Assert.False(aHatcher.IsDone(aH));
    }

    [Fact]
    public void The_settings_read_back_and_clear_what_they_change()
    {
        Hatcher2D aHatcher = NewHatcher();
        AddPolygon(aHatcher, Forward, 0, 0, 1, 0, 1, 1, 0, 1);
        int aH = aHatcher.AddHatching(Chord(-1, 0.5, 1, 0));
        AssertDomains(Domains(aHatcher, aH), 1, 2);
        aHatcher.Confusion2d(1e-6);
        Assert.Equal(1e-6, aHatcher.Confusion2d());
        Assert.False(aHatcher.TrimDone(aH));
        AssertDomains(Domains(aHatcher, aH), 1, 2);
        aHatcher.Confusion3d(1e-6);
        Assert.Equal(1e-6, aHatcher.Confusion3d());
        Assert.False(aHatcher.TrimDone(aH));
        AssertDomains(Domains(aHatcher, aH), 1, 2);
        aHatcher.KeepPoints(false);
        Assert.False(aHatcher.KeepPoints());
        Assert.False(aHatcher.IsDone(aH));
        aHatcher.KeepSegments(true);
        Assert.True(aHatcher.KeepSegments());
        AssertDomains(Domains(aHatcher, aH), 1, 2);
        aHatcher.ClrElements();
        Assert.False(aHatcher.TrimDone(aH));
        aHatcher.ClrHatchings();
    }

    [Fact]
    public void The_hatcher_answers_for_its_curves()
    {
        Hatcher2D aHatcher = NewHatcher();
        CurveAdaptor2D anElement = Segment(0, 0, 1, 0);
        int anE = aHatcher.AddElement(anElement);
        CurveAdaptor2D aHatch = Chord(0.5, -1, 0, 1);
        int aH = aHatcher.AddHatching(aHatch);
        Assert.Equal(1.0, aHatcher.ElementCurve(anE).LastParameter(), 12);
        Assert.Equal(100.0, aHatcher.HatchingCurve(aH).LastParameter(), 12);
    }

    [Fact]
    public void The_dump_names_what_it_holds()
    {
        Hatcher2D aHatcher = NewHatcher();
        AddPolygon(aHatcher, Forward, 0, 0, 1, 0, 1, 1, 0, 1);
        int aH = aHatcher.AddHatching(Chord(-1, 0.5, 1, 0));
        Domains(aHatcher, aH);
        var aWriter = new StringWriter();
        aHatcher.Dump(aWriter);
        Assert.Contains("Hatcher", aWriter.ToString(), StringComparison.OrdinalIgnoreCase);
        var aDefault = new Hatching2D();
        Assert.False(aDefault.IsDone());
        Assert.Equal(0, aDefault.PointCount);
    }

    [Fact]
    public void A_hatching_lying_wholly_along_an_edge_is_that_edge_with_segments_kept()
    {
        // from the edge's start to its end: the hatching begins and ends in the segment
        Hatcher2D aHatcher = NewHatcher(true, true);
        AddPolygon(aHatcher, Forward, 0, 0, 1, 0, 1, 1, 0, 1);
        int aH = aHatcher.AddHatching(Chord(0, 0, 1, 0, 1));
        AssertDomains(Domains(aHatcher, aH), 0, 1);
    }

    [Theory]
    [InlineData(0.25)]
    [InlineData(0.5)]
    public void A_hatching_starting_inside_an_edge_runs_along_it_to_its_end(double theStart)
    {
        // from inside the bottom edge, along it and on past the corner: the segment's part from the start
        Hatcher2D aHatcher = NewHatcher(true, true);
        AddPolygon(aHatcher, Forward, 0, 0, 1, 0, 1, 1, 0, 1);
        int aH = aHatcher.AddHatching(Chord(theStart, 0, 1, 0, 2));
        AssertDomains(Domains(aHatcher, aH), 0, 1 - theStart);
    }

    [Fact]
    public void An_element_added_after_a_trim_clears_it_and_the_next_trim_sees_it()
    {
        // a square of 4, then a hole of 2 in its middle: the line y = 2 had one stretch, and has two
        Hatcher2D aHatcher = NewHatcher();
        AddPolygon(aHatcher, Forward, 0, 0, 4, 0, 4, 4, 0, 4);
        int aH = aHatcher.AddHatching(Chord(-1, 2, 1, 0));
        AssertDomains(Domains(aHatcher, aH), 1, 5);
        AddPolygon(aHatcher, Reversed, 1, 1, 3, 1, 3, 3, 1, 3);
        Assert.Equal(0, aHatcher.NbPoints(aH));
        AssertDomains(Domains(aHatcher, aH), 1, 2, 4, 5);
    }
}
