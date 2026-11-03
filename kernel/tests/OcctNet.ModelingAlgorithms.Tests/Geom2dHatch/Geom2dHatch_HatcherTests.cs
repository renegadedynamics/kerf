// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using System.Globalization;
using static Occt.HatchGen_ErrorStatus;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Geom2dHatch_Hatcher on known configurations, set up as IntTools_Context::Hatcher does (intersector tolerances
/// 1e-10, confusions 1e-8, points kept, segments not) and used as BOPTools_AlgoTools3D::PointInFace does.
/// </summary>
public class Geom2dHatch_HatcherTests
{
    private static Geom2dHatch_Hatcher NewHatcher() =>
        new(new Geom2dHatch_Intersector(1.0e-10, 1.0e-10), 1.0e-8, 1.0e-8, true, false);

    // A segment as a trimmed line, as IntTools_Context trims the pcurves.
    private static CurveAdaptor2D Segment(double x1, double y1, double x2, double y2)
    {
        Point2D aP1 = new(x1, y1), aP2 = new(x2, y2);
        LineCurve2D aLine = new(aP1, new Direction2D(new Vector2D(aP1, aP2)));
        return new CurveAdaptor2D(new TrimmedCurve2D(aLine, 0.0, aP1.Distance(aP2)));
    }

    // A counterclockwise polygon (inside on the left), its edges added in order.
    private static void AddPolygon(Geom2dHatch_Hatcher theHatcher, Orientation theOr, params double[] theXY)
    {
        int aNb = theXY.Length / 2;
        for (int i = 0; i < aNb; i++)
        {
            int j = (i + 1) % aNb;
            theHatcher.AddElement(Segment(theXY[2 * i], theXY[2 * i + 1], theXY[2 * j], theXY[2 * j + 1]), theOr);
        }
    }

    // A half-line from (x, y) in the direction (dx, dy), as BOPTools_AlgoTools3D::PointNearEdge builds them.
    private static CurveAdaptor2D HalfLine(double x, double y, double dx, double dy) =>
        new(new TrimmedCurve2D(new LineCurve2D(new Point2D(x, y), new Direction2D(dx, dy)), 0.0,
                                    global::Kerf.Engine.Foundation.Precision.Infinite()));

    private static (double First, double Second)[] Domains(Geom2dHatch_Hatcher theHatcher, int theIndH)
    {
        theHatcher.Trim();
        Assert.True(theHatcher.TrimDone(theIndH));
        Assert.False(theHatcher.TrimFailed(theIndH));
        theHatcher.ComputeDomains(theIndH);
        Assert.True(theHatcher.IsDone(theIndH));
        Assert.Equal(HatchGen_NoProblem, theHatcher.Status(theIndH));
        var aResult = new (double, double)[theHatcher.NbDomains(theIndH)];
        for (int i = 1; i <= aResult.Length; i++)
        {
            HatchGen_Domain aDomain = theHatcher.Domain(theIndH, i);
            aResult[i - 1] = (aDomain.HasFirstPoint() ? aDomain.FirstPoint().Parameter() : double.NegativeInfinity,
                              aDomain.HasSecondPoint() ? aDomain.SecondPoint().Parameter() : double.PositiveInfinity);
        }
        return aResult;
    }

    [Fact]
    public void Square_LineThroughIt()
    {
        Geom2dHatch_Hatcher aHatcher = NewHatcher();
        AddPolygon(aHatcher, Forward, 0, 0, 1, 0, 1, 1, 0, 1);
        int aIH = aHatcher.AddHatching(HalfLine(-1, 0.5, 1, 0));
        Assert.Equal(1, aIH);

        (double First, double Second)[] aDomains = Domains(aHatcher, aIH);
        Assert.Single(aDomains);
        Assert.Equal(1.0, aDomains[0].First, 1e-12);
        Assert.Equal(2.0, aDomains[0].Second, 1e-12);

        // The points: entering through the left edge (element 4), leaving through the right one (element 2)
        Assert.Equal(2, aHatcher.NbPoints(aIH));
        HatchGen_PointOnHatching aP1 = aHatcher.Point(aIH, 1), aP2 = aHatcher.Point(aIH, 2);
        Assert.Equal(Out, aP1.StateBefore());
        Assert.Equal(In, aP1.StateAfter());
        Assert.Equal(In, aP2.StateBefore());
        Assert.Equal(Out, aP2.StateAfter());
        Assert.Equal(1, aP1.NbPoints());
        Assert.Equal(4, aP1.Point(1).Index());
        Assert.Equal(2, aP2.Point(1).Index());
        Assert.Equal(HatchGen_IntersectionType.HatchGen_TRUE, aP1.Point(1).IntersectionType());
        Assert.Equal(Internal, aP1.Point(1).Position());
        Assert.Equal(0.5, aP1.Point(1).Parameter(), 1e-12); // the middle of the left edge (from (0, 1) to (0, 0))
    }

    [Fact]
    public void SquareWithHole_TwoDomains()
    {
        Geom2dHatch_Hatcher aHatcher = NewHatcher();
        AddPolygon(aHatcher, Forward, 0, 0, 4, 0, 4, 4, 0, 4);
        AddPolygon(aHatcher, Forward, 1, 1, 1, 3, 3, 3, 3, 1); // clockwise: a hole
        int aIH = aHatcher.AddHatching(HalfLine(-1, 2, 1, 0));

        (double First, double Second)[] aDomains = Domains(aHatcher, aIH);
        Assert.Equal(2, aDomains.Length);
        Assert.Equal(1.0, aDomains[0].First, 1e-12);
        Assert.Equal(2.0, aDomains[0].Second, 1e-12);
        Assert.Equal(4.0, aDomains[1].First, 1e-12);
        Assert.Equal(5.0, aDomains[1].Second, 1e-12);
    }

    [Fact]
    public void SquareWithHole_ReversedElements()
    {
        // The hole counterclockwise but its elements REVERSED: the same domains
        Geom2dHatch_Hatcher aHatcher = NewHatcher();
        AddPolygon(aHatcher, Forward, 0, 0, 4, 0, 4, 4, 0, 4);
        AddPolygon(aHatcher, Reversed, 1, 1, 3, 1, 3, 3, 1, 3);
        int aIH = aHatcher.AddHatching(HalfLine(-1, 2, 1, 0));

        (double First, double Second)[] aDomains = Domains(aHatcher, aIH);
        Assert.Equal(2, aDomains.Length);
        Assert.Equal(1.0, aDomains[0].First, 1e-12);
        Assert.Equal(2.0, aDomains[0].Second, 1e-12);
        Assert.Equal(4.0, aDomains[1].First, 1e-12);
        Assert.Equal(5.0, aDomains[1].Second, 1e-12);
    }

    [Fact]
    public void Square_DiagonalThroughVertices()
    {
        // The diagonal meets two elements at each of the corners (0, 0) and (1, 1): each point on the hatching holds
        // two points on element, at their ends, and the complex transition gives OUT/IN then IN/OUT
        Geom2dHatch_Hatcher aHatcher = NewHatcher();
        AddPolygon(aHatcher, Forward, 0, 0, 1, 0, 1, 1, 0, 1);
        double s = Math.Sqrt(0.5);
        int aIH = aHatcher.AddHatching(HalfLine(-1, -1, s, s));

        (double First, double Second)[] aDomains = Domains(aHatcher, aIH);
        Assert.Single(aDomains);
        Assert.Equal(Math.Sqrt(2.0), aDomains[0].First, 1e-9);
        Assert.Equal(2.0 * Math.Sqrt(2.0), aDomains[0].Second, 1e-9);

        Assert.Equal(2, aHatcher.NbPoints(aIH));
        HatchGen_PointOnHatching aP1 = aHatcher.Point(aIH, 1);
        Assert.Equal(2, aP1.NbPoints());
        Assert.Equal(Out, aP1.StateBefore());
        Assert.Equal(In, aP1.StateAfter());
        Assert.Equal(2, aHatcher.Point(aIH, 2).NbPoints());
    }

    [Fact]
    public void Circle_Chord()
    {
        Geom2dHatch_Hatcher aHatcher = NewHatcher();
        aHatcher.AddElement(new CircleCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 1.0));
        int aIH = aHatcher.AddHatching(HalfLine(-2, 0.6, 1, 0));

        (double First, double Second)[] aDomains = Domains(aHatcher, aIH);
        Assert.Single(aDomains);
        Assert.Equal(2.0 - 0.8, aDomains[0].First, 1e-12);
        Assert.Equal(2.0 + 0.8, aDomains[0].Second, 1e-12);
    }

    [Fact]
    public void NoIntersection_ClassifiedInside_OneInfiniteDomain()
    {
        Geom2dHatch_Hatcher aHatcher = NewHatcher();
        AddPolygon(aHatcher, Forward, 0, 0, 1, 0, 1, 1, 0, 1);
        int aIH = aHatcher.AddHatching(Segment(0.2, 0.5, 0.8, 0.5));
        (double First, double Second)[] aDomains = Domains(aHatcher, aIH);
        Assert.Single(aDomains);
        Assert.Equal(double.NegativeInfinity, aDomains[0].First);
        Assert.Equal(double.PositiveInfinity, aDomains[0].Second);
        Assert.Equal(0, aHatcher.NbPoints(aIH));
    }

    [Fact]
    public void NoIntersection_ClassifiedOutside_NoDomain()
    {
        Geom2dHatch_Hatcher aHatcher = NewHatcher();
        AddPolygon(aHatcher, Forward, 0, 0, 1, 0, 1, 1, 0, 1);
        int aIH = aHatcher.AddHatching(Segment(2, 0.5, 3, 0.5));
        Assert.Empty(Domains(aHatcher, aIH));
    }

    [Fact]
    public void Reuse_AsIntToolsContext()
    {
        // BOPTools_AlgoTools3D::PointInFace: ClrHatchings, AddHatching (index 1 again), Trim, ComputeDomains,
        // RemHatching; the elements stay
        Geom2dHatch_Hatcher aHatcher = NewHatcher();
        AddPolygon(aHatcher, Forward, 0, 0, 1, 0, 1, 1, 0, 1);
        for (int k = 0; k < 3; k++)
        {
            aHatcher.ClrHatchings();
            double y = 0.25 * (k + 1);
            int aIH = aHatcher.AddHatching(HalfLine(-1, y, 1, 0));
            Assert.Equal(1, aIH);
            (double First, double Second)[] aDomains = Domains(aHatcher, aIH);
            Assert.Single(aDomains);
            Assert.Equal(1.0, aDomains[0].First, 1e-12);
            Assert.Equal(2.0, aDomains[0].Second, 1e-12);
            aHatcher.RemHatching(aIH);
        }
    }

    [Fact]
    public void Indices_ReuseTheLowestFreeOne()
    {
        Geom2dHatch_Hatcher aHatcher = NewHatcher();
        AddPolygon(aHatcher, Forward, 0, 0, 1, 0, 1, 1);
        Assert.Equal(4, aHatcher.AddElement(Segment(0, 0, 1, 1)));
        aHatcher.RemElement(2);
        Assert.Equal(2, aHatcher.AddElement(Segment(1, 0, 1, 1)));
        aHatcher.RemElement(4);
        Assert.Equal(4, aHatcher.AddElement(Segment(0, 0, 1, 1))); // the last index was freed, then reused

        int aIH1 = aHatcher.AddHatching(HalfLine(0, 0.5, 1, 0));
        int aIH2 = aHatcher.AddHatching(HalfLine(0, 0.25, 1, 0));
        Assert.Equal((1, 2), (aIH1, aIH2));
        aHatcher.RemHatching(1);
        Assert.Equal(1, aHatcher.AddHatching(HalfLine(0, 0.75, 1, 0)));
        Assert.True(aHatcher.ElementCurve(1).Curve() is LineCurve2D); // the adaptor unwraps the trimmed curve
        Assert.Equal(0.25, aHatcher.HatchingCurve(2).Value(0.0).Y());
    }

    [Fact]
    public void RemElement_KeepsTheOtherElementsPointsOnHatching()
    {
        // OCCT's RemElement removes the element's points from a copy of each point on the hatching (occt-issues.md):
        // a point holding the removed element and another keeps both; a point holding only the removed element goes
        Geom2dHatch_Hatcher aHatcher = NewHatcher();
        AddPolygon(aHatcher, Forward, 0, 0, 1, 0, 1, 1, 0, 1);
        double s = Math.Sqrt(0.5);
        int aDiag = aHatcher.AddHatching(HalfLine(-1, -1, s, s));
        int aMid = aHatcher.AddHatching(HalfLine(-1, 0.5, 1, 0));
        aHatcher.Trim();
        aHatcher.ComputeDomains();
        Assert.Equal(2, aHatcher.Point(aDiag, 1).NbPoints());

        aHatcher.RemElement(4); // the left edge, from (0, 1) to (0, 0)

        Assert.Equal(2, aHatcher.NbPoints(aDiag));
        Assert.Equal(2, aHatcher.Point(aDiag, 1).NbPoints()); // OCCT: still holds element 4's point
        Assert.False(aHatcher.IsDone(aDiag));
        Assert.Equal(1, aHatcher.NbPoints(aMid)); // the left crossing went
        Assert.False(aHatcher.IsDone(aMid));
    }

    [Fact]
    public void Settings_ClearPointsOrDomains()
    {
        Geom2dHatch_Hatcher aHatcher = NewHatcher();
        AddPolygon(aHatcher, Forward, 0, 0, 1, 0, 1, 1, 0, 1);
        int aIH = aHatcher.AddHatching(HalfLine(-1, 0.5, 1, 0));
        Domains(aHatcher, aIH);

        aHatcher.KeepSegments(true); // clears the domains, keeps the points
        Assert.True(aHatcher.KeepSegments());
        Assert.False(aHatcher.IsDone(aIH));
        Assert.Throws<global::Kerf.Engine.Foundation.NotDoneException>(() => aHatcher.NbDomains(aIH));
        Assert.True(aHatcher.TrimDone(aIH));
        Assert.Equal(2, aHatcher.NbPoints(aIH));

        aHatcher.Confusion2d(1.0e-7); // clears the points
        Assert.Equal(1.0e-7, aHatcher.Confusion2d());
        Assert.False(aHatcher.TrimDone(aIH));
        Assert.Equal(0, aHatcher.NbPoints(aIH));

        aHatcher.ComputeDomains(aIH); // trims first
        Assert.True(aHatcher.IsDone(aIH));
        Assert.Equal(1, aHatcher.NbDomains(aIH));

        aHatcher.Intersector(new Geom2dHatch_Intersector(1.0e-9, 1.0e-9));
        Assert.Equal(1.0e-9, aHatcher.Intersector().ConfusionTolerance());
        Assert.Equal(0, aHatcher.NbPoints(aIH));

        aHatcher.Clear();
        Assert.Throws<global::Kerf.Engine.Foundation.NoSuchObjectException>(() => aHatcher.NbPoints(aIH));
    }

    [Fact]
    public void Intersector_IsCopied()
    {
        Geom2dHatch_Intersector anIntersector = new(1.0e-10, 1.0e-10);
        Geom2dHatch_Hatcher aHatcher = new(anIntersector, 1.0e-8, 1.0e-8);
        anIntersector.SetConfusionTolerance(0.5);
        Assert.Equal(1.0e-10, aHatcher.Intersector().ConfusionTolerance());
        Assert.False(aHatcher.KeepPoints());
        Assert.False(aHatcher.KeepSegments());
        Assert.Equal(1.0e-8, aHatcher.Confusion3d());
    }

    [Fact]
    public void Dump_WritesTheHatcher()
    {
        Geom2dHatch_Hatcher aHatcher = NewHatcher();
        AddPolygon(aHatcher, Forward, 0, 0, 1, 0, 1, 1, 0, 1);
        int aIH = aHatcher.AddHatching(HalfLine(-1, 0.5, 1, 0));
        Domains(aHatcher, aIH);
        using StringWriter aWriter = new(CultureInfo.InvariantCulture);
        aHatcher.Dump(aWriter);
        string aText = aWriter.ToString();
        Assert.Contains("=== Dump of the hatcher", aText);
        Assert.Contains("Hatching # 1 contains 2 restriction points :", aText);
        Assert.Contains("--- Point on hatching #   1 ", aText);
        Assert.Contains("    State After           = IN", aText);
        Assert.Contains("Element # 4 is FORWARD", aText);
        Assert.Contains("4 elements", aText);
        Assert.Contains("=== Domain ", aHatcher.Domain(aIH, 1).ToString());
    }

    [Fact]
    public void Classifier_OnElements()
    {
        Geom2dHatch_Elements anElements = new();
        anElements.Bind(1, new Geom2dHatch_Element(Segment(0, 0, 1, 0)));
        anElements.Bind(2, new Geom2dHatch_Element(Segment(1, 0, 1, 1)));
        anElements.Bind(3, new Geom2dHatch_Element(Segment(1, 1, 0, 1)));
        anElements.Bind(4, new Geom2dHatch_Element(Segment(0, 1, 0, 0)));

        Assert.Equal(In, new Geom2dHatch_Classifier(anElements, new Point2D(0.3, 0.6), 1.0e-7).State());
        Assert.Equal(Out, new Geom2dHatch_Classifier(anElements, new Point2D(1.3, 0.6), 1.0e-7).State());
        Geom2dHatch_Classifier anOn = new(anElements, new Point2D(1.0, 0.4), 1.0e-7);
        Assert.Equal(On, anOn.State());
        Assert.Equal(0.4, anOn.EdgeParameter(), 1e-9);
        Assert.Equal(1.0, anOn.Edge().Value(0.0).X());
        Assert.False(anOn.Rejected());
        Assert.False(anOn.NoWires());

        Assert.Equal(In, new Geom2dHatch_Classifier(new Geom2dHatch_Elements(), new Point2D(), 1.0e-7).State());
    }

    [Fact]
    public void Elements_CopyConstructorCopiesNothing()
    {
        // OCCT's "magic constructor" (occt-issues.md)
        Geom2dHatch_Elements anElements = new();
        anElements.Bind(1, new Geom2dHatch_Element(Segment(0, 0, 1, 0)));
        Geom2dHatch_Elements aCopy = new(anElements);
        Assert.False(aCopy.IsBound(1));
        Assert.True(anElements.IsBound(1));
    }

    [Fact]
    public void Element_CopiesTheAdaptor()
    {
        CurveAdaptor2D aCurve = Segment(0, 0, 1, 0);
        Geom2dHatch_Element anElement = new(aCurve, Reversed);
        aCurve.Load(new LineCurve2D(new Point2D(5, 5), new Direction2D(0, 1)));
        Assert.Equal(0.0, anElement.Curve().Value(0.0).X());
        Assert.Equal(Reversed, anElement.Orientation());
        Geom2dHatch_Element aForward = anElement.WithOrientation(Forward);
        Assert.Equal(Forward, aForward.Orientation());
        Assert.Equal(Reversed, anElement.Orientation());
        Assert.Same(anElement.Curve(), aForward.Curve());
        Assert.NotEqual(anElement, aForward);
        Assert.Equal(anElement, aForward.WithOrientation(Reversed));
    }
}
