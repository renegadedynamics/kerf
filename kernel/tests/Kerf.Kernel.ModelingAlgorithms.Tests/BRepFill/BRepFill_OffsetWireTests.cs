// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Filling;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Measurement;
using Kerf.Engine.Offsets.Api;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of BRepFill_OffsetWire and BRepOffsetAPI_MakeOffset (unit 5O2): exact lengths of parallels of squares,
/// holed faces, circles and open wires with both join types; what is published is frozen and what isn't stays
/// unfrozen; the caller's frozen face and wires are never written; repeated and reused runs give the same results.
/// OCCT V8_0_1 has no GTests for these classes; BRepOffsetAPIMakeOffsetOracleTests compares MakeOffset with native.
/// </summary>
public class BRepFill_OffsetWireTests
{
    private static Wire Polygon(bool theClosed, params double[] theXY)
    {
        MakePolygon aMP = new();
        for (int i = 0; i < theXY.Length; i += 2)
        {
            aMP.Add(new Point3D(theXY[i], theXY[i + 1], 0));
        }
        if (theClosed)
        {
            aMP.Close();
        }
        return aMP.Wire();
    }

    private static Wire Square(double theSize = 10) => Polygon(true, 0, 0, theSize, 0, theSize, theSize, 0, theSize);

    private static Face Frozen(in Face theFace)
    {
        theFace.TShape()!.FreezeGraph();
        return theFace;
    }

    private static string Text(in Shape theShape)
    {
        using StringWriter aWriter = new();
        BrepTools.Write(theShape, aWriter);
        return aWriter.ToString();
    }

    private static double Length(in Shape theShape)
    {
        Properties aProps = new();
        MassPropertiesBuilder.LinearProperties(theShape, aProps);
        return aProps.Mass();
    }

    private static int Count(in Shape theShape, ShapeType theType)
    {
        IndexedSet<Shape> aMap = new(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, theType, aMap);
        return aMap.Count;
    }

    // BRepFill_OffsetWire offsets to the left of the face's wires: inside (MakeOffset reverses them for outside)
    [Theory]
    [InlineData(global::Kerf.Geometry.JoinKind.Arc, 1.0, 32.0)]
    [InlineData(global::Kerf.Geometry.JoinKind.Intersection, 1.0, 32.0)]
    [InlineData(global::Kerf.Geometry.JoinKind.Arc, 2.5, 20.0)]
    public void Parallel_inside_a_square(global::Kerf.Geometry.JoinKind theJoin, double theOffset, double theLength)
    {
        Face aFace = Frozen(new MakeFace(Square(), true).Face());
        string aText = Text(aFace);

        OffsetWire anOW = new(aFace, theJoin);
        anOW.Perform(theOffset);
        Assert.True(anOW.IsDone);
        Assert.Equal(theJoin, anOW.JoinType());
        Assert.False(anOW.InternalShape().TShape()!.IsFrozen);
        Shape aResult = anOW.Shape();
        Assert.True(aResult.TShape()!.IsFrozen);
        Assert.Equal(ShapeType.Wire, aResult.Type);
        Assert.True(aResult.ClosedFlag);
        Assert.Equal(theLength, Length(aResult), 1.0e-9);
        Assert.True(anOW.Spine().TShape()!.IsFrozen);

        // every edge of the square generates one parallel edge of the result
        for (Explorer anExp = new(aFace, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            IReadOnlyList<Shape> aGenerated = anOW.GeneratedShapes(anExp.Current());
            Assert.Single(aGenerated);
            Assert.Equal(ShapeType.Edge, aGenerated[0].Type);
            Assert.True(aGenerated[0].TShape()!.IsFrozen);
        }
        Assert.Equal(aText, Text(aFace));
    }

    [Fact]
    public void Parallel_inside_a_holed_face_and_too_far_inside()
    {
        MakeFace aMaker = new(Square(20), true);
        aMaker.Add(ShapeCasts.Wire(Polygon(true, 8, 8, 12, 8, 12, 12, 8, 12).Reversed()));
        Face aFace = Frozen(aMaker.Face());
        string aText = Text(aFace);

        // inward from both wires: a square 16 wide and one around the hole, rounded
        MakeWireOffset anOffset = new(aFace);
        anOffset.Perform(-2.0);
        Assert.True(anOffset.IsDone);
        Shape aResult = anOffset.Shape();
        Assert.True(aResult.TShape()!.IsFrozen);
        Assert.Equal(2, Count(aResult, ShapeType.Wire));
        Assert.Equal(64 + 16 + 4 * Math.PI, Length(aResult), 1.0e-9);

        // too far: the parallels vanish, an empty result that is done (as natively)
        anOffset.Perform(-5.0);
        Assert.True(anOffset.IsDone);
        Assert.Equal(0, Count(anOffset.Shape(), ShapeType.Edge));
        Assert.Equal(aText, Text(aFace));
    }

    [Fact]
    public void Circle_is_offset_directly()
    {
        Edge aCircle = new MakeEdge(new Circle3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 5)).Edge();
        Face aFace = Frozen(new MakeFace(new MakeWire(aCircle).Wire(), true).Face());
        OffsetWire anOW = new(aFace);
        anOW.Perform(2.0);
        Assert.True(anOW.IsDone);
        Assert.Equal(2 * Math.PI * 3, Length(anOW.Shape()), 1.0e-9);
        anOW.Perform(-2.0);
        Assert.Equal(2 * Math.PI * 7, Length(anOW.Shape()), 1.0e-9);
    }

    [Fact]
    public void Zero_offset_gives_the_spine_at_the_altitude()
    {
        Face aFace = Frozen(new MakeFace(Square(), true).Face());
        OffsetWire anOW = new(aFace);
        anOW.Perform(0.0, 3.0);
        Assert.True(anOW.IsDone);
        Assert.Equal(40.0, Length(anOW.Shape()), 1.0e-9);
        Box aBox = new();
        Bounds.Add(anOW.Shape(), ref aBox);
        aBox.Get(out _, out _, out double aZMin, out _, out _, out double aZMax);
        Assert.Equal(3.0, aZMin, 1.0e-6);
        Assert.Equal(3.0, aZMax, 1.0e-6);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Open_wire_closed_around_or_open(bool theIsOpenResult)
    {
        Wire aWire = Polygon(false, 0, 0, 10, 0, 10, 10);
        aWire.TShape()!.FreezeGraph();
        string aText = Text(aWire);
        MakeWireOffset anOffset = new(aWire, global::Kerf.Geometry.JoinKind.Arc, theIsOpenResult);
        anOffset.Perform(1.0);
        Assert.True(anOffset.IsDone);
        Shape aResult = anOffset.Shape();
        Assert.Equal(1, Count(aResult, ShapeType.Wire));
        if (theIsOpenResult)
        {
            // one open parallel, on the side of the positive offset: 10 + 10 and a quarter circle (as natively)
            Assert.False(aResult.ClosedFlag);
            Assert.Equal(20 + Math.PI / 2, Length(aResult), 1.0e-9);
        }
        else
        {
            // one closed wire around it: 10 + 10 and a quarter circle outside, 9 + 9 inside, half circles at the ends
            Assert.Equal(38 + Math.PI / 2 + 2 * Math.PI, Length(aResult), 1.0e-9);
        }
        Assert.Equal(aText, Text(aWire));
    }

    [Fact]
    public void Reused_maker_equals_fresh_makers_on_both_sides()
    {
        Wire aWire = Polygon(true, 0, 0, 10, 0, 10, 4, 4, 4, 4, 10, 0, 10);
        aWire.TShape()!.FreezeGraph();
        MakeWireOffset aReused = new(aWire, global::Kerf.Geometry.JoinKind.Intersection);
        foreach (double anOffset in new[] { 1.0, -1.0, 2.0, -0.5 })
        {
            aReused.Perform(anOffset);
            MakeWireOffset aFresh = new(aWire, global::Kerf.Geometry.JoinKind.Intersection);
            aFresh.Perform(anOffset);
            Assert.Equal(Length(aFresh.Shape()), Length(aReused.Shape()), 1.0e-12);
        }

        // repeated runs write the same text
        MakeWireOffset aFirst = new(aWire);
        aFirst.Perform(1.5);
        MakeWireOffset aSecond = new(aWire);
        aSecond.Perform(1.5);
        Assert.Equal(Text(aFirst.Shape()), Text(aSecond.Shape()));
    }

    [Fact]
    public void Generated_lists_the_parallels_of_the_spine_edges()
    {
        Face aFace = Frozen(new MakeFace(Square(), true).Face());
        MakeWireOffset anOffset = new(aFace);
        anOffset.Perform(1.0);
        Shape aResult = anOffset.Shape();
        IndexedSet<Shape> aResultEdges = new(ShapeHasher.Instance);
        Exploration.MapShapes(aResult, ShapeType.Edge, aResultEdges);
        for (Explorer anExp = new(aFace, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            IReadOnlyList<Shape> aGenerated = anOffset.Generated(anExp.Current());
            Assert.Single(aGenerated);
            Assert.True(aResultEdges.Contains(aGenerated[0]));
        }
        // the arcs come from the vertices
        for (Explorer anExp = new(aFace, ShapeType.Vertex); anExp.More(); anExp.Next())
        {
            Assert.NotEmpty(anOffset.Generated(anExp.Current()));
        }
    }

    [Fact]
    public void Approximation_and_ConvertFace()
    {
        Edge anEllipse = new MakeEdge(new Ellipse3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10, 5)).Edge();
        Face aFace = Frozen(new MakeFace(new MakeWire(anEllipse).Wire(), true).Face());
        string aText = Text(aFace);

        Face aConverted = MakeWireOffset.ConvertFace(aFace, 0.1);
        Assert.True(aConverted.TShape()!.IsFrozen);
        for (Explorer anExp = new(aConverted, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            CurveKind aType = new EdgeCurveAdaptor(ShapeCasts.Edge(anExp.Current())).GetType();
            Assert.True(aType is CurveKind.Line or CurveKind.Circle, $"{aType}");
        }

        MakeWireOffset anOffset = new(aFace);
        anOffset.SetApprox(true);
        anOffset.Perform(1.0);
        Assert.True(anOffset.IsDone);
        Assert.True(Length(anOffset.Shape()) > Length(aFace));
        Assert.Equal(aText, Text(aFace));
    }

    [Fact]
    public void Perform_without_a_spine_throws()
    {
        EngineAssert.Throws<NotDoneException>(() => new MakeWireOffset().Perform(1.0));
        // a non-planar wire: not done
        MakePolygon aMP = new(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(1, 1, 1), new Point3D(0, 1, 0), true);
        MakeWireOffset anOffset = new(aMP.Wire());
        anOffset.Perform(0.1);
        Assert.False(anOffset.IsDone);
    }

    [Fact]
    public void A_finer_polyline_whose_offset_has_an_edge_the_wrong_way_round_falls_back_to_OCCTs()
    {
        // tests/offset/wire_closed_inside_0_075/D4: at the distance where it closes up, the polyline at 0.001 of the distance
        // gave a closed loop of three edges, one the wrong way round (a face on it not closed and unorientable), as a
        // success; OCCT's polyline closes up, as natively (decisions 1980 and 1984)
        string aPath = Path.Combine(typeof(BRepFill_OffsetWireTests).Assembly.GetCustomAttributes(typeof(System.Reflection.AssemblyMetadataAttribute), false)
                                        .Cast<System.Reflection.AssemblyMetadataAttribute>().Single(a => a.Key == "OcctTestDataDir").Value!, "brep", "offset_wire_044.brep");
        if (!File.Exists(aPath) || !BrepTools.Read(out Shape aShape, aPath))
        {
            return;
        }
        MakeWireOffset anOffset = new(ShapeCasts.Wire(aShape));
        anOffset.Perform(-0.075 * 32.074);
        Assert.True(anOffset.IsDone);
        Assert.Equal(0, Count(anOffset.Shape(), ShapeType.Edge));
    }

    [Fact]
    public void An_arc_the_medial_axis_turns_into_a_B_spline_is_offset_on_the_finer_polyline()
    {
        // tests/offset/wire_unclosed_outside_0_075/A1, OCCT's known failure (TODO OCC23748): an open wire of lines and arcs,
        // offset round both sides by 0.075 of its length. The medial axis's explorer turns two pieces of the arc of radius
        // 0.133, whose end misses its neighbour's, into B-splines with the end pole moved, so the offset is past their
        // sharpest bend and each is stood in for by a polyline. At OCCT's 0.01 of the distance the medial axis of that
        // polyline fails, "Offset is not done"; at 0.001 (decision 1980) it's built: one closed wire, a valid face on it,
        // 10.6026 long. Why it changed at 12T (decision 2058)
        string aPath = Path.Combine(typeof(BRepFill_OffsetWireTests).Assembly.GetCustomAttributes(typeof(System.Reflection.AssemblyMetadataAttribute), false)
                                        .Cast<System.Reflection.AssemblyMetadataAttribute>().Single(a => a.Key == "OcctTestDataDir").Value!, "brep", "offset_wire_001.brep");
        if (!File.Exists(aPath) || !BrepTools.Read(out Shape aShape, aPath))
        {
            return;
        }
        MakeWireOffset anOffset = new();
        anOffset.Init(global::Kerf.Geometry.JoinKind.Arc);
        anOffset.AddWire(ShapeCasts.Wire(aShape));
        anOffset.Perform(-0.075 * Length(aShape));
        Assert.True(anOffset.IsDone);
        Shape aResult = anOffset.Shape();
        Assert.Equal(1, Count(aResult, ShapeType.Wire));
        Wire aWire = ShapeCasts.Wire(new Explorer(aResult, ShapeType.Wire).Current());
        Assert.True(global::Kerf.Engine.Topology.Brep.Tool.IsClosed(aWire));
        Assert.Equal(10.6026, Length(aResult), 1.0e-4);
        Assert.True(new global::Kerf.Engine.Healing.Checks.Analyzer(new MakeFace(aWire, true).Face()).IsValid());
    }

    [Fact]
    public void A_medial_axis_that_goes_round_in_circles_takes_the_nodes_it_circles_as_one()
    {
        // tests/offset/wire_closed_inside_0_025/G4: 36 B-splines offset inwards past their bends. The medial axis of the
        // polyline at 0.001 of the distance (216 edges, decision 1980) removes and puts back the same six pieces of the
        // contour on every pass from the 18th: each is a fan of chords round an arc of radius 5, whose bisectors meet
        // within 3e-7 of one point, too far apart for the map to take them as one node and too near for it to tell
        // them apart. OCCT's loop stops only after 47,524 passes (the square of the 218 elements), 9 s and 22.4 GB.
        // MatCycle sees the state come back, and from then on the map takes such nodes as one (decision 2265): it
        // ends, and the offset is right, 9.728275 from the wire all along, where OCCT's own (291.991, 20 edges, from
        // its coarser polyline, whose map its patch against loops breaks off) comes within 5.96 of it and crosses
        // itself (OCC24255)
        string aPath = Path.Combine(typeof(BRepFill_OffsetWireTests).Assembly.GetCustomAttributes(typeof(System.Reflection.AssemblyMetadataAttribute), false)
                                        .Cast<System.Reflection.AssemblyMetadataAttribute>().Single(a => a.Key == "OcctTestDataDir").Value!, "brep", "offset_wire_081.brep");
        if (!File.Exists(aPath) || !BrepTools.Read(out Shape aShape, aPath))
        {
            return;
        }
        MakeWireOffset anOffset = new();
        anOffset.Init(global::Kerf.Geometry.JoinKind.Arc);
        anOffset.AddWire(ShapeCasts.Wire(aShape));
        long aBefore = GC.GetAllocatedBytesForCurrentThread();
        anOffset.Perform(-0.025 * Length(aShape));
        Assert.InRange(GC.GetAllocatedBytesForCurrentThread() - aBefore, 0, 1_000_000_000);
        Assert.True(anOffset.IsDone);
        Shape aResult = anOffset.Shape();
        Assert.Equal(1, Count(aResult, ShapeType.Wire));
        Assert.Equal(18, Count(aResult, ShapeType.Edge));
        Assert.Equal(18, Count(aResult, ShapeType.Vertex));
        Assert.Equal(281.771, Length(aResult), 1.0e-3);
        Explorer aWires = new(aResult, ShapeType.Wire);
        Assert.True(new global::Kerf.Engine.Healing.Checks.Analyzer(new MakeFace(ShapeCasts.Wire(aWires.Current()), true).Face()).IsValid());
    }

    [Fact]
    public void A_medial_axis_that_OCCT_breaks_off_in_a_loop_is_finished_and_its_offset_built()
    {
        // tests/offset/wire_closed_inside_0_075/G4, OCCT's known failure (TODO OCC23748, "Offset is not done"): the same
        // wire offset inwards by 29.18. Its medial axes go round the same fans, and OCCT's patch against loops (the same
        // number of edges ten inner passes apart) stops their cancelling and leaves them unfinished. The nodes are taken as
        // one before the patch acts (decision 2265), the maps end, and the offset is one valid wire, 29.184825 from the
        // wire all along (and all of it: a grid of points in the wire's plane has none deeper that is outside it)
        string aPath = Path.Combine(typeof(BRepFill_OffsetWireTests).Assembly.GetCustomAttributes(typeof(System.Reflection.AssemblyMetadataAttribute), false)
                                        .Cast<System.Reflection.AssemblyMetadataAttribute>().Single(a => a.Key == "OcctTestDataDir").Value!, "brep", "offset_wire_081.brep");
        if (!File.Exists(aPath) || !BrepTools.Read(out Shape aShape, aPath))
        {
            return;
        }
        MakeWireOffset anOffset = new();
        anOffset.Init(global::Kerf.Geometry.JoinKind.Arc);
        anOffset.AddWire(ShapeCasts.Wire(aShape));
        anOffset.Perform(-0.075 * Length(aShape));
        Assert.True(anOffset.IsDone);
        Shape aResult = anOffset.Shape();
        Assert.Equal(1, Count(aResult, ShapeType.Wire));
        Assert.Equal(15, Count(aResult, ShapeType.Edge));
        Assert.Equal(125.03, Length(aResult), 1.0e-2);
        Explorer aWires = new(aResult, ShapeType.Wire);
        Assert.True(new global::Kerf.Engine.Healing.Checks.Analyzer(new MakeFace(ShapeCasts.Wire(aWires.Current()), true).Face()).IsValid());
    }

    [Fact]
    public void Corners_a_millionth_off_round_offset_past_their_radius_give_the_rectangle()
    {
        // A rectangle 100 by 60 whose corners are quarter ellipses of semi-axes 5.000001 and 5, offset inwards by 7: each
        // corner is a fan of chords whose bisectors meet within a millionth of one point. OCCT's patch broke the loop that
        // makes off and the offset came out as a success, 285.629 long in 28 edges, crossing itself and as near as 5.34
        // to the wire. With the nodes taken as one (decision 2265) it is the rectangle 86 by 46
        const double anA = 5.000001, aB = 5.0, aCx = 50 - anA, aCy = 30 - aB;
        MakeWire aMW = new();
        void Corner(double theX, double theY, double theFrom)
        {
            Ellipse3D anEllipse = new(new Frame3D(new Point3D(theX, theY, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), anA, aB);
            aMW.Add(new MakeEdge(anEllipse, theFrom, theFrom + Math.PI / 2).Edge());
        }
        void Side(double theX1, double theY1, double theX2, double theY2) =>
            aMW.Add(new MakeEdge(new Point3D(theX1, theY1, 0), new Point3D(theX2, theY2, 0)).Edge());
        Corner(aCx, aCy, 0);
        Side(aCx, 30, -aCx, 30);
        Corner(-aCx, aCy, Math.PI / 2);
        Side(-50, aCy, -50, -aCy);
        Corner(-aCx, -aCy, Math.PI);
        Side(-aCx, -30, aCx, -30);
        Corner(aCx, -aCy, 3 * Math.PI / 2);
        Side(50, -aCy, 50, aCy);
        Assert.True(aMW.IsDone);

        MakeWireOffset anOffset = new();
        anOffset.Init(global::Kerf.Geometry.JoinKind.Arc);
        anOffset.AddWire(aMW.Wire());
        anOffset.Perform(-7);
        Assert.True(anOffset.IsDone);
        Shape aResult = anOffset.Shape();
        Assert.Equal(1, Count(aResult, ShapeType.Wire));
        Assert.Equal(4, Count(aResult, ShapeType.Edge));
        Assert.Equal(264, Length(aResult), 1.0e-5);
        Explorer aWires = new(aResult, ShapeType.Wire);
        Assert.True(new global::Kerf.Engine.Healing.Checks.Analyzer(new MakeFace(ShapeCasts.Wire(aWires.Current()), true).Face()).IsValid());
    }

    [Theory]
    [InlineData("case_3_wire2.brep", 0.025)]
    [InlineData("offset_wire_034.brep", 0.025)]
    [InlineData("offset_wire_021.brep", -0.005)]
    [InlineData("offset_wire_027.brep", 0.075)]
    public void An_offset_that_comes_nearer_the_wire_than_its_distance_is_refused(string theFile, double thePart)
    {
        // tests/offset/wire_unclosed_outside_0_025/B4 and B3, wire_closed_inside_0_005/B7 and wire_unclosed_outside_0_075/A9:
        // each offset was a success with edges inside the wire's own offset, as near as 2.35 and 10.08 to the wire for
        // 11.24, 0.02 for 0.93, and 2,299 for 2,357 (three are OCCT's known problems, a face on the result faulty; B3
        // passed OCCT's script). A parallel with a point more than 2% nearer the spine than the offset, or further with
        // round corners, is no success, whatever went wrong in making it (decisions 1742, 32601, 32603)
        string aPath = Path.Combine(typeof(BRepFill_OffsetWireTests).Assembly.GetCustomAttributes(typeof(System.Reflection.AssemblyMetadataAttribute), false)
                                        .Cast<System.Reflection.AssemblyMetadataAttribute>().Single(a => a.Key == "OcctTestDataDir").Value!, "brep", theFile);
        if (!File.Exists(aPath) || !BrepTools.Read(out Shape aShape, aPath))
        {
            return;
        }
        MakeWireOffset anOffset = new();
        anOffset.Init(global::Kerf.Geometry.JoinKind.Arc);
        anOffset.AddWire(ShapeCasts.Wire(aShape));
        anOffset.Perform(thePart * Length(aShape));
        Assert.False(anOffset.IsDone);
    }

    [Theory]
    [InlineData(global::Kerf.Geometry.JoinKind.Arc, -2.0, 0.0, 24.0)]
    [InlineData(global::Kerf.Geometry.JoinKind.Arc, -2.0, 7.0, 24.0)]
    [InlineData(global::Kerf.Geometry.JoinKind.Arc, 2.0, 7.0, 40.0 + 4.0 * Math.PI)]
    [InlineData(global::Kerf.Geometry.JoinKind.Intersection, 2.0, 0.0, 56.0)]
    [InlineData(global::Kerf.Geometry.JoinKind.Intersection, 2.0, -3.0, 56.0)]
    public void An_offset_at_its_distance_is_not_refused_at_an_altitude_or_with_sharp_corners(global::Kerf.Geometry.JoinKind theJoin, double theOffset,
        double theAltitude, double theLength)
    {
        // The check of decisions 32601 and 32603 measures in the spine's plane, whatever the parallel's altitude, and a
        // sharp corner is further than the offset, and right
        MakeWireOffset anOffset = new(Square(), theJoin);
        anOffset.Perform(theOffset, theAltitude);
        Assert.True(anOffset.IsDone);
        Assert.Equal(theLength, Length(anOffset.Shape()), 1.0e-9);
    }

    [Theory]
    [InlineData(1, 1)]
    [InlineData(3, 9)]
    [InlineData(5, 25)]
    [InlineData(6, 28)]
    [InlineData(218, 452)]
    [InlineData(2_000_000_000, 4_000_000_016L)]
    public void A_medial_axis_fails_after_twice_its_elements_and_16_passes(int theElements, long theLimit)
    {
        // decision 2266: OCCT's square of the elements where that is fewer; never more than an int holds
        Assert.Equal((int)Math.Min(theLimit, int.MaxValue), global::Kerf.Engine.Offsets.MedialAxis.Mat2d.PassLimit(theElements));
    }
}
