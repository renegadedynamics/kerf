// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

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
using Schematiq.Collections;

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
        Assert.True(aResult.IsClosed);
        Assert.Equal(theLength, Length(aResult), 1.0e-9);
        Assert.True(anOW.Spine().TShape()!.IsFrozen);

        // every edge of the square generates one parallel edge of the result
        for (Explorer anExp = new(aFace, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            ISchematiqList<Shape> aGenerated = anOW.GeneratedShapes(anExp.Current());
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
            Assert.False(aResult.IsClosed);
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
            ISchematiqList<Shape> aGenerated = anOffset.Generated(anExp.Current());
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
        Assert.Throws<NotDoneException>(() => new MakeWireOffset().Perform(1.0));
        // a non-planar wire: not done
        MakePolygon aMP = new(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(1, 1, 1), new Point3D(0, 1, 0), true);
        MakeWireOffset anOffset = new(aMP.Wire());
        anOffset.Perform(0.1);
        Assert.False(anOffset.IsDone);
    }
}
