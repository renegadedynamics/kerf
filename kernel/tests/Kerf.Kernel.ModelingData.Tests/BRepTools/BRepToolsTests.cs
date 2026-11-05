// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Collections;
using Kerf.Engine.Geometry;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// BRepTools' statics (OCCT's BRepTools GTests read and write files, units 3C and 3H) and BRepAdaptor_Surface.
/// </summary>
public class BRepToolsTests
{
    private static Face FaceAt(in Shape theShape, int theIndex)
    {
        var anExp = new Explorer(theShape, ShapeType.Face);
        for (int i = 0; i < theIndex; i++)
        {
            anExp.Next();
        }

        return ShapeCasts.Face(anExp.Current());
    }

    /// <summary>A square face [0, 4] x [0, 4] on XOY with a square hole [1, 2] x [1, 3], the hole's wire first.</summary>
    private static Face FaceWithHole()
    {
        var aV = new Vertex[8];
        Point3D[] aPts = [new(0, 0, 0), new(4, 0, 0), new(4, 4, 0), new(0, 4, 0), new(1, 1, 0), new(1, 3, 0), new(2, 3, 0), new(2, 1, 0)];
        for (int i = 0; i < 8; i++)
        {
            aV[i] = TestShapes.Vertex(aPts[i]);
        }

        var anEdges = new Dictionary<(int, int), Edge>();
        var aPlane = new PlaneSurface(new CoordinateSystem(Constants.XOY()));
        Face anOuter = TestShapes.PolygonFace(aV, new Direction3D(0, 0, 1), anEdges, [0, 1, 2, 3], aPlane);
        Face aHole = TestShapes.PolygonFace(aV, new Direction3D(0, 0, 1), anEdges, [4, 5, 6, 7], aPlane);

        // one face on the plane, with the hole's wire first: the edges have their pcurves on that plane
        var aBuilder = new BrepBuilder();
        Face aFace = ShapeCasts.Face(anOuter.EmptyCopied());
        aBuilder.Add(aFace, new Iterator(aHole).Value());
        aBuilder.Add(aFace, new Iterator(anOuter).Value());
        return TestShapes.Frozen(aFace);
    }

    private static void AssertBounds((double, double, double, double) theExpected, (double, double, double, double) theActual)
    {
        Assert.Equal(theExpected.Item1, theActual.Item1, 1e-12);
        Assert.Equal(theExpected.Item2, theActual.Item2, 1e-12);
        Assert.Equal(theExpected.Item3, theActual.Item3, 1e-12);
        Assert.Equal(theExpected.Item4, theActual.Item4, 1e-12);
    }

    [Fact]
    public void UVBounds_of_faces_wires_and_edges()
    {
        Face aFace = FaceWithHole();
        BrepTools.UVBounds(aFace, out double u1, out double u2, out double v1, out double v2);
        AssertBounds((0.0, 4.0, 0.0, 4.0), (u1, u2, v1, v2));

        var aWireExp = new Explorer(aFace, ShapeType.Wire);
        BrepTools.UVBounds(aFace, ShapeCasts.Wire(aWireExp.Current()), out u1, out u2, out v1, out v2);
        AssertBounds((1.0, 2.0, 1.0, 3.0), (u1, u2, v1, v2));

        Edge anEdge = ShapeCasts.Edge(new Explorer(aFace, ShapeType.Edge).Current());
        BrepTools.UVBounds(aFace, anEdge, out u1, out u2, out v1, out v2);
        AssertBounds((1.0, 1.0, 1.0, 3.0), (u1, u2, v1, v2));

        // a face without edges: the surface's natural bounds; without a surface: zeros
        var aBuilder = new BrepBuilder();
        Face aBare = new();
        aBuilder.MakeFace(ref aBare, new TrimmedSurface(new PlaneSurface(new CoordinateSystem(Constants.XOY())), -1, 2, -3, 4), 1e-7);
        BrepTools.UVBounds(aBare, out u1, out u2, out v1, out v2);
        AssertBounds((-1.0, 2.0, -3.0, 4.0), (u1, u2, v1, v2));
        Face aNoSurface = new();
        aBuilder.MakeFace(ref aNoSurface);
        BrepTools.UVBounds(aNoSurface, out u1, out u2, out v1, out v2);
        Assert.Equal((0.0, 0.0, 0.0, 0.0), (u1, u2, v1, v2));
    }

    [Fact]
    public void OuterWire_is_the_one_containing_the_others()
    {
        Face aFace = FaceWithHole();
        Wire anOuter = BrepTools.OuterWire(aFace);
        var anIt = new Iterator(aFace);
        anIt.Next();
        Assert.True(anOuter.IsSame(anIt.Value())); // the second wire
        Assert.True(BrepTools.OuterWire(new Face()).IsNull);
    }

    [Fact]
    public void UVBounds_of_a_cylinder_face()
    {
        Face aFace = TestShapes.Frozen(TestShapes.CylinderFace(2.0, 3.0));
        BrepTools.UVBounds(aFace, out double u1, out double u2, out double v1, out double v2);
        Assert.Equal(0.0, u1);
        Assert.Equal(2.0 * Math.PI, u2, 15);
        Assert.Equal((0.0, 3.0), (v1, v2));

        var anAdaptor = new FaceSurfaceAdaptor(aFace);
        Assert.Equal(SurfaceKind.Cylinder, anAdaptor.GetType());
        Assert.Equal(3.0, anAdaptor.LastVParameter());
        Assert.True(anAdaptor.Value(0.0, 1.5).DistanceTo(new Point3D(2, 0, 1.5)) < 1e-15);
        Assert.True(anAdaptor.Face() == aFace);
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), anAdaptor.Tolerance());
    }

    [Fact]
    public void BRepAdaptor_Surface_applies_the_location()
    {
        Face aFace = TestShapes.Frozen(TestShapes.CylinderFace(2.0, 3.0));
        var aLoc = new Placement(Transform3D.Translation(new Vector3D(10, 0, 0)));
        Face aMoved = ShapeCasts.Face(aFace.Moved(aLoc));
        var anAdaptor = new FaceSurfaceAdaptor(aMoved);
        Assert.True(anAdaptor.Value(0.0, 0.0).DistanceTo(new Point3D(12, 0, 0)) < 1e-15);
        var aCopy = (FaceSurfaceAdaptor)anAdaptor.ShallowCopy();
        Assert.True(aCopy.Value(0.0, 0.0).DistanceTo(new Point3D(12, 0, 0)) < 1e-15);
        Assert.True(aCopy.Face() == aMoved);
        // without restriction, the surface's own bounds
        var anUnbounded = new FaceSurfaceAdaptor(aMoved, false);
        Assert.True(global::Kerf.Engine.Foundation.Precision.IsInfinite(anUnbounded.LastVParameter()));
    }

    [Fact]
    public void Compare_and_orientation_of_edges()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        Face aFace = FaceAt(aBox, 0);
        var anExp = new Explorer(aFace, ShapeType.Edge);
        Edge anEdge = ShapeCasts.Edge(anExp.Current());
        Assert.Equal(anEdge.Orientation, BrepTools.OriEdgeInFace(anEdge, aFace));
        Assert.True(BrepTools.Compare(anEdge, ShapeCasts.Edge(anEdge.Reversed())));
        EngineAssert.Throws<global::Kerf.Engine.Foundation.ConstructionErrorException>(
            () => BrepTools.OriEdgeInFace(TestShapes.Edge(new Point3D(), new Point3D(1, 0, 0)), aFace));

        Vertex aV1 = TestShapes.Vertex(new Point3D(0, 0, 0));
        Vertex aV2 = TestShapes.Vertex(new Point3D(0, 0, 0.5e-7));
        Vertex aV3 = TestShapes.Vertex(new Point3D(0, 0, 1));
        Assert.True(BrepTools.Compare(aV1, aV2));
        Assert.False(BrepTools.Compare(aV1, aV3));

        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        BrepTools.Map3DEdges(aBox, aMap);
        Assert.Equal(12, aMap.Count);
    }

    [Fact]
    public void Triangulation_and_CheckLocations()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        Assert.False(BrepTools.Triangulation(aBox, 0.1));

        var aList = new NCollection_List<Shape>();
        BrepTools.CheckLocations(aBox, aList);
        Assert.True(aList.IsEmpty());
        Shape aScaled = aBox.Moved(new Placement(Transform3D.Scale(new Point3D(), 2.0)));
        BrepTools.CheckLocations(aScaled, aList);
        Assert.Equal(1, aList.Extent());
    }

    [Fact]
    public void In_place_statics_refuse_frozen_shapes_and_change_unfrozen_ones()
    {
        // a face with a triangulation and edges with polygons on it
        Solid aBox = TestShapes.Box(1, 1, 1);
        Face aFace = FaceAt(aBox, 0);
        var aTri = new Triangulation(4, 2, false);
        var aBuilder = new BrepBuilder();
        aBuilder.UpdateFace(aFace, aTri);
        for (var anExp = new Explorer(aFace, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            aBuilder.UpdateEdge(ShapeCasts.Edge(anExp.Current()), new PolygonOnTriangulation(2, false), aTri, new Placement());
        }

        Solid aFrozen = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        EngineAssert.Throws<ImmutableObjectException>(() => BrepTools.CleanGeometry(aFrozen));

        BrepTools.Clean(aBox);
        Assert.Null(Tool.Triangulation(aFace, out _));
        Edge anEdge = ShapeCasts.Edge(new Explorer(aFace, ShapeType.Edge).Current());
        Assert.Null(Tool.PolygonOnTriangulation(anEdge, aTri, new Placement()));

        // CleanGeometry removes the surfaces and curves, and the pcurves on surfaces no face uses any more
        BrepTools.CleanGeometry(aBox);
        Assert.False(Tool.IsGeometric(aFace));
        Assert.False(Tool.IsGeometric(anEdge));
    }

    [Fact]
    public void RemoveInternals_removes_internal_sub_shapes()
    {
        var aBuilder = new BrepBuilder();
        Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);
        Edge anEdge = TestShapes.Edge(new Point3D(), new Point3D(1, 0, 0));
        aBuilder.Add(aCompound, anEdge);
        aBuilder.Add(aCompound, TestShapes.Edge(new Point3D(0, 1, 0), new Point3D(1, 1, 0)).Oriented(Internal));
        BrepTools.RemoveInternals(aCompound);
        Assert.Equal(1, aCompound.ChildCount);
        Assert.True(new Iterator(aCompound).Value() == anEdge);
        Assert.True(aCompound.Free()); // restored
    }
}
