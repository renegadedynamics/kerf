// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// BRepTools' statics (OCCT's BRepTools GTests read and write files, units 3C and 3H) and BRepAdaptor_Surface.
/// </summary>
public class BRepToolsTests
{
    private static TopoDS_Face FaceAt(in TopoDS_Shape theShape, int theIndex)
    {
        var anExp = new TopExp_Explorer(theShape, TopAbs_FACE);
        for (int i = 0; i < theIndex; i++)
        {
            anExp.Next();
        }

        return TopoDS.Face(anExp.Current());
    }

    /// <summary>A square face [0, 4] x [0, 4] on XOY with a square hole [1, 2] x [1, 3], the hole's wire first.</summary>
    private static TopoDS_Face FaceWithHole()
    {
        var aV = new TopoDS_Vertex[8];
        gp_Pnt[] aPts = [new(0, 0, 0), new(4, 0, 0), new(4, 4, 0), new(0, 4, 0), new(1, 1, 0), new(1, 3, 0), new(2, 3, 0), new(2, 1, 0)];
        for (int i = 0; i < 8; i++)
        {
            aV[i] = TestShapes.Vertex(aPts[i]);
        }

        var anEdges = new Dictionary<(int, int), TopoDS_Edge>();
        var aPlane = new Geom_Plane(new gp_Ax3(gp.XOY()));
        TopoDS_Face anOuter = TestShapes.PolygonFace(aV, new gp_Dir(0, 0, 1), anEdges, [0, 1, 2, 3], aPlane);
        TopoDS_Face aHole = TestShapes.PolygonFace(aV, new gp_Dir(0, 0, 1), anEdges, [4, 5, 6, 7], aPlane);

        // one face on the plane, with the hole's wire first: the edges have their pcurves on that plane
        var aBuilder = new BRep_Builder();
        TopoDS_Face aFace = TopoDS.Face(anOuter.EmptyCopied());
        aBuilder.Add(aFace, new TopoDS_Iterator(aHole).Value());
        aBuilder.Add(aFace, new TopoDS_Iterator(anOuter).Value());
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
        TopoDS_Face aFace = FaceWithHole();
        BRepTools.UVBounds(aFace, out double u1, out double u2, out double v1, out double v2);
        AssertBounds((0.0, 4.0, 0.0, 4.0), (u1, u2, v1, v2));

        var aWireExp = new TopExp_Explorer(aFace, TopAbs_WIRE);
        BRepTools.UVBounds(aFace, TopoDS.Wire(aWireExp.Current()), out u1, out u2, out v1, out v2);
        AssertBounds((1.0, 2.0, 1.0, 3.0), (u1, u2, v1, v2));

        TopoDS_Edge anEdge = TopoDS.Edge(new TopExp_Explorer(aFace, TopAbs_EDGE).Current());
        BRepTools.UVBounds(aFace, anEdge, out u1, out u2, out v1, out v2);
        AssertBounds((1.0, 1.0, 1.0, 3.0), (u1, u2, v1, v2));

        // a face without edges: the surface's natural bounds; without a surface: zeros
        var aBuilder = new BRep_Builder();
        TopoDS_Face aBare = new();
        aBuilder.MakeFace(ref aBare, new Geom_RectangularTrimmedSurface(new Geom_Plane(new gp_Ax3(gp.XOY())), -1, 2, -3, 4), 1e-7);
        BRepTools.UVBounds(aBare, out u1, out u2, out v1, out v2);
        AssertBounds((-1.0, 2.0, -3.0, 4.0), (u1, u2, v1, v2));
        TopoDS_Face aNoSurface = new();
        aBuilder.MakeFace(ref aNoSurface);
        BRepTools.UVBounds(aNoSurface, out u1, out u2, out v1, out v2);
        Assert.Equal((0.0, 0.0, 0.0, 0.0), (u1, u2, v1, v2));
    }

    [Fact]
    public void OuterWire_is_the_one_containing_the_others()
    {
        TopoDS_Face aFace = FaceWithHole();
        TopoDS_Wire anOuter = BRepTools.OuterWire(aFace);
        var anIt = new TopoDS_Iterator(aFace);
        anIt.Next();
        Assert.True(anOuter.IsSame(anIt.Value())); // the second wire
        Assert.True(BRepTools.OuterWire(new TopoDS_Face()).IsNull());
    }

    [Fact]
    public void UVBounds_of_a_cylinder_face()
    {
        TopoDS_Face aFace = TestShapes.Frozen(TestShapes.CylinderFace(2.0, 3.0));
        BRepTools.UVBounds(aFace, out double u1, out double u2, out double v1, out double v2);
        Assert.Equal(0.0, u1);
        Assert.Equal(2.0 * Math.PI, u2, 15);
        Assert.Equal((0.0, 3.0), (v1, v2));

        var anAdaptor = new BRepAdaptor_Surface(aFace);
        Assert.Equal(GeomAbs_SurfaceType.GeomAbs_Cylinder, anAdaptor.GetType());
        Assert.Equal(3.0, anAdaptor.LastVParameter());
        Assert.True(anAdaptor.Value(0.0, 1.5).Distance(new gp_Pnt(2, 0, 1.5)) < 1e-15);
        Assert.True(anAdaptor.Face() == aFace);
        Assert.Equal(Precision.Confusion(), anAdaptor.Tolerance());
    }

    [Fact]
    public void BRepAdaptor_Surface_applies_the_location()
    {
        TopoDS_Face aFace = TestShapes.Frozen(TestShapes.CylinderFace(2.0, 3.0));
        var aLoc = new TopLoc_Location(gp_Trsf.Translation(new gp_Vec(10, 0, 0)));
        TopoDS_Face aMoved = TopoDS.Face(aFace.Moved(aLoc));
        var anAdaptor = new BRepAdaptor_Surface(aMoved);
        Assert.True(anAdaptor.Value(0.0, 0.0).Distance(new gp_Pnt(12, 0, 0)) < 1e-15);
        var aCopy = (BRepAdaptor_Surface)anAdaptor.ShallowCopy();
        Assert.True(aCopy.Value(0.0, 0.0).Distance(new gp_Pnt(12, 0, 0)) < 1e-15);
        Assert.True(aCopy.Face() == aMoved);
        // without restriction, the surface's own bounds
        var anUnbounded = new BRepAdaptor_Surface(aMoved, false);
        Assert.True(Precision.IsInfinite(anUnbounded.LastVParameter()));
    }

    [Fact]
    public void Compare_and_orientation_of_edges()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        TopoDS_Face aFace = FaceAt(aBox, 0);
        var anExp = new TopExp_Explorer(aFace, TopAbs_EDGE);
        TopoDS_Edge anEdge = TopoDS.Edge(anExp.Current());
        Assert.Equal(anEdge.Orientation(), BRepTools.OriEdgeInFace(anEdge, aFace));
        Assert.True(BRepTools.Compare(anEdge, TopoDS.Edge(anEdge.Reversed())));
        Assert.Throws<Standard_ConstructionError>(
            () => BRepTools.OriEdgeInFace(TestShapes.Edge(new gp_Pnt(), new gp_Pnt(1, 0, 0)), aFace));

        TopoDS_Vertex aV1 = TestShapes.Vertex(new gp_Pnt(0, 0, 0));
        TopoDS_Vertex aV2 = TestShapes.Vertex(new gp_Pnt(0, 0, 0.5e-7));
        TopoDS_Vertex aV3 = TestShapes.Vertex(new gp_Pnt(0, 0, 1));
        Assert.True(BRepTools.Compare(aV1, aV2));
        Assert.False(BRepTools.Compare(aV1, aV3));

        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        BRepTools.Map3DEdges(aBox, aMap);
        Assert.Equal(12, aMap.Extent());
    }

    [Fact]
    public void Triangulation_and_CheckLocations()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        Assert.False(BRepTools.Triangulation(aBox, 0.1));

        var aList = new NCollection_List<TopoDS_Shape>();
        BRepTools.CheckLocations(aBox, aList);
        Assert.True(aList.IsEmpty());
        TopoDS_Shape aScaled = aBox.Moved(new TopLoc_Location(gp_Trsf.Scale(new gp_Pnt(), 2.0)));
        BRepTools.CheckLocations(aScaled, aList);
        Assert.Equal(1, aList.Extent());
    }

    [Fact]
    public void In_place_statics_refuse_frozen_shapes_and_change_unfrozen_ones()
    {
        // a face with a triangulation and edges with polygons on it
        TopoDS_Solid aBox = TestShapes.Box(1, 1, 1);
        TopoDS_Face aFace = FaceAt(aBox, 0);
        var aTri = new Poly_Triangulation(4, 2, false);
        var aBuilder = new BRep_Builder();
        aBuilder.UpdateFace(aFace, aTri);
        for (var anExp = new TopExp_Explorer(aFace, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            aBuilder.UpdateEdge(TopoDS.Edge(anExp.Current()), new Poly_PolygonOnTriangulation(2, false), aTri, new TopLoc_Location());
        }

        TopoDS_Solid aFrozen = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        Assert.Throws<TopoDS_LockedShape>(() => BRepTools.CleanGeometry(aFrozen));

        BRepTools.Clean(aBox);
        Assert.Null(BRep_Tool.Triangulation(aFace, out _));
        TopoDS_Edge anEdge = TopoDS.Edge(new TopExp_Explorer(aFace, TopAbs_EDGE).Current());
        Assert.Null(BRep_Tool.PolygonOnTriangulation(anEdge, aTri, new TopLoc_Location()));

        // CleanGeometry removes the surfaces and curves, and the pcurves on surfaces no face uses any more
        BRepTools.CleanGeometry(aBox);
        Assert.False(BRep_Tool.IsGeometric(aFace));
        Assert.False(BRep_Tool.IsGeometric(anEdge));
    }

    [Fact]
    public void RemoveInternals_removes_internal_sub_shapes()
    {
        var aBuilder = new BRep_Builder();
        TopoDS_Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);
        TopoDS_Edge anEdge = TestShapes.Edge(new gp_Pnt(), new gp_Pnt(1, 0, 0));
        aBuilder.Add(aCompound, anEdge);
        aBuilder.Add(aCompound, TestShapes.Edge(new gp_Pnt(0, 1, 0), new gp_Pnt(1, 1, 0)).Oriented(TopAbs_INTERNAL));
        BRepTools.RemoveInternals(aCompound);
        Assert.Equal(1, aCompound.NbChildren());
        Assert.True(new TopoDS_Iterator(aCompound).Value() == anEdge);
        Assert.True(aCompound.Free()); // restored
    }
}
