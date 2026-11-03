// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Measurement;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Known answers of BRepBndLib: boxes of a box (with tolerances, close, optimal), of a cylinder made of faces, of
/// located and rotated shapes, of shapes carrying meshes (used or not), of faces that are only a mesh and edges that
/// are only a polygon, and oriented boxes (from points and from the axes of inertia). The shapes are frozen: computing
/// their boxes doesn't change them.
/// </summary>
public class BRepBndLibTests
{
    private static void AssertBox(in Box theBox, double x1, double y1, double z1, double x2, double y2, double z2, double theTol)
    {
        Assert.False(theBox.IsVoid);
        theBox.Get(out double ax1, out double ay1, out double az1, out double ax2, out double ay2, out double az2);
        Assert.Equal(x1, ax1, theTol);
        Assert.Equal(y1, ay1, theTol);
        Assert.Equal(z1, az1, theTol);
        Assert.Equal(x2, ax2, theTol);
        Assert.Equal(y2, ay2, theTol);
        Assert.Equal(z2, az2, theTol);
    }

    [Fact]
    public void Box_of_a_box()
    {
        Shape aBox = TestShapes.Frozen(TestShapes.Box(10, 20, 30));

        // enlarged by the tolerances (1e-7)
        Box aB = new();
        Bounds.Add(aBox, ref aB);
        AssertBox(aB, 0, 0, 0, 10, 20, 30, 1e-6);
        Assert.True(aB.GetGap() >= 1e-7);
        Assert.Equal(aB, Bounds.Box(aBox));

        // close: the edges' curves only, no gap
        Box aClose = Bounds.BoxClose(aBox);
        AssertBox(aClose, 0, 0, 0, 10, 20, 30, 1e-12);
        Assert.Equal(0.0, aClose.GetGap());

        // optimal: exact without the tolerances, enlarged with them
        AssertBox(Bounds.BoxOptimal(aBox, false, false), 0, 0, 0, 10, 20, 30, 1e-12);
        AssertBox(Bounds.BoxOptimal(aBox, false, true), -1e-7, -1e-7, -1e-7, 10 + 1e-7, 20 + 1e-7, 30 + 1e-7, 1e-12);
    }

    [Fact]
    public void Add_enlarges_a_box_already_holding_something()
    {
        Shape aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        Box aB = new(new Point3D(-5, -5, -5), new Point3D(-4, -4, -4));
        Bounds.AddOptimal(aBox, ref aB, false, false);
        AssertBox(aB, -5, -5, -5, 1, 1, 1, 1e-12);
    }

    [Fact]
    public void Box_of_a_cylinder()
    {
        Shape aCylinder = TestShapes.Frozen(BRepGPropTestShapes.CylinderSolid(2.0, 3.0));
        Box aB = Bounds.Box(aCylinder);
        aB.Get(out double x1, out double y1, out double z1, out double x2, out double y2, out double z2);
        Assert.True(x1 <= -2 && y1 <= -2 && z1 <= 0 && x2 >= 2 && y2 >= 2 && z2 >= 3);
        Assert.True(x1 > -2.5 && x2 < 2.5);

        AssertBox(Bounds.BoxOptimal(aCylinder, false, false), -2, -2, 0, 2, 2, 3, 1e-6);
    }

    [Fact]
    public void Located_and_rotated_shapes()
    {
        Shape aBox = TestShapes.Frozen(TestShapes.Box(10, 20, 30));
        Shape aMoved = ClassificationTestShapes.Translated(aBox, new Vector3D(100, -50, 7));
        AssertBox(Bounds.BoxOptimal(aMoved, false, false), 100, -50, 7, 110, -30, 37, 1e-9);

        Shape aRotated = ClassificationTestShapes.RotatedZ(aBox, Math.PI / 2);
        AssertBox(Bounds.BoxOptimal(aRotated, false, false), -20, 0, 0, 0, 10, 30, 1e-9);
    }

    [Fact]
    public void Meshes_are_used_if_asked()
    {
        Solid aMeshed = TestShapes.Box(4, 5, 6);
        BRepGPropTestShapes.AttachQuadMeshes(aMeshed);
        Shape aShape = TestShapes.Frozen(aMeshed);

        // the meshes are exact here: the same box either way, but from the nodes with the mesh
        AssertBox(Bounds.Box(aShape, true), 0, 0, 0, 4, 5, 6, 1e-6);
        AssertBox(Bounds.Box(aShape, false), 0, 0, 0, 4, 5, 6, 1e-6);
        AssertBox(Bounds.BoxOptimal(aShape, true, false), 0, 0, 0, 4, 5, 6, 1e-6);

        // a face that is only a mesh gives its nodes' box, mesh asked for or not
        Face aMeshFace = TestShapes.Frozen(BRepGPropTestShapes.MeshFace(
            [new Point3D(0, 0, 0), new Point3D(3, 0, 0), new Point3D(0, 4, 1)], (1, 2, 3)));
        AssertBox(Bounds.Box(aMeshFace, false), 0, 0, 0, 3, 4, 1, 1e-6);

        // an edge that is only a polygon
        Edge aPolygon = TestShapes.Frozen(BRepGPropTestShapes.PolygonEdge(new Point3D(1, 2, 3), new Point3D(-1, 5, 0), new Point3D(2, 2, 2)));
        AssertBox(Bounds.Box(aPolygon), -1, 2, 0, 2, 5, 3, 1e-6);
        AssertBox(Bounds.BoxOptimal(aPolygon), -1, 2, 0, 2, 5, 3, 1e-12);
    }

    [Fact]
    public void Free_vertices_and_edges()
    {
        Vertex aV = TestShapes.Frozen(TestShapes.Vertex(new Point3D(1, 2, 3)));
        AssertBox(Bounds.Box(aV), 1, 2, 3, 1, 2, 3, 1e-6);
        AssertBox(Bounds.BoxOptimal(aV), 1, 2, 3, 1, 2, 3, 0.0);

        Edge anArc = TestShapes.Frozen(TestShapes.Edge(
            new CircleCurve3D(new Frame3D(new Point3D(), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 1.0), 0.0, Math.PI));
        AssertBox(Bounds.BoxOptimal(anArc), -1, 0, 0, 1, 1, 0, 1e-9);
    }

    [Fact]
    public void Oriented_box_of_boxes()
    {
        Shape aBox = TestShapes.Frozen(TestShapes.Box(10, 20, 30));
        OrientedBox anOBB = Bounds.OBB(aBox, true, false, false);
        Assert.False(anOBB.IsVoid);
        Assert.True(new Point3D(anOBB.Center()).IsEqual(new Point3D(5, 10, 15), 1e-9));
        double[] aHalf = [anOBB.XHSize(), anOBB.YHSize(), anOBB.ZHSize()];
        Array.Sort(aHalf);
        Assert.Equal(5.0, aHalf[0], 1e-9);
        Assert.Equal(10.0, aHalf[1], 1e-9);
        Assert.Equal(15.0, aHalf[2], 1e-9);

        // rotated: the same sizes, the box turned with the shape
        Shape aRotated = ClassificationTestShapes.RotatedZ(aBox, Math.PI / 6);
        OrientedBox aRotOBB = Bounds.OBB(aRotated, true, true, false);
        double[] aRotHalf = [aRotOBB.XHSize(), aRotOBB.YHSize(), aRotOBB.ZHSize()];
        Array.Sort(aRotHalf);
        Assert.Equal(5.0, aRotHalf[0], 1e-6);
        Assert.Equal(10.0, aRotHalf[1], 1e-6);
        Assert.Equal(15.0, aRotHalf[2], 1e-6);
        Assert.False(aRotOBB.IsAABox);

        // AddOBB enlarges a box already holding something
        OrientedBox anOther = Bounds.OBB(ClassificationTestShapes.Translated(aBox, new Vector3D(50, 0, 0)), true, false, false);
        Bounds.AddOBB(aBox, ref anOther, true, false, false);
        Assert.False(anOther.IsOut(new Point3D(1, 1, 1)));
        Assert.False(anOther.IsOut(new Point3D(59, 19, 29)));
    }

    [Fact]
    public void Oriented_box_of_a_cylinder_uses_the_axes_of_inertia()
    {
        Shape aCylinder = TestShapes.Frozen(BRepGPropTestShapes.CylinderSolid(2.0, 3.0));
        // curved faces without triangulations: the inertia path
        OrientedBox anOBB = Bounds.OBB(aCylinder, true, true, false);
        Assert.False(anOBB.IsVoid);
        Assert.True(new Point3D(anOBB.Center()).IsEqual(new Point3D(0, 0, 1.5), 1e-6));
        double[] aHalf = [anOBB.XHSize(), anOBB.YHSize(), anOBB.ZHSize()];
        Array.Sort(aHalf);
        Assert.Equal(1.5, aHalf[0], 1e-6);
        Assert.Equal(2.0, aHalf[1], 1e-6);
        Assert.Equal(2.0, aHalf[2], 1e-6);
    }
}
