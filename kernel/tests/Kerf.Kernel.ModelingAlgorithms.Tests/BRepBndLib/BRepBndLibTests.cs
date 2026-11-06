// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Meshing;
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
        Obb anOBB = Bounds.OBB(aBox, true, false, false);
        Assert.False(anOBB.IsVoid);
        Assert.True(new Point3D(anOBB.Center()).IsEqual(new Point3D(5, 10, 15), 1e-9));
        double[] aHalf = [anOBB.XHSize(), anOBB.YHSize(), anOBB.ZHSize()];
        Array.Sort(aHalf);
        Assert.Equal(5.0, aHalf[0], 1e-9);
        Assert.Equal(10.0, aHalf[1], 1e-9);
        Assert.Equal(15.0, aHalf[2], 1e-9);

        // rotated: the same sizes, the box turned with the shape
        Shape aRotated = ClassificationTestShapes.RotatedZ(aBox, Math.PI / 6);
        Obb aRotOBB = Bounds.OBB(aRotated, true, true, false);
        double[] aRotHalf = [aRotOBB.XHSize(), aRotOBB.YHSize(), aRotOBB.ZHSize()];
        Array.Sort(aRotHalf);
        Assert.Equal(5.0, aRotHalf[0], 1e-6);
        Assert.Equal(10.0, aRotHalf[1], 1e-6);
        Assert.Equal(15.0, aRotHalf[2], 1e-6);
        Assert.False(aRotOBB.IsAABox);

        // AddOBB enlarges a box already holding something
        Obb anOther = Bounds.OBB(ClassificationTestShapes.Translated(aBox, new Vector3D(50, 0, 0)), true, false, false);
        Bounds.AddOBB(aBox, ref anOther, true, false, false);
        Assert.False(anOther.IsOut(new Point3D(1, 1, 1)));
        Assert.False(anOther.IsOut(new Point3D(59, 19, 29)));
    }

    [Fact]
    public void Oriented_box_of_a_cylinder_uses_the_axes_of_inertia()
    {
        Shape aCylinder = TestShapes.Frozen(BRepGPropTestShapes.CylinderSolid(2.0, 3.0));
        // curved faces without triangulations: the inertia path
        Obb anOBB = Bounds.OBB(aCylinder, true, true, false);
        Assert.False(anOBB.IsVoid);
        Assert.True(new Point3D(anOBB.Center()).IsEqual(new Point3D(0, 0, 1.5), 1e-6));
        double[] aHalf = [anOBB.XHSize(), anOBB.YHSize(), anOBB.ZHSize()];
        Array.Sort(aHalf);
        Assert.Equal(1.5, aHalf[0], 1e-6);
        Assert.Equal(2.0, aHalf[1], 1e-6);
        Assert.Equal(2.0, aHalf[2], 1e-6);
    }

    private static void AssertBox(in Box theBox, double theXMin, double theYMin, double theZMin, double theXMax, double theYMax, double theZMax)
    {
        theBox.Get(out double a, out double b, out double c, out double d, out double e, out double f);
        Assert.Equal(theXMin, a, 1.0e-5);
        Assert.Equal(theYMin, b, 1.0e-5);
        Assert.Equal(theZMin, c, 1.0e-5);
        Assert.Equal(theXMax, d, 1.0e-5);
        Assert.Equal(theYMax, e, 1.0e-5);
        Assert.Equal(theZMax, f, 1.0e-5);
    }

    [Fact]
    public void Optimal_box_from_a_mesh_leaves_out_the_tolerance_unless_asked()
    {
        // OCCT #1698: the face's tolerance was added whatever useShapeTolerance said
        Shape aShape = new MakeBox(100, 50, 20).Shape();
        aShape = new Kerf.Engine.Healing.Fixes.ShapeToleranceFixer().WithTolerance(aShape, 2.0, ShapeType.Face);
        aShape = IncrementalMesh.Meshed(aShape, new global::Kerf.Engine.Meshing.Tools.ToolParameters { Deflection = 0.1 });
        Box aPlain = new();
        Bounds.AddOptimal(aShape, ref aPlain, true, false);
        AssertBox(aPlain, 0, 0, 0, 100, 50, 20);
        Box aWithTolerance = new();
        Bounds.AddOptimal(aShape, ref aWithTolerance, true, true);
        AssertBox(aWithTolerance, -2, -2, -2, 102, 52, 22);
    }

    [Fact]
    public void Optimal_box_of_a_scaled_sphere_reaches_an_extreme_at_a_cell_centre()
    {
        // OCCT #1604: native 8.0.1 gives -19.8263 for x: the extreme is at the centre of a sampling cell
        GeneralTransform3D aScale = new GeneralTransform3D().WithVectorialPart(new Matrix3x3(20, 0, 0, 0, 10, 0, 0, 0, 5));
        Shape aShape = new GTransform(new MakeSphere(1.0).Shape(), aScale, true).Shape();
        Box aBox = new();
        Bounds.AddOptimal(aShape, ref aBox, false, false);
        AssertBox(aBox, -20, -10, -5, 20, 10, 5);
    }

    [Theory]
    [InlineData(0.0)]
    [InlineData(15.0)]
    [InlineData(45.0)]
    [InlineData(90.0)]
    public void Optimal_box_of_a_loft_holds_its_middle_section_however_it_is_turned(double theDegrees)
    {
        // OCCT #1679: native falls up to 0.044 short
        static Wire Circle(double theRadius, double theZ)
        {
            var aWire = new MakeWire();
            aWire.Add(new MakeEdge(new Circle3D(new Frame3D(new Point3D(0, 0, theZ), new Direction3D(0, 0, 1)), theRadius)).Edge());
            return aWire.Wire();
        }

        var aLoft = new ThruSections(true, false);
        aLoft.AddWire(Circle(10, 0));
        aLoft.AddWire(Circle(30, 20));
        aLoft.AddWire(Circle(10, 40));
        aLoft.Build();
        Transform3D aTurn = Transform3D.Rotation(new Axis3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), theDegrees * Math.PI / 180);
        Shape aShape = new Kerf.Engine.Construction.Api.Transform(aLoft.Shape(), aTurn, true).Shape();
        Box aBox = new();
        Bounds.AddOptimal(aShape, ref aBox, false, false);
        AssertBox(aBox, -30, -30, 0, 30, 30, 40);
    }

    [Theory]
    [InlineData(5.0)]
    [InlineData(8.428571428571429)]
    [InlineData(9.0)]
    public void Area_inside_a_periodic_spline_does_not_depend_on_where_the_edge_starts(double theStart)
    {
        // OCCT #1685: the box of a pcurve's arc past the first period was trimmed to the period, so the face's bounds
        // lost part of it, and the precise area was up to 2.7% short with an estimate of 1e-10.
        int[,] aXY = { { 2, 0 }, { 4, 0 }, { 6, 2 }, { 6, 4 }, { 4, 6 }, { 2, 6 }, { 0, 4 }, { 0, 2 } };
        var aPoles = new NCollection_Array1<Point3D>(1, 8);
        for (int i = 1; i <= 8; ++i)
        {
            aPoles[i] = new Point3D(aXY[i - 1, 0], aXY[i - 1, 1], 0);
        }

        var aKnots = new NCollection_Array1<double>(1, 9);
        var aMults = new NCollection_Array1<int>(1, 9);
        for (int i = 1; i <= 9; ++i)
        {
            aKnots[i] = i - 1;
            aMults[i] = 1;
        }

        var aCurve = new BSplineCurve3D(aPoles, aKnots, aMults, 5, true);
        var aWire = new MakeWire();
        aWire.Add(new MakeEdge(aCurve, theStart, theStart + 8).Edge());
        Face aFace = new MakeFace(new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), aWire.Wire()).Face();
        var aProps = new Properties();
        double anError = MassPropertiesBuilder.SurfaceProperties(aFace, aProps, 1.0e-10);
        Assert.True(anError < 1.0e-9);
        Assert.Equal(2596859.0 / 113400.0, aProps.Mass(), 1.0e-7);
    }

    [Fact]
    public void Precise_volume_of_an_extruded_spline_refines_along_U()
    {
        // OCCT #1675: the precise path never refined along U, and left U's error out of its estimate of about 1e-16
        Point3D[] aPts = [new(0, 4, 0), new(1, 5, 0), new(2, 4, 0), new(3, 5, 0), new(4, 4, 0)];
        var aPoles = new NCollection_Array1<Point3D>(1, 5);
        for (int i = 1; i <= 5; ++i)
        {
            aPoles[i] = aPts[i - 1];
        }

        NCollection_Array1<double> aKnots = NCollection_Array1<double>.From([0.0, 0.5, 1.0]);
        NCollection_Array1<int> aMults = NCollection_Array1<int>.From([4, 1, 4]);
        var aCurve = new BSplineCurve3D(aPoles, aKnots, aMults, 3);
        var aWire = new MakeWire();
        aWire.Add(new MakeEdge(aCurve).Edge());
        aWire.Add(new MakeEdge(aPts[4], aPts[0]).Edge());
        Face aFace = new MakeFace(aWire.Wire(), true).Face();
        Shape aPrism = new ApiMakePrism(aFace, new Vector3D(0, 0, 2)).Shape();
        var aProps = new Properties();
        MassPropertiesBuilder.VolumeProperties(aPrism, aProps, 1.0e-9);
        Assert.Equal(3.8, Math.Abs(aProps.Mass()), 1.0e-8);
    }
}
