// Ported from build123d tests/test_direct_api/test_functions.py.
// Omitted:
// - Functions.parse_intersect_args: a TypeError for intersecting with a tuple of strings, which the compiler rules out.
// - MakeTopodsFaceFromWires.builder_failure: patches BRepBuilderAPI_MakeFace with a MagicMock.

namespace Kerf.Modeling.Tests.Topology;

using Kerf.Modeling;

public class FunctionsTests
{
    [Fact]
    public void EdgesToWires()
    {
        var squareEdges = Face.MakeRect(1, 1).Edges();
        var rectangleEdges = Face.MakeRect(2, 1, new Plane(new Point3D(5, 0, 0))).Edges();
        var wires = OneD.EdgesToWires(squareEdges.AddRange(rectangleEdges));
        Assert.Equal(2, wires.Count);
        Check.AlmostEqual(4, wires[0].Length, 5);
        Check.AlmostEqual(6, wires[1].Length, 5);
    }

    [Fact]
    public void Polar()
    {
        var pnt = TopologyUtils.Polar(1, 30);
        Check.AlmostEqual(Math.Sqrt(3) / 2, pnt.X, 5);
        Check.AlmostEqual(0.5, pnt.Y, 5);
    }

    [Fact]
    public void NewEdges()
    {
        var c = Solid.MakeCylinder(1, 5);
        var s = Solid.MakeSphere(2);
        var sMinusC = s - c;
        var seams = TopologyUtils.NewEdges([c, s], sMinusC);
        Assert.Single(seams);
        Check.AlmostEqual(1, seams[0].Radius, 5);
    }

    [Fact]
    public void Delta()
    {
        var cyl = Solid.MakeCylinder(1, 5);
        var sph = Solid.MakeSphere(2);
        var con = Solid.MakeCone(2, 1, 2);
        var plug = TopologyUtils.Delta([cyl, sph, con], [sph, con]);
        Assert.Single(plug);
        Assert.True(plug[0] == cyl);
    }

    [Fact]
    public void UnwrapTopodsCompound()
    {
        // Complex Compound
        var b1 = CompoundObjects.Box(1, 1, 1).SingleSolid();
        var b2 = CompoundObjects.Box(2, 2, 2).SingleSolid();
        var c1 = Compound.Make([b1, b2]);
        var c2 = Compound.Make([b1, c1]);
        var c3 = Compound.Make([c2]);
        var c4 = Compound.Make([c3]);
        Assert.Equal(1, c4.ChildCount);
        var c5 = (Compound)(TopoUtil.UnwrapCompound(((Compound)c4), false));
        Assert.Equal(2, c5.ChildCount);

        // unwrap fully
        var c0 = Compound.Make([b1]);
        c1 = Compound.Make([c0]);
        var result = (TopoUtil.UnwrapCompound(((Compound)c1), true));
        Check.Is<Solid>(result);

        // unwrap not fully
        result = (TopoUtil.UnwrapCompound(((Compound)c1), false));
        Check.Is<Compound>(result);
    }
}

/// <summary>
/// Face() rejects open wires with its own message, so these guards are only reachable through the helper itself.
/// </summary>
public class MakeTopodsFaceFromWiresTests
{
    readonly Wire _outer = Wire.MakeRect(10, 10);
    readonly Wire _inner = Wire.MakeCircle(0.5);
    readonly Wire _openWire = Wire.Make([Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(5, 0, 0)), Edge.MakeLine(new Point3D(5, 0, 0), new Point3D(5, 5, 0))]);

    [Fact]
    public void OpenOuterWire()
    {
        var error = Assert.Throws<ArgumentException>(() => TopoUtil.MakeFaceFromWires(_openWire, [_inner]));
        Assert.Contains("outer wire is not closed", error.Message);
    }

    [Fact]
    public void OpenInnerWire()
    {
        var error = Assert.Throws<ArgumentException>(() => TopoUtil.MakeFaceFromWires(_outer, [_openWire]));
        Assert.Contains("inner wire is not closed", error.Message);
    }
}

public class UnwrappedShapetypeTests
{
    [Fact]
    public void SingleShape() =>
        Assert.Equal(ShapeType.Solid, TopoUtil.UnwrappedShapeType(CompoundObjects.Box(1, 1, 1).SingleSolid()));

    [Fact]
    public void UniformCompound()
    {
        var compound = Compound.Make([CompoundObjects.Box(1, 1, 1).SingleSolid(), CompoundObjects.Box(2, 2, 2).SingleSolid()]);
        Assert.Equal(ShapeType.Solid, TopoUtil.UnwrappedShapeType(compound));
    }

    [Fact]
    public void MixedCompound()
    {
        var compound = Compound.Make([CompoundObjects.Box(1, 1, 1).SingleSolid(), Edge.MakeLine(new Point3D(5, 0, 0), new Point3D(6, 0, 0))]);
        Assert.Equal(ShapeType.Compound, TopoUtil.UnwrappedShapeType(compound));
    }
}
