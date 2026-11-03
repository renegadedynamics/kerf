// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Fillets.Api;
using Kerf.Engine.Fillets.Surfaces;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Kernel.ModelingData.Tests;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// No copy at entry in the fillets (Kerf unit 10H5, phase10-plan.md 2.4): the 3D fillet and chamfer builder computes on
/// the caller's frozen shape in its working overlay (the vertex tolerances it raises, the spines' 3D curves), and the 2D
/// fillet builds the 3D curves it needs in one. After each, the input is untouched, TShape for TShape and value for value
/// (<see cref="ShapeSnapshot"/>), the result is frozen, and it holds the input's own faces away from the fillets.
/// </summary>
public class FilletInputsUnchangedTests
{
    private static List<Shape> SubShapes(in Shape shape, ShapeType type) =>
        BRepOffset_MakeOffsetTests.SubShapes(shape, type);

    // The number of the input's sub-shapes of the type whose TShapes are in the result
    private static int Shared(in Shape result, in Shape input, ShapeType type)
    {
        var tShapes = new HashSet<TShape>(ReferenceEqualityComparer.Instance);
        foreach (Shape s in SubShapes(result, type))
        {
            tShapes.Add(s.TShape()!);
        }
        return SubShapes(input, type).Count(s => tShapes.Contains(s.TShape()!));
    }

    // An edge of the box's top face (z = 30)
    private static Edge TopEdge(in Shape box) =>
        ShapeCasts.Edge(SubShapes(box, ShapeType.Edge).First(e =>
        {
            Exploration.Vertices(ShapeCasts.Edge(e), out Vertex v1, out Vertex v2);
            return Tool.Pnt(v1).Z > 29.0 && Tool.Pnt(v2).Z > 29.0;
        }));

    // A fused part, as the modelling layer holds one: two boxes, frozen
    private static Shape Fused() =>
        new Fuse(new MakeBox(10, 20, 30).Shape(), new MakeBox(new Point3D(5, 5, 20), 10, 10, 20).Shape()).Shape();

    [Theory]
    [InlineData("box")]
    [InlineData("fused part")]
    public void A_fillet_leaves_its_input_alone_and_keeps_the_faces_away_from_it(string kind)
    {
        Shape input = kind == "box" ? new MakeBox(10, 20, 30).Shape() : Fused();
        Assert.True(input.TShape()!.IsFrozen);
        ShapeSnapshot snapshot = ShapeSnapshot.Of(input);
        Edge edge = kind == "box" ? TopEdge(input) : ShapeCasts.Edge(SubShapes(input, ShapeType.Edge)[0]);
        var fillet = new MakeFillet(input);
        fillet.Add(1.0, edge);
        fillet.Build();
        snapshot.AssertUnchanged(input);
        Assert.True(fillet.IsDone);
        Shape result = fillet.Shape();
        Assert.True(BRepOffset_MakeOffsetTests.AllFrozen(result));
        Assert.True(new Analyzer(result).IsValid());
        // the two faces along the edge are modified, the two at its ends trimmed; the faces away are the input's own
        Assert.True(Shared(result, input, ShapeType.Face) >= SubShapes(input, ShapeType.Face).Count - 4);
        Assert.NotEmpty(fillet.Generated(edge));
    }

    [Fact]
    public void A_chamfer_leaves_its_input_alone_and_keeps_the_faces_away_from_it()
    {
        Shape box = new MakeBox(10, 20, 30).Shape();
        ShapeSnapshot snapshot = ShapeSnapshot.Of(box);
        var chamfer = new MakeChamfer(box);
        chamfer.Add(1.0, TopEdge(box));
        chamfer.Build();
        snapshot.AssertUnchanged(box);
        Assert.True(chamfer.IsDone);
        Shape result = chamfer.Shape();
        Assert.True(BRepOffset_MakeOffsetTests.AllFrozen(result));
        Assert.True(new Analyzer(result).IsValid());
        Assert.Equal(2, Shared(result, box, ShapeType.Face)); // the bottom and the side opposite the edge
    }

    [Fact]
    public void Two_computations_on_one_builder_leave_the_input_alone()
    {
        // each Compute works in a working overlay of its own, from the caller's shape as given
        Shape box = new MakeBox(10, 20, 30).Shape();
        ShapeSnapshot snapshot = ShapeSnapshot.Of(box);
        Edge edge = TopEdge(box);
        var fillet = new MakeFillet(box);
        fillet.Add(1.0, edge);
        fillet.Build();
        Shape first = fillet.Shape();
        Edge other = ShapeCasts.Edge(SubShapes(box, ShapeType.Edge).First(e =>
        {
            Exploration.Vertices(ShapeCasts.Edge(e), out Vertex v1, out Vertex v2);
            return Tool.Pnt(v1).Z < 1.0 && Tool.Pnt(v2).Z < 1.0;
        }));
        fillet.Add(1.0, other);
        fillet.Build();
        Shape second = fillet.Shape();
        snapshot.AssertUnchanged(box);
        Assert.True(new Analyzer(first).IsValid());
        Assert.True(new Analyzer(second).IsValid());
        Assert.True(LocOpeTests.Volume(second) < LocOpeTests.Volume(first));
    }

    [Fact]
    public void The_fillet_surfaces_leave_the_shape_alone()
    {
        Shape box = new MakeBox(10, 20, 30).Shape();
        ShapeSnapshot snapshot = ShapeSnapshot.Of(box);
        var edges = new NCollection_List<Shape>();
        edges.Append(TopEdge(box));
        var surfaces = new SurfaceBuilder(box, edges, 2.0);
        surfaces.Perform();
        snapshot.AssertUnchanged(box);
        Assert.Equal(StatusDone.IsOk, surfaces.IsDone);
        Assert.True(surfaces.NbSurface() > 0);
        // the support faces handed back are the caller's
        List<Shape> faces = SubShapes(box, ShapeType.Face);
        Assert.Contains(faces, f => f.IsSame(surfaces.SupportFace1(1)));
    }

    [Fact]
    public void A_2d_fillet_leaves_the_face_alone()
    {
        // the builder opens a working overlay for the 3D curves OCCT builds in place on the face's edges (decision 854)
        Face face = BRepFeatTests.Rect(0, 10, 0, 10, 0);
        ShapeSnapshot snapshot = ShapeSnapshot.Of(face);
        var fillet2d = new MakeFillet2D(face);
        fillet2d.AddFillet(ShapeCasts.Vertex(SubShapes(face, ShapeType.Vertex)[0]), 2.0);
        fillet2d.Build();
        snapshot.AssertUnchanged(face);
        Assert.True(fillet2d.IsDone);
        Shape result = fillet2d.Shape();
        Assert.True(BRepOffset_MakeOffsetTests.AllFrozen(result));
        Assert.Equal(100.0 - (4.0 - Math.PI), LocOpeTests.Area(result), 1.0e-6);
        Assert.Equal(2, Shared(result, face, ShapeType.Edge)); // the edges away from the corner
    }
}
