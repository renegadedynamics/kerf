// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Exchange.Brep;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Offsets;
using Kerf.Engine.Offsets.Api;
using Kerf.Engine.Offsets.Blend;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Kernel.ModelingData.Tests;
using Kerf.Topology;
using System.Reflection;
using static Kerf.Geometry.JoinKind;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// No copy at entry in the offsets (Kerf unit 10H5, phase10-plan.md 2.4): the 3D offset, the thick solid, the pipes and
/// spheres of BRepOffset_Offset and BiTgte's rolling ball run on the caller's frozen shape in a working overlay. After
/// each, the input is untouched, TShape for TShape and value for value (<see cref="ShapeSnapshot"/>), the result is
/// frozen, and it holds the input's own sub-shapes wherever the algorithm didn't change them. OCC5805's thick solid,
/// whose offset surface OCCT enlarges in place (decision 1101), is the load-bearing case.
/// </summary>
public class OffsetInputsUnchangedTests
{
    private static string Metadata(string key) =>
        typeof(OffsetInputsUnchangedTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>()
                                          .Single(a => a.Key == key).Value!;

    private static Shape Box() => new MakeBox(10, 20, 30).Shape();

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

    // The face of the box at z = value
    private static Face FaceAtZ(in Shape box, double value) =>
        ShapeCasts.Face(SubShapes(box, ShapeType.Face).Single(f =>
        {
            var props = new Kerf.Engine.Measurement.Properties();
            Kerf.Engine.Measurement.MassPropertiesBuilder.SurfaceProperties(f, props);
            return Math.Abs(props.CentreOfMass().Z - value) < 1e-9;
        }));

    [Fact]
    public void The_offset_shell_of_a_frozen_box_leaves_the_box_alone()
    {
        Shape box = Box();
        ShapeSnapshot snapshot = ShapeSnapshot.Of(box);
        var offset = new MakeOffsetShape();
        offset.PerformByJoin(box, 1.0, 1.0e-7, global::Kerf.Offsets.OffsetMode.Skin, false, false, Arc);
        snapshot.AssertUnchanged(box);
        Assert.True(offset.IsDone);
        Shape result = offset.Shape();
        Assert.True(BRepOffset_MakeOffsetTests.AllFrozen(result));
        Assert.True(new Analyzer(result).IsValid());
        // the history answers for the caller's faces
        foreach (Shape face in SubShapes(box, ShapeType.Face))
        {
            Assert.Single(offset.Generated(face));
        }
    }

    [Theory]
    [InlineData(Arc)]
    [InlineData(Intersection)]
    public void A_thick_solid_of_a_frozen_box_holds_the_faces_it_left_alone(JoinKind join)
    {
        Shape box = Box();
        ShapeSnapshot snapshot = ShapeSnapshot.Of(box);
        Face top = FaceAtZ(box, 30), bottom = FaceAtZ(box, 0);
        var closing = new NCollection_List<Shape>();
        closing.Append(top);
        var thick = new MakeThickSolid();
        thick.MakeThickSolidByJoin(box, closing, -1.0, 1.0e-3, global::Kerf.Offsets.OffsetMode.Skin, join == Intersection, false, join);
        snapshot.AssertUnchanged(box);
        Assert.True(thick.IsDone);
        Shape result = thick.Shape();
        Assert.True(BRepOffset_MakeOffsetTests.AllFrozen(result));
        Assert.True(new Analyzer(result).IsValid());
        Assert.Equal(10.0 * 20.0 * 30.0 - 8.0 * 18.0 * 29.0, BRepOffset_MakeOffsetTests.Volume(result), 3);
        // the bottom face is the caller's own; the faces round the opening share its edges, whose vertices changed
        Assert.Contains(SubShapes(result, ShapeType.Face), f => ReferenceEquals(f.TShape(), bottom.TShape()));
        Assert.Equal(1, Shared(result, box, ShapeType.Face));
        Assert.True(thick.IsDeleted(top) || thick.Modified(top).Count > 0);
    }

    [Fact]
    public void OCC5805s_thick_solid_enlarges_the_offset_surface_in_the_overlay_only()
    {
        // bugs/modalg_2/bug5805_3: OCCT enlarges the offset surface of the caller's face in place, and the lateral
        // edges are extended to it (decision 1101); in the working overlay the caller's face keeps its surface
        string path = Path.Combine(Metadata("OcctTestDataDir"), "brep", "OCC5805.brep");
        Assert.True(BrepTools.Read(out Shape shape, path), path);
        shape.TShape()!.FreezeGraph();
        ShapeSnapshot snapshot = ShapeSnapshot.Of(shape);
        List<Shape> faces = SubShapes(shape, ShapeType.Face);
        var surfaces = faces.Select(f => Tool.Surface(ShapeCasts.Face(f))).ToList();
        var closing = new NCollection_List<Shape>();
        closing.Append(faces[5]);
        var thick = new MakeThickSolid();
        thick.MakeThickSolidByJoin(shape, closing, -1.0, 1.0e-7);
        snapshot.AssertUnchanged(shape);
        Assert.Equal(surfaces, faces.Select(f => Tool.Surface(ShapeCasts.Face(f))).ToList());
        Assert.True(thick.IsDone);
        Shape result = thick.Shape();
        Assert.True(BRepOffset_MakeOffsetTests.AllFrozen(result));
        Assert.Equal(7, SubShapes(result, ShapeType.Face).Count);
        Assert.Single(SubShapes(result, ShapeType.Shell));
        Assert.Equal(1185.03, LocOpeTests.Area(result), 1);
    }

    [Fact]
    public void A_pipe_on_frozen_edges_leaves_them_alone()
    {
        Shape box = Box();
        ShapeSnapshot snapshot = ShapeSnapshot.Of(box);
        Face top = FaceAtZ(box, 30);
        Face side = ShapeCasts.Face(SubShapes(box, ShapeType.Face).First(f => !f.IsSame(top)
            && SubShapes(f, ShapeType.Edge).Exists(e => SubShapes(top, ShapeType.Edge).Exists(t => t.IsSame(e)))));
        Edge edge = ShapeCasts.Edge(SubShapes(side, ShapeType.Edge).First(e => SubShapes(top, ShapeType.Edge).Exists(t => t.IsSame(e))));
        var topOffset = new Offset(top, 2.0);
        var sideOffset = new Offset(side, 2.0);
        Edge e1 = ShapeCasts.Edge(topOffset.Generated(edge)), e2 = ShapeCasts.Edge(sideOffset.Generated(edge));
        Shape f1 = topOffset.Face(), f2 = sideOffset.Face();
        ShapeSnapshot snapshot1 = ShapeSnapshot.Of(f1), snapshot2 = ShapeSnapshot.Of(f2);

        Face pipe = new Offset(edge, e1, e2, 2.0).Face();
        snapshot.AssertUnchanged(box);
        snapshot1.AssertUnchanged(f1);
        snapshot2.AssertUnchanged(f2);
        Assert.True(BRepOffset_MakeOffsetTests.AllFrozen(pipe));
        Assert.True(new Analyzer(pipe).IsValid());
        // the edges given got pcurves on the pipe: in the pipe they are new TShapes
        Assert.DoesNotContain(SubShapes(pipe, ShapeType.Edge), e => e.IsSame(e1) || e.IsSame(e2));
    }

    [Fact]
    public void The_rolling_ball_on_a_frozen_box_leaves_the_box_alone_and_shares_its_faces()
    {
        Shape box = new MakeBox(10, 10, 10).Shape();
        ShapeSnapshot snapshot = ShapeSnapshot.Of(box);
        List<Shape> faces = SubShapes(box, ShapeType.Face);
        var blend = new OffsetBlend(box, -2.0, 1.0e-4, false);
        blend.SetFaces(ShapeCasts.Face(faces[0]), ShapeCasts.Face(faces[2]));
        blend.Perform(true);
        snapshot.AssertUnchanged(box);
        Assert.True(blend.IsDone);
        Shape result = blend.Shape();
        Assert.True(BRepOffset_MakeOffsetTests.AllFrozen(result));
        // the faces it rebuilt keep the box's surfaces
        var surfaces = new HashSet<object>(SubShapes(box, ShapeType.Face).Select(f => (object)Tool.Surface(ShapeCasts.Face(f))!),
                                           ReferenceEqualityComparer.Instance);
        Assert.Contains(SubShapes(result, ShapeType.Face), f => surfaces.Contains(Tool.Surface(ShapeCasts.Face(f))!));
    }

    [Fact]
    public void The_simple_offset_solid_of_a_frozen_face_leaves_the_face_alone()
    {
        // left on its copy at the entry (decision 1421): the input is untouched all the same
        Face top = FaceAtZ(Box(), 30);
        ShapeSnapshot snapshot = ShapeSnapshot.Of(top);
        var simple = new MakeSimpleOffset(top, 1.0);
        simple.SetBuildSolidFlag(true);
        simple.Perform();
        snapshot.AssertUnchanged(top);
        Assert.True(simple.IsDone);
        Shape result = simple.GetResultShape();
        Assert.True(BRepOffset_MakeOffsetTests.AllFrozen(result));
        Assert.Equal(6, SubShapes(result, ShapeType.Face).Count);
    }
}
