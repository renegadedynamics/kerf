// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Features;
using Kerf.Engine.Features.Api;
using Kerf.Engine.Features.Local;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Kernel.ModelingData.Tests;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// No copy at entry in the features (Kerf unit 10H5, phase10-plan.md 2.4): BRepFeat's makers, LocOpe's local operations
/// (one working overlay shared by the operations they run, entered by each public call that writes) and the draft angle
/// run on the caller's frozen shapes. After each, the inputs are untouched, TShape for TShape and value for value
/// (<see cref="ShapeSnapshot"/>), the result is frozen, and it holds the inputs' own faces where the feature left them
/// alone.
/// </summary>
public class FeatureInputsUnchangedTests
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

    // A box 10 x 20 x 30, its top face, and a rectangle 4 x 6 on the top face (centred)
    private static (Shape Box, Face Top, Face Rect) BoxAndRect()
    {
        Shape box = LocOpeTests.Box();
        return (box, LocOpeTests.TopFace(box), BRepFeatTests.Rect(3, 7, 7, 13, 30));
    }

    private static Shape Untouched(Shape[] inputs, Func<Shape> feature)
    {
        ShapeSnapshot[] snapshots = [.. inputs.Select(s => ShapeSnapshot.Of(s))];
        Shape result = feature();
        for (int i = 0; i < inputs.Length; i++)
        {
            snapshots[i].AssertUnchanged(inputs[i]);
        }
        Assert.True(LocOpeTests.AllFrozen(result));
        Assert.True(new Analyzer(result).IsValid());
        return result;
    }

    [Theory]
    [InlineData("prism")]
    [InlineData("draft prism")]
    [InlineData("revolution")]
    [InlineData("pipe")]
    public void A_boss_leaves_the_part_and_the_profile_alone_and_keeps_the_bottom(string kind)
    {
        (Shape box, Face top, Face rect) = BoxAndRect();
        Shape result = Untouched([box, rect], () =>
        {
            Form form = kind switch
            {
                "prism" => new MakePrism(box, rect, top, new Direction3D(0, 0, 1), 1, true),
                "draft prism" => new MakeDraftPrism(box, rect, top, 5 * Math.PI / 180, 1, false),
                "revolution" => new MakeRevol(box, rect, top, new Axis3D(new Point3D(0, 15, 30), new Direction3D(1, 0, 0)), 1, true),
                _ => new MakePipe(box, rect, top, new MakeWire(new MakeEdge(new Point3D(5, 10, 30), new Point3D(5, 10, 35)).Edge()).Wire(), 1, true),
            };
            switch (form)
            {
                case MakePrism prism: prism.Perform(5.0); break;
                case MakeDraftPrism draft: draft.Perform(4.0); break;
                case MakeRevol revol: revol.Perform(30 * Math.PI / 180); break;
                case MakePipe pipe: pipe.Perform(); break;
            }
            Assert.True(form.IsDone);
            return form.Shape();
        });
        // the bottom face is the caller's own
        Assert.True(Shared(result, box, ShapeType.Face) >= 1);
    }

    [Fact]
    public void A_rib_leaves_the_part_and_the_contour_alone()
    {
        // OCCT's example of a linear form: a rib on an L-shaped prism
        var profile = new MakeWire();
        Point3D[] points = [new(0, 0, 0), new(200, 0, 0), new(200, 0, 50), new(50, 0, 50), new(50, 0, 200), new(0, 0, 200)];
        for (int i = 0; i < points.Length; i++)
        {
            profile.Add(new MakeEdge(points[i], points[(i + 1) % points.Length]).Edge());
        }
        Shape part = new ApiMakePrism(new MakeFace(profile.Wire()).Face(), new Vector3D(0, 100, 0)).Shape();
        Wire contour = new MakeWire(new MakeEdge(new Point3D(50, 45, 100), new Point3D(100, 45, 50)).Edge()).Wire();
        Untouched([part, contour], () =>
        {
            var rib = new MakeLinearForm(part, contour, new PlaneSurface(new Point3D(0, 45, 0), new Direction3D(0, 1, 0)),
                                         new Vector3D(0, 5, 0), new Vector3D(0, -3, 0), 1, true);
            rib.Perform();
            Assert.True(rib.IsDone);
            return rib.Shape();
        });
    }

    [Fact]
    public void A_split_and_a_gluing_leave_their_inputs_alone()
    {
        (Shape box, Face top, Face rect) = BoxAndRect();
        Shape split = Untouched([box, rect], () =>
        {
            var splitter = new SplitShape(box);
            splitter.Add(BrepTools.OuterWire(rect), top);
            splitter.Build();
            Assert.True(splitter.IsDone);
            return splitter.Shape();
        });
        Assert.Equal(5, Shared(split, box, ShapeType.Face));

        Shape glued = new ApiMakePrism(rect, new Vector3D(0, 0, 5)).Shape();
        Shape result = Untouched([box, glued], () =>
        {
            var gluer = new Gluer(glued, box);
            gluer.Bind(rect, top);
            var edges = new FindEdges(rect, top);
            for (edges.InitIterator(); edges.More(); edges.Next())
            {
                gluer.Bind(edges.EdgeFrom(), edges.EdgeTo());
            }
            return gluer.Shape();
        });
        Assert.Equal(5, Shared(result, box, ShapeType.Face));
    }

    [Fact]
    public void A_local_split_over_several_calls_leaves_the_shape_alone()
    {
        // LocOpe_SplitShape writes in its working overlay in each call; nothing stays open between them
        Shape box = LocOpeTests.Box();
        ShapeSnapshot snapshot = ShapeSnapshot.Of(box);
        Face top = LocOpeTests.TopFace(box);
        var split = new LocalSplitShape(box);
        Edge edge = ShapeCasts.Edge(SubShapes(top, ShapeType.Edge)[0]);
        Tool.Range(edge, out double f, out double l);
        Vertex vertex = new MakeVertex(Tool.Curve(edge, out _, out _)!.Value((f + l) / 2)).Vertex();
        split.Add(vertex, (f + l) / 2, edge);
        Assert.Null(WorkingOverlay.Current);
        snapshot.AssertUnchanged(box);
        Shape result = split.DescendantShapes(box).First();
        snapshot.AssertUnchanged(box);
        Assert.True(LocOpeTests.AllFrozen(result));
        Assert.Equal(13, LocOpeTests.Count(result, ShapeType.Edge));
    }

    [Theory]
    [InlineData("box side")]
    [InlineData("cylinder")]
    public void A_draft_leaves_its_input_alone(string kind)
    {
        // the cylinder's seam makes the draft correct the wires of the modifier's result in a working overlay
        Shape input = kind == "box side" ? new MakeBox(10, 10, 10).Shape() : new ApiMakeCylinder(5, 10).Shape();
        Face face = ShapeCasts.Face(SubShapes(input, ShapeType.Face).First(f =>
            Tool.Surface(ShapeCasts.Face(f)) is not PlaneSurface p || Math.Abs(p.Plane.Axis.Direction.Z) < 1e-9));
        Shape result = Untouched([input], () =>
        {
            var draft = new DraftAngle(input);
            draft.Add(face, new Direction3D(0, 0, 1), 5.0 * Math.PI / 180.0, new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)));
            Assert.True(draft.AddDone());
            draft.Build();
            Assert.True(draft.IsDone);
            return draft.Shape();
        });
        // the box's faces away from the drafted side are the input's own
        if (kind == "box side")
        {
            Assert.True(Shared(result, input, ShapeType.Face) >= 1);
        }
    }
}
