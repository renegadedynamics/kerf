// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Kernel.ModelingData.Tests;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The working overlay (Kerf unit 10H1, phase10-plan.md 2.4): writes into a frozen input go to working copies, reads
/// see them, the result shares what didn't change and its history is direct; the input never changes; overlays nest
/// and run in parallel.
/// </summary>
public class WorkingOverlayTests
{
    private static Shape FrozenBox() => new MakeBox(10, 20, 30).Shape(); // published: frozen

    private static List<Shape> SubShapes(in Shape shape, ShapeType type)
    {
        var map = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(shape, type, map);
        var list = new List<Shape>();
        for (int i = 0; i < map.Count; i++)
        {
            list.Add(map[i]);
        }
        return list;
    }

    private static HashSet<TShape> TShapes(in Shape shape)
    {
        var set = new HashSet<TShape>(ReferenceEqualityComparer.Instance);
        var stack = new Stack<TShape>();
        stack.Push(shape.TShape()!);
        while (stack.Count > 0)
        {
            TShape t = stack.Pop();
            if (set.Add(t))
            {
                for (var it = new Iterator(new Shape(t, new Placement(), Orientation.Forward)); it.More(); it.Next())
                {
                    stack.Push(it.Value().TShape()!);
                }
            }
        }
        return set;
    }

    private static bool Holds(in Shape shape, in Shape sub, ShapeType type)
    {
        TShape? tShape = sub.TShape();
        return SubShapes(shape, type).Exists(s => ReferenceEquals(s.TShape(), tShape));
    }

    [Fact]
    public void Without_an_overlay_a_frozen_shape_still_refuses_writes()
    {
        Shape box = FrozenBox();
        Vertex v = ShapeCasts.Vertex(SubShapes(box, ShapeType.Vertex)[0]);
        Edge e = ShapeCasts.Edge(SubShapes(box, ShapeType.Edge)[0]);
        EngineAssert.Throws<ImmutableObjectException>(() => new BrepBuilder().UpdateVertex(v, 1.0));
        EngineAssert.Throws<ImmutableObjectException>(() => new BrepBuilder().Range(e, 0, 1));
        EngineAssert.Throws<ImmutableObjectException>(() => ((BrepTVertex)v.TShape()!).Tolerance(1.0));
        EngineAssert.Throws<ImmutableObjectException>(() => box.Closed(false));
    }

    [Fact]
    public void Writes_are_read_back_and_the_input_never_changes()
    {
        Shape box = FrozenBox();
        ShapeSnapshot snapshot = ShapeSnapshot.Of(box);
        Vertex v = ShapeCasts.Vertex(SubShapes(box, ShapeType.Vertex)[0]);
        Edge e = ShapeCasts.Edge(SubShapes(box, ShapeType.Edge)[0]);
        Face f = ShapeCasts.Face(SubShapes(box, ShapeType.Face)[0]);
        double tolV = Tool.Tolerance(v);
        var builder = new BrepBuilder();

        Shape result;
        using (var overlay = WorkingOverlay.Begin(box))
        {
            builder.UpdateVertex(v, 0.5);                 // a vertex's tolerance
            builder.Range(e, 0.25, 0.75);                 // an edge's range
            builder.SameParameter(e, false);              // an edge's flag
            builder.UpdateFace(f, 0.01);                  // a face's tolerance
            builder.UpdateEdge(e, new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0)), PlaneOf(f), new Placement(), 1e-3);

            // reads go through the overlay, as the operation's own code reads them
            Assert.Equal(0.5, Tool.Tolerance(v));
            Tool.Range(e, out double first, out double last);
            Assert.Equal((0.25, 0.75), (first, last));
            Assert.False(Tool.SameParameter(e));
            Assert.Equal(0.01, Tool.Tolerance(f));
            Assert.Equal(3, overlay.WorkingCount); // v, e and f: the reads made no copies
            result = overlay.Finish(box);
        }

        snapshot.AssertUnchanged(box);
        Assert.Equal(tolV, Tool.Tolerance(v));
        Assert.True(Tool.SameParameter(e));
        Assert.True(box.TShape()!.IsFrozen);

        // the result holds the changes, under new TShapes
        Assert.Equal(0.5, SubShapes(result, ShapeType.Vertex).Max(s => Tool.Tolerance(ShapeCasts.Vertex(s))));
        Assert.Equal(0.01, SubShapes(result, ShapeType.Face).Max(s => Tool.Tolerance(ShapeCasts.Face(s))));
        Assert.Single(SubShapes(result, ShapeType.Edge), s => !Tool.SameParameter(ShapeCasts.Edge(s)));
        Assert.False(ReferenceEquals(result.TShape(), box.TShape()));
    }

    private static Surface PlaneOf(in Face face) => Tool.Surface(face)!;

    [Fact]
    public void Unchanged_sub_shapes_are_shared_and_only_the_path_above_a_change_is_new()
    {
        Shape box = FrozenBox();
        Vertex v = ShapeCasts.Vertex(SubShapes(box, ShapeType.Vertex)[0]);
        Shape result;
        using (var overlay = WorkingOverlay.Begin(box))
        {
            new BrepBuilder().UpdateVertex(v, 0.5);
            result = overlay.Finish(box);
        }

        HashSet<TShape> before = TShapes(box), after = TShapes(result);
        var fresh = new HashSet<TShape>(after, ReferenceEqualityComparer.Instance);
        fresh.ExceptWith(before);
        // the vertex, its 3 edges, the 3 wires and 3 faces holding them, the shell and the solid
        Assert.Equal(1 + 3 + 3 + 3 + 1 + 1, fresh.Count);
        Assert.Equal(before.Count, after.Count);
        foreach (Shape face in SubShapes(box, ShapeType.Face))
        {
            Assert.Equal(!Holds(face, v, ShapeType.Vertex), Holds(result, face, ShapeType.Face));
        }
        // the edges away from the vertex are the caller's own
        Assert.Equal(12 - 3, SubShapes(box, ShapeType.Edge).Count(edge => Holds(result, edge, ShapeType.Edge)));
        Assert.Equal(0.5, SubShapes(result, ShapeType.Vertex).Max(s => Tool.Tolerance(ShapeCasts.Vertex(s))));
    }

    [Fact]
    public void The_history_is_direct()
    {
        Shape box = FrozenBox();
        Vertex v = ShapeCasts.Vertex(SubShapes(box, ShapeType.Vertex)[0]);
        Shape result;
        History history;
        using (var overlay = WorkingOverlay.Begin(box))
        {
            new BrepBuilder().UpdateVertex(v, 0.5);
            result = overlay.Finish(box);
            history = overlay.History();
            Assert.True(overlay.Final(v).IsSame(history.Modified(v)[0]));
        }

        IReadOnlyList<Shape> modified = history.Modified(v);
        Assert.Single(modified);
        Assert.True(Holds(result, modified[0], ShapeType.Vertex));
        Assert.Equal(0.5, Tool.Tolerance(ShapeCasts.Vertex(modified[0])));
        foreach (Shape face in SubShapes(box, ShapeType.Face))
        {
            bool isTouched = Holds(face, v, ShapeType.Vertex);
            Assert.Equal(isTouched ? 1 : 0, history.Modified(face).Count);
            Assert.False(history.IsRemoved(face));
        }
        Assert.Single(history.Modified(box));
    }

    [Fact]
    public void Nothing_written_or_nothing_changed_gives_back_the_input()
    {
        Shape box = FrozenBox();
        Vertex v = ShapeCasts.Vertex(SubShapes(box, ShapeType.Vertex)[0]);
        using (var overlay = WorkingOverlay.Begin(box))
        {
            Assert.Equal(Tool.Tolerance(v), Tool.Tolerance(v));
            Assert.Same(box.TShape(), overlay.Finish(box).TShape());
            Assert.Empty(overlay.History().Modified(v));
        }

        // a write that leaves the data as it was: UpdateVertex only raises a tolerance; Modified is a cache bit. Since
        // unit 10H3 such a write makes no working copy (WorkingOverlay.TakesWrite)
        using (var overlay = WorkingOverlay.Begin(box))
        {
            new BrepBuilder().UpdateVertex(v, 1e-12);
            Assert.Equal(0, overlay.WorkingCount);
            Assert.Same(box.TShape(), overlay.Finish(box).TShape());
        }
    }

    [Fact]
    public void Sub_shapes_removed_and_added_in_the_overlay()
    {
        Shape box = FrozenBox();
        Face face = ShapeCasts.Face(SubShapes(box, ShapeType.Face)[0]);
        Wire wire = ShapeCasts.Wire(SubShapes(face, ShapeType.Wire)[0].Oriented(Orientation.Forward));
        Edge edge = ShapeCasts.Edge(SubShapes(wire, ShapeType.Edge)[0]);
        ShapeSnapshot snapshot = ShapeSnapshot.Of(box);
        Shape result;
        using (var overlay = WorkingOverlay.Begin(box))
        {
            var builder = new Builder();
            wire.Free(true); // OCCT's idiom before editing a shape in place
            Shape child = default;
            for (var it = new Iterator(wire, false, false); it.More(); it.Next())
            {
                if (it.Value().IsSame(edge))
                {
                    child = it.Value();
                }
            }
            builder.Remove(wire, child);
            Assert.Equal(3, wire.ChildCount);
            builder.Add(wire, child.Reversed());
            Assert.Equal(4, wire.ChildCount);
            result = overlay.Finish(box);
            Assert.Single(overlay.History().Modified(face)); // wires have no history (BRepTools_History)
        }

        snapshot.AssertUnchanged(box);
        Assert.Equal(4, wire.ChildCount);
        Shape newWire = SubShapes(result, ShapeType.Wire).Find(w => !Holds(box, w, ShapeType.Wire));
        Assert.False(newWire.IsNull);
        // the edge is the caller's own, reversed and last in the new wire
        Shape last = default;
        for (var it = new Iterator(newWire, false, false); it.More(); it.Next())
        {
            last = it.Value();
        }
        Assert.Same(edge.TShape(), last.TShape());
        Assert.Equal(12, SubShapes(result, ShapeType.Edge).Count(e => Holds(box, e, ShapeType.Edge)));
    }

    [Fact]
    public void Nested_overlays_see_the_outer_writes_and_keep_their_own()
    {
        Shape box = FrozenBox();
        Vertex v = ShapeCasts.Vertex(SubShapes(box, ShapeType.Vertex)[0]);
        Edge far = ShapeCasts.Edge(SubShapes(box, ShapeType.Edge).Find(e => !Holds(e, v, ShapeType.Vertex)));
        Tool.Range(far, out double first0, out double last0);
        var builder = new BrepBuilder();

        using var outer = WorkingOverlay.Begin(box);
        builder.UpdateVertex(v, 0.5);

        Shape inner;
        using (var overlay = WorkingOverlay.Begin(box))
        {
            Assert.Same(overlay, WorkingOverlay.Current);
            Assert.Same(outer, overlay.Parent);
            Assert.Equal(0.5, Tool.Tolerance(v));         // the outer write, seen inside
            builder.UpdateVertex(v, 0.75);                // the inner one's own
            builder.Range(far, 1, 2);
            Assert.Equal(0.75, Tool.Tolerance(v));
            inner = overlay.Finish(box);
        }

        // the inner writes didn't reach the outer operation
        Assert.Same(outer, WorkingOverlay.Current);
        Assert.Equal(0.5, Tool.Tolerance(v));
        Tool.Range(far, out double first, out double last);
        Assert.Equal((first0, last0), (first, last));
        Assert.Equal(1, outer.WorkingCount);

        // the inner result holds both its writes, as it saw them
        Assert.Equal(0.75, SubShapes(inner, ShapeType.Vertex).Max(s => Tool.Tolerance(ShapeCasts.Vertex(s))));
        Shape outerResult = outer.Finish(box);
        outer.Dispose();
        Assert.Null(WorkingOverlay.Current);
        Assert.Equal(0.5, SubShapes(outerResult, ShapeType.Vertex).Max(s => Tool.Tolerance(ShapeCasts.Vertex(s))));
        Assert.True(Holds(outerResult, far, ShapeType.Edge));
        Assert.Equal(0.75, SubShapes(inner, ShapeType.Vertex).Max(s => Tool.Tolerance(ShapeCasts.Vertex(s))));
        Assert.False(Holds(inner, far, ShapeType.Edge));
    }

    [Fact]
    public void Parallel_operations_on_one_input_have_their_own_overlays()
    {
        Shape box = FrozenBox();
        ShapeSnapshot snapshot = ShapeSnapshot.Of(box);
        List<Shape> vertices = SubShapes(box, ShapeType.Vertex);
        var results = new Shape[64];
        Parallel.For(0, results.Length, i =>
        {
            Vertex v = ShapeCasts.Vertex(vertices[i % vertices.Count]);
            using var overlay = WorkingOverlay.Begin(box);
            new BrepBuilder().UpdateVertex(v, 1.0 + i);
            Assert.Equal(1.0 + i, Tool.Tolerance(v));
            results[i] = overlay.Finish(box);
        });

        snapshot.AssertUnchanged(box);
        for (int i = 0; i < results.Length; i++)
        {
            var tolerances = SubShapes(results[i], ShapeType.Vertex).Select(s => Tool.Tolerance(ShapeCasts.Vertex(s))).ToList();
            Assert.Equal(1.0 + i, tolerances.Max());
            Assert.Equal(7, tolerances.Count(t => t < 1.0));
        }
    }

    [Fact]
    public void Parallel_bodies_of_one_operation_write_into_its_overlay()
    {
        Shape box = FrozenBox();
        ShapeSnapshot snapshot = ShapeSnapshot.Of(box);
        List<Shape> edges = SubShapes(box, ShapeType.Edge);
        Shape result;
        using (var overlay = WorkingOverlay.Begin(box))
        {
            Parallel.ForEach(edges, edge => new BrepBuilder().UpdateEdge(ShapeCasts.Edge(edge), 0.125));
            Assert.Equal(12, overlay.WorkingCount);
            Assert.All(edges, edge => Assert.Equal(0.125, Tool.Tolerance(ShapeCasts.Edge(edge))));
            result = overlay.Finish(box);
        }

        snapshot.AssertUnchanged(box);
        Assert.All(SubShapes(result, ShapeType.Edge), edge => Assert.Equal(0.125, Tool.Tolerance(ShapeCasts.Edge(edge))));
        Assert.All(SubShapes(box, ShapeType.Vertex), v => Assert.True(Holds(result, v, ShapeType.Vertex)));
    }

    [Fact]
    public void The_context_answers_for_the_callers_shapes()
    {
        Shape box = FrozenBox();
        Vertex v = ShapeCasts.Vertex(SubShapes(box, ShapeType.Vertex)[0]);
        Edge e = ShapeCasts.Edge(SubShapes(box, ShapeType.Edge).Find(s => Holds(s, v, ShapeType.Vertex)));
        Edge other = ShapeCasts.Edge(SubShapes(box, ShapeType.Edge).Find(s => !Holds(s, v, ShapeType.Vertex)));
        var context = new ReShapeBuilder();
        Shape result;
        using (var overlay = WorkingOverlay.Begin(box, context))
        {
            new BrepBuilder().UpdateVertex(v, 0.5);  // in place, no request
            context.Remove(other);                   // a request, keyed by the caller's edge
            result = overlay.Finish(box);
        }

        Assert.True(context.IsRecorded(other));
        Assert.True(context.Value(other).IsNull);
        Assert.True(context.IsRecorded(v));
        Assert.Equal(0.5, Tool.Tolerance(ShapeCasts.Vertex(context.Value(v))));
        Assert.True(Holds(result, context.Value(v), ShapeType.Vertex));
        // an edge above the vertex changed too: recorded as replaced by the result's
        Assert.True(context.IsRecorded(e));
        Assert.True(Holds(result, context.Value(e), ShapeType.Edge));
        Assert.True(Holds(context.Value(e), context.Value(v), ShapeType.Vertex));
    }

    [Fact]
    public void Finish_twice_or_History_before_Finish_is_a_program_error()
    {
        Shape box = FrozenBox();
        using var overlay = WorkingOverlay.Begin(box);
        EngineAssert.Throws<ProgramErrorException>(() => overlay.History());
        overlay.Finish(box);
        EngineAssert.Throws<ProgramErrorException>(() => overlay.Finish(box));
        // after Finish the overlay takes no more writes
        Vertex v = ShapeCasts.Vertex(SubShapes(box, ShapeType.Vertex)[0]);
        EngineAssert.Throws<ImmutableObjectException>(() => new BrepBuilder().UpdateVertex(v, 1.0));
    }

    // The in-place fix on an unfrozen exact copy, as OCCT runs it (the pilot's old path, decision 642, ran it on
    // CopyAtEntry's copy, which unit 10H6 removed; the copy gave back the input's unchanged sub-shapes, so the sharing
    // was compared too: the inputs-unchanged tests check it now).
    private static Shape OnCopy(in Shape shape, ReShapeBuilder? context, Func<Shape, bool> fix)
    {
        Shape copy = TestCopies.ExactCopyForWrite(shape, out _);
        fix(copy);
        copy.TShape()!.FreezeGraph();
        return copy;
    }

    private static Edge CircleOnTop()
    {
        var c = new CircleCurve3D(new Frame3D(new Point3D(0, 0, 10), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 5);
        return TestShapes.Edge(c, 0.0, 2.0 * Math.PI);
    }

    private static string BrepText(in Shape shape)
    {
        var text = new StringWriter();
        BrepTools.Write(shape, text);
        return text.ToString();
    }

    // The pilot stays at level 1 (plan 3.2): on the overlay it gives what the in-place fix gives on a copy, the same BRep
    // text (flags, tolerances, ranges, curves and surfaces).
    [Fact]
    public void The_pilot_gives_what_the_in_place_fix_gives_on_a_copy()
    {
        Face cylinder = TestShapes.Frozen(TestShapes.CylinderFace(5, 10));
        Edge circle = TestShapes.Frozen(CircleOnTop());
        Shape box = FrozenBox();
        Edge boxEdge = ShapeCasts.Edge(SubShapes(box, ShapeType.Edge)[0]);
        Face boxFace = ShapeCasts.Face(SubShapes(box, ShapeType.Face)[0]);
        var fixer = new EdgeFixer();

        void Same(in Shape input, Shape before, Shape after)
        {
            Assert.Equal(BrepText(before), BrepText(after));
        }

        Same(circle, OnCopy(circle, null, e => new EdgeFixer().FixAddPCurve(ShapeCasts.Edge(e), cylinder, false)),
             fixer.AddPCurveFixed(circle, cylinder, false, out _));
        Same(circle, OnCopy(circle, null, e => new EdgeFixer().FixVertexTolerance(ShapeCasts.Edge(e), cylinder)),
             fixer.VertexToleranceFixed(circle, cylinder, out _));
        Same(boxEdge, OnCopy(boxEdge, null, e => new EdgeFixer().FixSameParameter(ShapeCasts.Edge(e), boxFace, 1e-3)),
             fixer.SameParameterFixed(boxEdge, boxFace, out _, 1e-3));
        Same(boxEdge, OnCopy(boxEdge, null, e => new EdgeFixer().FixRemovePCurve(ShapeCasts.Edge(e), boxFace)),
             fixer.RemovePCurveFixed(boxEdge, boxFace, out _));
        Same(box, OnCopy(box, null, s => new ShapeToleranceFixer().LimitTolerance(s, 1e-3, 1e-2)),
             new ShapeToleranceFixer().ToleranceLimited(box, 1e-3, out _, 1e-2));
        Same(box, OnCopy(box, null, s => { new ShapeToleranceFixer().SetTolerance(s, 1e-4, ShapeType.Wire); return true; }),
             new ShapeToleranceFixer().WithTolerance(box, 1e-4, ShapeType.Wire));
    }
}
