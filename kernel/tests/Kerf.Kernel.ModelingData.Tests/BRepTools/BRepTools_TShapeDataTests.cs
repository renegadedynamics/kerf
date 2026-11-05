// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// BRepTools.CopyData and CopyFlags (an exact copy of a TShape's data: nothing recomputed), through the tests' exact
/// copy, and IsSameData (a working copy left equal to its original gives back the original), through the working
/// overlay. Until unit 10H6 they tested BRepTools.ExactCopyForWrite and KeepUnchanged, which copy at entry used.
/// </summary>
public class BRepTools_TShapeDataTests
{
    /// <summary>A frozen box whose first face's first edge has a pcurve over its own range [0, 1] (not same range).</summary>
    private static Solid BoxWithARangedPCurve()
    {
        Solid aBox = TestShapes.Box(1, 2, 3);
        Face aF = ShapeCasts.Face(new Explorer(aBox, ShapeType.Face).Current());
        Edge anE = ShapeCasts.Edge(new Explorer(aF, ShapeType.Edge).Current());
        var aB = new BrepBuilder();
        Curve2D aPC = Tool.CurveOnSurface(anE, aF, out _, out _)!;
        aB.UpdateEdge(anE, aPC, aF, global::Kerf.Engine.Foundation.Precision.Confusion());
        aB.Range(anE, aF, 0, 1);
        aB.SameRange(anE, false);
        return TestShapes.Frozen(aBox);
    }

    [Fact]
    public void An_exact_copy_has_the_values_of_the_original()
    {
        Solid aBox = BoxWithARangedPCurve();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        Shape aCopy = TestCopies.ExactCopyForWrite(aBox, out History aH);
        aSnap.AssertUnchanged(aBox);
        Assert.False(aCopy.TShape()!.IsFrozen);
        Assert.False(aCopy.IsSame(aBox));
        aSnap.AssertSameValues(ShapeSnapshot.Of(aCopy));
        for (var anExp = new Explorer(aBox, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Shape aNew = Assert.Single(aH.Modified(anExp.Current()));
            Assert.False(aNew.IsSame(anExp.Current()));
            Assert.Equal(anExp.Current().Placement, aNew.Placement);
        }
    }

    [Fact]
    public void An_unchanged_copy_gives_back_the_original()
    {
        Solid aBox = BoxWithARangedPCurve();
        Edge anE = ShapeCasts.Edge(new Explorer(aBox, ShapeType.Edge).Current());
        using var anOverlay = WorkingOverlay.Begin(aBox);
        new BrepBuilder().SameRange(anE, false); // written (a working copy), but the value it held
        Assert.Equal(1, anOverlay.WorkingCount);
        Shape aKept = anOverlay.Finish(aBox);
        Assert.True(aKept.IsEqual(aBox));
        Assert.False(anOverlay.History().HasModified);
    }

    [Fact]
    public void A_changed_vertex_takes_its_edges_faces_and_parents_with_it()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        Vertex aV = ShapeCasts.Vertex(new Explorer(aBox, ShapeType.Vertex).Current());
        Shape aKept;
        History aH;
        using (var anOverlay = WorkingOverlay.Begin(aBox))
        {
            new BrepBuilder().UpdateVertex(aV, 1e-3);
            aKept = anOverlay.Finish(aBox);
            aH = anOverlay.History();
        }
        Assert.False(aKept.IsSame(aBox));
        // a corner of the box: its 3 edges and 3 faces change; the other 7 vertices, 9 edges and 3 faces are the box's
        (int Kept, int Changed) Count(ShapeType theType)
        {
            var aMine = new IndexedSet<Shape>(ShapeHasher.Instance);
            var aTheirs = new IndexedSet<Shape>(ShapeHasher.Instance);
            Exploration.MapShapes(aKept, theType, aMine);
            Exploration.MapShapes(aBox, theType, aTheirs);
            int aKeptNb = 0, aChangedNb = 0;
            for (int i = 0; i < aTheirs.Count; i++)
            {
                if (aMine.Contains(aTheirs[i]))
                {
                    aKeptNb++;
                }
                else
                {
                    Assert.Single(aH.Modified(aTheirs[i]));
                    aChangedNb++;
                }
            }
            return (aKeptNb, aChangedNb);
        }
        Assert.Equal((7, 1), Count(ShapeType.Vertex));
        Assert.Equal((9, 3), Count(ShapeType.Edge));
        Assert.Equal((3, 3), Count(ShapeType.Face));
        Assert.Equal(1e-3, Tool.Tolerance(ShapeCasts.Vertex(aH.Modified(aV)[0])));
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), Tool.Tolerance(aV));
    }
}
