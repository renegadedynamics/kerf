// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// BRepTools.ExactCopyForWrite (an exact structural copy: nothing recomputed) and BRepTools.KeepUnchanged (the copy's
/// unchanged parts given back as the original's own).
/// </summary>
public class BRepTools_ExactCopyTests
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
        Shape aCopy = BrepTools.ExactCopyForWrite(aBox, out History aH);
        aSnap.AssertUnchanged(aBox);
        Assert.False(aCopy.TShape()!.IsFrozen);
        Assert.False(aCopy.IsSame(aBox));
        aSnap.AssertSameValues(ShapeSnapshot.Of(aCopy));
        for (var anExp = new Explorer(aBox, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Shape aNew = Assert.Single(aH.Modified(anExp.Current()));
            Assert.False(aNew.IsSame(anExp.Current()));
            Assert.Equal(anExp.Current().Location(), aNew.Location());
        }
    }

    [Fact]
    public void An_unchanged_copy_gives_back_the_original()
    {
        Solid aBox = BoxWithARangedPCurve();
        Shape aCopy = BrepTools.ExactCopyForWrite(aBox, out _);
        Shape aKept = BrepTools.KeepUnchanged(aBox, aCopy, out History aH);
        Assert.True(aKept.IsEqual(aBox));
        Assert.False(aH.HasModified);
    }

    [Fact]
    public void A_changed_vertex_takes_its_edges_faces_and_parents_with_it()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        Shape aCopy = BrepTools.ExactCopyForWrite(aBox, out History aCopyH);
        Vertex aV = ShapeCasts.Vertex(new Explorer(aBox, ShapeType.Vertex).Current());
        Vertex aVCopy = ShapeCasts.Vertex(aCopyH.Modified(aV)[0]);
        new BrepBuilder().UpdateVertex(aVCopy, 1e-3);

        Shape aKept = BrepTools.KeepUnchanged(aBox, aCopy, out History aH);
        Assert.False(aKept.IsSame(aBox));
        // a corner of the box: its 3 edges and 3 faces change; the other 7 vertices, 9 edges and 3 faces are the box's
        (int Kept, int Changed) Count(ShapeType theType)
        {
            var aMine = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
            var aTheirs = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
            Exploration.MapShapes(aKept, theType, aMine);
            Exploration.MapShapes(aBox, theType, aTheirs);
            int aKeptNb = 0, aChangedNb = 0;
            for (int i = 1; i <= aTheirs.Extent(); i++)
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
