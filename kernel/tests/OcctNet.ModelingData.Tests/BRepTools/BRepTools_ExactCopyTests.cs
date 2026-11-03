// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// BRepTools.ExactCopyForWrite (an exact structural copy: nothing recomputed) and BRepTools.KeepUnchanged (the copy's
/// unchanged parts given back as the original's own).
/// </summary>
public class BRepTools_ExactCopyTests
{
    /// <summary>A frozen box whose first face's first edge has a pcurve over its own range [0, 1] (not same range).</summary>
    private static TopoDS_Solid BoxWithARangedPCurve()
    {
        TopoDS_Solid aBox = TestShapes.Box(1, 2, 3);
        TopoDS_Face aF = TopoDS.Face(new TopExp_Explorer(aBox, TopAbs_FACE).Current());
        TopoDS_Edge anE = TopoDS.Edge(new TopExp_Explorer(aF, TopAbs_EDGE).Current());
        var aB = new BRep_Builder();
        Geom2d_Curve aPC = BRep_Tool.CurveOnSurface(anE, aF, out _, out _)!;
        aB.UpdateEdge(anE, aPC, aF, global::Kerf.Engine.Foundation.Precision.Confusion());
        aB.Range(anE, aF, 0, 1);
        aB.SameRange(anE, false);
        return TestShapes.Frozen(aBox);
    }

    [Fact]
    public void An_exact_copy_has_the_values_of_the_original()
    {
        TopoDS_Solid aBox = BoxWithARangedPCurve();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        TopoDS_Shape aCopy = BRepTools.ExactCopyForWrite(aBox, out BRepTools_History aH);
        aSnap.AssertUnchanged(aBox);
        Assert.False(aCopy.TShape()!.IsFrozen);
        Assert.False(aCopy.IsSame(aBox));
        aSnap.AssertSameValues(ShapeSnapshot.Of(aCopy));
        for (var anExp = new TopExp_Explorer(aBox, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            TopoDS_Shape aNew = Assert.Single(aH.Modified(anExp.Current()));
            Assert.False(aNew.IsSame(anExp.Current()));
            Assert.Equal(anExp.Current().Location(), aNew.Location());
        }
    }

    [Fact]
    public void An_unchanged_copy_gives_back_the_original()
    {
        TopoDS_Solid aBox = BoxWithARangedPCurve();
        TopoDS_Shape aCopy = BRepTools.ExactCopyForWrite(aBox, out _);
        TopoDS_Shape aKept = BRepTools.KeepUnchanged(aBox, aCopy, out BRepTools_History aH);
        Assert.True(aKept.IsEqual(aBox));
        Assert.False(aH.HasModified());
    }

    [Fact]
    public void A_changed_vertex_takes_its_edges_faces_and_parents_with_it()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        TopoDS_Shape aCopy = BRepTools.ExactCopyForWrite(aBox, out BRepTools_History aCopyH);
        TopoDS_Vertex aV = TopoDS.Vertex(new TopExp_Explorer(aBox, TopAbs_VERTEX).Current());
        TopoDS_Vertex aVCopy = TopoDS.Vertex(aCopyH.Modified(aV)[0]);
        new BRep_Builder().UpdateVertex(aVCopy, 1e-3);

        TopoDS_Shape aKept = BRepTools.KeepUnchanged(aBox, aCopy, out BRepTools_History aH);
        Assert.False(aKept.IsSame(aBox));
        // a corner of the box: its 3 edges and 3 faces change; the other 7 vertices, 9 edges and 3 faces are the box's
        (int Kept, int Changed) Count(TopAbs_ShapeEnum theType)
        {
            var aMine = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            var aTheirs = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            TopExp.MapShapes(aKept, theType, aMine);
            TopExp.MapShapes(aBox, theType, aTheirs);
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
        Assert.Equal((7, 1), Count(TopAbs_VERTEX));
        Assert.Equal((9, 3), Count(TopAbs_EDGE));
        Assert.Equal((3, 3), Count(TopAbs_FACE));
        Assert.Equal(1e-3, BRep_Tool.Tolerance(TopoDS.Vertex(aH.Modified(aV)[0])));
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), BRep_Tool.Tolerance(aV));
    }
}
