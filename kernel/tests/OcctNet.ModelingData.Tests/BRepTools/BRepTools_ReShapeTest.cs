// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// Port of: src/ModelingData/TKBRep/GTests/BRepTools_ReShape_Test.cxx (OCCT V8_0_1)

using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// OCCT's BRepTools_ReShape GTests. BRepBuilderAPI_MakeVertex and MakeEdge (unit 3K) are replaced by
/// <see cref="TestShapes"/>, frozen as the makers' results will be. The compounds the GTests build with BRep_Builder are
/// frozen too once built: a caller outside OCCT.NET makes them with <c>TopoDS_Builder.MakeCompound</c>, which freezes.
/// </summary>
public class BRepTools_ReShapeTest
{
    private static TopoDS_Vertex MakeVertex(double theX, double theY, double theZ)
        => TestShapes.Frozen(TestShapes.Vertex(new gp_Pnt(theX, theY, theZ)));

    private static TopoDS_Edge MakeEdge(in TopoDS_Vertex theV1, in TopoDS_Vertex theV2)
        => TestShapes.Frozen(TestShapes.Edge(theV1, theV2));

    private static TopoDS_Compound MakeCompound(params TopoDS_Shape[] theShapes)
    {
        var aBuilder = new BRep_Builder();
        TopoDS_Compound aC = new();
        aBuilder.MakeCompound(ref aC);
        foreach (TopoDS_Shape aS in theShapes)
        {
            aBuilder.Add(aC, aS);
        }

        return TestShapes.Frozen(aC);
    }

    // Value() returns the direct replacement only (one hop); ValueLeaf() follows the
    // chain through intermediate replacements to its terminal shape.
    [Fact]
    public void ValueLeaf_FollowsChainToLeaf()
    {
        TopoDS_Vertex aA = MakeVertex(0, 0, 0);
        TopoDS_Vertex aB = MakeVertex(1, 0, 0);
        TopoDS_Vertex aC = MakeVertex(2, 0, 0);

        var aReShape = new BRepTools_ReShape();
        aReShape.Replace(aA, aB);
        aReShape.Replace(aB, aC);

        Assert.True(aReShape.Value(aA).IsSame(aB), "Value() must yield only the direct replacement");
        Assert.True(aReShape.ValueLeaf(aA).IsSame(aC), "ValueLeaf() must walk the full chain");
        Assert.True(aReShape.ValueLeaf(aB).IsSame(aC));
        Assert.True(aReShape.ValueLeaf(aC).IsSame(aC), "Terminal element is a fixpoint");
    }

    // ValueLeaf() must terminate on an unrecorded shape without touching the map.
    [Fact]
    public void ValueLeaf_UnrecordedShapeIsIdentity()
    {
        TopoDS_Vertex aA = MakeVertex(0, 0, 0);

        var aReShape = new BRepTools_ReShape();
        Assert.True(aReShape.ValueLeaf(aA).IsSame(aA));
    }

    // A chain ending in a removal (null replacement) must surface as a null shape.
    [Fact]
    public void ValueLeaf_ChainEndingInRemoveReturnsNull()
    {
        TopoDS_Vertex aA = MakeVertex(0, 0, 0);
        TopoDS_Vertex aB = MakeVertex(1, 0, 0);

        var aReShape = new BRepTools_ReShape();
        aReShape.Replace(aA, aB);
        aReShape.Remove(aB);

        Assert.True(aReShape.ValueLeaf(aA).IsNull());
    }

    // A direct A->B then B->A creates a cycle in the replacement map; both bindings
    // are recorded, and the DFS in-flight guard inside Apply() handles termination.
    [Fact]
    public void Replace_DirectCycleHandledByApply()
    {
        TopoDS_Vertex aA = MakeVertex(0, 0, 0);
        TopoDS_Vertex aB = MakeVertex(1, 0, 0);

        var aReShape = new BRepTools_ReShape();
        aReShape.Replace(aA, aB);
        aReShape.Replace(aB, aA);

        Assert.True(aReShape.Value(aA).IsSame(aB));
        Assert.True(aReShape.Value(aB).IsSame(aA));

        // Apply() must terminate (DFS in-flight guard) and return a defined shape.
        TopoDS_Shape aResult = aReShape.Apply(aA);
        Assert.False(aResult.IsNull(), "Apply must terminate via the DFS guard");
    }

    // Longer cycles A -> B -> C -> A are accepted at Replace() time and broken at
    // traversal time by the DFS in-flight guard inside Apply().
    [Fact]
    public void Replace_LongerCycleHandledByApply()
    {
        TopoDS_Vertex aA = MakeVertex(0, 0, 0);
        TopoDS_Vertex aB = MakeVertex(1, 0, 0);
        TopoDS_Vertex aC = MakeVertex(2, 0, 0);

        var aReShape = new BRepTools_ReShape();
        aReShape.Replace(aA, aB);
        aReShape.Replace(aB, aC);
        aReShape.Replace(aC, aA);

        Assert.True(aReShape.Value(aA).IsSame(aB));
        Assert.True(aReShape.Value(aB).IsSame(aC));
        Assert.True(aReShape.Value(aC).IsSame(aA));

        // Apply() must terminate (DFS in-flight guard) and return a defined shape.
        TopoDS_Shape aResult = aReShape.Apply(aA);
        Assert.False(aResult.IsNull(), "Apply must terminate via the DFS guard");
    }

    // Apply() must not stack-overflow when a shape's replacement is a compound that
    // transitively contains the original shape as a sub-shape.
    [Fact]
    public void Apply_HandlesStructuralContainmentWithoutCrash()
    {
        TopoDS_Vertex aV1 = MakeVertex(0, 0, 0);
        TopoDS_Vertex aV2 = MakeVertex(1, 0, 0);
        TopoDS_Vertex aV3 = MakeVertex(2, 0, 0);
        TopoDS_Edge anE = MakeEdge(aV1, aV2);

        // Build a compound that contains the edge (plus another shape for flavour).
        TopoDS_Compound aContainer = MakeCompound(anE, aV3);

        var aReShape = new BRepTools_ReShape();
        aReShape.Replace(anE, aContainer); // e -> compound{e, v3}

        TopoDS_Shape aResult = aReShape.Apply(anE);
        Assert.False(aResult.IsNull());
    }

    // Legitimate diamond sharing (the same sub-shape reached via different parent
    // paths) must continue to process normally; the DFS guard must not mistake it
    // for a cycle. Two edges sharing a vertex form the minimal diamond.
    [Fact]
    public void Apply_DiamondSharingIsProcessedCorrectly()
    {
        TopoDS_Vertex aV1 = MakeVertex(0, 0, 0);
        TopoDS_Vertex aV2 = MakeVertex(1, 0, 0);
        TopoDS_Vertex aV3 = MakeVertex(2, 0, 0);
        TopoDS_Vertex aV2Repl = MakeVertex(1.5, 0, 0);

        // aE1 and aE2 both reference aV2 as an endpoint - that's the diamond.
        TopoDS_Edge aE1 = MakeEdge(aV1, aV2);
        TopoDS_Edge aE2 = MakeEdge(aV2, aV3);

        TopoDS_Compound aPair = MakeCompound(aE1, aE2);

        var aReShape = new BRepTools_ReShape();
        aReShape.Replace(aV2, aV2Repl);

        TopoDS_Shape aResult = aReShape.Apply(aPair);
        Assert.False(aResult.IsNull());

        // aV2 must have been substituted everywhere it appeared; aV2Repl must appear;
        // the original aV2 TShape must no longer be present in the result.
        int aNbReplVertices = 0;
        for (var anExp = new TopExp_Explorer(aResult, TopAbs_VERTEX); anExp.More(); anExp.Next())
        {
            Assert.False(ReferenceEquals(anExp.Current().TShape(), aV2.TShape()),
                         "Original v2 must have been replaced on every diamond arm");
            if (ReferenceEquals(anExp.Current().TShape(), aV2Repl.TShape()))
            {
                ++aNbReplVertices;
            }
        }

        Assert.True(aNbReplVertices >= 2, "Replacement must appear on both arms of the diamond");
    }

    // Clear() must drop every binding and reset the map to identity behaviour.
    [Fact]
    public void Clear_DropsAllBindings()
    {
        TopoDS_Vertex aA = MakeVertex(0, 0, 0);
        TopoDS_Vertex aB = MakeVertex(1, 0, 0);

        var aReShape = new BRepTools_ReShape();
        aReShape.Replace(aA, aB);
        Assert.True(aReShape.Value(aA).IsSame(aB));

        aReShape.Clear();
        Assert.True(aReShape.Value(aA).IsSame(aA));
        Assert.True(aReShape.ValueLeaf(aA).IsSame(aA));
    }

    // Re-replacing the same shape must overwrite the previous binding.
    [Fact]
    public void Replace_LastWins()
    {
        TopoDS_Vertex aA = MakeVertex(0, 0, 0);
        TopoDS_Vertex aB = MakeVertex(1, 0, 0);
        TopoDS_Vertex aC = MakeVertex(2, 0, 0);

        var aReShape = new BRepTools_ReShape();
        aReShape.Replace(aA, aB);
        aReShape.Replace(aA, aC); // overwrite

        Assert.True(aReShape.Value(aA).IsSame(aC));
        Assert.True(aReShape.ValueLeaf(aA).IsSame(aC));
    }

    // Replace(X, X) is a no-op - the shape equals itself, no binding should be stored.
    [Fact]
    public void Replace_SelfIsNoOp()
    {
        TopoDS_Vertex aA = MakeVertex(0, 0, 0);

        var aReShape = new BRepTools_ReShape();
        aReShape.Replace(aA, aA);

        Assert.False(aReShape.IsRecorded(aA));
        Assert.True(aReShape.Value(aA).IsSame(aA));
    }

    // A Remove request must propagate through Apply() so that the sub-shape is
    // dropped from the rebuilt parent.
    [Fact]
    public void Apply_RemoveDropsSubShapeFromParent()
    {
        TopoDS_Vertex aV1 = MakeVertex(0, 0, 0);
        TopoDS_Vertex aV2 = MakeVertex(1, 0, 0);
        TopoDS_Vertex aV3 = MakeVertex(2, 0, 0);

        TopoDS_Compound aParent = MakeCompound(aV1, aV2, aV3);

        var aReShape = new BRepTools_ReShape();
        aReShape.Remove(aV2);

        TopoDS_Shape aResult = aReShape.Apply(aParent);
        Assert.False(aResult.IsNull());

        int aNbVerts = 0;
        for (var anIt = new TopoDS_Iterator(aResult); anIt.More(); anIt.Next())
        {
            Assert.False(ReferenceEquals(anIt.Value().TShape(), aV2.TShape()), "v2 should have been removed");
            ++aNbVerts;
        }

        Assert.Equal(2, aNbVerts);
    }

    // An edge appears in two different parents, each parent is rebuilt through Apply,
    // and the edge replacement must propagate to both without triggering the
    // DFS cycle guard.
    [Fact]
    public void Apply_SharedEdgeAcrossTwoParents()
    {
        TopoDS_Vertex aV1 = MakeVertex(0, 0, 0);
        TopoDS_Vertex aV2 = MakeVertex(1, 0, 0);
        TopoDS_Vertex aV3 = MakeVertex(2, 0, 0);
        TopoDS_Edge aE1 = MakeEdge(aV1, aV2);
        TopoDS_Edge aE2 = MakeEdge(aV2, aV3);
        TopoDS_Edge aE1New = MakeEdge(MakeVertex(0.5, 0, 0), aV2);

        // Build two parents both referencing aE1: parentA{aE1, aV3}, parentB{aE1, aE2}.
        TopoDS_Compound aParentA = MakeCompound(aE1, aV3);
        TopoDS_Compound aParentB = MakeCompound(aE1, aE2);

        var aReShape = new BRepTools_ReShape();
        aReShape.Replace(aE1, aE1New);

        TopoDS_Shape aResultA = aReShape.Apply(aParentA);
        TopoDS_Shape aResultB = aReShape.Apply(aParentB);
        Assert.False(aResultA.IsNull());
        Assert.False(aResultB.IsNull());

        bool aFoundNewInA = false, aFoundNewInB = false;
        for (var anExp = new TopExp_Explorer(aResultA, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            Assert.False(ReferenceEquals(anExp.Current().TShape(), aE1.TShape()));
            if (ReferenceEquals(anExp.Current().TShape(), aE1New.TShape()))
            {
                aFoundNewInA = true;
            }
        }

        for (var anExp = new TopExp_Explorer(aResultB, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            Assert.False(ReferenceEquals(anExp.Current().TShape(), aE1.TShape()));
            if (ReferenceEquals(anExp.Current().TShape(), aE1New.TShape()))
            {
                aFoundNewInB = true;
            }
        }

        Assert.True(aFoundNewInA);
        Assert.True(aFoundNewInB);
    }

    // Idempotence: Apply() on a shape that has no bindings must return
    // the shape unchanged.
    [Fact]
    public void Apply_NoBindingsIsIdentity()
    {
        TopoDS_Vertex aV1 = MakeVertex(0, 0, 0);
        TopoDS_Vertex aV2 = MakeVertex(1, 0, 0);
        TopoDS_Edge anE = MakeEdge(aV1, aV2);

        var aReShape = new BRepTools_ReShape();
        TopoDS_Shape aResult = aReShape.Apply(anE);
        Assert.True(aResult.IsSame(anE));
    }

    // Deep structural containment: a replacement compound contains a nested
    // compound that in turn contains the original shape.
    [Fact]
    public void Apply_DeepStructuralContainmentWithoutCrash()
    {
        TopoDS_Vertex aV1 = MakeVertex(0, 0, 0);
        TopoDS_Vertex aV2 = MakeVertex(1, 0, 0);
        TopoDS_Vertex anExtra = MakeVertex(3, 0, 0);
        TopoDS_Edge anE = MakeEdge(aV1, aV2);

        // inner{e}, outer{inner, extra}: e is reachable two levels down.
        TopoDS_Compound anInner = MakeCompound(anE);
        TopoDS_Compound anOuter = MakeCompound(anInner, anExtra);

        var aReShape = new BRepTools_ReShape();
        aReShape.Replace(anE, anOuter); // e -> outer{inner{e}, extra}

        TopoDS_Shape aResult = aReShape.Apply(anE);
        Assert.False(aResult.IsNull());
    }

    // Locations must be tracked when ModeConsiderLocation is enabled.
    [Fact]
    public void ValueLeaf_ConsidersLocationMode()
    {
        TopoDS_Vertex aA = MakeVertex(0, 0, 0);
        TopoDS_Vertex aB = MakeVertex(1, 0, 0);

        var aReShape = new BRepTools_ReShape();
        aReShape.ModeConsiderLocation(true); // OCCT: ModeConsiderLocation() = true
        aReShape.Replace(aA, aB);

        Assert.True(aReShape.ValueLeaf(aA).IsSame(aB));
    }
}
