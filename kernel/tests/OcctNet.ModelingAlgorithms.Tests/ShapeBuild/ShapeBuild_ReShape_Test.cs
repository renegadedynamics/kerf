// Port of: src/ModelingAlgorithms/TKShHealing/GTests/ShapeBuild_ReShape_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>OCCT's GTests of ShapeBuild_ReShape (and the ShapeFix_Shape stability test of the same file).</summary>
public class ShapeBuild_ReShapeTest
{
    private static int CountSubShapes(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, theType, aMap);
        return aMap.Extent();
    }

    // Basic Apply() via ShapeBuild_ReShape (the subclass used throughout shape
    // healing) must produce the expected vertex substitution on a compound.
    [Fact]
    public void Apply_PerformsVertexReplacementOnCompound()
    {
        TopoDS_Vertex aV1 = new BRepBuilderAPI_MakeVertex(new Point3D(0, 0, 0)).Vertex();
        TopoDS_Vertex aV2 = new BRepBuilderAPI_MakeVertex(new Point3D(1, 0, 0)).Vertex();
        TopoDS_Vertex aV2Repl = new BRepBuilderAPI_MakeVertex(new Point3D(1.5, 0, 0)).Vertex();

        TopoDS_Compound aParent = new();
        var aBuilder = new BRep_Builder();
        aBuilder.MakeCompound(ref aParent);
        aBuilder.Add(aParent, aV1);
        aBuilder.Add(aParent, aV2);

        var aReShape = new ShapeBuild_ReShape();
        aReShape.Replace(aV2, aV2Repl);

        TopoDS_Shape aResult = aReShape.Apply(aParent);
        Assert.False(aResult.IsNull());

        bool aFoundReplacement = false;
        for (var anIt = new TopoDS_Iterator(aResult); anIt.More(); anIt.Next())
        {
            if (ReferenceEquals(anIt.Value().TShape(), aV2Repl.TShape()))
            {
                aFoundReplacement = true;
            }
            Assert.False(ReferenceEquals(anIt.Value().TShape(), aV2.TShape()));
        }
        Assert.True(aFoundReplacement);
    }

    // Structural containment through ShapeBuild_ReShape's Apply override.
    // The base class has its own DFS guard, the subclass has its own applyImpl;
    // both paths must survive cyclic containment without a stack overflow.
    [Fact]
    public void Apply_StructuralContainmentWithoutCrash()
    {
        TopoDS_Vertex aV1 = new BRepBuilderAPI_MakeVertex(new Point3D(0, 0, 0)).Vertex();
        TopoDS_Vertex aV2 = new BRepBuilderAPI_MakeVertex(new Point3D(1, 0, 0)).Vertex();
        TopoDS_Vertex aV3 = new BRepBuilderAPI_MakeVertex(new Point3D(2, 0, 0)).Vertex();
        TopoDS_Edge anE = new BRepBuilderAPI_MakeEdge(aV1, aV2).Edge();

        TopoDS_Compound aContainer = new();
        var aBuilder = new BRep_Builder();
        aBuilder.MakeCompound(ref aContainer);
        aBuilder.Add(aContainer, anE);
        aBuilder.Add(aContainer, aV3);

        var aReShape = new ShapeBuild_ReShape();
        aReShape.Replace(anE, aContainer);

        TopoDS_Shape aResult = aReShape.Apply(anE);
        Assert.False(aResult.IsNull());
    }

    // Regression test for the originally reported symptom: running shape healing
    // repeatedly on the same shape must not cause the edge count to grow.
    // (ShapeFix_ShapeStabilityTest in OCCT's file.)
    // Before the UpdateWire/Apply fixes, repeated ShapeFix_Shape::Perform on
    // shapes with shared sub-shapes produced geometrically growing edge counts
    // from cascading replacement chains.
    [Fact]
    public void RepeatedPerformDoesNotMultiplyEdges()
    {
        var aMakeBox = new BRepPrimAPI_MakeBox(10.0, 10.0, 10.0);
        TopoDS_Shape aBox = aMakeBox.Shape();
        Assert.True(aMakeBox.IsDone());

        int aNbEdgesInitial = CountSubShapes(aBox, TopAbs_EDGE);
        int aNbVerticesInitial = CountSubShapes(aBox, TopAbs_VERTEX);
        int aNbFacesInitial = CountSubShapes(aBox, TopAbs_FACE);
        Assert.Equal(12, aNbEdgesInitial);

        TopoDS_Shape aCurrent = aBox;
        for (int anIter = 0; anIter < 5; ++anIter)
        {
            var aFixer = new ShapeFix_Shape(aCurrent);
            aFixer.Perform();
            aCurrent = aFixer.Shape();
            Assert.False(aCurrent.IsNull(), $"iteration {anIter}");

            // Counts must remain identical across every iteration - no multiplication,
            // no loss of shapes on a valid input.
            Assert.Equal(aNbFacesInitial, CountSubShapes(aCurrent, TopAbs_FACE));
            Assert.Equal(aNbEdgesInitial, CountSubShapes(aCurrent, TopAbs_EDGE));
            Assert.Equal(aNbVerticesInitial, CountSubShapes(aCurrent, TopAbs_VERTEX));
        }
    }

    // Diamond sharing through ShapeBuild_ReShape: two compounds share a vertex
    // whose replacement is registered. Both parents must pick up the replacement
    // and the DFS guard must not mistake the shared reference for a cycle.
    [Fact]
    public void Apply_DiamondSharedVertexInTwoCompounds()
    {
        TopoDS_Vertex aShared = new BRepBuilderAPI_MakeVertex(new Point3D(0, 0, 0)).Vertex();
        TopoDS_Vertex aSharedNew = new BRepBuilderAPI_MakeVertex(new Point3D(0.1, 0, 0)).Vertex();
        TopoDS_Vertex anOther1 = new BRepBuilderAPI_MakeVertex(new Point3D(1, 0, 0)).Vertex();
        TopoDS_Vertex anOther2 = new BRepBuilderAPI_MakeVertex(new Point3D(2, 0, 0)).Vertex();

        TopoDS_Compound aParentA = new(), aParentB = new(), aGrand = new();
        var aBuilder = new BRep_Builder();
        aBuilder.MakeCompound(ref aParentA);
        aBuilder.Add(aParentA, aShared);
        aBuilder.Add(aParentA, anOther1);
        aBuilder.MakeCompound(ref aParentB);
        aBuilder.Add(aParentB, aShared);
        aBuilder.Add(aParentB, anOther2);
        aBuilder.MakeCompound(ref aGrand);
        aBuilder.Add(aGrand, aParentA);
        aBuilder.Add(aGrand, aParentB);

        var aReShape = new ShapeBuild_ReShape();
        aReShape.Replace(aShared, aSharedNew);

        TopoDS_Shape aResult = aReShape.Apply(aGrand);
        Assert.False(aResult.IsNull());

        int aNbReplVerts = 0;
        int aNbOriginals = 0;
        for (var anExp = new TopExp_Explorer(aResult, TopAbs_VERTEX); anExp.More(); anExp.Next())
        {
            if (ReferenceEquals(anExp.Current().TShape(), aSharedNew.TShape()))
            {
                ++aNbReplVerts;
            }
            if (ReferenceEquals(anExp.Current().TShape(), aShared.TShape()))
            {
                ++aNbOriginals;
            }
        }
        Assert.Equal(0, aNbOriginals); // Original vertex must be substituted everywhere
        Assert.Equal(2, aNbReplVerts); // Replacement must appear in both parentA and parentB
    }
}
