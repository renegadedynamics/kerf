// Port of: src/ModelingAlgorithms/TKShHealing/GTests/ShapeBuild_ReShape_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>OCCT's GTests of ShapeBuild_ReShape (and the ShapeFix_Shape stability test of the same file).</summary>
public class ShapeBuild_ReShapeTest
{
    private static int CountSubShapes(in Shape theShape, ShapeType theType)
    {
        var aMap = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, theType, aMap);
        return aMap.Extent();
    }

    // Basic Apply() via ShapeBuild_ReShape (the subclass used throughout shape
    // healing) must produce the expected vertex substitution on a compound.
    [Fact]
    public void Apply_PerformsVertexReplacementOnCompound()
    {
        Vertex aV1 = new MakeVertex(new Point3D(0, 0, 0)).Vertex();
        Vertex aV2 = new MakeVertex(new Point3D(1, 0, 0)).Vertex();
        Vertex aV2Repl = new MakeVertex(new Point3D(1.5, 0, 0)).Vertex();

        Compound aParent = new();
        var aBuilder = new BrepBuilder();
        aBuilder.MakeCompound(ref aParent);
        aBuilder.Add(aParent, aV1);
        aBuilder.Add(aParent, aV2);

        var aReShape = new ReShapeBuilder();
        aReShape.Replace(aV2, aV2Repl);

        Shape aResult = aReShape.Apply(aParent);
        Assert.False(aResult.IsNull);

        bool aFoundReplacement = false;
        for (var anIt = new Iterator(aResult); anIt.More(); anIt.Next())
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
        Vertex aV1 = new MakeVertex(new Point3D(0, 0, 0)).Vertex();
        Vertex aV2 = new MakeVertex(new Point3D(1, 0, 0)).Vertex();
        Vertex aV3 = new MakeVertex(new Point3D(2, 0, 0)).Vertex();
        Edge anE = new MakeEdge(aV1, aV2).Edge();

        Compound aContainer = new();
        var aBuilder = new BrepBuilder();
        aBuilder.MakeCompound(ref aContainer);
        aBuilder.Add(aContainer, anE);
        aBuilder.Add(aContainer, aV3);

        var aReShape = new ReShapeBuilder();
        aReShape.Replace(anE, aContainer);

        Shape aResult = aReShape.Apply(anE);
        Assert.False(aResult.IsNull);
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
        var aMakeBox = new MakeBox(10.0, 10.0, 10.0);
        Shape aBox = aMakeBox.Shape();
        Assert.True(aMakeBox.IsDone);

        int aNbEdgesInitial = CountSubShapes(aBox, ShapeType.Edge);
        int aNbVerticesInitial = CountSubShapes(aBox, ShapeType.Vertex);
        int aNbFacesInitial = CountSubShapes(aBox, ShapeType.Face);
        Assert.Equal(12, aNbEdgesInitial);

        Shape aCurrent = aBox;
        for (int anIter = 0; anIter < 5; ++anIter)
        {
            var aFixer = new ShapeFixer(aCurrent);
            aFixer.Perform();
            aCurrent = aFixer.Shape();
            Assert.False(aCurrent.IsNull, $"iteration {anIter}");

            // Counts must remain identical across every iteration - no multiplication,
            // no loss of shapes on a valid input.
            Assert.Equal(aNbFacesInitial, CountSubShapes(aCurrent, ShapeType.Face));
            Assert.Equal(aNbEdgesInitial, CountSubShapes(aCurrent, ShapeType.Edge));
            Assert.Equal(aNbVerticesInitial, CountSubShapes(aCurrent, ShapeType.Vertex));
        }
    }

    // Diamond sharing through ShapeBuild_ReShape: two compounds share a vertex
    // whose replacement is registered. Both parents must pick up the replacement
    // and the DFS guard must not mistake the shared reference for a cycle.
    [Fact]
    public void Apply_DiamondSharedVertexInTwoCompounds()
    {
        Vertex aShared = new MakeVertex(new Point3D(0, 0, 0)).Vertex();
        Vertex aSharedNew = new MakeVertex(new Point3D(0.1, 0, 0)).Vertex();
        Vertex anOther1 = new MakeVertex(new Point3D(1, 0, 0)).Vertex();
        Vertex anOther2 = new MakeVertex(new Point3D(2, 0, 0)).Vertex();

        Compound aParentA = new(), aParentB = new(), aGrand = new();
        var aBuilder = new BrepBuilder();
        aBuilder.MakeCompound(ref aParentA);
        aBuilder.Add(aParentA, aShared);
        aBuilder.Add(aParentA, anOther1);
        aBuilder.MakeCompound(ref aParentB);
        aBuilder.Add(aParentB, aShared);
        aBuilder.Add(aParentB, anOther2);
        aBuilder.MakeCompound(ref aGrand);
        aBuilder.Add(aGrand, aParentA);
        aBuilder.Add(aGrand, aParentB);

        var aReShape = new ReShapeBuilder();
        aReShape.Replace(aShared, aSharedNew);

        Shape aResult = aReShape.Apply(aGrand);
        Assert.False(aResult.IsNull);

        int aNbReplVerts = 0;
        int aNbOriginals = 0;
        for (var anExp = new Explorer(aResult, ShapeType.Vertex); anExp.More(); anExp.Next())
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
