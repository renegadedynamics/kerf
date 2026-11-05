// Port of: src/ModelingData/TKBRep/GTests/TopoDS_Iterator_Test.cxx
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// OCCT's TopoDS_Iterator GTests, with BRepBuilderAPI_MakeVertex and MakeEdge and BRepPrimAPI_MakeBox (whose results
/// are frozen).
/// </summary>
public class TopoDS_IteratorTests
{
    private static Vertex MakeVertex(in Point3D theP) => new MakeVertex(theP).Vertex();

    private static Edge MakeEdge(in Point3D theP1, in Point3D theP2) => new MakeEdge(theP1, theP2).Edge();

    private static Shape MakeBox(double dx, double dy, double dz) => new MakeBox(dx, dy, dz).Shape();

    [Fact]
    public void OCC30708_1_InitializeWithNullShape()
    {
        var it = new Iterator();
        Shape empty = new();

        // Should not throw exception when initializing with null shape
        it.Initialize(empty);

        // More() should return false on null shape
        Assert.False(it.More());
    }

    [Fact]
    public void EmptyCompound()
    {
        var aBuilder = new Builder();
        Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        var anIt = new Iterator(aCompound);
        Assert.False(anIt.More());
    }

    [Fact]
    public void CompoundWithVertices()
    {
        var aBuilder = new Builder();
        Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        // Add 5 vertices
        for (int i = 0; i < 5; ++i)
        {
            Vertex aV = MakeVertex(new Point3D(i, 0, 0));
            aBuilder.Add(aCompound, aV);
        }

        // Iterate and count
        int aCount = 0;
        for (var anIt = new Iterator(aCompound); anIt.More(); anIt.Next())
        {
            Assert.Equal(ShapeType.Vertex, anIt.Value().Type);
            ++aCount;
        }

        Assert.Equal(5, aCount);
    }

    [Fact]
    public void WireWithEdges()
    {
        var aBuilder = new Builder();
        Wire aWire = new();
        aBuilder.MakeWire(ref aWire);

        // Create a rectangular wire (4 edges)
        Edge aE1 = MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0));
        Edge aE2 = MakeEdge(new Point3D(1, 0, 0), new Point3D(1, 1, 0));
        Edge aE3 = MakeEdge(new Point3D(1, 1, 0), new Point3D(0, 1, 0));
        Edge aE4 = MakeEdge(new Point3D(0, 1, 0), new Point3D(0, 0, 0));

        aBuilder.Add(aWire, aE1);
        aBuilder.Add(aWire, aE2);
        aBuilder.Add(aWire, aE3);
        aBuilder.Add(aWire, aE4);

        // Iterate and verify
        int aCount = 0;
        for (var anIt = new Iterator(aWire); anIt.More(); anIt.Next())
        {
            Assert.Equal(ShapeType.Edge, anIt.Value().Type);
            ++aCount;
        }

        Assert.Equal(4, aCount);
    }

    [Fact]
    public void EdgeWithVertices()
    {
        Edge anEdge = MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 1, 1));

        int aCount = 0;
        for (var anIt = new Iterator(anEdge); anIt.More(); anIt.Next())
        {
            Assert.Equal(ShapeType.Vertex, anIt.Value().Type);
            ++aCount;
        }

        Assert.Equal(2, aCount);
    }

    [Fact]
    public void VertexNoChildren()
    {
        Vertex aVertex = MakeVertex(new Point3D(0, 0, 0));

        var anIt = new Iterator(aVertex);
        Assert.False(anIt.More());
    }

    [Fact]
    public void CumulativeOrientation()
    {
        var aBuilder = new Builder();
        Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        Vertex aV = MakeVertex(new Point3D(0, 0, 0));
        aBuilder.Add(aCompound, aV);

        // Reverse the compound
        aCompound.Reverse();

        // With cumOri=true (default), child orientation should be composed
        var anIt1 = new Iterator(aCompound, true, true);
        Assert.True(anIt1.More());
        Orientation anOri1 = anIt1.Value().Orientation;

        // With cumOri=false, child orientation should be original
        var anIt2 = new Iterator(aCompound, false, true);
        Assert.True(anIt2.More());
        Orientation anOri2 = anIt2.Value().Orientation;

        // Orientations should differ when compound is reversed
        Assert.NotEqual(anOri1, anOri2);
    }

    [Fact]
    public void ReInitialization()
    {
        var aBuilder = new Builder();
        Compound aCompound1 = new(), aCompound2 = new();
        aBuilder.MakeCompound(ref aCompound1);
        aBuilder.MakeCompound(ref aCompound2);

        // Add different number of vertices
        for (int i = 0; i < 3; ++i)
        {
            aBuilder.Add(aCompound1, MakeVertex(new Point3D(i, 0, 0)));
        }

        for (int i = 0; i < 5; ++i)
        {
            aBuilder.Add(aCompound2, MakeVertex(new Point3D(i, 1, 0)));
        }

        var anIt = new Iterator(aCompound1);

        // Count first compound
        int aCount1 = 0;
        for (; anIt.More(); anIt.Next())
        {
            ++aCount1;
        }

        Assert.Equal(3, aCount1);

        // Re-initialize with second compound
        anIt.Initialize(aCompound2);

        // Count second compound
        int aCount2 = 0;
        for (; anIt.More(); anIt.Next())
        {
            ++aCount2;
        }

        Assert.Equal(5, aCount2);
    }

    [Fact]
    public void BoxSolidIteration()
    {
        // Create a box
        Shape aBox = MakeBox(1.0, 2.0, 3.0);

        // Iterate over solid's children (should be 1 shell)
        int aShellCount = 0;
        for (var anIt = new Iterator(aBox); anIt.More(); anIt.Next())
        {
            Assert.Equal(ShapeType.Shell, anIt.Value().Type);
            ++aShellCount;

            // Iterate over shell's children (should be 6 faces)
            int aFaceCount = 0;
            for (var anIt2 = new Iterator(anIt.Value()); anIt2.More(); anIt2.Next())
            {
                Assert.Equal(ShapeType.Face, anIt2.Value().Type);
                ++aFaceCount;
            }

            Assert.Equal(6, aFaceCount);
        }

        Assert.Equal(1, aShellCount);
    }

    [Fact]
    public void ManyChildren()
    {
        var aBuilder = new Builder();
        Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        const int aNbVertices = 1000;

        // Add many vertices
        for (int i = 0; i < aNbVertices; ++i)
        {
            aBuilder.Add(aCompound, MakeVertex(new Point3D(i, 0, 0)));
        }

        // Iterate and count
        int aCount = 0;
        for (var anIt = new Iterator(aCompound); anIt.More(); anIt.Next())
        {
            ++aCount;
        }

        Assert.Equal(aNbVertices, aCount);
    }

    [Fact]
    public void DefaultConstructorThenInitialize()
    {
        var aBuilder = new Builder();
        Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);
        aBuilder.Add(aCompound, MakeVertex(new Point3D(0, 0, 0)));

        // Default construct, then initialize
        var anIt = new Iterator();
        Assert.False(anIt.More());

        anIt.Initialize(aCompound);
        Assert.True(anIt.More());
        Assert.Equal(ShapeType.Vertex, anIt.Value().Type);
    }

    [Fact]
    public void ValueThrowsWhenNotMore()
    {
        var aBuilder = new Builder();
        Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        var anIt = new Iterator(aCompound);
        Assert.False(anIt.More());

        // Value() should throw Standard_NoSuchObject when !More()
        EngineAssert.ThrowsAny<NoSuchObjectException>(() => anIt.Value());
    }

    [Fact]
    public void CumulativeLocation()
    {
        var aBuilder = new Builder();
        Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        Vertex aV = MakeVertex(new Point3D(0, 0, 0));
        aBuilder.Add(aCompound, aV);

        // Apply a translation to the compound
        Transform3D aTrsf = new Transform3D().WithTranslation(new Vector3D(10, 20, 30));
        var aLoc = new Placement(aTrsf);
        aCompound.Location(aLoc);

        // With cumLoc=true (default), child should have composed location
        var anIt1 = new Iterator(aCompound, true, true);
        Assert.True(anIt1.More());
        Placement aChildLoc1 = anIt1.Value().Placement;

        // With cumLoc=false, child should have original location
        var anIt2 = new Iterator(aCompound, true, false);
        Assert.True(anIt2.More());
        Placement aChildLoc2 = anIt2.Value().Placement;

        // Locations should differ
        Assert.NotEqual(aChildLoc1.IsIdentity, aChildLoc2.IsIdentity);
    }

    [Fact]
    public void NestedCompounds()
    {
        var aBuilder = new Builder();
        Compound aOuter = new(), aInner1 = new(), aInner2 = new();
        aBuilder.MakeCompound(ref aOuter);
        aBuilder.MakeCompound(ref aInner1);
        aBuilder.MakeCompound(ref aInner2);

        // Add vertices to inner compounds
        aBuilder.Add(aInner1, MakeVertex(new Point3D(0, 0, 0)));
        aBuilder.Add(aInner1, MakeVertex(new Point3D(1, 0, 0)));
        aBuilder.Add(aInner2, MakeVertex(new Point3D(2, 0, 0)));

        // Add inner compounds to outer
        aBuilder.Add(aOuter, aInner1);
        aBuilder.Add(aOuter, aInner2);

        // Outer should have 2 children (both compounds)
        int aOuterCount = 0;
        for (var anIt = new Iterator(aOuter); anIt.More(); anIt.Next())
        {
            Assert.Equal(ShapeType.Compound, anIt.Value().Type);
            ++aOuterCount;
        }

        Assert.Equal(2, aOuterCount);

        // First inner should have 2 children
        var anIt0 = new Iterator(aOuter);
        int aInner1Count = 0;
        for (var anIt2 = new Iterator(anIt0.Value()); anIt2.More(); anIt2.Next())
        {
            ++aInner1Count;
        }

        Assert.Equal(2, aInner1Count);
    }

    [Fact]
    public void FaceWithWires()
    {
        Shape aBox = MakeBox(1.0, 1.0, 1.0);

        // Get first face
        var anExp = new Explorer(aBox, ShapeType.Face);
        Assert.True(anExp.More());
        Face aFace = ShapeCasts.Face(anExp.Current());

        // Iterate over face's children (should be wires)
        int aWireCount = 0;
        for (var anIt = new Iterator(aFace); anIt.More(); anIt.Next())
        {
            Assert.Equal(ShapeType.Wire, anIt.Value().Type);
            ++aWireCount;
        }

        Assert.True(aWireCount >= 1);
    }

    [Fact]
    public void ShapeIdentity()
    {
        var aBuilder = new Builder();
        Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        Vertex aV1 = MakeVertex(new Point3D(1, 0, 0));
        Vertex aV2 = MakeVertex(new Point3D(2, 0, 0));
        aBuilder.Add(aCompound, aV1);
        aBuilder.Add(aCompound, aV2);

        // Iterate and check that TShape pointers match
        var anIt = new Iterator(aCompound);
        Assert.True(anIt.Value().TShape() == aV1.TShape() || anIt.Value().TShape() == aV2.TShape());
        anIt.Next();
        Assert.True(anIt.Value().TShape() == aV1.TShape() || anIt.Value().TShape() == aV2.TShape());
    }

    [Fact]
    public void ShellWithFaces()
    {
        Shape aBox = MakeBox(1.0, 1.0, 1.0);

        // Get shell from box
        var anExp = new Explorer(aBox, ShapeType.Shell);
        Assert.True(anExp.More());
        Shell aShell = ShapeCasts.Shell(anExp.Current());

        // Iterate over shell's faces
        int aFaceCount = 0;
        for (var anIt = new Iterator(aShell); anIt.More(); anIt.Next())
        {
            Assert.Equal(ShapeType.Face, anIt.Value().Type);
            ++aFaceCount;
        }

        Assert.Equal(6, aFaceCount);
    }
}
