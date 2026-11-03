// Port of: src/ModelingData/TKBRep/GTests/TopoDS_Iterator_Test.cxx
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// OCCT's TopoDS_Iterator GTests, with BRepBuilderAPI_MakeVertex and MakeEdge and BRepPrimAPI_MakeBox (whose results
/// are frozen).
/// </summary>
public class TopoDS_IteratorTests
{
    private static TopoDS_Vertex MakeVertex(in Point3D theP) => new BRepBuilderAPI_MakeVertex(theP).Vertex();

    private static TopoDS_Edge MakeEdge(in Point3D theP1, in Point3D theP2) => new BRepBuilderAPI_MakeEdge(theP1, theP2).Edge();

    private static TopoDS_Shape MakeBox(double dx, double dy, double dz) => new BRepPrimAPI_MakeBox(dx, dy, dz).Shape();

    [Fact]
    public void OCC30708_1_InitializeWithNullShape()
    {
        var it = new TopoDS_Iterator();
        TopoDS_Shape empty = new();

        // Should not throw exception when initializing with null shape
        it.Initialize(empty);

        // More() should return false on null shape
        Assert.False(it.More());
    }

    [Fact]
    public void EmptyCompound()
    {
        var aBuilder = new TopoDS_Builder();
        TopoDS_Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        var anIt = new TopoDS_Iterator(aCompound);
        Assert.False(anIt.More());
    }

    [Fact]
    public void CompoundWithVertices()
    {
        var aBuilder = new TopoDS_Builder();
        TopoDS_Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        // Add 5 vertices
        for (int i = 0; i < 5; ++i)
        {
            TopoDS_Vertex aV = MakeVertex(new Point3D(i, 0, 0));
            aBuilder.Add(aCompound, aV);
        }

        // Iterate and count
        int aCount = 0;
        for (var anIt = new TopoDS_Iterator(aCompound); anIt.More(); anIt.Next())
        {
            Assert.Equal(TopAbs_VERTEX, anIt.Value().ShapeType());
            ++aCount;
        }

        Assert.Equal(5, aCount);
    }

    [Fact]
    public void WireWithEdges()
    {
        var aBuilder = new TopoDS_Builder();
        TopoDS_Wire aWire = new();
        aBuilder.MakeWire(ref aWire);

        // Create a rectangular wire (4 edges)
        TopoDS_Edge aE1 = MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0));
        TopoDS_Edge aE2 = MakeEdge(new Point3D(1, 0, 0), new Point3D(1, 1, 0));
        TopoDS_Edge aE3 = MakeEdge(new Point3D(1, 1, 0), new Point3D(0, 1, 0));
        TopoDS_Edge aE4 = MakeEdge(new Point3D(0, 1, 0), new Point3D(0, 0, 0));

        aBuilder.Add(aWire, aE1);
        aBuilder.Add(aWire, aE2);
        aBuilder.Add(aWire, aE3);
        aBuilder.Add(aWire, aE4);

        // Iterate and verify
        int aCount = 0;
        for (var anIt = new TopoDS_Iterator(aWire); anIt.More(); anIt.Next())
        {
            Assert.Equal(TopAbs_EDGE, anIt.Value().ShapeType());
            ++aCount;
        }

        Assert.Equal(4, aCount);
    }

    [Fact]
    public void EdgeWithVertices()
    {
        TopoDS_Edge anEdge = MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 1, 1));

        int aCount = 0;
        for (var anIt = new TopoDS_Iterator(anEdge); anIt.More(); anIt.Next())
        {
            Assert.Equal(TopAbs_VERTEX, anIt.Value().ShapeType());
            ++aCount;
        }

        Assert.Equal(2, aCount);
    }

    [Fact]
    public void VertexNoChildren()
    {
        TopoDS_Vertex aVertex = MakeVertex(new Point3D(0, 0, 0));

        var anIt = new TopoDS_Iterator(aVertex);
        Assert.False(anIt.More());
    }

    [Fact]
    public void CumulativeOrientation()
    {
        var aBuilder = new TopoDS_Builder();
        TopoDS_Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        TopoDS_Vertex aV = MakeVertex(new Point3D(0, 0, 0));
        aBuilder.Add(aCompound, aV);

        // Reverse the compound
        aCompound.Reverse();

        // With cumOri=true (default), child orientation should be composed
        var anIt1 = new TopoDS_Iterator(aCompound, true, true);
        Assert.True(anIt1.More());
        TopAbs_Orientation anOri1 = anIt1.Value().Orientation();

        // With cumOri=false, child orientation should be original
        var anIt2 = new TopoDS_Iterator(aCompound, false, true);
        Assert.True(anIt2.More());
        TopAbs_Orientation anOri2 = anIt2.Value().Orientation();

        // Orientations should differ when compound is reversed
        Assert.NotEqual(anOri1, anOri2);
    }

    [Fact]
    public void ReInitialization()
    {
        var aBuilder = new TopoDS_Builder();
        TopoDS_Compound aCompound1 = new(), aCompound2 = new();
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

        var anIt = new TopoDS_Iterator(aCompound1);

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
        TopoDS_Shape aBox = MakeBox(1.0, 2.0, 3.0);

        // Iterate over solid's children (should be 1 shell)
        int aShellCount = 0;
        for (var anIt = new TopoDS_Iterator(aBox); anIt.More(); anIt.Next())
        {
            Assert.Equal(TopAbs_SHELL, anIt.Value().ShapeType());
            ++aShellCount;

            // Iterate over shell's children (should be 6 faces)
            int aFaceCount = 0;
            for (var anIt2 = new TopoDS_Iterator(anIt.Value()); anIt2.More(); anIt2.Next())
            {
                Assert.Equal(TopAbs_FACE, anIt2.Value().ShapeType());
                ++aFaceCount;
            }

            Assert.Equal(6, aFaceCount);
        }

        Assert.Equal(1, aShellCount);
    }

    [Fact]
    public void ManyChildren()
    {
        var aBuilder = new TopoDS_Builder();
        TopoDS_Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        const int aNbVertices = 1000;

        // Add many vertices
        for (int i = 0; i < aNbVertices; ++i)
        {
            aBuilder.Add(aCompound, MakeVertex(new Point3D(i, 0, 0)));
        }

        // Iterate and count
        int aCount = 0;
        for (var anIt = new TopoDS_Iterator(aCompound); anIt.More(); anIt.Next())
        {
            ++aCount;
        }

        Assert.Equal(aNbVertices, aCount);
    }

    [Fact]
    public void DefaultConstructorThenInitialize()
    {
        var aBuilder = new TopoDS_Builder();
        TopoDS_Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);
        aBuilder.Add(aCompound, MakeVertex(new Point3D(0, 0, 0)));

        // Default construct, then initialize
        var anIt = new TopoDS_Iterator();
        Assert.False(anIt.More());

        anIt.Initialize(aCompound);
        Assert.True(anIt.More());
        Assert.Equal(TopAbs_VERTEX, anIt.Value().ShapeType());
    }

    [Fact]
    public void ValueThrowsWhenNotMore()
    {
        var aBuilder = new TopoDS_Builder();
        TopoDS_Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        var anIt = new TopoDS_Iterator(aCompound);
        Assert.False(anIt.More());

        // Value() should throw Standard_NoSuchObject when !More()
        Assert.ThrowsAny<NoSuchObjectException>(() => anIt.Value());
    }

    [Fact]
    public void CumulativeLocation()
    {
        var aBuilder = new TopoDS_Builder();
        TopoDS_Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        TopoDS_Vertex aV = MakeVertex(new Point3D(0, 0, 0));
        aBuilder.Add(aCompound, aV);

        // Apply a translation to the compound
        Transform3D aTrsf = new Transform3D();
        aTrsf.SetTranslation(new Vector3D(10, 20, 30));
        var aLoc = new Placement(aTrsf);
        aCompound.Location(aLoc);

        // With cumLoc=true (default), child should have composed location
        var anIt1 = new TopoDS_Iterator(aCompound, true, true);
        Assert.True(anIt1.More());
        Placement aChildLoc1 = anIt1.Value().Location();

        // With cumLoc=false, child should have original location
        var anIt2 = new TopoDS_Iterator(aCompound, true, false);
        Assert.True(anIt2.More());
        Placement aChildLoc2 = anIt2.Value().Location();

        // Locations should differ
        Assert.NotEqual(aChildLoc1.IsIdentity, aChildLoc2.IsIdentity);
    }

    [Fact]
    public void NestedCompounds()
    {
        var aBuilder = new TopoDS_Builder();
        TopoDS_Compound aOuter = new(), aInner1 = new(), aInner2 = new();
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
        for (var anIt = new TopoDS_Iterator(aOuter); anIt.More(); anIt.Next())
        {
            Assert.Equal(TopAbs_COMPOUND, anIt.Value().ShapeType());
            ++aOuterCount;
        }

        Assert.Equal(2, aOuterCount);

        // First inner should have 2 children
        var anIt0 = new TopoDS_Iterator(aOuter);
        int aInner1Count = 0;
        for (var anIt2 = new TopoDS_Iterator(anIt0.Value()); anIt2.More(); anIt2.Next())
        {
            ++aInner1Count;
        }

        Assert.Equal(2, aInner1Count);
    }

    [Fact]
    public void FaceWithWires()
    {
        TopoDS_Shape aBox = MakeBox(1.0, 1.0, 1.0);

        // Get first face
        var anExp = new TopExp_Explorer(aBox, TopAbs_FACE);
        Assert.True(anExp.More());
        TopoDS_Face aFace = TopoDS.Face(anExp.Current());

        // Iterate over face's children (should be wires)
        int aWireCount = 0;
        for (var anIt = new TopoDS_Iterator(aFace); anIt.More(); anIt.Next())
        {
            Assert.Equal(TopAbs_WIRE, anIt.Value().ShapeType());
            ++aWireCount;
        }

        Assert.True(aWireCount >= 1);
    }

    [Fact]
    public void ShapeIdentity()
    {
        var aBuilder = new TopoDS_Builder();
        TopoDS_Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        TopoDS_Vertex aV1 = MakeVertex(new Point3D(1, 0, 0));
        TopoDS_Vertex aV2 = MakeVertex(new Point3D(2, 0, 0));
        aBuilder.Add(aCompound, aV1);
        aBuilder.Add(aCompound, aV2);

        // Iterate and check that TShape pointers match
        var anIt = new TopoDS_Iterator(aCompound);
        Assert.True(anIt.Value().TShape() == aV1.TShape() || anIt.Value().TShape() == aV2.TShape());
        anIt.Next();
        Assert.True(anIt.Value().TShape() == aV1.TShape() || anIt.Value().TShape() == aV2.TShape());
    }

    [Fact]
    public void ShellWithFaces()
    {
        TopoDS_Shape aBox = MakeBox(1.0, 1.0, 1.0);

        // Get shell from box
        var anExp = new TopExp_Explorer(aBox, TopAbs_SHELL);
        Assert.True(anExp.More());
        TopoDS_Shell aShell = TopoDS.Shell(anExp.Current());

        // Iterate over shell's faces
        int aFaceCount = 0;
        for (var anIt = new TopoDS_Iterator(aShell); anIt.More(); anIt.Next())
        {
            Assert.Equal(TopAbs_FACE, anIt.Value().ShapeType());
            ++aFaceCount;
        }

        Assert.Equal(6, aFaceCount);
    }
}
