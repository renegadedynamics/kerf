// Port of: src/ModelingData/TKBRep/GTests/TopoDS_Builder_Test.cxx
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// OCCT's TopoDS_Builder GTests, with BRepBuilderAPI_MakeVertex and MakeEdge (whose results are frozen).
/// </summary>
public class TopoDS_BuilderTests
{
    private static Vertex MakeVertex(in Point3D theP) => new MakeVertex(theP).Vertex();

    private static Edge MakeEdge(in Point3D theP1, in Point3D theP2) => new MakeEdge(theP1, theP2).Edge();

    private static int Count(in Shape theShape)
    {
        int aCount = 0;
        for (var anIt = new Iterator(theShape); anIt.More(); anIt.Next())
        {
            ++aCount;
        }

        return aCount;
    }

    [Fact]
    public void MakeWire()
    {
        var aBuilder = new Builder();
        Wire aWire = new();
        aBuilder.MakeWire(ref aWire);

        Assert.False(aWire.IsNull);
        Assert.Equal(ShapeType.Wire, aWire.ShapeType());
        Assert.True(aWire.Free());
    }

    [Fact]
    public void MakeShell()
    {
        var aBuilder = new Builder();
        Shell aShell = new();
        aBuilder.MakeShell(ref aShell);

        Assert.False(aShell.IsNull);
        Assert.Equal(ShapeType.Shell, aShell.ShapeType());
    }

    [Fact]
    public void MakeSolid()
    {
        var aBuilder = new Builder();
        Solid aSolid = new();
        aBuilder.MakeSolid(ref aSolid);

        Assert.False(aSolid.IsNull);
        Assert.Equal(ShapeType.Solid, aSolid.ShapeType());
    }

    [Fact]
    public void MakeCompSolid()
    {
        var aBuilder = new Builder();
        CompSolid aCompSolid = new();
        aBuilder.MakeCompSolid(ref aCompSolid);

        Assert.False(aCompSolid.IsNull);
        Assert.Equal(ShapeType.CompSolid, aCompSolid.ShapeType());
    }

    [Fact]
    public void MakeCompound()
    {
        var aBuilder = new Builder();
        Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        Assert.False(aCompound.IsNull);
        Assert.Equal(ShapeType.Compound, aCompound.ShapeType());
    }

    [Fact]
    public void Add_VerticesToCompound()
    {
        var aBuilder = new Builder();
        Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        // Create and add vertices
        Vertex aV1 = MakeVertex(new Point3D(0, 0, 0));
        Vertex aV2 = MakeVertex(new Point3D(1, 0, 0));
        Vertex aV3 = MakeVertex(new Point3D(0, 1, 0));

        aBuilder.Add(aCompound, aV1);
        aBuilder.Add(aCompound, aV2);
        aBuilder.Add(aCompound, aV3);

        Assert.Equal(3, Count(aCompound));
    }

    [Fact]
    public void Add_EdgesToWire()
    {
        var aBuilder = new Builder();
        Wire aWire = new();
        aBuilder.MakeWire(ref aWire);

        // Create edges forming a triangle
        Edge aE1 = MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0));
        Edge aE2 = MakeEdge(new Point3D(1, 0, 0), new Point3D(0.5, 1, 0));
        Edge aE3 = MakeEdge(new Point3D(0.5, 1, 0), new Point3D(0, 0, 0));

        aBuilder.Add(aWire, aE1);
        aBuilder.Add(aWire, aE2);
        aBuilder.Add(aWire, aE3);

        int aCount = 0;
        for (var anIt = new Iterator(aWire); anIt.More(); anIt.Next())
        {
            Assert.Equal(ShapeType.Edge, anIt.Value().ShapeType());
            ++aCount;
        }

        Assert.Equal(3, aCount);
    }

    [Fact]
    public void Remove_FromCompound()
    {
        var aBuilder = new Builder();
        Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        // Add vertices
        Vertex aV1 = MakeVertex(new Point3D(0, 0, 0));
        Vertex aV2 = MakeVertex(new Point3D(1, 0, 0));
        Vertex aV3 = MakeVertex(new Point3D(0, 1, 0));

        aBuilder.Add(aCompound, aV1);
        aBuilder.Add(aCompound, aV2);
        aBuilder.Add(aCompound, aV3);

        // Remove middle vertex
        aBuilder.Remove(aCompound, aV2);

        Assert.Equal(2, Count(aCompound));
    }

    [Fact]
    public void Add_ManyShapes()
    {
        var aBuilder = new Builder();
        Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        // Add 500 vertices
        for (int i = 0; i < 500; ++i)
        {
            Vertex aV = MakeVertex(new Point3D(i, 0, 0));
            aBuilder.Add(aCompound, aV);
        }

        Assert.Equal(500, Count(aCompound));
    }

    [Fact]
    public void TShapeFlags()
    {
        var aBuilder = new Builder();
        Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        // Test initial flags
        Assert.True(aCompound.Free());
        Assert.True(aCompound.Modified());

        // Test setting flags
        aCompound.Checked(true);
        Assert.True(aCompound.Checked());

        aCompound.Closed(true);
        Assert.True(aCompound.Closed());

        aCompound.Infinite(true);
        Assert.True(aCompound.Infinite());

        aCompound.Convex(true);
        Assert.True(aCompound.Convex());
    }

    [Fact]
    public void Remove_FirstChild()
    {
        var aBuilder = new Builder();
        Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        Vertex aV1 = MakeVertex(new Point3D(1, 0, 0));
        Vertex aV2 = MakeVertex(new Point3D(2, 0, 0));
        Vertex aV3 = MakeVertex(new Point3D(3, 0, 0));
        aBuilder.Add(aCompound, aV1);
        aBuilder.Add(aCompound, aV2);
        aBuilder.Add(aCompound, aV3);

        // Remove first vertex
        aBuilder.Remove(aCompound, aV1);

        Assert.Equal(2, Count(aCompound));
    }

    [Fact]
    public void Remove_LastChild()
    {
        var aBuilder = new Builder();
        Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        Vertex aV1 = MakeVertex(new Point3D(1, 0, 0));
        Vertex aV2 = MakeVertex(new Point3D(2, 0, 0));
        Vertex aV3 = MakeVertex(new Point3D(3, 0, 0));
        aBuilder.Add(aCompound, aV1);
        aBuilder.Add(aCompound, aV2);
        aBuilder.Add(aCompound, aV3);

        // Remove last vertex
        aBuilder.Remove(aCompound, aV3);

        Assert.Equal(2, Count(aCompound));
    }

    [Fact]
    public void Remove_AllChildren()
    {
        var aBuilder = new Builder();
        Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        Vertex aV1 = MakeVertex(new Point3D(1, 0, 0));
        Vertex aV2 = MakeVertex(new Point3D(2, 0, 0));
        Vertex aV3 = MakeVertex(new Point3D(3, 0, 0));
        aBuilder.Add(aCompound, aV1);
        aBuilder.Add(aCompound, aV2);
        aBuilder.Add(aCompound, aV3);

        // Remove all
        aBuilder.Remove(aCompound, aV1);
        aBuilder.Remove(aCompound, aV2);
        aBuilder.Remove(aCompound, aV3);

        // Should be empty
        var anIt = new Iterator(aCompound);
        Assert.False(anIt.More());
    }

    [Fact]
    public void Remove_FromWire()
    {
        var aBuilder = new Builder();
        Wire aWire = new();
        aBuilder.MakeWire(ref aWire);

        Edge aE1 = MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0));
        Edge aE2 = MakeEdge(new Point3D(1, 0, 0), new Point3D(1, 1, 0));
        Edge aE3 = MakeEdge(new Point3D(1, 1, 0), new Point3D(0, 0, 0));

        aBuilder.Add(aWire, aE1);
        aBuilder.Add(aWire, aE2);
        aBuilder.Add(aWire, aE3);

        // Remove middle edge
        aBuilder.Remove(aWire, aE2);

        Assert.Equal(2, Count(aWire));
    }

    [Fact]
    public void NestedCompounds()
    {
        var aBuilder = new Builder();
        Compound aOuter = new(), aInner = new();
        aBuilder.MakeCompound(ref aOuter);
        aBuilder.MakeCompound(ref aInner);

        // Add vertices to inner
        aBuilder.Add(aInner, MakeVertex(new Point3D(0, 0, 0)));
        aBuilder.Add(aInner, MakeVertex(new Point3D(1, 0, 0)));

        // Add inner to outer
        aBuilder.Add(aOuter, aInner);

        // Outer should have 1 child (the inner compound)
        int aOuterCount = 0;
        for (var anIt = new Iterator(aOuter); anIt.More(); anIt.Next())
        {
            Assert.Equal(ShapeType.Compound, anIt.Value().ShapeType());
            ++aOuterCount;
        }

        Assert.Equal(1, aOuterCount);

        // Inner should still have 2 vertices
        Assert.Equal(2, aInner.TShape()!.ChildCount);
    }

    [Fact]
    public void Add_SetsModifiedFlag()
    {
        var aBuilder = new Builder();
        Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        // Clear modified flag
        aCompound.TShape()!.Modified(false);
        Assert.False(aCompound.Modified());

        // Add should set modified
        aBuilder.Add(aCompound, MakeVertex(new Point3D(0, 0, 0)));
        Assert.True(aCompound.Modified());
    }

    [Fact]
    public void Remove_SetsModifiedFlag()
    {
        var aBuilder = new Builder();
        Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        Vertex aV = MakeVertex(new Point3D(0, 0, 0));
        aBuilder.Add(aCompound, aV);

        // Clear modified flag
        aCompound.TShape()!.Modified(false);
        Assert.False(aCompound.Modified());

        // Remove should set modified
        aBuilder.Remove(aCompound, aV);
        Assert.True(aCompound.Modified());
    }

    [Fact]
    public void AllTypes()
    {
        var aBuilder = new Builder();

        // Wire
        Wire aWire = new();
        aBuilder.MakeWire(ref aWire);
        Assert.False(aWire.IsNull);
        Assert.Equal(ShapeType.Wire, aWire.ShapeType());
        Assert.True(aWire.Free());

        // Shell
        Shell aShell = new();
        aBuilder.MakeShell(ref aShell);
        Assert.False(aShell.IsNull);
        Assert.Equal(ShapeType.Shell, aShell.ShapeType());
        Assert.True(aShell.Free());

        // Solid
        Solid aSolid = new();
        aBuilder.MakeSolid(ref aSolid);
        Assert.False(aSolid.IsNull);
        Assert.Equal(ShapeType.Solid, aSolid.ShapeType());
        Assert.True(aSolid.Free());

        // CompSolid
        CompSolid aCompSolid = new();
        aBuilder.MakeCompSolid(ref aCompSolid);
        Assert.False(aCompSolid.IsNull);
        Assert.Equal(ShapeType.CompSolid, aCompSolid.ShapeType());
        Assert.True(aCompSolid.Free());

        // Compound
        Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);
        Assert.False(aCompound.IsNull);
        Assert.Equal(ShapeType.Compound, aCompound.ShapeType());
        Assert.True(aCompound.Free());
    }

    [Fact]
    public void MixedShapesInCompound()
    {
        var aBuilder = new Builder();
        Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        // Add different shape types
        aBuilder.Add(aCompound, MakeVertex(new Point3D(0, 0, 0)));
        aBuilder.Add(aCompound, MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)));

        Wire aWire = new();
        aBuilder.MakeWire(ref aWire);
        aBuilder.Add(aCompound, aWire);

        Shell aShell = new();
        aBuilder.MakeShell(ref aShell);
        aBuilder.Add(aCompound, aShell);

        Compound aInner = new();
        aBuilder.MakeCompound(ref aInner);
        aBuilder.Add(aCompound, aInner);

        Assert.Equal(5, aCompound.TShape()!.ChildCount);

        // Verify types via iteration
        int aVertexCount = 0, aEdgeCount = 0, aWireCount = 0, aShellCount = 0, aCompoundCount = 0;
        for (var anIt = new Iterator(aCompound); anIt.More(); anIt.Next())
        {
            switch (anIt.Value().ShapeType())
            {
                case ShapeType.Vertex:
                    ++aVertexCount;
                    break;
                case ShapeType.Edge:
                    ++aEdgeCount;
                    break;
                case ShapeType.Wire:
                    ++aWireCount;
                    break;
                case ShapeType.Shell:
                    ++aShellCount;
                    break;
                case ShapeType.Compound:
                    ++aCompoundCount;
                    break;
                default:
                    break;
            }
        }

        Assert.Equal(1, aVertexCount);
        Assert.Equal(1, aEdgeCount);
        Assert.Equal(1, aWireCount);
        Assert.Equal(1, aShellCount);
        Assert.Equal(1, aCompoundCount);
    }

    // Additions: OCCT's rules and exceptions

    [Fact]
    public void Add_checks_compatibility_and_free_flag()
    {
        var aBuilder = new Builder();
        Wire aWire = new();
        aBuilder.MakeWire(ref aWire);
        Assert.Throws<IncompatibleShapesException>(() => aBuilder.Add(aWire, MakeVertex(new Point3D())));

        // a shape added to another isn't free any more: adding to it throws
        Compound anInner = new(), anOuter = new();
        aBuilder.MakeCompound(ref anInner);
        aBuilder.MakeCompound(ref anOuter);
        aBuilder.Add(anOuter, anInner);
        Assert.False(anInner.Free());
        Assert.Throws<FrozenShapeException>(() => aBuilder.Add(anInner, MakeVertex(new Point3D())));
        Assert.Throws<FrozenShapeException>(() => aBuilder.Remove(anInner, MakeVertex(new Point3D())));

        // self-insertion: the component is made non-free first, so the shape isn't free any more
        Compound aSelf = new();
        aBuilder.MakeCompound(ref aSelf);
        Assert.Throws<FrozenShapeException>(() => aBuilder.Add(aSelf, aSelf));
    }

    [Fact]
    public void Add_composes_the_parents_orientation_and_location()
    {
        var aBuilder = new Builder();
        Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);
        var aLoc = new Placement(Transform3D.Translation(new Vector3D(1, 2, 3)));
        aCompound.Location(aLoc);
        aCompound.Reverse();

        Vertex aV = MakeVertex(new Point3D());
        aBuilder.Add(aCompound, aV);

        // stored relative to the parent: reversed and moved by the inverse location
        Shape aStored = new Iterator(aCompound, false, false).Value();
        Assert.Equal(Orientation.Reversed, aStored.Orientation());
        Assert.Equal(aLoc.Inverted(), aStored.Location());
        // seen through the parent, it is the shape added
        Assert.True(new Iterator(aCompound).Value().IsEqual(aV));
        aBuilder.Remove(aCompound, aV);
        Assert.Equal(0, aCompound.ChildCount);
    }
}
