// Port of: src/ModelingData/TKBRep/GTests/TopoDS_Builder_Test.cxx
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// OCCT's TopoDS_Builder GTests, with BRepBuilderAPI_MakeVertex and MakeEdge (whose results are frozen).
/// </summary>
public class TopoDS_BuilderTests
{
    private static TopoDS_Vertex MakeVertex(in gp_Pnt theP) => new BRepBuilderAPI_MakeVertex(theP).Vertex();

    private static TopoDS_Edge MakeEdge(in gp_Pnt theP1, in gp_Pnt theP2) => new BRepBuilderAPI_MakeEdge(theP1, theP2).Edge();

    private static int Count(in TopoDS_Shape theShape)
    {
        int aCount = 0;
        for (var anIt = new TopoDS_Iterator(theShape); anIt.More(); anIt.Next())
        {
            ++aCount;
        }

        return aCount;
    }

    [Fact]
    public void MakeWire()
    {
        var aBuilder = new TopoDS_Builder();
        TopoDS_Wire aWire = new();
        aBuilder.MakeWire(ref aWire);

        Assert.False(aWire.IsNull());
        Assert.Equal(TopAbs_WIRE, aWire.ShapeType());
        Assert.True(aWire.Free());
    }

    [Fact]
    public void MakeShell()
    {
        var aBuilder = new TopoDS_Builder();
        TopoDS_Shell aShell = new();
        aBuilder.MakeShell(ref aShell);

        Assert.False(aShell.IsNull());
        Assert.Equal(TopAbs_SHELL, aShell.ShapeType());
    }

    [Fact]
    public void MakeSolid()
    {
        var aBuilder = new TopoDS_Builder();
        TopoDS_Solid aSolid = new();
        aBuilder.MakeSolid(ref aSolid);

        Assert.False(aSolid.IsNull());
        Assert.Equal(TopAbs_SOLID, aSolid.ShapeType());
    }

    [Fact]
    public void MakeCompSolid()
    {
        var aBuilder = new TopoDS_Builder();
        TopoDS_CompSolid aCompSolid = new();
        aBuilder.MakeCompSolid(ref aCompSolid);

        Assert.False(aCompSolid.IsNull());
        Assert.Equal(TopAbs_COMPSOLID, aCompSolid.ShapeType());
    }

    [Fact]
    public void MakeCompound()
    {
        var aBuilder = new TopoDS_Builder();
        TopoDS_Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        Assert.False(aCompound.IsNull());
        Assert.Equal(TopAbs_COMPOUND, aCompound.ShapeType());
    }

    [Fact]
    public void Add_VerticesToCompound()
    {
        var aBuilder = new TopoDS_Builder();
        TopoDS_Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        // Create and add vertices
        TopoDS_Vertex aV1 = MakeVertex(new gp_Pnt(0, 0, 0));
        TopoDS_Vertex aV2 = MakeVertex(new gp_Pnt(1, 0, 0));
        TopoDS_Vertex aV3 = MakeVertex(new gp_Pnt(0, 1, 0));

        aBuilder.Add(aCompound, aV1);
        aBuilder.Add(aCompound, aV2);
        aBuilder.Add(aCompound, aV3);

        Assert.Equal(3, Count(aCompound));
    }

    [Fact]
    public void Add_EdgesToWire()
    {
        var aBuilder = new TopoDS_Builder();
        TopoDS_Wire aWire = new();
        aBuilder.MakeWire(ref aWire);

        // Create edges forming a triangle
        TopoDS_Edge aE1 = MakeEdge(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0));
        TopoDS_Edge aE2 = MakeEdge(new gp_Pnt(1, 0, 0), new gp_Pnt(0.5, 1, 0));
        TopoDS_Edge aE3 = MakeEdge(new gp_Pnt(0.5, 1, 0), new gp_Pnt(0, 0, 0));

        aBuilder.Add(aWire, aE1);
        aBuilder.Add(aWire, aE2);
        aBuilder.Add(aWire, aE3);

        int aCount = 0;
        for (var anIt = new TopoDS_Iterator(aWire); anIt.More(); anIt.Next())
        {
            Assert.Equal(TopAbs_EDGE, anIt.Value().ShapeType());
            ++aCount;
        }

        Assert.Equal(3, aCount);
    }

    [Fact]
    public void Remove_FromCompound()
    {
        var aBuilder = new TopoDS_Builder();
        TopoDS_Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        // Add vertices
        TopoDS_Vertex aV1 = MakeVertex(new gp_Pnt(0, 0, 0));
        TopoDS_Vertex aV2 = MakeVertex(new gp_Pnt(1, 0, 0));
        TopoDS_Vertex aV3 = MakeVertex(new gp_Pnt(0, 1, 0));

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
        var aBuilder = new TopoDS_Builder();
        TopoDS_Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        // Add 500 vertices
        for (int i = 0; i < 500; ++i)
        {
            TopoDS_Vertex aV = MakeVertex(new gp_Pnt(i, 0, 0));
            aBuilder.Add(aCompound, aV);
        }

        Assert.Equal(500, Count(aCompound));
    }

    [Fact]
    public void TShapeFlags()
    {
        var aBuilder = new TopoDS_Builder();
        TopoDS_Compound aCompound = new();
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
        var aBuilder = new TopoDS_Builder();
        TopoDS_Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        TopoDS_Vertex aV1 = MakeVertex(new gp_Pnt(1, 0, 0));
        TopoDS_Vertex aV2 = MakeVertex(new gp_Pnt(2, 0, 0));
        TopoDS_Vertex aV3 = MakeVertex(new gp_Pnt(3, 0, 0));
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
        var aBuilder = new TopoDS_Builder();
        TopoDS_Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        TopoDS_Vertex aV1 = MakeVertex(new gp_Pnt(1, 0, 0));
        TopoDS_Vertex aV2 = MakeVertex(new gp_Pnt(2, 0, 0));
        TopoDS_Vertex aV3 = MakeVertex(new gp_Pnt(3, 0, 0));
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
        var aBuilder = new TopoDS_Builder();
        TopoDS_Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        TopoDS_Vertex aV1 = MakeVertex(new gp_Pnt(1, 0, 0));
        TopoDS_Vertex aV2 = MakeVertex(new gp_Pnt(2, 0, 0));
        TopoDS_Vertex aV3 = MakeVertex(new gp_Pnt(3, 0, 0));
        aBuilder.Add(aCompound, aV1);
        aBuilder.Add(aCompound, aV2);
        aBuilder.Add(aCompound, aV3);

        // Remove all
        aBuilder.Remove(aCompound, aV1);
        aBuilder.Remove(aCompound, aV2);
        aBuilder.Remove(aCompound, aV3);

        // Should be empty
        var anIt = new TopoDS_Iterator(aCompound);
        Assert.False(anIt.More());
    }

    [Fact]
    public void Remove_FromWire()
    {
        var aBuilder = new TopoDS_Builder();
        TopoDS_Wire aWire = new();
        aBuilder.MakeWire(ref aWire);

        TopoDS_Edge aE1 = MakeEdge(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0));
        TopoDS_Edge aE2 = MakeEdge(new gp_Pnt(1, 0, 0), new gp_Pnt(1, 1, 0));
        TopoDS_Edge aE3 = MakeEdge(new gp_Pnt(1, 1, 0), new gp_Pnt(0, 0, 0));

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
        var aBuilder = new TopoDS_Builder();
        TopoDS_Compound aOuter = new(), aInner = new();
        aBuilder.MakeCompound(ref aOuter);
        aBuilder.MakeCompound(ref aInner);

        // Add vertices to inner
        aBuilder.Add(aInner, MakeVertex(new gp_Pnt(0, 0, 0)));
        aBuilder.Add(aInner, MakeVertex(new gp_Pnt(1, 0, 0)));

        // Add inner to outer
        aBuilder.Add(aOuter, aInner);

        // Outer should have 1 child (the inner compound)
        int aOuterCount = 0;
        for (var anIt = new TopoDS_Iterator(aOuter); anIt.More(); anIt.Next())
        {
            Assert.Equal(TopAbs_COMPOUND, anIt.Value().ShapeType());
            ++aOuterCount;
        }

        Assert.Equal(1, aOuterCount);

        // Inner should still have 2 vertices
        Assert.Equal(2, aInner.TShape()!.NbChildren());
    }

    [Fact]
    public void Add_SetsModifiedFlag()
    {
        var aBuilder = new TopoDS_Builder();
        TopoDS_Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        // Clear modified flag
        aCompound.TShape()!.Modified(false);
        Assert.False(aCompound.Modified());

        // Add should set modified
        aBuilder.Add(aCompound, MakeVertex(new gp_Pnt(0, 0, 0)));
        Assert.True(aCompound.Modified());
    }

    [Fact]
    public void Remove_SetsModifiedFlag()
    {
        var aBuilder = new TopoDS_Builder();
        TopoDS_Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        TopoDS_Vertex aV = MakeVertex(new gp_Pnt(0, 0, 0));
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
        var aBuilder = new TopoDS_Builder();

        // Wire
        TopoDS_Wire aWire = new();
        aBuilder.MakeWire(ref aWire);
        Assert.False(aWire.IsNull());
        Assert.Equal(TopAbs_WIRE, aWire.ShapeType());
        Assert.True(aWire.Free());

        // Shell
        TopoDS_Shell aShell = new();
        aBuilder.MakeShell(ref aShell);
        Assert.False(aShell.IsNull());
        Assert.Equal(TopAbs_SHELL, aShell.ShapeType());
        Assert.True(aShell.Free());

        // Solid
        TopoDS_Solid aSolid = new();
        aBuilder.MakeSolid(ref aSolid);
        Assert.False(aSolid.IsNull());
        Assert.Equal(TopAbs_SOLID, aSolid.ShapeType());
        Assert.True(aSolid.Free());

        // CompSolid
        TopoDS_CompSolid aCompSolid = new();
        aBuilder.MakeCompSolid(ref aCompSolid);
        Assert.False(aCompSolid.IsNull());
        Assert.Equal(TopAbs_COMPSOLID, aCompSolid.ShapeType());
        Assert.True(aCompSolid.Free());

        // Compound
        TopoDS_Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);
        Assert.False(aCompound.IsNull());
        Assert.Equal(TopAbs_COMPOUND, aCompound.ShapeType());
        Assert.True(aCompound.Free());
    }

    [Fact]
    public void MixedShapesInCompound()
    {
        var aBuilder = new TopoDS_Builder();
        TopoDS_Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        // Add different shape types
        aBuilder.Add(aCompound, MakeVertex(new gp_Pnt(0, 0, 0)));
        aBuilder.Add(aCompound, MakeEdge(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0)));

        TopoDS_Wire aWire = new();
        aBuilder.MakeWire(ref aWire);
        aBuilder.Add(aCompound, aWire);

        TopoDS_Shell aShell = new();
        aBuilder.MakeShell(ref aShell);
        aBuilder.Add(aCompound, aShell);

        TopoDS_Compound aInner = new();
        aBuilder.MakeCompound(ref aInner);
        aBuilder.Add(aCompound, aInner);

        Assert.Equal(5, aCompound.TShape()!.NbChildren());

        // Verify types via iteration
        int aVertexCount = 0, aEdgeCount = 0, aWireCount = 0, aShellCount = 0, aCompoundCount = 0;
        for (var anIt = new TopoDS_Iterator(aCompound); anIt.More(); anIt.Next())
        {
            switch (anIt.Value().ShapeType())
            {
                case TopAbs_VERTEX:
                    ++aVertexCount;
                    break;
                case TopAbs_EDGE:
                    ++aEdgeCount;
                    break;
                case TopAbs_WIRE:
                    ++aWireCount;
                    break;
                case TopAbs_SHELL:
                    ++aShellCount;
                    break;
                case TopAbs_COMPOUND:
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
        var aBuilder = new TopoDS_Builder();
        TopoDS_Wire aWire = new();
        aBuilder.MakeWire(ref aWire);
        Assert.Throws<TopoDS_UnCompatibleShapes>(() => aBuilder.Add(aWire, MakeVertex(new gp_Pnt())));

        // a shape added to another isn't free any more: adding to it throws
        TopoDS_Compound anInner = new(), anOuter = new();
        aBuilder.MakeCompound(ref anInner);
        aBuilder.MakeCompound(ref anOuter);
        aBuilder.Add(anOuter, anInner);
        Assert.False(anInner.Free());
        Assert.Throws<TopoDS_FrozenShape>(() => aBuilder.Add(anInner, MakeVertex(new gp_Pnt())));
        Assert.Throws<TopoDS_FrozenShape>(() => aBuilder.Remove(anInner, MakeVertex(new gp_Pnt())));

        // self-insertion: the component is made non-free first, so the shape isn't free any more
        TopoDS_Compound aSelf = new();
        aBuilder.MakeCompound(ref aSelf);
        Assert.Throws<TopoDS_FrozenShape>(() => aBuilder.Add(aSelf, aSelf));
    }

    [Fact]
    public void Add_composes_the_parents_orientation_and_location()
    {
        var aBuilder = new TopoDS_Builder();
        TopoDS_Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);
        var aLoc = new TopLoc_Location(gp_Trsf.Translation(new gp_Vec(1, 2, 3)));
        aCompound.Location(aLoc);
        aCompound.Reverse();

        TopoDS_Vertex aV = MakeVertex(new gp_Pnt());
        aBuilder.Add(aCompound, aV);

        // stored relative to the parent: reversed and moved by the inverse location
        TopoDS_Shape aStored = new TopoDS_Iterator(aCompound, false, false).Value();
        Assert.Equal(TopAbs_Orientation.TopAbs_REVERSED, aStored.Orientation());
        Assert.Equal(aLoc.Inverted(), aStored.Location());
        // seen through the parent, it is the shape added
        Assert.True(new TopoDS_Iterator(aCompound).Value().IsEqual(aV));
        aBuilder.Remove(aCompound, aV);
        Assert.Equal(0, aCompound.NbChildren());
    }
}
