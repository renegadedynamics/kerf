// Port of: src/ModelingData/TKBRep/GTests/TopoDS_TShape_Test.cxx
// Port of: src/ModelingData/TKBRep/GTests/TopoDS_Edge_Test.cxx
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// OCCT's TopoDS_TShape and TopoDS_Edge GTests, with BRepBuilderAPI_MakeVertex and MakeEdge (whose results are
/// frozen).
/// </summary>
public class TopoDS_TShapeTests
{
    private static Vertex MakeVertex(in Point3D theP) => new MakeVertex(theP).Vertex();

    private static Edge MakeEdge(in Point3D theP1, in Point3D theP2) => new MakeEdge(theP1, theP2).Edge();

    [Fact]
    public void ShapeType_AllTypes()
    {
        var aBuilder = new Builder();

        // Create various shapes and verify ShapeType
        Vertex aVertex = MakeVertex(new Point3D(0, 0, 0));
        Assert.Equal(ShapeType.Vertex, aVertex.TShape()!.ShapeType());

        Edge anEdge = MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0));
        Assert.Equal(ShapeType.Edge, anEdge.TShape()!.ShapeType());

        Wire aWire = new();
        aBuilder.MakeWire(ref aWire);
        Assert.Equal(ShapeType.Wire, aWire.TShape()!.ShapeType());

        Shell aShell = new();
        aBuilder.MakeShell(ref aShell);
        Assert.Equal(ShapeType.Shell, aShell.TShape()!.ShapeType());

        Solid aSolid = new();
        aBuilder.MakeSolid(ref aSolid);
        Assert.Equal(ShapeType.Solid, aSolid.TShape()!.ShapeType());

        CompSolid aCompSolid = new();
        aBuilder.MakeCompSolid(ref aCompSolid);
        Assert.Equal(ShapeType.CompSolid, aCompSolid.TShape()!.ShapeType());

        Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);
        Assert.Equal(ShapeType.Compound, aCompound.TShape()!.ShapeType());
    }

    [Fact]
    public void FlagSettersGetters()
    {
        var aBuilder = new Builder();
        Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        TShape aTShape = aCompound.TShape()!;

        // Test Free flag
        Assert.True(aTShape.Free());
        aTShape.Free(false);
        Assert.False(aTShape.Free());
        aTShape.Free(true);
        Assert.True(aTShape.Free());

        // Test Modified flag
        Assert.True(aTShape.Modified());
        aTShape.Modified(false);
        Assert.False(aTShape.Modified());

        // Test Checked flag
        Assert.False(aTShape.Checked());
        aTShape.Checked(true);
        Assert.True(aTShape.Checked());

        // Test that setting Modified(true) clears Checked
        aTShape.Modified(true);
        Assert.False(aTShape.Checked());

        // Test Orientable flag
        Assert.False(aTShape.Orientable());

        // Test Closed flag
        Assert.False(aTShape.Closed());
        aTShape.Closed(true);
        Assert.True(aTShape.Closed());

        // Test Infinite flag
        Assert.False(aTShape.Infinite());
        aTShape.Infinite(true);
        Assert.True(aTShape.Infinite());

        // Test Convex flag
        Assert.False(aTShape.Convex());
        aTShape.Convex(true);
        Assert.True(aTShape.Convex());

        // Test Locked flag
        Assert.False(aTShape.Locked());
        aTShape.Locked(true);
        Assert.True(aTShape.Locked());
    }

    [Fact]
    public void NbChildren()
    {
        var aBuilder = new Builder();

        // Vertex has 0 children
        Vertex aVertex = MakeVertex(new Point3D(0, 0, 0));
        Assert.Equal(0, aVertex.TShape()!.ChildCount);

        // Edge has 2 children (vertices)
        Edge anEdge = MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0));
        Assert.Equal(2, anEdge.TShape()!.ChildCount);

        // Empty wire has 0 children
        Wire aWire = new();
        aBuilder.MakeWire(ref aWire);
        Assert.Equal(0, aWire.TShape()!.ChildCount);

        // Wire with edges
        aBuilder.Add(aWire, anEdge);
        Assert.Equal(1, aWire.TShape()!.ChildCount);

        // Compound with multiple children
        Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);
        for (int i = 0; i < 10; ++i)
        {
            aBuilder.Add(aCompound, MakeVertex(new Point3D(i, 0, 0)));
        }

        Assert.Equal(10, aCompound.TShape()!.ChildCount);
    }

    [Fact]
    public void EmptyCopy()
    {
        var aBuilder = new Builder();

        // Create compound with children
        Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);
        aBuilder.Add(aCompound, MakeVertex(new Point3D(0, 0, 0)));
        aBuilder.Add(aCompound, MakeVertex(new Point3D(1, 0, 0)));
        Assert.Equal(2, aCompound.TShape()!.ChildCount);

        // EmptyCopy should create same type with no children
        TShape aCopy = aCompound.TShape()!.EmptyCopy();
        Assert.Equal(ShapeType.Compound, aCopy.ShapeType());
        Assert.Equal(0, aCopy.ChildCount);
    }

    [Fact]
    public void EmptyCopy_AllTypes()
    {
        var aBuilder = new Builder();

        // Test Edge EmptyCopy
        {
            Edge anEdge = MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0));
            TShape aCopy = anEdge.TShape()!.EmptyCopy();
            Assert.Equal(ShapeType.Edge, aCopy.ShapeType());
            Assert.Equal(0, aCopy.ChildCount);
        }

        // Test Wire EmptyCopy
        {
            Wire aWire = new();
            aBuilder.MakeWire(ref aWire);
            Assert.Equal(ShapeType.Wire, aWire.TShape()!.EmptyCopy().ShapeType());
        }

        // Test Shell EmptyCopy
        {
            Shell aShell = new();
            aBuilder.MakeShell(ref aShell);
            Assert.Equal(ShapeType.Shell, aShell.TShape()!.EmptyCopy().ShapeType());
        }

        // Test Solid EmptyCopy
        {
            Solid aSolid = new();
            aBuilder.MakeSolid(ref aSolid);
            Assert.Equal(ShapeType.Solid, aSolid.TShape()!.EmptyCopy().ShapeType());
        }

        // Test CompSolid EmptyCopy
        {
            CompSolid aCompSolid = new();
            aBuilder.MakeCompSolid(ref aCompSolid);
            Assert.Equal(ShapeType.CompSolid, aCompSolid.TShape()!.EmptyCopy().ShapeType());
        }
    }

    [Fact]
    public void Orientable_DifferentTypes()
    {
        var aBuilder = new Builder();

        // Edge should be orientable
        Edge anEdge = MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0));
        Assert.True(anEdge.TShape()!.Orientable());

        // Compound should not be orientable
        Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);
        Assert.False(aCompound.TShape()!.Orientable());

        // Wire should be orientable
        Wire aWire = new();
        aBuilder.MakeWire(ref aWire);
        Assert.True(aWire.TShape()!.Orientable());
    }

    [Fact]
    public void FlagsIndependent()
    {
        var aBuilder = new Builder();
        Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        TShape aTShape = aCompound.TShape()!;

        // Set all flags
        aTShape.Closed(true);
        aTShape.Infinite(true);
        aTShape.Convex(true);
        aTShape.Locked(true);

        // Verify all are set
        Assert.True(aTShape.Closed());
        Assert.True(aTShape.Infinite());
        Assert.True(aTShape.Convex());
        Assert.True(aTShape.Locked());

        // Clear one flag, others should remain
        aTShape.Closed(false);
        Assert.False(aTShape.Closed());
        Assert.True(aTShape.Infinite());
        Assert.True(aTShape.Convex());
        Assert.True(aTShape.Locked());
    }

    [Fact]
    public void NbChildren_ConsistencyWithIterator()
    {
        var aBuilder = new Builder();

        // Create wire with edge
        Wire aWire = new();
        aBuilder.MakeWire(ref aWire);
        Edge anEdge = MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0));
        aBuilder.Add(aWire, anEdge);

        // Get NbChildren via TShape
        int aNbViaTShape = aWire.TShape()!.ChildCount;

        // Count via iterator
        int aNbViaIterator = 0;
        for (var anIt = new Iterator(aWire); anIt.More(); anIt.Next())
        {
            ++aNbViaIterator;
        }

        Assert.Equal(aNbViaIterator, aNbViaTShape);
        Assert.Equal(1, aNbViaTShape);
    }

    // TopoDS_Edge_Test.BUC60828_InfiniteFlag. OCCT sets the flag on the maker's result; in the port that result is
    // frozen (setting a flag on it throws, see FreezingTests), so the edge here is read through the maker's internal
    // accessor, as ported code that continues building on a maker's result does.
    [Fact]
    public void BUC60828_InfiniteFlag()
    {
        // Create a simple edge
        Edge anEdge = new MakeEdge(new Point3D(0.0, 0.0, 0.0), new Point3D(0.0, 0.0, 1.0)).InternalEdge();

        // Check initial flag value (should be false by default)
        bool anInitialValue = anEdge.IsInfinite;
        Assert.False(anInitialValue);

        // Set the flag to true
        anEdge.Infinite(true);

        // Verify the flag was set correctly
        bool aCurrentValue = anEdge.IsInfinite;
        Assert.True(aCurrentValue);
    }

    [Fact]
    public void Initial_flags_of_each_TShape_type_are_OCCTs()
    {
        // TopoDS_TShape: Free, Modified, Orientable; TCompound and TSolid aren't orientable; TVertex is closed and convex
        Vertex aV = new MakeVertex(new Point3D()).InternalVertex();
        Assert.True(aV.Free() && aV.Modified() && aV.IsOrientable && aV.IsClosed && aV.IsConvex);
        Solid aSolid = new();
        new Builder().MakeSolid(ref aSolid);
        Assert.False(aSolid.IsOrientable);
        Assert.False(aSolid.IsClosed);
        Assert.False(aSolid.Locked() || aSolid.Checked() || aSolid.IsInfinite || aSolid.IsConvex);
    }
}
