// Port of: src/ModelingData/TKBRep/GTests/TopoDS_TShape_Test.cxx
// Port of: src/ModelingData/TKBRep/GTests/TopoDS_Edge_Test.cxx
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// OCCT's TopoDS_TShape and TopoDS_Edge GTests. BRepBuilderAPI_MakeVertex and MakeEdge (unit 3K) are replaced by
/// <see cref="TestShapes"/>, frozen as the makers' results will be.
/// </summary>
public class TopoDS_TShapeTests
{
    private static TopoDS_Vertex MakeVertex(in gp_Pnt theP) => TestShapes.Frozen(TestShapes.Vertex(theP));

    private static TopoDS_Edge MakeEdge(in gp_Pnt theP1, in gp_Pnt theP2) => TestShapes.Frozen(TestShapes.Edge(theP1, theP2));

    [Fact]
    public void ShapeType_AllTypes()
    {
        var aBuilder = new TopoDS_Builder();

        // Create various shapes and verify ShapeType
        TopoDS_Vertex aVertex = MakeVertex(new gp_Pnt(0, 0, 0));
        Assert.Equal(TopAbs_VERTEX, aVertex.TShape()!.ShapeType());

        TopoDS_Edge anEdge = MakeEdge(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0));
        Assert.Equal(TopAbs_EDGE, anEdge.TShape()!.ShapeType());

        TopoDS_Wire aWire = new();
        aBuilder.MakeWire(ref aWire);
        Assert.Equal(TopAbs_WIRE, aWire.TShape()!.ShapeType());

        TopoDS_Shell aShell = new();
        aBuilder.MakeShell(ref aShell);
        Assert.Equal(TopAbs_SHELL, aShell.TShape()!.ShapeType());

        TopoDS_Solid aSolid = new();
        aBuilder.MakeSolid(ref aSolid);
        Assert.Equal(TopAbs_SOLID, aSolid.TShape()!.ShapeType());

        TopoDS_CompSolid aCompSolid = new();
        aBuilder.MakeCompSolid(ref aCompSolid);
        Assert.Equal(TopAbs_COMPSOLID, aCompSolid.TShape()!.ShapeType());

        TopoDS_Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);
        Assert.Equal(TopAbs_COMPOUND, aCompound.TShape()!.ShapeType());
    }

    [Fact]
    public void FlagSettersGetters()
    {
        var aBuilder = new TopoDS_Builder();
        TopoDS_Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        TopoDS_TShape aTShape = aCompound.TShape()!;

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
        var aBuilder = new TopoDS_Builder();

        // Vertex has 0 children
        TopoDS_Vertex aVertex = MakeVertex(new gp_Pnt(0, 0, 0));
        Assert.Equal(0, aVertex.TShape()!.NbChildren());

        // Edge has 2 children (vertices)
        TopoDS_Edge anEdge = MakeEdge(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0));
        Assert.Equal(2, anEdge.TShape()!.NbChildren());

        // Empty wire has 0 children
        TopoDS_Wire aWire = new();
        aBuilder.MakeWire(ref aWire);
        Assert.Equal(0, aWire.TShape()!.NbChildren());

        // Wire with edges
        aBuilder.Add(aWire, anEdge);
        Assert.Equal(1, aWire.TShape()!.NbChildren());

        // Compound with multiple children
        TopoDS_Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);
        for (int i = 0; i < 10; ++i)
        {
            aBuilder.Add(aCompound, MakeVertex(new gp_Pnt(i, 0, 0)));
        }

        Assert.Equal(10, aCompound.TShape()!.NbChildren());
    }

    [Fact]
    public void EmptyCopy()
    {
        var aBuilder = new TopoDS_Builder();

        // Create compound with children
        TopoDS_Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);
        aBuilder.Add(aCompound, MakeVertex(new gp_Pnt(0, 0, 0)));
        aBuilder.Add(aCompound, MakeVertex(new gp_Pnt(1, 0, 0)));
        Assert.Equal(2, aCompound.TShape()!.NbChildren());

        // EmptyCopy should create same type with no children
        TopoDS_TShape aCopy = aCompound.TShape()!.EmptyCopy();
        Assert.Equal(TopAbs_COMPOUND, aCopy.ShapeType());
        Assert.Equal(0, aCopy.NbChildren());
    }

    [Fact]
    public void EmptyCopy_AllTypes()
    {
        var aBuilder = new TopoDS_Builder();

        // Test Edge EmptyCopy
        {
            TopoDS_Edge anEdge = MakeEdge(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0));
            TopoDS_TShape aCopy = anEdge.TShape()!.EmptyCopy();
            Assert.Equal(TopAbs_EDGE, aCopy.ShapeType());
            Assert.Equal(0, aCopy.NbChildren());
        }

        // Test Wire EmptyCopy
        {
            TopoDS_Wire aWire = new();
            aBuilder.MakeWire(ref aWire);
            Assert.Equal(TopAbs_WIRE, aWire.TShape()!.EmptyCopy().ShapeType());
        }

        // Test Shell EmptyCopy
        {
            TopoDS_Shell aShell = new();
            aBuilder.MakeShell(ref aShell);
            Assert.Equal(TopAbs_SHELL, aShell.TShape()!.EmptyCopy().ShapeType());
        }

        // Test Solid EmptyCopy
        {
            TopoDS_Solid aSolid = new();
            aBuilder.MakeSolid(ref aSolid);
            Assert.Equal(TopAbs_SOLID, aSolid.TShape()!.EmptyCopy().ShapeType());
        }

        // Test CompSolid EmptyCopy
        {
            TopoDS_CompSolid aCompSolid = new();
            aBuilder.MakeCompSolid(ref aCompSolid);
            Assert.Equal(TopAbs_COMPSOLID, aCompSolid.TShape()!.EmptyCopy().ShapeType());
        }
    }

    [Fact]
    public void Orientable_DifferentTypes()
    {
        var aBuilder = new TopoDS_Builder();

        // Edge should be orientable
        TopoDS_Edge anEdge = MakeEdge(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0));
        Assert.True(anEdge.TShape()!.Orientable());

        // Compound should not be orientable
        TopoDS_Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);
        Assert.False(aCompound.TShape()!.Orientable());

        // Wire should be orientable
        TopoDS_Wire aWire = new();
        aBuilder.MakeWire(ref aWire);
        Assert.True(aWire.TShape()!.Orientable());
    }

    [Fact]
    public void FlagsIndependent()
    {
        var aBuilder = new TopoDS_Builder();
        TopoDS_Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);

        TopoDS_TShape aTShape = aCompound.TShape()!;

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
        var aBuilder = new TopoDS_Builder();

        // Create wire with edge
        TopoDS_Wire aWire = new();
        aBuilder.MakeWire(ref aWire);
        TopoDS_Edge anEdge = MakeEdge(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0));
        aBuilder.Add(aWire, anEdge);

        // Get NbChildren via TShape
        int aNbViaTShape = aWire.TShape()!.NbChildren();

        // Count via iterator
        int aNbViaIterator = 0;
        for (var anIt = new TopoDS_Iterator(aWire); anIt.More(); anIt.Next())
        {
            ++aNbViaIterator;
        }

        Assert.Equal(aNbViaIterator, aNbViaTShape);
        Assert.Equal(1, aNbViaTShape);
    }

    // TopoDS_Edge_Test.BUC60828_InfiniteFlag. OCCT sets the flag on the maker's result; in the port that result is
    // frozen (setting a flag on it throws, see FreezingTests), so the edge here is built unfrozen, as ported code
    // that continues building on a maker's result reads it through the maker's internal accessor.
    [Fact]
    public void BUC60828_InfiniteFlag()
    {
        // Create a simple edge
        TopoDS_Edge anEdge = TestShapes.Edge(new gp_Pnt(0.0, 0.0, 0.0), new gp_Pnt(0.0, 0.0, 1.0));

        // Check initial flag value (should be false by default)
        bool anInitialValue = anEdge.Infinite();
        Assert.False(anInitialValue);

        // Set the flag to true
        anEdge.Infinite(true);

        // Verify the flag was set correctly
        bool aCurrentValue = anEdge.Infinite();
        Assert.True(aCurrentValue);
    }

    [Fact]
    public void Initial_flags_of_each_TShape_type_are_OCCTs()
    {
        // TopoDS_TShape: Free, Modified, Orientable; TCompound and TSolid aren't orientable; TVertex is closed and convex
        TopoDS_Vertex aV = TestShapes.Vertex(new gp_Pnt());
        Assert.True(aV.Free() && aV.Modified() && aV.Orientable() && aV.Closed() && aV.Convex());
        TopoDS_Solid aSolid = new();
        new TopoDS_Builder().MakeSolid(ref aSolid);
        Assert.False(aSolid.Orientable());
        Assert.False(aSolid.Closed());
        Assert.False(aSolid.Locked() || aSolid.Checked() || aSolid.Infinite() || aSolid.Convex());
    }
}
