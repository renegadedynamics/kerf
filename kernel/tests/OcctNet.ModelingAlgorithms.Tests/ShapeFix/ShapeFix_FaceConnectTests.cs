// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using OcctNet.ModelingData.Tests;
using static Kerf.Topology.ShapeType;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>Unit tests of ShapeFix_FaceConnect (unit 6H3) on frozen shells.</summary>
public class ShapeFix_FaceConnectTests
{
    private static int Count(in Shape theShape, ShapeType theType)
    {
        var aMap = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, theType, aMap);
        return aMap.Extent();
    }

    // Two unit squares side by side, each with its own edges (theGap apart along x = 1), in one frozen shell.
    private static Shell TwoSquares(double theGap, out Face theF1, out Face theF2)
    {
        theF1 = new MakeFace(new Plane3D(), 0, 1, 0, 1).Face();
        theF2 = new MakeFace(new Plane3D(), 1 + theGap, 2, 0, 1).Face();
        var aB = new BrepBuilder();
        var aShell = new Shell();
        aB.MakeShell(ref aShell);
        aB.Add(aShell, theF1);
        aB.Add(aShell, theF2);
        aShell.TShape()!.FreezeGraph();
        return aShell;
    }

    [Fact]
    public void Add_NullFace_IsFalse_AndBuildWithoutConnections_GivesTheShell()
    {
        Shell aShell = TwoSquares(0, out Face aF1, out _);
        var aFC = new FaceConnectFixer();
        Assert.False(aFC.Add(aF1, new Face()));
        Assert.True(aFC.Add(aF1, aF1));
        aFC.Clear();

        Shell aResult = aFC.Build(aShell, 1e-4, 1e-4);
        Assert.True(aResult.IsSame(aShell));
    }

    // Needs ShapeFix_Face.FixOrientation (fork A): runs after the merge.
    [Fact]
    public void Build_SewsTheTwoFaces_InputUnchanged()
    {
        Shell aShell = TwoSquares(1e-7, out Face aF1, out Face aF2);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aShell);
        Assert.Equal(8, Count(aShell, ShapeType.Edge));

        var aFC = new FaceConnectFixer();
        Assert.True(aFC.Add(aF1, aF2));
        Shell aResult = aFC.Build(aShell, 1e-4, 1e-4);

        aSnap.AssertUnchanged(aShell);
        Assert.True(aResult.TShape()!.IsFrozen);
        Assert.Equal(2, Count(aResult, ShapeType.Face));
        Assert.Equal(7, Count(aResult, ShapeType.Edge)); // the edges at x = 1 sewn into one
        Assert.Equal(6, Count(aResult, ShapeType.Vertex));
    }
}
