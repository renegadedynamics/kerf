// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.ModelingData.Tests;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>Unit tests of ShapeFix_FaceConnect (unit 6H3) on frozen shells.</summary>
public class ShapeFix_FaceConnectTests
{
    private static int Count(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, theType, aMap);
        return aMap.Extent();
    }

    // Two unit squares side by side, each with its own edges (theGap apart along x = 1), in one frozen shell.
    private static TopoDS_Shell TwoSquares(double theGap, out TopoDS_Face theF1, out TopoDS_Face theF2)
    {
        theF1 = new BRepBuilderAPI_MakeFace(new gp_Pln(), 0, 1, 0, 1).Face();
        theF2 = new BRepBuilderAPI_MakeFace(new gp_Pln(), 1 + theGap, 2, 0, 1).Face();
        var aB = new BRep_Builder();
        var aShell = new TopoDS_Shell();
        aB.MakeShell(ref aShell);
        aB.Add(aShell, theF1);
        aB.Add(aShell, theF2);
        aShell.TShape()!.FreezeGraph();
        return aShell;
    }

    [Fact]
    public void Add_NullFace_IsFalse_AndBuildWithoutConnections_GivesTheShell()
    {
        TopoDS_Shell aShell = TwoSquares(0, out TopoDS_Face aF1, out _);
        var aFC = new ShapeFix_FaceConnect();
        Assert.False(aFC.Add(aF1, new TopoDS_Face()));
        Assert.True(aFC.Add(aF1, aF1));
        aFC.Clear();

        TopoDS_Shell aResult = aFC.Build(aShell, 1e-4, 1e-4);
        Assert.True(aResult.IsSame(aShell));
    }

    // Needs ShapeFix_Face.FixOrientation (fork A): runs after the merge.
    [Fact]
    public void Build_SewsTheTwoFaces_InputUnchanged()
    {
        TopoDS_Shell aShell = TwoSquares(1e-7, out TopoDS_Face aF1, out TopoDS_Face aF2);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aShell);
        Assert.Equal(8, Count(aShell, TopAbs_EDGE));

        var aFC = new ShapeFix_FaceConnect();
        Assert.True(aFC.Add(aF1, aF2));
        TopoDS_Shell aResult = aFC.Build(aShell, 1e-4, 1e-4);

        aSnap.AssertUnchanged(aShell);
        Assert.True(aResult.TShape()!.IsFrozen());
        Assert.Equal(2, Count(aResult, TopAbs_FACE));
        Assert.Equal(7, Count(aResult, TopAbs_EDGE)); // the edges at x = 1 sewn into one
        Assert.Equal(6, Count(aResult, TopAbs_VERTEX));
    }
}
