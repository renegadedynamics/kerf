// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Reflection;
using OcctNet.ModelingData.Tests;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of ShapeUpgrade_UnifySameDomain on frozen inputs (unit 6U, plan 2.7): the caller's shape is never
/// written whatever the safe input mode, the result is frozen, a shape needing nothing comes back as itself, the
/// unchanged sub-shapes are the caller's own, the history answers for the caller's sub-shapes, and an unfrozen input
/// (ported code's) is worked on in place, as in OCCT.
/// </summary>
public class ShapeUpgrade_UnifySameDomainTests
{
    private static string Metadata(string theKey) =>
        typeof(ShapeUpgrade_UnifySameDomainTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == theKey).Value!;

    // A shell of two coplanar rectangles [0, 5] x [0, 4] and [5, 10] x [0, 4] sharing the edge x = 5, built as a
    // caller would get it: the faces are planar faces of their own (two Geom_Plane objects).
    private static TopoDS_Shell TwoCoplanarFaces(bool theToFreeze, out TopoDS_Edge theSharedEdge)
    {
        var B = new BRep_Builder();
        gp_Pnt[] aP = [new(0, 0, 0), new(5, 0, 0), new(10, 0, 0), new(10, 4, 0), new(5, 4, 0), new(0, 4, 0)];
        var aV = new TopoDS_Vertex[6];
        for (int i = 0; i < 6; i++)
        {
            B.MakeVertex(ref aV[i], aP[i], Precision.Confusion());
        }
        TopoDS_Edge E(int i, int j) => new BRepBuilderAPI_MakeEdge(aV[i], aV[j]).InternalEdge();
        TopoDS_Edge e01 = E(0, 1), e12 = E(1, 2), e23 = E(2, 3), e34 = E(3, 4), e45 = E(4, 5), e50 = E(5, 0), e14 = E(1, 4);

        TopoDS_Face F(params TopoDS_Edge[] theEdges)
        {
            var aW = new TopoDS_Wire();
            B.MakeWire(ref aW);
            foreach (TopoDS_Edge anEdge in theEdges)
            {
                B.Add(aW, anEdge);
            }
            aW.Closed(true);
            return new BRepBuilderAPI_MakeFace(new gp_Pln(new gp_Ax3(gp.XOY())), aW).InternalFace();
        }
        TopoDS_Face aLeft = F(e01, e14, e45, e50);
        TopoDS_Face aRight = F(e12, e23, e34, TopoDS.Edge(e14.Reversed()));

        var aShell = new TopoDS_Shell();
        B.MakeShell(ref aShell);
        B.Add(aShell, aLeft);
        B.Add(aShell, aRight);
        if (theToFreeze)
        {
            aShell.TShape()!.FreezeGraph();
        }
        theSharedEdge = e14;
        return aShell;
    }

    private static int Count(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, theType, aMap);
        return aMap.Extent();
    }

    private static HashSet<TopoDS_TShape> TShapes(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        var aSet = new HashSet<TopoDS_TShape>(ReferenceEqualityComparer.Instance);
        for (var anExp = new TopExp_Explorer(theShape, theType); anExp.More(); anExp.Next())
        {
            aSet.Add(anExp.Current().TShape()!);
        }
        return aSet;
    }

    private static bool AllFrozen(in TopoDS_Shape theShape)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, aMap);
        return aMap.All(s => s.TShape()!.IsFrozen());
    }

    [Fact]
    public void FrozenShell_CoplanarFacesAreMerged_TheInputIsUnchanged_TheHistoryAnswersForTheCallersShapes()
    {
        TopoDS_Shell aShell = TwoCoplanarFaces(true, out TopoDS_Edge aShared);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aShell);

        var aUnifier = new ShapeUpgrade_UnifySameDomain(aShell, true, true, true);
        aUnifier.AllowInternalEdges(false);
        aUnifier.Build();
        TopoDS_Shape aResult = aUnifier.Shape();

        aSnap.AssertUnchanged(aShell);
        Assert.True(AllFrozen(aResult));
        Assert.Equal(1, Count(aResult, TopAbs_FACE));
        // the bottom and top edges are merged across x = 5: 4 edges, 4 vertices
        Assert.Equal(4, Count(aResult, TopAbs_EDGE));
        Assert.Equal(4, Count(aResult, TopAbs_VERTEX));

        // the history answers for the caller's sub-shapes
        BRepTools_History aHistory = aUnifier.History()!;
        TopoDS_Shape aFace = new TopExp_Explorer(aResult, TopAbs_FACE).Current();
        for (var anExp = new TopExp_Explorer(aShell, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            Assert.Single(aHistory.Modified(anExp.Current()));
            Assert.True(aHistory.Modified(anExp.Current())[0].IsSame(aFace));
        }
        Assert.True(aHistory.IsRemoved(aShared));

        // faces only: the six boundary edges are the caller's own
        var aFacesOnly = new ShapeUpgrade_UnifySameDomain(aShell, false, true);
        aFacesOnly.Build();
        TopoDS_Shape aResult2 = aFacesOnly.Shape();
        aSnap.AssertUnchanged(aShell);
        Assert.Equal(6, TShapes(aResult2, TopAbs_EDGE).Intersect(TShapes(aShell, TopAbs_EDGE)).Count());
        Assert.Equal(6, Count(aResult2, TopAbs_EDGE));
    }

    [Fact]
    public void NothingToUnify_GivesBackTheInputItself()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(new gp_Pnt(0, 0, 0), new gp_Pnt(10, 10, 10)).Shape();
        var aUnifier = new ShapeUpgrade_UnifySameDomain(aBox, true, true, true);
        aUnifier.Build();
        TopoDS_Shape aResult = aUnifier.Shape();
        Assert.Same(aBox.TShape(), aResult.TShape());
        BRepTools_History aHistory = aUnifier.History()!;
        Assert.False(aHistory.HasModified());
        Assert.False(aHistory.HasRemoved());
    }

    [Fact]
    public void SafeInputModeOff_OnAFrozenInput_ActsOnACopy()
    {
        TopoDS_Shell aShell = TwoCoplanarFaces(true, out _);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aShell);

        var aUnifier = new ShapeUpgrade_UnifySameDomain(aShell);
        aUnifier.SetSafeInputMode(false);
        aUnifier.Build();
        TopoDS_Shape aResult = aUnifier.Shape();

        aSnap.AssertUnchanged(aShell);
        Assert.Equal(1, Count(aResult, TopAbs_FACE));
        Assert.Equal(4, Count(aResult, TopAbs_EDGE));
    }

    [Fact]
    public void UnfrozenInput_IsWorkedOnInPlace_AndTheResultIsInternal()
    {
        TopoDS_Shell aShell = TwoCoplanarFaces(false, out _);
        HashSet<TopoDS_TShape> anEdges = TShapes(aShell, TopAbs_EDGE);

        var aUnifier = new ShapeUpgrade_UnifySameDomain(aShell, false, true);
        aUnifier.Build();
        TopoDS_Shape anInternal = aUnifier.InternalShape();
        Assert.False(anInternal.TShape()!.IsFrozen());
        Assert.Equal(1, Count(anInternal, TopAbs_FACE));
        // no copy: the boundary edges are the input's, as in OCCT
        Assert.Equal(6, TShapes(anInternal, TopAbs_EDGE).Intersect(anEdges).Count());
        Assert.True(aUnifier.Shape().TShape()!.IsFrozen());
    }

    [Fact]
    public void KeepShape_TheCallersEdge_ForbidsTheMerge()
    {
        TopoDS_Shell aShell = TwoCoplanarFaces(true, out TopoDS_Edge aShared);
        var aUnifier = new ShapeUpgrade_UnifySameDomain(aShell);
        aUnifier.KeepShape(aShared);
        aUnifier.Build();
        TopoDS_Shape aResult = aUnifier.Shape();
        Assert.Equal(2, Count(aResult, TopAbs_FACE));
        Assert.Same(aShell.TShape(), aResult.TShape());

        // a kept vertex forbids the merge of the edges through it
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aShell, TopAbs_VERTEX, aMap);
        var aUnifier2 = new ShapeUpgrade_UnifySameDomain(aShell);
        for (int i = 1; i <= aMap.Extent(); i++)
        {
            aUnifier2.KeepShape(aMap[i]);
        }
        aUnifier2.Build();
        TopoDS_Shape aResult2 = aUnifier2.Shape();
        Assert.Equal(1, Count(aResult2, TopAbs_FACE));
        Assert.Equal(6, Count(aResult2, TopAbs_EDGE));
    }

    [Fact]
    public void NullHistory_CollectsNone_AndInitializeReuses()
    {
        TopoDS_Shell aShell = TwoCoplanarFaces(true, out _);
        var aUnifier = new ShapeUpgrade_UnifySameDomain();
        aUnifier.Initialize(aShell);
        aUnifier.History(null);
        aUnifier.Build();
        Assert.Null(aUnifier.History());
        Assert.Equal(1, Count(aUnifier.Shape(), TopAbs_FACE));

        TopoDS_Shell aShell2 = TwoCoplanarFaces(true, out _);
        aUnifier.History(new BRepTools_History());
        aUnifier.Initialize(aShell2, true, false);
        aUnifier.Build();
        // edges only: the faces stay, nothing to merge
        Assert.Equal(2, Count(aUnifier.Shape(), TopAbs_FACE));
        Assert.Same(aShell2.TShape(), aUnifier.Shape().TShape());
    }

    // heal group's split_closed_faces/F6 input: Clean (ConcatBSplines) makes a closed B-spline surface periodic, which
    // OCCT does in place on the face's own surface (decision 6705)
    [Fact]
    public void PeriodicSurface_IsACopy_TheCallersSurfaceIsUnchanged()
    {
        string aRoot = Metadata("OcctTestDataDir");
        string aFile = Path.Combine(aRoot, "brep", "bowl_noremove.brep");
        if (string.IsNullOrEmpty(aRoot) || !File.Exists(aFile))
        {
            return;
        }
        Assert.True(BRepTools.Read(out TopoDS_Shape aShape, aFile));
        aShape.TShape()!.FreezeGraph();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aShape);
        var aSurfaces = new List<(Geom_BSplineSurface, bool, bool)>();
        for (var anExp = new TopExp_Explorer(aShape, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            if (BRep_Tool.Surface(TopoDS.Face(anExp.Current())) is Geom_BSplineSurface aBS)
            {
                aSurfaces.Add((aBS, aBS.IsUPeriodic(), aBS.IsVPeriodic()));
            }
        }

        var aUnifier = new ShapeUpgrade_UnifySameDomain(aShape, true, true, true);
        aUnifier.Build();
        TopoDS_Shape aResult = aUnifier.Shape();

        aSnap.AssertUnchanged(aShape);
        Assert.All(aSurfaces, s => Assert.Equal((s.Item2, s.Item3), (s.Item1.IsUPeriodic(), s.Item1.IsVPeriodic())));
        bool hasPeriodic = false;
        for (var anExp = new TopExp_Explorer(aResult, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            if (BRep_Tool.Surface(TopoDS.Face(anExp.Current())) is Geom_BSplineSurface aBS && (aBS.IsUPeriodic() || aBS.IsVPeriodic()))
            {
                hasPeriodic = true;
                Assert.DoesNotContain(aSurfaces, s => ReferenceEquals(s.Item1, aBS));
            }
        }
        Assert.True(hasPeriodic);
        Assert.True(AllFrozen(aResult));
    }
}
