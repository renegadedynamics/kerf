// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.ModelingData.Tests;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// ShapeFix_ShapeTolerance in place on owned shapes and its twins on a frozen box: the input unchanged, the result
/// frozen, the sub-shapes not concerned the caller's own, a no-op the input itself.
/// </summary>
public class ShapeFix_ShapeToleranceTests
{
    private static TopoDS_Shape Box() => new BRepPrimAPI_MakeBox(10, 20, 30).Shape();

    private static List<TopoDS_Shape> Subs(TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        var aList = new List<TopoDS_Shape>();
        for (var anExp = new TopExp_Explorer(theShape, theType); anExp.More(); anExp.Next())
        {
            aList.Add(anExp.Current());
        }
        return aList;
    }

    [Fact]
    public void LimitTolerance_InPlace_RaisesAndLowers()
    {
        TopoDS_Shape aBox = BRepTools.ExactCopyForWrite(Box(), out _);
        var aTool = new ShapeFix_ShapeTolerance();
        Assert.True(aTool.LimitTolerance(aBox, 1e-3));
        Assert.All(Subs(aBox, TopAbs_EDGE), e => Assert.Equal(1e-3, BRep_Tool.Tolerance(TopoDS.Edge(e))));
        Assert.All(Subs(aBox, TopAbs_FACE), f => Assert.Equal(1e-3, BRep_Tool.Tolerance(TopoDS.Face(f))));
        Assert.True(aTool.LimitTolerance(aBox, 0.0, 1e-5, TopAbs_VERTEX));
        Assert.All(Subs(aBox, TopAbs_VERTEX), v => Assert.Equal(1e-5, BRep_Tool.Tolerance(TopoDS.Vertex(v))));
        Assert.False(aTool.LimitTolerance(aBox, 0.0, 1.0));
        Assert.False(aTool.LimitTolerance(new TopoDS_Shape(), 0.0));
    }

    [Fact]
    public void LimitTolerance_Wire_DoesntCountTheEdges()
    {
        // OCCT: in WIRE mode only the vertices' changes make the result true
        TopoDS_Shape aW = BRepTools.ExactCopyForWrite(
            new BRepBuilderAPI_MakePolygon(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0), new gp_Pnt(1, 1, 0), true).Wire(), out _);
        var aTool = new ShapeFix_ShapeTolerance();
        aTool.SetTolerance(aW, 1e-3, TopAbs_VERTEX);
        Assert.False(aTool.LimitTolerance(aW, 1e-4, 1e-2, TopAbs_WIRE)); // edges raised from 1e-7, not counted
        Assert.All(Subs(aW, TopAbs_EDGE), e => Assert.Equal(1e-4, BRep_Tool.Tolerance(TopoDS.Edge(e))));
    }

    [Fact]
    public void SetTolerance_InPlace_ByType()
    {
        TopoDS_Shape aBox = BRepTools.ExactCopyForWrite(Box(), out _);
        var aTool = new ShapeFix_ShapeTolerance();
        aTool.SetTolerance(aBox, 0.5, TopAbs_WIRE);
        Assert.All(Subs(aBox, TopAbs_VERTEX), v => Assert.Equal(0.5, BRep_Tool.Tolerance(TopoDS.Vertex(v))));
        Assert.All(Subs(aBox, TopAbs_EDGE), e => Assert.Equal(0.5, BRep_Tool.Tolerance(TopoDS.Edge(e))));
        Assert.All(Subs(aBox, TopAbs_FACE), f => Assert.Equal(1e-7, BRep_Tool.Tolerance(TopoDS.Face(f))));
        aTool.SetTolerance(aBox, -1.0); // ignored
        Assert.All(Subs(aBox, TopAbs_EDGE), e => Assert.Equal(0.5, BRep_Tool.Tolerance(TopoDS.Edge(e))));
    }

    [Fact]
    public void Twins_FrozenBox()
    {
        TopoDS_Shape aBox = Box();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        var aTool = new ShapeFix_ShapeTolerance();

        TopoDS_Shape aRes = aTool.WithTolerance(aBox, 0.5, TopAbs_FACE);
        aSnap.AssertUnchanged(aBox);
        Assert.True(aRes.TShape()!.IsFrozen());
        Assert.All(Subs(aRes, TopAbs_FACE), f => Assert.Equal(0.5, BRep_Tool.Tolerance(TopoDS.Face(f))));
        // the edges didn't change: they are the caller's
        var aEdges = new HashSet<object>(Subs(aBox, TopAbs_EDGE).Select(e => (object)e.TShape()!), ReferenceEqualityComparer.Instance);
        Assert.All(Subs(aRes, TopAbs_EDGE), e => Assert.Contains(e.TShape()!, aEdges));

        aRes = aTool.ToleranceLimited(aBox, 0.0, out bool isDone, 1.0);
        Assert.False(isDone);
        Assert.Same(aBox.TShape(), aRes.TShape());

        aRes = aTool.ToleranceLimited(aBox, 1e-3, out isDone, 0.0, TopAbs_VERTEX);
        Assert.True(isDone);
        aSnap.AssertUnchanged(aBox);
        Assert.All(Subs(aRes, TopAbs_VERTEX), v => Assert.Equal(1e-3, BRep_Tool.Tolerance(TopoDS.Vertex(v))));
    }
}
