// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// ShapeFix_ShapeTolerance in place on owned shapes and its twins on a frozen box: the input unchanged, the result
/// frozen, the sub-shapes not concerned the caller's own, a no-op the input itself.
/// </summary>
public class ShapeFix_ShapeToleranceTests
{
    private static Shape Box() => new MakeBox(10, 20, 30).Shape();

    private static List<Shape> Subs(Shape theShape, ShapeType theType)
    {
        var aList = new List<Shape>();
        for (var anExp = new Explorer(theShape, theType); anExp.More(); anExp.Next())
        {
            aList.Add(anExp.Current());
        }
        return aList;
    }

    [Fact]
    public void LimitTolerance_InPlace_RaisesAndLowers()
    {
        Shape aBox = BrepTools.ExactCopyForWrite(Box(), out _);
        var aTool = new ShapeToleranceFixer();
        Assert.True(aTool.LimitTolerance(aBox, 1e-3));
        Assert.All(Subs(aBox, ShapeType.Edge), e => Assert.Equal(1e-3, Tool.Tolerance(ShapeCasts.Edge(e))));
        Assert.All(Subs(aBox, ShapeType.Face), f => Assert.Equal(1e-3, Tool.Tolerance(ShapeCasts.Face(f))));
        Assert.True(aTool.LimitTolerance(aBox, 0.0, 1e-5, ShapeType.Vertex));
        Assert.All(Subs(aBox, ShapeType.Vertex), v => Assert.Equal(1e-5, Tool.Tolerance(ShapeCasts.Vertex(v))));
        Assert.False(aTool.LimitTolerance(aBox, 0.0, 1.0));
        Assert.False(aTool.LimitTolerance(new Shape(), 0.0));
    }

    [Fact]
    public void LimitTolerance_Wire_DoesntCountTheEdges()
    {
        // OCCT: in WIRE mode only the vertices' changes make the result true
        Shape aW = BrepTools.ExactCopyForWrite(
            new MakePolygon(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(1, 1, 0), true).Wire(), out _);
        var aTool = new ShapeToleranceFixer();
        aTool.SetTolerance(aW, 1e-3, ShapeType.Vertex);
        Assert.False(aTool.LimitTolerance(aW, 1e-4, 1e-2, ShapeType.Wire)); // edges raised from 1e-7, not counted
        Assert.All(Subs(aW, ShapeType.Edge), e => Assert.Equal(1e-4, Tool.Tolerance(ShapeCasts.Edge(e))));
    }

    [Fact]
    public void SetTolerance_InPlace_ByType()
    {
        Shape aBox = BrepTools.ExactCopyForWrite(Box(), out _);
        var aTool = new ShapeToleranceFixer();
        aTool.SetTolerance(aBox, 0.5, ShapeType.Wire);
        Assert.All(Subs(aBox, ShapeType.Vertex), v => Assert.Equal(0.5, Tool.Tolerance(ShapeCasts.Vertex(v))));
        Assert.All(Subs(aBox, ShapeType.Edge), e => Assert.Equal(0.5, Tool.Tolerance(ShapeCasts.Edge(e))));
        Assert.All(Subs(aBox, ShapeType.Face), f => Assert.Equal(1e-7, Tool.Tolerance(ShapeCasts.Face(f))));
        aTool.SetTolerance(aBox, -1.0); // ignored
        Assert.All(Subs(aBox, ShapeType.Edge), e => Assert.Equal(0.5, Tool.Tolerance(ShapeCasts.Edge(e))));
    }

    [Fact]
    public void Twins_FrozenBox()
    {
        Shape aBox = Box();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        var aTool = new ShapeToleranceFixer();

        Shape aRes = aTool.WithTolerance(aBox, 0.5, ShapeType.Face);
        aSnap.AssertUnchanged(aBox);
        Assert.True(aRes.TShape()!.IsFrozen);
        Assert.All(Subs(aRes, ShapeType.Face), f => Assert.Equal(0.5, Tool.Tolerance(ShapeCasts.Face(f))));
        // the edges didn't change: they are the caller's
        var aEdges = new HashSet<object>(Subs(aBox, ShapeType.Edge).Select(e => (object)e.TShape()!), ReferenceEqualityComparer.Instance);
        Assert.All(Subs(aRes, ShapeType.Edge), e => Assert.Contains(e.TShape()!, aEdges));

        aRes = aTool.ToleranceLimited(aBox, 0.0, out bool isDone, 1.0);
        Assert.False(isDone);
        Assert.Same(aBox.TShape(), aRes.TShape());

        aRes = aTool.ToleranceLimited(aBox, 1e-3, out isDone, 0.0, ShapeType.Vertex);
        Assert.True(isDone);
        aSnap.AssertUnchanged(aBox);
        Assert.All(Subs(aRes, ShapeType.Vertex), v => Assert.Equal(1e-3, Tool.Tolerance(ShapeCasts.Vertex(v))));
    }
}
