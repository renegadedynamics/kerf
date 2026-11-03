// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using OcctNet.ModelingData.Tests;
using static Occt.ShapeExtend_Status;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>Polylines for the tests of ShapeFix_Shape and ShapeFix_Wireframe.</summary>
internal static class ShapeFixPolylines
{
    /// <summary>A wire through the points, its consecutive edges sharing their vertices (closed: back to the first).</summary>
    public static TopoDS_Wire Polyline(bool theClosed, params Point3D[] thePoints)
    {
        var aV = thePoints.Select(p => TestShapes.Vertex(p)).ToArray();
        var aB = new BRep_Builder();
        var aW = new TopoDS_Wire();
        aB.MakeWire(ref aW);
        int aNb = theClosed ? aV.Length : aV.Length - 1;
        for (int i = 0; i < aNb; i++)
        {
            aB.Add(aW, TestShapes.Edge(aV[i], aV[(i + 1) % aV.Length]));
        }
        aW.Closed(theClosed);
        return aW;
    }

    /// <summary>A closed wire of edges through the points, each with its own vertices (gaps between the edges).</summary>
    public static TopoDS_Wire WithGaps(double theGap, params Point3D[] thePoints)
    {
        var aB = new BRep_Builder();
        var aW = new TopoDS_Wire();
        aB.MakeWire(ref aW);
        for (int i = 0; i < thePoints.Length; i++)
        {
            Point3D aP2 = thePoints[(i + 1) % thePoints.Length];
            aB.Add(aW, TestShapes.Edge(thePoints[i], new Point3D(aP2.X() + theGap, aP2.Y() + theGap, aP2.Z())));
        }
        aW.Closed(true);
        return aW;
    }

    public static int NbEdges(in TopoDS_Shape theWire)
    {
        int n = 0;
        for (var anExp = new TopExp_Explorer(theWire, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            n++;
        }
        return n;
    }

    public static TopoDS_Compound Compound(params TopoDS_Shape[] theShapes)
    {
        var aB = new BRep_Builder();
        var aC = new TopoDS_Compound();
        aB.MakeCompound(ref aC);
        foreach (TopoDS_Shape aS in theShapes)
        {
            aB.Add(aC, aS);
        }
        return aC;
    }
}

/// <summary>
/// ShapeFix_Shape: the dispatch by type (wires, edges, compounds, solids with the face fixes off), the modes, a repeated
/// Perform, on frozen shapes (unchanged, results frozen, the caller's shapes kept), and ShapeFix::RemoveSmallEdges.
/// </summary>
public class ShapeFix_ShapeTests
{
    private static TopoDS_Wire GapSquare() => TestShapes.Frozen(ShapeFixPolylines.WithGaps(1e-4, new Point3D(0, 0, 0), new Point3D(10, 0, 0),
                                                                                        new Point3D(10, 10, 0), new Point3D(0, 10, 0)));

    [Fact]
    public void Wire_WithGaps_IsConnected()
    {
        TopoDS_Wire aWire = GapSquare();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aWire);
        var aFix = new ShapeFix_Shape(aWire);
        aFix.SetPrecision(1e-3);
        aFix.SetMaxTolerance(1e-3);
        Assert.True(aFix.Perform());
        Assert.True(aFix.Status(ShapeExtend_DONE2));
        TopoDS_Shape aResult = aFix.Shape();
        aSnap.AssertUnchanged(aWire);
        Assert.True(aResult.TShape()!.IsFrozen);
        Assert.Equal(TopAbs_WIRE, aResult.ShapeType());
        Assert.Equal(4, ShapeFixShapes.Count(aResult, TopAbs_VERTEX));
        Assert.True(aFix.Context()!.Apply(aWire).IsSame(aResult));
    }

    [Fact]
    public void Wire_Valid_GivesTheInput()
    {
        TopoDS_Wire aWire = TestShapes.Frozen(ShapeFixPolylines.Polyline(true, new Point3D(0, 0, 0), new Point3D(10, 0, 0), new Point3D(10, 10, 0)));
        var aFix = new ShapeFix_Shape(aWire);
        aFix.Perform();
        Assert.True(ReferenceEquals(aFix.Shape().TShape(), aWire.TShape()));
    }

    [Fact]
    public void FreeWireModeOff_LeavesTheWire()
    {
        TopoDS_Wire aWire = GapSquare();
        var aFix = new ShapeFix_Shape(aWire);
        aFix.FixFreeWireMode() = 0;
        aFix.FixSameParameterMode() = 0;
        Assert.False(aFix.Perform());
        Assert.True(ReferenceEquals(aFix.Shape().TShape(), aWire.TShape()));
    }

    [Fact]
    public void Compound_OfAWireAndAnEdge_KeepsWhatNeedsNothing()
    {
        TopoDS_Wire aWire = ShapeFixPolylines.WithGaps(1e-4, new Point3D(0, 0, 0), new Point3D(10, 0, 0), new Point3D(10, 10, 0));
        TopoDS_Edge anEdge = TestShapes.Edge(new Point3D(50, 0, 0), new Point3D(60, 0, 0));
        TopoDS_Compound aComp = TestShapes.Frozen(ShapeFixPolylines.Compound(aWire, anEdge));
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aComp);
        var aFix = new ShapeFix_Shape(aComp);
        aFix.SetPrecision(1e-3);
        aFix.SetMaxTolerance(1e-3);
        Assert.True(aFix.Perform());
        TopoDS_Shape aResult = aFix.Shape();
        aSnap.AssertUnchanged(aComp);
        Assert.Equal(TopAbs_COMPOUND, aResult.ShapeType());
        Assert.Equal(3, ShapeFixShapes.Count(aResult, TopAbs_VERTEX) - 2);
        // the free edge needed nothing: it is the caller's
        Assert.True(ShapeFixShapes.HasTShapeOf(aResult, anEdge, TopAbs_EDGE));
    }

    [Fact]
    public void Solid_FaceFixesOff_ReversedShellFixed_ValidBoxIsTheInput()
    {
        TopoDS_Shell aShell = ShapeFixShapes.Shell(ShapeFixShapes.BoxFaces(), -1, true);
        TopoDS_Solid aSolid = TestShapes.Frozen(ShapeFixShapes.Solid(aShell.Reversed()));
        var aFix = new ShapeFix_Shape(aSolid);
        aFix.FixShellTool().FixFaceMode() = 0;
        aFix.Perform();
        Assert.True(aFix.Status(ShapeExtend_DONE4));
        Assert.Equal(1000.0, ShapeFixShapes.Volume(aFix.Shape()), 9);

        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(10, 10, 10));
        var aFix2 = new ShapeFix_Shape(aBox);
        aFix2.FixShellTool().FixFaceMode() = 0;
        aFix2.Perform();
        Assert.True(ReferenceEquals(aFix2.Shape().TShape(), aBox.TShape()));
    }

    [Fact]
    public void RepeatedPerform_TakesTheEarlyReturn()
    {
        TopoDS_Wire aWire = GapSquare();
        var aFix = new ShapeFix_Shape(aWire);
        aFix.SetPrecision(1e-3);
        aFix.SetMaxTolerance(1e-3);
        Assert.True(aFix.Perform());
        TopoDS_Shape aFirst = aFix.Shape();
        aFix.Init(aWire);
        Assert.True(aFix.Perform()); // OCCT: a shape already fixed by this tool gives the context's result
        Assert.True(aFix.Shape().IsSame(aFirst));
    }

    [Fact]
    public void Edge_VertexTolerance_Status()
    {
        TopoDS_Edge anEdge = TestShapes.Frozen(TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(10, 0, 0)));
        var aFix = new ShapeFix_Shape(anEdge);
        aFix.Perform();
        Assert.False(aFix.Status(ShapeExtend_DONE1));
        Assert.True(ReferenceEquals(aFix.Shape().TShape(), anEdge.TShape()));
    }

    [Fact]
    public void RemoveSmallEdges_OnAWire()
    {
        TopoDS_Wire aWire = TestShapes.Frozen(ShapeFixPolylines.Polyline(true, new Point3D(0, 0, 0), new Point3D(10, 0, 0),
                                                                           new Point3D(10, 5e-8, 0), new Point3D(10, 10, 0),
                                                                           new Point3D(0, 10, 0)));
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aWire);
        TopoDS_Shape aResult = ShapeFix.RemoveSmallEdges(aWire, 1e-3, out ShapeBuild_ReShape aContext);
        aSnap.AssertUnchanged(aWire);
        Assert.True(aResult.TShape()!.IsFrozen);
        Assert.Equal(4, ShapeFixPolylines.NbEdges(aResult));
        Assert.True(aContext.Apply(aWire).IsSame(aResult));
    }
}

/// <summary>
/// ShapeFix_Wireframe: FixWireGaps and FixSmallEdges (merge, the limit angle, the drop mode) on free wires and faces,
/// frozen inputs unchanged and results frozen.
/// </summary>
public class ShapeFix_WireframeTests
{
    // a square with a step of height 1e-4 on its top edge: the small edge is at right angles to both neighbours
    private static TopoDS_Wire Step() => TestShapes.Frozen(ShapeFixPolylines.Polyline(true, new Point3D(0, 0, 0), new Point3D(10, 0, 0),
                                                                                   new Point3D(10, 10, 0), new Point3D(5, 10, 0),
                                                                                   new Point3D(5, 10.0001, 0), new Point3D(0, 10.0001, 0)));

    [Fact]
    public void FixWireGaps_FreeWire()
    {
        TopoDS_Wire aWire = TestShapes.Frozen(ShapeFixPolylines.WithGaps(1e-2, new Point3D(0, 0, 0), new Point3D(10, 0, 0),
                                                                          new Point3D(10, 10, 0), new Point3D(0, 10, 0)));
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aWire);
        var aFix = new ShapeFix_Wireframe(aWire);
        aFix.SetPrecision(1e-3);
        Assert.True(aFix.FixWireGaps());
        Assert.True(aFix.StatusWireGaps(ShapeExtend_DONE1));
        TopoDS_Shape aResult = aFix.Shape();
        aSnap.AssertUnchanged(aWire);
        Assert.True(aResult.TShape()!.IsFrozen);
        Assert.Equal(4, ShapeFixPolylines.NbEdges(aResult));
    }

    [Fact]
    public void FixSmallEdges_FreeWire_Merged()
    {
        TopoDS_Wire aWire = Step();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aWire);
        var aFix = new ShapeFix_Wireframe(aWire);
        aFix.SetPrecision(1e-3);
        Assert.True(aFix.FixSmallEdges());
        Assert.True(aFix.StatusSmallEdges(ShapeExtend_DONE1));
        TopoDS_Shape aResult = aFix.Shape();
        aSnap.AssertUnchanged(aWire);
        Assert.Equal(5, ShapeFixPolylines.NbEdges(aResult));
        // the edges far from the small one are the caller's
        Assert.True(ShapeFixShapes.HasTShapeOf(aResult, ShapeFixShapes.SubShapes(aWire, TopAbs_EDGE)[0], TopAbs_EDGE));
    }

    [Fact]
    public void FixSmallEdges_LimitAngle_KeepsIt_DropModeDropsIt()
    {
        var aFix = new ShapeFix_Wireframe(Step());
        aFix.SetPrecision(1e-3);
        aFix.SetLimitAngle(0.5);
        Assert.False(aFix.FixSmallEdges());
        Assert.Equal(6, ShapeFixPolylines.NbEdges(aFix.Shape()));

        var aFix2 = new ShapeFix_Wireframe(Step());
        aFix2.SetPrecision(1e-3);
        aFix2.SetLimitAngle(0.5);
        aFix2.ModeDropSmallEdges() = true;
        Assert.True(aFix2.FixSmallEdges());
        Assert.True(aFix2.StatusSmallEdges(ShapeExtend_DONE3));
        Assert.Equal(5, ShapeFixPolylines.NbEdges(aFix2.Shape()));
    }

    [Fact]
    public void FixSmallEdges_OnAFace()
    {
        TopoDS_Wire aWire = ShapeFixPolylines.Polyline(true, new Point3D(0, 0, 0), new Point3D(10, 0, 0), new Point3D(10, 1e-4, 0),
                                                       new Point3D(10, 10, 0), new Point3D(0, 10, 0));
        TopoDS_Face aFace = new BRepBuilderAPI_MakeFace(aWire, true).Face();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aFace);
        var aFix = new ShapeFix_Wireframe(aFace);
        aFix.SetPrecision(1e-3);
        Assert.True(aFix.FixSmallEdges());
        TopoDS_Shape aResult = aFix.Shape();
        aSnap.AssertUnchanged(aFace);
        Assert.Equal(TopAbs_FACE, aResult.ShapeType());
        Assert.Equal(4, ShapeFixShapes.Count(aResult, TopAbs_EDGE));
        Assert.True(aResult.TShape()!.IsFrozen);
    }

    [Fact]
    public void Compound_OfWires()
    {
        TopoDS_Compound aComp = TestShapes.Frozen(ShapeFixPolylines.Compound(
            ShapeFixPolylines.Polyline(true, new Point3D(0, 0, 0), new Point3D(10, 0, 0), new Point3D(10, 10, 0),
                                       new Point3D(5, 10, 0), new Point3D(5, 10.0001, 0), new Point3D(0, 10.0001, 0)),
            ShapeFixPolylines.Polyline(true, new Point3D(20, 0, 0), new Point3D(30, 0, 0), new Point3D(30, 10, 0))));
        var aFix = new ShapeFix_Wireframe(aComp);
        aFix.SetPrecision(1e-3);
        // OCCT: on a compound the status is the last child's (each child's fix clears it), here the second wire's
        Assert.False(aFix.FixSmallEdges());
        TopoDS_Shape aResult = aFix.Shape();
        Assert.Equal(8, ShapeFixShapes.Count(aResult, TopAbs_EDGE));
        // the second wire needed nothing: the caller's
        Assert.True(ShapeFixShapes.HasTShapeOf(aResult, ShapeFixShapes.SubShapes(aComp, TopAbs_WIRE)[1], TopAbs_WIRE));
    }
}
