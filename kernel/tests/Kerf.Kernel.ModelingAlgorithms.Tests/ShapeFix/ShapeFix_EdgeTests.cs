// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Engine.Healing.Extend.ExtendStatus;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// ShapeFix_Edge's fixes in place on edges the test owns (pcurves added on a cylinder and a seam, removed when
/// inconsistent; 3D curves added and removed; same parameter; vertex tolerances; reversed pcurves), and the public twins
/// on frozen edges: the input unchanged, the result frozen, unchanged parts the caller's own, a no-op the input itself.
/// </summary>
public class ShapeFix_EdgeTests
{
    private const double Conf = 1e-7;
    private const double R = 5.0, H = 10.0;

    private static PlaneSurface XOY() => new(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)));

    private static Face PlaneFace()
    {
        var aB = new BrepBuilder();
        Face aF = new();
        aB.MakeFace(ref aF, XOY(), Conf);
        return aF;
    }

    // the circle at height h around Z, one vertex, no pcurve
    private static Edge Circle(double h)
    {
        var aC = new CircleCurve3D(new Frame3D(new Point3D(0, 0, h), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), R);
        return TestShapes.Edge(aC, 0.0, 2.0 * Math.PI);
    }

    [Fact]
    public void FixAddPCurve_OnCylinder_ProjectsTheCircle()
    {
        Face aF = TestShapes.CylinderFace(R, H);
        Edge anE = Circle(H);
        var sfe = new EdgeFixer();
        Assert.True(sfe.FixAddPCurve(anE, aF, false));
        Assert.True(sfe.Status(ExtendStatus.DONE1));
        Assert.False(sfe.Status(ExtendStatus.Fail));
        Curve2D? aPC = Tool.CurveOnSurface(anE, aF, out double f, out double l);
        Assert.NotNull(aPC);
        Point2D aP = aPC!.Value((f + l) / 2);
        Assert.Equal(H, aP.Y(), 1e-6);
        Assert.Equal(Math.PI, aP.X(), 1e-6);
        // a second call does nothing
        Assert.False(sfe.FixAddPCurve(anE, aF, false));
    }

    [Fact]
    public void FixAddPCurve_OnPlane_IsNotComputed()
    {
        Edge anE = TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(10, 0, 0));
        var sfe = new EdgeFixer();
        // on a plane the pcurve is computed on the fly, so HasPCurve is already true; a plane is never projected
        Assert.False(sfe.FixAddPCurve(anE, XOY(), new Placement(), false));
    }

    [Fact]
    public void FixAddPCurve_Seam_GivesTwoPCurvesAPeriodApart()
    {
        Face aF = TestShapes.CylinderFace(R, H);
        Edge aSeam = TestShapes.Edge(new LineCurve3D(new Point3D(R, 0, 0), new Direction3D(0, 0, 1)), 0.0, H);
        var sfe = new EdgeFixer();
        Assert.True(sfe.FixAddPCurve(aSeam, aF, true));
        Assert.True(Tool.IsClosed(aSeam, aF));
        Curve2D aC1 = Tool.CurveOnSurface(aSeam, aF, out double f, out _)!;
        Curve2D aC2 = Tool.CurveOnSurface(ShapeCasts.Edge(aSeam.Reversed()), aF, out _, out _)!;
        Assert.Equal(2.0 * Math.PI, Math.Abs(aC1.Value(f).X() - aC2.Value(f).X()), 1e-9);
    }

    [Fact]
    public void FixRemovePCurve_RemovesAPCurveOffTheVertices()
    {
        Face aF = TestShapes.CylinderFace(R, H);
        Edge anE = Circle(H);
        var aB = new BrepBuilder();
        // a pcurve at the bottom of the cylinder: its ends are far from the edge's vertex at the top
        aB.UpdateEdge(anE, new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0)), aF, Conf);
        var sfe = new EdgeFixer();
        Assert.True(sfe.FixRemovePCurve(anE, aF));
        Assert.Null(Tool.CurveOnSurface(anE, aF, out _, out _));
    }

    [Fact]
    public void FixRemoveCurve3d_AndFixAddCurve3d()
    {
        // a 3D line off the vertices: removed
        var aB = new BrepBuilder();
        Edge anE = new();
        aB.MakeEdge(ref anE, new LineCurve3D(new Point3D(0, 5, 0), new Direction3D(1, 0, 0)), Conf);
        aB.Add(anE, TestShapes.Vertex(new Point3D(0, 0, 0)).Oriented(Forward));
        aB.Add(anE, TestShapes.Vertex(new Point3D(10, 0, 0)).Oriented(Reversed));
        aB.Range(anE, 0, 10);
        var sfe = new EdgeFixer();
        Assert.True(sfe.FixRemoveCurve3d(anE));
        Assert.False(new EdgeAnalysis().HasCurve3d(anE));

        // a pcurve on the plane: the 3D curve is built from it
        Face aF = PlaneFace();
        aB.UpdateEdge(anE, new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0)), aF, Conf);
        aB.Range(anE, aF, 0, 10);
        Assert.True(sfe.FixAddCurve3d(anE));
        Assert.True(sfe.Status(ExtendStatus.DONE1));
        Curve3D? aC = Tool.Curve(anE, out double f, out double l);
        Assert.NotNull(aC);
        Assert.True(aC!.Value(l).Distance(new Point3D(10, 0, 0)) < 1e-7);
        Assert.False(sfe.FixAddCurve3d(anE));
    }

    [Fact]
    public void FixSameParameter_MakesTheEdgeSameRangeAndSameParameter()
    {
        Face aF = PlaneFace();
        Edge anE = TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(10, 0, 0));
        var aB = new BrepBuilder();
        aB.UpdateEdge(anE, new LineCurve2D(new Point2D(-10, 0), new Direction2D(1, 0)), aF, Conf);
        aB.Range(anE, aF, 10, 20);
        aB.SameRange(anE, false);
        aB.SameParameter(anE, false);
        var sfe = new EdgeFixer();
        sfe.FixSameParameter(anE, aF);
        Assert.True(Tool.SameRange(anE));
        Assert.True(Tool.SameParameter(anE));
        Assert.True(Tool.Tolerance(anE) < 1e-6);
        Tool.Range(anE, aF, out double pf, out double pl);
        Assert.Equal(0.0, pf, 1e-9);
        Assert.Equal(10.0, pl, 1e-9);
    }

    [Fact]
    public void FixSameParameter_Degenerated_SetsTheFlag()
    {
        var aB = new BrepBuilder();
        Edge anE = new();
        aB.MakeEdge(ref anE);
        Vertex aV = TestShapes.Vertex(new Point3D(0, 0, 0));
        aB.Add(anE, aV.Oriented(Forward));
        aB.Add(anE, aV.Oriented(Reversed));
        aB.Degenerated(anE, true);
        aB.SameParameter(anE, false);
        Assert.False(new EdgeFixer().FixSameParameter(anE));
        Assert.True(Tool.SameParameter(anE));
    }

    // a segment (0,0,0)-(10,0,0) whose first vertex is 0.01 off its curve
    private static Edge OffVertexEdge()
    {
        var aB = new BrepBuilder();
        Edge anE = new();
        aB.MakeEdge(ref anE, new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), Conf);
        aB.Add(anE, TestShapes.Vertex(new Point3D(0, 0.01, 0)).Oriented(Forward));
        aB.Add(anE, TestShapes.Vertex(new Point3D(10, 0, 0)).Oriented(Reversed));
        aB.Range(anE, 0, 10);
        return anE;
    }

    [Fact]
    public void FixVertexTolerance_InPlace_RaisesTheVertex()
    {
        Edge anE = OffVertexEdge();
        var sfe = new EdgeFixer();
        Assert.True(sfe.FixVertexTolerance(anE));
        Assert.True(sfe.Status(ExtendStatus.Done));
        Assert.True(Tool.Tolerance(Exploration.FirstVertex(anE)) >= 0.01);
        Assert.Equal(Conf, Tool.Tolerance(Exploration.LastVertex(anE)), 1e-12);
    }

    [Fact]
    public void VertexToleranceFixed_FrozenEdge_CopiesAndKeepsTheUnchangedVertex()
    {
        Edge anE = TestShapes.Frozen(OffVertexEdge());
        ShapeSnapshot aSnap = ShapeSnapshot.Of(anE);
        var sfe = new EdgeFixer();
        Edge aRes = sfe.VertexToleranceFixed(anE, out bool isDone);
        Assert.True(isDone);
        aSnap.AssertUnchanged(anE);
        Assert.True(aRes.TShape()!.IsFrozen);
        Assert.NotSame(anE.TShape(), aRes.TShape());
        Assert.True(Tool.Tolerance(Exploration.FirstVertex(aRes)) >= 0.01);
        // the last vertex didn't change: it is the caller's
        Assert.Same(Exploration.LastVertex(anE).TShape(), Exploration.LastVertex(aRes).TShape());
    }

    [Fact]
    public void VertexToleranceFixed_WithContext_LeavesTheEdgeAndRecordsTheVertex()
    {
        Edge anE = TestShapes.Frozen(OffVertexEdge());
        var aContext = new ReShapeBuilder();
        var sfe = new EdgeFixer();
        sfe.SetContext(aContext);
        Edge aRes = sfe.VertexToleranceFixed(anE, out bool isDone);
        Assert.True(isDone);
        Assert.Same(anE.TShape(), aRes.TShape());
        Vertex aV = Exploration.FirstVertex(anE);
        Shape aNewV = aContext.Value(aV);
        Assert.NotSame(aV.TShape(), aNewV.TShape());
        Assert.True(Tool.Tolerance(ShapeCasts.Vertex(aNewV)) >= 0.01);
        Assert.True(Tool.Tolerance(aV) < 1e-6);
    }

    // a half circle at the top of the cylinder whose pcurve runs backwards (CheckCurve3dWithPCurve skips planes)
    private static (Edge E, Face F) ReversedPCurveEdge()
    {
        Face aF = TestShapes.CylinderFace(R, H);
        var aC = new CircleCurve3D(new Frame3D(new Point3D(0, 0, H), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), R);
        Edge anE = TestShapes.Edge(aC, 0.0, Math.PI);
        new BrepBuilder().UpdateEdge(anE, new LineCurve2D(new Point2D(Math.PI, H), new Direction2D(-1, 0)), aF, Conf);
        return (anE, aF);
    }

    [Fact]
    public void FixReversed2d_ReversesThePCurve()
    {
        (Edge anE, Face aF) = ReversedPCurveEdge();
        var sfe = new EdgeFixer();
        Assert.True(sfe.FixReversed2d(anE, aF));
        Curve2D aPC = Tool.CurveOnSurface(anE, aF, out double f, out double l)!;
        Assert.True(aPC.Value(f).Distance(new Point2D(0, H)) < 1e-9);
        Assert.True(aPC.Value(l).Distance(new Point2D(Math.PI, H)) < 1e-9);
        Assert.False(sfe.FixReversed2d(anE, aF));
    }

    [Fact]
    public void Reversed2dFixed_FrozenEdge_DoesntTouchTheSharedPCurve()
    {
        (Edge anE, Face aF) = ReversedPCurveEdge();
        anE = TestShapes.Frozen(anE);
        Curve2D aOldPC = Tool.CurveOnSurface(anE, aF, out _, out _)!;
        ShapeSnapshot aSnap = ShapeSnapshot.Of(anE);
        Edge aRes = new EdgeFixer().Reversed2dFixed(anE, aF, out bool isDone);
        Assert.True(isDone);
        aSnap.AssertUnchanged(anE);
        Assert.True(aOldPC.Value(0).Distance(new Point2D(Math.PI, H)) < 1e-12);
        Curve2D aPC = Tool.CurveOnSurface(aRes, aF, out double f, out _)!;
        Assert.True(aPC.Value(f).Distance(new Point2D(0, H)) < 1e-9);
        Assert.True(aRes.TShape()!.IsFrozen);
    }

    [Fact]
    public void AddPCurveFixed_FrozenEdge_KeepsTheCallersVertex()
    {
        Face aF = TestShapes.Frozen(TestShapes.CylinderFace(R, H));
        Edge anE = TestShapes.Frozen(Circle(H));
        ShapeSnapshot aSnap = ShapeSnapshot.Of(anE);
        var sfe = new EdgeFixer();
        Edge aRes = sfe.AddPCurveFixed(anE, aF, false, out bool isDone);
        Assert.True(isDone);
        aSnap.AssertUnchanged(anE);
        Assert.Null(Tool.CurveOnSurface(anE, aF, out _, out _));
        Assert.NotNull(Tool.CurveOnSurface(aRes, aF, out _, out _));
        Assert.True(aRes.TShape()!.IsFrozen);
        Assert.Same(Exploration.FirstVertex(anE).TShape(), Exploration.FirstVertex(aRes).TShape());

        // nothing to do: the input itself
        Edge aRes2 = sfe.AddPCurveFixed(aRes, aF, false, out isDone);
        Assert.False(isDone);
        Assert.Same(aRes.TShape(), aRes2.TShape());
    }

    [Fact]
    public void SameParameterFixed_And_Curve3dTwins_OnFrozenEdges()
    {
        // an edge already SameParameter: nothing changes
        Edge anE = TestShapes.Frozen(TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(10, 0, 0)));
        var sfe = new EdgeFixer();
        Edge aRes = sfe.SameParameterFixed(anE, out _);
        Assert.Same(anE.TShape(), aRes.TShape());

        // RemoveCurve3dFixed on a consistent edge: the input itself
        aRes = sfe.RemoveCurve3dFixed(anE, out bool isDone);
        Assert.False(isDone);
        Assert.Same(anE.TShape(), aRes.TShape());

        // AddCurve3dFixed on an edge that has one: the input itself
        aRes = sfe.AddCurve3dFixed(anE, out isDone);
        Assert.False(isDone);
        Assert.Same(anE.TShape(), aRes.TShape());
    }

    [Fact]
    public void RemovePCurveFixed_FrozenEdge()
    {
        Face aF = TestShapes.CylinderFace(R, H);
        Edge anE = Circle(H);
        new BrepBuilder().UpdateEdge(anE, new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0)), aF, Conf);
        anE = TestShapes.Frozen(anE);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(anE);
        Edge aRes = new EdgeFixer().RemovePCurveFixed(anE, aF, out bool isDone);
        Assert.True(isDone);
        aSnap.AssertUnchanged(anE);
        Assert.NotNull(Tool.CurveOnSurface(anE, aF, out _, out _));
        Assert.Null(Tool.CurveOnSurface(aRes, aF, out _, out _));
    }

    [Fact]
    public void Projector_IsTheToolsOwn()
    {
        var sfe = new EdgeFixer();
        Assert.Same(sfe.Projector(), sfe.Projector());
        Assert.Null(sfe.Context());
        Assert.True(sfe.Status(ExtendStatus.OK));
    }

    [Fact]
    public void ShapeFix_EdgeProjAux_FindsTheEndParameters()
    {
        Face aF = TestShapes.CylinderFace(R, H);
        Edge anE = Circle(H);
        var sfe = new EdgeFixer();
        sfe.FixAddPCurve(anE, aF, false);
        var aux = new EdgeProjAuxFixer(aF, anE);
        aux.Compute(1e-7);
        Assert.True(aux.IsFirstDone);
        Assert.True(aux.IsLastDone);
        Assert.True(aux.FirstParam() < aux.LastParam());
        Assert.Equal(2.0 * Math.PI, aux.LastParam() - aux.FirstParam(), 1e-6);
        Assert.False(aux.IsIso(null));
    }
}
