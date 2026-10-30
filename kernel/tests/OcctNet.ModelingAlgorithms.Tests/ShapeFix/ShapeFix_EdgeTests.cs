// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.ModelingData.Tests;
using static Occt.ShapeExtend_Status;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// ShapeFix_Edge's fixes in place on edges the test owns (pcurves added on a cylinder and a seam, removed when
/// inconsistent; 3D curves added and removed; same parameter; vertex tolerances; reversed pcurves), and the public twins
/// on frozen edges: the input unchanged, the result frozen, unchanged parts the caller's own, a no-op the input itself.
/// </summary>
public class ShapeFix_EdgeTests
{
    private const double Conf = 1e-7;
    private const double R = 5.0, H = 10.0;

    private static Geom_Plane XOY() => new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)));

    private static TopoDS_Face PlaneFace()
    {
        var aB = new BRep_Builder();
        TopoDS_Face aF = new();
        aB.MakeFace(ref aF, XOY(), Conf);
        return aF;
    }

    // the circle at height h around Z, one vertex, no pcurve
    private static TopoDS_Edge Circle(double h)
    {
        var aC = new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, h), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), R);
        return TestShapes.Edge(aC, 0.0, 2.0 * Math.PI);
    }

    [Fact]
    public void FixAddPCurve_OnCylinder_ProjectsTheCircle()
    {
        TopoDS_Face aF = TestShapes.CylinderFace(R, H);
        TopoDS_Edge anE = Circle(H);
        var sfe = new ShapeFix_Edge();
        Assert.True(sfe.FixAddPCurve(anE, aF, false));
        Assert.True(sfe.Status(ShapeExtend_DONE1));
        Assert.False(sfe.Status(ShapeExtend_FAIL));
        Geom2d_Curve? aPC = BRep_Tool.CurveOnSurface(anE, aF, out double f, out double l);
        Assert.NotNull(aPC);
        gp_Pnt2d aP = aPC!.Value((f + l) / 2);
        Assert.Equal(H, aP.Y(), 1e-6);
        Assert.Equal(Math.PI, aP.X(), 1e-6);
        // a second call does nothing
        Assert.False(sfe.FixAddPCurve(anE, aF, false));
    }

    [Fact]
    public void FixAddPCurve_OnPlane_IsNotComputed()
    {
        TopoDS_Edge anE = TestShapes.Edge(new gp_Pnt(0, 0, 0), new gp_Pnt(10, 0, 0));
        var sfe = new ShapeFix_Edge();
        // on a plane the pcurve is computed on the fly, so HasPCurve is already true; a plane is never projected
        Assert.False(sfe.FixAddPCurve(anE, XOY(), new TopLoc_Location(), false));
    }

    [Fact]
    public void FixAddPCurve_Seam_GivesTwoPCurvesAPeriodApart()
    {
        TopoDS_Face aF = TestShapes.CylinderFace(R, H);
        TopoDS_Edge aSeam = TestShapes.Edge(new Geom_Line(new gp_Pnt(R, 0, 0), new gp_Dir(0, 0, 1)), 0.0, H);
        var sfe = new ShapeFix_Edge();
        Assert.True(sfe.FixAddPCurve(aSeam, aF, true));
        Assert.True(BRep_Tool.IsClosed(aSeam, aF));
        Geom2d_Curve aC1 = BRep_Tool.CurveOnSurface(aSeam, aF, out double f, out _)!;
        Geom2d_Curve aC2 = BRep_Tool.CurveOnSurface(TopoDS.Edge(aSeam.Reversed()), aF, out _, out _)!;
        Assert.Equal(2.0 * Math.PI, Math.Abs(aC1.Value(f).X() - aC2.Value(f).X()), 1e-9);
    }

    [Fact]
    public void FixRemovePCurve_RemovesAPCurveOffTheVertices()
    {
        TopoDS_Face aF = TestShapes.CylinderFace(R, H);
        TopoDS_Edge anE = Circle(H);
        var aB = new BRep_Builder();
        // a pcurve at the bottom of the cylinder: its ends are far from the edge's vertex at the top
        aB.UpdateEdge(anE, new Geom2d_Line(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), aF, Conf);
        var sfe = new ShapeFix_Edge();
        Assert.True(sfe.FixRemovePCurve(anE, aF));
        Assert.Null(BRep_Tool.CurveOnSurface(anE, aF, out _, out _));
    }

    [Fact]
    public void FixRemoveCurve3d_AndFixAddCurve3d()
    {
        // a 3D line off the vertices: removed
        var aB = new BRep_Builder();
        TopoDS_Edge anE = new();
        aB.MakeEdge(ref anE, new Geom_Line(new gp_Pnt(0, 5, 0), new gp_Dir(1, 0, 0)), Conf);
        aB.Add(anE, TestShapes.Vertex(new gp_Pnt(0, 0, 0)).Oriented(TopAbs_FORWARD));
        aB.Add(anE, TestShapes.Vertex(new gp_Pnt(10, 0, 0)).Oriented(TopAbs_REVERSED));
        aB.Range(anE, 0, 10);
        var sfe = new ShapeFix_Edge();
        Assert.True(sfe.FixRemoveCurve3d(anE));
        Assert.False(new ShapeAnalysis_Edge().HasCurve3d(anE));

        // a pcurve on the plane: the 3D curve is built from it
        TopoDS_Face aF = PlaneFace();
        aB.UpdateEdge(anE, new Geom2d_Line(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), aF, Conf);
        aB.Range(anE, aF, 0, 10);
        Assert.True(sfe.FixAddCurve3d(anE));
        Assert.True(sfe.Status(ShapeExtend_DONE1));
        Geom_Curve? aC = BRep_Tool.Curve(anE, out double f, out double l);
        Assert.NotNull(aC);
        Assert.True(aC!.Value(l).Distance(new gp_Pnt(10, 0, 0)) < 1e-7);
        Assert.False(sfe.FixAddCurve3d(anE));
    }

    [Fact]
    public void FixSameParameter_MakesTheEdgeSameRangeAndSameParameter()
    {
        TopoDS_Face aF = PlaneFace();
        TopoDS_Edge anE = TestShapes.Edge(new gp_Pnt(0, 0, 0), new gp_Pnt(10, 0, 0));
        var aB = new BRep_Builder();
        aB.UpdateEdge(anE, new Geom2d_Line(new gp_Pnt2d(-10, 0), new gp_Dir2d(1, 0)), aF, Conf);
        aB.Range(anE, aF, 10, 20);
        aB.SameRange(anE, false);
        aB.SameParameter(anE, false);
        var sfe = new ShapeFix_Edge();
        sfe.FixSameParameter(anE, aF);
        Assert.True(BRep_Tool.SameRange(anE));
        Assert.True(BRep_Tool.SameParameter(anE));
        Assert.True(BRep_Tool.Tolerance(anE) < 1e-6);
        BRep_Tool.Range(anE, aF, out double pf, out double pl);
        Assert.Equal(0.0, pf, 1e-9);
        Assert.Equal(10.0, pl, 1e-9);
    }

    [Fact]
    public void FixSameParameter_Degenerated_SetsTheFlag()
    {
        var aB = new BRep_Builder();
        TopoDS_Edge anE = new();
        aB.MakeEdge(ref anE);
        TopoDS_Vertex aV = TestShapes.Vertex(new gp_Pnt(0, 0, 0));
        aB.Add(anE, aV.Oriented(TopAbs_FORWARD));
        aB.Add(anE, aV.Oriented(TopAbs_REVERSED));
        aB.Degenerated(anE, true);
        aB.SameParameter(anE, false);
        Assert.False(new ShapeFix_Edge().FixSameParameter(anE));
        Assert.True(BRep_Tool.SameParameter(anE));
    }

    // a segment (0,0,0)-(10,0,0) whose first vertex is 0.01 off its curve
    private static TopoDS_Edge OffVertexEdge()
    {
        var aB = new BRep_Builder();
        TopoDS_Edge anE = new();
        aB.MakeEdge(ref anE, new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0)), Conf);
        aB.Add(anE, TestShapes.Vertex(new gp_Pnt(0, 0.01, 0)).Oriented(TopAbs_FORWARD));
        aB.Add(anE, TestShapes.Vertex(new gp_Pnt(10, 0, 0)).Oriented(TopAbs_REVERSED));
        aB.Range(anE, 0, 10);
        return anE;
    }

    [Fact]
    public void FixVertexTolerance_InPlace_RaisesTheVertex()
    {
        TopoDS_Edge anE = OffVertexEdge();
        var sfe = new ShapeFix_Edge();
        Assert.True(sfe.FixVertexTolerance(anE));
        Assert.True(sfe.Status(ShapeExtend_DONE));
        Assert.True(BRep_Tool.Tolerance(TopExp.FirstVertex(anE)) >= 0.01);
        Assert.Equal(Conf, BRep_Tool.Tolerance(TopExp.LastVertex(anE)), 1e-12);
    }

    [Fact]
    public void VertexToleranceFixed_FrozenEdge_CopiesAndKeepsTheUnchangedVertex()
    {
        TopoDS_Edge anE = TestShapes.Frozen(OffVertexEdge());
        ShapeSnapshot aSnap = ShapeSnapshot.Of(anE);
        var sfe = new ShapeFix_Edge();
        TopoDS_Edge aRes = sfe.VertexToleranceFixed(anE, out bool isDone);
        Assert.True(isDone);
        aSnap.AssertUnchanged(anE);
        Assert.True(aRes.TShape()!.IsFrozen());
        Assert.NotSame(anE.TShape(), aRes.TShape());
        Assert.True(BRep_Tool.Tolerance(TopExp.FirstVertex(aRes)) >= 0.01);
        // the last vertex didn't change: it is the caller's
        Assert.Same(TopExp.LastVertex(anE).TShape(), TopExp.LastVertex(aRes).TShape());
    }

    [Fact]
    public void VertexToleranceFixed_WithContext_LeavesTheEdgeAndRecordsTheVertex()
    {
        TopoDS_Edge anE = TestShapes.Frozen(OffVertexEdge());
        var aContext = new ShapeBuild_ReShape();
        var sfe = new ShapeFix_Edge();
        sfe.SetContext(aContext);
        TopoDS_Edge aRes = sfe.VertexToleranceFixed(anE, out bool isDone);
        Assert.True(isDone);
        Assert.Same(anE.TShape(), aRes.TShape());
        TopoDS_Vertex aV = TopExp.FirstVertex(anE);
        TopoDS_Shape aNewV = aContext.Value(aV);
        Assert.NotSame(aV.TShape(), aNewV.TShape());
        Assert.True(BRep_Tool.Tolerance(TopoDS.Vertex(aNewV)) >= 0.01);
        Assert.True(BRep_Tool.Tolerance(aV) < 1e-6);
    }

    // a half circle at the top of the cylinder whose pcurve runs backwards (CheckCurve3dWithPCurve skips planes)
    private static (TopoDS_Edge E, TopoDS_Face F) ReversedPCurveEdge()
    {
        TopoDS_Face aF = TestShapes.CylinderFace(R, H);
        var aC = new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, H), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), R);
        TopoDS_Edge anE = TestShapes.Edge(aC, 0.0, Math.PI);
        new BRep_Builder().UpdateEdge(anE, new Geom2d_Line(new gp_Pnt2d(Math.PI, H), new gp_Dir2d(-1, 0)), aF, Conf);
        return (anE, aF);
    }

    [Fact]
    public void FixReversed2d_ReversesThePCurve()
    {
        (TopoDS_Edge anE, TopoDS_Face aF) = ReversedPCurveEdge();
        var sfe = new ShapeFix_Edge();
        Assert.True(sfe.FixReversed2d(anE, aF));
        Geom2d_Curve aPC = BRep_Tool.CurveOnSurface(anE, aF, out double f, out double l)!;
        Assert.True(aPC.Value(f).Distance(new gp_Pnt2d(0, H)) < 1e-9);
        Assert.True(aPC.Value(l).Distance(new gp_Pnt2d(Math.PI, H)) < 1e-9);
        Assert.False(sfe.FixReversed2d(anE, aF));
    }

    [Fact]
    public void Reversed2dFixed_FrozenEdge_DoesntTouchTheSharedPCurve()
    {
        (TopoDS_Edge anE, TopoDS_Face aF) = ReversedPCurveEdge();
        anE = TestShapes.Frozen(anE);
        Geom2d_Curve aOldPC = BRep_Tool.CurveOnSurface(anE, aF, out _, out _)!;
        ShapeSnapshot aSnap = ShapeSnapshot.Of(anE);
        TopoDS_Edge aRes = new ShapeFix_Edge().Reversed2dFixed(anE, aF, out bool isDone);
        Assert.True(isDone);
        aSnap.AssertUnchanged(anE);
        Assert.True(aOldPC.Value(0).Distance(new gp_Pnt2d(Math.PI, H)) < 1e-12);
        Geom2d_Curve aPC = BRep_Tool.CurveOnSurface(aRes, aF, out double f, out _)!;
        Assert.True(aPC.Value(f).Distance(new gp_Pnt2d(0, H)) < 1e-9);
        Assert.True(aRes.TShape()!.IsFrozen());
    }

    [Fact]
    public void AddPCurveFixed_FrozenEdge_KeepsTheCallersVertex()
    {
        TopoDS_Face aF = TestShapes.Frozen(TestShapes.CylinderFace(R, H));
        TopoDS_Edge anE = TestShapes.Frozen(Circle(H));
        ShapeSnapshot aSnap = ShapeSnapshot.Of(anE);
        var sfe = new ShapeFix_Edge();
        TopoDS_Edge aRes = sfe.AddPCurveFixed(anE, aF, false, out bool isDone);
        Assert.True(isDone);
        aSnap.AssertUnchanged(anE);
        Assert.Null(BRep_Tool.CurveOnSurface(anE, aF, out _, out _));
        Assert.NotNull(BRep_Tool.CurveOnSurface(aRes, aF, out _, out _));
        Assert.True(aRes.TShape()!.IsFrozen());
        Assert.Same(TopExp.FirstVertex(anE).TShape(), TopExp.FirstVertex(aRes).TShape());

        // nothing to do: the input itself
        TopoDS_Edge aRes2 = sfe.AddPCurveFixed(aRes, aF, false, out isDone);
        Assert.False(isDone);
        Assert.Same(aRes.TShape(), aRes2.TShape());
    }

    [Fact]
    public void SameParameterFixed_And_Curve3dTwins_OnFrozenEdges()
    {
        // an edge already SameParameter: nothing changes
        TopoDS_Edge anE = TestShapes.Frozen(TestShapes.Edge(new gp_Pnt(0, 0, 0), new gp_Pnt(10, 0, 0)));
        var sfe = new ShapeFix_Edge();
        TopoDS_Edge aRes = sfe.SameParameterFixed(anE, out _);
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
        TopoDS_Face aF = TestShapes.CylinderFace(R, H);
        TopoDS_Edge anE = Circle(H);
        new BRep_Builder().UpdateEdge(anE, new Geom2d_Line(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), aF, Conf);
        anE = TestShapes.Frozen(anE);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(anE);
        TopoDS_Edge aRes = new ShapeFix_Edge().RemovePCurveFixed(anE, aF, out bool isDone);
        Assert.True(isDone);
        aSnap.AssertUnchanged(anE);
        Assert.NotNull(BRep_Tool.CurveOnSurface(anE, aF, out _, out _));
        Assert.Null(BRep_Tool.CurveOnSurface(aRes, aF, out _, out _));
    }

    [Fact]
    public void Projector_IsTheToolsOwn()
    {
        var sfe = new ShapeFix_Edge();
        Assert.Same(sfe.Projector(), sfe.Projector());
        Assert.Null(sfe.Context());
        Assert.True(sfe.Status(ShapeExtend_OK));
    }

    [Fact]
    public void ShapeFix_EdgeProjAux_FindsTheEndParameters()
    {
        TopoDS_Face aF = TestShapes.CylinderFace(R, H);
        TopoDS_Edge anE = Circle(H);
        var sfe = new ShapeFix_Edge();
        sfe.FixAddPCurve(anE, aF, false);
        var aux = new ShapeFix_EdgeProjAux(aF, anE);
        aux.Compute(1e-7);
        Assert.True(aux.IsFirstDone());
        Assert.True(aux.IsLastDone());
        Assert.True(aux.FirstParam() < aux.LastParam());
        Assert.Equal(2.0 * Math.PI, aux.LastParam() - aux.FirstParam(), 1e-6);
        Assert.False(aux.IsIso(null));
    }
}
