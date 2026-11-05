// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Text.RegularExpressions;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Known answers of ShapeBuild (PlaneXOY), ShapeBuild_Vertex, ShapeBuild_Edge and ShapeAnalysis_TransferParameters;
/// the twins of ShapeBuild_Edge's in-place methods leave their frozen inputs unchanged and give the in-place result.
/// </summary>
public class ShapeBuildTests
{
    private const double Conf = 1e-7;

    private static PlaneSurface XOY() => new(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)));

    // BRep text with the flag lines masked: kept sub-shapes keep their Free and Modified bits, copies have their own
    private static string Text(in Shape theShape)
    {
        var aWriter = new StringWriter();
        BrepTools.Write(theShape, aWriter);
        return Regex.Replace(aWriter.ToString(), "(?m)^[01]{7}$", "flags");
    }

    private static void AssertTwin(in Edge theInput, Func<Edge, Edge> theTwin, Action<Edge> theInPlace)
    {
        ShapeSnapshot aSnap = ShapeSnapshot.Of(theInput);
        Edge aResult = theTwin(theInput);
        aSnap.AssertUnchanged(theInput);
        Assert.True(aResult.TShape()!.IsFrozen);

        Edge aCopy = ShapeCasts.Edge(TestCopies.ExactCopyForWrite(theInput, out _));
        theInPlace(aCopy);
        Assert.Equal(Text(aCopy), Text(aResult));
    }

    private static (Edge E, Face F) BoxEdge()
    {
        Shape aBox = new MakeBox(10, 20, 30).Shape();
        var aFExp = new Explorer(aBox, ShapeType.Face);
        Face aF = ShapeCasts.Face(aFExp.Current());
        Edge anE = ShapeCasts.Edge(new Explorer(aF, ShapeType.Edge).Current());
        return (anE, aF);
    }

    // a segment (0,0,0)-(10,0,0) on the plane z = 0 with a pcurve over [10, 20] (not same range); unfrozen
    private static (Edge E, Face F) ShiftedEdge()
    {
        var aB = new BrepBuilder();
        Face aF = new();
        aB.MakeFace(ref aF, XOY(), Conf);
        Edge anE = TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(10, 0, 0));
        aB.UpdateEdge(anE, new LineCurve2D(new Point2D(-10, 0), new Direction2D(1, 0)), aF, Conf);
        aB.Range(anE, aF, 10, 20);
        aB.SameRange(anE, false);
        Wire aW = new();
        aB.MakeWire(ref aW);
        aB.Add(aW, anE);
        aB.Add(aF, aW);
        return (anE, aF);
    }

    // an edge on the plane z = 0 with a pcurve only (no 3D curve), from (0,0) to (4,0)
    private static (Edge E, Face F) PCurveOnlyEdge()
    {
        var aB = new BrepBuilder();
        Face aF = new();
        aB.MakeFace(ref aF, XOY(), Conf);
        Edge anE = new();
        aB.MakeEdge(ref anE);
        aB.UpdateEdge(anE, new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0)), aF, Conf);
        aB.Add(anE, TestShapes.Vertex(new Point3D(0, 0, 0)).Oriented(Forward));
        aB.Add(anE, TestShapes.Vertex(new Point3D(4, 0, 0)).Oriented(Reversed));
        aB.Range(anE, aF, 0, 4);
        Wire aW = new();
        aB.MakeWire(ref aW);
        aB.Add(aW, anE);
        aB.Add(aF, aW);
        return (anE, aF);
    }

    [Fact]
    public void PlaneXOY_is_one_frozen_plane()
    {
        PlaneSurface aP = ShapeBuildBuilder.PlaneXOY();
        Assert.Same(aP, ShapeBuildBuilder.PlaneXOY());
        Assert.True(aP.IsFrozen);
        Assert.True(aP.Axis.Direction.IsEqual(new Direction3D(0, 0, 1), 0));
        Assert.Equal(0.0, aP.Location.Z);
    }

    [Fact]
    public void CombineVertex_covers_both_vertices()
    {
        var sbv = new VertexBuilder();
        // the first covers the second
        Vertex aV = sbv.CombineVertex(new Point3D(0, 0, 0), new Point3D(0.5, 0, 0), 1.0, 0.1);
        Assert.True(aV.TShape()!.IsFrozen);
        Assert.Equal(new Point3D(0, 0, 0), Tool.Pnt(aV));
        Assert.Equal(1.0 * 1.0001, Tool.Tolerance(aV));
        // the second covers the first
        aV = sbv.CombineVertex(new Point3D(0, 0, 0), new Point3D(0.5, 0, 0), 0.1, 1.0, 1.0);
        Assert.Equal(new Point3D(0.5, 0, 0), Tool.Pnt(aV));
        Assert.Equal(1.0, Tool.Tolerance(aV));
        // neither: the smallest sphere around both
        aV = sbv.CombineVertex(TestShapes.Vertex(new Point3D(0, 0, 0)), TestShapes.Vertex(new Point3D(1, 0, 0)), 1.0);
        Assert.Equal(0.5, Tool.Pnt(aV).X, 1e-15);
        Assert.Equal(0.5 * (1 + 2 * global::Kerf.Engine.Foundation.Precision.Confusion()), Tool.Tolerance(aV), 1e-15);
        // different tolerances: the centre moves towards the larger sphere
        aV = sbv.CombineVertex(new Point3D(0, 0, 0), new Point3D(2, 0, 0), 0.0, 1.0, 1.0);
        Assert.Equal(1.5, Tool.Tolerance(aV));
        Assert.Equal(1.5, Tool.Pnt(aV).X, 1e-15);
    }

    [Fact]
    public void CopyReplaceVertices_and_Copy_make_new_frozen_edges()
    {
        (Edge anE, Face aF) = BoxEdge();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(anE);
        var sbe = new BuildEdgeBuilder();
        Vertex aV1 = TestShapes.Vertex(new Point3D(-1, -1, -1));
        Edge aNew = sbe.CopyReplaceVertices(anE, aV1, new Vertex());
        aSnap.AssertUnchanged(anE);
        Assert.True(aNew.TShape()!.IsFrozen);
        Assert.False(aNew.IsSame(anE));
        var sae = new EdgeAnalysis();
        Exploration.Vertices(aNew, out Vertex aF1, out Vertex aL1);
        Exploration.Vertices(anE, out Vertex aF0, out Vertex aL0);
        Assert.True(aF1.IsSame(aV1));
        Assert.True(aL1.IsSame(aL0));
        Assert.False(aF1.IsSame(aF0));
        // the curves and ranges are the edge's
        Assert.Same(Tool.Curve(anE, out double f0, out double l0), Tool.Curve(aNew, out double f1, out double l1));
        Assert.Equal((f0, l0), (f1, l1));
        Assert.Same(Tool.CurveOnSurface(anE, aF, out _, out _), Tool.CurveOnSurface(aNew, aF, out _, out _));

        Edge aShared = sbe.Copy(anE);
        Assert.True(aShared.TShape()!.IsFrozen);
        Assert.Same(Tool.CurveOnSurface(anE, aF, out _, out _), Tool.CurveOnSurface(aShared, aF, out _, out _));
        Edge anOwn = sbe.Copy(anE, false);
        Curve2D aPC0 = Tool.CurveOnSurface(anE, aF, out double pf, out double pl)!;
        Curve2D aPC1 = Tool.CurveOnSurface(anOwn, aF, out double qf, out double ql)!;
        Assert.NotSame(aPC0, aPC1);
        Assert.Equal((pf, pl), (qf, ql));
        Assert.Equal(aPC0.Value(pl), aPC1.Value(ql));
        Assert.False(sbe.InternalCopy(anE).TShape()!.IsFrozen);
        aSnap.AssertUnchanged(anE);
    }

    [Fact]
    public void Twins_leave_their_inputs_and_give_the_in_place_results()
    {
        var sbe = new BuildEdgeBuilder();
        (Edge anE, Face aF) = BoxEdge();

        AssertTwin(anE, e => sbe.WithRange3d(e, 1, 2), e => sbe.SetRange3d(e, 1, 2));
        AssertTwin(anE, e => sbe.PCurveRemoved(e, aF), e => sbe.RemovePCurve(e, aF));
        Surface aS = Tool.Surface(aF, out Placement aL)!;
        AssertTwin(anE, e => sbe.PCurveRemoved(e, aS, aL), e => sbe.RemovePCurve(e, aS, aL));
        AssertTwin(anE, e => sbe.PCurveRemoved(e, aS), e => sbe.RemovePCurve(e, aS));
        var aLine = new LineCurve2D(new Point2D(5, 5), new Direction2D(0, 1));
        AssertTwin(anE, e => sbe.PCurveReplaced(e, aLine, aF), e => sbe.ReplacePCurve(e, aLine, aF));
        AssertTwin(anE, e => sbe.Curve3dRemoved(e), e => sbe.RemoveCurve3d(e));

        Edge aRemoved = sbe.Curve3dRemoved(anE);
        Assert.Null(Tool.Curve(aRemoved, out _, out _));
        Assert.NotNull(Tool.Curve(anE, out _, out _));
        // the vertices weren't changed: they are the input's own
        Exploration.Vertices(aRemoved, out Vertex aV1, out _);
        Exploration.Vertices(anE, out Vertex aV0, out _);
        Assert.Same(aV0.TShape(), aV1.TShape());
        // nothing changed: the input itself
        Edge aSame = sbe.WithRange3d(anE, Tool.Curve(anE, out double f, out double l) is not null ? f : 0, l);
        Assert.True(aSame.IsEqual(anE));

        (Edge aShifted, Face aShiftedF) = ShiftedEdge();
        Edge aFrom = TestShapes.Frozen(aShifted);
        Edge aTo = sbe.Copy(aFrom);
        AssertTwin(aTo, e => sbe.RangesCopied(e, aFrom, 0.25, 0.75), e => sbe.CopyRanges(e, aFrom, 0.25, 0.75));
        Edge aRanged = sbe.RangesCopied(aTo, aFrom, 0.25, 0.75);
        Tool.Range(aRanged, out f, out l);
        Assert.Equal((2.5, 7.5), (f, l));
        Tool.Range(aRanged, aShiftedF, out f, out l);
        Assert.Equal((12.5, 17.5), (f, l));

        Edge aBare = TestShapes.Frozen(TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(10, 0, 0)));
        AssertTwin(aBare, e => sbe.PCurvesCopied(e, aFrom), e => sbe.CopyPCurves(e, aFrom));
        Assert.True(new EdgeAnalysis().HasPCurve(sbe.PCurvesCopied(aBare, aFrom), aShiftedF));

        (Edge aPOnly, Face aPF) = PCurveOnlyEdge();
        Edge aFrozenPOnly = TestShapes.Frozen(aPOnly);
        AssertTwin(aFrozenPOnly, e => sbe.Curve3dBuilt(e, out _), e => sbe.BuildCurve3d(e));
        Edge aBuilt = sbe.Curve3dBuilt(aFrozenPOnly, out bool isDone);
        Assert.True(isDone);
        Curve3D aC = Tool.Curve(aBuilt, out f, out l)!;
        Assert.True(aC.Value(l).DistanceTo(new Point3D(4, 0, 0)) < 1e-7);
        _ = aPF;
    }

    [Fact]
    public void PCurveReassigned_moves_the_pcurve_to_another_face()
    {
        var sbe = new BuildEdgeBuilder();
        var aB = new BrepBuilder();
        (Edge anE, Face aF) = ShiftedEdge();
        Face aSub = new();
        aB.MakeFace(ref aSub, new PlaneSurface(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), Conf);
        Edge aFrozenE = TestShapes.Frozen(anE);
        Face aFrozenSub = TestShapes.Frozen(aSub);
        Curve2D aPC = Tool.CurveOnSurface(aFrozenE, aF, out _, out _)!;

        AssertTwin(aFrozenE, e => sbe.PCurveReassigned(e, aF, aFrozenSub, out _), e => sbe.ReassignPCurve(e, aF, aFrozenSub));
        Edge aResult = sbe.PCurveReassigned(aFrozenE, aF, aFrozenSub, out bool isDone);
        Assert.True(isDone);
        Assert.Same(aPC, Tool.CurveOnSurface(aResult, aFrozenSub, out double f, out double l));
        Assert.Equal((10.0, 20.0), (f, l));
        // the old face's pcurve is gone: BRep_Tool projects onto the plane instead
        Assert.NotSame(aPC, Tool.CurveOnSurface(aResult, aF, out _, out _));

        // no pcurve on old (not a plane, so none is computed): false, and the input comes back unchanged
        Face aCyl = ShapeCasts.Face(new Explorer(new ApiMakeCylinder(1, 1).Shape(), ShapeType.Face).Current());
        Edge anOther = TestShapes.Frozen(TestShapes.Edge(new Point3D(0, 0, 5), new Point3D(1, 0, 5)));
        Edge aSame = sbe.PCurveReassigned(anOther, aCyl, aFrozenSub, out isDone);
        Assert.False(isDone);
        Assert.True(aSame.IsEqual(anOther));
    }

    [Fact]
    public void In_place_writers_throw_on_frozen_edges()
    {
        var sbe = new BuildEdgeBuilder();
        (Edge anE, Face aF) = BoxEdge();
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.FailureException>(() => sbe.RemoveCurve3d(anE));
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.FailureException>(() => sbe.SetRange3d(anE, 0, 1));
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.FailureException>(() => sbe.RemovePCurve(anE, aF));
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.FailureException>(() => sbe.CopyRanges(anE, anE));

        // OCCT's BuildCurve3d catches every failure; the port lets the frozen edge's through
        (Edge aPOnly, _) = PCurveOnlyEdge();
        Edge aFrozen = TestShapes.Frozen(aPOnly);
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.FailureException>(() => sbe.BuildCurve3d(aFrozen));
    }

    [Fact]
    public void CopyRanges_shifts_into_the_period()
    {
        var sbe = new BuildEdgeBuilder();
        var aCircle = new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 1);
        Edge aFrom = TestShapes.Edge(aCircle, 7, 8);
        Edge aTo = TestShapes.Edge(aCircle, 0, 1);
        sbe.CopyRanges(aTo, aFrom);
        Tool.Range(aTo, out double f, out double l);
        Assert.Equal(7 - 2 * Math.PI, f, 1e-15);
        Assert.Equal(8 - 2 * Math.PI, l, 1e-15);
        Assert.False(Tool.SameRange(aTo));
        Assert.False(Tool.SameParameter(aTo));

        // within the period: copied as it is
        Edge aTo2 = TestShapes.Edge(aCircle, 0, 1);
        sbe.CopyRanges(aTo2, TestShapes.Edge(aCircle, 2, 3), 0.5, 1);
        Tool.Range(aTo2, out f, out l);
        Assert.Equal((2.5, 3.0), (f, l));
        Assert.True(Tool.SameRange(aTo2));
    }

    [Fact]
    public void TransformPCurve_scales_along_u()
    {
        var sbe = new BuildEdgeBuilder();
        var anId = new Transform2D();

        // a line: stays a line
        var aLine = new LineCurve2D(new Point2D(1, 1), new Direction2D(1, 0));
        double f = 0, l = 1;
        Curve2D aRes = sbe.TransformPCurve(aLine, anId, 2, ref f, ref l);
        LineCurve2D aResLine = Assert.IsType<LineCurve2D>(aRes);
        Assert.Equal((0.0, 2.0), (f, l));
        Assert.Equal(new Point2D(2, 1), aResLine.Value(f));
        Assert.Equal(new Point2D(4, 1), aResLine.Value(l));

        // uFact 1: a copy, transformed
        var aT = new Transform2D();
        aT = aT.WithTranslation(new Vector2D(0, 3));
        f = 0;
        l = 1;
        aRes = sbe.TransformPCurve(aLine, aT, 1, ref f, ref l);
        Assert.NotSame(aLine, aRes);
        Assert.False(aRes.IsFrozen);
        Assert.Equal(new Point2D(1, 4), aRes.Value(0));
        Assert.Equal(new Point2D(1, 1), aLine.Value(0));

        // a circle: a B-spline approximation over the range
        var aCircle = new CircleCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 1);
        f = 0;
        l = Math.PI / 2;
        aRes = sbe.TransformPCurve(aCircle, anId, 2, ref f, ref l);
        BSplineCurve2D aBS = Assert.IsType<BSplineCurve2D>(aRes);
        Assert.True(aBS.Value(f).DistanceTo(new Point2D(2, 0)) < 1e-6);
        Assert.True(aBS.Value(l).DistanceTo(new Point2D(0, 1)) < 1e-6);

        // a Bezier curve: its poles scaled
        var aBez = new BezierCurve2D(NCollection_Array1<Point2D>.From([new Point2D(1, 0), new Point2D(2, 1), new Point2D(3, 0)]));
        f = 0;
        l = 1;
        aRes = sbe.TransformPCurve(aBez, anId, 3, ref f, ref l);
        BezierCurve2D aResBez = Assert.IsType<BezierCurve2D>(aRes);
        Assert.Equal(new Point2D(6, 1), aResBez.Pole(2));
        Assert.Equal(new Point2D(2, 1), aBez.Pole(2));

        // a trimmed B-spline: the basis, its poles scaled
        var aBSp = new BSplineCurve2D(NCollection_Array1<Point2D>.From([new Point2D(0, 0), new Point2D(1, 1)]),
                                           NCollection_Array1<double>.From([0.0, 1.0]), NCollection_Array1<int>.From([2, 2]), 1);
        aBSp.Freeze();
        f = 0.25;
        l = 0.5;
        aRes = sbe.TransformPCurve(new TrimmedCurve2D(aBSp, 0.25, 0.5), anId, 2, ref f, ref l);
        BSplineCurve2D aResBSp = Assert.IsType<BSplineCurve2D>(aRes);
        Assert.Equal(new Point2D(2, 1), aResBSp.Pole(2));
        Assert.Equal((0.25, 0.5), (f, l));
        Assert.Equal(new Point2D(1, 1), aBSp.Pole(2));
    }

    [Fact]
    public void MakeEdge_from_curves_and_pcurves()
    {
        var sbe = new BuildEdgeBuilder();
        var aT = new Transform3D();
        aT = aT.WithTranslation(new Vector3D(0, 0, 5));
        var aLine = new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));

        Edge anE = new();
        sbe.MakeEdge(ref anE, aLine, new Placement(aT), 0, 3);
        Assert.True(anE.TShape()!.IsFrozen);
        Exploration.Vertices(anE, out Vertex aV1, out Vertex aV2);
        Assert.Equal(new Point3D(0, 0, 5), Tool.Pnt(aV1));
        Assert.Equal(new Point3D(3, 0, 5), Tool.Pnt(aV2));
        Curve3D aC = Tool.Curve(anE, out double f, out double l)!;
        Assert.Equal(5.0, aC.Value(l).Z);
        Assert.Equal((0.0, 3.0), (f, l));

        // an empty range fails: the edge is unchanged
        Edge aKept = anE;
        sbe.MakeEdge(ref aKept, aLine, new Placement(), 1, 1);
        Assert.True(aKept.IsEqual(anE));

        // on a face
        var aB = new BrepBuilder();
        Face aF = new();
        aB.MakeFace(ref aF, XOY(), Conf);
        Edge aPE = new();
        sbe.MakeEdge(ref aPE, new LineCurve2D(new Point2D(1, 1), new Direction2D(0, 1)), aF, 0, 2);
        Assert.True(aPE.TShape()!.IsFrozen);
        Exploration.Vertices(aPE, out aV1, out aV2);
        Assert.Equal(new Point3D(1, 3, 0), Tool.Pnt(aV2));

        // on a located surface
        Edge aLE = new();
        sbe.MakeEdge(ref aLE, new LineCurve2D(new Point2D(1, 1), new Direction2D(0, 1)), XOY(), new Placement(aT), 0, 2);
        Exploration.Vertices(aLE, out aV1, out aV2);
        Assert.Equal(new Point3D(1, 3, 5), Tool.Pnt(aV2));

        Edge anInternal = new();
        sbe.InternalMakeEdge(ref anInternal, aLine, new Placement(), 0, 1);
        Assert.False(anInternal.TShape()!.IsFrozen);
    }

    [Fact]
    public void TransferParameters_maps_ranges_linearly()
    {
        (Edge anE, Face aF) = ShiftedEdge();
        var aTP = new TransferParametersAnalysis(anE, aF);
        Assert.False(aTP.IsSameRange);
        Assert.Equal(15.0, aTP.Perform(5, true));
        Assert.Equal(5.0, aTP.Perform(15, false));
        var aSeq = new NCollection_HSequence<double>();
        aSeq.Append(0);
        aSeq.Append(10);
        NCollection_HSequence<double> aRes = aTP.Perform(aSeq, true);
        Assert.Equal(10.0, aRes.Value(1));
        Assert.Equal(20.0, aRes.Value(2));

        // a part of the edge: the ranges of [2, 4] in 3D
        var sbe = new BuildEdgeBuilder();
        Edge aPart = sbe.InternalCopy(anE);
        aTP.TransferRange(aPart, 2, 4, false);
        Tool.Range(aPart, out double f, out double l);
        Assert.Equal((2.0, 4.0), (f, l));
        Tool.Range(aPart, aF, out f, out l);
        Assert.Equal((12.0, 14.0), (f, l));
        // the same part given by its 2D parameters, backwards
        Edge aPart2 = sbe.InternalCopy(anE);
        aTP.TransferRange(aPart2, 14, 12, true);
        Tool.Range(aPart2, aF, out f, out l);
        Assert.Equal(12.0, f, 1e-14);
        Assert.Equal(14.0, l, 1e-14);

        // a box edge: same range; without a face, the identity
        (Edge aBoxE, Face aBoxF) = BoxEdge();
        Assert.True(new TransferParametersAnalysis(aBoxE, aBoxF).IsSameRange);
        var aFree = new TransferParametersAnalysis(aBoxE, new Face());
        Assert.True(aFree.IsSameRange);
        Assert.Equal(3.0, aFree.Perform(3, true));
        var anEmpty = new TransferParametersAnalysis();
        Assert.True(anEmpty.IsSameRange);
        anEmpty.SetMaxTolerance(1e-3);
    }
}
