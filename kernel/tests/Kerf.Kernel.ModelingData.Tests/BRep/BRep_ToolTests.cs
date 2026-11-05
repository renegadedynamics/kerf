// Port of: src/ModelingData/TKBRep/GTests/BRep_Tool_Test.cxx
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Geometry;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// OCCT's BRep_Tool GTests, with BRepBuilderAPI_MakeEdge (its internal accessor where the test goes on changing the
/// edge with BRep_Builder) and BRepPrimAPI_MakeBox (frozen), and further tests of BRep_Tool and BRep_Builder.
/// </summary>
public class BRep_ToolTests
{
    private static Shape MakeBox() => new MakeBox(10.0, 20.0, 30.0).Shape();

    private static Edge FullCircleEdge() =>
        FullCircleEdge(new CircleCurve3D(new Frame3D(new Point3D(), new Direction3D(0, 0, 1)), 1.0));

    private static Edge FullCircleEdge(CircleCurve3D aCircle) => new MakeEdge(aCircle).InternalEdge();

    [Fact]
    public void Pnt_FromVertex()
    {
        Shape aBox = MakeBox();
        var anExp = new Explorer(aBox, ShapeType.Vertex);
        Assert.True(anExp.More());
        Vertex aVertex = ShapeCasts.Vertex(anExp.Current());
        Point3D aPnt = Tool.Pnt(aVertex);

        // The point coordinates should be within the box bounds [0,10] x [0,20] x [0,30]
        Assert.True(aPnt.X >= -global::Kerf.Engine.Foundation.Precision.Confusion() && aPnt.X <= 10.0 + global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aPnt.Y >= -global::Kerf.Engine.Foundation.Precision.Confusion() && aPnt.Y <= 20.0 + global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aPnt.Z >= -global::Kerf.Engine.Foundation.Precision.Confusion() && aPnt.Z <= 30.0 + global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Tolerance_Vertex()
    {
        var anExp = new Explorer(MakeBox(), ShapeType.Vertex);
        Assert.True(anExp.More());
        Assert.True(Tool.Tolerance(ShapeCasts.Vertex(anExp.Current())) >= 0.0);
    }

    [Fact]
    public void Tolerance_Edge()
    {
        var anExp = new Explorer(MakeBox(), ShapeType.Edge);
        Assert.True(anExp.More());
        Assert.True(Tool.Tolerance(ShapeCasts.Edge(anExp.Current())) >= 0.0);
    }

    [Fact]
    public void Tolerance_Face()
    {
        var anExp = new Explorer(MakeBox(), ShapeType.Face);
        Assert.True(anExp.More());
        Assert.True(Tool.Tolerance(ShapeCasts.Face(anExp.Current())) >= 0.0);
    }

    [Fact]
    public void Curve_FromEdge()
    {
        var anExp = new Explorer(MakeBox(), ShapeType.Edge);
        Assert.True(anExp.More());
        Edge anEdge = ShapeCasts.Edge(anExp.Current());
        Curve3D? aCurve = Tool.Curve(anEdge, out double aFirst, out double aLast);
        Assert.NotNull(aCurve);
        Assert.True(aFirst < aLast);
    }

    [Fact]
    public void Surface_FromFace()
    {
        var anExp = new Explorer(MakeBox(), ShapeType.Face);
        Assert.True(anExp.More());
        Assert.NotNull(Tool.Surface(ShapeCasts.Face(anExp.Current())));
    }

    [Fact]
    public void IsClosed_CircleEdge()
    {
        Edge anEdge = TestShapes.Frozen(FullCircleEdge());
        Assert.True(Tool.IsClosed(anEdge));
    }

    [Fact]
    public void IsClosed_LineEdge()
    {
        Edge anEdge = new MakeEdge(new Point3D(0.0, 0.0, 0.0), new Point3D(10.0, 0.0, 0.0)).Edge();
        Assert.False(Tool.IsClosed(anEdge));
    }

    [Fact]
    public void Degenerated()
    {
        var anExp = new Explorer(MakeBox(), ShapeType.Edge);
        Assert.True(anExp.More());
        Assert.False(Tool.Degenerated(ShapeCasts.Edge(anExp.Current())));
    }

    [Fact]
    public void CurveOnPlane_RejectsEdgeRangeOutsideBoundedCurve()
    {
        var aPoles = new NCollection_Array1<Point3D>(1, 3);
        aPoles.SetValue(1, new Point3D(0.0, 0.0, 0.0));
        aPoles.SetValue(2, new Point3D(0.5, 0.25, 0.0));
        aPoles.SetValue(3, new Point3D(1.0, 0.0, 0.0));
        var aCurve = new BezierCurve3D(aPoles);
        Edge anEdge = new MakeEdge(aCurve).InternalEdge();

        const double THE_EDGE_FIRST = -0.000002673149646;
        const double THE_EDGE_LAST = 0.999997326850354;
        new BrepBuilder().Range(anEdge, THE_EDGE_FIRST, THE_EDGE_LAST, true);

        var aPlane = new PlaneSurface(new CoordinateSystem(Constants.XOY()));
        Curve2D? aPCurve =
            Tool.CurveOnSurface(anEdge, aPlane, new Placement(), out double aFirst, out double aLast, out bool isStored);
        Assert.Null(aPCurve);
        Assert.False(isStored);
        Assert.Equal(THE_EDGE_FIRST, aFirst);
        Assert.Equal(THE_EDGE_LAST, aLast);
    }

    [Fact]
    public void CurveOnPlane_ProjectsShiftedPeriodicRange()
    {
        var aCircle = new CircleCurve3D(new Frame3D(new Point3D(), new Direction3D(0, 0, 1)), 1.0);
        Edge anEdge = FullCircleEdge(aCircle);

        double aRangeFirst = aCircle.Period;
        double aRangeLast = 2.0 * aCircle.Period;
        new BrepBuilder().Range(anEdge, aRangeFirst, aRangeLast, true);

        var aPlane = new PlaneSurface(new CoordinateSystem(Constants.XOY()));
        Curve2D? aPCurve = Tool.CurveOnPlane(anEdge, aPlane, new Placement(), out double aFirst, out double aLast);
        Assert.NotNull(aPCurve);
        Assert.Equal(aRangeFirst, aFirst);
        Assert.Equal(aRangeLast, aLast);
    }

    // BRepBuilderAPI_MakeEdge(aCircle, 0, 0) widens the equal parameters to a whole period
    [Fact]
    public void CurveOnPlane_ProjectsPeriodicEdgeBuiltFromEqualParameters()
    {
        var aCircle = new CircleCurve3D(new Frame3D(new Point3D(), new Direction3D(0, 0, 1)), 1.0);
        var anEdgeMaker = new MakeEdge(aCircle, 0.0, 0.0);
        Assert.True(anEdgeMaker.IsDone);
        Edge anEdge = anEdgeMaker.Edge();

        Tool.Range(anEdge, out double aEdgeFirst, out double aEdgeLast);
        Assert.Equal(aCircle.Period, aEdgeLast - aEdgeFirst, global::Kerf.Engine.Foundation.Precision.PConfusion());

        var aPlane = new PlaneSurface(new CoordinateSystem(Constants.XOY()));
        Curve2D? aPCurve = Tool.CurveOnPlane(anEdge, aPlane, new Placement(), out double aFirst, out double aLast);
        Assert.NotNull(aPCurve);
        Assert.Equal(aEdgeFirst, aFirst);
        Assert.Equal(aEdgeLast, aLast);
    }

    [Fact]
    public void CurveOnPlane_RejectsEqualPeriodicRange()
    {
        Edge anEdge = FullCircleEdge();
        new BrepBuilder().Range(anEdge, 0.0, 0.0, true);

        var aPlane = new PlaneSurface(new CoordinateSystem(Constants.XOY()));
        Curve2D? aPCurve = Tool.CurveOnPlane(anEdge, aPlane, new Placement(), out double aFirst, out double aLast);
        Assert.Null(aPCurve);
        Assert.Equal(0.0, aFirst);
        Assert.Equal(0.0, aLast);
    }

    // Additions

    [Fact]
    public void CurveOnPlane_projects_an_edge_without_a_pcurve()
    {
        Edge anEdge = TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(1, 0, 0));
        var aPlane = new PlaneSurface(new CoordinateSystem(Constants.XOY()));
        Curve2D? aPCurve = Tool.CurveOnPlane(anEdge, aPlane, new Placement(), out double aFirst, out double aLast);
        Assert.NotNull(aPCurve);
        Assert.True(aPCurve.Value(aFirst).DistanceTo(new Point2D(0, 0)) < 1e-12);
        Assert.True(aPCurve.Value(aLast).DistanceTo(new Point2D(1, 0)) < 1e-12);
        // not a plane: null without projecting
        var aCyl = new CylindricalSurface(new CoordinateSystem(Constants.XOY()), 1.0);
        Assert.Null(Tool.CurveOnSurface(anEdge, aCyl, new Placement(), out _, out _));
    }

    [Fact]
    public void Box_faces_have_pcurves_matching_their_edges()
    {
        Shape aBox = MakeBox();
        for (var aFaceExp = new Explorer(aBox, ShapeType.Face); aFaceExp.More(); aFaceExp.Next())
        {
            Face aFace = ShapeCasts.Face(aFaceExp.Current());
            Surface aSurf = Tool.Surface(aFace)!;
            for (var anEdgeExp = new Explorer(aFace, ShapeType.Edge); anEdgeExp.More(); anEdgeExp.Next())
            {
                Edge anEdge = ShapeCasts.Edge(anEdgeExp.Current());
                Curve2D aPC = Tool.CurveOnSurface(anEdge, aFace, out double f, out double l, out bool isStored)!;
                Assert.True(isStored);
                Curve3D aC = Tool.Curve(anEdge, out double f3, out double l3)!;
                Assert.Equal(f3, f);
                Assert.Equal(l3, l);
                foreach (double t in new[] { f, (f + l) / 2, l })
                {
                    Point2D aUV = aPC.Value(t);
                    Assert.True(aSurf.Value(aUV.X, aUV.Y).DistanceTo(aC.Value(t)) < 1e-12);
                }

                // the UV points stored with the pcurve are its ends (computed by SetRange)
                Tool.UVPoints(anEdge, aFace, out Point2D aP1, out Point2D aP2);
                Assert.True(aP1.DistanceTo(aPC.Value(f)) < 1e-15 && aP2.DistanceTo(aPC.Value(l)) < 1e-15);
            }
        }
    }

    [Fact]
    public void Vertex_parameters_on_edges_and_faces()
    {
        Shape aBox = MakeBox();
        var anEdgeExp = new Explorer(aBox, ShapeType.Edge);
        Edge anEdge = ShapeCasts.Edge(anEdgeExp.Current());
        Exploration.Vertices(anEdge, out Vertex aV1, out Vertex aV2);
        Tool.Range(anEdge, out double f, out double l);
        Assert.Equal(f, Tool.Parameter(aV1, anEdge));
        Assert.Equal(l, Tool.Parameter(aV2, anEdge));
        Assert.True(Tool.Parameter(aV2, anEdge, out double aP) && aP == l);
        // on a reversed edge, the vertices keep their parameters
        Edge aRev = ShapeCasts.Edge(anEdge.Reversed());
        Assert.Equal(f, Tool.Parameter(aV1, aRev));

        // the parameters of a vertex on a face come from the UV points of its edges
        var aFaceExp = new Explorer(aBox, ShapeType.Face);
        Face aFace = ShapeCasts.Face(aFaceExp.Current());
        var aVExp = new Explorer(aFace, ShapeType.Vertex);
        Vertex aV = ShapeCasts.Vertex(aVExp.Current());
        Point2D aUV = Tool.Parameters(aV, aFace);
        Assert.True(Tool.Surface(aFace)!.Value(aUV.X, aUV.Y).DistanceTo(Tool.Pnt(aV)) < 1e-12);

        // a vertex not on the edge has no parameter
        Vertex aFar = TestShapes.Vertex(new Point3D(100, 100, 100));
        Assert.False(Tool.Parameter(aFar, anEdge, out _));
        EngineAssert.Throws<global::Kerf.Engine.Foundation.NoSuchObjectException>(() => Tool.Parameter(aFar, anEdge));
    }

    [Fact]
    public void Seam_edges_have_two_pcurves()
    {
        Face aFace = TestShapes.Frozen(TestShapes.CylinderFace(2.0, 3.0));
        int aSeams = 0;
        for (var anExp = new Explorer(aFace, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Edge anEdge = ShapeCasts.Edge(anExp.Current());
            if (!Tool.IsClosed(anEdge, aFace))
            {
                continue;
            }

            aSeams++;
            Assert.True(BrepTools.IsReallyClosed(anEdge, aFace));
            Curve2D aForward = Tool.CurveOnSurface(ShapeCasts.Edge(anEdge.Oriented(Forward)), aFace, out _, out _)!;
            Curve2D aReversed = Tool.CurveOnSurface(ShapeCasts.Edge(anEdge.Oriented(Reversed)), aFace, out _, out _)!;
            Assert.Equal(2.0 * Math.PI, aForward.Value(0).X, 15);
            Assert.Equal(0.0, aReversed.Value(0).X, 15);
            // on the reversed face the pcurves swap
            Curve2D aOnReversedFace = Tool.CurveOnSurface(ShapeCasts.Edge(anEdge.Oriented(Forward)),
                                                                     ShapeCasts.Face(aFace.Reversed()), out _, out _)!;
            Assert.Same(aReversed, aOnReversedFace);
        }

        Assert.Equal(2, aSeams); // the seam appears twice in the wire
        BrepTools.DetectClosedness(aFace, out bool isUClosed, out bool isVClosed);
        Assert.True(isUClosed);
        Assert.False(isVClosed);
    }

    [Fact]
    public void Locations_are_relative_to_the_edge_and_face()
    {
        // an edge built at a location keeps its curve in its own coordinates; the tool composes them
        var aBuilder = new BrepBuilder();
        var aLoc = new Placement(Transform3D.Translation(new Vector3D(5, 0, 0)));
        Edge anEdge = new();
        var aLine = new LineCurve3D(new Point3D(), new Direction3D(1, 0, 0));
        aBuilder.MakeEdge(ref anEdge, aLine, aLoc, 1e-7);
        Curve3D? aC = Tool.Curve(anEdge, out Placement aL, out _, out _);
        Assert.Same(aLine, aC);
        Assert.Equal(aLoc, aL);
        // the placed curve is a transformed copy
        Curve3D aPlaced = Tool.Curve(anEdge, out _, out _)!;
        Assert.NotSame(aLine, aPlaced);
        Assert.Equal(5.0, aPlaced.Value(0).X);

        // moving the edge composes its location with the representation's
        Edge aMoved = ShapeCasts.Edge(anEdge.Moved(aLoc));
        Tool.Curve(aMoved, out Placement aL2, out _, out _);
        Assert.Equal(aLoc * aLoc, aL2);

        // a pcurve stored for a face at a location is found from the moved edge on the moved face
        var aPlane = new PlaneSurface(new CoordinateSystem(Constants.XOY()));
        Face aFace = new();
        aBuilder.MakeFace(ref aFace, aPlane, 1e-7);
        var aPC = new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0));
        aBuilder.UpdateEdge(anEdge, aPC, aFace, 1e-7);
        Assert.Same(aPC, Tool.CurveOnSurface(anEdge, aFace, out _, out _));
        Face aMovedFace = ShapeCasts.Face(aFace.Moved(aLoc));
        Assert.Same(aPC, Tool.CurveOnSurface(aMoved, aMovedFace, out _, out _));
        // the unmoved edge has no pcurve on the moved plane, so BRep_Tool projects its 3D curve: the line from (5, 0, 0)
        // lies along the moved plane's u axis from its origin
        Curve2D? aProjected = Tool.CurveOnSurface(anEdge, aMovedFace, out double aFirst, out _, out bool isStored);
        Assert.NotNull(aProjected);
        Assert.False(isStored);
        Assert.True(aProjected.Value(aFirst).DistanceTo(new Point2D(aFirst, 0)) < 1e-12);
    }

    [Fact]
    public void Tolerances_flags_and_continuity()
    {
        Shape aBox = TestShapes.Box(1, 1, 1);
        var aBuilder = new BrepBuilder();
        var anExp = new Explorer(aBox, ShapeType.Edge);
        Edge anEdge = ShapeCasts.Edge(anExp.Current());

        Assert.True(Tool.SameParameter(anEdge) && Tool.SameRange(anEdge));
        aBuilder.SameParameter(anEdge, false);
        Assert.False(Tool.SameParameter(anEdge));
        aBuilder.UpdateEdge(anEdge, 0.25);
        Assert.Equal(0.25, Tool.Tolerance(anEdge));
        aBuilder.UpdateEdge(anEdge, 0.1); // tolerances only grow
        Assert.Equal(0.25, Tool.Tolerance(anEdge));
        Assert.Equal(0.25, Tool.MaxTolerance(aBox, ShapeType.Edge));
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), Tool.MaxTolerance(aBox, ShapeType.Vertex));
        Assert.Equal(0.0, Tool.MaxTolerance(aBox, ShapeType.Wire));

        // the continuity between the two faces of the edge
        var aFaces = new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(aBox, ShapeType.Edge, ShapeType.Face, aFaces);
        NCollection_List<Shape> aList = aFaces.GetValue(anEdge);
        Face aF1 = ShapeCasts.Face(aList.First()), aF2 = ShapeCasts.Face(aList.Last());
        Assert.False(Tool.HasContinuity(anEdge));
        Assert.Equal(C0, Tool.Continuity(anEdge, aF1, aF2));
        aBuilder.Continuity(anEdge, aF1, aF2, G1);
        Assert.True(Tool.HasContinuity(anEdge) && Tool.HasContinuity(anEdge, aF2, aF1));
        Assert.Equal(G1, Tool.Continuity(anEdge, aF2, aF1));
        Assert.Equal(G1, Tool.MaxContinuity(anEdge));

        // degenerated edges get a null 3D curve
        aBuilder.Degenerated(anEdge, true);
        Assert.True(Tool.Degenerated(anEdge));
        Assert.Null(Tool.Curve(anEdge, out _, out _));
        Assert.True(Tool.IsGeometric(anEdge)); // its pcurves remain
    }

    [Fact]
    public void IsClosed_shells_and_wires()
    {
        Shape aBox = MakeBox();
        var aShellExp = new Explorer(aBox, ShapeType.Shell);
        Assert.True(Tool.IsClosed(aShellExp.Current()));
        var aWireExp = new Explorer(aBox, ShapeType.Wire);
        Assert.True(Tool.IsClosed(aWireExp.Current()));
        var aFaceExp = new Explorer(aBox, ShapeType.Face);
        Assert.False(Tool.IsClosed(aFaceExp.Current())); // the Closed flag of a face
    }

    [Fact]
    public void Range_on_a_face_sets_Modified_unless_frozen()
    {
        Shape aBox = TestShapes.Box(1, 1, 1);
        var aFaceExp = new Explorer(aBox, ShapeType.Face);
        Face aFace = ShapeCasts.Face(aFaceExp.Current());
        Edge anEdge = ShapeCasts.Edge(new Explorer(aFace, ShapeType.Edge).Current());
        anEdge.TShape()!.Modified(false);
        Tool.Range(anEdge, aFace, out double f, out double l);
        Assert.Equal(0.0, f);
        Assert.Equal(1.0, l);
        Assert.True(anEdge.Modified()); // OCCT: a query that sets Modified

        TestShapes.Frozen(aBox);
        Tool.Range(anEdge, aFace, out f, out l); // no write on a frozen edge
        Assert.Equal(1.0, l);
    }

    [Fact]
    public void Polygons_and_triangulations()
    {
        var aBuilder = new BrepBuilder();
        Edge anEdge = TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(1, 0, 0));
        var aNodes = new NCollection_Array1<Point3D>(1, 2);
        aNodes[1] = new Point3D(0, 0, 0);
        aNodes[2] = new Point3D(1, 0, 0);
        var aPoly = new Polygon3D(aNodes);
        aBuilder.UpdateEdge(anEdge, aPoly);
        Assert.Same(aPoly, Tool.Polygon3D(anEdge, out Placement aL));
        Assert.True(aL.IsIdentity);
        Assert.True(aPoly.IsFrozen);
        aBuilder.UpdateEdge(anEdge, (Polygon3D?)null);
        Assert.Null(Tool.Polygon3D(anEdge, out _));

        // a face with only a triangulation, and an edge's polygon on it
        var aTri = new Triangulation(3, 1, false);
        aTri.SetNode(1, new Point3D(0, 0, 0));
        aTri.SetNode(2, new Point3D(1, 0, 0));
        aTri.SetNode(3, new Point3D(0, 1, 0));
        aTri.SetTriangle(1, new Triangle1(1, 2, 3));
        Face aFace = new();
        aBuilder.MakeFace(ref aFace, aTri);
        Assert.False(Tool.IsGeometric(aFace));
        Assert.Same(aTri, Tool.Triangulation(aFace, out _));
        Assert.True(aTri.IsFrozen);
        Assert.Equal(MeshPurpose.MeshPurpose_Active, aTri.MeshPurpose());
        var aPolyNodes = new NCollection_Array1<int>(1, 2);
        aPolyNodes[1] = 1;
        aPolyNodes[2] = 2;
        var aPolyOnTri = new PolygonOnTriangulation(aPolyNodes);
        aBuilder.UpdateEdge(anEdge, aPolyOnTri, aTri, new Placement());
        Assert.Same(aPolyOnTri, Tool.PolygonOnTriangulation(anEdge, aTri, new Placement()));
        Tool.PolygonOnTriangulation(anEdge, out PolygonOnTriangulation? aP, out Triangulation? aT, out _);
        Assert.Same(aPolyOnTri, aP);
        Assert.Same(aTri, aT);
        Assert.False(Tool.IsClosed(anEdge, aTri, new Placement()));
        Assert.Null(Tool.Triangulation(aFace, out _, MeshPurpose.MeshPurpose_Calculation));
        Assert.Same(aTri, Tool.Triangulation(aFace, out _, MeshPurpose.MeshPurpose_AnyFallback));
    }

    [Fact]
    public void PolygonOnClosedSurface_is_stored_without_the_location_as_in_OCCT()
    {
        // OCCT (occt-issues.md): BRep_Builder::UpdateEdge(E, P1, P2, S, L) stores the two polygons with the identity
        // location, where the one-polygon form stores L relative to the edge: on a face at a location, the pair isn't
        // found again
        var aBuilder = new BrepBuilder();
        var aLoc = new Placement(Transform3D.Translation(new Vector3D(0, 0, 1)));
        Face aFace = new();
        aBuilder.MakeFace(ref aFace, new CylindricalSurface(new CoordinateSystem(Constants.XOY()), 1.0), aLoc, 1e-7);
        Edge anEdge = TestShapes.Edge(new Point3D(1, 0, 0), new Point3D(1, 0, 1));
        var aP1 = new Polygon2D(2);
        var aP2 = new Polygon2D(2);
        aBuilder.UpdateEdge(anEdge, aP1, aP2, aFace);
        Assert.Null(Tool.PolygonOnSurface(anEdge, aFace));
        Tool.PolygonOnSurface(anEdge, out Polygon2D? aFound, out _, out Placement aL);
        Assert.Same(aP1, aFound);
        Assert.True(aL.IsIdentity);

        var aSingle = new Polygon2D(2);
        Edge anOther = TestShapes.Edge(new Point3D(1, 0, 0), new Point3D(1, 0, 1));
        aBuilder.UpdateEdge(anOther, aSingle, aFace);
        Assert.Same(aSingle, Tool.PolygonOnSurface(anOther, aFace));
    }

    [Fact]
    public void UpdateFaceUVPoints_stops_at_the_first_locked_edge_as_in_OCCT()
    {
        // OCCT (occt-issues.md): BRepTools::UpdateFaceUVPoints returns at the first locked edge instead of skipping it,
        // so the edges after it keep their old UV points
        var aBuilder = new BrepBuilder();
        Face aFace = new();
        aBuilder.MakeFace(ref aFace, new PlaneSurface(new CoordinateSystem(Constants.XOY())), 1e-7);
        Edge aLocked = TestShapes.Frozen(TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)));
        Edge aStale = TestShapes.Edge(new Point3D(1, 0, 0), new Point3D(1, 1, 0));
        aBuilder.UpdateEdge(aStale, new LineCurve2D(new Point2D(1, 0), new Direction2D(0, 1)), aFace, 1e-7);
        Tool.SetUVPoints(aStale, aFace, new Point2D(9, 9), new Point2D(9, 9));
        Wire aWire = new();
        aBuilder.MakeWire(ref aWire);
        aBuilder.Add(aWire, aLocked);
        aBuilder.Add(aWire, aStale);
        aBuilder.Add(aFace, aWire);

        BrepTools.UpdateFaceUVPoints(aFace);
        Tool.UVPoints(aStale, aFace, out Point2D aP1, out _);
        Assert.Equal(9.0, aP1.X);

        // with the locked edge after it, the stale edge is updated
        Face aFace2 = ShapeCasts.Face(aFace.EmptyCopied());
        Wire aWire2 = new();
        aBuilder.MakeWire(ref aWire2);
        aBuilder.Add(aWire2, aStale);
        aBuilder.Add(aWire2, aLocked);
        aBuilder.Add(aFace2, aWire2);
        BrepTools.UpdateFaceUVPoints(aFace2);
        Tool.UVPoints(aStale, aFace2, out aP1, out _);
        Assert.Equal(1.0, aP1.X);
    }
}
