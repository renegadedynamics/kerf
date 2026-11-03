// Port of: src/ModelingData/TKBRep/GTests/BRep_Tool_Test.cxx
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Meshing;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.Continuity;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// OCCT's BRep_Tool GTests, with BRepBuilderAPI_MakeEdge (its internal accessor where the test goes on changing the
/// edge with BRep_Builder) and BRepPrimAPI_MakeBox (frozen), and further tests of BRep_Tool and BRep_Builder.
/// </summary>
public class BRep_ToolTests
{
    private static TopoDS_Shape MakeBox() => new BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape();

    private static TopoDS_Edge FullCircleEdge() =>
        FullCircleEdge(new Geom_Circle(new Frame3D(new Point3D(), new Direction3D(0, 0, 1)), 1.0));

    private static TopoDS_Edge FullCircleEdge(Geom_Circle aCircle) => new BRepBuilderAPI_MakeEdge(aCircle).InternalEdge();

    [Fact]
    public void Pnt_FromVertex()
    {
        TopoDS_Shape aBox = MakeBox();
        var anExp = new TopExp_Explorer(aBox, TopAbs_VERTEX);
        Assert.True(anExp.More());
        TopoDS_Vertex aVertex = TopoDS.Vertex(anExp.Current());
        Point3D aPnt = BRep_Tool.Pnt(aVertex);

        // The point coordinates should be within the box bounds [0,10] x [0,20] x [0,30]
        Assert.True(aPnt.X() >= -global::Kerf.Engine.Foundation.Precision.Confusion() && aPnt.X() <= 10.0 + global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aPnt.Y() >= -global::Kerf.Engine.Foundation.Precision.Confusion() && aPnt.Y() <= 20.0 + global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aPnt.Z() >= -global::Kerf.Engine.Foundation.Precision.Confusion() && aPnt.Z() <= 30.0 + global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Tolerance_Vertex()
    {
        var anExp = new TopExp_Explorer(MakeBox(), TopAbs_VERTEX);
        Assert.True(anExp.More());
        Assert.True(BRep_Tool.Tolerance(TopoDS.Vertex(anExp.Current())) >= 0.0);
    }

    [Fact]
    public void Tolerance_Edge()
    {
        var anExp = new TopExp_Explorer(MakeBox(), TopAbs_EDGE);
        Assert.True(anExp.More());
        Assert.True(BRep_Tool.Tolerance(TopoDS.Edge(anExp.Current())) >= 0.0);
    }

    [Fact]
    public void Tolerance_Face()
    {
        var anExp = new TopExp_Explorer(MakeBox(), TopAbs_FACE);
        Assert.True(anExp.More());
        Assert.True(BRep_Tool.Tolerance(TopoDS.Face(anExp.Current())) >= 0.0);
    }

    [Fact]
    public void Curve_FromEdge()
    {
        var anExp = new TopExp_Explorer(MakeBox(), TopAbs_EDGE);
        Assert.True(anExp.More());
        TopoDS_Edge anEdge = TopoDS.Edge(anExp.Current());
        Geom_Curve? aCurve = BRep_Tool.Curve(anEdge, out double aFirst, out double aLast);
        Assert.NotNull(aCurve);
        Assert.True(aFirst < aLast);
    }

    [Fact]
    public void Surface_FromFace()
    {
        var anExp = new TopExp_Explorer(MakeBox(), TopAbs_FACE);
        Assert.True(anExp.More());
        Assert.NotNull(BRep_Tool.Surface(TopoDS.Face(anExp.Current())));
    }

    [Fact]
    public void IsClosed_CircleEdge()
    {
        TopoDS_Edge anEdge = TestShapes.Frozen(FullCircleEdge());
        Assert.True(BRep_Tool.IsClosed(anEdge));
    }

    [Fact]
    public void IsClosed_LineEdge()
    {
        TopoDS_Edge anEdge = new BRepBuilderAPI_MakeEdge(new Point3D(0.0, 0.0, 0.0), new Point3D(10.0, 0.0, 0.0)).Edge();
        Assert.False(BRep_Tool.IsClosed(anEdge));
    }

    [Fact]
    public void Degenerated()
    {
        var anExp = new TopExp_Explorer(MakeBox(), TopAbs_EDGE);
        Assert.True(anExp.More());
        Assert.False(BRep_Tool.Degenerated(TopoDS.Edge(anExp.Current())));
    }

    [Fact]
    public void CurveOnPlane_RejectsEdgeRangeOutsideBoundedCurve()
    {
        var aPoles = new NCollection_Array1<Point3D>(1, 3);
        aPoles.SetValue(1, new Point3D(0.0, 0.0, 0.0));
        aPoles.SetValue(2, new Point3D(0.5, 0.25, 0.0));
        aPoles.SetValue(3, new Point3D(1.0, 0.0, 0.0));
        var aCurve = new Geom_BezierCurve(aPoles);
        TopoDS_Edge anEdge = new BRepBuilderAPI_MakeEdge(aCurve).InternalEdge();

        const double THE_EDGE_FIRST = -0.000002673149646;
        const double THE_EDGE_LAST = 0.999997326850354;
        new BRep_Builder().Range(anEdge, THE_EDGE_FIRST, THE_EDGE_LAST, true);

        var aPlane = new Geom_Plane(new CoordinateSystem(Constants.XOY()));
        Geom2d_Curve? aPCurve =
            BRep_Tool.CurveOnSurface(anEdge, aPlane, new Placement(), out double aFirst, out double aLast, out bool isStored);
        Assert.Null(aPCurve);
        Assert.False(isStored);
        Assert.Equal(THE_EDGE_FIRST, aFirst);
        Assert.Equal(THE_EDGE_LAST, aLast);
    }

    [Fact]
    public void CurveOnPlane_ProjectsShiftedPeriodicRange()
    {
        var aCircle = new Geom_Circle(new Frame3D(new Point3D(), new Direction3D(0, 0, 1)), 1.0);
        TopoDS_Edge anEdge = FullCircleEdge(aCircle);

        double aRangeFirst = aCircle.Period();
        double aRangeLast = 2.0 * aCircle.Period();
        new BRep_Builder().Range(anEdge, aRangeFirst, aRangeLast, true);

        var aPlane = new Geom_Plane(new CoordinateSystem(Constants.XOY()));
        Geom2d_Curve? aPCurve = BRep_Tool.CurveOnPlane(anEdge, aPlane, new Placement(), out double aFirst, out double aLast);
        Assert.NotNull(aPCurve);
        Assert.Equal(aRangeFirst, aFirst);
        Assert.Equal(aRangeLast, aLast);
    }

    // BRepBuilderAPI_MakeEdge(aCircle, 0, 0) widens the equal parameters to a whole period
    [Fact]
    public void CurveOnPlane_ProjectsPeriodicEdgeBuiltFromEqualParameters()
    {
        var aCircle = new Geom_Circle(new Frame3D(new Point3D(), new Direction3D(0, 0, 1)), 1.0);
        var anEdgeMaker = new BRepBuilderAPI_MakeEdge(aCircle, 0.0, 0.0);
        Assert.True(anEdgeMaker.IsDone());
        TopoDS_Edge anEdge = anEdgeMaker.Edge();

        BRep_Tool.Range(anEdge, out double aEdgeFirst, out double aEdgeLast);
        Assert.Equal(aCircle.Period(), aEdgeLast - aEdgeFirst, global::Kerf.Engine.Foundation.Precision.PConfusion());

        var aPlane = new Geom_Plane(new CoordinateSystem(Constants.XOY()));
        Geom2d_Curve? aPCurve = BRep_Tool.CurveOnPlane(anEdge, aPlane, new Placement(), out double aFirst, out double aLast);
        Assert.NotNull(aPCurve);
        Assert.Equal(aEdgeFirst, aFirst);
        Assert.Equal(aEdgeLast, aLast);
    }

    [Fact]
    public void CurveOnPlane_RejectsEqualPeriodicRange()
    {
        TopoDS_Edge anEdge = FullCircleEdge();
        new BRep_Builder().Range(anEdge, 0.0, 0.0, true);

        var aPlane = new Geom_Plane(new CoordinateSystem(Constants.XOY()));
        Geom2d_Curve? aPCurve = BRep_Tool.CurveOnPlane(anEdge, aPlane, new Placement(), out double aFirst, out double aLast);
        Assert.Null(aPCurve);
        Assert.Equal(0.0, aFirst);
        Assert.Equal(0.0, aLast);
    }

    // Additions

    [Fact]
    public void CurveOnPlane_projects_an_edge_without_a_pcurve()
    {
        TopoDS_Edge anEdge = TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(1, 0, 0));
        var aPlane = new Geom_Plane(new CoordinateSystem(Constants.XOY()));
        Geom2d_Curve? aPCurve = BRep_Tool.CurveOnPlane(anEdge, aPlane, new Placement(), out double aFirst, out double aLast);
        Assert.NotNull(aPCurve);
        Assert.True(aPCurve.Value(aFirst).Distance(new Point2D(0, 0)) < 1e-12);
        Assert.True(aPCurve.Value(aLast).Distance(new Point2D(1, 0)) < 1e-12);
        // not a plane: null without projecting
        var aCyl = new Geom_CylindricalSurface(new CoordinateSystem(Constants.XOY()), 1.0);
        Assert.Null(BRep_Tool.CurveOnSurface(anEdge, aCyl, new Placement(), out _, out _));
    }

    [Fact]
    public void Box_faces_have_pcurves_matching_their_edges()
    {
        TopoDS_Shape aBox = MakeBox();
        for (var aFaceExp = new TopExp_Explorer(aBox, TopAbs_FACE); aFaceExp.More(); aFaceExp.Next())
        {
            TopoDS_Face aFace = TopoDS.Face(aFaceExp.Current());
            Geom_Surface aSurf = BRep_Tool.Surface(aFace)!;
            for (var anEdgeExp = new TopExp_Explorer(aFace, TopAbs_EDGE); anEdgeExp.More(); anEdgeExp.Next())
            {
                TopoDS_Edge anEdge = TopoDS.Edge(anEdgeExp.Current());
                Geom2d_Curve aPC = BRep_Tool.CurveOnSurface(anEdge, aFace, out double f, out double l, out bool isStored)!;
                Assert.True(isStored);
                Geom_Curve aC = BRep_Tool.Curve(anEdge, out double f3, out double l3)!;
                Assert.Equal(f3, f);
                Assert.Equal(l3, l);
                foreach (double t in new[] { f, (f + l) / 2, l })
                {
                    Point2D aUV = aPC.Value(t);
                    Assert.True(aSurf.Value(aUV.X(), aUV.Y()).Distance(aC.Value(t)) < 1e-12);
                }

                // the UV points stored with the pcurve are its ends (computed by SetRange)
                BRep_Tool.UVPoints(anEdge, aFace, out Point2D aP1, out Point2D aP2);
                Assert.True(aP1.Distance(aPC.Value(f)) < 1e-15 && aP2.Distance(aPC.Value(l)) < 1e-15);
            }
        }
    }

    [Fact]
    public void Vertex_parameters_on_edges_and_faces()
    {
        TopoDS_Shape aBox = MakeBox();
        var anEdgeExp = new TopExp_Explorer(aBox, TopAbs_EDGE);
        TopoDS_Edge anEdge = TopoDS.Edge(anEdgeExp.Current());
        TopExp.Vertices(anEdge, out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
        BRep_Tool.Range(anEdge, out double f, out double l);
        Assert.Equal(f, BRep_Tool.Parameter(aV1, anEdge));
        Assert.Equal(l, BRep_Tool.Parameter(aV2, anEdge));
        Assert.True(BRep_Tool.Parameter(aV2, anEdge, out double aP) && aP == l);
        // on a reversed edge, the vertices keep their parameters
        TopoDS_Edge aRev = TopoDS.Edge(anEdge.Reversed());
        Assert.Equal(f, BRep_Tool.Parameter(aV1, aRev));

        // the parameters of a vertex on a face come from the UV points of its edges
        var aFaceExp = new TopExp_Explorer(aBox, TopAbs_FACE);
        TopoDS_Face aFace = TopoDS.Face(aFaceExp.Current());
        var aVExp = new TopExp_Explorer(aFace, TopAbs_VERTEX);
        TopoDS_Vertex aV = TopoDS.Vertex(aVExp.Current());
        Point2D aUV = BRep_Tool.Parameters(aV, aFace);
        Assert.True(BRep_Tool.Surface(aFace)!.Value(aUV.X(), aUV.Y()).Distance(BRep_Tool.Pnt(aV)) < 1e-12);

        // a vertex not on the edge has no parameter
        TopoDS_Vertex aFar = TestShapes.Vertex(new Point3D(100, 100, 100));
        Assert.False(BRep_Tool.Parameter(aFar, anEdge, out _));
        Assert.Throws<global::Kerf.Engine.Foundation.NoSuchObjectException>(() => BRep_Tool.Parameter(aFar, anEdge));
    }

    [Fact]
    public void Seam_edges_have_two_pcurves()
    {
        TopoDS_Face aFace = TestShapes.Frozen(TestShapes.CylinderFace(2.0, 3.0));
        int aSeams = 0;
        for (var anExp = new TopExp_Explorer(aFace, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            TopoDS_Edge anEdge = TopoDS.Edge(anExp.Current());
            if (!BRep_Tool.IsClosed(anEdge, aFace))
            {
                continue;
            }

            aSeams++;
            Assert.True(BRepTools.IsReallyClosed(anEdge, aFace));
            Geom2d_Curve aForward = BRep_Tool.CurveOnSurface(TopoDS.Edge(anEdge.Oriented(TopAbs_FORWARD)), aFace, out _, out _)!;
            Geom2d_Curve aReversed = BRep_Tool.CurveOnSurface(TopoDS.Edge(anEdge.Oriented(TopAbs_REVERSED)), aFace, out _, out _)!;
            Assert.Equal(2.0 * Math.PI, aForward.Value(0).X(), 15);
            Assert.Equal(0.0, aReversed.Value(0).X(), 15);
            // on the reversed face the pcurves swap
            Geom2d_Curve aOnReversedFace = BRep_Tool.CurveOnSurface(TopoDS.Edge(anEdge.Oriented(TopAbs_FORWARD)),
                                                                     TopoDS.Face(aFace.Reversed()), out _, out _)!;
            Assert.Same(aReversed, aOnReversedFace);
        }

        Assert.Equal(2, aSeams); // the seam appears twice in the wire
        BRepTools.DetectClosedness(aFace, out bool isUClosed, out bool isVClosed);
        Assert.True(isUClosed);
        Assert.False(isVClosed);
    }

    [Fact]
    public void Locations_are_relative_to_the_edge_and_face()
    {
        // an edge built at a location keeps its curve in its own coordinates; the tool composes them
        var aBuilder = new BRep_Builder();
        var aLoc = new Placement(Transform3D.Translation(new Vector3D(5, 0, 0)));
        TopoDS_Edge anEdge = new();
        var aLine = new Geom_Line(new Point3D(), new Direction3D(1, 0, 0));
        aBuilder.MakeEdge(ref anEdge, aLine, aLoc, 1e-7);
        Geom_Curve? aC = BRep_Tool.Curve(anEdge, out Placement aL, out _, out _);
        Assert.Same(aLine, aC);
        Assert.Equal(aLoc, aL);
        // the placed curve is a transformed copy
        Geom_Curve aPlaced = BRep_Tool.Curve(anEdge, out _, out _)!;
        Assert.NotSame(aLine, aPlaced);
        Assert.Equal(5.0, aPlaced.Value(0).X());

        // moving the edge composes its location with the representation's
        TopoDS_Edge aMoved = TopoDS.Edge(anEdge.Moved(aLoc));
        BRep_Tool.Curve(aMoved, out Placement aL2, out _, out _);
        Assert.Equal(aLoc * aLoc, aL2);

        // a pcurve stored for a face at a location is found from the moved edge on the moved face
        var aPlane = new Geom_Plane(new CoordinateSystem(Constants.XOY()));
        TopoDS_Face aFace = new();
        aBuilder.MakeFace(ref aFace, aPlane, 1e-7);
        var aPC = new Geom2d_Line(new Point2D(0, 0), new Direction2D(1, 0));
        aBuilder.UpdateEdge(anEdge, aPC, aFace, 1e-7);
        Assert.Same(aPC, BRep_Tool.CurveOnSurface(anEdge, aFace, out _, out _));
        TopoDS_Face aMovedFace = TopoDS.Face(aFace.Moved(aLoc));
        Assert.Same(aPC, BRep_Tool.CurveOnSurface(aMoved, aMovedFace, out _, out _));
        // the unmoved edge has no pcurve on the moved plane, so BRep_Tool projects its 3D curve: the line from (5, 0, 0)
        // lies along the moved plane's u axis from its origin
        Geom2d_Curve? aProjected = BRep_Tool.CurveOnSurface(anEdge, aMovedFace, out double aFirst, out _, out bool isStored);
        Assert.NotNull(aProjected);
        Assert.False(isStored);
        Assert.True(aProjected.Value(aFirst).Distance(new Point2D(aFirst, 0)) < 1e-12);
    }

    [Fact]
    public void Tolerances_flags_and_continuity()
    {
        TopoDS_Shape aBox = TestShapes.Box(1, 1, 1);
        var aBuilder = new BRep_Builder();
        var anExp = new TopExp_Explorer(aBox, TopAbs_EDGE);
        TopoDS_Edge anEdge = TopoDS.Edge(anExp.Current());

        Assert.True(BRep_Tool.SameParameter(anEdge) && BRep_Tool.SameRange(anEdge));
        aBuilder.SameParameter(anEdge, false);
        Assert.False(BRep_Tool.SameParameter(anEdge));
        aBuilder.UpdateEdge(anEdge, 0.25);
        Assert.Equal(0.25, BRep_Tool.Tolerance(anEdge));
        aBuilder.UpdateEdge(anEdge, 0.1); // tolerances only grow
        Assert.Equal(0.25, BRep_Tool.Tolerance(anEdge));
        Assert.Equal(0.25, BRep_Tool.MaxTolerance(aBox, TopAbs_EDGE));
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), BRep_Tool.MaxTolerance(aBox, TopAbs_VERTEX));
        Assert.Equal(0.0, BRep_Tool.MaxTolerance(aBox, TopAbs_WIRE));

        // the continuity between the two faces of the edge
        var aFaces = new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(aBox, TopAbs_EDGE, TopAbs_FACE, aFaces);
        NCollection_List<TopoDS_Shape> aList = aFaces.FindFromKey(anEdge);
        TopoDS_Face aF1 = TopoDS.Face(aList.First()), aF2 = TopoDS.Face(aList.Last());
        Assert.False(BRep_Tool.HasContinuity(anEdge));
        Assert.Equal(C0, BRep_Tool.Continuity(anEdge, aF1, aF2));
        aBuilder.Continuity(anEdge, aF1, aF2, G1);
        Assert.True(BRep_Tool.HasContinuity(anEdge) && BRep_Tool.HasContinuity(anEdge, aF2, aF1));
        Assert.Equal(G1, BRep_Tool.Continuity(anEdge, aF2, aF1));
        Assert.Equal(G1, BRep_Tool.MaxContinuity(anEdge));

        // degenerated edges get a null 3D curve
        aBuilder.Degenerated(anEdge, true);
        Assert.True(BRep_Tool.Degenerated(anEdge));
        Assert.Null(BRep_Tool.Curve(anEdge, out _, out _));
        Assert.True(BRep_Tool.IsGeometric(anEdge)); // its pcurves remain
    }

    [Fact]
    public void IsClosed_shells_and_wires()
    {
        TopoDS_Shape aBox = MakeBox();
        var aShellExp = new TopExp_Explorer(aBox, TopAbs_SHELL);
        Assert.True(BRep_Tool.IsClosed(aShellExp.Current()));
        var aWireExp = new TopExp_Explorer(aBox, TopAbs_WIRE);
        Assert.True(BRep_Tool.IsClosed(aWireExp.Current()));
        var aFaceExp = new TopExp_Explorer(aBox, TopAbs_FACE);
        Assert.False(BRep_Tool.IsClosed(aFaceExp.Current())); // the Closed flag of a face
    }

    [Fact]
    public void Range_on_a_face_sets_Modified_unless_frozen()
    {
        TopoDS_Shape aBox = TestShapes.Box(1, 1, 1);
        var aFaceExp = new TopExp_Explorer(aBox, TopAbs_FACE);
        TopoDS_Face aFace = TopoDS.Face(aFaceExp.Current());
        TopoDS_Edge anEdge = TopoDS.Edge(new TopExp_Explorer(aFace, TopAbs_EDGE).Current());
        anEdge.TShape()!.Modified(false);
        BRep_Tool.Range(anEdge, aFace, out double f, out double l);
        Assert.Equal(0.0, f);
        Assert.Equal(1.0, l);
        Assert.True(anEdge.Modified()); // OCCT: a query that sets Modified

        TestShapes.Frozen(aBox);
        BRep_Tool.Range(anEdge, aFace, out f, out l); // no write on a frozen edge
        Assert.Equal(1.0, l);
    }

    [Fact]
    public void Polygons_and_triangulations()
    {
        var aBuilder = new BRep_Builder();
        TopoDS_Edge anEdge = TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(1, 0, 0));
        var aNodes = new NCollection_Array1<Point3D>(1, 2);
        aNodes[1] = new Point3D(0, 0, 0);
        aNodes[2] = new Point3D(1, 0, 0);
        var aPoly = new Polygon3D(aNodes);
        aBuilder.UpdateEdge(anEdge, aPoly);
        Assert.Same(aPoly, BRep_Tool.Polygon3D(anEdge, out Placement aL));
        Assert.True(aL.IsIdentity);
        Assert.True(aPoly.IsFrozen);
        aBuilder.UpdateEdge(anEdge, (Polygon3D?)null);
        Assert.Null(BRep_Tool.Polygon3D(anEdge, out _));

        // a face with only a triangulation, and an edge's polygon on it
        var aTri = new Triangulation(3, 1, false);
        aTri.SetNode(1, new Point3D(0, 0, 0));
        aTri.SetNode(2, new Point3D(1, 0, 0));
        aTri.SetNode(3, new Point3D(0, 1, 0));
        aTri.SetTriangle(1, new Triangle1(1, 2, 3));
        TopoDS_Face aFace = new();
        aBuilder.MakeFace(ref aFace, aTri);
        Assert.False(BRep_Tool.IsGeometric(aFace));
        Assert.Same(aTri, BRep_Tool.Triangulation(aFace, out _));
        Assert.True(aTri.IsFrozen);
        Assert.Equal(MeshPurpose.MeshPurpose_Active, aTri.MeshPurpose());
        var aPolyNodes = new NCollection_Array1<int>(1, 2);
        aPolyNodes[1] = 1;
        aPolyNodes[2] = 2;
        var aPolyOnTri = new PolygonOnTriangulation(aPolyNodes);
        aBuilder.UpdateEdge(anEdge, aPolyOnTri, aTri, new Placement());
        Assert.Same(aPolyOnTri, BRep_Tool.PolygonOnTriangulation(anEdge, aTri, new Placement()));
        BRep_Tool.PolygonOnTriangulation(anEdge, out PolygonOnTriangulation? aP, out Triangulation? aT, out _);
        Assert.Same(aPolyOnTri, aP);
        Assert.Same(aTri, aT);
        Assert.False(BRep_Tool.IsClosed(anEdge, aTri, new Placement()));
        Assert.Null(BRep_Tool.Triangulation(aFace, out _, MeshPurpose.MeshPurpose_Calculation));
        Assert.Same(aTri, BRep_Tool.Triangulation(aFace, out _, MeshPurpose.MeshPurpose_AnyFallback));
    }

    [Fact]
    public void PolygonOnClosedSurface_is_stored_without_the_location_as_in_OCCT()
    {
        // OCCT (occt-issues.md): BRep_Builder::UpdateEdge(E, P1, P2, S, L) stores the two polygons with the identity
        // location, where the one-polygon form stores L relative to the edge: on a face at a location, the pair isn't
        // found again
        var aBuilder = new BRep_Builder();
        var aLoc = new Placement(Transform3D.Translation(new Vector3D(0, 0, 1)));
        TopoDS_Face aFace = new();
        aBuilder.MakeFace(ref aFace, new Geom_CylindricalSurface(new CoordinateSystem(Constants.XOY()), 1.0), aLoc, 1e-7);
        TopoDS_Edge anEdge = TestShapes.Edge(new Point3D(1, 0, 0), new Point3D(1, 0, 1));
        var aP1 = new Polygon2D(2);
        var aP2 = new Polygon2D(2);
        aBuilder.UpdateEdge(anEdge, aP1, aP2, aFace);
        Assert.Null(BRep_Tool.PolygonOnSurface(anEdge, aFace));
        BRep_Tool.PolygonOnSurface(anEdge, out Polygon2D? aFound, out _, out Placement aL);
        Assert.Same(aP1, aFound);
        Assert.True(aL.IsIdentity);

        var aSingle = new Polygon2D(2);
        TopoDS_Edge anOther = TestShapes.Edge(new Point3D(1, 0, 0), new Point3D(1, 0, 1));
        aBuilder.UpdateEdge(anOther, aSingle, aFace);
        Assert.Same(aSingle, BRep_Tool.PolygonOnSurface(anOther, aFace));
    }

    [Fact]
    public void UpdateFaceUVPoints_stops_at_the_first_locked_edge_as_in_OCCT()
    {
        // OCCT (occt-issues.md): BRepTools::UpdateFaceUVPoints returns at the first locked edge instead of skipping it,
        // so the edges after it keep their old UV points
        var aBuilder = new BRep_Builder();
        TopoDS_Face aFace = new();
        aBuilder.MakeFace(ref aFace, new Geom_Plane(new CoordinateSystem(Constants.XOY())), 1e-7);
        TopoDS_Edge aLocked = TestShapes.Frozen(TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)));
        TopoDS_Edge aStale = TestShapes.Edge(new Point3D(1, 0, 0), new Point3D(1, 1, 0));
        aBuilder.UpdateEdge(aStale, new Geom2d_Line(new Point2D(1, 0), new Direction2D(0, 1)), aFace, 1e-7);
        BRep_Tool.SetUVPoints(aStale, aFace, new Point2D(9, 9), new Point2D(9, 9));
        TopoDS_Wire aWire = new();
        aBuilder.MakeWire(ref aWire);
        aBuilder.Add(aWire, aLocked);
        aBuilder.Add(aWire, aStale);
        aBuilder.Add(aFace, aWire);

        BRepTools.UpdateFaceUVPoints(aFace);
        BRep_Tool.UVPoints(aStale, aFace, out Point2D aP1, out _);
        Assert.Equal(9.0, aP1.X());

        // with the locked edge after it, the stale edge is updated
        TopoDS_Face aFace2 = TopoDS.Face(aFace.EmptyCopied());
        TopoDS_Wire aWire2 = new();
        aBuilder.MakeWire(ref aWire2);
        aBuilder.Add(aWire2, aStale);
        aBuilder.Add(aWire2, aLocked);
        aBuilder.Add(aFace2, aWire2);
        BRepTools.UpdateFaceUVPoints(aFace2);
        BRep_Tool.UVPoints(aStale, aFace2, out aP1, out _);
        Assert.Equal(1.0, aP1.X());
    }
}
