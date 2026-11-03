// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Measurement;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Schematiq.Collections;
using OcctNet.ModelingData.Tests;
using static Kerf.Engine.Construction.Api.EdgeError;
using static Kerf.Engine.Construction.Api.FaceError;
using static Kerf.Engine.Construction.Api.ShellError;
using static Kerf.Engine.Construction.Api.WireError;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.ShapeType;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Known answers of BRepBuilderAPI: the makers over BRepLib's (results, errors, freezing at publication, the internal
/// accessors, Add after a public read, Modified answering the copies of frozen inputs), the modifications (Transform
/// as a location or a modifier, Copy, GTransform, NurbsConvert, their ModifiedShape and Modified, histories through
/// <see cref="History.IAlgorithm"/>), Collect, FindPlane and the package statics. Inputs are frozen, as
/// callers hold them, and checked unchanged with <see cref="ShapeSnapshot"/>.
/// </summary>
public class BRepBuilderAPITests
{
    private static int Count(in Shape theS, ShapeType theType)
    {
        var aMap = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theS, theType, aMap);
        return aMap.Extent();
    }

    private static Shape Box10() => TestShapes.Frozen(TestShapes.Box(10.0, 10.0, 10.0));

    private static bool AllFrozen(in Shape theS)
    {
        for (var anExp = new Explorer(theS, ShapeType.Vertex); anExp.More(); anExp.Next())
        {
            if (!anExp.Current().TShape()!.IsFrozen)
            {
                return false;
            }
        }
        return theS.TShape()!.IsFrozen;
    }

    // ------------------------------------------------------------ makers

    [Fact]
    public void Vertex_is_frozen_when_read_and_not_through_the_internal_accessor()
    {
        var aMV = new MakeVertex(new Point3D(1, 2, 3));
        Assert.True(aMV.IsDone);
        Vertex anInternal = aMV.InternalVertex();
        Assert.False(anInternal.TShape()!.IsFrozen);
        Vertex aV = aMV.Vertex();
        Assert.True(aV.TShape()!.IsFrozen);
        Assert.True(aV.IsSame(anInternal));
        Assert.True(aV.IsSame((Vertex)aMV));
        Assert.Equal(new Point3D(1, 2, 3), Tool.Pnt(aV));
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), Tool.Tolerance(aV));
    }

    [Fact]
    public void Edge_maker_maps_BRepLib_errors()
    {
        var aSame = new MakeEdge(new Point3D(1, 1, 1), new Point3D(1, 1, 1));
        Assert.False(aSame.IsDone);
        Assert.Equal(EdgeError.LineThroughIdenticPoints, aSame.Error());
        Assert.Throws<global::Kerf.Engine.Foundation.NotDoneException>(() => aSame.Shape());

        var aCirc = new Circle3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 1);
        var aOff = new MakeEdge(aCirc, new Point3D(0, 0, 0), new Point3D(1, 0, 0));
        Assert.Equal(EdgeError.PointProjectionFailed, aOff.Error());

        var aLine = new MakeEdge(new Line3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), 0, 2);
        Assert.Equal(EdgeError.EdgeDone, aLine.Error());
        Assert.True(aLine.Vertex1().TShape()!.IsFrozen);
        Assert.True(aLine.Edge().TShape()!.IsFrozen);
        Assert.Equal(new Point3D(2, 0, 0), Tool.Pnt(aLine.Vertex2()));
    }

    [Fact]
    public void Edge_maker_Init_after_a_public_read_makes_a_new_edge()
    {
        var aME = new MakeEdge(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), 0, 1);
        Edge aFirst = aME.Edge();
        aME.Init(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(0, 1, 0)), 0, 3);
        Assert.True(aME.IsDone);
        Edge aSecond = aME.Edge();
        Assert.False(aSecond.IsSame(aFirst));
        Tool.Range(aFirst, out double f, out double l);
        Assert.Equal(1.0, l - f);
        Tool.Range(aSecond, out f, out l);
        Assert.Equal(3.0, l - f);
    }

    [Fact]
    public void Edge2d_lies_in_the_default_plane()
    {
        var aME = new MakeEdge2d(new Point2D(0, 0), new Point2D(3, 4));
        Assert.True(aME.IsDone);
        Assert.Equal(EdgeError.EdgeDone, aME.Error());
        Edge anE = aME.Edge();
        Curve2D? aC = Tool.CurveOnSurface(anE, BRepBuilderApi.Plane(), new Placement(), out double f, out double l);
        Assert.NotNull(aC);
        Assert.Equal(5.0, l - f, 1e-12);
        Assert.True(BRepBuilderApi.Plane().IsFrozen);
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), BRepBuilderApi.Precision());
        Assert.Equal(EdgeError.LineThroughIdenticPoints,
                     new MakeEdge2d(new Point2D(1, 1), new Point2D(1, 1)).Error());
    }

    [Fact]
    public void Polygon_adds_points_closes_and_reports_coincident_points()
    {
        var aMP = new MakePolygon();
        aMP.Add(new Point3D(0, 0, 0));
        Assert.False(aMP.IsDone);
        aMP.Add(new Point3D(1, 0, 0));
        Assert.True(aMP.Added());
        Assert.True(aMP.IsDone);
        aMP.Add(new Point3D(1, 0, 0));
        Assert.False(aMP.Added());
        aMP.Add(new Point3D(1, 1, 0));
        // the internal accessors don't freeze: the maker goes on in place
        Assert.False(aMP.InternalLastVertex().TShape()!.IsFrozen);
        aMP.Close();
        Wire aW = aMP.Wire();
        Assert.True(aW.Closed());
        Assert.Equal(3, Count(aW, ShapeType.Edge));
        Assert.Equal(3, Count(aW, ShapeType.Vertex));
        Assert.True(AllFrozen(aW));
        Assert.True(aMP.FirstVertex().IsSame(aMP.LastVertex()) || Tool.Pnt(aMP.FirstVertex()).Distance(new Point3D(0, 0, 0)) == 0);
        Assert.True(aMP.Shape().IsSame(aW));
    }

    [Fact]
    public void Wire_Add_after_a_public_read_leaves_the_published_wire_unchanged()
    {
        Edge e1 = new MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)).Edge();
        Edge e2 = new MakeEdge(new Point3D(1, 0, 0), new Point3D(1, 1, 0)).Edge();
        Edge e3 = new MakeEdge(new Point3D(5, 5, 5), new Point3D(6, 6, 6)).Edge();
        var aMW = new MakeWire(e1);
        Wire aFirst = aMW.Wire();
        Assert.Equal(1, Count(aFirst, ShapeType.Edge));
        aMW.Add(e2);
        Assert.True(aMW.IsDone);
        Assert.Equal(WireError.WireDone, aMW.Error());
        Wire aSecond = aMW.Wire();
        Assert.Equal(1, Count(aFirst, ShapeType.Edge));
        Assert.Equal(2, Count(aSecond, ShapeType.Edge));
        aMW.Add(e3);
        Assert.Equal(WireError.DisconnectedWire, aMW.Error());
        Assert.False(aMW.IsDone);
        Assert.Equal(WireError.EmptyWire, new MakeWire().Error());
    }

    [Fact]
    public void Wire_Modified_answers_the_copy_of_a_frozen_vertex_that_had_to_move()
    {
        Edge e1 = new MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)).Edge();
        Edge e2 = new MakeEdge(new Point3D(1 + 5e-8, 0, 0), new Point3D(1, 1, 0)).Edge();
        ShapeSnapshot s1 = ShapeSnapshot.Of(e1), s2 = ShapeSnapshot.Of(e2);
        var aMW = new MakeWire(e1, e2);
        Wire aW = aMW.Wire();
        s1.AssertUnchanged(e1);
        s2.AssertUnchanged(e2);
        ISchematiqList<Shape> aModified = aMW.Modified(e1);
        Shape aCopy = Assert.Single(aModified);
        Assert.True(aCopy.TShape()!.IsFrozen);
        Assert.True(new Explorer(aW, ShapeType.Edge).Current().IsSame(aCopy));
        Assert.False(aMW.IsDeleted(e1));

        // a wire of edges that meet exactly copies nothing, and Modified is empty as in OCCT
        var aPlain = new MakeWire(new MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)).Edge());
        Assert.Empty(aPlain.Modified(aPlain.Edge()));
    }

    [Fact]
    public void Face_from_a_frozen_wire_answers_its_replacements_in_Modified()
    {
        var aV = new Vertex[4];
        Point3D[] aP = [new(0, 0, 0), new(1, 0, 0), new(1, 1, 0), new(0, 1, 0)];
        for (int i = 0; i < 4; i++)
        {
            aV[i] = TestShapes.Vertex(aP[i]);
        }
        var aB = new BrepBuilder();
        Vertex aOff = new();
        aB.MakeVertex(ref aOff, new Point3D(0, 1.0001, 0), global::Kerf.Engine.Foundation.Precision.Confusion());
        var anEdges = new[]
        {
            TestShapes.Edge(aV[0], aV[1]), TestShapes.Edge(aV[1], aV[2]),
            TestShapes.Edge(new LineCurve3D(aP[2], new Direction3D(-1, 0, 0)), 0, 1, aV[2], aOff),
            TestShapes.Edge(new LineCurve3D(aP[3], new Direction3D(0, -1, 0)), 0, 1, aOff, aV[0]),
        };
        Wire aWire = new MakeWire(TestShapes.Frozen(anEdges[0]), TestShapes.Frozen(anEdges[1]),
                                                        TestShapes.Frozen(anEdges[2]), TestShapes.Frozen(anEdges[3])).Wire();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aWire);
        var aMF = new MakeFace(aWire, true);
        Assert.True(aMF.IsDone);
        Assert.Equal(FaceError.FaceDone, aMF.Error());
        Face aF = aMF.Face();
        aSnap.AssertUnchanged(aWire);
        Assert.Empty(aMF.Modified(anEdges[0]));
        Assert.Single(aMF.Modified(anEdges[2]));
        Vertex aNewOff = ShapeCasts.Vertex(Assert.Single(aMF.Modified(aOff)));
        Assert.True(aNewOff.TShape()!.IsFrozen);
        Assert.Equal(1e-4, Tool.Tolerance(aNewOff), 1e-9);
        Assert.True(AllFrozen(aF));

        // the history through BRepTools_History.IAlgorithm
        var anArgs = new NCollection_List<Shape>();
        anArgs.Append(aWire);
        var aHistory = new History(anArgs, aMF);
        Assert.True(aHistory.Modified(aOff)[0].IsSame(aNewOff));
        Assert.Empty(aHistory.Modified(anEdges[0]));
    }

    [Fact]
    public void Face_maker_maps_BRepLib_errors_and_adds_holes()
    {
        Wire aSkew = new MakePolygon(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(1, 1, 0.5),
                                                           new Point3D(0, 1, 0), true).Wire();
        var aNotPlanar = new MakeFace(aSkew, true);
        Assert.False(aNotPlanar.IsDone);
        Assert.Equal(FaceError.NotPlanar, aNotPlanar.Error());
        Assert.Equal(FaceError.NoFace, new MakeFace().Error());

        var aSquare = new MakeFace(new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), -2, 2, -2, 2);
        Face aBefore = aSquare.Face();
        Wire aHole = new MakePolygon(new Point3D(-1, -1, 0), new Point3D(-1, 1, 0), new Point3D(1, 1, 0),
                                                           new Point3D(1, -1, 0), true).Wire();
        aSquare.Add(aHole);
        Assert.True(aSquare.IsDone);
        Face anAfter = aSquare.Face();
        Assert.Equal(1, Count(aBefore, ShapeType.Wire));
        Assert.Equal(2, Count(anAfter, ShapeType.Wire));
        var aProps = new Properties();
        MassPropertiesBuilder.SurfaceProperties(anAfter, aProps);
        Assert.Equal(12.0, aProps.Mass(), 1e-9);
    }

    [Fact]
    public void Shell_and_solid_makers()
    {
        var aSurf = new CylindricalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 2);
        var aMS = new MakeShell(aSurf, 0, Math.PI, 0, 1);
        Assert.True(aMS.IsDone);
        Assert.Equal(ShellError.ShellDone, aMS.Error());
        Shell aShell = aMS.Shell();
        Assert.True(AllFrozen(aShell));
        Assert.Equal(1, Count(aShell, ShapeType.Face));

        Shape aBox = Box10();
        Shell aBoxShell = ShapeCasts.Shell(new Explorer(aBox, ShapeType.Shell).Current());
        var aSolid = new MakeSolid(aBoxShell);
        Assert.True(aSolid.IsDone);
        Solid aS = aSolid.Solid();
        Assert.True(aS.TShape()!.IsFrozen);
        Assert.True(new Iterator(aS).Value().IsSame(aBoxShell));
        Assert.False(aSolid.IsDeleted(new Explorer(aBox, ShapeType.Face).Current()));
        // Add after the public read: the published solid keeps one shell
        aSolid.Add(ShapeCasts.Shell(aShell));
        Assert.Equal(1, Count(aS, ShapeType.Shell));
        Assert.Equal(2, Count(aSolid.Solid(), ShapeType.Shell));
    }

    // ------------------------------------------------------------ modifications

    [Fact]
    public void Transform_by_a_rigid_motion_moves_the_input_and_shares_its_TShapes()
    {
        Shape aBox = Box10();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        var aTrsf = new Transform3D();
        aTrsf.SetRotation(new Axis3D(new Point3D(1, 2, 3), new Direction3D(0, 1, 0)), 0.3);
        var aT = new Transform(aBox, aTrsf);
        Assert.True(aT.IsDone);
        Shape aR = aT.Shape();
        Assert.True(aR.IsPartner(aBox));
        Assert.Equal(aTrsf, aR.Location().Transformation());
        Shape aFace = new Explorer(aBox, ShapeType.Face).Current();
        Shape aImage = aT.ModifiedShape(aFace);
        Assert.True(aImage.IsPartner(aFace));
        Assert.True(Assert.Single(aT.Modified(aFace)).IsEqual(aImage));
        aSnap.AssertUnchanged(aBox);

        // with copyGeom the modifier builds new shapes
        var aCopied = new Transform(aBox, aTrsf, true);
        Shape aR2 = aCopied.Shape();
        Assert.False(aR2.IsPartner(aBox));
        Assert.True(AllFrozen(aR2));
        Assert.Equal(Count(aBox, ShapeType.Edge), Count(aR2, ShapeType.Edge));
        Assert.Equal(Count(aBox, ShapeType.Vertex), Count(aR2, ShapeType.Vertex));
        aSnap.AssertUnchanged(aBox);
    }

    [Fact]
    public void Transform_by_a_scaling_rebuilds_the_geometry()
    {
        Shape aBox = Box10();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        var aTrsf = new Transform3D();
        aTrsf.SetScale(new Point3D(0, 0, 0), 0.5);
        var aT = new Transform(aTrsf);
        Assert.False(aT.IsDone);
        aT.Perform(aBox);
        Assert.True(aT.IsDone);
        Shape aR = aT.Shape();
        Assert.True(AllFrozen(aR));
        var aProps = new Properties();
        MassPropertiesBuilder.VolumeProperties(aR, aProps);
        Assert.Equal(125.0, aProps.Mass(), 1e-9);
        for (var anExp = new Explorer(aBox, ShapeType.Vertex); anExp.More(); anExp.Next())
        {
            Vertex aV = ShapeCasts.Vertex(aT.ModifiedShape(anExp.Current()));
            Point3D anOld = Tool.Pnt(ShapeCasts.Vertex(anExp.Current()));
            Assert.True(Tool.Pnt(aV).IsEqual(new Point3D(anOld.X() / 2, anOld.Y() / 2, anOld.Z() / 2), 1e-12));
        }
        aSnap.AssertUnchanged(aBox);

        // the same framework reused on another shape
        Edge anE = new MakeEdge(new Point3D(0, 0, 0), new Point3D(4, 0, 0)).Edge();
        aT.Perform(anE);
        var aLProps = new Properties();
        MassPropertiesBuilder.LinearProperties(aT.Shape(), aLProps);
        Assert.Equal(2.0, aLProps.Mass(), 1e-12);
    }

    [Fact]
    public void Copy_makes_new_TShapes_and_can_be_reused()
    {
        Shape aBox = Box10();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        var aC = new Copy();
        Assert.False(aC.IsDone);
        aC.Perform(aBox, false);
        Shape aFirst = aC.Shape();
        Assert.False(aFirst.IsPartner(aBox));
        Assert.True(AllFrozen(aFirst));
        Shape aFirstVertex = aC.ModifiedShape(new Explorer(aBox, ShapeType.Vertex).Current());

        // OCCT: copying the same shape again ("on force la copie") gives the same result, since the modifier keeps the
        // images of the shapes it already rebuilt; only the vertices are made again, so ModifiedShape of a vertex is
        // then not in the result (occt-issues.md)
        aC.Perform(aBox, false);
        Shape aSecond = aC.Shape();
        Assert.True(aSecond.IsPartner(aFirst));
        Shape aSecondVertex = aC.ModifiedShape(new Explorer(aBox, ShapeType.Vertex).Current());
        Assert.False(aSecondVertex.IsSame(aFirstVertex));
        var aResultVertices = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(aSecond, ShapeType.Vertex, aResultVertices);
        Assert.True(aResultVertices.Contains(aFirstVertex));
        Assert.False(aResultVertices.Contains(aSecondVertex));
        Assert.True(new Copy(aBox, false).Shape().IsPartner(aFirst) == false);
        // copyGeom false shares the surfaces
        Face aF = ShapeCasts.Face(new Explorer(aBox, ShapeType.Face).Current());
        Face aFc = ShapeCasts.Face(aC.ModifiedShape(aF));
        Assert.Same(Tool.Surface(aF), Tool.Surface(aFc));
        var aDeep = new Copy(aBox);
        Face aFd = ShapeCasts.Face(aDeep.ModifiedShape(aF));
        Assert.NotSame(Tool.Surface(aF), Tool.Surface(aFd));
        aSnap.AssertUnchanged(aBox);

        // the history of every sub-shape, as BRepTools.CopyForWrite records it
        var anArgs = new NCollection_List<Shape>();
        anArgs.Append(aBox);
        var aHistory = new History(anArgs, aDeep);
        Assert.True(aHistory.Modified(aF)[0].IsSame(aFd));
        Assert.Throws<global::Kerf.Engine.Foundation.NullObjectException>(() => new Copy(new Shape()));
    }

    [Fact]
    public void Copy_reused_on_a_shape_with_continuities_does_not_write_into_its_published_result()
    {
        // a box whose first edge is G1 between its two faces: the modifier sets the continuity on the copied edge,
        // and a second Perform on the same shape sets it again on the first result's edge, published (frozen) since
        Shape aBox = TestShapes.Box(10.0, 10.0, 10.0);
        var aMEF = new NCollection_IndexedDataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(aBox, ShapeType.Edge, ShapeType.Face, aMEF);
        Edge anE = ShapeCasts.Edge(aMEF.FindKey(1));
        Face aF1 = ShapeCasts.Face(aMEF[1].First()), aF2 = ShapeCasts.Face(aMEF[1].Last());
        new BrepBuilder().Continuity(anE, aF1, aF2, Continuity.G1);
        TestShapes.Frozen(aBox);

        var aC = new Copy(aBox);
        Shape aFirst = aC.Shape();
        Edge aCopiedE = ShapeCasts.Edge(aC.ModifiedShape(anE));
        Assert.Equal(Continuity.G1,
                     Tool.Continuity(aCopiedE, ShapeCasts.Face(aC.ModifiedShape(aF1)), ShapeCasts.Face(aC.ModifiedShape(aF2))));
        aC.Perform(aBox);
        Assert.True(aC.Shape().IsPartner(aFirst));
    }

    [Fact]
    public void NurbsConvert_converts_curves_and_surfaces_and_keeps_the_input()
    {
        Face aCyl = TestShapes.Frozen(TestShapes.CylinderFace(2, 3));
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aCyl);
        var aNC = new NurbsConvert(aCyl);
        Assert.True(aNC.IsDone);
        Shape aR = aNC.Shape();
        Assert.True(AllFrozen(aR));
        Face aRF = ShapeCasts.Face(new Explorer(aR, ShapeType.Face).Current());
        Assert.Equal(SurfaceKind.BSplineSurface, new FaceSurfaceAdaptor(aRF).GetType());
        for (var anExp = new Explorer(aCyl, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Edge anE = ShapeCasts.Edge(aNC.ModifiedShape(anExp.Current()));
            if (!Tool.Degenerated(anE))
            {
                Assert.Equal(BSplineCurve, new EdgeCurveAdaptor(anE).GetType());
            }
            Assert.True(Assert.Single(aNC.Modified(anExp.Current())).IsEqual(anE));
        }
        for (var anExp = new Explorer(aCyl, ShapeType.Vertex); anExp.More(); anExp.Next())
        {
            Vertex aV = ShapeCasts.Vertex(aNC.ModifiedShape(anExp.Current()));
            Assert.True(Tool.Tolerance(aV) >= Tool.Tolerance(ShapeCasts.Vertex(anExp.Current())));
        }
        aSnap.AssertUnchanged(aCyl);
    }

    // OCCT: NurbsConvertModification keeps a B-spline face's surface (NewSurface is false) but copies its
    // triangulation, and BRepTools_Modifier then makes the face from the triangulation alone (occt-issues.md)
    [Fact]
    public void OcctIssue_NurbsConvert_of_a_meshed_BSpline_face_drops_its_surface()
    {
        var aPoles = new NCollection_Array2<Point3D>(1, 2, 1, 2);
        aPoles[1, 1] = new Point3D(0, 0, 0);
        aPoles[1, 2] = new Point3D(0, 1, 0);
        aPoles[2, 1] = new Point3D(1, 0, 0);
        aPoles[2, 2] = new Point3D(1, 1, 0.5);
        var aSurf = new global::Kerf.Geometry.BezierSurface(aPoles);
        Face aF = new MakeFace(aSurf, 0, 1, 0, 1, global::Kerf.Engine.Foundation.Precision.Confusion()).InternalFace();
        var aTri = new Triangulation(4, 2, true);
        (double u, double v)[] aUV = [(0, 0), (1, 0), (1, 1), (0, 1)];
        for (int i = 0; i < 4; i++)
        {
            aTri.SetNode(i + 1, aSurf.Value(aUV[i].u, aUV[i].v));
            aTri.SetUVNode(i + 1, new Point2D(aUV[i].u, aUV[i].v));
        }
        aTri.SetTriangle(1, new Triangle1(1, 2, 3));
        aTri.SetTriangle(2, new Triangle1(1, 3, 4));
        new BrepBuilder().UpdateFace(aF, aTri);
        TestShapes.Frozen(aF);

        Face aConverted = ShapeCasts.Face(new NurbsConvert(aF).Shape());
        Assert.Null(Tool.Surface(aConverted));
        Assert.NotNull(Tool.Triangulation(aConverted, out _));
        Assert.NotNull(Tool.Surface(aF));

        // without the mesh the face keeps its surface
        Face aBare = new MakeFace(aSurf, 0, 1, 0, 1, global::Kerf.Engine.Foundation.Precision.Confusion()).Face();
        Assert.NotNull(Tool.Surface(ShapeCasts.Face(new NurbsConvert(aBare).Shape())));
    }

    [Fact]
    public void GTransform_by_an_affinity_turns_a_circle_into_a_B_spline_ellipse()
    {
        Edge aCircle = new MakeEdge(new Circle3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 1)).Edge();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aCircle);
        var aG = new GeneralTransform3D();
        aG.SetValue(1, 1, 3.0);
        var aGT = new GTransform(aCircle, aG);
        Assert.True(aGT.IsDone);
        Edge anE = ShapeCasts.Edge(aGT.Shape());
        Assert.True(anE.TShape()!.IsFrozen);
        var aC = new EdgeCurveAdaptor(anE);
        Assert.Equal(BSplineCurve, aC.GetType());
        Point3D aP = aC.Value(aC.FirstParameter());
        Assert.Equal(3.0, aP.X(), 1e-9);
        Assert.True(aGT.ModifiedShape(aCircle).IsSame(anE));
        // Modified: OCCT's list of the last converted shape's image
        Assert.True(Assert.Single(aGT.Modified(aCircle)).IsSame(anE));
        var aProps = new Properties();
        MassPropertiesBuilder.LinearProperties(anE, aProps);
        Assert.Equal(13.364893591540853, aProps.Mass(), 1e-3); // the perimeter of the ellipse (3, 1), by Gauss integration
        aSnap.AssertUnchanged(aCircle);
    }

    // ------------------------------------------------------------ helpers

    [Fact]
    public void Collect_records_images_against_the_initial_shapes()
    {
        Shape aBox = Box10();
        var aT = new Transform(aBox, Transform3D.Translation(new Vector3D(1, 0, 0)), true);
        var aCollect = new Collect();
        aCollect.Add(aBox, aT);
        Shape aF = new Explorer(aBox, ShapeType.Face).Current();
        Assert.True(aCollect.Modification().IsBound(aF));
        Shape aImage = aCollect.Modification()[aF].First();
        Assert.True(aImage.IsSame(aT.ModifiedShape(aF)));

        // a second modification of the image is recorded against the initial face
        var aNew = new MakeVertex(new Point3D(0, 0, 0)).Vertex();
        aCollect.AddModif(aImage, aNew);
        Assert.True(aCollect.Modification()[aF].First().IsSame(aNew));
        Assert.False(aCollect.Modification().IsBound(aImage));
        aCollect.AddGenerated(aF, aNew);
        Assert.True(aCollect.Generated().IsBound(aF));

        // Filter keeps only images in the shape: the vertex isn't one of the transformed box's
        aCollect.Filter(aT.Shape());
        Assert.Equal(0, aCollect.Modification()[aF].Extent());
        Shape anE = new Explorer(aBox, ShapeType.Edge).Current();
        Assert.Equal(1, aCollect.Modification()[anE].Extent());
    }

    [Fact]
    public void FindPlane_finds_the_plane_of_planar_edges_only()
    {
        Wire aSquare = new MakePolygon(new Point3D(0, 0, 1), new Point3D(1, 0, 1), new Point3D(1, 1, 1),
                                                             new Point3D(0, 1, 1), true).Wire();
        var aFP = new FindPlane(aSquare);
        Assert.True(aFP.Found());
        PlaneSurface aPl = aFP.Plane()!;
        Assert.True(aPl.IsFrozen);
        Assert.True(aPl.Pln().Axis().Direction().IsParallel(new Direction3D(0, 0, 1), 1e-12));
        Assert.Equal(0.0, aPl.Pln().Distance(new Point3D(0.3, 0.7, 1)), 1e-12);

        Wire aSkew = new MakePolygon(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(1, 1, 0.5),
                                                           new Point3D(0, 1, 0), true).Wire();
        Assert.False(new FindPlane(aSkew).Found());
        Assert.True(new FindPlane(aSkew, 0.5).Found());

        // a conic gives its own plane
        Edge aCirc = new MakeEdge(new Circle3D(new Frame3D(new Point3D(0, 0, 5), new Direction3D(1, 0, 0)), 2)).Edge();
        var aFC = new FindPlane(aCirc);
        Assert.True(aFC.Found());
        Assert.True(aFC.Plane()!.Pln().Axis().Direction().IsParallel(new Direction3D(1, 0, 0), 1e-12));
        Assert.False(new FindPlane().Found());
    }
}
