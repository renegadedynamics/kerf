// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Schematiq.Collections;
using OcctNet.ModelingData.Tests;
using static Occt.BRepBuilderAPI_EdgeError;
using static Occt.BRepBuilderAPI_FaceError;
using static Occt.BRepBuilderAPI_ShellError;
using static Occt.BRepBuilderAPI_WireError;
using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_SurfaceType;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Known answers of BRepBuilderAPI: the makers over BRepLib's (results, errors, freezing at publication, the internal
/// accessors, Add after a public read, Modified answering the copies of frozen inputs), the modifications (Transform
/// as a location or a modifier, Copy, GTransform, NurbsConvert, their ModifiedShape and Modified, histories through
/// <see cref="BRepTools_History.IAlgorithm"/>), Collect, FindPlane and the package statics. Inputs are frozen, as
/// callers hold them, and checked unchanged with <see cref="ShapeSnapshot"/>.
/// </summary>
public class BRepBuilderAPITests
{
    private static int Count(in TopoDS_Shape theS, TopAbs_ShapeEnum theType)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theS, theType, aMap);
        return aMap.Extent();
    }

    private static TopoDS_Shape Box10() => TestShapes.Frozen(TestShapes.Box(10.0, 10.0, 10.0));

    private static bool AllFrozen(in TopoDS_Shape theS)
    {
        for (var anExp = new TopExp_Explorer(theS, TopAbs_VERTEX); anExp.More(); anExp.Next())
        {
            if (!anExp.Current().TShape()!.IsFrozen())
            {
                return false;
            }
        }
        return theS.TShape()!.IsFrozen();
    }

    // ------------------------------------------------------------ makers

    [Fact]
    public void Vertex_is_frozen_when_read_and_not_through_the_internal_accessor()
    {
        var aMV = new BRepBuilderAPI_MakeVertex(new gp_Pnt(1, 2, 3));
        Assert.True(aMV.IsDone());
        TopoDS_Vertex anInternal = aMV.InternalVertex();
        Assert.False(anInternal.TShape()!.IsFrozen());
        TopoDS_Vertex aV = aMV.Vertex();
        Assert.True(aV.TShape()!.IsFrozen());
        Assert.True(aV.IsSame(anInternal));
        Assert.True(aV.IsSame((TopoDS_Vertex)aMV));
        Assert.Equal(new gp_Pnt(1, 2, 3), BRep_Tool.Pnt(aV));
        Assert.Equal(Precision.Confusion(), BRep_Tool.Tolerance(aV));
    }

    [Fact]
    public void Edge_maker_maps_BRepLib_errors()
    {
        var aSame = new BRepBuilderAPI_MakeEdge(new gp_Pnt(1, 1, 1), new gp_Pnt(1, 1, 1));
        Assert.False(aSame.IsDone());
        Assert.Equal(BRepBuilderAPI_LineThroughIdenticPoints, aSame.Error());
        Assert.Throws<StdFail_NotDone>(() => aSame.Shape());

        var aCirc = new gp_Circ(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 1);
        var aOff = new BRepBuilderAPI_MakeEdge(aCirc, new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0));
        Assert.Equal(BRepBuilderAPI_PointProjectionFailed, aOff.Error());

        var aLine = new BRepBuilderAPI_MakeEdge(new gp_Lin(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0)), 0, 2);
        Assert.Equal(BRepBuilderAPI_EdgeDone, aLine.Error());
        Assert.True(aLine.Vertex1().TShape()!.IsFrozen());
        Assert.True(aLine.Edge().TShape()!.IsFrozen());
        Assert.Equal(new gp_Pnt(2, 0, 0), BRep_Tool.Pnt(aLine.Vertex2()));
    }

    [Fact]
    public void Edge_maker_Init_after_a_public_read_makes_a_new_edge()
    {
        var aME = new BRepBuilderAPI_MakeEdge(new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0)), 0, 1);
        TopoDS_Edge aFirst = aME.Edge();
        aME.Init(new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(0, 1, 0)), 0, 3);
        Assert.True(aME.IsDone());
        TopoDS_Edge aSecond = aME.Edge();
        Assert.False(aSecond.IsSame(aFirst));
        BRep_Tool.Range(aFirst, out double f, out double l);
        Assert.Equal(1.0, l - f);
        BRep_Tool.Range(aSecond, out f, out l);
        Assert.Equal(3.0, l - f);
    }

    [Fact]
    public void Edge2d_lies_in_the_default_plane()
    {
        var aME = new BRepBuilderAPI_MakeEdge2d(new gp_Pnt2d(0, 0), new gp_Pnt2d(3, 4));
        Assert.True(aME.IsDone());
        Assert.Equal(BRepBuilderAPI_EdgeDone, aME.Error());
        TopoDS_Edge anE = aME.Edge();
        Geom2d_Curve? aC = BRep_Tool.CurveOnSurface(anE, BRepBuilderAPI.Plane(), new TopLoc_Location(), out double f, out double l);
        Assert.NotNull(aC);
        Assert.Equal(5.0, l - f, 1e-12);
        Assert.True(BRepBuilderAPI.Plane().IsFrozen());
        Assert.Equal(Precision.Confusion(), BRepBuilderAPI.Precision());
        Assert.Equal(BRepBuilderAPI_LineThroughIdenticPoints,
                     new BRepBuilderAPI_MakeEdge2d(new gp_Pnt2d(1, 1), new gp_Pnt2d(1, 1)).Error());
    }

    [Fact]
    public void Polygon_adds_points_closes_and_reports_coincident_points()
    {
        var aMP = new BRepBuilderAPI_MakePolygon();
        aMP.Add(new gp_Pnt(0, 0, 0));
        Assert.False(aMP.IsDone());
        aMP.Add(new gp_Pnt(1, 0, 0));
        Assert.True(aMP.Added());
        Assert.True(aMP.IsDone());
        aMP.Add(new gp_Pnt(1, 0, 0));
        Assert.False(aMP.Added());
        aMP.Add(new gp_Pnt(1, 1, 0));
        // the internal accessors don't freeze: the maker goes on in place
        Assert.False(aMP.InternalLastVertex().TShape()!.IsFrozen());
        aMP.Close();
        TopoDS_Wire aW = aMP.Wire();
        Assert.True(aW.Closed());
        Assert.Equal(3, Count(aW, TopAbs_EDGE));
        Assert.Equal(3, Count(aW, TopAbs_VERTEX));
        Assert.True(AllFrozen(aW));
        Assert.True(aMP.FirstVertex().IsSame(aMP.LastVertex()) || BRep_Tool.Pnt(aMP.FirstVertex()).Distance(new gp_Pnt(0, 0, 0)) == 0);
        Assert.True(aMP.Shape().IsSame(aW));
    }

    [Fact]
    public void Wire_Add_after_a_public_read_leaves_the_published_wire_unchanged()
    {
        TopoDS_Edge e1 = new BRepBuilderAPI_MakeEdge(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0)).Edge();
        TopoDS_Edge e2 = new BRepBuilderAPI_MakeEdge(new gp_Pnt(1, 0, 0), new gp_Pnt(1, 1, 0)).Edge();
        TopoDS_Edge e3 = new BRepBuilderAPI_MakeEdge(new gp_Pnt(5, 5, 5), new gp_Pnt(6, 6, 6)).Edge();
        var aMW = new BRepBuilderAPI_MakeWire(e1);
        TopoDS_Wire aFirst = aMW.Wire();
        Assert.Equal(1, Count(aFirst, TopAbs_EDGE));
        aMW.Add(e2);
        Assert.True(aMW.IsDone());
        Assert.Equal(BRepBuilderAPI_WireDone, aMW.Error());
        TopoDS_Wire aSecond = aMW.Wire();
        Assert.Equal(1, Count(aFirst, TopAbs_EDGE));
        Assert.Equal(2, Count(aSecond, TopAbs_EDGE));
        aMW.Add(e3);
        Assert.Equal(BRepBuilderAPI_DisconnectedWire, aMW.Error());
        Assert.False(aMW.IsDone());
        Assert.Equal(BRepBuilderAPI_EmptyWire, new BRepBuilderAPI_MakeWire().Error());
    }

    [Fact]
    public void Wire_Modified_answers_the_copy_of_a_frozen_vertex_that_had_to_move()
    {
        TopoDS_Edge e1 = new BRepBuilderAPI_MakeEdge(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0)).Edge();
        TopoDS_Edge e2 = new BRepBuilderAPI_MakeEdge(new gp_Pnt(1 + 5e-8, 0, 0), new gp_Pnt(1, 1, 0)).Edge();
        ShapeSnapshot s1 = ShapeSnapshot.Of(e1), s2 = ShapeSnapshot.Of(e2);
        var aMW = new BRepBuilderAPI_MakeWire(e1, e2);
        TopoDS_Wire aW = aMW.Wire();
        s1.AssertUnchanged(e1);
        s2.AssertUnchanged(e2);
        ISchematiqList<TopoDS_Shape> aModified = aMW.Modified(e1);
        TopoDS_Shape aCopy = Assert.Single(aModified);
        Assert.True(aCopy.TShape()!.IsFrozen());
        Assert.True(new TopExp_Explorer(aW, TopAbs_EDGE).Current().IsSame(aCopy));
        Assert.False(aMW.IsDeleted(e1));

        // a wire of edges that meet exactly copies nothing, and Modified is empty as in OCCT
        var aPlain = new BRepBuilderAPI_MakeWire(new BRepBuilderAPI_MakeEdge(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0)).Edge());
        Assert.Empty(aPlain.Modified(aPlain.Edge()));
    }

    [Fact]
    public void Face_from_a_frozen_wire_answers_its_replacements_in_Modified()
    {
        var aV = new TopoDS_Vertex[4];
        gp_Pnt[] aP = [new(0, 0, 0), new(1, 0, 0), new(1, 1, 0), new(0, 1, 0)];
        for (int i = 0; i < 4; i++)
        {
            aV[i] = TestShapes.Vertex(aP[i]);
        }
        var aB = new BRep_Builder();
        TopoDS_Vertex aOff = new();
        aB.MakeVertex(ref aOff, new gp_Pnt(0, 1.0001, 0), Precision.Confusion());
        var anEdges = new[]
        {
            TestShapes.Edge(aV[0], aV[1]), TestShapes.Edge(aV[1], aV[2]),
            TestShapes.Edge(new Geom_Line(aP[2], new gp_Dir(-1, 0, 0)), 0, 1, aV[2], aOff),
            TestShapes.Edge(new Geom_Line(aP[3], new gp_Dir(0, -1, 0)), 0, 1, aOff, aV[0]),
        };
        TopoDS_Wire aWire = new BRepBuilderAPI_MakeWire(TestShapes.Frozen(anEdges[0]), TestShapes.Frozen(anEdges[1]),
                                                        TestShapes.Frozen(anEdges[2]), TestShapes.Frozen(anEdges[3])).Wire();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aWire);
        var aMF = new BRepBuilderAPI_MakeFace(aWire, true);
        Assert.True(aMF.IsDone());
        Assert.Equal(BRepBuilderAPI_FaceDone, aMF.Error());
        TopoDS_Face aF = aMF.Face();
        aSnap.AssertUnchanged(aWire);
        Assert.Empty(aMF.Modified(anEdges[0]));
        Assert.Single(aMF.Modified(anEdges[2]));
        TopoDS_Vertex aNewOff = TopoDS.Vertex(Assert.Single(aMF.Modified(aOff)));
        Assert.True(aNewOff.TShape()!.IsFrozen());
        Assert.Equal(1e-4, BRep_Tool.Tolerance(aNewOff), 1e-9);
        Assert.True(AllFrozen(aF));

        // the history through BRepTools_History.IAlgorithm
        var anArgs = new NCollection_List<TopoDS_Shape>();
        anArgs.Append(aWire);
        var aHistory = new BRepTools_History(anArgs, aMF);
        Assert.True(aHistory.Modified(aOff)[0].IsSame(aNewOff));
        Assert.Empty(aHistory.Modified(anEdges[0]));
    }

    [Fact]
    public void Face_maker_maps_BRepLib_errors_and_adds_holes()
    {
        TopoDS_Wire aSkew = new BRepBuilderAPI_MakePolygon(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0), new gp_Pnt(1, 1, 0.5),
                                                           new gp_Pnt(0, 1, 0), true).Wire();
        var aNotPlanar = new BRepBuilderAPI_MakeFace(aSkew, true);
        Assert.False(aNotPlanar.IsDone());
        Assert.Equal(BRepBuilderAPI_NotPlanar, aNotPlanar.Error());
        Assert.Equal(BRepBuilderAPI_NoFace, new BRepBuilderAPI_MakeFace().Error());

        var aSquare = new BRepBuilderAPI_MakeFace(new gp_Pln(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), -2, 2, -2, 2);
        TopoDS_Face aBefore = aSquare.Face();
        TopoDS_Wire aHole = new BRepBuilderAPI_MakePolygon(new gp_Pnt(-1, -1, 0), new gp_Pnt(-1, 1, 0), new gp_Pnt(1, 1, 0),
                                                           new gp_Pnt(1, -1, 0), true).Wire();
        aSquare.Add(aHole);
        Assert.True(aSquare.IsDone());
        TopoDS_Face anAfter = aSquare.Face();
        Assert.Equal(1, Count(aBefore, TopAbs_WIRE));
        Assert.Equal(2, Count(anAfter, TopAbs_WIRE));
        var aProps = new GProp_GProps();
        BRepGProp.SurfaceProperties(anAfter, aProps);
        Assert.Equal(12.0, aProps.Mass(), 1e-9);
    }

    [Fact]
    public void Shell_and_solid_makers()
    {
        var aSurf = new Geom_CylindricalSurface(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 2);
        var aMS = new BRepBuilderAPI_MakeShell(aSurf, 0, Math.PI, 0, 1);
        Assert.True(aMS.IsDone());
        Assert.Equal(BRepBuilderAPI_ShellDone, aMS.Error());
        TopoDS_Shell aShell = aMS.Shell();
        Assert.True(AllFrozen(aShell));
        Assert.Equal(1, Count(aShell, TopAbs_FACE));

        TopoDS_Shape aBox = Box10();
        TopoDS_Shell aBoxShell = TopoDS.Shell(new TopExp_Explorer(aBox, TopAbs_SHELL).Current());
        var aSolid = new BRepBuilderAPI_MakeSolid(aBoxShell);
        Assert.True(aSolid.IsDone());
        TopoDS_Solid aS = aSolid.Solid();
        Assert.True(aS.TShape()!.IsFrozen());
        Assert.True(new TopoDS_Iterator(aS).Value().IsSame(aBoxShell));
        Assert.False(aSolid.IsDeleted(new TopExp_Explorer(aBox, TopAbs_FACE).Current()));
        // Add after the public read: the published solid keeps one shell
        aSolid.Add(TopoDS.Shell(aShell));
        Assert.Equal(1, Count(aS, TopAbs_SHELL));
        Assert.Equal(2, Count(aSolid.Solid(), TopAbs_SHELL));
    }

    // ------------------------------------------------------------ modifications

    [Fact]
    public void Transform_by_a_rigid_motion_moves_the_input_and_shares_its_TShapes()
    {
        TopoDS_Shape aBox = Box10();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        var aTrsf = new gp_Trsf();
        aTrsf.SetRotation(new gp_Ax1(new gp_Pnt(1, 2, 3), new gp_Dir(0, 1, 0)), 0.3);
        var aT = new BRepBuilderAPI_Transform(aBox, aTrsf);
        Assert.True(aT.IsDone());
        TopoDS_Shape aR = aT.Shape();
        Assert.True(aR.IsPartner(aBox));
        Assert.Equal(aTrsf, aR.Location().Transformation());
        TopoDS_Shape aFace = new TopExp_Explorer(aBox, TopAbs_FACE).Current();
        TopoDS_Shape aImage = aT.ModifiedShape(aFace);
        Assert.True(aImage.IsPartner(aFace));
        Assert.True(Assert.Single(aT.Modified(aFace)).IsEqual(aImage));
        aSnap.AssertUnchanged(aBox);

        // with copyGeom the modifier builds new shapes
        var aCopied = new BRepBuilderAPI_Transform(aBox, aTrsf, true);
        TopoDS_Shape aR2 = aCopied.Shape();
        Assert.False(aR2.IsPartner(aBox));
        Assert.True(AllFrozen(aR2));
        Assert.Equal(Count(aBox, TopAbs_EDGE), Count(aR2, TopAbs_EDGE));
        Assert.Equal(Count(aBox, TopAbs_VERTEX), Count(aR2, TopAbs_VERTEX));
        aSnap.AssertUnchanged(aBox);
    }

    [Fact]
    public void Transform_by_a_scaling_rebuilds_the_geometry()
    {
        TopoDS_Shape aBox = Box10();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        var aTrsf = new gp_Trsf();
        aTrsf.SetScale(new gp_Pnt(0, 0, 0), 0.5);
        var aT = new BRepBuilderAPI_Transform(aTrsf);
        Assert.False(aT.IsDone());
        aT.Perform(aBox);
        Assert.True(aT.IsDone());
        TopoDS_Shape aR = aT.Shape();
        Assert.True(AllFrozen(aR));
        var aProps = new GProp_GProps();
        BRepGProp.VolumeProperties(aR, aProps);
        Assert.Equal(125.0, aProps.Mass(), 1e-9);
        for (var anExp = new TopExp_Explorer(aBox, TopAbs_VERTEX); anExp.More(); anExp.Next())
        {
            TopoDS_Vertex aV = TopoDS.Vertex(aT.ModifiedShape(anExp.Current()));
            gp_Pnt anOld = BRep_Tool.Pnt(TopoDS.Vertex(anExp.Current()));
            Assert.True(BRep_Tool.Pnt(aV).IsEqual(new gp_Pnt(anOld.X() / 2, anOld.Y() / 2, anOld.Z() / 2), 1e-12));
        }
        aSnap.AssertUnchanged(aBox);

        // the same framework reused on another shape
        TopoDS_Edge anE = new BRepBuilderAPI_MakeEdge(new gp_Pnt(0, 0, 0), new gp_Pnt(4, 0, 0)).Edge();
        aT.Perform(anE);
        var aLProps = new GProp_GProps();
        BRepGProp.LinearProperties(aT.Shape(), aLProps);
        Assert.Equal(2.0, aLProps.Mass(), 1e-12);
    }

    [Fact]
    public void Copy_makes_new_TShapes_and_can_be_reused()
    {
        TopoDS_Shape aBox = Box10();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        var aC = new BRepBuilderAPI_Copy();
        Assert.False(aC.IsDone());
        aC.Perform(aBox, false);
        TopoDS_Shape aFirst = aC.Shape();
        Assert.False(aFirst.IsPartner(aBox));
        Assert.True(AllFrozen(aFirst));
        TopoDS_Shape aFirstVertex = aC.ModifiedShape(new TopExp_Explorer(aBox, TopAbs_VERTEX).Current());

        // OCCT: copying the same shape again ("on force la copie") gives the same result, since the modifier keeps the
        // images of the shapes it already rebuilt; only the vertices are made again, so ModifiedShape of a vertex is
        // then not in the result (occt-issues.md)
        aC.Perform(aBox, false);
        TopoDS_Shape aSecond = aC.Shape();
        Assert.True(aSecond.IsPartner(aFirst));
        TopoDS_Shape aSecondVertex = aC.ModifiedShape(new TopExp_Explorer(aBox, TopAbs_VERTEX).Current());
        Assert.False(aSecondVertex.IsSame(aFirstVertex));
        var aResultVertices = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aSecond, TopAbs_VERTEX, aResultVertices);
        Assert.True(aResultVertices.Contains(aFirstVertex));
        Assert.False(aResultVertices.Contains(aSecondVertex));
        Assert.True(new BRepBuilderAPI_Copy(aBox, false).Shape().IsPartner(aFirst) == false);
        // copyGeom false shares the surfaces
        TopoDS_Face aF = TopoDS.Face(new TopExp_Explorer(aBox, TopAbs_FACE).Current());
        TopoDS_Face aFc = TopoDS.Face(aC.ModifiedShape(aF));
        Assert.Same(BRep_Tool.Surface(aF), BRep_Tool.Surface(aFc));
        var aDeep = new BRepBuilderAPI_Copy(aBox);
        TopoDS_Face aFd = TopoDS.Face(aDeep.ModifiedShape(aF));
        Assert.NotSame(BRep_Tool.Surface(aF), BRep_Tool.Surface(aFd));
        aSnap.AssertUnchanged(aBox);

        // the history of every sub-shape, as BRepTools.CopyForWrite records it
        var anArgs = new NCollection_List<TopoDS_Shape>();
        anArgs.Append(aBox);
        var aHistory = new BRepTools_History(anArgs, aDeep);
        Assert.True(aHistory.Modified(aF)[0].IsSame(aFd));
        Assert.Throws<Standard_NullObject>(() => new BRepBuilderAPI_Copy(new TopoDS_Shape()));
    }

    [Fact]
    public void Copy_reused_on_a_shape_with_continuities_does_not_write_into_its_published_result()
    {
        // a box whose first edge is G1 between its two faces: the modifier sets the continuity on the copied edge,
        // and a second Perform on the same shape sets it again on the first result's edge, published (frozen) since
        TopoDS_Shape aBox = TestShapes.Box(10.0, 10.0, 10.0);
        var aMEF = new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(aBox, TopAbs_EDGE, TopAbs_FACE, aMEF);
        TopoDS_Edge anE = TopoDS.Edge(aMEF.FindKey(1));
        TopoDS_Face aF1 = TopoDS.Face(aMEF[1].First()), aF2 = TopoDS.Face(aMEF[1].Last());
        new BRep_Builder().Continuity(anE, aF1, aF2, GeomAbs_Shape.GeomAbs_G1);
        TestShapes.Frozen(aBox);

        var aC = new BRepBuilderAPI_Copy(aBox);
        TopoDS_Shape aFirst = aC.Shape();
        TopoDS_Edge aCopiedE = TopoDS.Edge(aC.ModifiedShape(anE));
        Assert.Equal(GeomAbs_Shape.GeomAbs_G1,
                     BRep_Tool.Continuity(aCopiedE, TopoDS.Face(aC.ModifiedShape(aF1)), TopoDS.Face(aC.ModifiedShape(aF2))));
        aC.Perform(aBox);
        Assert.True(aC.Shape().IsPartner(aFirst));
    }

    [Fact]
    public void NurbsConvert_converts_curves_and_surfaces_and_keeps_the_input()
    {
        TopoDS_Face aCyl = TestShapes.Frozen(TestShapes.CylinderFace(2, 3));
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aCyl);
        var aNC = new BRepBuilderAPI_NurbsConvert(aCyl);
        Assert.True(aNC.IsDone());
        TopoDS_Shape aR = aNC.Shape();
        Assert.True(AllFrozen(aR));
        TopoDS_Face aRF = TopoDS.Face(new TopExp_Explorer(aR, TopAbs_FACE).Current());
        Assert.Equal(GeomAbs_BSplineSurface, new BRepAdaptor_Surface(aRF).GetType());
        for (var anExp = new TopExp_Explorer(aCyl, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            TopoDS_Edge anE = TopoDS.Edge(aNC.ModifiedShape(anExp.Current()));
            if (!BRep_Tool.Degenerated(anE))
            {
                Assert.Equal(GeomAbs_BSplineCurve, new BRepAdaptor_Curve(anE).GetType());
            }
            Assert.True(Assert.Single(aNC.Modified(anExp.Current())).IsEqual(anE));
        }
        for (var anExp = new TopExp_Explorer(aCyl, TopAbs_VERTEX); anExp.More(); anExp.Next())
        {
            TopoDS_Vertex aV = TopoDS.Vertex(aNC.ModifiedShape(anExp.Current()));
            Assert.True(BRep_Tool.Tolerance(aV) >= BRep_Tool.Tolerance(TopoDS.Vertex(anExp.Current())));
        }
        aSnap.AssertUnchanged(aCyl);
    }

    // OCCT: NurbsConvertModification keeps a B-spline face's surface (NewSurface is false) but copies its
    // triangulation, and BRepTools_Modifier then makes the face from the triangulation alone (occt-issues.md)
    [Fact]
    public void OcctIssue_NurbsConvert_of_a_meshed_BSpline_face_drops_its_surface()
    {
        var aPoles = new NCollection_Array2<gp_Pnt>(1, 2, 1, 2);
        aPoles[1, 1] = new gp_Pnt(0, 0, 0);
        aPoles[1, 2] = new gp_Pnt(0, 1, 0);
        aPoles[2, 1] = new gp_Pnt(1, 0, 0);
        aPoles[2, 2] = new gp_Pnt(1, 1, 0.5);
        var aSurf = new Geom_BezierSurface(aPoles);
        TopoDS_Face aF = new BRepBuilderAPI_MakeFace(aSurf, 0, 1, 0, 1, Precision.Confusion()).InternalFace();
        var aTri = new Poly_Triangulation(4, 2, true);
        (double u, double v)[] aUV = [(0, 0), (1, 0), (1, 1), (0, 1)];
        for (int i = 0; i < 4; i++)
        {
            aTri.SetNode(i + 1, aSurf.Value(aUV[i].u, aUV[i].v));
            aTri.SetUVNode(i + 1, new gp_Pnt2d(aUV[i].u, aUV[i].v));
        }
        aTri.SetTriangle(1, new Poly_Triangle(1, 2, 3));
        aTri.SetTriangle(2, new Poly_Triangle(1, 3, 4));
        new BRep_Builder().UpdateFace(aF, aTri);
        TestShapes.Frozen(aF);

        TopoDS_Face aConverted = TopoDS.Face(new BRepBuilderAPI_NurbsConvert(aF).Shape());
        Assert.Null(BRep_Tool.Surface(aConverted));
        Assert.NotNull(BRep_Tool.Triangulation(aConverted, out _));
        Assert.NotNull(BRep_Tool.Surface(aF));

        // without the mesh the face keeps its surface
        TopoDS_Face aBare = new BRepBuilderAPI_MakeFace(aSurf, 0, 1, 0, 1, Precision.Confusion()).Face();
        Assert.NotNull(BRep_Tool.Surface(TopoDS.Face(new BRepBuilderAPI_NurbsConvert(aBare).Shape())));
    }

    [Fact]
    public void GTransform_by_an_affinity_turns_a_circle_into_a_B_spline_ellipse()
    {
        TopoDS_Edge aCircle = new BRepBuilderAPI_MakeEdge(new gp_Circ(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 1)).Edge();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aCircle);
        var aG = new gp_GTrsf();
        aG.SetValue(1, 1, 3.0);
        var aGT = new BRepBuilderAPI_GTransform(aCircle, aG);
        Assert.True(aGT.IsDone());
        TopoDS_Edge anE = TopoDS.Edge(aGT.Shape());
        Assert.True(anE.TShape()!.IsFrozen());
        var aC = new BRepAdaptor_Curve(anE);
        Assert.Equal(GeomAbs_BSplineCurve, aC.GetType());
        gp_Pnt aP = aC.Value(aC.FirstParameter());
        Assert.Equal(3.0, aP.X(), 1e-9);
        Assert.True(aGT.ModifiedShape(aCircle).IsSame(anE));
        // Modified: OCCT's list of the last converted shape's image
        Assert.True(Assert.Single(aGT.Modified(aCircle)).IsSame(anE));
        var aProps = new GProp_GProps();
        BRepGProp.LinearProperties(anE, aProps);
        Assert.Equal(13.364893591540853, aProps.Mass(), 1e-3); // the perimeter of the ellipse (3, 1), by Gauss integration
        aSnap.AssertUnchanged(aCircle);
    }

    // ------------------------------------------------------------ helpers

    [Fact]
    public void Collect_records_images_against_the_initial_shapes()
    {
        TopoDS_Shape aBox = Box10();
        var aT = new BRepBuilderAPI_Transform(aBox, gp_Trsf.Translation(new gp_Vec(1, 0, 0)), true);
        var aCollect = new BRepBuilderAPI_Collect();
        aCollect.Add(aBox, aT);
        TopoDS_Shape aF = new TopExp_Explorer(aBox, TopAbs_FACE).Current();
        Assert.True(aCollect.Modification().IsBound(aF));
        TopoDS_Shape aImage = aCollect.Modification()[aF].First();
        Assert.True(aImage.IsSame(aT.ModifiedShape(aF)));

        // a second modification of the image is recorded against the initial face
        var aNew = new BRepBuilderAPI_MakeVertex(new gp_Pnt(0, 0, 0)).Vertex();
        aCollect.AddModif(aImage, aNew);
        Assert.True(aCollect.Modification()[aF].First().IsSame(aNew));
        Assert.False(aCollect.Modification().IsBound(aImage));
        aCollect.AddGenerated(aF, aNew);
        Assert.True(aCollect.Generated().IsBound(aF));

        // Filter keeps only images in the shape: the vertex isn't one of the transformed box's
        aCollect.Filter(aT.Shape());
        Assert.Equal(0, aCollect.Modification()[aF].Extent());
        TopoDS_Shape anE = new TopExp_Explorer(aBox, TopAbs_EDGE).Current();
        Assert.Equal(1, aCollect.Modification()[anE].Extent());
    }

    [Fact]
    public void FindPlane_finds_the_plane_of_planar_edges_only()
    {
        TopoDS_Wire aSquare = new BRepBuilderAPI_MakePolygon(new gp_Pnt(0, 0, 1), new gp_Pnt(1, 0, 1), new gp_Pnt(1, 1, 1),
                                                             new gp_Pnt(0, 1, 1), true).Wire();
        var aFP = new BRepBuilderAPI_FindPlane(aSquare);
        Assert.True(aFP.Found());
        Geom_Plane aPl = aFP.Plane()!;
        Assert.True(aPl.IsFrozen());
        Assert.True(aPl.Pln().Axis().Direction().IsParallel(new gp_Dir(0, 0, 1), 1e-12));
        Assert.Equal(0.0, aPl.Pln().Distance(new gp_Pnt(0.3, 0.7, 1)), 1e-12);

        TopoDS_Wire aSkew = new BRepBuilderAPI_MakePolygon(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0), new gp_Pnt(1, 1, 0.5),
                                                           new gp_Pnt(0, 1, 0), true).Wire();
        Assert.False(new BRepBuilderAPI_FindPlane(aSkew).Found());
        Assert.True(new BRepBuilderAPI_FindPlane(aSkew, 0.5).Found());

        // a conic gives its own plane
        TopoDS_Edge aCirc = new BRepBuilderAPI_MakeEdge(new gp_Circ(new gp_Ax2(new gp_Pnt(0, 0, 5), new gp_Dir(1, 0, 0)), 2)).Edge();
        var aFC = new BRepBuilderAPI_FindPlane(aCirc);
        Assert.True(aFC.Found());
        Assert.True(aFC.Plane()!.Pln().Axis().Direction().IsParallel(new gp_Dir(1, 0, 0), 1e-12));
        Assert.False(new BRepBuilderAPI_FindPlane().Found());
    }
}
