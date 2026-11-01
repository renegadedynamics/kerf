// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.ModelingData.Tests;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Unit 7G's other classes on frozen inputs: BRepAlgo_NormalProjection and BRepOffsetAPI_NormalProjection,
/// BRepOffsetAPI_FindContigousEdges, BRepFill_Draft and BRepOffsetAPI_MakeDraft, BiTgte_Blend and its curves. The
/// inputs are never written, the results are frozen.
/// </summary>
public class Draft7GUnitTests
{
    private static List<TopoDS_Shape> Sub(in TopoDS_Shape theS, TopAbs_ShapeEnum theType) =>
        BRepOffset_MakeOffsetTests.SubShapes(theS, theType);

    private static TopoDS_Wire Square(double theZ) =>
        new BRepBuilderAPI_MakePolygon(new gp_Pnt(0, 0, theZ), new gp_Pnt(10, 0, theZ), new gp_Pnt(10, 10, theZ),
                                       new gp_Pnt(0, 10, theZ), true).Wire();

    [Fact]
    public void NormalProjection_of_a_circle_on_a_box_gives_edges_on_its_faces()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(new gp_Pnt(-10, -10, -10), 20, 20, 10).Shape();
        TopoDS_Edge aCircle = new BRepBuilderAPI_MakeEdge(new gp_Circ(new gp_Ax2(new gp_Pnt(0, 0, 3), new gp_Dir(0, 0, 1)), 4)).Edge();
        ShapeSnapshot aBoxSnap = ShapeSnapshot.Of(aBox), anEdgeSnap = ShapeSnapshot.Of(aCircle);

        var aProj = new BRepOffsetAPI_NormalProjection(aBox);
        aProj.Add(aCircle);
        aProj.Build();
        Assert.True(aProj.IsDone());
        TopoDS_Shape aResult = aProj.Shape();
        aBoxSnap.AssertUnchanged(aBox);
        anEdgeSnap.AssertUnchanged(aCircle);
        Assert.True(BRepOffset_MakeOffsetTests.AllFrozen(aResult));

        List<TopoDS_Shape> anEdges = Sub(aResult, TopAbs_EDGE);
        Assert.NotEmpty(anEdges);
        // the circle projects along the normals onto the top face (z = 0) and the bottom one (z = -10)
        Assert.Equal(2, anEdges.Count);
        double aLength = anEdges.Sum(e => { var p = new GProp_GProps(); BRepGProp.LinearProperties(e, p); return p.Mass(); });
        Assert.Equal(2 * 2 * Math.PI * 4, aLength, 1.0e-3);
        foreach (TopoDS_Shape anEdge in anEdges)
        {
            Assert.True(aProj.Ancestor(TopoDS.Edge(anEdge)).IsSame(aCircle));
            Assert.Contains(Sub(aBox, TopAbs_FACE), f => f.IsSame(aProj.Couple(TopoDS.Edge(anEdge))));
        }
        // OCCT: the edges the face bounds trim (BRepAlgoAPI_Section) are not recorded as generated
        Assert.Empty(aProj.Generated(aCircle));
        var aWires = new NCollection_List<TopoDS_Shape>();
        Assert.False(aProj.BuildWire(aWires)); // two loops, not one wire
    }

    [Fact]
    public void FindContigousEdges_finds_the_edge_two_separate_faces_share()
    {
        // two squares side by side, made apart: their common side is one contiguous edge; a box's faces share theirs
        TopoDS_Face aF1 = new BRepBuilderAPI_MakeFace(Square(0)).Face();
        gp_Trsf aT = new();
        aT.SetTranslation(new gp_Vec(10, 0, 0));
        TopoDS_Shape aF2 = new BRepBuilderAPI_Transform(new BRepBuilderAPI_MakeFace(Square(0)).Face(), aT, true).Shape();
        var aFinder = new BRepOffsetAPI_FindContigousEdges();
        aFinder.Add(aF1);
        aFinder.Add(aF2);
        aFinder.Perform();
        Assert.Equal(1, aFinder.NbContigousEdges());
        Assert.Equal(2, aFinder.ContigousEdgeCouple(1).Count);
        Assert.Throws<Standard_OutOfRange>(() => aFinder.ContigousEdge(2));
        Assert.Equal(0, aFinder.NbDegeneratedShapes());

        var aBoxFinder = new BRepOffsetAPI_FindContigousEdges();
        foreach (TopoDS_Shape aF in Sub(new BRepPrimAPI_MakeBox(10, 10, 10).Shape(), TopAbs_FACE))
        {
            aBoxFinder.Add(aF);
        }
        aBoxFinder.Perform();
        Assert.Equal(0, aBoxFinder.NbContigousEdges());
    }

    [Fact]
    public void MakeDraft_of_a_square_wire_by_length_is_a_shell_of_four_faces()
    {
        TopoDS_Wire aWire = Square(0);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aWire);
        var aDraft = new BRepOffsetAPI_MakeDraft(aWire, new gp_Dir(0, 0, 1), 5.0 * Math.PI / 180.0);
        Assert.False(aDraft.IsDone());
        aDraft.Perform(8.0);
        Assert.True(aDraft.IsDone());
        aSnap.AssertUnchanged(aWire);
        TopoDS_Shape aResult = aDraft.Shape();
        Assert.True(BRepOffset_MakeOffsetTests.AllFrozen(aResult));
        Assert.Equal(4, Sub(aResult, TopAbs_FACE).Count);
        Assert.Equal(4, Sub(aDraft.Shell(), TopAbs_FACE).Count);
        foreach (TopoDS_Shape anEdge in Sub(aWire, TopAbs_EDGE))
        {
            Assert.Single(aDraft.Generated(anEdge));
        }
    }

    [Fact]
    public void MakeDraft_of_a_face_up_to_a_plane_is_sewn_to_the_face()
    {
        TopoDS_Face aFace = new BRepBuilderAPI_MakeFace(Square(0)).Face();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aFace);
        var aDraft = new BRepOffsetAPI_MakeDraft(aFace, new gp_Dir(0, 0, 1), 5.0 * Math.PI / 180.0);
        aDraft.Perform(new Geom_Plane(new gp_Pnt(0, 0, 6), new gp_Dir(0, 0, 1)));
        Assert.True(aDraft.IsDone());
        aSnap.AssertUnchanged(aFace);
        Assert.True(Sub(aDraft.Shape(), TopAbs_FACE).Count >= 5);
        Assert.Throws<Standard_ConstructionError>(() => new BRepOffsetAPI_MakeDraft(new BRepPrimAPI_MakeBox(1, 1, 1).Shape(), new gp_Dir(0, 0, 1), 0.1));
    }

    [Fact]
    public void BiTgte_Blend_between_two_faces_of_a_frozen_box_keeps_the_box()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(10, 10, 10).Shape();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        List<TopoDS_Shape> aFaces = Sub(aBox, TopAbs_FACE);
        var aRoll = new BiTgte_Blend(aBox, -2.0, 1.0e-4, false);
        aRoll.SetFaces(TopoDS.Face(aFaces[0]), TopoDS.Face(aFaces[2]));
        aRoll.Perform(true);
        Assert.True(aRoll.IsDone());
        aSnap.AssertUnchanged(aBox);
        Assert.True(aRoll.NbSurfaces() > 0);
        TopoDS_Shape aResult = aRoll.Shape();
        Assert.True(BRepOffset_MakeOffsetTests.AllFrozen(aResult));
        Assert.True(BRepOffset_MakeOffsetTests.AllFrozen(aRoll.Face(1)));
        var aCenters = new NCollection_List<TopoDS_Shape>();
        aRoll.CenterLines(aCenters);
        Assert.Equal(aRoll.NbSurfaces(), aCenters.Extent());
        Assert.True(aRoll.NbBranches() >= 1);
        aRoll.IndicesOfBranche(1, out int aFrom, out int aTo);
        Assert.True(aFrom >= 1 && aTo >= aFrom);
        Assert.Null(aRoll.PCurveOnFace1(1));
    }

    [Fact]
    public void BiTgte_curves_evaluate_their_points()
    {
        TopoDS_Edge anEdge = new BRepBuilderAPI_MakeEdge(new gp_Pnt(0, 0, 0), new gp_Pnt(0, 0, 10)).Edge();
        TopoDS_Edge aCircle = new BRepBuilderAPI_MakeEdge(new gp_Circ(new gp_Ax2(new gp_Pnt(0, 0, 5), new gp_Dir(0, 0, 1)), 2)).Edge();
        var aCurve = new BiTgte_CurveOnEdge(aCircle, anEdge);
        Assert.Equal(GeomAbs_CurveType.GeomAbs_Circle, aCurve.GetType());
        Assert.Equal(0.0, aCurve.Circle().Radius());
        Assert.True(aCurve.Value(1.0).Distance(new gp_Pnt(0, 0, 5)) < 1.0e-9);
        Assert.Throws<Standard_NotImplemented>(() => aCurve.Continuity());

        TopoDS_Vertex aV = new BRepBuilderAPI_MakeVertex(new gp_Pnt(1, 2, 3)).Vertex();
        var aOnVertex = new BiTgte_CurveOnVertex(aCircle, aV);
        Assert.True(aOnVertex.Value(0.5).IsEqual(new gp_Pnt(1, 2, 3), 0.0));
        Assert.Equal(2 * Math.PI, aOnVertex.LastParameter(), 1.0e-12);
        Assert.Throws<Standard_NoSuchObject>(() => aOnVertex.Circle());
    }
}
