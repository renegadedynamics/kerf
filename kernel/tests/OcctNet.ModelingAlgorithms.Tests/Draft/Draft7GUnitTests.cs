// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using OcctNet.ModelingData.Tests;
using static Kerf.Topology.ShapeType;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Unit 7G's other classes on frozen inputs: BRepAlgo_NormalProjection and BRepOffsetAPI_NormalProjection,
/// BRepOffsetAPI_FindContigousEdges, BRepFill_Draft and BRepOffsetAPI_MakeDraft, BiTgte_Blend and its curves. The
/// inputs are never written, the results are frozen.
/// </summary>
public class Draft7GUnitTests
{
    private static List<Shape> Sub(in Shape theS, ShapeType theType) =>
        BRepOffset_MakeOffsetTests.SubShapes(theS, theType);

    private static Wire Square(double theZ) =>
        new BRepBuilderAPI_MakePolygon(new Point3D(0, 0, theZ), new Point3D(10, 0, theZ), new Point3D(10, 10, theZ),
                                       new Point3D(0, 10, theZ), true).Wire();

    [Fact]
    public void NormalProjection_of_a_circle_on_a_box_gives_edges_on_its_faces()
    {
        Shape aBox = new BRepPrimAPI_MakeBox(new Point3D(-10, -10, -10), 20, 20, 10).Shape();
        Edge aCircle = new BRepBuilderAPI_MakeEdge(new Circle3D(new Frame3D(new Point3D(0, 0, 3), new Direction3D(0, 0, 1)), 4)).Edge();
        ShapeSnapshot aBoxSnap = ShapeSnapshot.Of(aBox), anEdgeSnap = ShapeSnapshot.Of(aCircle);

        var aProj = new BRepOffsetAPI_NormalProjection(aBox);
        aProj.Add(aCircle);
        aProj.Build();
        Assert.True(aProj.IsDone());
        Shape aResult = aProj.Shape();
        aBoxSnap.AssertUnchanged(aBox);
        anEdgeSnap.AssertUnchanged(aCircle);
        Assert.True(BRepOffset_MakeOffsetTests.AllFrozen(aResult));

        List<Shape> anEdges = Sub(aResult, ShapeType.Edge);
        Assert.NotEmpty(anEdges);
        // the circle projects along the normals onto the top face (z = 0) and the bottom one (z = -10)
        Assert.Equal(2, anEdges.Count);
        double aLength = anEdges.Sum(e => { var p = new Properties(); BRepGProp.LinearProperties(e, p); return p.Mass(); });
        Assert.Equal(2 * 2 * Math.PI * 4, aLength, 1.0e-3);
        foreach (Shape anEdge in anEdges)
        {
            Assert.True(aProj.Ancestor(ShapeCasts.Edge(anEdge)).IsSame(aCircle));
            Assert.Contains(Sub(aBox, ShapeType.Face), f => f.IsSame(aProj.Couple(ShapeCasts.Edge(anEdge))));
        }
        // OCCT: the edges the face bounds trim (BRepAlgoAPI_Section) are not recorded as generated
        Assert.Empty(aProj.Generated(aCircle));
        var aWires = new NCollection_List<Shape>();
        Assert.False(aProj.BuildWire(aWires)); // two loops, not one wire
    }

    [Fact]
    public void FindContigousEdges_finds_the_edge_two_separate_faces_share()
    {
        // two squares side by side, made apart: their common side is one contiguous edge; a box's faces share theirs
        Face aF1 = new BRepBuilderAPI_MakeFace(Square(0)).Face();
        Transform3D aT = new();
        aT.SetTranslation(new Vector3D(10, 0, 0));
        Shape aF2 = new BRepBuilderAPI_Transform(new BRepBuilderAPI_MakeFace(Square(0)).Face(), aT, true).Shape();
        var aFinder = new BRepOffsetAPI_FindContigousEdges();
        aFinder.Add(aF1);
        aFinder.Add(aF2);
        aFinder.Perform();
        Assert.Equal(1, aFinder.NbContigousEdges());
        Assert.Equal(2, aFinder.ContigousEdgeCouple(1).Count);
        Assert.Throws<OutOfRangeException>(() => aFinder.ContigousEdge(2));
        Assert.Equal(0, aFinder.NbDegeneratedShapes());

        var aBoxFinder = new BRepOffsetAPI_FindContigousEdges();
        foreach (Shape aF in Sub(new BRepPrimAPI_MakeBox(10, 10, 10).Shape(), ShapeType.Face))
        {
            aBoxFinder.Add(aF);
        }
        aBoxFinder.Perform();
        Assert.Equal(0, aBoxFinder.NbContigousEdges());
    }

    [Fact]
    public void MakeDraft_of_a_square_wire_by_length_is_a_shell_of_four_faces()
    {
        Wire aWire = Square(0);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aWire);
        var aDraft = new BRepOffsetAPI_MakeDraft(aWire, new Direction3D(0, 0, 1), 5.0 * Math.PI / 180.0);
        Assert.False(aDraft.IsDone());
        aDraft.Perform(8.0);
        Assert.True(aDraft.IsDone());
        aSnap.AssertUnchanged(aWire);
        Shape aResult = aDraft.Shape();
        Assert.True(BRepOffset_MakeOffsetTests.AllFrozen(aResult));
        Assert.Equal(4, Sub(aResult, ShapeType.Face).Count);
        Assert.Equal(4, Sub(aDraft.Shell(), ShapeType.Face).Count);
        foreach (Shape anEdge in Sub(aWire, ShapeType.Edge))
        {
            Assert.Single(aDraft.Generated(anEdge));
        }
    }

    [Fact]
    public void MakeDraft_of_a_face_up_to_a_plane_is_sewn_to_the_face()
    {
        Face aFace = new BRepBuilderAPI_MakeFace(Square(0)).Face();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aFace);
        var aDraft = new BRepOffsetAPI_MakeDraft(aFace, new Direction3D(0, 0, 1), 5.0 * Math.PI / 180.0);
        aDraft.Perform(new PlaneSurface(new Point3D(0, 0, 6), new Direction3D(0, 0, 1)));
        Assert.True(aDraft.IsDone());
        aSnap.AssertUnchanged(aFace);
        Assert.True(Sub(aDraft.Shape(), ShapeType.Face).Count >= 5);
        Assert.Throws<ConstructionErrorException>(() => new BRepOffsetAPI_MakeDraft(new BRepPrimAPI_MakeBox(1, 1, 1).Shape(), new Direction3D(0, 0, 1), 0.1));
    }

    [Fact]
    public void BiTgte_Blend_between_two_faces_of_a_frozen_box_keeps_the_box()
    {
        Shape aBox = new BRepPrimAPI_MakeBox(10, 10, 10).Shape();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        List<Shape> aFaces = Sub(aBox, ShapeType.Face);
        var aRoll = new BiTgte_Blend(aBox, -2.0, 1.0e-4, false);
        aRoll.SetFaces(ShapeCasts.Face(aFaces[0]), ShapeCasts.Face(aFaces[2]));
        aRoll.Perform(true);
        Assert.True(aRoll.IsDone());
        aSnap.AssertUnchanged(aBox);
        Assert.True(aRoll.NbSurfaces() > 0);
        Shape aResult = aRoll.Shape();
        Assert.True(BRepOffset_MakeOffsetTests.AllFrozen(aResult));
        Assert.True(BRepOffset_MakeOffsetTests.AllFrozen(aRoll.Face(1)));
        var aCenters = new NCollection_List<Shape>();
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
        Edge anEdge = new BRepBuilderAPI_MakeEdge(new Point3D(0, 0, 0), new Point3D(0, 0, 10)).Edge();
        Edge aCircle = new BRepBuilderAPI_MakeEdge(new Circle3D(new Frame3D(new Point3D(0, 0, 5), new Direction3D(0, 0, 1)), 2)).Edge();
        var aCurve = new BiTgte_CurveOnEdge(aCircle, anEdge);
        Assert.Equal(CurveKind.Circle, aCurve.GetType());
        Assert.Equal(0.0, aCurve.Circle().Radius());
        Assert.True(aCurve.Value(1.0).Distance(new Point3D(0, 0, 5)) < 1.0e-9);
        Assert.Throws<MethodNotImplementedException>(() => aCurve.Continuity());

        Vertex aV = new BRepBuilderAPI_MakeVertex(new Point3D(1, 2, 3)).Vertex();
        var aOnVertex = new BiTgte_CurveOnVertex(aCircle, aV);
        Assert.True(aOnVertex.Value(0.5).IsEqual(new Point3D(1, 2, 3), 0.0));
        Assert.Equal(2 * Math.PI, aOnVertex.LastParameter(), 1.0e-12);
        Assert.Throws<NoSuchObjectException>(() => aOnVertex.Circle());
    }
}
