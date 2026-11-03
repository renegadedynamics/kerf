// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using OcctNet.ModelingData.Tests;
using static Occt.IntRes2d_Position;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_State;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Known answers of BRepClass: the face classifier on a square with a hole (2D and 3D points, reversed and located
/// faces, bounding boxes on or off), the closest edge and position of ON points, a natural face, and BRepClass_Edge.
/// </summary>
public class BRepClassTests
{
    private static TopoDS_Face Holed() => TestShapes.Frozen(ClassificationTestShapes.HoledSquare());

    [Theory]
    [InlineData(2.0, 2.0, TopAbs_IN)]
    [InlineData(8.0, 3.0, TopAbs_IN)]
    [InlineData(5.0, 5.0, TopAbs_OUT)] // in the hole
    [InlineData(12.0, 5.0, TopAbs_OUT)]
    [InlineData(-3.0, -7.0, TopAbs_OUT)]
    [InlineData(0.0, 5.0, TopAbs_ON)] // on the outer boundary
    [InlineData(4.0, 5.0, TopAbs_ON)] // on the hole's boundary
    [InlineData(0.0, 0.0, TopAbs_ON)] // at a vertex
    [InlineData(6.0, 6.0, TopAbs_ON)] // at a vertex of the hole
    public void FaceClassifier_2d_point_in_holed_square(double u, double v, TopAbs_State theExpected)
    {
        TopoDS_Face aFace = Holed();
        foreach (bool aUseBox in new[] { false, true })
        {
            var aClassifier = new BRepClass_FaceClassifier(aFace, new Point2D(u, v), 1e-7, aUseBox);
            Assert.Equal(theExpected, aClassifier.State());
            Assert.False(aClassifier.Rejected());
            Assert.False(aClassifier.NoWires());
        }

        // the orientation of the face doesn't change the classification
        var aReversed = new BRepClass_FaceClassifier(TopoDS.Face(aFace.Reversed()), new Point2D(u, v), 1e-7);
        Assert.Equal(theExpected, aReversed.State());
    }

    [Fact]
    public void FaceClassifier_tolerance_widens_the_boundary()
    {
        TopoDS_Face aFace = Holed();
        Assert.Equal(TopAbs_IN, new BRepClass_FaceClassifier(aFace, new Point2D(1e-3, 5), 1e-7).State());
        Assert.Equal(TopAbs_ON, new BRepClass_FaceClassifier(aFace, new Point2D(1e-3, 5), 1e-2).State());
        Assert.Equal(TopAbs_OUT, new BRepClass_FaceClassifier(aFace, new Point2D(-1e-3, 5), 1e-7).State());
        Assert.Equal(TopAbs_ON, new BRepClass_FaceClassifier(aFace, new Point2D(-1e-3, 5), 1e-2).State());
    }

    [Fact]
    public void FaceClassifier_on_point_gives_its_edge_and_position()
    {
        TopoDS_Face aFace = Holed();

        var aMiddle = new BRepClass_FaceClassifier(aFace, new Point2D(0.0, 5.0), 1e-7);
        Assert.Equal(TopAbs_ON, aMiddle.State());
        Assert.Equal(IntRes2d_Middle, aMiddle.Position());
        BRepClass_Edge anEdge = aMiddle.Edge();
        Assert.True(anEdge.Face().IsSame(aFace));
        // the edge from (0, 10) to (0, 0): the point is half way along it
        TopExp.Vertices(anEdge.Edge(), out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
        Assert.Equal(0.0, BRep_Tool.Pnt(aV1).X(), 1e-12);
        Assert.Equal(0.0, BRep_Tool.Pnt(aV2).X(), 1e-12);
        Assert.Equal(5.0, aMiddle.EdgeParameter(), 1e-9);

        var aCorner = new BRepClass_FaceClassifier(aFace, new Point2D(10.0, 0.0), 1e-7);
        Assert.Equal(TopAbs_ON, aCorner.State());
        Assert.True(aCorner.Position() == IntRes2d_Head || aCorner.Position() == IntRes2d_End);
    }

    [Theory]
    [InlineData(2.0, 2.0, 3.0, TopAbs_IN)] // projected into the face
    [InlineData(2.0, 2.0, -0.5, TopAbs_IN)]
    [InlineData(5.0, 5.0, 1.0, TopAbs_OUT)] // above the hole
    [InlineData(0.0, 7.0, 2.0, TopAbs_ON)] // above the boundary
    public void FaceClassifier_3d_point_classifies_its_projection(double x, double y, double z, TopAbs_State theExpected)
    {
        var aClassifier = new BRepClass_FaceClassifier(Holed(), new Point3D(x, y, z), 1e-7);
        Assert.Equal(theExpected, aClassifier.State());
    }

    [Fact]
    public void FaceClassifier_on_a_located_face()
    {
        // the location moves the face in 3D; 2D points keep their meaning, 3D points move with it
        TopoDS_Face aFace = TopoDS.Face(ClassificationTestShapes.Translated(Holed(), new Vector3D(0, 0, 100)));
        Assert.Equal(TopAbs_IN, new BRepClass_FaceClassifier(aFace, new Point2D(2, 2), 1e-7).State());
        Assert.Equal(TopAbs_IN, new BRepClass_FaceClassifier(aFace, new Point3D(2, 2, 100), 1e-7).State());
        Assert.Equal(TopAbs_OUT, new BRepClass_FaceClassifier(aFace, new Point3D(5, 5, 100), 1e-7).State());
    }

    [Fact]
    public void FaceClassifier_natural_face_has_no_wires_and_is_in()
    {
        TopoDS_Face aFace = TestShapes.Frozen(BRepGPropTestShapes.NaturalFace(
            new Geom_SphericalSurface(new CoordinateSystem(new Point3D(), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 2.0)));
        var aClassifier = new BRepClass_FaceClassifier(aFace, new Point2D(1.0, 0.5), 1e-7);
        Assert.True(aClassifier.NoWires());
        Assert.Equal(TopAbs_IN, aClassifier.State());
    }

    [Fact]
    public void FaceClassifier_with_an_explorer_matches_the_face_constructor()
    {
        TopoDS_Face aFace = Holed();
        var anExplorer = new BRepClass_FaceExplorer(aFace);
        var aClassifier = new BRepClass_FaceClassifier(anExplorer, new Point2D(2, 2), 1e-7);
        Assert.Equal(TopAbs_IN, aClassifier.State());
        aClassifier.Perform(anExplorer, new Point2D(5, 5), 1e-7);
        Assert.Equal(TopAbs_OUT, aClassifier.State());
    }

    [Fact]
    public void FaceExplorer_segments_aim_at_the_edges()
    {
        var anExplorer = new BRepClass_FaceExplorer(Holed());
        var aP = new Point2D(2, 2);
        Assert.True(anExplorer.Segment(aP, out Line2D aL, out double aPar));
        Assert.True(aPar > 0);
        Assert.True(aL.Location().IsEqual(aP, 1e-12));

        // each other segment is different, until the edges are used up
        int aCount = 1;
        while (anExplorer.OtherSegment(aP, out Line2D aL2, out double aPar2))
        {
            Assert.True(aPar2 > 0);
            aCount++;
            Assert.True(aCount < 100);
        }

        Assert.True(aCount >= 8);

        // a point far away is brought back towards the face's box
        var aFar = new Point2D(1e300, 1e300);
        Assert.False(anExplorer.CheckPoint(ref aFar));
        Assert.True(Math.Abs(aFar.X()) < 1e300);
    }

    [Fact]
    public void FaceExplorer_explores_wires_and_edges()
    {
        var anExplorer = new BRepClass_FaceExplorer(Holed());
        int aWires = 0, anEdges = 0;
        var anEdge = new BRepClass_Edge();
        for (anExplorer.InitWires(); anExplorer.MoreWires(); anExplorer.NextWire())
        {
            aWires++;
            for (anExplorer.InitEdges(); anExplorer.MoreEdges(); anExplorer.NextEdge())
            {
                anExplorer.CurrentEdge(ref anEdge, out TopAbs_Orientation anOr);
                Assert.Equal(TopAbs_FORWARD, anOr);
                Assert.False(anEdge.NextEdge().IsNull()); // each vertex of a square has two edges
                Assert.Equal(0.1, anEdge.MaxTolerance());
                anEdges++;
            }
        }

        Assert.Equal(2, aWires);
        Assert.Equal(8, anEdges);
    }

    [Fact]
    public void Edge_value_and_twins()
    {
        var anEdge = new BRepClass_Edge();
        Assert.True(anEdge.Edge().IsNull());
        Assert.True(global::Kerf.Engine.Foundation.Precision.IsInfinite(anEdge.MaxTolerance()));
        Assert.False(anEdge.UseBndBox());

        BRepClass_Edge aChanged = anEdge.WithMaxTolerance(0.5).WithUseBndBox(true);
        Assert.Equal(0.5, aChanged.MaxTolerance());
        Assert.True(aChanged.UseBndBox());
        Assert.True(global::Kerf.Engine.Foundation.Precision.IsInfinite(anEdge.MaxTolerance())); // the original is unchanged

        // default(T) is zeros (a zero maximum tolerance)
        Assert.Equal(0.0, default(BRepClass_Edge).MaxTolerance());
    }

    [Fact]
    public void Intersector_finds_the_crossing_of_a_segment_and_an_edge()
    {
        TopoDS_Face aFace = Holed();
        var anExp = new TopExp_Explorer(aFace, TopAbs_ShapeEnum.TopAbs_EDGE);
        TopoDS_Edge aFirst = TopoDS.Edge(anExp.Current()); // (0, 0) -> (10, 0)
        var anEdge = new BRepClass_Edge(aFirst, aFace);
        var anInter = new BRepClass_Intersector();

        // from (3, -2) upwards: crosses the edge at (3, 0), 2 along the line, 3 along the edge
        anInter.Perform(new Line2D(new Point2D(3, -2), new Direction2D(0, 1)), 10.0, 1e-7, anEdge);
        Assert.True(anInter.IsDone());
        Assert.Equal(1, anInter.NbPoints());
        Assert.Equal(2.0, anInter.Point(1).ParamOnFirst(), 1e-9);
        Assert.Equal(3.0, anInter.Point(1).ParamOnSecond(), 1e-9);

        // too short a segment
        anInter.Perform(new Line2D(new Point2D(3, -2), new Direction2D(0, 1)), 1.0, 1e-7, anEdge);
        Assert.Equal(0, anInter.NbPoints());

        // the origin on the edge: one ON point at parameter 0 on the line
        anInter.Perform(new Line2D(new Point2D(3, 0), new Direction2D(0, 1)), 10.0, 1e-7, anEdge);
        Assert.Equal(1, anInter.NbPoints());
        Assert.Equal(0.0, anInter.Point(1).ParamOnFirst());
        Assert.Equal(IntRes2d_Head, anInter.Point(1).TransitionOfFirst().PositionOnCurve());

        // the local geometry of a line: its direction, no curvature
        anInter.LocalGeometry(anEdge, 3.0, out Direction2D aT, out Direction2D aN, out double aC);
        Assert.Equal(1.0, aT.X(), 1e-12);
        Assert.Equal(0.0, aC);
        Assert.Equal(-1.0, aN.Y(), 1e-12);
    }

    [Fact]
    public void FacePassiveClassifier_compares_edges_given_one_by_one()
    {
        TopoDS_Face aFace = Holed();
        var aClassifier = new BRepClass_FacePassiveClassifier();
        // from (2, 2) towards +X: the first edge met is x = 10, going out
        aClassifier.Reset(new Line2D(new Point2D(2, 2), new Direction2D(1, 0)), RealLast(), 1e-7);
        for (var anExp = new TopExp_Explorer(aFace, TopAbs_ShapeEnum.TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            TopoDS_Edge anE = TopoDS.Edge(anExp.Current());
            aClassifier.Compare(new BRepClass_Edge(anE, aFace), anE.Orientation());
        }

        Assert.Equal(TopAbs_IN, aClassifier.State());
        Assert.Equal(8.0, aClassifier.Parameter(), 1e-9);
    }
}
