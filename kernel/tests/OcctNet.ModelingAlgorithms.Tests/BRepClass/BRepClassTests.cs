// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using OcctNet.ModelingData.Tests;
using static Kerf.Engine.Intersection.TwoD.Position2D;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Known answers of BRepClass: the face classifier on a square with a hole (2D and 3D points, reversed and located
/// faces, bounding boxes on or off), the closest edge and position of ON points, a natural face, and BRepClass_Edge.
/// </summary>
public class BRepClassTests
{
    private static Face Holed() => TestShapes.Frozen(ClassificationTestShapes.HoledSquare());

    [Theory]
    [InlineData(2.0, 2.0, In)]
    [InlineData(8.0, 3.0, In)]
    [InlineData(5.0, 5.0, Out)] // in the hole
    [InlineData(12.0, 5.0, Out)]
    [InlineData(-3.0, -7.0, Out)]
    [InlineData(0.0, 5.0, On)] // on the outer boundary
    [InlineData(4.0, 5.0, On)] // on the hole's boundary
    [InlineData(0.0, 0.0, On)] // at a vertex
    [InlineData(6.0, 6.0, On)] // at a vertex of the hole
    public void FaceClassifier_2d_point_in_holed_square(double u, double v, Containment theExpected)
    {
        Face aFace = Holed();
        foreach (bool aUseBox in new[] { false, true })
        {
            var aClassifier = new FaceClassifier(aFace, new Point2D(u, v), 1e-7, aUseBox);
            Assert.Equal(theExpected, aClassifier.State());
            Assert.False(aClassifier.Rejected());
            Assert.False(aClassifier.NoWires());
        }

        // the orientation of the face doesn't change the classification
        var aReversed = new FaceClassifier(ShapeCasts.Face(aFace.Reversed()), new Point2D(u, v), 1e-7);
        Assert.Equal(theExpected, aReversed.State());
    }

    [Fact]
    public void FaceClassifier_tolerance_widens_the_boundary()
    {
        Face aFace = Holed();
        Assert.Equal(In, new FaceClassifier(aFace, new Point2D(1e-3, 5), 1e-7).State());
        Assert.Equal(On, new FaceClassifier(aFace, new Point2D(1e-3, 5), 1e-2).State());
        Assert.Equal(Out, new FaceClassifier(aFace, new Point2D(-1e-3, 5), 1e-7).State());
        Assert.Equal(On, new FaceClassifier(aFace, new Point2D(-1e-3, 5), 1e-2).State());
    }

    [Fact]
    public void FaceClassifier_on_point_gives_its_edge_and_position()
    {
        Face aFace = Holed();

        var aMiddle = new FaceClassifier(aFace, new Point2D(0.0, 5.0), 1e-7);
        Assert.Equal(On, aMiddle.State());
        Assert.Equal(Position2D.Middle, aMiddle.Position());
        ClassifierEdge anEdge = aMiddle.Edge();
        Assert.True(anEdge.Face().IsSame(aFace));
        // the edge from (0, 10) to (0, 0): the point is half way along it
        Exploration.Vertices(anEdge.Edge(), out Vertex aV1, out Vertex aV2);
        Assert.Equal(0.0, Tool.Pnt(aV1).X(), 1e-12);
        Assert.Equal(0.0, Tool.Pnt(aV2).X(), 1e-12);
        Assert.Equal(5.0, aMiddle.EdgeParameter(), 1e-9);

        var aCorner = new FaceClassifier(aFace, new Point2D(10.0, 0.0), 1e-7);
        Assert.Equal(On, aCorner.State());
        Assert.True(aCorner.Position() == Position2D.Head || aCorner.Position() == Position2D.End);
    }

    [Theory]
    [InlineData(2.0, 2.0, 3.0, In)] // projected into the face
    [InlineData(2.0, 2.0, -0.5, In)]
    [InlineData(5.0, 5.0, 1.0, Out)] // above the hole
    [InlineData(0.0, 7.0, 2.0, On)] // above the boundary
    public void FaceClassifier_3d_point_classifies_its_projection(double x, double y, double z, Containment theExpected)
    {
        var aClassifier = new FaceClassifier(Holed(), new Point3D(x, y, z), 1e-7);
        Assert.Equal(theExpected, aClassifier.State());
    }

    [Fact]
    public void FaceClassifier_on_a_located_face()
    {
        // the location moves the face in 3D; 2D points keep their meaning, 3D points move with it
        Face aFace = ShapeCasts.Face(ClassificationTestShapes.Translated(Holed(), new Vector3D(0, 0, 100)));
        Assert.Equal(In, new FaceClassifier(aFace, new Point2D(2, 2), 1e-7).State());
        Assert.Equal(In, new FaceClassifier(aFace, new Point3D(2, 2, 100), 1e-7).State());
        Assert.Equal(Out, new FaceClassifier(aFace, new Point3D(5, 5, 100), 1e-7).State());
    }

    [Fact]
    public void FaceClassifier_natural_face_has_no_wires_and_is_in()
    {
        Face aFace = TestShapes.Frozen(BRepGPropTestShapes.NaturalFace(
            new SphericalSurface(new CoordinateSystem(new Point3D(), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 2.0)));
        var aClassifier = new FaceClassifier(aFace, new Point2D(1.0, 0.5), 1e-7);
        Assert.True(aClassifier.NoWires());
        Assert.Equal(In, aClassifier.State());
    }

    [Fact]
    public void FaceClassifier_with_an_explorer_matches_the_face_constructor()
    {
        Face aFace = Holed();
        var anExplorer = new FaceExplorer(aFace);
        var aClassifier = new FaceClassifier(anExplorer, new Point2D(2, 2), 1e-7);
        Assert.Equal(In, aClassifier.State());
        aClassifier.Perform(anExplorer, new Point2D(5, 5), 1e-7);
        Assert.Equal(Out, aClassifier.State());
    }

    [Fact]
    public void FaceExplorer_segments_aim_at_the_edges()
    {
        var anExplorer = new FaceExplorer(Holed());
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
        var anExplorer = new FaceExplorer(Holed());
        int aWires = 0, anEdges = 0;
        var anEdge = new ClassifierEdge();
        for (anExplorer.InitWires(); anExplorer.MoreWires(); anExplorer.NextWire())
        {
            aWires++;
            for (anExplorer.InitEdges(); anExplorer.MoreEdges(); anExplorer.NextEdge())
            {
                anExplorer.CurrentEdge(ref anEdge, out Orientation anOr);
                Assert.Equal(Forward, anOr);
                Assert.False(anEdge.NextEdge().IsNull); // each vertex of a square has two edges
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
        var anEdge = new ClassifierEdge();
        Assert.True(anEdge.Edge().IsNull);
        Assert.True(global::Kerf.Engine.Foundation.Precision.IsInfinite(anEdge.MaxTolerance()));
        Assert.False(anEdge.UseBndBox());

        ClassifierEdge aChanged = anEdge.WithMaxTolerance(0.5).WithUseBndBox(true);
        Assert.Equal(0.5, aChanged.MaxTolerance());
        Assert.True(aChanged.UseBndBox());
        Assert.True(global::Kerf.Engine.Foundation.Precision.IsInfinite(anEdge.MaxTolerance())); // the original is unchanged

        // default(T) is zeros (a zero maximum tolerance)
        Assert.Equal(0.0, default(ClassifierEdge).MaxTolerance());
    }

    [Fact]
    public void Intersector_finds_the_crossing_of_a_segment_and_an_edge()
    {
        Face aFace = Holed();
        var anExp = new Explorer(aFace, ShapeType.Edge);
        Edge aFirst = ShapeCasts.Edge(anExp.Current()); // (0, 0) -> (10, 0)
        var anEdge = new ClassifierEdge(aFirst, aFace);
        var anInter = new Intersector();

        // from (3, -2) upwards: crosses the edge at (3, 0), 2 along the line, 3 along the edge
        anInter.Perform(new Line2D(new Point2D(3, -2), new Direction2D(0, 1)), 10.0, 1e-7, anEdge);
        Assert.True(anInter.IsDone);
        Assert.Equal(1, anInter.PointCount);
        Assert.Equal(2.0, anInter.Point(1).ParamOnFirst(), 1e-9);
        Assert.Equal(3.0, anInter.Point(1).ParamOnSecond(), 1e-9);

        // too short a segment
        anInter.Perform(new Line2D(new Point2D(3, -2), new Direction2D(0, 1)), 1.0, 1e-7, anEdge);
        Assert.Equal(0, anInter.PointCount);

        // the origin on the edge: one ON point at parameter 0 on the line
        anInter.Perform(new Line2D(new Point2D(3, 0), new Direction2D(0, 1)), 10.0, 1e-7, anEdge);
        Assert.Equal(1, anInter.PointCount);
        Assert.Equal(0.0, anInter.Point(1).ParamOnFirst());
        Assert.Equal(Position2D.Head, anInter.Point(1).TransitionOfFirst().PositionOnCurve());

        // the local geometry of a line: its direction, no curvature
        anInter.LocalGeometry(anEdge, 3.0, out Direction2D aT, out Direction2D aN, out double aC);
        Assert.Equal(1.0, aT.X(), 1e-12);
        Assert.Equal(0.0, aC);
        Assert.Equal(-1.0, aN.Y(), 1e-12);
    }

    [Fact]
    public void FacePassiveClassifier_compares_edges_given_one_by_one()
    {
        Face aFace = Holed();
        var aClassifier = new FacePassiveClassifier();
        // from (2, 2) towards +X: the first edge met is x = 10, going out
        aClassifier.Reset(new Line2D(new Point2D(2, 2), new Direction2D(1, 0)), RealLast(), 1e-7);
        for (var anExp = new Explorer(aFace, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Edge anE = ShapeCasts.Edge(anExp.Current());
            aClassifier.Compare(new ClassifierEdge(anE, aFace), anE.Orientation());
        }

        Assert.Equal(In, aClassifier.State());
        Assert.Equal(8.0, aClassifier.Parameter(), 1e-9);
    }
}
