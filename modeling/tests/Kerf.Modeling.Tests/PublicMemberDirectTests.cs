// Unit 11J's second slice: members of Kerf.Modeling a user meets early that the tests only reached through other members,
// or never named. Each is called directly and its result checked against a worked value.
using Kerf.Geometry;

namespace Kerf.Modeling.Tests;

using Kerf.Modeling;

public class PublicMemberDirectTests
{
    private static Edge Segment(double x1, double y1, double x2, double y2) =>
        Edge.MakeLine(new Point3D(x1, y1, 0), new Point3D(x2, y2, 0));

    // A quarter of the unit circle in XY, from (1, 0) to (0, 1)
    private static Edge Quarter() => Edge.MakeCircle(1, Plane.XY, 0, 90);

    [Fact]
    public void The_tangent_angle_of_a_line_and_a_quarter_circle()
    {
        Assert.Equal(45, Segment(0, 0, 1, 1).TangentAngleAt(), 7);
        Assert.Equal(135, Quarter().TangentAngleAt(0.5), 7);
        Assert.Equal(90, Quarter().TangentAngleAt(0), 7);
        Assert.Equal(180, Quarter().TangentAngleAt(1), 7);
        // the other way round
        Assert.Equal(225, Segment(1, 1, 0, 0).TangentAngleAt(), 7);
    }

    [Fact]
    public void An_infinite_line_is_trimmed_about_its_centre_and_others_stay_as_they_are()
    {
        Edge aSegment = Segment(0, 0, 3, 0);
        Assert.Equal(3, aSegment.TrimInfinite(1).Length, 9);
        Assert.Equal(Math.PI / 2, Quarter().TrimInfinite(1).Length, 9);
    }

    [Fact]
    public void Two_lines_meet_where_they_cross_and_parallel_ones_nowhere()
    {
        IReadOnlyList<Point3D> aCross = Segment(0, 0, 2, 2).FindIntersections(Segment(0, 2, 2, 0));
        Assert.Single(aCross);
        Check.Near(new Point3D(1, 1, 0), aCross[0]);
        Assert.Empty(Segment(0, 0, 2, 0).FindIntersections(Segment(0, 1, 2, 1)));
        // a line through the circle whose quarter it is crosses that quarter once
        IReadOnlyList<Point3D> anArc = Quarter().FindIntersections(Segment(0, 0, 2, 2));
        Assert.Single(anArc);
        Check.Near(new Point3D(Math.Sqrt(0.5), Math.Sqrt(0.5), 0), anArc[0]);
    }

    [Fact]
    public void A_parabola_arc_lies_on_its_parabola_from_its_vertex()
    {
        const double aFocal = 2;
        Edge aParabola = Edge.MakeParabola(aFocal, Plane.XY, 0, 45);
        Check.Near(new Point3D(0, 0, 0), aParabola.StartPoint());
        // y^2 = 4 f x, along its length
        foreach (double t in new[] { 0.25, 0.5, 0.75, 1.0 })
        {
            Point3D aP = aParabola.PositionAt(t);
            Assert.Equal(aP.Y * aP.Y, 4 * aFocal * aP.X, 6);
        }
    }

    [Fact]
    public void A_hyperbola_arc_lies_on_its_hyperbola()
    {
        Edge anAlongX = Edge.MakeHyperbola(3, 2, Plane.XY, -45, 45);
        foreach (double t in new[] { 0.0, 0.25, 0.5, 0.75, 1.0 })
        {
            Point3D aP = anAlongX.PositionAt(t);
            Assert.Equal(1, (aP.X * aP.X / 9) - (aP.Y * aP.Y / 4), 6);
        }
        // with the larger radius along y, it opens along y
        Edge anAlongY = Edge.MakeHyperbola(2, 3, Plane.XY, 45, 135);
        foreach (double t in new[] { 0.0, 0.25, 0.5, 0.75, 1.0 })
        {
            Point3D aP = anAlongY.PositionAt(t);
            Assert.Equal(1, (aP.Y * aP.Y / 9) - (aP.X * aP.X / 4), 6);
        }
    }

    [Fact]
    public void The_edges_and_faces_that_meet_at_an_edge_of_a_box()
    {
        var aBox = new Box(1, 2, 3);
        Edge anEdge = aBox.Edges().First();
        // each end meets two others
        Assert.Equal(4, anEdge.ConnectedEdges(aBox.Shape).Count);
        // and two faces meet along it
        Assert.Equal(2, anEdge.ConnectedFaces(aBox.Shape).Count);
        // an edge of something else meets nothing there
        Assert.Empty(Segment(50, 50, 60, 60).ConnectedFaces(aBox.Shape));
    }

    [Fact]
    public void A_box_seen_from_the_front_and_back_has_opposite_normals()
    {
        Check.Near(new Vector3D(0, -1, 0), Plane.Front.ZDir);
        Check.Near(new Vector3D(0, 1, 0), Plane.Back.ZDir);
        Check.Near(new Vector3D(1, 0, 0), Plane.Front.XDir);
        Check.Near(new Vector3D(-1, 0, 0), Plane.Back.XDir);
    }

    [Fact]
    public void A_plane_gives_the_frame_it_stands_on()
    {
        var aPlane = new Plane(new Point3D(1, 2, 3), new Vector3D(0, 1, 0), new Vector3D(0, 0, 1));
        Frame3D aFrame = aPlane.ToFrame3D();
        Check.Near(new Point3D(1, 2, 3), aFrame.Location);
        Check.Near(new Vector3D(0, 1, 0), new Vector3D(aFrame.XDirection));
        Check.Near(new Vector3D(0, 0, 1), new Vector3D(aFrame.Direction));
    }

    [Fact]
    public void The_location_between_planes_rewrites_coordinates_in_one_as_coordinates_in_the_other()
    {
        var aFrom = new Plane(new Point3D(1, 0, 0), new Vector3D(1, 0, 0), new Vector3D(0, 0, 1));
        var aTo = new Plane(new Point3D(0, 5, 0), new Vector3D(0, 1, 0), new Vector3D(0, 0, 1));
        Location aMove = aFrom.LocationBetween(aTo);
        // (1, 0, 0) on the first plane is the world's (2, 0, 0), which on the second plane is (-5, -2, 0)
        Check.Near(new Point3D(-5, -2, 0), (aMove * new Location(new Point3D(1, 0, 0))).Position);
        // and between a plane and itself nothing changes
        Check.Near(new Point3D(1, 2, 3), (aFrom.LocationBetween(aFrom) * new Location(new Point3D(1, 2, 3))).Position);
    }

    [Fact]
    public void A_rotation_order_knows_whether_it_is_extrinsic()
    {
        RotationOrder anIntrinsic = Intrinsic.XYZ;
        Assert.False(anIntrinsic.IsExtrinsic);
        Assert.Equal(Intrinsic.XYZ, anIntrinsic.Intrinsic);
        Assert.Null(anIntrinsic.Extrinsic);
        RotationOrder anExtrinsic = Extrinsic.ZYX;
        Assert.True(anExtrinsic.IsExtrinsic);
        Assert.Equal(Extrinsic.ZYX, anExtrinsic.Extrinsic);
        Assert.Null(anExtrinsic.Intrinsic);
        Assert.Contains("Extrinsic", anExtrinsic.ToString());
    }

    [Fact]
    public void A_location_list_lists_what_it_was_given_in_order()
    {
        Location[] aLocations = [new(new Point3D(1, 0, 0)), new(new Point3D(2, 0, 0))];
        var aList = new LocationList(aLocations);
        Assert.Equal(2, aList.Count);
        Assert.Equal(2, aList.LocalLocations.Count);
        Assert.Equal(aList.LocalLocations, aList.Locations);
        Check.Near(new Point3D(2, 0, 0), aList[1].Position);
        Assert.Equal(new[] { 1.0, 2.0 }, aList.Select(l => l.Position.X));
    }

    [Fact]
    public void Points_and_vectors_project_on_planes_and_lines()
    {
        var aPlane = new Plane(new Point3D(0, 0, 2), new Vector3D(1, 0, 0), new Vector3D(0, 0, 1));
        Check.Near(new Point3D(3, 4, 2), new Point3D(3, 4, 5).ProjectToPlane(aPlane));
        Check.Near(new Vector3D(3, 3, 0), new Vector3D(1, 5, 7).ProjectToLine(new Vector3D(1, 1, 0)));
    }

    [Fact]
    public void A_position_on_a_curve_can_be_a_parameter_a_point_or_a_vertex()
    {
        Edge aSegment = Segment(0, 0, 4, 0);
        CurvePosition aParameter = 0.25, aPoint = new Point3D(1, 0, 0), aVertex = Vertex.Make(new Point3D(1, 0, 0));
        Check.Near(new Vector3D(1, 0, 0), aSegment.TangentAt(aParameter));
        Check.Near(new Vector3D(1, 0, 0), aSegment.TangentAt(aPoint));
        Check.Near(new Vector3D(1, 0, 0), aSegment.TangentAt(aVertex));
    }

    [Fact]
    public void A_perpendicular_line_is_centred_on_the_curve()
    {
        Edge aPerp = Segment(0, 0, 4, 0).PerpendicularLine(2, 0.5);
        Assert.Equal(2, aPerp.Length, 9);
        Check.Near(new Point3D(2, 0, 0), aPerp.PositionAt(0.5));
        Assert.Equal(0, aPerp.TangentAt(0.5).Dot(new Vector3D(1, 0, 0)), 9);
    }

    [Fact]
    public void A_history_records_what_became_of_a_shape()
    {
        Shape aBefore = new Box(1, 1, 1).Faces().First();
        Shape anAfter = new Box(2, 2, 2).Faces().First();
        Shape aBrought = new Box(3, 3, 3).Faces().First();
        ModelHistory aHistory = new ModelHistory().AddModified(aBefore, anAfter).WithInputs([aBefore], [aBrought]);
        // rebuilt from what was there before: not last
        Assert.False(aHistory.IsLast(anAfter));
        // what was brought in, and what nothing accounts for: last
        Assert.True(aHistory.IsLast(aBrought));
        Assert.True(aHistory.IsLast(new Box(4, 4, 4).Faces().First()));
    }
}
