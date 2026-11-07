// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Measurement;
using Kerf.Topology;
using static Kerf.Modeling.Operations;

namespace Kerf.Modeling.Tests.Topology;

/// <summary>
/// <c>Center</c> and <c>Plane(face)</c> take a planar face's centre in closed form (decision 1894). Each figure here is worked out
/// from the geometry by hand, and the integration has to agree to its own tolerance.
/// </summary>
public class PlanarCentreTests
{
    private static Face OneFace(Sketch sketch) => sketch.Faces().Single();

    private static void AssertCentre(Face face, double x, double y, double z = 0, double tolerance = 1e-11)
    {
        Point3D centre = face.Center();
        Assert.True((centre - new Point3D(x, y, z)).Magnitude <= tolerance, $"{centre.X} {centre.Y} {centre.Z} against {x} {y} {z}");
        Assert.True((new Plane(face).Origin - centre).Magnitude == 0, "A plane's origin is the centre");

        Point3D integrated = face.MassProperties(Dimension.Area, new MassOptions { Tolerance = 1e-9 }).CentreOfMass;
        Assert.True((integrated - centre).Magnitude <= 1e-7, $"The integration's {integrated.X} {integrated.Y} {integrated.Z} against {centre.X} {centre.Y} {centre.Z}");
    }

    [Fact]
    public void A_rectangle_at_its_middle() => AssertCentre(OneFace(new Rectangle(30, 10)), 0, 0);

    [Fact]
    public void A_circle_at_its_middle() => AssertCentre(OneFace(new Circle(7.5)), 0, 0);

    [Fact]
    public void A_right_triangle_a_third_of_the_way_along_each_leg()
    {
        // Corners (0, 0), (6, 0), (0, 3): the mean of the corners
        AssertCentre(OneFace(new Polygon(new Point3D(0, 0, 0), new Point3D(6, 0, 0), new Point3D(0, 3, 0))), 2, 1);
    }

    [Fact]
    public void A_plate_with_a_hole_off_centre_moves_away_from_the_hole()
    {
        // 80 x 60 with a hole of radius 10 at (20, 0): moments about the middle are -(pi 100)(20), over the area 4800 - 100 pi
        Face face = OneFace(new Rectangle(80, 60) - new Pos(20, 0.0) * new Circle(10));
        double hole = Math.PI * 100;
        AssertCentre(face, -hole * 20 / (4800 - hole), 0);
    }

    [Fact]
    public void A_ring_at_its_middle() => AssertCentre(OneFace(new Circle(10) - new Circle(4)), 0, 0);

    [Fact]
    public void A_half_disc_at_four_r_over_three_pi()
    {
        // The half of a circle of radius 9 with y > 0
        Face face = OneFace(new Circle(9) - new Pos(0, -10.0) * new Rectangle(30, 20));
        AssertCentre(face, 0, 4 * 9 / (3 * Math.PI));
    }

    [Fact]
    public void An_l_shape_is_the_two_rectangles_by_area()
    {
        // 10 x 10 less the 6 x 6 at the corner toward +x, +y: (100 (0, 0) - 36 (2, 2)) / 64
        Face face = OneFace(new Rectangle(10, 10) - new Pos(2, 2.0) * new Rectangle(6, 6));
        AssertCentre(face, -72.0 / 64, -72.0 / 64);
    }

    [Fact]
    public void Moved_and_turned_it_moves_with_the_face()
    {
        Face face = OneFace(new Rectangle(80, 60) - new Pos(20, 0.0) * new Circle(10));
        Point3D here = face.Center();
        var move = new Location(new Plane(new Point3D(1000, -2000, 500), new Vector3D(1, 1, 0), new Vector3D(0, 1, 1)));
        Face moved = face.Moved(move);
        Point3D expected = (move * new Location(here)).Position;
        Assert.True((moved.Center() - expected).Magnitude <= 1e-10);
        Assert.Equal(here, ((Face)face.AsShape().Reversed()).Center());
    }

    [Fact]
    public void A_face_not_bounded_by_lines_and_circles_integrates_as_before()
    {
        Face ellipse = OneFace(new Ellipse(8, 3));
        Assert.Equal(ellipse.MassProperties(Dimension.Area, new MassOptions { Tolerance = 1e-9 }).CentreOfMass, ellipse.Center());
    }
}
