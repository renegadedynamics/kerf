// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;

namespace Kerf.Kernel.Foundation.Tests;

/// <summary>
/// Bnd_B3's functional forms, spheres, and other boxes under a transformation, against boxes worked by hand (unit 11I).
/// OCCT's GTest (Bnd_B3Tests) stops short of them. Bnd_B3f is this file with its own type: they're one text in OCCT.
/// </summary>
public class Bnd_B3dFormsTests
{
    private const double Tol = 1e-5;

    private static B3d Cube(double theHalf, double theX = 0, double theY = 0, double theZ = 0)
        => new(new Xyz(theX, theY, theZ), new Xyz(theHalf, theHalf, theHalf));

    private static void Corners(B3d theBox, Xyz theMin, Xyz theMax)
    {
        Assert.False(theBox.IsVoid);
        Assert.True(theBox.CornerMin().IsEqual(theMin, Tol));
        Assert.True(theBox.CornerMax().IsEqual(theMax, Tol));
    }

    private static Transform3D Rotation(double theAngle, double theX = 0, double theY = 0)
    {
        Transform3D aTurn = new Transform3D().WithRotation(new Axis3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), theAngle);
        Transform3D aMove = new Transform3D().WithTranslation(new Vector3D(theX, theY, 0));
        return aMove.Multiplied(aTurn);
    }

    private static Transform3D Translation(double theX) => new Transform3D().WithTranslation(new Vector3D(theX, 0, 0));

    [Fact]
    public void The_functional_forms_leave_the_box_and_return_the_change()
    {
        B3d aBox = Cube(1);

        Assert.True(aBox.Cleared().IsVoid);
        Corners(aBox.Added(new Xyz(3, 0, 0)), new Xyz(-1, -1, -1), new Xyz(3, 1, 1));
        Corners(aBox.Added(new Point3D(0, 3, 0)), new Xyz(-1, -1, -1), new Xyz(1, 3, 1));
        Corners(aBox.Added(Cube(1, 2)), new Xyz(-1, -1, -1), new Xyz(3, 1, 1));
        Corners(aBox.Enlarged(0.5), new Xyz(-1.5, -1.5, -1.5), new Xyz(1.5, 1.5, 1.5));
        Corners(aBox.Limited(Cube(1, 1)), new Xyz(0, -1, -1), new Xyz(1, 1, 1));
        Corners(aBox.WithCenter(new Xyz(1, 1, 1)), new Xyz(0, 0, 0), new Xyz(2, 2, 2));
        Corners(aBox.WithCenter(new B3d.Array3(1, 1, 1)), new Xyz(0, 0, 0), new Xyz(2, 2, 2));
        Corners(aBox.WithHSize(new Xyz(2, 2, 2)), new Xyz(-2, -2, -2), new Xyz(2, 2, 2));
        Corners(aBox.WithHSize(new B3d.Array3(2, 2, 2)), new Xyz(-2, -2, -2), new Xyz(2, 2, 2));

        // and the original is as it was
        Corners(aBox, new Xyz(-1, -1, -1), new Xyz(1, 1, 1));
    }

    [Fact]
    public void A_box_limited_by_one_it_misses_stays_as_it_was()
    {
        B3d aBox = Cube(1);
        Corners(aBox.Limited(Cube(1, 5)), new Xyz(-1, -1, -1), new Xyz(1, 1, 1));
    }

    [Fact]
    public void Equality_is_of_centre_and_half_size()
    {
        Assert.Equal(Cube(1), Cube(1));
        Assert.True(Cube(1) == Cube(1));
        Assert.True(Cube(1) != Cube(2));
        Assert.False(Cube(1).Equals((object)Cube(1, 1)));
        Assert.True(Cube(1).Equals((object)Cube(1)));
        Assert.False(Cube(1).Equals("a box"));
        Assert.Equal(Cube(1).GetHashCode(), Cube(1).GetHashCode());
        Assert.NotEmpty(Cube(1).ToString());
    }

    [Fact]
    public void A_void_box_is_out_of_everything()
    {
        B3d aVoid = new();
        Assert.True(aVoid.IsOut(new Xyz(0, 0, 0)));
        Assert.True(aVoid.IsOut(new Xyz(0, 0, 0), 1));
        Assert.True(aVoid.IsOut(Cube(1)));
        Assert.True(aVoid.IsOut(Cube(1), Translation(1)));
        Assert.True(aVoid.Transformed(Translation(1)).IsVoid);
    }

    [Fact]
    public void A_sphere_is_out_by_its_distance_and_a_hollow_one_when_the_box_is_within_it()
    {
        B3d aBox = Cube(1);

        // past a face, an edge and a corner: at distances 1, sqrt(2) and sqrt(3) from the box
        Assert.True(aBox.IsOut(new Xyz(2, 0, 0), 0.9));
        Assert.False(aBox.IsOut(new Xyz(2, 0, 0), 1.1));
        Assert.True(aBox.IsOut(new Xyz(2, 2, 0), 1.4));
        Assert.False(aBox.IsOut(new Xyz(2, 2, 0), 1.5));
        Assert.True(aBox.IsOut(new Xyz(2, 2, 2), 1.7));
        Assert.False(aBox.IsOut(new Xyz(2, 2, 2), 1.8));

        // the box's farthest corner is sqrt(3) from its centre
        Assert.False(aBox.IsOut(new Xyz(0, 0, 0), 10));
        Assert.True(aBox.IsOut(new Xyz(0, 0, 0), 10, true));
        Assert.True(aBox.IsOut(new Xyz(0, 0, 0), 1.8, true));
        Assert.False(aBox.IsOut(new Xyz(0, 0, 0), 1.7, true));
        // a hollow sphere inside the box, and one crossing it from outside
        Assert.False(aBox.IsOut(new Xyz(0, 0, 0), 0.5, true));
        Assert.False(aBox.IsOut(new Xyz(2, 0, 0), 1.1, true));
        // nor does hollowness help a sphere that misses the box
        Assert.True(aBox.IsOut(new Xyz(5, 0, 0), 1, true));
    }

    [Fact]
    public void A_box_under_a_translation_is_out_when_it_has_moved_away()
    {
        B3d aBox = Cube(1);
        Assert.False(aBox.IsOut(Cube(1), Translation(1.5)));
        Assert.True(aBox.IsOut(Cube(1), Translation(2.5)));
    }

    [Fact]
    public void A_box_under_a_rotation_is_out_by_its_own_axes_not_its_bounding_box()
    {
        // a long thin box, half-size (5, 0.1, 0.1), moved to (4, 4, 0)
        B3d aBox = Cube(1);
        B3d aRod = new(new Xyz(0, 0, 0), new Xyz(5, 0.1, 0.1));

        // turned towards the cube, it reaches (0.46, 0.46)
        Assert.False(aBox.IsOut(aRod, Rotation(M_PI / 4, 4, 4)));
        // turned across, its bounding box still overlaps the cube, but x + y stays near 8 and the cube's at most 2
        Assert.True(aRod.Transformed(Rotation(-M_PI / 4, 4, 4)).CornerMin().X() < 1);
        Assert.True(aBox.IsOut(aRod, Rotation(-M_PI / 4, 4, 4)));
        // a quarter turn leaves it axis-aligned, half-size (0.1, 5, 0.1) at x = 3
        Assert.True(aBox.IsOut(aRod, Rotation(M_PI / 2, 3, 0)));
        Assert.False(aBox.IsOut(aRod, Rotation(M_PI / 2, 1, 0)));
    }

    [Fact]
    public void A_box_is_out_along_each_axis_of_the_other()
    {
        // each of the three directions of a box separated only along it, under a turn about each axis
        B3d aBox = Cube(1);
        Transform3D About(Direction3D theAxis, double theAngle, Vector3D theMove)
            => new Transform3D().WithTranslation(theMove).Multiplied(new Transform3D().WithRotation(new Axis3D(new Point3D(0, 0, 0), theAxis), theAngle));

        foreach (Direction3D anAxis in new[] { new Direction3D(1, 0, 0), new Direction3D(0, 1, 0), new Direction3D(0, 0, 1) })
        {
            foreach (Vector3D aMove in new[] { new Vector3D(4, 0, 0), new Vector3D(0, 4, 0), new Vector3D(0, 0, 4) })
            {
                Assert.True(aBox.IsOut(Cube(1), About(anAxis, M_PI / 6, aMove)));
                Assert.False(aBox.IsOut(Cube(1), About(anAxis, M_PI / 6, aMove / 2)));
            }
        }
    }

    [Fact]
    public void A_box_is_in_another_under_a_transformation_by_the_others_axes()
    {
        B3d aBox = Cube(1);

        // translated: [-2, 4] holds [-1, 1], [0, 6] doesn't
        Assert.True(aBox.IsIn(Cube(3), Translation(1)));
        Assert.False(aBox.IsIn(Cube(3), Translation(3)));

        // a corner of the cube is sqrt(2) along the turned axis
        Assert.True(aBox.IsIn(Cube(1.5), Rotation(M_PI / 4)));
        Assert.False(aBox.IsIn(Cube(1.3), Rotation(M_PI / 4)));
        Assert.False(new B3d().IsIn(Cube(1)));
        Assert.False(new B3d().IsIn(Cube(1), Translation(1)));
    }

    [Fact]
    public void A_box_transformed_holds_the_transformed_corners()
    {
        // a half-size (2, 1, 1) box a quarter-turned and moved to (10, 0, 0): half-size (1, 2, 1)
        B3d aBox = new(new Xyz(0, 0, 0), new Xyz(2, 1, 1));
        Corners(aBox.Transformed(Rotation(M_PI / 2, 10, 0)), new Xyz(9, -2, -1), new Xyz(11, 2, 1));
        // a scale doubles it, about the origin
        Transform3D aScale = new Transform3D().WithScale(new Point3D(0, 0, 0), 2);
        Corners(aBox.Transformed(aScale), new Xyz(-4, -2, -2), new Xyz(4, 2, 2));
    }

    [Fact]
    public void A_plane_is_out_when_the_box_lies_to_one_side()
    {
        B3d aBox = Cube(1);
        CoordinateSystem Plane(double theZ, Direction3D theNormal) => new(new Point3D(0, 0, theZ), theNormal);
        Assert.True(aBox.IsOut(Plane(2, new Direction3D(0, 0, 1))));
        Assert.False(aBox.IsOut(Plane(0.5, new Direction3D(0, 0, 1))));
        Assert.False(aBox.IsOut(Plane(0, new Direction3D(1, 1, 1))));
        Assert.True(aBox.IsOut(Plane(10, new Direction3D(1, 1, 1))));
    }

    [Fact]
    public void A_line_and_a_ray_by_where_they_pass()
    {
        B3d aBox = Cube(1);
        Axis3D Along(double theX, double theY, double theZ, Direction3D theDirection) => new(new Point3D(theX, theY, theZ), theDirection);
        Direction3D aDiagonal = new(1, 1, 1);

        Assert.False(aBox.IsOut(Along(-5, -5, -5, aDiagonal)));
        Assert.True(aBox.IsOut(Along(5, -5, -5, aDiagonal)));
        // along each axis, in and out
        Assert.False(aBox.IsOut(Along(-5, 0, 0, new Direction3D(1, 0, 0))));
        Assert.True(aBox.IsOut(Along(-5, 2, 0, new Direction3D(1, 0, 0))));
        Assert.False(aBox.IsOut(Along(0, -5, 0, new Direction3D(0, 1, 0))));
        Assert.True(aBox.IsOut(Along(2, -5, 0, new Direction3D(0, 1, 0))));
        Assert.False(aBox.IsOut(Along(0, 0, -5, new Direction3D(0, 0, 1))));
        Assert.True(aBox.IsOut(Along(0, 2, -5, new Direction3D(0, 0, 1))));
        // a ray pointing away misses, as its line doesn't; pointing at the box it hits
        Assert.False(aBox.IsOut(Along(5, 5, 5, aDiagonal)));
        Assert.True(aBox.IsOut(Along(5, 5, 5, aDiagonal), true));
        Assert.False(aBox.IsOut(Along(-5, -5, -5, aDiagonal), true));
        // over-thickness: a line 1.5 from the centre passes within 0.5 of the box's face at x = 1
        Assert.True(aBox.IsOut(Along(1.5, -5, 0, new Direction3D(0, 1, 0))));
        Assert.False(aBox.IsOut(Along(1.5, -5, 0, new Direction3D(0, 1, 0)), false, 1));
        Assert.True(aBox.IsOut(Along(1.5, -5, 0, new Direction3D(0, 1, 0)), false, 0.1));
    }

    private const double M_PI = Math.PI;
}
