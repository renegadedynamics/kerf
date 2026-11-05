// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;

namespace Kerf.Kernel.Foundation.Tests;

/// <summary>
/// Bnd_B2's functional forms, circles, segments, and other boxes under a transformation, against boxes worked by hand
/// (unit 11I). OCCT's GTest (Bnd_B2Tests) stops short of them. This is Bnd_B2dFormsTests with its own type: they're one text in OCCT.
/// </summary>
public class Bnd_B2fFormsTests
{
    private const double Tol = 1e-5;

    private static B2f Square(double theHalf, double theX = 0, double theY = 0)
        => new(new Xy(theX, theY), new Xy(theHalf, theHalf));

    private static void Corners(B2f theBox, Xy theMin, Xy theMax)
    {
        Assert.False(theBox.IsVoid);
        Assert.True(theBox.CornerMin().IsEqual(theMin, Tol));
        Assert.True(theBox.CornerMax().IsEqual(theMax, Tol));
    }

    private static Transform2D Rotation(double theAngle, double theX = 0, double theY = 0)
    {
        Transform2D aTurn = new Transform2D().WithRotation(new Point2D(0, 0), theAngle);
        Transform2D aMove = new Transform2D().WithTranslation(new Vector2D(theX, theY));
        return aMove.Multiplied(aTurn);
    }

    private static Transform2D Translation(double theX) => new Transform2D().WithTranslation(new Vector2D(theX, 0));

    [Fact]
    public void The_functional_forms_leave_the_box_and_return_the_change()
    {
        B2f aBox = Square(1);

        Assert.True(aBox.Cleared().IsVoid);
        Corners(aBox.Added(new Xy(3, 0)), new Xy(-1, -1), new Xy(3, 1));
        Corners(aBox.Added(new Point2D(0, 3)), new Xy(-1, -1), new Xy(1, 3));
        Corners(aBox.Added(Square(1, 2)), new Xy(-1, -1), new Xy(3, 1));
        Corners(aBox.Enlarged(0.5), new Xy(-1.5, -1.5), new Xy(1.5, 1.5));
        Corners(aBox.Limited(Square(1, 1)), new Xy(0, -1), new Xy(1, 1));
        Corners(aBox.WithCenter(new Xy(1, 1)), new Xy(0, 0), new Xy(2, 2));
        Corners(aBox.WithCenter(new B2f.Array2(1, 1)), new Xy(0, 0), new Xy(2, 2));
        Corners(aBox.WithHSize(new Xy(2, 2)), new Xy(-2, -2), new Xy(2, 2));
        Corners(aBox.WithHSize(new B2f.Array2(2, 2)), new Xy(-2, -2), new Xy(2, 2));

        // and the original is as it was
        Corners(aBox, new Xy(-1, -1), new Xy(1, 1));
    }

    [Fact]
    public void A_box_limited_by_one_it_misses_stays_as_it_was()
        => Corners(Square(1).Limited(Square(1, 5)), new Xy(-1, -1), new Xy(1, 1));

    [Fact]
    public void Equality_is_of_centre_and_half_size()
    {
        Assert.Equal(Square(1), Square(1));
        Assert.True(Square(1) == Square(1));
        Assert.True(Square(1) != Square(2));
        Assert.False(Square(1).Equals((object)Square(1, 1)));
        Assert.True(Square(1).Equals((object)Square(1)));
        Assert.False(Square(1).Equals("a box"));
        Assert.Equal(Square(1).GetHashCode(), Square(1).GetHashCode());
        Assert.NotEmpty(Square(1).ToString());
    }

    [Fact]
    public void A_void_box_is_out_of_everything()
    {
        B2f aVoid = new();
        Assert.True(aVoid.IsOut(new Xy(0, 0)));
        Assert.True(aVoid.IsOut(new Xy(0, 0), 1));
        Assert.True(aVoid.IsOut(Square(1)));
        Assert.True(aVoid.IsOut(Square(1), Translation(1)));
        Assert.True(aVoid.Transformed(Translation(1)).IsVoid);
    }

    [Fact]
    public void A_circle_is_out_by_its_distance_and_a_hollow_one_when_the_box_is_within_it()
    {
        B2f aBox = Square(1);

        // past a side and a corner: at distances 1 and sqrt(2) from the box
        Assert.True(aBox.IsOut(new Xy(2, 0), 0.9));
        Assert.False(aBox.IsOut(new Xy(2, 0), 1.1));
        Assert.True(aBox.IsOut(new Xy(2, 2), 1.4));
        Assert.False(aBox.IsOut(new Xy(2, 2), 1.5));

        // the box's farthest corner is sqrt(2) from its centre
        Assert.False(aBox.IsOut(new Xy(0, 0), 10));
        Assert.True(aBox.IsOut(new Xy(0, 0), 10, true));
        Assert.True(aBox.IsOut(new Xy(0, 0), 1.5, true));
        Assert.False(aBox.IsOut(new Xy(0, 0), 1.3, true));
        Assert.False(aBox.IsOut(new Xy(0, 0), 0.5, true));
        Assert.True(aBox.IsOut(new Xy(5, 0), 1, true));
    }

    [Fact]
    public void A_box_under_a_transformation_is_out_by_its_own_axes()
    {
        B2f aBox = Square(1);
        Assert.False(aBox.IsOut(Square(1), Translation(1.5)));
        Assert.True(aBox.IsOut(Square(1), Translation(2.5)));

        // a long thin box, half-size (5, 0.1), moved to (4, 4): towards the square it reaches (0.46, 0.46)
        B2f aRod = new(new Xy(0, 0), new Xy(5, 0.1));
        Assert.False(aBox.IsOut(aRod, Rotation(M_PI / 4, 4, 4)));
        // across, its bounding box overlaps the square, but x + y stays near 8 and the square's at most 2
        Assert.True(aRod.Transformed(Rotation(-M_PI / 4, 4, 4)).CornerMin().X() < 1);
        Assert.True(aBox.IsOut(aRod, Rotation(-M_PI / 4, 4, 4)));
    }

    [Fact]
    public void A_box_is_in_another_under_a_transformation_by_the_others_axes()
    {
        B2f aBox = Square(1);
        Assert.True(aBox.IsIn(Square(3), Translation(1)));
        Assert.False(aBox.IsIn(Square(3), Translation(3)));
        // a corner of the square is sqrt(2) along the turned axis
        Assert.True(aBox.IsIn(Square(1.5), Rotation(M_PI / 4)));
        Assert.False(aBox.IsIn(Square(1.3), Rotation(M_PI / 4)));
        Assert.False(new B2f().IsIn(Square(1)));
        Assert.False(new B2f().IsIn(Square(1), Translation(1)));
    }

    [Fact]
    public void A_box_transformed_holds_the_transformed_corners()
    {
        B2f aBox = new(new Xy(0, 0), new Xy(2, 1));
        Corners(aBox.Transformed(Rotation(M_PI / 2, 10, 0)), new Xy(9, -2), new Xy(11, 2));
        Corners(aBox.Transformed(new Transform2D().WithScale(new Point2D(0, 0), 2)), new Xy(-4, -2), new Xy(4, 2));
    }

    [Fact]
    public void A_line_and_a_segment_by_where_they_pass()
    {
        B2f aBox = Square(1);
        Assert.False(aBox.IsOut(new Axis2D(new Point2D(-5, 0), new Direction2D(1, 0))));
        Assert.True(aBox.IsOut(new Axis2D(new Point2D(-5, 2), new Direction2D(1, 0))));
        Assert.False(aBox.IsOut(new Axis2D(new Point2D(-5, -5), new Direction2D(1, 1))));
        Assert.True(aBox.IsOut(new Axis2D(new Point2D(5, -5), new Direction2D(1, 1))));

        // through, stopping short, beyond, and clipping a corner's outside
        Assert.False(aBox.IsOut(new Xy(-5, 0), new Xy(5, 0)));
        Assert.True(aBox.IsOut(new Xy(-5, 0), new Xy(-2, 0)));
        Assert.True(aBox.IsOut(new Xy(2, 0), new Xy(5, 0)));
        Assert.False(aBox.IsOut(new Xy(0, 0), new Xy(0.5, 0.5)));
        Assert.True(aBox.IsOut(new Xy(0, 3), new Xy(3, 0)));
        Assert.False(aBox.IsOut(new Xy(0, 1.5), new Xy(1.5, 0)));
    }

    private const double M_PI = Math.PI;
}
