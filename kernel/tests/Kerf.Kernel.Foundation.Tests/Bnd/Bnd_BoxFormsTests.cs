// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;

namespace Kerf.Kernel.Foundation.Tests;

/// <summary>
/// Bnd_Box and Bnd_Box2d's functional forms, open sides, and lines and boxes against open boxes, worked by hand (unit 11I):
/// OCCT's GTests (Bnd_BoxTests, Bnd_Box2dTests) mostly use closed boxes.
/// </summary>
public class Bnd_BoxFormsTests
{
    private const double Tol = 1e-9;
    private const double Infinite = 1e100;

    private static Box Cube(double theMin, double theMax)
    {
        Box aBox = new();
        aBox.Update(theMin, theMin, theMin, theMax, theMax, theMax);
        return aBox;
    }

    private static Box2d Square(double theMin, double theMax)
    {
        Box2d aBox = new();
        aBox.Update(theMin, theMin, theMax, theMax);
        return aBox;
    }

    [Fact]
    public void The_functional_forms_leave_the_box_and_return_the_change()
    {
        Box aBox = Cube(0, 1);
        Assert.True(Box.Void().IsVoid);
        Assert.True(aBox.WithVoid().IsVoid);
        Assert.True(aBox.WithWhole().IsWhole);
        Assert.Equal(0.5, aBox.WithGap(0.5).GetGap());
        Assert.Equal(-0.5, aBox.WithGap(0.5).GetXMin());
        Assert.Equal(0.25, aBox.Enlarged(0.25).GetGap());
        Assert.Equal(3.0, aBox.Updated(-1, -2, -3, 1, 2, 3).GetZMax());
        Assert.Equal(-3.0, aBox.Updated(-1, -2, -3, 1, 2, 3).GetZMin());
        Assert.Equal(7.0, aBox.Updated(7, 7, 7).GetXMax());
        Assert.Equal(9.0, aBox.Added(new Point3D(9, 0, 0)).GetXMax());
        Assert.Equal(5.0, aBox.Added(Cube(2, 5)).GetYMax());

        Assert.True(aBox.OpenedXmin().IsOpenXmin);
        Assert.True(aBox.OpenedXmax().IsOpenXmax);
        Assert.True(aBox.OpenedYmin().IsOpenYmin);
        Assert.True(aBox.OpenedYmax().IsOpenYmax);
        Assert.True(aBox.OpenedZmin().IsOpenZmin);
        Assert.True(aBox.OpenedZmax().IsOpenZmax);

        // a point with a direction: the point, and the side the direction goes towards
        Box aFromPoint = Box.FromPoint(new Point3D(1, 2, 3), new Direction3D(1, 0, 0));
        Assert.True(aFromPoint.IsOpenXmax);
        Assert.False(aFromPoint.IsOpenXmin);
        Assert.Equal(3.0, aFromPoint.GetZMax());
        Assert.Equal(1.0, Box.FromPoint(new Point3D(1, 2, 3)).GetXMin());
        Assert.True(aBox.Added(new Point3D(5, 5, 5), new Direction3D(0, 1, 0)).IsOpenYmax);
        Assert.Equal(5.0, aBox.Added(new Point3D(5, 5, 5), new Direction3D(0, 1, 0)).GetXMax());
        Assert.True(aBox.Added(new Direction3D(0, 0, -1)).IsOpenZmin);

        // and the original is as it was
        Assert.Equal(Cube(0, 1), aBox);
        Assert.False(aBox.IsOpen);
    }

    [Fact]
    public void An_open_side_reads_as_infinite_and_its_corner_too()
    {
        Box aBox = Cube(0, 1).OpenedXmin().OpenedYmax().OpenedZmax();
        Assert.True(aBox.IsOpen);
        Assert.Equal(-Infinite, aBox.GetXMin());
        Assert.Equal(1.0, aBox.GetXMax());
        Assert.Equal(Infinite, aBox.GetYMax());
        Assert.Equal(-Infinite, aBox.CornerMin().X);
        Assert.Equal(Infinite, aBox.CornerMax().Z);
        aBox.Get(out double aXMin, out _, out double aZMin, out _, out double aYMax, out _);
        Assert.Equal((-Infinite, 0.0, Infinite), (aXMin, aZMin, aYMax));
        Assert.Equal(new Box.Limits(-Infinite, 1, 0, Infinite, 0, Infinite), aBox.Get());
    }

    [Fact]
    public void A_thin_box_by_each_axis_and_the_open_sides_never_thin()
    {
        Box aFlat = new();
        aFlat.Update(0, 0, 0, 10, 10, 0.01);
        Assert.False(aFlat.IsXThin(0.1));
        Assert.False(aFlat.IsYThin(0.1));
        Assert.True(aFlat.IsZThin(0.1));
        Assert.False(aFlat.IsThin(0.1));

        Box aPoint = Box.FromPoint(new Point3D(1, 1, 1));
        Assert.True(aPoint.IsThin(0.1));
        Assert.True(new Box().IsThin(0.1));
        Assert.True(new Box().IsXThin(0.1));
        Assert.False(aPoint.OpenedXmin().IsXThin(0.1));
        Assert.False(aPoint.OpenedXmax().IsXThin(0.1));
        Assert.False(aPoint.OpenedYmin().IsYThin(0.1));
        Assert.False(aPoint.OpenedYmax().IsYThin(0.1));
        Assert.False(aPoint.OpenedZmin().IsZThin(0.1));
        Assert.False(aPoint.OpenedZmax().IsZThin(0.1));
        Assert.False(aPoint.OpenedZmax().IsThin(0.1));
        Assert.False(new Box().WithWhole().IsXThin(0.1));
    }

    [Fact]
    public void A_box_is_out_of_another_only_on_its_closed_sides()
    {
        Box aBox = Cube(0, 1);
        Box aFar = Cube(5, 6);
        Assert.True(aBox.IsOut(aFar));
        Assert.True(aFar.IsOut(aBox));
        Box aOpenedAll = aBox.OpenedXmax().OpenedYmax().OpenedZmax();
        Assert.False(aOpenedAll.IsOut(aFar));
        Assert.False(aFar.IsOut(aOpenedAll));
        // opened the other way, it stays out
        Assert.True(aBox.OpenedXmin().OpenedYmin().OpenedZmin().IsOut(aFar));
        Assert.True(aFar.IsOut(aBox.OpenedXmin().OpenedYmin().OpenedZmin()));

        Assert.False(new Box().WithWhole().IsOut(aFar));
        Assert.False(aFar.IsOut(new Box().WithWhole()));
        Assert.True(new Box().IsOut(aFar));
        Assert.True(aFar.IsOut(new Box()));

        // the gap counts
        Assert.False(aBox.WithGap(2).IsOut(aFar.WithGap(2.1)));
        Assert.True(aBox.WithGap(1).IsOut(aFar.WithGap(1)));

        // along each axis alone
        foreach ((double aX, double aY, double aZ) in new[] { (5.0, 0.0, 0.0), (0.0, 5.0, 0.0), (0.0, 0.0, 5.0) })
        {
            Box aShifted = new();
            aShifted.Update(aX, aY, aZ, aX + 1, aY + 1, aZ + 1);
            Assert.True(aBox.IsOut(aShifted));
            Assert.True(aShifted.IsOut(aBox));
        }
    }

    [Fact]
    public void A_box_under_transformations_is_compared_where_it_lands()
    {
        Transform3D aMove = new Transform3D().WithTranslation(new Vector3D(5, 0, 0));
        Assert.True(Cube(0, 1).IsOut(Cube(0, 1), aMove));
        Assert.False(Cube(0, 1).IsOut(aMove, Cube(0, 1), aMove));
        Assert.True(Cube(0, 1).IsOut(new Transform3D(), Cube(0, 1), aMove));
    }

    [Fact]
    public void A_line_against_a_box_along_and_across_each_axis()
    {
        Box aBox = Cube(0, 1);
        Line3D Along(double theX, double theY, double theZ, double theDx, double theDy, double theDz)
            => new(new Point3D(theX, theY, theZ), new Direction3D(theDx, theDy, theDz));

        // parallel to each axis, through and beside
        Assert.False(aBox.IsOut(Along(-5, 0.5, 0.5, 1, 0, 0)));
        Assert.True(aBox.IsOut(Along(-5, 2, 0.5, 1, 0, 0)));
        Assert.True(aBox.IsOut(Along(-5, 0.5, 2, 1, 0, 0)));
        Assert.False(aBox.IsOut(Along(0.5, -5, 0.5, 0, 1, 0)));
        Assert.True(aBox.IsOut(Along(2, -5, 0.5, 0, 1, 0)));
        Assert.True(aBox.IsOut(Along(0.5, -5, 2, 0, 1, 0)));
        Assert.False(aBox.IsOut(Along(0.5, 0.5, -5, 0, 0, 1)));
        Assert.True(aBox.IsOut(Along(2, 0.5, -5, 0, 0, 1)));
        Assert.True(aBox.IsOut(Along(0.5, 2, -5, 0, 0, 1)));

        // in each pair of axes' plane
        Assert.False(aBox.IsOut(Along(0.5, 0.5, 0.5, 1, 1, 0)));
        Assert.True(aBox.IsOut(Along(5, 0, 0.5, 1, 1, 0)));
        Assert.True(aBox.IsOut(Along(0.5, 0.5, 5, 1, 1, 0)));
        Assert.False(aBox.IsOut(Along(0.5, 0.5, 0.5, 0, 1, 1)));
        Assert.True(aBox.IsOut(Along(0.5, 5, 0, 0, 1, 1)));
        Assert.True(aBox.IsOut(Along(5, 0.5, 0.5, 0, 1, 1)));
        Assert.False(aBox.IsOut(Along(0.5, 0.5, 0.5, 1, 0, 1)));
        Assert.True(aBox.IsOut(Along(5, 0.5, 0, 1, 0, 1)));
        Assert.True(aBox.IsOut(Along(0.5, 5, 0.5, 1, 0, 1)));

        // along all three
        Assert.False(aBox.IsOut(Along(-1, -1, -1, 1, 1, 1)));
        Assert.True(aBox.IsOut(Along(5, -1, -1, 1, 1, 1)));

        Assert.True(new Box().IsOut(Along(0, 0, 0, 1, 0, 0)));
        Assert.False(new Box().WithWhole().IsOut(Along(0, 0, 0, 1, 0, 0)));
    }

    [Fact]
    public void A_plane_against_a_box_and_a_point_against_open_sides()
    {
        Box aBox = Cube(0, 1);
        Plane3D Plane(double theZ) => new(new Point3D(0, 0, theZ), new Direction3D(0, 0, 1));
        Assert.False(aBox.IsOut(Plane(0.5)));
        Assert.True(aBox.IsOut(Plane(5)));
        Assert.False(aBox.OpenedZmax().IsOut(Plane(5)));
        Assert.True(new Box().IsOut(Plane(0)));
        Assert.False(new Box().WithWhole().IsOut(Plane(0)));

        Point3D aPast = new(5, 0.5, 0.5);
        Assert.True(aBox.IsOut(aPast));
        Assert.False(aBox.OpenedXmax().IsOut(aPast));
        Assert.True(aBox.OpenedXmin().IsOut(aPast));
        Assert.True(new Box().IsOut(aPast));
        Assert.False(new Box().WithWhole().IsOut(aPast));
    }

    [Fact]
    public void Equality_and_text()
    {
        Assert.True(Cube(0, 1) == Cube(0, 1));
        Assert.True(Cube(0, 1) != Cube(0, 2));
        Assert.True(Cube(0, 1).Equals((object)Cube(0, 1)));
        Assert.False(Cube(0, 1).Equals("a box"));
        Assert.Equal(Cube(0, 1).GetHashCode(), Cube(0, 1).GetHashCode());
        Assert.NotEmpty(Cube(0, 1).ToString());
        Assert.True(Square(0, 1) == Square(0, 1));
        Assert.True(Square(0, 1) != Square(0, 2));
        Assert.True(Square(0, 1).Equals((object)Square(0, 1)));
        Assert.False(Square(0, 1).Equals("a box"));
        Assert.Equal(Square(0, 1).GetHashCode(), Square(0, 1).GetHashCode());
        Assert.NotEmpty(Square(0, 1).ToString());
    }

    // Box2d

    [Fact]
    public void The_2d_functional_forms_leave_the_box_and_return_the_change()
    {
        Box2d aBox = Square(0, 1);
        Assert.True(Box2d.Void().IsVoid);
        Assert.True(aBox.WithVoid().IsVoid);
        Assert.True(aBox.WithWhole().IsWhole);
        Assert.Equal(-0.5, aBox.WithGap(0.5).GetXMin());
        Assert.Equal(0.25, aBox.Enlarged(0.25).GetGap());
        Assert.Equal(3.0, aBox.Updated(-1, -2, 1, 3).GetYMax());
        Assert.Equal(7.0, aBox.Updated(7, 7).GetXMax());
        Assert.Equal(9.0, aBox.Added(new Point2D(9, 0)).GetXMax());
        Assert.Equal(5.0, aBox.Added(Square(2, 5)).GetYMax());

        Assert.True(aBox.OpenedXmin().IsOpenXmin);
        Assert.True(aBox.OpenedXmax().IsOpenXmax);
        Assert.True(aBox.OpenedYmin().IsOpenYmin);
        Assert.True(aBox.OpenedYmax().IsOpenYmax);

        Box2d aFromPoint = Box2d.FromPoint(new Point2D(1, 2), new Direction2D(1, 0));
        Assert.True(aFromPoint.IsOpenXmax);
        Assert.Equal(2.0, aFromPoint.GetYMax());
        Assert.Equal(1.0, Box2d.FromPoint(new Point2D(1, 2)).GetXMin());
        Assert.True(aBox.Added(new Point2D(5, 5), new Direction2D(0, 1)).IsOpenYmax);
        Assert.True(aBox.Added(new Direction2D(-1, 0)).IsOpenXmin);

        Assert.Equal(Square(0, 1), aBox);
    }

    [Fact]
    public void A_2d_box_with_open_sides_transforms_them_too()
    {
        // a quarter turn takes +x to +y, and +y to -x
        Transform2D aTurn = new Transform2D().WithRotation(new Point2D(0, 0), Math.PI / 2);

        Assert.True(Square(0, 1).OpenedXmax().Transformed(aTurn).IsOpenYmax);
        Assert.True(Square(0, 1).OpenedXmin().Transformed(aTurn).IsOpenYmin);
        Assert.True(Square(0, 1).OpenedYmax().Transformed(aTurn).IsOpenXmin);
        Assert.True(Square(0, 1).OpenedYmin().Transformed(aTurn).IsOpenXmax);
        Assert.True(new Box2d().WithWhole().Transformed(aTurn).IsWhole);
        Assert.True(new Box2d().Transformed(aTurn).IsVoid);

        // turned closed, the corners land in [-1, 0] x [0, 1]
        Box2d aClosed = Square(0, 1).Transformed(aTurn);
        Assert.Equal(-1.0, aClosed.GetXMin(), Tol);
        Assert.Equal(1.0, aClosed.GetYMax(), Tol);
    }

    [Fact]
    public void A_2d_box_against_a_point_a_line_a_segment_and_another_box()
    {
        Box2d aBox = Square(0, 1);
        Point2D aPast = new(5, 0.5);
        Assert.True(aBox.IsOut(aPast));
        Assert.False(aBox.OpenedXmax().IsOut(aPast));
        Assert.True(aBox.OpenedXmin().IsOut(aPast));
        Assert.True(new Box2d().IsOut(aPast));
        Assert.False(new Box2d().WithWhole().IsOut(aPast));
        Assert.True(aBox.Contains(new Point2D(0.5, 0.5)));

        Assert.False(aBox.IsOut(new Line2D(new Point2D(-5, 0.5), new Direction2D(1, 0))));
        Assert.True(aBox.IsOut(new Line2D(new Point2D(-5, 2), new Direction2D(1, 0))));
        Assert.True(new Box2d().IsOut(new Line2D(new Point2D(-5, 2), new Direction2D(1, 0))));
        Assert.False(new Box2d().WithWhole().IsOut(new Line2D(new Point2D(-5, 2), new Direction2D(1, 0))));

        Assert.False(aBox.IsOut(new Point2D(-5, 0.5), new Point2D(5, 0.5)));
        Assert.True(aBox.IsOut(new Point2D(-5, 0.5), new Point2D(-1, 0.5)));
        Assert.True(aBox.IsOut(new Point2D(0, 3), new Point2D(3, 0)));
        Assert.True(new Box2d().IsOut(new Point2D(0, 0), new Point2D(1, 1)));

        Box2d aFar = Square(5, 6);
        Assert.True(aBox.IsOut(aFar));
        Assert.False(aBox.Intersects(aFar));
        Assert.False(aBox.OpenedXmax().OpenedYmax().IsOut(aFar));
        Assert.True(aBox.OpenedXmin().OpenedYmin().IsOut(aFar));
        Assert.True(new Box2d().IsOut(aFar));
        Assert.False(new Box2d().WithWhole().IsOut(aFar));

        Transform2D aMove = new Transform2D().WithTranslation(new Vector2D(5, 5));
        Assert.False(Square(5, 6).IsOut(aBox, aMove));
        Assert.True(Square(0, 1).IsOut(aBox, aMove));
        Assert.False(Square(0, 1).IsOut(aMove, Square(0, 1), aMove));
    }

    [Fact]
    public void The_distance_between_2d_boxes_is_the_gap_between_them()
    {
        Box2d aBox = Square(0, 1);
        Assert.Equal(0.0, aBox.Distance(aBox), Tol);
        Assert.Equal(Math.Sqrt(2) * 4, aBox.Distance(Square(5, 6)), Tol);
        Box2d aRight = new();
        aRight.Update(4, 0, 5, 1);
        Assert.Equal(3.0, aBox.Distance(aRight), Tol);
        Assert.Equal(2.0, aBox.SquareExtent(), Tol);
    }
}
