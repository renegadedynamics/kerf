// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>
/// Unit 11J: the geometric values' members by name, where <see cref="GeometryValueLawsTests"/> reaches them by reflection
/// and looks only at their location. Here the whole value counts: its axes, its sizes under a negative scale, the implicit
/// equations against points on the shape, and what a bad size does.
/// </summary>
public class GeometryDirectTests
{
    private static double Cell(Matrix3x3 m, int row, int column)
    {
        var all = new double[9];
        m.CopyTo(all);
        return all[(row - 1) * 3 + column - 1];
    }

    private static readonly Point3D P = new(1, 2, 3);
    private static readonly Direction3D Up = Direction3D.UnitZ;
    private static readonly Frame3D Frame = new(P, Up, Direction3D.UnitX);
    private static readonly CoordinateSystem System3 = new(P, Up, Direction3D.UnitX);
    private static readonly Point2D P2 = new(1, 2);
    private static readonly Frame2D Frame2 = new(P2, new Direction2D(0, 1));
    private static readonly Axis3D AboutZ = new(new Point3D(0, 0, 0), Up);
    private static readonly Transform3D Quarter = Transform3D.Rotation(AboutZ, Math.PI / 2);

    private static void Near(Point3D a, Point3D b) => Assert.True(a.IsEqual(b, 1e-9), $"{a} and {b}");

    private static void Near(Point2D a, Point2D b) => Assert.True(a.IsEqual(b, 1e-9), $"{a} and {b}");

    private static void Near(Direction3D a, Direction3D b) => Assert.True(a.IsEqual(b, 1e-9), $"{a} and {b}");

    private static void Near(Direction2D a, Direction2D b) => Assert.True(a.IsEqual(b, 1e-9), $"{a} and {b}");

    // ---- Implicit equations hold at points on the shape ----

    [Fact]
    public void A_2D_line_s_equation_holds_along_it_and_measures_distance()
    {
        var line = new Line2D(new Point2D(1, 1), new Direction2D(3, 4));
        line.Coefficients(out double a, out double b, out double c);
        foreach (double t in new[] { -2.0, 0, 5 })
        {
            Point2D on = new(1 + t * 0.6, 1 + t * 0.8);
            Assert.Equal(0, a * on.X + b * on.Y + c, 9);
            Assert.True(line.Contains(on, 1e-9));
        }
        // normalised: its value off the line is the distance
        var off = new Point2D(1 - 0.8, 1 + 0.6);
        Assert.Equal(1, Math.Abs(a * off.X + b * off.Y + c), 9);
        Assert.Equal(1, line.SquareDistanceTo(off), 9);
        Assert.False(line.Contains(off, 0.5));
        Assert.True(line.Contains(off, 1.5));
    }

    [Fact]
    public void Two_2D_lines_measure_their_angle_and_gap()
    {
        var x = new Line2D(new Point2D(0, 0), new Direction2D(1, 0));
        var parallel = new Line2D(new Point2D(0, 3), new Direction2D(1, 0));
        var crossing = new Line2D(new Point2D(5, 5), new Direction2D(0, 1));
        Assert.Equal(3, x.DistanceTo(parallel), 12);
        Assert.Equal(0, x.DistanceTo(crossing), 12);
        Assert.Equal(Math.PI / 2, x.AngleTo(crossing), 12);
        Assert.Equal(-Math.PI / 2, crossing.AngleTo(x), 12);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void The_equation_of_a_circle_or_ellipse_in_2D_holds_on_it(bool direct)
    {
        var frame = new Frame2D(new Axis2D(P2, new Direction2D(1, 1)), direct);
        var circle = new Circle2D(frame, 2);
        var ellipse = new Ellipse2D(frame, 3, 2);
        Assert.Equal(direct, ellipse.IsDirect);
        Assert.Equal(direct, frame.YDirection.X * frame.XDirection.Y - frame.YDirection.Y * frame.XDirection.X < 0);
        circle.Coefficients(out double a, out double b, out double c, out double d, out double e, out double f);
        ellipse.Coefficients(out double a2, out double b2, out double c2, out double d2, out double e2, out double f2);
        foreach (double t in new[] { 0.0, 0.7, 2, 4 })
        {
            Point2D onC = P2.Translated((new Vector2D(frame.XDirection) * (2 * Math.Cos(t))) + (new Vector2D(frame.YDirection) * (2 * Math.Sin(t))));
            Assert.Equal(0, a * onC.X * onC.X + b * onC.Y * onC.Y + 2 * c * onC.X * onC.Y + 2 * d * onC.X + 2 * e * onC.Y + f, 8);
            Point2D onE = P2.Translated((new Vector2D(frame.XDirection) * (3 * Math.Cos(t))) + (new Vector2D(frame.YDirection) * (2 * Math.Sin(t))));
            Assert.Equal(0, a2 * onE.X * onE.X + b2 * onE.Y * onE.Y + 2 * c2 * onE.X * onE.Y + 2 * d2 * onE.X + 2 * e2 * onE.Y + f2, 8);
        }
    }

    [Fact]
    public void A_2D_hyperbola_s_equation_holds_on_both_branches()
    {
        var hyperbola = new Hyperbola2D(Frame2, 3, 2);
        Assert.True(hyperbola.IsDirect);
        hyperbola.Coefficients(out double a, out double b, out double c, out double d, out double e, out double f);
        Hyperbola2D other = hyperbola.OtherBranch();
        Near(hyperbola.Location, other.Location);
        Near(hyperbola.XAxis.Direction.Reversed(), other.XAxis.Direction);
        foreach (double t in new[] { -1.0, 0.0, 1.5 })
        {
            foreach (double side in new[] { 1.0, -1.0 })
            {
                Point2D on = P2.Translated((new Vector2D(Frame2.XDirection) * (side * 3 * Math.Cosh(t))) + (new Vector2D(Frame2.YDirection) * (2 * Math.Sinh(t))));
                Assert.Equal(0, a * on.X * on.X + b * on.Y * on.Y + 2 * c * on.X * on.Y + 2 * d * on.X + 2 * e * on.Y + f, 7);
            }
        }
    }

    [Fact]
    public void The_equation_of_a_plane_cylinder_and_sphere_holds_on_them_and_is_zero_nowhere_else()
    {
        var sphere = new Sphere3D(System3, 2);
        sphere.Coefficients(out double a1, out double a2, out double a3, out double b1, out double b2, out double b3, out double c1, out double c2, out double c3, out double d);
        var cylinder = new Cylinder3D(System3, 2);
        cylinder.Coefficients(out double p1, out double p2, out double p3, out double q1, out double q2, out double q3, out double r1, out double r2, out double r3, out double s);
        double Quadric(Point3D x, double aa, double ab, double ac, double ba, double bb, double bc, double ca, double cb, double cc, double dd) =>
            aa * x.X * x.X + ab * x.Y * x.Y + ac * x.Z * x.Z + 2 * (ba * x.X * x.Y + bb * x.X * x.Z + bc * x.Y * x.Z) + 2 * (ca * x.X + cb * x.Y + cc * x.Z) + dd;
        foreach (double t in new[] { 0.0, 1.0, 2.5 })
        {
            var onSphere = new Point3D(P.X + 2 * Math.Cos(t) * Math.Cos(0.4), P.Y + 2 * Math.Sin(t) * Math.Cos(0.4), P.Z + 2 * Math.Sin(0.4));
            Assert.Equal(0, Quadric(onSphere, a1, a2, a3, b1, b2, b3, c1, c2, c3, d), 9);
            var onCylinder = new Point3D(P.X + 2 * Math.Cos(t), P.Y + 2 * Math.Sin(t), 17);
            Assert.Equal(0, Quadric(onCylinder, p1, p2, p3, q1, q2, q3, r1, r2, r3, s), 9);
        }
        Assert.NotEqual(0, Quadric(P, a1, a2, a3, b1, b2, b3, c1, c2, c3, d), 3);
        Assert.NotEqual(0, Quadric(P, p1, p2, p3, q1, q2, q3, r1, r2, r3, s), 3);
    }

    // ---- Planes measure by the side their normal is on ----

    [Fact]
    public void A_plane_measures_points_lines_and_planes_by_its_normal_s_side()
    {
        var plane = new Plane3D(new Point3D(0, 0, 1), Up);
        Assert.Equal(2, plane.SignedDistance(new Point3D(5, 5, 3)), 12);
        Assert.Equal(-2, plane.SignedDistance(new Point3D(5, 5, -1)), 12);
        Assert.Equal(4, plane.SquareDistanceTo(new Point3D(5, 5, -1)), 12);

        var above = new Line3D(new Point3D(0, 0, 4), Direction3D.UnitX);
        var through = new Line3D(new Point3D(0, 0, 1), Direction3D.UnitZ);
        Assert.Equal(3, plane.SignedDistance(above), 12);
        Assert.Equal(3, plane.DistanceTo(above), 12);
        Assert.Equal(9, plane.SquareDistanceTo(above), 12);
        Assert.Equal(0, plane.SignedDistance(through), 12);
        Assert.Equal(0, plane.DistanceTo(through), 12);

        var parallel = new Plane3D(new Point3D(0, 0, -2), Up);
        var tilted = new Plane3D(new Point3D(0, 0, 9), Direction3D.UnitX);
        Assert.Equal(-3, plane.SignedDistance(parallel), 12);
        Assert.Equal(3, plane.DistanceTo(parallel), 12);
        Assert.Equal(9, plane.SquareDistanceTo(parallel), 12);
        Assert.Equal(0, plane.DistanceTo(tilted), 12);

        Assert.True(plane.Contains(new Line3D(new Point3D(7, 7, 1), Direction3D.UnitY), 1e-9, 1e-9));
        Assert.False(plane.Contains(above, 1e-9, 1e-9));
        Assert.False(plane.Contains(through, 1e-9, 1e-9));
        Near(plane.Position.YDirection, plane.YAxis.Direction);
    }

    // ---- 3D lines ----

    [Fact]
    public void A_3D_line_measures_points_and_skew_lines()
    {
        var line = new Line3D(new Point3D(0, 0, 0), Direction3D.UnitX);
        Assert.Equal(25, line.SquareDistanceTo(new Point3D(9, 3, 4)), 12);
        var skew = new Line3D(new Point3D(0, 0, 7), Direction3D.UnitY);
        Assert.Equal(49, line.SquareDistanceTo(skew), 12);
        Assert.Equal(0, line.SquareDistanceTo(new Line3D(new Point3D(5, 0, 0), Direction3D.UnitY)), 12);
        // the normal through a point runs from its foot on the line to the point
        Line3D normal = line.Normal(new Point3D(9, 3, 4));
        Near(new Point3D(9, 3, 4), normal.Location);
        Near(new Direction3D(0, 3, 4), normal.Direction);
        // on the line itself there is no normal
        Assert.ThrowsAny<Exception>(() => line.Normal(new Point3D(9, 0, 0)));
    }

    // ---- Each value's transformations against its own Transformed ----

    [Fact]
    public void A_3D_circle_s_moves_keep_its_radius_and_axes()
    {
        var c = new Circle3D(Frame, 2);
        Assert.Equal(2, c.Mirrored(AboutZ).Radius);
        Near(new Point3D(-1, -2, 3), c.Mirrored(AboutZ).Location);
        Near(new Point3D(1, 2, -3), c.Mirrored(Frame3D.World).Location);
        Near(Up, c.Mirrored(AboutZ).Axis.Direction);
        Assert.Equal(2, c.Scaled(P, -3).Radius / 3);
        Near(P, c.Scaled(P, 5).Location);
        Near(new Point3D(1, 2, 8), c.Translated(new Point3D(0, 0, 0), new Point3D(0, 0, 5)).Location);
        Assert.Equal(25, c.SquareDistanceTo(new Point3D(1, 2, 3 + 5)) - 4, 9);
        Near(c.Position.YDirection, c.YAxis.Direction);
        Assert.Equal(9, c.WithRadius(9).Radius);
        Near(new Point3D(0, 0, 0), c.WithPosition(Frame3D.World).Location);
        Assert.ThrowsAny<ArgumentException>(() => c.WithRadius(-1));
        Assert.False(c == c.WithRadius(9));
        Assert.True(c != c.WithRadius(9));
        Assert.True(c.Equals(new Circle3D(Frame, 2)));
    }

    [Fact]
    public void An_ellipse_s_moves_keep_its_radii_and_refuses_the_wrong_ones()
    {
        var e = new Ellipse3D(Frame, 3, 2);
        foreach (Ellipse3D moved in new[]
        {
            e.Mirrored(new Point3D(0, 0, 0)), e.Mirrored(AboutZ), e.Mirrored(Frame3D.World), e.Rotated(AboutZ, 1), e.Transformed(Quarter),
            e.Translated(new Vector3D(1, 1, 1)), e.Translated(new Point3D(0, 0, 0), new Point3D(1, 1, 1)),
        })
        {
            Assert.Equal(3, moved.MajorRadius, 12);
            Assert.Equal(2, moved.MinorRadius, 12);
        }
        Near(new Point3D(-2, 1, 3), e.Transformed(Quarter).Location);
        Near(Direction3D.UnitY, e.Transformed(Quarter).XAxis.Direction);
        Near(new Direction3D(-1, 0, 0), e.Transformed(Quarter).YAxis.Direction);
        // a negative scale still has positive radii
        Ellipse3D scaled = e.Scaled(P, -2);
        Assert.Equal(6, scaled.MajorRadius, 12);
        Assert.Equal(4, scaled.MinorRadius, 12);
        Assert.Equal(5, e.WithMajorRadius(5).MajorRadius);
        Assert.Equal(1, e.WithMinorRadius(1).MinorRadius);
        Near(new Point3D(0, 0, 0), e.WithPosition(Frame3D.World).Location);
        Assert.ThrowsAny<ArgumentException>(() => e.WithMajorRadius(1));
        Assert.ThrowsAny<ArgumentException>(() => e.WithMinorRadius(4));
        Assert.ThrowsAny<ArgumentException>(() => e.WithMinorRadius(-1));
        Assert.True(e.Equals(new Ellipse3D(Frame, 3, 2)));
        Assert.True(e == new Ellipse3D(Frame, 3, 2));
        Assert.True(e != e.WithMajorRadius(5));
    }

    [Fact]
    public void A_hyperbola_s_and_a_parabola_s_moves_keep_their_sizes_under_any_sign_of_scale()
    {
        var h = new Hyperbola3D(Frame, 3, 2);
        foreach (Hyperbola3D moved in new[] { h.Mirrored(P), h.Mirrored(AboutZ), h.Mirrored(Frame3D.World), h.Rotated(AboutZ, 1), h.Transformed(Quarter), h.Translated(new Vector3D(1, 1, 1)), h.Translated(P, P) })
        {
            Assert.Equal(3, moved.MajorRadius, 12);
            Assert.Equal(2, moved.MinorRadius, 12);
        }
        Assert.Equal(6, h.Scaled(P, -2).MajorRadius, 12);
        Near(h.Position.XDirection, h.XAxis.Direction);
        Near(h.Position.YDirection, h.YAxis.Direction);
        Assert.True(h == new Hyperbola3D(Frame, 3, 2));
        Assert.True(h != h.Scaled(P, 2));
        Assert.True(h.Equals(new Hyperbola3D(Frame, 3, 2)));

        var p = new Parabola3D(Frame, 2);
        // the parameter is twice the focal length
        Assert.Equal(4, p.Parameter, 12);
        foreach (Parabola3D moved in new[] { p.Mirrored(P), p.Mirrored(AboutZ), p.Mirrored(Frame3D.World), p.Rotated(AboutZ, 1), p.Transformed(Quarter), p.Translated(new Vector3D(1, 1, 1)), p.Translated(P, P) })
        {
            Assert.Equal(2, moved.Focal, 12);
        }
        Assert.Equal(4, p.Scaled(P, -2).Focal, 12);
        Near(p.Position.XDirection, p.XAxis.Direction);
        Near(p.Position.YDirection, p.YAxis.Direction);
        Assert.True(p == new Parabola3D(Frame, 2));
        Assert.True(p != p.Scaled(P, 2));
        Assert.True(p.Equals(new Parabola3D(Frame, 2)));
    }

    [Fact]
    public void A_cylinder_s_cone_s_sphere_s_and_torus_s_moves_keep_their_sizes()
    {
        var cylinder = new Cylinder3D(System3, 2);
        foreach (Cylinder3D m in new[] { cylinder.Mirrored(P), cylinder.Mirrored(AboutZ), cylinder.Mirrored(Frame3D.World), cylinder.Rotated(AboutZ, 1), cylinder.Transformed(Quarter), cylinder.Translated(new Vector3D(1, 1, 1)), cylinder.Translated(P, P) })
        {
            Assert.Equal(2, m.Radius, 12);
        }
        Assert.Equal(6, cylinder.Scaled(P, -3).Radius, 12);
        Assert.Equal(7, cylinder.WithRadius(7).Radius);
        Near(Direction3D.UnitX, cylinder.WithAxis(new Axis3D(P, Direction3D.UnitX)).Axis.Direction);
        Near(new Point3D(0, 0, 0), cylinder.WithPosition(CoordinateSystem.World).Location);
        Near(cylinder.Position.XDirection, cylinder.XAxis.Direction);
        Near(cylinder.Position.YDirection, cylinder.YAxis.Direction);
        Assert.ThrowsAny<ArgumentException>(() => cylinder.WithRadius(-1));
        Assert.True(cylinder == new Cylinder3D(System3, 2));
        Assert.True(cylinder != cylinder.WithRadius(7));
        Assert.True(cylinder.Equals(new Cylinder3D(System3, 2)));

        var cone = new Cone3D(System3, 0.5, 2);
        foreach (Cone3D m in new[] { cone.Mirrored(P), cone.Mirrored(AboutZ), cone.Mirrored(Frame3D.World), cone.Rotated(AboutZ, 1), cone.Transformed(Quarter), cone.Translated(new Vector3D(1, 1, 1)), cone.Translated(P, P) })
        {
            Assert.Equal(2, m.RefRadius, 12);
            Assert.Equal(0.5, m.SemiAngle, 12);
        }
        Assert.Equal(6, cone.Scaled(P, -3).RefRadius, 12);
        Assert.Equal(0.25, cone.WithSemiAngle(0.25).SemiAngle);
        Assert.Equal(7, cone.WithRadius(7).RefRadius);
        Near(new Point3D(0, 0, 0), cone.WithPosition(CoordinateSystem.World).Location);
        Near(cone.Position.XDirection, cone.XAxis.Direction);
        Near(cone.Position.YDirection, cone.YAxis.Direction);
        Assert.ThrowsAny<Exception>(() => cone.WithSemiAngle(0));
        Assert.ThrowsAny<Exception>(() => cone.WithSemiAngle(Math.PI / 2));
        Assert.ThrowsAny<ArgumentException>(() => cone.WithRadius(-1));
        Assert.True(cone == new Cone3D(System3, 0.5, 2));
        Assert.True(cone != cone.WithRadius(7));
        Assert.True(cone.Equals(new Cone3D(System3, 0.5, 2)));

        var sphere = new Sphere3D(System3, 2);
        foreach (Sphere3D m in new[] { sphere.Mirrored(P), sphere.Mirrored(AboutZ), sphere.Mirrored(Frame3D.World), sphere.Rotated(AboutZ, 1), sphere.Transformed(Quarter), sphere.Translated(new Vector3D(1, 1, 1)), sphere.Translated(P, P) })
        {
            Assert.Equal(2, m.Radius, 12);
        }
        Assert.Equal(6, sphere.Scaled(P, -3).Radius, 12);
        Assert.Equal(7, sphere.WithRadius(7).Radius);
        Near(new Point3D(0, 0, 0), sphere.WithPosition(CoordinateSystem.World).Location);
        Near(sphere.Position.XDirection, sphere.XAxis.Direction);
        Near(sphere.Position.YDirection, sphere.YAxis.Direction);
        Assert.True(sphere == new Sphere3D(System3, 2));
        Assert.True(sphere != sphere.WithRadius(7));
        Assert.True(sphere.Equals(new Sphere3D(System3, 2)));

        var torus = new Torus3D(System3, 5, 2);
        foreach (Torus3D m in new[] { torus.Mirrored(P), torus.Mirrored(AboutZ), torus.Mirrored(Frame3D.World), torus.Rotated(AboutZ, 1), torus.Transformed(Quarter), torus.Translated(new Vector3D(1, 1, 1)), torus.Translated(P, P) })
        {
            Assert.Equal(5, m.MajorRadius, 12);
            Assert.Equal(2, m.MinorRadius, 12);
        }
        Torus3D scaled = torus.Scaled(P, -3);
        Assert.Equal(15, scaled.MajorRadius, 12);
        Assert.Equal(6, scaled.MinorRadius, 12);
        Near(torus.Position.XDirection, torus.XAxis.Direction);
        Near(torus.Position.YDirection, torus.YAxis.Direction);
        // reversing V keeps the same surface in the other sense
        Assert.Equal(5, torus.VReversed().MajorRadius);
        Assert.True(torus == new Torus3D(System3, 5, 2));
        Assert.True(torus != scaled);
        Assert.True(torus.Equals(new Torus3D(System3, 5, 2)));
    }

    // ---- 2D values ----

    [Fact]
    public void Two_dimensional_curves_move_as_their_frames_do()
    {
        var mirror = new Axis2D(new Point2D(0, 0), new Direction2D(1, 0));
        var c = new Circle2D(Frame2, 2);
        Near(new Point2D(1, -2), c.Mirrored(mirror).Location);
        Near(new Point2D(-1, -2), c.Mirrored(new Point2D(0, 0)).Location);
        Near(new Point2D(-2, 1), c.Rotated(new Point2D(0, 0), Math.PI / 2).Location);
        Assert.Equal(6, c.Scaled(P2, -3).Radius, 12);
        Near(new Point2D(2, 3), c.Translated(new Vector2D(1, 1)).Location);
        Near(new Point2D(2, 3), c.Translated(new Point2D(0, 0), new Point2D(1, 1)).Location);
        Assert.Equal(7, c.WithRadius(7).Radius);
        Near(new Point2D(5, 5), c.WithLocation(new Point2D(5, 5)).Location);
        // reversing swaps the sense, not the circle
        Assert.Equal(2, c.Reversed().Radius);
        Near(c.Position.XDirection, c.Reversed().Position.XDirection);
        Near(c.Position.YDirection.Reversed(), c.Reversed().Position.YDirection);
        Near(c.Position.XDirection, c.Axis.XDirection);
        Assert.True(c == new Circle2D(Frame2, 2));
        Assert.True(c != c.WithRadius(7));
        Assert.True(c.Equals(new Circle2D(Frame2, 2)));

        var e = new Ellipse2D(Frame2, 3, 2);
        Near(new Point2D(1, -2), e.Mirrored(mirror).Location);
        Near(new Point2D(-1, -2), e.Mirrored(new Point2D(0, 0)).Location);
        Near(new Point2D(-2, 1), e.Rotated(new Point2D(0, 0), Math.PI / 2).Location);
        Assert.Equal(6, e.Scaled(P2, -2).MajorRadius, 12);
        Near(new Point2D(2, 3), e.Translated(new Vector2D(1, 1)).Location);
        Near(new Point2D(2, 3), e.Translated(new Point2D(0, 0), new Point2D(1, 1)).Location);
        Near(new Point2D(2, 2), e.Transformed(Transform2D.Translation(new Vector2D(1, 0))).Location);
        Assert.Equal(5, e.WithMajorRadius(5).MajorRadius);
        Assert.Equal(1, e.WithMinorRadius(1).MinorRadius);
        Assert.Equal(3, e.Reversed().MajorRadius);
        Assert.ThrowsAny<ArgumentException>(() => e.WithMajorRadius(1));
        Assert.True(e == new Ellipse2D(Frame2, 3, 2));
        Assert.True(e != e.WithMajorRadius(5));
        Assert.True(e.Equals(new Ellipse2D(Frame2, 3, 2)));

        var h = new Hyperbola2D(Frame2, 3, 2);
        Near(new Point2D(1, -2), h.Mirrored(mirror).Location);
        Near(new Point2D(-1, -2), h.Mirrored(new Point2D(0, 0)).Location);
        Near(new Point2D(-2, 1), h.Rotated(new Point2D(0, 0), Math.PI / 2).Location);
        Assert.Equal(6, h.Scaled(P2, -2).MajorRadius, 12);
        Near(new Point2D(2, 3), h.Translated(new Vector2D(1, 1)).Location);
        Near(new Point2D(2, 3), h.Translated(new Point2D(0, 0), new Point2D(1, 1)).Location);
        Assert.True(h == new Hyperbola2D(Frame2, 3, 2));
        Assert.True(h != h.Scaled(P2, 2));
        Assert.True(h.Equals(new Hyperbola2D(Frame2, 3, 2)));

        var p = new Parabola2D(Frame2, 2);
        Assert.True(p.IsDirect);
        Near(new Point2D(1, -2), p.Mirrored(mirror).Location);
        Near(new Point2D(-1, -2), p.Mirrored(new Point2D(0, 0)).Location);
        Near(new Point2D(-2, 1), p.Rotated(new Point2D(0, 0), Math.PI / 2).Location);
        Assert.Equal(4, p.Scaled(P2, -2).Focal, 12);
        Near(new Point2D(2, 3), p.Translated(new Vector2D(1, 1)).Location);
        Near(new Point2D(2, 3), p.Translated(new Point2D(0, 0), new Point2D(1, 1)).Location);
        Assert.True(p == new Parabola2D(Frame2, 2));
        Assert.True(p != p.Scaled(P2, 2));
        Assert.True(p.Equals(new Parabola2D(Frame2, 2)));
        // through a directrix and a focus: its vertex half way between them
        var byFocus = new Parabola2D(new Axis2D(new Point2D(0, 0), new Direction2D(0, 1)), new Point2D(4, 0), true);
        Near(new Point2D(2, 0), byFocus.Location);
        Assert.Equal(2, byFocus.Focal, 12);
    }

    [Fact]
    public void A_2D_line_s_and_axis_s_moves_follow_their_direction()
    {
        var mirror = new Axis2D(new Point2D(0, 0), new Direction2D(1, 0));
        var l = new Line2D(P2, new Direction2D(1, 1));
        Near(new Direction2D(1, -1), l.Mirrored(mirror).Direction);
        Near(new Point2D(1, -2), l.Mirrored(mirror).Location);
        Near(new Point2D(-1, -2), l.Mirrored(new Point2D(0, 0)).Location);
        Near(new Direction2D(-1, -1), l.Mirrored(new Point2D(0, 0)).Direction);
        Near(new Direction2D(-1, 1), l.Rotated(new Point2D(0, 0), Math.PI / 2).Direction);
        Near(new Point2D(-1, -2), l.Scaled(new Point2D(0, 0), -1).Location);
        Near(new Direction2D(-1, -1), l.Scaled(new Point2D(0, 0), -1).Direction);
        Near(new Point2D(2, 3), l.Translated(new Vector2D(1, 1)).Location);
        Near(new Point2D(2, 3), l.Translated(new Point2D(0, 0), new Point2D(1, 1)).Location);
        Near(new Point2D(2, 2), l.Transformed(Transform2D.Translation(new Vector2D(1, 0))).Location);
        Near(new Direction2D(-1, -1), l.Reversed().Direction);
        Near(new Direction2D(1, 0), l.WithDirection(new Direction2D(1, 0)).Direction);
        Near(new Point2D(9, 9), l.WithLocation(new Point2D(9, 9)).Location);
        Assert.True(l == new Line2D(P2, new Direction2D(1, 1)));
        Assert.True(l != l.Reversed());
        Assert.True(l.Equals(new Line2D(P2, new Direction2D(1, 1))));

        var a = new Axis2D(P2, new Direction2D(1, 1));
        Near(new Direction2D(1, -1), a.Mirrored(mirror).Direction);
        Near(new Direction2D(-1, -1), a.Mirrored(new Point2D(0, 0)).Direction);
        Near(new Direction2D(-1, 1), a.Rotated(new Point2D(0, 0), Math.PI / 2).Direction);
        Near(new Direction2D(-1, -1), a.Scaled(new Point2D(0, 0), -1).Direction);
        Near(new Point2D(2, 2), a.Transformed(Transform2D.Translation(new Vector2D(1, 0))).Location);
        Near(new Point2D(2, 3), a.Translated(new Vector2D(1, 1)).Location);
        Near(new Point2D(2, 3), a.Translated(new Point2D(0, 0), new Point2D(1, 1)).Location);
        Near(new Direction2D(-1, -1), a.Reversed().Direction);
        Near(new Direction2D(1, 0), a.WithDirection(new Direction2D(1, 0)).Direction);
        Near(new Point2D(9, 9), a.WithLocation(new Point2D(9, 9)).Location);
        Assert.Equal(Math.PI / 2, new Axis2D(P2, new Direction2D(1, 0)).AngleTo(new Axis2D(P2, new Direction2D(0, 1))), 12);
        Assert.True(a.IsParallel(a.Reversed(), 1e-9));
        Assert.False(a.IsParallel(new Axis2D(P2, new Direction2D(1, 0)), 1e-9));
        Assert.True(a == new Axis2D(P2, new Direction2D(1, 1)));
        Assert.True(a != a.Reversed());
        Assert.True(a.Equals(new Axis2D(P2, new Direction2D(1, 1))));
    }

    // ---- 3D axes, frames and coordinate systems ----

    [Fact]
    public void A_3D_axis_s_tests_and_moves()
    {
        var a = new Axis3D(P, Up);
        Assert.Equal(Math.PI / 2, a.AngleTo(new Axis3D(P, Direction3D.UnitX)), 12);
        Assert.True(a.IsNormal(new Axis3D(P, Direction3D.UnitX), 1e-9));
        Assert.False(a.IsNormal(a, 1e-9));
        Assert.True(a.IsOpposite(a.Reversed(), 1e-9));
        Assert.False(a.IsOpposite(a, 1e-9));
        Near(Up, a.Mirrored(new Axis3D(new Point3D(0, 0, 0), Up)).Direction);
        Near(new Point3D(-1, -2, 3), a.Mirrored(AboutZ).Location);
        Near(Up.Reversed(), a.Mirrored(Frame3D.World).Direction);
        Near(new Point3D(-1, -2, -3), a.Mirrored(new Point3D(0, 0, 0)).Location);
        Near(new Point3D(-2, 1, 3), a.Rotated(AboutZ, Math.PI / 2).Location);
        Near(Up.Reversed(), a.Scaled(new Point3D(0, 0, 0), -1).Direction);
        Near(new Point3D(2, 3, 4), a.Translated(new Vector3D(1, 1, 1)).Location);
        Near(new Point3D(2, 3, 4), a.Translated(new Point3D(0, 0, 0), new Point3D(1, 1, 1)).Location);
        Near(Direction3D.UnitX, a.WithDirection(Direction3D.UnitX).Direction);
        Near(new Point3D(9, 9, 9), a.WithLocation(new Point3D(9, 9, 9)).Location);
        Assert.True(a == new Axis3D(P, Up));
        Assert.True(a != a.Reversed());
        Assert.True(a.Equals(new Axis3D(P, Up)));
    }

    [Fact]
    public void A_frame_stays_right_handed_and_a_coordinate_system_says_if_it_isn_t()
    {
        foreach (Frame3D f in new[] { Frame.Mirrored(AboutZ), Frame.Mirrored(Frame3D.World), Frame.Mirrored(new Point3D(0, 0, 0)), Frame.Scaled(P, -2), Frame.Rotated(AboutZ, 1) })
        {
            Near(f.Direction, new Direction3D(f.XDirection.Crossed(f.YDirection).X, f.XDirection.Crossed(f.YDirection).Y, f.XDirection.Crossed(f.YDirection).Z));
        }
        Near(P, Frame.Scaled(P, -2).Location);
        Near(new Point3D(2, 3, 4), Frame.Translated(new Point3D(0, 0, 0), new Point3D(1, 1, 1)).Location);
        Near(Direction3D.UnitX, Frame.WithAxis(new Axis3D(P, Direction3D.UnitX)).Direction);
        Assert.True(Frame.IsCoplanar(new Axis3D(new Point3D(9, 9, 3), Direction3D.UnitX), 1e-9, 1e-9));
        Assert.False(Frame.IsCoplanar(new Axis3D(new Point3D(9, 9, 4), Direction3D.UnitX), 1e-9, 1e-9));
        Assert.False(Frame.IsCoplanar(new Axis3D(P, Up), 1e-9, 1e-9));
        Assert.True(Frame == new Frame3D(P, Up, Direction3D.UnitX));
        Assert.True(Frame != Frame.Scaled(P, -2));
        Assert.True(Frame.Equals(new Frame3D(P, Up, Direction3D.UnitX)));

        CoordinateSystem s = System3;
        Assert.True(s.IsDirect);
        Assert.False(s.Mirrored(Frame3D.World).IsDirect);
        Assert.Equal(s.IsDirect, s.Mirrored(AboutZ).IsDirect);
        Assert.False(s.Mirrored(new Point3D(0, 0, 0)).IsDirect);
        Assert.False(s.Scaled(P, -1).IsDirect);
        Near(new Point3D(-2, 1, 3), s.Rotated(AboutZ, Math.PI / 2).Location);
        Near(new Point3D(-2, 1, 3), s.Transformed(Quarter).Location);
        Near(new Point3D(2, 3, 4), s.Translated(new Point3D(0, 0, 0), new Point3D(1, 1, 1)).Location);
        Near(Up, s.Axis.Direction);
        Assert.True(s.IsCoplanar(new CoordinateSystem(new Point3D(9, 9, 3), Up, Direction3D.UnitX), 1e-9, 1e-9));
        Assert.False(s.IsCoplanar(new CoordinateSystem(new Point3D(9, 9, 4), Up, Direction3D.UnitX), 1e-9, 1e-9));
        Assert.True(s == new CoordinateSystem(P, Up, Direction3D.UnitX));
        Assert.True(s != s.Scaled(P, -1));
        Assert.True(s.Equals(new CoordinateSystem(P, Up, Direction3D.UnitX)));

        var f2 = new Frame2D(new Axis2D(P2, new Direction2D(1, 0)), true);
        Near(new Direction2D(0, 1), f2.YDirection);
        var left = new Frame2D(new Axis2D(P2, new Direction2D(1, 0)), false);
        Near(new Direction2D(0, -1), left.YDirection);
        Near(new Direction2D(0, 1), f2.YAxis.Direction);
        var mirror = new Axis2D(new Point2D(0, 0), new Direction2D(1, 0));
        Near(new Point2D(1, -2), f2.Mirrored(mirror).Location);
        Near(new Point2D(-1, -2), f2.Mirrored(new Point2D(0, 0)).Location);
        Near(new Point2D(-2, 1), f2.Rotated(new Point2D(0, 0), Math.PI / 2).Location);
        Near(new Point2D(-1, -2), f2.Scaled(new Point2D(0, 0), -1).Location);
        Near(new Point2D(2, 2), f2.Transformed(Transform2D.Translation(new Vector2D(1, 0))).Location);
        Near(new Point2D(2, 3), f2.Translated(new Vector2D(1, 1)).Location);
        Near(new Point2D(2, 3), f2.Translated(new Point2D(0, 0), new Point2D(1, 1)).Location);
        Near(new Point2D(9, 9), f2.WithLocation(new Point2D(9, 9)).Location);
        Near(new Direction2D(0, 1), f2.WithAxis(new Frame2D(P2, new Direction2D(0, 1))).XDirection);
        Assert.True(f2 == new Frame2D(new Axis2D(P2, new Direction2D(1, 0)), true));
        Assert.True(f2 != left);
        Assert.True(f2.Equals(new Frame2D(new Axis2D(P2, new Direction2D(1, 0)), true)));
    }

    // ---- Vectors, directions and matrices ----

    [Fact]
    public void Two_dimensional_vectors_and_directions_by_their_arithmetic()
    {
        var a = new Vector2D(3, 4);
        var b = new Vector2D(1, 2);
        Assert.Equal(new Vector2D(4, 6), a.Added(b));
        Assert.Equal(2, a.CrossMagnitude(b), 12);
        Assert.Equal(4, a.CrossSquareMagnitude(b), 12);
        Assert.Equal(new Vector2D(6, 8), a.Multiplied(2));
        Assert.Equal(new Vector2D(6, 8), 2 * a);
        Assert.Equal(new Vector2D(-3, -4), -a);
        Vector2D turned = a.Rotated(Math.PI / 2);
        Assert.Equal(-4, turned.X, 12);
        Assert.Equal(3, turned.Y, 12);
        Vector2D moved = a.Transformed(Transform2D.Rotation(new Point2D(5, 5), Math.PI));
        Assert.Equal(-3, moved.X, 12);
        Assert.Equal(new Vector2D(9, 4), a.WithX(9));
        Assert.Equal(new Vector2D(3, 9), a.WithY(9));
        Assert.True(a == new Vector2D(3, 4));
        Assert.True(a != b);
        Assert.True(a.Equals(new Vector2D(3, 4)));

        var x = new Direction2D(1, 0);
        var y = new Direction2D(0, 1);
        Near(new Direction2D(-1, 0), x.Reversed());
        Assert.Equal(1, x ^ y, 12);
        Assert.Equal(0, x * y, 12);
        Assert.True(x != y);
        Assert.True(x.Equals(new Direction2D(5, 0)));

        var p = new Point2D(1, 2);
        Assert.True(p == new Point2D(1, 2));
        Assert.True(p != new Point2D(2, 1));
        Assert.True(p.Equals(new Point2D(1, 2)));
    }

    [Fact]
    public void Three_dimensional_vectors_by_their_arithmetic_and_linear_forms()
    {
        var a = new Vector3D(1, 2, 3);
        var b = new Vector3D(4, 5, 6);
        var c = new Vector3D(7, 8, 10);
        Assert.Equal(new Vector3D(5, 7, 9), a.Added(b));
        Assert.Equal(new Vector3D(9, 2, 3), a.WithX(9));
        Assert.Equal(new Vector3D(1, 9, 3), a.WithY(9));
        Assert.Equal(new Vector3D(1, 2, 9), a.WithZ(9));
        Assert.Equal(a.Crossed(b).SquareMagnitude, a.CrossSquareMagnitude(b), 9);
        Assert.Equal(new Vector3D(5, 7, 9), Vector3D.LinearForm(a, b));
        Assert.Equal(new Vector3D(6, 9, 12), Vector3D.LinearForm(2, a, b));
        Assert.Equal(2 * a + 3 * b + c, Vector3D.LinearForm(2, a, 3, b, c));
        Assert.Equal(2 * a + 3 * b + 4 * c, Vector3D.LinearForm(2, a, 3, b, 4, c));
        Assert.Equal(2 * a + 3 * b + 4 * c + a, Vector3D.LinearForm(2, a, 3, b, 4, c, a));
        var x = new Vector3D(1, 0, 0);
        var y = new Vector3D(0, 1, 0);
        var z = new Vector3D(0, 0, 1);
        Assert.Equal(Math.PI / 2, x.AngleWithRef(y, z), 12);
        Assert.Equal(-Math.PI / 2, x.AngleWithRef(y, -z), 12);
        Assert.Equal(1, Direction3D.UnitX.DotCross(Direction3D.UnitY, Direction3D.UnitZ), 12);
        Near(Direction3D.UnitX, Direction3D.UnitX.Mirrored(Direction3D.UnitX));
        Assert.True(Direction3D.UnitX != Direction3D.UnitY);
    }

    [Fact]
    public void Matrix_arithmetic_by_worked_values()
    {
        var a = new Matrix3x3(1, 2, 3, 4, 5, 6, 7, 8, 10);
        var b = new Matrix3x3(0, 1, 0, 1, 0, 0, 0, 0, 1);
        Assert.Equal(new Matrix3x3(1, 3, 3, 5, 5, 6, 7, 8, 11), a.Added(b));
        Assert.Equal(a.Added(b), a + b);
        Assert.Equal(new Matrix3x3(1, 1, 3, 3, 5, 6, 7, 8, 9), a.Subtracted(b));
        Assert.Equal(new Matrix3x3(2, 4, 6, 8, 10, 12, 14, 16, 20), a.Multiplied(2));
        Assert.Equal(a, new Matrix3x3(2, 4, 6, 8, 10, 12, 14, 16, 20).Divided(2));
        Assert.ThrowsAny<ArgumentException>(() => a.Divided(0));
        // pre-multiplying by a swap swaps rows; post-multiplying swaps columns
        Assert.Equal(new Matrix3x3(4, 5, 6, 1, 2, 3, 7, 8, 10), a.PreMultiplied(b));
        Assert.Equal(new Matrix3x3(2, 1, 3, 5, 4, 6, 8, 7, 10), a.Multiplied(b));
        // powers, inverse ones too
        Assert.Equal(a.Multiplied(a), a.Powered(2));
        Assert.Equal(Matrix3x3.Identity, b.Powered(2));
        Assert.Equal(b, b.Powered(-1));
        Matrix3x3 id = a.Powered(-1).Multiplied(a);
        for (int r = 1; r <= 3; r++)
        {
            for (int c = 1; c <= 3; c++)
            {
                Assert.Equal(r == c ? 1 : 0, Cell(id, r, c), 9);
            }
        }
        Assert.Equal(new Matrix3x3(9, 2, 3, 4, 8, 6, 7, 8, 7), a.WithDiagonal(9, 8, 7));
        Assert.Equal(new Matrix3x3(3, 0, 0, 0, 3, 0, 0, 0, 3), a.WithScale(3));
        Span<double> into = stackalloc double[9];
        a.CopyTo(into);
        Assert.Equal(1, into[0]);
        Assert.Equal(10, into[8]);
        Assert.Throws<ArgumentException>(() => { Span<double> shortOne = new double[8]; a.CopyTo(shortOne); });
        Assert.True(a != b);
        Assert.True(a.Equals(new Matrix3x3(1, 2, 3, 4, 5, 6, 7, 8, 10)));
    }

    // ---- Transformations ----

    [Fact]
    public void Transform_builders_agree_with_what_they_say()
    {
        // reflection in a plane through the origin normal to... an axis: 180 degrees about it
        Point3D about = Transform3D.Identity.WithMirror(AboutZ) * new Point3D(1, 2, 3);
        Near(new Point3D(-1, -2, 3), about);
        Near(new Point3D(2, 2, 3), Transform3D.Identity.WithTranslation(new Point3D(0, 0, 0), new Point3D(1, 0, 0)) * P);
        // coordinates in one system into those in another
        var from = new CoordinateSystem(P, Up, Direction3D.UnitX);
        Near(new Point3D(-1, -2, -3), Transform3D.Identity.WithTransformation(CoordinateSystem.World, from) * new Point3D(0, 0, 0));
        Near(new Point3D(1, 2, 3), Transform3D.Identity.WithTransformation(from, CoordinateSystem.World) * new Point3D(0, 0, 0));

        Transform3D values = Transform3D.Identity.WithValues(0, -1, 0, 5, 1, 0, 0, 6, 0, 0, 1, 7);
        Near(new Point3D(5 - 2, 6 + 1, 7 + 3), values * P);
        Assert.ThrowsAny<ArgumentException>(() => Transform3D.Identity.WithValues(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0));
        Assert.True(Quarter == Transform3D.Rotation(AboutZ, Math.PI / 2));
        Assert.True(Quarter != Transform3D.Identity);
        Assert.True(Quarter.Equals(Transform3D.Rotation(AboutZ, Math.PI / 2)));

        Transform2D turn = Transform2D.Rotation(new Point2D(0, 0), Math.PI / 2);
        Assert.Equal(Math.PI / 2, turn.RotationAngle, 12);
        Assert.False(turn.IsNegative);
        Assert.True(Transform2D.Scale(new Point2D(0, 0), -1).IsNegative == false);
        Assert.True(Transform2D.Identity.WithMirror(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0))).IsNegative);
        Transform2D twice = turn * turn;
        Assert.Equal(Math.PI, Math.Abs(twice.RotationAngle), 12);
        Assert.True(turn == Transform2D.Rotation(new Point2D(0, 0), Math.PI / 2));
        Assert.True(turn != twice);
        Assert.True(turn.Equals(Transform2D.Rotation(new Point2D(0, 0), Math.PI / 2)));
        Transform2D values2 = Transform2D.Identity.WithValues(0, -1, 5, 1, 0, 6);
        Near(new Point2D(5 - 2, 6 + 1), P2.Transformed(values2));
        // a change of 2D system: the origin of the target goes to zero
        Axis2D target = new(new Point2D(3, 4), new Direction2D(1, 0));
        Near(new Point2D(0, 0), new Point2D(3, 4).Transformed(Transform2D.Identity.WithTransformation(target)));
        Near(new Point2D(3, 4), new Point2D(0, 0).Transformed(Transform2D.Identity.WithTransformation(target, Axis2D.XAxis)));
    }

    [Fact]
    public void General_transforms_compose_and_refuse_what_is_not_affine()
    {
        ReadOnlySpan<double> values = [2, 0, 0, 1, 0, 3, 0, 2, 0, 0, 4, 3, 0, 0, 0, 1];
        GeneralTransform3D stretch = GeneralTransform3D.Mat4(values);
        Point3D moved = stretch * P;
        Near(new Point3D(3, 8, 15), moved);
        Assert.False(stretch.IsNegative);
        Assert.Equal(2, stretch[1, 1], 12);
        Assert.Equal(1, stretch[1, 4], 12);
        Assert.Equal(new Matrix3x3(2, 0, 0, 0, 3, 0, 0, 0, 4), stretch.VectorialPart);
        Assert.True(new GeneralTransform3D().WithMat4(values) == stretch);
        Assert.True(stretch != new GeneralTransform3D());
        Assert.True(stretch.Equals(GeneralTransform3D.Mat4(values)));
        // composed, applied right to left
        Near(stretch * (stretch * P), stretch.Multiplied(stretch) * P);
        Assert.True(new GeneralTransform3D().WithVectorialPart(new Matrix3x3(-1, 0, 0, 0, 1, 0, 0, 0, 1)).IsNegative);
        Assert.ThrowsAny<Exception>(() => { ReadOnlySpan<double> shortOne = [1, 2, 3]; GeneralTransform3D.Mat4(shortOne); });

        var flat = new GeneralTransform2D(Transform2D.Translation(new Vector2D(1, 2)));
        Assert.True(flat == new GeneralTransform2D(Transform2D.Translation(new Vector2D(1, 2))));
        Assert.True(flat != new GeneralTransform2D());
        Assert.True(flat.Equals(new GeneralTransform2D(Transform2D.Translation(new Vector2D(1, 2)))));
        Near(new Point2D(2, 4), P2.Translated(flat.ToTransform().TranslationPart));
        Assert.True(new GeneralTransform2D().WithTransform(Transform2D.Translation(new Vector2D(1, 2))) == flat);
    }
}
