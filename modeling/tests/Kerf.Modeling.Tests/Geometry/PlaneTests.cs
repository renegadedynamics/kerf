// Ported from build123d tests/test_direct_api/test_plane.py. Cases that need topology (Face, Solid,
// Vertex) or builders follow with those ports. Python's origin/x_dir/y_dir setters and move() change the
// plane; the port's WithOrigin/WithXDir/WithYDir and Moved return new planes.
using Kerf.Geometry;

namespace Kerf.Modeling.Tests.Geometry;

using Kerf.Modeling;

public class PlaneTests
{
    static readonly double R2 = Math.Sqrt(2) / 2;

    public static TheoryData<string, Vector3D, Vector3D> StandardPlanes => new()
    {
        { "XY", new Vector3D(1, 0, 0), new Vector3D(0, 0, 1) },
        { "YZ", new Vector3D(0, 1, 0), new Vector3D(1, 0, 0) },
        { "ZX", new Vector3D(0, 0, 1), new Vector3D(0, 1, 0) },
        { "XZ", new Vector3D(1, 0, 0), new Vector3D(0, -1, 0) },
        { "YX", new Vector3D(0, 1, 0), new Vector3D(0, 0, -1) },
        { "ZY", new Vector3D(0, 0, 1), new Vector3D(-1, 0, 0) },
        { "Front", new Vector3D(1, 0, 0), new Vector3D(0, -1, 0) },
        { "Back", new Vector3D(-1, 0, 0), new Vector3D(0, 1, 0) },
        { "Left", new Vector3D(0, -1, 0), new Vector3D(-1, 0, 0) },
        { "Right", new Vector3D(0, 1, 0), new Vector3D(1, 0, 0) },
        { "Top", new Vector3D(1, 0, 0), new Vector3D(0, 0, 1) },
        { "Bottom", new Vector3D(1, 0, 0), new Vector3D(0, 0, -1) },
        { "Isometric", new Vector3D(1 / Math.Sqrt(2), 1 / Math.Sqrt(2), 0), new Vector3D(1 / Math.Sqrt(3), -1 / Math.Sqrt(3), 1 / Math.Sqrt(3)) },
    };

    [Theory]
    [MemberData(nameof(StandardPlanes))]
    public void StandardPlane(string name, Vector3D xDir, Vector3D zDir)
    {
        var plane = (Plane)typeof(Plane).GetProperty(name)!.GetValue(null)!;
        Check.AlmostEqual(xDir, plane.XDir, 5);
        Check.AlmostEqual(zDir, plane.ZDir, 5);
    }

    [Fact]
    public void FromOrigin()
    {
        Point3D o = new Point3D(0, 0, 0);
        Vector3D x = new Vector3D(1, 0, 0), y = new Vector3D(0, 1, 0), z = new Vector3D(0, 0, 1);
        Plane[] planes = [new(o), new(o, x), new(o, x, z), new(o, x, zDir: z), new(o, xDir: x), new(o, zDir: z)];
        foreach (var p in planes)
        {
            Check.AlmostEqual(o, p.Origin, 6);
            Check.AlmostEqual(x, p.XDir, 6);
            Check.AlmostEqual(y, p.YDir, 6);
            Check.AlmostEqual(z, p.ZDir, 6);
        }
    }

    [Fact]
    public void FromLocation()
    {
        // Rotated about Z
        var loc = new Location(new Point3D(0, 0, 0), new Vector3D(0, 0, 45));
        var p = new Plane(loc);
        Check.AlmostEqual(new Point3D(0, 0, 0), p.Origin, 6);
        Check.AlmostEqual(new Vector3D(R2, R2, 0), p.XDir, 6);
        Check.AlmostEqual(new Vector3D(-R2, R2, 0), p.YDir, 6);
        Check.AlmostEqual(new Vector3D(0, 0, 1), p.ZDir, 6);
        Check.AlmostEqual(loc.Position, p.Location.Position, 6);
        Check.AlmostEqual(loc.Orientation, p.Location.Orientation, 6);

        // Rotated about X, away from the origin
        loc = new Location(new Point3D(0, 2, -1), new Vector3D(45, 0, 0));
        p = new Plane(loc);
        Check.AlmostEqual(new Point3D(0, 2, -1), p.Origin, 6);
        Check.AlmostEqual(new Vector3D(1, 0, 0), p.XDir, 6);
        Check.AlmostEqual(new Vector3D(0, R2, R2), p.YDir, 6);
        Check.AlmostEqual(new Vector3D(0, -R2, R2), p.ZDir, 6);
        Check.AlmostEqual(loc.Position, p.Location.Position, 6);
        Check.AlmostEqual(loc.Orientation, p.Location.Orientation, 6);
    }

    [Fact]
    public void ThroughPoints()
    {
        var p = Plane.ThroughPoints(new Point3D(0, 1, 2), new Point3D(-3, 4, 5), new Point3D(6, 7, -8));
        Check.AlmostEqual(new Point3D(0, 1, 2), p.Origin, 6);
        Check.AlmostEqual(new Vector3D(-3, 3, 3).Normalized(), p.XDir, 6);
        Check.AlmostEqual(new Vector3D(-3, 3, 3).Cross(new Vector3D(6, 6, -10)).Normalized(), p.ZDir, 6);

        Assert.Throws<ArgumentException>(() => Plane.ThroughPoints(new Point3D(0, 0, 0), new Point3D(1, 1, 1), new Point3D(2, 2, 2)));
    }

    [Fact]
    public void FromAxis()
    {
        var origin = new Point3D(1, 2, 3);
        var plane = new Plane(new Axis(origin, new Vector3D(0, 0, 1)));
        Assert.Equal(origin, plane.Origin);
        Check.AlmostEqual(1.0, plane.XDir.ToVector().Magnitude, 12);
        Check.AlmostEqual(1.0, plane.YDir.ToVector().Magnitude, 12);
        Check.AlmostEqual(1.0, plane.ZDir.ToVector().Magnitude, 12);

        plane = new Plane(new Axis(new Point3D(0, 0, 0), new Vector3D(0, 0, 1)), new Vector3D(1, 0, 0));
        Check.Near(new Point3D(0, 0, 0), plane.Origin);
        Check.Near(new Vector3D(0, 0, 1), plane.ZDir);
        Check.Near(new Vector3D(1, 0, 0), plane.XDir);
        Check.Near(new Vector3D(0, 0, 1).Cross(new Vector3D(1, 0, 0)).Normalized(), plane.YDir);

        plane = new Plane(new Axis(new Point3D(0, 0, 0), new Vector3D(0, 1, 0)), xDir: new Vector3D(1, 0, 0));
        Check.Near(new Vector3D(0, 1, 0), plane.ZDir);
        Check.Near(new Vector3D(1, 0, 0), plane.XDir);

        plane = new Plane(new Axis(new Point3D(0, 0, 0), new Vector3D(1, 0, 0)));
        Check.Near(new Vector3D(1, 0, 0), plane.ZDir);
        Assert.True(plane.ZDir.Cross(plane.XDir).Dot(plane.YDir) > 0.99);

        var zero = Assert.Throws<ArgumentException>(() => new Plane(new Axis(new Point3D(0, 0, 0), new Vector3D(0, 0, 1)), xDir: new Vector3D(0, 0, 0)));
        Assert.Equal("A plane's directions can't have zero length", zero.Message);
    }

    [Fact]
    public void LeftHandedPlanesAreMadeRightHanded()
    {
        var mirrorAxes = new Frame3D();
        var xy = Plane.XY.ToPlane3D();
        var mirrored = xy.Mirrored(mirrorAxes);
        var position = mirrored.Position;
        Assert.False(position.IsDirect);

        var p = new Plane(mirrored);
        var pln = p.ToPlane3D();
        var fixedPosition = pln.Position;
        Assert.True(fixedPosition.IsDirect);
    }

    [Fact]
    public void WithOrigin()
    {
        var p0 = new Plane(new Point3D(1, 2, 3), new Vector3D(4, 5, 6), new Vector3D(7, 8, 9));
        var p = p0.WithOrigin(new Point3D(10, 11, 12));
        Check.AlmostEqual(new Point3D(10, 11, 12), p.Origin);
        Check.AlmostEqual(p0.ZDir, p.ZDir);
        Check.AlmostEqual(p0.XDir, p.XDir);
    }

    [Fact]
    public void WithXDir()
    {
        var p0 = Plane.XY.Offset(-1);
        var p = p0.WithXDir(new Vector3D(1, 2, 0));
        Check.AlmostEqual(new Vector3D(1, 2, 0).Normalized(), p.XDir);
        Check.AlmostEqual(p0.Origin, p.Origin);
        Check.AlmostEqual(p0.ZDir, p.ZDir);
    }

    [Fact]
    public void WithYDir()
    {
        var p0 = Plane.XY.Offset(-1);
        var p = p0.WithYDir(new Vector3D(-2, 1, 0));
        Check.AlmostEqual(new Vector3D(-2, 1, 0).Normalized(), p.YDir);
        Check.AlmostEqual(p0.Origin, p.Origin);
        Check.AlmostEqual(p0.ZDir, p.ZDir);
    }

    [Fact]
    public void Negate()
    {
        var p = new Plane(new Point3D(1, 2, 3), new Vector3D(1, 2, 3).Normalized(), new Vector3D(4, 5, 6).Normalized());
        foreach (var reversed in new[] { -p, p.Reverse() })
        {
            Check.AlmostEqual(p.Origin, reversed.Origin, 6);
            Check.AlmostEqual(p.XDir, reversed.XDir, 6);
            Check.AlmostEqual(-p.ZDir, reversed.ZDir, 6);
            Check.AlmostEqual((-p.ZDir).Cross(p.XDir).Normalized(), reversed.YDir, 6);
        }
    }

    [Fact]
    public void MultiplyByLocation()
    {
        var p = new Plane(new Point3D(1, 2, 3), new Vector3D(1, 0, 0), new Vector3D(0, 0, 1));
        var p2 = new Location(new Point3D(1, 2, -1), new Vector3D(0, 0, 45)) * p;
        Check.AlmostEqual(new Point3D(2, 4, 2), p2.Origin, 6);
        Check.AlmostEqual(new Vector3D(R2, R2, 0), p2.XDir, 6);
        Check.AlmostEqual(new Vector3D(-R2, R2, 0), p2.YDir, 6);
        Check.AlmostEqual(new Vector3D(0, 0, 1), p2.ZDir, 6);

        p2 = new Location(new Point3D(1, 2, -1), new Vector3D(0, 45, 0)) * p;
        Check.AlmostEqual(new Point3D(2, 4, 2), p2.Origin, 6);
        Check.AlmostEqual(new Vector3D(R2, 0, -R2), p2.XDir, 6);
        Check.AlmostEqual(new Vector3D(0, 1, 0), p2.YDir, 6);
        Check.AlmostEqual(new Vector3D(R2, 0, R2), p2.ZDir, 6);

        p2 = new Location(new Point3D(1, 2, -1), new Vector3D(45, 0, 0)) * p;
        Check.AlmostEqual(new Point3D(2, 4, 2), p2.Origin, 6);
        Check.AlmostEqual(new Vector3D(1, 0, 0), p2.XDir, 6);
        Check.AlmostEqual(new Vector3D(0, R2, R2), p2.YDir, 6);
        Check.AlmostEqual(new Vector3D(0, -R2, R2), p2.ZDir, 6);
    }

    [Fact]
    public void MultiplyByLists()
    {
        var p = Plane.XY;
        Location l1 = new Pos(1, 2, 3), l2 = new Pos(4, 5, 6);
        Assert.Equal([p * l1, p * l2], p * new[] { l1, l2 });
        Assert.Equal([l1 * p, l2 * p], new[] { l1, l2 } * p);

        Plane p1 = Plane.XZ.Offset(1), p2 = Plane.ZY.Offset(-2);
        Assert.Equal([p * p1, p * p2], p * new[] { p1, p2 });
        // Planes multiplying a plane act as locations
        Assert.Equal([new Location(p1) * p, new Location(p2) * p], new[] { p1, p2 } * p);
    }

    [Fact]
    public void Formatting()
    {
        // The directions given aren't orthogonal, so x_dir doesn't show as given
        Assert.Equal("((1.00, 2.00, 3.00), (-0.76, -0.06, 0.64), (0.50, 0.57, 0.65))",
                     new Plane(new Point3D(1, 2, 3), new Vector3D(4, 5, 6), new Vector3D(7, 8, 9)).ToString("F2"));
        Assert.Equal("((1, 2, 3), (-0.76, -0.059, 0.64), (0.5, 0.57, 0.65))",
                     new Plane(new Point3D(1, 2, 3), new Vector3D(4, 5, 6), new Vector3D(7, 8, 9)).ToString("G2"));
        Assert.Contains("((1, 2, 3), ", new Plane(new Point3D(1, 2, 3), new Vector3D(4, 5, 6), new Vector3D(7, 8, 9)).ToString(null));
        Assert.Equal("Plane: (origin=(0, 0, 0), x_dir=(1, 0, 0), z_dir=(0, 0, 1))", Plane.XY.ToString());
    }

    [Fact]
    public void ShiftOriginByLocalCoordinates()
    {
        var plane = new Plane(new Point3D(10, 20, 30), xDir: new Vector3D(0, 1, 0), zDir: new Vector3D(1, 0, 0));
        foreach (var (point, expected) in new[] { (new Point3D(0.0, 0.0, 0), new Point3D(10, 20, 30)), (new Point3D(1, 2, 0), new Point3D(10, 21, 32)), (new Point3D(-2, 3, 0), new Point3D(10, 18, 33)) })
        {
            var shifted = plane.ShiftOriginBy(point.X, point.Y);
            Assert.Equal(expected, shifted.Origin);
            Assert.Equal(plane.XDir, shifted.XDir);
            Assert.Equal(plane.ZDir, shifted.ZDir);
        }
        Check.Near(new Point3D(10, 20, 30), plane.Origin);

        var iso = Plane.Isometric.Offset(10);
        var isoShifted = iso.ShiftOriginBy(1, 2);
        Assert.Equal(iso.Origin + iso.XDir + 2 * iso.YDir, isoShifted.Origin);
        Assert.Equal(iso.XDir, isoShifted.XDir);
        Assert.Equal(iso.ZDir, isoShifted.ZDir);

        Check.Near(new Point3D(2, 1, 0), Plane.YX.ShiftOriginBy(1, 2).Origin);
        Check.Near(new Point3D(1, 2, 0), Plane.XY.ShiftOriginBy(1, 2).Origin);
    }

    [Fact]
    public void ShiftOriginToGlobalPoints()
    {
        var plane = new Plane(new Point3D(10, 20, 30), xDir: new Vector3D(0, 1, 0), zDir: new Vector3D(1, 0, 0));
        Check.Near(new Point3D(10, 1, 2), plane.ShiftOrigin(new Point3D(10, 1, 2)).Origin);
        Assert.Throws<ArgumentException>(() => plane.ShiftOrigin(new Point3D(1, 2, 0)));
        Assert.Throws<ArgumentException>(() => plane.ShiftOrigin(new Point3D(1, 2, 0)));
    }

    [Fact]
    public void ShiftOriginErrors()
    {
        Assert.Throws<ArgumentException>(() => Plane.XY.ShiftOrigin(new Point3D(1, 1, 1)));
        Assert.Throws<ArgumentException>(() => Plane.XY.ShiftOrigin(new Axis(new Point3D(0, 0, 1), new Vector3D(0, 1, 0))));
    }

    [Fact]
    public void ShiftOriginToAnAxis()
    {
        var shifted = Plane.XY.Offset(2).ShiftOrigin(new Axis(new Point3D(1, 1, 0), new Vector3D(0, 0, 1)));
        Check.AlmostEqual(new Point3D(1, 1, 2), shifted.Origin, 6);
    }

    [Fact]
    public void Moved()
    {
        var original = Plane.XY;
        var pln = original.Moved(new Location(new Point3D(1, 2, 3)));
        Check.AlmostEqual(new Point3D(1, 2, 3), pln.Origin, 5);
        Check.AlmostEqual(new Point3D(0, 0, 0), original.Origin, 5);

        Assert.Equal(Plane.XY.Moved(Plane.XZ.Offset(1)), Plane.XY.Moved(Plane.XZ.Offset(1).Location));
    }

    [Fact]
    public void Rotated()
    {
        var rotated = Plane.XY.Offset(2).Rotated(new Vector3D(45, 0, 0));
        Check.AlmostEqual(new Vector3D(1, 0, 0), rotated.XDir, 5);
        Check.AlmostEqual(new Vector3D(0, -R2, R2), rotated.ZDir, 5);
        Check.AlmostEqual(new Point3D(0, 0, 2), rotated.Origin, 5);
    }

    [Fact]
    public void RotatedAboutThePlanesOwnAxes()
    {
        // A turn about the normal keeps the plane where it is
        var turned = Plane.XZ.Rotated(new Vector3D(0, 0, 90));
        Check.AlmostEqual(Plane.XZ.ZDir, turned.ZDir, 5);
        Check.AlmostEqual(new Vector3D(0, 0, 1), turned.XDir, 5);
        Check.AlmostEqual(new Vector3D(-1, 0, 0), turned.YDir, 5);

        // A turn about the plane's x tips the normal towards the plane's y
        var tipped = Plane.YZ.Rotated(new Vector3D(90, 0, 0));
        Check.AlmostEqual(Plane.YZ.XDir, tipped.XDir, 5);
        Check.AlmostEqual(new Vector3D(0, 0, -1), tipped.ZDir, 5);

        // The origin stays put
        var elsewhere = Plane.XZ.Offset(5).Rotated(new Vector3D(0, 0, 90));
        Check.AlmostEqual(Plane.XZ.Offset(5).Origin, elsewhere.Origin, 5);
        Check.AlmostEqual(new Vector3D(0, 0, 1), elsewhere.XDir, 5);

        // ZYX reads the angles as (z, y, x)
        Assert.Equal(Plane.XZ.Rotated(new Vector3D(90, 0, 0)), Plane.XZ.Rotated(new Vector3D(0, 0, 90), Intrinsic.ZYX));

        foreach (RotationOrder ordering in new RotationOrder[] { Intrinsic.XYZ, Extrinsic.XYZ, Intrinsic.ZYX })
            Assert.Equal(new Plane(Plane.YZ.Location * new Rotation(30, 20, 10, ordering)),
                         Plane.YZ.Rotated(new Vector3D(30, 20, 10), ordering));
    }

    [Fact]
    public void InvalidPlanes()
    {
        Assert.Throws<ArgumentException>(() => new Plane(new Point3D(0, 0, 0), new Vector3D(0, 0, 0), new Vector3D(0, 1, 1)));
        Assert.Throws<ArgumentException>(() => new Plane(new Point3D(0, 0, 0), new Vector3D(1, 0, 0), new Vector3D(0, 0, 0)));
    }

    [Fact]
    public void Equality()
    {
        Assert.Equal(new Plane(new Point3D(0, 0, 0), new Vector3D(1, 0, 0), new Vector3D(0, 0, 1)), new Plane(new Point3D(0, 0, 0), new Vector3D(1, 0, 0), new Vector3D(0, 0, 1)));
        Assert.Equal(new Plane(new Point3D(2, 1, -1), new Vector3D(1, 0, 0), new Vector3D(0, 0, 1)), new Plane(new Point3D(2, 1, -1), new Vector3D(1, 0, 0), new Vector3D(0, 0, 1)));
        Assert.Equal(new Plane(new Point3D(0, 0, 0), new Vector3D(1, 1, 0), new Vector3D(0, 0, 1)), new Plane(new Point3D(0, 0, 0), new Vector3D(1, 1, 0), new Vector3D(0, 0, 1)));
        Assert.Equal(new Plane(new Point3D(0, 0, 0), new Vector3D(1, 0, 0), new Vector3D(0, 1, 1)), new Plane(new Point3D(0, 0, 0), new Vector3D(1, 0, 0), new Vector3D(0, 1, 1)));

        var xy = new Plane(new Point3D(0, 0, 0), new Vector3D(1, 0, 0), new Vector3D(0, 0, 1));
        Assert.False(xy.Equals(null));
        Assert.NotEqual(xy, new Plane(new Point3D(0, 0, 1), new Vector3D(1, 0, 0), new Vector3D(0, 0, 1)));
        Assert.NotEqual(xy, new Plane(new Point3D(0, 0, 0), new Vector3D(1, 1, 0), new Vector3D(0, 0, 1)));
        Assert.NotEqual(xy, new Plane(new Point3D(0, 0, 0), new Vector3D(1, 0, 0), new Vector3D(0, 1, 1)));
    }

    [Fact]
    public void EqualPlanesHashAlike()
    {
        var plane = new Plane(new Point3D(0, 1, 2), new Vector3D(1, 1, 0), new Vector3D(0, 0, 1));
        var equal = new Plane(
            new Point3D(plane.Origin.X + 1e-7, plane.Origin.Y, plane.Origin.Z),
            new Vector3D(plane.XDir.X + 1e-7, plane.XDir.Y, plane.XDir.Z),
            new Vector3D(plane.ZDir.X + 1e-7, plane.ZDir.Y, plane.ZDir.Z));
        Assert.Equal(plane, equal);
        Assert.Equal(plane.GetHashCode(), equal.GetHashCode());
        Assert.Single(new HashSet<Plane> { plane, equal });

        var unequal = new Plane(
            new Point3D(plane.Origin.X + 1e-4, plane.Origin.Y, plane.Origin.Z),
            new Vector3D(plane.XDir.X + 1e-4, plane.XDir.Y, plane.XDir.Z),
            new Vector3D(plane.ZDir.X + 1e-4, plane.ZDir.Y, plane.ZDir.Z));
        Assert.NotEqual(plane, unequal);
        Assert.Equal(2, new HashSet<Plane> { plane, unequal }.Count);
    }

    [Fact]
    public void ToLocation()
    {
        var loc = new Plane(new Point3D(1, 2, 3), new Vector3D(0, 1, 0), new Vector3D(0, 0, 1)).Location;
        Check.AlmostEqual(new Point3D(1, 2, 3), loc.Position, 5);
        Check.AlmostEqual(new Vector3D(0, 0, 90), loc.Orientation, 5);
    }

    [Fact]
    public void Intersect()
    {
        Check.AlmostEqual(new Point3D(1, 2, 0), Assert.IsType<Point3D>(Plane.XY.Intersect(new Axis(new Point3D(1, 2, 3), new Vector3D(0, 0, -1)))), 5);
        Assert.Null(Plane.XY.Intersect(new Axis(new Point3D(1, 2, 3), new Vector3D(0, 1, 0))));
        Assert.Equal(Axis.X, Plane.XY.Intersect(Plane.XZ));
        Assert.Null(Plane.XY.Intersect(Plane.XY.Offset(1)));

        Assert.Equal(Axis.X, Plane.XY.Intersect(Plane.XZ));
        Check.AlmostEqual(new Point3D(1, 2, 0), (Plane.XY.Intersect(new Point3D(1, 2, 0)))!.Value, 5);

        var a = new Axis(new Point3D(0, 0, 0), new Vector3D(1, 1, 0));
        Assert.Equal(a, Assert.IsType<Axis>(Plane.XY.Intersect(a)));

        a = new Axis(new Point3D(1, 2, -1), new Vector3D(0, 0, 1));
        Check.AlmostEqual(new Point3D(1, 2, 0), Assert.IsType<Point3D>(Plane.XY.Intersect(a)), 5);
    }

    [Fact]
    public void FromXAndYDirections()
    {
        var p = Plane.FromXAndYDirections(new Point3D(1, 2, 3), new Vector3D(2, 0, 0), new Vector3D(0, 3, 0));   // not unit on purpose
        Check.AlmostEqual(new Point3D(1, 2, 3), p.Origin, 6);
        Check.AlmostEqual(new Vector3D(1, 0, 0), p.XDir, 6);
        Check.AlmostEqual(new Vector3D(0, 1, 0), p.YDir, 6);
        Check.AlmostEqual(new Vector3D(0, 0, 1), p.ZDir, 6);
        Check.AlmostEqual(p.XDir.Cross(p.YDir).Normalized(), p.ZDir, 6);
        Check.AlmostEqual(p.ZDir.Cross(p.XDir).Normalized(), p.YDir, 6);

        Assert.Throws<ArgumentException>(() => Plane.FromXAndYDirections(new Point3D(0, 0, 0), new Vector3D(0, 0, 0), new Vector3D(0, 1, 0)));
        Assert.Throws<ArgumentException>(() => Plane.FromXAndYDirections(new Point3D(0, 0, 0), new Vector3D(1, 0, 0), new Vector3D(0, 0, 0)));
        Assert.Throws<ArgumentException>(() => Plane.FromXAndYDirections(new Point3D(0, 0, 0), new Vector3D(1, 0, 0), new Vector3D(2, 0, 0)));
    }

    [Fact]
    public void LocalCoordinates()
    {
        var random = new Random(42);
        double x = random.NextDouble(), y = random.NextDouble(), z = random.NextDouble();
        Check.AlmostEqual(new Point3D(y, z, x), Plane.YZ.ToLocalCoords(new Point3D(x, y, z)), 5);
        Check.AlmostEqual(new Point3D(x, y, z), Plane.YZ.FromLocalCoords(Plane.YZ.ToLocalCoords(new Point3D(x, y, z))), 9);
    }
}
