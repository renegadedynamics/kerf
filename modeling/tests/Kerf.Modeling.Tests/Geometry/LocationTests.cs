// Ported from build123d tests/test_direct_api/test_location.py and test_rotation.py. Cases that need
// topology (Edge, Solid) or builders (GridLocations) follow with those ports; argument-type validation
// is the compiler's job in C#.
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;

namespace Build123d.Tests.Geometry;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

public class LocationTests
{
    static Vector Translation(Location loc)
    {
        var trsf = loc.ToGpTrsf();
        var t = trsf.TranslationXyz();
        return new Vector(t);
    }

    static double RotationAngle(Location loc)
    {
        var trsf = loc.ToGpTrsf();
        var q = trsf.GetRotation();
        return q.GetRotationAngle() * 180 / Math.PI;
    }

    [Fact]
    public void Constructors()
    {
        var loc0 = new Location();
        Check.AlmostEqual((0, 0, 0), Translation(loc0), 5);
        Check.AlmostEqual(0, RotationAngle(loc0));

        Check.AlmostEqual((0, 0, 1), Translation(new Location((0, 0, 1))), 5);
        var loc1 = new Location(new Vector(0, 0, 1));
        Check.AlmostEqual((0, 0, 1), Translation(loc1), 5);

        // Rotation and translation
        var loc2 = new Location(new Vector(0, 0, 1), new Vector(0, 0, 1), 45);
        Check.AlmostEqual(45, RotationAngle(loc2));

        // gp_Trsf
        var t = new Transform3D();
        var tz = new Vector3D(0, 0, 1);
        t = t.WithTranslation(tz);
        var loc3 = new Location(t);
        Assert.Equal(Translation(loc1).Z, Translation(loc3).Z);

        var identity = new Transform3D();
        var (position, orientation) = new Location(identity);
        Check.AlmostEqual((0, 0, 0), position, 5);
        Check.AlmostEqual((0, 0, 0), orientation, 5);
    }

    [Fact]
    public void Composition()
    {
        var loc1 = new Location(new Vector(0, 0, 1));
        var loc4 = new Location((0, 0, 0), new Vector(0, 0, 1), 15);

        var loc5 = loc1 * loc4;
        Check.AlmostEqual((0, 0, 1), Translation(loc5), 5);
        Check.AlmostEqual(15, RotationAngle(loc5));
        Check.AlmostEqual(30, RotationAngle(loc4 * loc4));
        Check.AlmostEqual(30, RotationAngle(loc4.Pow(2)));
    }

    [Fact]
    public void EulerAnglesMatchTheRotationMatrix()
    {
        var random = new Random(1234);
        for (int n = 0; n < 20; n++)
        {
            double aboutX = random.NextDouble() * 4 * Math.PI - 2 * Math.PI;
            double aboutY = random.NextDouble() * 4 * Math.PI - 2 * Math.PI;
            double aboutZ = random.NextDouble() * 4 * Math.PI - 2 * Math.PI;

            var origin = new Point3D(0, 0, 0);
            var rotX = Rotate(origin, (1, 0, 0), aboutX);
            var rotY = Rotate(origin, (0, 1, 0), aboutY);
            var rotZ = Rotate(origin, (0, 0, 1), aboutZ);
            var xy = rotX.Multiplied(rotY);
            var xyz = xy.Multiplied(rotZ);
            var loc1 = new Location(xyz);

            var q = new Quaternion3();
            q = q.WithEulerAngles(EulerSequence.Intrinsic_XYZ, aboutX, aboutY, aboutZ);
            var t = new Transform3D();
            t = t.WithRotationPart(q);
            var loc2 = new Location(t);

            Check.AlmostEqual(loc1.Position, loc2.Position, 5);
            Check.AlmostEqual(loc1.Orientation, loc2.Orientation, 5);
        }

        static Transform3D Rotate(Point3D origin, Vector dir, double angle)
        {
            var trsf = new Transform3D();
            var d = dir.ToGpDir();
            var ax = new Axis3D(origin, d);
            trsf = trsf.WithRotation(ax, angle);
            return trsf;
        }
    }

    [Fact]
    public void PositionAndAngle()
    {
        var loc1 = new Location((1, 2), 34);
        Check.AlmostEqual((1, 2, 0), loc1.Position, 5);
        Check.AlmostEqual((0, 0, 34), loc1.Orientation, 5);

        var angles = new Vector(-115.00, 35.00, -135.00);
        var loc2 = new Location((1, 2, 3), angles);
        Check.AlmostEqual((1, 2, 3), loc2.Position, 5);
        Check.AlmostEqual(angles, loc2.Orientation, 5);

        var loc3 = new Location(loc2);
        Check.AlmostEqual((1, 2, 3), loc3.Position, 5);
        Check.AlmostEqual(angles, loc3.Orientation, 5);
    }

    [Fact]
    public void Parameters()
    {
        Check.AlmostEqual((10, 20, 30), new Location((10, 20, 30)).Position, 5);

        var loc = new Location((10, 20, 30), (10, 20, 30));
        Check.AlmostEqual((10, 20, 30), loc.Position, 5);
        Check.AlmostEqual((10, 20, 30), loc.Orientation, 5);

        loc = new Location((10, 20, 30), (10, 20, 30), Intrinsic.XYZ);
        Check.AlmostEqual((10, 20, 30), loc.Orientation, 5);

        loc = new Location((10, 20, 30), (30, 20, 10), Extrinsic.ZYX);
        Check.AlmostEqual((10, 20, 30), loc.Position, 5);
        Check.AlmostEqual((10, 20, 30), loc.Orientation, 5);

        loc = new Location((10, 20, 30), (90, 0, 90), Extrinsic.XYZ);
        Check.AlmostEqual((0, 90, 90), loc.Orientation, 5);

        loc = new Location(Plane.Isometric);
        Check.AlmostEqual((0, 0, 0), loc.Position, 5);
        Check.AlmostEqual((45.00, 35.26, 30.00), loc.Orientation, 2);

        Check.AlmostEqual((0, 0, 0), new Location(new Location()).Position, 5);
    }

    [Fact]
    public void Formatting()
    {
        Assert.Equal("((1.00, 2.00, 3.00), (4.00, 5.00, 6.00))", new Location((1, 2, 3), (4, 5, 6)).ToString(".2f"));
        Assert.Equal("((1, 2, 3), (4, 5, 6))", new Location((1, 2, 3), (4, 5, 6)).ToString(".2g"));
        Assert.Contains("((1.0, 2.0, 3.0), ", new Location((1, 2, 3), (4, 5, 6)).ToString(".2t"));
        Assert.Equal("Location: (position=(0, 0, 0), orientation=(0, 0, 0))", new Location().ToString());
        Assert.Equal("Location: (position=(1, 2, 3), orientation=(33, 45, 67))",
                     new Location((1, 2, 3), (33, 45, 67)).ToString());
    }

    [Fact]
    public void Inverse() => Check.AlmostEqual((-90, 0, 0), new Location(Plane.XZ).Inverse().Orientation, 6);

    [Fact]
    public void WithPosition()
    {
        var original = new Location(Plane.XZ);
        var loc = original.WithPosition((1, 2, 3));
        Check.AlmostEqual((1, 2, 3), loc.Position, 6);
        Check.AlmostEqual((90, 0, 0), loc.Orientation, 6);
        Check.AlmostEqual((0, 0, 0), original.Position, 6);   // unchanged
    }

    [Fact]
    public void WithOrientation()
    {
        var loc = new Location((1, 2, 3), (90, 0, 0)).WithOrientation((-90, 0, 0));
        Check.AlmostEqual((1, 2, 3), loc.Position, 6);
        Check.AlmostEqual((-90, 0, 0), loc.Orientation, 6);
    }

    [Fact]
    public void Equality()
    {
        Assert.Equal(new Location((1, 2, 3), (4, 5, 6)), new Location((1, 2, 3), (4, 5, 6)));

        var loc = new Location((1, 2, 3), (40, 50, 60));
        Assert.NotEqual(loc, new Location((3, 2, 1), (40, 50, 60)));
        Assert.NotEqual(loc, new Location((1, 2, 3), (60, 50, 40)));
        Assert.False(loc.Equals(new object()));
    }

    [Fact]
    public void EqualLocationsHashAlike()
    {
        var l0 = new Location((0, 1, 2), (3, 4, 5));
        for (int i = 1; i < 8; i++)
        {
            for (int j = 1; j < 8; j++)
            {
                var l1 = new Location(
                    (l0.Position.X + 1.0 / Math.Pow(10, i), l0.Position.Y, l0.Position.Z),
                    (l0.Orientation.X + 1.0 / Math.Pow(10, j), l0.Orientation.Y, l0.Orientation.Z));
                Assert.Equal(l0 == l1 ? 1 : 2, new HashSet<Location> { l0, l1 }.Count);
            }
        }
    }

    [Fact]
    public void Negate()
    {
        var negated = -new Location((1, 2, 3), (0, 35, 127));
        Check.AlmostEqual((1, 2, 3), negated.Position, 5);
        Check.AlmostEqual((180, -35, -127), negated.Orientation, 5);
    }

    [Fact]
    public void Intersect()
    {
        // The end of the line (0,0,0)-(1,1,1), pointing along it
        var l1 = Axis.FromPoints((0, 0, 0), (1, 1, 1)).WithPosition((1, 1, 1)).Location;
        var l0 = l1.WithPosition((0, 0, 0));
        Assert.Null(l0 & l1);
        Assert.Equal(l1, l1 & l1);

        Check.AlmostEqual((1, 1, 1), Assert.IsType<Vector>(l1 & new Vector(1, 1, 1)), 5);
        Assert.Equal(l1, Assert.IsType<Location>(l1 & new Axis((0.5, 0.5, 0.5), (1, 1, 1))));

        var p = Plane.XY.Rotated((45, 0, 0)).ShiftOrigin(new Vector(1, 0, 0));
        var l = new Location((1, 0, 0), (1, 0, 0), 45);
        var i = Assert.IsType<Location>(l & p);
        Check.AlmostEqual((1, 0, 0), i.Position, 5);
        Check.AlmostEqual(l.Orientation, i.Orientation, 5);
    }

    [Fact]
    public void Pos()
    {
        Assert.Equal(new Vector(1, 2, 3), new Pos(1, 2, 3).Position);
        Assert.Equal(new Vector(1, 2, 3), new Pos((1, 2, 3)).Position);
        Assert.Equal(new Vector(1, 2, 3), new Pos(x: 1, y: 2, z: 3).Position);
        Assert.Equal(new Vector(1, 2, 3), new Pos(new Vector(1, 2, 3)).Position);
    }

    [Fact]
    public void Center() => Assert.Equal(new Vector(2, 4, 8), new Location((2, 4, 8), (1, 2, 3)).Center());

    [Fact]
    public void Mirror()
    {
        var mirrored = new Location((10, 0, 5), (30, 45, 60)).Mirror(Plane.YZ);
        Assert.Equal(new Vector(-10, 0, 5), mirrored.Position);

        // Still right-handed
        var plane = new Plane(mirrored);
        Assert.True(plane.XDir.Cross(plane.YDir).Dot(plane.ZDir) > 0.999);
    }

    [Fact]
    public void SameInstanceSharesOcctIdentity()
    {
        // OCCT compares locations by identity: copies of one Location share it, separate equal ones don't
        var loc = new Location((1, 2, 3));
        var a = loc.ToTopLoc();
        var b = new Location(loc).ToTopLoc();
        var c = new Location((1, 2, 3)).ToTopLoc();
        Assert.True(a.IsEqual(b));
        Assert.False(a.IsEqual(c));
    }
}

public class RotationTests
{
    [Fact]
    public void Parameters()
    {
        Check.AlmostEqual((10, 20, 30), new Rotation(10, 20, 30).Orientation, 5);
        Check.AlmostEqual((10, 20, 30), new Rotation(10, 20, z: 30).Orientation, 5);
        Check.AlmostEqual((10, 20, 30), new Rotation(10, 20, z: 30, ordering: Intrinsic.XYZ).Orientation, 5);
        Check.AlmostEqual((10, 20, 30), new Rotation((10, 20, 30)).Orientation, 5);
        Check.AlmostEqual((10, 20, 30), new Rotation(10, 20, 30, Intrinsic.XYZ).Orientation, 5);
        Check.AlmostEqual((10, 20, 30), new Rotation((30, 20, 10), Extrinsic.ZYX).Orientation, 5);
        Check.AlmostEqual((10, 20, 30), new Rotation((30, 20, 10), ordering: Extrinsic.ZYX).Orientation, 5);
    }

    [Fact]
    public void AboutAxis()
    {
        var r = new Rotation(Axis.Z, 45);
        Check.AlmostEqual((0, 0, 45), r.Orientation, 5);
        Check.AlmostEqual((0, 0, 0), new Rotation(Axis.Z, 0).Orientation, 5);
    }
}
