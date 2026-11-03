// Ported from build123d tests/test_direct_api/test_axis.py. Cases that need topology (Edge, Vertex)
// follow with the topology port.
using Kerf.Geometry;
using Occt;

namespace Build123d.Tests.Geometry;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

public class AxisTests
{
    [Fact]
    public void Constructors()
    {
        var axis = new Axis((1, 2, 3), (0, 0, 1));
        Check.AlmostEqual((1, 2, 3), axis.Position, 5);
        Check.AlmostEqual((0, 0, 1), axis.Direction, 5);

        axis = Axis.FromPoints((1, 2, 3), (1, 2, 4));
        Check.AlmostEqual((1, 2, 3), axis.Position, 5);
        Check.AlmostEqual((0, 0, 1), axis.Direction, 5);

        axis = Axis.FromPoints((1, 2, 3), (2, 3, 3));
        Check.AlmostEqual((1, 2, 3), axis.Position, 5);
        Check.AlmostEqual(new Vector(1, 1, 0).Normalized(), axis.Direction, 5);

        Assert.Throws<ArgumentException>(() => Axis.FromPoints((1, 2, 3), (1, 2, 3)));
    }

    [Fact]
    public void FromOcct()
    {
        var p = new Point3D(1, 1, 1);
        var d = new Direction3D(0, 1, 0);
        var occt = new Axis3D(p, d);
        var axis = new Axis(occt);
        Check.AlmostEqual((1, 1, 1), axis.Position, 5);
        Check.AlmostEqual((0, 1, 0), axis.Direction, 5);
    }

    [Fact]
    public void Formatting()
    {
        Assert.Equal("((1.00, 2.00, 3.00), (0.46, 0.57, 0.68))", new Axis((1, 2, 3), (4, 5, 6)).ToString(".2f"));
        Assert.Equal("((1, 2, 3), (0.46, 0.57, 0.68))", new Axis((1, 2, 3), (4, 5, 6)).ToString(".2g"));
        Assert.Contains("((1.0, 2.0, 3.0), ", new Axis((1, 2, 3), (4, 5, 6)).ToString(".2t"));
        Assert.Equal("Axis: (position=(0, 0, 0), direction=(0, 1, 0))", Axis.Y.ToString());
    }

    [Fact]
    public void ToLocation()
    {
        var location = Axis.X.Location;
        Check.AlmostEqual((0, 0, 0), location.Position, 5);
        Check.AlmostEqual((0, 90, 180), location.Orientation, 5);
    }

    [Fact]
    public void Located()
    {
        var yAxis = Axis.Z.Located(new Location((0, 0, 1), (-90, 0, 0)));
        Check.AlmostEqual((0, 0, 1), yAxis.Position, 5);
        Check.AlmostEqual((0, 1, 0), yAxis.Direction, 5);
    }

    [Fact]
    public void FromLocation()
    {
        var axis = new Axis(new Location((1, 2, 3), (-90, 0, 0)));
        Check.AlmostEqual((1, 2, 3), axis.Position, 6);
        Check.AlmostEqual((0, 1, 0), axis.Direction, 6);
    }

    [Fact]
    public void IsCoaxial()
    {
        Assert.True(Axis.X.IsCoaxial(new Axis((0, 0, 0), (1, 0, 0))));
        Assert.False(Axis.X.IsCoaxial(new Axis((0, 0, 1), (1, 0, 0))));
        Assert.False(Axis.X.IsCoaxial(new Axis((0, 0, 0), (0, 1, 0))));
    }

    [Fact]
    public void IsNormal()
    {
        Assert.True(Axis.X.IsNormal(Axis.Y));
        Assert.False(Axis.X.IsNormal(Axis.X));
    }

    [Fact]
    public void IsOpposite()
    {
        Assert.True(Axis.X.IsOpposite(new Axis((1, 1, 1), (-1, 0, 0))));
        Assert.False(Axis.X.IsOpposite(Axis.X));
    }

    [Fact]
    public void IsParallel()
    {
        Assert.True(Axis.X.IsParallel(new Axis((1, 1, 1), (1, 0, 0))));
        Assert.False(Axis.X.IsParallel(Axis.Y));
    }

    [Fact]
    public void IsSkew()
    {
        Assert.True(Axis.X.IsSkew(new Axis((0, 1, 1), (0, 0, 1))));          // skew
        Assert.False(Axis.X.IsSkew(Axis.Y));                               // perpendicular, intersecting
        Assert.False(Axis.X.IsSkew(Axis.X));                               // coincident
        Assert.True(Axis.X.IsSkew(new Axis((0, 1, 0), (1, 0, 0))));          // parallel but distinct
        Assert.True(new Axis((0, 0, 0), (1, 1, 0)).IsSkew(new Axis((0, 1, 0), (1, 1, 0))));   // coplanar, not meeting
    }

    [Fact]
    public void AngleBetween()
    {
        Check.AlmostEqual(90, Axis.X.AngleBetween(Axis.Y), 5);
        Check.AlmostEqual(180, Axis.X.AngleBetween(new Axis((1, 1, 1), (-1, 0, 0))), 5);
    }

    [Fact]
    public void Reverse()
    {
        Check.AlmostEqual((-1, 0, 0), Axis.X.Reverse().Direction, 5);
        Check.AlmostEqual((-1, 0, 0), (-Axis.X).Direction, 5);
    }

    [Fact]
    public void Intersect()
    {
        Check.AlmostEqual((1, 0, 0), Assert.IsType<Vector>(Axis.X & new Axis((1, 0, 0), (0, 1, 0))), 5);
        Assert.Equal(Axis.X, Axis.X & new Axis((1, 0, 0), (1, 0, 0)));
        Assert.Null(Axis.X.Intersect(new Axis((0, 1, 1), (0, 0, 1))));   // skew

        Check.AlmostEqual((1, 2, 0), Assert.IsType<Vector>(new Axis((1, 2, 3), (0, 0, 1)) & Plane.XY), 5);

        var point = new Axis((0, 0, 1), (1, 1, 1)) & new Vector(0.5, 0.5, 1.5);
        Check.AlmostEqual((0.5, 0.5, 1.5), Assert.IsType<Vector>(point), 5);
        Assert.Null(Axis.Y & new Vector(2, 0, 0));
    }

    [Fact]
    public void IntersectLocation()
    {
        // A location on the axis pointing along it, at the end of the line (0,0,1)-(0,0,2)
        var onAxis = new Location((0, 0, 2));
        var location = Assert.IsType<Location>(Axis.Z & onAxis);
        Check.AlmostEqual(onAxis.Position, location.Position, 5);
        Check.AlmostEqual(onAxis.Orientation, location.Orientation, 5);

        // On the axis but pointing elsewhere: just the position
        Assert.IsType<Vector>(Axis.Z & new Location((0, 0, 1), (0, 90, 0)));
        Assert.Null(Axis.Z & new Location((1, 0, 2)));
    }

    [Fact]
    public void Equality()
    {
        Assert.Equal(Axis.X, Axis.X);
        Assert.Equal(Axis.Y, Axis.Y);
        Assert.Equal(Axis.Z, Axis.Z);
        Assert.NotEqual(Axis.X, Axis.Y);
        Assert.False(Axis.X.Equals(new object()));
    }

    [Fact]
    public void EqualAxesHashAlike()
    {
        var a0 = new Axis((0, 1, 2), (3, 4, 5));
        for (int i = 1; i < 8; i++)
        {
            for (int j = 1; j < 8; j++)
            {
                var a1 = new Axis(
                    (a0.Position.X + 1.0 / Math.Pow(10, i), a0.Position.Y, a0.Position.Z),
                    (a0.Direction.X + 1.0 / Math.Pow(10, j), a0.Direction.Y, a0.Direction.Z));
                Assert.Equal(a0 == a1 ? 1 : 2, new HashSet<Axis> { a0, a1 }.Count);
            }
        }
    }

    [Fact]
    public void WithPosition()
    {
        var axis = Axis.X.WithPosition((1, 2, 3));
        Check.AlmostEqual((1, 2, 3), axis.Position);
        axis = axis.WithPosition(axis.Position + (1, 2, 3));
        Check.AlmostEqual((2, 4, 6), axis.Position);
        var ax1 = axis.ToGpAx1();
        Check.AlmostEqual((2, 4, 6), new Axis(ax1).Position);
        Check.AlmostEqual((0, 0, 0), Axis.X.Position);   // the original is unchanged
    }

    [Fact]
    public void WithDirection()
    {
        var axis = Axis.X.WithDirection((1, 2, 3));
        Check.AlmostEqual(new Vector(1, 2, 3).Normalized(), axis.Direction);
        axis = axis.WithDirection(axis.Direction + (5, 3, 1));
        var expected = (new Vector(1, 2, 3).Normalized() + new Vector(5, 3, 1)).Normalized();
        Check.AlmostEqual(expected, axis.Direction);
        var ax1 = axis.ToGpAx1();
        Check.AlmostEqual(expected, new Axis(ax1).Direction);
    }
}
