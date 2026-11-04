// Ported from build123d tests/test_direct_api/test_vector.py. Cases that need topology (Vertex, Solid)
// follow with the topology port.
using Kerf.Geometry;

namespace Build123d.Tests.Geometry;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

public class VectorTests
{
    [Fact]
    public void Constructors()
    {
        var gpVec = new Vector3D(1, 2, 3);
        var gpPnt = new Point3D(1, 2, 3);
        Vector[] full = [new(1, 2, 3), (1, 2, 3), new(gpVec), new([1, 2, 3]), new(gpPnt)];
        foreach (var v in full)
            Check.AlmostEqual((1, 2, 3), v, 4);

        Vector[] twoD = [(1, 2), new([1, 2]), new(1, 2)];
        foreach (var v in twoD)
            Check.AlmostEqual((1, 2, 0), v, 4);

        Check.AlmostEqual((0, 0, 0), new Vector(), 4);
        Check.AlmostEqual((1, 2, 3), new Vector([1, 2, 3, 4]), 4);
        Check.AlmostEqual((1, 0, 0), new Vector([1]), 4);
    }

    [Fact]
    public void Rotate()
    {
        var x = new Vector(1, 0, 1).Rotate(Axis.X, 45);
        var y = new Vector(1, 2, 1).Rotate(Axis.Y, 45);
        var z = new Vector(-1, -1, 3).Rotate(Axis.Z, 45);
        Check.AlmostEqual((1, -Math.Sqrt(2) / 2, Math.Sqrt(2) / 2), x, 7);
        Check.AlmostEqual((Math.Sqrt(2), 2, 0), y, 7);
        Check.AlmostEqual((0, -Math.Sqrt(2), 3), z, 7);
    }

    [Fact]
    public void SignedAngle()
    {
        double a = Math.PI / 3;
        var v1 = new Vector(1, 0, 0);
        var v2 = new Vector(Math.Cos(a), -Math.Sin(a), 0);
        Check.AlmostEqual(a * 180 / Math.PI, v1.GetSignedAngle(v2));
        Check.AlmostEqual(-a * 180 / Math.PI, v1.GetSignedAngle(v2, new Vector(0, 0, 1)));
    }

    [Fact]
    public void Center()
    {
        var v = new Vector(1, 1, 1);
        Check.AlmostEqual(v, v.Center());
    }

    [Fact]
    public void Dot() => Assert.Equal(2.0, new Vector(2, 2, 2).Dot(new Vector(1, -1, 1)));

    [Fact]
    public void Add() => Check.AlmostEqual((1, 2, 3), new Vector(1, 2, 0) + new Vector(0, 0, 3), 3);

    [Fact]
    public void Operators()
    {
        Assert.Equal(new Vector(3, 3, 3), new Vector(1, 1, 1) + new Vector(2, 2, 2));
        Assert.Equal(new Vector(-2, 0, 2), new Vector(1, 2, 3) - new Vector(3, 2, 1));
        Assert.Equal(new Vector(2, 4, 6), new Vector(1, 2, 3) * 2);
        Assert.Equal(new Vector(3, 6, 9), 3 * new Vector(1, 2, 3));
        Assert.Equal(new Vector(1, 2, 3), new Vector(2, 4, 6) / 2);
        Assert.Equal(new Vector(-1, -1, -1), -new Vector(1, 1, 1));
        Assert.Equal(0, new Vector(0, 0, 0).Length);
        Assert.Equal(1, new Vector(1, 0, 0).Length);
        Assert.Equal(Math.Pow(1 + 4 + 9, 0.5), new Vector(1, 2, 3).Length);
    }

    [Fact]
    public void Equality()
    {
        var a = new Vector(1, 2, 3);
        Assert.Equal(a, new Vector(1, 2, 3));
        Assert.Equal(a, new Vector(1, 2, 3.000001));
        Assert.NotEqual(a, new Vector(3, 2, 1));
        Assert.False(a.Equals(new object()));
    }

    [Fact]
    public void EqualVectorsHashAlike()
    {
        // Equality and hashing agree, so sets work
        var a = new Vector(1, 2, 3);
        for (int i = 1; i < 8; i++)
        {
            var v = new Vector(a.X + 1.0 / Math.Pow(10, i), a.Y, a.Z);
            Assert.Equal(v == a ? 1 : 2, new HashSet<Vector> { a, v }.Count);
        }
    }

    [Fact]
    public void DistanceToPlanes()
    {
        var v = new Vector(1, 2, 3);
        Check.AlmostEqual(1, v.SignedDistanceFromPlane(Plane.YZ));
        Check.AlmostEqual(2, v.SignedDistanceFromPlane(Plane.ZX));
        Check.AlmostEqual(3, v.SignedDistanceFromPlane(Plane.XY));
        Check.AlmostEqual(-1, v.SignedDistanceFromPlane(Plane.ZY));
        Check.AlmostEqual(-2, v.SignedDistanceFromPlane(Plane.XZ));
        Check.AlmostEqual(-3, v.SignedDistanceFromPlane(Plane.YX));

        Check.AlmostEqual(1, v.DistanceToPlane(Plane.YZ));
        Check.AlmostEqual(2, v.DistanceToPlane(Plane.ZX));
        Check.AlmostEqual(3, v.DistanceToPlane(Plane.XY));
        Check.AlmostEqual(1, v.DistanceToPlane(Plane.ZY));
        Check.AlmostEqual(2, v.DistanceToPlane(Plane.XZ));
        Check.AlmostEqual(3, v.DistanceToPlane(Plane.YX));
    }

    [Fact]
    public void Project()
    {
        const int places = 9;
        var point = new Vector(10, 11, 12).ProjectToPlane(new Plane((5, 7, 9), (1, 0, 0), (1, 2, 3)));
        Check.AlmostEqual((59.0 / 7, 55.0 / 7, 51.0 / 7), point, places);

        var vec = new Vector(10, 10, 10);
        var line = new Vector(3, 4, 5);
        double angle = vec.GetAngle(line) * Math.PI / 180;
        var projection = vec.ProjectToLine(line);
        Check.AlmostEqual(line.Normalized(), projection.Normalized(), places);
        Check.AlmostEqual(vec.Length * Math.Cos(angle), projection.Length, places);
    }

    [Fact]
    public void Formatting()
    {
        Assert.Equal("(1.000, 2.000, 3.000)", new Vector(1, 2, 3).ToString("F3"));
        Assert.Equal("(1, 2, 3)", new Vector(1, 2, 3).ToString("G3"));
        Assert.Equal("(1, 2, 3)", new Vector(1, 2, 3).ToString(null));
        Assert.Equal("Vector: (X=1, Y=2, Z=3)", new Vector(1, 2, 3).ToString());
        Assert.Equal("Vector: (X=10, Y=-23.65, Z=0)",
                     new Vector(9.99999999999999, -23.649999999999995, -7.37188088351e-15).ToString());
    }

    [Fact]
    public void Deconstruct()
    {
        var (x, y, z) = new Vector(1, 2, 3);
        Assert.Equal(6, x + y + z);
    }

    [Fact]
    public void Reverse() => Check.AlmostEqual((-1, -2, -3), new Vector(1, 2, 3).Reverse(), 7);

    [Fact]
    public void Sum()
    {
        Vector[] vectors = [new(1, 2, 3), new(4, 5, 6), new(7, 8, 9)];
        Check.AlmostEqual((12, 15, 18), vectors.Aggregate((a, b) => a + b), 5);
    }

    [Fact]
    public void Hash()
    {
        Vector[] vectors = [new(1, 2, 3), new(4, 5, 6), new(7, 8, 9), new(1, 2, 3)];
        Assert.Equal(3, vectors.ToHashSet().Count);
    }

    [Fact]
    public void Transform()
    {
        var a = new Vector(1, 2, 3);
        var pxy = Plane.XY;
        var pxyO1 = Plane.XY.Offset(1);
        Assert.Equal(a, a.Transform(pxy.ForwardTransform));
        Assert.Equal(a.Normalized(), a.Transform(pxy.ForwardTransform, isDirection: true));
        Assert.Equal(new Vector(1, 2, 2), a.Transform(pxyO1.ForwardTransform));
        Assert.Equal(a.Normalized(), a.Transform(pxyO1.ForwardTransform, isDirection: true));

        // Matrices that aren't similarities apply too
        var stretch = new Matrix(new double[,] { { 2, 0, 0, 1 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 } });
        Check.AlmostEqual((3, 2, 3), a.Transform(stretch), 7);
        Check.AlmostEqual(new Vector(2, 2, 3).Normalized(), a.Transform(stretch, isDirection: true), 7);
        Assert.Equal(new Vector(1, 2, 4), a.Transform(pxyO1.ReverseTransform));
        Assert.Equal(a.Normalized(), a.Transform(pxyO1.ReverseTransform, isDirection: true));
    }

    [Fact]
    public void Intersect()
    {
        var v1 = new Vector(1, 2, 3);
        Check.AlmostEqual((1, 2, 3), (v1 & new Vector(1, 2, 3))!.Value, 5);
        Assert.Null(v1 & new Vector(0, 0, 0));

        Check.AlmostEqual((1, 2, 3), (v1 & new Location((1, 2, 3)))!.Value, 5);
        Assert.Null(v1 & new Location());

        Check.AlmostEqual((1, 2, 3), (v1 & new Axis((1, 2, 3), (1, 0, 0)))!.Value, 5);
        Assert.Null(v1 & Axis.X);

        Check.AlmostEqual((1, 2, 3), (v1 & new Plane((1, 2, 3)))!.Value, 5);
        Assert.Null(v1 & Plane.XY);
    }

    [Fact]
    public void NormalizingZeroThrows() =>
        Assert.Throws<InvalidOperationException>(() => new Vector().Normalized());
}
