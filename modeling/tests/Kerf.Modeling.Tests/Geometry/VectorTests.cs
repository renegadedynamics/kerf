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
        // A position and an offset convert to each other through the origin
        var gpVec = new Vector3D(1, 2, 3);
        var gpPnt = new Point3D(1, 2, 3);
        Point3D[] full = [new(1, 2, 3), gpVec.ToPoint(), gpPnt];
        foreach (var v in full)
            Check.AlmostEqual(new Point3D(1, 2, 3), v, 4);

        Check.AlmostEqual(new Point3D(0, 0, 0), new Point3D(), 4);
        Check.AlmostEqual(gpVec, gpPnt.ToVector(), 4);
        Check.AlmostEqual(new Vector3D(1, 0, 0), new Direction3D(5, 0, 0), 4);
        Check.AlmostEqual(new Vector3D(2, 0, 0), new Direction3D(1, 0, 0) * 2, 4);
    }

    [Fact]
    public void Rotate()
    {
        var x = new Vector3D(1, 0, 1).Rotate(Axis.X, 45);
        var y = new Vector3D(1, 2, 1).Rotate(Axis.Y, 45);
        var z = new Vector3D(-1, -1, 3).Rotate(Axis.Z, 45);
        Check.AlmostEqual(new Vector3D(1, -Math.Sqrt(2) / 2, Math.Sqrt(2) / 2), x, 7);
        Check.AlmostEqual(new Vector3D(Math.Sqrt(2), 2, 0), y, 7);
        Check.AlmostEqual(new Vector3D(0, -Math.Sqrt(2), 3), z, 7);
    }

    [Fact]
    public void SignedAngle()
    {
        double a = Math.PI / 3;
        var v1 = new Vector3D(1, 0, 0);
        var v2 = new Vector3D(Math.Cos(a), -Math.Sin(a), 0);
        Check.AlmostEqual(a * 180 / Math.PI, v1.GetSignedAngle(v2));
        Check.AlmostEqual(-a * 180 / Math.PI, v1.GetSignedAngle(v2, new Vector3D(0, 0, 1)));
    }

    [Fact]
    public void Center()
    {
        var v = new Point3D(1, 1, 1);
        Check.AlmostEqual(v, v.Center());
    }

    [Fact]
    public void Dot() => Assert.Equal(2.0, new Vector3D(2, 2, 2).Dot(new Vector3D(1, -1, 1)));

    [Fact]
    public void Add() => Check.AlmostEqual(new Point3D(1, 2, 3), new Point3D(1, 2, 0) + new Vector3D(0, 0, 3), 3);

    [Fact]
    public void Operators()
    {
        Check.Near(new Point3D(3, 3, 3), new Point3D(1, 1, 1) + new Vector3D(2, 2, 2));
        Check.Near(new Vector3D(-2, 0, 2), new Point3D(1, 2, 3) - new Point3D(3, 2, 1));
        Check.Near(new Vector3D(2, 4, 6), new Vector3D(1, 2, 3) * 2);
        Check.Near(new Vector3D(3, 6, 9), 3 * new Vector3D(1, 2, 3));
        Check.Near(new Vector3D(1, 2, 3), new Vector3D(2, 4, 6) / 2);
        Check.Near(new Vector3D(-1, -1, -1), -new Vector3D(1, 1, 1));
        Assert.Equal(0, new Vector3D(0, 0, 0).Magnitude);
        Assert.Equal(1, new Vector3D(1, 0, 0).Magnitude);
        Assert.Equal(Math.Pow(1 + 4 + 9, 0.5), new Vector3D(1, 2, 3).Magnitude);
    }

    [Fact]
    public void Equality()
    {
        // The layer's tolerance has a name; the kernel's equality is exact
        var a = new Point3D(1, 2, 3);
        Assert.True(a.IsNear(new Point3D(1, 2, 3)));
        Assert.True(a.IsNear(new Point3D(1, 2, 3.000001)));
        Assert.NotEqual(a, new Point3D(1, 2, 3.000001));
        Assert.False(a.IsNear(new Point3D(3, 2, 1)));
        Assert.False(a.Equals(new object()));
        Assert.True(new Vector3D(1, 2, 3).IsNear(new Vector3D(1, 2, 3.000001)));
        Assert.True(new Direction3D(1, 0, 0).IsNear(new Direction3D(1, 1e-7, 0)));
    }

    [Fact]
    public void EqualVectorsHashAlike()
    {
        // Equality and hashing agree, so sets work
        var a = new Point3D(1, 2, 3);
        for (int i = 1; i < 8; i++)
        {
            var v = new Point3D(a.X + 1.0 / Math.Pow(10, i), a.Y, a.Z);
            Assert.Equal(v.IsNear(a) ? 1 : 2, new HashSet<Point3D>(PointComparer.Near) { a, v }.Count);
        }
    }

    [Fact]
    public void DistanceToPlanes()
    {
        var v = new Point3D(1, 2, 3);
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
        var point = new Point3D(10, 11, 12).ProjectToPlane(new Plane(new Point3D(5, 7, 9), new Vector3D(1, 0, 0), new Vector3D(1, 2, 3)));
        Check.AlmostEqual(new Point3D(59.0 / 7, 55.0 / 7, 51.0 / 7), point, places);

        var vec = new Vector3D(10, 10, 10);
        var line = new Vector3D(3, 4, 5);
        double angle = vec.GetAngle(line) * Math.PI / 180;
        var projection = vec.ProjectToLine(line);
        Check.AlmostEqual(line.Normalized(), projection.Normalized(), places);
        Check.AlmostEqual(vec.Magnitude * Math.Cos(angle), projection.Magnitude, places);
    }

    [Fact]
    public void Formatting()
    {
        Assert.Equal("(1.000, 2.000, 3.000)", new Point3D(1, 2, 3).Describe("F3"));
        Assert.Equal("(1, 2, 3)", new Point3D(1, 2, 3).Describe("G3"));
        Assert.Equal("(1, 2, 3)", new Point3D(1, 2, 3).Describe(null));
        Assert.Equal("Point: (X=1, Y=2, Z=3)", new Point3D(1, 2, 3).Describe());
        Assert.Equal("Vector: (X=10, Y=-23.65, Z=0)",
                     new Vector3D(9.99999999999999, -23.649999999999995, -7.37188088351e-15).Describe());
    }

    [Fact]
    public void Deconstruct()
    {
        var (x, y, z) = new Point3D(1, 2, 3);
        Assert.Equal(6, x + y + z);
    }

    [Fact]
    public void Reverse() => Check.AlmostEqual(new Vector3D(-1, -2, -3), new Vector3D(1, 2, 3).Reversed(), 7);

    [Fact]
    public void Sum()
    {
        Vector3D[] vectors = [new(1, 2, 3), new(4, 5, 6), new(7, 8, 9)];
        Check.AlmostEqual(new Vector3D(12, 15, 18), vectors.Aggregate((a, b) => a + b), 5);
    }

    [Fact]
    public void Hash()
    {
        Point3D[] vectors = [new(1, 2, 3), new(4, 5, 6), new(7, 8, 9), new(1, 2, 3)];
        Assert.Equal(3, vectors.ToHashSet(PointComparer.Near).Count);
    }

    [Fact]
    public void Transform()
    {
        var a = new Vector3D(1, 2, 3);
        var pxy = Plane.XY;
        var pxyO1 = Plane.XY.Offset(1);
        Check.Near(a, a.Transform(pxy.ForwardTransform));
        Check.Near(a.Normalized(), a.Transform(pxy.ForwardTransform, isDirection: true));
        Check.Near(new Vector3D(1, 2, 2), a.Transform(pxyO1.ForwardTransform));
        Check.Near(a.Normalized(), a.Transform(pxyO1.ForwardTransform, isDirection: true));

        // Matrices that aren't similarities apply too
        var stretch = new Matrix(new double[,] { { 2, 0, 0, 1 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 } });
        Check.AlmostEqual(new Vector3D(3, 2, 3), a.Transform(stretch), 7);
        Check.AlmostEqual(new Vector3D(2, 2, 3).Normalized(), a.Transform(stretch, isDirection: true), 7);
        Check.Near(new Vector3D(1, 2, 4), a.Transform(pxyO1.ReverseTransform));
        Check.Near(a.Normalized(), a.Transform(pxyO1.ReverseTransform, isDirection: true));
    }

    [Fact]
    public void Intersect()
    {
        var v1 = new Point3D(1, 2, 3);
        Check.AlmostEqual(new Point3D(1, 2, 3), (v1.Intersect(new Point3D(1, 2, 3)))!.Value, 5);
        Assert.Null(v1.Intersect(new Point3D(0, 0, 0)));

        Check.AlmostEqual(new Point3D(1, 2, 3), (v1.Intersect(new Location(new Point3D(1, 2, 3))))!.Value, 5);
        Assert.Null(v1.Intersect(new Location()));

        Check.AlmostEqual(new Point3D(1, 2, 3), (v1.Intersect(new Axis(new Point3D(1, 2, 3), new Vector3D(1, 0, 0))))!.Value, 5);
        Assert.Null(v1.Intersect(Axis.X));

        Check.AlmostEqual(new Point3D(1, 2, 3), (v1.Intersect(new Plane(new Point3D(1, 2, 3))))!.Value, 5);
        Assert.Null(v1.Intersect(Plane.XY));
    }

    [Fact]
    public void NormalizingZeroThrows() =>
        Assert.Throws<InvalidOperationException>(() => new Vector3D().Normalized());
}
