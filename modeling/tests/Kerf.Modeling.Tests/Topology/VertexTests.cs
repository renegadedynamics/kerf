// Ported from build123d tests/test_direct_api/test_vertex.py. The convexity cases build boxes, sketches and
// extrusions, and follow with the 3D topology and the objects.
namespace Build123d.Tests.Topology;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

public class VertexTests
{
    [Fact]
    public void BasicVertex()
    {
        var v = Vertex.Make(0, 0, 0);
        Assert.Equal(0, v.X);
        Assert.True(default(Vertex).IsEmpty);

        v = Vertex.Make(1, 1, 1);
        Assert.Equal(1, v.X);
        Assert.IsType<Point3D>(v.Center());

        Check.AlmostEqual(new Point3D(1, 2, 3), (Vertex.Make(new Point3D(1, 2, 3)).Point), 7);
        Check.AlmostEqual(new Point3D(4, 5, 6), (Vertex.Make(new Point3D(4, 5, 6)).Point), 7);
        Check.AlmostEqual(new Point3D(8, 9, 0), (Vertex.Make(8, 9).Point), 7);
    }

    [Fact]
    public void CoordinatesIncludeLocation()
    {
        var vertex = Vertex.Make(1, 0, 0).Moved(new Location(new Point3D(1, 2, 3)));
        Assert.Equal((2.0, 2.0, 3.0), (vertex.X, vertex.Y, vertex.Z));
    }

    [Fact]
    public void VertexVolume() => Check.AlmostEqual(0, Vertex.Make(1, 1, 1).Volume, 5);

    [Fact]
    public void VertexAdd()
    {
        var v = Vertex.Make(0, 0, 0);
        Check.AlmostEqual(new Point3D(100, -40, 10), (v + new Vector3D(100, -40, 10)).Point, 7);
        Check.AlmostEqual(new Point3D(100, -40, 10), (v + Vertex.Make(100, -40, 10).Point.ToVector()).Point, 7);
    }

    [Fact]
    public void VertexSub()
    {
        var v = Vertex.Make(0, 0, 0);
        Check.AlmostEqual(new Point3D(-100, 40, -10), (v - new Vector3D(100, -40, 10)).Point, 7);
        Check.AlmostEqual(new Point3D(-100, 40, -10), (v - Vertex.Make(100, -40, 10).Point.ToVector()).Point, 7);
    }

    [Fact]
    public void VertexStr() => Assert.Equal("Vertex(0, 0, 0)", Vertex.Make(0, 0, 0).Describe());

    [Fact]
    public void NoIntersect() =>
        Assert.Throws<NotSupportedException>(() => Vertex.Make(1, 2, 3) & Vertex.Make(5, 6, 7));

    [Fact]
    public void ALoneVertexHasNothingToClassifyAgainst()
    {
        var e = Assert.Throws<InvalidOperationException>(() => default(Compound).ConvexityOf(Vertex.Make(1, 2, 3)));
        Assert.Contains("not selected from a shape", e.Message);
    }
}

/// <summary>Smoke tests of the topology core through edges and wires, ahead of the full test port.</summary>
public class EdgeWireSmokeTests
{
    [Fact]
    public void LineHasLengthEndsAndTangent()
    {
        var line = Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(3, 4, 0));
        Check.AlmostEqual(5, line.Length);
        Check.AlmostEqual(new Point3D(0, 0, 0), line.StartPoint());
        Check.AlmostEqual(new Point3D(3, 4, 0), line.EndPoint());
        Check.AlmostEqual(new Point3D(1.5, 2, 0), line.PositionAt(0.5));
        Check.AlmostEqual(new Vector3D(3, 4, 0).Normalized(), line.TangentAt(0.2));
        Assert.Equal(GeomType.Line, line.GeomType);
        Assert.Equal(2, line.Vertices().Count);
        Check.AlmostEqual(0.25, line.ParamAtPoint(new Point3D(0.75, 1, 0)));
    }

    [Fact]
    public void CircleHasRadiusAndCenter()
    {
        var circle = Edge.MakeCircle(2);
        Check.AlmostEqual(2, circle.Radius);
        Check.AlmostEqual(4 * Math.PI, circle.Length);
        Check.AlmostEqual(new Point3D(0, 0, 0), circle.ArcCenter);
        Assert.True(circle.IsClosed);
        var arc = Edge.MakeCircle(1, startAngle: 0, endAngle: 90);
        Check.AlmostEqual(new Point3D(1, 0, 0), arc.StartPoint());
        Check.AlmostEqual(new Point3D(0, 1, 0), arc.EndPoint());
    }

    [Fact]
    public void RectangleWireHasFourEdgesInOrder()
    {
        var rect = Wire.MakeRect(4, 2);
        var edges = rect.Edges();
        Assert.Equal(4, edges.Count);
        Assert.True(rect.IsClosed);
        Check.AlmostEqual(12, rect.Length);
        for (int i = 0; i < 4; i++)
            Check.AlmostEqual(edges[i].EndPoint(), edges[(i + 1) % 4].StartPoint());
        // Each edge knows the wire it was taken from
        Check.Same(rect, edges.Origin!);
        Assert.Equal(2, edges.ParallelTo(Axis.X).Count);
        Check.AlmostEqual(-1, edges.OrderBy(Axis.Y)[0].Center().Y);
    }

    [Fact]
    public void EdgesJoinIntoAWire()
    {
        var a = Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(1, 0, 0));
        var b = Edge.MakeLine(new Point3D(1, 0, 0), new Point3D(1, 1, 0));
        var joined = a + b;
        var wire = Check.Is<Wire>(joined);
        Assert.Equal(2, wire.Edges().Count);
        Check.AlmostEqual(2, wire.Length);
    }

    [Fact]
    public void MovedSharesGeometryAndKeepsClass()
    {
        var edge = new Curve((Shape)Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(1, 0, 0))).WithLabel("e");
        var moved = new Location(new Point3D(0, 0, 5)) * edge;
        Assert.Equal("e", moved.Label);
        Check.AlmostEqual(5, moved.Center().Z);
        Check.AlmostEqual(0, edge.Center().Z);   // the original is unchanged
        Assert.True(moved.Shape.IsPartner(edge.Shape));
    }

    [Fact]
    public void SplineThroughPointsAndTrim()
    {
        var spline = Edge.MakeSpline([new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 0, 0)]);
        Assert.Equal(GeomType.BSpline, spline.GeomType);
        Check.AlmostEqual(new Point3D(1, 1, 0), spline.PositionAt(spline.ParamAtPoint(new Point3D(1, 1, 0))), 6);
        var half = spline.Trim(0, 0.5);
        Check.AlmostEqual(spline.Length / 2, half.Length, 6);
    }

    [Fact]
    public void CommonPlaneOfAnArc()
    {
        var arc = Edge.MakeThreePointArc(new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 0, 0));
        var plane = arc.CommonPlane();
        Assert.NotNull(plane);
        Check.AlmostEqual(1, Math.Abs(plane!.ZDir.Z));
    }

    [Fact]
    public void LinesCrossAtAVertex()
    {
        var a = Edge.MakeLine(new Point3D(-1, 0, 0), new Point3D(1, 0, 0));
        var b = Edge.MakeLine(new Point3D(0, -1, 0), new Point3D(0, 1, 0));
        var hit = a.Intersect(b);
        var vertex = Check.Is<Vertex>(Assert.Single(hit));
        Check.AlmostEqual(new Point3D(0, 0, 0), vertex.Point);
    }
}
