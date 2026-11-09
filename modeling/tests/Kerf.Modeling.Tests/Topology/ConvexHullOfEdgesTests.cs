// Wire.MakeConvexHull (decision 1997): lines add their ends, curves are sampled to tolerance times their length, and each
// joining line is made to touch its curves. Checked against closed forms, and on random edges against a bracket that
// doesn't depend on how the hull was built: it holds the hull of 4,000 points along each edge, and lies within that
// hull grown by its sampling's sagitta.
using Kerf.Geometry;

namespace Kerf.Modeling.Tests.Topology;

using Kerf.Modeling;

public class ConvexHullOfEdgesTests
{
    private static double Area(IEnumerable<Edge> edges) => Face.Make(Wire.MakeConvexHull(edges)).Area;

    [Fact]
    public void TwoCircles_IsTheClosedForm()
    {
        // Circles of radius 10 and 7, 20 apart: the outer tangents' points and the two arcs between them
        double r1 = 10, r2 = 7, d = 20, a = Math.PI / 2 - Math.Asin((r1 - r2) / d);
        double trapezoid = (r1 * Math.Sin(a) + r2 * Math.Sin(a)) * (d + r2 * Math.Cos(a) - r1 * Math.Cos(a));
        double segments = r1 * r1 / 2 * (2 * Math.PI - 2 * a - Math.Sin(2 * Math.PI - 2 * a)) + r2 * r2 / 2 * (2 * a - Math.Sin(2 * a));
        Edge[] edges = [Edge.MakeCircle(r1, new Plane(new Point3D(-10, 0, 0))), Edge.MakeCircle(r2, new Plane(new Point3D(10, 0, 0)))];
        Assert.Equal(trapezoid + segments, Area(edges), 1e-9);
    }

    [Fact]
    public void ArcsAndALine_MeetingAtEnds_IsTheClosedForm()
    {
        // Two arcs of radius 10 meeting at (10, 0), and a line: the hull's corners are the arcs' far ends
        Edge[] edges =
        [
            Edge.MakeCircle(10, endAngle: 45), Edge.MakeCircle(10, startAngle: 315, endAngle: 360),
            Edge.MakeLine(new Point3D(-10, 10, 0), new Point3D(-10, -10, 0)),
        ];
        double c = 10 * Math.Sqrt(0.5);
        Point3D[] corners = [new(-10, -10, 0), new(c, -c, 0), new(c, c, 0), new(-10, 10, 0)];
        double shoelace = 0.5 * Math.Abs(Enumerable.Range(0, 4).Sum(i => corners[i].X * corners[(i + 1) % 4].Y - corners[(i + 1) % 4].X * corners[i].Y));
        Assert.Equal(shoelace + 50 * (Math.PI / 2 - 1), Area(edges), 1e-9);
    }

    [Fact]
    public void PointAndCircle_TouchesExactly()
    {
        // A line's end 30 from a circle of radius 10's centre: the tangents from it meet the circle at acos(1/3)
        var hull = Wire.MakeConvexHull([Edge.MakeCircle(10), Edge.MakeLine(new Point3D(30, 0, 0), new Point3D(30, 0.5, 0))]);
        double angle = Math.Acos(10.0 / 30);
        foreach (var edge in hull.Edges().Where(e => e.GeomType == GeomType.Line && e.Length > 1))
        {
            // Each tangent line is perpendicular to the radius where it touches
            var touch = new[] { edge.PositionAt(0), edge.PositionAt(1) }.MinBy(p => Math.Abs(p.ToVector().Magnitude - 10));
            Assert.Equal(10, touch.ToVector().Magnitude, 1e-9);
            Assert.Equal(0, touch.ToVector().Normalized().Dot(edge.TangentAt(0)), 1e-9);
            Assert.Equal(angle, Math.Abs(Math.Atan2(touch.Y, touch.X)), 0.02);   // from (30, 0) or (30, 0.5): near acos(1/3)
        }
    }

    [Fact]
    public void ManyLines_IsTheirPolygon()
    {
        var ring = Enumerable.Range(0, 120).Select(i => new Point3D(100 * Math.Cos(i * Math.PI / 60), 60 * Math.Sin(i * Math.PI / 60), 0)).ToList();
        var lines = Enumerable.Range(0, 120).Select(i => Edge.MakeLine(ring[i], ring[(i + 1) % 120])).ToList();
        double polygon = 0.5 * Enumerable.Range(0, 120).Sum(i => ring[i].X * ring[(i + 1) % 120].Y - ring[(i + 1) % 120].X * ring[i].Y);
        var hull = Wire.MakeConvexHull(lines);
        Assert.Equal(120, hull.Edges().Count);
        Assert.Equal(polygon, Face.Make(hull).Area, 1e-9);
    }

    private static (double Area, double Perimeter) DenseHull(List<Edge> edges, double deflection)
    {
        // The kernel's deflection sampling and 4,000 even steps of the curve's parameter: the first alone can miss a
        // tight lobe (decision 1997)
        var dense = edges.SelectMany(e =>
        {
            var curve = e.AsCurve();
            var range = curve.Range;
            return e.Positions(deflection).Concat(Enumerable.Range(0, 4000).Select(i => curve.Evaluate(range.Min + (range.Max - range.Min) * i / 3999)));
        }).Select(p => (p.X, p.Y)).ToList();
        var cycle = ConvexHull2D.Simplices(dense);
        double area = 0.5 * cycle.Sum(s => dense[s.Item1].X * dense[s.Item2].Y - dense[s.Item2].X * dense[s.Item1].Y);
        double perimeter = cycle.Sum(s => Math.Sqrt(Math.Pow(dense[s.Item2].X - dense[s.Item1].X, 2) + Math.Pow(dense[s.Item2].Y - dense[s.Item1].Y, 2)));
        return (area, perimeter);
    }

    /// <summary>Sets of two to six edges: circles, arcs, lines and splines, apart or touching end to end.</summary>
    internal static List<List<Edge>> RandomSets(int seed, int sets)
    {
        var random = new Random(seed);
        var all = new List<List<Edge>>();
        for (int k = 0; k < sets; k++)
        {
            var edges = new List<Edge>();
            int count = 2 + random.Next(5);
            Point3D R() => new(random.NextDouble() * 100, random.NextDouble() * 60, 0);
            for (int i = 0; i < count; i++)
            {
                var p = i > 0 && random.Next(2) == 0 ? edges[^1].PositionAt(1) : R();
                edges.Add(random.Next(4) switch
                {
                    0 => Edge.MakeCircle(1 + random.NextDouble() * 20, new Plane(p)),
                    1 => Edge.MakeThreePointArc(p, R(), R()),
                    2 => Edge.MakeLine(p, R()),
                    _ => Edge.MakeSpline([p, R(), R(), R()]),
                });
            }
            all.Add(edges);
        }
        return all;
    }

    [Theory]
    [InlineData(1, 40)]
    [InlineData(2, 40)]
    [InlineData(3, 40)]
    public void RandomEdges_LieWithinTheBracket(int seed, int sets)
    {
        if (Environment.GetEnvironmentVariable("HULL_SETS") is { } more)
            sets = int.Parse(more);
        var wrong = new List<string>();
        foreach (var (edges, k) in RandomSets(seed, sets).Select((e, i) => (e, i)))
        {
            double hull = Area(edges);
            // The bracket: the hull of points along each edge to a deflection of 1e-5 is inside the true hull, and the
            // true hull is within 1e-5 of it, so within 1e-5 times its perimeter in area
            var (inner, perimeter) = DenseHull(edges, 1e-5);
            if (hull < inner - 1e-9)
                wrong.Add($"set {k}: {hull:R} under the samples' {inner:R}");
            if (hull > inner + 1e-5 * perimeter + 1e-9)
                wrong.Add($"set {k}: {hull:R} over {inner:R} + {1e-5 * perimeter:G3}");
        }
        Assert.Empty(wrong);
    }
}
