// ConvexPolyhedron (decision 1993): a valid solid for any point set whose hull has volume, one face for each flat side,
// and one exception type, saying why, for a set whose hull has none. Each hull is checked independently of how it was
// built: every corner is an input point, every input point is inside every face's plane (to the merge distance), each
// face is flat to 5e-8, and the solid is valid. Together those make it the hull, to that distance.
using System.Globalization;
using Kerf.Geometry;

namespace Kerf.Modeling.Tests.Objects;

using Kerf.Modeling;

public class ConvexPolyhedronTests
{
    private static readonly Point3D[] Cube =
        [new(0, 0, 0), new(1, 0, 0), new(1, 1, 0), new(0, 1, 0), new(0, 0, 1), new(1, 0, 1), new(1, 1, 1), new(0, 1, 1)];

    /// <summary>Checks the solid is the hull of the points, to <paramref name="slack"/>, and returns it.</summary>
    private static Solid CheckHull(IReadOnlyList<Point3D> points, double slack = Constants.Tolerance)
    {
        var part = new ConvexPolyhedron(points);
        var solid = part.SingleSolid();
        Assert.True(solid.IsValid);
        Assert.True(solid.Volume > 0);
        foreach (var vertex in solid.Vertices())
        {
            // Every corner is an input point
            var p = vertex.Center();
            Assert.Contains(points, q => (q - p).Magnitude <= 1e-12 * Math.Max(1, p.ToVector().Magnitude));
        }
        foreach (var face in solid.Faces())
        {
            Assert.Equal(GeomType.Plane, face.GeomType);
            var corners = face.Vertices().Select(v => v.Center()).ToList();
            var origin = face.Center();
            var normal = face.NormalAt(origin);
            // Flat to 5e-8, and every input point behind it (so the solid is convex and holds them all)
            Assert.All(corners, c => Assert.True(Math.Abs(normal.Dot(c - origin)) <= 5e-8 + 1e-12));
            Assert.All(points, q => Assert.True(normal.Dot(q - origin) <= slack + 1e-9,
                $"{q} is {normal.Dot(q - origin):G3} in front of a face"));
        }
        return solid;
    }

    private static double TetrahedraVolume(IReadOnlyList<Point3D> pts, IEnumerable<int[]> facets) =>
        facets.Sum(f => (pts[f[0]] - Point3D.Origin).Dot((pts[f[1]] - Point3D.Origin).Cross(pts[f[2]] - Point3D.Origin))) / 6;

    // ---- The report's cases ----

    [Fact]
    public void Cube_IsSixFaces()
    {
        var solid = CheckHull(Cube);
        Assert.Equal(1, solid.Volume, 12);
        Assert.Equal((6, 12, 8), (solid.Faces().Count, solid.Edges().Count, solid.Vertices().Count));
    }

    [Theory]
    [InlineData(0.0, 6)]
    [InlineData(1e-9, 6)]
    [InlineData(1e-7, 6)]
    [InlineData(5e-7, 6)]
    [InlineData(1e-6, 6)]
    [InlineData(3e-6, 7)]
    [InlineData(1e-3, 7)]
    public void NearDuplicateCorner_IsValid(double apart, int faces)
    {
        // Two points 1e-7 to 1e-6 apart threw "Face can only be created with closed wires"; now they are one point
        var points = Cube.Append(new Point3D(1, 1, 1 + apart)).ToList();
        var solid = CheckHull(points);
        // Apart, a pyramid on the top face: the two sides it shares a plane with take a triangle each
        Assert.Equal(faces == 6 ? 1 : 1 + apart / 3, solid.Volume, 1e-9);
        Assert.Equal(faces, solid.Faces().Count);
    }

    [Theory]
    [InlineData(4e-6)]
    [InlineData(1e-4)]
    public void PointJustOutsideAnEdge_IsValid(double below)
    {
        var solid = CheckHull(Cube.Append(new Point3D(0.5, 0, -below)).ToList());
        Assert.Equal(1 + below / 3, solid.Volume, 1e-9);
    }

    [Theory]
    [InlineData(1e-2)]
    [InlineData(1e-3)]
    [InlineData(3.3e-4)]
    [InlineData(1e-4)]
    [InlineData(1e-6)]
    [InlineData(1e-7)]
    public void PrismWithACornerCut_HasFlatEnds(double cut)
    {
        Point3D[] end = [new(0, 0, 0), new(24, 0, 0), new(24, 24 - cut, 0), new(24 - cut, 24, 0), new(0, 24, 0)];
        var solid = CheckHull([.. end, .. end.Select(p => new Point3D(p.X, p.Y, 3))]);
        Assert.Equal((24 * 24 - cut * cut / 2) * 3, solid.Volume, 24 * 3 * Constants.Tolerance);
        // The ends are one face each, whatever the triangles across them; a cut shorter than the merge distance is no cut
        Assert.Equal(cut * Math.Sqrt(2) > Constants.Tolerance ? 7 : 6, solid.Faces().Count);
    }

    [Fact]
    public void BumperHousing_IsValid()
    {
        // 162 points from a user's model: a convex section at x = 156 and the same inset 3 mm at x = 159, two points
        // 0.33 µm apart and a hull triangle 4 µm high in the x = 156 plane. It threw "fix face failed"
        var points = File.ReadAllLines(Path.Combine(AppContext.BaseDirectory, "Reference", "Hulls", "bumper-housing.txt"))
            .Select(l => l.Split(' ').Select(v => double.Parse(v, CultureInfo.InvariantCulture)).ToArray())
            .Select(a => new Point3D(a[0], a[1], a[2])).ToList();
        var solid = CheckHull(points);
        // SciPy 1.18.1's ConvexHull of the same points: volume 7271.738599576939, area 5615.128496991793. Merging the
        // two close points and leaving out corners within 1e-6 moves it by no more than that times the area
        Assert.Equal(7271.738599576939, solid.Volume, 2e-6 * 5615.13);
        Assert.Equal(5615.128496991793, solid.Area, 1e-4);
        // The two ends are one face each
        Assert.Equal(2, solid.Faces().Count(f => Math.Abs(f.NormalAt().X) > 1 - 1e-12));
    }

    // ---- Generated sets ----

    [Theory]
    [InlineData(1, 20)]
    [InlineData(2, 200)]
    [InlineData(3, 1000)]
    public void RandomCloud_IsTheHull(int seed, int count)
    {
        var random = new Random(seed);
        var points = Enumerable.Range(0, count)
            .Select(_ => new Point3D(random.NextDouble() * 100, random.NextDouble() * 50, random.NextDouble() * 10 - 5)).ToList();
        var solid = CheckHull(points);
        var facets = ConvexHull3D.Facets(points);
        Assert.Equal(TetrahedraVolume(points, facets), solid.Volume, 1e-9 * solid.Volume);
    }

    [Theory]
    [InlineData(4, 300)]
    [InlineData(5, 2000)]
    public void PointsOnASphere_AreAllCorners(int seed, int count)
    {
        var random = new Random(seed);
        var points = Enumerable.Range(0, count).Select(_ =>
        {
            double z = random.NextDouble() * 2 - 1, t = random.NextDouble() * 2 * Math.PI, r = Math.Sqrt(1 - z * z);
            return new Point3D(10 * r * Math.Cos(t), 10 * r * Math.Sin(t), 10 * z);
        }).ToList();
        var solid = CheckHull(points);
        Assert.True(solid.Volume < 4.0 / 3 * Math.PI * 1000 && solid.Volume > 0.95 * 4.0 / 3 * Math.PI * 1000);
    }

    [Theory]
    [InlineData(6, 0.0)]
    [InlineData(7, 1e-9)]
    [InlineData(8, 1e-7)]
    [InlineData(9, 1e-6)]
    [InlineData(10, 1e-5)]
    public void DuplicatedPoints_AreMerged(int seed, double jitter)
    {
        // Each point of a cloud given three times, moved by up to the jitter in each direction
        var random = new Random(seed);
        var cloud = Enumerable.Range(0, 100).Select(_ => new Point3D(random.NextDouble(), random.NextDouble(), random.NextDouble())).ToList();
        double J() => (random.NextDouble() * 2 - 1) * jitter;
        var points = cloud.SelectMany(p => new[] { p, new Point3D(p.X + J(), p.Y + J(), p.Z + J()), new Point3D(p.X + J(), p.Y + J(), p.Z + J()) }).ToList();
        var solid = CheckHull(points, slack: Constants.Tolerance + 2 * jitter);
        Assert.Equal(new ConvexPolyhedron(cloud).Volume, solid.Volume, 1e-4);
    }

    [Theory]
    [InlineData(2)]
    [InlineData(5)]
    [InlineData(11)]
    public void Grid_IsABox(int n)
    {
        // Every point of an n x n x n grid: most on the hull's sides and edges, none of them corners but eight
        var points = (from x in Enumerable.Range(0, n) from y in Enumerable.Range(0, n) from z in Enumerable.Range(0, n)
                      select new Point3D(x * 2.0 / (n - 1), y * 3.0 / (n - 1), z * 0.1 / (n - 1))).ToList();
        var solid = CheckHull(points);
        Assert.Equal(0.6, solid.Volume, 12);
        Assert.Equal((6, 12, 8), (solid.Faces().Count, solid.Edges().Count, solid.Vertices().Count));
    }

    [Fact]
    public void RotatedGrid_IsABox()
    {
        // The same, turned so that no side is exactly flat in floating point
        var axis = new Vector3D(1, 2, 3).Normalized();
        double angle = 37 * Math.PI / 180;
        Point3D Turn(Point3D p)
        {
            // Rodrigues' rotation about the axis, then a shift
            var v = p - Point3D.Origin;
            var turned = v * Math.Cos(angle) + axis.Cross(v) * Math.Sin(angle) + axis * (axis.Dot(v) * (1 - Math.Cos(angle)));
            return new Point3D(1 + turned.X, 2 + turned.Y, 3 + turned.Z);
        }
        var points = (from x in Enumerable.Range(0, 7) from y in Enumerable.Range(0, 7) from z in Enumerable.Range(0, 7)
                      select Turn(new Point3D(x / 6.0, y / 6.0, z / 6.0))).ToList();
        var solid = CheckHull(points);
        Assert.Equal(1, solid.Volume, 9);
        Assert.Equal((6, 12, 8), (solid.Faces().Count, solid.Edges().Count, solid.Vertices().Count));
    }

    [Fact]
    public void SampledCylinder_HasFlatEnds()
    {
        // Two circles of 360 points: the ends flat, the side a band of quadrilaterals
        var points = Enumerable.Range(0, 360).SelectMany(i =>
        {
            double t = i * Math.PI / 180;
            return new[] { new Point3D(5 * Math.Cos(t), 5 * Math.Sin(t), 0), new Point3D(5 * Math.Cos(t), 5 * Math.Sin(t), 20) };
        }).ToList();
        var solid = CheckHull(points);
        Assert.Equal(2 + 360, solid.Faces().Count);
        Assert.Equal(20 * 360 * 25 * Math.Sin(Math.PI / 180) / 2, solid.Volume, 1e-9);
    }

    [Theory]
    [InlineData(1930)]
    [InlineData(2071)]
    [InlineData(1699)]
    [InlineData(7102)]
    [InlineData(9994)]
    public void SweepFailures_AreValid(int number)
    {
        // Sets from a sweep of 13,000 that failed on the way: jittered grids, jitter 2.6e-8 to 8.9e-8, near the
        // tolerance of a flat side. The first two sewed into an unorientable shell; the others put a thin triangle
        // turned sideways into a side, folding its outline
        var points = File.ReadAllLines(Path.Combine(AppContext.BaseDirectory, "Reference", "Hulls", $"sweep-{number}.txt"))
            .Select(l => l.Split(' ').Select(v => double.Parse(v, CultureInfo.InvariantCulture)).ToArray())
            .Select(a => new Point3D(a[0], a[1], a[2])).ToList();
        CheckHull(points, slack: 2e-6);
    }

    [Fact]
    public void AwkwardSets_AreValidOrHaveNoVolume()
    {
        // 1,200 small sets of six kinds: random; a jittered grid; a slab thinner than the jitter or not; pairs of points
        // the jitter apart; a thin gridded plate; a jittered sphere. Jitter from 1e-10 to 1e-4. Each is a valid solid,
        // or has no volume and says so
        var random = new Random(99);
        int valid = 0, flat = 0;
        for (int k = 0; k < 1200; k++)
        {
            int kind = k % 6;
            int n = 4 + random.Next(kind == 5 ? 300 : 40);
            double jitter = Math.Pow(10, -4 - 6 * random.NextDouble());
            double J() => (random.NextDouble() * 2 - 1) * jitter;
            var pts = new List<Point3D>();
            for (int i = 0; i < n; i++)
            {
                pts.Add(kind switch
                {
                    0 => new Point3D(random.NextDouble(), random.NextDouble(), random.NextDouble()),
                    1 => new Point3D(random.Next(4) / 3.0 + J(), random.Next(4) / 3.0 + J(), random.Next(4) / 3.0 + J()),
                    2 => new Point3D(random.NextDouble() * 100, random.NextDouble() * 100, random.NextDouble() * jitter * 100),
                    3 => i % 2 == 0 ? new Point3D(random.NextDouble(), random.NextDouble(), random.NextDouble()) : pts[^1] + new Vector3D(J(), J(), J()),
                    4 => new Point3D(random.Next(3) + J(), random.Next(3) + J(), random.Next(2) * 1e-3 + J()),
                    _ => OnSphere(random, 50, jitter),
                });
            }
            var result = Operations.MakeConvexPolyhedron(pts);
            if (result.TryGetValue(out var part))
            {
                Assert.True(part.IsValid, $"set {k}");
                valid++;
            }
            else
            {
                var error = result.Diagnostics.First(d => d.IsError);
                Assert.Equal(ModelingDiagnostics.HullFailed, error.Code);
                Assert.True(error.Message.Contains("no volume") || error.Message.Contains("four distinct"), $"set {k}: {error.Message}");
                flat++;
            }
        }
        Assert.True(valid > 1000 && flat > 50, $"{valid} valid, {flat} flat");
    }

    private static Point3D OnSphere(Random random, double r, double jitter)
    {
        double z = random.NextDouble() * 2 - 1, t = random.NextDouble() * 2 * Math.PI, s = Math.Sqrt(1 - z * z);
        return new Point3D(r * s * Math.Cos(t) + jitter, r * s * Math.Sin(t), r * z);
    }

    // ---- As a result ----

    [Fact]
    public void MakeConvexPolyhedron_IsAResult()
    {
        var made = Operations.MakeConvexPolyhedron(Cube);
        Assert.True(made.IsSuccess);
        Assert.Equal(1, made.Value.Volume, 12);
        // Aligned and rotated as the constructor does it
        var placed = Operations.MakeConvexPolyhedron(Cube, new Rotation(0, 0, 45), Align.Center).Value;
        Assert.Equal(new ConvexPolyhedron(Cube, new Rotation(0, 0, 45), Align.Center).Bounds().Max.X, placed.Bounds().Max.X, 12);

        var flat = Operations.MakeConvexPolyhedron([new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(0, 1, 0), new Point3D(1, 1, 0)]);
        Assert.False(flat.IsSuccess);
        var error = flat.Diagnostics.First(d => d.IsError);
        Assert.Equal(ModelingDiagnostics.HullFailed, error.Code);
        Assert.Contains("in one plane", error.Message);
        Assert.Throws<ArgumentNullException>(() => Operations.MakeConvexPolyhedron(null!));
    }

    // ---- No volume ----

    [Theory]
    [InlineData("none")]
    [InlineData("one")]
    [InlineData("three")]
    [InlineData("four close")]
    [InlineData("collinear")]
    [InlineData("coplanar")]
    [InlineData("square and a near point")]
    [InlineData("flat to 1e-7")]
    [InlineData("not finite")]
    public void NoVolume_Throws(string what)
    {
        var random = new Random(11);
        Point3D[] points = what switch
        {
            "none" => [],
            "one" => [new(1, 2, 3)],
            "three" => [new(0, 0, 0), new(1, 0, 0), new(0, 1, 0), new(0, 1, 0)],
            "four close" => [new(0, 0, 0), new(1e-7, 0, 0), new(0, 1e-7, 0), new(0, 0, 1e-7)],
            "collinear" => [.. Enumerable.Range(0, 20).Select(i => new Point3D(i, 2 * i, 3 * i))],
            "coplanar" => [.. Enumerable.Range(0, 50).Select(_ => new Point3D(random.NextDouble(), random.NextDouble(), 0))],
            "square and a near point" => [new(0, 0, 0), new(1, 0, 0), new(1, 1, 0), new(0, 1, 0), new(0.5, 0.5, 5e-7)],
            "flat to 1e-7" => [.. Enumerable.Range(0, 50).Select(_ => new Point3D(random.NextDouble(), random.NextDouble(), random.NextDouble() * 1e-7))],
            _ => [.. Cube, new(double.NaN, 0, 0)],
        };
        var e = Assert.Throws<ModelingException>(() => new ConvexPolyhedron(points));
        var expected = what switch
        {
            "none" or "one" or "three" or "four close" => "at least four distinct points",
            "collinear" => "on one line",
            "not finite" => "finite",
            _ => "in one plane",
        };
        Assert.Contains(expected, e.Message);
    }

    // ---- The exact test ----

    [Fact]
    public void Orientation_IsConsistentWhereFloatingPointIsNot()
    {
        // Points nearly in the plane of three others: swapping two of the three must flip the sign, and turning them
        // round must keep it. The floating-point volume fails that on some of them; the exact test never does
        var random = new Random(12);
        int floatingWrong = 0;
        for (int k = 0; k < 2000; k++)
        {
            Point3D R() => new(random.NextDouble() * 10, random.NextDouble() * 10, random.NextDouble() * 10);
            Point3D p = R(), q = R(), r = R();
            double a = random.NextDouble(), b = random.NextDouble();
            var s = p + (q - p) * a + (r - p) * b;
            int o = ConvexHull3D.Orientation(p, q, r, s);
            Assert.Equal(-o, ConvexHull3D.Orientation(q, p, r, s));
            Assert.Equal(o, ConvexHull3D.Orientation(q, r, p, s));
            Assert.Equal(-o, ConvexHull3D.Orientation(p, r, q, s));
            Assert.Equal(-o, ConvexHull3D.Orientation(s, q, r, p));
            static int F(Point3D p, Point3D q, Point3D r, Point3D s) => Math.Sign((q - p).Cross(r - p).Dot(s - p));
            if (F(p, q, r, s) != -F(q, p, r, s) || F(p, q, r, s) != F(q, r, p, s))
                floatingWrong++;
        }
        Assert.True(floatingWrong > 0, "no case where floating point is inconsistent: the test shows nothing");
        // Exactly in the plane
        Assert.Equal(0, ConvexHull3D.Orientation(new(0.1, 0.2, 0.3), new(1.1, 0.7, 0.3), new(0.6, 1.9, 0.3), new(0.35, 0.45, 0.3)));
    }
}
