// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Exchange;
using Kerf.Geometry;
using Kerf.Topology;
using Xunit.Abstractions;
using static Kerf.Modeling.Operations;

namespace Kerf.Modeling.Tests.Operations;

/// <summary>
/// An extrusion skips its clean when the profile has nothing to merge (decision 1895). The test of that is the only safe one: over
/// many profiles, with and without the skip, the same BREP text, so the same faces and edges in the same order with the same
/// orientation, geometry and tolerances, and the same empty record; and a clean that would have been skipped hands back the very
/// shape it was given.
/// </summary>
public class ExtrudeCleanSkipTests(ITestOutputHelper output)
{
    // How many of the random profiles of each family; raised by hand for a longer run
    private const int Scale = 1;

    private static readonly double[] Depths = [10, -7.5];

    private static string Text(IEnumerable<Shape> shapes)
    {
        var all = new List<string>();
        foreach (var shape in shapes)
        {
            using var stream = new MemoryStream();
            Assert.True(Brep.Write(shape, stream).IsSuccess);
            all.Add(Convert.ToBase64String(stream.GetBuffer(), 0, (int)stream.Length));
        }
        return string.Join('\n', all);
    }

    private sealed class Tally
    {
        public int Extrusions, Skipped, CleanedMerging, CleanedNotMerging;
    }

    // Extrudes every way it can, and compares with the same solid cleaned as it always was
    private static void Compare(Face profile, string what, Tally tally)
    {
        foreach (double depth in Depths)
        {
            using var log = HistoryLog.Open();
            var plain = Solid.Extrude(profile, profile.NormalAt() * depth);
            bool skips = TopoUtil.NothingToCleanAfterExtruding(profile, profile.NormalAt() * depth);
            var (cleaned, record) = ShapeTransforms.Clean(plain);
            bool same = cleaned.IsSame(plain) && cleaned.Orientation == plain.Orientation;
            tally.Extrusions++;
            if (skips)
            {
                tally.Skipped++;
                Assert.True(same, $"{what}: skipped, but the clean makes another shape");
                Assert.NotNull(record);
                Assert.Empty(record!.ToHistory().Entries);
            }
            else if (same)
                tally.CleanedNotMerging++;
            else
                tally.CleanedMerging++;

            // Through the operation, which skips, against no clean and then the clean
            Part made = Extrude(profile, depth).Value;
            Part unclean = Extrude(profile, depth, new ExtrudeOptions { Clean = false }).Value;
            var byHand = unclean.Solids().Select(s => (Shape)s.Clean()).ToList();
            Assert.True(Text(made.Solids().Select(s => (Shape)s)) == Text(byHand), $"{what}, depth {depth}: not the same shape");

            // The record of a skipped solid, in a log, is the clean's: there, and with nothing in it
            if (skips)
                Assert.Empty(made.Solids()[0].AsShape().History!.ToHistory().Entries);
        }
    }

    private static Edge L(double x1, double y1, double x2, double y2) => Edge.MakeLine(new Point3D(x1, y1, 0), new Point3D(x2, y2, 0));

    private static Edge Arc(double x1, double y1, double xm, double ym, double x2, double y2) =>
        Edge.MakeThreePointArc(new Point3D(x1, y1, 0), new Point3D(xm, ym, 0), new Point3D(x2, y2, 0));

    private static Face FaceOf(params Edge[] edges) => Face.Make(Wire.Make(edges));

    private static Face Polygon(IEnumerable<(double X, double Y)> corners) =>
        Face.Make(Wire.MakePolygon(corners.Select(c => new Point3D(c.X, c.Y, 0))));

    private static IEnumerable<(string, Face)> GeneratedProfiles()
    {
        var random = new Random(12);

        // Polygons, convex and star-shaped, on and off a line of neighbours
        for (int i = 0; i < 300 * Scale; i++)
        {
            int n = random.Next(3, 24);
            double phase = random.NextDouble() * 6;
            bool star = i % 2 == 1;
            var corners = Enumerable.Range(0, n).Select(k =>
            {
                double angle = phase + 2 * Math.PI * k / n;
                double r = (star ? 5 + 10 * random.NextDouble() : 10) * (1 + i % 7);
                return (r * Math.Cos(angle), r * Math.Sin(angle));
            }).ToList();
            yield return ($"polygon {i}", Polygon(corners));
        }

        // Lines and arcs mixed at random round a circle, some neighbouring arcs bulging alike, placed anywhere
        for (int i = 0; i < 200 * Scale; i++)
        {
            int n = random.Next(3, 16);
            double r = 5 + 50 * random.NextDouble();
            double bulge = i % 3 == 0 ? r : r * (1 + 0.2 * random.NextDouble());
            var edges = new List<Edge>();
            for (int k = 0; k < n; k++)
            {
                double a0 = 2 * Math.PI * k / n, a1 = 2 * Math.PI * (k + 1) / n, am = (a0 + a1) / 2;
                (double X, double Y) p0 = (r * Math.Cos(a0), r * Math.Sin(a0)), p1 = (r * Math.Cos(a1), r * Math.Sin(a1));
                if (k + 1 == n)
                    p1 = (r, 0);
                edges.Add(random.Next(2) == 0
                    ? L(p0.X, p0.Y, p1.X, p1.Y)
                    : Arc(p0.X, p0.Y, bulge * Math.Cos(am), bulge * Math.Sin(am), p1.X, p1.Y));
            }
            Face mixed = MakeOrNull(() => FaceOf([.. edges]));
            if (mixed.IsEmpty)
                continue;
            yield return ($"mixed {i}", mixed);
            yield return ($"mixed {i}, placed", (Face)mixed.Moved(new Location(new Plane(new Point3D(random.NextDouble() * 500, -300, 40), new Vector3D(random.NextDouble() - 0.5, 1, 0), new Vector3D(0.1, 0.2, 1)))));
            if (i % 5 == 0)
                yield return ($"mixed {i}, mirrored", mixed.Mirror(Plane.YZ));
        }

        // Neighbours nearly on one line, from plainly not to within rounding, and the point of the margin between
        foreach (double angle in new[] { 1e-1, 1e-2, 1e-3, 1e-4, 1e-5, 2e-6, 1.1e-6, 9e-7, 5e-7, 1e-7, 1e-8, 1e-9, 1e-10, 1e-11, 1e-12, 1e-13, 1e-14, 1e-15, 0 })
        {
            foreach (double length in new[] { 1e-2, 1.0, 100, 1e4 })
            {
                double y = length * Math.Sin(angle);
                yield return ($"nearly collinear {angle} at {length}", Polygon([(-length, 0), (0, 0), (length, y), (length, 2 * length), (-length, 2 * length)]));
            }
        }

        // A line followed by itself, with the vertex plainly on it
        yield return ("collinear", Polygon([(0, 0), (5, 0), (10, 0), (10, 10), (0, 10)]));
        yield return ("collinear both sides", Polygon([(0, 0), (5, 0), (10, 0), (10, 5), (10, 10), (5, 10), (0, 10), (0, 5)]));

        // A circle: whole, split in two and three, from different places
        foreach (double radius in new[] { 0.01, 1, 7.5, 1000, 1e6 })
        {
            yield return ($"circle {radius}", Face.Make(Wire.MakeCircle(radius)));
            yield return ($"circle in two {radius}", FaceOf(Arc(radius, 0, 0, radius, -radius, 0), Arc(-radius, 0, 0, -radius, radius, 0)));
            double c = radius * Math.Cos(2 * Math.PI / 3), s = radius * Math.Sin(2 * Math.PI / 3);
            yield return ($"circle in three {radius}", FaceOf(
                Arc(radius, 0, radius * Math.Cos(Math.PI / 3), radius * Math.Sin(Math.PI / 3), c, s),
                Arc(c, s, -radius, 0, c, -s),
                Arc(c, -s, radius * Math.Cos(Math.PI / 3), -radius * Math.Sin(Math.PI / 3), radius, 0)));
        }

        // Arcs nearly on one circle: the same centre with radii apart, then the same radius with centres apart
        foreach (double gap in new[] { 1.0, 0.1, 1e-2, 1e-3, 1e-4, 2e-5, 1.1e-5, 9e-6, 1e-6, 1e-7, 5e-8, 1e-8, 1e-9, 1e-10, 1e-12, 0 })
        {
            const double R = 10;
            // One arc on a circle of radius R, the next on a smaller one inside it, so they meet at (R, 0), tangent there
            var first = Arc(0, R, R * Math.Cos(Math.PI / 4), R * Math.Sin(Math.PI / 4), R, 0);
            double r2 = R - gap;
            var second = Arc(R, 0, gap + r2 * Math.Cos(-Math.PI / 4), r2 * Math.Sin(-Math.PI / 4), gap, -r2);
            yield return ($"nearly equal arcs, radii {gap}", FaceOf(first, second, L(gap, -r2, 0, 0), L(0, 0, 0, R)));

            // Equal radii, centres apart by the gap: the left of one and the right of the other, meeting where they cross
            double join = Math.Sqrt(R * R - gap * gap / 4);
            yield return ($"nearly equal arcs, centres {gap}", MakeOrNull(() => FaceOf(
                Arc(gap / 2, -join, -R, 0, gap / 2, join),
                Arc(gap / 2, join, gap + R, 0, gap / 2, -join))));
        }

        // Lines meeting arcs: tangent and not, tight and of a huge radius, as rounded corners are
        foreach (double corner in new[] { 0.01, 0.5, 2, 4.9 })
        {
            double a = 10, b = 6, k = corner * (1 - Math.Sqrt(0.5));
            yield return ($"rounded rectangle {corner}", FaceOf(
                L(-a + corner, -b, a - corner, -b), Arc(a - corner, -b, a - k, -b + k, a, -b + corner),
                L(a, -b + corner, a, b - corner), Arc(a, b - corner, a - k, b - k, a - corner, b),
                L(a - corner, b, -a + corner, b), Arc(-a + corner, b, -a + k, b - k, -a, b - corner),
                L(-a, b - corner, -a, -b + corner), Arc(-a, -b + corner, -a + k, -b + k, -a + corner, -b)));
        }
        foreach (double sagitta in new[] { 1.0, 1e-3, 1e-5, 1e-7, 1e-9 })
            yield return ($"a very flat arc {sagitta}", MakeOrNull(() => FaceOf(L(-10, -5, -10, 0), Arc(-10, 0, 0, sagitta, 10, 0), L(10, 0, 10, -5), L(10, -5, -10, -5))));
        yield return ("two circles' tangent arcs", FaceOf(Arc(-4, 0, -2, 2, 0, 0), Arc(0, 0, 2, -2, 4, 0), L(4, 0, 4, 5), L(4, 5, -4, 5), L(-4, 5, -4, 0)));

        // Splines: alone, closed, as neighbours, and one that is really straight
        for (int i = 0; i < 40 * Scale; i++)
        {
            var points = new List<Point3D> { new(0, 0, 0) };
            for (int k = 1; k < 5; k++)
                points.Add(new Point3D(k * 5, 3 * random.NextDouble(), 0));
            points.Add(new Point3D(25, 0, 0));
            var spline = Edge.MakeSpline(points);
            yield return ($"spline {i}", FaceOf(spline, L(25, 0, 25, -6), L(25, -6, 0, -6), L(0, -6, 0, 0)));
        }
        {
            var left = Edge.MakeSpline([new Point3D(0, 0, 0), new Point3D(3, 1, 0), new Point3D(5, 0, 0)]);
            var right = Edge.MakeSpline([new Point3D(5, 0, 0), new Point3D(7, -1, 0), new Point3D(10, 0, 0)]);
            yield return ("two splines", FaceOf(left, right, L(10, 0, 10, -6), L(10, -6, 0, -6), L(0, -6, 0, 0)));
            var straight = Edge.MakeSpline([new Point3D(0, 0, 0), new Point3D(2, 0, 0), new Point3D(5, 0, 0)]);
            yield return ("a straight spline by a line", FaceOf(straight, L(5, 0, 10, 0), L(10, 0, 10, 6), L(10, 6, 0, 6), L(0, 6, 0, 0)));
            yield return ("a closed spline", Face.Make(Wire.Make([Edge.MakeSpline([new Point3D(5, 0, 0), new Point3D(0, 5, 0), new Point3D(-5, 0, 0), new Point3D(0, -5, 0)], periodic: true)])));
        }

        // An ellipse, which isn't a line or a circle
        yield return ("an ellipse", Face.Make(Wire.MakeEllipse(8, 3)));

        // Holes: round, square, several, with the outside's neighbours on one line
        for (int i = 0; i < 60 * Scale; i++)
        {
            double r = 1 + 3 * random.NextDouble();
            var holes = new List<Wire>
            {
                ((Wire)Wire.MakeCircle(r).Moved(new Location(new Point3D(-20, 5 * random.NextDouble(), 0)))),
                Wire.MakePolygon([new Point3D(15, -3, 0), new Point3D(19, -3, 0), new Point3D(19, 3, 0), new Point3D(15, 3, 0)]),
            };
            yield return ($"holes {i}", Face.Make(Wire.MakePolygon([new Point3D(-40, -20, 0), new Point3D(40, -20, 0), new Point3D(40, 20, 0), new Point3D(-40, 20, 0)]), holes));
        }
        yield return ("a hole in a split circle", Face.Make(
            Wire.Make([Arc(20, 0, 0, 20, -20, 0), Arc(-20, 0, 0, -20, 20, 0)]),
            [Wire.MakePolygon([new Point3D(-3, -3, 0), new Point3D(3, -3, 0), new Point3D(0, 3, 0)])]));

        // Sketches as Bento makes them, placed anywhere, including with arithmetic that leaves a cut outline
        foreach (var (name, sketch) in new (string, Sketch)[]
        {
            ("rectangle", new Rectangle(30, 10)),
            ("circle", new Circle(7)),
            ("rounded", new RectangleRounded(30, 10, 2)),
            ("slot", new SlotOverall(30, 10)),
            ("hexagon", new RegularPolygon(10, 6)),
            ("rectangle less circle", new Rectangle(40, 30) - new Circle(5)),
            ("circle less rectangle", new Circle(60) - new Rectangle(40, 30)),
            ("two squares fused, in line", new Rectangle(10, 10) + new Pos(10, 0.0) * new Rectangle(10, 10)),
            ("square fused off line", new Rectangle(10, 10) + new Pos(10, 3.0) * new Rectangle(10, 10)),
            ("half circle", new Circle(9) - new Pos(0, -10.0) * new Rectangle(30, 20)),
            ("ellipse less circle", new Ellipse(20, 10) - new Circle(2)),
        })
        {
            foreach (var face in sketch.Faces())
            {
                yield return (name, face);
                yield return (name + ", placed", (Face)face.Moved(new Location(new Plane(new Point3D(100, -200, 50), new Vector3D(1, 1, 0), new Vector3D(0, 1, 1)))));
                yield return (name + ", reversed", (Face)face.AsShape().Reversed());
            }
        }
    }

    private static Face MakeOrNull(Func<Face> make)
    {
        try
        {
            return make();
        }
        catch (Exception e) when (e is ArgumentException or InvalidOperationException)
        {
            return default;
        }
    }

    [Fact]
    public void Generated_profiles_are_the_same_with_and_without_the_skip()
    {
        var tally = new Tally();
        int profiles = 0, missing = 0;
        foreach (var (what, face) in GeneratedProfiles())
        {
            if (face.IsEmpty)
            {
                missing++;
                continue;
            }
            profiles++;
            Compare(face, what, tally);
        }
        output.WriteLine($"{profiles} profiles ({missing} didn't make a face), {tally.Extrusions} extrusions: {tally.Skipped} skipped, " +
                         $"{tally.CleanedMerging} cleaned and merged, {tally.CleanedNotMerging} cleaned when they needn't have been");
        Assert.True(tally.Skipped > 800);
        Assert.True(tally.CleanedMerging > 30);
    }

    [Fact]
    public void What_can_merge_is_not_skipped()
    {
        Vector3D z = Vector3D.UnitZ * 10;
        Assert.False(TopoUtil.NothingToCleanAfterExtruding(Polygon([(0, 0), (5, 0), (10, 0), (10, 10), (0, 10)]), z));
        Assert.False(TopoUtil.NothingToCleanAfterExtruding(FaceOf(Arc(5, 0, 0, 5, -5, 0), Arc(-5, 0, 0, -5, 5, 0)), z));
        Assert.False(TopoUtil.NothingToCleanAfterExtruding(Polygon([(0, 0), (5, 0), (10, 1e-7), (10, 10), (0, 10)]), z));
        Assert.False(TopoUtil.NothingToCleanAfterExtruding(Face.Make(Wire.MakeEllipse(8, 3)), z));
        Assert.True(TopoUtil.NothingToCleanAfterExtruding(Polygon([(0, 0), (10, 0), (10, 10), (0, 10)]), z));
        Assert.True(TopoUtil.NothingToCleanAfterExtruding(Face.Make(Wire.MakeCircle(5)), z));
        // Not along the normal, which isn't a plain prism's plane and cylinder
        Assert.False(TopoUtil.NothingToCleanAfterExtruding(Polygon([(0, 0), (10, 0), (10, 10), (0, 10)]), new Vector3D(1, 0, 10)));
    }

    [Fact]
    public void Other_extrusions_clean_as_before()
    {
        Face rectangle = new Rectangle(20, 10).Faces()[0];
        Face split = FaceOf(Arc(5, 0, 0, 5, -5, 0), Arc(-5, 0, 0, -5, 5, 0));
        Shape target = new Pos(0.0, 0.0, 30.0) * new Box(100, 100, 5);
        foreach (Face face in new[] { rectangle, split })
        {
            foreach (var (name, options) in new (string, ExtrudeOptions)[]
            {
                ("both", new ExtrudeOptions { Both = true }),
                ("taper", new ExtrudeOptions { Taper = 5 }),
                ("skew", new ExtrudeOptions { Direction = new Vector3D(1, 0, 3) }),
                ("skew both", new ExtrudeOptions { Direction = new Vector3D(1, 0, 3), Both = true }),
            })
            {
                Part made = Extrude(face, 10, options).Value;
                Part unclean = Extrude(face, 10, options with { Clean = false }).Value;
                Assert.True(Text(made.Solids().Select(s => (Shape)s)) == Text(unclean.Solids().Select(s => (Shape)s.Clean())), name);
            }

            foreach (Until until in new[] { Until.Next, Until.Last })
            {
                Part made = Extrude(face, until, target).Value;
                Part unclean = Extrude(face, until, target, new ExtrudeOptions { Clean = false }).Value;
                Assert.True(Text(made.Solids().Select(s => (Shape)s)) == Text(unclean.Solids().Select(s => (Shape)s.Clean())), until.ToString());
            }
        }
    }

    [Fact]
    public void An_extrusion_in_a_log_carries_the_cleans_empty_record()
    {
        Face face = new Rectangle(20, 10).Faces()[0];
        using var log = HistoryLog.Open();
        Part skipped = Extrude(face, 10).Value;
        Assert.Empty(skipped.Solids()[0].AsShape().History!.ToHistory().Entries);

        Face split = FaceOf(Arc(5, 0, 0, 5, -5, 0), Arc(-5, 0, 0, -5, 5, 0));
        Part cleaned = Extrude(split, 10).Value;
        Assert.NotEmpty(cleaned.Solids()[0].AsShape().History!.ToHistory().Entries);
    }
}
