using System.Globalization;
using Kerf.Fillets;
using Kerf.Geometry;
using Kerf.Modeling;
using Kerf.Topology;
using static Kerf.Modeling.Operations;

namespace Kerf.Modeling.Tests.Topology;

/// <summary>
/// A radius swept across each limit, at sizes from 0.01 to 10,000 (decisions 22101 to 22103): below it, within tolerance, just
/// above and far above. Volume and area come from the section of each shape, worked out here from its corners and not from how
/// the fillet is made, so that a result that merely agrees with its builder doesn't pass.
/// </summary>
public class FilletLimitSweepTests
{
    public static TheoryData<double> Sizes => new() { 0.01, 0.1, 1, 10, 1000, 10000 };

    private static Edge At(Shape shape, double x, double y, double z) =>
        shape.Edges().OrderBy(e => Math.Sqrt(Math.Pow(e.Center().X - x, 2) + Math.Pow(e.Center().Y - y, 2) + Math.Pow(e.Center().Z - z, 2))).First();

    /// <summary>A case: its true limit, worked out by hand, and its closed forms. Null where the limit isn't built yet.</summary>
    private sealed record Case(string Name, Shape Solid, Edge[] Edges, double Limit, Func<double, (double Volume, double? Area)> Answer, bool BuiltAtLimit = true);

    // The change to a prism's section from rounding a corner where the inward directions make the angle: its area and perimeter
    private static (double Area, double Perimeter) Corner(double r, double angle, bool convex = true)
    {
        double spandrel = r * r / Math.Tan(angle / 2) - r * r * (Math.PI - angle) / 2;
        return (convex ? -spandrel : spandrel, r * (Math.PI - angle) - 2 * r / Math.Tan(angle / 2));
    }

    private static (double Volume, double? Area) PrismAnswer(double sectionArea, double perimeter, double height, params (double Area, double Perimeter)[] corners)
    {
        double a = sectionArea + corners.Sum(c => c.Area), p = perimeter + corners.Sum(c => c.Perimeter);
        return (a * height, 2 * a + p * height);
    }

    private static Part Prism(double s, double height, params (double X, double Y)[] points) =>
        Extrude(new Polygon(points.Select(q => new Point3D(q.X * s, q.Y * s, 0))), height * s).Value;

    private static IEnumerable<Case> Cases(double s)
    {
        double Half = Math.PI / 2;

        // Boxes: one edge, each way, and moved and turned
        var cube = new Box(s, s, s);
        yield return new("cube, one edge", cube, [At(cube, s / 2, s / 2, 0)], s, r => PrismAnswer(s * s, 4 * s, s, Corner(r, Half)));
        var wide = new Box(s, 2 * s, 3 * s);
        yield return new("box along Z", wide, [At(wide, s / 2, s, 0)], s, r => PrismAnswer(2 * s * s, 6 * s, 3 * s, Corner(r, Half)));
        yield return new("box along X", wide, [At(wide, 0, s, 1.5 * s)], 2 * s, r => PrismAnswer(6 * s * s, 10 * s, s, Corner(r, Half)));
        Part moved = (Part)(new Pos(3 * s, 4 * s, 5 * s) * new Rotation(10, 20, 30) * (Shape)cube);
        yield return new("cube, moved and turned", moved, [moved.Edges().First()], s, r => PrismAnswer(s * s, 4 * s, s, Corner(r, Half)));

        // Several edges apart: sharing a face, not sharing one, and four to a cylinder
        yield return new("two meeting on a face", cube, [At(cube, s / 2, 0, s / 2), At(cube, -s / 2, 0, s / 2)], s / 2, r => PrismAnswer(s * s, 4 * s, s, Corner(r, Half), Corner(r, Half)));
        yield return new("two across a diagonal", cube, [At(cube, s / 2, s / 2, 0), At(cube, -s / 2, -s / 2, 0)], s, r => PrismAnswer(s * s, 4 * s, s, Corner(r, Half), Corner(r, Half)));
        var bar = new Box(s, s, 4 * s);
        yield return new("four to a cylinder", bar, [.. bar.Edges().Where(e => Math.Abs(e.Center().Z) < 1e-6 * s)], s / 2,
                         r => PrismAnswer(s * s, 4 * s, 4 * s, Corner(r, Half), Corner(r, Half), Corner(r, Half), Corner(r, Half)));

        // Faces at other angles: acute, obtuse, a hexagon's and a triangle's
        var hexagon = Extrude(new RegularPolygon(s, 6), 3 * s).Value;
        Edge[] hexagonal = [.. hexagon.Edges().Where(e => e.Length > 2.5 * s)];
        double hexArea = 3 * Math.Sqrt(3) / 2 * s * s;
        yield return new("hexagon, one edge", hexagon, [hexagonal[0]], s * Math.Tan(Math.PI / 3), r => PrismAnswer(hexArea, 6 * s, 3 * s, Corner(r, 2 * Math.PI / 3)));
        yield return new("hexagon to a cylinder", hexagon, hexagonal, s * Math.Sqrt(3) / 2,
                         r => PrismAnswer(hexArea, 6 * s, 3 * s, [.. Enumerable.Repeat(Corner(r, 2 * Math.PI / 3), 6)]));
        var triangle = Extrude(new RegularPolygon(s, 3), 3 * s).Value;
        Edge[] triangular = [.. triangle.Edges().Where(e => e.Length > 2.5 * s)];
        double triArea = 3 * Math.Sqrt(3) / 4 * s * s;
        yield return new("triangle, one edge", triangle, [triangular[0]], s, r => PrismAnswer(triArea, 3 * Math.Sqrt(3) * s, 3 * s, Corner(r, Math.PI / 3)));
        yield return new("triangle to a cylinder", triangle, triangular, s / 2,
                         r => PrismAnswer(triArea, 3 * Math.Sqrt(3) * s, 3 * s, [.. Enumerable.Repeat(Corner(r, Math.PI / 3), 3)]));
        Part trapezoid = Prism(s, 2, (0, 0), (4, 0), (3, 1), (1, 1));
        double trapezoidEdges = 4 + 2 + 2 * Math.Sqrt(2);
        yield return new("trapezoid, 45 degrees", trapezoid, [At(trapezoid, 0, 0, s)], Math.Sqrt(2) * s * Math.Tan(Math.PI / 8),
                         r => PrismAnswer(3 * s * s, trapezoidEdges * s, 2 * s, Corner(r, Math.PI / 4)));

        // A concave edge: it adds. An L, with a wall one high and a ledge two wide
        Part l = Prism(s, 2, (0, 0), (3, 0), (3, 1), (1, 1), (1, 2), (0, 2));
        yield return new("concave", l, [At(l, s, s, s)], s, r => PrismAnswer(4 * s * s, 10 * s, 2 * s, Corner(r, Half, convex: false)));

        // Slanted end. The section in XY runs along Y, from y = 0 to a slant, so the edge at x = 0, z = 0 ends on the slant, and the
        // strip taken is longer the further it is from the edge across the end face: length L + k x. By integrals over the spandrel
        double k = Math.Tan(Math.PI / 6), along = (4 - k) * s;
        Part slanted = Prism(s, 1, (0, 0), (1, 0), (1, 4), (0, 4 - k));
        yield return new("slanted end", slanted, [At(slanted, 0, along / 2, 0)], s, r =>
        {
            double spandrel = r * r * (1 - Math.PI / 4);
            double firstMoment = r * r * r * (10 - 3 * Math.PI) / 12;
            double volume = (4 - k / 2) * s * s * s - (spandrel * along + k * firstMoment);
            double area = 2 * (4 - k / 2) * s * s + (1 + 4 + along / s + Math.Sqrt(1 + k * k)) * s * s;
            double arc = Half * r * along + k * r * r * (Half - 1);
            return (volume, area - (r * along + k * r * r / 2) - r * along + arc - spandrel - spandrel * Math.Sqrt(1 + k * k));
        }, BuiltAtLimit: false);

        // A chain round a top: no cylinders' corners, as the upright edges stay sharp, so two cylinders meet at each. By layers
        var top = new Box(2 * s, 3 * s, s);
        yield return new("chain round a top", top, [.. top.Edges().Where(e => Math.Abs(e.Center().Z - s / 2) < 1e-6 * s)], s,
                         r => (6 * s * s * s - 2 * (2 * s + 3 * s) * r * r * (1 - Math.PI / 4) + 4 * r * r * r * (5.0 / 3 - Half), null), BuiltAtLimit: false);

        // All of a cube's edges: a rounded cube, with a sphere's corners, and at the limit a sphere
        yield return new("all of a cube", cube, [.. cube.Edges()], s / 2, r =>
        {
            double f = s - 2 * r;
            return (f * f * f + 6 * r * f * f + 3 * Math.PI * r * r * f + 4 * Math.PI * r * r * r / 3, 6 * f * f + 6 * Math.PI * r * f + 4 * Math.PI * r * r);
        }, BuiltAtLimit: false);
    }

    private static void Right(Part part, (double Volume, double? Area) answer, string what, double relative = 1e-6)
    {
        Assert.True(part.IsValid, what + ": not valid");
        Assert.True(Math.Abs(part.Volume - answer.Volume) <= relative * answer.Volume, $"{what}: volume {part.Volume}, by the geometry {answer.Volume}");
        if (answer.Area is { } area)
        {
            Assert.True(Math.Abs(part.Area - area) <= relative * area, $"{what}: area {part.Area}, by the geometry {area}");
        }
    }

    // Fillet on the shape, as a caller makes it
    private static Result<Part> Round(Case c, double radius) => Fillet(c.Solid, c.Edges, radius);

    [Theory]
    [MemberData(nameof(Sizes))]
    public void Below_a_limit_the_answer_is_the_geometrys(double s)
    {
        foreach (Case c in Cases(s))
        {
            foreach (double radius in new[] { 0.5 * c.Limit, 0.9 * c.Limit, 0.99 * c.Limit, 0.999 * c.Limit, c.Limit - 1e-5, c.Limit - 1e-6 })
            {
                string what = $"{c.Name} at size {s}, radius {radius:R}";
                var made = Round(c, radius);
                Assert.True(made.IsSuccess, what + ": " + made.Error?.Message);
                // ChFi3d works to absolute tolerances (1e-4 on its spine), a hundredth of the smallest shape: its corners are less close there
                Right(made.Value, c.Answer(radius), what, s < 0.5 ? 1e-5 : 1e-6);
            }
        }
    }

    [Theory]
    [MemberData(nameof(Sizes))]
    public void Within_tolerance_of_a_limit_is_the_limit(double s)
    {
        foreach (Case c in Cases(s))
        {
            var atLimit = Round(c, c.Limit);
            // The tolerance is 1e-7 on where the contact line lies; these are within it with the sharpest corners here, two to a face
            foreach (double offset in new[] { -2.5e-8, -1e-8, -1e-10, 1e-10, 1e-8, 2.5e-8 })
            {
                string what = $"{c.Name} at size {s}, limit {offset:+0e0;-0e0}";
                var made = Round(c, c.Limit + offset);
                Assert.True(atLimit.IsSuccess == made.IsSuccess, $"{what}: at the limit {atLimit.IsSuccess} {atLimit.Error?.Message}; here {made.IsSuccess} {made.Error?.Message}");
                if (atLimit.IsSuccess)
                {
                    // The same in every figure: it's the same radius
                    Assert.True(atLimit.Value.Volume == made.Value.Volume, $"{what}: {atLimit.Value.Volume:R}, {made.Value.Volume:R}");
                    Assert.True(atLimit.Value.Area == made.Value.Area, $"{what}: {atLimit.Value.Area:R}, {made.Value.Area:R}");
                    Assert.Equal(atLimit.Value.Faces().Count, made.Value.Faces().Count);
                }
                else
                {
                    Assert.Equal(atLimit.Error!.Message, made.Error!.Message);
                }
            }
            string name = $"{c.Name} at size {s}, at the limit";
            if (c.BuiltAtLimit)
            {
                Assert.True(atLimit.IsSuccess, name + ": " + atLimit.Error?.Message);
                Right(atLimit.Value, c.Answer(c.Limit), name);
            }
            else
            {
                Assert.True(atLimit.IsFailure, name);
                Assert.Contains("can't be built yet", atLimit.Error!.Message, StringComparison.Ordinal);
            }
        }
    }

    [Theory]
    [MemberData(nameof(Sizes))]
    public void Above_a_limit_it_is_refused_and_says_what_fits(double s)
    {
        foreach (Case c in Cases(s))
        {
            foreach (double radius in new[] { c.Limit + 2e-7, c.Limit + 1e-6, c.Limit + 1e-3, c.Limit * 1.001, c.Limit * 1.1, c.Limit * 1.5, c.Limit * 2, c.Limit * 10 })
            {
                string what = $"{c.Name} at size {s}, radius {radius:R}";
                var failed = Round(c, radius);
                Assert.True(failed.IsFailure, what);
                Assert.Equal(ModelingDiagnostics.FilletFailed, failed.Error!.Code);
                string message = failed.Error.Message;
                const string Most = "the most that fits is ";
                Assert.Contains(Most, message, StringComparison.Ordinal);

                // Refused from the geometry: nothing of ChFi3d's in what it says
                Assert.DoesNotContain(failed.Diagnostics, d => d.Code != FilletOperations.TooLarge && d.Code != ModelingDiagnostics.FilletFailed);

                // And what it says fits is the limit, to tolerance: asked for, it does what the limit does
                int start = message.IndexOf(Most, StringComparison.Ordinal) + Most.Length;
                string said = message[start..message.IndexOf(", where", start, StringComparison.Ordinal)];
                double most = double.Parse(said, CultureInfo.CurrentCulture);
                Assert.True(Math.Abs(most - c.Limit) <= 1e-7, $"{what}: says {said}");
            }
        }
    }

    [Theory]
    [MemberData(nameof(Sizes))]
    public void The_limit_claimed_is_the_one_worked_out_by_hand(double s)
    {
        foreach (Case c in Cases(s))
        {
            double? claimed = c.Solid.FilletLimit(c.Edges);
            Assert.NotNull(claimed);
            Assert.True(Math.Abs(claimed.Value - c.Limit) <= 1e-8, $"{c.Name} at size {s}: {claimed}, by hand {c.Limit}");
        }
    }

    [Theory]
    [InlineData(0.01)]
    [InlineData(1)]
    [InlineData(10)]
    [InlineData(10000)]
    public void A_fine_sweep_either_side_changes_outcome_once(double s)
    {
        // Worst case first: from well below to well above, in steps that bunch up at the limit, the outcome goes from built to refused
        // once on a box, at the limit's far side, and never comes back. Each result is right on the way
        var cube = new Box(s, s, s);
        Edge edge = At(cube, s / 2, s / 2, 0);
        const double Loss = 1 - Math.PI / 4;
        bool seenRefused = false;
        foreach (int sign in new[] { -1, 1 })
        {
            for (int e = sign < 0 ? -3 : -12; sign < 0 ? e >= -12 : e <= -3; e += sign < 0 ? -1 : 1)
            {
                foreach (double m in new[] { 1, 2, 3, 5, 7 })
                {
                    double radius = s + sign * m * Math.Pow(10, e) * Math.Min(1, s);
                    var made = Fillet(cube, edge, radius);
                    double past = radius - s;
                    if (past > 1.1e-7)
                    {
                        seenRefused = true;
                        Assert.True(made.IsFailure, $"radius {radius:R}, past by {past}");
                    }
                    else if (Math.Abs(past) < 0.9e-7)
                    {
                        // Clear of the rounding of the band's edges: the limit
                        Assert.True(made.IsSuccess, $"radius {radius:R}");
                        Assert.Equal(5, made.Value.Faces().Count);
                        Assert.Equal(Math.PI / 4 * s * s * s, made.Value.Volume, 1e-6 * s * s * s);
                    }
                    else if (past < -1.1e-7)
                    {
                        Assert.True(made.IsSuccess, $"radius {radius:R}");
                        Assert.Equal(7, made.Value.Faces().Count);
                        double expected = s * s * s - Loss * radius * radius * s;
                        Assert.True(Math.Abs(made.Value.Volume - expected) <= 1e-6 * expected, $"radius {radius:R}: {made.Value.Volume}, {expected}");
                    }
                }
            }
        }
        Assert.True(seenRefused);
    }

    [Theory]
    [InlineData(0.1, true)] // Near-tangent: ChFi3d rolls on past it, and is right
    [InlineData(1, true)]
    [InlineData(5, true)]
    [InlineData(5.7, true)]
    [InlineData(5.74, false)] // Just past 0.1 radians, what it calls tangent
    [InlineData(10, false)]
    [InlineData(45, false)]
    public void A_far_edge_the_algorithm_calls_tangent_is_no_limit(double degrees, bool carriesOn)
    {
        // A prism whose top goes from (0,0) to (5,0), and then falls away by this angle
        double tilt = degrees * Math.PI / 180, fall = 20 * Math.Tan(tilt), bottom = -Math.Max(10, fall + 10);
        Part shape = Prism(1, 30, (0, 0), (0, bottom), (25, bottom), (25, -fall), (5, 0));
        Edge edge = At(shape, 0, 0, 15);
        double radius = 7.5;
        var made = Fillet(shape, edge, radius);
        double? limit = ((Shape)shape).FilletLimit([edge]);
        if (!carriesOn)
        {
            Assert.Equal(5, limit!.Value, 1e-9);
            Assert.True(made.IsFailure);
            Assert.Contains("the most that fits is 5,", made.Error!.Message, StringComparison.Ordinal);
            return;
        }

        // The other face beside the edge still limits it, at its 10 or more
        Assert.True(limit is null or >= 10);

        // A ball touching the side x = 0 and the next face, worked out from the section
        double nx = -Math.Sin(tilt), ny = -Math.Cos(tilt); // the next face's normal, inwards
        double yc = (radius - (radius - 5) * nx) / ny;
        double fx = radius - radius * nx, fy = yc - radius * ny;
        double swept = Math.Atan2(radius * Math.Abs(fy - yc), -radius * (fx - radius));
        double quad = Math.Abs(0 * 0 - 0 * yc + (0 * 0 - 5 * 0) + (5 * fy - fx * 0) + (fx * yc - radius * fy) + (radius * yc - 0 * yc)) / 2;
        double removed = quad - radius * radius * swept / 2;
        double section = 25 * -bottom - (20 * fall / 2);
        Assert.True(made.IsSuccess, made.Error?.Message);
        Assert.True(made.Value.IsValid);
        Assert.Equal((section - removed) * 30, made.Value.Volume, 1e-6 * section * 30);
    }

    [Fact]
    public void MaxFillet_gives_what_the_full_window_gave_without_the_tries_that_cannot_fit()
    {
        // The search as it was, counting its tries, and with the limit's window: the same answers to the last bit, or the same throw
        (Part Shape, IEnumerable<Edge> Edges)[] cases =
        [
            (new Box(10, 10, 10), new Box(10, 10, 10).Edges()),
            (new Box(10, 8, 2), new Box(10, 8, 2).Edges()),
            (new Box(1, 1, 1), new Box(1, 1, 1).Edges()),
            (new Box(10, 10, 10), new Box(10, 10, 10).Edges().Where(e => Math.Abs(e.Center().Z - 5) < 1e-6)),
        ];
        foreach ((Part shape, IEnumerable<Edge> edges) in cases)
        {
            Edge[] list = [.. shape.Edges().Where(e => edges.Any(o => Math.Abs(o.Center().X - e.Center().X) + Math.Abs(o.Center().Y - e.Center().Y) + Math.Abs(o.Center().Z - e.Center().Z) < 1e-6))];
            double limit = ((Shape)shape).FilletLimit(list)!.Value;
            int before = 0, after = 0;
            double Search(double min, double max, int iteration, bool skip, ref int tries)
            {
                double mid = (min + max) / 2;
                if (iteration == 10)
                {
                    return min;
                }
                bool works = false;
                if (!skip || mid < limit)
                {
                    tries++;
                    works = ((Shape)shape).Fillet(list, mid).IsSuccess;
                }
                if (!works)
                {
                    return Search(min, mid, iteration + 1, skip, ref tries);
                }
                return mid - min <= 0.1 ? mid : Search(mid, max, iteration + 1, skip, ref tries);
            }
            double diagonal = shape.Bounds().Diagonal;
            double all = Search(0, 2 * diagonal, 0, false, ref before);
            double fewer = Search(0, 2 * diagonal, 0, true, ref after);
            Assert.Equal(all, fewer);
            Assert.Equal(all, shape.MaxFillet(list));
            Assert.True(after < before, $"{after} tries, from {before}");
        }
    }
}
