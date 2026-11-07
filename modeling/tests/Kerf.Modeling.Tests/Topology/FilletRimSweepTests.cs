using System.Globalization;
using Kerf.Fillets;
using Kerf.Geometry;
using Kerf.Modeling;
using Kerf.Topology;
using static Kerf.Modeling.Operations;

namespace Kerf.Modeling.Tests.Topology;

/// <summary>
/// A radius swept across the limit of each circular edge between surfaces turned about one axis (decisions 1904 to 1907), at sizes
/// from 0.01 to 10,000. The answers come from each shape's section, by Pappus and Green's theorem worked out here along the lines
/// and arc of the section, and not from how the fillet is made.
/// </summary>
public class FilletRimSweepTests
{
    public static TheoryData<double> Sizes => new() { 0.01, 0.1, 1, 10, 1000, 10000 };

    /// <summary>A rim in section: the corner, and the directions from it along the two faces, in (distance from the axis, along the axis).</summary>
    private readonly record struct Rim(double X, double Z, (double X, double Z) Along1, (double X, double Z) Along2, bool Convex);

    private sealed record Case(string Name, Shape Solid, Edge[] Edges, double Limit, double Volume, double Area, Rim[] Rims, int? FacesAtLimit = null, bool Ball = false);

    private static (double X, double Z) Unit(double x, double z)
    {
        double l = Math.Sqrt(x * x + z * z);
        return (x / l, z / l);
    }

    // What rounding a rim changes, by the section: the volume by Pappus through Green's theorem round its boundary (corner, contact, arc,
    // other contact), and the area by Pappus along the arc and the two strips it replaces
    private static (double Volume, double Area) Change(Rim rim, double r)
    {
        (double X, double Z) a = rim.Along1, b = rim.Along2;
        double angle = Math.Acos(a.X * b.X + a.Z * b.Z);
        double on = r / Math.Tan(angle / 2);
        (double X, double Z) p = (rim.X + on * a.X, rim.Z + on * a.Z), q = (rim.X + on * b.X, rim.Z + on * b.Z);
        (double X, double Z) mid = Unit(a.X + b.X, a.Z + b.Z);
        (double X, double Z) c = (rim.X + r / Math.Sin(angle / 2) * mid.X, rim.Z + r / Math.Sin(angle / 2) * mid.Z);

        // Integral of x^2 / 2 dz on each line; the arc by parts, finely. Signed by the way round
        static double Line((double X, double Z) u, (double X, double Z) v) => (v.Z - u.Z) * (u.X * u.X + u.X * v.X + v.X * v.X) / 6;
        double from = Math.Atan2(p.Z - c.Z, p.X - c.X), to = Math.Atan2(q.Z - c.Z, q.X - c.X);
        double sweep = Math.IEEERemainder(to - from, 2 * Math.PI);
        const int Steps = 4000;
        double arcGreen = 0, arcPappus = 0;
        for (int i = 0; i < Steps; i++)
        {
            double t = from + sweep * (i + 0.5) / Steps;
            double x = c.X + r * Math.Cos(t);
            arcGreen += x * x / 2 * r * Math.Cos(t) * sweep / Steps;
            arcPappus += x * r * Math.Abs(sweep) / Steps;
        }
        double moment = Math.Abs(Line((rim.X, rim.Z), p) + arcGreen + Line(q, (rim.X, rim.Z)));
        double strips = on * (rim.X + p.X) / 2 + on * (rim.X + q.X) / 2;
        return (2 * Math.PI * moment * (rim.Convex ? -1 : 1), 2 * Math.PI * (arcPappus - strips));
    }

    private static (double Volume, double Area) Answer(Case c, double r)
    {
        double volume = c.Volume, area = c.Area;
        foreach (Rim rim in c.Rims)
        {
            (double v, double a) = Change(rim, r);
            volume += v;
            area += a;
        }
        return (volume, area);
    }

    private static Edge[] Circles(Shape shape, Func<Point3D, double, bool> pick) =>
        [.. shape.Edges().Where(e => e.GeomType == GeomType.Circle && pick(e.Center(), e.Length / (2 * Math.PI)))];

    private static double CylinderArea(double r, double h) => 2 * Math.PI * r * r + 2 * Math.PI * r * h;

    // Cases at a size: dimensions as parts of s, the largest radius or so
    private static IEnumerable<Case> Cases(double s)
    {
        double u = s / 10;
        const double Pi = Math.PI;

        // A disc: its side used up
        var disc = new Cylinder(10 * u, 5 * u);
        Rim discTop = new(10 * u, 2.5 * u, (-1, 0), (0, -1), true), discBottom = new(10 * u, -2.5 * u, (-1, 0), (0, 1), true);
        yield return new("disc's rim", disc, Circles(disc, (c, _) => c.Z > 0), 5 * u, Pi * 500 * u * u * u, CylinderArea(10 * u, 5 * u), [discTop], 3);
        yield return new("disc's both rims, a full round", disc, Circles(disc, (_, _) => true), 2.5 * u, Pi * 500 * u * u * u, CylinderArea(10 * u, 5 * u), [discTop, discBottom], 4);

        // A hemisphere and a sphere
        var hemi = new Cylinder(5 * u, 5 * u);
        yield return new("hemisphere", hemi, Circles(hemi, (c, _) => c.Z > 0), 5 * u, Pi * 125 * u * u * u, CylinderArea(5 * u, 5 * u), [new(5 * u, 2.5 * u, (-1, 0), (0, -1), true)], 2);
        var ball = new Cylinder(5 * u, 10 * u);
        yield return new("sphere", ball, Circles(ball, (_, _) => true), 5 * u, Pi * 250 * u * u * u, CylinderArea(5 * u, 10 * u),
                         [new(5 * u, 5 * u, (-1, 0), (0, -1), true), new(5 * u, -5 * u, (-1, 0), (0, 1), true)], 1, Ball: true);

        // A cone's: a frustum at the bottom, where its slanted side is the shorter
        var frustum = new Cone(10 * u, 5 * u, 8 * u);
        double slant = Math.Sqrt(25 + 64) * u;
        double frustumVolume = Pi * 8 * u / 3 * (100 + 50 + 25) * u * u, frustumArea = Pi * (100 + 25) * u * u + Pi * 15 * u * slant;
        var along = Unit(-5, 8);
        double bottomLimit = slant * Math.Tan(Math.Acos(-along.X) / 2); // the contact on the side reaches the top
        yield return new("frustum's lower rim", frustum, Circles(frustum, (c, _) => c.Z < 0), bottomLimit, frustumVolume, frustumArea, [new(10 * u, -4 * u, (-1, 0), along, true)], 3);

        // Rings, with a plane between two circles
        Shape ring = new Cylinder(10 * u, 5 * u) - new Cylinder(4 * u, 5 * u);
        double ringVolume = Pi * 84 * 5 * u * u * u, ringArea = 2 * Pi * 84 * u * u + 2 * Pi * 14 * 5 * u * u;
        Rim outer = new(10 * u, 2.5 * u, (-1, 0), (0, -1), true), inner = new(4 * u, 2.5 * u, (1, 0), (0, -1), true);
        yield return new("ring's outer rim", ring, Circles(ring, (c, r) => c.Z > 0 && r > 5 * u), 5 * u, ringVolume, ringArea, [outer], 4);
        yield return new("ring's inner rim", ring, Circles(ring, (c, r) => c.Z > 0 && r < 5 * u), 5 * u, ringVolume, ringArea, [inner], 4);
        yield return new("ring's rims", ring, Circles(ring, (c, _) => c.Z > 0), 3 * u, ringVolume, ringArea, [outer, inner], 5);

        // A hole's rim in a plate, its wall used up: the plate's sides aren't coaxial, and don't stop it
        Shape plate = new Box(50 * u, 50 * u, 10 * u) - new Cylinder(5 * u, 10 * u);
        double plateVolume = (25000 - Pi * 250) * u * u * u, plateArea = (2 * 2500 + 4 * 500 - 2 * Pi * 25 + 2 * Pi * 50) * u * u;
        yield return new("hole's rim", plate, Circles(plate, (c, _) => c.Z > 0), 10 * u, plateVolume, plateArea, [new(5 * u, 5 * u, (1, 0), (0, -1), true)], 7);

        // Concave: bosses and pockets, with the wall used up
        Shape boss = new Box(50 * u, 50 * u, 10 * u) + new Pos(0, 0, 8 * u) * new Cylinder(5 * u, 6 * u);
        double bossVolume = (25000 + Pi * 150) * u * u * u, bossArea = (2 * 2500 + 4 * 500 + 2 * Pi * 5 * 6) * u * u;
        yield return new("boss on a plate", boss, Circles(boss, (c, _) => c.Z > 0 && c.Z < 6 * u), 6 * u, bossVolume, bossArea, [new(5 * u, 5 * u, (1, 0), (0, 1), false)], 8);
        Shape step = new Cylinder(10 * u, 5 * u) + new Pos(0, 0, 4 * u) * new Cylinder(5 * u, 3 * u);
        double stepVolume = (Pi * 500 + Pi * 75) * u * u * u, stepArea = (CylinderArea(10, 5) + 2 * Pi * 5 * 3) * u * u;
        yield return new("step", step, Circles(step, (c, r) => r < 6 * u && c.Z < 3 * u), 3 * u, stepVolume, stepArea, [new(5 * u, 2.5 * u, (1, 0), (0, 1), false)], 5);
        Shape pocket = new Cylinder(20 * u, 10 * u) - new Pos(0, 0, 3 * u) * new Cylinder(8 * u, 6 * u);
        // Open at the top: its floor at the middle, its wall five high
        double pocketVolume = Pi * (4000 - 64 * 5) * u * u * u, pocketArea = (CylinderArea(20, 10) + 2 * Pi * 8 * 5) * u * u;
        yield return new("pocket, one wall", pocket, Circles(pocket, (c, r) => r < 9 * u && Math.Abs(c.Z) < u), 5 * u, pocketVolume, pocketArea, [new(8 * u, 0, (-1, 0), (0, 1), false)], 5);
    }

    private static void Right(Shape made, (double Volume, double Area) answer, string what, double relative = 1e-6)
    {
        Assert.True(made.IsValid, what + ": not valid");
        Assert.True(Math.Abs(made.Volume - answer.Volume) <= relative * Math.Abs(answer.Volume), $"{what}: volume {made.Volume}, by the section {answer.Volume}");
        Assert.True(Math.Abs(made.Area - answer.Area) <= relative * Math.Abs(answer.Area), $"{what}: area {made.Area}, by the section {answer.Area}");
    }

    private static Result<Part> Round(Case c, double radius) => Fillet((Part)c.Solid, c.Edges, radius);

    [Theory]
    [MemberData(nameof(Sizes))]
    public void The_limit_claimed_is_the_one_worked_out_by_hand(double s)
    {
        foreach (Case c in Cases(s))
        {
            double? claimed = ((Kerf.Topology.Shape)c.Solid).FilletLimit(c.Edges);
            Assert.NotNull(claimed);
            Assert.True(Math.Abs(claimed.Value - c.Limit) <= 1e-8, $"{c.Name} at size {s}: {claimed}, by hand {c.Limit}");
        }
    }

    [Theory]
    [MemberData(nameof(Sizes))]
    public void Below_a_limit_the_answer_is_the_sections(double s)
    {
        foreach (Case c in Cases(s))
        {
            foreach (double radius in new[] { 0.5 * c.Limit, 0.9 * c.Limit, 0.99 * c.Limit, 0.999 * c.Limit, c.Limit - 1e-5, c.Limit - 1e-6 })
            {
                string what = $"{c.Name} at size {s}, radius {radius:R}";
                var made = Round(c, radius);
                Assert.True(made.IsSuccess, what + ": " + made.Error?.Message);
                Right(made.Value, Answer(c, radius), what);
            }
        }
    }

    [Theory]
    [MemberData(nameof(Sizes))]
    public void Within_tolerance_of_a_limit_is_the_limit_and_it_is_built(double s)
    {
        foreach (Case c in Cases(s))
        {
            var atLimit = Round(c, c.Limit);
            string name = $"{c.Name} at size {s}, at the limit";
            Assert.True(atLimit.IsSuccess, name + ": " + atLimit.Error?.Message);
            Right(atLimit.Value, Answer(c, c.Limit), name);
            Assert.Equal(c.FacesAtLimit, atLimit.Value.Faces().Count);
            foreach (double offset in new[] { -2.5e-8, -1e-8, -1e-10, 1e-10, 1e-8, 2.5e-8 })
            {
                var made = Round(c, c.Limit + offset);
                Assert.True(made.IsSuccess, $"{name} {offset}: {made.Error?.Message}");
                Assert.True(atLimit.Value.Volume == made.Value.Volume, $"{name} {offset}: {atLimit.Value.Volume:R}, {made.Value.Volume:R}");
                Assert.True(atLimit.Value.Area == made.Value.Area, $"{name} {offset}: {atLimit.Value.Area:R}, {made.Value.Area:R}");
            }
        }
    }

    [Theory]
    [MemberData(nameof(Sizes))]
    public void Above_a_limit_it_is_refused_and_says_what_fits(double s)
    {
        foreach (Case c in Cases(s))
        {
            foreach (double radius in new[] { c.Limit + 2e-6, c.Limit + 1e-3, c.Limit * 1.001, c.Limit * 1.1, c.Limit * 1.5, c.Limit * 2, c.Limit * 10 })
            {
                string what = $"{c.Name} at size {s}, radius {radius:R}";
                var failed = Round(c, radius);
                Assert.True(failed.IsFailure, what);
                string message = failed.Error!.Message;
                const string Most = "the most that fits is ";
                Assert.Contains(Most, message, StringComparison.Ordinal);
                int start = message.IndexOf(Most, StringComparison.Ordinal) + Most.Length;
                double most = double.Parse(message[start..message.IndexOf(", where", start, StringComparison.Ordinal)], CultureInfo.CurrentCulture);
                Assert.True(Math.Abs(most - c.Limit) <= 1e-7, $"{what}: says {most}");

                // The limit is true of the algorithm too: alone, it fails here, or makes something invalid
                var alone = ((Kerf.Topology.Shape)c.Solid).Fillet(c.Edges, radius, new FilletOptions { CheckResult = false });
                Assert.False(alone.IsSuccess && ((Part)alone.Value.Shape).IsValid, what + ": the algorithm alone made something valid");
            }
        }
    }

    [Theory]
    [InlineData(0.01)]
    [InlineData(1)]
    [InlineData(10)]
    [InlineData(10000)]
    public void A_fine_sweep_either_side_changes_outcome_once(double s)
    {
        // Worst case first: bunching up at the limit from both sides, the outcome goes from built to refused once, and each built is right
        foreach (Case c in Cases(s))
        {
            bool refused = false;
            double atLimit = Round(c, c.Limit).Value.Volume;
            foreach (int sign in new[] { -1, 1 })
            {
                for (int e = sign < 0 ? -3 : -12; sign < 0 ? e >= -12 : e <= -3; e += sign < 0 ? -1 : 1)
                {
                    foreach (double m in new[] { 1, 2, 5 })
                    {
                        double radius = c.Limit + sign * m * Math.Pow(10, e) * Math.Min(1, s);
                        string what = $"{c.Name} at size {s}, radius {radius:R}";
                        var made = Round(c, radius);
                        if (made.IsSuccess)
                        {
                            Assert.False(refused, what + ": built after a refusal");

                            // Within tolerance of the limit it's the limit: at the smallest size, that's a part in 50,000 of the radius
                            bool isLimit = Math.Abs(radius - c.Limit) <= 2e-6 && made.Value.Volume == atLimit;
                            Right(made.Value, Answer(c, isLimit ? c.Limit : radius), what);
                        }
                        else
                        {
                            refused = true;
                            Assert.True(radius > c.Limit, what + ": " + made.Error!.Message);
                        }
                    }
                }
            }
        }
    }

    [Fact]
    public void A_full_round_of_equal_sizes_is_a_sphere()
    {
        Part cylinder = new Cylinder(5, 10);
        var made = Fillet(cylinder, cylinder.Edges().Where(e => e.GeomType == GeomType.Circle), 5);
        Assert.True(made.IsSuccess, made.Error?.Message);
        Face only = Assert.Single(made.Value.Faces());
        Assert.Equal(GeomType.Sphere, only.GeomType);
    }

    [Theory]
    [InlineData(5, GeomType.Torus)]    // A tube as wide as the torus
    [InlineData(7, GeomType.Torus)]    // Wider: the hole's wall used up and a thin plate
    public void The_new_surface_is_exact_however_the_tube_compares(double radius, GeomType kind)
    {
        Part disc = new Cylinder(10, radius);
        var made = Fillet(disc, disc.Edges().Where(e => e.GeomType == GeomType.Circle && e.Center().Z > 0), radius);
        Assert.True(made.IsSuccess, made.Error?.Message);
        Assert.Contains(made.Value.Faces(), f => f.GeomType == kind);
    }
}
