// Shaft gauges (Kerf), one per shaft size: a flat rounded hexagon with a U-shaped cutout in each side, to sort blade shafts by diameter
// before the clamp is fitted (JP, 2026-11-10).
//
// Each cutout is a slot of one diameter, with a semicircular bottom and straight sides out to the edge, and its size is
// cut into the top face beside it. A shaft fits every cutout at or above its own diameter.
//
// Frame: the gauge lies flat, centred on the origin, its bed face at z = 0 and +Z up. Cutout 0 faces -Y, and the rest
// follow anticlockwise seen from above, smallest to largest.

using Kerf;
using Kerf.Geometry;
using Kerf.Modeling;
using static Kerf.Modeling.Operations;

namespace Renegade.ShaftGauge;

public static class Gauge
{
    /// <summary>The shaft sizes there are gauges for, by nominal diameter.</summary>
    public static readonly int[] Nominals = [35, 50, 58];

    /// <summary>The size being built: Use(size) picks it. Parts are cached per size.</summary>
    public static int N { get; private set; } = 35;

    public static void Use(int size)
    {
        if (!Nominals.Contains(size))
            throw new ArgumentException($"no gauge for {size} mm; there are {string.Join(", ", Nominals)}");
        N = size;
    }

    /// <summary>The cutouts: size marked, diameter. JP's for 35 mm; the same steps either side of 50 and 58.</summary>
    public static IReadOnlyList<(string Size, double D)> Sizes =>
        [("U/S", N - 1.0), ($"{N}XS", N - 0.6), ($"{N}S", N - 0.2), ($"{N}M", N + 0.2), ($"{N}L", N + 0.6), ($"{N}XL", N + 1.0)];

    /// <summary>The hexagon: the smallest multiple of 5 that leaves WEB between neighbouring cutouts' shafts.</summary>
    public static double ACROSS_FLATS
    {
        get
        {
            var s = Sizes;
            var widest = Enumerable.Range(0, s.Count).Max(i => (s[i].D + s[(i + 1) % s.Count].D) / 2);
            return 5 * Math.Ceiling(2 * (WEB + STRAIGHT + widest) / 5 - 1e-9);
        }
    }

    public static string NAME => $"Ø{N}";   // in the middle

    public const double T = 4.0;              // thick enough not to spread when pushed over a shaft
    public const double WEB = 11.0;           // between neighbouring cutouts: stiff jaws, and a hexagon, not a star
    public const double CORNER_R = 6.0;       // the hexagon's corners
    public const double STRAIGHT = 3.0;       // the cutout's straight sides: edge to its semicircle's centre
    public const double MOUTH_R = 1.0;        // where the cutout's sides meet the edge
    public const double CHAMFER = 0.4;        // all round both faces: clears elephant's foot, leads the shaft in
    public const double LABEL_SIZE = 6.0;     // the font's em: capitals about 4.2 high
    public const double LABEL_DEPTH = 0.6;
    public const double LABEL_GAP = 2.0;      // cutout's bottom to the label's box
    public const double NAME_SIZE = 9.0;

    /// <summary>Unit vector from the centre out through side i: side 0 faces -Y, anticlockwise from there.</summary>
    public static Vector3D SideDir(int i)
    {
        var a = (-90 + 60 * i) * Math.PI / 180;
        return new Vector3D(Math.Cos(a), Math.Sin(a), 0);
    }

    /// <summary>The centre of cutout i's semicircle, at z = 0: where a shaft's axis lies when it sits in it.</summary>
    public static Point3D Seat(int i) => Point3D.Origin + SideDir(i) * (ACROSS_FLATS / 2 - STRAIGHT);

    static T Ok<T>(this Result<T> r, string what) =>
        r.TryGetValue(out var v) ? v : throw new InvalidOperationException($"{what}: {r.Error}");

    static readonly Dictionary<int, Part> blanks = [], gauges = [];

    /// <summary>The hexagon with its cutouts, rounded and chamfered, before anything is cut into its face.</summary>
    public static Part Blank() => blanks.TryGetValue(N, out var b) ? b : blanks[N] = MakeBlank();

    static Part MakeBlank()
    {
        Sketch hexagon = new RegularPolygon(ACROSS_FLATS / 2 / Math.Cos(Math.PI / 6), 6);   // flats face ±Y
        Part g = Extrude(hexagon, T).Ok("hexagon");
        g = Fillet(g, g.Edges().ParallelTo(Axis.Z), CORNER_R).Ok("corners");

        var mouths = new List<Point3D>();
        for (int i = 0; i < Sizes.Count; i++)
        {
            var (u, r) = (SideDir(i), Sizes[i].D / 2);
            var across = new Vector3D(-u.Y, u.X, 0);
            var angle = Math.Atan2(u.Y, u.X) * 180 / Math.PI;
            var seat = Seat(i);
            var len = STRAIGHT + 5;   // out past the edge
            g -= new Pos(seat.X, seat.Y, T / 2) * new Cylinder(r, T + 2);
            g -= new Pos(seat + u * (len / 2) + new Vector3D(0, 0, T / 2)) * new Rotation(0, 0, angle) * new Box(len, 2 * r, T + 2);
            mouths.Add(seat + u * STRAIGHT + across * r);
            mouths.Add(seat + u * STRAIGHT - across * r);
        }
        var mouthEdges = g.Edges().ParallelTo(Axis.Z)
            .Where(e => mouths.Any(m => Math.Abs(e.Center().X - m.X) < 1e-3 && Math.Abs(e.Center().Y - m.Y) < 1e-3)).ToShapeList();
        if (mouthEdges.Count != 2 * Sizes.Count)
            throw new InvalidOperationException($"found {mouthEdges.Count} mouth edges, not {2 * Sizes.Count}");
        g = Fillet(g, mouthEdges, MOUTH_R).Ok("mouths");

        var faces = g.Faces().OrderBy(Axis.Z).ToList();
        var rims = faces[0].Edges().Concat(faces[^1].Edges()).ToShapeList();
        return Chamfer(g, rims, CHAMFER).Ok("chamfer");
    }

    /// <summary>The middle of cutout i's label: in from the bottom of the cutout, towards the gauge's centre.</summary>
    public static Point3D LabelCentre(int i) => Seat(i) - SideDir(i) * (Sizes[i].D / 2 + LABEL_GAP + LABEL_SIZE * 0.35);

    /// <summary>The gauge as printed: the blank with the sizes and its name cut into the top face.</summary>
    public static Part Make() => gauges.TryGetValue(N, out var g) ? g : gauges[N] = MakeGauge();

    static Part Engraving(string text, double size, Point3D at, double angle)
    {
        var t = MakeText(text, size, "Liberation Sans", fontStyle: Kerf.Fonts.FontStyle.Bold).Ok($"text {text}");
        Sketch placed = new Pos(at.X, at.Y, T) * new Rotation(0, 0, angle) * (Sketch)t;
        return Extrude(placed, -LABEL_DEPTH).Ok($"engrave {text}");
    }

    static Part MakeGauge()
    {
        var g = Blank();
        for (int i = 0; i < Sizes.Count; i++)
        {
            var u = SideDir(i);
            // upright when the gauge sits on the shaft: the letters' tops point away from the cutout
            g -= Engraving(Sizes[i].Size, LABEL_SIZE, LabelCentre(i), Math.Atan2(u.Y, u.X) * 180 / Math.PI + 90);
        }
        return g - Engraving(NAME, NAME_SIZE, Point3D.Origin, 0);
    }
}
