using System.Collections;

namespace Kerf.Modeling;

/// <summary>
/// A set of locations to place objects at: <c>new GridLocations(10, 10, 2, 2) * new Circle(1)</c> places four circles.
/// In builder mode (phase 5) it will also be a context the builder's objects are placed at.
/// </summary>
public class LocationList : IReadOnlyList<Location>
{
    /// <summary>A location list of the given locations.</summary>
    public LocationList(IEnumerable<Location> locations) => LocalLocations = [.. locations];

    /// <summary>The locations, in local construction coordinates.</summary>
    public IReadOnlyList<Location> LocalLocations { get; }

    /// <summary>The locations (the same as <see cref="LocalLocations"/>).</summary>
    public IReadOnlyList<Location> Locations => LocalLocations;

    /// <inheritdoc/>
    public int Count => LocalLocations.Count;

    /// <inheritdoc/>
    public Location this[int index] => LocalLocations[index];

    /// <inheritdoc/>
    public IEnumerator<Location> GetEnumerator() => LocalLocations.GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();

    /// <summary>
    /// The locations moved as a group to each of the enclosing context's locations. Without an enclosing context (algebra
    /// mode) they are returned as they are.
    /// </summary>
    internal static IEnumerable<Location> MoveToExisting(IEnumerable<Location> local) => local;
}

/// <summary>Locations at the given points, vertices, locations, planes, axes or (planar) faces.</summary>
public class Locations : LocationList
{
    /// <summary>Locations at each of the given objects.</summary>
    public Locations(params IEnumerable<object> points) : base(MoveToExisting(points.SelectMany(Flatten).Select(ToLocation)))
    {
    }

    private static IEnumerable<object> Flatten(object o) =>
        o is IEnumerable e and not IShape ? e.Cast<object>().SelectMany(Flatten) : [o];

    private static Location ToLocation(object point) => point switch
    {
        Location l => l,
        Vector v => new Location(v),
        Vertex v => new Location(v.ToVector()),
        ValueTuple<double, double> t => new Location(new Vector(t.Item1, t.Item2)),
        ValueTuple<double, double, double> t => new Location(new Vector(t.Item1, t.Item2, t.Item3)),
        ValueTuple<int, int> t => new Location(new Vector(t.Item1, t.Item2)),
        ValueTuple<int, int, int> t => new Location(new Vector(t.Item1, t.Item2, t.Item3)),
        Plane p => new Location(p),
        Axis a => a.Location,
        Face f => new Location(new Plane(f)),
        _ => throw new ArgumentException($"Locations doesn't accept type {point.GetType().Name}"),
    };
}

/// <summary>A rectangular grid of locations, aligned (by default centred) about the origin.</summary>
public class GridLocations : LocationList
{
    /// <summary>A grid of <paramref name="xCount"/> × <paramref name="yCount"/> locations.</summary>
    /// <exception cref="ArgumentException">A count is less than one.</exception>
    public GridLocations(double xSpacing, double ySpacing, int xCount, int yCount, Alignment? align = null)
        : base(MoveToExisting(Make(xSpacing, ySpacing, xCount, yCount, Expand(align), out var size, out var min)))
    {
        (XSpacing, YSpacing, XCount, YCount, Align) = (xSpacing, ySpacing, xCount, yCount, Expand(align));
        (Size, Min) = (size, min);
        Max = Min + Size;
    }

    /// <summary>The horizontal spacing.</summary>
    public double XSpacing { get; }

    /// <summary>The vertical spacing.</summary>
    public double YSpacing { get; }

    /// <summary>The number of locations along X.</summary>
    public int XCount { get; }

    /// <summary>The number of locations along Y.</summary>
    public int YCount { get; }

    /// <summary>The alignment of the grid.</summary>
    public Alignment Align { get; }

    /// <summary>The size of the grid.</summary>
    public Vector Size { get; }

    /// <summary>The grid's bottom left corner.</summary>
    public Vector Min { get; }

    /// <summary>The grid's top right corner.</summary>
    public Vector Max { get; }

    internal static Alignment Expand(Alignment? align) =>
        align is { } a ? (a.Count == 1 ? new Alignment(a[0]!.Value, a[0]!.Value) : a) : new Alignment(Kerf.Modeling.Align.Center, Kerf.Modeling.Align.Center);

    private static List<Location> Make(double xSpacing, double ySpacing, int xCount, int yCount, Alignment align, out Vector size, out Vector min)
    {
        if (xCount < 1 || yCount < 1)
            throw new ArgumentException($"At least 1 elements required, requested {xCount}, {yCount}");
        size = new Vector(xSpacing * (xCount - 1), ySpacing * (yCount - 1));
        var offset = align.Offset(new Vector(0, 0), size);
        min = offset;
        var locations = new List<Location>();
        for (int i = 0; i < xCount; i++)
            for (int j = 0; j < yCount; j++)
                locations.Add(new Location(offset + new Vector(i * xSpacing, j * ySpacing)));
        return locations;
    }
}

/// <summary>Locations evenly round a circle, by default turned to follow it.</summary>
public class PolarLocations : LocationList
{
    /// <summary>
    /// <paramref name="count"/> locations on a circle of <paramref name="radius"/>, from <paramref name="startAngle"/> over
    /// <paramref name="angularRange"/> degrees (including the end with <paramref name="endpoint"/>), each turned to the
    /// circle's tangent unless <paramref name="rotate"/> is false.
    /// </summary>
    /// <exception cref="ArgumentException">The count is less than one.</exception>
    public PolarLocations(double radius, int count, double startAngle = 0.0, double angularRange = 360.0, bool rotate = true,
                          bool endpoint = false)
        : base(MoveToExisting(Make(radius, count, startAngle, angularRange, rotate, endpoint)))
    {
    }

    private static List<Location> Make(double radius, int count, double startAngle, double angularRange, bool rotate, bool endpoint)
    {
        if (count < 1)
            throw new ArgumentException($"At least 1 elements required, requested {count}");
        var step = count == 1 ? 0.0 : angularRange / (count - (endpoint ? 1 : 0));
        var locations = new List<Location>();
        for (int i = 0; i < count; i++)
            locations.Add(new Location(new Vector(radius, 0).Rotate(Axis.Z, startAngle + step * i), new Vector(0, 0, 1),
                                       rotate ? step * i + startAngle : 0));
        return locations;
    }
}

/// <summary>A hexagonal array of locations (as for a honeycomb of circles), aligned (by default centred) about the origin.</summary>
public class HexLocations : LocationList
{
    /// <summary>
    /// A hex array of <paramref name="xCount"/> × <paramref name="yCount"/> locations. <paramref name="radius"/> is the
    /// apothem (the inscribed radius), or with <paramref name="majorRadius"/> the radius to the hexagons' vertices; for an
    /// array of circles use the circle's radius plus half the spacing.
    /// </summary>
    /// <exception cref="ArgumentException">A spacing or count isn't positive.</exception>
    public HexLocations(double radius, int xCount, int yCount, bool majorRadius = false, Alignment? align = null)
        : base(MoveToExisting(Make(radius, xCount, yCount, majorRadius, GridLocations.Expand(align), out var diagonal, out var apothem)))
    {
        (Radius, XCount, YCount, MajorRadius, Align, Diagonal, Apothem) =
            (radius, xCount, yCount, majorRadius, GridLocations.Expand(align), diagonal, apothem);
    }

    /// <summary>The radius as given.</summary>
    public double Radius { get; }

    /// <summary>The radius of the inscribed circle (the minor radius).</summary>
    public double Apothem { get; }

    /// <summary>The major diameter: twice the major radius.</summary>
    public double Diagonal { get; }

    /// <summary>The number of columns.</summary>
    public int XCount { get; }

    /// <summary>The number of rows.</summary>
    public int YCount { get; }

    /// <summary>Whether <see cref="Radius"/> is the major radius.</summary>
    public bool MajorRadius { get; }

    /// <summary>The alignment of the array.</summary>
    public Alignment Align { get; }

    private static List<Location> Make(double radius, int xCount, int yCount, bool majorRadius, Alignment align,
                                       out double diagonal, out double apothem)
    {
        if (majorRadius)
            (diagonal, apothem) = (2 * radius, radius * Math.Cos(Math.PI / 6));
        else
            (diagonal, apothem) = (4 * radius / Math.Sqrt(3), radius);
        var xSpacing = 3 * diagonal / 4;
        var ySpacing = diagonal * Math.Sqrt(3) / 2;
        if (xSpacing <= 0 || ySpacing <= 0 || xCount < 1 || yCount < 1)
            throw new ArgumentException("Spacing and count must be > 0 ");
        // The raw coordinates from the bottom left
        var points = new List<Vector>();
        for (int x = 0; x < xCount; x += 2)
            for (int y = 0; y < yCount; y++)
                points.Add(new Vector(xSpacing * x, ySpacing * y + ySpacing / 2));
        for (int x = 1; x < xCount; x += 2)
            for (int y = 0; y < yCount; y++)
                points.Add(new Vector(xSpacing * x, ySpacing * y + ySpacing));
        // The minimum point and size of the array, as sorting by axis finds them
        var byX = points.OrderBy(p => p.X).ToList();
        var byY = points.OrderBy(p => p.Y).ToList();
        var size = new Vector(byX[^1].X - byX[0].X, byY[^1].Y - byY[0].Y);
        var minCorner = new Vector(byX[0].X, byY[0].Y);
        var offset = align.Offset(new Vector(0, 0), size);
        return [.. points.Select(p => new Location(p + offset - minCorner))];
    }
}
