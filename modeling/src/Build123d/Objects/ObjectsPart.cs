using Occt;

namespace Build123d;

/// <summary>The combination mode of a builder object: how it joins the builder's shape (used in builder mode).</summary>
public interface IBuilderObject
{
    /// <summary>How the object combines with the builder's shape.</summary>
    Mode Mode { get; }
}

/// <summary>
/// The base of the part objects (<see cref="Box"/>, <see cref="Cylinder"/>, …): a part made from a solid, aligned and then
/// rotated.
/// </summary>
public abstract class BasePartObject : Part, IBuilderObject
{
    /// <summary>A part object from a solid (or part), aligned by its bounding box and then rotated.</summary>
    /// <param name="part">The solid to make the object of.</param>
    /// <param name="rotation">Angles to rotate it about X, Y and Z.</param>
    /// <param name="align">How to align the solid's bounding box with the origin; null for no alignment.</param>
    /// <param name="mode">How the object combines with a builder's part.</param>
    protected BasePartObject(Shape part, Rotation? rotation = null, Alignment? align = null, Mode mode = Mode.Add)
        : base(Prepare(part, rotation, align), part.Label)
    {
        Rotation = rotation ?? new Rotation();
        Mode = mode;
    }

    private static TopoDS_Shape Prepare(Shape part, Rotation? rotation, Alignment? align)
    {
        if (align is { } a)
            part = part.Moved(new Location(part.BoundingBox().ToAlignOffset(a)));
        var rotated = rotation is not null && rotation != new Rotation() ? part.Moved(rotation) : part;
        // Don't add extra layers of compound
        return rotated is Compound ? rotated.TopoShape : TopoUtil.MakeCompound([rotated.TopoShape]);
    }

    /// <summary>The rotation applied to the object.</summary>
    public Rotation Rotation { get; }

    /// <inheritdoc/>
    public Mode Mode { get; }

    /// <summary>A centred alignment on all three axes, the default of most part objects.</summary>
    internal static readonly Alignment Centered = (Align.Center, Align.Center, Align.Center);
}

/// <summary>A box of a length (along X), width (along Y) and height (along Z); centred by default.</summary>
public class Box : BasePartObject
{
    /// <summary>A box; <paramref name="align"/> defaults to centred.</summary>
    public Box(double length, double width, double height, Rotation? rotation = null, Alignment? align = null, Mode mode = Mode.Add)
        : base(Solid.MakeBox(length, width, height), rotation, align ?? Centered, mode)
    {
        (Length, Width, BoxHeight) = (length, width, height);
    }

    /// <summary>The size along X.</summary>
    public double Length { get; }

    /// <summary>The size along Y.</summary>
    public double Width { get; }

    /// <summary>The size along Z (build123d's <c>box_height</c>).</summary>
    public double BoxHeight { get; }
}

/// <summary>A full or partial cone of a bottom and top radius (which may be zero) and a height; centred by default.</summary>
public class Cone : BasePartObject
{
    /// <summary>A cone; <paramref name="arcSize"/> is its angular size in degrees.</summary>
    public Cone(double bottomRadius, double topRadius, double height, double arcSize = 360, Rotation? rotation = null,
                Alignment? align = null, Mode mode = Mode.Add)
        : base(Solid.MakeCone(bottomRadius, topRadius, height, angle: arcSize), rotation, align ?? Centered, mode)
    {
        (BottomRadius, TopRadius, ConeHeight, ArcSize, Align) = (bottomRadius, topRadius, height, arcSize, align ?? Centered);
    }

    /// <summary>The bottom radius.</summary>
    public double BottomRadius { get; }

    /// <summary>The top radius.</summary>
    public double TopRadius { get; }

    /// <summary>The height (build123d's <c>cone_height</c>).</summary>
    public double ConeHeight { get; }

    /// <summary>The angular size in degrees.</summary>
    public double ArcSize { get; }

    /// <summary>The alignment.</summary>
    public Alignment Align { get; }
}

/// <summary>A full or partial cylinder of a radius and height; centred by default.</summary>
public class Cylinder : BasePartObject
{
    /// <summary>A cylinder; <paramref name="arcSize"/> is its angular size in degrees.</summary>
    public Cylinder(double radius, double height, double arcSize = 360, Rotation? rotation = null, Alignment? align = null,
                    Mode mode = Mode.Add)
        : base(Solid.MakeCylinder(radius, height, angle: arcSize), rotation, align ?? Centered, mode)
    {
        (Radius, CylinderHeight, ArcSize, Align) = (radius, height, arcSize, align ?? Centered);
    }

    /// <summary>The radius.</summary>
    public double Radius { get; }

    /// <summary>The height (build123d's <c>cylinder_height</c>).</summary>
    public double CylinderHeight { get; }

    /// <summary>The angular size in degrees.</summary>
    public double ArcSize { get; }

    /// <summary>The alignment.</summary>
    public Alignment Align { get; }
}

/// <summary>A full or partial sphere of a radius; centred by default.</summary>
public class Sphere : BasePartObject
{
    /// <summary>A sphere from latitude <paramref name="arcSize1"/> to <paramref name="arcSize2"/>, revolved through <paramref name="arcSize3"/> degrees.</summary>
    public Sphere(double radius, double arcSize1 = -90, double arcSize2 = 90, double arcSize3 = 360, Rotation? rotation = null,
                  Alignment? align = null, Mode mode = Mode.Add)
        : base(Solid.MakeSphere(radius, angle1: arcSize1, angle2: arcSize2, angle3: arcSize3), rotation, align ?? Centered, mode)
    {
        (Radius, ArcSize1, ArcSize2, ArcSize3, Align) = (radius, arcSize1, arcSize2, arcSize3, align ?? Centered);
    }

    /// <summary>The radius.</summary>
    public double Radius { get; }

    /// <summary>The angular size of the bottom hemisphere.</summary>
    public double ArcSize1 { get; }

    /// <summary>The angular size of the top hemisphere.</summary>
    public double ArcSize2 { get; }

    /// <summary>The angle revolved about the pole.</summary>
    public double ArcSize3 { get; }

    /// <summary>The alignment.</summary>
    public Alignment Align { get; }
}

/// <summary>A full or partial torus of a major and minor radius; centred by default.</summary>
public class Torus : BasePartObject
{
    /// <summary>A torus whose minor arc runs from <paramref name="minorStartAngle"/> to <paramref name="minorEndAngle"/>, revolved through <paramref name="majorAngle"/> degrees.</summary>
    public Torus(double majorRadius, double minorRadius, double minorStartAngle = 0, double minorEndAngle = 360, double majorAngle = 360,
                 Rotation? rotation = null, Alignment? align = null, Mode mode = Mode.Add)
        : base(Solid.MakeTorus(majorRadius, minorRadius, startAngle: minorStartAngle, endAngle: minorEndAngle, majorAngle: majorAngle),
               rotation, align ?? Centered, mode)
    {
        (MajorRadius, MinorRadius, MinorStartAngle, MinorEndAngle, MajorAngle, Align) =
            (majorRadius, minorRadius, minorStartAngle, minorEndAngle, majorAngle, align ?? Centered);
    }

    /// <summary>The major radius.</summary>
    public double MajorRadius { get; }

    /// <summary>The minor radius.</summary>
    public double MinorRadius { get; }

    /// <summary>Where the minor arc starts.</summary>
    public double MinorStartAngle { get; }

    /// <summary>Where the minor arc ends.</summary>
    public double MinorEndAngle { get; }

    /// <summary>The angle the minor arc is revolved through.</summary>
    public double MajorAngle { get; }

    /// <summary>The alignment.</summary>
    public Alignment Align { get; }
}

/// <summary>
/// A wedge: a near face of <c>xsize</c> by <c>zsize</c>, a far face spanning <c>xmin</c> to
/// <c>xmax</c> and <c>zmin</c> to <c>zmax</c>, and a depth of <c>ysize</c>.
/// </summary>
public class Wedge : BasePartObject
{
    /// <summary>A wedge; centred by default.</summary>
    /// <exception cref="ArgumentException">A size isn't positive.</exception>
    public Wedge(double xsize, double ysize, double zsize, double xmin, double zmin, double xmax, double zmax, Rotation? rotation = null,
                 Alignment? align = null, Mode mode = Mode.Add)
        : base(Make(xsize, ysize, zsize, xmin, zmin, xmax, zmax), rotation, align ?? Centered, mode)
    {
        (XSize, YSize, ZSize, XMin, ZMin, XMax, ZMax, Align) = (xsize, ysize, zsize, xmin, zmin, xmax, zmax, align ?? Centered);
    }

    private static Solid Make(double xsize, double ysize, double zsize, double xmin, double zmin, double xmax, double zmax)
    {
        if (xsize <= 0 || ysize <= 0 || zsize <= 0)
            throw new ArgumentException("xsize, ysize & zsize must all be greater than zero");
        return Solid.MakeWedge(xsize, ysize, zsize, xmin, zmin, xmax, zmax);
    }

#pragma warning disable CS1591 // The constructor's parameters
    public double XSize { get; }
    public double YSize { get; }
    public double ZSize { get; }
    public double XMin { get; }
    public double ZMin { get; }
    public double XMax { get; }
    public double ZMax { get; }
#pragma warning restore CS1591

    /// <summary>The alignment.</summary>
    public Alignment Align { get; }
}

/// <summary>The convex solid enclosing a set of points (their convex hull); not aligned by default.</summary>
public class ConvexPolyhedron : BasePartObject
{
    /// <summary>The convex hull of the points.</summary>
    public ConvexPolyhedron(IEnumerable<Vector> points, Rotation? rotation = null, Alignment? align = null, Mode mode = Mode.Add)
        : base(Make(points), rotation, align ?? Build123d.Align.None, mode)
    {
    }

    private static Shape Make(IEnumerable<Vector> points)
    {
        var pts = points.ToList();
        var faces = ConvexHull3D.Facets(pts).Select(f => new Face(Wire.MakePolygon(f.Select(i => pts[i])))).ToList();
        return new Solid(new Shell(faces)).Clean();
    }
}

/// <summary>The base of the hole operations, which cut into a builder's part.</summary>
public abstract class BaseHole : BasePartObject
{
    private protected BaseHole(Shape part, Alignment? align, Mode mode) : base(part, null, align, mode) { }

    /// <summary>A depth that goes right through the builder's part; without a builder there is none.</summary>
    /// <exception cref="ArgumentException">There is no builder (algebra mode), so a depth must be given.</exception>
    private protected static double ThroughHoleDepth() => throw new ArgumentException("No depth provided");
}

/// <summary>A counterbored hole: a hole of <c>radius</c>, and a counterbore of a larger radius and a depth.</summary>
public class CounterBoreHole : BaseHole
{
    /// <summary>A counterbored hole, down from the origin; through the builder's part if <paramref name="depth"/> is null.</summary>
    public CounterBoreHole(double radius, double counterBoreRadius, double counterBoreDepth, double? depth = null, Mode mode = Mode.Subtract)
        : this(radius, counterBoreRadius, counterBoreDepth, depth ?? ThroughHoleDepth(), mode, true) { }

    private CounterBoreHole(double radius, double counterBoreRadius, double counterBoreDepth, double depth, Mode mode, bool _)
        : base(Make(radius, counterBoreRadius, counterBoreDepth, depth), null, mode)
    {
        (Radius, CounterBoreRadius, CounterBoreDepth, HoleDepth) = (radius, counterBoreRadius, counterBoreDepth, depth);
    }

    private static Shape Make(double radius, double counterBoreRadius, double counterBoreDepth, double depth) =>
        Solid.MakeCylinder(radius, depth, new Plane((0, 0, 0), zDir: (0, 0, -1)))
            .Fuse(Solid.MakeCylinder(counterBoreRadius, counterBoreDepth + depth, new Plane((0, 0, -counterBoreDepth))));

#pragma warning disable CS1591 // The constructor's parameters
    public double Radius { get; }
    public double CounterBoreRadius { get; }
    public double CounterBoreDepth { get; }
    public double HoleDepth { get; }
#pragma warning restore CS1591
}

/// <summary>A countersunk hole: a hole of <c>radius</c>, and a countersink of a larger radius and a cone angle.</summary>
public class CounterSinkHole : BaseHole
{
    /// <summary>A countersunk hole, down from the origin; through the builder's part if <paramref name="depth"/> is null.</summary>
    public CounterSinkHole(double radius, double counterSinkRadius, double? depth = null, double counterSinkAngle = 82, Mode mode = Mode.Subtract)
        : this(radius, counterSinkRadius, depth ?? ThroughHoleDepth(), counterSinkAngle, mode, true) { }

    private CounterSinkHole(double radius, double counterSinkRadius, double depth, double counterSinkAngle, Mode mode, bool _)
        : base(Make(radius, counterSinkRadius, depth, counterSinkAngle), null, mode)
    {
        (Radius, CounterSinkRadius, HoleDepth, CounterSinkAngle) = (radius, counterSinkRadius, depth, counterSinkAngle);
    }

    private static Shape Make(double radius, double counterSinkRadius, double depth, double angle)
    {
        var coneHeight = counterSinkRadius / Math.Tan(angle / 2.0 * Constants.DegToRad);
        var down = new Plane((0, 0, 0), zDir: (0, 0, -1));
        return Solid.MakeCylinder(radius, depth, down)
            .Fuse(Solid.MakeCone(counterSinkRadius, 0.0, coneHeight, down), Solid.MakeCylinder(counterSinkRadius, depth));
    }

#pragma warning disable CS1591 // The constructor's parameters
    public double Radius { get; }
    public double CounterSinkRadius { get; }
    public double HoleDepth { get; }
    public double CounterSinkAngle { get; }
#pragma warning restore CS1591
}

/// <summary>A hole of a radius and depth.</summary>
public class Hole : BaseHole
{
    /// <summary>
    /// A hole down from the origin. With <paramref name="depth"/> null it goes right through the builder's part, starting
    /// above it.
    /// </summary>
    public Hole(double radius, double? depth = null, Mode mode = Mode.Subtract)
        : this(radius, 2 * (depth ?? ThroughHoleDepth()), depth is null, mode) { }

    private Hole(double radius, double holeDepth, bool through, Mode mode)
        : base(Solid.MakeCylinder(radius, holeDepth, new Plane(through ? (0, 0, holeDepth / 2) : (0, 0, 0), zDir: (0, 0, -1))),
               Centered, mode)
    {
        (Radius, HoleDepth) = (radius, holeDepth);
    }

    /// <summary>The radius.</summary>
    public double Radius { get; }

    /// <summary>The depth of the cylinder cut: twice the depth given.</summary>
    public double HoleDepth { get; }
}

/// <summary>Computes the triangular facets of a 3D convex hull, as <c>scipy.spatial.ConvexHull(points).simplices</c> does.</summary>
internal static class ConvexHull3D
{
    /// <summary>The hull's facets as point-index triples, outward-facing (incremental hull; coplanar points left out).</summary>
    internal static List<int[]> Facets(IReadOnlyList<Vector> points)
    {
        var scale = points.Max(p => Math.Max(Math.Abs(p.X), Math.Max(Math.Abs(p.Y), Math.Abs(p.Z))));
        var eps = 1e-10 * Math.Max(1, scale);
        // An initial tetrahedron of four points not in one plane
        int a = 0, b = -1, c = -1, d = -1;
        for (int i = 1; i < points.Count && b < 0; i++)
            if ((points[i] - points[a]).Length > eps)
                b = i;
        for (int i = 1; i < points.Count && c < 0 && b >= 0; i++)
            if ((points[b] - points[a]).Cross(points[i] - points[a]).Length > eps)
                c = i;
        for (int i = 1; i < points.Count && d < 0 && c >= 0; i++)
            if (Math.Abs(Volume(points, a, b, c, i)) > eps)
                d = i;
        if (d < 0)
            throw new ArgumentException("The points of a convex polyhedron must not all lie in one plane");
        var faces = new List<int[]>();
        void AddFacet(int p, int q, int r, int inside)
        {
            // Orient each facet away from a point inside the hull
            faces.Add(Volume(points, p, q, r, inside) > 0 ? [p, r, q] : [p, q, r]);
        }
        AddFacet(a, b, c, d);
        AddFacet(a, b, d, c);
        AddFacet(a, c, d, b);
        AddFacet(b, c, d, a);
        for (int i = 0; i < points.Count; i++)
        {
            if (i == a || i == b || i == c || i == d)
                continue;
            var visible = faces.Where(f => Volume(points, f[0], f[1], f[2], i) > eps).ToList();
            if (visible.Count == 0)
                continue;   // inside the hull (or on it)
            // The horizon: edges of visible facets not shared with another visible facet
            var edges = new Dictionary<(int, int), int>();
            foreach (var f in visible)
                for (int k = 0; k < 3; k++)
                {
                    var e = (f[k], f[(k + 1) % 3]);
                    edges[e] = edges.GetValueOrDefault(e) + 1;
                }
            faces.RemoveAll(visible.Contains);
            foreach (var ((p, q), _) in edges.Where(e => !edges.ContainsKey((e.Key.Item2, e.Key.Item1))))
                faces.Add([p, q, i]);
        }
        return faces;
    }

    /// <summary>Six times the signed volume of the tetrahedron (p, q, r, s): positive when s is in front of the facet p, q, r.</summary>
    private static double Volume(IReadOnlyList<Vector> pts, int p, int q, int r, int s) =>
        (pts[q] - pts[p]).Cross(pts[r] - pts[p]).Dot(pts[s] - pts[p]);
}
