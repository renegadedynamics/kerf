using Kerf.Healing;
using Kerf.Construction;

namespace Kerf.Modeling;

/// <summary>
/// The base of the part objects (<see cref="Box"/>, <see cref="Cylinder"/>, …): a part made from a solid, aligned and then
/// rotated.
/// </summary>
public abstract class BasePartObject : Part
{
    /// <summary>A part object from a solid (or part), aligned by its bounding box and then rotated.</summary>
    /// <param name="part">The solid to make the object of.</param>
    /// <param name="rotation">Angles to rotate it about X, Y and Z.</param>
    /// <param name="align">How to align the solid's bounding box with the origin; null for no alignment.</param>
    protected BasePartObject(Shape part, Rotation? rotation = null, Alignment? align = null)
        : base(Prepare(part, rotation, align))
    {
        Rotation = rotation ?? new Rotation();
    }

    private static Shape Prepare(Shape part, Rotation? rotation, Alignment? align)
    {
        if (align is { } a)
            part = part.Moved(new Location(part.Bounds().ToAlignOffset(a)));
        var rotated = rotation is not null && rotation != new Rotation() ? part.Moved(rotation) : part;
        // Don't add extra layers of compound
        return rotated.TryAs<Compound>(out _) ? rotated : TopoUtil.MakeCompound([rotated]);
    }

    /// <summary>The rotation applied to the object.</summary>
    public Rotation Rotation { get; }

    /// <summary>A centred alignment on all three axes, the default of most part objects.</summary>
    internal static readonly Alignment Centered = new Alignment(Align.Center, Align.Center, Align.Center);
}

/// <summary>A box of a length (along X), width (along Y) and height (along Z); centred by default.</summary>
public class Box : BasePartObject
{
    /// <summary>A box; <paramref name="align"/> defaults to centred.</summary>
    public Box(double length, double width, double height, Rotation? rotation = null, Alignment? align = null)
        : base(Solid.MakeBox(length, width, height), rotation, align ?? Centered)
    {
        (Length, Width, Height) = (length, width, height);
    }

    /// <summary>The size along X.</summary>
    public double Length { get; }

    /// <summary>The size along Y.</summary>
    public double Width { get; }

    /// <summary>The size along Z (build123d's <c>box_height</c>).</summary>
    public double Height { get; }
}

/// <summary>A full or partial cone of a bottom and top radius (which may be zero) and a height; centred by default.</summary>
public class Cone : BasePartObject
{
    /// <summary>A cone; <paramref name="arcSize"/> is its angular size in degrees.</summary>
    public Cone(double bottomRadius, double topRadius, double height, double arcSize = 360, Rotation? rotation = null,
                Alignment? align = null)
        : base(Solid.MakeCone(bottomRadius, topRadius, height, angle: arcSize), rotation, align ?? Centered)
    {
        (BottomRadius, TopRadius, Height, ArcSize, Align) = (bottomRadius, topRadius, height, arcSize, align ?? Centered);
    }

    /// <summary>The bottom radius.</summary>
    public double BottomRadius { get; }

    /// <summary>The top radius.</summary>
    public double TopRadius { get; }

    /// <summary>The height (build123d's <c>cone_height</c>).</summary>
    public double Height { get; }

    /// <summary>The angular size in degrees.</summary>
    public double ArcSize { get; }

    /// <summary>How it is placed relative to its bounding box on each axis, as it was built.</summary>
    public Alignment Align { get; }
}

/// <summary>A full or partial cylinder of a radius and height; centred by default.</summary>
public class Cylinder : BasePartObject
{
    /// <summary>A cylinder; <paramref name="arcSize"/> is its angular size in degrees.</summary>
    public Cylinder(double radius, double height, double arcSize = 360, Rotation? rotation = null, Alignment? align = null)
        : base(Solid.MakeCylinder(radius, height, angle: arcSize), rotation, align ?? Centered)
    {
        (Radius, Height, ArcSize, Align) = (radius, height, arcSize, align ?? Centered);
    }

    /// <summary>The radius.</summary>
    public double Radius { get; }

    /// <summary>The height (build123d's <c>cylinder_height</c>).</summary>
    public double Height { get; }

    /// <summary>The angular size in degrees.</summary>
    public double ArcSize { get; }

    /// <summary>How it is placed relative to its bounding box on each axis, as it was built.</summary>
    public Alignment Align { get; }
}

/// <summary>A full or partial sphere of a radius; centred by default.</summary>
public class Sphere : BasePartObject
{
    /// <summary>A sphere from latitude <paramref name="arcSize1"/> to <paramref name="arcSize2"/>, revolved through <paramref name="arcSize3"/> degrees.</summary>
    public Sphere(double radius, double arcSize1 = -90, double arcSize2 = 90, double arcSize3 = 360, Rotation? rotation = null,
                  Alignment? align = null)
        : base(Solid.MakeSphere(radius, angle1: arcSize1, angle2: arcSize2, angle3: arcSize3), rotation, align ?? Centered)
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

    /// <summary>How it is placed relative to its bounding box on each axis, as it was built.</summary>
    public Alignment Align { get; }
}

/// <summary>A full or partial torus of a major and minor radius; centred by default.</summary>
public class Torus : BasePartObject
{
    /// <summary>A torus whose minor arc runs from <paramref name="minorStartAngle"/> to <paramref name="minorEndAngle"/>, revolved through <paramref name="majorAngle"/> degrees.</summary>
    public Torus(double majorRadius, double minorRadius, double minorStartAngle = 0, double minorEndAngle = 360, double majorAngle = 360,
                 Rotation? rotation = null, Alignment? align = null)
        : base(Solid.MakeTorus(majorRadius, minorRadius, startAngle: minorStartAngle, endAngle: minorEndAngle, majorAngle: majorAngle),
               rotation, align ?? Centered)
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

    /// <summary>How it is placed relative to its bounding box on each axis, as it was built.</summary>
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
    public Wedge(double xSize, double ySize, double zSize, double xMin, double zMin, double xMax, double zMax, Rotation? rotation = null,
                 Alignment? align = null)
        : base(Make(xSize, ySize, zSize, xMin, zMin, xMax, zMax), rotation, align ?? Centered)
    {
        (XSize, YSize, ZSize, XMin, ZMin, XMax, ZMax, Align) = (xSize, ySize, zSize, xMin, zMin, xMax, zMax, align ?? Centered);
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

    /// <summary>How it is placed relative to its bounding box on each axis, as it was built.</summary>
    public Alignment Align { get; }
}

/// <summary>
/// The convex solid enclosing a set of points (their convex hull), with one planar face for each flat side; not aligned
/// by default.
/// </summary>
/// <remarks>
/// Points closer than <see cref="Constants.Tolerance"/> count as one, and a corner within that distance of the hull of
/// the others is left out, so the solid is within that distance of the exact hull. Points within 5e-8 of a flat side's
/// plane are on it.
/// </remarks>
public class ConvexPolyhedron : BasePartObject
{
    /// <summary>The convex hull of the points.</summary>
    /// <exception cref="ArgumentException">
    /// The hull has no volume: fewer than four distinct points, or all of them on one line or in one plane (to within
    /// <see cref="Constants.Tolerance"/>); or a coordinate isn't finite. The message says which.
    /// </exception>
    public ConvexPolyhedron(IEnumerable<Point3D> points, Rotation? rotation = null, Alignment? align = null)
        : base(Make(points), rotation, align ?? Kerf.Modeling.Align.None)
    {
    }

    private static Shape Make(IEnumerable<Point3D> points)
    {
        var hull = ConvexHull3D.Sides(points);
        // The faces share their corners and edges as built, so nothing is sewn
        var polygons = hull.Sides.Select(s => (new Plane3D(s.Origin, new Direction3D(s.Normal)), s.Corners));
        var shell = Failures.Get(Shell.FromPolygons(hull.Points, polygons), () => "convex polyhedron: shell");
        var solid = Solid.Make(shell);
        var report = solid.Validate();
        if (!report.IsValid || solid.Volume <= 0)
            throw Failures.Invalid(solid, report, "The convex polyhedron");
        return solid;
    }
}

/// <summary>The base of the hole operations, which cut into a builder's part.</summary>
public abstract class BaseHole : BasePartObject
{
    private protected BaseHole(Shape part, Alignment? align) : base(part, null, align) { }

    /// <summary>A depth that goes right through the builder's part; without a builder there is none.</summary>
    /// <exception cref="ArgumentException">There is no builder (algebra mode), so a depth must be given.</exception>
    private protected static double ThroughHoleDepth() => throw new ArgumentException("No depth provided");
}

/// <summary>A counterbored hole: a hole of <c>radius</c>, and a counterbore of a larger radius and a depth.</summary>
public class CounterBoreHole : BaseHole
{
    /// <summary>A counterbored hole, down from the origin; through the builder's part if <paramref name="depth"/> is null.</summary>
    public CounterBoreHole(double radius, double counterBoreRadius, double counterBoreDepth, double? depth = null)
        : this(radius, counterBoreRadius, counterBoreDepth, depth ?? ThroughHoleDepth(), true) { }

    private CounterBoreHole(double radius, double counterBoreRadius, double counterBoreDepth, double depth, bool _)
        : base(Make(radius, counterBoreRadius, counterBoreDepth, depth), null)
    {
        (Radius, CounterBoreRadius, CounterBoreDepth, HoleDepth) = (radius, counterBoreRadius, counterBoreDepth, depth);
    }

    private static Shape Make(double radius, double counterBoreRadius, double counterBoreDepth, double depth) =>
        Solid.MakeCylinder(radius, depth, new Plane(new Point3D(0, 0, 0), zDir: new Vector3D(0, 0, -1)))
            .Fuse(Solid.MakeCylinder(counterBoreRadius, counterBoreDepth + depth, new Plane(new Point3D(0, 0, -counterBoreDepth))));

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
    public CounterSinkHole(double radius, double counterSinkRadius, double? depth = null, double counterSinkAngle = 82)
        : this(radius, counterSinkRadius, depth ?? ThroughHoleDepth(), counterSinkAngle, true) { }

    private CounterSinkHole(double radius, double counterSinkRadius, double depth, double counterSinkAngle, bool _)
        : base(Make(radius, counterSinkRadius, depth, counterSinkAngle), null)
    {
        (Radius, CounterSinkRadius, HoleDepth, CounterSinkAngle) = (radius, counterSinkRadius, depth, counterSinkAngle);
    }

    private static Shape Make(double radius, double counterSinkRadius, double depth, double angle)
    {
        var coneHeight = counterSinkRadius / Math.Tan(angle / 2.0 * Constants.DegToRad);
        var down = new Plane(new Point3D(0, 0, 0), zDir: new Vector3D(0, 0, -1));
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
    public Hole(double radius, double? depth = null)
        : this(radius, 2 * (depth ?? ThroughHoleDepth()), depth is null) { }

    private Hole(double radius, double holeDepth, bool through)
        : base(Solid.MakeCylinder(radius, holeDepth, new Plane(through ? new Point3D(0, 0, holeDepth / 2) : new Point3D(0, 0, 0), zDir: new Vector3D(0, 0, -1))),
               Centered)
    {
        (Radius, HoleDepth) = (radius, holeDepth);
    }

    /// <summary>The radius.</summary>
    public double Radius { get; }

    /// <summary>The depth of the cylinder cut: twice the depth given.</summary>
    public double HoleDepth { get; }
}

