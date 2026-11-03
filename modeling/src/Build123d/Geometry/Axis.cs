using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Occt;

namespace Build123d;

/// <summary>An immutable axis: a position and a unit direction.</summary>
/// <remarks>Equality compares position and direction rounded to five decimal places.</remarks>
public readonly struct Axis : IEquatable<Axis>, IFormattable, IGeometry
{
    /// <summary>The global position of the axis origin.</summary>
    public Vector Position { get; }

    /// <summary>The unit direction.</summary>
    public Vector Direction { get; }

    /// <summary>The X axis.</summary>
    public static Axis X => new((0, 0, 0), (1, 0, 0));

    /// <summary>The Y axis.</summary>
    public static Axis Y => new((0, 0, 0), (0, 1, 0));

    /// <summary>The Z axis.</summary>
    public static Axis Z => new((0, 0, 0), (0, 0, 1));

    /// <summary>An axis through <paramref name="origin"/> along <paramref name="direction"/>.</summary>
    /// <exception cref="ArgumentException">The direction has zero length.</exception>
    public Axis(Vector origin, Vector direction)
    {
        Vector unit;
        try
        {
            // As gp_Dir(*direction.normalized()): normalised, then normalised again by gp_Dir
            var dir = direction.Normalized().ToGpDir();
            unit = new Vector(dir);
        }
        catch (FailureException ex)
        {
            throw new ArgumentException("Invalid Axis parameters", nameof(direction), ex);
        }
        Position = origin;
        Direction = unit;
    }

    /// <summary>An axis from an OCCT gp_Ax1.</summary>
    public Axis(Axis3D ax1)
    {
        var loc = ax1.Location();
        var dir = ax1.Direction();
        Position = new Vector(loc);
        Direction = new Vector(dir);
    }

    /// <summary>The axis along an edge's curve where it starts (ignoring the edge's orientation), as build123d's <c>Axis(edge)</c>.</summary>
    public Axis(Edge edge)
    {
        double first = 0, last = 0;
        var curve = Tool.Curve(edge.TopoEdge, out first, out last)
                          ?? throw new ArgumentException($"Invalid edge argument: {edge}");
        double min = 0, max = 0;
        Tool.Range(edge.TopoEdge, out min, out max);
        var origin = new Point3D();
        var tangent = new Vector3D();
        curve.D1(min, out origin, out tangent);
        this = new Axis(new Vector(origin), new Vector(tangent));
    }

    /// <summary>The Z axis of a location.</summary>
    public Axis(Location location) => this = Z.Located(location);

    // Exact copy without re-normalising, for values OCCT has already produced
    Axis(Vector position, Vector direction, bool _) => (Position, Direction) = (position, direction);

    /// <summary>An axis from <paramref name="origin"/> towards <paramref name="endPoint"/>.</summary>
    public static Axis FromPoints(Vector origin, Vector endPoint) => new(origin, endPoint - origin);

    /// <summary>This axis moved to a new position, with the same direction.</summary>
    public Axis WithPosition(Vector position) => new(position, Direction, true);

    /// <summary>This axis with a new direction (normalised), at the same position.</summary>
    /// <exception cref="FailureException">The direction has zero length.</exception>
    public Axis WithDirection(Vector direction)
    {
        var d = direction.ToGpDir();
        return new Axis(Position, new Vector(d), true);
    }

    /// <summary>This axis as a location: at its position, with its direction as Z.</summary>
    public Location Location => new(new Plane(Position, zDir: Direction));

    /// <summary>A new OCCT gp_Ax1 for this axis.</summary>
    public Axis3D ToGpAx1()
    {
        var p = Position.ToGpPnt();
        var d = Direction.ToGpDir();
        return new Axis3D(p, d);
    }

    /// <summary>This axis moved to a new location, which can change both position and direction.</summary>
    public Axis Located(Location location)
    {
        var ax1 = ToGpAx1();
        var trsf = location.Wrapped.Transformation();
        var moved = ax1.Transformed(trsf);
        return new Axis(moved);
    }

    static double Rad(double degrees) => degrees * (Math.PI / 180);

    /// <summary>
    /// True if the angle between the axes is at most <paramref name="angularTolerance"/> degrees and the
    /// distance between them at most <paramref name="linearTolerance"/>.
    /// </summary>
    public bool IsCoaxial(Axis other, double angularTolerance = 1e-5, double linearTolerance = 1e-5)
    {
        var a = ToGpAx1();
        var b = other.ToGpAx1();
        return a.IsCoaxial(b, Rad(angularTolerance), linearTolerance);
    }

    /// <summary>True if the axes are at 90° to each other, within the tolerance in degrees.</summary>
    public bool IsNormal(Axis other, double angularTolerance = 1e-5)
    {
        var a = ToGpAx1();
        var b = other.ToGpAx1();
        return a.IsNormal(b, Rad(angularTolerance));
    }

    /// <summary>True if the axes are parallel and point opposite ways, within the tolerance in degrees.</summary>
    public bool IsOpposite(Axis other, double angularTolerance = 1e-5)
    {
        var a = ToGpAx1();
        var b = other.ToGpAx1();
        return a.IsOpposite(b, Rad(angularTolerance));
    }

    /// <summary>True if the axes are parallel, pointing the same or opposite ways, within the tolerance in degrees.</summary>
    public bool IsParallel(Axis other, double angularTolerance = 1e-5)
    {
        var a = ToGpAx1();
        var b = other.ToGpAx1();
        return a.IsParallel(b, Rad(angularTolerance));
    }

    /// <summary>True if the axes are skew: neither parallel nor coplanar, so they never meet.</summary>
    public bool IsSkew(Axis other, double tolerance = 1e-5)
    {
        if (IsParallel(other, tolerance))
        {
            // Parallel axes are skew only if distinct
            var parallelOffset = (Position - other.Position).Cross(Direction);
            return parallelOffset.Length > tolerance;
        }

        // Coplanar when the offset between them lies in the plane their directions span
        var coplanarity = (Position - other.Position).Dot(Direction.Cross(other.Direction));
        return Math.Abs(coplanarity) > tolerance;
    }

    /// <summary>The angle between the directions, in degrees (0 to 180).</summary>
    public double AngleBetween(Axis other)
    {
        var a = ToGpAx1();
        var b = other.ToGpAx1();
        return a.Angle(b) * Constants.RadToDeg;
    }

    /// <summary>This axis with its direction reversed.</summary>
    public Axis Reverse() => new(Position, -Direction, true);

#pragma warning disable CS1591 // Operators are self-describing
    public static Axis operator -(Axis a) => a.Reverse();
    public static bool operator ==(Axis a, Axis b) => a.Equals(b);
    public static bool operator !=(Axis a, Axis b) => !a.Equals(b);
    public static IGeometry? operator &(Axis a, Axis b) => a.Intersect(b);
    public static IGeometry? operator &(Axis a, Plane b) => a.Intersect(b);
    public static IGeometry? operator &(Axis a, Location b) => a.Intersect(b);
    public static Vector? operator &(Axis a, Vector b) => a.Intersect(b);

    /// <summary>Where a shape meets this axis (the shape's <see cref="Shape.Intersect(object[])"/>).</summary>
    public static ShapeList<Shape> operator &(Axis a, Shape b) => a.Intersect(b);

    /// <summary>Where a shape meets this axis (the shape's <see cref="Shape.Intersect(object[])"/>), empty if nowhere.</summary>
    public ShapeList<Shape> Intersect(Shape shape) => shape.Intersect(this);
#pragma warning restore CS1591

    /// <summary>
    /// The intersection with another axis: this axis if they are coaxial, the point where they cross,
    /// or null if they are skew or parallel.
    /// </summary>
    public IGeometry? Intersect(Axis axis)
    {
        if (IsCoaxial(axis))
            return this;
        if (IsSkew(axis))
            return null;

        // Solve p1 + t1 d1 = p2 + t2 d2 + t3 (d1 x d2) for t1. The columns d1, -d2 and d1 x d2 are
        // independent (the axes aren't parallel), so the system is square and full rank and Cramer's
        // rule gives the same answer as build123d's least-squares solve
        var d1 = Direction;
        var c1 = -axis.Direction;
        var c2 = d1.Cross(axis.Direction);
        var rhs = axis.Position - Position;
        var c1xc2 = c1.Cross(c2);
        double t1 = rhs.Dot(c1xc2) / d1.Dot(c1xc2);
        return Position + d1 * t1;
    }

    /// <summary>The intersection with a plane: this axis if it lies in the plane, the crossing point, or null.</summary>
    public IGeometry? Intersect(Plane plane) => plane.Intersect(this);

    /// <summary>The point if it lies on this axis, else null.</summary>
    public Vector? Intersect(Vector point)
    {
        var toPoint = point - Position;
        var projected = Direction * toPoint.Dot(Direction) + Position;
        return point == projected ? point : null;
    }

    /// <summary>
    /// The intersection with a location: the location if it lies on this axis with its Z along it, its
    /// position if it lies on the axis otherwise, or null.
    /// </summary>
    public IGeometry? Intersect(Location location)
    {
        var locationDir = new Plane(location).ZDir;
        if (Intersect(location.Position) is null)
            return null;
        return locationDir == Direction ? location : location.Position;
    }

    internal ((double, double, double), (double, double, double)) Key => (Position.Key, Direction.Key);

    /// <inheritdoc/>
    public bool Equals(Axis other) => Key == other.Key;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is Axis a && Equals(a);

    /// <inheritdoc/>
    public override int GetHashCode() => Key.GetHashCode();

    /// <summary>"Axis: (position=(0, 0, 0), direction=(1, 0, 0))".</summary>
    public override string ToString() =>
        $"Axis: (position={Position.ToString(".6g")}, direction={Direction.ToString(".6g")})";

    /// <summary>"(position, direction)" with components formatted as <see cref="Vector.ToString(string?, IFormatProvider?)"/>.</summary>
    public string ToString(string? format, IFormatProvider? formatProvider = null) =>
        $"({Position.ToString(format)}, {Direction.ToString(format)})";
}
