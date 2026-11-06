namespace Kerf.Modeling;

/// <summary>An immutable axis: a position and a unit direction.</summary>
/// <remarks>Equality compares position and direction rounded to five decimal places.</remarks>
public readonly struct Axis : IEquatable<Axis>, IFormattable, IGeometry
{
    /// <summary>The global position of the axis origin.</summary>
    public Point3D Position { get; }

    /// <summary>The unit direction.</summary>
    public Direction3D Direction { get; }

    /// <summary>The X axis.</summary>
    public static Axis X => new(Point3D.Origin, new Vector3D(1, 0, 0));

    /// <summary>The Y axis.</summary>
    public static Axis Y => new(Point3D.Origin, new Vector3D(0, 1, 0));

    /// <summary>The Z axis.</summary>
    public static Axis Z => new(Point3D.Origin, new Vector3D(0, 0, 1));

    /// <summary>An axis through <paramref name="origin"/> along <paramref name="direction"/>.</summary>
    /// <exception cref="ArgumentException">The direction has zero length.</exception>
    public Axis(Point3D origin, Vector3D direction)
    {
        Direction3D unit;
        try
        {
            // As gp_Dir(*direction.normalized()): normalised, then normalised again by gp_Dir
            unit = direction.Normalized().ToDirection();
        }
        catch (Exception ex) when (Failures.IsKernelFailure(ex) || ex is InvalidOperationException)
        {
            throw new ArgumentException("Invalid Axis parameters", nameof(direction), ex);
        }
        Position = origin;
        Direction = unit;
    }

    /// <summary>An axis from an OCCT gp_Ax1.</summary>
    public Axis(Axis3D axis)
    {
        var loc = axis.Location;
        var dir = axis.Direction;
        Position = loc;
        Direction = dir;
    }

    /// <summary>The axis along an edge's curve where it starts (ignoring the edge's orientation), as build123d's <c>Axis(edge)</c>.</summary>
    public Axis(Edge edge)
    {
        if (!edge.TryGetCurve(out var curve, out _))
            throw new ArgumentException($"Invalid edge argument: {edge}");
        var start = curve.Derivatives(edge.Range.Min, 1);
        this = new Axis(start.Point, start.D1);
    }

    /// <summary>The Z axis of a location.</summary>
    public Axis(Location location) => this = Z.Located(location);

    // Exact copy without re-normalising, for values OCCT has already produced
    Axis(Point3D position, Direction3D direction, bool _) => (Position, Direction) = (position, direction);

    /// <summary>An axis from <paramref name="origin"/> towards <paramref name="endPoint"/>.</summary>
    public static Axis FromPoints(Point3D origin, Point3D endPoint) => new(origin, endPoint - origin);

    /// <summary>This axis moved to a new position, with the same direction.</summary>
    public Axis WithPosition(Point3D position) => new(position, Direction, true);

    /// <summary>This axis with a new direction (normalised), at the same position.</summary>
    /// <exception cref="Exception">The direction has zero length (the kernel's construction error).</exception>
    public Axis WithDirection(Vector3D direction)
    {
        var d = direction.ToDirection();
        return new Axis(Position, d, true);
    }

    /// <summary>This axis as a location: at its position, with its direction as Z.</summary>
    public Location Location => new(new Plane(Position, zDir: Direction));

    /// <summary>A new OCCT gp_Ax1 for this axis.</summary>
    public Axis3D ToAxis3D()
    {
        var p = Position;
        var d = Direction.ToDirection();
        return new Axis3D(p, d);
    }

    /// <summary>This axis moved to a new location, which can change both position and direction.</summary>
    public Axis Located(Location location)
    {
        var ax1 = ToAxis3D();
        var trsf = location.Wrapped.Transformation;
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
        var a = ToAxis3D();
        var b = other.ToAxis3D();
        return a.IsCoaxial(b, Rad(angularTolerance), linearTolerance);
    }

    /// <summary>True if the axes are at 90° to each other, within the tolerance in degrees.</summary>
    public bool IsNormal(Axis other, double angularTolerance = 1e-5)
    {
        var a = ToAxis3D();
        var b = other.ToAxis3D();
        return a.IsNormal(b, Rad(angularTolerance));
    }

    /// <summary>True if the axes are parallel and point opposite ways, within the tolerance in degrees.</summary>
    public bool IsOpposite(Axis other, double angularTolerance = 1e-5)
    {
        var a = ToAxis3D();
        var b = other.ToAxis3D();
        return a.IsOpposite(b, Rad(angularTolerance));
    }

    /// <summary>True if the axes are parallel, pointing the same or opposite ways, within the tolerance in degrees.</summary>
    public bool IsParallel(Axis other, double angularTolerance = 1e-5)
    {
        var a = ToAxis3D();
        var b = other.ToAxis3D();
        return a.IsParallel(b, Rad(angularTolerance));
    }

    /// <summary>True if the axes are skew: neither parallel nor coplanar, so they never meet.</summary>
    public bool IsSkew(Axis other, double tolerance = 1e-5)
    {
        if (IsParallel(other, tolerance))
        {
            // Parallel axes are skew only if distinct
            var parallelOffset = (Position - other.Position).Cross(Direction);
            return parallelOffset.Magnitude > tolerance;
        }

        // Coplanar when the offset between them lies in the plane their directions span
        var coplanarity = (Position - other.Position).Dot(Direction.Cross(other.Direction));
        return Math.Abs(coplanarity) > tolerance;
    }

    /// <summary>The angle between the directions, in degrees (0 to 180).</summary>
    public double AngleBetween(Axis other)
    {
        var a = ToAxis3D();
        var b = other.ToAxis3D();
        return a.AngleTo(b) * Constants.RadToDeg;
    }

    /// <summary>This axis with its direction reversed.</summary>
    public Axis Reverse() => new(Position, -Direction, true);

#pragma warning disable CS1591 // Operators are self-describing
    public static Axis operator -(Axis a) => a.Reverse();
    public static bool operator ==(Axis a, Axis b) => a.Equals(b);
    public static bool operator !=(Axis a, Axis b) => !a.Equals(b);


    /// <summary>Where a shape meets this axis (the shape's <c>Intersect</c>), empty if nowhere.</summary>
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
    public Point3D? Intersect(Point3D point)
    {
        var toPoint = point - Position;
        var projected = Position + Direction * toPoint.Dot(Direction);
        return point.IsNear(projected) ? point : null;
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
        return locationDir.IsNear(Direction) ? location : location.Position;
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
        $"Axis: (position={Position.Describe("G6")}, direction={Direction.Describe("G6")})";

    /// <summary>"(position, direction)" with components formatted as <c>Describe(format)</c> does.</summary>
    public string ToString(string? format, IFormatProvider? formatProvider = null) =>
        $"({Position.Describe(format)}, {Direction.Describe(format)})";
}
