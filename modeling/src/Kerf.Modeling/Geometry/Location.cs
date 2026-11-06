using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Modeling;

/// <summary>
/// An immutable position and orientation in 3D space, absolute or relative depending on use. It is the
/// preferred way to place objects.
/// </summary>
/// <remarks>
/// A location holds one OCCT TopLoc_Location for its lifetime. OCCT compares shape locations by
/// identity, so shapes placed with the same <see cref="Location"/> instance share their location, as in
/// build123d. Equality here compares position and rotation rounded to five decimal places.
/// </remarks>
public class Location : IEquatable<Location>, IFormattable, IGeometry
{
    /// <summary>The OCCT location. Never modified: TopLoc_Location's mutators must not be called on it.</summary>
    internal Placement Wrapped { get; }

    /// <summary>The identity location: no translation or rotation.</summary>
    public Location() : this(new Placement(), owned: true) { }

    /// <summary>A copy of another location, sharing its OCCT location.</summary>
    public Location(Location location) : this(location.Wrapped, owned: true) { }

    /// <summary>A location from an OCCT TopLoc_Location (keeping its identity).</summary>
    public Location(Placement location) : this(location, owned: true) { }

    /// <summary>A location from an OCCT transformation.</summary>
    public Location(Transform3D transform) : this(new Placement(transform), owned: true) { }

    /// <summary>A location at a position, without rotation.</summary>
    public Location(Point3D position) : this(Build(position, trsf => trsf), owned: true) { }

    /// <summary>A location at an offset from the origin, without rotation.</summary>
    public Location(Vector3D offset) : this(offset.ToPoint()) { }

    /// <summary>A location at a position, rotated about the Z axis by an angle in degrees.</summary>
    public Location(Point3D position, double angle) : this(position, new Vector3D(0, 0, 1), angle) { }

    /// <summary>A location at a position, rotated about <paramref name="direction"/> by an angle in degrees.</summary>
    public Location(Point3D position, Vector3D direction, double angle)
        : this(Build(position, trsf =>
        {
            var origin = new Point3D(0, 0, 0);
            var dir = direction.ToDirection();
            var axis = new Axis3D(origin, dir);
            return trsf.WithRotation(axis, angle * Constants.DegToRad);
        }), owned: true)
    {
    }

    /// <summary>
    /// A location at a position, oriented by Euler angles in degrees applied in <paramref name="ordering"/>
    /// (default <see cref="Intrinsic.XYZ"/>).
    /// </summary>
    public Location(Point3D position, Vector3D orientation, RotationOrder ordering = default)
        : this(Build(position, _ => Transform3D.Rotation(ordering.ToOcct(), orientation.X * Constants.DegToRad,
                                                         orientation.Y * Constants.DegToRad, orientation.Z * Constants.DegToRad)),
               owned: true)
    {
    }

    /// <summary>The location of a plane: at its origin, with its axes.</summary>
    public Location(Plane plane) : this(FromPlane(plane), owned: true) { }

    /// <summary>Takes ownership of a TopLoc_Location without copying it.</summary>
    private protected Location(Placement location, bool owned) => Wrapped = location;

    /// <summary>Wraps a new TopLoc_Location this location takes ownership of.</summary>
    static Location Own(Placement location) => new(location, owned: true);

    static Placement Build(Point3D position, Func<Transform3D, Transform3D> rotate)
    {
        var trsf = rotate(new Transform3D());
        var translation = position.ToVector();
        trsf = trsf.WithTranslationPart(translation);
        return new Placement(trsf);
    }

    static Placement FromPlane(Plane plane)
    {
        var origin = plane.Origin;
        var z = plane.ZDir.ToDirection();
        var x = plane.XDir.ToDirection();
        var cs = new CoordinateSystem(origin, z, x);
        var trsf = new Transform3D();
        trsf = trsf.WithTransformation(cs);
        trsf = trsf.Inverted();
        return new Placement(trsf);
    }

    /// <summary>The OCCT location (the same identity).</summary>
    public Placement ToPlacement() => Wrapped;

    /// <summary>A new OCCT gp_Trsf for this location's transformation.</summary>
    public Transform3D ToTransform3D() => Wrapped.Transformation;

    /// <summary>The position (translation) part.</summary>
    public Point3D Position
    {
        get
        {
            return Wrapped.Transformation.TranslationPart.ToPoint();
        }
    }

    /// <summary>The orientation part, as intrinsic XYZ Euler angles in degrees.</summary>
    public Vector3D Orientation
    {
        get
        {
            var (a, b, c) = Wrapped.Transformation.GetEulerAngles(EulerSequence.IntrinsicXYZ);
            return new Vector3D(a * Constants.RadToDeg, b * Constants.RadToDeg, c * Constants.RadToDeg);
        }
    }

    /// <summary>Deconstructs into position and orientation (intrinsic XYZ Euler angles in degrees).</summary>
    public void Deconstruct(out Point3D position, out Vector3D orientation) => (position, orientation) = (Position, Orientation);

    /// <summary>This location with a new position and the same orientation.</summary>
    public Location WithPosition(Point3D position)
    {
        var trsfPosition = new Transform3D();
        var v = position.ToVector();
        trsfPosition = trsfPosition.WithTranslationPart(v);
        var trsfOrientation = Wrapped.Transformation.ToRotation();
        var combined = trsfPosition.Multiplied(trsfOrientation);
        return Own(new Placement(combined));
    }

    /// <summary>This location with a new orientation (intrinsic XYZ Euler angles in degrees) and the same position.</summary>
    public Location WithOrientation(Vector3D orientation)
    {
        var trsfPosition = new Transform3D();
        var v = Wrapped.Transformation.TranslationPart;
        trsfPosition = trsfPosition.WithTranslationPart(v);
        var trsfOrientation = Transform3D.Rotation(EulerSequence.IntrinsicXYZ, orientation.X * Constants.DegToRad,
                                                   orientation.Y * Constants.DegToRad, orientation.Z * Constants.DegToRad);
        var combined = trsfPosition.Multiplied(trsfOrientation);
        return Own(new Placement(combined));
    }

    /// <summary>The X axis of this location used as a plane.</summary>
    public Axis XAxis { get { var p = new Plane(this); return new Axis(p.Origin, p.XDir); } }

    /// <summary>The Y axis of this location used as a plane.</summary>
    public Axis YAxis { get { var p = new Plane(this); return new Axis(p.Origin, p.YDir); } }

    /// <summary>The Z axis of this location used as a plane.</summary>
    public Axis ZAxis { get { var p = new Plane(this); return new Axis(p.Origin, p.ZDir); } }

    /// <summary>The inverse location.</summary>
    public Location Inverse() => Own(Wrapped.Inverted());

    /// <summary>This location applied <paramref name="exponent"/> times (negative for the inverse).</summary>
    public Location Pow(int exponent) => Own(Wrapped.Powered(exponent));

    /// <summary>The centre of a location is its position, which is useful for sorting.</summary>
    public Point3D Center() => Position;

    /// <summary>
    /// This location mirrored across a plane. A true mirror image is left-handed, so the X and Z
    /// directions are mirrored and Y is recomputed as Z × X to keep the frame right-handed.
    /// </summary>
    public Location Mirror(Plane mirrorPlane)
    {
        Vector3D MirrorDir(Vector3D v) => v - 2 * v.Dot(mirrorPlane.ZDir) * mirrorPlane.ZDir;

        var toPlane = Position - mirrorPlane.Origin;
        double distance = toPlane.Dot(mirrorPlane.ZDir);
        var pos = Position - 2 * distance * mirrorPlane.ZDir;

        var locPlane = new Plane(this);
        return new Location(Plane.Create(pos, MirrorDir(locPlane.XDir), MirrorDir(locPlane.ZDir)));
    }

#pragma warning disable CS1591 // Operators are self-describing
    /// <summary>The composition: <paramref name="b"/> applied within <paramref name="a"/>.</summary>
    public static Location operator *(Location a, Location b) => Own(a.Wrapped.Multiplied(b.Wrapped));

    /// <summary>This location composed with each location in turn.</summary>
    public static IReadOnlyList<Location> operator *(Location a, IEnumerable<Location> locations) =>
        locations.Select(b => a * b).ToArray();

    /// <summary>The same position with the orientation flipped (Z reversed).</summary>
    public static Location operator -(Location a) => new(-new Plane(a));

    public static bool operator ==(Location? a, Location? b) => a is null ? b is null : a.Equals(b);
    public static bool operator !=(Location? a, Location? b) => !(a == b);


    /// <summary>Where a shape meets this location (the shape's <c>Intersect</c>), empty if nowhere.</summary>
    public ShapeList<Shape> Intersect(Shape shape) => shape.Intersect(this);
#pragma warning restore CS1591

    /// <summary>The point if it is this location's position, else null.</summary>
    public Point3D? Intersect(Point3D point) => Position.IsNear(point) ? point : null;

    /// <summary>This location if the two are equal, the shared position if only that matches, else null.</summary>
    public IGeometry? Intersect(Location location) =>
        this == location ? this : Position.IsNear(location.Position) ? location.Position : null;

    /// <summary>The intersection with an axis (see <see cref="Axis.Intersect(Location)"/>).</summary>
    public IGeometry? Intersect(Axis axis) => axis.Intersect(this);

    /// <summary>The intersection with a plane (see <see cref="Plane.Intersect(Location)"/>).</summary>
    public IGeometry? Intersect(Plane plane) => plane.Intersect(this);

    internal ((double, double, double), (double, double, double, double)) Key
    {
        get
        {
            return (Position.Key, GeomKey.Quaternion(Wrapped.Transformation.GetQuaternion()));
        }
    }

    /// <inheritdoc/>
    public bool Equals(Location? other) => other is not null && Key == other.Key;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is Location l && Equals(l);

    /// <inheritdoc/>
    public override int GetHashCode() => Key.GetHashCode();

    /// <summary>"Location: (position=(1, 2, 3), orientation=(10, 20, 30))".</summary>
    public override string ToString() =>
        $"Location: (position={Position.Describe("G6")}, orientation={Orientation.Describe("G6")})";

    /// <summary>"(position, orientation)" with components formatted as <c>Describe(format)</c> does.</summary>
    public string ToString(string? format, IFormatProvider? formatProvider = null) =>
        $"({Position.Describe(format)}, {Orientation.Describe(format)})";
}

/// <summary>A <see cref="Location"/> that only rotates, about the origin.</summary>
public class Rotation : Location
{
    /// <summary>A rotation by Euler angles in degrees, applied in <paramref name="ordering"/> (default <see cref="Intrinsic.XYZ"/>).</summary>
    public Rotation(double x = 0, double y = 0, double z = 0, RotationOrder ordering = default)
        : base(new Point3D(0, 0, 0), new Vector3D(x, y, z), ordering)
    {
    }

    /// <summary>A rotation by Euler angles (X, Y, Z) in degrees, applied in <paramref name="ordering"/>.</summary>
    public Rotation(Vector3D angles, RotationOrder ordering = default) : base(new Point3D(0, 0, 0), angles, ordering) { }

    /// <summary>A rotation about an axis by an angle in degrees.</summary>
    public Rotation(Axis axis, double angle) : base(AboutAxis(axis, angle), owned: true) { }

    /// <summary>A copy of another rotation.</summary>
    public Rotation(Rotation rotation) : base(rotation) { }


    static Placement AboutAxis(Axis axis, double angle)
    {
        var trsf = new Transform3D();
        if (angle != 0.0)
        {
            var ax1 = axis.ToAxis3D();
            trsf = trsf.WithRotation(ax1, angle * Constants.DegToRad);
        }
        return new Placement(trsf);
    }
}

/// <summary>A <see cref="Location"/> that only translates.</summary>
public class Pos : Location
{
    /// <summary>A translation by (x, y, z).</summary>
    public Pos(double x = 0, double y = 0, double z = 0) : base(new Point3D(x, y, z)) { }

    /// <summary>A translation by a vector.</summary>
    public Pos(Vector3D v) : base(v) { }

    /// <summary>A translation to a point.</summary>
    public Pos(Point3D p) : base(p) { }
}
