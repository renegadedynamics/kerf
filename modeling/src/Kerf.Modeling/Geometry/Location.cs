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
    public Location(Transform3D trsf) : this(new Placement(trsf), owned: true) { }

    /// <summary>A location at a position, without rotation.</summary>
    public Location(Vector position) : this(Build(position, trsf => trsf), owned: true) { }

    /// <summary>A location at a position, rotated about the Z axis by an angle in degrees.</summary>
    public Location(Vector position, double angle) : this(position, new Vector(0, 0, 1), angle) { }

    /// <summary>A location at a position, rotated about <paramref name="direction"/> by an angle in degrees.</summary>
    public Location(Vector position, Vector direction, double angle)
        : this(Build(position, trsf =>
        {
            var origin = new Point3D(0, 0, 0);
            var dir = direction.ToGpDir();
            var axis = new Axis3D(origin, dir);
            return trsf.WithRotation(axis, angle * Constants.DegToRad);
        }), owned: true)
    {
    }

    /// <summary>
    /// A location at a position, oriented by Euler angles in degrees applied in <paramref name="ordering"/>
    /// (default <see cref="Intrinsic.XYZ"/>).
    /// </summary>
    public Location(Vector position, Vector orientation, RotationOrder ordering = default)
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

    static Placement Build(Vector position, Func<Transform3D, Transform3D> rotate)
    {
        var trsf = rotate(new Transform3D());
        var translation = position.ToGpVec();
        trsf = trsf.WithTranslationPart(translation);
        return new Placement(trsf);
    }

    static Placement FromPlane(Plane plane)
    {
        var origin = plane.Origin.ToGpPnt();
        var z = plane.ZDir.ToGpDir();
        var x = plane.XDir.ToGpDir();
        var cs = new CoordinateSystem(origin, z, x);
        var trsf = new Transform3D();
        trsf = trsf.WithTransformation(cs);
        trsf = trsf.Inverted();
        return new Placement(trsf);
    }

    /// <summary>The OCCT location (the same identity).</summary>
    public Placement ToTopLoc() => Wrapped;

    /// <summary>A new OCCT gp_Trsf for this location's transformation.</summary>
    public Transform3D ToGpTrsf() => Wrapped.Transformation;

    /// <summary>The position (translation) part.</summary>
    public Vector Position
    {
        get
        {
            return new Vector(Wrapped.Transformation.TranslationPart);
        }
    }

    /// <summary>The orientation part, as intrinsic XYZ Euler angles in degrees.</summary>
    public Vector Orientation
    {
        get
        {
            var (a, b, c) = Wrapped.Transformation.GetEulerAngles(EulerSequence.Intrinsic_XYZ);
            return new Vector(a * Constants.RadToDeg, b * Constants.RadToDeg, c * Constants.RadToDeg);
        }
    }

    /// <summary>Deconstructs into position and orientation (intrinsic XYZ Euler angles in degrees).</summary>
    public void Deconstruct(out Vector position, out Vector orientation) => (position, orientation) = (Position, Orientation);

    /// <summary>This location with a new position and the same orientation.</summary>
    public Location WithPosition(Vector position)
    {
        var trsfPosition = new Transform3D();
        var v = position.ToGpVec();
        trsfPosition = trsfPosition.WithTranslationPart(v);
        var trsfOrientation = Wrapped.Transformation.ToRotation();
        var combined = trsfPosition.Multiplied(trsfOrientation);
        return Own(new Placement(combined));
    }

    /// <summary>This location with a new orientation (intrinsic XYZ Euler angles in degrees) and the same position.</summary>
    public Location WithOrientation(Vector orientation)
    {
        var trsfPosition = new Transform3D();
        var v = Wrapped.Transformation.TranslationPart;
        trsfPosition = trsfPosition.WithTranslationPart(v);
        var trsfOrientation = Transform3D.Rotation(EulerSequence.Intrinsic_XYZ, orientation.X * Constants.DegToRad,
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
    public Vector Center() => Position;

    /// <summary>
    /// This location mirrored across a plane. A true mirror image is left-handed, so the X and Z
    /// directions are mirrored and Y is recomputed as Z × X to keep the frame right-handed.
    /// </summary>
    public Location Mirror(Plane mirrorPlane)
    {
        Vector MirrorDir(Vector v) => v - 2 * v.Dot(mirrorPlane.ZDir) * mirrorPlane.ZDir;

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
    public static IGeometry? operator &(Location a, Location b) => a.Intersect(b);
    public static IGeometry? operator &(Location a, Axis b) => a.Intersect(b);
    public static IGeometry? operator &(Location a, Plane b) => a.Intersect(b);
    public static Vector? operator &(Location a, Vector b) => a.Intersect(b);

    /// <summary>Where a shape meets this location (the shape's <see cref="Shape.Intersect(object[])"/>).</summary>
    public static ShapeList<Shape> operator &(Location a, Shape b) => a.Intersect(b);

    /// <summary>Where a shape meets this location (the shape's <see cref="Shape.Intersect(object[])"/>), empty if nowhere.</summary>
    public ShapeList<Shape> Intersect(Shape shape) => shape.Intersect(this);
#pragma warning restore CS1591

    /// <summary>The point if it is this location's position, else null.</summary>
    public Vector? Intersect(Vector point) => Position == point ? point : null;

    /// <summary>This location if the two are equal, the shared position if only that matches, else null.</summary>
    public IGeometry? Intersect(Location location) =>
        this == location ? this : Position == location.Position ? location.Position : null;

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

    /// <summary>Python's repr: "Location((1, 2, 3), (10, 20, 30))", components to six significant figures.</summary>
    public string Repr() => $"{GetType().Name}({Position.ToString(".6g")}, {Orientation.ToString(".6g")})";

    /// <summary>"Location: (position=(1, 2, 3), orientation=(10, 20, 30))".</summary>
    public override string ToString() =>
        $"Location: (position={Position.ToString(".6g")}, orientation={Orientation.ToString(".6g")})";

    /// <summary>"(position, orientation)" with components formatted as <see cref="Vector.ToString(string?, IFormatProvider?)"/>.</summary>
    public string ToString(string? format, IFormatProvider? formatProvider = null) =>
        $"({Position.ToString(format)}, {Orientation.ToString(format)})";
}

/// <summary>A <see cref="Location"/> that only rotates, about the origin.</summary>
public class Rotation : Location
{
    /// <summary>A rotation by Euler angles in degrees, applied in <paramref name="ordering"/> (default <see cref="Intrinsic.XYZ"/>).</summary>
    public Rotation(double x = 0, double y = 0, double z = 0, RotationOrder ordering = default)
        : base(new Vector(0, 0, 0), new Vector(x, y, z), ordering)
    {
    }

    /// <summary>A rotation by Euler angles (X, Y, Z) in degrees, applied in <paramref name="ordering"/>.</summary>
    public Rotation(Vector angles, RotationOrder ordering = default) : base(new Vector(0, 0, 0), angles, ordering) { }

    /// <summary>A rotation about an axis by an angle in degrees.</summary>
    public Rotation(Axis axis, double angle) : base(AboutAxis(axis, angle), owned: true) { }

    /// <summary>A copy of another rotation.</summary>
    public Rotation(Rotation rotation) : base(rotation) { }

    /// <summary>A rotation by Euler angles (X, Y, Z) in degrees, as build123d's <c>RotationLike</c> tuples are.</summary>
    public static implicit operator Rotation((double X, double Y, double Z) angles) => new(angles.X, angles.Y, angles.Z);

    static Placement AboutAxis(Axis axis, double angle)
    {
        var trsf = new Transform3D();
        if (angle != 0.0)
        {
            var ax1 = axis.ToGpAx1();
            trsf = trsf.WithRotation(ax1, angle * Constants.DegToRad);
        }
        return new Placement(trsf);
    }
}

/// <summary>A <see cref="Location"/> that only translates.</summary>
public class Pos : Location
{
    /// <summary>A translation by (x, y, z).</summary>
    public Pos(double x = 0, double y = 0, double z = 0) : base(new Vector(x, y, z)) { }

    /// <summary>A translation by a vector.</summary>
    public Pos(Vector v) : base(v) { }
}
