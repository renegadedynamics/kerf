using Build123d.Occt;

namespace Build123d;

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
    internal TopLoc_Location Wrapped { get; }

    /// <summary>The identity location: no translation or rotation.</summary>
    public Location() : this(new TopLoc_Location(), owned: true) { }

    /// <summary>A copy of another location, sharing its OCCT location.</summary>
    public Location(Location location) : this(location.Wrapped, owned: true) { }

    /// <summary>A location from an OCCT TopLoc_Location (copied, keeping its identity).</summary>
    public Location(TopLoc_Location location) : this(new TopLoc_Location(location), owned: true) { }

    /// <summary>A location from an OCCT transformation.</summary>
    public Location(gp_Trsf trsf) : this(new TopLoc_Location(trsf), owned: true) { }

    /// <summary>A location at a position, without rotation.</summary>
    public Location(Vector position) : this(Build(position, trsf => { }), owned: true) { }

    /// <summary>A location at a position, rotated about the Z axis by an angle in degrees.</summary>
    public Location(Vector position, double angle) : this(position, new Vector(0, 0, 1), angle) { }

    /// <summary>A location at a position, rotated about <paramref name="direction"/> by an angle in degrees.</summary>
    public Location(Vector position, Vector direction, double angle)
        : this(Build(position, trsf =>
        {
            using var origin = new gp_Pnt(0, 0, 0);
            using var dir = direction.ToGpDir();
            using var axis = new gp_Ax1(origin, dir);
            trsf.SetRotation(axis, angle * Constants.DegToRad);
        }), owned: true)
    {
    }

    /// <summary>
    /// A location at a position, oriented by Euler angles in degrees applied in <paramref name="ordering"/>
    /// (default <see cref="Intrinsic.XYZ"/>).
    /// </summary>
    public Location(Vector position, Vector orientation, RotationOrder ordering = default)
        : this(Build(position, trsf =>
        {
            using var q = new gp_Quaternion();
            q.SetEulerAngles(ordering.ToOcct(), orientation.X * Constants.DegToRad,
                             orientation.Y * Constants.DegToRad, orientation.Z * Constants.DegToRad);
            trsf.SetRotation(q);
        }), owned: true)
    {
    }

    /// <summary>The location of a plane: at its origin, with its axes.</summary>
    public Location(Plane plane) : this(FromPlane(plane), owned: true) { }

    /// <summary>Takes ownership of a TopLoc_Location without copying it.</summary>
    private protected Location(TopLoc_Location location, bool owned) => Wrapped = location;

    /// <summary>Wraps a new TopLoc_Location this location takes ownership of.</summary>
    static Location Own(TopLoc_Location location) => new(location, owned: true);

    static TopLoc_Location Build(Vector position, Action<gp_Trsf> rotate)
    {
        using var trsf = new gp_Trsf();
        rotate(trsf);
        using var translation = position.ToGpVec();
        trsf.SetTranslationPart(translation);
        return new TopLoc_Location(trsf);
    }

    static TopLoc_Location FromPlane(Plane plane)
    {
        using var origin = plane.Origin.ToGpPnt();
        using var z = plane.ZDir.ToGpDir();
        using var x = plane.XDir.ToGpDir();
        using var cs = new gp_Ax3(origin, z, x);
        using var trsf = new gp_Trsf();
        trsf.SetTransformation(cs);
        trsf.Invert();
        return new TopLoc_Location(trsf);
    }

    /// <summary>The OCCT location, as a new TopLoc_Location sharing this one's identity.</summary>
    public TopLoc_Location ToTopLoc() => new(Wrapped);

    /// <summary>A new OCCT gp_Trsf for this location's transformation.</summary>
    public gp_Trsf ToGpTrsf() => Wrapped.Transformation();

    /// <summary>The position (translation) part.</summary>
    public Vector Position
    {
        get
        {
            using var trsf = Wrapped.Transformation();
            using var t = trsf.TranslationPart();
            return new Vector(t);
        }
    }

    /// <summary>The orientation part, as intrinsic XYZ Euler angles in degrees.</summary>
    public Vector Orientation
    {
        get
        {
            using var trsf = Wrapped.Transformation();
            using var q = trsf.GetRotation();
            double a = 0, b = 0, c = 0;
            q.GetEulerAngles(gp_EulerSequence.gp_Intrinsic_XYZ, ref a, ref b, ref c);
            return new Vector(a * Constants.RadToDeg, b * Constants.RadToDeg, c * Constants.RadToDeg);
        }
    }

    /// <summary>Deconstructs into position and orientation (intrinsic XYZ Euler angles in degrees).</summary>
    public void Deconstruct(out Vector position, out Vector orientation) => (position, orientation) = (Position, Orientation);

    /// <summary>This location with a new position and the same orientation.</summary>
    public Location WithPosition(Vector position)
    {
        using var trsfPosition = new gp_Trsf();
        using var v = position.ToGpVec();
        trsfPosition.SetTranslationPart(v);
        using var current = Wrapped.Transformation();
        using var rotation = current.GetRotation();
        using var trsfOrientation = new gp_Trsf();
        trsfOrientation.SetRotation(rotation);
        using var combined = trsfPosition.Multiplied(trsfOrientation);
        return Own(new TopLoc_Location(combined));
    }

    /// <summary>This location with a new orientation (intrinsic XYZ Euler angles in degrees) and the same position.</summary>
    public Location WithOrientation(Vector orientation)
    {
        using var current = Wrapped.Transformation();
        using var xyz = current.TranslationPart();
        using var trsfPosition = new gp_Trsf();
        using var v = new gp_Vec(xyz.X(), xyz.Y(), xyz.Z());
        trsfPosition.SetTranslationPart(v);
        using var q = new gp_Quaternion();
        q.SetEulerAngles(gp_EulerSequence.gp_Intrinsic_XYZ, orientation.X * Constants.DegToRad,
                         orientation.Y * Constants.DegToRad, orientation.Z * Constants.DegToRad);
        using var trsfOrientation = new gp_Trsf();
        trsfOrientation.SetRotation(q);
        using var combined = trsfPosition.Multiplied(trsfOrientation);
        return Own(new TopLoc_Location(combined));
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
            using var trsf = Wrapped.Transformation();
            using var q = trsf.GetRotation();
            return (Position.Key, GeomKey.Quaternion(q));
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

    static TopLoc_Location AboutAxis(Axis axis, double angle)
    {
        using var trsf = new gp_Trsf();
        if (angle != 0.0)
        {
            using var ax1 = axis.ToGpAx1();
            trsf.SetRotation(ax1, angle * Constants.DegToRad);
        }
        return new TopLoc_Location(trsf);
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
