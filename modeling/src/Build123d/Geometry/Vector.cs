using Occt;

namespace Build123d;

/// <summary>
/// An immutable 3D vector, used for points and directions alike. A missing Z is zero.
/// Converts implicitly from (x, y) and (x, y, z) tuples.
/// </summary>
/// <remarks>
/// Equality compares components rounded to five decimal places, and the hash agrees with it.
/// </remarks>
public readonly struct Vector : IEquatable<Vector>, IFormattable, IGeometry
{
    // gp::Resolution(): the smallest normal double, below which OCCT treats a magnitude as zero
    internal const double GpResolution = 2.2250738585072014E-308;

    /// <summary>The X component.</summary>
    public double X { get; }

    /// <summary>The Y component.</summary>
    public double Y { get; }

    /// <summary>The Z component.</summary>
    public double Z { get; }

    /// <summary>A vector from its components.</summary>
    public Vector(double x, double y, double z = 0) => (X, Y, Z) = (x, y, z);

    /// <summary>A vector from up to three components; missing ones are zero.</summary>
    public Vector(ReadOnlySpan<double> values)
        : this(values.Length > 0 ? values[0] : 0, values.Length > 1 ? values[1] : 0, values.Length > 2 ? values[2] : 0)
    {
    }

    /// <summary>A vector from an OCCT gp_XYZ.</summary>
    public Vector(gp_XYZ xyz) : this(xyz.X(), xyz.Y(), xyz.Z()) { }

    /// <summary>A vector from an OCCT gp_Pnt.</summary>
    public Vector(gp_Pnt pnt) : this(pnt.X(), pnt.Y(), pnt.Z()) { }

    /// <summary>A vector from an OCCT gp_Vec.</summary>
    public Vector(gp_Vec vec) : this(vec.X(), vec.Y(), vec.Z()) { }

    /// <summary>A vector from an OCCT gp_Dir.</summary>
    public Vector(gp_Dir dir) : this(dir.X(), dir.Y(), dir.Z()) { }

    /// <summary>The point a vertex is at.</summary>
    public Vector(Vertex vertex) => this = vertex.ToVector();

    /// <summary>The vector (x, y, 0).</summary>
    public static implicit operator Vector((double X, double Y) v) => new(v.X, v.Y);

    /// <summary>The vector (x, y, z).</summary>
    public static implicit operator Vector((double X, double Y, double Z) v) => new(v.X, v.Y, v.Z);

    /// <summary>Deconstructs into components.</summary>
    public void Deconstruct(out double x, out double y, out double z) => (x, y, z) = (X, Y, Z);

    /// <summary>A component by index: 0, 1 or 2.</summary>
    public double this[int index] => index switch
    {
        0 => X,
        1 => Y,
        2 => Z,
        _ => throw new ArgumentOutOfRangeException(nameof(index)),
    };

    /// <summary>The length (magnitude).</summary>
    public double Length => Math.Sqrt(X * X + Y * Y + Z * Z);

    /// <summary>The cross product.</summary>
    public Vector Cross(Vector v) => new(Y * v.Z - Z * v.Y, Z * v.X - X * v.Z, X * v.Y - Y * v.X);

    /// <summary>The dot product.</summary>
    public double Dot(Vector v) => X * v.X + Y * v.Y + Z * v.Z;

#pragma warning disable CS1591 // Operators are self-describing
    public static Vector operator +(Vector a, Vector b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
    public static Vector operator -(Vector a, Vector b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
    public static Vector operator -(Vector a) => a * -1.0;
    public static Vector operator *(Vector a, double scale) => new(a.X * scale, a.Y * scale, a.Z * scale);
    public static Vector operator *(double scale, Vector a) => a * scale;
    public static Vector operator /(Vector a, double denominator) => a * (1.0 / denominator);
    public static bool operator ==(Vector a, Vector b) => a.Equals(b);
    public static bool operator !=(Vector a, Vector b) => !a.Equals(b);

    /// <summary>The intersection with another geometric value, as <see cref="Intersect(Vector)"/>.</summary>
    public static Vector? operator &(Vector a, Vector b) => a.Intersect(b);
    public static Vector? operator &(Vector a, Axis b) => a.Intersect(b);
    public static Vector? operator &(Vector a, Plane b) => a.Intersect(b);
    public static Vector? operator &(Vector a, Location b) => a.Intersect(b);

    /// <summary>Where a shape meets this vector (the shape's <see cref="Shape.Intersect(object[])"/>).</summary>
    public static ShapeList<Shape> operator &(Vector a, Shape b) => a.Intersect(b);

    /// <summary>Where a shape meets this vector (the shape's <see cref="Shape.Intersect(object[])"/>), empty if nowhere.</summary>
    public ShapeList<Shape> Intersect(Shape shape) => shape.Intersect(this);
#pragma warning restore CS1591

    /// <summary>The vector scaled to length 1.</summary>
    /// <exception cref="Standard_Failure">The vector has zero length.</exception>
    public Vector Normalized()
    {
        double d = Length;
        if (d <= GpResolution)   // as gp_Vec::Normalized checks
            throw new Standard_ConstructionError("gp_Vec::Normalized() - vector has zero norm");
        return new Vector(X / d, Y / d, Z / d);
    }

    /// <summary>The same magnitude, pointing the opposite way.</summary>
    public Vector Reverse() => this * -1.0;

    /// <summary>The centre of a point is the point itself; shapes and vectors share this interface.</summary>
    public Vector Center() => this;

    /// <summary>The unsigned angle to another vector, in degrees.</summary>
    public double GetAngle(Vector v)
    {
        var a = ToGpVec();
        var b = v.ToGpVec();
        return a.Angle(b) * Constants.RadToDeg;
    }

    /// <summary>
    /// The signed angle to another vector, in degrees, positive when the turn is anticlockwise seen from
    /// the tip of <paramref name="normal"/> (default -Z).
    /// </summary>
    public double GetSignedAngle(Vector v, Vector? normal = null)
    {
        var a = ToGpVec();
        var b = v.ToGpVec();
        var n = (normal ?? new Vector(0, 0, -1)).ToGpVec();
        return a.AngleWithRef(b, n) * Constants.RadToDeg;
    }

    /// <summary>The projection of this vector onto the line through the origin along <paramref name="line"/>.</summary>
    public Vector ProjectToLine(Vector line)
    {
        double length = line.Length;
        return line * (Dot(line) / (length * length));
    }

    /// <summary>The unsigned distance to a plane.</summary>
    public double DistanceToPlane(Plane plane)
    {
        var p = ToGpPnt();
        return plane.Pln.Distance(p);
    }

    /// <summary>The distance from a plane, positive on the side its Z direction points to.</summary>
    public double SignedDistanceFromPlane(Plane plane) => (this - plane.Origin).Dot(plane.ZDir);

    /// <summary>The projection of this point onto a plane.</summary>
    public Vector ProjectToPlane(Plane plane)
    {
        var normal = plane.ZDir;
        return this - normal * ((this - plane.Origin).Dot(normal) / Math.Pow(normal.Length, 2));
    }

    /// <summary>
    /// This vector transformed by a matrix: rotated, scaled, sheared and translated as it prescribes.
    /// </summary>
    /// <param name="transform">Any affine transformation.</param>
    /// <param name="isDirection">Treat this as a direction: turned but not translated, and returned with unit length.</param>
    public Vector Transform(Matrix transform, bool isDirection = false)
    {
        // gp_GTrsf.Transforms applies the general transform directly; converting to a gp_Trsf first
        // would reject every matrix that isn't a similarity
        var coords = ToGpXyz();
        transform.GTrsf.Transforms(ref coords);
        if (!isDirection)
            return new Vector(coords);

        // A direction has no position: remove the translation by subtracting where the origin lands
        var origin = new gp_XYZ(0, 0, 0);
        transform.GTrsf.Transforms(ref origin);
        return (new Vector(coords) - new Vector(origin)).Normalized();
    }

    /// <summary>This vector rotated about an axis by an angle in degrees.</summary>
    public Vector Rotate(Axis axis, double angle)
    {
        var v = ToGpVec();
        var ax = axis.ToGpAx1();
        var rotated = v.Rotated(ax, angle * Constants.DegToRad);
        return new Vector(rotated);
    }

    /// <summary>This vector with each component rounded to a number of decimal places.</summary>
    public Vector Round(int digits) => new(Math.Round(X, digits), Math.Round(Y, digits), Math.Round(Z, digits));

    /// <summary>The other vector if the two are equal, else null.</summary>
    public Vector? Intersect(Vector other) => this == other ? other : null;

    /// <summary>This point if it lies on the axis, else null.</summary>
    public Vector? Intersect(Axis axis) => axis.Intersect(this);

    /// <summary>This point if it lies in the plane, else null.</summary>
    public Vector? Intersect(Plane plane) => plane.Intersect(this);

    /// <summary>This point if it is the location's position, else null.</summary>
    public Vector? Intersect(Location location) => location.Intersect(this);

    /// <summary>A new OCCT gp_Pnt at this point.</summary>
    public gp_Pnt ToGpPnt() => new(X, Y, Z);

    /// <summary>A new OCCT gp_Dir in this direction.</summary>
    /// <exception cref="Standard_Failure">The vector has zero length.</exception>
    public gp_Dir ToGpDir() => new(X, Y, Z);

    /// <summary>A new OCCT gp_Vec with these components.</summary>
    public gp_Vec ToGpVec() => new(X, Y, Z);

    /// <summary>A new OCCT gp_XYZ with these components.</summary>
    public gp_XYZ ToGpXyz() => new(X, Y, Z);

    internal (double, double, double) Key => (GeomKey.Round(X), GeomKey.Round(Y), GeomKey.Round(Z));

    /// <inheritdoc/>
    public bool Equals(Vector other) => Key == other.Key;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is Vector v && Equals(v);

    /// <inheritdoc/>
    public override int GetHashCode() => Key.GetHashCode();

    /// <summary>"Vector: (X=1, Y=2, Z=3)", with components to six significant figures.</summary>
    public override string ToString()
    {
        string c(double v) => PyFormat.Component(v, ".6g");
        return $"Vector: (X={c(X)}, Y={c(Y)}, Z={c(Z)})";
    }

    /// <summary>Python's repr: "Vector(1, 2.5, 3)", with components to 13 significant figures.</summary>
    public string Repr() => "Vector" + ToString(".13g");

    /// <summary>
    /// "(x, y, z)" with each component formatted by <paramref name="format"/>. Python-style "f" and "g"
    /// specs (".3f", "g") print values within tolerance of zero as zero; others are .NET formats.
    /// </summary>
    public string ToString(string? format, IFormatProvider? formatProvider = null) =>
        $"({PyFormat.Component(X, format)}, {PyFormat.Component(Y, format)}, {PyFormat.Component(Z, format)})";
}
