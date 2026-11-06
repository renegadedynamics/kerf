namespace Kerf.Modeling;

/// <summary>
/// build123d's conveniences on the kernel's <see cref="Point3D"/>, <see cref="Vector3D"/> and <see cref="Direction3D"/>,
/// which took the place of the layer's <c>Vector</c>: a position is a point, an offset a vector, a unit direction that
/// came from the kernel a direction.
/// </summary>
/// <remarks>
/// <c>==</c> on the kernel's values is exact. <c>IsNear</c> is what the layer's <c>Vector</c> called equal: the
/// components rounded to five decimal places. <see cref="PointComparer"/> is the same for sets.
/// </remarks>
public static class PointExtensions
{
    internal static (double, double, double) KeyOf(double x, double y, double z) => (GeomKey.Round(x), GeomKey.Round(y), GeomKey.Round(z));

    static string Text(double x, double y, double z, string? format) =>
        $"({Formatting.Component(x, format)}, {Formatting.Component(y, format)}, {Formatting.Component(z, format)})";

    static string Labelled(string name, double x, double y, double z)
    {
        static string Component(double v) => Formatting.Component(v, "G6");
        return $"{name}: (X={Component(x)}, Y={Component(y)}, Z={Component(z)})";
    }

    extension(Point3D p)
    {
        internal (double, double, double) Key => KeyOf(p.X, p.Y, p.Z);

        /// <summary>The same point to within five decimal places: each component rounded, then compared.</summary>
        public bool IsNear(Point3D other) => p.Key == other.Key;

        /// <summary>The vector from the origin to this point.</summary>
        public Vector3D ToVector() => new(p.X, p.Y, p.Z);

        /// <summary>The centre of a point is the point itself, as for shapes.</summary>
        public Point3D Center() => p;

        /// <summary>The unsigned distance to a plane.</summary>
        public double DistanceToPlane(Plane plane) => plane.Pln.DistanceTo(p);

        /// <summary>The distance from a plane, positive on the side its Z direction points to.</summary>
        public double SignedDistanceFromPlane(Plane plane) => (p - plane.Origin).Dot(plane.ZDir);

        /// <summary>The projection of this point onto a plane.</summary>
        public Point3D ProjectToPlane(Plane plane)
        {
            Vector3D normal = plane.ZDir;
            return p - normal * ((p - plane.Origin).Dot(normal) / Math.Pow(normal.Magnitude, 2));
        }

        /// <summary>This point transformed by a matrix: rotated, scaled, sheared and translated as it prescribes.</summary>
        // gp_GTrsf.Transforms applies the general transform directly; converting to a gp_Trsf first
        // would reject every matrix that isn't a similarity
        public Point3D Transform(GeneralTransform3D transform) => transform * p;

        /// <summary>This point with each component rounded to a number of decimal places.</summary>
        public Point3D Round(int digits) => new(Math.Round(p.X, digits), Math.Round(p.Y, digits), Math.Round(p.Z, digits));

        /// <summary>The other point if the two are near (<c>IsNear</c>), else null.</summary>
        public Point3D? Intersect(Point3D other) => p.IsNear(other) ? other : null;

        /// <summary>This point if it lies on the axis, else null.</summary>
        public Point3D? Intersect(Axis axis) => axis.Intersect(p);

        /// <summary>This point if it lies in the plane, else null.</summary>
        public Point3D? Intersect(Plane plane) => plane.Intersect(p);

        /// <summary>This point if it is the location's position, else null.</summary>
        public Point3D? Intersect(Location location) => location.Intersect(p);

        /// <summary>Where a shape meets this point (the shape's <c>Intersect</c>), empty if nowhere.</summary>
        public ShapeList<Shape> Intersect(Shape shape) => shape.Intersect(p);

        /// <summary>"Point: (X=1, Y=2, Z=3)", with components to six significant figures.</summary>
        public string Describe() => Labelled("Point", p.X, p.Y, p.Z);

        /// <summary>"(x, y, z)" with each component in <paramref name="format"/>, a .NET numeric format ("G6" when null): values within tolerance of zero print as zero.</summary>
        public string Describe(string? format) => Text(p.X, p.Y, p.Z, format);
    }

    extension(Vector3D v)
    {
        internal (double, double, double) Key => KeyOf(v.X, v.Y, v.Z);

        /// <summary>The same vector to within five decimal places: each component rounded, then compared.</summary>
        public bool IsNear(Vector3D other) => v.Key == other.Key;

        /// <summary>The point at this offset from the origin.</summary>
        public Point3D ToPoint() => new(v.X, v.Y, v.Z);

        /// <summary>This vector as a unit direction.</summary>
        /// <exception cref="Exception">The vector has zero length (the kernel's construction error).</exception>
        public Direction3D ToDirection() => new(v.X, v.Y, v.Z);

        /// <summary>The cross product. The kernel's <c>Crossed</c> and <c>^</c> are the same; this one reads like a direction's.</summary>
        public Vector3D Cross(Vector3D other) => v.Crossed(other);

        /// <summary>The unsigned angle to another vector, in degrees.</summary>
        public double GetAngle(Vector3D other) => v.AngleTo(other) * Constants.RadToDeg;

        /// <summary>
        /// The signed angle to another vector, in degrees, positive when the turn is anticlockwise seen from
        /// the tip of <paramref name="normal"/> (default -Z).
        /// </summary>
        public double GetSignedAngle(Vector3D other, Vector3D? normal = null) =>
            v.AngleWithRef(other, normal ?? new Vector3D(0, 0, -1)) * Constants.RadToDeg;

        /// <summary>The projection of this vector onto the line through the origin along <paramref name="line"/>.</summary>
        public Vector3D ProjectToLine(Vector3D line)
        {
            double length = line.Magnitude;
            return line * (v.Dot(line) / (length * length));
        }

        /// <summary>This vector rotated by an angle in degrees about an axis's direction (its position doesn't matter to a vector).</summary>
        public Vector3D Rotate(Axis axis, double angle) => v.Rotated(axis.ToAxis3D(), angle * Constants.DegToRad);

        /// <summary>This vector transformed by a matrix as the point at its tip would be, or, as a direction, turned but not translated and returned with unit length.</summary>
        /// <param name="transform">Any affine transformation.</param>
        /// <param name="isDirection">Treat this as a direction: turned but not translated, and returned with unit length.</param>
        public Vector3D Transform(GeneralTransform3D transform, bool isDirection = false)
        {
            var coords = transform * v.ToPoint();
            if (!isDirection)
                return coords.ToVector();

            // A direction has no position: remove the translation by subtracting where the origin lands
            var origin = transform * Point3D.Origin;
            return (coords.ToVector() - origin.ToVector()).Normalized();
        }

        /// <summary>This vector with each component rounded to a number of decimal places.</summary>
        public Vector3D Round(int digits) => new(Math.Round(v.X, digits), Math.Round(v.Y, digits), Math.Round(v.Z, digits));

        /// <summary>"Vector: (X=1, Y=2, Z=3)", with components to six significant figures.</summary>
        public string Describe() => Labelled("Vector", v.X, v.Y, v.Z);

        /// <summary>"(x, y, z)" with each component in <paramref name="format"/>, a .NET numeric format ("G6" when null): values within tolerance of zero print as zero.</summary>
        public string Describe(string? format) => Text(v.X, v.Y, v.Z, format);
    }

    extension(Direction3D d)
    {
        internal (double, double, double) Key => KeyOf(d.X, d.Y, d.Z);

        /// <summary>The same direction to within five decimal places: each component rounded, then compared.</summary>
        public bool IsNear(Direction3D other) => d.Key == other.Key;


        /// <summary>This direction made again from its components, normalised once more, as build123d's <c>gp_Dir(*v)</c> does.</summary>
        public Direction3D ToDirection() => new(d.X, d.Y, d.Z);

        /// <summary>The dot product with a vector.</summary>
        public double Dot(Vector3D other) => d.ToVector().Dot(other);

        /// <summary>The cross product, as a vector: not normalised, unlike the kernel's <c>Crossed</c>, which refuses parallel directions.</summary>
        public Vector3D Cross(Vector3D other) => d.ToVector().Crossed(other);

        /// <summary>The unsigned angle to a vector, in degrees.</summary>
        public double GetAngle(Vector3D other) => d.ToVector().GetAngle(other);

        /// <summary>The signed angle to a vector, in degrees, positive when the turn is anticlockwise seen from the tip of <paramref name="normal"/> (default -Z).</summary>
        public double GetSignedAngle(Vector3D other, Vector3D? normal = null) => d.ToVector().GetSignedAngle(other, normal);

        /// <summary>"Direction: (X=1, Y=0, Z=0)", with components to six significant figures.</summary>
        public string Describe() => Labelled("Direction", d.X, d.Y, d.Z);

        /// <summary>"(x, y, z)" with each component in <paramref name="format"/>, a .NET numeric format ("G6" when null): values within tolerance of zero print as zero.</summary>
        public string Describe(string? format) => Text(d.X, d.Y, d.Z, format);
    }
}

/// <summary>Points compared as <c>IsNear</c> does, for sets: components rounded to five decimal places.</summary>
public sealed class PointComparer : IEqualityComparer<Point3D>
{
    /// <summary>The comparer.</summary>
    public static PointComparer Near { get; } = new();

    /// <inheritdoc/>
    public bool Equals(Point3D a, Point3D b) => a.IsNear(b);

    /// <inheritdoc/>
    public int GetHashCode(Point3D p) => p.Key.GetHashCode();
}
