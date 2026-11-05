using Kerf.Geometry;
using System.Diagnostics;
using Kerf.Measurement;

namespace Kerf.Modeling;

/// <summary>
/// An immutable plane: a right-handed coordinate system with an origin and X, Y and Z directions, Z
/// normal to the plane. Local 2D coordinates on a plane map to global 3D ones.
/// </summary>
/// <remarks>
/// <list type="table">
/// <listheader><term>Plane</term><description>x_dir, y_dir, z_dir</description></listheader>
/// <item><term>XY, Top</term><description>+x, +y, +z</description></item>
/// <item><term>YZ, Right</term><description>+y, +z, +x</description></item>
/// <item><term>ZX</term><description>+z, +x, +y</description></item>
/// <item><term>XZ, Front</term><description>+x, +z, -y</description></item>
/// <item><term>YX</term><description>+y, +x, -z</description></item>
/// <item><term>ZY</term><description>+z, +y, -x</description></item>
/// <item><term>Back</term><description>-x, +z, +y</description></item>
/// <item><term>Left</term><description>-y, +z, -x</description></item>
/// <item><term>Bottom</term><description>+x, -y, -z</description></item>
/// <item><term>Isometric</term><description>+x+y, -x+y+z, +x+y-z</description></item>
/// </list>
/// Equality compares origin, X and Z directions rounded to five decimal places.
/// </remarks>
public sealed class Plane : IEquatable<Plane>, IFormattable, IGeometry
{
    /// <summary>The OCCT plane. Never modified.</summary>
    internal Plane3D Pln { get; }

    /// <summary>The global position of the local origin.</summary>
    public Point3D Origin { get; }

    /// <summary>The local X direction.</summary>
    public Direction3D XDir { get; }

    /// <summary>The local Y direction.</summary>
    public Direction3D YDir { get; }

    /// <summary>The local Z direction, normal to the plane.</summary>
    public Direction3D ZDir { get; }

#pragma warning disable CS1591 // The standard planes are described in the remarks
    public static Plane XY { get; } = new(new Point3D(0, 0, 0), new Vector3D(1, 0, 0), new Vector3D(0, 0, 1));
    public static Plane YZ { get; } = new(new Point3D(0, 0, 0), new Vector3D(0, 1, 0), new Vector3D(1, 0, 0));
    public static Plane ZX { get; } = new(new Point3D(0, 0, 0), new Vector3D(0, 0, 1), new Vector3D(0, 1, 0));
    public static Plane XZ { get; } = new(new Point3D(0, 0, 0), new Vector3D(1, 0, 0), new Vector3D(0, -1, 0));
    public static Plane YX { get; } = new(new Point3D(0, 0, 0), new Vector3D(0, 1, 0), new Vector3D(0, 0, -1));
    public static Plane ZY { get; } = new(new Point3D(0, 0, 0), new Vector3D(0, 0, 1), new Vector3D(-1, 0, 0));
    public static Plane Front { get; } = new(new Point3D(0, 0, 0), new Vector3D(1, 0, 0), new Vector3D(0, -1, 0));
    public static Plane Back { get; } = new(new Point3D(0, 0, 0), new Vector3D(-1, 0, 0), new Vector3D(0, 1, 0));
    public static Plane Left { get; } = new(new Point3D(0, 0, 0), new Vector3D(0, -1, 0), new Vector3D(-1, 0, 0));
    public static Plane Right { get; } = new(new Point3D(0, 0, 0), new Vector3D(0, 1, 0), new Vector3D(1, 0, 0));
    public static Plane Top { get; } = new(new Point3D(0, 0, 0), new Vector3D(1, 0, 0), new Vector3D(0, 0, 1));
    public static Plane Bottom { get; } = new(new Point3D(0, 0, 0), new Vector3D(1, 0, 0), new Vector3D(0, 0, -1));
    public static Plane Isometric { get; } = new(
        new Point3D(0, 0, 0), new Vector3D(1 / Math.Sqrt(2), 1 / Math.Sqrt(2), 0), new Vector3D(1 / Math.Sqrt(3), -1 / Math.Sqrt(3), 1 / Math.Sqrt(3)));
#pragma warning restore CS1591

    /// <summary>Wraps a gp_Pln this plane takes ownership of, making it right-handed if it isn't.</summary>
    Plane(Plane3D pln, bool owned)
    {
        Pln = EnsureRightHanded(pln);
        var loc = Pln.Location;
        var axis = Pln.Axis;
        var z = axis.Direction;
        var xAxis = Pln.XAxis;
        var x = xAxis.Direction;
        var yAxis = Pln.YAxis;
        var y = yAxis.Direction;
        (Origin, XDir, YDir, ZDir) = (loc, x, y, z);
    }

    /// <summary>A plane from an OCCT gp_Pln.</summary>
    public Plane(Plane3D pln) : this(pln, owned: true) { }

    /// <summary>
    /// A plane at <paramref name="origin"/> with normal <paramref name="zDir"/> (default +Z) and X direction
    /// <paramref name="xDir"/>. Without an X direction, OCCT chooses one perpendicular to the normal.
    /// </summary>
    /// <exception cref="ArgumentException">A direction has zero length.</exception>
    /// <exception cref="Exception">The X direction is parallel to the normal (the kernel's construction error).</exception>
    public Plane(Point3D origin, Vector3D? xDir = null, Vector3D? zDir = null)
        : this(Build(origin, xDir is { } x ? NormalizedArg(x) : null, zDir is { } z ? NormalizedArg(z) : new Vector3D(0, 0, 1)), owned: true)
    {
    }

    /// <summary>The plane of a location: at its position, with its axes.</summary>
    public Plane(Location location) : this(FromLocation(location), owned: true) { }

    /// <summary>
    /// The plane of a planar face: at its centre of mass, normal to it, with X along the surface's own X direction (or
    /// <paramref name="xDir"/>).
    /// </summary>
    /// <exception cref="ArgumentException">The face isn't planar.</exception>
    public Plane(Face face, Vector3D? xDir = null) : this(FromFace(face, xDir), owned: true) { }

    static Plane3D FromFace(Face face, Vector3D? xDir)
    {
        if (face.IsPlanar is null)
            throw new ArgumentException("Planes can only be created from planar faces");
        var topoFace = face;
        topoFace.TryGetSurface(out var surface);
        var origin = topoFace.MassProperties(Dimension.Area).CentreOfMass;
        Vector3D faceXDir;
        if (surface is BoundedSurface bounded)
        {
            faceXDir = (bounded.Derivatives(0.5, 0.5, 1).DU);
        }
        else
        {
            var elementary = (surface as ElementarySurface)!;
            var position = elementary.Position;
            var x = position.XDirection;
            faceXDir = x;
        }
        var x14 = (xDir ?? faceXDir).Round(14);
        var z14 = FaceNormal(topoFace, surface).Round(14);
        return Build(origin, x14, z14);
    }

    /// <summary>The normal of a face where it is nearest the global origin (build123d's <c>get_topods_face_normal</c>).</summary>
    static Vector3D FaceNormal(Face face, Surface? surface)
    {
        if (surface is null || !surface.TryProjectNearest(Point3D.Origin, out var nearest))
            throw new InvalidOperationException("The face's surface has no point nearest the origin");
        return TopoUtil.FaceNormalVector(face, nearest.U, nearest.V);
    }

    /// <summary>A plane through the axis position, normal to the axis, with an optional X direction.</summary>
    public Plane(Axis axis, Vector3D? xDir = null) : this(Build(axis.Position, xDir, axis.Direction), owned: true) { }

    /// <summary>
    /// A plane at <paramref name="origin"/> from its X and Y directions. The normal is X × Y, and Y is
    /// adjusted to be perpendicular to X.
    /// </summary>
    /// <exception cref="ArgumentException">A direction has zero length, or the two are parallel.</exception>
    public static Plane FromXAndYDirections(Point3D origin, Vector3D xDir, Vector3D yDir)
    {
        if (xDir.Magnitude == 0.0)
            throw new ArgumentException("The X direction can't be null.", nameof(xDir));
        if (yDir.Magnitude == 0.0)
            throw new ArgumentException("The Y direction can't be null.", nameof(yDir));
        var x = xDir.Normalized();
        var yInput = yDir.Normalized();
        var zFromXy = x.Cross(yInput);
        if (zFromXy.Magnitude == 0.0)
            throw new ArgumentException("The X and Y directions can't be parallel.", nameof(yDir));
        var z = zFromXy.Normalized();
        var y = z.Cross(x).Normalized();
        x = y.Cross(z).Normalized();
        return new Plane(Ax3Plane(origin, z, x), owned: true);
    }

    /// <summary>The plane through three points: origin at the first, X towards the second, the third on the plane.</summary>
    public static Plane ThroughPoints(Point3D p0, Point3D p1, Point3D p2)
    {
        var xDir = p1 - p0;
        return Create(p0, xDir, xDir.Cross(p2 - p0));
    }

    /// <summary>
    /// A plane from directions used as given: build123d's keyword-argument construction. The public
    /// constructor normalises its direction arguments once more first, as build123d's positional one does.
    /// </summary>
    internal static Plane Create(Point3D origin, Vector3D? xDir, Vector3D zDir) => new(Build(origin, xDir, zDir), owned: true);

    static Vector3D NormalizedArg(Vector3D v)
    {
        try
        {
            return v.Normalized();
        }
        catch (InvalidOperationException ex)
        {
            throw new ArgumentException("A plane's directions can't have zero length", ex);
        }
    }

    static Plane3D Build(Point3D origin, Vector3D? xDir, Vector3D zDir)
    {
        if (zDir.Magnitude == 0.0)
            throw new ArgumentException("The Z direction can't be null.", nameof(zDir));
        var z = zDir.Normalized();
        Vector3D x;
        if (xDir is null)
        {
            // OCCT picks an X direction perpendicular to the normal
            var p = origin;
            var n = z.ToDirection();
            var ax3 = new CoordinateSystem(p, n);
            var xd = ax3.XDirection;
            x = xd.ToVector().Normalized();
        }
        else
        {
            if (xDir.Value.Magnitude == 0.0)
                throw new ArgumentException("The X direction can't be null.", nameof(xDir));
            x = xDir.Value.Normalized();
        }
        return Ax3Plane(origin, z, x);
    }

    static Plane3D Ax3Plane(Point3D origin, Vector3D z, Vector3D x)
    {
        var p = origin;
        var n = z.ToDirection();
        var vx = x.ToDirection();
        var ax3 = new CoordinateSystem(p, n, vx);
        return new Plane3D(ax3);
    }

    static Plane3D FromLocation(Location location)
    {
        // build123d moves a planar XY face to the location and reads back its surface's X direction and
        // its normal. Both follow from transforming the XY frame's directions as gp_Ax3::Transform does:
        // X and Y each transformed as a gp_Dir, and the normal their cross product.
        var trsf = location.Wrapped.Transformation;
        var dx = new Direction3D(1, 0, 0);
        var dy = new Direction3D(0, 1, 0);
        var tx = dx.Transformed(trsf);
        var ty = dy.Transformed(trsf);
        Vector3D x = tx;
        var normal = x.Cross(ty);
        return Build(location.Position, x.Round(14), normal.Round(14));
    }

    static Plane3D EnsureRightHanded(Plane3D pln)
    {
        var position = pln.Position;
        if (position.IsDirect)
            return pln;

        Trace.TraceWarning("Trying to set a left-handed plane");
        var loc = pln.Location;
        var axis = pln.Axis;
        var n = axis.Direction;
        var xAxis = pln.XAxis;
        var vx = xAxis.Direction;
        var ax2 = new Frame3D(loc, n, vx);
        var ax3 = new CoordinateSystem(ax2);
        return new Plane3D(ax3);
    }

    /// <summary>The OCCT gp_Pln of this plane.</summary>
    public Plane3D ToPlane3D() => Pln;

    /// <summary>A new OCCT gp_Ax3 for this plane's coordinate system.</summary>
    public CoordinateSystem ToCoordinateSystem() => Pln.Position;

    /// <summary>A new OCCT gp_Ax2 for this plane's coordinate system.</summary>
    public Frame3D ToFrame3D()
    {
        var ax3 = Pln.Position;
        return ax3.Frame;
    }

    /// <summary>The plane moved by <paramref name="amount"/> along its Z direction.</summary>
    public Plane Offset(double amount) => Create(Origin + ZDir * amount, XDir, ZDir);

    /// <summary>The plane with its Z direction reversed (X unchanged, so Y reverses too).</summary>
    public Plane Reverse() => -this;

    /// <summary>This plane with a new origin and the same directions.</summary>
    public Plane WithOrigin(Point3D origin) => Create(origin, XDir, ZDir);

    /// <summary>This plane with a new X direction, projected to be perpendicular to the normal.</summary>
    public Plane WithXDir(Vector3D xDir)
    {
        var ax2 = ToFrame3D();
        var d = xDir.ToDirection();
        ax2 = ax2.WithXDirection(d);
        var ax3 = new CoordinateSystem(ax2);
        return new Plane(new Plane3D(ax3), owned: true);
    }

    /// <summary>This plane with a new Y direction, projected to be perpendicular to the normal.</summary>
    public Plane WithYDir(Vector3D yDir)
    {
        var ax2 = ToFrame3D();
        var d = yDir.ToDirection();
        ax2 = ax2.WithYDirection(d);
        var ax3 = new CoordinateSystem(ax2);
        return new Plane(new Plane3D(ax3), owned: true);
    }

    /// <summary>
    /// This plane with its origin moved within it to a global point; the directions are unchanged.
    /// </summary>
    /// <exception cref="ArgumentException">The point isn't in the plane.</exception>
    public Plane ShiftOrigin(Point3D point)
    {
        if (!Contains(point))
            throw new ArgumentException($"{point.Describe()} is not located within plane", nameof(point));
        return Create(point, XDir, ZDir);
    }

    /// <summary>This plane with its origin moved to a vertex in it.</summary>
    /// <exception cref="ArgumentException">The vertex isn't in the plane.</exception>
    public Plane ShiftOrigin(Vertex vertex)
    {
        var point = vertex.Point;
        if (!Contains(point))
            throw new ArgumentException($"{vertex} is not located within plane", nameof(vertex));
        return Create(point, XDir, ZDir);
    }

    /// <summary>This plane with its origin moved by local (x, y) coordinates; the directions are unchanged.</summary>
    public Plane ShiftOriginBy(double x, double y) => Create(FromLocalCoords(new Point3D(x, y, 0)), XDir, ZDir);

    /// <summary>This plane with its origin moved to where an axis crosses it; the directions are unchanged.</summary>
    /// <exception cref="ArgumentException">The axis doesn't cross the plane at a single point.</exception>
    public Plane ShiftOrigin(Axis axis)
    {
        if (Intersect(axis) is not Point3D point)
            throw new ArgumentException($"{axis} doesn't intersect the plane", nameof(axis));
        return Create(point, XDir, ZDir);
    }

    /// <summary>
    /// This plane rotated about its own X, Y and Z directions (not the world's) by angles in degrees,
    /// in <paramref name="ordering"/> (default <see cref="Intrinsic.XYZ"/>). The origin doesn't move.
    /// </summary>
    public Plane Rotated(Vector3D rotation, RotationOrder ordering = default) =>
        new(Location * new Rotation(rotation, ordering));

    /// <summary>This plane moved by a relative location.</summary>
    public Plane Moved(Location location) => new(Location * location);

    /// <summary>This plane moved by the location of another plane.</summary>
    public Plane Moved(Plane plane) => Moved(plane.Location);

    /// <summary>This plane as a location: at its origin, with its axes.</summary>
    public Location Location => new(this);

    /// <summary>The matrix taking global coordinates to this plane's local ones.</summary>
    public GeneralTransform3D ForwardTransform
    {
        get
        {
            var global = new CoordinateSystem();
            var local = Pln.Position;
            var t = new Transform3D();
            t = t.WithTransformation(global, local);
            return new GeneralTransform3D(t);
        }
    }

    /// <summary>The matrix taking this plane's local coordinates to global ones.</summary>
    public GeneralTransform3D ReverseTransform
    {
        get
        {
            var global = new CoordinateSystem();
            var local = Pln.Position;
            var t = new Transform3D();
            t = t.WithTransformation(local, global);
            return new GeneralTransform3D(t);
        }
    }

    /// <summary>A global point in this plane's local coordinates.</summary>
    public Point3D ToLocalCoords(Point3D point) => point.Transform(ForwardTransform);

    /// <summary>A point in this plane's local coordinates in global ones.</summary>
    public Point3D FromLocalCoords(Point3D point) => point.Transform(ReverseTransform);

    /// <summary>A bounding box's corners in this plane's local coordinates, each as it lands.</summary>
    public BoxCorners ToLocalCoords(BoundingBox box) => TransformBox(box, ForwardTransform);

    /// <summary>A bounding box's corners, in this plane's local coordinates, in global ones, each as it lands.</summary>
    public BoxCorners FromLocalCoords(BoundingBox box) => TransformBox(box, ReverseTransform);

    /// <summary>A copy of the shape repositioned relative to this plane: its global coordinates become local ones.</summary>
    public T ToLocalCoords<T>(T shape) where T : IShape => TopoUtil.Rewrap(shape, ShapeTransforms.TransformedBy(shape.AsShape(), ForwardTransform));

    /// <summary>A copy of the shape repositioned from this plane's local coordinates to global ones.</summary>
    public T FromLocalCoords<T>(T shape) where T : IShape => TopoUtil.Rewrap(shape, ShapeTransforms.TransformedBy(shape.AsShape(), ReverseTransform));

    static BoxCorners TransformBox(BoundingBox box, GeneralTransform3D transform)
    {
        // The two corners are transformed as they are; the box isn't re-fitted around the result
        var (min, max) = box.IsEmpty ? (Point3D.Origin, Point3D.Origin) : (box.Min, box.Max);
        return new BoxCorners(min.Transform(transform), max.Transform(transform));
    }

    /// <summary>The location taking this plane to another.</summary>
    public Location LocationBetween(Plane other)
    {
        var t = new Transform3D();
        var from = Pln.Position;
        var to = other.Pln.Position;
        t = t.WithTransformation(from, to);
        return new Location(t);
    }

    /// <summary>True if the point lies in the plane, within the tolerance.</summary>
    public bool Contains(Point3D point, double tolerance = Constants.Tolerance)
    {
        var p = point;
        return Pln.Contains(p, tolerance);
    }

    /// <summary>True if the whole axis lies in the plane, within the tolerance (linear and angular).</summary>
    public bool Contains(Axis axis, double tolerance = Constants.Tolerance)
    {
        var p = axis.Position;
        var d = axis.Direction.ToDirection();
        var line = new Line3D(p, d);
        return Pln.Contains(line, tolerance, tolerance);
    }

#pragma warning disable CS1591 // Operators are self-describing
    /// <summary>The plane with its Z direction reversed.</summary>
    public static Plane operator -(Plane p) => new(p.Origin, p.XDir, -p.ZDir);

    /// <summary>The plane's location composed with another location.</summary>
    public static Location operator *(Plane p, Location l) => new Location(p) * l;

    /// <summary>The plane's location composed with another plane's.</summary>
    public static Location operator *(Plane p, Plane q) => new Location(p) * q.Location;

    /// <summary>The plane moved by a location.</summary>
    public static Plane operator *(Location l, Plane p) => p.Moved(l);

    /// <summary>The plane's location composed with each location in turn.</summary>
    public static IReadOnlyList<Location> operator *(Plane p, IEnumerable<Location> locations) =>
        locations.Select(l => p * l).ToArray();

    /// <summary>The plane's location composed with each plane's location in turn.</summary>
    public static IReadOnlyList<Location> operator *(Plane p, IEnumerable<Plane> planes) =>
        planes.Select(q => p * q).ToArray();

    /// <summary>The plane moved by each location in turn.</summary>
    public static IReadOnlyList<Plane> operator *(IEnumerable<Location> locations, Plane p) =>
        locations.Select(l => p.Moved(l)).ToArray();

    /// <summary>The plane moved by each plane's location in turn.</summary>
    public static IReadOnlyList<Plane> operator *(IEnumerable<Plane> planes, Plane p) =>
        planes.Select(q => p.Moved(q)).ToArray();

    public static bool operator ==(Plane? a, Plane? b) => a is null ? b is null : a.Equals(b);
    public static bool operator !=(Plane? a, Plane? b) => !(a == b);


    /// <summary>Where a shape meets this plane (the shape's <c>Intersect</c>), empty if nowhere.</summary>
    public ShapeList<Shape> Intersect(Shape shape) => shape.Intersect(this);
#pragma warning restore CS1591

    /// <summary>The intersection with an axis: the axis if it lies in the plane, the crossing point, or null.</summary>
    public IGeometry? Intersect(Axis axis)
    {
        if (Contains(axis))
            return axis;
        var ax1 = axis.ToAxis3D();
        var line = new LineCurve3D(ax1);
        var ax3 = Pln.Position;
        var plane = new PlaneSurface(ax3);
        if (line.Intersect(plane) is { IsSuccess: true, Value: [var hit] })
            return hit.Point;
        return null;
    }

    /// <summary>The intersection with another plane: this plane if they coincide, the line where they cross, or null.</summary>
    public IGeometry? Intersect(Plane plane)
    {
        if (Contains(plane.Origin) && ZDir.IsNear(plane.ZDir))
            return this;
        var s1 = new PlaneSurface(Pln);
        var s2 = new PlaneSurface(plane.Pln);
        if (s1.Intersect(s2, Constants.Tolerance) is { IsSuccess: true, Value: [var curve, ..] })
        {
            var line = (curve as LineCurve3D) ?? throw new InvalidOperationException("plane intersection isn't a line");
            var position = line.Position;
            return new Axis(position);
        }
        return null;
    }

    /// <summary>The point if it lies in the plane, else null.</summary>
    public Point3D? Intersect(Point3D point) => Contains(point) ? point : null;

    /// <summary>
    /// The intersection with a location: the location if it lies in the plane with the same normal, its
    /// position if it lies in the plane otherwise, or null.
    /// </summary>
    public IGeometry? Intersect(Location location)
    {
        var locationPlane = new Plane(location);
        if (!Contains(locationPlane.Origin))
            return null;
        return ZDir.IsNear(locationPlane.ZDir) ? location : locationPlane.Origin;
    }

    internal ((double, double, double), (double, double, double), (double, double, double)) Key =>
        (Origin.Key, XDir.Key, ZDir.Key);

    /// <inheritdoc/>
    public bool Equals(Plane? other) => other is not null && Key == other.Key;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is Plane p && Equals(p);

    /// <inheritdoc/>
    public override int GetHashCode() => Key.GetHashCode();

    /// <summary>"Plane: (origin=(0, 0, 0), x_dir=(1, 0, 0), z_dir=(0, 0, 1))".</summary>
    public override string ToString() =>
        $"Plane: (origin={Origin.Describe("G6")}, x_dir={XDir.Describe("G6")}, z_dir={ZDir.Describe("G6")})";

    /// <summary>"(origin, x_dir, z_dir)" with components formatted as <c>Describe(format)</c> does.</summary>
    public string ToString(string? format, IFormatProvider? formatProvider = null) =>
        $"({Origin.Describe(format)}, {XDir.Describe(format)}, {ZDir.Describe(format)})";
}
