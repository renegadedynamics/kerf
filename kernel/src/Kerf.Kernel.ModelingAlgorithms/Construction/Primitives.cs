// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Construction;

/// <summary>
/// The primitive solids (docs/api/construction.md; OCCT's <c>BRepPrimAPI</c>), as static extension members of
/// <see cref="Solid"/>: <c>Solid.Box(80, 60, 10)</c> or <c>Primitives.Box(80, 60, 10)</c>. Sizes must be positive
/// (misuse throws); an overload without a frame uses <see cref="Frame3D.World"/>.
/// </summary>
public static class Primitives
{
    /// <summary>A primitive the engine couldn't build: <c>construction.failed</c>.</summary>
    public static DiagnosticCode Failed { get; } = new("construction.failed");

    private static readonly Diagnostic PrimitiveFailed = Diagnostic.Error(Failed, "The primitive couldn't be built.");

    // The least cosine over an angle range: -1 if it passes through an odd multiple of pi
    private static double LeastCosine(double from, double to) =>
        Math.Floor((to - Math.PI) / (2 * Math.PI)) >= Math.Ceiling((from - Math.PI) / (2 * Math.PI)) ? -1 : Math.Min(Math.Cos(from), Math.Cos(to));

    extension(Solid)
    {
        /// <summary>A box of the given sizes with a corner at the origin, along the world axes.</summary>
        public static Result<Built<Solid>> Box(double dx, double dy, double dz) => Box(dx, dy, dz, Frame3D.World);

        /// <summary>A box of the given sizes with a corner at <paramref name="frame"/>'s origin, along its axes.</summary>
        public static Result<Built<Solid>> Box(double dx, double dy, double dz, in Frame3D frame)
        {
            MakerOutcome.Positive(dx, nameof(dx));
            MakerOutcome.Positive(dy, nameof(dy));
            MakerOutcome.Positive(dz, nameof(dz));
            MakerOutcome.Valid(frame, nameof(frame));
            Frame3D f = frame;
            return Run(() => new MakeBox(f, dx, dy, dz));
        }

        /// <summary>The box between two opposite corners, along the world axes.</summary>
        /// <exception cref="ArgumentException">The corners share a coordinate.</exception>
        public static Result<Built<Solid>> Box(Point3D corner1, Point3D corner2)
        {
            if (Math.Abs(corner1.X - corner2.X) <= Tolerances.Confusion || Math.Abs(corner1.Y - corner2.Y) <= Tolerances.Confusion
                || Math.Abs(corner1.Z - corner2.Z) <= Tolerances.Confusion)
            {
                throw new ArgumentException("The corners must differ in every coordinate.", nameof(corner2));
            }
            return Run(() => new MakeBox(corner1, corner2));
        }

        /// <summary>A cylinder on the world Z axis, its base at the origin.</summary>
        public static Result<Built<Solid>> Cylinder(double radius, double height, double angle = 2 * Math.PI) =>
            Cylinder(radius, height, Frame3D.World, angle);

        /// <summary>A cylinder on <paramref name="frame"/>'s axis, its base at its origin, swept by <paramref name="angle"/>
        /// from its X direction.</summary>
        public static Result<Built<Solid>> Cylinder(double radius, double height, in Frame3D frame, double angle = 2 * Math.PI)
        {
            MakerOutcome.Positive(radius, nameof(radius));
            MakerOutcome.Positive(height, nameof(height));
            Angle(angle, nameof(angle));
            MakerOutcome.Valid(frame, nameof(frame));
            Frame3D f = frame;
            return Run(() => new ApiMakeCylinder(f, radius, height, angle));
        }

        /// <summary>A cone (or truncated cone) on the world Z axis, of radius <paramref name="radius1"/> at the origin and
        /// <paramref name="radius2"/> at <paramref name="height"/>; one radius may be zero.</summary>
        public static Result<Built<Solid>> Cone(double radius1, double radius2, double height, double angle = 2 * Math.PI) =>
            Cone(radius1, radius2, height, Frame3D.World, angle);

        /// <summary>A cone on <paramref name="frame"/>'s axis.</summary>
        public static Result<Built<Solid>> Cone(double radius1, double radius2, double height, in Frame3D frame, double angle = 2 * Math.PI)
        {
            ArgumentOutOfRangeException.ThrowIfNegative(radius1);
            ArgumentOutOfRangeException.ThrowIfNegative(radius2);
            if (Math.Abs(radius1 - radius2) <= Tolerances.Confusion)
            {
                throw new ArgumentException("A cone's radii must differ; use a cylinder.", nameof(radius2));
            }
            MakerOutcome.Positive(height, nameof(height));
            Angle(angle, nameof(angle));
            MakerOutcome.Valid(frame, nameof(frame));
            Frame3D f = frame;
            return Run(() => new ApiMakeCone(f, radius1, radius2, height, angle));
        }

        /// <summary>A sphere centred at the origin.</summary>
        public static Result<Built<Solid>> Sphere(double radius, SphereOptions? options = null) => Sphere(radius, Frame3D.World, options);

        /// <summary>A sphere centred at <paramref name="frame"/>'s origin, its poles on its axis.</summary>
        public static Result<Built<Solid>> Sphere(double radius, in Frame3D frame, SphereOptions? options = null)
        {
            options ??= SphereOptions.Default;
            MakerOutcome.Positive(radius, nameof(radius));
            Angle(options.Angle, nameof(options));
            if (!(options.MinLatitude < options.MaxLatitude) || options.MinLatitude < -Math.PI / 2 - Tolerances.Angular
                || options.MaxLatitude > Math.PI / 2 + Tolerances.Angular)
            {
                throw new ArgumentException("The latitudes must increase, within -π/2 to π/2.", nameof(options));
            }
            MakerOutcome.Valid(frame, nameof(frame));
            Frame3D f = frame;
            return Run(() => new MakeSphere(f, radius, options.MinLatitude, options.MaxLatitude, options.Angle));
        }

        /// <summary>A torus about the world Z axis.</summary>
        public static Result<Built<Solid>> Torus(double majorRadius, double minorRadius, TorusOptions? options = null) =>
            Torus(majorRadius, minorRadius, Frame3D.World, options);

        /// <summary>A torus about <paramref name="frame"/>'s axis.</summary>
        /// <exception cref="ArgumentException">The minor circle reaches the axis: the solid would pass through itself.</exception>
        public static Result<Built<Solid>> Torus(double majorRadius, double minorRadius, in Frame3D frame, TorusOptions? options = null)
        {
            options ??= TorusOptions.Default;
            MakerOutcome.Positive(majorRadius, nameof(majorRadius));
            MakerOutcome.Positive(minorRadius, nameof(minorRadius));
            if (majorRadius + minorRadius * LeastCosine(options.MinorStart, options.MinorEnd) <= Tolerances.Confusion)
            {
                throw new ArgumentException("The torus's minor circle reaches its axis, so the solid would pass through itself.", nameof(minorRadius));
            }

            Angle(options.Angle, nameof(options));
            MakerOutcome.Valid(frame, nameof(frame));
            Frame3D f = frame;
            double span = options.MinorEnd - options.MinorStart;
            return span > 0 && span < 2 * Math.PI - Tolerances.Angular
                ? Run(() => TubeSlice(f, majorRadius, minorRadius, options.MinorStart, options.MinorEnd, options.Angle))
                : Run(() => new MakeTorus(f, majorRadius, minorRadius, options.MinorStart, options.MinorEnd, options.Angle));
        }

        /// <summary>A wedge: a box of the given sizes whose face at Y = dy is narrowed as <paramref name="options"/> says.</summary>
        public static Result<Built<Solid>> Wedge(double dx, double dy, double dz, WedgeOptions options) =>
            Wedge(dx, dy, dz, options, Frame3D.World);

        /// <summary>A wedge along <paramref name="frame"/>'s axes.</summary>
        /// <exception cref="ArgumentException">The options give neither a top length nor both top corners.</exception>
        public static Result<Built<Solid>> Wedge(double dx, double dy, double dz, WedgeOptions options, in Frame3D frame)
        {
            ArgumentNullException.ThrowIfNull(options);
            MakerOutcome.Positive(dx, nameof(dx));
            MakerOutcome.Positive(dy, nameof(dy));
            MakerOutcome.Positive(dz, nameof(dz));
            MakerOutcome.Valid(frame, nameof(frame));
            Frame3D f = frame;
            if (options.TopLength is { } ltx)
            {
                ArgumentOutOfRangeException.ThrowIfNegative(ltx, nameof(options));
                return Run(() => new MakeWedge(f, dx, dy, dz, ltx));
            }
            if (options is { TopMin: { } min, TopMax: { } max })
            {
                return Run(() => new MakeWedge(f, dx, dy, dz, min.X, min.Z, max.X, max.Z));
            }
            throw new ArgumentException("Give the wedge's top length, or both of its top corners.", nameof(options));
        }

        /// <summary>The half-space on <paramref name="inside"/>'s side of a face (an infinite solid, for Booleans).</summary>
        public static Result<Built<Solid>> HalfSpace(Face boundary, Point3D inside)
        {
            MakerOutcome.NotNull(boundary, nameof(boundary));
            return Run(() => new MakeHalfSpace(boundary, inside), boundary);
        }

        /// <summary>The half-space on <paramref name="inside"/>'s side of a shell (an infinite solid, for Booleans).</summary>
        public static Result<Built<Solid>> HalfSpace(Shell boundary, Point3D inside)
        {
            MakerOutcome.NotNull(boundary, nameof(boundary));
            return Run(() => new MakeHalfSpace(boundary, inside), boundary);
        }
    }

    private static void Angle(double angle, string name)
    {
        if (!(angle > Tolerances.Angular) || angle > 2 * Math.PI + Tolerances.Angular)
        {
            throw new ArgumentOutOfRangeException(name, angle, "The angle must be more than 0 and at most 2π.");
        }
    }

    /// <summary>
    /// A slice of a torus's tube: the arc of the minor circle from <paramref name="start"/> to <paramref name="end"/>, closed
    /// by the chord across its ends, revolved about the frame's axis.
    /// </summary>
    private static ApiMakeRevol TubeSlice(in Frame3D frame, double major, double minor, double start, double end, double angle)
    {
        Direction3D x = frame.XDirection, y = frame.YDirection, z = frame.Direction;
        Point3D centre = frame.Location.Translated(new Vector3D(x) * major);
        // In the plane of X and Z, angles running from X towards Z as a torus's V does; the plane faces the way it's revolved
        var circle = new Circle3D(new Frame3D(centre, y.Reversed(), x), minor);
        Edge arc = new MakeEdge(circle, start, end).Edge();
        Point3D from = centre.Translated(new Vector3D(x) * (minor * Math.Cos(start)) + new Vector3D(z) * (minor * Math.Sin(start)));
        Point3D to = centre.Translated(new Vector3D(x) * (minor * Math.Cos(end)) + new Vector3D(z) * (minor * Math.Sin(end)));
        var section = new MakeWire(arc, new MakeEdge(to, from).Edge()).Wire();
        Face face = new MakeFace(new Plane3D(new CoordinateSystem(new Frame3D(centre, y, x))), section).Face();
        return new ApiMakeRevol(face, new Axis3D(frame.Location, z), angle);
    }

    private static Result<Built<Solid>> Run(Func<MakeShape> make, Shape input = default) =>
        EngineOperation.Run<Solid>(range => MakerOutcome.Of(make(), range, input.IsNull ? [] : [input]), PrimitiveFailed);
}
