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

    // The size under which a primitive's result is checked (decision 25802). OCCT builds a cylinder, sphere or torus whose
    // radius is under the confusion tolerance, an arc of a nanometre, or a wedge's top of one, and the solid isn't valid;
    // from 2e-7 up every primitive tried was valid as built, so the check is kept to the sizes where it can find something
    private const double Small = 1e-5;

    // What to call the primitive if it's checked, as EngineOperation.Run takes it: only if a length it's made of is small
    private static string? CheckedIf(double smallest, string what) => smallest < Small ? what : null;

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
            return Run(() => new MakeBox(f, dx, dy, dz), checkedAs: CheckedIf(Math.Min(dx, Math.Min(dy, dz)), "The box"));
        }

        /// <summary>The box between two opposite corners, along the world axes.</summary>
        /// <exception cref="ArgumentException">The corners share a coordinate.</exception>
        public static Result<Built<Solid>> Box(Point3D corner1, Point3D corner2)
        {
            if (Math.Abs(corner1.X - corner2.X) <= Tolerances.Confusion || Math.Abs(corner1.Y - corner2.Y) <= Tolerances.Confusion
                || Math.Abs(corner1.Z - corner2.Z) <= Tolerances.Confusion)
            {
                throw new KernelArgumentException("The corners must differ in every coordinate.", nameof(corner2));
            }
            double smallest = Math.Min(Math.Abs(corner1.X - corner2.X), Math.Min(Math.Abs(corner1.Y - corner2.Y), Math.Abs(corner1.Z - corner2.Z)));
            return Run(() => new MakeBox(corner1, corner2), checkedAs: CheckedIf(smallest, "The box"));
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
            return Run(() => new ApiMakeCylinder(f, radius, height, angle),
                       checkedAs: CheckedIf(Math.Min(radius, Math.Min(height, radius * angle)), "The cylinder"));
        }

        /// <summary>A cone (or truncated cone) on the world Z axis, of radius <paramref name="radius1"/> at the origin and
        /// <paramref name="radius2"/> at <paramref name="height"/>; one radius may be zero.</summary>
        public static Result<Built<Solid>> Cone(double radius1, double radius2, double height, double angle = 2 * Math.PI) =>
            Cone(radius1, radius2, height, Frame3D.World, angle);

        /// <summary>A cone on <paramref name="frame"/>'s axis.</summary>
        public static Result<Built<Solid>> Cone(double radius1, double radius2, double height, in Frame3D frame, double angle = 2 * Math.PI)
        {
            KernelArgumentOutOfRangeException.ThrowIfNegative(radius1);
            KernelArgumentOutOfRangeException.ThrowIfNegative(radius2);
            if (Math.Abs(radius1 - radius2) <= Tolerances.Confusion)
            {
                throw new KernelArgumentException("A cone's radii must differ; use a cylinder.", nameof(radius2));
            }
            MakerOutcome.Positive(height, nameof(height));
            Angle(angle, nameof(angle));
            MakerOutcome.Valid(frame, nameof(frame));
            Frame3D f = frame;
            double ends = Math.Min(radius1 > 0 ? radius1 : double.PositiveInfinity, radius2 > 0 ? radius2 : double.PositiveInfinity);
            double smallest = Math.Min(Math.Min(ends, height), Math.Min(Math.Abs(radius1 - radius2), Math.Max(radius1, radius2) * angle));
            return Run(() => new ApiMakeCone(f, radius1, radius2, height, angle), checkedAs: CheckedIf(smallest, "The cone"));
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
                throw new KernelArgumentException("The latitudes must increase, within -π/2 to π/2.", nameof(options));
            }
            MakerOutcome.Valid(frame, nameof(frame));
            Frame3D f = frame;
            double smallest = radius * Math.Min(1, Math.Min(options.MaxLatitude - options.MinLatitude, options.Angle));
            return Run(() => new MakeSphere(f, radius, options.MinLatitude, options.MaxLatitude, options.Angle),
                       checkedAs: CheckedIf(smallest, "The sphere"));
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
                throw new KernelArgumentException("The torus's minor circle reaches its axis, so the solid would pass through itself.", nameof(minorRadius));
            }

            Angle(options.Angle, nameof(options));
            MakerOutcome.Valid(frame, nameof(frame));
            Frame3D f = frame;
            double span = options.MinorEnd - options.MinorStart;
            // The minor circle, its arc if partial, and its nearest approach to the axis, each as long as the revolution
            double clearance = majorRadius + minorRadius * LeastCosine(options.MinorStart, options.MinorEnd);
            double arc = span > 0 ? Math.Min(span, 2 * Math.PI) : 2 * Math.PI;
            double smallest = Math.Min(Math.Min(minorRadius, minorRadius * arc), Math.Min(clearance, clearance * options.Angle));
            string? checkedAs = CheckedIf(smallest, "The torus");
            return span > 0 && span < 2 * Math.PI - Tolerances.Angular
                ? Run(() => TubeSlice(f, majorRadius, minorRadius, options.MinorStart, options.MinorEnd, options.Angle), checkedAs: checkedAs)
                : Run(() => new MakeTorus(f, majorRadius, minorRadius, options.MinorStart, options.MinorEnd, options.Angle), checkedAs: checkedAs);
        }

        /// <summary>A wedge: a box of the given sizes whose face at Y = dy is narrowed as <paramref name="options"/> says.</summary>
        public static Result<Built<Solid>> Wedge(double dx, double dy, double dz, WedgeOptions options) =>
            Wedge(dx, dy, dz, options, Frame3D.World);

        /// <summary>A wedge along <paramref name="frame"/>'s axes.</summary>
        /// <exception cref="ArgumentException">The options give neither a top length nor both top corners.</exception>
        public static Result<Built<Solid>> Wedge(double dx, double dy, double dz, WedgeOptions options, in Frame3D frame)
        {
            KernelArgumentNullException.ThrowIfNull(options);
            MakerOutcome.Positive(dx, nameof(dx));
            MakerOutcome.Positive(dy, nameof(dy));
            MakerOutcome.Positive(dz, nameof(dz));
            MakerOutcome.Valid(frame, nameof(frame));
            Frame3D f = frame;
            // A top of no length or width is a ridge or a point, and is checked as a small one is
            double sides = Math.Min(dx, Math.Min(dy, dz));
            if (options.TopLength is { } ltx)
            {
                KernelArgumentOutOfRangeException.ThrowIfNegative(ltx, nameof(options));
                return Run(() => new MakeWedge(f, dx, dy, dz, ltx), checkedAs: CheckedIf(Math.Min(sides, ltx), "The wedge"));
            }
            if (options is { TopMin: { } min, TopMax: { } max })
            {
                double top = Math.Min(Math.Abs(max.X - min.X), Math.Abs(max.Z - min.Z));
                return Run(() => new MakeWedge(f, dx, dy, dz, min.X, min.Z, max.X, max.Z), checkedAs: CheckedIf(Math.Min(sides, top), "The wedge"));
            }
            throw new KernelArgumentException("Give the wedge's top length, or both of its top corners.", nameof(options));
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
            throw new KernelArgumentOutOfRangeException(name, angle, "The angle must be more than 0 and at most 2π.");
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

    // checkedAs: what to call the result if it's checked, which a failure carrying it then says isn't valid (decision 1742)
    private static Result<Built<Solid>> Run(Func<MakeShape> make, Shape input = default, string? checkedAs = null) =>
        EngineOperation.Run<Solid>(range => MakerOutcome.Of(make(), range, input.IsNull ? [] : [input]), PrimitiveFailed, checkedAs: checkedAs);
}
