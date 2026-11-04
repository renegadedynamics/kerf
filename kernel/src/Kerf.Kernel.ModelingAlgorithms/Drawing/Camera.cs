// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.HiddenLines.Algorithm;
using Kerf.Geometry;

namespace Kerf.Drawing;

/// <summary>How a <see cref="Camera"/> projects.</summary>
public enum Projection
{
    /// <summary>Parallel projection along the view direction: sizes don't change with depth.</summary>
    Orthographic,

    /// <summary>Central projection from the eye: nearer things look larger.</summary>
    Perspective,
}

/// <summary>
/// A view of a model, shared by line drawings (<see cref="DrawingOperations"/>) and pictures (<c>Renderer</c>):
/// an eye looking at a target, with an up direction, orthographic or perspective (docs/api/drawing.md).
/// </summary>
/// <remarks>
/// <para>
/// The view's frame is OCCT's projector's: its origin is the target, its Z axis points from the target towards the eye,
/// its X axis to the right and its Y axis up (the up direction made perpendicular to the view). A point's view
/// coordinates are its coordinates in that frame; its projection is (X, Y) for an orthographic camera, and
/// (X, Y) / (1 - Z / d) for a perspective one, d being the distance from the eye to the target, so the target's plane
/// keeps its scale. build123d's <c>project_to_viewport(viewport_origin, viewport_up, look_at)</c> is
/// <c>Camera.LookAt(viewport_origin, look_at, viewport_up)</c>.
/// </para>
/// <para>
/// An orthographic camera uses only the direction from the eye to the target, not its distance. The eye and the target
/// must differ, and the up direction mustn't be along the view: the operations taking a camera throw an
/// <see cref="ArgumentException"/> otherwise.
/// </para>
/// </remarks>
public sealed record Camera
{
    /// <summary>The eye: where the view is from.</summary>
    public Point3D Eye { get; init; } = new(0, 0, 1);

    /// <summary>The target: the point looked at, the centre of the view.</summary>
    public Point3D Target { get; init; } = Point3D.Origin;

    /// <summary>The direction that is up in the view (made perpendicular to the view direction).</summary>
    public Direction3D Up { get; init; } = Direction3D.UnitY;

    /// <summary>Orthographic (the default) or perspective.</summary>
    public Projection Projection { get; init; }

    /// <summary>A camera at <paramref name="eye"/> looking at <paramref name="target"/>, with Z up unless said otherwise.</summary>
    public static Camera LookAt(Point3D eye, Point3D target, Direction3D? up = null, Projection projection = Projection.Orthographic)
        => new() { Eye = eye, Target = target, Up = up ?? Direction3D.UnitZ, Projection = projection };

    /// <summary>An orthographic camera looking along <paramref name="viewDirection"/> at the origin, with Z up unless
    /// said otherwise.</summary>
    public static Camera Along(Direction3D viewDirection, Direction3D? up = null)
        => new()
        {
            Eye = new Point3D(-viewDirection.X, -viewDirection.Y, -viewDirection.Z),
            Target = Point3D.Origin,
            Up = up ?? Direction3D.UnitZ,
        };

    /// <summary>Looking down the Z axis, Y up: the plan.</summary>
    public static Camera Top { get; } = new() { Eye = new Point3D(0, 0, 1), Up = Direction3D.UnitY };

    /// <summary>Looking along +Y, Z up: the front elevation.</summary>
    public static Camera Front { get; } = new() { Eye = new Point3D(0, -1, 0), Up = Direction3D.UnitZ };

    /// <summary>Looking along -X, Z up: the right elevation.</summary>
    public static Camera Right { get; } = new() { Eye = new Point3D(1, 0, 0), Up = Direction3D.UnitZ };

    /// <summary>Looking from (1, -1, 1) at the origin, Z up: the usual isometric view.</summary>
    public static Camera Isometric { get; } = new() { Eye = new Point3D(1, -1, 1), Up = Direction3D.UnitZ };

    /// <summary>The direction looked along: from the eye to the target.</summary>
    /// <exception cref="ArgumentException">The eye is the target.</exception>
    public Direction3D ViewDirection
    {
        get
        {
            Vector3D v = Target - Eye;
            return v.Magnitude > 0 ? new Direction3D(v) : throw new ArgumentException("The camera's eye is its target.");
        }
    }

    /// <summary>The distance from the eye to the target.</summary>
    public double Distance => Eye.DistanceTo(Target);

    /// <summary>The view's frame: origin the target, Z towards the eye, X to the right, Y up.</summary>
    /// <exception cref="ArgumentException">The eye is the target, or the up direction is along the view.</exception>
    public Frame3D ViewFrame
    {
        get
        {
            Direction3D z = ViewDirection;
            z = new Direction3D(-z.X, -z.Y, -z.Z);
            Direction3D up = Up;
            if (up.X == 0 && up.Y == 0 && up.Z == 0)
            {
                throw new ArgumentException("The camera's up direction is a default one.");
            }

            if (up.IsParallel(z, 1e-9))
            {
                throw new ArgumentException("The camera's up direction is along its view direction.");
            }

            return new Frame3D(Target, z, up.Crossed(z));
        }
    }

    /// <summary>The point's projection in the view plane (X right, Y up), as the drawings' 2D curves are.</summary>
    /// <exception cref="ArgumentException">The camera isn't valid (see <see cref="ViewFrame"/>).</exception>
    public Point2D Project(Point3D point)
    {
        ViewTransform view = ViewTransform.Of(this);
        view.ToView(point.X, point.Y, point.Z, out double x, out double y, out double z);
        view.Projected(x, y, z, out double px, out double py);
        return new Point2D(px, py);
    }

    /// <summary>The engine's projector for this camera (HLRAlgo_Projector).</summary>
    internal AlgorithmProjector ToProjector()
    {
        Frame3D frame = ViewFrame;
        return Projection == Projection.Perspective ? new AlgorithmProjector(frame, Distance) : new AlgorithmProjector(frame);
    }
}

/// <summary>A camera's view coordinates and projection as plain arithmetic, the same on every machine.</summary>
internal readonly struct ViewTransform
{
    private readonly double _ox, _oy, _oz;
    private readonly double _xx, _xy, _xz, _yx, _yy, _yz, _zx, _zy, _zz;

    private ViewTransform(Point3D origin, Direction3D x, Direction3D y, Direction3D z, double focus)
    {
        (_ox, _oy, _oz) = (origin.X, origin.Y, origin.Z);
        (_xx, _xy, _xz) = (x.X, x.Y, x.Z);
        (_yx, _yy, _yz) = (y.X, y.Y, y.Z);
        (_zx, _zy, _zz) = (z.X, z.Y, z.Z);
        Focus = focus;
    }

    /// <summary>The eye's distance from the target for a perspective camera; zero for an orthographic one.</summary>
    public double Focus { get; }

    public bool IsPerspective => Focus > 0;

    public static ViewTransform Of(Camera camera)
    {
        Frame3D frame = camera.ViewFrame;
        double focus = camera.Projection == Projection.Perspective ? camera.Distance : 0;
        return new ViewTransform(frame.Location, frame.XDirection, frame.YDirection, frame.Direction, focus);
    }

    /// <summary>World coordinates to the view's.</summary>
    public void ToView(double px, double py, double pz, out double x, out double y, out double z)
    {
        double dx = px - _ox, dy = py - _oy, dz = pz - _oz;
        x = dx * _xx + dy * _xy + dz * _xz;
        y = dx * _yx + dy * _yy + dz * _yz;
        z = dx * _zx + dy * _zy + dz * _zz;
    }

    /// <summary>A world direction in the view's frame.</summary>
    public void DirectionToView(double dx, double dy, double dz, out double x, out double y, out double z)
    {
        x = dx * _xx + dy * _xy + dz * _xz;
        y = dx * _yx + dy * _yy + dz * _yz;
        z = dx * _zx + dy * _zy + dz * _zz;
    }

    /// <summary>View coordinates projected on the view plane.</summary>
    public void Projected(double x, double y, double z, out double px, out double py)
    {
        if (Focus > 0)
        {
            double r = 1 - z / Focus;
            px = x / r;
            py = y / r;
        }
        else
        {
            px = x;
            py = y;
        }
    }
}
