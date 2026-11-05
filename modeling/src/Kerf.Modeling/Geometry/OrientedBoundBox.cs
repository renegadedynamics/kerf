using Kerf.Geometry;

namespace Kerf.Modeling;

/// <summary>
/// An immutable oriented bounding box: the box's centre, principal axes and extents along them.
/// </summary>
/// <remarks>The axes are arbitrary and may differ between platforms and OCCT versions.</remarks>
public sealed class OrientedBoundBox
{
    /// <summary>The kernel's box.</summary>
    internal OrientedBox Wrapped { get; }

    OrientedBoundBox(OrientedBox box) => Wrapped = box;

    /// <summary>An oriented bounding box from the kernel's.</summary>
    public static OrientedBoundBox FromOrientedBox(OrientedBox box) => new(box);

    /// <summary>True for the box of nothing, which has no size or axes.</summary>
    public bool IsEmpty => Wrapped.IsEmpty;

    /// <summary>
    /// The distinct corners. A degenerate box (of a line or a planar face) has only two or four, and a
    /// flat box's corners are in order around it, ready to make a polygon.
    /// </summary>
    public IReadOnlyList<Point3D> Corners
    {
        get
        {
            var hs = Size * 0.5;
            (int, int, int)[] order = (hs.X < Constants.Tolerance, hs.Y < Constants.Tolerance, hs.Z < Constants.Tolerance) switch
            {
                // Straight lines
                (true, true, false) => [(1, 1, 1), (1, 1, -1)],
                (true, false, true) => [(1, 1, 1), (1, -1, 1)],
                (false, true, true) => [(1, 1, 1), (-1, 1, 1)],
                // Planar faces
                (true, false, false) => [(1, 1, 1), (1, 1, -1), (1, -1, -1), (1, -1, 1)],
                (false, true, false) => [(1, 1, 1), (1, 1, -1), (-1, 1, -1), (-1, 1, 1)],
                (false, false, true) => [(1, 1, 1), (1, -1, 1), (-1, -1, 1), (-1, 1, 1)],
                // A solid, in itertools.product((-1, 1), repeat=3) order
                (false, false, false) =>
                [
                    (-1, -1, -1), (-1, -1, 1), (-1, 1, -1), (-1, 1, 1),
                    (1, -1, -1), (1, -1, 1), (1, 1, -1), (1, 1, 1),
                ],
                _ => [(1, 1, 1)],
            };
            var plane = Plane;
            return order.Select(o => plane.FromLocalCoords(new Point3D(o.Item1 * hs.X, o.Item2 * hs.Y, o.Item3 * hs.Z))).ToArray();
        }
    }

    /// <summary>The length of the body diagonal: the object's maximum size.</summary>
    public double Diagonal => Wrapped.Diagonal;

    /// <summary>The location of the box's centre, with its axes.</summary>
    public Location Location => new(Plane);

    /// <summary>The box's coordinate system: its centre, with its primary (X) and tertiary (Z) directions.</summary>
    public Plane Plane => IsEmpty ? Plane.XY : Plane.Create(Center(), XDirection, ZDirection);

    /// <summary>The full extents along the box's axes.</summary>
    public Vector3D Size => IsEmpty
        ? new Vector3D(0, 0, 0)
        : Wrapped.HalfSize * 2.0;

    /// <summary>The primary (X) direction, a unit vector.</summary>
    public Direction3D XDirection => IsEmpty ? new Direction3D(1, 0, 0) : Wrapped.XAxis;

    /// <summary>The secondary (Y) direction, a unit vector.</summary>
    public Direction3D YDirection => IsEmpty ? new Direction3D(0, 1, 0) : Wrapped.YAxis;

    /// <summary>The tertiary (Z) direction, a unit vector.</summary>
    public Direction3D ZDirection => IsEmpty ? new Direction3D(0, 0, 1) : Wrapped.ZAxis;

    /// <summary>The centre point.</summary>
    public Point3D Center() => IsEmpty ? new Point3D(0, 0, 0) : Wrapped.Centre;

    /// <summary>True if <paramref name="other"/> lies entirely within this box, by OCCT's criteria.</summary>
    public bool IsCompletelyInside(OrientedBoundBox other) => Wrapped.Contains(other.Wrapped);

    /// <summary>True if the point is neither inside the box nor on its surface.</summary>
    public bool IsOutside(Point3D point) => !Wrapped.Contains(point);

    /// <inheritdoc/>
    public override string ToString() => $"OrientedBoundBox(center={Center().Describe("G13")}, size={Size.Describe("G13")}, plane={Plane.ToString("G13")})";
}
