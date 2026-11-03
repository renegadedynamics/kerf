using Kerf.Engine.Geometry.Bounds;
using Occt;

namespace Kerf.Modeling;

/// <summary>
/// An immutable oriented bounding box: the box's centre, principal axes and extents along them.
/// </summary>
/// <remarks>The axes are arbitrary and may differ between platforms and OCCT versions.</remarks>
public sealed class OrientedBoundBox
{
    /// <summary>The OCCT box. Never modified.</summary>
    internal OrientedBox Wrapped { get; }

    OrientedBoundBox(OrientedBox obb) => Wrapped = obb;

    /// <summary>Wraps a Bnd_OBB this box takes ownership of.</summary>
    internal static OrientedBoundBox Own(OrientedBox obb) => new(obb);

    /// <summary>An oriented bounding box from an OCCT Bnd_OBB.</summary>
    public static OrientedBoundBox FromBndObb(OrientedBox obb) => new(obb);

    /// <summary>True for the box of nothing, which has no size or axes.</summary>
    public bool IsEmpty => Wrapped.IsVoid;

    /// <summary>
    /// The distinct corners. A degenerate box (of a line or a planar face) has only two or four, and a
    /// flat box's corners are in order around it, ready to make a polygon.
    /// </summary>
    public IReadOnlyList<Vector> Corners
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
            return order.Select(o => plane.FromLocalCoords(new Vector(o.Item1 * hs.X, o.Item2 * hs.Y, o.Item3 * hs.Z))).ToArray();
        }
    }

    /// <summary>The length of the body diagonal: the object's maximum size.</summary>
    public double Diagonal => Math.Sqrt(Wrapped.SquareExtent());

    /// <summary>The location of the box's centre, with its axes.</summary>
    public Location Location => new(Plane);

    /// <summary>The box's coordinate system: its centre, with its primary (X) and tertiary (Z) directions.</summary>
    public Plane Plane => IsEmpty ? Plane.XY : Plane.Create(Center(), XDirection, ZDirection);

    /// <summary>The full extents along the box's axes.</summary>
    public Vector Size => IsEmpty
        ? new Vector(0, 0, 0)
        : new Vector(Wrapped.XHSize(), Wrapped.YHSize(), Wrapped.ZHSize()) * 2.0;

    /// <summary>The primary (X) direction, a unit vector.</summary>
    public Vector XDirection { get { var d = Wrapped.XDirection(); return new Vector(d); } }

    /// <summary>The secondary (Y) direction, a unit vector.</summary>
    public Vector YDirection { get { var d = Wrapped.YDirection(); return new Vector(d); } }

    /// <summary>The tertiary (Z) direction, a unit vector.</summary>
    public Vector ZDirection { get { var d = Wrapped.ZDirection(); return new Vector(d); } }

    /// <summary>The centre point.</summary>
    public Vector Center() { var c = Wrapped.Center(); return new Vector(c); }

    /// <summary>True if <paramref name="other"/> lies entirely within this box, by OCCT's criteria.</summary>
    public bool IsCompletelyInside(OrientedBoundBox other) => Wrapped.IsCompletelyInside(other.Wrapped);

    /// <summary>True if the point is neither inside the box nor on its surface.</summary>
    public bool IsOutside(Vector point)
    {
        var p = point.ToGpPnt();
        return Wrapped.IsOut(p);
    }

    /// <inheritdoc/>
    public override string ToString() => $"OrientedBoundBox(center={Center().Repr()}, size={Size.Repr()}, plane={Plane.Repr()})";
}
