// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;

namespace Kerf.Geometry;

/// <summary>
/// A box in any orientation: a centre, three orthonormal axes and the half-lengths along them. It replaces OCCT's
/// <c>Bnd_OBB</c> (a mutable struct) in the public API; the measurement area builds them
/// (<c>MeasureOperations.OrientedBox</c>).
/// </summary>
/// <remarks><c>default(OrientedBox)</c> is <see cref="Empty"/>: it contains nothing. Equality is exact.</remarks>
public readonly struct OrientedBox : IEquatable<OrientedBox>
{
    private readonly Point3D _centre;
    private readonly Direction3D _xAxis, _yAxis, _zAxis;
    private readonly Vector3D _halfSize;
    private readonly bool _isSet;

    /// <summary>The box centred at <paramref name="centre"/>, with half-lengths <paramref name="halfSize"/> along
    /// <paramref name="xAxis"/>, <paramref name="yAxis"/> and <paramref name="zAxis"/>.</summary>
    /// <exception cref="ArgumentException">An axis is a default (zero) direction, the axes aren't perpendicular, or a
    /// half-length is negative or NaN.</exception>
    public OrientedBox(Point3D centre, Direction3D xAxis, Direction3D yAxis, Direction3D zAxis, Vector3D halfSize)
    {
        if (IsZero(xAxis) || IsZero(yAxis) || IsZero(zAxis))
        {
            throw new ArgumentException("The box's axes must be valid directions, not default ones.", nameof(xAxis));
        }

        if (Math.Abs(xAxis.Dot(yAxis)) > Tolerances.Angular * 1e6 || Math.Abs(yAxis.Dot(zAxis)) > Tolerances.Angular * 1e6
            || Math.Abs(zAxis.Dot(xAxis)) > Tolerances.Angular * 1e6)
        {
            throw new ArgumentException("The box's axes must be perpendicular.", nameof(yAxis));
        }

        if (!(halfSize.X >= 0 && halfSize.Y >= 0 && halfSize.Z >= 0))
        {
            throw new ArgumentException($"The box's half-lengths {halfSize} must be zero or more.", nameof(halfSize));
        }

        _centre = centre;
        _xAxis = xAxis;
        _yAxis = yAxis;
        _zAxis = zAxis;
        _halfSize = halfSize;
        _isSet = true;
    }

    /// <summary>The axis-aligned box <paramref name="box"/> as an oriented one; empty if it is empty.</summary>
    public static OrientedBox FromBoundingBox(in BoundingBox box)
        => box.IsEmpty
            ? Empty
            : new OrientedBox(box.Centre, Direction3D.UnitX, Direction3D.UnitY, Direction3D.UnitZ, 0.5 * box.Size);

    /// <summary>The empty box, which contains nothing.</summary>
    public static OrientedBox Empty => default;

    /// <summary>True if the box contains nothing.</summary>
    public bool IsEmpty => !_isSet;

    /// <summary>The centre.</summary>
    /// <exception cref="InvalidOperationException">The box is empty.</exception>
    public Point3D Centre => _isSet ? _centre : throw EmptyBox();

    /// <summary>The box's first axis.</summary>
    /// <exception cref="InvalidOperationException">The box is empty.</exception>
    public Direction3D XAxis => _isSet ? _xAxis : throw EmptyBox();

    /// <summary>The box's second axis.</summary>
    /// <exception cref="InvalidOperationException">The box is empty.</exception>
    public Direction3D YAxis => _isSet ? _yAxis : throw EmptyBox();

    /// <summary>The box's third axis.</summary>
    /// <exception cref="InvalidOperationException">The box is empty.</exception>
    public Direction3D ZAxis => _isSet ? _zAxis : throw EmptyBox();

    /// <summary>The half-lengths along <see cref="XAxis"/>, <see cref="YAxis"/> and <see cref="ZAxis"/>; zero if
    /// the box is empty.</summary>
    public Vector3D HalfSize => _isSet ? _halfSize : Vector3D.Zero;

    /// <summary>The volume; zero if the box is empty.</summary>
    public double Volume => 8 * HalfSize.X * HalfSize.Y * HalfSize.Z;

    /// <summary>The eight corners, or none if the box is empty.</summary>
    public Point3D[] Corners()
    {
        if (!_isSet)
        {
            return [];
        }

        var corners = new Point3D[8];
        int i = 0;
        foreach (double z in (ReadOnlySpan<double>)[-1, 1])
        {
            foreach (double y in (ReadOnlySpan<double>)[-1, 1])
            {
                foreach (double x in (ReadOnlySpan<double>)[-1, 1])
                {
                    corners[i++] = _centre + x * _halfSize.X * new Vector3D(_xAxis) + y * _halfSize.Y * new Vector3D(_yAxis)
                                   + z * _halfSize.Z * new Vector3D(_zAxis);
                }
            }
        }

        return corners;
    }

    /// <summary>True if <paramref name="point"/> is inside the box or on its boundary.</summary>
    public bool Contains(Point3D point) => _isSet && !ToEngine().IsOut(point);

    /// <summary>True if this box and <paramref name="other"/> share a point (touching counts).</summary>
    public bool Intersects(in OrientedBox other) => _isSet && other._isSet && !ToEngine().IsOut(other.ToEngine());

    /// <summary>The smallest axis-aligned box containing this one.</summary>
    public BoundingBox ToBoundingBox() => BoundingBox.Of(Corners());

    /// <inheritdoc/>
    public bool Equals(OrientedBox other)
        => _isSet == other._isSet
           && (!_isSet || (_centre == other._centre && _xAxis == other._xAxis && _yAxis == other._yAxis && _zAxis == other._zAxis
                           && _halfSize == other._halfSize));

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is OrientedBox other && Equals(other);

    /// <inheritdoc/>
    public override int GetHashCode() => _isSet ? HashCode.Combine(_centre, _xAxis, _yAxis, _zAxis, _halfSize) : 0;

    /// <summary>Exact equality.</summary>
    public static bool operator ==(in OrientedBox left, in OrientedBox right) => left.Equals(right);

    /// <summary>Exact inequality.</summary>
    public static bool operator !=(in OrientedBox left, in OrientedBox right) => !left.Equals(right);

    /// <inheritdoc/>
    public override string ToString()
        => _isSet ? $"OrientedBox(centre {_centre}, axes {_xAxis} {_yAxis} {_zAxis}, half-size {_halfSize})" : "OrientedBox(empty)";

    /// <summary>The engine's box: <c>Bnd_OBB</c>.</summary>
    internal Obb ToEngine()
        => _isSet ? new Obb(_centre, _xAxis, _yAxis, _zAxis, _halfSize.X, _halfSize.Y, _halfSize.Z) : Obb.Void();

    /// <summary>The engine's box as a public one; empty if it is void.</summary>
    internal static OrientedBox FromEngine(in Obb box)
    {
        if (box.IsVoid)
        {
            return Empty;
        }

        Obb.HalfSizes half = box.GetHalfSizes();
        return new OrientedBox(new Point3D(box.Center()), new Direction3D(box.XDirection()), new Direction3D(box.YDirection()),
                               new Direction3D(box.ZDirection()), new Vector3D(half.X, half.Y, half.Z));
    }

    private static bool IsZero(Direction3D direction) => direction.X == 0 && direction.Y == 0 && direction.Z == 0;

    private static InvalidOperationException EmptyBox() => new("The box is empty.");
}
