// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Geometry;

/// <summary>
/// An axis-aligned box: the bounds of a shape, a set of points or a curve. It replaces OCCT's <c>Bnd_Box</c> (a mutable
/// class) in the public API; the measurement area builds them.
/// </summary>
/// <remarks>
/// <c>default(BoundingBox)</c> is <see cref="Empty"/>: it contains nothing, and a union with it is the other box. A box
/// of an infinite shape has infinite bounds. Equality is exact.
/// </remarks>
public readonly struct BoundingBox : IEquatable<BoundingBox>
{
    private readonly Point3D _min;
    private readonly Point3D _max;
    private readonly bool _isSet;

    /// <summary>The box from <paramref name="min"/> to <paramref name="max"/>.</summary>
    /// <exception cref="ArgumentException">A coordinate of <paramref name="min"/> is greater than <paramref name="max"/>'s,
    /// or one is NaN.</exception>
    public BoundingBox(Point3D min, Point3D max)
    {
        if (!(min.X <= max.X && min.Y <= max.Y && min.Z <= max.Z))
        {
            throw new KernelArgumentException($"The box's minimum {min} is not below its maximum {max} in every coordinate.", nameof(min));
        }

        _min = min;
        _max = max;
        _isSet = true;
    }

    /// <summary>The empty box, which contains nothing.</summary>
    public static BoundingBox Empty => default;

    /// <summary>The smallest box containing every point of <paramref name="points"/>; empty if there are none.</summary>
    public static BoundingBox Of(params ReadOnlySpan<Point3D> points)
    {
        BoundingBox box = Empty;
        foreach (Point3D point in points)
        {
            box = box.Including(point);
        }

        return box;
    }

    /// <summary>True if the box contains nothing.</summary>
    public bool IsEmpty => !_isSet;

    /// <summary>The corner of smallest coordinates.</summary>
    /// <exception cref="InvalidOperationException">The box is empty.</exception>
    public Point3D Min => _isSet ? _min : throw EmptyBox();

    /// <summary>The corner of largest coordinates.</summary>
    /// <exception cref="InvalidOperationException">The box is empty.</exception>
    public Point3D Max => _isSet ? _max : throw EmptyBox();

    /// <summary>The box's extent along each axis; zero if it is empty.</summary>
    public Vector3D Size => _isSet ? _max - _min : Vector3D.Zero;

    /// <summary>The centre.</summary>
    /// <exception cref="InvalidOperationException">The box is empty.</exception>
    public Point3D Centre => _isSet
        ? new Point3D(0.5 * (_min.X + _max.X), 0.5 * (_min.Y + _max.Y), 0.5 * (_min.Z + _max.Z))
        : throw EmptyBox();

    /// <summary>True if a bound is infinite: beyond half of <see cref="Tolerances.Infinite"/>, as OCCT's
    /// <c>Precision::IsInfinite</c>.</summary>
    public bool IsInfinite => _isSet && (Far(_min.X) || Far(_min.Y) || Far(_min.Z) || Far(_max.X) || Far(_max.Y) || Far(_max.Z));

    /// <summary>The smallest box containing this one and <paramref name="other"/>.</summary>
    public BoundingBox Union(in BoundingBox other)
    {
        if (!other._isSet)
        {
            return this;
        }

        if (!_isSet)
        {
            return other;
        }

        return new BoundingBox(new Point3D(Math.Min(_min.X, other._min.X), Math.Min(_min.Y, other._min.Y), Math.Min(_min.Z, other._min.Z)),
                               new Point3D(Math.Max(_max.X, other._max.X), Math.Max(_max.Y, other._max.Y), Math.Max(_max.Z, other._max.Z)));
    }

    /// <summary>The smallest box containing this one and <paramref name="point"/>.</summary>
    public BoundingBox Including(Point3D point) => Union(new BoundingBox(point, point));

    /// <summary>The box grown by <paramref name="gap"/> on every side; an empty box stays empty.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="gap"/> is negative or NaN.</exception>
    public BoundingBox Enlarged(double gap)
    {
        if (!(gap >= 0))
        {
            throw new KernelArgumentOutOfRangeException(nameof(gap), gap, "The gap must be zero or more.");
        }

        return _isSet
            ? new BoundingBox(new Point3D(_min.X - gap, _min.Y - gap, _min.Z - gap), new Point3D(_max.X + gap, _max.Y + gap, _max.Z + gap))
            : this;
    }

    /// <summary>True if <paramref name="point"/> is inside the box or on its boundary.</summary>
    public bool Contains(Point3D point)
        => _isSet && point.X >= _min.X && point.X <= _max.X && point.Y >= _min.Y && point.Y <= _max.Y && point.Z >= _min.Z && point.Z <= _max.Z;

    /// <summary>True if this box and <paramref name="other"/> share a point (touching counts).</summary>
    public bool Intersects(in BoundingBox other)
        => _isSet && other._isSet
           && _min.X <= other._max.X && other._min.X <= _max.X
           && _min.Y <= other._max.Y && other._min.Y <= _max.Y
           && _min.Z <= other._max.Z && other._min.Z <= _max.Z;

    /// <summary>The box transformed: the smallest axis-aligned box containing its transformed corners.</summary>
    public BoundingBox Transformed(in Transform3D transform)
    {
        if (!_isSet)
        {
            return this;
        }

        Span<Point3D> corners =
        [
            new(_min.X, _min.Y, _min.Z), new(_max.X, _min.Y, _min.Z), new(_min.X, _max.Y, _min.Z), new(_max.X, _max.Y, _min.Z),
            new(_min.X, _min.Y, _max.Z), new(_max.X, _min.Y, _max.Z), new(_min.X, _max.Y, _max.Z), new(_max.X, _max.Y, _max.Z),
        ];
        foreach (ref Point3D corner in corners)
        {
            corner = corner.Transformed(transform);
        }

        return Of(corners);
    }

    /// <inheritdoc/>
    public bool Equals(BoundingBox other) => _isSet == other._isSet && (!_isSet || (_min == other._min && _max == other._max));

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is BoundingBox other && Equals(other);

    /// <inheritdoc/>
    public override int GetHashCode() => _isSet ? HashCode.Combine(_min, _max) : 0;

    /// <summary>Exact equality.</summary>
    public static bool operator ==(in BoundingBox left, in BoundingBox right) => left.Equals(right);

    /// <summary>Exact inequality.</summary>
    public static bool operator !=(in BoundingBox left, in BoundingBox right) => !left.Equals(right);

    /// <inheritdoc/>
    public override string ToString()
        => _isSet ? $"BoundingBox({_min.X}, {_min.Y}, {_min.Z} .. {_max.X}, {_max.Y}, {_max.Z})" : "BoundingBox(empty)";

    private static bool Far(double value) => Math.Abs(value) >= 0.5 * Tolerances.Infinite;

    private static InvalidOperationException EmptyBox() => new("The box is empty.");
}
