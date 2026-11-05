namespace Kerf.Modeling;

/// <summary>
/// How to align an object: one <see cref="Align"/> for every axis, or one per axis for two or three
/// axes. Converts implicitly from an <see cref="Align"/>, the same on every axis.
/// </summary>
public readonly record struct Alignment
{
    readonly Align _x, _y, _z;

    /// <summary>The number of axes given: 1 (the same for every axis), 2 or 3.</summary>
    public int Count { get; }

    Alignment(int count, Align x, Align y, Align z) => (Count, _x, _y, _z) = (count, x, y, z);

    /// <summary>The alignment for one axis (0, 1 or 2), or null where none was given.</summary>
    public Align? this[int axis] => Count == 1 ? _x : axis < Count ? axis switch { 0 => _x, 1 => _y, _ => _z } : null;

    /// <summary>The same alignment on every axis.</summary>
    public static implicit operator Alignment(Align align) => new(1, align, align, align);

    /// <summary>Alignments for the X and Y axes.</summary>
    public Alignment(Align x, Align y) : this(2, x, y, Align.None) { }

    /// <summary>Alignments for the X, Y and Z axes.</summary>
    public Alignment(Align x, Align y, Align z) : this(3, x, y, z) { }

    /// <summary>
    /// The amount to move an object spanning <paramref name="min"/> to <paramref name="max"/> to align it.
    /// </summary>
    /// <param name="min">The object's minimum corner.</param>
    /// <param name="max">The object's maximum corner.</param>
    /// <param name="center">The point Center aligns; defaults to the midpoint of min and max.</param>
    public Vector3D Offset(Point3D min, Point3D max, Point3D? center = null)
    {
        var c = center ?? ((min.ToVector() + max.ToVector()) / 2).ToPoint();
        if (Count == 1)
        {
            return _x switch
            {
                Align.None => new Vector3D(0, 0, 0),
                Align.Min => -min.ToVector(),
                Align.Max => -max.ToVector(),
                _ => -c.ToVector(),
            };
        }

        // Per-axis alignment; a two-axis alignment leaves Z at zero
        Span<double> offset = [0, 0, 0];
        ReadOnlySpan<double> mins = [min.X, min.Y, min.Z], centres = [c.X, c.Y, c.Z], maxes = [max.X, max.Y, max.Z];
        for (int i = 0; i < Count; i++)
        {
            offset[i] = this[i] switch
            {
                Align.Min => -mins[i],
                Align.Center => -centres[i],
                Align.Max => -maxes[i],
                _ => 0,
            };
        }
        return new Vector3D(offset[0], offset[1], offset[2]);
    }
}
