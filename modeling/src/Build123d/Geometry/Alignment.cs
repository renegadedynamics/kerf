namespace Build123d;

/// <summary>
/// How to align an object: one <see cref="Align"/> for every axis, or one per axis for two or three
/// axes. Converts implicitly from an <see cref="Align"/> or a tuple of them.
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
    public static implicit operator Alignment((Align X, Align Y) align) => new(2, align.X, align.Y, Align.None);

    /// <summary>Alignments for the X, Y and Z axes.</summary>
    public static implicit operator Alignment((Align X, Align Y, Align Z) align) => new(3, align.X, align.Y, align.Z);

    /// <summary>
    /// The amount to move an object spanning <paramref name="min"/> to <paramref name="max"/> to align it.
    /// </summary>
    /// <param name="min">The object's minimum corner.</param>
    /// <param name="max">The object's maximum corner.</param>
    /// <param name="center">The point Center aligns; defaults to the midpoint of min and max.</param>
    public Vector Offset(Vector min, Vector max, Vector? center = null)
    {
        var c = center ?? (min + max) / 2;
        if (Count == 1)
        {
            return _x switch
            {
                Align.None => new Vector(0, 0, 0),
                Align.Min => -min,
                Align.Max => -max,
                _ => -c,
            };
        }

        // Per-axis alignment; a two-axis alignment leaves Z at zero
        Span<double> offset = [0, 0, 0];
        for (int i = 0; i < Count; i++)
        {
            offset[i] = this[i] switch
            {
                Align.Min => -min[i],
                Align.Center => -c[i],
                Align.Max => -max[i],
                _ => 0,
            };
        }
        return new Vector(offset[0], offset[1], offset[2]);
    }
}
