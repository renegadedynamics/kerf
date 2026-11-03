// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;

namespace Kerf.Topology;

/// <summary>
/// The two ways to compare shapes, for dictionaries and sets of shapes (which must name one: KERF0002).
/// </summary>
/// <remarks>
/// Both compare a shape's data by identity and its placement as the engine does (by the identity of its datums, decision
/// 1349, question 62): two shapes moved by equal transforms built separately are not the same.
/// </remarks>
public sealed class ShapeComparer : IEqualityComparer<Shape>
{
    private readonly bool _orientation;

    private ShapeComparer(bool orientation) => _orientation = orientation;

    /// <summary>The same data and placement, either orientation (<see cref="Shape.IsSame"/>): "the same face" of a solid,
    /// whichever way it is used.</summary>
    public static ShapeComparer Same { get; } = new(false);

    /// <summary>The same data, placement and orientation (<see cref="Shape.IsEqual"/>, and <c>==</c>).</summary>
    public static ShapeComparer Equal { get; } = new(true);

    /// <inheritdoc/>
    public bool Equals(Shape x, Shape y) => _orientation ? x.IsEqual(y) : x.IsSame(y);

    /// <inheritdoc/>
    public int GetHashCode(Shape obj) => ShapeHasher.Instance.GetHashCode(obj);
}
