// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Topology;

/// <summary>
/// What the nine shape types share: <see cref="Shape"/> and the typed <see cref="Vertex"/>, <see cref="Edge"/>,
/// <see cref="Wire"/>, <see cref="Face"/>, <see cref="Shell"/>, <see cref="Solid"/>, <see cref="CompSolid"/> and
/// <see cref="Compound"/>. Generic code uses it: <c>shape.Explore&lt;Face&gt;()</c>, <c>shape.TryAs(out Edge edge)</c>.
/// </summary>
/// <typeparam name="TSelf">The shape type itself.</typeparam>
public interface IShape<TSelf> where TSelf : struct, IShape<TSelf>
{
    /// <summary>The shape type this type holds: <see cref="ShapeType.Shape"/> for <see cref="Shape"/>, which holds any.</summary>
    static abstract ShapeType Type { get; }

    /// <summary>This shape as a <see cref="Shape"/>.</summary>
    Shape AsShape();
}
