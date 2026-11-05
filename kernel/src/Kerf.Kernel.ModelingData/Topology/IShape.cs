// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Topology;

/// <summary>
/// Anything that is, or holds, one shape: the nine shape types, and the modelling layer's objects. An extension written on
/// it (<c>extension&lt;T&gt;(T shape) where T : IShape</c>) is on all of them, without boxing the structs.
/// </summary>
public interface IShape
{
    /// <summary>This shape as a <see cref="Shape"/>.</summary>
    Shape AsShape();
}

/// <summary>A one-dimensional shape: <see cref="Edge"/> or <see cref="Wire"/>.</summary>
public interface IShape1D : IShape;

/// <summary>A two-dimensional shape: <see cref="Face"/> or <see cref="Shell"/>.</summary>
public interface IShape2D : IShape;

/// <summary>A shape of solids or of mixed content: <see cref="Solid"/>, <see cref="CompSolid"/> or <see cref="Compound"/>.</summary>
public interface IShape3D : IShape;

/// <summary>
/// What the nine shape types share: <see cref="Shape"/> and the typed <see cref="Vertex"/>, <see cref="Edge"/>,
/// <see cref="Wire"/>, <see cref="Face"/>, <see cref="Shell"/>, <see cref="Solid"/>, <see cref="CompSolid"/> and
/// <see cref="Compound"/>. Generic code uses it: <c>shape.Explore&lt;Face&gt;()</c>, <c>shape.TryAs(out Edge edge)</c>.
/// </summary>
/// <typeparam name="TSelf">The shape type itself.</typeparam>
public interface IShape<TSelf> : IShape where TSelf : struct, IShape<TSelf>
{
    /// <summary>The shape type this type holds: <see cref="ShapeType.Shape"/> for <see cref="Shape"/>, which holds any.</summary>
    static abstract ShapeType Type { get; }
}
