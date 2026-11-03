// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using Kerf.Engine.Topology;
using Kerf.Topology;

namespace Kerf;

/// <summary>
/// A <see cref="Diagnostic"/>'s sub-shapes. <see cref="Diagnostic"/> lives in Kerf.Kernel.Foundation, which can't name
/// shapes, so they are read and set here, as extension members: <c>diagnostic.Shapes</c>,
/// <c>diagnostic.WithShapes(face)</c>.
/// </summary>
public static class DiagnosticShapes
{
    extension(Diagnostic diagnostic)
    {
        /// <summary>The sub-shapes the diagnostic concerns: empty if it names none.</summary>
        public ImmutableArray<Shape> Shapes => diagnostic.ShapeData is ImmutableArray<Shape> shapes ? shapes : [];

        /// <summary>The diagnostic, concerning <paramref name="shapes"/> in place of any it named.</summary>
        public Diagnostic WithShapes(params ReadOnlySpan<Shape> shapes) =>
            diagnostic with { ShapeData = shapes.IsEmpty ? null : ImmutableArray.Create(shapes) };

        /// <summary>The diagnostic, concerning <paramref name="shapes"/> in place of any it named.</summary>
        public Diagnostic WithShapes(IEnumerable<Shape> shapes)
        {
            ArgumentNullException.ThrowIfNull(shapes);
            var array = shapes.ToImmutableArray();
            return diagnostic with { ShapeData = array.IsEmpty ? null : array };
        }
    }
}

/// <summary>
/// A shape an operation built, with the history of how its sub-shapes came from the inputs' (decision 1332). It converts
/// to the shape, so the common case stays short: <c>Solid part = BooleanOperations.Cut(block, hole).Value;</c>.
/// </summary>
/// <typeparam name="TShape">The shape's type: <see cref="Shape"/> or one of the eight typed shapes.</typeparam>
/// <param name="Shape">The shape built.</param>
/// <param name="History">What the operation did to its inputs' sub-shapes.</param>
public sealed record Built<TShape>(TShape Shape, ShapeHistory History) where TShape : struct, IShape<TShape>
{
    /// <summary>The history; never null.</summary>
    public ShapeHistory History
    {
        get;
        init => field = value ?? throw new ArgumentNullException(nameof(value));
    } = History ?? throw new ArgumentNullException(nameof(History));

    /// <summary>The shape built.</summary>
    public static implicit operator TShape(Built<TShape> built)
    {
        ArgumentNullException.ThrowIfNull(built);
        return built.Shape;
    }
}
