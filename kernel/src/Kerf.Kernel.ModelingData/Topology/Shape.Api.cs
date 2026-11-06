// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// Unit 10L1: the shapes' public surface (docs/api/topology.md): IShape<T>, the casts, and the conversions between the
// nine shape types.

namespace Kerf.Topology;

public partial struct Shape : IShape<Shape>
{
    static ShapeType IShape<Shape>.Type => ShapeType.Shape;

    /// <inheritdoc/>
    public readonly Shape AsShape() => this;

    /// <summary>
    /// The sub-shapes of type <typeparamref name="T"/>, depth first, in OCCT's order, each as often as it is reached;
    /// those inside a sub-shape of type <paramref name="avoid"/> are skipped (OCCT's <c>TopExp_Explorer</c>).
    /// </summary>
    /// <exception cref="ArgumentException"><typeparamref name="T"/> is <see cref="Shape"/>.</exception>
    public readonly SubShapes<T> Explore<T>(ShapeType? avoid = null) where T : struct, IShape<T> => ShapeTraversal.Explore<T>(this, avoid);

    /// <summary>This shape as a <typeparamref name="TShape"/>, if it is one (a null shape is any type).</summary>
    public readonly bool TryAs<TShape>(out TShape shape) where TShape : struct, IShape<TShape>
    {
        if (IsNull || TShape.Type == ShapeType.Shape || Type == TShape.Type)
        {
            shape = ShapeConversions.Narrow<TShape>(this);
            return true;
        }

        shape = default;
        return false;
    }

    /// <summary>This shape as a <typeparamref name="TShape"/>.</summary>
    /// <exception cref="InvalidCastException">The shape is of another type.</exception>
    public readonly TShape As<TShape>() where TShape : struct, IShape<TShape>
        => TryAs(out TShape shape) ? shape : throw new InvalidCastException($"The shape is a {Type}, not a {TShape.Type}.");
}

public partial struct Vertex : IShape<Vertex>
{
    static ShapeType IShape<Vertex>.Type => ShapeType.Vertex;

    /// <inheritdoc/>
    public readonly Shape AsShape() => this;

    /// <summary>
    /// The sub-shapes of type <typeparamref name="T"/>, depth first, in OCCT's order, each as often as it is reached;
    /// those inside a sub-shape of type <paramref name="avoid"/> are skipped (OCCT's <c>TopExp_Explorer</c>).
    /// </summary>
    /// <exception cref="ArgumentException"><typeparamref name="T"/> is <see cref="Shape"/>.</exception>
    public readonly SubShapes<T> Explore<T>(ShapeType? avoid = null) where T : struct, IShape<T> => ShapeTraversal.Explore<T>(this, avoid);

    /// <summary>The shape as a vertex.</summary>
    /// <exception cref="InvalidCastException">The shape isn't a vertex (or null).</exception>
    public static explicit operator Vertex(in Shape shape) => shape.As<Vertex>();
}

public partial struct Edge : IShape<Edge>, IShape1D
{
    static ShapeType IShape<Edge>.Type => ShapeType.Edge;

    /// <inheritdoc/>
    public readonly Shape AsShape() => this;

    /// <summary>
    /// The sub-shapes of type <typeparamref name="T"/>, depth first, in OCCT's order, each as often as it is reached;
    /// those inside a sub-shape of type <paramref name="avoid"/> are skipped (OCCT's <c>TopExp_Explorer</c>).
    /// </summary>
    /// <exception cref="ArgumentException"><typeparamref name="T"/> is <see cref="Shape"/>.</exception>
    public readonly SubShapes<T> Explore<T>(ShapeType? avoid = null) where T : struct, IShape<T> => ShapeTraversal.Explore<T>(this, avoid);

    /// <summary>The shape as an edge.</summary>
    /// <exception cref="InvalidCastException">The shape isn't an edge (or null).</exception>
    public static explicit operator Edge(in Shape shape) => shape.As<Edge>();
}

public partial struct Wire : IShape<Wire>, IShape1D
{
    static ShapeType IShape<Wire>.Type => ShapeType.Wire;

    /// <inheritdoc/>
    public readonly Shape AsShape() => this;

    /// <summary>
    /// The sub-shapes of type <typeparamref name="T"/>, depth first, in OCCT's order, each as often as it is reached;
    /// those inside a sub-shape of type <paramref name="avoid"/> are skipped (OCCT's <c>TopExp_Explorer</c>).
    /// </summary>
    /// <exception cref="ArgumentException"><typeparamref name="T"/> is <see cref="Shape"/>.</exception>
    public readonly SubShapes<T> Explore<T>(ShapeType? avoid = null) where T : struct, IShape<T> => ShapeTraversal.Explore<T>(this, avoid);

    /// <summary>The shape as a wire.</summary>
    /// <exception cref="InvalidCastException">The shape isn't a wire (or null).</exception>
    public static explicit operator Wire(in Shape shape) => shape.As<Wire>();
}

public partial struct Face : IShape<Face>, IShape2D
{
    static ShapeType IShape<Face>.Type => ShapeType.Face;

    /// <inheritdoc/>
    public readonly Shape AsShape() => this;

    /// <summary>
    /// The sub-shapes of type <typeparamref name="T"/>, depth first, in OCCT's order, each as often as it is reached;
    /// those inside a sub-shape of type <paramref name="avoid"/> are skipped (OCCT's <c>TopExp_Explorer</c>).
    /// </summary>
    /// <exception cref="ArgumentException"><typeparamref name="T"/> is <see cref="Shape"/>.</exception>
    public readonly SubShapes<T> Explore<T>(ShapeType? avoid = null) where T : struct, IShape<T> => ShapeTraversal.Explore<T>(this, avoid);

    /// <summary>The shape as a face.</summary>
    /// <exception cref="InvalidCastException">The shape isn't a face (or null).</exception>
    public static explicit operator Face(in Shape shape) => shape.As<Face>();
}

public partial struct Shell : IShape<Shell>, IShape2D
{
    static ShapeType IShape<Shell>.Type => ShapeType.Shell;

    /// <inheritdoc/>
    public readonly Shape AsShape() => this;

    /// <summary>
    /// The sub-shapes of type <typeparamref name="T"/>, depth first, in OCCT's order, each as often as it is reached;
    /// those inside a sub-shape of type <paramref name="avoid"/> are skipped (OCCT's <c>TopExp_Explorer</c>).
    /// </summary>
    /// <exception cref="ArgumentException"><typeparamref name="T"/> is <see cref="Shape"/>.</exception>
    public readonly SubShapes<T> Explore<T>(ShapeType? avoid = null) where T : struct, IShape<T> => ShapeTraversal.Explore<T>(this, avoid);

    /// <summary>The shape as a shell.</summary>
    /// <exception cref="InvalidCastException">The shape isn't a shell (or null).</exception>
    public static explicit operator Shell(in Shape shape) => shape.As<Shell>();
}

public partial struct Solid : IShape<Solid>, IShape3D
{
    static ShapeType IShape<Solid>.Type => ShapeType.Solid;

    /// <inheritdoc/>
    public readonly Shape AsShape() => this;

    /// <summary>
    /// The sub-shapes of type <typeparamref name="T"/>, depth first, in OCCT's order, each as often as it is reached;
    /// those inside a sub-shape of type <paramref name="avoid"/> are skipped (OCCT's <c>TopExp_Explorer</c>).
    /// </summary>
    /// <exception cref="ArgumentException"><typeparamref name="T"/> is <see cref="Shape"/>.</exception>
    public readonly SubShapes<T> Explore<T>(ShapeType? avoid = null) where T : struct, IShape<T> => ShapeTraversal.Explore<T>(this, avoid);

    /// <summary>The shape as a solid.</summary>
    /// <exception cref="InvalidCastException">The shape isn't a solid (or null).</exception>
    public static explicit operator Solid(in Shape shape) => shape.As<Solid>();
}

public partial struct CompSolid : IShape<CompSolid>, IShape3D
{
    static ShapeType IShape<CompSolid>.Type => ShapeType.CompSolid;

    /// <inheritdoc/>
    public readonly Shape AsShape() => this;

    /// <summary>
    /// The sub-shapes of type <typeparamref name="T"/>, depth first, in OCCT's order, each as often as it is reached;
    /// those inside a sub-shape of type <paramref name="avoid"/> are skipped (OCCT's <c>TopExp_Explorer</c>).
    /// </summary>
    /// <exception cref="ArgumentException"><typeparamref name="T"/> is <see cref="Shape"/>.</exception>
    public readonly SubShapes<T> Explore<T>(ShapeType? avoid = null) where T : struct, IShape<T> => ShapeTraversal.Explore<T>(this, avoid);

    /// <summary>The shape as a composite solid.</summary>
    /// <exception cref="InvalidCastException">The shape isn't a composite solid (or null).</exception>
    public static explicit operator CompSolid(in Shape shape) => shape.As<CompSolid>();
}

public partial struct Compound : IShape<Compound>, IShape3D
{
    static ShapeType IShape<Compound>.Type => ShapeType.Compound;

    /// <inheritdoc/>
    public readonly Shape AsShape() => this;

    /// <summary>
    /// The sub-shapes of type <typeparamref name="T"/>, depth first, in OCCT's order, each as often as it is reached;
    /// those inside a sub-shape of type <paramref name="avoid"/> are skipped (OCCT's <c>TopExp_Explorer</c>).
    /// </summary>
    /// <exception cref="ArgumentException"><typeparamref name="T"/> is <see cref="Shape"/>.</exception>
    public readonly SubShapes<T> Explore<T>(ShapeType? avoid = null) where T : struct, IShape<T> => ShapeTraversal.Explore<T>(this, avoid);

    /// <summary>The shape as a compound.</summary>
    /// <exception cref="InvalidCastException">The shape isn't a compound (or null).</exception>
    public static explicit operator Compound(in Shape shape) => shape.As<Compound>();
}

/// <summary>Conversions between the nine shape types without a check: the caller knows the type.</summary>
internal static class ShapeConversions
{
    public static TShape Narrow<TShape>(in Shape shape) where TShape : struct, IShape<TShape>
    {
        // The JIT keeps one branch per instantiation, and (TShape)(object) of a value type of the same type doesn't box
        if (typeof(TShape) == typeof(Shape))
        {
            return (TShape)(object)shape;
        }

        if (typeof(TShape) == typeof(Vertex))
        {
            return (TShape)(object)new Vertex(shape.TShape(), shape.Placement, shape.Orientation);
        }

        if (typeof(TShape) == typeof(Edge))
        {
            return (TShape)(object)new Edge(shape.TShape(), shape.Placement, shape.Orientation);
        }

        if (typeof(TShape) == typeof(Wire))
        {
            return (TShape)(object)new Wire(shape.TShape(), shape.Placement, shape.Orientation);
        }

        if (typeof(TShape) == typeof(Face))
        {
            return (TShape)(object)new Face(shape.TShape(), shape.Placement, shape.Orientation);
        }

        if (typeof(TShape) == typeof(Shell))
        {
            return (TShape)(object)new Shell(shape.TShape(), shape.Placement, shape.Orientation);
        }

        if (typeof(TShape) == typeof(Solid))
        {
            return (TShape)(object)new Solid(shape.TShape(), shape.Placement, shape.Orientation);
        }

        if (typeof(TShape) == typeof(CompSolid))
        {
            return (TShape)(object)new CompSolid(shape.TShape(), shape.Placement, shape.Orientation);
        }

        if (typeof(TShape) == typeof(Compound))
        {
            return (TShape)(object)new Compound(shape.TShape(), shape.Placement, shape.Orientation);
        }

        throw new InvalidOperationException($"{typeof(TShape)} is not one of the nine shape types.");
    }
}
