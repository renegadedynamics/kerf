// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Concurrent;
using System.Collections.Immutable;
using Kerf.Engine.Collections;
using Kerf.Engine.Topology;
using Occt;

namespace Kerf.Topology;

/// <summary>
/// A shape's sub-shapes, indexed, with their ancestors: built once per shape, then queried (OCCT's
/// <c>TopExp::MapShapes</c> and <c>MapShapesAndUniqueAncestors</c>). Sub-shapes are compared with <see cref="Shape.IsSame"/>.
/// </summary>
/// <remarks>Safe to use from several threads: the ancestor tables are built on first use.</remarks>
public sealed class ShapeGraph
{
    private readonly Shape _root;
    private readonly IndexedSet<Shape> _all;
    private readonly ConcurrentDictionary<(ShapeType, ShapeType), IndexedMap<Shape, NCollection_List<Shape>>> _ancestors = new();

    private ShapeGraph(Shape root)
    {
        _root = root;
        _all = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(root, _all);
    }

    /// <summary>The graph of <paramref name="root"/> and its sub-shapes.</summary>
    /// <exception cref="ArgumentException"><paramref name="root"/> is null.</exception>
    public static ShapeGraph Build(Shape root)
        => root.IsNull ? throw new ArgumentException("The shape is null.", nameof(root)) : new ShapeGraph(root);

    /// <summary>The root shape.</summary>
    public Shape Root => _root;

    /// <summary>How many distinct sub-shapes the root has, itself included.</summary>
    public int Count => _all.Count;

    /// <summary>The index of <paramref name="shape"/> among the root's sub-shapes (0-based, the root first), or -1.</summary>
    public int IndexOf(Shape shape) => _all.IndexOf(shape);

    /// <summary>The distinct sub-shapes of <paramref name="type"/>, in index order.</summary>
    public ImmutableArray<Shape> All(ShapeType type)
    {
        var result = ImmutableArray.CreateBuilder<Shape>();
        foreach (Shape shape in _all)
        {
            if (type == ShapeType.Shape || shape.Type == type)
            {
                result.Add(shape);
            }
        }

        return result.ToImmutable();
    }

    /// <summary>
    /// The sub-shapes of type <paramref name="ancestorType"/> that contain <paramref name="shape"/> (the faces of an
    /// edge), each once; empty if it has none or isn't in the graph.
    /// </summary>
    /// <exception cref="ArgumentException"><paramref name="ancestorType"/> isn't a type that contains
    /// <paramref name="shape"/>'s.</exception>
    public ImmutableArray<Shape> Ancestors(Shape shape, ShapeType ancestorType)
    {
        if (shape.IsNull)
        {
            return [];
        }

        ShapeType type = shape.Type;
        if (ancestorType >= type || ancestorType == ShapeType.Shape)
        {
            throw new ArgumentException($"A {type} has no ancestors of type {ancestorType}.", nameof(ancestorType));
        }

        IndexedMap<Shape, NCollection_List<Shape>> map = _ancestors.GetOrAdd((type, ancestorType), static (key, root) =>
        {
            var built = new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
            Exploration.MapShapesAndUniqueAncestors(root, key.Item1, key.Item2, built);
            return built;
        }, _root);
        return map.TryGetValue(shape, out NCollection_List<Shape>? ancestors) && ancestors is not null ? [.. ancestors] : [];
    }
}
