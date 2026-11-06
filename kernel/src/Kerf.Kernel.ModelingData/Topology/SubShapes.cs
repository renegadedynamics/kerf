// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections;
using Kerf.Engine.Topology;

namespace Kerf.Topology;

/// <summary>
/// The sub-shapes of a shape, enumerated lazily: a shape's <c>Children</c> (OCCT's <c>TopoDS_Iterator</c>) or what
/// <c>Explore&lt;T&gt;()</c> finds (OCCT's <c>TopExp_Explorer</c>). <c>foreach</c> allocates only the engine's iterator.
/// </summary>
/// <remarks>
/// Explored sub-shapes come in OCCT's order, a sub-shape shared by several parents once for each (use
/// <c>Faces()</c> and the like for each one once). Placements and orientations are composed with the parents'.
/// </remarks>
/// <typeparam name="TShape">The shape type enumerated.</typeparam>
public readonly struct SubShapes<TShape> : IEnumerable<TShape> where TShape : struct, IShape<TShape>
{
    private readonly Shape _root;
    private readonly ShapeType _toFind;
    private readonly ShapeType _toAvoid;
    private readonly bool _children;

    internal SubShapes(Shape root, ShapeType toFind, ShapeType toAvoid, bool children)
    {
        _root = root;
        _toFind = toFind;
        _toAvoid = toAvoid;
        _children = children;
    }

    /// <summary>The enumerator.</summary>
    public Enumerator GetEnumerator() => new(_root, _toFind, _toAvoid, _children);

    IEnumerator<TShape> IEnumerable<TShape>.GetEnumerator() => GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();

    // An enumerator moves: it is mutable by contract, as List<T>.Enumerator is (KERF0003 is about values)
#pragma warning disable KERF0003

    /// <summary>Enumerates the sub-shapes.</summary>
    public struct Enumerator : IEnumerator<TShape>
    {
        // Values, moved in place: one of the two is in use, or neither for a null shape
        private Explorer _explorer;
        private Engine.Topology.Iterator _iterator;
        private readonly bool _usesExplorer;
        private bool _started;
        private TShape _current;

        internal Enumerator(Shape root, ShapeType toFind, ShapeType toAvoid, bool children)
        {
            if (root.IsNull)
            {
                return;
            }

            if (children)
            {
                _iterator = new Engine.Topology.Iterator(root);
            }
            else
            {
                _explorer = new Explorer(root, toFind, toAvoid);
                _usesExplorer = true;
            }
        }

        /// <inheritdoc/>
        public readonly TShape Current => _current;

        readonly object IEnumerator.Current => _current;

        /// <inheritdoc/>
        public bool MoveNext()
        {
            if (_usesExplorer)
            {
                if (_started && _explorer.More())
                {
                    _explorer.Next();
                }

                _started = true;
                if (_explorer.More())
                {
                    _current = ShapeConversions.Narrow<TShape>(_explorer.Current());
                    return true;
                }
            }
            else
            {
                if (_started && _iterator.More())
                {
                    _iterator.Next();
                }

                _started = true;
                if (_iterator.More())
                {
                    _current = ShapeConversions.Narrow<TShape>(_iterator.Value());
                    return true;
                }
            }

            _current = default;
            return false;
        }

        /// <summary>Not supported: enumerate again from the <see cref="SubShapes{TShape}"/>.</summary>
        public readonly void Reset() => throw new NotSupportedException();

        /// <inheritdoc/>
        public readonly void Dispose()
        {
        }
    }
#pragma warning restore KERF0003
}
