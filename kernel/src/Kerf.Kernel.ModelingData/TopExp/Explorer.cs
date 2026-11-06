// Port of: src/ModelingData/TKBRep/TopExp/TopExp_Explorer.hxx
// Port of: src/ModelingData/TKBRep/TopExp/TopExp_Explorer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;

namespace Kerf.Engine.Topology;

/// <summary>
/// Visits a shape's structure depth first, finding the sub-shapes of a type (<c>ToFind</c>) that aren't inside a
/// sub-shape of another type (<c>ToAvoid</c>, SHAPE for no restriction).
/// </summary>
/// <remarks>
/// <code>
/// for (var Ex = new TopExp_Explorer(S, TopAbs_FACE); Ex.More(); Ex.Next())
/// {
///     ProcessFace(TopoDS.Face(Ex.Current()));
/// }
/// </code>
/// A type to avoid that is the same as, or simpler than, the type to find has no effect. A value on the stack, with its
/// levels inline, as OCCT's is, so an exploration allocates nothing unless the shape is nested very deeply. A copy moves on
/// its own, so hold one in a variable or a field that isn't <c>readonly</c>, and pass it by <c>ref</c>. OCCT's range-for
/// support is left out.
/// </remarks>
internal struct Explorer
{
    // Solid, shell, face, wire, edge and one compound: deeper compounds go on to _deeperLevels
    private const int InlineLevels = 6;

    // One level of the depth-first walk: what an iterator holds, but for its current shape, which only the top needs
    private struct Level
    {
        public NCollection_List<Shape> Children;
        public int Position;
        public Orientation Orientation;
        public Placement Location;
    }

    [InlineArray(InlineLevels)]
    private struct Levels
    {
        private Level _first;
    }

    private Levels _levels;
    private Level[]? _deeperLevels;
    private int _depth;
    // The top level's current child
    private Shape _current;
    private Shape _shape;
    private ShapeType _toFind;
    private ShapeType _toAvoid;
    private bool _hasMore;

    /// <summary>An empty explorer, useful after <see cref="Init"/>.</summary>
    public Explorer()
    {
        _shape = new Shape();
        _toFind = ShapeType.Shape;
        _toAvoid = ShapeType.Shape;
        _hasMore = false;
    }

    /// <summary>An explorer of <paramref name="s"/> for the sub-shapes of type ToFind not inside a ToAvoid.</summary>
    /// <param name="s">The shape to explore.</param>
    /// <param name="toFind">The type of the shapes to find: VERTEX, EDGE, ...</param>
    /// <param name="toAvoid">The type of the shapes to skip; no effect if SHAPE, or if the same as or simpler than ToFind.</param>
    public Explorer(in Shape s, ShapeType toFind, ShapeType toAvoid = ShapeType.Shape)
    {
        _shape = new Shape();
        _toFind = toFind;
        _toAvoid = toAvoid;
        _hasMore = false;
        Init(s, toFind, toAvoid);
    }

    /// <summary>Restarts the exploration on <paramref name="s"/>.</summary>
    /// <param name="s">The shape to explore.</param>
    /// <param name="toFind">The type of the shapes to find: VERTEX, EDGE, ...</param>
    /// <param name="toAvoid">The type of the shapes to skip; no effect if SHAPE, or if the same as or simpler than ToFind.</param>
    public void Init(in Shape s, ShapeType toFind, ShapeType toAvoid = ShapeType.Shape)
    {
        Clear();

        _shape = s;
        _toFind = toFind;
        _toAvoid = toAvoid;

        if (s.IsNull)
        {
            _hasMore = false;
            return;
        }

        if (_toFind == ShapeType.Shape)
        {
            _hasMore = false;
        }
        else
        {
            ShapeType ty = s.Type;

            if (ty > _toFind)
            {
                _hasMore = false;
            }
            else if (!IsSameType(ty, _toFind))
            {
                _hasMore = true;
                Next();
            }
            else
            {
                _hasMore = true;
            }
        }
    }

    /// <summary>True if there is a current shape.</summary>
    public readonly bool More() => _hasMore;

    /// <summary>Moves to the next shape.</summary>
    public void Next()
    {
        if (_depth == 0)
        {
            ShapeType ty = _shape.Type;
            if (IsSameType(ty, _toFind))
            {
                _hasMore = false;
                return;
            }
            else if (ShouldAvoid(ty, _toAvoid))
            {
                _hasMore = false;
                return;
            }
            else
            {
                PushIterator(_shape);
            }
        }
        else
        {
            AdvanceTop();
        }

        for (;;)
        {
            if (TopHasMore())
            {
                ShapeType ty = _current.Type;
                if (IsSameType(ty, _toFind))
                {
                    _hasMore = true;
                    return;
                }
                else if (IsMoreComplex(ty, _toFind) && !ShouldAvoid(ty, _toAvoid))
                {
                    PushIterator(_current);
                }
                else
                {
                    AdvanceTop();
                }
            }
            else
            {
                PopIterator();
                if (_depth == 0)
                {
                    break;
                }

                AdvanceTop();
            }
        }

        _hasMore = false;
    }

    /// <summary>The current shape.</summary>
    public readonly Shape Value() => Current();

    /// <summary>The current shape.</summary>
    public readonly Shape Current() => _depth == 0 ? _shape : _current;

    /// <summary>Restarts the exploration with the same arguments.</summary>
    public void ReInit() => Init(_shape, _toFind, _toAvoid);

    /// <summary>The explored shape.</summary>
    public readonly Shape ExploredShape() => _shape;

    /// <summary>The current depth of the exploration: 0 is the explored shape itself.</summary>
    public readonly int Depth() => _depth;

    /// <summary>Clears the explorer.</summary>
    public void Clear()
    {
        // Release the levels' references, in case this explorer is a field that outlives its shape
        while (_depth > 0)
        {
            PopIterator();
        }

        _hasMore = false;
    }

    [UnscopedRef]
    private ref Level LevelAt(int index)
    {
        if (index < InlineLevels)
        {
            return ref _levels[index];
        }

        return ref _deeperLevels![index - InlineLevels];
    }

    private bool TopHasMore()
    {
        ref Level top = ref LevelAt(_depth - 1);
        return top.Position < top.Children.Extent();
    }

    // Moves the top level on, and its current child with it
    private void AdvanceTop()
    {
        ref Level top = ref LevelAt(_depth - 1);
        top.Position++;
        if (top.Position < top.Children.Extent())
        {
            _current = Iterator.Child(top.Children, top.Position, top.Orientation, top.Location);
        }
    }

    // Starts a level on the shape's children.
    private void PushIterator(in Shape shape)
    {
        // The argument can be _current, which the first child overwrites
        Placement location = shape.Placement;
        Orientation orientation = shape.Orientation;
        NCollection_List<Shape> children = WorkingOverlay.View(shape.TShape()!)._shapes;
        if (_depth >= InlineLevels)
        {
            int deeper = _depth - InlineLevels;
            if (_deeperLevels is null)
            {
                _deeperLevels = new Level[InlineLevels];
            }
            else if (deeper >= _deeperLevels.Length)
            {
                Array.Resize(ref _deeperLevels, 2 * _deeperLevels.Length);
            }
        }

        ++_depth;
        ref Level top = ref LevelAt(_depth - 1);
        top.Children = children;
        top.Position = 0;
        top.Orientation = orientation;
        top.Location = location;
        if (children.Extent() > 0)
        {
            _current = Iterator.Child(children, 0, orientation, location);
        }
    }

    // Ends the top level, releasing its references.
    private void PopIterator()
    {
        LevelAt(_depth - 1) = default;
        --_depth;
    }

    // Returns true if the given type matches the type to find.
    private static bool IsSameType(ShapeType type, ShapeType toFind) => toFind == type;

    // Returns true if the given type should be avoided.
    private static bool ShouldAvoid(ShapeType type, ShapeType toAvoid)
        => toAvoid != ShapeType.Shape && toAvoid == type;

    // Returns true if the given type is more complex than the type to find.
    private static bool IsMoreComplex(ShapeType type, ShapeType toFind) => toFind > type;
}
