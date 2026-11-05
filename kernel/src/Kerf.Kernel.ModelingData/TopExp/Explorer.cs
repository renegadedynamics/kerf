// Port of: src/ModelingData/TKBRep/TopExp/TopExp_Explorer.hxx
// Port of: src/ModelingData/TKBRep/TopExp/TopExp_Explorer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Topology.ShapeType;

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
/// A type to avoid that is the same as, or simpler than, the type to find has no effect. A class (a working object),
/// where OCCT's is used by value; its stack of iterators is reused across <see cref="Init"/>. OCCT's range-for support
/// is left out.
/// </remarks>
internal sealed class Explorer
{
    // The DFS stack: iterators are kept when popped and reused when pushed again
    private readonly List<Iterator> _stack = [];
    private int _stackTop = -1;
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
    public bool More() => _hasMore;

    /// <summary>Moves to the next shape.</summary>
    public void Next()
    {
        if (_stackTop < 0)
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
            _stack[_stackTop].Next();
        }

        for (;;)
        {
            Iterator topIter = _stack[_stackTop];
            if (topIter.More())
            {
                Shape shapTop = topIter.Value();
                ShapeType ty = shapTop.Type;
                if (IsSameType(ty, _toFind))
                {
                    _hasMore = true;
                    return;
                }
                else if (IsMoreComplex(ty, _toFind) && !ShouldAvoid(ty, _toAvoid))
                {
                    PushIterator(shapTop);
                }
                else
                {
                    topIter.Next();
                }
            }
            else
            {
                PopIterator();
                if (_stackTop < 0)
                {
                    break;
                }

                _stack[_stackTop].Next();
            }
        }

        _hasMore = false;
    }

    /// <summary>The current shape.</summary>
    public Shape Value() => Current();

    /// <summary>The current shape.</summary>
    public Shape Current() => _stackTop < 0 ? _shape : _stack[_stackTop].Value();

    /// <summary>Restarts the exploration with the same arguments.</summary>
    public void ReInit() => Init(_shape, _toFind, _toAvoid);

    /// <summary>The explored shape.</summary>
    public Shape ExploredShape() => _shape;

    /// <summary>The current depth of the exploration: 0 is the explored shape itself.</summary>
    public int Depth() => _stackTop + 1;

    /// <summary>Clears the explorer.</summary>
    public void Clear()
    {
        // Reset live iterators to release shape references, keeping them for reuse
        for (int i = _stackTop; i >= 0; --i)
        {
            _stack[i].Initialize(new Shape());
        }

        _stackTop = -1;
        _hasMore = false;
    }

    // Pushes an iterator on the shape onto the stack, reusing a popped one.
    private void PushIterator(in Shape shape)
    {
        ++_stackTop;
        if (_stackTop >= _stack.Count)
        {
            _stack.Add(new Iterator(shape));
        }
        else
        {
            _stack[_stackTop].Initialize(shape);
        }
    }

    // Pops the top iterator, resetting it to release its shape references.
    private void PopIterator()
    {
        _stack[_stackTop].Initialize(new Shape());
        --_stackTop;
    }

    // Returns true if the given type matches the type to find.
    private static bool IsSameType(ShapeType type, ShapeType toFind) => toFind == type;

    // Returns true if the given type should be avoided.
    private static bool ShouldAvoid(ShapeType type, ShapeType toAvoid)
        => toAvoid != ShapeType.Shape && toAvoid == type;

    // Returns true if the given type is more complex than the type to find.
    private static bool IsMoreComplex(ShapeType type, ShapeType toFind) => toFind > type;
}
