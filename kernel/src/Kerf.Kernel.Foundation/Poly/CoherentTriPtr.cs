// Port of: src/FoundationClasses/TKMath/Poly/Poly_CoherentTriPtr.hxx
// Port of: src/FoundationClasses/TKMath/Poly/Poly_CoherentTriPtr.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2007-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Meshing;

/// <summary>
/// A node of a circular, doubly linked list of triangles (<see cref="CoherentTriangle"/>), and the list itself:
/// the triangles around a <see cref="CoherentNode"/>.
/// </summary>
/// <remarks>OCCT's allocator arguments are dropped; removing a node unlinks it and leaves it to the garbage collector.</remarks>
public class CoherentTriPtr
{
    /// <summary>
    /// Iterates a list from any of its members, forward, until it comes back to that member.
    /// </summary>
    public sealed class Iterator
    {
        private readonly CoherentTriPtr? _first;
        private CoherentTriPtr? _current;

        /// <summary>An empty iterator.</summary>
        public Iterator()
        {
            _first = null;
            _current = null;
        }

        /// <summary>
        /// An iterator from <paramref name="ptr"/>; null gives an empty iteration (OCCT dereferences the null list
        /// of a free node with the same effect).
        /// </summary>
        public Iterator(CoherentTriPtr? ptr)
        {
            _first = ptr;
            _current = ptr;
        }

        /// <summary>The triangle that started the iteration, or null.</summary>
        public CoherentTriangle? First() => _first?.GetTriangle();

        /// <summary>True if there is a current triangle.</summary>
        public bool More() => _current is not null;

        /// <summary>Moves to the next triangle.</summary>
        public void Next()
        {
            if (_current is not null)
            {
                _current = _current._next;
                if (_current == _first)
                {
                    _current = null;
                }
            }
        }

        /// <summary>The current triangle.</summary>
        public CoherentTriangle Value() => _current!.GetTriangle();

        /// <summary>The current triangle, for writing.</summary>
        public CoherentTriangle ChangeValue() => _current!.GetTriangle();

        /// <summary>The current list node.</summary>
        public CoherentTriPtr PtrValue() => _current!;
    }

    private CoherentTriangle _mypTriangle;
    private CoherentTriPtr _next;
    private CoherentTriPtr _previous;

    /// <summary>A list of one triangle.</summary>
    public CoherentTriPtr(CoherentTriangle tri)
    {
        _mypTriangle = tri;
        _next = this;
        _previous = this;
    }

    /// <summary>The triangle.</summary>
    public CoherentTriangle GetTriangle() => _mypTriangle;

    /// <summary>Sets the triangle.</summary>
    public void SetTriangle(CoherentTriangle pTri) => _mypTriangle = pTri;

    /// <summary>The next node of the list.</summary>
    public CoherentTriPtr Next() => _next;

    /// <summary>The previous node of the list.</summary>
    public CoherentTriPtr Previous() => _previous;

    /// <summary>Inserts a triangle after this node.</summary>
    public void Append(CoherentTriangle pTri)
    {
        var newPtr = new CoherentTriPtr(pTri);
        newPtr._next = _next;
        _next._previous = newPtr;
        newPtr._previous = this;
        _next = newPtr;
    }

    /// <summary>Inserts a triangle before this node.</summary>
    public void Prepend(CoherentTriangle pTri)
    {
        var newPtr = new CoherentTriPtr(pTri);
        newPtr._previous = _previous;
        _previous._next = newPtr;
        newPtr._next = this;
        _previous = newPtr;
    }

    /// <summary>Removes a node from its list.</summary>
    public static void Remove(CoherentTriPtr ptr)
    {
        ptr._previous._next = ptr._next;
        ptr._next._previous = ptr._previous;
        ptr._next = ptr;
        ptr._previous = ptr;
        // OCCT frees the node here
    }

    /// <summary>Releases a whole list (OCCT frees its nodes; here the garbage collector does).</summary>
    public static void RemoveList(CoherentTriPtr? ptr)
    {
        _ = ptr;
    }
}
