// Port of: src/FoundationClasses/TKMath/TopLoc/TopLoc_SListOfItemLocation.hxx
// Port of: src/FoundationClasses/TKMath/TopLoc/TopLoc_SListOfItemLocation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Occt;
using System.Runtime.CompilerServices;

namespace Kerf.Engine.Topology;

/// <summary>
/// A LISP-like list of <see cref="ItemLocation"/>s: empty, or a value and a tail which is another list.
/// The tail of an empty list is an empty list. Lists share their nodes.
/// </summary>
/// <remarks>
/// <para>
/// A list is also an iterator: <c>for (TopLoc_SListOfItemLocation it = S; it.More(); it.Next()) X = it.Value();</c>
/// (on a local copy, since <c>Next</c> is internal).
/// </para>
/// <para>
/// Each node's transformation is the product of its own and its tail's, computed once when the node is built.
/// <c>default(TopLoc_SListOfItemLocation)</c> is the empty list, as OCCT's default constructor. The internal mutators
/// (<c>Construct</c>, <c>ToTail</c>, <c>Next</c>, <c>Clear</c>, <c>Assign</c>) only replace the head reference; their
/// non-mutating twins are the constructor, <see cref="Tail"/> and <c>new TopLoc_SListOfItemLocation()</c>.
/// </para>
/// </remarks>
public partial struct SListOfItemLocation : IEquatable<SListOfItemLocation>
{
    // Internal: TopLoc_Location::IsEqual compares the node pointers through a cast in OCCT.
    internal SListNodeOfItemLocation? _node;

    /// <summary>The empty list.</summary>
    public SListOfItemLocation() => _node = null;

    /// <summary>The list with anItem as value and aTail as tail.</summary>
    public SListOfItemLocation(in ItemLocation anItem, in SListOfItemLocation aTail)
    {
        _node = new SListNodeOfItemLocation(anItem, aTail);
        if (!_node.Tail().IsEmpty)
        {
            Transform3D t = _node.Tail().Value()._trsf;
            _node.Value()._trsf.PreMultiply(t);
        }
    }

    /// <summary>Makes this list share Other's nodes.</summary>
    /// <remarks>OCCT returns the list itself; the port returns nothing.</remarks>
    internal void Assign(in SListOfItemLocation other)
    {
        // OCCT's self-assignment test (this == &Other) has no effect here: the assignment below is then a no-op.
        Clear();
        _node = other._node;
    }

    /// <summary>True if this list is empty.</summary>
    public readonly bool IsEmpty => _node is null;

    /// <summary>Makes the list empty.</summary>
    internal void Clear() => _node = null;

    /// <summary>The first value of the list.</summary>
    /// <exception cref="NoSuchObjectException">The list is empty.</exception>
    public readonly ItemLocation Value()
    {
        Standard_NoSuchObject_Raise_if(_node is null, "TopLoc_SListOfItemLocation::Value");
        return _node!.Value();
    }

    /// <summary>The tail of the list; the tail of an empty list is the list itself.</summary>
    public readonly SListOfItemLocation Tail()
    {
        if (_node is not null)
        {
            return _node.Tail();
        }
        else
        {
            return this;
        }
    }

    /// <summary>Replaces the list by a list with anItem as value and this list as tail.</summary>
    internal void Construct(in ItemLocation anItem) => Assign(new SListOfItemLocation(anItem, this));

    /// <summary>Replaces the list by its tail.</summary>
    internal void ToTail() => Assign(Tail());

    /// <summary>True if the iterator has a current value (the list is not empty).</summary>
    public readonly bool More() => !IsEmpty;

    /// <summary>Moves the iterator to the next value; an empty list stays empty.</summary>
    internal void Next() => ToTail();

    // Value semantics: lists are equal when they share their first node, as OCCT compares the handles.

    public readonly bool Equals(SListOfItemLocation other) => ReferenceEquals(_node, other._node);

    public override readonly bool Equals(object? obj) => obj is SListOfItemLocation other && Equals(other);

    public override readonly int GetHashCode() => _node is null ? 0 : RuntimeHelpers.GetHashCode(_node);

    public static bool operator ==(in SListOfItemLocation left, in SListOfItemLocation right)
        => left.Equals(right);

    public static bool operator !=(in SListOfItemLocation left, in SListOfItemLocation right)
        => !left.Equals(right);

    public override readonly string ToString()
    {
        int length = 0;
        for (SListOfItemLocation iter = this; iter.More(); iter.Next())
        {
            length++;
        }

        return $"TopLoc_SListOfItemLocation({length} items)";
    }
}
