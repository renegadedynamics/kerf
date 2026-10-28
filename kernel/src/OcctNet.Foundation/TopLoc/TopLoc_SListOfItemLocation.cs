// Port of: src/FoundationClasses/TKMath/TopLoc/TopLoc_SListOfItemLocation.hxx
// Port of: src/FoundationClasses/TKMath/TopLoc/TopLoc_SListOfItemLocation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.CompilerServices;

namespace Occt;

/// <summary>
/// A LISP-like list of <see cref="TopLoc_ItemLocation"/>s: empty, or a value and a tail which is another list.
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
public partial struct TopLoc_SListOfItemLocation : IEquatable<TopLoc_SListOfItemLocation>
{
    // Internal: TopLoc_Location::IsEqual compares the node pointers through a cast in OCCT.
    internal TopLoc_SListNodeOfItemLocation? myNode;

    /// <summary>The empty list.</summary>
    public TopLoc_SListOfItemLocation() => myNode = null;

    /// <summary>The list with anItem as value and aTail as tail.</summary>
    public TopLoc_SListOfItemLocation(in TopLoc_ItemLocation anItem, in TopLoc_SListOfItemLocation aTail)
    {
        myNode = new TopLoc_SListNodeOfItemLocation(anItem, aTail);
        if (!myNode.Tail().IsEmpty())
        {
            gp_Trsf aT = myNode.Tail().Value().myTrsf;
            myNode.Value().myTrsf.PreMultiply(aT);
        }
    }

    /// <summary>Makes this list share Other's nodes.</summary>
    /// <remarks>OCCT returns the list itself; the port returns nothing.</remarks>
    internal void Assign(in TopLoc_SListOfItemLocation Other)
    {
        // OCCT's self-assignment test (this == &Other) has no effect here: the assignment below is then a no-op.
        Clear();
        myNode = Other.myNode;
    }

    /// <summary>True if this list is empty.</summary>
    public readonly bool IsEmpty() => myNode is null;

    /// <summary>Makes the list empty.</summary>
    internal void Clear() => myNode = null;

    /// <summary>The first value of the list.</summary>
    /// <exception cref="Standard_NoSuchObject">The list is empty.</exception>
    public readonly TopLoc_ItemLocation Value()
    {
        Standard_NoSuchObject_Raise_if(myNode is null, "TopLoc_SListOfItemLocation::Value");
        return myNode!.Value();
    }

    /// <summary>The tail of the list; the tail of an empty list is the list itself.</summary>
    public readonly TopLoc_SListOfItemLocation Tail()
    {
        if (myNode is not null)
        {
            return myNode.Tail();
        }
        else
        {
            return this;
        }
    }

    /// <summary>Replaces the list by a list with anItem as value and this list as tail.</summary>
    internal void Construct(in TopLoc_ItemLocation anItem) => Assign(new TopLoc_SListOfItemLocation(anItem, this));

    /// <summary>Replaces the list by its tail.</summary>
    internal void ToTail() => Assign(Tail());

    /// <summary>True if the iterator has a current value (the list is not empty).</summary>
    public readonly bool More() => !IsEmpty();

    /// <summary>Moves the iterator to the next value; an empty list stays empty.</summary>
    internal void Next() => ToTail();

    // Value semantics: lists are equal when they share their first node, as OCCT compares the handles.

    public readonly bool Equals(TopLoc_SListOfItemLocation other) => ReferenceEquals(myNode, other.myNode);

    public override readonly bool Equals(object? obj) => obj is TopLoc_SListOfItemLocation other && Equals(other);

    public override readonly int GetHashCode() => myNode is null ? 0 : RuntimeHelpers.GetHashCode(myNode);

    public static bool operator ==(in TopLoc_SListOfItemLocation theLeft, in TopLoc_SListOfItemLocation theRight)
        => theLeft.Equals(theRight);

    public static bool operator !=(in TopLoc_SListOfItemLocation theLeft, in TopLoc_SListOfItemLocation theRight)
        => !theLeft.Equals(theRight);

    public override readonly string ToString()
    {
        int aLength = 0;
        for (TopLoc_SListOfItemLocation anIter = this; anIter.More(); anIter.Next())
        {
            aLength++;
        }

        return $"TopLoc_SListOfItemLocation({aLength} items)";
    }
}
