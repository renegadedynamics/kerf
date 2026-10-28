// Port of: src/FoundationClasses/TKMath/TopLoc/TopLoc_SListNodeOfItemLocation.hxx
// Port of: src/FoundationClasses/TKMath/TopLoc/TopLoc_SListNodeOfItemLocation.lxx
// Port of: src/FoundationClasses/TKMath/TopLoc/TopLoc_SListNodeOfItemLocation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A node of a <see cref="TopLoc_SListOfItemLocation"/>: a value and the rest of the list.</summary>
/// <remarks>
/// Nodes are shared between lists. OCCT hands out mutable references to the value and the tail from const
/// methods; the port keeps them internal, so a node is immutable outside OCCT.NET.
/// </remarks>
public class TopLoc_SListNodeOfItemLocation : Standard_Transient
{
    private TopLoc_SListOfItemLocation myTail;
    private TopLoc_ItemLocation myValue;

    internal TopLoc_SListNodeOfItemLocation(in TopLoc_ItemLocation I, in TopLoc_SListOfItemLocation T)
    {
        myTail = T;
        myValue = I;
    }

    internal ref TopLoc_SListOfItemLocation Tail() => ref myTail;

    internal ref TopLoc_ItemLocation Value() => ref myValue;
}
