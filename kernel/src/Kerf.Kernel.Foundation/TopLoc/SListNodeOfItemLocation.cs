// Port of: src/FoundationClasses/TKMath/TopLoc/TopLoc_SListNodeOfItemLocation.hxx
// Port of: src/FoundationClasses/TKMath/TopLoc/TopLoc_SListNodeOfItemLocation.lxx
// Port of: src/FoundationClasses/TKMath/TopLoc/TopLoc_SListNodeOfItemLocation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Occt;

namespace Kerf.Engine.Topology;

/// <summary>A node of a <see cref="SListOfItemLocation"/>: a value and the rest of the list.</summary>
/// <remarks>
/// Nodes are shared between lists. OCCT hands out mutable references to the value and the tail from const
/// methods; the port keeps them internal, so a node is immutable outside OCCT.NET.
/// </remarks>
internal class SListNodeOfItemLocation : Transient
{
    private SListOfItemLocation _tail;
    private ItemLocation _value;

    internal SListNodeOfItemLocation(in ItemLocation i, in SListOfItemLocation t)
    {
        _tail = t;
        _value = i;
    }

    internal ref SListOfItemLocation Tail() => ref _tail;

    internal ref ItemLocation Value() => ref _value;
}
