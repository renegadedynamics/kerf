// Port of: src/FoundationClasses/TKMath/TopLoc/TopLoc_ItemLocation.hxx
// Port of: src/FoundationClasses/TKMath/TopLoc/TopLoc_ItemLocation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using System.Runtime.CompilerServices;

namespace Kerf.Engine.Topology;

/// <summary>
/// An elementary coordinate system in a <see cref="Placement"/>: a datum, the power it is raised to, and
/// the transformation of the composition (this item and the ones after it in the list).
/// </summary>
/// <remarks>
/// Its fields are read by TopLoc_Location and TopLoc_SListOfItemLocation, OCCT's friends of this class.
/// <c>default(TopLoc_ItemLocation)</c> has no datum; OCCT has no default constructor.
/// </remarks>
internal partial struct ItemLocation : IEquatable<ItemLocation>
{
    internal Datum3D _datum;
    internal int _power;
    internal Transform3D _trsf;

    /// <summary>The datum D raised to the power P.</summary>
    public ItemLocation(Datum3D d, int p)
    {
        _datum = d;
        _power = p;
        _trsf = d.Transformation().Powered(p);
    }

    // Value semantics: the datum is compared by identity, as OCCT compares handles.

    public readonly bool Equals(ItemLocation other)
        => ReferenceEquals(_datum, other._datum) && _power == other._power && _trsf.Equals(other._trsf);

    public override readonly bool Equals(object? obj) => obj is ItemLocation other && Equals(other);

    public override readonly int GetHashCode()
        => HashCode.Combine(_datum is null ? 0 : RuntimeHelpers.GetHashCode(_datum), _power, _trsf);

    public static bool operator ==(in ItemLocation left, in ItemLocation right) => left.Equals(right);

    public static bool operator !=(in ItemLocation left, in ItemLocation right) => !left.Equals(right);

    public override readonly string ToString() => $"TopLoc_ItemLocation(power {_power}, {_trsf})";
}
