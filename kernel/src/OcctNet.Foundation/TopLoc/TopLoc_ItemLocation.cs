// Port of: src/FoundationClasses/TKMath/TopLoc/TopLoc_ItemLocation.hxx
// Port of: src/FoundationClasses/TKMath/TopLoc/TopLoc_ItemLocation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.CompilerServices;

namespace Occt;

/// <summary>
/// An elementary coordinate system in a <see cref="TopLoc_Location"/>: a datum, the power it is raised to, and
/// the transformation of the composition (this item and the ones after it in the list).
/// </summary>
/// <remarks>
/// Its fields are read by TopLoc_Location and TopLoc_SListOfItemLocation, OCCT's friends of this class.
/// <c>default(TopLoc_ItemLocation)</c> has no datum; OCCT has no default constructor.
/// </remarks>
public partial struct TopLoc_ItemLocation : IEquatable<TopLoc_ItemLocation>
{
    internal TopLoc_Datum3D myDatum;
    internal int myPower;
    internal gp_Trsf myTrsf;

    /// <summary>The datum D raised to the power P.</summary>
    public TopLoc_ItemLocation(TopLoc_Datum3D D, int P)
    {
        myDatum = D;
        myPower = P;
        myTrsf = D.Transformation().Powered(P);
    }

    // Value semantics: the datum is compared by identity, as OCCT compares handles.

    public readonly bool Equals(TopLoc_ItemLocation other)
        => ReferenceEquals(myDatum, other.myDatum) && myPower == other.myPower && myTrsf.Equals(other.myTrsf);

    public override readonly bool Equals(object? obj) => obj is TopLoc_ItemLocation other && Equals(other);

    public override readonly int GetHashCode()
        => HashCode.Combine(myDatum is null ? 0 : RuntimeHelpers.GetHashCode(myDatum), myPower, myTrsf);

    public static bool operator ==(in TopLoc_ItemLocation theLeft, in TopLoc_ItemLocation theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in TopLoc_ItemLocation theLeft, in TopLoc_ItemLocation theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => $"TopLoc_ItemLocation(power {myPower}, {myTrsf})";
}
