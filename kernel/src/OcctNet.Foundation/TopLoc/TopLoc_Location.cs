// Port of: src/FoundationClasses/TKMath/TopLoc/TopLoc_Location.hxx
// Port of: src/FoundationClasses/TKMath/TopLoc/TopLoc_Location.lxx
// Port of: src/FoundationClasses/TKMath/TopLoc/TopLoc_Location.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1990-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.CompilerServices;

namespace Occt;

/// <summary>
/// A composite transition: a series of elementary coordinate systems (<see cref="TopLoc_Datum3D"/>s), each raised
/// to a power. <c>this = NextLocation() * FirstDatum() ^ FirstPower()</c>.
/// </summary>
/// <remarks>
/// <para>
/// Locations are compared by the <b>identity</b> of their datums, not by their transformations: two locations
/// built separately from the same <see cref="gp_Trsf"/> are different, while a location and a copy of it, or
/// <c>L * L.Inverted()</c> and the identity, are equal. <see cref="Equals(TopLoc_Location)"/>,
/// <see cref="GetHashCode"/>, <c>==</c> and <c>!=</c> follow OCCT's <see cref="IsEqual"/> and <see cref="HashCode"/>,
/// so locations work as keys of NCollection and .NET maps as they do in OCCT.
/// </para>
/// <para>
/// A struct holding one reference to a shared, immutable list, as OCCT's class is. <c>default(TopLoc_Location)</c>
/// and <c>new TopLoc_Location()</c> are both the identity. The internal mutators <c>Identity</c> and <c>Clear</c>
/// only drop the list; their twin is <c>new TopLoc_Location()</c>.
/// </para>
/// </remarks>
public partial struct TopLoc_Location : IEquatable<TopLoc_Location>
{
    private TopLoc_SListOfItemLocation myItems;

    private static readonly gp_Trsf THE_IDENTITY_TRSF = new gp_Trsf();

    /// <summary>The identity ("empty" location).</summary>
    public TopLoc_Location() => myItems = new TopLoc_SListOfItemLocation();

    /// <summary>The location of a new datum built from T.</summary>
    public TopLoc_Location(in gp_Trsf T)
    {
        TopLoc_Datum3D D = new TopLoc_Datum3D(T);
        myItems = new TopLoc_SListOfItemLocation();
        myItems.Construct(new TopLoc_ItemLocation(D, 1));
    }

    /// <summary>The location of the datum D.</summary>
    public TopLoc_Location(TopLoc_Datum3D D)
    {
        myItems = new TopLoc_SListOfItemLocation();
        myItems.Construct(new TopLoc_ItemLocation(D, 1));
    }

    // NextLocation: OCCT reinterprets the tail list as a location.
    private TopLoc_Location(in TopLoc_SListOfItemLocation theItems) => myItems = theItems;

    /// <summary>True if this location is the identity.</summary>
    public readonly bool IsIdentity() => myItems.IsEmpty();

    /// <summary>Resets this location to the identity.</summary>
    internal void Identity() => myItems.Clear();

    /// <summary>The first elementary datum. Use <see cref="NextLocation"/> to reach the others.</summary>
    /// <exception cref="Standard_NoSuchObject">This location is the identity.</exception>
    public readonly TopLoc_Datum3D FirstDatum() => myItems.Value().myDatum;

    /// <summary>The power of the first elementary datum.</summary>
    /// <exception cref="Standard_NoSuchObject">This location is the identity.</exception>
    public readonly int FirstPower() => myItems.Value().myPower;

    /// <summary>This location without its first datum: <c>this = NextLocation() * FirstDatum() ^ FirstPower()</c>.</summary>
    /// <remarks>OCCT documents a Standard_NoSuchObject for the identity, but returns the identity, as the port does.</remarks>
    public readonly TopLoc_Location NextLocation() => new TopLoc_Location(myItems.Tail());

    /// <summary>The transformation of this location.</summary>
    public readonly gp_Trsf Transformation()
    {
        if (IsIdentity())
        {
            return THE_IDENTITY_TRSF;
        }
        else
        {
            return myItems.Value().myTrsf;
        }
    }

    /// <summary>The transformation of the location, as OCCT's conversion operator to gp_Trsf.</summary>
    /// <remarks>Explicit: the port has no implicit conversions.</remarks>
    public static explicit operator gp_Trsf(in TopLoc_Location theLocation) => theLocation.Transformation();

    /// <summary>The inverse of this location: the datums in reverse order with opposite powers.</summary>
    public readonly TopLoc_Location Inverted()
    {
        if (IsIdentity())
        {
            return this;
        }

        //
        // the inverse of a Location is a chain in revert order
        // with opposite powers and same Local
        //
        TopLoc_Location result = new TopLoc_Location();
        TopLoc_SListOfItemLocation items = myItems;
        while (items.More())
        {
            result.myItems.Construct(new TopLoc_ItemLocation(items.Value().myDatum, -items.Value().myPower));
            items.Next();
        }

        return result;
    }

    /// <summary><c>this * Other</c>: the datums are concatenated, and powers of the same datum meeting at the join are added (dropped if they cancel).</summary>
    public readonly TopLoc_Location Multiplied(in TopLoc_Location Other)
    {
        // prepend the chain Other in front of this
        // cancelling null exponents

        if (IsIdentity())
        {
            return Other;
        }

        if (Other.IsIdentity())
        {
            return this;
        }

        // prepend the queue of Other
        TopLoc_Location result = Multiplied(Other.NextLocation());
        // does the head of Other cancel the head of result

        int p = Other.FirstPower();
        if (!result.IsIdentity())
        {
            if (Other.FirstDatum() == result.FirstDatum())
            {
                p += result.FirstPower();
                result.myItems.ToTail();
            }
        }

        if (p != 0)
        {
            result.myItems.Construct(new TopLoc_ItemLocation(Other.FirstDatum(), p));
        }

        return result;
    }

    public static TopLoc_Location operator *(in TopLoc_Location theLeft, in TopLoc_Location theRight)
        => theLeft.Multiplied(theRight);

    /// <summary><c>this * Other.Inverted()</c>.</summary>
    public readonly TopLoc_Location Divided(in TopLoc_Location Other) => Multiplied(Other.Inverted());

    public static TopLoc_Location operator /(in TopLoc_Location theLeft, in TopLoc_Location theRight)
        => theLeft.Divided(theRight);

    /// <summary><c>Other.Inverted() * this</c>.</summary>
    public readonly TopLoc_Location Predivided(in TopLoc_Location Other)
    {
        if (Other.IsIdentity())
        {
            return this;
        }

        if (IsIdentity())
        {
            return Other.Inverted();
        }

        if (this == Other)
        {
            return new TopLoc_Location();
        }

        return Other.Inverted().Multiplied(this);
    }

    /// <summary>This location to the power pwr; the identity for 0, and negative powers invert.</summary>
    public readonly TopLoc_Location Powered(int pwr)
    {
        if (IsIdentity())
        {
            return this;
        }

        if (pwr == 1)
        {
            return this;
        }

        if (pwr == 0)
        {
            return new TopLoc_Location();
        }

        // optimisation when just one element
        if (myItems.Tail().IsEmpty())
        {
            TopLoc_Location result = new TopLoc_Location();
            result.myItems.Construct(new TopLoc_ItemLocation(FirstDatum(), FirstPower() * pwr));
            return result;
        }

        if (pwr > 0)
        {
            return Multiplied(Powered(pwr - 1));
        }
        else
        {
            return Inverted().Powered(-pwr);
        }
    }

    /// <summary>A hash of the datums (by identity) and powers, consistent with <see cref="IsEqual"/>; 0 for the identity.</summary>
    /// <remarks>
    /// OCCT combines the datums' addresses and the powers with MurmurHash into a <c>size_t</c>; the port combines the
    /// datums' runtime hash codes and the powers in the same order with <see cref="System.HashCode"/>. Both values
    /// depend on the process, so only the structure is kept.
    /// </remarks>
    public readonly int HashCode()
    {
        if (myItems.IsEmpty())
        {
            return 0;
        }

        int aHash = OptimalSeed;
        for (TopLoc_SListOfItemLocation anIter = myItems; anIter.More(); anIter.Next())
        {
            TopLoc_ItemLocation anItem = anIter.Value();
            aHash = System.HashCode.Combine(RuntimeHelpers.GetHashCode(anItem.myDatum), anItem.myPower, aHash);
        }

        return aHash;
    }

    // opencascade::MurmurHash::optimalSeed<size_t>() (0xA329F1D3A586), truncated to an int
    private const int OptimalSeed = unchecked((int)0xF1D3A586);

    /// <summary>
    /// True if both locations have the same series of datums (by identity) and powers. Locations with equal
    /// transformations built from different datums are not equal.
    /// </summary>
    public readonly bool IsEqual(in TopLoc_Location theOther)
    {
        if (ReferenceEquals(myItems.myNode, theOther.myItems.myNode))
        {
            return true;
        }

        // Iterate through both lists in parallel.
        TopLoc_SListOfItemLocation anIter1 = myItems;
        TopLoc_SListOfItemLocation anIter2 = theOther.myItems;
        while (anIter1.More() && anIter2.More())
        {
            TopLoc_ItemLocation anItem1 = anIter1.Value();
            TopLoc_ItemLocation anItem2 = anIter2.Value();
            if (anItem1.myDatum != anItem2.myDatum)
            {
                return false;
            }

            if (anItem1.myPower != anItem2.myPower)
            {
                return false;
            }

            anIter1.Next();
            anIter2.Next();
        }

        // Both lists should be exhausted at the same time for equality.
        return !anIter1.More() && !anIter2.More();
    }

    /// <summary>The negation of <see cref="IsEqual"/>.</summary>
    public readonly bool IsDifferent(in TopLoc_Location theOther) => !IsEqual(theOther);

    /// <summary>Writes the powers and datums of this location to S.</summary>
    public readonly void ShallowDump(TextWriter S)
    {
        S.Write("TopLoc_Location : ");
        TopLoc_SListOfItemLocation items = myItems;
        if (items.IsEmpty())
        {
            S.Write("Identity" + '\n');
        }

        while (items.More())
        {
            S.Write("\n");
            S.Write("       Exponent : " + items.Value().myPower + '\n');
            items.Value().myDatum.ShallowDump(S);
            items.Next();
        }

        S.Write("\n");
    }

    /// <summary>Makes this location the identity.</summary>
    internal void Clear() => myItems.Clear();

    /// <summary>The precision of scale factors, 1e-14.</summary>
    public static double ScalePrec() => 1.0e-14;

    // Equality follows OCCT's IsEqual: datums by identity, then powers.

    public readonly bool Equals(TopLoc_Location other) => IsEqual(other);

    public override readonly bool Equals(object? obj) => obj is TopLoc_Location other && IsEqual(other);

    public override readonly int GetHashCode() => HashCode();

    public static bool operator ==(in TopLoc_Location theLeft, in TopLoc_Location theRight) => theLeft.IsEqual(theRight);

    public static bool operator !=(in TopLoc_Location theLeft, in TopLoc_Location theRight) => theLeft.IsDifferent(theRight);

    public override readonly string ToString()
        => IsIdentity() ? "TopLoc_Location(Identity)" : $"TopLoc_Location({Transformation()})";
}
