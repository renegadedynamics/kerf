// Port of: src/FoundationClasses/TKMath/TopLoc/TopLoc_Location.hxx
// Port of: src/FoundationClasses/TKMath/TopLoc/TopLoc_Location.lxx
// Port of: src/FoundationClasses/TKMath/TopLoc/TopLoc_Location.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1990-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Engine;
using System.Runtime.CompilerServices;

namespace Kerf.Topology;

/// <summary>
/// A composite transition: a series of elementary coordinate systems (<see cref="Datum3D"/>s), each raised
/// to a power. <c>this = NextLocation() * FirstDatum() ^ FirstPower()</c>.
/// </summary>
/// <remarks>
/// <para>
/// <see cref="Equals(Placement)"/>, <see cref="GetHashCode"/>, <c>==</c> and <c>!=</c> compare the transformations, exactly
/// (decision 1349, question 62; unit 10L1). The engine compares locations as OCCT does, by the <b>identity</b> of their
/// datums (<see cref="IsEqual"/>, <see cref="HashCode"/>, <see cref="PlacementIdentityComparer"/>): two locations built
/// separately from the same <see cref="Transform3D"/> are different there, and whether two shapes are the same
/// (<c>Shape.IsSame</c>) depends on it.
/// </para>
/// <para>
/// A struct holding one reference to a shared, immutable list, as OCCT's class is. <c>default(TopLoc_Location)</c>
/// and <c>new TopLoc_Location()</c> are both the identity. It is a readonly struct (unit 10G): OCCT's
/// <c>Identity</c> and <c>Clear</c>, which dropped the list, are <c>new TopLoc_Location()</c>.
/// </para>
/// </remarks>
public readonly partial struct Placement : IEquatable<Placement>
{
    private readonly SListOfItemLocation _items;

    private static readonly Transform3D THE_IDENTITY_TRSF = new Transform3D();

    /// <summary>The identity ("empty" location).</summary>
    public Placement() => _items = new SListOfItemLocation();

    /// <summary>The location of a new datum built from T.</summary>
    public Placement(in Transform3D t)
    {
        Datum3D d = new Datum3D(t);
        _items = new SListOfItemLocation();
        _items.Construct(new ItemLocation(d, 1));
    }

    /// <summary>The location of the datum D.</summary>
    internal Placement(Datum3D d)
    {
        _items = new SListOfItemLocation();
        _items.Construct(new ItemLocation(d, 1));
    }

    // NextLocation: OCCT reinterprets the tail list as a location.
    private Placement(in SListOfItemLocation items) => _items = items;

    /// <summary>True if this location is the identity.</summary>
    public readonly bool IsIdentity => _items.IsEmpty;

    /// <summary>The first elementary datum. Use <see cref="NextLocation"/> to reach the others.</summary>
    /// <exception cref="NoSuchObjectException">This location is the identity.</exception>
    internal readonly Datum3D FirstDatum() => _items.Value()._datum;

    /// <summary>The power of the first elementary datum.</summary>
    /// <exception cref="NoSuchObjectException">This location is the identity.</exception>
    internal readonly int FirstPower() => _items.Value()._power;

    /// <summary>This location without its first datum: <c>this = NextLocation() * FirstDatum() ^ FirstPower()</c>.</summary>
    /// <remarks>OCCT documents a Standard_NoSuchObject for the identity, but returns the identity, as the port does.</remarks>
    internal readonly Placement NextLocation() => new Placement(_items.Tail());

    /// <summary>The transformation of this location.</summary>
    public readonly Transform3D Transformation
    {
        get
        {
            if (IsIdentity)
            {
                return THE_IDENTITY_TRSF;
            }
            else
            {
                return _items.Value()._trsf;
            }
        }
    }

    /// <summary>The transformation of the location, as OCCT's conversion operator to gp_Trsf.</summary>
    /// <remarks>Explicit: the port has no implicit conversions.</remarks>
    public static explicit operator Transform3D(in Placement location) => location.Transformation;

    /// <summary>The inverse of this location: the datums in reverse order with opposite powers.</summary>
    public readonly Placement Inverted()
    {
        if (IsIdentity)
        {
            return this;
        }

        //
        // the inverse of a Location is a chain in revert order
        // with opposite powers and same Local
        //
        SListNodeOfItemLocation head = _items._node!;
        SListNodeOfItemLocation? known = Volatile.Read(ref head._inverse);
        if (known is not null)
        {
            SListOfItemLocation inverse = new SListOfItemLocation { _node = known };
            return new Placement(inverse);
        }

        SListOfItemLocation result = new SListOfItemLocation();
        SListOfItemLocation items = _items;
        while (items.More())
        {
            result.Construct(new ItemLocation(items.Value()._datum, -items.Value()._power));
            items.Next();
        }

        Volatile.Write(ref head._inverse, result._node);
        return new Placement(result);
    }

    /// <summary><c>this * Other</c>: the datums are concatenated, and powers of the same datum meeting at the join are added (dropped if they cancel).</summary>
    public readonly Placement Multiplied(in Placement other)
    {
        // prepend the chain Other in front of this
        // cancelling null exponents

        if (IsIdentity)
        {
            return other;
        }

        if (other.IsIdentity)
        {
            return this;
        }

        // prepend the queue of Other
        Placement head = Multiplied(other.NextLocation());
        SListOfItemLocation result = head._items;
        // does the head of Other cancel the head of result

        int p = other.FirstPower();
        if (!head.IsIdentity)
        {
            if (other.FirstDatum() == head.FirstDatum())
            {
                p += head.FirstPower();
                result.ToTail();
            }
        }

        if (p != 0)
        {
            result.Construct(new ItemLocation(other.FirstDatum(), p));
        }

        return new Placement(result);
    }

    public static Placement operator *(in Placement left, in Placement right)
        => left.Multiplied(right);

    /// <summary><c>this * Other.Inverted()</c>.</summary>
    public readonly Placement Divided(in Placement other) => Multiplied(other.Inverted());

    public static Placement operator /(in Placement left, in Placement right)
        => left.Divided(right);

    /// <summary><c>Other.Inverted() * this</c>.</summary>
    public readonly Placement Predivided(in Placement other)
    {
        if (other.IsIdentity)
        {
            return this;
        }

        if (IsIdentity)
        {
            return other.Inverted();
        }

        if (this.IsEqual(other))
        {
            return new Placement();
        }

        return other.Inverted().Multiplied(this);
    }

    /// <summary>This location to the power pwr; the identity for 0, and negative powers invert.</summary>
    public readonly Placement Powered(int pwr)
    {
        if (IsIdentity)
        {
            return this;
        }

        if (pwr == 1)
        {
            return this;
        }

        if (pwr == 0)
        {
            return new Placement();
        }

        // optimisation when just one element
        if (_items.Tail().IsEmpty)
        {
            SListOfItemLocation result = new SListOfItemLocation();
            result.Construct(new ItemLocation(FirstDatum(), FirstPower() * pwr));
            return new Placement(result);
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
    internal readonly int HashCode()
    {
        if (_items.IsEmpty)
        {
            return 0;
        }

        int hash = OptimalSeed;
        for (SListOfItemLocation iter = _items; iter.More(); iter.Next())
        {
            ItemLocation item = iter.Value();
            hash = System.HashCode.Combine(RuntimeHelpers.GetHashCode(item._datum), item._power, hash);
        }

        return hash;
    }

    // opencascade::MurmurHash::optimalSeed<size_t>() (0xA329F1D3A586), truncated to an int
    private const int OptimalSeed = unchecked((int)0xF1D3A586);

    /// <summary>
    /// True if both locations have the same series of datums (by identity) and powers. Locations with equal
    /// transformations built from different datums are not equal.
    /// </summary>
    internal readonly bool IsEqual(in Placement other)
    {
        if (ReferenceEquals(_items._node, other._items._node))
        {
            return true;
        }

        // Iterate through both lists in parallel.
        SListOfItemLocation iter1 = _items;
        SListOfItemLocation iter2 = other._items;
        while (iter1.More() && iter2.More())
        {
            ItemLocation item1 = iter1.Value();
            ItemLocation item2 = iter2.Value();
            if (item1._datum != item2._datum)
            {
                return false;
            }

            if (item1._power != item2._power)
            {
                return false;
            }

            iter1.Next();
            iter2.Next();
        }

        // Both lists should be exhausted at the same time for equality.
        return !iter1.More() && !iter2.More();
    }

    /// <summary>The negation of <see cref="IsEqual"/>.</summary>
    internal readonly bool IsDifferent(in Placement other) => !IsEqual(other);

    /// <summary>Writes the powers and datums of this location to S.</summary>
    internal readonly void ShallowDump(TextWriter s)
    {
        s.Write("TopLoc_Location : ");
        SListOfItemLocation items = _items;
        if (items.IsEmpty)
        {
            s.Write("Identity" + '\n');
        }

        while (items.More())
        {
            s.Write("\n");
            s.Write("       Exponent : " + items.Value()._power + '\n');
            items.Value()._datum.ShallowDump(s);
            items.Next();
        }

        s.Write("\n");
    }

    /// <summary>The precision of scale factors, 1e-14.</summary>
    internal static double ScalePrec() => 1.0e-14;

    // Public equality is by value (decision 1349, 62): the transformations, exactly. The engine uses IsEqual (identity)

    /// <summary>True if both placements have exactly the same transformation, however they were built.</summary>
    public readonly bool Equals(Placement other) => IsEqual(other) || Transformation == other.Transformation;

    /// <inheritdoc cref="Equals(Placement)"/>
    public override readonly bool Equals(object? obj) => obj is Placement other && Equals(other);

    /// <summary>A hash of the transformation, consistent with <see cref="Equals(Placement)"/>.</summary>
    public override readonly int GetHashCode() => IsIdentity ? 0 : Transformation.GetHashCode();

    /// <summary>True if both placements have exactly the same transformation.</summary>
    public static bool operator ==(in Placement left, in Placement right) => left.Equals(right);

    /// <summary>True unless both placements have exactly the same transformation.</summary>
    public static bool operator !=(in Placement left, in Placement right) => !left.Equals(right);

    public override readonly string ToString()
        => IsIdentity ? "TopLoc_Location(Identity)" : $"TopLoc_Location({Transformation})";
}

/// <summary>
/// Compares placements as OCCT does, by the identity of their datums (<see cref="Placement.IsEqual"/>): the engine's
/// maps of locations (the BREP writers' location tables) keep OCCT's behaviour with it.
/// </summary>
internal sealed class PlacementIdentityComparer : IEqualityComparer<Placement>
{
    public static PlacementIdentityComparer Instance { get; } = new();

    public bool Equals(Placement x, Placement y) => x.IsEqual(y);

    public int GetHashCode(Placement obj) => obj.HashCode();
}
