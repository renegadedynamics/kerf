// Port of: src/FoundationClasses/TKMath/BVH/BVH_Ray.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Numerics;

namespace Occt;

/// <summary>A ray of BVH vectors: an origin, a direction, and the reciprocal of the direction.</summary>
/// <remarks>
/// OCCT's public data members are properties with internal setters. <c>default(BVH_Ray)</c> equals OCCT's
/// default (all zero).
/// </remarks>
public partial struct BVH_Ray<T, TVec> : IEquatable<BVH_Ray<T, TVec>>
    where T : unmanaged, IFloatingPointIeee754<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    /// <summary>Ray origin point.</summary>
    public TVec Origin { readonly get; internal set; }

    /// <summary>Ray direction vector.</summary>
    public TVec Direct { readonly get; internal set; }

    /// <summary>Reciprocal of direction (1/Direct), infinite for a zero component.</summary>
    public TVec InvDirect { readonly get; internal set; }

    /// <summary>A ray with the given origin and direction.</summary>
    public BVH_Ray(in TVec theOrigin, in TVec theDirect)
    {
        Origin = theOrigin;
        Direct = theDirect;
        InvDirect = computeInvDirect(theDirect);
    }

    // The reciprocal of a direction component (infinity for zero).
    private static T invComponent(T theDir) => (theDir != T.Zero) ? (T.One / theDir) : T.PositiveInfinity;

    // The reciprocal of the direction vector.
    private static TVec computeInvDirect(in TVec theDirect)
    {
        TVec anInv = theDirect;
        for (int i = 0; i < TVec.Length(); ++i)
        {
            NCollection_VecTools.Change<T, TVec>(ref anInv, i) = invComponent(theDirect[i]);
        }

        return anInv;
    }

    // Copies with a member replaced (InvDirect is left as it is, as when C++ assigns a member)

    public readonly BVH_Ray<T, TVec> WithOrigin(in TVec theOrigin) => this with { Origin = theOrigin };

    public readonly BVH_Ray<T, TVec> WithDirect(in TVec theDirect) => this with { Direct = theDirect };

    public readonly BVH_Ray<T, TVec> WithInvDirect(in TVec theInvDirect) => this with { InvDirect = theInvDirect };

    public readonly bool Equals(BVH_Ray<T, TVec> other)
        => Origin.Equals(other.Origin) && Direct.Equals(other.Direct) && InvDirect.Equals(other.InvDirect);

    public override readonly bool Equals(object? obj) => obj is BVH_Ray<T, TVec> other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(Origin, Direct, InvDirect);

    public static bool operator ==(in BVH_Ray<T, TVec> theLeft, in BVH_Ray<T, TVec> theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in BVH_Ray<T, TVec> theLeft, in BVH_Ray<T, TVec> theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => $"BVH_Ray({Origin}, {Direct})";
}
