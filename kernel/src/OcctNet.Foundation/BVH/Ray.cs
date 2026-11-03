// Port of: src/FoundationClasses/TKMath/BVH/BVH_Ray.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using System.Numerics;

namespace Kerf.Engine.Geometry.Bvh;

/// <summary>A ray of BVH vectors: an origin, a direction, and the reciprocal of the direction.</summary>
/// <remarks>
/// OCCT's public data members are properties with internal setters. <c>default(BVH_Ray)</c> equals OCCT's
/// default (all zero).
/// </remarks>
public partial struct Ray<T, TVec> : IEquatable<Ray<T, TVec>>
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
    public Ray(in TVec origin, in TVec direct)
    {
        Origin = origin;
        Direct = direct;
        InvDirect = ComputeInvDirect(direct);
    }

    // The reciprocal of a direction component (infinity for zero).
    private static T InvComponent(T dir) => (dir != T.Zero) ? (T.One / dir) : T.PositiveInfinity;

    // The reciprocal of the direction vector.
    private static TVec ComputeInvDirect(in TVec direct)
    {
        TVec inv = direct;
        for (int i = 0; i < TVec.Length(); ++i)
        {
            NCollection_VecTools.Change<T, TVec>(ref inv, i) = InvComponent(direct[i]);
        }

        return inv;
    }

    // Copies with a member replaced (InvDirect is left as it is, as when C++ assigns a member)

    public readonly Ray<T, TVec> WithOrigin(in TVec origin) => this with { Origin = origin };

    public readonly Ray<T, TVec> WithDirect(in TVec direct) => this with { Direct = direct };

    public readonly Ray<T, TVec> WithInvDirect(in TVec invDirect) => this with { InvDirect = invDirect };

    public readonly bool Equals(Ray<T, TVec> other)
        => Origin.Equals(other.Origin) && Direct.Equals(other.Direct) && InvDirect.Equals(other.InvDirect);

    public override readonly bool Equals(object? obj) => obj is Ray<T, TVec> other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(Origin, Direct, InvDirect);

    public static bool operator ==(in Ray<T, TVec> left, in Ray<T, TVec> right) => left.Equals(right);

    public static bool operator !=(in Ray<T, TVec> left, in Ray<T, TVec> right) => !left.Equals(right);

    public override readonly string ToString() => $"BVH_Ray({Origin}, {Direct})";
}
