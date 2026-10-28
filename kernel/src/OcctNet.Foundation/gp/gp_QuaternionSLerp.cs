// Port of: src/FoundationClasses/TKMath/gp/gp_QuaternionSLerp.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Spherical linear interpolation of quaternions; the result is a unit quaternion.</summary>
/// <remarks>
/// <c>new gp_QuaternionSLerp()</c> holds identities and a zero angle (OCCT's default constructor leaves the angle
/// uninitialised); <c>default(gp_QuaternionSLerp)</c> is not valid.
/// </remarks>
public partial struct gp_QuaternionSLerp : IEquatable<gp_QuaternionSLerp>
{
    private gp_Quaternion myQStart;
    private gp_Quaternion myQEnd;
    private double myOmega;

    /// <summary>The quaternion interpolated between two quaternions.</summary>
    /// <param name="theQStart">The first quaternion.</param>
    /// <param name="theQEnd">The second quaternion.</param>
    /// <param name="theT">The interpolation coefficient in [0, 1]: 0 gives <paramref name="theQStart"/>, 1 <paramref name="theQEnd"/>.</param>
    public static gp_Quaternion Interpolate(in gp_Quaternion theQStart, in gp_Quaternion theQEnd, double theT)
    {
        gp_QuaternionSLerp aLerp = new(theQStart, theQEnd);
        aLerp.Interpolate(theT, out gp_Quaternion aResult);
        return aResult;
    }

    /// <summary>The tool initialised with unit quaternions, which are not normalised again (OCCT's <c>InitFromUnit</c>).</summary>
    public static gp_QuaternionSLerp FromUnit(in gp_Quaternion theQStart, in gp_Quaternion theQEnd)
    {
        gp_QuaternionSLerp aLerp = new();
        aLerp.InitFromUnit(theQStart, theQEnd);
        return aLerp;
    }

    public gp_QuaternionSLerp()
    {
        myQStart = new gp_Quaternion();
        myQEnd = new gp_Quaternion();
        myOmega = 0.0;
    }

    /// <summary>The tool initialised with two quaternions, normalised first (OCCT's <c>Init</c>).</summary>
    public gp_QuaternionSLerp(in gp_Quaternion theQStart, in gp_Quaternion theQEnd) : this() => Init(theQStart, theQEnd);

    // Mutators (internal: see docs/translation-rules.md)

    internal void Init(in gp_Quaternion theQStart, in gp_Quaternion theQEnd)
        => InitFromUnit(theQStart.Normalized(), theQEnd.Normalized());

    internal void InitFromUnit(in gp_Quaternion theQStart, in gp_Quaternion theQEnd)
    {
        myQStart = theQStart;
        myQEnd = theQEnd;
        double cosOmega = myQStart.Dot(myQEnd);
        if (cosOmega < 0.0)
        {
            cosOmega = -cosOmega;
            myQEnd = -myQEnd;
        }

        if (cosOmega > 0.9999)
        {
            cosOmega = 0.9999;
        }

        myOmega = Math.Acos(cosOmega);
        double invSinOmega = (1.0 / Math.Sin(myOmega));
        myQStart.Scale(invSinOmega);
        myQEnd.Scale(invSinOmega);
    }

    // Queries

    /// <summary>The quaternion interpolated at <paramref name="theT"/> (from 0 to 1).</summary>
    public readonly void Interpolate(double theT, out gp_Quaternion theResultQ)
        => theResultQ = myQStart * Math.Sin((1.0 - theT) * myOmega) + myQEnd * Math.Sin(theT * myOmega);

    public readonly bool Equals(gp_QuaternionSLerp other)
        => myQStart.Equals(other.myQStart) && myQEnd.Equals(other.myQEnd) && myOmega.Equals(other.myOmega);

    public override readonly bool Equals(object? obj) => obj is gp_QuaternionSLerp other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(myQStart, myQEnd, myOmega);

    public static bool operator ==(in gp_QuaternionSLerp theLeft, in gp_QuaternionSLerp theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in gp_QuaternionSLerp theLeft, in gp_QuaternionSLerp theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => FormattableString.Invariant($"gp_QuaternionSLerp({myQStart}, {myQEnd}, {myOmega:R})");
}
