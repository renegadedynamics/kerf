// Port of: src/FoundationClasses/TKMath/gp/gp_QuaternionNLerp.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Normalised linear interpolation of quaternions (an approximate rotation interpolation). The result is not a
/// unit quaternion: its length lies between sqrt(2)/2 and 1.
/// </summary>
/// <remarks>
/// <c>new gp_QuaternionNLerp()</c> holds identities, as in OCCT; <c>default(gp_QuaternionNLerp)</c> is not valid.
/// </remarks>
public partial struct gp_QuaternionNLerp : IEquatable<gp_QuaternionNLerp>
{
    private gp_Quaternion myQStart;
    private gp_Quaternion myQEnd;

    /// <summary>The quaternion interpolated between two quaternions.</summary>
    /// <param name="theQStart">The first quaternion.</param>
    /// <param name="theQEnd">The second quaternion.</param>
    /// <param name="theT">The interpolation coefficient in [0, 1]: 0 gives <paramref name="theQStart"/>, 1 <paramref name="theQEnd"/>.</param>
    public static gp_Quaternion Interpolate(in gp_Quaternion theQStart, in gp_Quaternion theQEnd, double theT)
    {
        gp_QuaternionNLerp aLerp = new(theQStart, theQEnd);
        aLerp.Interpolate(theT, out gp_Quaternion aResult);
        return aResult;
    }

    /// <summary>The tool initialised with unit quaternions, which are not normalised again (OCCT's <c>InitFromUnit</c>).</summary>
    public static gp_QuaternionNLerp FromUnit(in gp_Quaternion theQStart, in gp_Quaternion theQEnd)
    {
        gp_QuaternionNLerp aLerp = new();
        aLerp.InitFromUnit(theQStart, theQEnd);
        return aLerp;
    }

    public gp_QuaternionNLerp()
    {
        myQStart = new gp_Quaternion();
        myQEnd = new gp_Quaternion();
    }

    /// <summary>The tool initialised with two quaternions, normalised first (OCCT's <c>Init</c>).</summary>
    public gp_QuaternionNLerp(in gp_Quaternion theQStart, in gp_Quaternion theQEnd) : this() => Init(theQStart, theQEnd);

    // Mutators (internal: see docs/translation-rules.md)

    internal void Init(in gp_Quaternion theQStart, in gp_Quaternion theQEnd)
        => InitFromUnit(theQStart.Normalized(), theQEnd.Normalized());

    internal void InitFromUnit(in gp_Quaternion theQStart, in gp_Quaternion theQEnd)
    {
        myQStart = theQStart;
        myQEnd = theQEnd;
        double anInner = myQStart.Dot(myQEnd);
        if (anInner < 0.0)
        {
            myQEnd = -myQEnd;
        }

        myQEnd -= myQStart;
    }

    // Queries

    /// <summary>The quaternion interpolated at <paramref name="theT"/> (from 0 to 1).</summary>
    public readonly void Interpolate(double theT, out gp_Quaternion theResultQ) => theResultQ = myQStart + myQEnd * theT;

    public readonly bool Equals(gp_QuaternionNLerp other) => myQStart.Equals(other.myQStart) && myQEnd.Equals(other.myQEnd);

    public override readonly bool Equals(object? obj) => obj is gp_QuaternionNLerp other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(myQStart, myQEnd);

    public static bool operator ==(in gp_QuaternionNLerp theLeft, in gp_QuaternionNLerp theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in gp_QuaternionNLerp theLeft, in gp_QuaternionNLerp theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => $"gp_QuaternionNLerp({myQStart}, {myQEnd})";
}
