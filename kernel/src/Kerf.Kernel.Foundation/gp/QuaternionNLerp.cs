// Port of: src/FoundationClasses/TKMath/gp/gp_QuaternionNLerp.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Geometry;

/// <summary>
/// Normalised linear interpolation of quaternions (an approximate rotation interpolation). The result is not a
/// unit quaternion: its length lies between sqrt(2)/2 and 1.
/// </summary>
/// <remarks>
/// <c>new gp_QuaternionNLerp()</c> holds identities, as in OCCT; <c>default(gp_QuaternionNLerp)</c> is not valid.
/// </remarks>
public partial struct QuaternionNLerp : IEquatable<QuaternionNLerp>
{
    private Quaternion3 _qStart;
    private Quaternion3 _qEnd;

    /// <summary>The quaternion interpolated between two quaternions.</summary>
    /// <param name="qStart">The first quaternion.</param>
    /// <param name="qEnd">The second quaternion.</param>
    /// <param name="t">The interpolation coefficient in [0, 1]: 0 gives <paramref name="qStart"/>, 1 <paramref name="qEnd"/>.</param>
    public static Quaternion3 Interpolate(in Quaternion3 qStart, in Quaternion3 qEnd, double t)
    {
        QuaternionNLerp lerp = new(qStart, qEnd);
        lerp.Interpolate(t, out Quaternion3 result);
        return result;
    }

    /// <summary>The tool initialised with unit quaternions, which are not normalised again (OCCT's <c>InitFromUnit</c>).</summary>
    public static QuaternionNLerp FromUnit(in Quaternion3 qStart, in Quaternion3 qEnd)
    {
        QuaternionNLerp lerp = new();
        lerp.InitFromUnit(qStart, qEnd);
        return lerp;
    }

    public QuaternionNLerp()
    {
        _qStart = new Quaternion3();
        _qEnd = new Quaternion3();
    }

    /// <summary>The tool initialised with two quaternions, normalised first (OCCT's <c>Init</c>).</summary>
    public QuaternionNLerp(in Quaternion3 qStart, in Quaternion3 qEnd) : this() => Init(qStart, qEnd);

    // Mutators (internal: see docs/translation-rules.md)

    internal void Init(in Quaternion3 qStart, in Quaternion3 qEnd)
        => InitFromUnit(qStart.Normalized(), qEnd.Normalized());

    internal void InitFromUnit(in Quaternion3 qStart, in Quaternion3 qEnd)
    {
        _qStart = qStart;
        _qEnd = qEnd;
        double inner = _qStart.Dot(_qEnd);
        if (inner < 0.0)
        {
            _qEnd = -_qEnd;
        }

        _qEnd -= _qStart;
    }

    // Queries

    /// <summary>The quaternion interpolated at <paramref name="t"/> (from 0 to 1).</summary>
    public readonly void Interpolate(double t, out Quaternion3 resultQ) => resultQ = _qStart + _qEnd * t;

    public readonly bool Equals(QuaternionNLerp other) => _qStart.Equals(other._qStart) && _qEnd.Equals(other._qEnd);

    public override readonly bool Equals(object? obj) => obj is QuaternionNLerp other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_qStart, _qEnd);

    public static bool operator ==(in QuaternionNLerp left, in QuaternionNLerp right) => left.Equals(right);

    public static bool operator !=(in QuaternionNLerp left, in QuaternionNLerp right) => !left.Equals(right);

    public override readonly string ToString() => $"gp_QuaternionNLerp({_qStart}, {_qEnd})";
}
