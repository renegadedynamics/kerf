// Port of: src/FoundationClasses/TKMath/gp/gp_QuaternionSLerp.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Geometry;

/// <summary>Spherical linear interpolation of quaternions; the result is a unit quaternion.</summary>
/// <remarks>
/// <c>new QuaternionSLerp()</c> holds identities and a zero angle (OCCT's default constructor leaves the angle
/// uninitialised); <c>default(QuaternionSLerp)</c> is not valid.
/// </remarks>
internal partial struct QuaternionSLerp : IEquatable<QuaternionSLerp>
{
    private Quaternion3 _qStart;
    private Quaternion3 _qEnd;
    private double _omega;

    /// <summary>The quaternion interpolated between two quaternions.</summary>
    /// <param name="qStart">The first quaternion.</param>
    /// <param name="qEnd">The second quaternion.</param>
    /// <param name="t">The interpolation coefficient in [0, 1]: 0 gives <paramref name="qStart"/>, 1 <paramref name="qEnd"/>.</param>
    public static Quaternion3 Interpolate(in Quaternion3 qStart, in Quaternion3 qEnd, double t)
    {
        QuaternionSLerp lerp = new(qStart, qEnd);
        lerp.Interpolate(t, out Quaternion3 result);
        return result;
    }

    /// <summary>The tool initialised with unit quaternions, which are not normalised again (OCCT's <c>InitFromUnit</c>).</summary>
    public static QuaternionSLerp FromUnit(in Quaternion3 qStart, in Quaternion3 qEnd)
    {
        QuaternionSLerp lerp = new();
        lerp.InitFromUnit(qStart, qEnd);
        return lerp;
    }

    public QuaternionSLerp()
    {
        _qStart = new Quaternion3();
        _qEnd = new Quaternion3();
        _omega = 0.0;
    }

    /// <summary>The tool initialised with two quaternions, normalised first (OCCT's <c>Init</c>).</summary>
    public QuaternionSLerp(in Quaternion3 qStart, in Quaternion3 qEnd) : this() => Init(qStart, qEnd);

    // Mutators (internal: see docs/translation-rules.md)

    internal void Init(in Quaternion3 qStart, in Quaternion3 qEnd)
        => InitFromUnit(qStart.Normalized(), qEnd.Normalized());

    internal void InitFromUnit(in Quaternion3 qStart, in Quaternion3 qEnd)
    {
        _qStart = qStart;
        _qEnd = qEnd;
        double cosOmega = _qStart.Dot(_qEnd);
        if (cosOmega < 0.0)
        {
            cosOmega = -cosOmega;
            _qEnd = -_qEnd;
        }

        if (cosOmega > 0.9999)
        {
            cosOmega = 0.9999;
        }

        _omega = Math.Acos(cosOmega);
        double invSinOmega = (1.0 / Math.Sin(_omega));
        _qStart.Scale(invSinOmega);
        _qEnd.Scale(invSinOmega);
    }

    // Queries

    /// <summary>The quaternion interpolated at <paramref name="t"/> (from 0 to 1).</summary>
    public readonly void Interpolate(double t, out Quaternion3 resultQ)
        => resultQ = _qStart * Math.Sin((1.0 - t) * _omega) + _qEnd * Math.Sin(t * _omega);

    public readonly bool Equals(QuaternionSLerp other)
        => _qStart.Equals(other._qStart) && _qEnd.Equals(other._qEnd) && _omega.Equals(other._omega);

    public override readonly bool Equals(object? obj) => obj is QuaternionSLerp other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_qStart, _qEnd, _omega);

    public static bool operator ==(in QuaternionSLerp left, in QuaternionSLerp right) => left.Equals(right);

    public static bool operator !=(in QuaternionSLerp left, in QuaternionSLerp right) => !left.Equals(right);

    public override readonly string ToString() => FormattableString.Invariant($"QuaternionSLerp({_qStart}, {_qEnd}, {_omega:R})");
}
