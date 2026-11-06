// Port of: src/FoundationClasses/TKMath/gp/gp_TrsfNLerp.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Engine.Geometry;

/// <summary>
/// Linear interpolation of <see cref="Transform3D"/>: the rotation (with <see cref="QuaternionNLerp"/>), the
/// translation and the scale factor are interpolated independently. There is no well-defined interpolation
/// between arbitrary transformations, so this suits simple cases and small intervals.
/// </summary>
/// <remarks>
/// OCCT names this class <c>NCollection_Lerp&lt;gp_Trsf&gt;</c>, a specialisation of the <c>NCollection_Lerp</c>
/// template. The port names it after its header, and writes its translation and scale members (an
/// <c>NCollection_Lerp</c> of <c>gp_XYZ</c> and of <c>double</c>) as start and end fields interpolated with the
/// template's formula <c>(1 - t) * start + t * end</c>.
/// <c>new gp_TrsfNLerp()</c> holds identities; <c>default(gp_TrsfNLerp)</c> is not valid.
/// </remarks>
internal partial struct TransformNLerp : IEquatable<TransformNLerp>
{
    private Xyz _locLerpStart;
    private Xyz _locLerpEnd;
    private double _scaleLerpStart;
    private double _scaleLerpEnd;
    private QuaternionNLerp _rotLerp;
    private Transform3D _trsfStart;
    private Transform3D _trsfEnd;

    /// <summary>The transformation interpolated between two transformations (OCCT's static <c>NCollection_Lerp::Interpolate</c>).</summary>
    public static Transform3D Interpolate(in Transform3D start, in Transform3D end, double t)
    {
        TransformNLerp lerp = new(start, end);
        lerp.Interpolate(t, out Transform3D result);
        return result;
    }

    public TransformNLerp()
    {
        _locLerpStart = new Xyz();
        _locLerpEnd = new Xyz();
        _scaleLerpStart = 0.0;
        _scaleLerpEnd = 0.0;
        _rotLerp = new QuaternionNLerp();
        _trsfStart = new Transform3D();
        _trsfEnd = new Transform3D();
    }

    /// <summary>The tool initialised with two transformations (OCCT's <c>Init</c>).</summary>
    public TransformNLerp(in Transform3D start, in Transform3D end) : this() => Init(start, end);

    // Mutators (internal: see docs/translation-rules.md)

    internal void Init(in Transform3D start, in Transform3D end)
    {
        _trsfStart = start;
        _trsfEnd = end;
        _locLerpStart = start.TranslationXyz();
        _locLerpEnd = end.TranslationXyz();
        _rotLerp.Init(start.GetRotation(), end.GetRotation());
        _scaleLerpStart = start.ScaleFactor;
        _scaleLerpEnd = end.ScaleFactor;
    }

    // Queries

    /// <summary>The transformation interpolated at <paramref name="t"/> in [0, 1].</summary>
    public readonly void Interpolate(double t, out Transform3D result)
    {
        if (Math.Abs(t - 0.0) < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            result = _trsfStart;
            return;
        }
        else if (Math.Abs(t - 1.0) < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            result = _trsfEnd;
            return;
        }

        Xyz loc = (1.0 - t) * _locLerpStart + t * _locLerpEnd; // myLocLerp.Interpolate(theT, aLoc)
        _rotLerp.Interpolate(t, out Quaternion3 rot);
        double scale = (1.0 - t) * _scaleLerpStart + t * _scaleLerpEnd; // myScaleLerp.Interpolate(theT, aScale)
        result = new Transform3D();
        result = result.WithRotation(rot);
        result = result.WithTranslationPart(new Vector3D(loc));
        result = result.WithScaleFactor(scale);
    }

    public readonly bool Equals(TransformNLerp other)
        => _locLerpStart.Equals(other._locLerpStart) && _locLerpEnd.Equals(other._locLerpEnd)
           && _scaleLerpStart.Equals(other._scaleLerpStart) && _scaleLerpEnd.Equals(other._scaleLerpEnd)
           && _rotLerp.Equals(other._rotLerp) && _trsfStart.Equals(other._trsfStart) && _trsfEnd.Equals(other._trsfEnd);

    public override readonly bool Equals(object? obj) => obj is TransformNLerp other && Equals(other);

    public override readonly int GetHashCode()
        => HashCode.Combine(_locLerpStart, _locLerpEnd, _scaleLerpStart, _scaleLerpEnd, _rotLerp, _trsfStart, _trsfEnd);

    public static bool operator ==(in TransformNLerp left, in TransformNLerp right) => left.Equals(right);

    public static bool operator !=(in TransformNLerp left, in TransformNLerp right) => !left.Equals(right);

    public override readonly string ToString() => $"TransformNLerp({_trsfStart}, {_trsfEnd})";
}
