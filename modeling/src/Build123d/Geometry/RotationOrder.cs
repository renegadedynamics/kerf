using Kerf.Geometry;
using Occt;

namespace Build123d;

/// <summary>
/// The order and mode in which Euler angles apply: an <see cref="Intrinsic"/> or <see cref="Extrinsic"/>
/// sequence, either of which converts implicitly. The default is <see cref="Intrinsic.XYZ"/>.
/// </summary>
public readonly record struct RotationOrder
{
    readonly int _value;   // 0..11 intrinsic, 12..23 extrinsic; 0 (the default) is Intrinsic.XYZ

    RotationOrder(int value) => _value = value;

    /// <summary>True for an extrinsic sequence, about the fixed world axes.</summary>
    public bool IsExtrinsic => _value >= 12;

    /// <summary>The intrinsic sequence, if this is one.</summary>
    public Intrinsic? Intrinsic => IsExtrinsic ? null : (Intrinsic)_value;

    /// <summary>The extrinsic sequence, if this is one.</summary>
    public Extrinsic? Extrinsic => IsExtrinsic ? (Extrinsic)(_value - 12) : null;

    /// <summary>An intrinsic order.</summary>
    public static implicit operator RotationOrder(Intrinsic order) => new((int)order);

    /// <summary>An extrinsic order.</summary>
    public static implicit operator RotationOrder(Extrinsic order) => new(12 + (int)order);

    /// <summary>The matching OCCT Euler sequence.</summary>
    internal EulerSequence ToOcct() => IsExtrinsic
        ? (Build123d.Extrinsic)(_value - 12) switch
        {
            Build123d.Extrinsic.XYZ => EulerSequence.Extrinsic_XYZ,
            Build123d.Extrinsic.XZY => EulerSequence.Extrinsic_XZY,
            Build123d.Extrinsic.YZX => EulerSequence.Extrinsic_YZX,
            Build123d.Extrinsic.YXZ => EulerSequence.Extrinsic_YXZ,
            Build123d.Extrinsic.ZXY => EulerSequence.Extrinsic_ZXY,
            Build123d.Extrinsic.ZYX => EulerSequence.Extrinsic_ZYX,
            Build123d.Extrinsic.XYX => EulerSequence.Extrinsic_XYX,
            Build123d.Extrinsic.XZX => EulerSequence.Extrinsic_XZX,
            Build123d.Extrinsic.YZY => EulerSequence.Extrinsic_YZY,
            Build123d.Extrinsic.YXY => EulerSequence.Extrinsic_YXY,
            Build123d.Extrinsic.ZXZ => EulerSequence.Extrinsic_ZXZ,
            _ => EulerSequence.Extrinsic_ZYZ,
        }
        : (Build123d.Intrinsic)_value switch
        {
            Build123d.Intrinsic.XYZ => EulerSequence.Intrinsic_XYZ,
            Build123d.Intrinsic.XZY => EulerSequence.Intrinsic_XZY,
            Build123d.Intrinsic.YZX => EulerSequence.Intrinsic_YZX,
            Build123d.Intrinsic.YXZ => EulerSequence.Intrinsic_YXZ,
            Build123d.Intrinsic.ZXY => EulerSequence.Intrinsic_ZXY,
            Build123d.Intrinsic.ZYX => EulerSequence.Intrinsic_ZYX,
            Build123d.Intrinsic.XYX => EulerSequence.Intrinsic_XYX,
            Build123d.Intrinsic.XZX => EulerSequence.Intrinsic_XZX,
            Build123d.Intrinsic.YZY => EulerSequence.Intrinsic_YZY,
            Build123d.Intrinsic.YXY => EulerSequence.Intrinsic_YXY,
            Build123d.Intrinsic.ZXZ => EulerSequence.Intrinsic_ZXZ,
            _ => EulerSequence.Intrinsic_ZYZ,
        };

    /// <inheritdoc/>
    public override string ToString() => IsExtrinsic ? $"Extrinsic.{Extrinsic}" : $"Intrinsic.{Intrinsic}";
}
