using Kerf.Geometry;
using Occt;

namespace Kerf.Modeling;

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
        ? (Kerf.Modeling.Extrinsic)(_value - 12) switch
        {
            Kerf.Modeling.Extrinsic.XYZ => EulerSequence.Extrinsic_XYZ,
            Kerf.Modeling.Extrinsic.XZY => EulerSequence.Extrinsic_XZY,
            Kerf.Modeling.Extrinsic.YZX => EulerSequence.Extrinsic_YZX,
            Kerf.Modeling.Extrinsic.YXZ => EulerSequence.Extrinsic_YXZ,
            Kerf.Modeling.Extrinsic.ZXY => EulerSequence.Extrinsic_ZXY,
            Kerf.Modeling.Extrinsic.ZYX => EulerSequence.Extrinsic_ZYX,
            Kerf.Modeling.Extrinsic.XYX => EulerSequence.Extrinsic_XYX,
            Kerf.Modeling.Extrinsic.XZX => EulerSequence.Extrinsic_XZX,
            Kerf.Modeling.Extrinsic.YZY => EulerSequence.Extrinsic_YZY,
            Kerf.Modeling.Extrinsic.YXY => EulerSequence.Extrinsic_YXY,
            Kerf.Modeling.Extrinsic.ZXZ => EulerSequence.Extrinsic_ZXZ,
            _ => EulerSequence.Extrinsic_ZYZ,
        }
        : (Kerf.Modeling.Intrinsic)_value switch
        {
            Kerf.Modeling.Intrinsic.XYZ => EulerSequence.Intrinsic_XYZ,
            Kerf.Modeling.Intrinsic.XZY => EulerSequence.Intrinsic_XZY,
            Kerf.Modeling.Intrinsic.YZX => EulerSequence.Intrinsic_YZX,
            Kerf.Modeling.Intrinsic.YXZ => EulerSequence.Intrinsic_YXZ,
            Kerf.Modeling.Intrinsic.ZXY => EulerSequence.Intrinsic_ZXY,
            Kerf.Modeling.Intrinsic.ZYX => EulerSequence.Intrinsic_ZYX,
            Kerf.Modeling.Intrinsic.XYX => EulerSequence.Intrinsic_XYX,
            Kerf.Modeling.Intrinsic.XZX => EulerSequence.Intrinsic_XZX,
            Kerf.Modeling.Intrinsic.YZY => EulerSequence.Intrinsic_YZY,
            Kerf.Modeling.Intrinsic.YXY => EulerSequence.Intrinsic_YXY,
            Kerf.Modeling.Intrinsic.ZXZ => EulerSequence.Intrinsic_ZXZ,
            _ => EulerSequence.Intrinsic_ZYZ,
        };

    /// <inheritdoc/>
    public override string ToString() => IsExtrinsic ? $"Extrinsic.{Extrinsic}" : $"Intrinsic.{Intrinsic}";
}
