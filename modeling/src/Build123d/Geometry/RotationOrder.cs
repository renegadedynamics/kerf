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
    internal gp_EulerSequence ToOcct() => IsExtrinsic
        ? (Build123d.Extrinsic)(_value - 12) switch
        {
            Build123d.Extrinsic.XYZ => gp_EulerSequence.gp_Extrinsic_XYZ,
            Build123d.Extrinsic.XZY => gp_EulerSequence.gp_Extrinsic_XZY,
            Build123d.Extrinsic.YZX => gp_EulerSequence.gp_Extrinsic_YZX,
            Build123d.Extrinsic.YXZ => gp_EulerSequence.gp_Extrinsic_YXZ,
            Build123d.Extrinsic.ZXY => gp_EulerSequence.gp_Extrinsic_ZXY,
            Build123d.Extrinsic.ZYX => gp_EulerSequence.gp_Extrinsic_ZYX,
            Build123d.Extrinsic.XYX => gp_EulerSequence.gp_Extrinsic_XYX,
            Build123d.Extrinsic.XZX => gp_EulerSequence.gp_Extrinsic_XZX,
            Build123d.Extrinsic.YZY => gp_EulerSequence.gp_Extrinsic_YZY,
            Build123d.Extrinsic.YXY => gp_EulerSequence.gp_Extrinsic_YXY,
            Build123d.Extrinsic.ZXZ => gp_EulerSequence.gp_Extrinsic_ZXZ,
            _ => gp_EulerSequence.gp_Extrinsic_ZYZ,
        }
        : (Build123d.Intrinsic)_value switch
        {
            Build123d.Intrinsic.XYZ => gp_EulerSequence.gp_Intrinsic_XYZ,
            Build123d.Intrinsic.XZY => gp_EulerSequence.gp_Intrinsic_XZY,
            Build123d.Intrinsic.YZX => gp_EulerSequence.gp_Intrinsic_YZX,
            Build123d.Intrinsic.YXZ => gp_EulerSequence.gp_Intrinsic_YXZ,
            Build123d.Intrinsic.ZXY => gp_EulerSequence.gp_Intrinsic_ZXY,
            Build123d.Intrinsic.ZYX => gp_EulerSequence.gp_Intrinsic_ZYX,
            Build123d.Intrinsic.XYX => gp_EulerSequence.gp_Intrinsic_XYX,
            Build123d.Intrinsic.XZX => gp_EulerSequence.gp_Intrinsic_XZX,
            Build123d.Intrinsic.YZY => gp_EulerSequence.gp_Intrinsic_YZY,
            Build123d.Intrinsic.YXY => gp_EulerSequence.gp_Intrinsic_YXY,
            Build123d.Intrinsic.ZXZ => gp_EulerSequence.gp_Intrinsic_ZXZ,
            _ => gp_EulerSequence.gp_Intrinsic_ZYZ,
        };

    /// <inheritdoc/>
    public override string ToString() => IsExtrinsic ? $"Extrinsic.{Extrinsic}" : $"Intrinsic.{Intrinsic}";
}
