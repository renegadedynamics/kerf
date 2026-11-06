using Kerf.Geometry;

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
            Kerf.Modeling.Extrinsic.XYZ => EulerSequence.ExtrinsicXYZ,
            Kerf.Modeling.Extrinsic.XZY => EulerSequence.ExtrinsicXZY,
            Kerf.Modeling.Extrinsic.YZX => EulerSequence.ExtrinsicYZX,
            Kerf.Modeling.Extrinsic.YXZ => EulerSequence.ExtrinsicYXZ,
            Kerf.Modeling.Extrinsic.ZXY => EulerSequence.ExtrinsicZXY,
            Kerf.Modeling.Extrinsic.ZYX => EulerSequence.ExtrinsicZYX,
            Kerf.Modeling.Extrinsic.XYX => EulerSequence.ExtrinsicXYX,
            Kerf.Modeling.Extrinsic.XZX => EulerSequence.ExtrinsicXZX,
            Kerf.Modeling.Extrinsic.YZY => EulerSequence.ExtrinsicYZY,
            Kerf.Modeling.Extrinsic.YXY => EulerSequence.ExtrinsicYXY,
            Kerf.Modeling.Extrinsic.ZXZ => EulerSequence.ExtrinsicZXZ,
            _ => EulerSequence.ExtrinsicZYZ,
        }
        : (Kerf.Modeling.Intrinsic)_value switch
        {
            Kerf.Modeling.Intrinsic.XYZ => EulerSequence.IntrinsicXYZ,
            Kerf.Modeling.Intrinsic.XZY => EulerSequence.IntrinsicXZY,
            Kerf.Modeling.Intrinsic.YZX => EulerSequence.IntrinsicYZX,
            Kerf.Modeling.Intrinsic.YXZ => EulerSequence.IntrinsicYXZ,
            Kerf.Modeling.Intrinsic.ZXY => EulerSequence.IntrinsicZXY,
            Kerf.Modeling.Intrinsic.ZYX => EulerSequence.IntrinsicZYX,
            Kerf.Modeling.Intrinsic.XYX => EulerSequence.IntrinsicXYX,
            Kerf.Modeling.Intrinsic.XZX => EulerSequence.IntrinsicXZX,
            Kerf.Modeling.Intrinsic.YZY => EulerSequence.IntrinsicYZY,
            Kerf.Modeling.Intrinsic.YXY => EulerSequence.IntrinsicYXY,
            Kerf.Modeling.Intrinsic.ZXZ => EulerSequence.IntrinsicZXZ,
            _ => EulerSequence.IntrinsicZYZ,
        };

    /// <inheritdoc/>
    public override string ToString() => IsExtrinsic ? $"Extrinsic.{Extrinsic}" : $"Intrinsic.{Intrinsic}";
}
