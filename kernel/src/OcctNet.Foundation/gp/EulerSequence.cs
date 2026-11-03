// Port of: src/FoundationClasses/TKMath/gp/gp_EulerSequence.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Geometry;

/// <summary>The order of rotations for Euler angles and other three-angle representations of a rotation.</summary>
/// <remarks>An unscoped enum in C++; ported code imports it with <c>using static Occt.gp_EulerSequence;</c>.</remarks>
public enum EulerSequence
{
    /// <summary>Classic Euler angles, alias for Intrinsic_ZXZ.</summary>
    EulerAngles,
    /// <summary>Yaw Pitch Roll (or nautical) angles, alias for Intrinsic_ZYX.</summary>
    YawPitchRoll,
    Extrinsic_XYZ,
    Extrinsic_XZY,
    Extrinsic_YZX,
    Extrinsic_YXZ,
    Extrinsic_ZXY,
    Extrinsic_ZYX,
    Intrinsic_XYZ,
    Intrinsic_XZY,
    Intrinsic_YZX,
    Intrinsic_YXZ,
    Intrinsic_ZXY,
    Intrinsic_ZYX,
    Extrinsic_XYX,
    Extrinsic_XZX,
    Extrinsic_YZY,
    Extrinsic_YXY,
    Extrinsic_ZYZ,
    Extrinsic_ZXZ,
    Intrinsic_XYX,
    Intrinsic_XZX,
    Intrinsic_YZY,
    Intrinsic_YXY,
    Intrinsic_ZXZ,
    Intrinsic_ZYZ,
}
