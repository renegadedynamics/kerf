// Port of: src/FoundationClasses/TKMath/gp/gp_EulerSequence.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The order of rotations for Euler angles and other three-angle representations of a rotation.</summary>
/// <remarks>An unscoped enum in C++; ported code imports it with <c>using static Occt.gp_EulerSequence;</c>.</remarks>
public enum gp_EulerSequence
{
    /// <summary>Classic Euler angles, alias for Intrinsic_ZXZ.</summary>
    gp_EulerAngles,
    /// <summary>Yaw Pitch Roll (or nautical) angles, alias for Intrinsic_ZYX.</summary>
    gp_YawPitchRoll,
    gp_Extrinsic_XYZ,
    gp_Extrinsic_XZY,
    gp_Extrinsic_YZX,
    gp_Extrinsic_YXZ,
    gp_Extrinsic_ZXY,
    gp_Extrinsic_ZYX,
    gp_Intrinsic_XYZ,
    gp_Intrinsic_XZY,
    gp_Intrinsic_YZX,
    gp_Intrinsic_YXZ,
    gp_Intrinsic_ZXY,
    gp_Intrinsic_ZYX,
    gp_Extrinsic_XYX,
    gp_Extrinsic_XZX,
    gp_Extrinsic_YZY,
    gp_Extrinsic_YXY,
    gp_Extrinsic_ZYZ,
    gp_Extrinsic_ZXZ,
    gp_Intrinsic_XYX,
    gp_Intrinsic_XZX,
    gp_Intrinsic_YZY,
    gp_Intrinsic_YXY,
    gp_Intrinsic_ZXZ,
    gp_Intrinsic_ZYZ,
}
