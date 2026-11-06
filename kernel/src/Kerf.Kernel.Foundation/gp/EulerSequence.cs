// Port of: src/FoundationClasses/TKMath/gp/gp_EulerSequence.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Geometry;

/// <summary>The order of rotations for Euler angles and other three-angle representations of a rotation.</summary>
public enum EulerSequence
{
    /// <summary>Classic Euler angles, alias for IntrinsicZXZ.</summary>
    EulerAngles,
    /// <summary>Yaw Pitch Roll (or nautical) angles, alias for IntrinsicZYX.</summary>
    YawPitchRoll,
    ExtrinsicXYZ,
    ExtrinsicXZY,
    ExtrinsicYZX,
    ExtrinsicYXZ,
    ExtrinsicZXY,
    ExtrinsicZYX,
    IntrinsicXYZ,
    IntrinsicXZY,
    IntrinsicYZX,
    IntrinsicYXZ,
    IntrinsicZXY,
    IntrinsicZYX,
    ExtrinsicXYX,
    ExtrinsicXZX,
    ExtrinsicYZY,
    ExtrinsicYXY,
    ExtrinsicZYZ,
    ExtrinsicZXZ,
    IntrinsicXYX,
    IntrinsicXZX,
    IntrinsicYZY,
    IntrinsicYXY,
    IntrinsicZXZ,
    IntrinsicZYZ,
}
