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
    /// <summary>Rotations about the fixed X, then Y, then Z axes.</summary>
    ExtrinsicXYZ,
    /// <summary>Rotations about the fixed X, then Z, then Y axes.</summary>
    ExtrinsicXZY,
    /// <summary>Rotations about the fixed Y, then Z, then X axes.</summary>
    ExtrinsicYZX,
    /// <summary>Rotations about the fixed Y, then X, then Z axes.</summary>
    ExtrinsicYXZ,
    /// <summary>Rotations about the fixed Z, then X, then Y axes.</summary>
    ExtrinsicZXY,
    /// <summary>Rotations about the fixed Z, then Y, then X axes.</summary>
    ExtrinsicZYX,
    /// <summary>Rotations about X, then the rotated Y, then the twice-rotated Z.</summary>
    IntrinsicXYZ,
    /// <summary>Rotations about X, then the rotated Z, then the twice-rotated Y.</summary>
    IntrinsicXZY,
    /// <summary>Rotations about Y, then the rotated Z, then the twice-rotated X.</summary>
    IntrinsicYZX,
    /// <summary>Rotations about Y, then the rotated X, then the twice-rotated Z.</summary>
    IntrinsicYXZ,
    /// <summary>Rotations about Z, then the rotated X, then the twice-rotated Y.</summary>
    IntrinsicZXY,
    /// <summary>Rotations about Z, then the rotated Y, then the twice-rotated X.</summary>
    IntrinsicZYX,
    /// <summary>Rotations about the fixed X, then Y, then X axes.</summary>
    ExtrinsicXYX,
    /// <summary>Rotations about the fixed X, then Z, then X axes.</summary>
    ExtrinsicXZX,
    /// <summary>Rotations about the fixed Y, then Z, then Y axes.</summary>
    ExtrinsicYZY,
    /// <summary>Rotations about the fixed Y, then X, then Y axes.</summary>
    ExtrinsicYXY,
    /// <summary>Rotations about the fixed Z, then Y, then Z axes.</summary>
    ExtrinsicZYZ,
    /// <summary>Rotations about the fixed Z, then X, then Z axes.</summary>
    ExtrinsicZXZ,
    /// <summary>Rotations about X, then the rotated Y, then the twice-rotated X.</summary>
    IntrinsicXYX,
    /// <summary>Rotations about X, then the rotated Z, then the twice-rotated X.</summary>
    IntrinsicXZX,
    /// <summary>Rotations about Y, then the rotated Z, then the twice-rotated Y.</summary>
    IntrinsicYZY,
    /// <summary>Rotations about Y, then the rotated X, then the twice-rotated Y.</summary>
    IntrinsicYXY,
    /// <summary>Rotations about Z, then the rotated X, then the twice-rotated Z.</summary>
    IntrinsicZXZ,
    /// <summary>Rotations about Z, then the rotated Y, then the twice-rotated Z.</summary>
    IntrinsicZYZ,
}
