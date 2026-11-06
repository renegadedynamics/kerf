// Port of: src/FoundationClasses/TKMath/gp/gp_TrsfForm.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Geometry;

/// <summary>The kind of a <see cref="Transform3D"/> or gp_GTrsf.</summary>
public enum TransformForm
{
    /// <summary>No transformation (the matrix is the identity).</summary>
    Identity,
    /// <summary>A rotation about a point in 2D, or an axis in 3D.</summary>
    Rotation,
    /// <summary>A translation.</summary>
    Translation,
    /// <summary>The central symmetry through a point.</summary>
    PointMirror,
    /// <summary>A half-turn about an axis in 3D, or a reflection in a line in 2D.</summary>
    AxisMirror,
    /// <summary>A reflection in a plane.</summary>
    PlaneMirror,
    /// <summary>A uniform scale about a point.</summary>
    Scale,
    /// <summary>A combination of the above.</summary>
    Composite,
    /// <summary>A transformation with a non-orthogonal matrix.</summary>
    Other,
}
