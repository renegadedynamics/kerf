// Port of: src/FoundationClasses/TKMath/gp/gp_TrsfForm.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Geometry;

/// <summary>The kind of a <see cref="Transform3D"/> or gp_GTrsf.</summary>
/// <remarks>An unscoped enum in C++; ported code imports it with <c>using static Occt.gp_TrsfForm;</c>.</remarks>
public enum TransformForm
{
    /// <summary>No transformation (the matrix is the identity).</summary>
    Identity,
    Rotation,
    Translation,
    /// <summary>Central symmetry.</summary>
    PntMirror,
    /// <summary>Rotational symmetry.</summary>
    Ax1Mirror,
    /// <summary>Bilateral symmetry.</summary>
    Ax2Mirror,
    Scale,
    /// <summary>A combination of the above.</summary>
    CompoundTrsf,
    /// <summary>A transformation with a non-orthogonal matrix.</summary>
    Other,
}
