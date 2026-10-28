// Port of: src/FoundationClasses/TKMath/gp/gp_TrsfForm.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The kind of a <see cref="gp_Trsf"/> or gp_GTrsf.</summary>
/// <remarks>An unscoped enum in C++; ported code imports it with <c>using static Occt.gp_TrsfForm;</c>.</remarks>
public enum gp_TrsfForm
{
    /// <summary>No transformation (the matrix is the identity).</summary>
    gp_Identity,
    gp_Rotation,
    gp_Translation,
    /// <summary>Central symmetry.</summary>
    gp_PntMirror,
    /// <summary>Rotational symmetry.</summary>
    gp_Ax1Mirror,
    /// <summary>Bilateral symmetry.</summary>
    gp_Ax2Mirror,
    gp_Scale,
    /// <summary>A combination of the above.</summary>
    gp_CompoundTrsf,
    /// <summary>A transformation with a non-orthogonal matrix.</summary>
    gp_Other,
}
