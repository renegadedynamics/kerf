// Port of: src/FoundationClasses/TKMath/Poly/Poly_MeshPurpose.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2021-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// What a triangulation is used for: bit flags. OCCT's <c>typedef unsigned int</c> with an anonymous enum of its
/// values; here a flags enum over <c>uint</c>, so application-defined bits (from <see cref="Poly_MeshPurpose_USER"/>
/// up) are values of the enum too. Files that use it write <c>using static Occt.Poly_MeshPurpose;</c>.
/// </summary>
[Flags]
public enum Poly_MeshPurpose : uint
{
    // main flags

    /// <summary>No special use (the default).</summary>
    Poly_MeshPurpose_NONE = 0,

    /// <summary>A mesh for algorithms.</summary>
    Poly_MeshPurpose_Calculation = 0x0001,

    /// <summary>A mesh for presentation (levels of detail).</summary>
    Poly_MeshPurpose_Presentation = 0x0002,

    // special purpose bits (should not be set externally)

    /// <summary>The mesh is marked as currently active in a list.</summary>
    Poly_MeshPurpose_Active = 0x0004,

    /// <summary>The mesh has currently loaded data.</summary>
    Poly_MeshPurpose_Loaded = 0x0008,

    /// <summary>
    /// A special flag for BRep_Tools::Triangulation() to return any other defined mesh, if none matching other
    /// criteria was found.
    /// </summary>
    Poly_MeshPurpose_AnyFallback = 0x0010,

    /// <summary>The first application-defined flag; user-defined flags should have higher values.</summary>
    Poly_MeshPurpose_USER = 0x0020,
}
