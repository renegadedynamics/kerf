// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_ThruSectionErrorStatus.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2022-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Construction.Filling;

/// <summary>Errors that can occur in the through-sections algorithm.</summary>
internal enum ThruSectionErrorStatus
{
    /// <summary>The algorithm is done.</summary>
    ThruSectionErrorStatus_Done,

    /// <summary>The algorithm is not done.</summary>
    ThruSectionErrorStatus_NotDone,

    /// <summary>The profiles don't all have the same topology (they should be all closed or all open).</summary>
    ThruSectionErrorStatus_NotSameTopology,

    /// <summary>The profiles are inconsistent.</summary>
    ThruSectionErrorStatus_ProfilesInconsistent,

    /// <summary>Wrong use of punctual sections.</summary>
    ThruSectionErrorStatus_WrongUsage,

    /// <summary>An edge has no 3D curve.</summary>
    ThruSectionErrorStatus_Null3DCurve,

    /// <summary>The algorithm has failed.</summary>
    ThruSectionErrorStatus_Failed,
}
