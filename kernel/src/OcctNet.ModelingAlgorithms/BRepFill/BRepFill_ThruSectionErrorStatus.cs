// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_ThruSectionErrorStatus.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2022-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Errors that can occur in the through-sections algorithm.</summary>
public enum BRepFill_ThruSectionErrorStatus
{
    /// <summary>The algorithm is done.</summary>
    BRepFill_ThruSectionErrorStatus_Done,

    /// <summary>The algorithm is not done.</summary>
    BRepFill_ThruSectionErrorStatus_NotDone,

    /// <summary>The profiles don't all have the same topology (they should be all closed or all open).</summary>
    BRepFill_ThruSectionErrorStatus_NotSameTopology,

    /// <summary>The profiles are inconsistent.</summary>
    BRepFill_ThruSectionErrorStatus_ProfilesInconsistent,

    /// <summary>Wrong use of punctual sections.</summary>
    BRepFill_ThruSectionErrorStatus_WrongUsage,

    /// <summary>An edge has no 3D curve.</summary>
    BRepFill_ThruSectionErrorStatus_Null3DCurve,

    /// <summary>The algorithm has failed.</summary>
    BRepFill_ThruSectionErrorStatus_Failed,
}
