// Port of: src/ModelingAlgorithms/TKMesh/IMeshData/IMeshData_StatusOwner.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The status functionality of the discrete model's faces, wires and edges (OCCT's mix-in class
/// <c>IMeshData_StatusOwner</c>).
/// </summary>
/// <remarks>
/// OCCT's <see cref="IMeshData_Face"/>, <see cref="IMeshData_Wire"/> and <see cref="IMeshData_Edge"/> derive from
/// both <see cref="IMeshData_TessellatedShape"/> and this mix-in. C# has one base class, so the mix-in is an interface
/// and its state, the status mask (<see cref="IMeshData_Status.IMeshData_NoError"/> at first), lives in each of the
/// three classes, with the same members.
/// </remarks>
public interface IMeshData_StatusOwner
{
    /// <summary>True if the status is exactly the given value.</summary>
    bool IsEqual(IMeshData_Status theValue);

    /// <summary>True if the given status flag is set.</summary>
    bool IsSet(IMeshData_Status theValue);

    /// <summary>Adds the status to the status flags.</summary>
    void SetStatus(IMeshData_Status theValue);

    /// <summary>Removes the status from the status flags.</summary>
    void UnsetStatus(IMeshData_Status theValue);

    /// <summary>The complete status mask.</summary>
    int GetStatusMask();
}
