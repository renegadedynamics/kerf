// Port of: src/ModelingAlgorithms/TKMesh/IMeshData/IMeshData_StatusOwner.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Meshing.Model;

/// <summary>
/// The status functionality of the discrete model's faces, wires and edges (OCCT's mix-in class
/// <c>IMeshData_StatusOwner</c>).
/// </summary>
/// <remarks>
/// OCCT's <see cref="FaceModel"/>, <see cref="WireModel"/> and <see cref="EdgeModel"/> derive from
/// both <see cref="TessellatedShapeModel"/> and this mix-in. C# has one base class, so the mix-in is an interface
/// and its state, the status mask (<see cref="StatusModel.NoError"/> at first), lives in each of the
/// three classes, with the same members.
/// </remarks>
internal interface IStatusOwnerModel
{
    /// <summary>True if the status is exactly the given value.</summary>
    bool IsEqual(StatusModel value);

    /// <summary>True if the given status flag is set.</summary>
    bool IsSet(StatusModel value);

    /// <summary>Adds the status to the status flags.</summary>
    void SetStatus(StatusModel value);

    /// <summary>Removes the status from the status flags.</summary>
    void UnsetStatus(StatusModel value);

    /// <summary>The complete status mask.</summary>
    int GetStatusMask();
}
