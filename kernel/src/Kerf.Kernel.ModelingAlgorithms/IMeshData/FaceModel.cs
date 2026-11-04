// Port of: src/ModelingAlgorithms/TKMesh/IMeshData/IMeshData_Face.hxx
// Port of: src/ModelingAlgorithms/TKMesh/IMeshData/IMeshData_Face.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Meshing.Model.StatusModel;

namespace Kerf.Engine.Meshing.Model;

/// <summary>The discrete model of a face: one or several wires, the first of which is the outer one.</summary>
/// <remarks>
/// The face's <see cref="GetSurface"/> is one <see cref="FaceSurfaceAdaptor"/> (without restriction), which the
/// mesher's loops may evaluate from several threads (phase4-plan 2.2: porters give each thread a shallow copy where
/// needed, as decision 431).
/// </remarks>
internal abstract class FaceModel : TessellatedShapeModel, IStatusOwnerModel
{
    private readonly FaceSurfaceAdaptor _surface;
    private int _status; // IMeshData_StatusOwner

    /// <summary>An empty model of the face.</summary>
    protected FaceModel(in Face face) : base(face)
    {
        _status = (int)StatusModel.NoError;
        _surface = new FaceSurfaceAdaptor(GetFace(), false);
    }

    /// <summary>The number of wires.</summary>
    public abstract int WiresNb();

    /// <summary>Adds a wire to the discrete model of the face.</summary>
    public abstract WireModel AddWire(in Wire wire, int edgeNb = 0);

    /// <summary>The discrete wire with the given index.</summary>
    public abstract WireModel GetWire(int index);

    /// <summary>The face's surface.</summary>
    public FaceSurfaceAdaptor GetSurface() => _surface;

    /// <summary>The face of the model.</summary>
    public Face GetFace() => ShapeCasts.Face(GetShape());

    /// <summary>
    /// Kerf: the triangulation the face's mesher made, in the face's definition, until the post-processor finishes it
    /// (its deflection and parameters) and attaches it, frozen (unit 10L5). OCCT attaches it when it is made.
    /// </summary>
    internal Triangulation? MadeTriangulation { get; set; }

    /// <summary>Whether the face's discrete model is valid.</summary>
    public bool IsValid
    {
        get
        {
            return (IsEqual(StatusModel.NoError) || IsEqual(StatusModel.ReMesh)
                    || IsEqual(StatusModel.UnorientedWire));
        }
    }

    // IMeshData_StatusOwner

    /// <inheritdoc/>
    public bool IsEqual(StatusModel value) => (_status == (int)value);

    /// <inheritdoc/>
    public bool IsSet(StatusModel value) => (_status & (int)value) != 0;

    /// <inheritdoc/>
    public void SetStatus(StatusModel value) => _status |= (int)value;

    /// <inheritdoc/>
    public void UnsetStatus(StatusModel value) => _status &= ~(int)value;

    /// <inheritdoc/>
    public int GetStatusMask() => _status;
}
