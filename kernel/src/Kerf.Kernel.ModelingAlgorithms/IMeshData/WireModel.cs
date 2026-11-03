// Port of: src/ModelingAlgorithms/TKMesh/IMeshData/IMeshData_Wire.hxx
// Port of: src/ModelingAlgorithms/TKMesh/IMeshData/IMeshData_Wire.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Meshing.Model.StatusModel;

namespace Kerf.Engine.Meshing.Model;

/// <summary>The discrete model of a wire: an ordered chain of discrete edges with their orientations.</summary>
public abstract class WireModel : TessellatedShapeModel, IStatusOwnerModel
{
    private int _status; // IMeshData_StatusOwner

    /// <summary>An empty model of the wire.</summary>
    protected WireModel(in Wire wire) : base(wire)
    {
        _status = (int)StatusModel.NoError;
    }

    /// <summary>The wire of the model.</summary>
    public Wire GetWire() => ShapeCasts.Wire(GetShape());

    /// <summary>The number of edges.</summary>
    public abstract int EdgesNb();

    /// <summary>Adds a discrete edge with the given orientation to the wire's chain; returns its index.</summary>
    public abstract int AddEdge(EdgeModel dEdge, Orientation orientation);

    /// <summary>The discrete edge with the given index.</summary>
    public abstract EdgeModel GetEdge(int index);

    /// <summary>The orientation of the discrete edge with the given index.</summary>
    public abstract Orientation GetEdgeOrientation(int index);

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
