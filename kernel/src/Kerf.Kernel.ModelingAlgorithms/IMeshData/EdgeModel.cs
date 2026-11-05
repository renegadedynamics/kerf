// Port of: src/ModelingAlgorithms/TKMesh/IMeshData/IMeshData_Edge.hxx
// Port of: src/ModelingAlgorithms/TKMesh/IMeshData/IMeshData_Edge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Engine.Meshing.Model.StatusModel;

namespace Kerf.Engine.Meshing.Model;

/// <summary>
/// The discrete model of an edge: its 3D curve and its pcurves, one per discrete face (two on a seam), with the
/// edge's flags as the mesher sees them.
/// </summary>
internal abstract class EdgeModel : TessellatedShapeModel, IStatusOwnerModel
{
    private bool _sameParam;
    private bool _sameRange;
    private bool _degenerated;
    private double _angDeflection;
    private CurveModel? _curve;
    private int _status; // IMeshData_StatusOwner

    /// <summary>An empty model of the edge, with the edge's SameParameter, SameRange and Degenerated flags.</summary>
    protected EdgeModel(in Edge edge) : base(edge)
    {
        _sameParam = Tool.SameParameter(edge);
        _sameRange = Tool.SameRange(edge);
        _degenerated = Tool.Degenerated(edge);
        _angDeflection = RealLast();
        _status = (int)StatusModel.NoError;
    }

    /// <summary>The edge of the model.</summary>
    public Edge GetEdge() => ShapeCasts.Edge(GetShape());

    /// <summary>The number of pcurves of the edge.</summary>
    public abstract int PCurvesNb();

    /// <summary>Adds a discrete pcurve for the given discrete face.</summary>
    public abstract PCurveModel AddPCurve(FaceModel dFace, Orientation orientation);

    /// <summary>The pcurve for the given discrete face and orientation.</summary>
    public abstract PCurveModel GetPCurve(FaceModel dFace, Orientation orientation);

    /// <summary>The pcurve with the given index.</summary>
    public abstract PCurveModel GetPCurve(int index);

    /// <summary>The indices of the pcurves for the given discrete face.</summary>
    public abstract NCollection_List<int> GetPCurves(FaceModel dFace);

    /// <summary>Clears the curve and all the pcurves of the edge.</summary>
    public void Clear(bool isKeepEndPoints)
    {
        _curve!.Clear(isKeepEndPoints);
        for (int pCurveIt = 0; pCurveIt < PCurvesNb(); ++pCurveIt)
        {
            GetPCurve(pCurveIt).Clear(isKeepEndPoints);
        }
    }

    /// <summary>True if the edge is free, that is, it has no pcurves.</summary>
    public bool IsFree => (PCurvesNb() == 0);

    /// <summary>Sets the 3D curve of the edge.</summary>
    public void SetCurve(CurveModel? curve) => _curve = curve;

    /// <summary>The 3D curve of the edge.</summary>
    public CurveModel GetCurve() => _curve!;

    /// <summary>The angular deflection of the discrete model.</summary>
    public double GetAngularDeflection() => _angDeflection;

    /// <summary>Sets the angular deflection of the discrete model.</summary>
    public void SetAngularDeflection(double value) => _angDeflection = value;

    /// <summary>The SameParameter flag; by default the edge's.</summary>
    public bool GetSameParam() => _sameParam;

    /// <summary>Updates the SameParameter flag.</summary>
    public void SetSameParam(bool value) => _sameParam = value;

    /// <summary>The SameRange flag; by default the edge's.</summary>
    public bool GetSameRange() => _sameRange;

    /// <summary>Updates the SameRange flag.</summary>
    public void SetSameRange(bool value) => _sameRange = value;

    /// <summary>The Degenerated flag; by default the edge's.</summary>
    public bool GetDegenerated() => _degenerated;

    /// <summary>Updates the Degenerated flag.</summary>
    public void SetDegenerated(bool value) => _degenerated = value;

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
