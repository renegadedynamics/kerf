// Port of: src/ModelingAlgorithms/TKMesh/BRepMeshData/BRepMeshData_PCurve.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMeshData/BRepMeshData_PCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing.Model;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Meshing.Data;

/// <summary>The default discrete pcurve of an edge on a face.</summary>
internal class PCurveData : PCurveModel
{
    private readonly List<Point2D> _points2d = [];
    private readonly List<double> _parameters = [];
    private readonly List<int> _indices = [];

    /// <summary>An empty pcurve on the discrete face, for the edge with the given orientation.</summary>
    public PCurveData(FaceModel dFace, Orientation orientation)
        : base(dFace, orientation)
    {
    }

    /// <inheritdoc/>
    public override void InsertPoint(int position, in Point2D point, double paramOnPCurve)
    {
        _points2d.Insert(position, point);
        _parameters.Insert(position, paramOnPCurve);
        _indices.Insert(position, 0);
    }

    /// <inheritdoc/>
    public override void AddPoint(in Point2D point, double paramOnPCurve)
    {
        _points2d.Add(point);
        _parameters.Add(paramOnPCurve);
        _indices.Add(0);
    }

    /// <inheritdoc/>
    public override ref Point2D GetPoint(int index)
    {
        Standard_OutOfRange_Raise_if(index < 0 || index >= _points2d.Count,
                                     "BRepMeshData_PCurve::GetPoint");
        return ref CurveData.At(_points2d, index);
    }

    /// <inheritdoc/>
    public override ref int GetIndex(int index)
    {
        Standard_OutOfRange_Raise_if(index < 0 || index >= _indices.Count,
                                     "BRepMeshData_PCurve::GetIndex");
        return ref CurveData.At(_indices, index);
    }

    /// <inheritdoc/>
    public override ref double GetParameter(int index)
    {
        Standard_OutOfRange_Raise_if(index < 0 || index >= ParametersNb(),
                                     "BRepMeshData_PCurve::GetParameter");
        return ref CurveData.At(_parameters, index);
    }

    /// <inheritdoc/>
    public override int ParametersNb() => _parameters.Count;

    /// <inheritdoc/>
    public override void RemovePoint(int index)
    {
        _points2d.RemoveAt(index);
        _indices.RemoveAt(index);
        RemoveParameter(index);
    }

    /// <inheritdoc/>
    protected override void RemoveParameter(int index)
    {
        _parameters.RemoveAt(index);
    }

    /// <inheritdoc/>
    public override void Clear(bool isKeepEndPoints)
    {
        if (!isKeepEndPoints)
        {
            _points2d.Clear();
            _parameters.Clear();
            _indices.Clear();
        }
        else if (ParametersNb() > 2)
        {
            _points2d.RemoveRange(1, _points2d.Count - 2);
            _parameters.RemoveRange(1, _parameters.Count - 2);
            _indices.RemoveRange(1, _indices.Count - 2);
        }
    }
}
