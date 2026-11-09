// Port of: src/ModelingAlgorithms/TKMesh/BRepMeshData/BRepMeshData_Curve.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMeshData/BRepMeshData_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing.Model;
using Kerf.Geometry;
using System.Runtime.InteropServices;

namespace Kerf.Engine.Meshing.Data;

/// <summary>The default discrete 3D curve of an edge.</summary>
/// <remarks>
/// OCCT's <c>std::deque</c>s on an incremental allocator are lists (the allocator argument goes). An index out of
/// range throws <see cref="ArgumentOutOfRangeException"/> where OCCT's release build reads out of bounds.
/// </remarks>
internal class CurveData : CurveModel
{
    private readonly List<Point3D> _points = [];
    private readonly List<double> _parameters = [];

    /// <summary>An empty curve.</summary>
    public CurveData()
    {
    }

    /// <inheritdoc/>
    public override void InsertPoint(int position, in Point3D point, double paramOnPCurve)
    {
        _points.Insert(position, point);
        _parameters.Insert(position, paramOnPCurve);
    }

    /// <inheritdoc/>
    public override void AddPoint(in Point3D point, double paramOnPCurve)
    {
        _points.Add(point);
        _parameters.Add(paramOnPCurve);
    }

    /// <inheritdoc/>
    public override ref Point3D GetPoint(int index) => ref At(_points, index);

    /// <inheritdoc/>
    public override ref double GetParameter(int index) => ref At(_parameters, index);

    /// <inheritdoc/>
    public override int ParametersNb() => _parameters.Count;

    /// <inheritdoc/>
    public override void RemovePoint(int index)
    {
        _points.RemoveAt(index);
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
            _points.Clear();
            _parameters.Clear();
        }
        else if (ParametersNb() > 2)
        {
            _points.RemoveRange(1, _points.Count - 2);
            _parameters.RemoveRange(1, _parameters.Count - 2);
        }
    }

    // std::deque::operator[] as a reference into the list's storage.
    internal static ref T At<T>(List<T> list, int index)
    {
        KernelArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual((uint)index, (uint)list.Count, nameof(index));
        return ref CollectionsMarshal.AsSpan(list)[index];
    }
}
