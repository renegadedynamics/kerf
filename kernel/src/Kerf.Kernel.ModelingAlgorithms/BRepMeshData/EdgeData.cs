// Port of: src/ModelingAlgorithms/TKMesh/BRepMeshData/BRepMeshData_Edge.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMeshData/BRepMeshData_Edge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Meshing.Model;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Meshing.Data;

/// <summary>The default discrete edge.</summary>
/// <remarks>The pcurves of a face are found by the discrete face's reference (OCCT: its address).</remarks>
internal class EdgeData : EdgeModel
{
    private readonly List<PCurveModel> _pCurves;
    private readonly Dictionary<FaceModel, NCollection_List<int>> _pCurvesMap;

    /// <summary>A discrete edge of the edge, with an empty 3D curve.</summary>
    public EdgeData(in Edge edge) : base(edge)
    {
        _pCurves = new List<PCurveModel>();
        _pCurvesMap = new Dictionary<FaceModel, NCollection_List<int>>(ReferenceEqualityComparer.Instance);
        SetCurve(new CurveData());
    }

    /// <inheritdoc/>
    public override int PCurvesNb() => _pCurves.Count;

    /// <inheritdoc/>
    public override PCurveModel AddPCurve(FaceModel dFace, Orientation orientation)
    {
        int pCurveIndex = PCurvesNb();
        // Add pcurve to list of pcurves
        PCurveModel pCurve = new PCurveData(dFace, orientation);
        _pCurves.Add(pCurve);

        // Map pcurve to faces.
        if (!_pCurvesMap.ContainsKey(dFace))
        {
            _pCurvesMap[dFace] = new NCollection_List<int>();
        }

        NCollection_List<int> listOfPCurves = _pCurvesMap.GetValue(dFace);
        listOfPCurves.Append(pCurveIndex);

        return GetPCurve(pCurveIndex);
    }

    /// <inheritdoc/>
    public override PCurveModel GetPCurve(FaceModel dFace, Orientation orientation)
    {
        NCollection_List<int> listOfPCurves = _pCurvesMap.GetValue(dFace);
        PCurveModel pCurve1 = _pCurves[listOfPCurves.First()];
        return (pCurve1.GetOrientation() == orientation) ? pCurve1 : _pCurves[listOfPCurves.Last()];
    }

    /// <inheritdoc/>
    public override PCurveModel GetPCurve(int index) => _pCurves[index];

    /// <inheritdoc/>
    public override NCollection_List<int> GetPCurves(FaceModel dFace) => _pCurvesMap.GetValue(dFace);
}
