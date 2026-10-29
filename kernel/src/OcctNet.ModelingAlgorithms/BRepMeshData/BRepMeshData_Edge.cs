// Port of: src/ModelingAlgorithms/TKMesh/BRepMeshData/BRepMeshData_Edge.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMeshData/BRepMeshData_Edge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The default discrete edge.</summary>
/// <remarks>The pcurves of a face are found by the discrete face's reference (OCCT: its address).</remarks>
public class BRepMeshData_Edge : IMeshData_Edge
{
    private readonly NCollection_DynamicArray<IMeshData_PCurve> myPCurves;
    private readonly NCollection_DataMap<IMeshData_Face, NCollection_List<int>> myPCurvesMap;

    /// <summary>A discrete edge of the edge, with an empty 3D curve.</summary>
    public BRepMeshData_Edge(in TopoDS_Edge theEdge) : base(theEdge)
    {
        myPCurves = new NCollection_DynamicArray<IMeshData_PCurve>(256);
        myPCurvesMap = new NCollection_DataMap<IMeshData_Face, NCollection_List<int>>(ReferenceEqualityComparer.Instance);
        SetCurve(new BRepMeshData_Curve());
    }

    /// <inheritdoc/>
    public override int PCurvesNb() => myPCurves.Length();

    /// <inheritdoc/>
    public override IMeshData_PCurve AddPCurve(IMeshData_Face theDFace, TopAbs_Orientation theOrientation)
    {
        int aPCurveIndex = PCurvesNb();
        // Add pcurve to list of pcurves
        IMeshData_PCurve aPCurve = new BRepMeshData_PCurve(theDFace, theOrientation);
        myPCurves.Append(aPCurve);

        // Map pcurve to faces.
        if (!myPCurvesMap.IsBound(theDFace))
        {
            myPCurvesMap.Bind(theDFace, new NCollection_List<int>());
        }

        NCollection_List<int> aListOfPCurves = myPCurvesMap.ChangeFind(theDFace);
        aListOfPCurves.Append(aPCurveIndex);

        return GetPCurve(aPCurveIndex);
    }

    /// <inheritdoc/>
    public override IMeshData_PCurve GetPCurve(IMeshData_Face theDFace, TopAbs_Orientation theOrientation)
    {
        NCollection_List<int> aListOfPCurves = myPCurvesMap.Find(theDFace);
        IMeshData_PCurve aPCurve1 = myPCurves.Value(aListOfPCurves.First());
        return (aPCurve1.GetOrientation() == theOrientation) ? aPCurve1 : myPCurves.Value(aListOfPCurves.Last());
    }

    /// <inheritdoc/>
    public override IMeshData_PCurve GetPCurve(int theIndex) => myPCurves.Value(theIndex);

    /// <inheritdoc/>
    public override NCollection_List<int> GetPCurves(IMeshData_Face theDFace) => myPCurvesMap.Find(theDFace);
}
