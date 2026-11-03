// Port of: src/ModelingAlgorithms/TKMesh/BRepMeshData/BRepMeshData_PCurve.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMeshData/BRepMeshData_PCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>The default discrete pcurve of an edge on a face.</summary>
public class BRepMeshData_PCurve : IMeshData_PCurve
{
    private readonly List<Point2D> myPoints2d = [];
    private readonly List<double> myParameters = [];
    private readonly List<int> myIndices = [];

    /// <summary>An empty pcurve on the discrete face, for the edge with the given orientation.</summary>
    public BRepMeshData_PCurve(IMeshData_Face theDFace, TopAbs_Orientation theOrientation)
        : base(theDFace, theOrientation)
    {
    }

    /// <inheritdoc/>
    public override void InsertPoint(int thePosition, in Point2D thePoint, double theParamOnPCurve)
    {
        myPoints2d.Insert(thePosition, thePoint);
        myParameters.Insert(thePosition, theParamOnPCurve);
        myIndices.Insert(thePosition, 0);
    }

    /// <inheritdoc/>
    public override void AddPoint(in Point2D thePoint, double theParamOnPCurve)
    {
        myPoints2d.Add(thePoint);
        myParameters.Add(theParamOnPCurve);
        myIndices.Add(0);
    }

    /// <inheritdoc/>
    public override ref Point2D GetPoint(int theIndex)
    {
        Standard_OutOfRange_Raise_if(theIndex < 0 || theIndex >= myPoints2d.Count,
                                     "BRepMeshData_PCurve::GetPoint");
        return ref BRepMeshData_Curve.at(myPoints2d, theIndex);
    }

    /// <inheritdoc/>
    public override ref int GetIndex(int theIndex)
    {
        Standard_OutOfRange_Raise_if(theIndex < 0 || theIndex >= myIndices.Count,
                                     "BRepMeshData_PCurve::GetIndex");
        return ref BRepMeshData_Curve.at(myIndices, theIndex);
    }

    /// <inheritdoc/>
    public override ref double GetParameter(int theIndex)
    {
        Standard_OutOfRange_Raise_if(theIndex < 0 || theIndex >= ParametersNb(),
                                     "BRepMeshData_PCurve::GetParameter");
        return ref BRepMeshData_Curve.at(myParameters, theIndex);
    }

    /// <inheritdoc/>
    public override int ParametersNb() => myParameters.Count;

    /// <inheritdoc/>
    public override void RemovePoint(int theIndex)
    {
        myPoints2d.RemoveAt(theIndex);
        myIndices.RemoveAt(theIndex);
        removeParameter(theIndex);
    }

    /// <inheritdoc/>
    protected override void removeParameter(int theIndex)
    {
        myParameters.RemoveAt(theIndex);
    }

    /// <inheritdoc/>
    public override void Clear(bool isKeepEndPoints)
    {
        if (!isKeepEndPoints)
        {
            myPoints2d.Clear();
            myParameters.Clear();
            myIndices.Clear();
        }
        else if (ParametersNb() > 2)
        {
            myPoints2d.RemoveRange(1, myPoints2d.Count - 2);
            myParameters.RemoveRange(1, myParameters.Count - 2);
            myIndices.RemoveRange(1, myIndices.Count - 2);
        }
    }
}
