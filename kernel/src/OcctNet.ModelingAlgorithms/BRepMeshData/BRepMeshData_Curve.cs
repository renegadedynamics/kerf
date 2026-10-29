// Port of: src/ModelingAlgorithms/TKMesh/BRepMeshData/BRepMeshData_Curve.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMeshData/BRepMeshData_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.InteropServices;

namespace Occt;

/// <summary>The default discrete 3D curve of an edge.</summary>
/// <remarks>
/// OCCT's <c>std::deque</c>s on an incremental allocator are lists (the allocator argument goes). An index out of
/// range throws <see cref="ArgumentOutOfRangeException"/> where OCCT's release build reads out of bounds.
/// </remarks>
public class BRepMeshData_Curve : IMeshData_Curve
{
    private readonly List<gp_Pnt> myPoints = [];
    private readonly List<double> myParameters = [];

    /// <summary>An empty curve.</summary>
    public BRepMeshData_Curve()
    {
    }

    /// <inheritdoc/>
    public override void InsertPoint(int thePosition, in gp_Pnt thePoint, double theParamOnPCurve)
    {
        myPoints.Insert(thePosition, thePoint);
        myParameters.Insert(thePosition, theParamOnPCurve);
    }

    /// <inheritdoc/>
    public override void AddPoint(in gp_Pnt thePoint, double theParamOnPCurve)
    {
        myPoints.Add(thePoint);
        myParameters.Add(theParamOnPCurve);
    }

    /// <inheritdoc/>
    public override ref gp_Pnt GetPoint(int theIndex) => ref at(myPoints, theIndex);

    /// <inheritdoc/>
    public override ref double GetParameter(int theIndex) => ref at(myParameters, theIndex);

    /// <inheritdoc/>
    public override int ParametersNb() => myParameters.Count;

    /// <inheritdoc/>
    public override void RemovePoint(int theIndex)
    {
        myPoints.RemoveAt(theIndex);
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
            myPoints.Clear();
            myParameters.Clear();
        }
        else if (ParametersNb() > 2)
        {
            myPoints.RemoveRange(1, myPoints.Count - 2);
            myParameters.RemoveRange(1, myParameters.Count - 2);
        }
    }

    // std::deque::operator[] as a reference into the list's storage.
    internal static ref T at<T>(List<T> theList, int theIndex)
    {
        ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual((uint)theIndex, (uint)theList.Count, nameof(theIndex));
        return ref CollectionsMarshal.AsSpan(theList)[theIndex];
    }
}
