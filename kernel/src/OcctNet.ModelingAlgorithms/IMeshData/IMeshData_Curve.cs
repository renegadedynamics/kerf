// Port of: src/ModelingAlgorithms/TKMesh/IMeshData/IMeshData_Curve.hxx
// Port of: src/ModelingAlgorithms/TKMesh/IMeshData/IMeshData_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>The discrete 3D curve of an edge: points with their parameters, indexed from zero.</summary>
public abstract class IMeshData_Curve : IMeshData_ParametersList
{
    /// <summary>An empty curve.</summary>
    protected IMeshData_Curve()
    {
    }

    /// <summary>Inserts a discretization point at the given position.</summary>
    public abstract void InsertPoint(int thePosition, in Point3D thePoint, double theParamOnPCurve);

    /// <summary>Adds a discretization point.</summary>
    public abstract void AddPoint(in Point3D thePoint, double theParamOnCurve);

    /// <summary>The discretization point with the given index, for reading and writing.</summary>
    public abstract ref Point3D GetPoint(int theIndex);

    /// <summary>Removes the point with the given index.</summary>
    public abstract void RemovePoint(int theIndex);
}
