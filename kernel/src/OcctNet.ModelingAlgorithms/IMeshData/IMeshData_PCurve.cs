// Port of: src/ModelingAlgorithms/TKMesh/IMeshData/IMeshData_PCurve.hxx
// Port of: src/ModelingAlgorithms/TKMesh/IMeshData/IMeshData_PCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;

namespace Occt;

/// <summary>
/// The pcurve of an edge on a discrete face: points in the face's parametric space with their parameters and their
/// indices in the face's mesh, indexed from zero.
/// </summary>
public abstract class IMeshData_PCurve : IMeshData_ParametersList
{
    private readonly IMeshData_Face myDFace;
    private readonly Orientation myOrientation;

    /// <summary>An empty pcurve on the discrete face, for the edge with the given orientation.</summary>
    protected IMeshData_PCurve(IMeshData_Face theDFace, Orientation theOrientation)
    {
        myDFace = theDFace;
        myOrientation = theOrientation;
    }

    /// <summary>Inserts a discretization point at the given position.</summary>
    public abstract void InsertPoint(int thePosition, in Point2D thePoint, double theParamOnPCurve);

    /// <summary>Adds a discretization point.</summary>
    public abstract void AddPoint(in Point2D thePoint, double theParamOnPCurve);

    /// <summary>The discretization point with the given index, for reading and writing.</summary>
    public abstract ref Point2D GetPoint(int theIndex);

    /// <summary>The index in the mesh of the discretization point with the given index, for reading and writing.</summary>
    public abstract ref int GetIndex(int theIndex);

    /// <summary>Removes the point with the given index.</summary>
    public abstract void RemovePoint(int theIndex);

    /// <summary>True unless the edge is reversed on the face.</summary>
    public bool IsForward() => (myOrientation != Reversed);

    /// <summary>True if the edge is internal to the face.</summary>
    public bool IsInternal() => (myOrientation == Internal);

    /// <summary>The orientation of the edge on the face.</summary>
    public Orientation GetOrientation() => myOrientation;

    /// <summary>The discrete face of the pcurve.</summary>
    public IMeshData_Face GetFace() => myDFace;
}
