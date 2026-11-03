// Port of: src/ModelingAlgorithms/TKMesh/IMeshData/IMeshData_Model.hxx
// Port of: src/ModelingAlgorithms/TKMesh/IMeshData/IMeshData_Model.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;

namespace Occt;

/// <summary>The discrete model of a shape: its discrete faces and edges, indexed from zero.</summary>
public abstract class IMeshData_Model : IMeshData_Shape
{
    /// <summary>An empty model of the shape.</summary>
    protected IMeshData_Model(in Shape theShape) : base(theShape)
    {
    }

    /// <summary>The maximum size of the shape.</summary>
    public abstract double GetMaxSize();

    // discrete faces

    /// <summary>The number of faces in the discrete model.</summary>
    public abstract int FacesNb();

    /// <summary>Adds a new face to the model.</summary>
    public abstract IMeshData_Face AddFace(in Face theFace);

    /// <summary>The model's face with the given index.</summary>
    public abstract IMeshData_Face GetFace(int theIndex);

    // discrete edges

    /// <summary>The number of edges in the discrete model.</summary>
    public abstract int EdgesNb();

    /// <summary>Adds a new edge to the model.</summary>
    public abstract IMeshData_Edge AddEdge(in Edge theEdge);

    /// <summary>The model's edge with the given index.</summary>
    public abstract IMeshData_Edge GetEdge(int theIndex);
}
