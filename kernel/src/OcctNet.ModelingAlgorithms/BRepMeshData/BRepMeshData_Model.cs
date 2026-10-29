// Port of: src/ModelingAlgorithms/TKMesh/BRepMeshData/BRepMeshData_Model.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMeshData/BRepMeshData_Model.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The default discrete model of a shape.</summary>
/// <remarks>
/// OCCT's model owns a thread-safe incremental allocator for all its entities; the port's entities are ordinary
/// objects. <see cref="AddFace"/> and <see cref="AddEdge"/> aren't thread-safe, as in OCCT: the mesher adds them
/// before its parallel loops.
/// </remarks>
public class BRepMeshData_Model : IMeshData_Model
{
    private double myMaxSize;
    private readonly NCollection_DynamicArray<IMeshData_Face> myDFaces;
    private readonly NCollection_DynamicArray<IMeshData_Edge> myDEdges;

    /// <summary>An empty model of the shape.</summary>
    public BRepMeshData_Model(in TopoDS_Shape theShape) : base(theShape)
    {
        myMaxSize = 0.0;
        myDFaces = new NCollection_DynamicArray<IMeshData_Face>(256);
        myDEdges = new NCollection_DynamicArray<IMeshData_Edge>(256);
    }

    /// <summary>The maximum size of the shape's bounding box.</summary>
    public override double GetMaxSize() => myMaxSize;

    /// <summary>Sets the maximum size of the shape's bounding box.</summary>
    public void SetMaxSize(double theValue) => myMaxSize = theValue;

    /// <inheritdoc/>
    public override int FacesNb() => myDFaces.Length();

    /// <inheritdoc/>
    public override IMeshData_Face AddFace(in TopoDS_Face theFace)
    {
        IMeshData_Face aFace = new BRepMeshData_Face(theFace);
        return myDFaces.Append(aFace);
    }

    /// <inheritdoc/>
    public override IMeshData_Face GetFace(int theIndex) => myDFaces.Value(theIndex);

    /// <inheritdoc/>
    public override int EdgesNb() => myDEdges.Length();

    /// <inheritdoc/>
    public override IMeshData_Edge AddEdge(in TopoDS_Edge theEdge)
    {
        IMeshData_Edge aEdge = new BRepMeshData_Edge(theEdge);
        return myDEdges.Append(aEdge);
    }

    /// <inheritdoc/>
    public override IMeshData_Edge GetEdge(int theIndex) => myDEdges.Value(theIndex);
}
