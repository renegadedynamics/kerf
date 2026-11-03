// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_EdgeTessellationExtractor.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_EdgeTessellationExtractor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>
/// The tessellation of an edge read back from its polygon on the face's triangulation: the triangulation's nodes and
/// the polygon's parameters (refined on the pcurve unless the edge is SameParameter).
/// </summary>
/// <remarks>
/// Only reads the triangulation and the polygon, which may be frozen (an input's mesh being reused).
/// </remarks>
public class BRepMesh_EdgeTessellationExtractor : IMeshTools_CurveTessellator
{
    private readonly BRepMesh_EdgeParameterProvider<NCollection_HArray1<double>> myProvider = new();
    private readonly Triangulation myTriangulation;
    private readonly NCollection_Array1<int> myIndices;
    private readonly Placement myLoc;

    /// <summary>An extractor of the edge's polygon on the face's triangulation.</summary>
    /// <remarks>
    /// The face must have a triangulation on which the edge has a polygon with parameters; OCCT dereferences null
    /// otherwise, and the port throws <see cref="NullReferenceException"/>, which the mesher's catches handle as OCCT's
    /// do.
    /// </remarks>
    public BRepMesh_EdgeTessellationExtractor(IMeshData_Edge theEdge, IMeshData_Face theFace)
    {
        Triangulation? aTriangulation = BRep_Tool.Triangulation(theFace.GetFace(), out myLoc);

        PolygonOnTriangulation? aPolygon =
            BRep_Tool.PolygonOnTriangulation(theEdge.GetEdge(), aTriangulation, myLoc);

        myTriangulation = aTriangulation!;
        myIndices = aPolygon!.Nodes();
        myProvider.Init(theEdge, TopAbs_FORWARD, theFace, aPolygon.Parameters()!);
    }

    /// <inheritdoc/>
    public override int PointsNb() => myIndices.Length();

    /// <inheritdoc/>
    public override bool Value(int theIndex, out Point3D thePoint, out double theParameter)
    {
        Point3D aRefPnt = myTriangulation.Node(myIndices.Value(theIndex));
        thePoint = BRepMesh_ShapeTool.UseLocation(aRefPnt, myLoc);

        theParameter = myProvider.Parameter(theIndex, thePoint);
        return true;
    }
}
