// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_EdgeTessellationExtractor.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_EdgeTessellationExtractor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing;
using Kerf.Engine.Meshing.Model;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Meshing;

/// <summary>
/// The tessellation of an edge read back from its polygon on the face's triangulation: the triangulation's nodes and
/// the polygon's parameters (refined on the pcurve unless the edge is SameParameter).
/// </summary>
/// <remarks>
/// Only reads the triangulation and the polygon, which may be frozen (an input's mesh being reused).
/// </remarks>
public class EdgeTessellationExtractor : CurveTessellator
{
    private readonly EdgeParameterProvider<NCollection_HArray1<double>> _provider = new();
    private readonly Triangulation _triangulation;
    private readonly NCollection_Array1<int> _indices;
    private readonly Placement _loc;

    /// <summary>An extractor of the edge's polygon on the face's triangulation.</summary>
    /// <remarks>
    /// The face must have a triangulation on which the edge has a polygon with parameters; OCCT dereferences null
    /// otherwise, and the port throws <see cref="NullReferenceException"/>, which the mesher's catches handle as OCCT's
    /// do.
    /// </remarks>
    public EdgeTessellationExtractor(EdgeModel edge, FaceModel face)
    {
        Triangulation? triangulation = Tool.Triangulation(face.GetFace(), out _loc);

        PolygonOnTriangulation? polygon =
            Tool.PolygonOnTriangulation(edge.GetEdge(), triangulation, _loc);

        _triangulation = triangulation!;
        _indices = polygon!.Nodes();
        _provider.Init(edge, Forward, face, polygon.Parameters()!);
    }

    /// <inheritdoc/>
    public override int PointsNb() => _indices.Length();

    /// <inheritdoc/>
    public override bool Value(int index, out Point3D point, out double parameter)
    {
        Point3D refPnt = _triangulation.Node(_indices.Value(index));
        point = ShapeTool.UseLocation(refPnt, _loc);

        parameter = _provider.Parameter(index, point);
        return true;
    }
}
