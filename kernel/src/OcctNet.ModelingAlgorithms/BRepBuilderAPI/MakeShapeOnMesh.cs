// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeShapeOnMesh.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeShapeOnMesh.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2018-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Construction.Api;

/// <summary>
/// Builds a shape from a triangulation: a compound of planar faces, one per triangle, sharing their vertices (one per
/// mesh node) and edges (one per mesh link). Degenerate triangles (repeated nodes, or two nodes closer than
/// <see cref="Constants.Resolution"/>) are skipped.
/// </summary>
/// <remarks>
/// The result is published frozen by <see cref="MakeShape.Shape"/>; the triangulation is only read.
/// OCCT's map of mesh links is a hash map with insertion-ordered indices (NCollection_IndexedDataMap), as the port's.
/// </remarks>
public class MakeShapeOnMesh : MakeShape
{
    private readonly Triangulation? _mesh;

    /// <summary>Sets the mesh to process.</summary>
    public MakeShapeOnMesh(Triangulation? mesh)
    {
        _mesh = mesh;
    }

    // A mesh link: its node indices, the lower first
    private readonly struct Edge : IEquatable<Edge>
    {
        public Edge(int theIdx1, int theIdx2)
        {
            Idx1 = Math.Min(theIdx1, theIdx2);
            Idx2 = Math.Max(theIdx1, theIdx2);
        }

        // First index. It is lower or equal than the second.
        public readonly int Idx1;

        // Second index.
        public readonly int Idx2;

        public bool Equals(Edge other) => other.Idx1 == Idx1 && other.Idx2 == Idx2;

        public override bool Equals(object? obj) => obj is Edge other && Equals(other);

        public override int GetHashCode() => HashCode.Combine(Idx1, Idx2);
    }

    /// <summary>Builds the shape on the mesh. Does nothing for a null or empty mesh, or on a user break.</summary>
    public override void Build(ProgressRange range = default)
    {
        // Generally, this method guarantees topology sharing by mapping mesh primitives
        // into topological counterparts.
        // mesh points -> topological vertices
        // mesh edges  -> topological edges

        // Cannot reconstruct anything from null or empty mesh.
        if (_mesh is null || _mesh.NodeCount == 0 || _mesh.TriangleCount == 0)
        {
            return;
        }

        int nbNodes = _mesh.NodeCount;
        int nbTriangles = _mesh.TriangleCount;

        // We are going to have three loops: iterate once over nodes and iterate twice
        // over triangles of input mesh.
        using var ps = new ProgressScope(range, "Per-facet shape construction", (double)nbNodes + 2.0 * nbTriangles);

        // Build shared vertices.
        var pnt2VertexMap = new NCollection_IndexedDataMap<int, Vertex>();

        for (int i = 1; i <= nbNodes; ++i)
        {
            ps.Next();
            if (ps.UserBreak())
            {
                return;
            }

            Point3D p = _mesh.Node(i);
            Vertex v = new MakeVertex(p).InternalVertex();
            pnt2VertexMap.Add(i, v);
        }

        // Build shared edges.
        var edgeToTEgeMap = new NCollection_IndexedDataMap<Edge, global::Kerf.Topology.Edge>();
        for (int i = 1; i <= nbTriangles; ++i)
        {
            ps.Next();
            if (ps.UserBreak())
            {
                return;
            }

            Triangle1 triangle = _mesh.Triangle(i);
            triangle.Get(out int idx0, out int idx1, out int idx2);

            // Skip degenerated triangles.
            if (idx0 == idx1 || idx0 == idx2 || idx1 == idx2)
            {
                continue;
            }

            Point3D p1 = _mesh.Node(idx0);
            Point3D p2 = _mesh.Node(idx1);
            Point3D p3 = _mesh.Node(idx2);
            double d1 = p1.SquareDistance(p2);
            double d2 = p1.SquareDistance(p3);
            double d3 = p2.SquareDistance(p3);
            if (d1 < Constants.Resolution() || d2 < Constants.Resolution() || d3 < Constants.Resolution())
            {
                continue;
            }

            // Edges are constructed in forward order for the existing normals orientation.
            // In Poly_Triangulation, positive direction is defined as cross product:
            // (aV1, aV2) x (aV1, aV3).
            Vertex v1 = pnt2VertexMap.FindFromKey(idx0);
            Vertex v2 = pnt2VertexMap.FindFromKey(idx1);
            Vertex v3 = pnt2VertexMap.FindFromKey(idx2);

            var meshEdge1 = new Edge(idx0, idx1);
            var meshEdge2 = new Edge(idx1, idx2);
            var meshEdge3 = new Edge(idx2, idx0);

            var maker1 = new MakeEdge(v1, v2);
            var maker2 = new MakeEdge(v2, v3);
            var maker3 = new MakeEdge(v3, v1);

            global::Kerf.Topology.Edge te1 = maker1.InternalEdge();
            if (idx1 < idx0)
            {
                te1.Reverse();
            }

            global::Kerf.Topology.Edge te2 = maker2.InternalEdge();
            if (idx2 < idx1)
            {
                te2.Reverse();
            }

            global::Kerf.Topology.Edge te3 = maker3.InternalEdge();
            if (idx0 < idx2)
            {
                te3.Reverse();
            }

            edgeToTEgeMap.Add(meshEdge1, te1);
            edgeToTEgeMap.Add(meshEdge2, te2);
            edgeToTEgeMap.Add(meshEdge3, te3);
        }

        // Construct planar faces using shared topology.
        Compound result = new();
        BrepBuilder bb = new();
        bb.MakeCompound(ref result);
        for (int i = 1; i <= nbTriangles; ++i)
        {
            ps.Next();
            if (ps.UserBreak())
            {
                return;
            }

            Triangle1 triangle = _mesh.Triangle(i);
            triangle.Get(out int idx0, out int idx1, out int idx2);

            var meshEdge1 = new Edge(idx0, idx1);
            var meshEdge2 = new Edge(idx1, idx2);
            var meshEdge3 = new Edge(idx2, idx0);
            bool isReversed1 = idx1 < idx0;
            bool isReversed2 = idx2 < idx1;
            bool isReversed3 = idx0 < idx2;

            // Edges can be skipped in case of mesh defects - topologically or geometrically
            // degenerated triangles.
            bool hasAllEdges = edgeToTEgeMap.Contains(meshEdge1)
                                && edgeToTEgeMap.Contains(meshEdge2)
                                && edgeToTEgeMap.Contains(meshEdge3);
            if (!hasAllEdges)
            {
                continue;
            }

            global::Kerf.Topology.Edge tEdge1 = edgeToTEgeMap.FindFromKey(meshEdge1);
            if (isReversed1)
            {
                tEdge1.Reverse();
            }
            global::Kerf.Topology.Edge tEdge2 = edgeToTEgeMap.FindFromKey(meshEdge2);
            if (isReversed2)
            {
                tEdge2.Reverse();
            }
            global::Kerf.Topology.Edge tEdge3 = edgeToTEgeMap.FindFromKey(meshEdge3);
            if (isReversed3)
            {
                tEdge3.Reverse();
            }

            var wireMaker = new MakeWire();
            wireMaker.Add(tEdge1);
            wireMaker.Add(tEdge2);
            wireMaker.Add(tEdge3);
            Wire wire = wireMaker.InternalWire();

            // Construct plane explicitly since it is faster than automatic construction
            // within BRepBuilderAPI_MakeFace.
            var c1 = new EdgeCurveAdaptor(tEdge1);
            var c2 = new EdgeCurveAdaptor(tEdge2);
            Direction3D d1 = c1.Line().Direction();
            Direction3D d2 = c2.Line().Direction();
            Xyz n = d1.XYZ().Crossed(d2.XYZ());
            if (n.SquareModulus() < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
            {
                continue;
            }
            if (tEdge1.Orientation() == Reversed)
            {
                n.Reverse();
            }
            if (tEdge2.Orientation() == Reversed)
            {
                n.Reverse();
            }
            var norm = new Direction3D(n);
            var pln = new Plane3D(_mesh.Node(idx0), norm);

            var faceMaker = new MakeFace(pln, wire);
            Face face = faceMaker.InternalFace();

            bb.Add(result, face);
        }

        Done();
        _shape = result;
    }
}
