// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// An Kerf addition, not a port: the managed equivalent of Kerf.Modeling's native mesh extraction
// (native/shim/src/b3d_mesh.cpp, b3d_mesh_extract), which fills Bento's viewer's buffers. Open question 12.

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Measurement;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Meshing;

/// <summary>
/// Every face's triangulation and every edge's polyline of a meshed shape, in a few flat buffers, for a viewer: the
/// managed equivalent of Kerf.Modeling's native <c>Extras.ExtractMesh</c>, float for float.
/// </summary>
/// <remarks>
/// <para>
/// The shape is meshed first (<see cref="IncrementalMesh"/>); extraction only reads it, so it is safe from any
/// thread on a frozen shape, and a shape shared by several callers is never written. Native OCCT computes the nodal
/// normals into the face's own triangulation (<see cref="ToolTriangulatedShape.ComputeNormals(in Face, Triangulation?)"/>);
/// here they are computed on an unattached copy (phase3-design section 5), with the same values.
/// </para>
/// <para>
/// A face without a triangulation gives no nodes. An edge is taken from the polygon on triangulation of the first of
/// its faces (in <see cref="Exploration.MapShapesAndAncestors"/> order) that carries one, so it lies exactly on the shaded
/// surface; else from the mesher's 3D polygon of a free edge; else it is discretised with
/// <see cref="TangentialDeflection"/> to the given deflection and angle. An edge's points run the way the edge
/// does.
/// </para>
/// </remarks>
internal static class MeshExtraction
{
    /// <summary>
    /// The buffers of <paramref name="faces"/> and <paramref name="edges"/>, sub-shapes of the meshed
    /// <paramref name="root"/>, in the order given (b3d_mesh_extract's arguments).
    /// </summary>
    /// <param name="root">The meshed shape: its faces are the edges' ancestors.</param>
    /// <param name="faces">The faces to extract, with their locations and orientations.</param>
    /// <param name="edges">The edges to extract, with their locations and orientations.</param>
    /// <param name="deflection">The linear deflection for edges without a polygon.</param>
    /// <param name="angle">The angular deflection, in radians, for edges without a polygon.</param>
    public static global::Kerf.Engine.Meshing.MeshBuffers Extract(in Shape root, IReadOnlyList<Shape> faces,
                                      IReadOnlyList<Shape> edges, double deflection, double angle)
        => Extract(root, faces, edges, deflection, angle, out _, out _);

    /// <summary>
    /// The buffers, as the other form, with each face's triangulation (null for a face without one) and the placement
    /// it is in: what the public <c>ShapeMesh</c> keeps for the double-precision nodes and the mesh writers (unit 10L3).
    /// </summary>
    internal static global::Kerf.Engine.Meshing.MeshBuffers Extract(in Shape root, IReadOnlyList<Shape> faces,
                                      IReadOnlyList<Shape> edges, double deflection, double angle,
                                      out Triangulation?[] triangulations, out Placement[] placements)
    {
        ArgumentNullException.ThrowIfNull(faces);
        ArgumentNullException.ThrowIfNull(edges);

        var edgeFaces = new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(root, ShapeType.Edge, ShapeType.Face, edgeFaces);

        // Sizes first, so each buffer is allocated once at its final length
        var tris = new Triangulation?[faces.Count];
        var locations = new Placement[faces.Count];
        long nbNodes = 0, nbIndices = 0;
        for (int i = 0; i < faces.Count; i++)
        {
            tris[i] = Tool.Triangulation(ShapeCasts.Face(faces[i]), out locations[i]);
            if (tris[i] is { } tri)
            {
                nbNodes += tri.NodeCount;
                nbIndices += 3L * tri.TriangleCount;
            }
        }

        var builder = new Builder(checked((int)(3 * nbNodes)), checked((int)nbIndices), faces.Count, edges.Count);
        for (int i = 0; i < faces.Count; i++)
        {
            builder.AddFace(ShapeCasts.Face(faces[i]), tris[i], locations[i]);
        }
        builder.FaceNodes[faces.Count] = builder.NodeCount;
        builder.FaceTriangles[faces.Count] = builder.IndexCount;

        for (int i = 0; i < edges.Count; i++)
        {
            builder.AddEdge(ShapeCasts.Edge(edges[i]), edgeFaces, deflection, angle);
        }
        triangulations = tris;
        placements = locations;
        return builder.Finish();
    }

    /// <summary>
    /// The buffers of every face and every edge of the meshed <paramref name="root"/>, in the order of
    /// <see cref="Faces"/> and <see cref="Edges"/>: Kerf.Modeling's <c>ShapeMesh</c>.
    /// </summary>
    public static global::Kerf.Engine.Meshing.MeshBuffers Extract(in Shape root, double deflection, double angle)
        => Extract(root, Faces(root), Edges(root), deflection, angle);

    /// <summary>
    /// The distinct faces of a shape in exploration order, as Kerf.Modeling's <c>Shape.Faces()</c>: each face once
    /// (<see cref="Shape.IsSame"/>), at its first place, with the orientation of its last occurrence.
    /// </summary>
    public static List<Shape> Faces(in Shape shape) => Entities(shape, ShapeType.Face);

    /// <summary>The distinct edges of a shape, as <see cref="Faces"/>: Kerf.Modeling's <c>Shape.Edges()</c>.</summary>
    public static List<Shape> Edges(in Shape shape) => Entities(shape, ShapeType.Edge);

    private static List<Shape> Entities(in Shape shape, ShapeType type)
    {
        var index = new Dictionary<Shape, int>(ShapeHasher.Instance);
        var found = new List<Shape>();
        if (shape.IsNull)
        {
            return found;
        }
        for (var exp = new Explorer(shape, type); exp.More(); exp.Next())
        {
            Shape item = exp.Current();
            if (index.TryGetValue(item, out int i))
            {
                found[i] = item;
            }
            else
            {
                index.Add(item, found.Count);
                found.Add(item);
            }
        }
        return found;
    }

    /// <summary>Fills the buffers: b3d_mesh.cpp's addFace and addEdge.</summary>
    private sealed class Builder(int theNodeFloats, int theIndices, int theFaces, int theEdges)
    {
        public readonly float[] Positions = new float[theNodeFloats];
        public readonly float[] Normals = new float[theNodeFloats];
        public readonly int[] Triangles = new int[theIndices];
        public readonly long[] FaceNodes = new long[theFaces + 1];
        public readonly long[] FaceTriangles = new long[theFaces + 1];
        public readonly int[] FaceTypes = new int[theFaces];
        public readonly long[] EdgePoints = new long[theEdges + 1];
        public readonly int[] EdgeTypes = new int[theEdges];
        private readonly List<float> _points = new(Math.Max(16, 4 * theNodeFloats / 10));
        private int _nodeFloats, _indices, _face, _edge;

        public long NodeCount => _nodeFloats / 3;

        public long IndexCount => _indices;

        public void AddFace(in Face face, Triangulation? tri, in Placement location)
        {
            FaceNodes[_face] = NodeCount;
            FaceTriangles[_face] = _indices;
            FaceTypes[_face++] = (int)new FaceSurfaceAdaptor(face, false).GetType();
            if (tri is null)
            {
                return;
            }

            // The surface's normals at the nodes, written straight into the buffer (unit 10H6: they were computed on a copy
            // of the triangulation, which for a big mesh allocated several times the buffers); the face's own
            // triangulation is never written
            Span<float> normals = Normals.AsSpan(_nodeFloats, 3 * tri.NodeCount);
            Triangulation aTri = ToolTriangulatedShape.TryWriteSurfaceNormals(face, tri, normals)
                                     ? tri
                                     : ToolTriangulatedShape.NormalsComputed(face, tri)!;
            bool isWritten = ReferenceEquals(aTri, tri) && !tri.HasNormals;
            Transform3D trsf = location.Transformation;
            bool isReversed = face.Orientation == Reversed;
            // Winding follows the surface's natural normal: flipped for a reversed face, and again by a mirroring location
            bool toFlip = isReversed != trsf.IsNegative;
            for (int i = 1; i <= aTri.NodeCount; ++i)
            {
                Point3D p = aTri.Node(i).Transformed(trsf);
                Positions[_nodeFloats] = (float)p.X;
                Positions[_nodeFloats + 1] = (float)p.Y;
                Positions[_nodeFloats + 2] = (float)p.Z;
                Direction3D n = isWritten
                                    ? new Direction3D(Normals[_nodeFloats], Normals[_nodeFloats + 1], Normals[_nodeFloats + 2])
                                    : aTri.Normal(i);
                n = n.Transformed(trsf);
                if (isReversed)
                {
                    n = n.Reversed();
                }
                Normals[_nodeFloats] = (float)n.X;
                Normals[_nodeFloats + 1] = (float)n.Y;
                Normals[_nodeFloats + 2] = (float)n.Z;
                _nodeFloats += 3;
            }
            for (int i = 1; i <= aTri.TriangleCount; ++i)
            {
                aTri.Triangle(i).Get(out int a, out int b, out int c);
                Triangles[_indices] = a - 1;
                Triangles[_indices + 1] = toFlip ? c - 1 : b - 1;
                Triangles[_indices + 2] = toFlip ? b - 1 : c - 1;
                _indices += 3;
            }
        }

        private void AddPoint(in Point3D p)
        {
            _points.Add((float)p.X);
            _points.Add((float)p.Y);
            _points.Add((float)p.Z);
        }

        public void AddEdge(in Edge edge, IndexedMap<Shape, NCollection_List<Shape>> edgeFaces,
                            double deflection, double angle)
        {
            EdgePoints[_edge] = _points.Count / 3;
            var curve = new EdgeCurveAdaptor(edge);
            EdgeTypes[_edge++] = (int)curve.GetType();
            int start = _points.Count;

            // On a face's triangulation, so the edge sits exactly on the shaded surface
            bool isDone = false;
            if (edgeFaces.TryGetValue(edge, out NCollection_List<Shape> faces))
            {
                foreach (Shape shape in faces)
                {
                    Triangulation? tri = Tool.Triangulation(ShapeCasts.Face(shape), out Placement location);
                    if (tri is null)
                    {
                        continue;
                    }
                    PolygonOnTriangulation? polygon = Tool.PolygonOnTriangulation(edge, tri, location);
                    if (polygon is null)
                    {
                        continue;
                    }
                    Transform3D trsf = location.Transformation;
                    for (int i = 1; i <= polygon.NodeCount; ++i)
                    {
                        AddPoint(tri.Node(polygon.Node(i)).Transformed(trsf));
                    }
                    isDone = true;
                    break;
                }
            }
            // A free edge's own polygon, from the mesher
            if (!isDone)
            {
                Polygon3D? polygon = Tool.Polygon3D(edge, out Placement location);
                if (polygon is not null)
                {
                    Transform3D trsf = location.Transformation;
                    NCollection_Array1<Point3D> nodes = polygon.Nodes();
                    for (int i = 1; i <= polygon.NodeCount; ++i)
                    {
                        AddPoint(nodes.Value(i).Transformed(trsf));
                    }
                    isDone = true;
                }
            }
            // Else discretised to the same tolerances
            if (!isDone)
            {
                var discretizer = new TangentialDeflection(curve, angle, deflection);
                for (int i = 1; i <= discretizer.PointCount; ++i)
                {
                    AddPoint(discretizer.Value(i));
                }
            }
            // Polygons follow the curve's parameter: reversed to run the way the edge does
            if (edge.Orientation == Reversed)
            {
                int count = (_points.Count - start) / 3;
                for (int i = 0; i < count / 2; ++i)
                {
                    for (int k = 0; k < 3; ++k)
                    {
                        int p = start + 3 * i + k, q = start + 3 * (count - 1 - i) + k;
                        (_points[p], _points[q]) = (_points[q], _points[p]);
                    }
                }
            }
        }

        public global::Kerf.Engine.Meshing.MeshBuffers Finish()
        {
            EdgePoints[_edge] = _points.Count / 3;
            return new global::Kerf.Engine.Meshing.MeshBuffers(Positions, Normals, Triangles, FaceNodes, FaceTriangles, FaceTypes, [.. _points],
                                   EdgePoints, EdgeTypes);
        }
    }
}

/// <summary>
/// The buffers <see cref="MeshExtraction.Extract(in Shape, IReadOnlyList{Shape}, IReadOnlyList{Shape}, double, double)"/>
/// fills, laid out as Kerf.Modeling's <c>MeshBuffers</c>: every face's nodes (x, y, z each, located), their unit
/// normals (facing out of a reversed face too) and its triangles (index triples into the face's own nodes, wound to
/// face out) end to end, then every edge's polyline. The offsets have one entry per face or edge and a final total.
/// </summary>
/// <remarks>The arrays are the extraction's own; nothing else holds them.</remarks>
internal sealed class MeshBuffers
{
    internal MeshBuffers(float[] positions, float[] normals, int[] triangles, long[] faceNodes, long[] faceTriangles,
                         int[] faceTypes, float[] points, long[] edgePoints, int[] edgeTypes)
    {
        Positions = positions;
        Normals = normals;
        Triangles = triangles;
        FaceNodes = faceNodes;
        FaceTriangles = faceTriangles;
        FaceTypes = faceTypes;
        Points = points;
        EdgePoints = edgePoints;
        EdgeTypes = edgeTypes;
    }

    /// <summary>The faces' nodes, x, y, z each, with their locations applied.</summary>
    public float[] Positions { get; }

    /// <summary>The unit normal at each node, x, y, z each, facing out of the face as oriented.</summary>
    public float[] Normals { get; }

    /// <summary>The triangles: three indices each into their face's own nodes (from 0), counter-clockwise seen from outside.</summary>
    public int[] Triangles { get; }

    /// <summary>Each face's first node, then the total.</summary>
    public long[] FaceNodes { get; }

    /// <summary>Each face's first triangle index (three per triangle), then the total.</summary>
    public long[] FaceTriangles { get; }

    /// <summary>Each face's <see cref="SurfaceKind"/>.</summary>
    public int[] FaceTypes { get; }

    /// <summary>The edges' polylines, x, y, z per point, with their locations applied.</summary>
    public float[] Points { get; }

    /// <summary>Each edge's first point, then the total.</summary>
    public long[] EdgePoints { get; }

    /// <summary>Each edge's <see cref="CurveKind"/>.</summary>
    public int[] EdgeTypes { get; }
}
