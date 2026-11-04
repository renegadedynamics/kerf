// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Meshing;

/// <summary>How a shape is meshed (OCCT's <c>IMeshTools_Parameters</c>; docs/api/meshing.md).</summary>
public sealed record MeshOptions
{
    /// <summary>OCCT's defaults, but in parallel.</summary>
    public static MeshOptions Default { get; } = new();

    /// <summary>The largest distance between the mesh and the surface, in model units (or relative to each edge's
    /// size with <see cref="Relative"/>).</summary>
    public double LinearDeflection
    {
        get;
        init => field = value > 0 ? value : throw new ArgumentOutOfRangeException(nameof(value), value, "The deflection must be more than zero.");
    } = 1e-3;

    /// <summary>The largest angle between neighbouring facets, in radians.</summary>
    public double AngularDeflection
    {
        get;
        init => field = value > 0 ? value : throw new ArgumentOutOfRangeException(nameof(value), value, "The angle must be more than zero.");
    } = 0.5;

    /// <summary>Whether <see cref="LinearDeflection"/> is relative to each edge's size; the faces then take their edges'
    /// largest.</summary>
    public bool Relative { get; init; }

    /// <summary>Mesh the edges, then the faces, in parallel: the result is the serial one, bit for bit.</summary>
    public bool Parallel { get; init; } = true;

    /// <summary>The smallest size of a mesh element; zero (the default) for a tenth of the deflection.</summary>
    public double MinSize
    {
        get;
        init => field = value >= 0 ? value : throw new ArgumentOutOfRangeException(nameof(value), value, "The size must be zero or more.");
    }

    /// <summary>Whether a face's internal vertices become nodes of its mesh.</summary>
    public bool InternalVertices { get; init; } = true;

    /// <summary>
    /// The linear deflection for edges the mesher leaves without a polygon (edges of no face: wires, a compound's free
    /// edges), which <see cref="ShapeMesh"/> discretises itself; null for <see cref="LinearDeflection"/>.
    /// </summary>
    public double? EdgeDeflection
    {
        get;
        init => field = value is null or > 0 ? value : throw new ArgumentOutOfRangeException(nameof(value), value, "The deflection must be more than zero.");
    }

    /// <summary>The engine's parameters: OCCT's defaults for what the options leave alone.</summary>
    internal Engine.Meshing.Tools.ToolParameters ToEngine()
    {
        var parameters = new Engine.Meshing.Tools.ToolParameters
        {
            Deflection = LinearDeflection,
            Angle = AngularDeflection,
            Relative = Relative,
            InParallel = Parallel,
            InternalVerticesMode = InternalVertices,
        };
        if (MinSize > 0)
        {
            parameters.MinSize = MinSize;
        }

        return parameters;
    }
}

/// <summary>
/// A shape's mesh in a few flat buffers, for viewers, renderers and the mesh writers: every face's nodes, unit normals
/// and triangles end to end, and every edge's polyline. Coordinates are global (the sub-shapes' placements applied).
/// </summary>
/// <remarks>
/// <para>
/// Meshing doesn't change the shape (decision 1349, question 59): <see cref="Mesher.Mesh"/> meshes in a working overlay,
/// reads the meshes out and drops it, so the faces hold no triangles. The same shape and options always give the same
/// mesh.
/// </para>
/// <para>
/// The triangles are index triples into their own face's nodes, wound counter-clockwise seen from outside (a reversed
/// face's flipped): add the face's <see cref="FaceMesh.FirstNode"/> to index <see cref="Positions"/>. The buffers are
/// shared, read-only slices of a few arrays; the mesh never changes.
/// </para>
/// </remarks>
public sealed class ShapeMesh
{
    private readonly FaceMesh[] _faces;
    private readonly EdgePolyline[] _edges;

    internal ShapeMesh(Shape shape, MeshOptions options, MeshBuffers buffers, IReadOnlyList<Shape> faces, IReadOnlyList<Shape> edges,
                       Engine.Meshing.Triangulation?[] triangulations, Placement[] placements, bool hideFaces = false)
    {
        Shape = shape;
        Options = options;
        Positions = buffers.Positions;
        Normals = buffers.Normals;
        Triangles = buffers.Triangles;
        EdgePoints = buffers.Points;
        _faces = new FaceMesh[faces.Count];
        for (int i = 0; i < _faces.Length; i++)
        {
            int node = (int)buffers.FaceNodes[i], nodes = (int)buffers.FaceNodes[i + 1] - node;
            int index = (int)buffers.FaceTriangles[i], indices = (int)buffers.FaceTriangles[i + 1] - index;
            _faces[i] = new FaceMesh(hideFaces ? default : faces[i].As<Face>(), this, node, nodes, index, indices, (SurfaceKind)buffers.FaceTypes[i],
                                     triangulations[i], placements[i], faces[i].Orientation);
        }

        _edges = new EdgePolyline[edges.Count];
        for (int i = 0; i < _edges.Length; i++)
        {
            int point = (int)buffers.EdgePoints[i], points = (int)buffers.EdgePoints[i + 1] - point;
            _edges[i] = new EdgePolyline(edges[i].As<Edge>(), EdgePoints.Slice(3 * point, 3 * points), (CurveKind)buffers.EdgeTypes[i]);
        }

        NodeCount = Positions.Length / 3;
        TriangleCount = Triangles.Length / 3;
    }

    /// <summary>The shape meshed; a null shape for a mesh read from a file (STL, glTF).</summary>
    public Shape Shape { get; }

    /// <summary>The options it was meshed with.</summary>
    public MeshOptions Options { get; }

    /// <summary>Every face's nodes, x, y, z each.</summary>
    public ReadOnlyMemory<float> Positions { get; }

    /// <summary>The unit normal at each node, x, y, z each, facing out of the face as oriented in the shape.</summary>
    public ReadOnlyMemory<float> Normals { get; }

    /// <summary>Every face's triangles, three indices each into the face's own nodes (from 0).</summary>
    public ReadOnlyMemory<int> Triangles { get; }

    /// <summary>Every edge's polyline, x, y, z per point.</summary>
    public ReadOnlyMemory<float> EdgePoints { get; }

    /// <summary>One per distinct face, in the shape's order of exploration (a face met again keeps its first place, with
    /// the orientation of its last occurrence).</summary>
    public IReadOnlyList<FaceMesh> Faces => _faces;

    /// <summary>One per distinct edge, in the same order as the faces.</summary>
    public IReadOnlyList<EdgePolyline> Edges => _edges;

    /// <summary>The number of nodes of all the faces.</summary>
    public int NodeCount { get; }

    /// <summary>The number of triangles of all the faces.</summary>
    public int TriangleCount { get; }

    /// <summary>
    /// Every face's triangles, each indexing the whole <see cref="Positions"/> buffer (its face's
    /// <see cref="FaceMesh.FirstNode"/> added): for a renderer that draws the mesh in one call. Made on each call.
    /// </summary>
    public Triangle[] GetTriangles()
    {
        var triangles = new Triangle[TriangleCount];
        foreach (FaceMesh face in _faces)
        {
            face.CopyTriangles(triangles.AsSpan(face.FirstTriangle, face.TriangleCount), face.FirstNode);
        }

        return triangles;
    }

    /// <summary>
    /// The mesh as the engine's writers read one: a compound of faces holding only the triangulations, each at its
    /// placement and orientation (an engine shape, never handed out: decision 1349, question 59).
    /// </summary>
    internal Shape ToMeshOnlyShape()
    {
        var builder = new BrepBuilder();
        var faces = new List<Shape>(_faces.Length);
        foreach (FaceMesh mesh in _faces)
        {
            if (mesh.EngineTriangulation is not { } triangulation)
            {
                continue;
            }

            var face = new Face();
            builder.MakeFace(ref face, triangulation);
            faces.Add(face.Located(mesh.EnginePlacement).Oriented(mesh.EngineOrientation));
        }

        Shape compound = Builder.MakeCompound(faces);
        compound.TShape()!.FreezeGraph();
        return compound;
    }

    /// <summary>
    /// The mesh of an engine shape whose faces hold only triangulations (as the STL and glTF readers make them): its
    /// faces aren't handed out, so the mesh has no shape (decision 1349, question 59).
    /// </summary>
    internal static ShapeMesh FromMeshOnlyShape(in Shape meshOnly)
    {
        List<Shape> faces = MeshExtraction.Faces(meshOnly);
        MeshBuffers buffers = MeshExtraction.Extract(meshOnly, faces, [], MeshOptions.Default.LinearDeflection, MeshOptions.Default.AngularDeflection,
                                                     out Engine.Meshing.Triangulation?[] triangulations, out Placement[] placements);
        return new ShapeMesh(default, MeshOptions.Default, buffers, faces, [], triangulations, placements, hideFaces: true);
    }
}

/// <summary>A face's part of a <see cref="ShapeMesh"/>: views into its buffers, and the face's exact nodes.</summary>
public readonly struct FaceMesh
{
    private readonly ShapeMesh _mesh;

    internal FaceMesh(Face face, ShapeMesh mesh, int firstNode, int nodeCount, int firstIndex, int indexCount, SurfaceKind kind,
                      Engine.Meshing.Triangulation? triangulation, Placement placement, Orientation orientation)
    {
        EngineOrientation = orientation;
        Face = face;
        _mesh = mesh;
        FirstNode = firstNode;
        NodeCount = nodeCount;
        FirstTriangle = firstIndex / 3;
        TriangleCount = indexCount / 3;
        Kind = kind;
        EngineTriangulation = triangulation;
        EnginePlacement = placement;
    }

    /// <summary>The face; a null face in a mesh read from a file.</summary>
    public Face Face { get; }

    /// <summary>The kind of the face's surface.</summary>
    public SurfaceKind Kind { get; }

    /// <summary>The face's first node in <see cref="ShapeMesh.Positions"/> (nodes, not floats).</summary>
    public int FirstNode { get; }

    /// <summary>The number of the face's nodes; zero if it couldn't be meshed.</summary>
    public int NodeCount { get; }

    /// <summary>The face's first triangle in <see cref="ShapeMesh.Triangles"/> (triangles, not indices).</summary>
    public int FirstTriangle { get; }

    /// <summary>The number of the face's triangles.</summary>
    public int TriangleCount { get; }

    /// <summary>The face's nodes, x, y, z each.</summary>
    public ReadOnlyMemory<float> Positions => _mesh is null ? default : _mesh.Positions.Slice(3 * FirstNode, 3 * NodeCount);

    /// <summary>The unit normals at the face's nodes, x, y, z each.</summary>
    public ReadOnlyMemory<float> Normals => _mesh is null ? default : _mesh.Normals.Slice(3 * FirstNode, 3 * NodeCount);

    /// <summary>The face's triangles: three indices each into its own nodes, from 0.</summary>
    public ReadOnlyMemory<int> Triangles => _mesh is null ? default : _mesh.Triangles.Slice(3 * FirstTriangle, 3 * TriangleCount);

    /// <summary>The face's triangles, indexing its own nodes (from 0), as <see cref="Triangles"/> holds them. Made on
    /// each call.</summary>
    public Triangle[] GetTriangles()
    {
        var triangles = new Triangle[TriangleCount];
        CopyTriangles(triangles, 0);
        return triangles;
    }

    internal void CopyTriangles(Span<Triangle> triangles, int offset)
    {
        ReadOnlySpan<int> indices = Triangles.Span;
        for (int i = 0; i < triangles.Length; i++)
        {
            triangles[i] = new Triangle(indices[3 * i] + offset, indices[3 * i + 1] + offset, indices[3 * i + 2] + offset);
        }
    }

    /// <summary>The largest distance between the face's mesh and its surface, as the mesher estimated it.</summary>
    public double Deflection => EngineTriangulation?.Deflection() ?? 0;

    /// <summary>The face's nodes in double precision, its placement applied (computed on each call).</summary>
    public Point3D[] GetNodes()
    {
        if (EngineTriangulation is not { } triangulation)
        {
            return [];
        }

        Transform3D transform = EnginePlacement.Transformation;
        var nodes = new Point3D[triangulation.NodeCount];
        for (int i = 0; i < nodes.Length; i++)
        {
            nodes[i] = triangulation.Node(i + 1).Transformed(transform);
        }

        return nodes;
    }

    /// <summary>The nodes' parameters on the face's surface, or none for a face without them.</summary>
    public Point2D[] GetSurfaceParameters()
    {
        if (EngineTriangulation is not { HasUVNodes: true } triangulation)
        {
            return [];
        }

        var nodes = new Point2D[triangulation.NodeCount];
        for (int i = 0; i < nodes.Length; i++)
        {
            nodes[i] = triangulation.UVNode(i + 1);
        }

        return nodes;
    }

    internal Engine.Meshing.Triangulation? EngineTriangulation { get; }

    internal Placement EnginePlacement { get; }

    internal Orientation EngineOrientation { get; }
}

/// <summary>An edge's polyline in a <see cref="ShapeMesh"/>, running the way the edge does. Where the edge bounds a
/// face it is the face mesh's own boundary, so it lies exactly on the shaded surface.</summary>
public readonly struct EdgePolyline
{
    internal EdgePolyline(Edge edge, ReadOnlyMemory<float> points, CurveKind kind)
    {
        Edge = edge;
        Points = points;
        Kind = kind;
    }

    /// <summary>The edge.</summary>
    public Edge Edge { get; }

    /// <summary>The kind of the edge's curve.</summary>
    public CurveKind Kind { get; }

    /// <summary>The points, x, y, z each.</summary>
    public ReadOnlyMemory<float> Points { get; }

    /// <summary>The number of points.</summary>
    public int PointCount => Points.Length / 3;
}

/// <summary>A triangle of a <see cref="ShapeMesh"/>: three node indices from 0, wound counter-clockwise seen from outside
/// (meshing.md).</summary>
/// <param name="A">The first node.</param>
/// <param name="B">The second node.</param>
/// <param name="C">The third node.</param>
public readonly record struct Triangle(int A, int B, int C);
