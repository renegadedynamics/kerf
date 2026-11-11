// New in Kerf (unit 8T): the mesh writers' common part, written fresh over the assembly tree (decision 1232).
// Its rules follow OCCT's RWMesh_FaceIterator (src/DataExchange/TKDE/RWMesh): each face's triangulation in the part's
// coordinates, the winding swapped for a reversed face or a mirroring location, normals reversed for a reversed face.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Library;
using Kerf.Engine.Exchange.Documents;
using Kerf.Engine.Foundation;

using Kerf.Engine.Meshing;

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;

using Kerf.Topology;

namespace Kerf.Engine.Exchange.Mesh;

/// <summary>
/// A part's triangles, in the part's coordinates, gathered from its faces' triangulations into groups of faces sharing
/// a sub-shape colour (faces with none form the group that takes the instance's colour).
/// </summary>
/// <remarks>Faces without a triangulation are skipped (mesh the tree first: <see cref="Tree.Meshed"/>).</remarks>
internal sealed class ProductMesh
{
    /// <summary>A group of faces: the colour of their sub-shapes (null: the instance's), and their triangles.</summary>
    internal sealed class Group
    {
        public ColorWithAlpha? Color;
        public readonly List<double> Positions = []; // x, y, z per node
        public readonly List<float> Normals = [];    // x, y, z per node, if every face has normals
        public readonly List<int> Indices = [];      // three per triangle, from 0
        public readonly List<int> NodeIds = [];      // per node, if shared: the part's mesh node (0 up), or one face's (below 0)
        public bool HasNormals = true;

        // Sharing: the group's node of each part node (and normal, a crease giving a node a normal per face)
        public Dictionary<(int Id, float X, float Y, float Z), int>? ByNode;

        public int NbNodes => Positions.Count / 3;

        public int NbTriangles => Indices.Count / 3;
    }

    private ProductMesh(List<Group> groups) => Groups = groups;

    /// <summary>The groups, in the order their first faces come; empty groups left out.</summary>
    public List<Group> Groups { get; }

    /// <summary>The number of triangles.</summary>
    public int NbTriangles => Groups.Sum(g => g.NbTriangles);

    /// <summary>
    /// The part's mesh: its faces (in TopExp_Explorer's order), each with its triangulation placed by the face's
    /// location; with normals (the triangulation's, else computed from the surface if <paramref name="toComputeNormals"/>).
    /// Faces of hidden sub-shapes are left out. Faces of a colour make one group, unless
    /// <paramref name="toMergeFaces"/> is false: then each face is a group of its own (unit 8G1, OCCT's glTF writer
    /// without <c>-mergeFaces</c>).
    /// </summary>
    /// <remarks>
    /// With <paramref name="toShareNodes"/>, faces meeting at an edge share that edge's nodes (Kerf, decision 30801):
    /// the mesher discretises each edge once, and each face's triangulation holds the edge's polygon as indices into
    /// its own nodes, so a node of an edge (or a vertex) is one node of the part, taken from the first face meeting it,
    /// and every face's copy of it is that node, not a copy placed by the face's own location (which can differ in the
    /// last digits, so that a slicer, joining only identical vertices, reads the mesh as open there). A seam's two sides
    /// and a degenerate edge's nodes (a cone's apex, a sphere's pole) become one node each; triangles left with two
    /// corners the same (zero area, at a degenerate edge) are dropped. A group holds a part node once per normal, and
    /// <see cref="Group.NodeIds"/> names the part node of each of its nodes, for a writer sharing nodes across groups.
    /// </remarks>
    public static ProductMesh Make(DocumentProduct part, bool toComputeNormals, bool toMergeFaces = true, bool toShareNodes = false)
    {
        NodeSharing? sharing = toShareNodes ? new NodeSharing() : null;
        Dictionary<Shape, Style> faceStyles = FaceStyles(part);
        var groups = new List<Group>();
        var byColor = new Dictionary<ColorWithAlpha, Group>();
        Group? inherited = null;

        for (var exp = new Explorer(part.Shape, ShapeType.Face); exp.More(); exp.Next())
        {
            Face face = ShapeCasts.Face(exp.Current());
            ColorWithAlpha? color = null;
            if (faceStyles.TryGetValue(face, out Style style))
            {
                if (!style.IsVisible)
                {
                    continue;
                }
                color = style.SurfaceColor;
            }
            Triangulation? tri = Tool.Triangulation(face, out Placement loc);
            if (tri is null || tri.TriangleCount == 0 || tri.NodeCount == 0)
            {
                continue;
            }

            Group aGroup;
            if (!toMergeFaces)
            {
                aGroup = new Group { Color = color };
                groups.Add(aGroup);
            }
            else if (color is ColorWithAlpha key)
            {
                if (!byColor.TryGetValue(key, out aGroup!))
                {
                    aGroup = new Group { Color = key };
                    byColor.Add(key, aGroup);
                    groups.Add(aGroup);
                }
            }
            else
            {
                if (inherited is null)
                {
                    inherited = new Group();
                    groups.Add(inherited);
                }
                aGroup = inherited;
            }
            AddFace(aGroup, face, tri, loc, toComputeNormals, sharing);
        }
        foreach (Group aGroup in groups)
        {
            if (!aGroup.HasNormals)
            {
                aGroup.Normals.Clear();
            }
            aGroup.ByNode = null;
        }
        return new ProductMesh(groups);
    }

    // The styles the part's sub-shapes give its faces: the sub-shapes taken from the most general type (compound) to
    // the most specific (face), so that a face's own colour overrides its solid's (decision 1232)
    private static Dictionary<Shape, Style> FaceStyles(DocumentProduct part)
    {
        var result = new Dictionary<Shape, Style>(ShapeHasher.Instance);
        if (part.SubShapes.IsEmpty)
        {
            return result;
        }
        var subs = part.SubShapes
                           .Select((s, i) => (Sub: s, Index: i))
                           .OrderBy(p => (int)p.Sub.Shape.Type)
                           .ThenBy(p => p.Index)
                           .ToList();
        foreach ((SubShape sub, int _) in subs)
        {
            Attributes attributes = sub.Attributes;
            if (!attributes.HasColor && attributes.IsVisible)
            {
                continue;
            }
            for (var exp = new Explorer(sub.Shape, ShapeType.Face); exp.More(); exp.Next())
            {
                Shape face = exp.Current();
                Style aBase = result.TryGetValue(face, out Style known) ? known : new Style(null, null);
                result[face] = aBase.Merged(attributes);
            }
        }
        return result;
    }

    // The part's mesh nodes met so far, when faces share their edges' nodes: a node per vertex and per inner node of an
    // edge's polygon, and each node's position (from the first face meeting it); and the count of nodes of one face
    // alone, whose ids are below zero
    private sealed class NodeSharing
    {
        public readonly Dictionary<Shape, int> Vertices = new(ShapeHasher.Instance);
        public readonly Dictionary<Shape, int[]> Edges = new(ShapeHasher.Instance);
        public readonly List<Point3D> Positions = [];
        public int FaceNodes;

    }

    // The part node of each node of the face's triangulation (from 1; -1 for a node of the face alone), from its edges'
    // polygons on the triangulation; new part nodes are left -2, for AddFace to make from the face's own positions
    private static int[] SharedNodes(NodeSharing sharing, in Face face, Triangulation tri, in Placement loc)
    {
        int nbNodes = tri.NodeCount;
        int[] ids = new int[nbNodes + 1];
        Array.Fill(ids, -1);
        var pending = new List<(int Node, Shape Key, int Index)>();
        for (var exp = new Explorer(face, ShapeType.Edge); exp.More(); exp.Next())
        {
            Edge edge = ShapeCasts.Edge(exp.Current());
            PolygonOnTriangulation? polygon = Tool.PolygonOnTriangulation(edge, tri, loc);
            if (polygon is null || polygon.NodeCount < 2)
            {
                continue; // no polygon: the edge's nodes stay the face's own
            }
            int n = polygon.NodeCount;
            bool isDegenerated = Tool.Degenerated(edge);
            Exploration.Vertices(edge, out Vertex v1, out Vertex v2);
            if (!sharing.Edges.TryGetValue(edge, out int[]? inner))
            {
                inner = new int[n];
                Array.Fill(inner, -2);
                sharing.Edges.Add(edge, inner);
            }
            bool sameCount = inner.Length == n; // the same edge has one polygon for all its faces; if not, keep it apart
            for (int k = 1; k <= n; k++)
            {
                int node = polygon.Node(k);
                if (node < 1 || node > nbNodes || ids[node] != -1)
                {
                    continue;
                }
                Shape vertex = isDegenerated ? v1 : k == 1 ? v1 : k == n ? v2 : default;
                if (!vertex.IsNull)
                {
                    pending.Add((node, vertex, -1));
                }
                else if (sameCount && !isDegenerated)
                {
                    pending.Add((node, edge, k - 1));
                }
                else
                {
                    continue;
                }
                ids[node] = -2;
            }
        }
        foreach ((int node, Shape key, int index) in pending)
        {
            int id;
            if (index < 0)
            {
                if (!sharing.Vertices.TryGetValue(key, out id))
                {
                    id = NewNode(sharing, tri, loc, node);
                    sharing.Vertices.Add(key, id);
                }
            }
            else
            {
                int[] inner = sharing.Edges[key];
                id = inner[index];
                if (id < 0)
                {
                    id = NewNode(sharing, tri, loc, node);
                    inner[index] = id;
                }
            }
            ids[node] = id;
        }
        return ids;
    }

    private static int NewNode(NodeSharing sharing, Triangulation tri, in Placement loc, int node)
    {
        Point3D p = tri.Node(node);
        if (!loc.IsIdentity)
        {
            p = p.Transformed(loc.Transformation);
        }
        sharing.Positions.Add(p);
        return sharing.Positions.Count - 1;
    }

    private static void AddFace(Group theGroup, in Face face, Triangulation tri, in Placement loc, bool toComputeNormals, NodeSharing? sharing)
    {
        Transform3D trsf = loc.Transformation;
        bool isIdentity = loc.IsIdentity;
        bool isMirrored = trsf.VectorialPart.Determinant < 0.0;
        bool isReversed = face.Orientation == Orientation.Reversed;

        Triangulation? normalsTri = tri.HasNormals
                                            ? tri
                                            : toComputeNormals ? ToolTriangulatedShape.NormalsComputed(face, tri) : null;
        if (normalsTri is null || !normalsTri.HasNormals)
        {
            theGroup.HasNormals = false;
        }

        int aBase = theGroup.NbNodes;
        int nbNodes = tri.NodeCount;
        int[]? shared = sharing is null ? null : SharedNodes(sharing, face, tri, loc);
        int[]? groupNodes = shared is null ? null : new int[nbNodes + 1];
        for (int i = 1; i <= nbNodes; i++)
        {
            Point3D p;
            int id = -1;
            if (shared is not null && shared[i] >= 0)
            {
                id = shared[i];
                p = sharing!.Positions[id];
            }
            else
            {
                p = tri.Node(i);
                if (!isIdentity)
                {
                    p = p.Transformed(trsf);
                }
                if (sharing is not null)
                {
                    id = -++sharing.FaceNodes; // a node of this face alone: an id of its own, below zero
                }
            }

            float nx = 0, ny = 0, nz = 0;
            if (theGroup.HasNormals)
            {
                Direction3D n = normalsTri!.Normal(i);
                if (isReversed)
                {
                    n = n.Reversed();
                }
                if (!isIdentity)
                {
                    n = n.Transformed(trsf);
                }
                (nx, ny, nz) = ((float)n.X, (float)n.Y, (float)n.Z);
            }

            if (groupNodes is not null)
            {
                if (id >= 0)
                {
                    theGroup.ByNode ??= [];
                    if (theGroup.ByNode.TryGetValue((id, nx, ny, nz), out int known))
                    {
                        groupNodes[i] = known;
                        continue;
                    }
                    theGroup.ByNode.Add((id, nx, ny, nz), theGroup.NbNodes);
                }
                groupNodes[i] = theGroup.NbNodes;
                theGroup.NodeIds.Add(id);
            }
            theGroup.Positions.Add(p.X);
            theGroup.Positions.Add(p.Y);
            theGroup.Positions.Add(p.Z);
            if (theGroup.HasNormals)
            {
                theGroup.Normals.Add(nx);
                theGroup.Normals.Add(ny);
                theGroup.Normals.Add(nz);
            }
        }

        bool toSwap = isReversed ^ isMirrored;
        int nbTris = tri.TriangleCount;
        for (int i = 1; i <= nbTris; i++)
        {
            tri.Triangle(i).Get(out int n1, out int n2, out int n3);
            if (n1 < 1 || n2 < 1 || n3 < 1 || n1 > nbNodes || n2 > nbNodes || n3 > nbNodes)
            {
                continue; // a broken triangle: skipped, as RWMesh's writers would write garbage
            }
            if (toSwap)
            {
                (n2, n3) = (n3, n2);
            }
            if (groupNodes is not null)
            {
                int s1 = shared![n1], s2 = shared[n2], s3 = shared[n3];
                if (s1 >= 0 && (s1 == s2 || s1 == s3) || s2 >= 0 && s2 == s3)
                {
                    continue; // two corners one node: a triangle of no area, at a degenerate edge
                }
                theGroup.Indices.Add(groupNodes[n1]);
                theGroup.Indices.Add(groupNodes[n2]);
                theGroup.Indices.Add(groupNodes[n3]);
                continue;
            }
            theGroup.Indices.Add(aBase + n1 - 1);
            theGroup.Indices.Add(aBase + n2 - 1);
            theGroup.Indices.Add(aBase + n3 - 1);
        }
    }
}
