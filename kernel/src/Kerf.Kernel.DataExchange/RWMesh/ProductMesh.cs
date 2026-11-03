// New in OCCT.NET (unit 8T): the mesh writers' common part, written fresh over the assembly tree (decision 1232).
// Its rules follow OCCT's RWMesh_FaceIterator (src/DataExchange/TKDE/RWMesh): each face's triangulation in the part's
// coordinates, the winding swapped for a reversed face or a mirroring location, normals reversed for a reversed face.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Library;
using Kerf.Engine.Exchange.Documents;
using Kerf.Engine.Foundation;

using Kerf.Engine.Meshing;

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;

using Kerf.Topology;
using Occt;

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
        public bool HasNormals = true;

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
    public static ProductMesh Make(DocumentProduct part, bool toComputeNormals, bool toMergeFaces = true)
    {
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
            AddFace(aGroup, face, tri, loc, toComputeNormals);
        }
        foreach (Group aGroup in groups)
        {
            if (!aGroup.HasNormals)
            {
                aGroup.Normals.Clear();
            }
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
                           .OrderBy(p => (int)p.Sub.Shape.ShapeType())
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

    private static void AddFace(Group theGroup, in Face face, Triangulation tri, in Placement loc, bool toComputeNormals)
    {
        Transform3D trsf = loc.Transformation();
        bool isIdentity = loc.IsIdentity;
        bool isMirrored = trsf.VectorialPart().Determinant() < 0.0;
        bool isReversed = face.Orientation() == Orientation.Reversed;

        Triangulation? normalsTri = tri.HasNormals
                                            ? tri
                                            : toComputeNormals ? ToolTriangulatedShape.NormalsComputed(face, tri) : null;
        if (normalsTri is null || !normalsTri.HasNormals)
        {
            theGroup.HasNormals = false;
        }

        int aBase = theGroup.NbNodes;
        int nbNodes = tri.NodeCount;
        for (int i = 1; i <= nbNodes; i++)
        {
            Point3D p = tri.Node(i);
            if (!isIdentity)
            {
                p = p.Transformed(trsf);
            }
            theGroup.Positions.Add(p.X());
            theGroup.Positions.Add(p.Y());
            theGroup.Positions.Add(p.Z());
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
                theGroup.Normals.Add((float)n.X());
                theGroup.Normals.Add((float)n.Y());
                theGroup.Normals.Add((float)n.Z());
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
            theGroup.Indices.Add(aBase + n1 - 1);
            theGroup.Indices.Add(aBase + n2 - 1);
            theGroup.Indices.Add(aBase + n3 - 1);
        }
    }
}
