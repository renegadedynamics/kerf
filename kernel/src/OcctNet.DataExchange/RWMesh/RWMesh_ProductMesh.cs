// New in OCCT.NET (unit 8T): the mesh writers' common part, written fresh over the assembly tree (decision 1232).
// Its rules follow OCCT's RWMesh_FaceIterator (src/DataExchange/TKDE/RWMesh): each face's triangulation in the part's
// coordinates, the winding swapped for a reversed face or a mirroring location, normals reversed for a reversed face.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Meshing;

using Kerf.Geometry;

using Kerf.Topology;

namespace Occt;

/// <summary>
/// A part's triangles, in the part's coordinates, gathered from its faces' triangulations into groups of faces sharing
/// a sub-shape colour (faces with none form the group that takes the instance's colour).
/// </summary>
/// <remarks>Faces without a triangulation are skipped (mesh the tree first: <see cref="XdeDoc_Tree.Meshed"/>).</remarks>
internal sealed class RWMesh_ProductMesh
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

    private RWMesh_ProductMesh(List<Group> theGroups) => Groups = theGroups;

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
    public static RWMesh_ProductMesh Make(XdeDoc_Product thePart, bool toComputeNormals, bool toMergeFaces = true)
    {
        Dictionary<TopoDS_Shape, XdeDoc_Style> aFaceStyles = faceStyles(thePart);
        var aGroups = new List<Group>();
        var aByColor = new Dictionary<ColorWithAlpha, Group>();
        Group? anInherited = null;

        for (var anExp = new TopExp_Explorer(thePart.Shape, TopAbs_ShapeEnum.TopAbs_FACE); anExp.More(); anExp.Next())
        {
            TopoDS_Face aFace = TopoDS.Face(anExp.Current());
            ColorWithAlpha? aColor = null;
            if (aFaceStyles.TryGetValue(aFace, out XdeDoc_Style aStyle))
            {
                if (!aStyle.IsVisible)
                {
                    continue;
                }
                aColor = aStyle.SurfaceColor;
            }
            Triangulation? aTri = BRep_Tool.Triangulation(aFace, out Placement aLoc);
            if (aTri is null || aTri.TriangleCount == 0 || aTri.NodeCount == 0)
            {
                continue;
            }

            Group aGroup;
            if (!toMergeFaces)
            {
                aGroup = new Group { Color = aColor };
                aGroups.Add(aGroup);
            }
            else if (aColor is ColorWithAlpha aKey)
            {
                if (!aByColor.TryGetValue(aKey, out aGroup!))
                {
                    aGroup = new Group { Color = aKey };
                    aByColor.Add(aKey, aGroup);
                    aGroups.Add(aGroup);
                }
            }
            else
            {
                if (anInherited is null)
                {
                    anInherited = new Group();
                    aGroups.Add(anInherited);
                }
                aGroup = anInherited;
            }
            addFace(aGroup, aFace, aTri, aLoc, toComputeNormals);
        }
        foreach (Group aGroup in aGroups)
        {
            if (!aGroup.HasNormals)
            {
                aGroup.Normals.Clear();
            }
        }
        return new RWMesh_ProductMesh(aGroups);
    }

    // The styles the part's sub-shapes give its faces: the sub-shapes taken from the most general type (compound) to
    // the most specific (face), so that a face's own colour overrides its solid's (decision 1232)
    private static Dictionary<TopoDS_Shape, XdeDoc_Style> faceStyles(XdeDoc_Product thePart)
    {
        var aResult = new Dictionary<TopoDS_Shape, XdeDoc_Style>(TopTools_ShapeMapHasher.Instance);
        if (thePart.SubShapes.IsEmpty)
        {
            return aResult;
        }
        var aSubs = thePart.SubShapes
                           .Select((s, i) => (Sub: s, Index: i))
                           .OrderBy(p => (int)p.Sub.Shape.ShapeType())
                           .ThenBy(p => p.Index)
                           .ToList();
        foreach ((XdeDoc_SubShape aSub, int _) in aSubs)
        {
            XdeDoc_Attributes anAttributes = aSub.Attributes;
            if (!anAttributes.HasColor && anAttributes.IsVisible)
            {
                continue;
            }
            for (var anExp = new TopExp_Explorer(aSub.Shape, TopAbs_ShapeEnum.TopAbs_FACE); anExp.More(); anExp.Next())
            {
                TopoDS_Shape aFace = anExp.Current();
                XdeDoc_Style aBase = aResult.TryGetValue(aFace, out XdeDoc_Style aKnown) ? aKnown : new XdeDoc_Style(null, null);
                aResult[aFace] = aBase.Merged(anAttributes);
            }
        }
        return aResult;
    }

    private static void addFace(Group theGroup, in TopoDS_Face theFace, Triangulation theTri, in Placement theLoc, bool toComputeNormals)
    {
        Transform3D aTrsf = theLoc.Transformation();
        bool isIdentity = theLoc.IsIdentity;
        bool isMirrored = aTrsf.VectorialPart().Determinant() < 0.0;
        bool isReversed = theFace.Orientation() == TopAbs_Orientation.TopAbs_REVERSED;

        Triangulation? aNormalsTri = theTri.HasNormals
                                            ? theTri
                                            : toComputeNormals ? BRepLib_ToolTriangulatedShape.NormalsComputed(theFace, theTri) : null;
        if (aNormalsTri is null || !aNormalsTri.HasNormals)
        {
            theGroup.HasNormals = false;
        }

        int aBase = theGroup.NbNodes;
        int aNbNodes = theTri.NodeCount;
        for (int i = 1; i <= aNbNodes; i++)
        {
            Point3D aP = theTri.Node(i);
            if (!isIdentity)
            {
                aP = aP.Transformed(aTrsf);
            }
            theGroup.Positions.Add(aP.X());
            theGroup.Positions.Add(aP.Y());
            theGroup.Positions.Add(aP.Z());
            if (theGroup.HasNormals)
            {
                Direction3D aN = aNormalsTri!.Normal(i);
                if (isReversed)
                {
                    aN = aN.Reversed();
                }
                if (!isIdentity)
                {
                    aN = aN.Transformed(aTrsf);
                }
                theGroup.Normals.Add((float)aN.X());
                theGroup.Normals.Add((float)aN.Y());
                theGroup.Normals.Add((float)aN.Z());
            }
        }

        bool toSwap = isReversed ^ isMirrored;
        int aNbTris = theTri.TriangleCount;
        for (int i = 1; i <= aNbTris; i++)
        {
            theTri.Triangle(i).Get(out int aN1, out int aN2, out int aN3);
            if (aN1 < 1 || aN2 < 1 || aN3 < 1 || aN1 > aNbNodes || aN2 > aNbNodes || aN3 > aNbNodes)
            {
                continue; // a broken triangle: skipped, as RWMesh's writers would write garbage
            }
            if (toSwap)
            {
                (aN2, aN3) = (aN3, aN2);
            }
            theGroup.Indices.Add(aBase + aN1 - 1);
            theGroup.Indices.Add(aBase + aN2 - 1);
            theGroup.Indices.Add(aBase + aN3 - 1);
        }
    }
}
