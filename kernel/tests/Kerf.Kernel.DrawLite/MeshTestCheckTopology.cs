// Port of: src/Draw/TKTopTest/MeshTest/MeshTest_CheckTopology.hxx
// Port of: src/Draw/TKTopTest/MeshTest/MeshTest_CheckTopology.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2004-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Geometry;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// Checks the topology of the mesh of a shape (tricheck): free links inside the faces' triangulations, cross-face
/// errors (a node of an edge's polygon that is further from its counterpart on the other face than the edge's
/// tolerance), asynchronous edges (polygons of different lengths on two faces), free nodes (used by no triangle)
/// and triangles of null area.
/// </summary>
internal sealed class MeshTestCheckTopology(Shape theShape)
{
    private readonly Shape myShape = theShape;

    // NCollection_IndexedDataMap<int, HSequence<int>>: the faces with free links, in the order they were met
    private readonly List<(int Face, List<int> Links)> myMapFaceLinks = [];
    private readonly List<int> myErrors = [];
    private readonly List<double> myErrorsVal = [];
    private readonly List<int> myAsyncEdges = [];
    private readonly List<int> myFreeNodeFaces = [];
    private readonly List<int> myFreeNodeNums = [];
    private readonly List<int> mySmallTrianglesFaces = [];
    private readonly List<int> mySmallTrianglesTriangles = [];

    /// <summary>The area of the triangle of three 3D points.</summary>
    private static double ComputeArea(in Xyz theP1, in Xyz theP2, in Xyz theP3)
        => 0.5 * theP3.Subtracted(theP1).Crossed(theP2.Subtracted(theP1)).Modulus();

    /// <summary>The area of the triangle of three 2D points.</summary>
    private static double ComputeArea(in Xy theP1, in Xy theP2, in Xy theP3)
        => 0.5 * Math.Abs(theP3.Subtracted(theP1).Crossed(theP2.Subtracted(theP1)));

    /// <summary>Performs the checks; "face N has no triangulation" lines go to <paramref name="di"/>.</summary>
    public void Perform(DrawResult di)
    {
        var aMapF = new IndexedSet<Shape>(global::Kerf.Engine.Topology.ShapeHasher.Instance);
        var aMapEF = new IndexedMap<Shape, NCollection_List<Shape>>(global::Kerf.Engine.Topology.ShapeHasher.Instance);
        global::Kerf.Engine.Topology.Exploration.MapShapes(myShape, ShapeType.Face, aMapF);
        global::Kerf.Engine.Topology.Exploration.MapShapesAndAncestors(myShape, ShapeType.Edge, ShapeType.Face, aMapEF);

        // check polygons
        for (int ie = 1; ie <= aMapEF.Count; ie++)
        {
            Edge aEdge = global::Kerf.Engine.Topology.ShapeCasts.Edge(aMapEF.KeyAt(ie - 1));
            NCollection_List<Shape> aFaces = aMapEF.ValueAt(ie - 1);
            if (aFaces.Extent() < 2)
            {
                continue;
            }

            // get polygon on first face
            Face aFace1 = global::Kerf.Engine.Topology.ShapeCasts.Face(aFaces.First());
            Triangulation? aT1 = Tool.Triangulation(aFace1, out Placement aLoc1);
            PolygonOnTriangulation? aPoly1 = Tool.PolygonOnTriangulation(aEdge, aT1, aLoc1);
            if (aPoly1 is null || aT1 is null)
            {
                continue;
            }

            NCollection_Array1<int> aNodes1 = aPoly1.Nodes();

            // cycle on other polygons
            var it = new NCollection_List<Shape>.Iterator(aFaces);
            it.Next();
            for (; it.More(); it.Next())
            {
                Face aFace2 = global::Kerf.Engine.Topology.ShapeCasts.Face(it.Value());
                Triangulation? aT2 = Tool.Triangulation(aFace2, out Placement aLoc2);
                PolygonOnTriangulation? aPoly2 = Tool.PolygonOnTriangulation(aEdge, aT2, aLoc2);
                if (aPoly2 is null || aT2 is null)
                {
                    continue;
                }

                NCollection_Array1<int> aNodes2 = aPoly2.Nodes();

                // check equality of polygons lengths
                if (aNodes2.Length() != aNodes1.Length())
                {
                    myAsyncEdges.Add(ie);
                    break;
                }

                // check distances between corresponding points
                double aSqDefle = Tool.Tolerance(aEdge);
                aSqDefle *= aSqDefle;
                int iF1 = aMapF.IndexOf(aFace1) + 1;
                int iF2 = aMapF.IndexOf(aFace2) + 1;
                int i1 = aNodes1.Lower();
                int i2 = aNodes2.Lower();
                Transform3D aTrsf1 = aFace1.Placement.Transformation;
                Transform3D aTrsf2 = aFace2.Placement.Transformation;
                for (; i1 <= aNodes1.Upper(); i1++, i2++)
                {
                    Point3D aP1 = aT1.Node(aNodes1[i1]).Transformed(aTrsf1);
                    Point3D aP2 = aT2.Node(aNodes2[i2]).Transformed(aTrsf2);
                    double aSqDist = aP1.SquareDistanceTo(aP2);
                    if (aSqDist > aSqDefle)
                    {
                        // OCCT bug: it records the polygon's indices i1 and i2, which tricheck prints as nodes
                        myErrors.Add(iF1);
                        myErrors.Add(i1);
                        myErrors.Add(iF2);
                        myErrors.Add(i2);
                        myErrorsVal.Add(Math.Sqrt(aSqDist));
                    }
                }
            }
        }

        // check triangulations
        for (int iF = 1; iF <= aMapF.Count; iF++)
        {
            Face aFace = global::Kerf.Engine.Topology.ShapeCasts.Face(aMapF[iF - 1]);
            Triangulation? aT = Tool.Triangulation(aFace, out Placement aLoc);
            if (aT is null)
            {
                di.Append("face ").Append(iF).Append(" has no triangulation\n");
                continue;
            }

            Transform3D aTrsf = aLoc.Transformation;

            // remember boundary nodes
            var aMapBndNodes = new HashSet<int>();
            for (var ex = new global::Kerf.Engine.Topology.Explorer(aFace, ShapeType.Edge); ex.More(); ex.Next())
            {
                PolygonOnTriangulation? aPoly = Tool.PolygonOnTriangulation(global::Kerf.Engine.Topology.ShapeCasts.Edge(ex.Current()), aT, aLoc);
                if (aPoly is null)
                {
                    continue;
                }

                NCollection_Array1<int> aNodes = aPoly.Nodes();
                for (int i = aNodes.Lower(); i <= aNodes.Upper(); i++)
                {
                    aMapBndNodes.Add(aNodes[i]);
                }
            }

            var aUsedNodes = new HashSet<int>();

            // check of free links and nodes
            var aConn = new Connect(aT);
            int nbTri = aT.TriangleCount;
            int[] n = new int[3];
            int[] t = new int[3];
            for (int i = 1; i <= nbTri; i++)
            {
                aT.Triangle(i).Get(out n[0], out n[1], out n[2]);

                aUsedNodes.Add(n[0]);
                aUsedNodes.Add(n[1]);
                aUsedNodes.Add(n[2]);

                Point3D aPts0 = aT.Node(n[0]).Transformed(aTrsf);
                Point3D aPts1 = aT.Node(n[1]).Transformed(aTrsf);
                Point3D aPts2 = aT.Node(n[2]).Transformed(aTrsf);

                double anArea = ComputeArea(aPts0.XYZ(), aPts1.XYZ(), aPts2.XYZ());
                if (anArea < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                {
                    mySmallTrianglesFaces.Add(iF);
                    mySmallTrianglesTriangles.Add(i);
                }
                else if (aT.HasUVNodes)
                {
                    anArea = ComputeArea(aT.UVNode(n[0]).XY(), aT.UVNode(n[1]).XY(), aT.UVNode(n[2]).XY());
                    if (anArea < global::Kerf.Engine.Foundation.Precision.SquarePConfusion())
                    {
                        mySmallTrianglesFaces.Add(iF);
                        mySmallTrianglesTriangles.Add(i);
                    }
                }

                aConn.Triangles(i, out t[0], out t[1], out t[2]);
                for (int j = 0; j < 3; j++)
                {
                    if (t[j] == 0)
                    {
                        // free link found
                        int k = (j + 1) % 3; // the following node of the edge
                        int n1 = n[j];
                        int n2 = n[k];

                        // skip if it is on boundary
                        if (aMapBndNodes.Contains(n1) && aMapBndNodes.Contains(n2))
                        {
                            continue;
                        }

                        if (myMapFaceLinks.Count == 0 || myMapFaceLinks[^1].Face != iF)
                        {
                            myMapFaceLinks.Add((iF, []));
                        }

                        List<int> aSeq = myMapFaceLinks[^1].Links;
                        aSeq.Add(n1);
                        aSeq.Add(n2);
                    }
                }
            }

            // check of free nodes
            int aNbNodes = aT.NodeCount;
            for (int k = 1; k <= aNbNodes; k++)
            {
                if (!aUsedNodes.Contains(k))
                {
                    myFreeNodeFaces.Add(iF);
                    myFreeNodeNums.Add(k);
                }
            }
        }
    }

    /// <summary>The number of faces with free links.</summary>
    public int NbFacesWithFL() => myMapFaceLinks.Count;

    /// <summary>The index (in the map of faces) of the face with free links at <paramref name="theIndex"/>.</summary>
    public int GetFaceNumWithFL(int theIndex) => myMapFaceLinks[theIndex - 1].Face;

    /// <summary>The number of free links of the face with free links at <paramref name="theIndex"/>.</summary>
    public int NbFreeLinks(int theIndex) => myMapFaceLinks[theIndex - 1].Links.Count / 2;

    /// <summary>The nodes of a free link of a face with free links.</summary>
    public void GetFreeLink(int theFaceIndex, int theLinkIndex, out int theNode1, out int theNode2)
    {
        List<int> aSeq = myMapFaceLinks[theFaceIndex - 1].Links;
        int aInd = (theLinkIndex - 1) * 2;
        theNode1 = aSeq[aInd];
        theNode2 = aSeq[aInd + 1];
    }

    public int NbCrossFaceErrors() => myErrorsVal.Count;

    /// <summary>The faces, polygon indices and distance of a cross-face error.</summary>
    public void GetCrossFaceError(int theIndex, out int theFace1, out int theNode1, out int theFace2, out int theNode2, out double theValue)
    {
        int aInd = (theIndex - 1) * 4;
        theFace1 = myErrors[aInd];
        theNode1 = myErrors[aInd + 1];
        theFace2 = myErrors[aInd + 2];
        theNode2 = myErrors[aInd + 3];
        theValue = myErrorsVal[theIndex - 1];
    }

    public int NbAsyncEdges() => myAsyncEdges.Count;

    /// <summary>The index (in the map of edges) of an asynchronous edge.</summary>
    public int GetAsyncEdgeNum(int theIndex) => myAsyncEdges[theIndex - 1];

    public int NbFreeNodes() => myFreeNodeFaces.Count;

    public void GetFreeNodeNum(int theIndex, out int theFaceNum, out int theNodeNum)
    {
        theFaceNum = myFreeNodeFaces[theIndex - 1];
        theNodeNum = myFreeNodeNums[theIndex - 1];
    }

    public int NbSmallTriangles() => mySmallTrianglesFaces.Count;

    public void GetSmallTriangle(int theIndex, out int theFaceNum, out int theTriNum)
    {
        theFaceNum = mySmallTrianglesFaces[theIndex - 1];
        theTriNum = mySmallTrianglesTriangles[theIndex - 1];
    }
}
