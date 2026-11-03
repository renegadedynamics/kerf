// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_TriangleSet.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_TriangleSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2014-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bvh;
using Kerf.Engine.Meshing;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Geometry.Bvh.Constants;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// The triangles of the triangulations of faces (and the segments of the 3D polygons of edges, as degenerate
/// triangles) of a list of shapes, in a BVH, for collision detection.
/// </summary>
/// <remarks>
/// A working object (decision 21): OCCT's accessors return its own arrays. It only reads the shapes' meshes. As in
/// OCCT, <see cref="Clear"/> (and so <see cref="Init"/>) doesn't clear the per-shape vectors, so a set initialised
/// twice keeps the first shapes' entries in front (occt-issues.md).
/// </remarks>
public class BRepExtrema_TriangleSet : PrimitiveSet<double, NCollection_Vec3<double>>
{
    /// <summary>The vertex indices of the triangles (x, y, z) and their shape index (w).</summary>
    protected NCollection_LinearVector<NCollection_Vec4<int>> myTriangles = new();

    /// <summary>The vertex coordinates.</summary>
    protected NCollection_LinearVector<NCollection_Vec3<double>> myVertexArray = new();

    /// <summary>The index of the shape of each vertex.</summary>
    protected NCollection_DynamicArray<int> myShapeIdxOfVtxVec = new();

    /// <summary>The number of vertices of each shape.</summary>
    protected NCollection_DynamicArray<int> myNumVtxInShapeVec = new();

    /// <summary>The number of triangles of each shape.</summary>
    protected NCollection_DynamicArray<int> myNumTrgInShapeVec = new();

    /// <summary>The triangles' indices after (key) and before (value) swapping.</summary>
    protected NCollection_DataMap<int, int> myTrgIdxMap = new();

    /// <summary>An empty triangle set.</summary>
    public BRepExtrema_TriangleSet()
        : base(new LinearBuilder<double, NCollection_Vec3<double>>(BVH_Constants_LeafNodeSizeDefault, BVH_Constants_MaxTreeDepth))
    {
        // Set default builder - linear BVH (LBVH)
    }

    /// <summary>A triangle set of the given shapes.</summary>
    public BRepExtrema_TriangleSet(NCollection_DynamicArray<TopoDS_Shape> theFaces)
        : base(new LinearBuilder<double, NCollection_Vec3<double>>(BVH_Constants_LeafNodeSizeDefault, BVH_Constants_MaxTreeDepth))
    {
        // Set default builder - linear BVH (LBVH)
        Init(theFaces);
    }

    /// <summary>The total number of triangles.</summary>
    public override int Size() => myTriangles.Size();

    /// <summary>The AABB of the given triangle.</summary>
    public override Box<double, NCollection_Vec3<double>> Box(int theIndex)
    {
        NCollection_Vec4<int> aTriangle = myTriangles[theIndex];

        NCollection_Vec3<double> aMinPnt = myVertexArray[aTriangle.X()].CwiseMin(
            myVertexArray[aTriangle.Y()].CwiseMin(myVertexArray[aTriangle.Z()]));
        NCollection_Vec3<double> aMaxPnt = myVertexArray[aTriangle.X()].CwiseMax(
            myVertexArray[aTriangle.Y()].CwiseMax(myVertexArray[aTriangle.Z()]));

        return new Box<double, NCollection_Vec3<double>>(aMinPnt, aMaxPnt);
    }

    /// <summary>The centroid of the given triangle along the axis.</summary>
    public override double Center(int theIndex, int theAxis)
    {
        NCollection_Vec4<int> aTriangle = myTriangles[theIndex];

        if (theAxis == 0)
        {
            return (1.0 / 3.0)
                   * (myVertexArray[aTriangle.X()].X() + myVertexArray[aTriangle.Y()].X()
                      + myVertexArray[aTriangle.Z()].X());
        }
        else if (theAxis == 1)
        {
            return (1.0 / 3.0)
                   * (myVertexArray[aTriangle.X()].Y() + myVertexArray[aTriangle.Y()].Y()
                      + myVertexArray[aTriangle.Z()].Y());
        }
        else
        {
            return (1.0 / 3.0)
                   * (myVertexArray[aTriangle.X()].Z() + myVertexArray[aTriangle.Y()].Z()
                      + myVertexArray[aTriangle.Z()].Z());
        }
    }

    /// <summary>Swaps two triangles.</summary>
    public override void Swap(int theIndex1, int theIndex2)
    {
        (myTriangles[theIndex1], myTriangles[theIndex2]) = (myTriangles[theIndex2], myTriangles[theIndex1]);
        (myTrgIdxMap[theIndex1], myTrgIdxMap[theIndex2]) = (myTrgIdxMap[theIndex2], myTrgIdxMap[theIndex1]);
    }

    /// <summary>The shape index of the given triangle.</summary>
    public int GetFaceID(int theIndex) => myTriangles[theIndex].W();

    /// <summary>The shape index of the given vertex.</summary>
    public int GetShapeIDOfVtx(int theIndex) => myShapeIdxOfVtxVec.Value(theIndex);

    /// <summary>The index of a vertex of the whole set in the triangulation of its shape.</summary>
    public int GetVtxIdxInShape(int theIndex)
    {
        int aShID = myShapeIdxOfVtxVec.Value(theIndex);
        int aNumVertices = 0;

        for (int aSIdx = 0; aSIdx < aShID; aSIdx++)
        {
            aNumVertices += myNumVtxInShapeVec.Value(aSIdx);
        }

        return theIndex - aNumVertices;
    }

    /// <summary>
    /// The index (before swapping) in the triangulation of its shape of a triangle of the whole set (after swapping).
    /// </summary>
    public int GetTrgIdxInShape(int theIndex)
    {
        int aShID = GetFaceID(theIndex);
        int aNumTriangles = 0;

        for (int aSIdx = 0; aSIdx < aShID; aSIdx++)
        {
            aNumTriangles += myNumTrgInShapeVec.Value(aSIdx);
        }

        return myTrgIdxMap[theIndex] - aNumTriangles;
    }

    /// <summary>All the vertices: the set's own array (OCCT's const reference).</summary>
    public NCollection_LinearVector<NCollection_Vec3<double>> GetVertices() => myVertexArray;

    /// <summary>The vertices of the given triangle.</summary>
    public void GetVertices(int theIndex,
                            out NCollection_Vec3<double> theVertex1,
                            out NCollection_Vec3<double> theVertex2,
                            out NCollection_Vec3<double> theVertex3)
    {
        NCollection_Vec4<int> aTriangle = myTriangles[theIndex];

        theVertex1 = myVertexArray[aTriangle.X()];
        theVertex2 = myVertexArray[aTriangle.Y()];
        theVertex3 = myVertexArray[aTriangle.Z()];
    }

    /// <summary>The vertex indices of the given triangle, in a new array indexed from 0.</summary>
    public void GetVtxIndices(int theIndex, out NCollection_Array1<int> theVtxIndices)
    {
        NCollection_Vec4<int> aTriangle = myTriangles[theIndex];
        theVtxIndices = new NCollection_Array1<int>(0, 2);
        theVtxIndices.SetValue(0, aTriangle.X());
        theVtxIndices.SetValue(1, aTriangle.Y());
        theVtxIndices.SetValue(2, aTriangle.Z());
    }

    /// <summary>Clears the triangles and the vertices.</summary>
    public void Clear()
    {
        myTriangles.Clear(true);
        myVertexArray.Clear(true);
        // OCCT: myShapeIdxOfVtxVec, myNumVtxInShapeVec and myNumTrgInShapeVec are not cleared, so after a second
        // Init the shape of a vertex is read from the first set's entries (occt-issues.md)
    }

    /// <summary>
    /// Initialises the set from the shapes (faces with triangulations, edges with 3D polygons) and builds its BVH.
    /// As in OCCT, it stops at the first shape without a mesh and still returns true.
    /// </summary>
    public bool Init(NCollection_DynamicArray<TopoDS_Shape> theShapes)
    {
        Clear();

        bool isOK = true; // OCCT: isOK stops the loop at a shape without a mesh, but Init returns true regardless
        for (int aShapeIdx = 0; aShapeIdx < theShapes.Length() && isOK; ++aShapeIdx)
        {
            if (theShapes[aShapeIdx].ShapeType() == TopAbs_FACE)
            {
                isOK = initFace(TopoDS.Face(theShapes[aShapeIdx]), aShapeIdx);
            }
            else if (theShapes[aShapeIdx].ShapeType() == TopAbs_EDGE)
            {
                isOK = initEdge(TopoDS.Edge(theShapes[aShapeIdx]), aShapeIdx);
            }
        }

        int aNumTrg = myTriangles.Size();
        myTrgIdxMap.Clear();
        // OCCT reserves aNumTrg buckets (ReSize): the port's map grows by itself
        for (int aTrgIdx = 0; aTrgIdx < aNumTrg; ++aTrgIdx)
        {
            myTrgIdxMap.Bind(aTrgIdx, aTrgIdx);
        }

        MarkDirty(); // needs BVH rebuilding

        // Standard_ASSERT_RETURN: "Error: Failed to build BVH for primitive set"
        if (BVH() is null)
        {
            return false;
        }
        return true;
    }

    private bool initFace(in TopoDS_Face theFace, int theIndex)
    {
        Triangulation? aTriangulation = BRep_Tool.Triangulation(theFace, out Placement aLocation);
        if (aTriangulation is null)
        {
            return false;
        }

        int aVertOffset = myVertexArray.Size() - 1;

        initNodes(aTriangulation.MapNodeArray()!.ChangeArray1(), aLocation.Transformation(), theIndex);

        for (int aTriIdx = 1; aTriIdx <= aTriangulation.TriangleCount; ++aTriIdx)
        {
            aTriangulation.Triangle(aTriIdx).Get(out int aVertex1, out int aVertex2, out int aVertex3);

            myTriangles.Append(new NCollection_Vec4<int>(aVertex1 + aVertOffset,
                                                         aVertex2 + aVertOffset,
                                                         aVertex3 + aVertOffset,
                                                         theIndex));
        }

        myNumTrgInShapeVec.SetValue(theIndex, aTriangulation.TriangleCount);

        return true;
    }

    private bool initEdge(in TopoDS_Edge theEdge, int theIndex)
    {
        Polygon3D? aPolygon = BRep_Tool.Polygon3D(theEdge, out Placement aLocation);
        if (aPolygon is null)
        {
            return false;
        }

        int aVertOffset = myVertexArray.Size() - 1;

        initNodes(aPolygon.Nodes(), aLocation.Transformation(), theIndex);

        for (int aVertIdx = 1; aVertIdx < aPolygon.NodeCount; ++aVertIdx)
        {
            // segment as degenerate triangle
            myTriangles.Append(new NCollection_Vec4<int>(aVertIdx + aVertOffset,
                                                         aVertIdx + aVertOffset + 1,
                                                         aVertIdx + aVertOffset + 1,
                                                         theIndex));
        }
        return true;
    }

    private void initNodes(NCollection_Array1<Point3D> theNodes, in Transform3D theTrsf, int theIndex)
    {
        for (int aVertIdx = 1; aVertIdx <= theNodes.Length(); ++aVertIdx)
        {
            Point3D aVertex = theNodes.Value(aVertIdx);

            aVertex.Transform(theTrsf);

            myVertexArray.Append(new NCollection_Vec3<double>(aVertex.X(), aVertex.Y(), aVertex.Z()));
            myShapeIdxOfVtxVec.Append(theIndex);
        }

        myNumVtxInShapeVec.SetValue(theIndex, theNodes.Length());
    }
}
