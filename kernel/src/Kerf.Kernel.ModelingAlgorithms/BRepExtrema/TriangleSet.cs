// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_TriangleSet.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_TriangleSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2014-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Geometry.Bvh;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Geometry.Bvh.Constants;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Measurement.Distance;

/// <summary>
/// The triangles of the triangulations of faces (and the segments of the 3D polygons of edges, as degenerate
/// triangles) of a list of shapes, in a BVH, for collision detection.
/// </summary>
/// <remarks>
/// A working object (decision 21): OCCT's accessors return its own arrays. It only reads the shapes' meshes. As in
/// OCCT, <see cref="Clear"/> (and so <see cref="Init"/>) doesn't clear the per-shape vectors, so a set initialised
/// twice keeps the first shapes' entries in front (occt-issues.md).
/// </remarks>
public class TriangleSet : PrimitiveSet<double, NCollection_Vec3<double>>
{
    /// <summary>The vertex indices of the triangles (x, y, z) and their shape index (w).</summary>
    protected NCollection_LinearVector<NCollection_Vec4<int>> _triangles = new();

    /// <summary>The vertex coordinates.</summary>
    protected NCollection_LinearVector<NCollection_Vec3<double>> _vertexArray = new();

    /// <summary>The index of the shape of each vertex.</summary>
    protected List<int> _shapeIdxOfVtxVec = new();

    /// <summary>The number of vertices of each shape.</summary>
    protected List<int> _numVtxInShapeVec = new();

    /// <summary>The number of triangles of each shape.</summary>
    protected List<int> _numTrgInShapeVec = new();

    /// <summary>The triangles' indices after (key) and before (value) swapping.</summary>
    protected Dictionary<int, int> _trgIdxMap = new();

    /// <summary>An empty triangle set.</summary>
    public TriangleSet()
        : base(new LinearBuilder<double, NCollection_Vec3<double>>(BVH_Constants_LeafNodeSizeDefault, BVH_Constants_MaxTreeDepth))
    {
        // Set default builder - linear BVH (LBVH)
    }

    /// <summary>A triangle set of the given shapes.</summary>
    public TriangleSet(List<Shape> faces)
        : base(new LinearBuilder<double, NCollection_Vec3<double>>(BVH_Constants_LeafNodeSizeDefault, BVH_Constants_MaxTreeDepth))
    {
        // Set default builder - linear BVH (LBVH)
        Init(faces);
    }

    /// <summary>The total number of triangles.</summary>
    public override int Size() => _triangles.Size();

    /// <summary>The AABB of the given triangle.</summary>
    public override Box<double, NCollection_Vec3<double>> Box(int index)
    {
        NCollection_Vec4<int> triangle = _triangles[index];

        NCollection_Vec3<double> minPnt = _vertexArray[triangle.X()].CwiseMin(
            _vertexArray[triangle.Y()].CwiseMin(_vertexArray[triangle.Z()]));
        NCollection_Vec3<double> maxPnt = _vertexArray[triangle.X()].CwiseMax(
            _vertexArray[triangle.Y()].CwiseMax(_vertexArray[triangle.Z()]));

        return new Box<double, NCollection_Vec3<double>>(minPnt, maxPnt);
    }

    /// <summary>The centroid of the given triangle along the axis.</summary>
    public override double Center(int index, int axis)
    {
        NCollection_Vec4<int> triangle = _triangles[index];

        if (axis == 0)
        {
            return (1.0 / 3.0)
                   * (_vertexArray[triangle.X()].X() + _vertexArray[triangle.Y()].X()
                      + _vertexArray[triangle.Z()].X());
        }
        else if (axis == 1)
        {
            return (1.0 / 3.0)
                   * (_vertexArray[triangle.X()].Y() + _vertexArray[triangle.Y()].Y()
                      + _vertexArray[triangle.Z()].Y());
        }
        else
        {
            return (1.0 / 3.0)
                   * (_vertexArray[triangle.X()].Z() + _vertexArray[triangle.Y()].Z()
                      + _vertexArray[triangle.Z()].Z());
        }
    }

    /// <summary>Swaps two triangles.</summary>
    public override void Swap(int index1, int index2)
    {
        (_triangles[index1], _triangles[index2]) = (_triangles[index2], _triangles[index1]);
        (_trgIdxMap.GetValueRef(index1), _trgIdxMap.GetValueRef(index2)) = (_trgIdxMap.GetValue(index2), _trgIdxMap.GetValue(index1));
    }

    /// <summary>The shape index of the given triangle.</summary>
    public int GetFaceID(int index) => _triangles[index].W();

    /// <summary>The shape index of the given vertex.</summary>
    public int GetShapeIDOfVtx(int index) => _shapeIdxOfVtxVec[index];

    /// <summary>The index of a vertex of the whole set in the triangulation of its shape.</summary>
    public int GetVtxIdxInShape(int index)
    {
        int shID = _shapeIdxOfVtxVec[index];
        int numVertices = 0;

        for (int sIdx = 0; sIdx < shID; sIdx++)
        {
            numVertices += _numVtxInShapeVec[sIdx];
        }

        return index - numVertices;
    }

    /// <summary>
    /// The index (before swapping) in the triangulation of its shape of a triangle of the whole set (after swapping).
    /// </summary>
    public int GetTrgIdxInShape(int index)
    {
        int shID = GetFaceID(index);
        int numTriangles = 0;

        for (int sIdx = 0; sIdx < shID; sIdx++)
        {
            numTriangles += _numTrgInShapeVec[sIdx];
        }

        return _trgIdxMap.GetValue(index) - numTriangles;
    }

    /// <summary>All the vertices: the set's own array (OCCT's const reference).</summary>
    public NCollection_LinearVector<NCollection_Vec3<double>> GetVertices() => _vertexArray;

    /// <summary>The vertices of the given triangle.</summary>
    public void GetVertices(int index,
                            out NCollection_Vec3<double> vertex1,
                            out NCollection_Vec3<double> vertex2,
                            out NCollection_Vec3<double> vertex3)
    {
        NCollection_Vec4<int> triangle = _triangles[index];

        vertex1 = _vertexArray[triangle.X()];
        vertex2 = _vertexArray[triangle.Y()];
        vertex3 = _vertexArray[triangle.Z()];
    }

    /// <summary>The vertex indices of the given triangle, in a new array indexed from 0.</summary>
    public void GetVtxIndices(int index, out NCollection_Array1<int> vtxIndices)
    {
        NCollection_Vec4<int> triangle = _triangles[index];
        vtxIndices = new NCollection_Array1<int>(0, 2);
        vtxIndices.SetValue(0, triangle.X());
        vtxIndices.SetValue(1, triangle.Y());
        vtxIndices.SetValue(2, triangle.Z());
    }

    /// <summary>Clears the triangles and the vertices.</summary>
    public void Clear()
    {
        _triangles.Clear(true);
        _vertexArray.Clear(true);
        // OCCT: myShapeIdxOfVtxVec, myNumVtxInShapeVec and myNumTrgInShapeVec are not cleared, so after a second
        // Init the shape of a vertex is read from the first set's entries (occt-issues.md)
    }

    /// <summary>
    /// Initialises the set from the shapes (faces with triangulations, edges with 3D polygons) and builds its BVH.
    /// As in OCCT, it stops at the first shape without a mesh and still returns true.
    /// </summary>
    public bool Init(List<Shape> shapes)
    {
        Clear();

        bool isOK = true; // OCCT: isOK stops the loop at a shape without a mesh, but Init returns true regardless
        for (int shapeIdx = 0; shapeIdx < shapes.Count && isOK; ++shapeIdx)
        {
            if (shapes[shapeIdx].ShapeType() == ShapeType.Face)
            {
                isOK = InitFace(ShapeCasts.Face(shapes[shapeIdx]), shapeIdx);
            }
            else if (shapes[shapeIdx].ShapeType() == ShapeType.Edge)
            {
                isOK = InitEdge(ShapeCasts.Edge(shapes[shapeIdx]), shapeIdx);
            }
        }

        int numTrg = _triangles.Size();
        _trgIdxMap.Clear();
        // OCCT reserves aNumTrg buckets (ReSize): the port's map grows by itself
        for (int trgIdx = 0; trgIdx < numTrg; ++trgIdx)
        {
            _trgIdxMap[trgIdx] = trgIdx;
        }

        MarkDirty(); // needs BVH rebuilding

        // Standard_ASSERT_RETURN: "Error: Failed to build BVH for primitive set"
        if (BVH() is null)
        {
            return false;
        }
        return true;
    }

    private bool InitFace(in Face face, int index)
    {
        Triangulation? triangulation = Tool.Triangulation(face, out Placement location);
        if (triangulation is null)
        {
            return false;
        }

        int vertOffset = _vertexArray.Size() - 1;

        InitNodes(triangulation.MapNodeArray()!.ChangeArray1(), location.Transformation(), index);

        for (int triIdx = 1; triIdx <= triangulation.TriangleCount; ++triIdx)
        {
            triangulation.Triangle(triIdx).Get(out int vertex1, out int vertex2, out int vertex3);

            _triangles.Append(new NCollection_Vec4<int>(vertex1 + vertOffset,
                                                         vertex2 + vertOffset,
                                                         vertex3 + vertOffset,
                                                         index));
        }

        _numTrgInShapeVec.SetAt(index, triangulation.TriangleCount);

        return true;
    }

    private bool InitEdge(in Edge edge, int index)
    {
        Polygon3D? polygon = Tool.Polygon3D(edge, out Placement location);
        if (polygon is null)
        {
            return false;
        }

        int vertOffset = _vertexArray.Size() - 1;

        InitNodes(polygon.Nodes(), location.Transformation(), index);

        for (int vertIdx = 1; vertIdx < polygon.NodeCount; ++vertIdx)
        {
            // segment as degenerate triangle
            _triangles.Append(new NCollection_Vec4<int>(vertIdx + vertOffset,
                                                         vertIdx + vertOffset + 1,
                                                         vertIdx + vertOffset + 1,
                                                         index));
        }
        return true;
    }

    private void InitNodes(NCollection_Array1<Point3D> nodes, in Transform3D trsf, int index)
    {
        for (int vertIdx = 1; vertIdx <= nodes.Length(); ++vertIdx)
        {
            Point3D vertex = nodes.Value(vertIdx);

            vertex = vertex.Transformed(trsf);

            _vertexArray.Append(new NCollection_Vec3<double>(vertex.X(), vertex.Y(), vertex.Z()));
            _shapeIdxOfVtxVec.Add(index);
        }

        _numVtxInShapeVec.SetAt(index, nodes.Length());
    }
}
