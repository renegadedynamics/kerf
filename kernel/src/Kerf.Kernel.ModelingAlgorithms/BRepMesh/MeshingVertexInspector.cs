// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_VertexInspector.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_VertexTool.cxx (BRepMesh_VertexInspector::Inspect)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2011-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Geometry;
using Kerf.Engine;
using System.Runtime.InteropServices;
using static Kerf.Engine.Meshing.DegreeOfFreedom;
using static Kerf.Engine.Collections.CellFilterAction;

namespace Kerf.Engine.Meshing;

/// <summary>
/// The cell filter's inspector of mesh vertices: stores the vertices (indices from 1) and finds the one nearest to a
/// point within a tolerance.
/// </summary>
/// <remarks>A working object, as OCCT's (the allocator argument is dropped).</remarks>
internal sealed class MeshingVertexInspector : ICellFilterInspector<int, Xy>
{
    private int _index;
    private double _minSqDist;
    private double _tolerance0;
    private double _tolerance1;
    private List<MeshVertex> _vertices;
    private readonly NCollection_List<int> _delNodes;
    private Xy _point;

    public static int Dimension => 2;

    /// <summary>The i-th coordinate (from 0) of the point.</summary>
    public static double Coord(int i, Xy pnt) => pnt.Coord(i + 1);

    /// <summary>The point shifted by the tolerance in both directions.</summary>
    public static Xy Shift(in Xy pnt, double tol) => new(pnt.X() + tol, pnt.Y() + tol);

    /// <summary>An empty inspector with the tolerance <c>Precision::Confusion()</c>.</summary>
    public MeshingVertexInspector()
    {
        _index = 0;
        _minSqDist = RealLast();
        _vertices = new List<MeshVertex>();
        _delNodes = new NCollection_List<int>();
        SetTolerance(global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>Stores a vertex, reusing the first deleted index if any; returns its index.</summary>
    public int Add(in MeshVertex vertex)
    {
        if (_delNodes.IsEmpty())
        {
            _vertices.Add(vertex);
            return _vertices.Count;
        }

        int nodeIndex = _delNodes.First();
        _vertices[nodeIndex - 1] = vertex;
        _delNodes.RemoveFirst();
        return nodeIndex;
    }

    /// <summary>Sets one tolerance for both directions (compared with the distance).</summary>
    public void SetTolerance(double tolerance)
    {
        _tolerance0 = tolerance * tolerance;
        _tolerance1 = 0.0;
    }

    /// <summary>Sets a tolerance per direction.</summary>
    public void SetTolerance(double toleranceX, double toleranceY)
    {
        _tolerance0 = toleranceX * toleranceX;
        _tolerance1 = toleranceY * toleranceY;
    }

    /// <summary>Removes every vertex.</summary>
    public void Clear()
    {
        _vertices.Clear();
        _delNodes.Clear();
    }

    /// <summary>Marks the vertex of the given index deleted; its index is reused by the next <see cref="Add"/>.</summary>
    public void Delete(int index)
    {
        CollectionsMarshal.AsSpan(_vertices)[index - 1].SetMovability(DegreeOfFreedom.Deleted);
        _delNodes.Append(index);
    }

    /// <summary>The number of vertices stored (deleted ones included).</summary>
    public int VertexCount => _vertices.Count;

    /// <summary>The vertex of the given index (from 1).</summary>
    public MeshVertex Vertex(int index) => _vertices[index - 1];

    /// <summary>The vertex of the given index (from 1), for writing.</summary>
    public ref MeshVertex GetVertex(int index) => ref CollectionsMarshal.AsSpan(_vertices)[index - 1];

    /// <summary>Sets the point to search for, and clears the result.</summary>
    public void SetPoint(in Xy point)
    {
        _index = 0;
        _minSqDist = RealLast();
        _point = point;
    }

    /// <summary>The index of the vertex found nearest to the point within the tolerance, or 0.</summary>
    public int GetCoincidentPoint() => _index;

    /// <summary>The indices of the deleted vertices, to be reused.</summary>
    public NCollection_List<int> GetListOfDelPoints() => _delNodes;

    /// <summary>The vertices.</summary>
    public List<MeshVertex> Vertices() => _vertices;

    /// <summary>The vertices' array, for replacing.</summary>
    public ref List<MeshVertex> ChangeVertices() => ref _vertices;

    /// <summary>Keeps the vertex if it is the nearest so far within the tolerance; purges deleted vertices.</summary>
    public CellFilterAction Inspect(int target)
    {
        MeshVertex vertex = _vertices[target - 1];
        if (vertex.Movability() == DegreeOfFreedom.Deleted)
        {
            _delNodes.Append(target);
            return CellFilter_Purge;
        }

        Xy vec = (_point - vertex.Coord());
        bool inTol;
        if (Math.Abs(_tolerance1) < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            inTol = vec.SquareModulus() < _tolerance0;
        }
        else
        {
            inTol = ((vec.X() * vec.X()) < _tolerance0) && ((vec.Y() * vec.Y()) < _tolerance1);
        }

        if (inTol)
        {
            double sqDist = vec.SquareModulus();
            if (sqDist < _minSqDist)
            {
                _minSqDist = sqDist;
                _index = target;
            }
        }

        return CellFilter_Keep;
    }

    /// <summary>Whether two targets are the same vertex.</summary>
    public static bool IsEqual(int index, int targetIndex) => index == targetIndex;
}
