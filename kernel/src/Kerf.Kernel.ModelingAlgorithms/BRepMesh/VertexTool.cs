// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_VertexTool.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_VertexTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

namespace Kerf.Engine.Meshing;

/// <summary>
/// The vertices of a mesh, with a cell filter to find a vertex coinciding with a point within a tolerance. Indices
/// start at 1; a deleted vertex's index is reused.
/// </summary>
/// <remarks>A working object, as OCCT's (the allocator argument is dropped).</remarks>
internal sealed class VertexTool : Transient
{
    private readonly CellFilter<MeshingVertexInspector, int, Xy> _cellFilter;
    private MeshingVertexInspector _selector;
    private double _tolerance0;
    private double _tolerance1;

    /// <summary>An empty tool with the tolerance and cell size of <c>Precision::Confusion()</c>.</summary>
    public VertexTool()
    {
        _cellFilter = new CellFilter<MeshingVertexInspector, int, Xy>(0.0);
        _selector = new MeshingVertexInspector();
        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        SetCellSize(tol + 0.05 * tol);
        SetTolerance(tol, tol);
    }

    /// <summary>Sets the size of the cells, the same in both directions; clears the cells.</summary>
    public void SetCellSize(double size)
    {
        _cellFilter.Reset(size);
        _selector.Clear();
    }

    /// <summary>Sets the size of the cells in each direction; clears the cells.</summary>
    public void SetCellSize(double sizeX, double sizeY)
    {
        NCollection_Array1<double> cellSize = new(1, 2);
        cellSize[1] = sizeX;
        cellSize[2] = sizeY;
        _cellFilter.Reset(cellSize);
        _selector.Clear();
    }

    /// <summary>Sets the tolerance, the same in both directions.</summary>
    public void SetTolerance(double tolerance)
    {
        _selector.SetTolerance(tolerance);
        _tolerance0 = tolerance;
        _tolerance1 = tolerance;
    }

    /// <summary>Sets the tolerance in each direction.</summary>
    public void SetTolerance(double toleranceX, double toleranceY)
    {
        _selector.SetTolerance(toleranceX, toleranceY);
        _tolerance0 = toleranceX;
        _tolerance1 = toleranceY;
    }

    /// <summary>The tolerance in each direction.</summary>
    public void GetTolerance(out double toleranceX, out double toleranceY)
    {
        toleranceX = _tolerance0;
        toleranceY = _tolerance1;
    }

    /// <summary>
    /// Adds the vertex unless one coincides with it within the tolerance (or <paramref name="isForceAdd"/>); returns
    /// the index of the vertex added or found.
    /// </summary>
    public int Add(in MeshVertex vertex, bool isForceAdd)
    {
        int index = isForceAdd ? 0 : FindIndex(vertex);
        if (index == 0)
        {
            index = _selector.Add(vertex);

            ExpandPoint(vertex.Coord(), out Xy minPnt, out Xy maxPnt);
            _cellFilter.Add(index, minPnt, maxPnt);
        }

        return index;
    }

    /// <summary>Deletes the vertex of the given index.</summary>
    public void DeleteVertex(int index)
    {
        ref MeshVertex v = ref _selector.GetVertex(index);

        ExpandPoint(v.Coord(), out Xy minPnt, out Xy maxPnt);

        _cellFilter.Remove(index, minPnt, maxPnt);
        _selector.Delete(index);
    }

    /// <summary>The vertices.</summary>
    public List<MeshVertex> Vertices() => _selector.Vertices();

    /// <summary>The vertices' array, for replacing.</summary>
    public ref List<MeshVertex> ChangeVertices() => ref _selector.ChangeVertices();

    /// <summary>The vertex of the given index.</summary>
    public MeshVertex FindKey(int index) => _selector.Vertex(index);

    /// <summary>The index of the vertex nearest to the given one within the tolerance, or 0.</summary>
    public int FindIndex(in MeshVertex vertex)
    {
        _selector.SetPoint(vertex.Coord());
        _cellFilter.Inspect(vertex.Coord(), ref _selector);
        return _selector.GetCoincidentPoint();
    }

    /// <summary>The number of vertices (deleted ones included).</summary>
    public int Extent() => _selector.VertexCount;

    /// <summary>Whether there is no vertex.</summary>
    public bool IsEmpty => Extent() == 0;

    /// <summary>Replaces the vertex of the given index.</summary>
    public void Substitute(int index, in MeshVertex vertex)
    {
        ref MeshVertex v = ref _selector.GetVertex(index);

        ExpandPoint(v.Coord(), out Xy minPnt, out Xy maxPnt);

        _cellFilter.Remove(index, minPnt, maxPnt);

        v = vertex;
        ExpandPoint(v.Coord(), out minPnt, out maxPnt);
        _cellFilter.Add(index, minPnt, maxPnt);
    }

    /// <summary>Deletes the last vertex.</summary>
    public void RemoveLast() => DeleteVertex(Extent());

    /// <summary>The indices of the deleted vertices, reused by the next additions.</summary>
    public NCollection_List<int> GetListOfDelNodes() => _selector.GetListOfDelPoints();

    private void ExpandPoint(in Xy point, out Xy minPoint, out Xy maxPoint)
    {
        minPoint = new Xy(point.X() - _tolerance0, point.Y() - _tolerance1);
        maxPoint = new Xy(point.X() + _tolerance0, point.Y() + _tolerance1);
    }
}
