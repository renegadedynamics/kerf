// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_VertexTool.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_VertexTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The vertices of a mesh, with a cell filter to find a vertex coinciding with a point within a tolerance. Indices
/// start at 1; a deleted vertex's index is reused.
/// </summary>
/// <remarks>A working object, as OCCT's (the allocator argument is dropped).</remarks>
public sealed class BRepMesh_VertexTool : Standard_Transient
{
    private readonly NCollection_CellFilter<BRepMesh_VertexInspector, int, gp_XY> myCellFilter;
    private BRepMesh_VertexInspector mySelector;
    private double myTolerance0;
    private double myTolerance1;

    /// <summary>An empty tool with the tolerance and cell size of <c>Precision::Confusion()</c>.</summary>
    public BRepMesh_VertexTool()
    {
        myCellFilter = new NCollection_CellFilter<BRepMesh_VertexInspector, int, gp_XY>(0.0);
        mySelector = new BRepMesh_VertexInspector();
        double aTol = Precision.Confusion();
        SetCellSize(aTol + 0.05 * aTol);
        SetTolerance(aTol, aTol);
    }

    /// <summary>Sets the size of the cells, the same in both directions; clears the cells.</summary>
    public void SetCellSize(double theSize)
    {
        myCellFilter.Reset(theSize);
        mySelector.Clear();
    }

    /// <summary>Sets the size of the cells in each direction; clears the cells.</summary>
    public void SetCellSize(double theSizeX, double theSizeY)
    {
        NCollection_Array1<double> aCellSize = new(1, 2);
        aCellSize[1] = theSizeX;
        aCellSize[2] = theSizeY;
        myCellFilter.Reset(aCellSize);
        mySelector.Clear();
    }

    /// <summary>Sets the tolerance, the same in both directions.</summary>
    public void SetTolerance(double theTolerance)
    {
        mySelector.SetTolerance(theTolerance);
        myTolerance0 = theTolerance;
        myTolerance1 = theTolerance;
    }

    /// <summary>Sets the tolerance in each direction.</summary>
    public void SetTolerance(double theToleranceX, double theToleranceY)
    {
        mySelector.SetTolerance(theToleranceX, theToleranceY);
        myTolerance0 = theToleranceX;
        myTolerance1 = theToleranceY;
    }

    /// <summary>The tolerance in each direction.</summary>
    public void GetTolerance(out double theToleranceX, out double theToleranceY)
    {
        theToleranceX = myTolerance0;
        theToleranceY = myTolerance1;
    }

    /// <summary>
    /// Adds the vertex unless one coincides with it within the tolerance (or <paramref name="isForceAdd"/>); returns
    /// the index of the vertex added or found.
    /// </summary>
    public int Add(in BRepMesh_Vertex theVertex, bool isForceAdd)
    {
        int aIndex = isForceAdd ? 0 : FindIndex(theVertex);
        if (aIndex == 0)
        {
            aIndex = mySelector.Add(theVertex);

            expandPoint(theVertex.Coord(), out gp_XY aMinPnt, out gp_XY aMaxPnt);
            myCellFilter.Add(aIndex, aMinPnt, aMaxPnt);
        }

        return aIndex;
    }

    /// <summary>Deletes the vertex of the given index.</summary>
    public void DeleteVertex(int theIndex)
    {
        ref BRepMesh_Vertex aV = ref mySelector.GetVertex(theIndex);

        expandPoint(aV.Coord(), out gp_XY aMinPnt, out gp_XY aMaxPnt);

        myCellFilter.Remove(theIndex, aMinPnt, aMaxPnt);
        mySelector.Delete(theIndex);
    }

    /// <summary>The vertices.</summary>
    public NCollection_DynamicArray<BRepMesh_Vertex> Vertices() => mySelector.Vertices();

    /// <summary>The vertices' array, for replacing.</summary>
    public ref NCollection_DynamicArray<BRepMesh_Vertex> ChangeVertices() => ref mySelector.ChangeVertices();

    /// <summary>The vertex of the given index.</summary>
    public BRepMesh_Vertex FindKey(int theIndex) => mySelector.Vertex(theIndex);

    /// <summary>The index of the vertex nearest to the given one within the tolerance, or 0.</summary>
    public int FindIndex(in BRepMesh_Vertex theVertex)
    {
        mySelector.SetPoint(theVertex.Coord());
        myCellFilter.Inspect(theVertex.Coord(), ref mySelector);
        return mySelector.GetCoincidentPoint();
    }

    /// <summary>The number of vertices (deleted ones included).</summary>
    public int Extent() => mySelector.NbVertices();

    /// <summary>Whether there is no vertex.</summary>
    public bool IsEmpty() => Extent() == 0;

    /// <summary>Replaces the vertex of the given index.</summary>
    public void Substitute(int theIndex, in BRepMesh_Vertex theVertex)
    {
        ref BRepMesh_Vertex aV = ref mySelector.GetVertex(theIndex);

        expandPoint(aV.Coord(), out gp_XY aMinPnt, out gp_XY aMaxPnt);

        myCellFilter.Remove(theIndex, aMinPnt, aMaxPnt);

        aV = theVertex;
        expandPoint(aV.Coord(), out aMinPnt, out aMaxPnt);
        myCellFilter.Add(theIndex, aMinPnt, aMaxPnt);
    }

    /// <summary>Deletes the last vertex.</summary>
    public void RemoveLast() => DeleteVertex(Extent());

    /// <summary>The indices of the deleted vertices, reused by the next additions.</summary>
    public NCollection_List<int> GetListOfDelNodes() => mySelector.GetListOfDelPoints();

    private void expandPoint(in gp_XY thePoint, out gp_XY theMinPoint, out gp_XY theMaxPoint)
    {
        theMinPoint = new gp_XY(thePoint.X() - myTolerance0, thePoint.Y() - myTolerance1);
        theMaxPoint = new gp_XY(thePoint.X() + myTolerance0, thePoint.Y() + myTolerance1);
    }
}
