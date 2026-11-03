// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_VertexInspector.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_VertexTool.cxx (BRepMesh_VertexInspector::Inspect)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2011-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Geometry;
using static Occt.BRepMesh_DegreeOfFreedom;
using static Kerf.Engine.Collections.CellFilterAction;

namespace Occt;

/// <summary>
/// The cell filter's inspector of mesh vertices: stores the vertices (indices from 1) and finds the one nearest to a
/// point within a tolerance.
/// </summary>
/// <remarks>A working object, as OCCT's (the allocator argument is dropped).</remarks>
public sealed class BRepMesh_VertexInspector : ICellFilterInspector<int, Xy>
{
    private int myIndex;
    private double myMinSqDist;
    private double myTolerance0;
    private double myTolerance1;
    private NCollection_DynamicArray<BRepMesh_Vertex> myVertices;
    private readonly NCollection_List<int> myDelNodes;
    private Xy myPoint;

    public static int Dimension => 2;

    /// <summary>The i-th coordinate (from 0) of the point.</summary>
    public static double Coord(int i, Xy thePnt) => thePnt.Coord(i + 1);

    /// <summary>The point shifted by the tolerance in both directions.</summary>
    public static Xy Shift(in Xy thePnt, double theTol) => new(thePnt.X() + theTol, thePnt.Y() + theTol);

    /// <summary>An empty inspector with the tolerance <c>Precision::Confusion()</c>.</summary>
    public BRepMesh_VertexInspector()
    {
        myIndex = 0;
        myMinSqDist = RealLast();
        myVertices = new NCollection_DynamicArray<BRepMesh_Vertex>();
        myDelNodes = new NCollection_List<int>();
        SetTolerance(global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>Stores a vertex, reusing the first deleted index if any; returns its index.</summary>
    public int Add(in BRepMesh_Vertex theVertex)
    {
        if (myDelNodes.IsEmpty())
        {
            myVertices.Append(theVertex);
            return myVertices.Length();
        }

        int aNodeIndex = myDelNodes.First();
        myVertices.ChangeValue(aNodeIndex - 1) = theVertex;
        myDelNodes.RemoveFirst();
        return aNodeIndex;
    }

    /// <summary>Sets one tolerance for both directions (compared with the distance).</summary>
    public void SetTolerance(double theTolerance)
    {
        myTolerance0 = theTolerance * theTolerance;
        myTolerance1 = 0.0;
    }

    /// <summary>Sets a tolerance per direction.</summary>
    public void SetTolerance(double theToleranceX, double theToleranceY)
    {
        myTolerance0 = theToleranceX * theToleranceX;
        myTolerance1 = theToleranceY * theToleranceY;
    }

    /// <summary>Removes every vertex.</summary>
    public void Clear()
    {
        myVertices.Clear();
        myDelNodes.Clear();
    }

    /// <summary>Marks the vertex of the given index deleted; its index is reused by the next <see cref="Add"/>.</summary>
    public void Delete(int theIndex)
    {
        myVertices.ChangeValue(theIndex - 1).SetMovability(BRepMesh_Deleted);
        myDelNodes.Append(theIndex);
    }

    /// <summary>The number of vertices stored (deleted ones included).</summary>
    public int NbVertices() => myVertices.Length();

    /// <summary>The vertex of the given index (from 1).</summary>
    public BRepMesh_Vertex Vertex(int theIndex) => myVertices.Value(theIndex - 1);

    /// <summary>The vertex of the given index (from 1), for writing.</summary>
    public ref BRepMesh_Vertex GetVertex(int theIndex) => ref myVertices.ChangeValue(theIndex - 1);

    /// <summary>Sets the point to search for, and clears the result.</summary>
    public void SetPoint(in Xy thePoint)
    {
        myIndex = 0;
        myMinSqDist = RealLast();
        myPoint = thePoint;
    }

    /// <summary>The index of the vertex found nearest to the point within the tolerance, or 0.</summary>
    public int GetCoincidentPoint() => myIndex;

    /// <summary>The indices of the deleted vertices, to be reused.</summary>
    public NCollection_List<int> GetListOfDelPoints() => myDelNodes;

    /// <summary>The vertices.</summary>
    public NCollection_DynamicArray<BRepMesh_Vertex> Vertices() => myVertices;

    /// <summary>The vertices' array, for replacing.</summary>
    public ref NCollection_DynamicArray<BRepMesh_Vertex> ChangeVertices() => ref myVertices;

    /// <summary>Keeps the vertex if it is the nearest so far within the tolerance; purges deleted vertices.</summary>
    public CellFilterAction Inspect(int theTarget)
    {
        BRepMesh_Vertex aVertex = myVertices.Value(theTarget - 1);
        if (aVertex.Movability() == BRepMesh_Deleted)
        {
            myDelNodes.Append(theTarget);
            return CellFilter_Purge;
        }

        Xy aVec = (myPoint - aVertex.Coord());
        bool inTol;
        if (Math.Abs(myTolerance1) < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            inTol = aVec.SquareModulus() < myTolerance0;
        }
        else
        {
            inTol = ((aVec.X() * aVec.X()) < myTolerance0) && ((aVec.Y() * aVec.Y()) < myTolerance1);
        }

        if (inTol)
        {
            double aSqDist = aVec.SquareModulus();
            if (aSqDist < myMinSqDist)
            {
                myMinSqDist = aSqDist;
                myIndex = theTarget;
            }
        }

        return CellFilter_Keep;
    }

    /// <summary>Whether two targets are the same vertex.</summary>
    public static bool IsEqual(int theIndex, int theTargetIndex) => theIndex == theTargetIndex;
}
