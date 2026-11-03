// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_CircleInspector.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2008-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Geometry;
using static Kerf.Engine.Collections.CellFilterAction;

namespace Occt;

/// <summary>
/// The cell filter's inspector of circles: finds the circles (circumcircles of triangles, by index) that contain a
/// point within a tolerance.
/// </summary>
/// <remarks>A working object, as OCCT's (the allocator argument is dropped).</remarks>
public sealed class BRepMesh_CircleInspector : ICellFilterInspector<int, Xy>
{
    private readonly double mySqTolerance;
    private readonly NCollection_List<int> myResIndices;
    private readonly NCollection_DynamicArray<BRepMesh_Circle> myCircles;
    private Xy myPoint;

    public static int Dimension => 2;

    /// <summary>The i-th coordinate (from 0) of the point.</summary>
    public static double Coord(int i, Xy thePnt) => thePnt.Coord(i + 1);

    /// <summary>The point shifted by the tolerance in both directions.</summary>
    public static Xy Shift(in Xy thePnt, double theTol) => new(thePnt.X() + theTol, thePnt.Y() + theTol);

    /// <summary>An inspector with the given linear tolerance, reserving room for <paramref name="theReservedSize"/>
    /// circles.</summary>
    public BRepMesh_CircleInspector(double theTolerance, int theReservedSize)
    {
        mySqTolerance = theTolerance * theTolerance;
        myResIndices = new NCollection_List<int>();
        myCircles = new NCollection_DynamicArray<BRepMesh_Circle>(theReservedSize);
    }

    /// <summary>Binds the circle to the index.</summary>
    public void Bind(int theIndex, in BRepMesh_Circle theCircle)
    {
        myCircles.SetValue(theIndex, theCircle);
    }

    /// <summary>The circles, by index.</summary>
    public NCollection_DynamicArray<BRepMesh_Circle> Circles() => myCircles;

    /// <summary>The circle of the given index, for writing.</summary>
    public ref BRepMesh_Circle Circle(int theIndex) => ref myCircles[theIndex];

    /// <summary>Sets the point to inspect and clears the result.</summary>
    public void SetPoint(in Xy thePoint)
    {
        myResIndices.Clear();
        myPoint = thePoint;
    }

    /// <summary>The indices of the circles found to contain the point (the inspector's own list).</summary>
    public NCollection_List<int> GetShotCircles() => myResIndices;

    /// <summary>Adds the circle to the result if it contains the point; purges deleted circles.</summary>
    public CellFilterAction Inspect(int theTargetIndex)
    {
        BRepMesh_Circle aCircle = myCircles[theTargetIndex];
        double aRadius = aCircle.Radius();
        if (aRadius < 0.0)
        {
            return CellFilter_Purge;
        }

        Xy aLoc = aCircle.Location();

        double aDX = myPoint.Coord(1) - aLoc.Coord(1);
        double aDY = myPoint.Coord(2) - aLoc.Coord(2);

        // OCCT's own comment: this check is wrong, the input tolerance being linear, it should compare with
        // (aRadius + aToler)^2; changing it hangs the test case "perf mesh bug27119", so it stays.
        if ((aDX * aDX + aDY * aDY) - (aRadius * aRadius) <= mySqTolerance)
        {
            myResIndices.Append(theTargetIndex);
        }

        return CellFilter_Keep;
    }

    /// <summary>Whether two targets are the same circle.</summary>
    public static bool IsEqual(int theIndex, int theTargetIndex) => theIndex == theTargetIndex;
}
