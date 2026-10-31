// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_VertexInspector.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_Sewing.cxx (BRepBuilderAPI_VertexInspector::Inspect)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2011-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Inspector for <see cref="NCollection_CellFilter{TInspector, TTarget, TPoint}"/> working with gp_XYZ points in 3D
/// space: finds the points coinciding with the current one within a tolerance (per coordinate).
/// </summary>
/// <remarks>A working object, as OCCT's. Targets are 1-based indexes into the points added with <see cref="Add"/>.</remarks>
public sealed class BRepBuilderAPI_VertexInspector : INCollection_CellFilter_Inspector<int, gp_XYZ>
{
    private readonly double myTol;
    private readonly NCollection_List<int> myResInd = new();
    private readonly NCollection_DynamicArray<gp_XYZ> myPoints = new();
    private gp_XYZ myCurrent;

    public static int Dimension => 3;

    /// <summary>The i-th coordinate (from 0) of the point.</summary>
    public static double Coord(int i, gp_XYZ thePnt) => thePnt.Coord(i + 1);

    /// <summary>The point shifted by the tolerance along each axis.</summary>
    public static gp_XYZ Shift(in gp_XYZ thePnt, double theTol) =>
        new(thePnt.X() + theTol, thePnt.Y() + theTol, thePnt.Z() + theTol);

    /// <summary>An inspector for the tolerance <paramref name="theTol"/>.</summary>
    public BRepBuilderAPI_VertexInspector(double theTol)
    {
        myTol = theTol * theTol;
    }

    /// <summary>Keeps a point used for comparison.</summary>
    public void Add(in gp_XYZ thePnt) => myPoints.Append(thePnt);

    /// <summary>Clears the list of adjacent points.</summary>
    public void ClearResList() => myResInd.Clear();

    /// <summary>Sets the current point to search coincidences for.</summary>
    public void SetCurrent(in gp_XYZ theCurPnt) => myCurrent = theCurPnt;

    /// <summary>The indexes of the points adjacent to the current one (the inspector's own list).</summary>
    public NCollection_List<int> ResInd() => myResInd;

    /// <summary>Keeps the target if it is within the tolerance of the current point along each axis.</summary>
    public NCollection_CellFilter_Action Inspect(int theTarget)
    {
        gp_XYZ aPnt = myPoints.Value(theTarget - 1);
        double aDx, aDy, aDz;
        aDx = myCurrent.X() - aPnt.X();
        aDy = myCurrent.Y() - aPnt.Y();
        aDz = myCurrent.Z() - aPnt.Z();

        if ((aDx * aDx <= myTol) && (aDy * aDy <= myTol) && (aDz * aDz <= myTol))
        {
            myResInd.Append(theTarget);
        }
        return NCollection_CellFilter_Action.CellFilter_Keep;
    }
}
