// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_VertexInspector.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_Sewing.cxx (BRepBuilderAPI_VertexInspector::Inspect)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2011-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;

using Kerf.Engine.Geometry;

namespace Occt;

/// <summary>
/// Inspector for <see cref="CellFilter{TInspector, TTarget, TPoint}"/> working with gp_XYZ points in 3D
/// space: finds the points coinciding with the current one within a tolerance (per coordinate).
/// </summary>
/// <remarks>A working object, as OCCT's. Targets are 1-based indexes into the points added with <see cref="Add"/>.</remarks>
public sealed class BRepBuilderAPI_VertexInspector : ICellFilterInspector<int, Xyz>
{
    private readonly double myTol;
    private readonly NCollection_List<int> myResInd = new();
    private readonly NCollection_DynamicArray<Xyz> myPoints = new();
    private Xyz myCurrent;

    public static int Dimension => 3;

    /// <summary>The i-th coordinate (from 0) of the point.</summary>
    public static double Coord(int i, Xyz thePnt) => thePnt.Coord(i + 1);

    /// <summary>The point shifted by the tolerance along each axis.</summary>
    public static Xyz Shift(in Xyz thePnt, double theTol) =>
        new(thePnt.X() + theTol, thePnt.Y() + theTol, thePnt.Z() + theTol);

    /// <summary>An inspector for the tolerance <paramref name="theTol"/>.</summary>
    public BRepBuilderAPI_VertexInspector(double theTol)
    {
        myTol = theTol * theTol;
    }

    /// <summary>Keeps a point used for comparison.</summary>
    public void Add(in Xyz thePnt) => myPoints.Append(thePnt);

    /// <summary>Clears the list of adjacent points.</summary>
    public void ClearResList() => myResInd.Clear();

    /// <summary>Sets the current point to search coincidences for.</summary>
    public void SetCurrent(in Xyz theCurPnt) => myCurrent = theCurPnt;

    /// <summary>The indexes of the points adjacent to the current one (the inspector's own list).</summary>
    public NCollection_List<int> ResInd() => myResInd;

    /// <summary>Keeps the target if it is within the tolerance of the current point along each axis.</summary>
    public CellFilterAction Inspect(int theTarget)
    {
        Xyz aPnt = myPoints.Value(theTarget - 1);
        double aDx, aDy, aDz;
        aDx = myCurrent.X() - aPnt.X();
        aDy = myCurrent.Y() - aPnt.Y();
        aDz = myCurrent.Z() - aPnt.Z();

        if ((aDx * aDx <= myTol) && (aDy * aDy <= myTol) && (aDz * aDz <= myTol))
        {
            myResInd.Append(theTarget);
        }
        return CellFilterAction.CellFilter_Keep;
    }
}
