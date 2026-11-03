// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_CircleTool.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_CircleTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The circumcircles of the triangles of a Delaunay triangulation, in a cell filter, to find quickly the triangles
/// whose circumcircle contains a point.
/// </summary>
/// <remarks>
/// A working object, as OCCT's (a class used by value there; the allocator argument is dropped). It isn't shared:
/// <see cref="BRepMesh_Delaun"/> owns one.
/// </remarks>
public sealed class BRepMesh_CircleTool
{
    private double myTolerance;
    private readonly CellFilter<BRepMesh_CircleInspector, int, Xy> myCellFilter;
    private BRepMesh_CircleInspector mySelector;
    private Xy myFaceMax;
    private Xy myFaceMin;

    /// <summary>An empty tool.</summary>
    public BRepMesh_CircleTool()
    {
        myTolerance = global::Kerf.Engine.Foundation.Precision.PConfusion();
        myCellFilter = new CellFilter<BRepMesh_CircleInspector, int, Xy>(10.0);
        mySelector = new BRepMesh_CircleInspector(myTolerance, 64);
    }

    /// <summary>An empty tool reserving room for <paramref name="theReservedSize"/> circles.</summary>
    public BRepMesh_CircleTool(int theReservedSize)
    {
        myTolerance = global::Kerf.Engine.Foundation.Precision.PConfusion();
        myCellFilter = new CellFilter<BRepMesh_CircleInspector, int, Xy>(10.0);
        mySelector = new BRepMesh_CircleInspector(myTolerance, Max(theReservedSize, 64));
    }

    /// <summary>Resets the tolerance (OCCT ignores the reserved size).</summary>
    public void Init(int theReservedSize)
    {
        _ = theReservedSize;
        myTolerance = global::Kerf.Engine.Foundation.Precision.PConfusion();
    }

    /// <summary>Sets the size of the cells, the same in both directions, and clears the cells.</summary>
    public void SetCellSize(double theSize) => myCellFilter.Reset(theSize);

    /// <summary>Sets the size of the cells in each direction, and clears the cells.</summary>
    public void SetCellSize(double theSizeX, double theSizeY)
    {
        NCollection_Array1<double> aCellSize = new(1, 2);
        aCellSize[1] = theSizeX;
        aCellSize[2] = theSizeY;
        myCellFilter.Reset(aCellSize);
    }

    /// <summary>Sets the limits of the face, to which the circles' extents are clipped.</summary>
    public void SetMinMaxSize(in Xy theMin, in Xy theMax)
    {
        myFaceMin = theMin;
        myFaceMax = theMax;
    }

    /// <summary>Whether no circle is bound.</summary>
    public bool IsEmpty() => mySelector.Circles().IsEmpty();

    /// <summary>Binds the circle to the index.</summary>
    public void Bind(int theIndex, Circle2D theCircle)
    {
        Xy aCoord = theCircle.Location().Coord();
        double aRadius = theCircle.Radius();
        bind(theIndex, aCoord, aRadius);
    }

    /// <summary>
    /// Computes the circle through three points: false if two of them are within <c>Precision::PConfusion()</c> or
    /// they are collinear. The radius is the largest distance from the centre to the points plus 2 RealEpsilon().
    /// </summary>
    public static bool MakeCircle(in Xy thePoint1,
                                  in Xy thePoint2,
                                  in Xy thePoint3,
                                  out Xy theLocation,
                                  out double theRadius)
    {
        double aPrecision = global::Kerf.Engine.Foundation.Precision.PConfusion();
        double aSqPrecision = aPrecision * aPrecision;
        theLocation = new Xy();
        theRadius = 0.0;

        Xy aLink1 = new(thePoint3.Coord(1) - thePoint2.Coord(1),
                           thePoint2.Coord(2) - thePoint3.Coord(2));
        if (aLink1.SquareModulus() < aSqPrecision)
        {
            return false;
        }

        Xy aLink2 = new(thePoint1.Coord(1) - thePoint3.Coord(1),
                           thePoint3.Coord(2) - thePoint1.Coord(2));
        if (aLink2.SquareModulus() < aSqPrecision)
        {
            return false;
        }

        Xy aLink3 = new(thePoint2.Coord(1) - thePoint1.Coord(1),
                           thePoint1.Coord(2) - thePoint2.Coord(2));
        if (aLink3.SquareModulus() < aSqPrecision)
        {
            return false;
        }

        double aD = 2
                    * (thePoint1.Coord(1) * aLink1.Y()
                       + thePoint2.Coord(1) * aLink2.Y()
                       + thePoint3.Coord(1) * aLink3.Y());

        if (Math.Abs(aD) < Constants.Resolution())
        {
            return false;
        }

        double aInvD = 1.0 / aD;
        double aSqMod1 = thePoint1.SquareModulus();
        double aSqMod2 = thePoint2.SquareModulus();
        double aSqMod3 = thePoint3.SquareModulus();
        theLocation.ChangeCoord(1) =
            (aSqMod1 * aLink1.Y() + aSqMod2 * aLink2.Y() + aSqMod3 * aLink3.Y()) * aInvD;

        theLocation.ChangeCoord(2) =
            (aSqMod1 * aLink1.X() + aSqMod2 * aLink2.X() + aSqMod3 * aLink3.X()) * aInvD;

        theRadius = Math.Sqrt(Max(Max((thePoint1 - theLocation).SquareModulus(),
                                      (thePoint2 - theLocation).SquareModulus()),
                                  (thePoint3 - theLocation).SquareModulus()))
                    + 2 * RealEpsilon();

        return true;
    }

    /// <summary>Binds the circle through three points to the index; false if <see cref="MakeCircle"/> fails.</summary>
    public bool Bind(int theIndex, in Xy thePoint1, in Xy thePoint2, in Xy thePoint3)
    {
        if (!MakeCircle(thePoint1, thePoint2, thePoint3, out Xy aLocation, out double aRadius))
        {
            return false;
        }

        bind(theIndex, aLocation, aRadius);
        return true;
    }

    /// <summary>Binds a null circle (radius -1) to the index, outside the cells.</summary>
    public void MocBind(int theIndex)
    {
        BRepMesh_Circle aNullCir = new(Constants.Origin2d().Coord(), -1.0);
        mySelector.Bind(theIndex, aNullCir);
    }

    /// <summary>Marks the circle of the given index deleted (radius -1); the cells purge it when they meet it.</summary>
    public void Delete(int theIndex)
    {
        ref BRepMesh_Circle aCircle = ref mySelector.Circle(theIndex);
        if (aCircle.Radius() > 0.0)
        {
            aCircle.SetRadius(-1);
        }
    }

    /// <summary>
    /// The indices of the circles containing the point. The list is the tool's own, valid until the next call; callers
    /// remove items from it, as OCCT's do.
    /// </summary>
    public NCollection_List<int> Select(in Xy thePoint)
    {
        mySelector.SetPoint(thePoint);
        myCellFilter.Inspect(thePoint, ref mySelector);
        return mySelector.GetShotCircles();
    }

    private void bind(int theIndex, in Xy theLocation, double theRadius)
    {
        BRepMesh_Circle aCirle = new(theLocation, theRadius);

        // compute coords
        double aMaxX = Min(theLocation.X() + theRadius, myFaceMax.X());
        double aMinX = Max(theLocation.X() - theRadius, myFaceMin.X());
        double aMaxY = Min(theLocation.Y() + theRadius, myFaceMax.Y());
        double aMinY = Max(theLocation.Y() - theRadius, myFaceMin.Y());

        Xy aMinPnt = new(aMinX, aMinY);
        Xy aMaxPnt = new(aMaxX, aMaxY);

        myCellFilter.Add(theIndex, aMinPnt, aMaxPnt);
        mySelector.Bind(theIndex, aCirle);
    }
}
