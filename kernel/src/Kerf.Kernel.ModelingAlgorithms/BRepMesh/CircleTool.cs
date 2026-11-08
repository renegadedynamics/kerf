// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_CircleTool.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_CircleTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace Kerf.Engine.Meshing;

/// <summary>
/// The circumcircles of the triangles of a Delaunay triangulation, in a cell filter, to find quickly the triangles
/// whose circumcircle contains a point.
/// </summary>
/// <remarks>
/// A working object, as OCCT's (a class used by value there; the allocator argument is dropped). It isn't shared:
/// <see cref="Delaun"/> owns one.
/// </remarks>
internal sealed class CircleTool
{
    private double _tolerance;
    private readonly CellFilter<CircleInspector, int, Xy> _cellFilter;
    private CircleInspector _selector;
    private Xy _faceMax;
    private Xy _faceMin;

    /// <summary>An empty tool.</summary>
    public CircleTool()
    {
        _tolerance = global::Kerf.Engine.Foundation.Precision.PConfusion();
        _cellFilter = new CellFilter<CircleInspector, int, Xy>(10.0);
        _selector = new CircleInspector(_tolerance, 64);
    }

    /// <summary>An empty tool reserving room for <paramref name="reservedSize"/> circles.</summary>
    public CircleTool(int reservedSize, MeshWorkList<MeshCircle>? circles = null)
    {
        _tolerance = global::Kerf.Engine.Foundation.Precision.PConfusion();
        _cellFilter = new CellFilter<CircleInspector, int, Xy>(10.0);
        _selector = new CircleInspector(_tolerance, Max(reservedSize, 64), circles);
    }

    /// <summary>Resets the tolerance (OCCT ignores the reserved size).</summary>
    public void Init(int reservedSize)
    {
        _ = reservedSize;
        _tolerance = global::Kerf.Engine.Foundation.Precision.PConfusion();
    }

    /// <summary>Sets the size of the cells, the same in both directions, and clears the cells.</summary>
    public void SetCellSize(double size) => _cellFilter.Reset(size);

    /// <summary>
    /// Kerf: how many circles in cells a face may hold whatever its mesh, and how many more for each circle (decision
    /// 22801). A circle is entered in every cell it covers, and a range or surface damaged to 1e17 makes cells so small
    /// beside the circles that 200 triangles took 54 million entries, 400 MB, and then all the memory there was. A mesh
    /// that isn't damaged was measured at no more than 360 for each circle past the first 16,384 entries, over the
    /// corpus's meshes (33 million entries at most, 117 a circle). The floor is what a refused face may take, about 450 MB
    /// at the peak; at a quarter of it, two damaged files that main meshed in seconds were refused, and one still is.
    /// </summary>
    internal const int EntriesFloor = 1 << 24, EntriesPerCircle = 1024;

    private long _proportionCheckAt = long.MaxValue;

    /// <summary>Kerf: whether the circles in cells must stay in proportion to the circles (decision 1942).</summary>
    public void LimitToProportion(bool limit) => _proportionCheckAt = limit ? EntriesFloor : long.MaxValue;

    /// <summary>Kerf: how many circles in cells it may hold (decision 1856).</summary>
    public void SetMaxEntries(int entries)
    {
        _cellFilter.MaxNodes = entries;
        // The limit is read against the array's size when it's full, so a larger kept array mustn't put it off
        if (_cellFilter.NodeCapacity > entries)
        {
            _cellFilter.UseNodes([]);
        }
    }

    /// <summary>Starts the cells on an array kept from an earlier face, before any circle is bound.</summary>
    internal void UseCellNodes(CellFilterNode<int>[] nodes) => _cellFilter.UseNodes(nodes);

    /// <summary>The cells' array, to keep for the next face. This tool can't be used again.</summary>
    internal CellFilterNode<int>[] TakeCellNodes() => _cellFilter.TakeNodes();

    /// <summary>Sets the size of the cells in each direction, and clears the cells.</summary>
    public void SetCellSize(double sizeX, double sizeY)
    {
        NCollection_Array1<double> cellSize = new(1, 2);
        cellSize[1] = sizeX;
        cellSize[2] = sizeY;
        _cellFilter.Reset(cellSize);
        _cellFilter.UseWindow(_faceMin, _faceMax);
    }

    /// <summary>Sets the limits of the face, to which the circles' extents are clipped.</summary>
    public void SetMinMaxSize(in Xy min, in Xy max)
    {
        _faceMin = min;
        _faceMax = max;
    }

    /// <summary>Whether no circle is bound.</summary>
    public bool IsEmpty => _selector.Circles().Count == 0;

    /// <summary>Binds the circle to the index.</summary>
    public void Bind(int index, Circle2D circle)
    {
        Xy coord = circle.Location.Coord();
        double radius = circle.Radius;
        BindCore(index, coord, radius);
    }

    /// <summary>
    /// Computes the circle through three points: false if two of them are within <c>Precision::PConfusion()</c> or
    /// they are collinear. The radius is the largest distance from the centre to the points plus 2 RealEpsilon().
    /// </summary>
    public static bool MakeCircle(in Xy point1,
                                  in Xy point2,
                                  in Xy point3,
                                  out Xy location,
                                  out double radius)
    {
        double precision = global::Kerf.Engine.Foundation.Precision.PConfusion();
        double sqPrecision = precision * precision;
        location = new Xy();
        radius = 0.0;

        Xy link1 = new(point3.Coord(1) - point2.Coord(1),
                           point2.Coord(2) - point3.Coord(2));
        if (link1.SquareModulus() < sqPrecision)
        {
            return false;
        }

        Xy link2 = new(point1.Coord(1) - point3.Coord(1),
                           point3.Coord(2) - point1.Coord(2));
        if (link2.SquareModulus() < sqPrecision)
        {
            return false;
        }

        Xy link3 = new(point2.Coord(1) - point1.Coord(1),
                           point1.Coord(2) - point2.Coord(2));
        if (link3.SquareModulus() < sqPrecision)
        {
            return false;
        }

        double d = 2
                    * (point1.Coord(1) * link1.Y()
                       + point2.Coord(1) * link2.Y()
                       + point3.Coord(1) * link3.Y());

        if (Math.Abs(d) < Constants.Resolution())
        {
            return false;
        }

        double invD = 1.0 / d;
        double sqMod1 = point1.SquareModulus();
        double sqMod2 = point2.SquareModulus();
        double sqMod3 = point3.SquareModulus();
        location.ChangeCoord(1) =
            (sqMod1 * link1.Y() + sqMod2 * link2.Y() + sqMod3 * link3.Y()) * invD;

        location.ChangeCoord(2) =
            (sqMod1 * link1.X() + sqMod2 * link2.X() + sqMod3 * link3.X()) * invD;

        radius = Math.Sqrt(Max(Max((point1 - location).SquareModulus(),
                                      (point2 - location).SquareModulus()),
                                  (point3 - location).SquareModulus()))
                    + 2 * RealEpsilon();

        return true;
    }

    /// <summary>Binds the circle through three points to the index; false if <see cref="MakeCircle"/> fails.</summary>
    public bool Bind(int index, in Xy point1, in Xy point2, in Xy point3)
    {
        if (!MakeCircle(point1, point2, point3, out Xy location, out double radius))
        {
            return false;
        }

        BindCore(index, location, radius);
        return true;
    }

    /// <summary>Binds a null circle (radius -1) to the index, outside the cells.</summary>
    public void MocBind(int index)
    {
        MeshCircle nullCir = new(Constants.Origin2d().Coord(), -1.0);
        _selector.Bind(index, nullCir);
    }

    /// <summary>Marks the circle of the given index deleted (radius -1); the cells purge it when they meet it.</summary>
    public void Delete(int index)
    {
        ref MeshCircle circle = ref _selector.Circle(index);
        if (circle.Radius() > 0.0)
        {
            circle.SetRadius(-1);
        }
    }

    /// <summary>
    /// The indices of the circles containing the point, in the order found. The list is the tool's own, valid until the
    /// next call; callers may overwrite its items.
    /// </summary>
    public List<int> Select(in Xy point)
    {
        _selector.SetPoint(point);
        _cellFilter.Inspect(point, ref _selector);
        return _selector.GetShotCircles();
    }

    private void BindCore(int index, in Xy location, double radius)
    {
        MeshCircle cirle = new(location, radius);

        // compute coords
        double maxX = Min(location.X() + radius, _faceMax.X());
        double minX = Max(location.X() - radius, _faceMin.X());
        double maxY = Min(location.Y() + radius, _faceMax.Y());
        double minY = Max(location.Y() - radius, _faceMin.Y());

        Xy minPnt = new(minX, minY);
        Xy maxPnt = new(maxX, maxY);

        _cellFilter.Add(index, minPnt, maxPnt);
        _selector.Bind(index, cirle);
        if (_cellFilter.NodeCount > _proportionCheckAt)
        {
            CheckProportion();
        }
    }

    // Kerf: refuses the face if its circles in cells are out of proportion to its circles; otherwise checks again at the
    // limit for as many circles as there are now, so a mesh that keeps in proportion is asked once for each 1,024 circles
    private void CheckProportion()
    {
        int circles = _selector.Circles().Count;
        long limit = EntriesFloor + (long)EntriesPerCircle * circles;
        if (_cellFilter.NodeCount > limit)
        {
            throw new Tools.MeshFaceRefusedException(
                $"{_cellFilter.NodeCount} circles in cells for {circles} triangles: the face's surface or range is out of proportion to its mesh");
        }

        _proportionCheckAt = limit;
    }
}
