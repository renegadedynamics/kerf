// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_CircleInspector.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2008-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Geometry;
using System.Runtime.InteropServices;
using static Kerf.Engine.Collections.CellFilterAction;

namespace Kerf.Engine.Meshing;

/// <summary>
/// The cell filter's inspector of circles: finds the circles (circumcircles of triangles, by index) that contain a
/// point within a tolerance.
/// </summary>
/// <remarks>A working object, as OCCT's (the allocator argument is dropped).</remarks>
internal struct CircleInspector : ICellFilterInspector<int, Xy>
{
    private readonly double _sqTolerance;
    private readonly List<int> _resIndices;
    private readonly List<MeshCircle> _circles;
    private Xy _point;

    public static int Dimension => 2;

    /// <summary>The i-th coordinate (from 0) of the point.</summary>
    public static double Coord(int i, Xy pnt) => pnt.Coord(i + 1);

    /// <summary>The point shifted by the tolerance in both directions.</summary>
    public static Xy Shift(in Xy pnt, double tol) => new(pnt.X() + tol, pnt.Y() + tol);

    /// <summary>An inspector with the given linear tolerance, reserving room for <paramref name="reservedSize"/>
    /// circles.</summary>
    public CircleInspector(double tolerance, int reservedSize, List<MeshCircle>? circles = null)
    {
        _sqTolerance = tolerance * tolerance;
        _resIndices = [];
        _circles = circles ?? new List<MeshCircle>(reservedSize);
        _circles.EnsureCapacity(reservedSize);
    }

    /// <summary>Binds the circle to the index.</summary>
    public void Bind(int index, in MeshCircle circle)
    {
        _circles.SetAt(index, circle);
    }

    /// <summary>The circles, by index.</summary>
    public List<MeshCircle> Circles() => _circles;

    /// <summary>The circle of the given index, for writing.</summary>
    public ref MeshCircle Circle(int index) => ref CollectionsMarshal.AsSpan(_circles)[index];

    /// <summary>Sets the point to inspect and clears the result.</summary>
    public void SetPoint(in Xy point)
    {
        _resIndices.Clear();
        _point = point;
    }

    /// <summary>The indices of the circles found to contain the point (the inspector's own list).</summary>
    public List<int> GetShotCircles() => _resIndices;

    /// <summary>Adds the circle to the result if it contains the point; purges deleted circles.</summary>
    public CellFilterAction Inspect(int targetIndex)
    {
        MeshCircle circle = _circles[targetIndex];
        double radius = circle.Radius();
        if (radius < 0.0)
        {
            return CellFilter_Purge;
        }

        Xy loc = circle.Location();

        double dx = _point.Coord(1) - loc.Coord(1);
        double dy = _point.Coord(2) - loc.Coord(2);

        // OCCT's own comment: this check is wrong, the input tolerance being linear, it should compare with
        // (aRadius + aToler)^2; changing it hangs the test case "perf mesh bug27119", so it stays.
        if ((dx * dx + dy * dy) - (radius * radius) <= _sqTolerance)
        {
            _resIndices.Add(targetIndex);
        }

        return CellFilter_Keep;
    }

    /// <summary>Whether two targets are the same circle.</summary>
    public static bool IsEqual(int index, int targetIndex) => index == targetIndex;
}
