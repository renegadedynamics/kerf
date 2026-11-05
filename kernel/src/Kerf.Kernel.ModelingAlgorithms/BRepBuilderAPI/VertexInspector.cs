// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_VertexInspector.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_Sewing.cxx (BRepBuilderAPI_VertexInspector::Inspect)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2011-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;

using Kerf.Engine.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Construction.Api;

/// <summary>
/// Inspector for <see cref="CellFilter{TInspector, TTarget, TPoint}"/> working with gp_XYZ points in 3D
/// space: finds the points coinciding with the current one within a tolerance (per coordinate).
/// </summary>
/// <remarks>A working object, as OCCT's. Targets are 1-based indexes into the points added with <see cref="Add"/>.</remarks>
internal sealed class VertexInspector : ICellFilterInspector<int, Xyz>
{
    private readonly double _tol;
    private readonly NCollection_List<int> _resInd = new();
    private readonly List<Xyz> _points = new();
    private Xyz _current;

    public static int Dimension => 3;

    /// <summary>The i-th coordinate (from 0) of the point.</summary>
    public static double Coord(int i, Xyz pnt) => pnt.Coord(i + 1);

    /// <summary>The point shifted by the tolerance along each axis.</summary>
    public static Xyz Shift(in Xyz pnt, double tol) =>
        new(pnt.X() + tol, pnt.Y() + tol, pnt.Z() + tol);

    /// <summary>An inspector for the tolerance <paramref name="tol"/>.</summary>
    public VertexInspector(double tol)
    {
        _tol = tol * tol;
    }

    /// <summary>Keeps a point used for comparison.</summary>
    public void Add(in Xyz pnt) => _points.Add(pnt);

    /// <summary>Clears the list of adjacent points.</summary>
    public void ClearResList() => _resInd.Clear();

    /// <summary>Sets the current point to search coincidences for.</summary>
    public void SetCurrent(in Xyz curPnt) => _current = curPnt;

    /// <summary>The indexes of the points adjacent to the current one (the inspector's own list).</summary>
    public NCollection_List<int> ResInd() => _resInd;

    /// <summary>Keeps the target if it is within the tolerance of the current point along each axis.</summary>
    public CellFilterAction Inspect(int target)
    {
        Xyz pnt = _points[target - 1];
        double dx, dy, dz;
        dx = _current.X() - pnt.X();
        dy = _current.Y() - pnt.Y();
        dz = _current.Z() - pnt.Z();

        if ((dx * dx <= _tol) && (dy * dy <= _tol) && (dz * dz <= _tol))
        {
            _resInd.Append(target);
        }
        return CellFilterAction.CellFilter_Keep;
    }
}
