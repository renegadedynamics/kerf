// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_IntSS.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_IntSS.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.Surfaces;
using Kerf.Geometry;

namespace Kerf.Engine.Geometry.Api;

/// <summary>
/// The intersection of two surfaces: the lines are the approximated (or exact, for conics) intersection curves
/// computed by <see cref="IntSS"/> with approximation on.
/// </summary>
/// <remarks>
/// A working object, as OCCT's. The curves <see cref="Line"/> returns are frozen (design 4.2): they are the
/// algorithm's results handed to the caller.
/// </remarks>
internal class SurfaceSurfaceIntersection
{
    private readonly IntSS _intersec = new();

    /// <summary>An empty algorithm; call <see cref="Perform"/>.</summary>
    public SurfaceSurfaceIntersection() { }

    /// <summary>Intersects S1 and S2 with the tolerance Tol.</summary>
    public SurfaceSurfaceIntersection(Surface s1, Surface s2, double tol)
    {
        _intersec.Perform(s1, s2, tol, true);
    }

    /// <summary>Intersects S1 and S2 with the tolerance Tol.</summary>
    public void Perform(Surface s1, Surface s2, double tol)
    {
        _intersec.Perform(s1, s2, tol, true);
    }

    /// <summary>True if the intersection was computed.</summary>
    public bool IsDone => _intersec.IsDone;

    /// <summary>The number of intersection lines. Throws StdFail_NotDone if not done.</summary>
    public int LineCount => _intersec.LineCount;

    /// <summary>The intersection line of index Index (1 to NbLines), frozen. Throws StdFail_NotDone if not done,
    /// Standard_OutOfRange if Index is out of range.</summary>
    public Curve3D Line(int index) => _intersec.Line(index);
}
