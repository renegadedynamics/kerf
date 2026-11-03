// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_IntSS.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_IntSS.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The intersection of two surfaces: the lines are the approximated (or exact, for conics) intersection curves
/// computed by <see cref="GeomInt_IntSS"/> with approximation on.
/// </summary>
/// <remarks>
/// A working object, as OCCT's. The curves <see cref="Line"/> returns are frozen (design 4.2): they are the
/// algorithm's results handed to the caller.
/// </remarks>
public class GeomAPI_IntSS
{
    private readonly GeomInt_IntSS myIntersec = new();

    /// <summary>An empty algorithm; call <see cref="Perform"/>.</summary>
    public GeomAPI_IntSS() { }

    /// <summary>Intersects S1 and S2 with the tolerance Tol.</summary>
    public GeomAPI_IntSS(Surface S1, Surface S2, double Tol)
    {
        myIntersec.Perform(S1, S2, Tol, true);
    }

    /// <summary>Intersects S1 and S2 with the tolerance Tol.</summary>
    public void Perform(Surface S1, Surface S2, double Tol)
    {
        myIntersec.Perform(S1, S2, Tol, true);
    }

    /// <summary>True if the intersection was computed.</summary>
    public bool IsDone() => myIntersec.IsDone();

    /// <summary>The number of intersection lines. Throws StdFail_NotDone if not done.</summary>
    public int NbLines() => myIntersec.NbLines();

    /// <summary>The intersection line of index Index (1 to NbLines), frozen. Throws StdFail_NotDone if not done,
    /// Standard_OutOfRange if Index is out of range.</summary>
    public Curve3D Line(int Index) => myIntersec.Line(Index);
}
