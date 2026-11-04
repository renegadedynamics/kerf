// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_ExtPC.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_ExtPC.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Measurement.Distance;

/// <summary>The extrema of the distance between a vertex and an edge.</summary>
/// <remarks>A working object (decision 21): initialize with an edge, then perform with any number of vertices.</remarks>
internal sealed class ExtPC
{
    private readonly CurvePointExtrema _extPC = new();
    private EdgeCurveAdaptor? _hc;

    public ExtPC()
    {
    }

    /// <summary>Computes all the extrema between the vertex <paramref name="v"/> and the edge <paramref name="e"/>.</summary>
    public ExtPC(in Vertex v, in Edge e)
    {
        Initialize(e);
        Perform(v);
    }

    public void Initialize(in Edge e)
    {
        if (!Tool.IsGeometric(e))
        {
            return; // protect against non-geometric type (e.g. polygon)
        }
        double u1, u2;
        _hc = new EdgeCurveAdaptor(e);
        double tol = Min(Tool.Tolerance(e), global::Kerf.Engine.Foundation.Precision.Confusion());
        tol = Max(_hc.Resolution(tol), global::Kerf.Engine.Foundation.Precision.PConfusion());
        Tool.Range(e, out u1, out u2);
        _extPC.Initialize(_hc, u1, u2, tol);
    }

    /// <summary>Computes the extrema for the vertex <paramref name="v"/> (nothing if the edge isn't geometric).</summary>
    public void Perform(in Vertex v)
    {
        if (_hc is not null)
        {
            Point3D p = Tool.Pnt(v);
            _extPC.Perform(p);
        }
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone => _extPC.IsDone;

    /// <summary>The number of extremum distances.</summary>
    public int ExtCount => _extPC.ExtCount;

    /// <summary>True if the <paramref name="n"/>th extremum distance is a minimum.</summary>
    public bool IsMin(int n) => _extPC.IsMin(n);

    /// <summary>The <paramref name="n"/>th extremum square distance.</summary>
    public double SquareDistance(int n) => _extPC.SquareDistance(n);

    /// <summary>The parameter on the edge of the <paramref name="n"/>th extremum distance.</summary>
    public double Parameter(int n) => _extPC.Point(n).Parameter();

    /// <summary>The point of the <paramref name="n"/>th extremum distance.</summary>
    public Point3D Point(int n) => _extPC.Point(n).Value();

    /// <summary>For a trimmed curve, the square distances to the points at its first and last parameters.</summary>
    public void TrimmedSquareDistances(out double dist1, out double dist2, out Point3D pnt1, out Point3D pnt2)
        => _extPC.TrimmedSquareDistances(out dist1, out dist2, out pnt1, out pnt2);
}
