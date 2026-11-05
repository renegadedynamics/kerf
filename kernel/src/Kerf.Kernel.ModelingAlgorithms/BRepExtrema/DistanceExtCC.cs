// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_ExtCC.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_ExtCC.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;

namespace Kerf.Engine.Measurement.Distance;

/// <summary>The extrema of the distance between two edges.</summary>
/// <remarks>
/// A working object (decision 21): initialize with the second edge, then perform with any number of first edges.
/// OCCT copies a local adaptor into a new handle; the port keeps the adaptor it made (decision 394).
/// </remarks>
internal sealed class DistanceExtCC
{
    private readonly ExtCC _extCC = new();
    private EdgeCurveAdaptor? _hc;

    public DistanceExtCC()
    {
    }

    /// <summary>Computes all the extrema between the edges.</summary>
    public DistanceExtCC(in Edge e1, in Edge e2)
    {
        Initialize(e2);
        Perform(e1);
    }

    public void Initialize(in Edge e2)
    {
        if (!Tool.IsGeometric(e2))
        {
            return; // protect against non-geometric type (e.g. polygon)
        }
        double v1, v2;
        var curv = new EdgeCurveAdaptor(e2);
        _hc = curv;
        double tol = Min(Tool.Tolerance(e2), global::Kerf.Engine.Foundation.Precision.Confusion());
        tol = Max(curv.Resolution(tol), global::Kerf.Engine.Foundation.Precision.PConfusion());
        Tool.Range(e2, out v1, out v2);
        _extCC.SetCurve(2, _hc, v1, v2);
        _extCC.SetTolerance(2, tol);
    }

    public void Perform(in Edge e1)
    {
        if (!Tool.IsGeometric(e1))
        {
            return; // protect against non-geometric type (e.g. polygon)
        }
        double u1, u2;
        var curv = new EdgeCurveAdaptor(e1);
        EdgeCurveAdaptor hc = curv;
        double tol = Min(Tool.Tolerance(e1), global::Kerf.Engine.Foundation.Precision.Confusion());
        tol = Max(curv.Resolution(tol), global::Kerf.Engine.Foundation.Precision.PConfusion());
        Tool.Range(e1, out u1, out u2);
        _extCC.SetCurve(1, hc, u1, u2);
        _extCC.SetTolerance(1, tol);
        // If we enable SetSingleSolutionFlag Extrema will run much quicker on almost parallel curves
        // (e.g. bug 27665), however some solutions will be lost, e.g. see bug 28183.
        // myExtCC.SetSingleSolutionFlag(true);
        _extCC.Perform();
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone => _extCC.IsDone;

    /// <summary>The number of extremum distances.</summary>
    public int ExtCount => _extCC.ExtCount;

    /// <summary>True if the edges are parallel.</summary>
    public bool IsParallel => _extCC.IsParallel;

    /// <summary>The <paramref name="n"/>th extremum square distance.</summary>
    public double SquareDistance(int n) => _extCC.SquareDistance(n);

    /// <summary>The parameter on the first edge of the <paramref name="n"/>th extremum distance.</summary>
    public double ParameterOnE1(int n)
    {
        _extCC.Points(n, out POnCurv pOnE1, out _);
        return pOnE1.Parameter();
    }

    /// <summary>The point on the first edge of the <paramref name="n"/>th extremum distance.</summary>
    public Point3D PointOnE1(int n)
    {
        _extCC.Points(n, out POnCurv pOnE1, out _);
        return pOnE1.Value();
    }

    /// <summary>The parameter on the second edge of the <paramref name="n"/>th extremum distance.</summary>
    public double ParameterOnE2(int n)
    {
        _extCC.Points(n, out _, out POnCurv pOnE2);
        return pOnE2.Parameter();
    }

    /// <summary>The point on the second edge of the <paramref name="n"/>th extremum distance.</summary>
    public Point3D PointOnE2(int n)
    {
        _extCC.Points(n, out _, out POnCurv pOnE2);
        return pOnE2.Value();
    }

    /// <summary>For trimmed curves, the square distances between their end points.</summary>
    public void TrimmedSquareDistances(out double dist11, out double dist12, out double dist21, out double dist22,
                                       out Point3D pnt11, out Point3D pnt12, out Point3D pnt21, out Point3D pnt22)
        => _extCC.TrimmedSquareDistances(out dist11, out dist12, out dist21, out dist22, out pnt11, out pnt12, out pnt21, out pnt22);
}
