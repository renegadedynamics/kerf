// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_ExtCC.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_ExtCC.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

namespace Occt;

/// <summary>The extrema of the distance between two edges.</summary>
/// <remarks>
/// A working object (decision 21): initialize with the second edge, then perform with any number of first edges.
/// OCCT copies a local adaptor into a new handle; the port keeps the adaptor it made (decision 394).
/// </remarks>
public sealed class BRepExtrema_ExtCC
{
    private readonly ExtCC myExtCC = new();
    private EdgeCurveAdaptor? myHC;

    public BRepExtrema_ExtCC()
    {
    }

    /// <summary>Computes all the extrema between the edges.</summary>
    public BRepExtrema_ExtCC(in Edge E1, in Edge E2)
    {
        Initialize(E2);
        Perform(E1);
    }

    public void Initialize(in Edge E2)
    {
        if (!Tool.IsGeometric(E2))
        {
            return; // protect against non-geometric type (e.g. polygon)
        }
        double V1, V2;
        var Curv = new EdgeCurveAdaptor(E2);
        myHC = Curv;
        double Tol = Min(Tool.Tolerance(E2), global::Kerf.Engine.Foundation.Precision.Confusion());
        Tol = Max(Curv.Resolution(Tol), global::Kerf.Engine.Foundation.Precision.PConfusion());
        Tool.Range(E2, out V1, out V2);
        myExtCC.SetCurve(2, myHC, V1, V2);
        myExtCC.SetTolerance(2, Tol);
    }

    public void Perform(in Edge E1)
    {
        if (!Tool.IsGeometric(E1))
        {
            return; // protect against non-geometric type (e.g. polygon)
        }
        double U1, U2;
        var Curv = new EdgeCurveAdaptor(E1);
        EdgeCurveAdaptor HC = Curv;
        double Tol = Min(Tool.Tolerance(E1), global::Kerf.Engine.Foundation.Precision.Confusion());
        Tol = Max(Curv.Resolution(Tol), global::Kerf.Engine.Foundation.Precision.PConfusion());
        Tool.Range(E1, out U1, out U2);
        myExtCC.SetCurve(1, HC, U1, U2);
        myExtCC.SetTolerance(1, Tol);
        // If we enable SetSingleSolutionFlag Extrema will run much quicker on almost parallel curves
        // (e.g. bug 27665), however some solutions will be lost, e.g. see bug 28183.
        // myExtCC.SetSingleSolutionFlag(true);
        myExtCC.Perform();
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone() => myExtCC.IsDone;

    /// <summary>The number of extremum distances.</summary>
    public int NbExt() => myExtCC.ExtCount;

    /// <summary>True if the edges are parallel.</summary>
    public bool IsParallel() => myExtCC.IsParallel;

    /// <summary>The <paramref name="N"/>th extremum square distance.</summary>
    public double SquareDistance(int N) => myExtCC.SquareDistance(N);

    /// <summary>The parameter on the first edge of the <paramref name="N"/>th extremum distance.</summary>
    public double ParameterOnE1(int N)
    {
        myExtCC.Points(N, out POnCurv POnE1, out _);
        return POnE1.Parameter();
    }

    /// <summary>The point on the first edge of the <paramref name="N"/>th extremum distance.</summary>
    public Point3D PointOnE1(int N)
    {
        myExtCC.Points(N, out POnCurv POnE1, out _);
        return POnE1.Value();
    }

    /// <summary>The parameter on the second edge of the <paramref name="N"/>th extremum distance.</summary>
    public double ParameterOnE2(int N)
    {
        myExtCC.Points(N, out _, out POnCurv POnE2);
        return POnE2.Parameter();
    }

    /// <summary>The point on the second edge of the <paramref name="N"/>th extremum distance.</summary>
    public Point3D PointOnE2(int N)
    {
        myExtCC.Points(N, out _, out POnCurv POnE2);
        return POnE2.Value();
    }

    /// <summary>For trimmed curves, the square distances between their end points.</summary>
    public void TrimmedSquareDistances(out double dist11, out double dist12, out double dist21, out double dist22,
                                       out Point3D pnt11, out Point3D pnt12, out Point3D pnt21, out Point3D pnt22)
        => myExtCC.TrimmedSquareDistances(out dist11, out dist12, out dist21, out dist22, out pnt11, out pnt12, out pnt21, out pnt22);
}
