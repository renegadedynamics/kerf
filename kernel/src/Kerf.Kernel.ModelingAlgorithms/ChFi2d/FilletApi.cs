// Port of: src/ModelingAlgorithms/TKFillet/ChFi2d/ChFi2d_FilletAPI.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFi2d/ChFi2d_FilletAPI.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Fillets.Planar;

/// <summary>
/// The interface to the two 2D fillet algorithms: <see cref="AnaFilletAlgo"/> (analytical, for lines and arcs
/// of circle with a common point) and <see cref="FilletAlgo"/> (iterative, for any curves, even without a
/// common point), chosen from the arguments (a wire or two edges).
/// </summary>
/// <remarks>A working object; its results are new, frozen edges. The edges it is given are only read.</remarks>
public sealed class FilletApi
{
    // Implementation of the fillet algorithm.
    private readonly FilletAlgo _filletAlgo = new();
    private readonly AnaFilletAlgo _anaFilletAlgo = new();
    private bool _isAnalytical;

    // An empty constructor of the fillet algorithm.
    // Call a method Init() to initialize the algorithm
    // before calling of a Perform() method.
    public FilletApi()
    {
        _isAnalytical = false;
    }

    /// <summary>A fillet algorithm on a wire of two edges in a plane.</summary>
    public FilletApi(in Wire wire, in Plane3D plane)
    {
        _isAnalytical = false;
        Init(wire, plane);
    }

    /// <summary>A fillet algorithm on two edges in a plane.</summary>
    public FilletApi(in Edge edge1, in Edge edge2, in Plane3D plane)
    {
        _isAnalytical = false;
        Init(edge1, edge2, plane);
    }

    /// <summary>Initializes a fillet algorithm: accepts a wire consisting of two edges in a plane.</summary>
    public void Init(in Wire wire, in Plane3D plane)
    {
        // Decide whether we may apply an analytical solution.
        Edge e1 = new(), e2 = new();
        var itr = new Iterator(wire);
        for (; itr.More(); itr.Next())
        {
            if (e1.IsNull)
            {
                e1 = ShapeCasts.Edge(itr.Value());
            }
            else if (e2.IsNull)
            {
                e2 = ShapeCasts.Edge(itr.Value());
            }
            else
            {
                break;
            }
        }
        if (!e1.IsNull && !e2.IsNull)
        {
            _isAnalytical = IsAnalytical(e1, e2);
        }

        // Initialize the algorithm.
        if (_isAnalytical)
        {
            _anaFilletAlgo.Init(wire, plane);
        }
        else
        {
            _filletAlgo.Init(wire, plane);
        }
    }

    /// <summary>Initializes a fillet algorithm: accepts two edges in a plane.</summary>
    public void Init(in Edge edge1, in Edge edge2, in Plane3D plane)
    {
        // Decide whether we may apply an analytical solution.
        _isAnalytical = IsAnalytical(edge1, edge2);

        // Initialize the algorithm.
        if (_isAnalytical)
        {
            _anaFilletAlgo.Init(edge1, edge2, plane);
        }
        else
        {
            _filletAlgo.Init(edge1, edge2, plane);
        }
    }

    /// <summary>Constructs a fillet edge. Returns true if at least one result was found.</summary>
    public bool Perform(double radius)
    {
        return _isAnalytical ? _anaFilletAlgo.Perform(radius) : _filletAlgo.Perform(radius);
    }

    /// <summary>
    /// The number of solutions; <paramref name="point"/> (the common point of the edges) chooses among several
    /// fillets.
    /// </summary>
    public int NbResults(in Point3D point)
    {
        return _isAnalytical ? 1 : _filletAlgo.NbResults(point);
    }

    /// <summary>
    /// The fillet edge and the two edges it trims (all frozen): the solution nearest to <paramref name="point"/> if
    /// <paramref name="iSolution"/> is -1, else that solution.
    /// </summary>
    public Edge Result(in Point3D point, ref Edge edge1, ref Edge edge2, int iSolution = -1)
    {
        return _isAnalytical ? _anaFilletAlgo.Result(ref edge1, ref edge2)
                              : _filletAlgo.Result(point, ref edge1, ref edge2, iSolution);
    }

    // Decides whether the input parameters may use an analytical algorithm
    // for calculation of the fillets, or an iteration-recursive method is needed.
    // The analytical solution is applicable for linear and circular edges having a common point.
    private static bool IsAnalytical(in Edge edge1, in Edge edge2)
    {
        bool ret = false;
        var ac1 = new EdgeCurveAdaptor(edge1);
        var ac2 = new EdgeCurveAdaptor(edge2);
        if ((ac1.GetType() == Line || ac1.GetType() == Circle)
            && (ac2.GetType() == Line || ac2.GetType() == Circle))
        {
            // The edges are lines or arcs of circle.
            // Now check whether they have a common point.
            Point3D p11 = ac1.Value(ac1.FirstParameter());
            Point3D p12 = ac1.Value(ac1.LastParameter());
            Point3D p21 = ac2.Value(ac2.FirstParameter());
            Point3D p22 = ac2.Value(ac2.LastParameter());
            if (p11.SquareDistance(p21) < global::Kerf.Engine.Foundation.Precision.SquareConfusion()
                || p11.SquareDistance(p22) < global::Kerf.Engine.Foundation.Precision.SquareConfusion()
                || p12.SquareDistance(p21) < global::Kerf.Engine.Foundation.Precision.SquareConfusion()
                || p12.SquareDistance(p22) < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
            {
                ret = true;
            }
        }
        return ret;
    }
}
