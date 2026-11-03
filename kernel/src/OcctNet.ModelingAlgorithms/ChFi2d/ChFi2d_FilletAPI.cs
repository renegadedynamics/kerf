// Port of: src/ModelingAlgorithms/TKFillet/ChFi2d/ChFi2d_FilletAPI.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFi2d/ChFi2d_FilletAPI.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>
/// The interface to the two 2D fillet algorithms: <see cref="ChFi2d_AnaFilletAlgo"/> (analytical, for lines and arcs
/// of circle with a common point) and <see cref="ChFi2d_FilletAlgo"/> (iterative, for any curves, even without a
/// common point), chosen from the arguments (a wire or two edges).
/// </summary>
/// <remarks>A working object; its results are new, frozen edges. The edges it is given are only read.</remarks>
public sealed class ChFi2d_FilletAPI
{
    // Implementation of the fillet algorithm.
    private readonly ChFi2d_FilletAlgo myFilletAlgo = new();
    private readonly ChFi2d_AnaFilletAlgo myAnaFilletAlgo = new();
    private bool myIsAnalytical;

    // An empty constructor of the fillet algorithm.
    // Call a method Init() to initialize the algorithm
    // before calling of a Perform() method.
    public ChFi2d_FilletAPI()
    {
        myIsAnalytical = false;
    }

    /// <summary>A fillet algorithm on a wire of two edges in a plane.</summary>
    public ChFi2d_FilletAPI(in TopoDS_Wire theWire, in Plane3D thePlane)
    {
        myIsAnalytical = false;
        Init(theWire, thePlane);
    }

    /// <summary>A fillet algorithm on two edges in a plane.</summary>
    public ChFi2d_FilletAPI(in TopoDS_Edge theEdge1, in TopoDS_Edge theEdge2, in Plane3D thePlane)
    {
        myIsAnalytical = false;
        Init(theEdge1, theEdge2, thePlane);
    }

    /// <summary>Initializes a fillet algorithm: accepts a wire consisting of two edges in a plane.</summary>
    public void Init(in TopoDS_Wire theWire, in Plane3D thePlane)
    {
        // Decide whether we may apply an analytical solution.
        TopoDS_Edge E1 = new(), E2 = new();
        var itr = new TopoDS_Iterator(theWire);
        for (; itr.More(); itr.Next())
        {
            if (E1.IsNull())
            {
                E1 = TopoDS.Edge(itr.Value());
            }
            else if (E2.IsNull())
            {
                E2 = TopoDS.Edge(itr.Value());
            }
            else
            {
                break;
            }
        }
        if (!E1.IsNull() && !E2.IsNull())
        {
            myIsAnalytical = IsAnalytical(E1, E2);
        }

        // Initialize the algorithm.
        if (myIsAnalytical)
        {
            myAnaFilletAlgo.Init(theWire, thePlane);
        }
        else
        {
            myFilletAlgo.Init(theWire, thePlane);
        }
    }

    /// <summary>Initializes a fillet algorithm: accepts two edges in a plane.</summary>
    public void Init(in TopoDS_Edge theEdge1, in TopoDS_Edge theEdge2, in Plane3D thePlane)
    {
        // Decide whether we may apply an analytical solution.
        myIsAnalytical = IsAnalytical(theEdge1, theEdge2);

        // Initialize the algorithm.
        if (myIsAnalytical)
        {
            myAnaFilletAlgo.Init(theEdge1, theEdge2, thePlane);
        }
        else
        {
            myFilletAlgo.Init(theEdge1, theEdge2, thePlane);
        }
    }

    /// <summary>Constructs a fillet edge. Returns true if at least one result was found.</summary>
    public bool Perform(double theRadius)
    {
        return myIsAnalytical ? myAnaFilletAlgo.Perform(theRadius) : myFilletAlgo.Perform(theRadius);
    }

    /// <summary>
    /// The number of solutions; <paramref name="thePoint"/> (the common point of the edges) chooses among several
    /// fillets.
    /// </summary>
    public int NbResults(in Point3D thePoint)
    {
        return myIsAnalytical ? 1 : myFilletAlgo.NbResults(thePoint);
    }

    /// <summary>
    /// The fillet edge and the two edges it trims (all frozen): the solution nearest to <paramref name="thePoint"/> if
    /// <paramref name="iSolution"/> is -1, else that solution.
    /// </summary>
    public TopoDS_Edge Result(in Point3D thePoint, ref TopoDS_Edge theEdge1, ref TopoDS_Edge theEdge2, int iSolution = -1)
    {
        return myIsAnalytical ? myAnaFilletAlgo.Result(ref theEdge1, ref theEdge2)
                              : myFilletAlgo.Result(thePoint, ref theEdge1, ref theEdge2, iSolution);
    }

    // Decides whether the input parameters may use an analytical algorithm
    // for calculation of the fillets, or an iteration-recursive method is needed.
    // The analytical solution is applicable for linear and circular edges having a common point.
    private static bool IsAnalytical(in TopoDS_Edge theEdge1, in TopoDS_Edge theEdge2)
    {
        bool ret = false;
        var AC1 = new BRepAdaptor_Curve(theEdge1);
        var AC2 = new BRepAdaptor_Curve(theEdge2);
        if ((AC1.GetType() == Line || AC1.GetType() == Circle)
            && (AC2.GetType() == Line || AC2.GetType() == Circle))
        {
            // The edges are lines or arcs of circle.
            // Now check whether they have a common point.
            Point3D p11 = AC1.Value(AC1.FirstParameter());
            Point3D p12 = AC1.Value(AC1.LastParameter());
            Point3D p21 = AC2.Value(AC2.FirstParameter());
            Point3D p22 = AC2.Value(AC2.LastParameter());
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
