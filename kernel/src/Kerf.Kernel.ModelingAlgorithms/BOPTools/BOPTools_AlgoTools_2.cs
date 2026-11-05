// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_AlgoTools_2.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Geometry.CurveKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans.Tools;

// BOPTools_AlgoTools_2.cxx: vertices, edges, ranges and dimensions.
internal static partial class AlgoTools
{
    /// <summary>
    /// Raises the tolerance of <paramref name="aVF"/> to cover <paramref name="aNewVertex"/> (distance plus its
    /// tolerance). Writes into <paramref name="aVF"/>.
    /// </summary>
    public static void UpdateVertex(in Vertex aVF, in Vertex aNewVertex)
    {
        double tolVF, tolNewVertex, dist, newTol;
        //
        Point3D pvf = Tool.Pnt(aVF);
        Point3D pNewVertex = Tool.Pnt(aNewVertex);
        tolVF = Tool.Tolerance(aVF);
        tolNewVertex = Tool.Tolerance(aNewVertex);

        dist = pvf.DistanceTo(pNewVertex);
        newTol = dist + tolNewVertex;

        if (newTol > tolVF)
        {
            BrepBuilder bb = new();
            bb.UpdateVertex(aVF, newTol + DTolerance());
        }
    }

    /// <summary>
    /// Raises the vertex's tolerance to cover the point of the edge's 3D curve at the parameter. Writes into
    /// <paramref name="aV"/>.
    /// </summary>
    public static void UpdateVertex(in Edge aE, double aT, in Vertex aV)
    {
        double tolV, dist;
        Point3D pc;

        Point3D pv = Tool.Pnt(aV);
        tolV = Tool.Tolerance(aV);

        CurveAdaptor ca = new(Tool.Curve(aE, out _, out _)!);
        ca.D0(aT, out pc);
        dist = pv.DistanceTo(pc);
        if (dist > tolV)
        {
            BrepBuilder bb = new();
            bb.UpdateVertex(aV, dist + DTolerance());
        }
    }

    /// <summary>
    /// Raises the vertex's tolerance to cover the point of the intersection curve at the parameter. Writes into
    /// <paramref name="aV"/>.
    /// </summary>
    public static void UpdateVertex(in IntersectionCurve aC, double aT, in Vertex aV)
    {
        double tolV, dist;
        Point3D pc;

        Point3D pv = Tool.Pnt(aV);
        tolV = Tool.Tolerance(aV);

        CurveAdaptor ca = new(aC.Curve()!);
        ca.D0(aT, out pc);
        dist = pv.DistanceTo(pc);
        if (dist > tolV)
        {
            BrepBuilder bb = new();
            bb.UpdateVertex(aV, dist + DTolerance());
        }
    }

    /// <summary>
    /// Makes an edge on the intersection curve between the vertices at the parameters, keeping the range as given.
    /// </summary>
    public static void MakeSectEdge(in IntersectionCurve aIC, in Vertex aV1, double aP1, in Vertex aV2, double aP2,
                                    out Edge aNewEdge)
    {
        Curve3D c = aIC.Curve()!;

        MakeEdge makeEdge = new(c, aV1, aV2, aP1, aP2);

        // The maker's unfrozen result: the edge is new, and the range is written into it.
        Edge e = ShapeCasts.Edge(makeEdge.InternalShape());
        //
        // Range must be as it was !
        BrepBuilder bb = new();
        bb.Range(e, aP1, aP2);
        //
        aNewEdge = e;
    }

    /// <summary>A new edge with the edge's geometry and sub-shapes (an empty copy refilled), and its orientation.</summary>
    public static Edge CopyEdge(in Edge edge)
    {
        Edge newEdge = ShapeCasts.Edge(edge.Oriented(Forward));
        newEdge.EmptyCopy();
        for (Iterator it = new(edge, false); it.More(); it.Next())
        {
            new BrepBuilder().Add(newEdge, it.Value());
        }
        newEdge.Orient(edge.Orientation);
        return newEdge;
    }

    /// <summary>
    /// A split of the edge between the vertices at the parameters (in increasing order): an empty copy of the edge
    /// with the two vertices and the range, with the edge's orientation.
    /// </summary>
    public static void MakeSplitEdge(in Edge aE, in Vertex aV1, double aP1, in Vertex aV2, double aP2,
                                     out Edge aNewEdge)
    {
        Edge e = ShapeCasts.Edge(aE.Oriented(Forward));
        e.EmptyCopy();
        //
        BrepBuilder bb = new();
        if (!aV1.IsNull)
        {
            if (aP1 < aP2)
            {
                bb.Add(e, ShapeCasts.Vertex(aV1.Oriented(Forward)));
            }
            else
            {
                bb.Add(e, ShapeCasts.Vertex(aV1.Oriented(Reversed)));
            }
        }
        if (!aV2.IsNull)
        {
            if (aP1 < aP2)
            {
                bb.Add(e, ShapeCasts.Vertex(aV2.Oriented(Reversed)));
            }
            else
            {
                bb.Add(e, ShapeCasts.Vertex(aV2.Oriented(Forward)));
            }
        }

        if (aP1 < aP2)
        {
            bb.Range(e, aP1, aP2);
        }
        else
        {
            bb.Range(e, aP2, aP1);
        }

        aNewEdge = e;
        aNewEdge.Orient(aE.Orientation);
    }

    /// <summary>
    /// A new vertex at the middle of the two vertices, with the larger tolerance plus half their distance.
    /// </summary>
    public static void MakeNewVertex(in Vertex aV1, in Vertex aV2, out Vertex aNewVertex)
    {
        Point3D pnt1 = Tool.Pnt(aV1);
        double tol1 = Tool.Tolerance(aV1);

        Point3D pnt2 = Tool.Pnt(aV2);
        double tol2 = Tool.Tolerance(aV2);

        double maxTol, dist;

        dist = pnt1.DistanceTo(pnt2);
        maxTol = (tol1 > tol2) ? tol1 : tol2;
        maxTol = maxTol + 0.5 * dist;

        Xyz xyz1 = pnt1.XYZ();
        Xyz xyz2 = pnt2.XYZ();
        Xyz newXYZ = 0.5 * (xyz1 + xyz2);

        Point3D newPnt = new(newXYZ);
        BrepBuilder bb = new();
        aNewVertex = new Vertex();
        bb.MakeVertex(ref aNewVertex, newPnt, maxTol);
    }

    /// <summary>A new vertex at the point with the tolerance.</summary>
    public static void MakeNewVertex(in Point3D aP, double aTol, out Vertex aNewVertex)
    {
        BrepBuilder bb = new();
        aNewVertex = new Vertex();
        bb.MakeVertex(ref aNewVertex, aP, aTol);
    }

    /// <summary>
    /// A new vertex at the middle of the two edges' points at the parameters, with the larger edge tolerance plus half
    /// their distance.
    /// </summary>
    public static void MakeNewVertex(in Edge aE1, double aParm1, in Edge aE2, double aParm2, out Vertex aNewVertex)
    {
        double tol1, tol2, maxTol, dist;

        PointOnEdge(aE1, aParm1, out Point3D pnt1);
        PointOnEdge(aE2, aParm2, out Point3D pnt2);

        tol1 = Tool.Tolerance(aE1);
        tol2 = Tool.Tolerance(aE2);

        dist = pnt1.DistanceTo(pnt2);
        maxTol = (tol1 > tol2) ? tol1 : tol2;
        maxTol = maxTol + 0.5 * dist;

        Xyz xyz1 = pnt1.XYZ();
        Xyz xyz2 = pnt2.XYZ();
        Xyz newXYZ = 0.5 * (xyz1 + xyz2);

        Point3D newPnt = new(newXYZ);
        BrepBuilder bb = new();
        aNewVertex = new Vertex();
        bb.MakeVertex(ref aNewVertex, newPnt, maxTol);
    }

    /// <summary>
    /// A new vertex at the edge's point at the parameter, with the sum of the edge's and the face's tolerances.
    /// </summary>
    public static void MakeNewVertex(in Edge aE1, double aParm1, in Face aF1, out Vertex aNewVertex)
    {
        double tol1, tol2, maxTol;

        PointOnEdge(aE1, aParm1, out Point3D pnt);

        tol1 = Tool.Tolerance(aE1);
        tol2 = Tool.Tolerance(aF1);
        //
        maxTol = tol1 + tol2 + DTolerance();
        //
        BrepBuilder bb = new();
        aNewVertex = new Vertex();
        bb.MakeVertex(ref aNewVertex, pnt, maxTol);
    }

    /// <summary>The point of the edge's 3D curve at the parameter.</summary>
    public static void PointOnEdge(in Edge aE, double aParm, out Point3D aPnt)
    {
        Curve3D c1 = Tool.Curve(aE, out _, out _)!;
        c1.D0(aParm, out aPnt);
    }

    /// <summary>
    /// The range shrunk at both ends by the resolution of twice the two edges' tolerances on the first edge (not for
    /// lines); the range itself if it would become shorter than PConfusion.
    /// </summary>
    public static void CorrectRange(in Edge aE1, in Edge aE2, in ParameterRange aSR, out ParameterRange aNewSR)
    {
        int i;
        double res, tolE1, tolE2, tf, tl, dT;
        EdgeCurveAdaptor bc = new();
        CurveKind ct;
        //
        aNewSR = aSR;
        //
        bc.Initialize(aE1);
        ct = bc.GetType();
        if (ct == Line)
        {
            return;
        }
        //
        dT = global::Kerf.Engine.Foundation.Precision.PConfusion();
        tf = aSR.First();
        tl = aSR.Last();
        //
        tolE1 = Tool.Tolerance(aE1);
        tolE2 = Tool.Tolerance(aE2);
        //
        for (i = 0; i < 2; ++i)
        {
            res = 2.0 * (tolE1 + tolE2);
            //
            if (ct == BezierCurve || ct == BSplineCurve || ct == OffsetCurve
                || ct == OtherCurve)
            {
                Point3D p;
                Vector3D der;
                if (i == 0)
                {
                    bc.D1(tf, out p, out der);
                }
                else
                {
                    bc.D1(tl, out p, out der);
                }
                //
                double mgn = der.Magnitude;

                if (mgn > 1.0e-12)
                {
                    res = res / mgn;
                }
                else
                {
                    res = bc.Resolution(res);
                }
            } // if (aCT==GeomAbs_BezierCurve||...
            else
            {
                res = bc.Resolution(res);
            }
            //
            if (i == 0)
            {
                aNewSR.SetFirst(tf + res);
            }
            else
            {
                aNewSR.SetLast(tl - res);
            }
            //
            if ((aNewSR.Last() - aNewSR.First()) < dT)
            {
                aNewSR = aSR;
            }
            // aNewSR=((aNewSR.Last()-aNewSR.First()) < dT) ? aSR : aNewSR;
        }
    }

    /// <summary>
    /// The range shrunk at both ends by the resolution of the face's tolerance on the edge; the range itself if it
    /// would become shorter than PConfusion.
    /// </summary>
    public static void CorrectRange(in Edge aE, in Face aF, in ParameterRange aSR, out ParameterRange aNewSR)
    {
        int i;
        double res, tolF, tf, tl, dT;
        EdgeCurveAdaptor bc = new();
        CurveKind ct;
        //
        aNewSR = aSR;
        //
        dT = global::Kerf.Engine.Foundation.Precision.PConfusion();
        tf = aSR.First();
        tl = aSR.Last();
        //
        bc.Initialize(aE);
        ct = bc.GetType();
        //
        tolF = Tool.Tolerance(aF);
        //
        for (i = 0; i < 2; ++i)
        {
            res = tolF;

            if (ct == BezierCurve || ct == BSplineCurve || ct == OffsetCurve
                || ct == OtherCurve)
            {
                Point3D p;
                Vector3D der;
                if (i == 0)
                {
                    bc.D1(tf, out p, out der);
                }
                else
                {
                    bc.D1(tl, out p, out der);
                }
                //
                double mgn = der.Magnitude;

                if (mgn > 1.0e-12)
                {
                    res = res / mgn;
                }
                else
                {
                    res = bc.Resolution(res);
                }
            } // if (aCT==GeomAbs_BezierCurve||...
            else
            {
                res = bc.Resolution(res);
            }
            //
            if (i == 0)
            {
                aNewSR.SetFirst(tf + res);
            }
            else
            {
                aNewSR.SetLast(tl - res);
            }
            //
            if ((aNewSR.Last() - aNewSR.First()) < dT)
            {
                aNewSR = aSR;
            }
        }
    }

    // The dimension of an elementary shape (-1 for compounds).
    private static int DimensionCore(in Shape s)
    {
        switch (s.Type)
        {
            case ShapeType.Vertex:
                return 0;
            case ShapeType.Edge:
            case ShapeType.Wire:
                return 1;
            case ShapeType.Face:
            case ShapeType.Shell:
                return 2;
            case ShapeType.Solid:
            case ShapeType.CompSolid:
                return 3;
            default:
                return -1;
        }
    }

    /// <summary>
    /// The minimal and maximal dimensions of the shape's elementary parts (compounds explored); -1 and -1 for an empty
    /// shape.
    /// </summary>
    public static void Dimensions(in Shape s, out int dMin, out int dMax)
    {
        dMin = dMax = DimensionCore(s);
        if (dMax >= 0)
        {
            return;
        }

        NCollection_List<Shape> ls = new();
        HashSet<Shape> mFence = new(ShapeHasher.Instance);
        TreatCompound(s, ls, mFence);
        if (ls.IsEmpty())
        {
            // empty shape
            dMin = dMax = -1;
            return;
        }

        dMin = 3;
        dMax = 0;
        for (NCollection_List<Shape>.Iterator it = new(ls); it.More(); it.Next())
        {
            int dim = DimensionCore(it.Value());
            if (dim < dMin)
            {
                dMin = dim;
            }
            if (dim > dMax)
            {
                dMax = dim;
            }
        }
    }

    /// <summary>The dimension of the shape, or -1 for a shape mixing dimensions (or empty).</summary>
    public static int Dimension(in Shape s)
    {
        Dimensions(s, out int dMin, out int dMax);
        return (dMin == dMax) ? dMin : -1;
    }

    /// <summary>
    /// Appends the non-compound parts of the shape (exploring compounds) to the list, each once if a fence map is
    /// given.
    /// </summary>
    public static void TreatCompound(in Shape s, NCollection_List<Shape> ls, HashSet<Shape>? mFence = null)
    {
        ShapeType type = s.Type;
        if (type != ShapeType.Compound)
        {
            if (mFence is null || mFence.Add(s))
            {
                ls.Append(s);
            }
            return;
        }

        for (Iterator it = new(s); it.More(); it.Next())
        {
            TreatCompound(it.Value(), ls, mFence);
        }
    }
}
