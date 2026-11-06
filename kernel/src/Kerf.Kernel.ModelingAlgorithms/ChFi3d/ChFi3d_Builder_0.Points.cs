// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_0.cxx (ChFi3d_ComputeArete to findIndexPoint)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Fillets.Blend;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.Walking;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Fillets;

internal static partial class Builder_0
{
    //=======================================================================
    // function : ComputeArete
    // purpose  :
    // to fill with s.d. a fillet with pcurves constructed as follows
    // firstpoint on S1 -------------edge:curve3d/pcurves--->lastpoint on S1
    //  |                                                              |
    //  |                                                              |
    //  |                                                              |
    // edge:curve 3d/pcurves           fillet                         edge
    //  |         attention it is necessary to test orientation of the fillet before|
    //  |         determining the transitions pcurves/fillet           |
    //  |                                                              |
    //  \/                                                             \/
    // firstpoint sur S2 -------------edge:courbe3d/pcurves--->lastpoint sur S2
    //
    //=======================================================================

    /// <summary>
    /// The edge of a fillet between two of its corners: its 3D curve <paramref name="c3d"/> and pcurve
    /// <paramref name="pcurv"/> on <paramref name="surf"/> (<paramref name="iFlag"/> 0), or only the pcurve of the
    /// given curve (<paramref name="iFlag"/> &gt; 0, its parameters imposed for 2).
    /// </summary>
    internal static void ChFi3d_ComputeArete(in CommonPoint p1, in Point2D uv1, in CommonPoint p2,
                                             in Point2D uv2, Surface surf, ref Curve3D? c3d,
                                             ref Curve2D? pcurv, ref double pardeb, ref double parfin,
                                             double tol3d, double tol2d, out double tolreached, int iFlag)
    {
        // IFlag=0 pcurve et courbe 3d
        // IFlag>0 pcurve (parametrage impose si IFlag=2)
        /*szv:static*/
        SurfaceAdaptor hs = new();
        /*szv:static*/
        CurveAdaptor hc = new();

        tolreached = tol3d;

        if (Math.Abs(uv1.X - uv2.X) <= tol2d)
        {
            if (iFlag == 0)
            {
                pardeb = uv1.Y;
                parfin = uv2.Y;
                c3d = surf.UIso(uv1.X);
                if (pardeb > parfin)
                {
                    pardeb = c3d.ReversedParameter(pardeb);
                    parfin = c3d.ReversedParameter(parfin);
                    c3d.Reverse();
                }
                if (c3d is TrimmedCurve3D tc)
                {
                    c3d = tc.BasisCurve;
                    if (c3d.IsPeriodic)
                    {
                        global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(c3d.FirstParameter, c3d.LastParameter, tol2d, ref pardeb,
                                              ref parfin);
                    }
                }
            }
            if (iFlag != 1)
            {
                hs.Load(surf);
                hc.Load(c3d!, pardeb, parfin);
                CurveAdaptorBase hCurve = hc; // to avoid ambiguity
                ChFi3d_ComputePCurv(hCurve, uv1, uv2, out Curve2D aPcurv, hs, pardeb, parfin, tol3d,
                                    out tolreached, false);
                pcurv = aPcurv;
            }
            else
            {
                pcurv = new LineCurve2D(uv1, new Direction2D(new Vector2D(uv1, uv2)));
            }
        }
        else if (Math.Abs(uv1.Y - uv2.Y) <= tol2d)
        {
            // iso v
            if (iFlag == 0)
            {
                pardeb = uv1.X;
                parfin = uv2.X;
                c3d = surf.VIso(uv1.Y);
                if (pardeb > parfin)
                {
                    pardeb = c3d.ReversedParameter(pardeb);
                    parfin = c3d.ReversedParameter(parfin);
                    c3d.Reverse();
                }
                if (c3d is TrimmedCurve3D tc)
                {
                    c3d = tc.BasisCurve;
                    if (c3d.IsPeriodic)
                    {
                        global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(c3d.FirstParameter, c3d.LastParameter, tol2d, ref pardeb,
                                              ref parfin);
                    }
                }
            }
            if (iFlag != 1)
            {
                hs.Load(surf);
                hc.Load(c3d!, pardeb, parfin);
                CurveAdaptorBase hCurve = hc; // to avoid ambiguity
                ChFi3d_ComputePCurv(hCurve, uv1, uv2, out Curve2D aPcurv, hs, pardeb, parfin, tol3d,
                                    out tolreached, false);
                pcurv = aPcurv;
            }
            else
            {
                pcurv = new LineCurve2D(uv1, new Direction2D(new Vector2D(uv1, uv2)));
            }
        }
        else if (iFlag == 0)
        {
            if (p1.IsVertex || p2.IsVertex || !p1.IsOnArc || !p2.IsOnArc)
            {
                // A straight line is constructed to avoid
                // arc and tangent.
                NCollection_Array1<Point2D> qoles = new(1, 2);
                qoles[1] = uv1;
                qoles[2] = uv2;
                pcurv = new BezierCurve2D(qoles);
            }
            else
            {
                EdgeCurveAdaptor c1 = new(p1.Arc());
                c1.D1(p1.ParameterOnArc(), out Point3D pp, out Vector3D vv1);
                c1.Initialize(p2.Arc());
                c1.D1(p2.ParameterOnArc(), out pp, out Vector3D vv2);
                hs.Load(surf);
                pcurv = ChFi3d_BuildPCurve(hs, uv1, vv1, uv2, vv2, true);
                // There are some cases when PCurve constructed in this way
                // leaves the surface, in particular if it results from an
                // extension. A posteriori checking is required and if
                // the curve leaves the surface it is replaced by straight line UV1 UV2
                // non regarding the tangency with neighboring arcs!
                Box2d bs = new();
                double umin, umax, vmin, vmax;
                surf.ParameterBounds(out umin, out umax, out vmin, out vmax);
                bs.Update(umin, vmin, umax, vmax);
                bs.SetGap(global::Kerf.Engine.Foundation.Precision.PConfusion());
                bool aIN = true;
                for (int ii = 1; ii <= 4 && aIN; ii++)
                {
                    if (bs.IsOut(((BezierCurve2D)pcurv).Pole(ii)))
                    {
                        aIN = false;
                        NCollection_Array1<Point2D> qoles = new(1, 2);
                        qoles[1] = uv1;
                        qoles[2] = uv2;
                        pcurv = new BezierCurve2D(qoles);
                    }
                }
            }
            CurveAdaptor2D ac = new(pcurv);
            CurveAdaptor2D ahc = (CurveAdaptor2D)ac.ShallowCopy();
            SurfaceAdaptor AS = new(surf);
            SurfaceAdaptor ahs = (SurfaceAdaptor)AS.ShallowCopy();
            CurveOnSurface cs = new(ahc, ahs);
            pardeb = cs.FirstParameter();
            parfin = cs.LastParameter();
            double avtol;
            global::Kerf.Engine.Geometry.Library.GeomLib.BuildCurve3d(tol3d, cs, pardeb, parfin, ref c3d, out tolreached, out avtol);
        }
        else
        {
            hs.Load(surf);
            hc.Load(c3d!, pardeb, parfin);
            ChFi3d_ProjectPCurv(hc, hs, ref pcurv, tol3d, ref tolreached);
            Point2D p2d = pcurv!.Value(pardeb);
            if (!uv1.IsEqual(p2d, global::Kerf.Engine.Foundation.Precision.PConfusion()))
            {
                Vector2D v2d = new(p2d, uv1);
                pcurv = Writable(pcurv);
                pcurv.Translate(v2d);
            }
        }
    }

    //=================================================================================================

    /// <summary>The interference of the curve <paramref name="icurv"/> on the surface <paramref name="isurf"/> by its
    /// pcurve, with the transition <paramref name="et"/>.</summary>
    internal static DsSurfaceCurveInterference ChFi3d_FilCurveInDS(int icurv, int isurf,
                                                                              Curve2D? pcurv,
                                                                              Orientation et)
    {
        DsSurfaceCurveInterference sc1;
        sc1 = new DsSurfaceCurveInterference(new DsTransition(et), DsKind.Surface, isurf,
                                                        DsKind.Curve, icurv, pcurv);
        return sc1;
    }

    //=================================================================================================

    /// <summary>The orientation of a transition of the walking: In is FORWARD, Out REVERSED, else INTERNAL.</summary>
    internal static Orientation ChFi3d_TrsfTrans(WalkingTypeTrans t1)
    {
        switch (t1)
        {
            case WalkingTypeTrans.In:
                return Forward;
            case WalkingTypeTrans.Out:
                return Reversed;
            default:
                break;
        }
        return Internal;
    }

    //=======================================================================
    // function : FilCommonPoint
    // purpose  : Loading of the common point
    //           management of the case when it happens on already existing vertex.
    //=======================================================================

    /// <summary>
    /// Loads the common point <paramref name="cp"/> from the walking's extremity <paramref name="sp"/>: its point,
    /// tangent and parameter on the guide, the vertex it lies on, or the arc with its transition, and the tolerance.
    /// </summary>
    /// <remarks>Reads the extremity's vertices and edges (the builder's working shape) and writes only
    /// <paramref name="cp"/>.</remarks>
    internal static void ChFi3d_FilCommonPoint(in Extremity sp, WalkingTypeTrans transLine, bool start,
                                               ref CommonPoint cp, double tol)
    {
        //  BRep_Tool Outil;
        double dist, maxtol = Math.Max(tol, cp.Tolerance());

        cp.SetPoint(sp.Value()); // One starts with the point and the vector
        if (sp.HasTangent)
        {
            if (start)
            {
                cp.SetVector(sp.Tangent().Reversed()); // The tangent is oriented to the exit
            }
            else
            {
                cp.SetVector(sp.Tangent());
            }
        }

        cp.SetParameter(sp.ParameterOnGuide()); // and the parameter of the spine

        if (sp.IsVertex)
        { // the Vertex is loaded if required
            // (inside of a face)
            Vertex v = ((TopologyHVertex)sp.Vertex()).Vertex();

            cp.SetVertex(v);
            dist = (sp.Value()).DistanceTo(Tool.Pnt(v));
            //// modified by jgv, 18.09.02 for OCC571 ////
            // maxtol += Dist;
            maxtol = Math.Max(dist, maxtol);
            //////////////////////////////////////////////
            cp.SetPoint(Tool.Pnt(v));

            // the sequence of arcs the information is known by thee vertex (ancestor)
            // in this case the transitions are not computed, it is done by this program
        }

        if (sp.PointOnRstCount != 0)
        { //  An arc, and/or a vertex is loaded

            PointOnRst pr = sp.PointOnRst(1);
            if (pr.Arc() is EdgeCurve2DAdaptor harc)
            {
                double distF, distL, leParamAmoi;
                int Index_min;
                Edge e = harc.Edge();

                Vertex[] v = new Vertex[2];
                Exploration.Vertices(e, out v[0], out v[1]);

                distF = (sp.Value()).DistanceTo(Tool.Pnt(v[0]));
                distL = (sp.Value()).DistanceTo(Tool.Pnt(v[1]));
                if (distF < distL)
                {
                    Index_min = 0;
                    dist = distF;
                }
                else
                {
                    Index_min = 1;
                    dist = distL;
                }

                if (dist <= maxtol + Tool.Tolerance(v[Index_min]))
                {
                    // a preexisting vertex has been met
                    cp.SetVertex(v[Index_min]); // the old vertex is loaded
                    cp.SetPoint(Tool.Pnt(v[Index_min]));
                    maxtol = Math.Max(Tool.Tolerance(v[Index_min]), maxtol);
                    //// modified by jgv, 18.09.02 for OCC571 ////
                    // maxtol += Dist;
                    maxtol = Math.Max(dist, maxtol);
                    //////////////////////////////////////////////
                    leParamAmoi = Tool.Parameter(v[Index_min], e);
                }
                else
                { // Creation of an arc only
                    maxtol = Math.Max(Tool.Tolerance(e), maxtol);
                    maxtol = Math.Max(sp.Tolerance(), maxtol);
                    leParamAmoi = pr.ParameterOnArc();
                }

                // Definition of the arc
                Orientation tr;
                Orientation or = e.Orientation;
                if (start)
                {
                    tr = global::Kerf.Engine.Topology.TopAbs.Reverse(global::Kerf.Engine.Topology.TopAbs.Compose(ChFi3d_TrsfTrans(transLine), or));
                }
                else
                {
                    tr = global::Kerf.Engine.Topology.TopAbs.Compose(ChFi3d_TrsfTrans(transLine), or);
                }
                cp.SetArc(maxtol, e, leParamAmoi, tr);
            }
        }
        cp.SetTolerance(maxtol); // Finally, the tolerance.
    }

    //=================================================================================================

    /// <summary>Adds to the data structure the solid (or shell) of the spine's first edge; returns its index.</summary>
    /// <exception cref="FailureException">The spine has no edge.</exception>
    internal static int ChFi3d_SolidIndex(Spine? sp, DsDataStructure dStr, Map mapESo,
                                          Map mapESh)
    {
        if (sp is null || sp.EdgeCount == 0)
        {
            throw new FailureException("SolidIndex : Spine incomplete");
        }
        Shape edref = sp.Edges(1);
        Shape shellousolid;
        if (!mapESo[edref].IsEmpty())
        {
            shellousolid = mapESo[edref].First();
        }
        else
        {
            shellousolid = mapESh[edref].First();
        }
        int solidindex = dStr.AddShape(shellousolid);
        return solidindex;
    }

    //=================================================================================================

    /// <summary>
    /// Adds the common point to the data structure: its vertex, after setting the vertex's point and raising its
    /// tolerance to the common point's, or else a new point. Returns its index.
    /// </summary>
    /// <remarks>
    /// <b>This writes into the vertex</b> (<c>BRep_Builder::UpdateVertex</c>, OCCT's line 2333 and plan 2.2): the
    /// common points hold the vertices of the shape the builder works on, written in the builder's working overlay
    /// (decision 1087, unit 10H5). Outside one, a locked vertex throws <see cref="ImmutableObjectException"/>.
    /// </remarks>
    internal static int ChFi3d_IndexPointInDS(in CommonPoint p1, DsDataStructure dStr)
    {
        if (p1.IsVertex)
        {
            // --------------------------------->  !*!*!*
            // Attention : it is necessary ti implement a mechanism
            // controlling tolerance.
            BrepBuilder b = new();
            b.UpdateVertex(p1.Vertex(), p1.Point(), p1.Tolerance());
            return dStr.AddShape(p1.Vertex());
        }
        return dStr.AddPoint(new DsPoint(p1.Point(), p1.Tolerance()));
    }

    //=================================================================================================

    /// <summary>The interference of the point (or vertex) <paramref name="ip"/> on the curve <paramref name="ic"/>
    /// at <paramref name="par"/>.</summary>
    internal static DsCurvePointInterference ChFi3d_FilPointInDS(Orientation et, int ic, int ip,
                                                                            double par, bool isVertex = false)
    {
        DsCurvePointInterference cp1;
        if (isVertex)
        {
            cp1 = new DsCurvePointInterference(new DsTransition(et), DsKind.Curve, ic,
                                                          DsKind.Vertex, ip, par);
        }
        else
        {
            cp1 = new DsCurvePointInterference(new DsTransition(et), DsKind.Curve, ic,
                                                          DsKind.Point, ip, par);
        }
        return cp1;
    }

    //=================================================================================================

    /// <summary>The interference of the vertex <paramref name="ip"/> on the curve <paramref name="ic"/> at
    /// <paramref name="par"/>.</summary>
    internal static DsCurvePointInterference ChFi3d_FilVertexInDS(Orientation et, int ic, int ip,
                                                                             double par)
    {
        DsCurvePointInterference cp1 =
            new(new DsTransition(et), DsKind.Curve, ic, DsKind.Vertex, ip, par);
        return cp1;
    }

    //=======================================================================
    // function : Orientation
    // purpose  : returns the orientation of the interference (the first found
    //           in the list).
    //=======================================================================

    private static bool ChFi3d_Orientation(NCollection_List<DsInterference> li, int igros, int ipetit,
                                           ref Orientation or, bool isvertex = false, bool aprendre = false)
    {
        // In case, when it is necessary to insert a point/vertex, it should be
        // known if this is a point or a vertex, because their index can be the same.
        DsKind typepetit;
        if (isvertex)
        {
            typepetit = DsKind.Vertex;
        }
        else
        {
            typepetit = DsKind.Point;
        }
        foreach (DsInterference cur in li)
        {
            cur.GKGSKS(out DsKind gk, out int g, out DsKind sk, out int s);
            if (aprendre)
            {
                if (s == igros && g == ipetit && gk == typepetit)
                {
                    or = cur.Transition().Orientation(global::Kerf.Topology.Containment.In);
                    return true;
                }
            }
            else
            {
                if (s == igros && g == ipetit)
                {
                    or = cur.Transition().Orientation(global::Kerf.Topology.Containment.In);
                    return true;
                }
            }
        }
        return false;
    }

    //=======================================================================
    // function : Contains
    // purpose  : Check if the interference does not already exist.
    //====================================================================

    private static bool ChFi3d_Contains(NCollection_List<DsInterference> li, int igros, int ipetit,
                                        bool isvertex = false, bool aprendre = false)
    {
        Orientation bidOr = Forward;
        return ChFi3d_Orientation(li, igros, ipetit, ref bidOr, isvertex, aprendre);
    }

    //=================================================================================================

    private static void QueryAddVertexInEdge(NCollection_List<DsInterference> li, int ic, int iv,
                                             double par, Orientation or)
    {
        foreach (DsInterference cur in li)
        {
            if (cur is DsCurvePointInterference cpi)
            {
                int newIV = cpi.Geometry();
                DsKind kv = cpi.GeometryType();
                Orientation newOr = cpi.Transition().Orientation(global::Kerf.Topology.Containment.In);
                double newpar = cpi.Parameter();
                if (iv == newIV && kv == DsKind.Vertex && or == newOr && Math.Abs(par - newpar) < 1.0e-10)
                {
                    return;
                }
            }
        }
        DsCurvePointInterference interf = ChFi3d_FilVertexInDS(or, ic, iv, par);
        li.Append(interf);
    }

    //=================================================================================================

    private static void CutEdge(in Vertex v, SurfData sd, DsDataStructure dStr, bool _,
                                int ons)
    {
        if (!sd.IsOnCurve(ons))
        {
            return;
        }
        int ic = sd.IndexOfC(ons);
        int iv = dStr.AddShape(v);
        NCollection_List<DsInterference> li = dStr.ChangeShapeInterferences(ic);
        Edge e = ShapeCasts.Edge(dStr.Shape(ic));
        e.Orient(Forward);
        Explorer ex = new();

        // process them checking that it has not been done already.
        for (ex.Init(e, ShapeType.Vertex); ex.More(); ex.Next())
        {
            Vertex vv = ShapeCasts.Vertex(ex.Current());
            if (vv.IsSame(v))
            {
                Orientation or = global::Kerf.Engine.Topology.TopAbs.Reverse(vv.Orientation);
                double par = Tool.Parameter(vv, e);
                QueryAddVertexInEdge(li, ic, iv, par, or);
            }
        }
    }

    //=======================================================================
    // function : findIndexPoint
    // purpose  : returns in <ipon> index of point bounding a courve interfering
    //           with <Fd> and coinciding with last common point on <OnS> face
    //=======================================================================

    private static bool FindIndexPoint(DsDataStructure dStr, SurfData fd, int onS, out int ipoin)
    {
        ipoin = 0;
        Point3D p = fd.Vertex(false, onS).Point();

        foreach (DsInterference sciIt in dStr.SurfaceInterferences(fd.Surf()))
        {
            if (sciIt is not DsSurfaceCurveInterference sci)
            {
                continue;
            }

            foreach (DsInterference cpiIt in dStr.CurveInterferences(sci.Geometry()))
            {
                if (cpiIt is not DsCurvePointInterference cpi)
                {
                    continue;
                }

                int iPoint = cpi.Geometry();
                DsPoint tp = dStr.Point(iPoint);
                if (p.IsEqual(tp.Point(), tp.Tolerance()))
                {
                    ipoin = iPoint;
                    return true;
                }
            }
        }
        return false;
    }
}
