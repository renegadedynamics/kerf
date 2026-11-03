// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_0.cxx (ChFi3d_ComputeArete to findIndexPoint)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using static Occt.IntSurf_TypeTrans;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;
using static Occt.TopOpeBRepDS_Kind;

namespace Occt;

internal static partial class ChFi3d_Builder_0
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
    /// The edge of a fillet between two of its corners: its 3D curve <paramref name="C3d"/> and pcurve
    /// <paramref name="Pcurv"/> on <paramref name="Surf"/> (<paramref name="IFlag"/> 0), or only the pcurve of the
    /// given curve (<paramref name="IFlag"/> &gt; 0, its parameters imposed for 2).
    /// </summary>
    internal static void ChFi3d_ComputeArete(in ChFiDS_CommonPoint P1, in Point2D UV1, in ChFiDS_CommonPoint P2,
                                             in Point2D UV2, Geom_Surface Surf, ref Geom_Curve? C3d,
                                             ref Geom2d_Curve? Pcurv, ref double Pardeb, ref double Parfin,
                                             double tol3d, double tol2d, out double tolreached, int IFlag)
    {
        // IFlag=0 pcurve et courbe 3d
        // IFlag>0 pcurve (parametrage impose si IFlag=2)
        /*szv:static*/
        GeomAdaptor_Surface hs = new();
        /*szv:static*/
        GeomAdaptor_Curve hc = new();

        tolreached = tol3d;

        if (Math.Abs(UV1.X() - UV2.X()) <= tol2d)
        {
            if (IFlag == 0)
            {
                Pardeb = UV1.Y();
                Parfin = UV2.Y();
                C3d = Surf.UIso(UV1.X());
                if (Pardeb > Parfin)
                {
                    Pardeb = C3d.ReversedParameter(Pardeb);
                    Parfin = C3d.ReversedParameter(Parfin);
                    C3d.Reverse();
                }
                if (C3d is Geom_TrimmedCurve tc)
                {
                    C3d = tc.BasisCurve();
                    if (C3d.IsPeriodic())
                    {
                        global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(C3d.FirstParameter(), C3d.LastParameter(), tol2d, ref Pardeb,
                                              ref Parfin);
                    }
                }
            }
            if (IFlag != 1)
            {
                hs.Load(Surf);
                hc.Load(C3d!, Pardeb, Parfin);
                Adaptor3d_Curve aHCurve = hc; // to avoid ambiguity
                ChFi3d_ComputePCurv(aHCurve, UV1, UV2, out Geom2d_Curve aPcurv, hs, Pardeb, Parfin, tol3d,
                                    out tolreached, false);
                Pcurv = aPcurv;
            }
            else
            {
                Pcurv = new Geom2d_Line(UV1, new Direction2D(new Vector2D(UV1, UV2)));
            }
        }
        else if (Math.Abs(UV1.Y() - UV2.Y()) <= tol2d)
        {
            // iso v
            if (IFlag == 0)
            {
                Pardeb = UV1.X();
                Parfin = UV2.X();
                C3d = Surf.VIso(UV1.Y());
                if (Pardeb > Parfin)
                {
                    Pardeb = C3d.ReversedParameter(Pardeb);
                    Parfin = C3d.ReversedParameter(Parfin);
                    C3d.Reverse();
                }
                if (C3d is Geom_TrimmedCurve tc)
                {
                    C3d = tc.BasisCurve();
                    if (C3d.IsPeriodic())
                    {
                        global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(C3d.FirstParameter(), C3d.LastParameter(), tol2d, ref Pardeb,
                                              ref Parfin);
                    }
                }
            }
            if (IFlag != 1)
            {
                hs.Load(Surf);
                hc.Load(C3d!, Pardeb, Parfin);
                Adaptor3d_Curve aHCurve = hc; // to avoid ambiguity
                ChFi3d_ComputePCurv(aHCurve, UV1, UV2, out Geom2d_Curve aPcurv, hs, Pardeb, Parfin, tol3d,
                                    out tolreached, false);
                Pcurv = aPcurv;
            }
            else
            {
                Pcurv = new Geom2d_Line(UV1, new Direction2D(new Vector2D(UV1, UV2)));
            }
        }
        else if (IFlag == 0)
        {
            if (P1.IsVertex() || P2.IsVertex() || !P1.IsOnArc() || !P2.IsOnArc())
            {
                // A straight line is constructed to avoid
                // arc and tangent.
                NCollection_Array1<Point2D> qoles = new(1, 2);
                qoles[1] = UV1;
                qoles[2] = UV2;
                Pcurv = new Geom2d_BezierCurve(qoles);
            }
            else
            {
                BRepAdaptor_Curve C1 = new(P1.Arc());
                C1.D1(P1.ParameterOnArc(), out Point3D Pp, out Vector3D Vv1);
                C1.Initialize(P2.Arc());
                C1.D1(P2.ParameterOnArc(), out Pp, out Vector3D Vv2);
                hs.Load(Surf);
                Pcurv = ChFi3d_BuildPCurve(hs, UV1, Vv1, UV2, Vv2, true);
                // There are some cases when PCurve constructed in this way
                // leaves the surface, in particular if it results from an
                // extension. A posteriori checking is required and if
                // the curve leaves the surface it is replaced by straight line UV1 UV2
                // non regarding the tangency with neighboring arcs!
                Box2d bs = new();
                double umin, umax, vmin, vmax;
                Surf.Bounds(out umin, out umax, out vmin, out vmax);
                bs.Update(umin, vmin, umax, vmax);
                bs.SetGap(global::Kerf.Engine.Foundation.Precision.PConfusion());
                bool aIN = true;
                for (int ii = 1; ii <= 4 && aIN; ii++)
                {
                    if (bs.IsOut(((Geom2d_BezierCurve)Pcurv).Pole(ii)))
                    {
                        aIN = false;
                        NCollection_Array1<Point2D> qoles = new(1, 2);
                        qoles[1] = UV1;
                        qoles[2] = UV2;
                        Pcurv = new Geom2d_BezierCurve(qoles);
                    }
                }
            }
            Geom2dAdaptor_Curve AC = new(Pcurv);
            Geom2dAdaptor_Curve AHC = (Geom2dAdaptor_Curve)AC.ShallowCopy();
            GeomAdaptor_Surface AS = new(Surf);
            GeomAdaptor_Surface AHS = (GeomAdaptor_Surface)AS.ShallowCopy();
            Adaptor3d_CurveOnSurface Cs = new(AHC, AHS);
            Pardeb = Cs.FirstParameter();
            Parfin = Cs.LastParameter();
            double avtol;
            GeomLib.BuildCurve3d(tol3d, Cs, Pardeb, Parfin, ref C3d, out tolreached, out avtol);
        }
        else
        {
            hs.Load(Surf);
            hc.Load(C3d!, Pardeb, Parfin);
            ChFi3d_ProjectPCurv(hc, hs, ref Pcurv, tol3d, ref tolreached);
            Point2D p2d = Pcurv!.Value(Pardeb);
            if (!UV1.IsEqual(p2d, global::Kerf.Engine.Foundation.Precision.PConfusion()))
            {
                Vector2D v2d = new(p2d, UV1);
                Pcurv = Writable(Pcurv);
                Pcurv.Translate(v2d);
            }
        }
    }

    //=================================================================================================

    /// <summary>The interference of the curve <paramref name="Icurv"/> on the surface <paramref name="Isurf"/> by its
    /// pcurve, with the transition <paramref name="Et"/>.</summary>
    internal static TopOpeBRepDS_SurfaceCurveInterference ChFi3d_FilCurveInDS(int Icurv, int Isurf,
                                                                              Geom2d_Curve? Pcurv,
                                                                              TopAbs_Orientation Et)
    {
        TopOpeBRepDS_SurfaceCurveInterference SC1;
        SC1 = new TopOpeBRepDS_SurfaceCurveInterference(new TopOpeBRepDS_Transition(Et), TopOpeBRepDS_SURFACE, Isurf,
                                                        TopOpeBRepDS_CURVE, Icurv, Pcurv);
        return SC1;
    }

    //=================================================================================================

    /// <summary>The orientation of a transition of the walking: In is FORWARD, Out REVERSED, else INTERNAL.</summary>
    internal static TopAbs_Orientation ChFi3d_TrsfTrans(IntSurf_TypeTrans T1)
    {
        switch (T1)
        {
            case IntSurf_In:
                return TopAbs_FORWARD;
            case IntSurf_Out:
                return TopAbs_REVERSED;
            default:
                break;
        }
        return TopAbs_INTERNAL;
    }

    //=======================================================================
    // function : FilCommonPoint
    // purpose  : Loading of the common point
    //           management of the case when it happens on already existing vertex.
    //=======================================================================

    /// <summary>
    /// Loads the common point <paramref name="CP"/> from the walking's extremity <paramref name="SP"/>: its point,
    /// tangent and parameter on the guide, the vertex it lies on, or the arc with its transition, and the tolerance.
    /// </summary>
    /// <remarks>Reads the extremity's vertices and edges (the builder's working shape) and writes only
    /// <paramref name="CP"/>.</remarks>
    internal static void ChFi3d_FilCommonPoint(in BRepBlend_Extremity SP, IntSurf_TypeTrans TransLine, bool Start,
                                               ref ChFiDS_CommonPoint CP, double Tol)
    {
        //  BRep_Tool Outil;
        double Dist, maxtol = Math.Max(Tol, CP.Tolerance());

        CP.SetPoint(SP.Value()); // One starts with the point and the vector
        if (SP.HasTangent())
        {
            if (Start)
            {
                CP.SetVector(SP.Tangent().Reversed()); // The tangent is oriented to the exit
            }
            else
            {
                CP.SetVector(SP.Tangent());
            }
        }

        CP.SetParameter(SP.ParameterOnGuide()); // and the parameter of the spine

        if (SP.IsVertex())
        { // the Vertex is loaded if required
            // (inside of a face)
            TopoDS_Vertex V = ((BRepTopAdaptor_HVertex)SP.Vertex()).Vertex();

            CP.SetVertex(V);
            Dist = (SP.Value()).Distance(BRep_Tool.Pnt(V));
            //// modified by jgv, 18.09.02 for OCC571 ////
            // maxtol += Dist;
            maxtol = Math.Max(Dist, maxtol);
            //////////////////////////////////////////////
            CP.SetPoint(BRep_Tool.Pnt(V));

            // the sequence of arcs the information is known by thee vertex (ancestor)
            // in this case the transitions are not computed, it is done by this program
        }

        if (SP.NbPointOnRst() != 0)
        { //  An arc, and/or a vertex is loaded

            BRepBlend_PointOnRst PR = SP.PointOnRst(1);
            if (PR.Arc() is BRepAdaptor_Curve2d Harc)
            {
                double DistF, DistL, LeParamAmoi;
                int Index_min;
                TopoDS_Edge E = Harc.Edge();

                TopoDS_Vertex[] V = new TopoDS_Vertex[2];
                TopExp.Vertices(E, out V[0], out V[1]);

                DistF = (SP.Value()).Distance(BRep_Tool.Pnt(V[0]));
                DistL = (SP.Value()).Distance(BRep_Tool.Pnt(V[1]));
                if (DistF < DistL)
                {
                    Index_min = 0;
                    Dist = DistF;
                }
                else
                {
                    Index_min = 1;
                    Dist = DistL;
                }

                if (Dist <= maxtol + BRep_Tool.Tolerance(V[Index_min]))
                {
                    // a preexisting vertex has been met
                    CP.SetVertex(V[Index_min]); // the old vertex is loaded
                    CP.SetPoint(BRep_Tool.Pnt(V[Index_min]));
                    maxtol = Math.Max(BRep_Tool.Tolerance(V[Index_min]), maxtol);
                    //// modified by jgv, 18.09.02 for OCC571 ////
                    // maxtol += Dist;
                    maxtol = Math.Max(Dist, maxtol);
                    //////////////////////////////////////////////
                    LeParamAmoi = BRep_Tool.Parameter(V[Index_min], E);
                }
                else
                { // Creation of an arc only
                    maxtol = Math.Max(BRep_Tool.Tolerance(E), maxtol);
                    maxtol = Math.Max(SP.Tolerance(), maxtol);
                    LeParamAmoi = PR.ParameterOnArc();
                }

                // Definition of the arc
                TopAbs_Orientation Tr;
                TopAbs_Orientation Or = E.Orientation();
                if (Start)
                {
                    Tr = TopAbs.Reverse(TopAbs.Compose(ChFi3d_TrsfTrans(TransLine), Or));
                }
                else
                {
                    Tr = TopAbs.Compose(ChFi3d_TrsfTrans(TransLine), Or);
                }
                CP.SetArc(maxtol, E, LeParamAmoi, Tr);
            }
        }
        CP.SetTolerance(maxtol); // Finally, the tolerance.
    }

    //=================================================================================================

    /// <summary>Adds to the data structure the solid (or shell) of the spine's first edge; returns its index.</summary>
    /// <exception cref="FailureException">The spine has no edge.</exception>
    internal static int ChFi3d_SolidIndex(ChFiDS_Spine? sp, TopOpeBRepDS_DataStructure DStr, ChFiDS_Map MapESo,
                                          ChFiDS_Map MapESh)
    {
        if (sp is null || sp.NbEdges() == 0)
        {
            throw new FailureException("SolidIndex : Spine incomplete");
        }
        TopoDS_Shape edref = sp.Edges(1);
        TopoDS_Shape shellousolid;
        if (!MapESo[edref].IsEmpty())
        {
            shellousolid = MapESo[edref].First();
        }
        else
        {
            shellousolid = MapESh[edref].First();
        }
        int solidindex = DStr.AddShape(shellousolid);
        return solidindex;
    }

    //=================================================================================================

    /// <summary>
    /// Adds the common point to the data structure: its vertex, after setting the vertex's point and raising its
    /// tolerance to the common point's, or else a new point. Returns its index.
    /// </summary>
    /// <remarks>
    /// <b>This writes into the vertex</b> (<c>BRep_Builder::UpdateVertex</c>, OCCT's line 2333 and plan 2.2): the
    /// common points hold the vertices of the shape the builder works on, which must therefore be the builder's
    /// working copy, unlocked (decision 1087). A locked vertex throws <see cref="TopoDS_LockedShape"/>.
    /// </remarks>
    internal static int ChFi3d_IndexPointInDS(in ChFiDS_CommonPoint P1, TopOpeBRepDS_DataStructure DStr)
    {
        if (P1.IsVertex())
        {
            // --------------------------------->  !*!*!*
            // Attention : it is necessary ti implement a mechanism
            // controlling tolerance.
            BRep_Builder B = new();
            B.UpdateVertex(P1.Vertex(), P1.Point(), P1.Tolerance());
            return DStr.AddShape(P1.Vertex());
        }
        return DStr.AddPoint(new TopOpeBRepDS_Point(P1.Point(), P1.Tolerance()));
    }

    //=================================================================================================

    /// <summary>The interference of the point (or vertex) <paramref name="Ip"/> on the curve <paramref name="Ic"/>
    /// at <paramref name="Par"/>.</summary>
    internal static TopOpeBRepDS_CurvePointInterference ChFi3d_FilPointInDS(TopAbs_Orientation Et, int Ic, int Ip,
                                                                            double Par, bool IsVertex = false)
    {
        TopOpeBRepDS_CurvePointInterference CP1;
        if (IsVertex)
        {
            CP1 = new TopOpeBRepDS_CurvePointInterference(new TopOpeBRepDS_Transition(Et), TopOpeBRepDS_CURVE, Ic,
                                                          TopOpeBRepDS_VERTEX, Ip, Par);
        }
        else
        {
            CP1 = new TopOpeBRepDS_CurvePointInterference(new TopOpeBRepDS_Transition(Et), TopOpeBRepDS_CURVE, Ic,
                                                          TopOpeBRepDS_POINT, Ip, Par);
        }
        return CP1;
    }

    //=================================================================================================

    /// <summary>The interference of the vertex <paramref name="Ip"/> on the curve <paramref name="Ic"/> at
    /// <paramref name="Par"/>.</summary>
    internal static TopOpeBRepDS_CurvePointInterference ChFi3d_FilVertexInDS(TopAbs_Orientation Et, int Ic, int Ip,
                                                                             double Par)
    {
        TopOpeBRepDS_CurvePointInterference CP1 =
            new(new TopOpeBRepDS_Transition(Et), TopOpeBRepDS_CURVE, Ic, TopOpeBRepDS_VERTEX, Ip, Par);
        return CP1;
    }

    //=======================================================================
    // function : Orientation
    // purpose  : returns the orientation of the interference (the first found
    //           in the list).
    //=======================================================================

    private static bool ChFi3d_Orientation(NCollection_List<TopOpeBRepDS_Interference> LI, int igros, int ipetit,
                                           ref TopAbs_Orientation Or, bool isvertex = false, bool aprendre = false)
    {
        // In case, when it is necessary to insert a point/vertex, it should be
        // known if this is a point or a vertex, because their index can be the same.
        TopOpeBRepDS_Kind typepetit;
        if (isvertex)
        {
            typepetit = TopOpeBRepDS_VERTEX;
        }
        else
        {
            typepetit = TopOpeBRepDS_POINT;
        }
        foreach (TopOpeBRepDS_Interference cur in LI)
        {
            cur.GKGSKS(out TopOpeBRepDS_Kind GK, out int G, out TopOpeBRepDS_Kind SK, out int S);
            if (aprendre)
            {
                if (S == igros && G == ipetit && GK == typepetit)
                {
                    Or = cur.Transition().Orientation(TopAbs_IN);
                    return true;
                }
            }
            else
            {
                if (S == igros && G == ipetit)
                {
                    Or = cur.Transition().Orientation(TopAbs_IN);
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

    private static bool ChFi3d_Contains(NCollection_List<TopOpeBRepDS_Interference> LI, int igros, int ipetit,
                                        bool isvertex = false, bool aprendre = false)
    {
        TopAbs_Orientation bidOr = TopAbs_FORWARD;
        return ChFi3d_Orientation(LI, igros, ipetit, ref bidOr, isvertex, aprendre);
    }

    //=================================================================================================

    private static void QueryAddVertexInEdge(NCollection_List<TopOpeBRepDS_Interference> LI, int IC, int IV,
                                             double par, TopAbs_Orientation Or)
    {
        foreach (TopOpeBRepDS_Interference cur in LI)
        {
            if (cur is TopOpeBRepDS_CurvePointInterference cpi)
            {
                int newIV = cpi.Geometry();
                TopOpeBRepDS_Kind kv = cpi.GeometryType();
                TopAbs_Orientation newOr = cpi.Transition().Orientation(TopAbs_IN);
                double newpar = cpi.Parameter();
                if (IV == newIV && kv == TopOpeBRepDS_VERTEX && Or == newOr && Math.Abs(par - newpar) < 1.0e-10)
                {
                    return;
                }
            }
        }
        TopOpeBRepDS_CurvePointInterference interf = ChFi3d_FilVertexInDS(Or, IC, IV, par);
        LI.Append(interf);
    }

    //=================================================================================================

    private static void CutEdge(in TopoDS_Vertex V, ChFiDS_SurfData SD, TopOpeBRepDS_DataStructure DStr, bool _,
                                int ons)
    {
        if (!SD.IsOnCurve(ons))
        {
            return;
        }
        int IC = SD.IndexOfC(ons);
        int IV = DStr.AddShape(V);
        NCollection_List<TopOpeBRepDS_Interference> LI = DStr.ChangeShapeInterferences(IC);
        TopoDS_Edge E = TopoDS.Edge(DStr.Shape(IC));
        E.Orientation(TopAbs_FORWARD);
        TopExp_Explorer ex = new();

        // process them checking that it has not been done already.
        for (ex.Init(E, TopAbs_VERTEX); ex.More(); ex.Next())
        {
            TopoDS_Vertex vv = TopoDS.Vertex(ex.Current());
            if (vv.IsSame(V))
            {
                TopAbs_Orientation Or = TopAbs.Reverse(vv.Orientation());
                double par = BRep_Tool.Parameter(vv, E);
                QueryAddVertexInEdge(LI, IC, IV, par, Or);
            }
        }
    }

    //=======================================================================
    // function : findIndexPoint
    // purpose  : returns in <ipon> index of point bounding a courve interfering
    //           with <Fd> and coinciding with last common point on <OnS> face
    //=======================================================================

    private static bool findIndexPoint(TopOpeBRepDS_DataStructure DStr, ChFiDS_SurfData Fd, int OnS, out int ipoin)
    {
        ipoin = 0;
        Point3D P = Fd.Vertex(false, OnS).Point();

        foreach (TopOpeBRepDS_Interference SCIIt in DStr.SurfaceInterferences(Fd.Surf()))
        {
            if (SCIIt is not TopOpeBRepDS_SurfaceCurveInterference SCI)
            {
                continue;
            }

            foreach (TopOpeBRepDS_Interference CPIIt in DStr.CurveInterferences(SCI.Geometry()))
            {
                if (CPIIt is not TopOpeBRepDS_CurvePointInterference CPI)
                {
                    continue;
                }

                int iPoint = CPI.Geometry();
                TopOpeBRepDS_Point tp = DStr.Point(iPoint);
                if (P.IsEqual(tp.Point(), tp.Tolerance()))
                {
                    ipoin = iPoint;
                    return true;
                }
            }
        }
        return false;
    }
}
