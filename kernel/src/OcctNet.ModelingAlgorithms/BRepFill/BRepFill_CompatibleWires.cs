// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_CompatibleWires.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_CompatibleWires.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Schematiq.Collections;
using static Occt.BRepCheck_Status;
using static Occt.BRepExtrema_SupportType;
using static Occt.BRepFill_ThruSectionErrorStatus;
using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Makes a sequence of wires compatible for a loft or a sweep: the same number of edges in each wire, with origins and
/// orientations chosen to avoid twisted results. Closed sections are cut by a polar method (or reordered when they
/// already have as many edges), open ones by reduced curvilinear abscissa; a first or last section may be a point.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). The wires it returns hold the caller's edges where they weren't cut and new edges on
/// the caller's curves where they were; the caller's shapes are never written (decision 905). <see cref="Shape"/>,
/// <see cref="GeneratedShapes"/> and <see cref="Generated"/> hand out frozen shapes; ported callers (the lofts and
/// sweeps) read the algorithm's own containers through the <c>Internal</c> forms.
/// </para>
/// </remarks>
public sealed class BRepFill_CompatibleWires
{
#pragma warning disable CS0414 // set and never read, as in OCCT
    private NCollection_Sequence<TopoDS_Shape> myInit = new();
#pragma warning restore CS0414
    private NCollection_Sequence<TopoDS_Shape> myWork = new();
    private double myPercent;
    private bool myDegen1;
    private bool myDegen2;
    private BRepFill_ThruSectionErrorStatus myStatus;
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myMap = new(TopTools_ShapeMapHasher.Instance);

    //=======================================================================
    // function : AddNewEdge
    // purpose  : for <theEdge> find all newest edges
    //           in <theEdgeNewEdges> recursively
    //=======================================================================

    private static void AddNewEdge(in TopoDS_Shape theEdge,
                                   NCollection_DataMap<TopoDS_Shape, NCollection_Sequence<TopoDS_Shape>> theEdgeNewEdges,
                                   NCollection_List<TopoDS_Shape> ListNewEdges)
    {
        if (theEdgeNewEdges.IsBound(theEdge))
        {
            NCollection_Sequence<TopoDS_Shape> NewEdges = theEdgeNewEdges.Find(theEdge);
            for (int i = 1; i <= NewEdges.Length(); i++)
            {
                TopoDS_Shape anEdge = NewEdges[i];
                AddNewEdge(anEdge, theEdgeNewEdges, ListNewEdges);
            }
        }
        else
        {
            ListNewEdges.Append(theEdge);
        }
    }

    private static void SeqOfVertices(in TopoDS_Wire W, NCollection_Sequence<TopoDS_Shape> S)
    {
        S.Clear();
        int jj;
        TopExp_Explorer PE = new();
        for (PE.Init(W, TopAbs_VERTEX); PE.More(); PE.Next())
        {
            bool trouve = false;
            for (jj = 1; jj <= S.Length() && (!trouve); jj++)
            {
                if (S.Value(jj).IsSame(PE.Current()))
                {
                    trouve = true;
                }
            }
            if (!trouve)
            {
                S.Append(PE.Current());
            }
        }
    }

    private static bool PlaneOfWire(in TopoDS_Wire W, out gp_Pln P)
    {
        P = new gp_Pln();
        bool isplane = true;
        BRepLib_FindSurface findPlanarSurf = new();
        Geom_Surface? S;
        TopLoc_Location L;

        GProp_GProps GP = new();
        gp_Pnt Bary = new();
        bool isBaryDefined = false;

        // shielding for particular cases : only one edge circle or ellipse
        // on a closed wire !

        bool wClosed = W.Closed();
        if (!wClosed)
        {
            // it is checked if the vertices are the same.
            TopExp.Vertices(W, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
            if (V1.IsSame(V2))
            {
                wClosed = true;
            }
        }

        if (wClosed)
        {
            int nbEdges = 0;
            TopoDS_Iterator anIter = new();
            anIter.Initialize(W);
            for (; anIter.More(); anIter.Next())
            {
                nbEdges++;
            }

            if (nbEdges == 1)
            {
                GeomAdaptor_Curve AdC = new();
                anIter.Initialize(W);
                AdC.Load(BRep_Tool.Curve(TopoDS.Edge(anIter.Value()), out _, out _)!);

                if (AdC.GetType() == GeomAbs_Circle)
                {
                    Bary = AdC.Circle().Location();
                    isBaryDefined = true;
                }

                if (AdC.GetType() == GeomAbs_Ellipse)
                {
                    Bary = AdC.Ellipse().Location();
                    isBaryDefined = true;
                }
            }
        }

        if (!isBaryDefined)
        {
            BRepGProp.LinearProperties(W, GP);
            Bary = GP.CentreOfMass();
        }

        findPlanarSurf.Init(W, -1, true);
        if (findPlanarSurf.Found())
        {
            S = findPlanarSurf.Surface();
            L = findPlanarSurf.Location();
            if (!L.IsIdentity())
            {
                S = (Geom_Surface)S!.Transformed(L.Transformation());
            }
            P = ((Geom_Plane)S!).Pln();
            P.SetLocation(Bary);
        }
        else
        {
            // wire not plane !
            GProp_PrincipalProps Pp = GP.PrincipalProperties();
            gp_Vec Vec = new();
            double Tol = Precision.Confusion();
            Pp.RadiusOfGyration(out double R1, out double R2, out double R3);
            double RMax = Max(Max(R1, R2), R3);
            if ((Math.Abs(RMax - R1) < Tol && Math.Abs(RMax - R2) < Tol)
                || (Math.Abs(RMax - R1) < Tol && Math.Abs(RMax - R3) < Tol)
                || (Math.Abs(RMax - R2) < Tol && Math.Abs(RMax - R3) < Tol))
            {
                isplane = false;
            }
            else
            {
                if (R1 >= R2 && R1 >= R3)
                {
                    Vec = Pp.FirstAxisOfInertia();
                }
                else if (R2 >= R1 && R2 >= R3)
                {
                    Vec = Pp.SecondAxisOfInertia();
                }
                else if (R3 >= R1 && R3 >= R2)
                {
                    Vec = Pp.ThirdAxisOfInertia();
                }
                gp_Dir NDir = new(Vec);
                if (R3 <= R2 && R3 <= R1)
                {
                    Vec = Pp.ThirdAxisOfInertia();
                }
                else if (R2 <= R1 && R2 <= R3)
                {
                    Vec = Pp.SecondAxisOfInertia();
                }
                else if (R1 <= R2 && R1 <= R3)
                {
                    Vec = Pp.FirstAxisOfInertia();
                }
                gp_Dir XDir = new(Vec);
                gp_Ax3 repere = new(Bary, NDir, XDir);
                Geom_Plane GPlan = new(repere);
                P = GPlan.Pln();
            }
        }

        return isplane;
    }

    private static void WireContinuity(in TopoDS_Wire W, out GeomAbs_Shape contW)
    {
        contW = GeomAbs_CN;
        GeomAbs_Shape cont;
        bool IsDegenerated = false;

        BRepTools_WireExplorer anExp = new();
        int nbEdges = 0;
        NCollection_HSequence<TopoDS_Shape> Edges = new();
        for (anExp.Init(W); anExp.More(); anExp.Next())
        {
            nbEdges++;
            Edges.Append(anExp.Current());
            if (BRep_Tool.Degenerated(anExp.Current()))
            {
                IsDegenerated = true;
            }
        }

        if (!IsDegenerated)
        {

            bool testconti = true;

            for (int j = 1; j <= nbEdges; j++)
            {

                TopoDS_Edge Edge1, Edge2;

                if (j == nbEdges)
                {
                    Edge1 = TopoDS.Edge(Edges.Value(nbEdges));
                    Edge2 = TopoDS.Edge(Edges.Value(1));
                }
                else
                {
                    Edge1 = TopoDS.Edge(Edges.Value(j));
                    Edge2 = TopoDS.Edge(Edges.Value(j + 1));
                }

                TopExp.Vertices(Edge1, out _, out TopoDS_Vertex V1, true);
                TopExp.Vertices(Edge2, out TopoDS_Vertex V2, out _, true);
                double U1 = BRep_Tool.Parameter(V1, Edge1);
                double U2 = BRep_Tool.Parameter(V2, Edge2);
                BRepAdaptor_Curve Curve1 = new(Edge1);
                BRepAdaptor_Curve Curve2 = new(Edge2);
                double Eps = BRep_Tool.Tolerance(V2) + BRep_Tool.Tolerance(V1);

                if (j == nbEdges)
                {
                    testconti = Curve1.Value(U1).IsEqual(Curve2.Value(U2), Eps);
                }

                if (testconti)
                {
                    cont = BRepLProp.Continuity(Curve1, Curve2, U1, U2, Eps, Precision.Angular());
                    if (cont <= contW)
                    {
                        contW = cont;
                    }
                }
            }
        }
    }

    private static void TrimEdge(in TopoDS_Edge CurrentEdge,
                                 NCollection_Sequence<double> CutValues,
                                 double t0,
                                 double t1,
                                 bool SeqOrder,
                                 NCollection_Sequence<TopoDS_Shape> S)

    {
        S.Clear();
        int j, ndec = CutValues.Length();
        double m0, m1;
        Geom_Curve C = BRep_Tool.Curve(CurrentEdge, out double first, out double last)!;

        TopoDS_Vertex Vbid = new(), V0, V1;
        TopAbs_Orientation CurrentOrient = CurrentEdge.Orientation();
        TopExp.Vertices(CurrentEdge, out TopoDS_Vertex Vf, out TopoDS_Vertex Vl);
        Vbid.Nullify();

        if (SeqOrder)
        {
            // from first to last
            m0 = first;
            V0 = Vf;
            for (j = 1; j <= ndec; j++)
            {
                // piece of edge
                m1 = (CutValues.Value(j) - t0) * (last - first) / (t1 - t0) + first;
                if (Math.Abs(m0 - m1) < Precision.Confusion())
                {
                    return;
                }
                TopoDS_Edge CutE = new BRepLib_MakeEdge(C, V0, Vbid, m0, m1).InternalEdge();
                CutE.Orientation(CurrentOrient);
                S.Append(CutE);
                m0 = m1;
                V0 = TopExp.LastVertex(CutE);
                if (j == ndec)
                {
                    // last piece
                    if (Math.Abs(m0 - last) < Precision.Confusion())
                    {
                        return;
                    }
                    TopoDS_Edge LastE = new BRepLib_MakeEdge(C, V0, Vl, m0, last).InternalEdge();
                    LastE.Orientation(CurrentOrient);
                    S.Append(LastE);
                }
            }
        }
        else
        {
            // from last to first
            m1 = last;
            V1 = Vl;
            for (j = ndec; j >= 1; j--)
            {
                // piece of edge
                m0 = (CutValues.Value(j) - t0) * (last - first) / (t1 - t0) + first;
                if (Math.Abs(m0 - m1) < Precision.Confusion())
                {
                    return;
                }
                TopoDS_Edge CutE = new BRepLib_MakeEdge(C, Vbid, V1, m0, m1).InternalEdge();
                CutE.Orientation(CurrentOrient);
                S.Append(CutE);
                m1 = m0;
                V1 = TopExp.FirstVertex(CutE);
                if (j == 1)
                {
                    // last piece
                    if (Math.Abs(first - m1) < Precision.Confusion())
                    {
                        return;
                    }
                    TopoDS_Edge LastE = new BRepLib_MakeEdge(C, Vf, V1, first, m1).InternalEdge();
                    LastE.Orientation(CurrentOrient);
                    S.Append(LastE);
                }
            }
        }
    }

    // OCCT: native iterates the map in the hash order of TShape addresses; the port iterates in binding order
    // (decision 18). The first key whose list holds V wins, so the order matters only when V is in several lists
    // (decision 908).
    private static bool SearchRoot(in TopoDS_Vertex V,
                                   NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> Map,
                                   out TopoDS_Vertex VRoot)
    {
        bool trouve = false;
        VRoot = new TopoDS_Vertex();
        VRoot.Nullify();
        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> it in Map)
        {
            NCollection_List<TopoDS_Shape> List = it.Value;
            NCollection_List<TopoDS_Shape>.Iterator itL = new();
            bool ilyest = false;
            for (itL.Initialize(List); itL.More(); itL.Next())
            {
                TopoDS_Vertex Vcur = TopoDS.Vertex(itL.Value());
                if (Vcur.IsSame(V))
                {
                    ilyest = true;
                }
                if (ilyest)
                {
                    break;
                }
            }
            if (ilyest)
            {
                trouve = true;
                VRoot = TopoDS.Vertex(it.Key);
            }
            if (trouve)
            {
                break;
            }
        }
        return trouve;
    }

    private static bool SearchVertex(NCollection_List<TopoDS_Shape> List, in TopoDS_Wire W, out TopoDS_Vertex VonW)
    {
        bool trouve = false;
        VonW = new TopoDS_Vertex();
        VonW.Nullify();
        NCollection_Sequence<TopoDS_Shape> SeqV = new();
        SeqOfVertices(W, SeqV);
        for (int ii = 1; ii <= SeqV.Length(); ii++)
        {
            TopoDS_Vertex Vi = TopoDS.Vertex(SeqV.Value(ii));
            NCollection_List<TopoDS_Shape>.Iterator itL = new();
            bool ilyest = false;
            for (itL.Initialize(List); itL.More(); itL.Next())
            {
                TopoDS_Vertex Vcur = TopoDS.Vertex(itL.Value());
                if (Vcur.IsSame(Vi))
                {
                    ilyest = true;
                }
                if (ilyest)
                {
                    break;
                }
            }
            if (ilyest)
            {
                trouve = true;
                VonW = Vi;
            }
            if (trouve)
            {
                break;
            }
        }
        return trouve;
    }

    private static bool EdgeIntersectOnWire(in gp_Pnt P1,
                                            in gp_Pnt P2,
                                            double percent,
                                            NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> Map,
                                            in TopoDS_Wire W,
                                            ref TopoDS_Vertex Vsol,
                                            ref TopoDS_Wire newW,
                                            NCollection_DataMap<TopoDS_Shape, NCollection_Sequence<TopoDS_Shape>> theEdgeNewEdges)
    {

        BRepTools_WireExplorer anExp = new();

        // construction of the edge of intersection
        bool NewVertex = false;
        gp_Lin droite = new(P1, new gp_Dir(new gp_Vec(P1, P2)));
        // ATTENTION : it is required to construct a half-straight
        //             but there is a bug in BRepExtrema_DistShapeShape
        //             it is enough to take 100 * distance between P1 and P2
        //             hoping that it is enough until the bug is corrected
        //  double dernierparam = Precision::Infinite();
        // ATTENTION : return !!
        //             100 is better than 10 but it is too much !
        //             finally, nothing is better than a blocking box
        //  double dernierparam = 100 * P1.Distance(P2);
        Bnd_Box B = new();
        BRepBndLib.Add(W, ref B);
        B.Get(out double x1, out double y1, out double z1, out double x2, out double y2, out double z2);
        gp_Pnt BP1 = new(x1, y1, z1), BP2 = new(x2, y2, z2);
        double diag = BP1.Distance(BP2);
        double dernierparam = diag;
        BRepLib_MakeEdge ME = new(droite, 0.0, dernierparam);
        TopoDS_Edge ECur = new BRepLib_MakeEdge(droite, 0.0, P1.Distance(P2)).InternalEdge();

        // calculate the intersection by BRepExtrema (point of min distance)
        BRepExtrema_DistShapeShape DSS = new(ME.InternalEdge(), W);
        if (DSS.IsDone())
        {
            // choose the solution closest to P2
            int isol = 1;
            gp_Pnt Psol = DSS.PointOnShape2(isol);
            double dss = P2.Distance(Psol);
            for (int iss = 2; iss <= DSS.NbSolution(); iss++)
            {
                gp_Pnt Pss = DSS.PointOnShape2(iss);
                double aDist = P2.Distance(Pss);
                if (dss > aDist)
                {
                    dss = aDist;
                    isol = iss;
                }
            }

            // is the solution a new vertex ?
            NewVertex = (DSS.SupportTypeShape2(isol) != BRepExtrema_IsVertex);
            if (NewVertex)
            {
                TopoDS_Edge E = TopoDS.Edge(DSS.SupportOnShape2(isol));
                double tol = Precision.PConfusion();
                BRep_Tool.Range(E, out double first, out double last);
                tol = Max(tol, percent * Math.Abs(last - first));
                DSS.ParOnEdgeS2(isol, out double param);
                if (Math.Abs(first - param) < tol)
                {
                    NewVertex = false;
                    Vsol = TopExp.FirstVertex(E);
                }
                else if (Math.Abs(last - param) < tol)
                {
                    NewVertex = false;
                    Vsol = TopExp.LastVertex(E);
                }
                // check
                if (!NewVertex)
                {
                    if (SearchRoot(Vsol, Map, out _))
                    {
                        NewVertex = true;
                    }
                }
            }
            else
            {
                TopoDS_Shape aLocalShape = DSS.SupportOnShape2(isol);
                Vsol = TopoDS.Vertex(aLocalShape);
                //      Vsol = TopoDS::Vertex(DSS.SupportOnShape2(isol));
            }

            // it is required to cut the edge
            if (NewVertex)
            {
                TopoDS_Shape aLocalShape = DSS.SupportOnShape2(isol);
                TopoDS_Edge E = TopoDS.Edge(aLocalShape);
                //      TopoDS_Edge E = TopoDS::Edge(DSS.SupportOnShape2(isol));
                NCollection_Sequence<TopoDS_Shape> EmptySeq = new();
                theEdgeNewEdges.Bind(E, EmptySeq);
                DSS.ParOnEdgeS2(isol, out double param);
                BRep_Tool.Range(E, out double first, out double last);
                BRepLib_MakeWire MW = new();
                for (anExp.Init(W); anExp.More(); anExp.Next())
                {
                    if (E.IsSame(anExp.Current()))
                    {
                        bool SO = (anExp.CurrentVertex().IsSame(TopExp.FirstVertex(E)));
                        NCollection_Sequence<TopoDS_Shape> SE = new();
                        SE.Clear();
                        NCollection_Sequence<double> SR = new();
                        SR.Clear();
                        SR.Append(param);
                        TrimEdge(E, SR, first, last, SO, SE);
                        if (SE.IsEmpty())
                        {
                            return false;
                        }
                        theEdgeNewEdges[E] = SE;
                        TopExp.Vertices(TopoDS.Edge(SE.Value(1)), out TopoDS_Vertex VV1, out TopoDS_Vertex VV2);
                        if (TopExp.FirstVertex(E).IsSame(VV1) || TopExp.LastVertex(E).IsSame(VV1))
                        {
                            Vsol = VV2;
                        }
                        if (TopExp.FirstVertex(E).IsSame(VV2) || TopExp.LastVertex(E).IsSame(VV2))
                        {
                            Vsol = VV1;
                        }
                        for (int k = 1; k <= SE.Length(); k++)
                        {
                            MW.Add(TopoDS.Edge(SE.Value(k)));
                        }
                    }
                    else
                    {
                        MW.Add(anExp.Current());
                    }
                }
                newW = MW.InternalWire();
            }
            else
            {
                newW = W;
            }
        }

        return NewVertex;
    }

    private static void Transform(bool WithRotation,
                                  in gp_Pnt P,
                                  in gp_Pnt Pos1,
                                  in gp_Vec Ax1,
                                  in gp_Pnt Pos2,
                                  in gp_Vec Ax2,
                                  out gp_Pnt Pnew)
    {

        Pnew = P.Translated(Pos1, Pos2);
        gp_Vec axe1 = Ax1, axe2 = Ax2;
        if (!axe1.IsParallel(axe2, 1.0e-4))
        {
            gp_Vec Vtrans = new(Pos1, Pos2), Vsign = new();
            double alpha, beta, sign = 1;
            alpha = Vtrans.Dot(axe1);
            beta = Vtrans.Dot(axe2);
            if (alpha < -1.0e-7)
            {
                axe1 *= -1;
            }
            if (beta < 1.0e-7)
            {
                axe2 *= -1;
            }
            alpha = Vtrans.Dot(axe1);
            beta = Vtrans.Dot(axe2);
            gp_Vec norm2 = axe1 ^ axe2;
            Vsign.SetLinearForm(Vtrans.Dot(axe1), axe2, -Vtrans.Dot(axe2), axe1);
            alpha = Vsign.Dot(axe1);
            beta = Vsign.Dot(axe2);
            bool pasnul = (Math.Abs(alpha) > 1.0e-4 && Math.Abs(beta) > 1.0e-4);
            if (alpha * beta > 0.0 && pasnul)
            {
                sign = -1;
            }
            gp_Ax1 Norm = new(Pos2, new gp_Dir(norm2));
            double ang = axe1.AngleWithRef(axe2, norm2);
            if (!WithRotation)
            {
                if (ang > M_PI / 2)
                {
                    ang = ang - M_PI;
                }
                if (ang < -M_PI / 2)
                {
                    ang = ang + M_PI;
                }
            }
            ang *= sign;
            Pnew = Pnew.Rotated(Norm, ang);
        }
    }

    private static void BuildConnectedEdges(in TopoDS_Wire aWire,
                                            in TopoDS_Edge StartEdge,
                                            in TopoDS_Vertex StartVertex,
                                            NCollection_List<TopoDS_Shape> ConnectedEdges)
    {
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> MapVE = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(aWire, TopAbs_VERTEX, TopAbs_EDGE, MapVE);
        TopoDS_Edge CurEdge = StartEdge;
        TopoDS_Vertex CurVertex = StartVertex;
        TopoDS_Vertex Origin;
        TopExp.Vertices(StartEdge, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
        Origin = (V1.IsSame(StartVertex)) ? V2 : V1;

        for (;;)
        {
            NCollection_List<TopoDS_Shape>.Iterator itE = new(MapVE.FindFromKey(CurVertex));
            for (; itE.More(); itE.Next())
            {
                TopoDS_Edge anEdge = TopoDS.Edge(itE.Value());
                if (!anEdge.IsSame(CurEdge))
                {
                    ConnectedEdges.Append(anEdge);
                    TopExp.Vertices(anEdge, out V1, out V2);
                    CurVertex = (V1.IsSame(CurVertex)) ? V2 : V1;
                    CurEdge = anEdge;
                    break;
                }
            }
            if (CurVertex.IsSame(Origin))
            {
                break;
            }
        }
    }

    //=================================================================================================

    /// <summary>An empty algorithm; <see cref="Init"/> gives it the sections.</summary>
    public BRepFill_CompatibleWires()
    {
        myStatus = BRepFill_ThruSectionErrorStatus_NotDone;
    }

    //=================================================================================================

    /// <summary>The algorithm on the sections <paramref name="Sections"/> (wires, in order).</summary>
    public BRepFill_CompatibleWires(NCollection_Sequence<TopoDS_Shape> Sections)
    {
        Init(Sections);
    }

    //=================================================================================================

    /// <summary>Sets the sections (wires, in order) and resets the algorithm.</summary>
    public void Init(NCollection_Sequence<TopoDS_Shape> Sections)
    {
        myInit = new NCollection_Sequence<TopoDS_Shape>(Sections);
        myWork = new NCollection_Sequence<TopoDS_Shape>(Sections);
        myPercent = 0.1;
        myStatus = BRepFill_ThruSectionErrorStatus_NotDone;
        myMap.Clear();
    }

    //=================================================================================================

    /// <summary>
    /// Sets the fraction of an edge's range within which an intersection is taken as the edge's vertex rather than a
    /// cut (only values strictly between 0 and 1 are taken; 0.1 by default).
    /// </summary>
    public void SetPercent(double Percent = 0.01)
    {
        if (0.0 < Percent && Percent < 1.0)
        {
            myPercent = Percent;
        }
    }

    //=================================================================================================

    /// <summary>True if <see cref="Perform"/> succeeded.</summary>
    public bool IsDone()
    {
        return myStatus == BRepFill_ThruSectionErrorStatus_Done;
    }

    /// <summary>The status of the last <see cref="Perform"/>.</summary>
    public BRepFill_ThruSectionErrorStatus GetStatus() => myStatus;

    //=================================================================================================

    /// <summary>The compatible wires, one per section, frozen.</summary>
    public ISchematiqList<TopoDS_Shape> Shape()
    {
        var aShapes = new TopoDS_Shape[myWork.Length()];
        for (int i = 1; i <= myWork.Length(); i++)
        {
            aShapes[i - 1] = myWork.Value(i);
            aShapes[i - 1].TShape()?.FreezeGraph();
        }
        return PackedList.Own(aShapes);
    }

    /// <summary>The algorithm's own sequence of wires, not frozen, for ported callers.</summary>
    internal NCollection_Sequence<TopoDS_Shape> InternalShape()
    {
        return myWork;
    }

    //=================================================================================================

    /// <summary>The edges made from the edge <paramref name="SubSection"/> of a section, frozen (empty if none).</summary>
    public ISchematiqList<TopoDS_Shape> GeneratedShapes(in TopoDS_Edge SubSection) => published(InternalGeneratedShapes(SubSection));

    private static readonly NCollection_List<TopoDS_Shape> theEmpty = new();

    /// <summary>The algorithm's own list of the edges made from <paramref name="SubSection"/>, for ported callers.</summary>
    internal NCollection_List<TopoDS_Shape> InternalGeneratedShapes(in TopoDS_Edge SubSection)
    {

        if (myMap.IsBound(SubSection))
        {
            return myMap.Find(SubSection);
        }
        else
        {
            return theEmpty;
        }
    }

    /// <summary>
    /// Each edge of the sections with the edges made from it (itself when it wasn't cut), in the order of the
    /// sections and their edges, frozen.
    /// </summary>
    public ISchematiqList<KeyValuePair<TopoDS_Shape, ISchematiqList<TopoDS_Shape>>> Generated()
    {
        var aPairs = new KeyValuePair<TopoDS_Shape, ISchematiqList<TopoDS_Shape>>[myMap.Extent()];
        int i = 0;
        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anIt in myMap)
        {
            TopoDS_Shape aKey = anIt.Key;
            aKey.TShape()?.FreezeGraph();
            aPairs[i++] = new KeyValuePair<TopoDS_Shape, ISchematiqList<TopoDS_Shape>>(aKey, published(anIt.Value));
        }
        return PackedList.Own(aPairs);
    }

    /// <summary>The algorithm's own map from the sections' edges to the edges made from them, for ported callers.</summary>
    internal NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> InternalGenerated()
    {
        return myMap;
    }

    private static ISchematiqList<TopoDS_Shape> published(NCollection_List<TopoDS_Shape> theList)
    {
        if (theList.IsEmpty())
        {
            return PackedList<TopoDS_Shape>.Empty;
        }
        var aShapes = new TopoDS_Shape[theList.Extent()];
        int i = 0;
        for (NCollection_List<TopoDS_Shape>.Iterator anIt = new(theList); anIt.More(); anIt.Next())
        {
            aShapes[i] = anIt.Value();
            aShapes[i].TShape()?.FreezeGraph();
            i++;
        }
        return PackedList.Own(aShapes);
    }

    //=================================================================================================

    /// <summary>True if the first section is a point (a wire of degenerated edges).</summary>
    public bool IsDegeneratedFirstSection()
    {
        return myDegen1;
    }

    //=================================================================================================

    /// <summary>True if the last section is a point (a wire of degenerated edges).</summary>
    public bool IsDegeneratedLastSection()
    {
        return myDegen2;
    }

    //=================================================================================================

    /// <summary>
    /// Makes the wires compatible: same number of edges, origins and orientations chosen to avoid twisting
    /// (<paramref name="WithRotation"/> lets the polar method follow the rotation between the sections' planes).
    /// </summary>
    public void Perform(bool WithRotation = true)
    {
        myStatus = BRepFill_ThruSectionErrorStatus_Done;
        // compute origin and orientation on wires to avoid twisted results
        // and update wires to have same number of edges

        // determination of report:
        // if the number of elements is the same and if the wires have discontinuities
        // by tangency, the report is not carried out by curvilinear abscissa
        int nbSects = myWork.Length(), i;
        BRepTools_WireExplorer anExp = new();
        int nbmax = 0, nbmin = 0;
        NCollection_Array1<int> nbEdges = new(1, nbSects);
        bool report;
        GeomAbs_Shape contS = GeomAbs_CN;
        GeomAbs_Shape cont;
        for (i = 1; i <= nbSects; i++)
        {
            TopoDS_Shape aLocalShape = myWork[i].Oriented(TopAbs_FORWARD);
            myWork[i] = TopoDS.Wire(aLocalShape);
            //    myWork(i) = TopoDS::Wire(myWork(i).Oriented(TopAbs_FORWARD));
            TopoDS_Wire W = TopoDS.Wire(myWork[i]);
            WireContinuity(W, out cont);
            if (cont < contS)
            {
                contS = cont;
            }
            nbEdges[i] = 0;
            for (anExp.Init(W); anExp.More(); anExp.Next())
            {
                nbEdges[i]++;
            }
            if (i == 1)
            {
                nbmin = nbEdges[i];
            }
            if (nbmax < nbEdges[i])
            {
                nbmax = nbEdges[i];
            }
            if (nbmin > nbEdges[i])
            {
                nbmin = nbEdges[i];
            }
        }
        // if the number of elements is not the same or if all wires are at least
        // C1, the report is carried out by curvilinear abscissa of cuts, otherwise
        // a report vertex / Vertex is done
        report = (nbmax != nbmin || contS >= GeomAbs_C1);

        // initialization of the map
        for (i = 1; i <= nbSects; i++)
        {
            TopoDS_Wire W = TopoDS.Wire(myWork[i]);
            for (anExp.Init(W); anExp.More(); anExp.Next())
            {
                TopoDS_Edge E = TopoDS.Edge(anExp.Current());
                myMap.Bind(E, new NCollection_List<TopoDS_Shape>());
                myMap[E].Append(E);
            }
        }

        // open/closed sections
        // initialisation of myDegen1, myDegen2
        int ideb = 1, ifin = myWork.Length();
        // check if the first wire is punctual
        myDegen1 = true;
        for (anExp.Init(TopoDS.Wire(myWork[ideb])); anExp.More(); anExp.Next())
        {
            myDegen1 = myDegen1 && (BRep_Tool.Degenerated(anExp.Current()));
        }
        if (myDegen1)
        {
            ideb++;
        }
        // check if the last wire is punctual
        myDegen2 = true;
        for (anExp.Init(TopoDS.Wire(myWork[ifin])); anExp.More(); anExp.Next())
        {
            myDegen2 = myDegen2 && (BRep_Tool.Degenerated(anExp.Current()));
        }
        if (myDegen2)
        {
            ifin--;
        }

        bool wClosed, allClosed = true, allOpen = true;
        for (i = ideb; i <= ifin; i++)
        {
            wClosed = myWork[i].Closed();
            if (!wClosed)
            {
                // check if the vertices are the same.
                TopExp.Vertices(TopoDS.Wire(myWork[i]), out TopoDS_Vertex V1, out TopoDS_Vertex V2);
                if (V1.IsSame(V2))
                {
                    wClosed = true;
                }
            }
            allClosed = (allClosed && wClosed);
            allOpen = (allOpen && !wClosed);
        }

        if (allClosed)
        {
            // All sections are closed
            if (report)
            {
                // same number of elements
                SameNumberByPolarMethod(WithRotation);
            }
            else
            {
                // origin
                ComputeOrigin(false);
            }
        }
        else if (allOpen)
        {
            // All sections are open
            // origin
            SearchOrigin();
            if (myStatus != BRepFill_ThruSectionErrorStatus_Done)
            {
                return;
            }
            // same number of elements
            if (report)
            {
                SameNumberByACR(report);
            }
        }
        else
        {
            // There are open and closed sections :
            // not processed
            myStatus = BRepFill_ThruSectionErrorStatus_NotSameTopology;
            return;
        }
    }

    //=================================================================================================

    private void SameNumberByPolarMethod(bool WithRotation)
    {

        // initialisation
        int NbSects = myWork.Length();
        BRepTools_WireExplorer anExp = new();
        NCollection_DataMap<TopoDS_Shape, NCollection_Sequence<TopoDS_Shape>> EdgeNewEdges = new(TopTools_ShapeMapHasher.Instance);

        bool allClosed = true;
        int i, ii, ideb = 1, ifin = NbSects;

        for (i = 1; i <= NbSects; i++)
        {
            BRepCheck_Wire Checker = new(TopoDS.Wire(myWork[i]));
            allClosed = (allClosed && (Checker.Closed() == BRepCheck_NoError));
            // allClosed = (allClosed && myWork(i).Closed());
        }
        if (!allClosed)
        {
            myStatus = BRepFill_ThruSectionErrorStatus_NotSameTopology;
            return;
        }

        // sections ponctuelles, sections bouclantes ?
        if (myDegen1)
        {
            ideb++;
        }
        if (myDegen2)
        {
            ifin--;
        }
        bool vClosed = (!myDegen1) && (!myDegen2) && (myWork[ideb].IsSame(myWork[ifin]));

        // Removing degenerated edges
        for (i = ideb; i <= ifin; i++)
        {
            bool hasDegEdge = false;
            TopoDS_Iterator anItw = new(myWork[i]);
            for (; anItw.More(); anItw.Next())
            {
                TopoDS_Edge anEdge = TopoDS.Edge(anItw.Value());
                if (BRep_Tool.Degenerated(anEdge))
                {
                    hasDegEdge = true;
                    break;
                }
            }
            if (hasDegEdge)
            {
                TopoDS_Wire aNewWire = new();
                BRep_Builder aBBuilder = new();
                aBBuilder.MakeWire(ref aNewWire);
                for (anItw.Initialize(myWork[i]); anItw.More(); anItw.Next())
                {
                    TopoDS_Edge anEdge = TopoDS.Edge(anItw.Value());
                    if (!BRep_Tool.Degenerated(anEdge))
                    {
                        aBBuilder.Add(aNewWire, anEdge);
                    }
                }
                myWork[i] = aNewWire;
            }
        }

        // Nombre max de decoupes possibles
        int NbMaxV = 0;
        for (i = 1; i <= NbSects; i++)
        {
            for (anExp.Init(TopoDS.Wire(myWork[i])); anExp.More(); anExp.Next())
            {
                NbMaxV++;
            }
        }

        // construction of tables of planes of wires
        NCollection_HArray1<gp_Pnt> Pos = new(1, NbSects);
        NCollection_HArray1<gp_Vec> Axe = new(1, NbSects);
        for (i = ideb; i <= ifin; i++)
        {
            if (PlaneOfWire(TopoDS.Wire(myWork[i]), out gp_Pln P))
            {
                Pos.SetValue(i, P.Location());
                Axe.SetValue(i, new gp_Vec(P.Axis().Direction()));
            }
        }
        NCollection_Sequence<TopoDS_Shape> SeqV = new();
        if (myDegen1)
        {
            SeqOfVertices(TopoDS.Wire(myWork[1]), SeqV);
            Pos.SetValue(1, BRep_Tool.Pnt(TopoDS.Vertex(SeqV.Value(1))));
            Axe.SetValue(1, Axe.Value(ideb));
        }
        if (myDegen2)
        {
            SeqOfVertices(TopoDS.Wire(myWork[NbSects]), SeqV);
            Pos.SetValue(NbSects, BRep_Tool.Pnt(TopoDS.Vertex(SeqV.Value(1))));
            Axe.SetValue(NbSects, Axe.Value(ifin));
        }

        // construction of RMap, map of reports of wire i to wire i-1
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> RMap = new(TopTools_ShapeMapHasher.Instance);
        RMap.Clear();

        // loop on i
        for (i = ifin; i > ideb; i--)
        {

            TopoDS_Wire wire1 = TopoDS.Wire(myWork[i]);

            // sequence of vertices of the first wire
            SeqOfVertices(wire1, SeqV);
            if (SeqV.Length() > NbMaxV)
            {
                myStatus = BRepFill_ThruSectionErrorStatus_Failed;
                return;
            }

            // loop on vertices of wire1
            for (ii = 1; ii <= SeqV.Length(); ii++)
            {

                TopoDS_Vertex Vi = TopoDS.Vertex(SeqV.Value(ii));

                // init of RMap for Vi
                NCollection_List<TopoDS_Shape> Init = new();
                Init.Clear();
                RMap.Bind(Vi, Init);

                // it is required to find intersection Vi - wire2
                gp_Pnt Pi = BRep_Tool.Pnt(Vi);

                // return Pi in the current plane
                Transform(WithRotation,
                          Pi,
                          Pos.Value(i),
                          Axe.Value(i),
                          Pos.Value(i - 1),
                          Axe.Value(i - 1),
                          out gp_Pnt Pnew);

                // calculate the intersection
                bool NewVertex;
                TopoDS_Vertex Vsol = new();
                TopoDS_Wire newwire = new();
                if (Pnew.Distance(Pos.Value(i - 1)) > Precision.Confusion())
                {
                    double percent = myPercent;
                    NewVertex = EdgeIntersectOnWire(Pos.Value(i - 1),
                                                    Pnew,
                                                    percent,
                                                    RMap,
                                                    TopoDS.Wire(myWork[i - 1]),
                                                    ref Vsol,
                                                    ref newwire,
                                                    EdgeNewEdges);
                    if (NewVertex)
                    {
                        myWork[i - 1] = newwire;
                    }
                    RMap[Vi].Append(Vsol);
                }

            } // loop on  ii
        } // loop on  i

        // initialisation of MapVLV, map of correspondences vertex - list of vertices
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> MapVLV = new(TopTools_ShapeMapHasher.Instance);
        SeqOfVertices(TopoDS.Wire(myWork[ideb]), SeqV);
        int SizeMap = SeqV.Length();
        MapVLV.Clear();
        for (ii = 1; ii <= SizeMap; ii++)
        {
            TopoDS_Vertex Vi = TopoDS.Vertex(SeqV.Value(ii));
            NCollection_List<TopoDS_Shape> Init = new();
            Init.Clear();
            Init.Append(Vi);
            MapVLV.Bind(Vi, Init);
            int NbV = 1;
            TopoDS_Vertex V0;
            V0 = Vi;
            bool tantque = SearchRoot(V0, RMap, out TopoDS_Vertex V1);
            while (tantque)
            {
                MapVLV[Vi].Append(V1);
                NbV++;
                // test on NbV required for looping sections
                if (V1.IsSame(Vi) || NbV >= myWork.Length())
                {
                    tantque = false;
                }
                else
                {
                    V0 = V1;
                    tantque = SearchRoot(V0, RMap, out V1);
                }
            }
        }

        // loop on i
        for (i = ideb; i < ifin; i++)
        {

            TopoDS_Wire wire1 = TopoDS.Wire(myWork[i]);

            // sequence of vertices of the first wire
            SeqOfVertices(wire1, SeqV);
            if (SeqV.Length() > NbMaxV || SeqV.Length() > SizeMap)
            {
                myStatus = BRepFill_ThruSectionErrorStatus_Failed;
                return;
            }

            // next wire
            TopoDS_Wire wire2 = TopoDS.Wire(myWork[i + 1]);

            // loop on vertices of wire1
            for (ii = 1; ii <= SeqV.Length(); ii++)
            {

                TopoDS_Vertex Vi = TopoDS.Vertex(SeqV.Value(ii));
                bool intersect = true;
                if (SearchRoot(Vi, MapVLV, out TopoDS_Vertex VRoot))
                {
                    NCollection_List<TopoDS_Shape> LVi = MapVLV.Find(VRoot);
                    intersect = (!SearchVertex(LVi, wire2, out _));
                }

                if (intersect)
                {
                    // it is necessary to find intersection Vi - wire2
                    gp_Pnt Pi = BRep_Tool.Pnt(Vi);

                    // return Pi in the current plane
                    Transform(WithRotation,
                              Pi,
                              Pos.Value(i),
                              Axe.Value(i),
                              Pos.Value(i + 1),
                              Axe.Value(i + 1),
                              out gp_Pnt Pnew);

                    // calculate the intersection
                    bool NewVertex;
                    TopoDS_Vertex Vsol = new();
                    TopoDS_Wire newwire = new();
                    if (Pnew.Distance(Pos.Value(i + 1)) > Precision.Confusion())
                    {
                        double percent = myPercent;
                        NewVertex = EdgeIntersectOnWire(Pos.Value(i + 1),
                                                        Pnew,
                                                        percent,
                                                        MapVLV,
                                                        TopoDS.Wire(myWork[i + 1]),
                                                        ref Vsol,
                                                        ref newwire,
                                                        EdgeNewEdges);
                        // VRoot is null when SearchRoot found nothing: the map then has no such key and both OCCT's
                        // operator() and the port's indexer throw Standard_NoSuchObject.
                        MapVLV[VRoot].Append(Vsol);
                        if (NewVertex)
                        {
                            myWork[i + 1] = newwire;
                        }
                    }
                }
            } // loop on ii
        } // loop on i

        // regularize wires following MapVLV
        TopoDS_Wire wire = TopoDS.Wire(myWork[ideb]);

        // except for the last if the sections loop
        int ibout = ifin;
        if (vClosed)
        {
            ibout--;
        }

        for (i = ideb + 1; i <= ibout; i++)
        {

            BRepLib_MakeWire MW = new();

            anExp.Init(wire);
            TopoDS_Edge ECur = anExp.Current();
            TopExp.Vertices(ECur, out TopoDS_Vertex VF, out TopoDS_Vertex VL, true);
            double U1 = BRep_Tool.Parameter(VF, ECur);
            double U2 = BRep_Tool.Parameter(VL, ECur);
            BRepAdaptor_Curve Curve = new(ECur);
            gp_Pnt PPs = Curve.Value(0.1 * (U1 + 9 * U2));
            NCollection_List<TopoDS_Shape>.Iterator itF = new(MapVLV.Find(VF)), itL = new(MapVLV.Find(VL));
            int rang = ideb;
            while (rang < i && itF.More() && itL.More())
            {
                itF.Next();
                itL.Next();
                rang++;
            }
            if (!itF.More() || !itL.More())
            {
                // Correspondence chain shorter than the section index; fail gracefully.
                myStatus = BRepFill_ThruSectionErrorStatus_Failed;
                return;
            }
            TopoDS_Vertex V1 = TopoDS.Vertex(itF.Value()), V2 = TopoDS.Vertex(itL.Value());
            TopoDS_Edge Esol = new();
            double scalmax = 0.0;
            TopoDS_Iterator itW = new(myWork[i]);

            for (; itW.More(); itW.Next())
            {
                TopoDS_Edge E = TopoDS.Edge(itW.Value());
                TopExp.Vertices(E, out TopoDS_Vertex VVF, out TopoDS_Vertex VVL, true);

                // parse candidate edges
                double scal1, scal2;
                if ((V1.IsSame(VVF) && V2.IsSame(VVL)) || (V2.IsSame(VVF) && V1.IsSame(VVL)))
                {
                    double U1param = BRep_Tool.Parameter(VVF, E);
                    double U2param = BRep_Tool.Parameter(VVL, E);
                    BRepAdaptor_Curve CurveE = new(E);
                    gp_Pnt PP1 = CurveE.Value(0.1 * (U1param + 9 * U2param));
                    gp_Pnt PP2 = CurveE.Value(0.1 * (9 * U1param + U2param));

                    for (rang = i; rang > ideb; rang--)
                    {
                        Transform(WithRotation,
                                  PP1,
                                  Pos.Value(rang),
                                  Axe.Value(rang),
                                  Pos.Value(rang - 1),
                                  Axe.Value(rang - 1),
                                  out PP1);
                        Transform(WithRotation,
                                  PP2,
                                  Pos.Value(rang),
                                  Axe.Value(rang),
                                  Pos.Value(rang - 1),
                                  Axe.Value(rang - 1),
                                  out PP2);
                    }
                    gp_Vec Ns = new(Pos.Value(ideb), PPs);
                    Ns = Ns.Normalized();
                    gp_Vec N1 = new(Pos.Value(ideb), PP1);
                    N1 = N1.Normalized();
                    gp_Vec N2 = new(Pos.Value(ideb), PP2);
                    N2 = N2.Normalized();
                    scal1 = N1.Dot(Ns);
                    if (scal1 > scalmax)
                    {
                        scalmax = scal1;
                        Esol = E;
                    }
                    scal2 = N2.Dot(Ns);
                    if (scal2 > scalmax)
                    {
                        scalmax = scal2;
                        TopoDS_Shape aLocalShape = E.Reversed();
                        Esol = TopoDS.Edge(aLocalShape);
                    }
                }
            } // end of for(; itW.More(); itW.Next())
            if (Esol.IsNull())
            {
                myStatus = BRepFill_ThruSectionErrorStatus_ProfilesInconsistent;
                return;
            }
            MW.Add(Esol);

            NCollection_List<TopoDS_Shape> ConnectedEdges = new();
            BuildConnectedEdges(TopoDS.Wire(myWork[i]), Esol, V2, ConnectedEdges);

            NCollection_List<TopoDS_Shape>.Iterator itCE = new(ConnectedEdges);
            for (; anExp.More() && itCE.More(); anExp.Next(), itCE.Next())
            {
                ECur = anExp.Current();
                TopExp.Vertices(ECur, out VF, out VL, true);
                U1 = BRep_Tool.Parameter(VF, ECur);
                U2 = BRep_Tool.Parameter(VL, ECur);
                Curve.Initialize(ECur);
                PPs = Curve.Value(0.1 * (U1 + 9 * U2));

                TopoDS_Edge E = TopoDS.Edge(itCE.Value());
                TopExp.Vertices(E, out TopoDS_Vertex VVF, out TopoDS_Vertex VVL, true);

                // parse candidate edges
                double scal1, scal2;
                U1 = BRep_Tool.Parameter(VVF, E);
                U2 = BRep_Tool.Parameter(VVL, E);
                Curve.Initialize(E);
                gp_Pnt PP1 = Curve.Value(0.1 * (U1 + 9 * U2));
                gp_Pnt PP2 = Curve.Value(0.1 * (9 * U1 + U2));

                for (rang = i; rang > ideb; rang--)
                {
                    Transform(WithRotation,
                              PP1,
                              Pos.Value(rang),
                              Axe.Value(rang),
                              Pos.Value(rang - 1),
                              Axe.Value(rang - 1),
                              out PP1);
                    Transform(WithRotation,
                              PP2,
                              Pos.Value(rang),
                              Axe.Value(rang),
                              Pos.Value(rang - 1),
                              Axe.Value(rang - 1),
                              out PP2);
                }
                gp_Vec Ns = new(Pos.Value(ideb), PPs);
                Ns = Ns.Normalized();
                gp_Vec N1 = new(Pos.Value(ideb), PP1);
                N1 = N1.Normalized();
                gp_Vec N2 = new(Pos.Value(ideb), PP2);
                N2 = N2.Normalized();
                scal1 = N1.Dot(Ns);
                scal2 = N2.Dot(Ns);
                if (scal2 > scal1)
                {
                    E.Reverse();
                }
                MW.Add(E);
            }
            myWork[i] = MW.InternalWire();
        }

        // blocking sections?
        if (vClosed)
        {
            TopoDS_Iterator iter1 = new(myWork[myWork.Length()]), iter2 = new(myWork[1]);
            for (; iter1.More(); iter1.Next(), iter2.Next())
            {
                TopoDS_Shape anEdge = iter1.Value();
                TopoDS_Shape aNewEdge = iter2.Value();
                if (!anEdge.IsSame(aNewEdge))
                {
                    NCollection_Sequence<TopoDS_Shape> aSeq = new();
                    aSeq.Append(aNewEdge);
                    EdgeNewEdges.Bind(anEdge, aSeq);
                }
            }
            myWork[myWork.Length()] = myWork[1];
        }

        // check the number of edges for debug
        int nbmax = 0, nbmin = 0;
        for (i = ideb; i <= ifin; i++)
        {
            int nbEdges = 0;
            for (anExp.Init(TopoDS.Wire(myWork[i])); anExp.More(); anExp.Next())
            {
                nbEdges++;
            }
            if (i == ideb)
            {
                nbmin = nbEdges;
            }
            if (nbmax < nbEdges)
            {
                nbmax = nbEdges;
            }
            if (nbmin > nbEdges)
            {
                nbmin = nbEdges;
            }
        }
        if (nbmin != nbmax)
        {
            myStatus = BRepFill_ThruSectionErrorStatus_Failed;
            return;
        }

        // Fill <myMap>
        foreach (TopoDS_Shape anEdge in new List<TopoDS_Shape>(myMap.Keys))
        {
            NCollection_List<TopoDS_Shape> ListOfNewEdges = new();

            // for each edge of <myMap> find all newest edges
            // in <EdgeNewEdges> recursively
            AddNewEdge(anEdge, EdgeNewEdges, ListOfNewEdges);

            myMap[anEdge] = ListOfNewEdges;
        }
    }

    //=================================================================================================

    private void SameNumberByACR(bool report)
    {
        // find the dimension
        int ideb = 1, ifin = myWork.Length();
        BRepTools_WireExplorer anExp = new();

        // point sections, blocking  sections?
        if (myDegen1)
        {
            ideb++;
        }
        if (myDegen2)
        {
            ifin--;
        }
        bool vClosed = (!myDegen1) && (!myDegen2) && (myWork[ideb].IsSame(myWork[ifin]));

        int nbSects = myWork.Length(), i;
        int nbmax = 0, nbmin = 0;
        NCollection_Array1<int> nbEdges = new(1, nbSects);
        for (i = 1; i <= nbSects; i++)
        {
            nbEdges[i] = 0;
            for (anExp.Init(TopoDS.Wire(myWork[i])); anExp.More(); anExp.Next())
            {
                nbEdges[i]++;
            }
            if (i == 1)
            {
                nbmin = nbEdges[i];
            }
            if (nbmax < nbEdges[i])
            {
                nbmax = nbEdges[i];
            }
            if (nbmin > nbEdges[i])
            {
                nbmin = nbEdges[i];
            }
        }

        if (nbmax > 1)
        {
            // several edges

            if (report || nbmin < nbmax)
            {
                // insertion of cuts
                int nbdec = (nbmax - 1) * nbSects + 1;
                NCollection_Array1<double> dec = new(1, nbdec);
                dec.Init(0);
                dec[2] = 1;

                NCollection_Array1<double> WireLen = new(1, nbSects);

                // calculate the table of cuts
                int j, k, l;
                for (i = 1; i <= nbSects; i++)
                {
                    // current wire
                    TopoDS_Wire wire1 = TopoDS.Wire(myWork[i]);
                    int nbE = 0;
                    for (anExp.Init(wire1); anExp.More(); anExp.Next())
                    {
                        nbE++;
                    }
                    // length and ACR of the wire
                    NCollection_Array1<double> ACR = new(0, nbE);
                    ACR.Init(0);
                    BRepFill.ComputeACR(wire1, ACR);
                    WireLen[i] = ACR[0];
                    // insertion of ACR of the wire in the table of cuts
                    for (j = 1; j < ACR.Length() - 1; j++)
                    {
                        k = 1;
                        while (dec[k] < ACR[j])
                        {
                            k++;
                            if (k > nbdec)
                            {
                                break;
                            }
                        }
                        if (dec[k - 1] < ACR[j] && ACR[j] < dec[k])
                        {
                            for (l = nbdec - 1; l >= k; l--)
                            {
                                dec[l + 1] = dec[l];
                            }
                            dec[k] = ACR[j];
                        }
                    }
                }

                // table of cuts
                k = 1;
                while (dec[k] < 1)
                {
                    k++;
                    if (k > nbdec)
                    {
                        break;
                    }
                }
                nbdec = k - 1;
                NCollection_Array1<double> dec2 = new(1, nbdec);
                for (k = 1; k <= nbdec; k++)
                {
                    dec2[k] = dec[k];
                }

                // Check of cuts: are all the new edges long enough or not
                NCollection_Map<int> CutsToRemove = new();
                for (k = 1; k <= nbdec; k++)
                {
                    double Knot1 = dec2[k];
                    double Knot2 = (k == nbdec) ? 1.0 : dec2[k + 1];
                    bool AllLengthsNull = true; // OCCT: declared double, holding a bool
                    for (i = 1; i <= nbSects; i++)
                    {
                        double EdgeLen = (Knot2 - Knot1) * WireLen[i];
                        if (EdgeLen > Precision.Confusion())
                        {
                            AllLengthsNull = false;
                            break;
                        }
                    }
                    if (AllLengthsNull)
                    {
                        CutsToRemove.Add(k);
                    }
                }
                int NewNbDec = nbdec - CutsToRemove.Extent();
                NCollection_Array1<double> dec3 = new(1, NewNbDec);
                i = 1;
                for (k = 1; k <= nbdec; k++)
                {
                    if (!CutsToRemove.Contains(k))
                    {
                        dec3[i++] = dec2[k];
                    }
                }
                ///////////////////

                // insertion of cuts in each wire
                for (i = 1; i <= nbSects; i++)
                {
                    TopoDS_Wire oldwire = TopoDS.Wire(myWork[i]);
                    double tol = Precision.Confusion();
                    if (WireLen[i] > gp.Resolution())
                    {
                        tol /= WireLen[i];
                    }
                    TopoDS_Wire newwire = BRepFill.InternalInsertACR(oldwire, dec3, tol);
                    BRepTools_WireExplorer anExp1 = new(), anExp2 = new();
                    anExp1.Init(oldwire);
                    anExp2.Init(newwire);
                    for (; anExp1.More(); anExp1.Next())
                    {
                        TopoDS_Edge Ecur = anExp1.Current();
                        if (!Ecur.IsSame(TopoDS.Edge(anExp2.Current())))
                        {
                            NCollection_List<TopoDS_Shape> LE = new();
                            LE.Clear();
                            gp_Pnt P1 = new(), P2 = new();
                            TopoDS_Vertex V1 = anExp1.CurrentVertex();
                            TopExp.Vertices(Ecur, out TopoDS_Vertex VF, out TopoDS_Vertex VR, true);
                            if (V1.IsSame(VF))
                            {
                                P1 = BRep_Tool.Pnt(VR);
                            }
                            if (V1.IsSame(VR))
                            {
                                P1 = BRep_Tool.Pnt(VF);
                            }
                            TopoDS_Vertex V2 = anExp2.CurrentVertex();
                            TopExp.Vertices(TopoDS.Edge(anExp2.Current()), out VF, out VR, true);
                            if (V2.IsSame(VF))
                            {
                                P2 = BRep_Tool.Pnt(VR);
                            }
                            if (V2.IsSame(VR))
                            {
                                P2 = BRep_Tool.Pnt(VF);
                            }
                            while (P1.Distance(P2) > 1.0e-3)
                            {
                                LE.Append(anExp2.Current());
                                anExp2.Next();
                                V2 = anExp2.CurrentVertex();
                                TopExp.Vertices(TopoDS.Edge(anExp2.Current()), out VF, out VR, true);
                                if (V2.IsSame(VF))
                                {
                                    P2 = BRep_Tool.Pnt(VR);
                                }
                                if (V2.IsSame(VR))
                                {
                                    P2 = BRep_Tool.Pnt(VF);
                                }
                                if (P1.Distance(P2) <= 1.0e-3)
                                {
                                    LE.Append(anExp2.Current());
                                    anExp2.Next();
                                }
                            }

                            // NCollection_List<TopoDS_Shape>::Iterator itlist;
                            TopoDS_Edge Ancestor;
                            bool found = false;

                            // OCCT iterates myMap in address order; the port in binding order (decision 908). An edge
                            // is in one list only, so the order doesn't change the result.
                            foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> itmap in myMap)
                            {
                                if (found)
                                {
                                    break;
                                }
                                NCollection_List<TopoDS_Shape>.Iterator itlist = new(itmap.Value);
                                while (itlist.More() && (!found))
                                {
                                    TopoDS_Edge ECur = TopoDS.Edge(itlist.Value());

                                    if (Ecur.IsSame(ECur))
                                    {
                                        Ancestor = TopoDS.Edge(itmap.Key);
                                        found = true;
                                        // myMap(Ancestor).InsertBefore(LE, itlist): the list LE before the iterator's
                                        // item, the iterator staying on that item
                                        NCollection_List<TopoDS_Shape> aList = myMap[Ancestor];
                                        for (NCollection_List<TopoDS_Shape>.Iterator itLE = new(LE); itLE.More(); itLE.Next())
                                        {
                                            aList.InsertBefore(itLE.Value(), itlist);
                                        }
                                        aList.Remove(itlist);
                                    }
                                    if (itlist.More())
                                    {
                                        itlist.Next();
                                    }
                                }
                            }
                        }
                        else
                        {
                            anExp2.Next();
                        }
                    }
                    myWork[i] = newwire;
                }
            }
        }

        // blocking sections ?
        if (vClosed)
        {
            myWork[myWork.Length()] = myWork[1];
        }

        // check the number of edges for debug
        nbmax = 0;
        for (i = ideb; i <= ifin; i++)
        {
            nbEdges[i] = 0;
            for (anExp.Init(TopoDS.Wire(myWork[i])); anExp.More(); anExp.Next())
            {
                nbEdges[i]++;
            }
            if (i == ideb)
            {
                nbmin = nbEdges[i];
            }
            if (nbmax < nbEdges[i])
            {
                nbmax = nbEdges[i];
            }
            if (nbmin > nbEdges[i])
            {
                nbmin = nbEdges[i];
            }
        }
        if (nbmax != nbmin)
        {
            myStatus = BRepFill_ThruSectionErrorStatus_Failed;
            return;
        }
    }

    //=================================================================================================

    private void ComputeOrigin(bool polar)
    {
        _ = polar;
        // reorganize the wires respecting orientation and origin

        BRepTools_WireExplorer anExp = new();

        bool wClosed, allClosed = true;

        int NbSects = myWork.Length();
        int i, ideb = 1, ifin = NbSects;

        // point sections, blocking sections
        if (myDegen1)
        {
            ideb++;
        }
        if (myDegen2)
        {
            ifin--;
        }
        bool vClosed = (!myDegen1) && (!myDegen2) && (myWork[ideb].IsSame(myWork[ifin]));

        for (i = ideb; i <= ifin; i++)
        {
            wClosed = myWork[i].Closed();
            if (!wClosed)
            {
                // check if the vertices are the same.
                TopExp.Vertices(TopoDS.Wire(myWork[i]), out TopoDS_Vertex V1, out TopoDS_Vertex V2);
                if (V1.IsSame(V2))
                {
                    wClosed = true;
                }
            }
            allClosed = (allClosed && wClosed);
        }
        if (!allClosed)
        {
            myStatus = BRepFill_ThruSectionErrorStatus_NotSameTopology;
            return;
        }

        // Consider that all wires have same number of edges (polar==false)
        NCollection_Sequence<TopoDS_Shape> PrevSeq = new();
        NCollection_Sequence<TopoDS_Shape> PrevEseq = new();
        int theLength = 0;
        TopoDS_Wire wire = TopoDS.Wire(myWork[ideb]);
        for (anExp.Init(wire); anExp.More(); anExp.Next())
        {
            PrevSeq.Append(anExp.CurrentVertex());
            PrevEseq.Append(anExp.Current());
            theLength++;
        }

        int nbs, NbSamples = 0;
        if (theLength <= 2)
        {
            NbSamples = 4;
        }
        PlaneOfWire(TopoDS.Wire(myWork[ideb]), out gp_Pln FirstPlane);
        gp_Pnt PrevBary = FirstPlane.Location();
        gp_Vec NormalOfFirstPlane = new(FirstPlane.Axis().Direction());
        for (i = ideb + 1; i <= ifin; i++)
        {
            TopoDS_Wire aWire = TopoDS.Wire(myWork[i]);

            // Compute offset vector as current bary center projected on first plane
            // to first bary center
            PlaneOfWire(aWire, out gp_Pln CurPlane);
            gp_Pnt CurBary = CurPlane.Location();
            gp_Vec aVec = new(PrevBary, CurBary);
            gp_Vec anOffsetProj = (aVec * NormalOfFirstPlane) * NormalOfFirstPlane;
            CurBary.Translate(-anOffsetProj); // projected current bary center
            gp_Vec Offset = new(CurBary, PrevBary);

            TopoDS_Wire newwire = new();
            BRep_Builder BB = new();
            BB.MakeWire(ref newwire);

            NCollection_Sequence<TopoDS_Shape> SeqVertices = new(), SeqEdges = new();
            for (anExp.Init(aWire); anExp.More(); anExp.Next())
            {
                SeqVertices.Append(anExp.CurrentVertex());
                SeqEdges.Append(anExp.Current());
            }

            double MinSumDist = Precision.Infinite();
            int jmin = 1, j, k, n;
            bool forward = false;
            if (i == myWork.Length() && myDegen2)
            {
                // last point section
                jmin = 1;
                forward = true;
            }
            else
            {
                for (j = 1; j <= theLength; j++)
                {
                    // Forward
                    double SumDist = 0.0;
                    for (k = j, n = 1; k <= theLength; k++, n++)
                    {
                        TopoDS_Vertex Vprev = TopoDS.Vertex(PrevSeq[n]);
                        gp_Pnt Pprev = BRep_Tool.Pnt(Vprev);
                        TopoDS_Vertex V = TopoDS.Vertex(SeqVertices[k]);
                        gp_Pnt P = new(BRep_Tool.Pnt(V).XYZ() + Offset.XYZ());
                        SumDist += Pprev.Distance(P);
                        if (NbSamples > 0)
                        {
                            TopoDS_Edge PrevEdge = TopoDS.Edge(PrevEseq[n]);
                            TopoDS_Edge CurEdge = TopoDS.Edge(SeqEdges[k]);
                            BRepAdaptor_Curve PrevEcurve = new(PrevEdge);
                            BRepAdaptor_Curve Ecurve = new(CurEdge);
                            double SampleOnPrev =
                              (PrevEcurve.LastParameter() - PrevEcurve.FirstParameter()) / NbSamples;
                            double SampleOnCur = (Ecurve.LastParameter() - Ecurve.FirstParameter()) / NbSamples;
                            for (nbs = 1; nbs <= NbSamples - 1; nbs++)
                            {
                                double ParOnPrev =
                                  (PrevEdge.Orientation() == TopAbs_FORWARD)
                                    ? (PrevEcurve.FirstParameter() + nbs * SampleOnPrev)
                                    : (PrevEcurve.FirstParameter() + (NbSamples - nbs) * SampleOnPrev);
                                double ParOnCur = (CurEdge.Orientation() == TopAbs_FORWARD)
                                                    ? (Ecurve.FirstParameter() + nbs * SampleOnCur)
                                                    : (Ecurve.FirstParameter() + (NbSamples - nbs) * SampleOnCur);
                                gp_Pnt PonPrev = PrevEcurve.Value(ParOnPrev);
                                gp_Pnt PonCur = new(Ecurve.Value(ParOnCur).XYZ() + Offset.XYZ());
                                SumDist += PonPrev.Distance(PonCur);
                            }
                        }
                    }
                    for (k = 1; k < j; k++, n++)
                    {
                        TopoDS_Vertex Vprev = TopoDS.Vertex(PrevSeq[n]);
                        gp_Pnt Pprev = BRep_Tool.Pnt(Vprev);
                        TopoDS_Vertex V = TopoDS.Vertex(SeqVertices[k]);
                        gp_Pnt P = new(BRep_Tool.Pnt(V).XYZ() + Offset.XYZ());
                        SumDist += Pprev.Distance(P);
                        if (NbSamples > 0)
                        {
                            TopoDS_Edge PrevEdge = TopoDS.Edge(PrevEseq[n]);
                            TopoDS_Edge CurEdge = TopoDS.Edge(SeqEdges[k]);
                            BRepAdaptor_Curve PrevEcurve = new(PrevEdge);
                            BRepAdaptor_Curve Ecurve = new(CurEdge);
                            double SampleOnPrev =
                              (PrevEcurve.LastParameter() - PrevEcurve.FirstParameter()) / NbSamples;
                            double SampleOnCur = (Ecurve.LastParameter() - Ecurve.FirstParameter()) / NbSamples;
                            for (nbs = 1; nbs <= NbSamples - 1; nbs++)
                            {
                                double ParOnPrev =
                                  (PrevEdge.Orientation() == TopAbs_FORWARD)
                                    ? (PrevEcurve.FirstParameter() + nbs * SampleOnPrev)
                                    : (PrevEcurve.FirstParameter() + (NbSamples - nbs) * SampleOnPrev);
                                double ParOnCur = (CurEdge.Orientation() == TopAbs_FORWARD)
                                                    ? (Ecurve.FirstParameter() + nbs * SampleOnCur)
                                                    : (Ecurve.FirstParameter() + (NbSamples - nbs) * SampleOnCur);
                                gp_Pnt PonPrev = PrevEcurve.Value(ParOnPrev);
                                gp_Pnt PonCur = new(Ecurve.Value(ParOnCur).XYZ() + Offset.XYZ());
                                SumDist += PonPrev.Distance(PonCur);
                            }
                        }
                    }
                    if (SumDist < MinSumDist)
                    {
                        MinSumDist = SumDist;
                        jmin = j;
                        forward = true;
                    }

                    // Backward
                    SumDist = 0.0;
                    for (k = j, n = 1; k >= 1; k--, n++)
                    {
                        TopoDS_Vertex Vprev = TopoDS.Vertex(PrevSeq[n]);
                        gp_Pnt Pprev = BRep_Tool.Pnt(Vprev);
                        TopoDS_Vertex V = TopoDS.Vertex(SeqVertices[k]);
                        gp_Pnt P = new(BRep_Tool.Pnt(V).XYZ() + Offset.XYZ());
                        SumDist += Pprev.Distance(P);
                        if (NbSamples > 0)
                        {
                            int k_cur = k - 1;
                            if (k_cur == 0)
                            {
                                k_cur = theLength;
                            }
                            TopoDS_Edge PrevEdge = TopoDS.Edge(PrevEseq[n]);
                            TopoDS_Edge CurEdge = TopoDS.Edge(SeqEdges[k_cur]);
                            BRepAdaptor_Curve PrevEcurve = new(PrevEdge);
                            BRepAdaptor_Curve Ecurve = new(CurEdge);
                            double SampleOnPrev =
                              (PrevEcurve.LastParameter() - PrevEcurve.FirstParameter()) / NbSamples;
                            double SampleOnCur = (Ecurve.LastParameter() - Ecurve.FirstParameter()) / NbSamples;
                            for (nbs = 1; nbs <= NbSamples - 1; nbs++)
                            {
                                double ParOnPrev =
                                  (PrevEdge.Orientation() == TopAbs_FORWARD)
                                    ? (PrevEcurve.FirstParameter() + nbs * SampleOnPrev)
                                    : (PrevEcurve.FirstParameter() + (NbSamples - nbs) * SampleOnPrev);
                                double ParOnCur = (CurEdge.Orientation() == TopAbs_FORWARD)
                                                    ? (Ecurve.FirstParameter() + (NbSamples - nbs) * SampleOnCur)
                                                    : (Ecurve.FirstParameter() + nbs * SampleOnCur);
                                gp_Pnt PonPrev = PrevEcurve.Value(ParOnPrev);
                                gp_Pnt PonCur = new(Ecurve.Value(ParOnCur).XYZ() + Offset.XYZ());
                                SumDist += PonPrev.Distance(PonCur);
                            }
                        }
                    }
                    for (k = theLength; k > j; k--, n++)
                    {
                        TopoDS_Vertex Vprev = TopoDS.Vertex(PrevSeq[n]);
                        gp_Pnt Pprev = BRep_Tool.Pnt(Vprev);
                        TopoDS_Vertex V = TopoDS.Vertex(SeqVertices[k]);
                        gp_Pnt P = new(BRep_Tool.Pnt(V).XYZ() + Offset.XYZ());
                        SumDist += Pprev.Distance(P);
                        if (NbSamples > 0)
                        {
                            TopoDS_Edge PrevEdge = TopoDS.Edge(PrevEseq[n]);
                            TopoDS_Edge CurEdge = TopoDS.Edge(SeqEdges[k - 1]);
                            BRepAdaptor_Curve PrevEcurve = new(PrevEdge);
                            BRepAdaptor_Curve Ecurve = new(CurEdge);
                            double SampleOnPrev =
                              (PrevEcurve.LastParameter() - PrevEcurve.FirstParameter()) / NbSamples;
                            double SampleOnCur = (Ecurve.LastParameter() - Ecurve.FirstParameter()) / NbSamples;
                            for (nbs = 1; nbs <= NbSamples - 1; nbs++)
                            {
                                double ParOnPrev =
                                  (PrevEdge.Orientation() == TopAbs_FORWARD)
                                    ? (PrevEcurve.FirstParameter() + nbs * SampleOnPrev)
                                    : (PrevEcurve.FirstParameter() + (NbSamples - nbs) * SampleOnPrev);
                                double ParOnCur = (CurEdge.Orientation() == TopAbs_FORWARD)
                                                    ? (Ecurve.FirstParameter() + (NbSamples - nbs) * SampleOnCur)
                                                    : (Ecurve.FirstParameter() + nbs * SampleOnCur);
                                gp_Pnt PonPrev = PrevEcurve.Value(ParOnPrev);
                                gp_Pnt PonCur = new(Ecurve.Value(ParOnCur).XYZ() + Offset.XYZ());
                                SumDist += PonPrev.Distance(PonCur);
                            }
                        }
                    }
                    if (SumDist < MinSumDist)
                    {
                        MinSumDist = SumDist;
                        jmin = j;
                        forward = false;
                    }
                }
            }

            PrevSeq.Clear();
            PrevEseq.Clear();
            if (forward)
            {
                for (j = jmin; j <= theLength; j++)
                {
                    BB.Add(newwire, TopoDS.Edge(SeqEdges[j]));
                    PrevSeq.Append(SeqVertices[j]);
                    PrevEseq.Append(SeqEdges[j]);
                }
                for (j = 1; j < jmin; j++)
                {
                    BB.Add(newwire, TopoDS.Edge(SeqEdges[j]));
                    PrevSeq.Append(SeqVertices[j]);
                    PrevEseq.Append(SeqEdges[j]);
                }
            }
            else
            {
                for (j = jmin - 1; j >= 1; j--)
                {
                    TopoDS_Shape aLocalShape = SeqEdges[j].Reversed();
                    BB.Add(newwire, TopoDS.Edge(aLocalShape));
                    // PrevSeq.Append( SeqVertices(j) );
                    PrevEseq.Append(SeqEdges[j].Reversed());
                }
                for (j = theLength; j >= jmin; j--)
                {
                    TopoDS_Shape aLocalShape = SeqEdges[j].Reversed();
                    BB.Add(newwire, TopoDS.Edge(aLocalShape));
                    // PrevSeq.Append( SeqVertices(j) );
                    PrevEseq.Append(SeqEdges[j].Reversed());
                }
                for (j = jmin; j >= 1; j--)
                {
                    PrevSeq.Append(SeqVertices[j]);
                }
                for (j = theLength; j > jmin; j--)
                {
                    PrevSeq.Append(SeqVertices[j]);
                }
            }

            newwire.Closed(true);
            newwire.Orientation(TopAbs_FORWARD);
            myWork[i] = newwire;

            PrevBary = CurBary;
        }

        // blocking sections ?
        if (vClosed)
        {
            myWork[myWork.Length()] = myWork[1];
        }
    }

    //=================================================================================================

    private void SearchOrigin()
    {
        // reorganize the open wires respecting orientation and origin

        gp_Pln P0, P;

        TopoDS_Vertex Vdeb, Vfin;
        gp_Pnt Pdeb, Pfin; //,Psuiv;

        BRepTools_WireExplorer anExp = new();

        bool allOpen = true;
        int ideb = 1, ifin = myWork.Length();
        if (myDegen1)
        {
            ideb++;
        }
        if (myDegen2)
        {
            ifin--;
        }
        bool vClosed = (!myDegen1) && (!myDegen2) && (myWork[ideb].IsSame(myWork[ifin]));

        //  for (int i=ideb; i<=ifin; i++) {
        int i;
        for (i = ideb; i <= ifin; i++)
        {
            allOpen = (allOpen && !myWork[i].Closed());
        }
        if (!allOpen)
        {
            myStatus = BRepFill_ThruSectionErrorStatus_NotSameTopology;
            return;
        }

        // init

        TopoDS_Wire wire1 = TopoDS.Wire(myWork[ideb]);
        wire1.Orientation(TopAbs_FORWARD);
        TopExp.Vertices(wire1, out Vdeb, out Vfin);
        Pdeb = BRep_Tool.Pnt(Vdeb);
        Pfin = BRep_Tool.Pnt(Vfin);
        bool isline0 = (!PlaneOfWire(wire1, out P0)), isline;
        myWork[ideb] = wire1;
        // OCC86
        anExp.Init(wire1);
        TopoDS_Edge E0 = anExp.Current(), E;

        for (i = ideb + 1; i <= ifin; i++)
        {

            TopoDS_Wire wire = TopoDS.Wire(myWork[i]);
            wire.Orientation(TopAbs_FORWARD);

            NCollection_Sequence<TopoDS_Shape> SeqEdges = new();
            SeqEdges.Clear();
            int nbEdges = 0;
            // OCC86  for(anExp.Init(wire); anExp.More(); anExp.Next()) {
            for (anExp.Init(wire), E = anExp.Current(); anExp.More(); anExp.Next())
            {
                SeqEdges.Append(anExp.Current());
                nbEdges++;
            }
            TopExp.Vertices(wire, out Vdeb, out Vfin);
            isline = (!PlaneOfWire(wire, out P));

            TopoDS_Wire newwire = new();
            BRep_Builder BW = new();
            BW.MakeWire(ref newwire);
            bool parcours = true;

            if (isline0 || isline)
            {

                // particular case of straight segments
                gp_Pnt P1 = BRep_Tool.Pnt(Vdeb), P2 = BRep_Tool.Pnt(Vfin);
                double dist1, dist2;
                dist1 = Pdeb.Distance(P1) + Pfin.Distance(P2);
                dist2 = Pdeb.Distance(P2) + Pfin.Distance(P1);
                parcours = (dist2 >= dist1);
            }

            else
            {
                // OCC86
                gp_Pnt P1 = BRep_Tool.Pnt(Vdeb), P1o = Pdeb, P2 = BRep_Tool.Pnt(Vfin), P2o = Pfin;
                if (P1.IsEqual(P2, Precision.Confusion()) || P1o.IsEqual(P2o, Precision.Confusion()))
                {
                    BRepAdaptor_Curve Curve0 = new(E0), Curve = new(E);
                    Curve0.D0(Curve0.FirstParameter() + Precision.Confusion(), out P2o);
                    Curve.D0(Curve.FirstParameter() + Precision.Confusion(), out P2);
                }
                gp_Vec VDebFin0 = new(P1o, P2o), VDebFin = new(P1, P2);
                double AStraight = VDebFin0.Angle(VDebFin);
                parcours = (AStraight < M_PI / 2.0);
            }

            // reconstruction of the wire
            int rang;
            if (parcours)
            {
                for (rang = 1; rang <= nbEdges; rang++)
                {
                    TopoDS_Shape alocalshape = SeqEdges.Value(rang);
                    BW.Add(newwire, TopoDS.Edge(alocalshape));
                    //	BW.Add(newwire,TopoDS::Edge(SeqEdges.Value(rang)));
                }
            }
            else
            {
                for (rang = nbEdges; rang >= 1; rang--)
                {
                    TopoDS_Shape alocalshape = SeqEdges.Value(rang).Reversed();
                    BW.Add(newwire, TopoDS.Edge(alocalshape));
                    //	BW.Add(newwire,TopoDS::Edge(SeqEdges.Value(rang).Reversed()));
                }
            }

            // orientation of the wire
            // OCCT: newwire.Oriented(TopAbs_FORWARD) discards its result (a no-op; the new wire is FORWARD anyway)
            myWork[i] = newwire;

            // passe to the next wire
            if (parcours)
            {
                Pdeb = BRep_Tool.Pnt(Vdeb);
                Pfin = BRep_Tool.Pnt(Vfin);
            }
            else
            {
                Pfin = BRep_Tool.Pnt(Vdeb);
                Pdeb = BRep_Tool.Pnt(Vfin);
            }
            P0 = P;
            isline0 = isline;
            // OCC86
            E0 = E;
        }

        // blocking sections ?
        if (vClosed)
        {
            myWork[myWork.Length()] = myWork[1];
        }
    }
}
