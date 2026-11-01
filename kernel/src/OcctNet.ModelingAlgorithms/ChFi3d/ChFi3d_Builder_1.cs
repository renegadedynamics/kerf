// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.ChFi3d_Builder_0;
using static Occt.ChFi3d_Builder_1;
using static Occt.ChFiDS_ErrorStatus;
using static Occt.ChFiDS_State;
using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;
using static Occt.GeomAbs_SurfaceType;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;
using static Occt.TopOpeBRepDS_Kind;

namespace Occt;
public partial class ChFi3d_Builder
{
    //=================================================================================================

    /// <summary>
    /// Initializes the builder on <paramref name="S"/> with the angular tolerance <paramref name="Ta"/>; a shape
    /// holding frozen TShapes is copied first and the maps are built on the copy (decision 1087).
    /// </summary>
    protected ChFi3d_Builder(in TopoDS_Shape S, double Ta)
    {
        done = false;
        myCopyAtEntry = new BRepTools_CopyAtEntry(S);
        myShape = myCopyAtEntry.Shape();
        myDS = new TopOpeBRepDS_HDataStructure();
        myCoup = ChFi3d_Reconstruction.Create(mkbuildtool());
        myEFMap.Fill(myShape, TopAbs_EDGE, TopAbs_FACE);
        myESoMap.Fill(myShape, TopAbs_EDGE, TopAbs_SOLID);
        myEShMap.Fill(myShape, TopAbs_EDGE, TopAbs_SHELL);
        myVFMap.Fill(myShape, TopAbs_VERTEX, TopAbs_FACE);
        myVEMap.Fill(myShape, TopAbs_VERTEX, TopAbs_EDGE);
        SetParams(Ta, 1.0e-4, 1.0e-5, 1.0e-4, 1.0e-5, 1.0e-3);
        SetContinuity(GeomAbs_C1, Ta);
    }

    //=================================================================================================

    public void SetParams(double Tang, double Tesp, double T2d, double TApp3d, double TolApp2d, double Fleche)
    {
        angular = Tang;
        tolesp = Tesp;
        tol2d = T2d;
        tolapp3d = TApp3d;
        tolapp2d = TolApp2d;
        fleche = Fleche;
    }

    //=================================================================================================

    public void SetContinuity(GeomAbs_Shape InternalContinuity, double AngularTolerance)
    {
        myConti = InternalContinuity;
        tolappangle = AngularTolerance;
    }

    //=================================================================================================

    public bool IsDone()
    {
        return done;
    }

    //=================================================================================================

    /// <summary>The result (frozen: a publication point, decision 414).</summary>
    /// <exception cref="Standard_NoSuchObject">The computation is not done.</exception>
    public TopoDS_Shape Shape()
    {
        TopoDS_Shape aResult = InternalShape();
        aResult.TShape()?.FreezeGraph();
        return aResult;
    }

    /// <summary>The result, unfrozen, for ported callers (OCCT.NET).</summary>
    internal TopoDS_Shape InternalShape()
    {
        Standard_NoSuchObject_Raise_if(!done, "ChFi3d_Builder.Shape() - no result");
        return myShapeResult;
    }

    //=================================================================================================

    public int NbFaultyContours()
    {
        return badstripes.Extent();
    }

    //=================================================================================================

    public int FaultyContour(int I)
    {
        NCollection_List<ChFiDS_Stripe>.Iterator itel = new();
        int k = 0;
        ChFiDS_Stripe? st = null;
        for (itel.Initialize(badstripes); itel.More(); itel.Next())
        {
            k += 1;
            if (k == I)
            {
                st = itel.Value();
                break;
            }
        }
        if (st is null)
        {
            return 0;
        }
        k = 0;
        for (itel.Initialize(myListStripe); itel.More(); itel.Next())
        {
            k += 1;
            if (st == itel.Value())
            {
                return k;
            }
        }
        return 0;
    }

    //=================================================================================================

    public int NbComputedSurfaces(int IC)
    {
        NCollection_List<ChFiDS_Stripe>.Iterator itel = new();
        int k = 0;
        ChFiDS_Stripe? st = null;
        for (itel.Initialize(myListStripe); itel.More(); itel.Next())
        {
            k += 1;
            if (k == IC)
            {
                st = itel.Value();
                break;
            }
        }
        if (st is null)
        {
            return 0;
        }
        if (st.Spine() is null)
        {
            return 0;
        }
        NCollection_HSequence<ChFiDS_SurfData>? hd = st.SetOfSurfData();
        if (hd is null)
        {
            return 0;
        }
        return hd.Length();
    }

    //=================================================================================================

    public Geom_Surface? ComputedSurface(int IC, int IS)
    {
        NCollection_List<ChFiDS_Stripe>.Iterator itel = new();
        int k = 0;
        ChFiDS_Stripe? st = null;
        for (itel.Initialize(myListStripe); itel.More(); itel.Next())
        {
            k += 1;
            if (k == IC)
            {
                st = itel.Value();
                break;
            }
        }
        NCollection_HSequence<ChFiDS_SurfData>? hd = st.SetOfSurfData();
        int isurf = hd.Value(IS).Surf();
        return myDS.Surface(isurf).Surface();
    }

    //=================================================================================================

    public int NbFaultyVertices()
    {
        return badvertices.Extent();
    }

    //=================================================================================================

    public TopoDS_Vertex FaultyVertex(int IV)
    {
        NCollection_List<TopoDS_Shape>.Iterator it = new();
        TopoDS_Vertex V = new();
        int k = 0;
        for (it.Initialize(badvertices); it.More(); it.Next())
        {
            k += 1;
            if (k == IV)
            {
                V = TopoDS.Vertex(it.Value());
                break;
            }
        }
        return TopoDS.Vertex(FromWork(V));
    }

    //=================================================================================================

    public bool HasResult()
    {
        return hasresult;
    }

    //=================================================================================================

    /// <summary>The partial result when a vertex failed (frozen).</summary>
    /// <exception cref="Standard_NoSuchObject">There is no partial result (<see cref="HasResult"/> is false).</exception>
    public TopoDS_Shape BadShape()
    {
        Standard_NoSuchObject_Raise_if(!hasresult, "ChFi3d_Builder.BadShape() - no result");
        badShape.TShape()?.FreezeGraph();
        return badShape;
    }

    //=================================================================================================

    public ChFiDS_ErrorStatus StripeStatus(int IC)
    {
        NCollection_List<ChFiDS_Stripe>.Iterator itel = new();
        int k = 0;
        ChFiDS_Stripe? st = null;
        for (itel.Initialize(myListStripe); itel.More(); itel.Next())
        {
            k += 1;
            if (k == IC)
            {
                st = itel.Value();
                break;
            }
        }
        ChFiDS_ErrorStatus stat = st.Spine().ErrorStatus();
        return stat;
    }

    //=================================================================================================

    /// <summary>
    /// OCCT's <c>TopOpeBRepBuild_HBuilder</c>: the reconstruction (unit 7R). Internal (decision 11003): its type is
    /// OCCT.NET's seam, not an OCCT class.
    /// </summary>
    internal ChFi3d_Reconstruction Builder()
    {
        return myCoup;
    }

    //=======================================================================
    // function : ChFi3d_FaceTangency
    // purpose  : determine if the faces opposing to edges are tangent
    //           to go from opposing faces on e0 to opposing faces
    //           on e1, consider all faces starting at a common top.
    //=======================================================================

    private bool FaceTangency(in TopoDS_Edge E0, in TopoDS_Edge E1, in TopoDS_Vertex V)
    {
        NCollection_List<TopoDS_Shape>.Iterator It = new(), Jt = new();
        TopoDS_Edge Ec = new();
        int Nbf = 0;
        TopoDS_Face[] F = new TopoDS_Face[2];

        // It is checked if the connection is not on a regular edge.
        for (It.Initialize(myEFMap[E1]), Nbf = 0; It.More(); It.Next(), Nbf++)
        {
            if (Nbf > 1)
            {
                throw new Standard_ConstructionError("ChFi3d_Builder:only 2 faces");
            }
            F[Nbf] = TopoDS.Face(It.Value());
        }
        if (Nbf < 2)
        {
            return false;
        }
        //  Modified by Sergey KHROMOV - Fri Dec 21 17:44:19 2001 Begin
        // if (BRep_Tool::Continuity(E1,F[0],F[1]) != GeomAbs_C0) {
        if (ChFi3d.IsTangentFaces(E1, F[0], F[1]))
        {
            //  Modified by Sergey KHROMOV - Fri Dec 21 17:44:21 2001 End
            return false;
        }

        for (Jt.Initialize(myVEMap[V]); Jt.More(); Jt.Next())
        {
            Ec = TopoDS.Edge(Jt.Value());
            if (!Ec.IsSame(E0) && !Ec.IsSame(E1) && Ec.Orientation() != TopAbs_INTERNAL
                    && Ec.Orientation() != TopAbs_EXTERNAL && !BRep_Tool.Degenerated(Ec))
            {
                for (It.Initialize(myEFMap[Ec]), Nbf = 0; It.More(); It.Next(), Nbf++)
                {
                    if (Nbf > 1)
                    {
                        throw new Standard_ConstructionError("ChFi3d_Builder:only 2 faces");
                    }
                    F[Nbf] = TopoDS.Face(It.Value());
                }
                if (Nbf < 2)
                {
                    return false;
                }
                //  Modified by Sergey KHROMOV - Tue Dec 18 18:10:40 2001 Begin
                //    if (BRep_Tool::Continuity(Ec,F[0],F[1]) < GeomAbs_G1) {
                if (!ChFi3d.IsTangentFaces(Ec, F[0], F[1]))
                {
                    //  Modified by Sergey KHROMOV - Tue Dec 18 18:10:41 2001 End
                    return false;
                }
            }
        }
        return true;
    }

    //=======================================================================
    // function : TangentExtremity
    // purpose  : Test if 2 faces are tangent at the end of an edge
    //=======================================================================
    //=======================================================================
    // function : TangentOnVertex
    // purpose  : Test if support faces of an edge are tangent at end.
    //=======================================================================
    //=======================================================================
    // function : PerformExtremity
    // purpose  : In case if PerformElement returned BreakPoint at one or
    //           another extremity, it is attempted to refine
    //           depending on concavities between neighbour faces of the top.
    //=======================================================================

    internal void PerformExtremity(ChFiDS_Spine? Spine)
    {
        int NbG1Connections = 0;

        for (int ii = 1; ii <= 2; ii++)
        {
            TopoDS_Edge[] E = new TopoDS_Edge[3];
            TopoDS_Vertex V = new();
            ChFiDS_State sst = default;
            int iedge = 0;
            BRepAdaptor_Surface? hs1 = null, hs2 = null;
            if (ii == 1)
            {
                sst = Spine.FirstStatus();
                iedge = 1;
                V = Spine.FirstVertex();
            }
            else
            {
                sst = Spine.LastStatus();
                iedge = Spine.NbEdges();
                V = Spine.LastVertex();
            }
            // Before all it is checked if the tangency is not dead.
            E[0] = Spine.Edges(iedge);
            ConexFaces(Spine, iedge, ref hs1, ref hs2);
            if (TangentExtremity(V, E[0], hs1, hs2, angular))
            {
                Spine.SetTangencyExtremity(true, (ii == 1));
            }

            if (sst == ChFiDS_BreakPoint)
            {
                int aLocNbG1Connections = 0;
                NCollection_List<TopoDS_Shape>.Iterator It = new(); //,Jt;
                bool sommetpourri = false;
                // OCCT's default hasher compares with IsEqual (orientation counts): a closed edge enters twice
                NCollection_IndexedMap<TopoDS_Shape> EdgesOfV = new(EqualityComparer<TopoDS_Shape>.Default);
                NCollection_Map<TopoDS_Shape> Edges = new(TopTools_ShapeMapHasher.Instance);
                Edges.Add(E[0]);
                EdgesOfV.Add(E[0]);
                int IndOfE = 0;
                for (It.Initialize(myVEMap[V]); It.More(); It.Next())
                {
                    TopoDS_Edge anEdge = TopoDS.Edge(It.Value());
                    if (BRep_Tool.Degenerated(anEdge))
                    {
                        continue;
                    }
                    TopoDS_Face F1 = new(), F2 = new();
                    ChFi3d_conexfaces(anEdge, out F1, out F2, myEFMap);
                    if (!F2.IsNull() && ChFi3d.IsTangentFaces(anEdge, F1, F2, GeomAbs_G2)) // smooth edge
                    {
                        if (!F1.IsSame(F2))
                        {
                            NbG1Connections++;
                            aLocNbG1Connections++;
                        }
                        continue;
                    }

                    if (Edges.Add(anEdge))
                    {
                        EdgesOfV.Add(anEdge);
                        if (IndOfE < 2)
                        {
                            IndOfE++;
                            E[IndOfE] = anEdge;
                        }
                    }
                    else
                    {
                        TopoDS_Vertex V1 = new(), V2 = new();
                        TopExp.Vertices(anEdge, out V1, out V2);
                        if (V1.IsSame(V2)) // edge is closed - two ends of the edge in the vertex
                        {
                            int anInd = EdgesOfV.FindIndex(anEdge);
                            if (anInd == 0)
                            {
                                anInd = EdgesOfV.FindIndex(anEdge.Reversed());
                            }
                            anEdge = TopoDS.Edge(EdgesOfV[anInd]);
                            anEdge.Reverse();
                            if (EdgesOfV.Add(anEdge) != 0) // OCCT: an index, always true
                            {
                                if (IndOfE < 2)
                                {
                                    IndOfE++;
                                    E[IndOfE] = anEdge;
                                }
                            }
                        }
                    }
                }

                if (EdgesOfV.Extent() != 3)
                {
                    sommetpourri = true;
                }

                if (!sommetpourri && aLocNbG1Connections < 4)
                {
                    sst = ChFi3d_EdgeState(E, myEFMap);
                }
                if (ii == 1)
                {
                    Spine.SetFirstStatus(sst);
                }
                else
                {
                    Spine.SetLastStatus(sst);
                }
            }
        }

        if (!Spine.IsPeriodic())
        {
            NCollection_List<TopoDS_Shape>.Iterator It = new(), Jt = new();
            int nbf = 0, jf = 0;
            for (It.Initialize(myVFMap[Spine.FirstVertex()]); It.More(); It.Next())
            {
                jf++;
                int kf = 1;
                TopoDS_Shape cur = It.Value();
                for (Jt.Initialize(myVFMap[Spine.FirstVertex()]); Jt.More() && (kf < jf); Jt.Next(), kf++)
                {
                    if (cur.IsSame(Jt.Value()))
                    {
                        break;
                    }
                }
                if (kf == jf)
                {
                    nbf++;
                }
            }
            nbf -= NbG1Connections;
            if (nbf > 3)
            {
                Spine.SetFirstStatus(ChFiDS_BreakPoint);
            }
            nbf = 0;
            jf = 0;
            for (It.Initialize(myVFMap[Spine.LastVertex()]); It.More(); It.Next())
            {
                jf++;
                int kf = 1;
                TopoDS_Shape cur = It.Value();
                for (Jt.Initialize(myVFMap[Spine.LastVertex()]); Jt.More() && (kf < jf); Jt.Next(), kf++)
                {
                    if (cur.IsSame(Jt.Value()))
                    {
                        break;
                    }
                }
                if (kf == jf)
                {
                    nbf++;
                }
            }
            nbf -= NbG1Connections;
            if (nbf > 3)
            {
                Spine.SetLastStatus(ChFiDS_BreakPoint);
            }
        }
    }

    //=======================================================================
    // function : PerformElement
    // purpose  :  find all mutually tangent edges ;
    // Each edge has 2 opposing faces. For 2 adjacent tangent edges it is required that
    // the opposing faces were tangent.
    //=======================================================================

    internal bool PerformElement(ChFiDS_Spine? Spine, double Offset, in TopoDS_Face theFirstFace)
    {
        double ta = angular;
        NCollection_List<TopoDS_Shape>.Iterator It = new();
        int Nbface = 0;
        NCollection_List<TopoDS_Shape>.Iterator Jt = new();
        double Wl = 0.0, Wf = 0.0;
        bool degeneOnEc = false;
        gp_Pnt P2 = new();
        gp_Vec V1 = new(), V2 = new();
        TopoDS_Vertex Ve1 = new(), VStart = new(), FVEc = new(), LVEc = new(), FVEv = new(), LVEv = new();
        TopoDS_Edge Ev = new(), Ec = Spine.Edges(1);
        if (BRep_Tool.Degenerated(Ec))
        {
            return false;
        }
        // it is checked if the edge is a cut edge
        TopoDS_Face ff1 = new(), ff2 = new();
        ChFi3d_conexfaces(Ec, out ff1, out ff2, myEFMap);
        if (ff1.IsNull() || ff2.IsNull())
        {
            return false;
        }
        //  Modified by Sergey KHROMOV - Fri Dec 21 17:46:22 2001 End
        // if(BRep_Tool::Continuity(Ec,ff1,ff2) != GeomAbs_C0) return 0;
        if (ChFi3d.IsTangentFaces(Ec, ff1, ff2))
        {
            return false;
        }
        //  Modified by Sergey KHROMOV - Fri Dec 21 17:46:24 2001 Begin

        TopoDS_Face FirstFace = ff1;
        if (!theFirstFace.IsNull() && ff2.IsSame(theFirstFace))
        {
            FirstFace = ff2;
            ff2 = ff1;
            ff1 = FirstFace;
        }
        myEdgeFirstFace.Bind(Ec, FirstFace);

        // Define concavity
        ChFiDS_TypeOfConcavity TypeOfConcavity = ChFi3d.DefineConnectType(Ec, ff1, ff2, 1.0e-5, true);
        Spine.SetTypeOfConcavity(TypeOfConcavity);

        bool ToRestrict = Offset > 0;
        BRepAdaptor_Surface Sb1 = new(ff1, ToRestrict);
        BRepAdaptor_Surface Sb2 = new(ff2, ToRestrict);
        if (Offset > 0)
        {
            TopoDS_Edge OffsetEdge = MakeOffsetEdge(Ec, Offset, Sb1, Sb2);
            OffsetEdge.Orientation(Ec.Orientation());
            Spine.SetOffsetEdges(OffsetEdge);
        }

        BRepAdaptor_Curve CEc = new(), CEv = new();
        TopAbs_Orientation curor = Ec.Orientation();
        TopExp.Vertices(Ec, out VStart, out LVEc);

        bool Fini = false;
        int Nb = 0;
        ChFiDS_State CurSt = ChFiDS_Closed;
        if (VStart.IsSame(LVEc))
        { // case if only one edge is closed
            CEc.Initialize(Ec);
            Wl = BRep_Tool.Parameter(VStart, Ec);
            CEc.D1(Wl, out P2, out V1);
            Wl = BRep_Tool.Parameter(LVEc, Ec);
            CEc.D1(Wl, out P2, out V2);
            bool IsFaceTangency = FaceTangency(Ec, Ec, VStart);
            if (V1.IsParallel(V2, ta) || IsFaceTangency)
            {
                if (IsFaceTangency)
                {
                    CurSt = ChFiDS_Closed;
                }
                else
                {
                    CurSt = ChFiDS_BreakPoint;
                }
            }
            else
            {
                CurSt = ChFiDS_BreakPoint;
            }
            Spine.SetLastStatus(CurSt);
            Spine.SetFirstStatus(CurSt);
        }
        else
        { // Downstream progression
            FVEc = VStart;
            TopAbs_Orientation Or1 = default;
            while (!Fini)
            {
                CurSt = ChFiDS_FreeBoundary;
                Wl = BRep_Tool.Parameter(LVEc, Ec);
                degeneOnEc = TangentOnVertex(LVEc, Ec, myEFMap, ta);
                CEc.Initialize(Ec);
                CEc.D1(Wl, out P2, out V1);
                Nb = Spine.NbEdges();

                for (It.Initialize(myVEMap[LVEc]); It.More(); It.Next())
                {
                    Ev = TopoDS.Edge(It.Value());
                    if (!Ev.IsSame(Ec) && !BRep_Tool.Degenerated(Ev))
                    {
                        TopExp.Vertices(Ev, out FVEv, out LVEv);
                        if (LVEc.IsSame(LVEv))
                        {
                            Ve1 = FVEv;
                            FVEv = LVEv;
                            LVEv = Ve1;
                            Or1 = TopAbs_REVERSED;
                        }
                        else
                        {
                            Or1 = TopAbs_FORWARD;
                        }

                        Wf = BRep_Tool.Parameter(FVEv, Ev);
                        CEv.Initialize(Ev);
                        CEv.D1(Wf, out P2, out V2);
                        double av1v2 = V1.Angle(V2);
                        bool rev = (Or1 != curor);
                        bool OnAjoute = false;
                        if (FaceTangency(Ec, Ev, FVEv))
                        {
                            // there is no need of tolerance
                            // to make a decision (PRO9486) the regularity is enough.
                            // However, the absence of turn-back is checked (PRO9810)
                            OnAjoute = ((!rev && av1v2 < M_PI / 2) || (rev && av1v2 > M_PI / 2));
                            // mate attention to the single case (cf CTS21610_1)
                            if (OnAjoute && (degeneOnEc || TangentOnVertex(LVEc, Ev, myEFMap, ta)))
                            {
                                OnAjoute = ((!rev && av1v2 < ta) || (rev && (M_PI - av1v2) < ta));
                            }
                        }
                        if (OnAjoute)
                        {
                            Fini = false; // If this can be useful (Cf PRO14713)
                            TopoDS_Vertex CommonVertex = new();
                            TopExp.CommonVertex(Ec, Ev, ref CommonVertex);
                            TopoDS_Edge PrevEdge = Ec;
                            Ec = Ev;
                            //	    Ec = TopoDS::Edge(Ev);
                            Ec.Orientation(Or1);
                            Wl = Wf;
                            LVEc = LVEv;
                            Spine.SetEdges(Ec);
                            TopoDS_Face CurF1 = new(), CurF2 = new();
                            ChFi3d_conexfaces(Ec, out CurF1, out CurF2, myEFMap);
                            ReorderFaces(ref CurF1, ref CurF2, FirstFace, PrevEdge, CommonVertex, myEFMap);
                            myEdgeFirstFace.Bind(Ec, CurF1);
                            if (Offset > 0)
                            {
                                BRepAdaptor_Surface CurSb1 = new(CurF1), CurSb2 = new(CurF2);
                                TopoDS_Edge anOffsetEdge = MakeOffsetEdge(Ec, Offset, CurSb1, CurSb2);
                                anOffsetEdge.Orientation(Or1);
                                Spine.SetOffsetEdges(anOffsetEdge);
                            }
                            FirstFace = CurF1;
                            curor = Or1;
                            if (VStart.IsSame(LVEv))
                            {
                                if (FaceTangency(Ev, Spine.Edges(1), LVEv))
                                {
                                    CurSt = ChFiDS_Closed;
                                    Fini = true;
                                }
                                else
                                {
                                    CurSt = ChFiDS_BreakPoint;
                                    Fini = true;
                                }
                            }
                            break;
                        }
                        else
                        {
                            for (Jt.Initialize(myEFMap[Ev]), Nbface = 0; Jt.More(); Jt.Next(), Nbface++)
                            {
                            }
                            if (Nbface > 1)
                            {
                                CurSt = ChFiDS_BreakPoint;
                            }
                            Fini = ((!rev && av1v2 < ta) || (rev && (M_PI - av1v2) < ta));
                        }
                    }
                }
                Fini = Fini || (Nb == Spine.NbEdges());
            }
            Spine.SetLastStatus(CurSt);
            if (CurSt == ChFiDS_Closed)
            {
                Spine.SetFirstStatus(CurSt);
            }
            else
            { // Upstream progression
                Fini = false;
                Ec = Spine.Edges(1);
                FirstFace = TopoDS.Face(myEdgeFirstFace[Ec]);
                curor = Ec.Orientation();
                FVEc = VStart;
                while (!Fini)
                {
                    CurSt = ChFiDS_FreeBoundary;
                    Wl = BRep_Tool.Parameter(FVEc, Ec);
                    degeneOnEc = TangentOnVertex(FVEc, Ec, myEFMap, ta);
                    CEc.Initialize(Ec);
                    CEc.D1(Wl, out P2, out V1);
                    Nb = Spine.NbEdges();

                    for (It.Initialize(myVEMap[FVEc]); It.More(); It.Next())
                    {
                        Ev = TopoDS.Edge(It.Value());
                        if (!Ev.IsSame(Ec) && !BRep_Tool.Degenerated(Ev))
                        {
                            TopExp.Vertices(Ev, out FVEv, out LVEv);
                            if (FVEc.IsSame(FVEv))
                            {
                                Ve1 = FVEv;
                                FVEv = LVEv;
                                LVEv = Ve1;
                                Or1 = TopAbs_REVERSED;
                            }
                            else
                            {
                                Or1 = TopAbs_FORWARD;
                            }
                            Wf = BRep_Tool.Parameter(LVEv, Ev);
                            CEv.Initialize(Ev);
                            CEv.D1(Wf, out P2, out V2);
                            double av1v2 = V1.Angle(V2);
                            bool rev = (Or1 != curor);
                            bool OnAjoute = false;
                            if (FaceTangency(Ec, Ev, LVEv))
                            {
                                OnAjoute = ((!rev && av1v2 < M_PI / 2) || (rev && av1v2 > M_PI / 2));
                                if (OnAjoute && (degeneOnEc || TangentOnVertex(FVEc, Ev, myEFMap, ta)))
                                {
                                    OnAjoute = ((!rev && av1v2 < ta) || (rev && (M_PI - av1v2) < ta));
                                }
                            }
                            if (OnAjoute)
                            {
                                TopoDS_Vertex CommonVertex = new();
                                TopExp.CommonVertex(Ec, Ev, ref CommonVertex);
                                TopoDS_Edge PrevEdge = Ec;
                                Ec = Ev;
                                //	      Ec = TopoDS::Edge(Ev);
                                Ec.Orientation(Or1);
                                Wl = Wf;
                                FVEc = FVEv;
                                Spine.PutInFirst(Ec);
                                TopoDS_Face CurF1 = new(), CurF2 = new();
                                ChFi3d_conexfaces(Ec, out CurF1, out CurF2, myEFMap);
                                ReorderFaces(ref CurF1, ref CurF2, FirstFace, PrevEdge, CommonVertex, myEFMap);
                                myEdgeFirstFace.Bind(Ec, CurF1);
                                if (Offset > 0)
                                {
                                    BRepAdaptor_Surface CurSb1 = new(CurF1), CurSb2 = new(CurF2);
                                    TopoDS_Edge anOffsetEdge = MakeOffsetEdge(Ec, Offset, CurSb1, CurSb2);
                                    anOffsetEdge.Orientation(Or1);
                                    Spine.PutInFirstOffset(anOffsetEdge);
                                }
                                FirstFace = CurF1;
                                curor = Or1;
                                break;
                            }
                            else
                            {
                                for (Jt.Initialize(myEFMap[Ev]), Nbface = 0; Jt.More(); Jt.Next(), Nbface++)
                                {
                                }
                                if (Nbface > 1)
                                {
                                    CurSt = ChFiDS_BreakPoint;
                                }
                                Fini = ((!rev && av1v2 < ta) || (rev && (M_PI - av1v2) < ta));
                            }
                        }
                    }
                    Fini = Fini || (Nb == Spine.NbEdges());
                }
                Spine.SetFirstStatus(CurSt);
            }
        }
        return true;
    }

    //=================================================================================================

    public void Remove(in TopoDS_Edge theE)
    {
        TopoDS_Edge E = TopoDS.Edge(ToWork(theE));
        NCollection_List<ChFiDS_Stripe>.Iterator itel = new(myListStripe);

        for (; itel.More(); itel.Next())
        {
            ChFiDS_Spine? sp = itel.Value().Spine();
            for (int j = 1; j <= sp.NbEdges(); j++)
            {
                if (E.IsSame(sp.Edges(j)))
                {
                    myListStripe.Remove(itel);
                    return;
                }
            }
        }
    }

    //=================================================================================================

    public ChFiDS_Spine? Value(int I)
    {
        NCollection_List<ChFiDS_Stripe>.Iterator itel = new(myListStripe);
        for (int ic = 1; ic < I; ic++)
        {
            itel.Next();
        }
        return itel.Value().Spine();
    }

    //=================================================================================================

    public int NbElements()
    {
        int i = 0;
        NCollection_List<ChFiDS_Stripe>.Iterator itel = new(myListStripe);
        for (; itel.More(); itel.Next())
        {
            ChFiDS_Spine? sp = itel.Value().Spine();
            if (sp is null)
            {
                break;
            }
            i++;
        }
        return i;
    }

    //=================================================================================================

    public int Contains(in TopoDS_Edge theE)
    {
        TopoDS_Edge E = TopoDS.Edge(ToWork(theE));
        int i = 1, j;
        NCollection_List<ChFiDS_Stripe>.Iterator itel = new(myListStripe);
        for (; itel.More(); itel.Next(), i++)
        {
            ChFiDS_Spine? sp = itel.Value().Spine();
            if (sp is null)
            {
                break;
            }
            for (j = 1; j <= sp.NbEdges(); j++)
            {
                if (E.IsSame(sp.Edges(j)))
                {
                    return i;
                }
            }
        }
        return 0;
    }

    //=================================================================================================

    public int Contains(in TopoDS_Edge theE, out int IndexInSpine)
    {
        TopoDS_Edge E = TopoDS.Edge(ToWork(theE));
        int i = 1, j;
        IndexInSpine = 0;
        NCollection_List<ChFiDS_Stripe>.Iterator itel = new(myListStripe);
        for (; itel.More(); itel.Next(), i++)
        {
            ChFiDS_Spine? sp = itel.Value().Spine();
            if (sp is null)
            {
                break;
            }
            for (j = 1; j <= sp.NbEdges(); j++)
            {
                if (E.IsSame(sp.Edges(j)))
                {
                    IndexInSpine = j;
                    return i;
                }
            }
        }
        return 0;
    }

    //=================================================================================================

    public double Length(int IC)
    {
        if (IC <= NbElements())
        {
            ChFiDS_Spine? sp = Value(IC);
            return sp.LastParameter(sp.NbEdges());
        }
        return -1;
    }

    //=================================================================================================

    public TopoDS_Vertex FirstVertex(int IC)
    {
        if (IC <= NbElements())
        {
            return TopoDS.Vertex(FromWork(Value(IC).FirstVertex()));
        }
        return new TopoDS_Vertex();
    }

    //=================================================================================================

    public TopoDS_Vertex LastVertex(int IC)
    {
        if (IC <= NbElements())
        {
            return TopoDS.Vertex(FromWork(Value(IC).LastVertex()));
        }
        return new TopoDS_Vertex();
    }

    //=================================================================================================

    public double Abscissa(int IC, in TopoDS_Vertex V)
    {
        if (IC <= NbElements())
        {
            return Value(IC).Absc(TopoDS.Vertex(ToWork(V)));
        }
        return -1;
    }

    //=================================================================================================

    public double RelativeAbscissa(int IC, in TopoDS_Vertex V)
    {
        if (IC <= NbElements())
        {
            return Abscissa(IC, V) / Length(IC);
        }
        return -1;
    }

    //=================================================================================================

    public bool Closed(int IC)
    {
        if (IC <= NbElements())
        {
            return Value(IC).IsClosed();
        }
        return false;
    }

    //=================================================================================================

    public bool ClosedAndTangent(int IC)
    {
        if (IC <= NbElements())
        {
            return Value(IC).IsPeriodic();
        }
        return false;
    }

}

internal static class ChFi3d_Builder_1
{
    internal static void ReorderFaces(ref TopoDS_Face theF1, ref TopoDS_Face theF2, in TopoDS_Face theFirstFace, in TopoDS_Edge thePrevEdge, in TopoDS_Vertex theCommonVertex, ChFiDS_Map theEFmap)
    {
        if (theF1.IsSame(theFirstFace))
        {
            return;
        }
        else if (theF2.IsSame(theFirstFace))
        {
            (theF1, theF2) = (theF2, theF1);
            return;
        }

        // Loop until find <theF1> or <theF2>
        TopoDS_Edge PrevEdge = thePrevEdge;
        TopoDS_Edge CurEdge = new();
        TopoDS_Face PrevFace = theFirstFace;
        TopoDS_Face CurFace = new();
        for (;;)
        {
            NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aVertexEdgeMap =
                new(TopTools_ShapeMapHasher.Instance);
            TopExp.MapShapesAndAncestors(PrevFace, TopAbs_VERTEX, TopAbs_EDGE, aVertexEdgeMap);
            foreach (TopoDS_Shape anEdge in aVertexEdgeMap.FindFromKey(theCommonVertex))
            {
                if (anEdge.IsSame(PrevEdge))
                {
                    continue;
                }

                CurEdge = TopoDS.Edge(anEdge);
                NCollection_List<TopoDS_Shape> Flist = theEFmap.FindFromKey(CurEdge);
                CurFace = PrevFace.IsSame(TopoDS.Face(Flist.First())) ? TopoDS.Face(Flist.Last())
                                                                      : TopoDS.Face(Flist.First());
                if (CurFace.IsSame(theF1))
                {
                    return;
                }
                else if (CurFace.IsSame(theF2))
                {
                    (theF1, theF2) = (theF2, theF1);
                    return;
                }
            }

            PrevEdge = CurEdge;
            PrevFace = CurFace;
        }
    }

    internal static void ConcatCurves(NCollection_Sequence<Geom_Curve> theCurves, NCollection_Sequence<Geom_Curve> theNewCurves)
    {
        while (!theCurves.IsEmpty())
        {
            GeomConvert_CompCurveToBSplineCurve Concat = new();
            bool Success = false;
            for (int i = 1; i <= theCurves.Length(); i++)
            {
                Geom_Curve? aCurve = theCurves[i];
                Geom_BoundedCurve? aBoundedCurve = (aCurve as Geom_BoundedCurve);
                Success = Concat.Add(aBoundedCurve, 1.0e-5, true);
                if (!Success)
                {
                    Success = Concat.Add(aBoundedCurve, 1.0e-5, false);
                }
                if (Success)
                {
                    theCurves.Remove(i);
                    i--;
                }
            }
            Geom_Curve? aNewCurve = Concat.BSplineCurve();
            theNewCurves.Append(aNewCurve);
        }
    }

    internal static TopoDS_Edge MakeOffsetEdge(in TopoDS_Edge theEdge, double Distance, BRepAdaptor_Surface S1, BRepAdaptor_Surface S2)
    {
        TopoDS_Edge OffsetEdge = new();

        TopoDS_Face F1 = S1.Face();
        TopoDS_Face F2 = S2.Face();
        Geom_Surface? GS1 = BRep_Tool.Surface(F1);
        Geom_Surface? TrGS1 = new Geom_RectangularTrimmedSurface(GS1, S1.FirstUParameter(), S1.LastUParameter(),
                                                                 S1.FirstVParameter(), S1.LastVParameter());
        double Offset = -Distance;
        if (F1.Orientation() == TopAbs_REVERSED)
        {
            Offset = Distance;
        }
        Geom_OffsetSurface? MakeOffsetSurf = new Geom_OffsetSurface(TrGS1, Offset);
        Geom_Surface? OffsetTrGS1 = MakeOffsetSurf.Surface();
        if (OffsetTrGS1 is null)
        {
            OffsetTrGS1 = MakeOffsetSurf;
        }
        Geom_Surface? GS2 = BRep_Tool.Surface(F2);
        Geom_Surface? TrGS2 = new Geom_RectangularTrimmedSurface(GS2, S2.FirstUParameter(), S2.LastUParameter(),
                                                                 S2.FirstVParameter(), S2.LastVParameter());
        GeomInt_IntSS Intersector = new(OffsetTrGS1, TrGS2, Precision.Confusion());
        if (!Intersector.IsDone() || Intersector.NbLines() == 0)
        {
            return OffsetEdge;
        }

        Geom_Curve? IntCurve = Intersector.Line(1);
        gp_Pnt[] Ends = new gp_Pnt[2];
        BRepAdaptor_Curve aBAcurve = new(theEdge);
        Ends[0] = aBAcurve.Value(aBAcurve.FirstParameter());
        Ends[1] = aBAcurve.Value(aBAcurve.LastParameter());

        if (Intersector.NbLines() > 1)
        {
            NCollection_Sequence<Geom_Curve> Curves = new(), NewCurves = new();
            for (int i = 1; i <= Intersector.NbLines(); i++)
            {
                Curves.Append(Intersector.Line(i));
            }

            ConcatCurves(Curves, NewCurves);

            double MinDist = RealLast();
            int imin = 1;
            for (int i = 1; i <= NewCurves.Length(); i++)
            {
                GeomAdaptor_Curve GAcurve = new(NewCurves[i]);
                Extrema_ExtPC Projector = new(Ends[0], GAcurve);
                if (!Projector.IsDone() || Projector.NbExt() == 0)
                {
                    continue;
                }
                for (int iext = 1; iext <= Projector.NbExt(); iext++)
                {
                    double aDist = Projector.SquareDistance(iext);
                    if (aDist < MinDist)
                    {
                        MinDist = aDist;
                        imin = i;
                    }
                }
            }
            IntCurve = NewCurves[imin];
        }
        if (IntCurve is null)
        {
            return OffsetEdge;
        }
        // Projection of extremities onto <IntCurve>
        GeomAdaptor_Curve GAcurve2 = new(IntCurve); // GAcurve2 in OCCT (C# forbids reusing the name)
        double[] Params = new double[2];
        for (int ind_end = 0; ind_end < 2; ind_end++)
        {
            if (ind_end == 1 && aBAcurve.IsClosed() /*HGuide.IsPeriodic()*/ /*HGuide.IsClosed()*/)
            {
                break;
            }
            Extrema_ExtPC Projector = new(Ends[ind_end], GAcurve2);
            double[] param = new double[4], dist = new double[4];
            gp_Pnt[] Pnt = new gp_Pnt[4];
            param[1] = GAcurve2.FirstParameter();
            param[2] = GAcurve2.LastParameter();
            Projector.TrimmedSquareDistances(out dist[1], out dist[2], out Pnt[1], out Pnt[2]);
            dist[3] = RealLast();
            if (Projector.IsDone() && Projector.NbExt() > 0)
            {
                int imin1 = 1; // imin in OCCT (C# forbids reusing the name below)
                for (int i = 2; i <= Projector.NbExt(); i++)
                {
                    if (Projector.SquareDistance(i) < Projector.SquareDistance(imin1))
                    {
                        imin1 = i;
                    }
                }
                param[3] = Projector.Point(imin1).Parameter();
                dist[3] = Projector.SquareDistance(imin1);
                Pnt[3] = Projector.Point(imin1).Value();
            }

            int imin = 1;
            for (int i = 2; i <= 3; i++)
            {
                if (dist[i] < dist[imin])
                {
                    imin = i;
                }
            }

            Params[ind_end] = param[imin]; // Projector.Point(imin).Parameter();
        }
        if (aBAcurve.IsClosed() /*HGuide.IsPeriodic()*/ /*HGuide.IsClosed()*/)
        {
            Params[1] = GAcurve2.LastParameter(); // temporary
        }
        if (Params[0] > Params[1])
        {
            bool IsClosed = false;
            gp_Pnt fpnt = IntCurve.Value(IntCurve.FirstParameter());
            gp_Pnt lpnt = IntCurve.Value(IntCurve.LastParameter());
            if (fpnt.SquareDistance(lpnt) <= Precision.SquareConfusion())
            {
                IsClosed = true;
            }
            if (IsClosed)
            {
                Params[1] = IntCurve.LastParameter();
            }
            else
            {
                double NewFirstPar = IntCurve.ReversedParameter(Params[0]);
                double NewLastPar = IntCurve.ReversedParameter(Params[1]);
                IntCurve = Reversible(IntCurve);
                IntCurve.Reverse();
                Params[0] = NewFirstPar;
                Params[1] = NewLastPar;
            }
        }
        if (aBAcurve.IsClosed() /*HGuide.IsPeriodic()*/ /*HGuide.IsClosed()*/) // check the direction of closed curve
        {
            gp_Pnt aPnt = new(), anOffsetPnt = new();
            gp_Vec Tangent = new(), OffsetTangent = new();
            aBAcurve.D1(aBAcurve.FirstParameter(), out aPnt, out Tangent);
            IntCurve.D1(Params[0], out anOffsetPnt, out OffsetTangent);
            if (Tangent * OffsetTangent < 0)
            {
                IntCurve = Reversible(IntCurve);
                IntCurve.Reverse();
            }
        }

        /*
        double ParTol = 1.0e-5;
        double FirstDiff = aBAcurve.FirstParameter() - Params[0];
        double LastDiff = aBAcurve.LastParameter() - Params[1];
        if (Math.Abs(FirstDiff) > ParTol ||
                Math.Abs(LastDiff) > ParTol)
        {
            Geom_BSplineCurve? BsplCurve = (IntCurve as Geom_BSplineCurve);
            NCollection_Array1<double> aKnots = new(BsplCurve.Knots());
            BSplCLib.Reparametrize(aBAcurve.FirstParameter(), aBAcurve.LastParameter(), aKnots);
            BsplCurve.SetKnots(aKnots);
            if (aBAcurve.IsPeriodic() && !BsplCurve.IsPeriodic())
                BsplCurve.SetPeriodic();
            IntCurve = BsplCurve;
        }
        */

        OffsetEdge = new BRepLib_MakeEdge(IntCurve, Params[0], Params[1]).Edge();
        return OffsetEdge;
    }

    // OCCT reverses the intersection curve in place; a frozen one is reversed on a copy (decision 1088).
    private static Geom_Curve Reversible(Geom_Curve theCurve) =>
        theCurve.IsFrozen() ? (Geom_Curve)theCurve.Copy() : theCurve;

    internal static TopOpeBRepDS_BuildTool mkbuildtool()
    {
        TopOpeBRepTool_GeomTool GT2 = new(TopOpeBRepTool_OutCurveType.TopOpeBRepTool_BSPLINE1, true, false, false);
        TopOpeBRepDS_BuildTool BT = new(GT2);
        BT.OverWrite(false);
        BT.Translate(false);
        return BT;
    }

    internal static bool TangentExtremity(in TopoDS_Vertex V, in TopoDS_Edge E, BRepAdaptor_Surface? hs1,
                                          BRepAdaptor_Surface? hs2, /* const double t3d, */ double tang)
    {
        TopoDS_Face f1 = hs1.Face();
        TopAbs_Orientation O1 = f1.Orientation();
        f1.Orientation(TopAbs_FORWARD);
        TopoDS_Face f2 = hs2.Face();
        TopAbs_Orientation O2 = f2.Orientation();
        f2.Orientation(TopAbs_FORWARD);
        TopoDS_Edge e1 = E, e2 = E;
        e1.Orientation(TopAbs_FORWARD);
        e2.Orientation(TopAbs_FORWARD);
        if (f1.IsSame(f2) && BRep_Tool.IsClosed(e1, f1))
        {
            e2.Orientation(TopAbs_REVERSED);
        }
        double p1 = BRep_Tool.Parameter(V, e1, f1);
        double p2 = BRep_Tool.Parameter(V, e2, f2);
        double u = 0.0, v = 0.0, f = 0.0, l = 0.0, Eps = 1.0e-9;
        gp_Vec n1 = new(), n2 = new();//   gp_Pnt pt1,pt2;
        Geom2d_Curve? pc1 = BRep_Tool.CurveOnSurface(e1, f1, out f, out l);
        pc1.Value(p1).Coord(out u, out v);
        BRepLProp_SLProps theProp1 = new(hs1, u, v, 1, Eps);
        if (theProp1.IsNormalDefined())
        {
            n1.SetXYZ(theProp1.Normal().XYZ());
            if (O1 == TopAbs_REVERSED)
            {
                n1.Reverse();
            }
        }
        else
        {
            return false; // It is not known...
        }

        Geom2d_Curve? pc2 = BRep_Tool.CurveOnSurface(e2, f2, out f, out l);
        pc2.Value(p2).Coord(out u, out v);
        BRepLProp_SLProps theProp2 = new(hs2, u, v, 1, Eps);
        if (theProp2.IsNormalDefined())
        {
            n2.SetXYZ(theProp2.Normal().XYZ());
            if (O2 == TopAbs_REVERSED)
            {
                n2.Reverse();
            }
        }
        else
        {
            return false; //  It is not known...
        }

        return (n1.Angle(n2) < tang);
    }

    internal static bool TangentOnVertex(in TopoDS_Vertex V, in TopoDS_Edge E, ChFiDS_Map EFMap, double tang)
    {
        TopoDS_Face ff1 = new(), ff2 = new();
        ChFi3d_conexfaces(E, out ff1, out ff2, EFMap);
        if (ff1.IsNull() || ff2.IsNull())
        {
            return false;
        }
        BRepAdaptor_Surface? S1 = new BRepAdaptor_Surface(ff1);
        BRepAdaptor_Surface? S2 = new BRepAdaptor_Surface(ff2);
        return TangentExtremity(V, E, S1, S2, tang);
    }


}
