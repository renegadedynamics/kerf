// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Properties;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.ChFi3d_Builder_0;
using static Occt.ChFi3d_Builder_1;
using static Occt.ChFiDS_ErrorStatus;
using static Occt.ChFiDS_State;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;
using static Occt.TopOpeBRepDS_Kind;

namespace Occt;
public partial class ChFi3d_Builder
{
    //=================================================================================================

    /// <summary>
    /// Initializes the builder on <paramref name="S"/> with the angular tolerance <paramref name="Ta"/>; a shape
    /// holding frozen TShapes is copied first and the maps are built on the copy (decision 1087).
    /// </summary>
    protected ChFi3d_Builder(in Shape S, double Ta)
    {
        done = false;
        myCopyAtEntry = new CopyAtEntry(S);
        myShape = myCopyAtEntry.Shape();
        myDS = new TopOpeBRepDS_HDataStructure();
        myCoup = ChFi3d_Reconstruction.Create(mkbuildtool());
        myEFMap.Fill(myShape, ShapeType.Edge, ShapeType.Face);
        myESoMap.Fill(myShape, ShapeType.Edge, ShapeType.Solid);
        myEShMap.Fill(myShape, ShapeType.Edge, ShapeType.Shell);
        myVFMap.Fill(myShape, ShapeType.Vertex, ShapeType.Face);
        myVEMap.Fill(myShape, ShapeType.Vertex, ShapeType.Edge);
        SetParams(Ta, 1.0e-4, 1.0e-5, 1.0e-4, 1.0e-5, 1.0e-3);
        SetContinuity(C1, Ta);
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

    public void SetContinuity(Continuity InternalContinuity, double AngularTolerance)
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
    /// <exception cref="NoSuchObjectException">The computation is not done.</exception>
    public Shape Shape()
    {
        Shape aResult = InternalShape();
        aResult.TShape()?.FreezeGraph();
        return aResult;
    }

    /// <summary>The result, unfrozen, for ported callers (OCCT.NET).</summary>
    internal Shape InternalShape()
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

    public Surface? ComputedSurface(int IC, int IS)
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

    public Vertex FaultyVertex(int IV)
    {
        NCollection_List<Shape>.Iterator it = new();
        Vertex V = new();
        int k = 0;
        for (it.Initialize(badvertices); it.More(); it.Next())
        {
            k += 1;
            if (k == IV)
            {
                V = ShapeCasts.Vertex(it.Value());
                break;
            }
        }
        return ShapeCasts.Vertex(FromWork(V));
    }

    //=================================================================================================

    public bool HasResult()
    {
        return hasresult;
    }

    //=================================================================================================

    /// <summary>The partial result when a vertex failed (frozen).</summary>
    /// <exception cref="NoSuchObjectException">There is no partial result (<see cref="HasResult"/> is false).</exception>
    public Shape BadShape()
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
    /// OCCT's <c>TopOpeBRepBuild_HBuilder</c>: the reconstruction (unit 7R). Internal (decision 1137): its type is
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

    private bool FaceTangency(in Edge E0, in Edge E1, in Vertex V)
    {
        NCollection_List<Shape>.Iterator It = new(), Jt = new();
        Edge Ec = new();
        int Nbf = 0;
        Face[] F = new Face[2];

        // It is checked if the connection is not on a regular edge.
        for (It.Initialize(myEFMap[E1]), Nbf = 0; It.More(); It.Next(), Nbf++)
        {
            if (Nbf > 1)
            {
                throw new ConstructionErrorException("ChFi3d_Builder:only 2 faces");
            }
            F[Nbf] = ShapeCasts.Face(It.Value());
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
            Ec = ShapeCasts.Edge(Jt.Value());
            if (!Ec.IsSame(E0) && !Ec.IsSame(E1) && Ec.Orientation() != Internal
                    && Ec.Orientation() != External && !Tool.Degenerated(Ec))
            {
                for (It.Initialize(myEFMap[Ec]), Nbf = 0; It.More(); It.Next(), Nbf++)
                {
                    if (Nbf > 1)
                    {
                        throw new ConstructionErrorException("ChFi3d_Builder:only 2 faces");
                    }
                    F[Nbf] = ShapeCasts.Face(It.Value());
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
            Edge[] E = new Edge[3];
            Vertex V = new();
            ChFiDS_State sst = default;
            int iedge = 0;
            FaceSurfaceAdaptor? hs1 = null, hs2 = null;
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
                NCollection_List<Shape>.Iterator It = new(); //,Jt;
                bool sommetpourri = false;
                // OCCT's default hasher compares with IsEqual (orientation counts): a closed edge enters twice
                NCollection_IndexedMap<Shape> EdgesOfV = new(EqualityComparer<Shape>.Default);
                NCollection_Map<Shape> Edges = new(ShapeHasher.Instance);
                Edges.Add(E[0]);
                EdgesOfV.Add(E[0]);
                int IndOfE = 0;
                for (It.Initialize(myVEMap[V]); It.More(); It.Next())
                {
                    Edge anEdge = ShapeCasts.Edge(It.Value());
                    if (Tool.Degenerated(anEdge))
                    {
                        continue;
                    }
                    Face F1 = new(), F2 = new();
                    ChFi3d_conexfaces(anEdge, out F1, out F2, myEFMap);
                    if (!F2.IsNull && ChFi3d.IsTangentFaces(anEdge, F1, F2, G2)) // smooth edge
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
                        Vertex V1 = new(), V2 = new();
                        Exploration.Vertices(anEdge, out V1, out V2);
                        if (V1.IsSame(V2)) // edge is closed - two ends of the edge in the vertex
                        {
                            int anInd = EdgesOfV.FindIndex(anEdge);
                            if (anInd == 0)
                            {
                                anInd = EdgesOfV.FindIndex(anEdge.Reversed());
                            }
                            anEdge = ShapeCasts.Edge(EdgesOfV[anInd]);
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
            NCollection_List<Shape>.Iterator It = new(), Jt = new();
            int nbf = 0, jf = 0;
            for (It.Initialize(myVFMap[Spine.FirstVertex()]); It.More(); It.Next())
            {
                jf++;
                int kf = 1;
                Shape cur = It.Value();
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
                Shape cur = It.Value();
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

    internal bool PerformElement(ChFiDS_Spine? Spine, double Offset, in Face theFirstFace)
    {
        double ta = angular;
        NCollection_List<Shape>.Iterator It = new();
        int Nbface = 0;
        NCollection_List<Shape>.Iterator Jt = new();
        double Wl = 0.0, Wf = 0.0;
        bool degeneOnEc = false;
        Point3D P2 = new();
        Vector3D V1 = new(), V2 = new();
        Vertex Ve1 = new(), VStart = new(), FVEc = new(), LVEc = new(), FVEv = new(), LVEv = new();
        Edge Ev = new(), Ec = Spine.Edges(1);
        if (Tool.Degenerated(Ec))
        {
            return false;
        }
        // it is checked if the edge is a cut edge
        Face ff1 = new(), ff2 = new();
        ChFi3d_conexfaces(Ec, out ff1, out ff2, myEFMap);
        if (ff1.IsNull || ff2.IsNull)
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

        Face FirstFace = ff1;
        if (!theFirstFace.IsNull && ff2.IsSame(theFirstFace))
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
        FaceSurfaceAdaptor Sb1 = new(ff1, ToRestrict);
        FaceSurfaceAdaptor Sb2 = new(ff2, ToRestrict);
        if (Offset > 0)
        {
            Edge OffsetEdge = MakeOffsetEdge(Ec, Offset, Sb1, Sb2);
            OffsetEdge.Orientation(Ec.Orientation());
            Spine.SetOffsetEdges(OffsetEdge);
        }

        EdgeCurveAdaptor CEc = new(), CEv = new();
        Orientation curor = Ec.Orientation();
        Exploration.Vertices(Ec, out VStart, out LVEc);

        bool Fini = false;
        int Nb = 0;
        ChFiDS_State CurSt = ChFiDS_Closed;
        if (VStart.IsSame(LVEc))
        { // case if only one edge is closed
            CEc.Initialize(Ec);
            Wl = Tool.Parameter(VStart, Ec);
            CEc.D1(Wl, out P2, out V1);
            Wl = Tool.Parameter(LVEc, Ec);
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
            Orientation Or1 = default;
            while (!Fini)
            {
                CurSt = ChFiDS_FreeBoundary;
                Wl = Tool.Parameter(LVEc, Ec);
                degeneOnEc = TangentOnVertex(LVEc, Ec, myEFMap, ta);
                CEc.Initialize(Ec);
                CEc.D1(Wl, out P2, out V1);
                Nb = Spine.NbEdges();

                for (It.Initialize(myVEMap[LVEc]); It.More(); It.Next())
                {
                    Ev = ShapeCasts.Edge(It.Value());
                    if (!Ev.IsSame(Ec) && !Tool.Degenerated(Ev))
                    {
                        Exploration.Vertices(Ev, out FVEv, out LVEv);
                        if (LVEc.IsSame(LVEv))
                        {
                            Ve1 = FVEv;
                            FVEv = LVEv;
                            LVEv = Ve1;
                            Or1 = Reversed;
                        }
                        else
                        {
                            Or1 = Forward;
                        }

                        Wf = Tool.Parameter(FVEv, Ev);
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
                            Vertex CommonVertex = new();
                            Exploration.CommonVertex(Ec, Ev, ref CommonVertex);
                            Edge PrevEdge = Ec;
                            Ec = Ev;
                            //	    Ec = TopoDS::Edge(Ev);
                            Ec.Orientation(Or1);
                            Wl = Wf;
                            LVEc = LVEv;
                            Spine.SetEdges(Ec);
                            Face CurF1 = new(), CurF2 = new();
                            ChFi3d_conexfaces(Ec, out CurF1, out CurF2, myEFMap);
                            ReorderFaces(ref CurF1, ref CurF2, FirstFace, PrevEdge, CommonVertex, myEFMap);
                            myEdgeFirstFace.Bind(Ec, CurF1);
                            if (Offset > 0)
                            {
                                FaceSurfaceAdaptor CurSb1 = new(CurF1), CurSb2 = new(CurF2);
                                Edge anOffsetEdge = MakeOffsetEdge(Ec, Offset, CurSb1, CurSb2);
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
                FirstFace = ShapeCasts.Face(myEdgeFirstFace[Ec]);
                curor = Ec.Orientation();
                FVEc = VStart;
                while (!Fini)
                {
                    CurSt = ChFiDS_FreeBoundary;
                    Wl = Tool.Parameter(FVEc, Ec);
                    degeneOnEc = TangentOnVertex(FVEc, Ec, myEFMap, ta);
                    CEc.Initialize(Ec);
                    CEc.D1(Wl, out P2, out V1);
                    Nb = Spine.NbEdges();

                    for (It.Initialize(myVEMap[FVEc]); It.More(); It.Next())
                    {
                        Ev = ShapeCasts.Edge(It.Value());
                        if (!Ev.IsSame(Ec) && !Tool.Degenerated(Ev))
                        {
                            Exploration.Vertices(Ev, out FVEv, out LVEv);
                            if (FVEc.IsSame(FVEv))
                            {
                                Ve1 = FVEv;
                                FVEv = LVEv;
                                LVEv = Ve1;
                                Or1 = Reversed;
                            }
                            else
                            {
                                Or1 = Forward;
                            }
                            Wf = Tool.Parameter(LVEv, Ev);
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
                                Vertex CommonVertex = new();
                                Exploration.CommonVertex(Ec, Ev, ref CommonVertex);
                                Edge PrevEdge = Ec;
                                Ec = Ev;
                                //	      Ec = TopoDS::Edge(Ev);
                                Ec.Orientation(Or1);
                                Wl = Wf;
                                FVEc = FVEv;
                                Spine.PutInFirst(Ec);
                                Face CurF1 = new(), CurF2 = new();
                                ChFi3d_conexfaces(Ec, out CurF1, out CurF2, myEFMap);
                                ReorderFaces(ref CurF1, ref CurF2, FirstFace, PrevEdge, CommonVertex, myEFMap);
                                myEdgeFirstFace.Bind(Ec, CurF1);
                                if (Offset > 0)
                                {
                                    FaceSurfaceAdaptor CurSb1 = new(CurF1), CurSb2 = new(CurF2);
                                    Edge anOffsetEdge = MakeOffsetEdge(Ec, Offset, CurSb1, CurSb2);
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

    public void Remove(in Edge theE)
    {
        Edge E = ShapeCasts.Edge(ToWork(theE));
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

    public int Contains(in Edge theE)
    {
        Edge E = ShapeCasts.Edge(ToWork(theE));
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

    public int Contains(in Edge theE, out int IndexInSpine)
    {
        Edge E = ShapeCasts.Edge(ToWork(theE));
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

    public Vertex FirstVertex(int IC)
    {
        if (IC <= NbElements())
        {
            return ShapeCasts.Vertex(FromWork(Value(IC).FirstVertex()));
        }
        return new Vertex();
    }

    //=================================================================================================

    public Vertex LastVertex(int IC)
    {
        if (IC <= NbElements())
        {
            return ShapeCasts.Vertex(FromWork(Value(IC).LastVertex()));
        }
        return new Vertex();
    }

    //=================================================================================================

    public double Abscissa(int IC, in Vertex V)
    {
        if (IC <= NbElements())
        {
            return Value(IC).Absc(ShapeCasts.Vertex(ToWork(V)));
        }
        return -1;
    }

    //=================================================================================================

    public double RelativeAbscissa(int IC, in Vertex V)
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
    internal static void ReorderFaces(ref Face theF1, ref Face theF2, in Face theFirstFace, in Edge thePrevEdge, in Vertex theCommonVertex, ChFiDS_Map theEFmap)
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
        Edge PrevEdge = thePrevEdge;
        Edge CurEdge = new();
        Face PrevFace = theFirstFace;
        Face CurFace = new();
        for (;;)
        {
            NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aVertexEdgeMap =
                new(ShapeHasher.Instance);
            Exploration.MapShapesAndAncestors(PrevFace, ShapeType.Vertex, ShapeType.Edge, aVertexEdgeMap);
            foreach (Shape anEdge in aVertexEdgeMap.FindFromKey(theCommonVertex))
            {
                if (anEdge.IsSame(PrevEdge))
                {
                    continue;
                }

                CurEdge = ShapeCasts.Edge(anEdge);
                NCollection_List<Shape> Flist = theEFmap.FindFromKey(CurEdge);
                CurFace = PrevFace.IsSame(ShapeCasts.Face(Flist.First())) ? ShapeCasts.Face(Flist.Last())
                                                                      : ShapeCasts.Face(Flist.First());
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

    internal static void ConcatCurves(NCollection_Sequence<Curve3D> theCurves, NCollection_Sequence<Curve3D> theNewCurves)
    {
        while (!theCurves.IsEmpty())
        {
            CompCurveToBSplineCurve Concat = new();
            bool Success = false;
            for (int i = 1; i <= theCurves.Length(); i++)
            {
                Curve3D? aCurve = theCurves[i];
                BoundedCurve3D? aBoundedCurve = (aCurve as BoundedCurve3D);
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
            Curve3D? aNewCurve = Concat.BSplineCurve();
            theNewCurves.Append(aNewCurve);
        }
    }

    internal static Edge MakeOffsetEdge(in Edge theEdge, double Distance, FaceSurfaceAdaptor S1, FaceSurfaceAdaptor S2)
    {
        Edge OffsetEdge = new();

        Face F1 = S1.Face();
        Face F2 = S2.Face();
        Surface? GS1 = Tool.Surface(F1);
        Surface? TrGS1 = new TrimmedSurface(GS1, S1.FirstUParameter(), S1.LastUParameter(),
                                                                 S1.FirstVParameter(), S1.LastVParameter());
        double Offset = -Distance;
        if (F1.Orientation() == Reversed)
        {
            Offset = Distance;
        }
        global::Kerf.Geometry.OffsetSurface? MakeOffsetSurf = new global::Kerf.Geometry.OffsetSurface(TrGS1, Offset);
        Surface? OffsetTrGS1 = MakeOffsetSurf.Surface();
        if (OffsetTrGS1 is null)
        {
            OffsetTrGS1 = MakeOffsetSurf;
        }
        Surface? GS2 = Tool.Surface(F2);
        Surface? TrGS2 = new TrimmedSurface(GS2, S2.FirstUParameter(), S2.LastUParameter(),
                                                                 S2.FirstVParameter(), S2.LastVParameter());
        GeomInt_IntSS Intersector = new(OffsetTrGS1, TrGS2, global::Kerf.Engine.Foundation.Precision.Confusion());
        if (!Intersector.IsDone() || Intersector.NbLines() == 0)
        {
            return OffsetEdge;
        }

        Curve3D? IntCurve = Intersector.Line(1);
        Point3D[] Ends = new Point3D[2];
        EdgeCurveAdaptor aBAcurve = new(theEdge);
        Ends[0] = aBAcurve.Value(aBAcurve.FirstParameter());
        Ends[1] = aBAcurve.Value(aBAcurve.LastParameter());

        if (Intersector.NbLines() > 1)
        {
            NCollection_Sequence<Curve3D> Curves = new(), NewCurves = new();
            for (int i = 1; i <= Intersector.NbLines(); i++)
            {
                Curves.Append(Intersector.Line(i));
            }

            ConcatCurves(Curves, NewCurves);

            double MinDist = RealLast();
            int imin = 1;
            for (int i = 1; i <= NewCurves.Length(); i++)
            {
                CurveAdaptor GAcurve = new(NewCurves[i]);
                CurvePointExtrema Projector = new(Ends[0], GAcurve);
                if (!Projector.IsDone || Projector.ExtCount == 0)
                {
                    continue;
                }
                for (int iext = 1; iext <= Projector.ExtCount; iext++)
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
        CurveAdaptor GAcurve2 = new(IntCurve); // GAcurve2 in OCCT (C# forbids reusing the name)
        double[] Params = new double[2];
        for (int ind_end = 0; ind_end < 2; ind_end++)
        {
            if (ind_end == 1 && aBAcurve.IsClosed /*HGuide.IsPeriodic()*/ /*HGuide.IsClosed()*/)
            {
                break;
            }
            CurvePointExtrema Projector = new(Ends[ind_end], GAcurve2);
            double[] param = new double[4], dist = new double[4];
            Point3D[] Pnt = new Point3D[4];
            param[1] = GAcurve2.FirstParameter();
            param[2] = GAcurve2.LastParameter();
            Projector.TrimmedSquareDistances(out dist[1], out dist[2], out Pnt[1], out Pnt[2]);
            dist[3] = RealLast();
            if (Projector.IsDone && Projector.ExtCount > 0)
            {
                int imin1 = 1; // imin in OCCT (C# forbids reusing the name below)
                for (int i = 2; i <= Projector.ExtCount; i++)
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
        if (aBAcurve.IsClosed /*HGuide.IsPeriodic()*/ /*HGuide.IsClosed()*/)
        {
            Params[1] = GAcurve2.LastParameter(); // temporary
        }
        if (Params[0] > Params[1])
        {
            bool IsClosed = false;
            Point3D fpnt = IntCurve.Value(IntCurve.FirstParameter());
            Point3D lpnt = IntCurve.Value(IntCurve.LastParameter());
            if (fpnt.SquareDistance(lpnt) <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
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
        if (aBAcurve.IsClosed /*HGuide.IsPeriodic()*/ /*HGuide.IsClosed()*/) // check the direction of closed curve
        {
            Point3D aPnt = new(), anOffsetPnt = new();
            Vector3D Tangent = new(), OffsetTangent = new();
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
    private static Curve3D Reversible(Curve3D theCurve) =>
        theCurve.IsFrozen ? (Curve3D)theCurve.Copy() : theCurve;

    internal static TopOpeBRepDS_BuildTool mkbuildtool()
    {
        TopOpeBRepTool_GeomTool GT2 = new(TopOpeBRepTool_OutCurveType.TopOpeBRepTool_BSPLINE1, true, false, false);
        TopOpeBRepDS_BuildTool BT = new(GT2);
        BT.OverWrite(false);
        BT.Translate(false);
        return BT;
    }

    internal static bool TangentExtremity(in Vertex V, in Edge E, FaceSurfaceAdaptor? hs1,
                                          FaceSurfaceAdaptor? hs2, /* const double t3d, */ double tang)
    {
        Face f1 = hs1.Face();
        Orientation O1 = f1.Orientation();
        f1.Orientation(Forward);
        Face f2 = hs2.Face();
        Orientation O2 = f2.Orientation();
        f2.Orientation(Forward);
        Edge e1 = E, e2 = E;
        e1.Orientation(Forward);
        e2.Orientation(Forward);
        if (f1.IsSame(f2) && Tool.IsClosed(e1, f1))
        {
            e2.Orientation(Reversed);
        }
        double p1 = Tool.Parameter(V, e1, f1);
        double p2 = Tool.Parameter(V, e2, f2);
        double u = 0.0, v = 0.0, f = 0.0, l = 0.0, Eps = 1.0e-9;
        Vector3D n1 = new(), n2 = new();//   gp_Pnt pt1,pt2;
        Curve2D? pc1 = Tool.CurveOnSurface(e1, f1, out f, out l);
        pc1.Value(p1).Coord(out u, out v);
        SLProps theProp1 = new(hs1, u, v, 1, Eps);
        if (theProp1.IsNormalDefined)
        {
            n1.SetXYZ(theProp1.Normal().XYZ());
            if (O1 == Reversed)
            {
                n1.Reverse();
            }
        }
        else
        {
            return false; // It is not known...
        }

        Curve2D? pc2 = Tool.CurveOnSurface(e2, f2, out f, out l);
        pc2.Value(p2).Coord(out u, out v);
        SLProps theProp2 = new(hs2, u, v, 1, Eps);
        if (theProp2.IsNormalDefined)
        {
            n2.SetXYZ(theProp2.Normal().XYZ());
            if (O2 == Reversed)
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

    internal static bool TangentOnVertex(in Vertex V, in Edge E, ChFiDS_Map EFMap, double tang)
    {
        Face ff1 = new(), ff2 = new();
        ChFi3d_conexfaces(E, out ff1, out ff2, EFMap);
        if (ff1.IsNull || ff2.IsNull)
        {
            return false;
        }
        FaceSurfaceAdaptor? S1 = new FaceSurfaceAdaptor(ff1);
        FaceSurfaceAdaptor? S2 = new FaceSurfaceAdaptor(ff2);
        return TangentExtremity(V, E, S1, S2, tang);
    }


}
