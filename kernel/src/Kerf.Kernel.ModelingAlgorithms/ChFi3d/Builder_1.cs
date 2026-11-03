// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Booleans.Legacy.Tools;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Extrema;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Intersection.Surfaces;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Properties;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Fillets.Builder_0;
using static Kerf.Engine.Fillets.Builder_1;
using static Kerf.Engine.Fillets.Data.ErrorStatus;
using static Kerf.Engine.Fillets.Data.State;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;
using static Kerf.Engine.Booleans.Legacy.Data.DsKind;

namespace Kerf.Engine.Fillets;
public partial class FilletBuilderBase
{
    //=================================================================================================

    /// <summary>
    /// Initializes the builder on <paramref name="s"/> with the angular tolerance <paramref name="ta"/>; a shape
    /// holding frozen TShapes is copied first and the maps are built on the copy (decision 1087).
    /// </summary>
    protected FilletBuilderBase(in Shape s, double ta)
    {
        _done = false;
        _copyAtEntry = new CopyAtEntry(s);
        _shape = _copyAtEntry.Shape();
        _ds = new DsHDataStructure();
        _coup = Reconstruction.Create(Mkbuildtool());
        _efMap.Fill(_shape, ShapeType.Edge, ShapeType.Face);
        _eSoMap.Fill(_shape, ShapeType.Edge, ShapeType.Solid);
        _eShMap.Fill(_shape, ShapeType.Edge, ShapeType.Shell);
        _vfMap.Fill(_shape, ShapeType.Vertex, ShapeType.Face);
        _veMap.Fill(_shape, ShapeType.Vertex, ShapeType.Edge);
        SetParams(ta, 1.0e-4, 1.0e-5, 1.0e-4, 1.0e-5, 1.0e-3);
        SetContinuity(C1, ta);
    }

    //=================================================================================================

    public void SetParams(double tang, double tesp, double t2d, double tApp3d, double tolApp2d, double fleche)
    {
        _angular = tang;
        _tolesp = tesp;
        _tol2d = t2d;
        _tolapp3d = tApp3d;
        _tolapp2d = tolApp2d;
        _fleche = fleche;
    }

    //=================================================================================================

    public void SetContinuity(Continuity internalContinuity, double angularTolerance)
    {
        _conti = internalContinuity;
        _tolappangle = angularTolerance;
    }

    //=================================================================================================

    public bool IsDone
    {
        get
        {
            return _done;
        }
    }

    //=================================================================================================

    /// <summary>The result (frozen: a publication point, decision 414).</summary>
    /// <exception cref="NoSuchObjectException">The computation is not done.</exception>
    public Shape Shape()
    {
        Shape result = InternalShape();
        result.TShape()?.FreezeGraph();
        return result;
    }

    /// <summary>The result, unfrozen, for ported callers (OCCT.NET).</summary>
    internal Shape InternalShape()
    {
        Standard_NoSuchObject_Raise_if(!_done, "ChFi3d_Builder.Shape() - no result");
        return _shapeResult;
    }

    //=================================================================================================

    public int FaultyContourCount
    {
        get
        {
            return _badstripes.Extent();
        }
    }

    //=================================================================================================

    public int FaultyContour(int i)
    {
        NCollection_List<Stripe>.Iterator itel = new();
        int k = 0;
        Stripe? st = null;
        for (itel.Initialize(_badstripes); itel.More(); itel.Next())
        {
            k += 1;
            if (k == i)
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
        for (itel.Initialize(_listStripe); itel.More(); itel.Next())
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

    public int NbComputedSurfaces(int ic)
    {
        NCollection_List<Stripe>.Iterator itel = new();
        int k = 0;
        Stripe? st = null;
        for (itel.Initialize(_listStripe); itel.More(); itel.Next())
        {
            k += 1;
            if (k == ic)
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
        NCollection_HSequence<SurfData>? hd = st.SetOfSurfData();
        if (hd is null)
        {
            return 0;
        }
        return hd.Length();
    }

    //=================================================================================================

    public Surface? ComputedSurface(int ic, int IS)
    {
        NCollection_List<Stripe>.Iterator itel = new();
        int k = 0;
        Stripe? st = null;
        for (itel.Initialize(_listStripe); itel.More(); itel.Next())
        {
            k += 1;
            if (k == ic)
            {
                st = itel.Value();
                break;
            }
        }
        NCollection_HSequence<SurfData>? hd = st.SetOfSurfData();
        int isurf = hd.Value(IS).Surf();
        return _ds.Surface(isurf).Surface();
    }

    //=================================================================================================

    public int FaultyVertexCount
    {
        get
        {
            return _badvertices.Extent();
        }
    }

    //=================================================================================================

    public Vertex FaultyVertex(int iv)
    {
        NCollection_List<Shape>.Iterator it = new();
        Vertex v = new();
        int k = 0;
        for (it.Initialize(_badvertices); it.More(); it.Next())
        {
            k += 1;
            if (k == iv)
            {
                v = ShapeCasts.Vertex(it.Value());
                break;
            }
        }
        return ShapeCasts.Vertex(FromWork(v));
    }

    //=================================================================================================

    public bool HasResult
    {
        get
        {
            return _hasresult;
        }
    }

    //=================================================================================================

    /// <summary>The partial result when a vertex failed (frozen).</summary>
    /// <exception cref="NoSuchObjectException">There is no partial result (<see cref="HasResult"/> is false).</exception>
    public Shape BadShape()
    {
        Standard_NoSuchObject_Raise_if(!_hasresult, "ChFi3d_Builder.BadShape() - no result");
        _badShape.TShape()?.FreezeGraph();
        return _badShape;
    }

    //=================================================================================================

    public ErrorStatus StripeStatus(int ic)
    {
        NCollection_List<Stripe>.Iterator itel = new();
        int k = 0;
        Stripe? st = null;
        for (itel.Initialize(_listStripe); itel.More(); itel.Next())
        {
            k += 1;
            if (k == ic)
            {
                st = itel.Value();
                break;
            }
        }
        ErrorStatus stat = st.Spine().ErrorStatus();
        return stat;
    }

    //=================================================================================================

    /// <summary>
    /// OCCT's <c>TopOpeBRepBuild_HBuilder</c>: the reconstruction (unit 7R). Internal (decision 1137): its type is
    /// OCCT.NET's seam, not an OCCT class.
    /// </summary>
    internal Reconstruction Builder()
    {
        return _coup;
    }

    //=======================================================================
    // function : ChFi3d_FaceTangency
    // purpose  : determine if the faces opposing to edges are tangent
    //           to go from opposing faces on e0 to opposing faces
    //           on e1, consider all faces starting at a common top.
    //=======================================================================

    private bool FaceTangency(in Edge e0, in Edge e1, in Vertex v)
    {
        NCollection_List<Shape>.Iterator it = new(), jt = new();
        Edge ec = new();
        int nbf = 0;
        Face[] f = new Face[2];

        // It is checked if the connection is not on a regular edge.
        for (it.Initialize(_efMap[e1]), nbf = 0; it.More(); it.Next(), nbf++)
        {
            if (nbf > 1)
            {
                throw new ConstructionErrorException("ChFi3d_Builder:only 2 faces");
            }
            f[nbf] = ShapeCasts.Face(it.Value());
        }
        if (nbf < 2)
        {
            return false;
        }
        //  Modified by Sergey KHROMOV - Fri Dec 21 17:44:19 2001 Begin
        // if (BRep_Tool::Continuity(E1,F[0],F[1]) != GeomAbs_C0) {
        if (global::Kerf.Engine.Fillets.ChFi3d.IsTangentFaces(e1, f[0], f[1]))
        {
            //  Modified by Sergey KHROMOV - Fri Dec 21 17:44:21 2001 End
            return false;
        }

        for (jt.Initialize(_veMap[v]); jt.More(); jt.Next())
        {
            ec = ShapeCasts.Edge(jt.Value());
            if (!ec.IsSame(e0) && !ec.IsSame(e1) && ec.Orientation() != Internal
                    && ec.Orientation() != External && !Tool.Degenerated(ec))
            {
                for (it.Initialize(_efMap[ec]), nbf = 0; it.More(); it.Next(), nbf++)
                {
                    if (nbf > 1)
                    {
                        throw new ConstructionErrorException("ChFi3d_Builder:only 2 faces");
                    }
                    f[nbf] = ShapeCasts.Face(it.Value());
                }
                if (nbf < 2)
                {
                    return false;
                }
                //  Modified by Sergey KHROMOV - Tue Dec 18 18:10:40 2001 Begin
                //    if (BRep_Tool::Continuity(Ec,F[0],F[1]) < GeomAbs_G1) {
                if (!global::Kerf.Engine.Fillets.ChFi3d.IsTangentFaces(ec, f[0], f[1]))
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

    internal void PerformExtremity(Spine? spine)
    {
        int nbG1Connections = 0;

        for (int ii = 1; ii <= 2; ii++)
        {
            Edge[] e = new Edge[3];
            Vertex v = new();
            State sst = default;
            int iedge = 0;
            FaceSurfaceAdaptor? hs1 = null, hs2 = null;
            if (ii == 1)
            {
                sst = spine.FirstStatus();
                iedge = 1;
                v = spine.FirstVertex();
            }
            else
            {
                sst = spine.LastStatus();
                iedge = spine.EdgeCount;
                v = spine.LastVertex();
            }
            // Before all it is checked if the tangency is not dead.
            e[0] = spine.Edges(iedge);
            ConexFaces(spine, iedge, ref hs1, ref hs2);
            if (TangentExtremity(v, e[0], hs1, hs2, _angular))
            {
                spine.SetTangencyExtremity(true, (ii == 1));
            }

            if (sst == State.BreakPoint)
            {
                int locNbG1Connections = 0;
                NCollection_List<Shape>.Iterator it = new(); //,Jt;
                bool sommetpourri = false;
                // OCCT's default hasher compares with IsEqual (orientation counts): a closed edge enters twice
                NCollection_IndexedMap<Shape> edgesOfV = new(EqualityComparer<Shape>.Default);
                HashSet<Shape> edges = new(ShapeHasher.Instance);
                edges.Add(e[0]);
                edgesOfV.Add(e[0]);
                int indOfE = 0;
                for (it.Initialize(_veMap[v]); it.More(); it.Next())
                {
                    Edge edge = ShapeCasts.Edge(it.Value());
                    if (Tool.Degenerated(edge))
                    {
                        continue;
                    }
                    Face f1 = new(), f2 = new();
                    ChFi3d_conexfaces(edge, out f1, out f2, _efMap);
                    if (!f2.IsNull && global::Kerf.Engine.Fillets.ChFi3d.IsTangentFaces(edge, f1, f2, G2)) // smooth edge
                    {
                        if (!f1.IsSame(f2))
                        {
                            nbG1Connections++;
                            locNbG1Connections++;
                        }
                        continue;
                    }

                    if (edges.Add(edge))
                    {
                        edgesOfV.Add(edge);
                        if (indOfE < 2)
                        {
                            indOfE++;
                            e[indOfE] = edge;
                        }
                    }
                    else
                    {
                        Vertex v1 = new(), v2 = new();
                        Exploration.Vertices(edge, out v1, out v2);
                        if (v1.IsSame(v2)) // edge is closed - two ends of the edge in the vertex
                        {
                            int ind = edgesOfV.FindIndex(edge);
                            if (ind == 0)
                            {
                                ind = edgesOfV.FindIndex(edge.Reversed());
                            }
                            edge = ShapeCasts.Edge(edgesOfV[ind]);
                            edge.Reverse();
                            if (edgesOfV.Add(edge) != 0) // OCCT: an index, always true
                            {
                                if (indOfE < 2)
                                {
                                    indOfE++;
                                    e[indOfE] = edge;
                                }
                            }
                        }
                    }
                }

                if (edgesOfV.Extent() != 3)
                {
                    sommetpourri = true;
                }

                if (!sommetpourri && locNbG1Connections < 4)
                {
                    sst = ChFi3d_EdgeState(e, _efMap);
                }
                if (ii == 1)
                {
                    spine.SetFirstStatus(sst);
                }
                else
                {
                    spine.SetLastStatus(sst);
                }
            }
        }

        if (!spine.IsPeriodic)
        {
            NCollection_List<Shape>.Iterator it = new(), jt = new();
            int nbf = 0, jf = 0;
            for (it.Initialize(_vfMap[spine.FirstVertex()]); it.More(); it.Next())
            {
                jf++;
                int kf = 1;
                Shape cur = it.Value();
                for (jt.Initialize(_vfMap[spine.FirstVertex()]); jt.More() && (kf < jf); jt.Next(), kf++)
                {
                    if (cur.IsSame(jt.Value()))
                    {
                        break;
                    }
                }
                if (kf == jf)
                {
                    nbf++;
                }
            }
            nbf -= nbG1Connections;
            if (nbf > 3)
            {
                spine.SetFirstStatus(State.BreakPoint);
            }
            nbf = 0;
            jf = 0;
            for (it.Initialize(_vfMap[spine.LastVertex()]); it.More(); it.Next())
            {
                jf++;
                int kf = 1;
                Shape cur = it.Value();
                for (jt.Initialize(_vfMap[spine.LastVertex()]); jt.More() && (kf < jf); jt.Next(), kf++)
                {
                    if (cur.IsSame(jt.Value()))
                    {
                        break;
                    }
                }
                if (kf == jf)
                {
                    nbf++;
                }
            }
            nbf -= nbG1Connections;
            if (nbf > 3)
            {
                spine.SetLastStatus(State.BreakPoint);
            }
        }
    }

    //=======================================================================
    // function : PerformElement
    // purpose  :  find all mutually tangent edges ;
    // Each edge has 2 opposing faces. For 2 adjacent tangent edges it is required that
    // the opposing faces were tangent.
    //=======================================================================

    internal bool PerformElement(Spine? spine, double offset, in Face firstFace)
    {
        double ta = _angular;
        NCollection_List<Shape>.Iterator it = new();
        int nbface = 0;
        NCollection_List<Shape>.Iterator jt = new();
        double wl = 0.0, wf = 0.0;
        bool degeneOnEc = false;
        Point3D p2 = new();
        Vector3D v1 = new(), v2 = new();
        Vertex ve1 = new(), vStart = new(), fvEc = new(), lvEc = new(), fvEv = new(), lvEv = new();
        Edge ev = new(), ec = spine.Edges(1);
        if (Tool.Degenerated(ec))
        {
            return false;
        }
        // it is checked if the edge is a cut edge
        Face ff1 = new(), ff2 = new();
        ChFi3d_conexfaces(ec, out ff1, out ff2, _efMap);
        if (ff1.IsNull || ff2.IsNull)
        {
            return false;
        }
        //  Modified by Sergey KHROMOV - Fri Dec 21 17:46:22 2001 End
        // if(BRep_Tool::Continuity(Ec,ff1,ff2) != GeomAbs_C0) return 0;
        if (global::Kerf.Engine.Fillets.ChFi3d.IsTangentFaces(ec, ff1, ff2))
        {
            return false;
        }
        //  Modified by Sergey KHROMOV - Fri Dec 21 17:46:24 2001 Begin

        Face FirstFace = ff1;
        if (!firstFace.IsNull && ff2.IsSame(firstFace))
        {
            FirstFace = ff2;
            ff2 = ff1;
            ff1 = FirstFace;
        }
        _edgeFirstFace.Bind(ec, FirstFace);

        // Define concavity
        TypeOfConcavity typeOfConcavity = global::Kerf.Engine.Fillets.ChFi3d.DefineConnectType(ec, ff1, ff2, 1.0e-5, true);
        spine.SetTypeOfConcavity(typeOfConcavity);

        bool toRestrict = offset > 0;
        FaceSurfaceAdaptor sb1 = new(ff1, toRestrict);
        FaceSurfaceAdaptor sb2 = new(ff2, toRestrict);
        if (offset > 0)
        {
            Edge OffsetEdge = MakeOffsetEdge(ec, offset, sb1, sb2);
            OffsetEdge.Orientation(ec.Orientation());
            spine.SetOffsetEdges(OffsetEdge);
        }

        EdgeCurveAdaptor cEc = new(), cEv = new();
        Orientation curor = ec.Orientation();
        Exploration.Vertices(ec, out vStart, out lvEc);

        bool fini = false;
        int nb = 0;
        State curSt = State.Closed;
        if (vStart.IsSame(lvEc))
        { // case if only one edge is closed
            cEc.Initialize(ec);
            wl = Tool.Parameter(vStart, ec);
            cEc.D1(wl, out p2, out v1);
            wl = Tool.Parameter(lvEc, ec);
            cEc.D1(wl, out p2, out v2);
            bool isFaceTangency = FaceTangency(ec, ec, vStart);
            if (v1.IsParallel(v2, ta) || isFaceTangency)
            {
                if (isFaceTangency)
                {
                    curSt = State.Closed;
                }
                else
                {
                    curSt = State.BreakPoint;
                }
            }
            else
            {
                curSt = State.BreakPoint;
            }
            spine.SetLastStatus(curSt);
            spine.SetFirstStatus(curSt);
        }
        else
        { // Downstream progression
            fvEc = vStart;
            Orientation or1 = default;
            while (!fini)
            {
                curSt = State.FreeBoundary;
                wl = Tool.Parameter(lvEc, ec);
                degeneOnEc = TangentOnVertex(lvEc, ec, _efMap, ta);
                cEc.Initialize(ec);
                cEc.D1(wl, out p2, out v1);
                nb = spine.EdgeCount;

                for (it.Initialize(_veMap[lvEc]); it.More(); it.Next())
                {
                    ev = ShapeCasts.Edge(it.Value());
                    if (!ev.IsSame(ec) && !Tool.Degenerated(ev))
                    {
                        Exploration.Vertices(ev, out fvEv, out lvEv);
                        if (lvEc.IsSame(lvEv))
                        {
                            ve1 = fvEv;
                            fvEv = lvEv;
                            lvEv = ve1;
                            or1 = Reversed;
                        }
                        else
                        {
                            or1 = Forward;
                        }

                        wf = Tool.Parameter(fvEv, ev);
                        cEv.Initialize(ev);
                        cEv.D1(wf, out p2, out v2);
                        double av1v2 = v1.Angle(v2);
                        bool rev = (or1 != curor);
                        bool onAjoute = false;
                        if (FaceTangency(ec, ev, fvEv))
                        {
                            // there is no need of tolerance
                            // to make a decision (PRO9486) the regularity is enough.
                            // However, the absence of turn-back is checked (PRO9810)
                            onAjoute = ((!rev && av1v2 < M_PI / 2) || (rev && av1v2 > M_PI / 2));
                            // mate attention to the single case (cf CTS21610_1)
                            if (onAjoute && (degeneOnEc || TangentOnVertex(lvEc, ev, _efMap, ta)))
                            {
                                onAjoute = ((!rev && av1v2 < ta) || (rev && (M_PI - av1v2) < ta));
                            }
                        }
                        if (onAjoute)
                        {
                            fini = false; // If this can be useful (Cf PRO14713)
                            Vertex commonVertex = new();
                            Exploration.CommonVertex(ec, ev, ref commonVertex);
                            Edge prevEdge = ec;
                            ec = ev;
                            //	    Ec = TopoDS::Edge(Ev);
                            ec.Orientation(or1);
                            wl = wf;
                            lvEc = lvEv;
                            spine.SetEdges(ec);
                            Face curF1 = new(), curF2 = new();
                            ChFi3d_conexfaces(ec, out curF1, out curF2, _efMap);
                            ReorderFaces(ref curF1, ref curF2, FirstFace, prevEdge, commonVertex, _efMap);
                            _edgeFirstFace.Bind(ec, curF1);
                            if (offset > 0)
                            {
                                FaceSurfaceAdaptor curSb1 = new(curF1), curSb2 = new(curF2);
                                Edge offsetEdge = MakeOffsetEdge(ec, offset, curSb1, curSb2);
                                offsetEdge.Orientation(or1);
                                spine.SetOffsetEdges(offsetEdge);
                            }
                            FirstFace = curF1;
                            curor = or1;
                            if (vStart.IsSame(lvEv))
                            {
                                if (FaceTangency(ev, spine.Edges(1), lvEv))
                                {
                                    curSt = State.Closed;
                                    fini = true;
                                }
                                else
                                {
                                    curSt = State.BreakPoint;
                                    fini = true;
                                }
                            }
                            break;
                        }
                        else
                        {
                            for (jt.Initialize(_efMap[ev]), nbface = 0; jt.More(); jt.Next(), nbface++)
                            {
                            }
                            if (nbface > 1)
                            {
                                curSt = State.BreakPoint;
                            }
                            fini = ((!rev && av1v2 < ta) || (rev && (M_PI - av1v2) < ta));
                        }
                    }
                }
                fini = fini || (nb == spine.EdgeCount);
            }
            spine.SetLastStatus(curSt);
            if (curSt == State.Closed)
            {
                spine.SetFirstStatus(curSt);
            }
            else
            { // Upstream progression
                fini = false;
                ec = spine.Edges(1);
                FirstFace = ShapeCasts.Face(_edgeFirstFace[ec]);
                curor = ec.Orientation();
                fvEc = vStart;
                while (!fini)
                {
                    curSt = State.FreeBoundary;
                    wl = Tool.Parameter(fvEc, ec);
                    degeneOnEc = TangentOnVertex(fvEc, ec, _efMap, ta);
                    cEc.Initialize(ec);
                    cEc.D1(wl, out p2, out v1);
                    nb = spine.EdgeCount;

                    for (it.Initialize(_veMap[fvEc]); it.More(); it.Next())
                    {
                        ev = ShapeCasts.Edge(it.Value());
                        if (!ev.IsSame(ec) && !Tool.Degenerated(ev))
                        {
                            Exploration.Vertices(ev, out fvEv, out lvEv);
                            if (fvEc.IsSame(fvEv))
                            {
                                ve1 = fvEv;
                                fvEv = lvEv;
                                lvEv = ve1;
                                or1 = Reversed;
                            }
                            else
                            {
                                or1 = Forward;
                            }
                            wf = Tool.Parameter(lvEv, ev);
                            cEv.Initialize(ev);
                            cEv.D1(wf, out p2, out v2);
                            double av1v2 = v1.Angle(v2);
                            bool rev = (or1 != curor);
                            bool onAjoute = false;
                            if (FaceTangency(ec, ev, lvEv))
                            {
                                onAjoute = ((!rev && av1v2 < M_PI / 2) || (rev && av1v2 > M_PI / 2));
                                if (onAjoute && (degeneOnEc || TangentOnVertex(fvEc, ev, _efMap, ta)))
                                {
                                    onAjoute = ((!rev && av1v2 < ta) || (rev && (M_PI - av1v2) < ta));
                                }
                            }
                            if (onAjoute)
                            {
                                Vertex commonVertex = new();
                                Exploration.CommonVertex(ec, ev, ref commonVertex);
                                Edge prevEdge = ec;
                                ec = ev;
                                //	      Ec = TopoDS::Edge(Ev);
                                ec.Orientation(or1);
                                wl = wf;
                                fvEc = fvEv;
                                spine.PutInFirst(ec);
                                Face curF1 = new(), curF2 = new();
                                ChFi3d_conexfaces(ec, out curF1, out curF2, _efMap);
                                ReorderFaces(ref curF1, ref curF2, FirstFace, prevEdge, commonVertex, _efMap);
                                _edgeFirstFace.Bind(ec, curF1);
                                if (offset > 0)
                                {
                                    FaceSurfaceAdaptor curSb1 = new(curF1), curSb2 = new(curF2);
                                    Edge offsetEdge = MakeOffsetEdge(ec, offset, curSb1, curSb2);
                                    offsetEdge.Orientation(or1);
                                    spine.PutInFirstOffset(offsetEdge);
                                }
                                FirstFace = curF1;
                                curor = or1;
                                break;
                            }
                            else
                            {
                                for (jt.Initialize(_efMap[ev]), nbface = 0; jt.More(); jt.Next(), nbface++)
                                {
                                }
                                if (nbface > 1)
                                {
                                    curSt = State.BreakPoint;
                                }
                                fini = ((!rev && av1v2 < ta) || (rev && (M_PI - av1v2) < ta));
                            }
                        }
                    }
                    fini = fini || (nb == spine.EdgeCount);
                }
                spine.SetFirstStatus(curSt);
            }
        }
        return true;
    }

    //=================================================================================================

    public void Remove(in Edge e)
    {
        Edge E = ShapeCasts.Edge(ToWork(e));
        NCollection_List<Stripe>.Iterator itel = new(_listStripe);

        for (; itel.More(); itel.Next())
        {
            Spine? sp = itel.Value().Spine();
            for (int j = 1; j <= sp.EdgeCount; j++)
            {
                if (E.IsSame(sp.Edges(j)))
                {
                    _listStripe.Remove(itel);
                    return;
                }
            }
        }
    }

    //=================================================================================================

    public Spine? Value(int i)
    {
        NCollection_List<Stripe>.Iterator itel = new(_listStripe);
        for (int ic = 1; ic < i; ic++)
        {
            itel.Next();
        }
        return itel.Value().Spine();
    }

    //=================================================================================================

    public int NbElements()
    {
        int i = 0;
        NCollection_List<Stripe>.Iterator itel = new(_listStripe);
        for (; itel.More(); itel.Next())
        {
            Spine? sp = itel.Value().Spine();
            if (sp is null)
            {
                break;
            }
            i++;
        }
        return i;
    }

    //=================================================================================================

    public int Contains(in Edge e)
    {
        Edge E = ShapeCasts.Edge(ToWork(e));
        int i = 1, j;
        NCollection_List<Stripe>.Iterator itel = new(_listStripe);
        for (; itel.More(); itel.Next(), i++)
        {
            Spine? sp = itel.Value().Spine();
            if (sp is null)
            {
                break;
            }
            for (j = 1; j <= sp.EdgeCount; j++)
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

    public int Contains(in Edge e, out int indexInSpine)
    {
        Edge E = ShapeCasts.Edge(ToWork(e));
        int i = 1, j;
        indexInSpine = 0;
        NCollection_List<Stripe>.Iterator itel = new(_listStripe);
        for (; itel.More(); itel.Next(), i++)
        {
            Spine? sp = itel.Value().Spine();
            if (sp is null)
            {
                break;
            }
            for (j = 1; j <= sp.EdgeCount; j++)
            {
                if (E.IsSame(sp.Edges(j)))
                {
                    indexInSpine = j;
                    return i;
                }
            }
        }
        return 0;
    }

    //=================================================================================================

    public double Length(int ic)
    {
        if (ic <= NbElements())
        {
            Spine? sp = Value(ic);
            return sp.LastParameter(sp.EdgeCount);
        }
        return -1;
    }

    //=================================================================================================

    public Vertex FirstVertex(int ic)
    {
        if (ic <= NbElements())
        {
            return ShapeCasts.Vertex(FromWork(Value(ic).FirstVertex()));
        }
        return new Vertex();
    }

    //=================================================================================================

    public Vertex LastVertex(int ic)
    {
        if (ic <= NbElements())
        {
            return ShapeCasts.Vertex(FromWork(Value(ic).LastVertex()));
        }
        return new Vertex();
    }

    //=================================================================================================

    public double Abscissa(int ic, in Vertex v)
    {
        if (ic <= NbElements())
        {
            return Value(ic).Absc(ShapeCasts.Vertex(ToWork(v)));
        }
        return -1;
    }

    //=================================================================================================

    public double RelativeAbscissa(int ic, in Vertex v)
    {
        if (ic <= NbElements())
        {
            return Abscissa(ic, v) / Length(ic);
        }
        return -1;
    }

    //=================================================================================================

    public bool Closed(int ic)
    {
        if (ic <= NbElements())
        {
            return Value(ic).IsClosed;
        }
        return false;
    }

    //=================================================================================================

    public bool ClosedAndTangent(int ic)
    {
        if (ic <= NbElements())
        {
            return Value(ic).IsPeriodic;
        }
        return false;
    }

}

internal static class Builder_1
{
    internal static void ReorderFaces(ref Face f1, ref Face f2, in Face firstFace, in Edge prevEdge, in Vertex commonVertex, Map eFmap)
    {
        if (f1.IsSame(firstFace))
        {
            return;
        }
        else if (f2.IsSame(firstFace))
        {
            (f1, f2) = (f2, f1);
            return;
        }

        // Loop until find <theF1> or <theF2>
        Edge PrevEdge = prevEdge;
        Edge curEdge = new();
        Face prevFace = firstFace;
        Face curFace = new();
        for (;;)
        {
            NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> vertexEdgeMap =
                new(ShapeHasher.Instance);
            Exploration.MapShapesAndAncestors(prevFace, ShapeType.Vertex, ShapeType.Edge, vertexEdgeMap);
            foreach (Shape edge in vertexEdgeMap.FindFromKey(commonVertex))
            {
                if (edge.IsSame(PrevEdge))
                {
                    continue;
                }

                curEdge = ShapeCasts.Edge(edge);
                NCollection_List<Shape> flist = eFmap.FindFromKey(curEdge);
                curFace = prevFace.IsSame(ShapeCasts.Face(flist.First())) ? ShapeCasts.Face(flist.Last())
                                                                      : ShapeCasts.Face(flist.First());
                if (curFace.IsSame(f1))
                {
                    return;
                }
                else if (curFace.IsSame(f2))
                {
                    (f1, f2) = (f2, f1);
                    return;
                }
            }

            PrevEdge = curEdge;
            prevFace = curFace;
        }
    }

    internal static void ConcatCurves(NCollection_Sequence<Curve3D> curves, NCollection_Sequence<Curve3D> newCurves)
    {
        while (!curves.IsEmpty())
        {
            CompCurveToBSplineCurve concat = new();
            bool success = false;
            for (int i = 1; i <= curves.Length(); i++)
            {
                Curve3D? curve = curves[i];
                BoundedCurve3D? boundedCurve = (curve as BoundedCurve3D);
                success = concat.Add(boundedCurve, 1.0e-5, true);
                if (!success)
                {
                    success = concat.Add(boundedCurve, 1.0e-5, false);
                }
                if (success)
                {
                    curves.Remove(i);
                    i--;
                }
            }
            Curve3D? newCurve = concat.BSplineCurve();
            newCurves.Append(newCurve);
        }
    }

    internal static Edge MakeOffsetEdge(in Edge edge, double distance, FaceSurfaceAdaptor s1, FaceSurfaceAdaptor s2)
    {
        Edge offsetEdge = new();

        Face f1 = s1.Face();
        Face f2 = s2.Face();
        Surface? gs1 = Tool.Surface(f1);
        Surface? trGS1 = new TrimmedSurface(gs1, s1.FirstUParameter(), s1.LastUParameter(),
                                                                 s1.FirstVParameter(), s1.LastVParameter());
        double offset = -distance;
        if (f1.Orientation() == Reversed)
        {
            offset = distance;
        }
        global::Kerf.Geometry.OffsetSurface? makeOffsetSurf = new global::Kerf.Geometry.OffsetSurface(trGS1, offset);
        Surface? offsetTrGS1 = makeOffsetSurf.Surface();
        if (offsetTrGS1 is null)
        {
            offsetTrGS1 = makeOffsetSurf;
        }
        Surface? gs2 = Tool.Surface(f2);
        Surface? trGS2 = new TrimmedSurface(gs2, s2.FirstUParameter(), s2.LastUParameter(),
                                                                 s2.FirstVParameter(), s2.LastVParameter());
        IntSS intersector = new(offsetTrGS1, trGS2, global::Kerf.Engine.Foundation.Precision.Confusion());
        if (!intersector.IsDone || intersector.LineCount == 0)
        {
            return offsetEdge;
        }

        Curve3D? intCurve = intersector.Line(1);
        Point3D[] ends = new Point3D[2];
        EdgeCurveAdaptor bAcurve = new(edge);
        ends[0] = bAcurve.Value(bAcurve.FirstParameter());
        ends[1] = bAcurve.Value(bAcurve.LastParameter());

        if (intersector.LineCount > 1)
        {
            NCollection_Sequence<Curve3D> curves = new(), newCurves = new();
            for (int i = 1; i <= intersector.LineCount; i++)
            {
                curves.Append(intersector.Line(i));
            }

            ConcatCurves(curves, newCurves);

            double minDist = RealLast();
            int imin = 1;
            for (int i = 1; i <= newCurves.Length(); i++)
            {
                CurveAdaptor gAcurve = new(newCurves[i]);
                CurvePointExtrema projector = new(ends[0], gAcurve);
                if (!projector.IsDone || projector.ExtCount == 0)
                {
                    continue;
                }
                for (int iext = 1; iext <= projector.ExtCount; iext++)
                {
                    double aDist = projector.SquareDistance(iext);
                    if (aDist < minDist)
                    {
                        minDist = aDist;
                        imin = i;
                    }
                }
            }
            intCurve = newCurves[imin];
        }
        if (intCurve is null)
        {
            return offsetEdge;
        }
        // Projection of extremities onto <IntCurve>
        CurveAdaptor gAcurve2 = new(intCurve); // GAcurve2 in OCCT (C# forbids reusing the name)
        double[] Params = new double[2];
        for (int ind_end = 0; ind_end < 2; ind_end++)
        {
            if (ind_end == 1 && bAcurve.IsClosed /*HGuide.IsPeriodic()*/ /*HGuide.IsClosed()*/)
            {
                break;
            }
            CurvePointExtrema projector = new(ends[ind_end], gAcurve2);
            double[] param = new double[4], dist = new double[4];
            Point3D[] Pnt = new Point3D[4];
            param[1] = gAcurve2.FirstParameter();
            param[2] = gAcurve2.LastParameter();
            projector.TrimmedSquareDistances(out dist[1], out dist[2], out Pnt[1], out Pnt[2]);
            dist[3] = RealLast();
            if (projector.IsDone && projector.ExtCount > 0)
            {
                int imin1 = 1; // imin in OCCT (C# forbids reusing the name below)
                for (int i = 2; i <= projector.ExtCount; i++)
                {
                    if (projector.SquareDistance(i) < projector.SquareDistance(imin1))
                    {
                        imin1 = i;
                    }
                }
                param[3] = projector.Point(imin1).Parameter();
                dist[3] = projector.SquareDistance(imin1);
                Pnt[3] = projector.Point(imin1).Value();
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
        if (bAcurve.IsClosed /*HGuide.IsPeriodic()*/ /*HGuide.IsClosed()*/)
        {
            Params[1] = gAcurve2.LastParameter(); // temporary
        }
        if (Params[0] > Params[1])
        {
            bool isClosed = false;
            Point3D fpnt = intCurve.Value(intCurve.FirstParameter());
            Point3D lpnt = intCurve.Value(intCurve.LastParameter());
            if (fpnt.SquareDistance(lpnt) <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
            {
                isClosed = true;
            }
            if (isClosed)
            {
                Params[1] = intCurve.LastParameter();
            }
            else
            {
                double newFirstPar = intCurve.ReversedParameter(Params[0]);
                double newLastPar = intCurve.ReversedParameter(Params[1]);
                intCurve = Reversible(intCurve);
                intCurve.Reverse();
                Params[0] = newFirstPar;
                Params[1] = newLastPar;
            }
        }
        if (bAcurve.IsClosed /*HGuide.IsPeriodic()*/ /*HGuide.IsClosed()*/) // check the direction of closed curve
        {
            Point3D pnt = new(), offsetPnt = new();
            Vector3D tangent = new(), offsetTangent = new();
            bAcurve.D1(bAcurve.FirstParameter(), out pnt, out tangent);
            intCurve.D1(Params[0], out offsetPnt, out offsetTangent);
            if (tangent * offsetTangent < 0)
            {
                intCurve = Reversible(intCurve);
                intCurve.Reverse();
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

        offsetEdge = new LibraryMakeEdge(intCurve, Params[0], Params[1]).Edge();
        return offsetEdge;
    }

    // OCCT reverses the intersection curve in place; a frozen one is reversed on a copy (decision 1088).
    private static Curve3D Reversible(Curve3D curve) =>
        curve.IsFrozen ? (Curve3D)curve.Copy() : curve;

    internal static DsBuildTool Mkbuildtool()
    {
        ToolGeomTool gt2 = new(OutCurveType.BSPLINE1, true, false, false);
        DsBuildTool bt = new(gt2);
        bt.OverWrite(false);
        bt.Translate(false);
        return bt;
    }

    internal static bool TangentExtremity(in Vertex V, in Edge e, FaceSurfaceAdaptor? hs1,
                                          FaceSurfaceAdaptor? hs2, /* const double t3d, */ double tang)
    {
        Face f1 = hs1.Face();
        Orientation o1 = f1.Orientation();
        f1.Orientation(Forward);
        Face f2 = hs2.Face();
        Orientation o2 = f2.Orientation();
        f2.Orientation(Forward);
        Edge e1 = e, e2 = e;
        e1.Orientation(Forward);
        e2.Orientation(Forward);
        if (f1.IsSame(f2) && Tool.IsClosed(e1, f1))
        {
            e2.Orientation(Reversed);
        }
        double p1 = Tool.Parameter(V, e1, f1);
        double p2 = Tool.Parameter(V, e2, f2);
        double u = 0.0, v = 0.0, f = 0.0, l = 0.0, eps = 1.0e-9;
        Vector3D n1 = new(), n2 = new();//   gp_Pnt pt1,pt2;
        Curve2D? pc1 = Tool.CurveOnSurface(e1, f1, out f, out l);
        pc1.Value(p1).Coord(out u, out v);
        SLProps theProp1 = new(hs1, u, v, 1, eps);
        if (theProp1.IsNormalDefined)
        {
            n1 = n1.WithXYZ(theProp1.Normal().XYZ());
            if (o1 == Reversed)
            {
                n1 = n1.Reversed();
            }
        }
        else
        {
            return false; // It is not known...
        }

        Curve2D? pc2 = Tool.CurveOnSurface(e2, f2, out f, out l);
        pc2.Value(p2).Coord(out u, out v);
        SLProps theProp2 = new(hs2, u, v, 1, eps);
        if (theProp2.IsNormalDefined)
        {
            n2 = n2.WithXYZ(theProp2.Normal().XYZ());
            if (o2 == Reversed)
            {
                n2 = n2.Reversed();
            }
        }
        else
        {
            return false; //  It is not known...
        }

        return (n1.Angle(n2) < tang);
    }

    internal static bool TangentOnVertex(in Vertex v, in Edge e, Map efMap, double tang)
    {
        Face ff1 = new(), ff2 = new();
        ChFi3d_conexfaces(e, out ff1, out ff2, efMap);
        if (ff1.IsNull || ff2.IsNull)
        {
            return false;
        }
        FaceSurfaceAdaptor? s1 = new FaceSurfaceAdaptor(ff1);
        FaceSurfaceAdaptor? s2 = new FaceSurfaceAdaptor(ff2);
        return TangentExtremity(v, e, s1, s2, tang);
    }


}
