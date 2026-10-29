// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_WireExplorer.hxx
// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_WireExplorer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_SurfaceType;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Explores the edges of a wire in connection order: each edge is connected to the previous one by its origin.
/// </summary>
/// <remarks>
/// <para>
/// If the wire is not closed, only a segment of edges is returned, whose length depends on the edge the exploration
/// starts with. The wire is assumed valid: loops, wrongly oriented edges (two edges going into or out of a shared
/// vertex), branches and INTERNAL or EXTERNAL edges can stop the exploration early.
/// </para>
/// <para>
/// A working object, as OCCT's (which is used by value). An infinite wire without free vertices starts at an
/// infinite edge found in a hash set, whose iteration order isn't OCCT's (decision 18): with several such edges, the
/// first one can differ.
/// </para>
/// </remarks>
public sealed class BRepTools_WireExplorer
{
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myMap =
        new(TopTools_ShapeMapHasher.Instance);

    private TopoDS_Edge myEdge = new();
    private TopoDS_Vertex myVertex = new();
    private TopoDS_Face myFace = new();
    private readonly NCollection_Map<TopoDS_Shape> myDoubles = new(TopTools_ShapeMapHasher.Instance);
    private bool myReverse;
    private double myTolU;
    private double myTolV;

    /// <summary>An empty explorer (initialise it with <see cref="Init(in TopoDS_Wire)"/>).</summary>
    public BRepTools_WireExplorer()
    {
        myReverse = false;
        myTolU = 0.0;
        myTolV = 0.0;
    }

    /// <summary>Starts an exploration of the wire.</summary>
    public BRepTools_WireExplorer(in TopoDS_Wire W)
    {
        TopoDS_Face F = new TopoDS_Face();
        Init(W, F);
    }

    /// <summary>
    /// Starts an exploration of the wire; the face is used to select the edge connected to the previous one in its
    /// parametric space.
    /// </summary>
    public BRepTools_WireExplorer(in TopoDS_Wire W, in TopoDS_Face F)
    {
        Init(W, F);
    }

    /// <summary>Starts an exploration of the wire.</summary>
    public void Init(in TopoDS_Wire W)
    {
        TopoDS_Face F = new TopoDS_Face();
        Init(W, F);
    }

    /// <summary>
    /// Starts an exploration of the wire; the face is used to select the edge connected to the previous one in its
    /// parametric space.
    /// </summary>
    public void Init(in TopoDS_Wire W, in TopoDS_Face F)
    {
        myEdge = new TopoDS_Edge();
        myVertex = new TopoDS_Vertex();
        myMap.Clear();
        myDoubles.Clear();

        if (W.IsNull())
        {
            return;
        }

        double UMin = 0.0, UMax = 0.0, VMin = 0.0, VMax = 0.0;
        if (!F.IsNull())
        {
            // For the faces based on Cone, BSpline and Bezier compute the
            // UV bounds to precise the UV tolerance values
            GeomAbs_SurfaceType aSurfType = new BRepAdaptor_Surface(F, false).GetType();
            if (aSurfType == GeomAbs_Cone || aSurfType == GeomAbs_BSplineSurface
                || aSurfType == GeomAbs_BezierSurface)
            {
                BRepTools.UVBounds(F, out UMin, out UMax, out VMin, out VMax);
            }
        }

        Init(W, F, UMin, UMax, VMin, VMax);
    }

    /// <summary>
    /// Starts an exploration of the wire; the face is used to select the edge connected to the previous one in its
    /// parametric space, and UMin to VMax are its UV bounds.
    /// </summary>
    public void Init(in TopoDS_Wire W, in TopoDS_Face F, double UMin, double UMax, double VMin, double VMax)
    {
        myEdge = new TopoDS_Edge();
        myVertex = new TopoDS_Vertex();
        myMap.Clear();
        myDoubles.Clear();

        if (W.IsNull())
        {
            return;
        }

        myFace = F;
        double dfVertToler = 0.0;
        myReverse = false;

        if (!myFace.IsNull())
        {
            Geom_Surface? aSurf = BRep_Tool.Surface(myFace, out TopLoc_Location aL);
            GeomAdaptor_Surface aGAS = new GeomAdaptor_Surface(aSurf!);
            var anExp = new TopExp_Explorer(W, TopAbs_VERTEX);
            for (; anExp.More(); anExp.Next())
            {
                TopoDS_Vertex aV = TopoDS.Vertex(anExp.Current());
                dfVertToler = Max(BRep_Tool.Tolerance(aV), dfVertToler);
            }
            if (dfVertToler < Precision.Confusion())
            {
                // Use tolerance of edges
                for (var it0 = new TopoDS_Iterator(W); it0.More(); it0.Next())
                {
                    dfVertToler = Max(BRep_Tool.Tolerance(TopoDS.Edge(it0.Value())), dfVertToler);
                }

                if (dfVertToler < Precision.Confusion())
                {
                    // empty wire
                    return;
                }
            }
            myTolU = 2.0 * aGAS.UResolution(dfVertToler);
            myTolV = 2.0 * aGAS.VResolution(dfVertToler);

            // uresolution for cone with infinite vmin vmax is too small.
            if (aGAS.GetType() == GeomAbs_Cone)
            {
                aGAS.D1(UMin, VMin, out gp_Pnt aP, out gp_Vec aD1U, out gp_Vec aD1V);
                double tol1, tol2, maxtol = .0005 * (UMax - UMin);
                double a = aD1U.Magnitude();

                if (a <= Precision.Confusion())
                {
                    tol1 = maxtol;
                }
                else
                {
                    tol1 = Min(maxtol, dfVertToler / a);
                }

                aGAS.D1(UMin, VMax, out aP, out aD1U, out aD1V);
                a = aD1U.Magnitude();
                if (a <= Precision.Confusion())
                {
                    tol2 = maxtol;
                }
                else
                {
                    tol2 = Min(maxtol, dfVertToler / a);
                }

                myTolU = 2.0 * Max(tol1, tol2);
            }

            if (aGAS.GetType() == GeomAbs_BSplineSurface || aGAS.GetType() == GeomAbs_BezierSurface)
            {
                double maxTol = Max(myTolU, myTolV);
                aGAS.D1((UMax - UMin) / 2.0, (VMax - VMin) / 2.0, out gp_Pnt aP, out gp_Vec aDU, out gp_Vec aDV);
                double mod = Math.Sqrt(aDU * aDU + aDV * aDV);
                if (mod > gp.Resolution())
                {
                    if (mod * maxTol / dfVertToler < 1.5)
                    {
                        maxTol = 1.5 * dfVertToler / mod;
                    }
                    myTolU = maxTol;
                    myTolV = maxTol;
                }
            }

            myReverse = (myFace.Orientation() == TopAbs_REVERSED);
        }

        // map of vertices to know if the wire is open
        var vmap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        //  map of infinite edges
        var anInfEmap = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);

        // list the vertices
        TopoDS_Vertex V1 = new TopoDS_Vertex(), V2 = new TopoDS_Vertex();

        var it = new TopoDS_Iterator(W);
        while (it.More())
        {
            TopoDS_Edge E = TopoDS.Edge(it.Value());
            TopAbs_Orientation Eori = E.Orientation();
            if (Eori == TopAbs_INTERNAL || Eori == TopAbs_EXTERNAL)
            {
                it.Next();
                continue;
            }
            TopExp.Vertices(E, out V1, out V2, true);

            if (!V1.IsNull())
            {
                if (!myMap.IsBound(V1))
                {
                    // OCCT binds a copy of one empty list; the port's lists are classes, so each vertex gets its own
                    myMap.Bind(V1, new NCollection_List<TopoDS_Shape>());
                }
                myMap[V1].Append(E);

                // add or remove in the vertex map
                V1.Orientation(TopAbs_FORWARD);
                int currsize = vmap.Extent(), ind = vmap.Add(V1);
                if (currsize >= ind)
                {
                    vmap.RemoveKey(V1);
                }
            }

            if (!V2.IsNull())
            {
                V2.Orientation(TopAbs_REVERSED);
                int currsize = vmap.Extent(), ind = vmap.Add(V2);
                if (currsize >= ind)
                {
                    vmap.RemoveKey(V2);
                }
            }

            if (V1.IsNull() || V2.IsNull())
            {
                BRep_Tool.Range(E, out double aF, out double aL);

                if (Eori == TopAbs_FORWARD)
                {
                    if (aF == -Precision.Infinite())
                    {
                        anInfEmap.Add(E);
                    }
                }
                else
                { // Eori == TopAbs_REVERSED
                    if (aL == Precision.Infinite())
                    {
                        anInfEmap.Add(E);
                    }
                }
            }
            it.Next();
        }

        // Construction of the set of double edges.
        var it2 = new TopoDS_Iterator(W);
        var emap = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        while (it2.More())
        {
            if (!emap.Add(it2.Value()))
            {
                myDoubles.Add(it2.Value());
            }
            it2.Next();
        }

        // if vmap is not empty the wire is open, let us find the first vertex
        if (!vmap.IsEmpty())
        {
            int ind = 0;
            for (ind = 1; ind <= vmap.Extent(); ++ind)
            {
                if (vmap[ind].Orientation() == TopAbs_FORWARD)
                {
                    V1 = TopoDS.Vertex(vmap[ind]);
                    break;
                }
            }
        }
        else
        {
            //   The wire is infinite Try to find the first vertex. It may be NULL.
            if (!anInfEmap.IsEmpty())
            {
                foreach (TopoDS_Shape aKey in anInfEmap)
                {
                    TopoDS_Edge anEdge = TopoDS.Edge(aKey);
                    TopAbs_Orientation anOri = anEdge.Orientation();

                    BRep_Tool.Range(anEdge, out double aF, out double aL);
                    if ((anOri == TopAbs_FORWARD && aF == -Precision.Infinite())
                        || (anOri == TopAbs_REVERSED && aL == Precision.Infinite()))
                    {
                        myEdge = anEdge;
                        myVertex = new TopoDS_Vertex();

                        return;
                    }
                }
            }

            // use the first vertex in iterator
            it.Initialize(W);
            while (it.More())
            {
                TopoDS_Edge E = TopoDS.Edge(it.Value());
                TopAbs_Orientation Eori = E.Orientation();
                if (Eori == TopAbs_INTERNAL || Eori == TopAbs_EXTERNAL)
                {
                    // JYL 10-03-97 : waiting for correct processing
                    // of INTERNAL/EXTERNAL edges
                    it.Next();
                    continue;
                }
                TopExp.Vertices(E, out V1, out V2, true);
                break;
            }
        }

        if (V1.IsNull())
        {
            return;
        }
        if (!myMap.IsBound(V1))
        {
            return;
        }

        NCollection_List<TopoDS_Shape> l = myMap[V1];
        myEdge = TopoDS.Edge(l.First());
        l.RemoveFirst();
        myVertex = TopExp.FirstVertex(myEdge, true);
    }

    /// <summary>True if there is a current edge.</summary>
    public bool More() => !myEdge.IsNull();

    /// <summary>Moves to the next edge.</summary>
    public void Next()
    {
        myVertex = TopExp.LastVertex(myEdge, true);

        if (myVertex.IsNull())
        {
            myEdge = new TopoDS_Edge();
            return;
        }
        if (!myMap.IsBound(myVertex))
        {
            myEdge = new TopoDS_Edge();
            return;
        }

        NCollection_List<TopoDS_Shape> l = myMap[myVertex];

        if (l.IsEmpty())
        {
            myEdge = new TopoDS_Edge();
        }
        else if (l.Extent() == 1)
        {
            //  Modified by Sergey KHROMOV - Fri Jun 21 10:28:01 2002 OCC325 Begin
            TopoDS_Edge aNextEdge = TopoDS.Edge(l.First());

            TopExp.Vertices(aNextEdge, out TopoDS_Vertex aV1, out TopoDS_Vertex aV2, true);

            if (!aV1.IsSame(myVertex))
            {
                myEdge = new TopoDS_Edge();
                return;
            }
            if (!myFace.IsNull() && aV1.IsSame(aV2))
            {
                Geom2d_Curve? aPrevPC = BRep_Tool.CurveOnSurface(myEdge, myFace, out double aPar11, out double aPar12);
                Geom2d_Curve? aNextPC = BRep_Tool.CurveOnSurface(aNextEdge, myFace, out double aPar21, out double aPar22);
                double aPrevPar;
                double aNextFPar;
                double aNextLPar;

                if (aPrevPC is null || aNextPC is null)
                {
                    myEdge = new TopoDS_Edge();
                    return;
                }

                if (myEdge.Orientation() == TopAbs_FORWARD)
                {
                    aPrevPar = aPar12;
                }
                else
                {
                    aPrevPar = aPar11;
                }

                if (aNextEdge.Orientation() == TopAbs_FORWARD)
                {
                    aNextFPar = aPar21;
                    aNextLPar = aPar22;
                }
                else
                {
                    aNextFPar = aPar22;
                    aNextLPar = aPar21;
                }

                gp_Pnt2d aPPrev = aPrevPC.Value(aPrevPar);
                gp_Pnt2d aPNextF = aNextPC.Value(aNextFPar);
                gp_Pnt2d aPNextL = aNextPC.Value(aNextLPar);

                if (aPPrev.SquareDistance(aPNextF) > aPPrev.SquareDistance(aPNextL))
                {
                    myEdge = new TopoDS_Edge();
                    return;
                }
            }
            //  Modified by Sergey KHROMOV - Fri Jun 21 11:08:16 2002 End
            myEdge = TopoDS.Edge(l.First());
            l.Clear();
        }
        else
        {
            if (myFace.IsNull())
            {
                // Without Face - try to return edges
                // as logically as possible
                // At first degenerated edges.
                TopoDS_Edge E = myEdge;
                if (SelectDegenerated(l, ref E))
                {
                    myEdge = E;
                    return;
                }
                // At second double edges.
                E = myEdge;
                if (SelectDouble(myDoubles, l, ref E))
                {
                    myEdge = E;
                    return;
                }

                var it = new NCollection_List<TopoDS_Shape>.Iterator(l);
                bool notfound = true;
                while (it.More())
                {
                    if (!it.Value().IsSame(myEdge))
                    {
                        myEdge = TopoDS.Edge(it.Value());
                        l.Remove(it);
                        notfound = false;
                        break;
                    }
                    it.Next();
                }

                if (notfound)
                {
                    myEdge = new TopoDS_Edge();
                    return;
                }
            }
            else
            {
                // If we have more than one edge attached to the list
                // probably wire that we explore contains a loop or loops.
                Geom2d_Curve? aPCurve = BRep_Tool.CurveOnSurface(myEdge, myFace, out double dfFPar, out double dfLPar);
                if (aPCurve is null)
                {
                    myEdge = new TopoDS_Edge();
                    return;
                }
                // Note: current < myVertex > which is last on < myEdge >
                //       equals in 2D to following 2D points:
                //       edge is FORWARD  - point with MAX parameter on PCurve;
                //       edge is REVERSED - point with MIN parameter on PCurve.

                // Get 2D point equals to < myVertex > in 2D for current edge.
                gp_Pnt2d PRef;
                if (myEdge.Orientation() == TopAbs_REVERSED)
                {
                    aPCurve.D0(dfFPar, out PRef);
                }
                else
                {
                    aPCurve.D0(dfLPar, out PRef);
                }

                // Get next 2D point from current edge's PCurve with parameter
                // F + dP (REV) or L - dP (FOR)
                bool isrevese = (myEdge.Orientation() == TopAbs_REVERSED);
                double dfMPar = GetNextParamOnPC(aPCurve, PRef, dfFPar, dfLPar, myTolU, myTolV, isrevese);

                aPCurve.D0(dfMPar, out gp_Pnt2d PRefm);
                // Get vector from PRef to PRefm
                gp_Vec2d anERefDir = new gp_Vec2d(PRef, PRefm);
                if (anERefDir.SquareMagnitude() < gp.Resolution())
                {
                    myEdge = new TopoDS_Edge();
                    return;
                }

                // Search the list of edges looking for the edge having hearest
                // 2D point of connected vertex to current one and smallest angle.
                // First process all degenerated edges, then - all others.

                var it = new NCollection_List<TopoDS_Shape>.Iterator();
                int k = 1, kMin = 0, iDone = 0;
                bool isDegenerated = true;
                double dmin = RealLast();
                double dfMinAngle = 3.0 * Math.PI, dfCurAngle = 3.0 * Math.PI;

                for (iDone = 0; iDone < 2; iDone++)
                {
                    it.Initialize(l);
                    while (it.More())
                    {
                        TopoDS_Edge E = TopoDS.Edge(it.Value());
                        if (E.IsSame(myEdge))
                        {
                            it.Next();
                            k++;
                            continue;
                        }

                        TopExp.Vertices(E, out TopoDS_Vertex aVert1, out TopoDS_Vertex aVert2, true);
                        if (aVert1.IsNull() || aVert2.IsNull())
                        {
                            it.Next();
                            k++;
                            continue;
                        }

                        aPCurve = BRep_Tool.CurveOnSurface(E, myFace, out dfFPar, out dfLPar);
                        if (aPCurve is null)
                        {
                            it.Next();
                            k++;
                            continue;
                        }

                        gp_Pnt2d aPEb, aPEe;
                        if (aVert1.IsSame(aVert2) == isDegenerated)
                        {
                            if (E.Orientation() == TopAbs_REVERSED)
                            {
                                aPCurve.D0(dfLPar, out aPEb);
                            }
                            else
                            {
                                aPCurve.D0(dfFPar, out aPEb);
                            }

                            if (Math.Abs(dfLPar - dfFPar) > Precision.PConfusion())
                            {
                                isrevese = (E.Orientation() == TopAbs_REVERSED);
                                isrevese = !isrevese;
                                double aEPm =
                                    GetNextParamOnPC(aPCurve, aPEb, dfFPar, dfLPar, myTolU, myTolV, isrevese);

                                aPCurve.D0(aEPm, out aPEe);
                                if (aPEb.SquareDistance(aPEe) <= gp.Resolution())
                                {
                                    // seems to be very short curve
                                    aPCurve.D1(aEPm, out aPEe, out gp_Vec2d aD);
                                    if (E.Orientation() == TopAbs_REVERSED)
                                    {
                                        aPEe.SetXY(aPEb.XY() - aD.XY());
                                    }
                                    else
                                    {
                                        aPEe.SetXY(aPEb.XY() + aD.XY());
                                    }

                                    if (aPEb.SquareDistance(aPEe) <= gp.Resolution())
                                    {
                                        it.Next();
                                        k++;
                                        continue;
                                    }
                                }
                                gp_Vec2d anEDir = new gp_Vec2d(aPEb, aPEe);
                                dfCurAngle = Math.Abs(anEDir.Angle(anERefDir));
                            }

                            if (dfCurAngle <= dfMinAngle)
                            {
                                double d = PRef.SquareDistance(aPEb);
                                if (d <= Precision.PConfusion())
                                {
                                    d = 0.0;
                                }
                                if (Math.Abs(aPEb.X() - PRef.X()) < myTolU && Math.Abs(aPEb.Y() - PRef.Y()) < myTolV)
                                {
                                    if (d <= dmin)
                                    {
                                        dfMinAngle = dfCurAngle;
                                        kMin = k;
                                        dmin = d;
                                    }
                                }
                            }
                        }
                        it.Next();
                        k++;
                    } // while it

                    if (kMin == 0)
                    {
                        isDegenerated = false;
                        k = 1;
                        dmin = RealLast();
                    }
                    else
                    {
                        break;
                    }
                } // for iDone

                if (kMin == 0)
                {
                    // probably unclosed in 2d space wire
                    myEdge = new TopoDS_Edge();
                    return;
                }

                // Selection the edge.
                it.Initialize(l);
                k = 1;
                while (it.More())
                {
                    if (k == kMin)
                    {
                        myEdge = TopoDS.Edge(it.Value());
                        l.Remove(it);
                        break;
                    }
                    it.Next();
                    k++;
                }
            } // else face != NULL && l > 1
        } // else l > 1
    }

    /// <summary>The current edge.</summary>
    public TopoDS_Edge Current() => myEdge;

    /// <summary>The orientation of the vertex connecting the current edge to the previous one, in the current edge.</summary>
    /// <exception cref="Standard_NoSuchObject">The vertex isn't in the current edge.</exception>
    public TopAbs_Orientation Orientation()
    {
        if (myVertex.IsNull() && !myEdge.IsNull())
        {
            // infinite edge
            return TopAbs_FORWARD;
        }

        var it = new TopoDS_Iterator(myEdge, false);
        while (it.More())
        {
            if (myVertex.IsSame(it.Value()))
            {
                return it.Value().Orientation();
            }
            it.Next();
        }
        throw new Standard_NoSuchObject("BRepTools_WireExplorer::Orientation");
    }

    /// <summary>The vertex connecting the current edge to the previous one.</summary>
    public TopoDS_Vertex CurrentVertex() => myVertex;

    /// <summary>Clears the explorer.</summary>
    public void Clear()
    {
        myMap.Clear();
        myDoubles.Clear();
        myEdge = new TopoDS_Edge();
        myFace = new TopoDS_Face();
        myVertex = new TopoDS_Vertex();
    }

    private static bool SelectDouble(NCollection_Map<TopoDS_Shape> Doubles, NCollection_List<TopoDS_Shape> L, ref TopoDS_Edge E)
    {
        var it = new NCollection_List<TopoDS_Shape>.Iterator(L);

        for (; it.More(); it.Next())
        {
            TopoDS_Shape CE = it.Value();
            if (Doubles.Contains(CE) && (!E.IsSame(CE)))
            {
                E = TopoDS.Edge(CE);
                L.Remove(it);
                return true;
            }
        }
        return false;
    }

    private static bool SelectDegenerated(NCollection_List<TopoDS_Shape> L, ref TopoDS_Edge E)
    {
        var it = new NCollection_List<TopoDS_Shape>.Iterator(L);
        while (it.More())
        {
            if (!it.Value().IsSame(E))
            {
                E = TopoDS.Edge(it.Value());
                if (BRep_Tool.Degenerated(E))
                {
                    L.Remove(it);
                    return true;
                }
            }
            it.Next();
        }
        return false;
    }

    private static double GetNextParamOnPC(Geom2d_Curve aPC,
                                           in gp_Pnt2d aPRef,
                                           double fP,
                                           double lP,
                                           double tolU,
                                           double tolV,
                                           bool reverse)
    {
        double result = (reverse) ? fP : lP;
        double dP = Math.Abs(lP - fP) / 1000.0; // was / 16.;
        if (reverse)
        {
            double startPar = fP;
            bool nextPntOnEdge = false;
            while (!nextPntOnEdge && startPar < lP)
            {
                startPar += dP;
                aPC.D0(startPar, out gp_Pnt2d pnt);
                if (Math.Abs(aPRef.X() - pnt.X()) < tolU && Math.Abs(aPRef.Y() - pnt.Y()) < tolV)
                {
                    continue;
                }
                else
                {
                    result = startPar;
                    nextPntOnEdge = true;
                    break;
                }
            }

            if (!nextPntOnEdge)
            {
                result = lP;
            }

            if (result > lP)
            {
                result = lP;
            }
        }
        else
        {
            double startPar = lP;
            bool nextPntOnEdge = false;
            while (!nextPntOnEdge && startPar > fP)
            {
                startPar -= dP;
                aPC.D0(startPar, out gp_Pnt2d pnt);
                if (Math.Abs(aPRef.X() - pnt.X()) < tolU && Math.Abs(aPRef.Y() - pnt.Y()) < tolV)
                {
                    continue;
                }
                else
                {
                    result = startPar;
                    nextPntOnEdge = true;
                    break;
                }
            }

            if (!nextPntOnEdge)
            {
                result = fP;
            }

            if (result < fP)
            {
                result = fP;
            }
        }

        return result;
    }
}
