// Port of: src/ModelingAlgorithms/TKBool/BRepAlgo/BRepAlgo_Loop.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepAlgo/BRepAlgo_Loop.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Builds the wires (loops) of a face from a set of edges with pcurves on it: edges to cut at given vertices, and
/// constant edges; then, optionally, the faces those wires bound (<see cref="WiresToFaces"/>).
/// </summary>
/// <remarks>
/// A working object of the offset algorithms. As in OCCT, it writes into the edges it is given (it replaces their
/// vertices by close ones and raises vertex tolerances): they must be unfrozen, as ported callers' are; a frozen one
/// throws at the first write. The shapes handed out are not frozen.
/// </remarks>
public sealed class BRepAlgo_Loop
{
    private TopoDS_Face myFace = new();
    private readonly NCollection_List<TopoDS_Shape> myConstEdges = new();
    private readonly NCollection_List<TopoDS_Shape> myEdges = new();
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myVerOnEdges = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_List<TopoDS_Shape> myNewWires = new();
    private readonly NCollection_List<TopoDS_Shape> myNewFaces = new();
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myCutEdges = new(TopTools_ShapeMapHasher.Instance);
    private NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> myVerticesForSubstitute = new(TopTools_ShapeMapHasher.Instance);
    private readonly BRepAlgo_Image myImageVV = new();
    private double myTolConf;

    /// <summary>An empty loop builder.</summary>
    public BRepAlgo_Loop()
    {
        myTolConf = 0.001;
    }

    /// <summary>Starts again on the face <paramref name="F"/>; the edges must have pcurves on it.</summary>
    public void Init(in TopoDS_Face F)
    {
        myConstEdges.Clear();
        myEdges.Clear();
        myVerOnEdges.Clear();
        myNewWires.Clear();
        myNewFaces.Clear();
        myCutEdges.Clear();
        myFace = F;
    }

    // Orders the sequence of vertices by increasing parameter on E (removing duplicates)
    private static void Bubble(in TopoDS_Edge E, NCollection_Sequence<TopoDS_Shape> Seq)
    {
        // Remove duplicates
        for (int i = 1; i < Seq.Length(); i++)
        {
            for (int j = i + 1; j <= Seq.Length(); j++)
            {
                if (Seq.Value(i) == Seq.Value(j))
                {
                    Seq.Remove(j);
                    j--;
                }
            }
        }

        bool Invert = true;
        int NbPoints = Seq.Length();
        double U1, U2;
        TopoDS_Vertex V1, V2;

        while (Invert)
        {
            Invert = false;
            for (int i = 1; i < NbPoints; i++)
            {
                TopoDS_Shape aLocalV = Seq.Value(i).Oriented(TopAbs_INTERNAL);
                V1 = TopoDS.Vertex(aLocalV);
                aLocalV = Seq.Value(i + 1).Oriented(TopAbs_INTERNAL);
                V2 = TopoDS.Vertex(aLocalV);

                U1 = BRep_Tool.Parameter(V1, E);
                U2 = BRep_Tool.Parameter(V2, E);
                if (U2 < U1)
                {
                    Seq.Exchange(i, i + 1);
                    Invert = true;
                }
            }
        }
    }

    /// <summary>Adds the edge <paramref name="E"/>, to be cut at the vertices <paramref name="LV"/>.</summary>
    public void AddEdge(in TopoDS_Edge E, NCollection_List<TopoDS_Shape> LV)
    {
        myEdges.Append(E);
        myVerOnEdges.Bind(E, new NCollection_List<TopoDS_Shape>(LV));
    }

    /// <summary>Adds the constant edge <paramref name="E"/>, which can be in the result as it is.</summary>
    public void AddConstEdge(in TopoDS_Edge E)
    {
        myConstEdges.Append(E);
    }

    /// <summary>Adds the constant edges <paramref name="LE"/>.</summary>
    public void AddConstEdges(NCollection_List<TopoDS_Shape> LE)
    {
        NCollection_List<TopoDS_Shape>.Iterator itl = new(LE);
        for (; itl.More(); itl.Next())
        {
            myConstEdges.Append(itl.Value());
        }
    }

    /// <summary>Sets the images of vertices into vertices (a copy).</summary>
    public void SetImageVV(BRepAlgo_Image theImageVV)
    {
        myImageVV.Assign(theImageVV);
    }

    // If the first or the last vertex of intersection coincides with the closing vertex, it is removed from SV; it will
    // be added at the beginning and the end of SV by the caller
    private static TopoDS_Vertex UpdateClosedEdge(in TopoDS_Edge E, NCollection_Sequence<TopoDS_Shape> SV)
    {
        TopoDS_Vertex[] VB = [new(), new()];
        TopoDS_Vertex VRes = new();
        Point3D P, PC;
        bool OnStart = false, OnEnd = false;
        //// modified by jgv, 13.04.04 for OCC5634 ////
        TopExp.Vertices(E, out TopoDS_Vertex V1, out _);
        double Tol = BRep_Tool.Tolerance(V1);
        ///////////////////////////////////////////////

        if (SV.IsEmpty())
        {
            return VRes;
        }

        VB[0] = TopoDS.Vertex(SV.First());
        VB[1] = TopoDS.Vertex(SV.Last());
        PC = BRep_Tool.Pnt(V1);

        for (int i = 0; i < 2; i++)
        {
            P = BRep_Tool.Pnt(VB[i]);
            if (P.IsEqual(PC, Tol))
            {
                VRes = VB[i];
                if (i == 0)
                {
                    OnStart = true;
                }
                else
                {
                    OnEnd = true;
                }
            }
        }
        if (OnStart && OnEnd)
        {
            if (!VB[0].IsSame(VB[1]))
            {
                // Two different vertices on the closing vertex
            }
            else
            {
                SV.Remove(1);
                if (!SV.IsEmpty())
                {
                    SV.Remove(SV.Length());
                }
            }
        }
        else if (OnStart)
        {
            SV.Remove(1);
        }
        else if (OnEnd)
        {
            SV.Remove(SV.Length());
        }

        return VRes;
    }

    // Removes the hanging edges
    private static void RemovePendingEdges(NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> MVE)
    {
        //--------------------------------
        // Remove hanging edges.
        //--------------------------------
        NCollection_List<TopoDS_Shape>.Iterator itl = new();
        bool YaSupress = true;

        while (YaSupress)
        {
            YaSupress = false;
            NCollection_List<TopoDS_Shape> VToRemove = new();
            NCollection_Map<TopoDS_Shape> EToRemove = new(TopTools_ShapeMapHasher.Instance);

            for (int iV = 1; iV <= MVE.Extent(); iV++)
            {
                TopoDS_Shape aVertex = MVE.FindKey(iV);
                NCollection_List<TopoDS_Shape> anEdges = MVE.FindFromIndex(iV);
                if (anEdges.IsEmpty())
                {
                    VToRemove.Append(aVertex);
                }
                if (anEdges.Extent() == 1)
                {
                    TopoDS_Edge E = TopoDS.Edge(anEdges.First());
                    TopExp.Vertices(E, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
                    if (!V1.IsSame(V2))
                    {
                        VToRemove.Append(aVertex);
                        EToRemove.Add(anEdges.First());
                    }
                }
            }

            if (!VToRemove.IsEmpty())
            {
                YaSupress = true;
                for (itl.Initialize(VToRemove); itl.More(); itl.Next())
                {
                    MVE.RemoveKey(itl.Value());
                }
                if (!EToRemove.IsEmpty())
                {
                    for (int iV = 1; iV <= MVE.Extent(); iV++)
                    {
                        NCollection_List<TopoDS_Shape> LE = MVE.ChangeFromIndex(iV);
                        itl.Initialize(LE);
                        while (itl.More())
                        {
                            if (EToRemove.Contains(itl.Value()))
                            {
                                LE.Remove(itl);
                            }
                            else
                            {
                                itl.Next();
                            }
                        }
                    }
                }
            }
        }
    }

    // Whether E1's start and E2's end meet at V in the face's parametric space (100 times V's tolerance)
    private static bool SamePnt2d(in TopoDS_Vertex V, in TopoDS_Edge E1, in TopoDS_Edge E2, in TopoDS_Face F)
    {
        Point2D P1, P2;
        TopoDS_Shape aLocalF = F.Oriented(TopAbs_FORWARD);
        TopoDS_Face FF = TopoDS.Face(aLocalF);
        Geom2d_Curve C1 = BRep_Tool.CurveOnSurface(E1, FF, out double f1, out double l1)!;
        Geom2d_Curve C2 = BRep_Tool.CurveOnSurface(E2, FF, out double f2, out double l2)!;
        if (E1.Orientation() == TopAbs_FORWARD)
        {
            P1 = C1.Value(f1);
        }
        else
        {
            P1 = C1.Value(l1);
        }

        if (E2.Orientation() == TopAbs_FORWARD)
        {
            P2 = C2.Value(l2);
        }
        else
        {
            P2 = C2.Value(f2);
        }
        double Tol = 100 * BRep_Tool.Tolerance(V);
        double Dist = P1.Distance(P2);
        return Dist < Tol;
    }

    // Finds the edge NE connected to CE by the vertex CV in LE and removes it from LE; also removes CE from LE if it is
    // there with the same orientation
    private static bool SelectEdge(in TopoDS_Face F,
                                   in TopoDS_Edge CE,
                                   in TopoDS_Vertex CV,
                                   out TopoDS_Edge NE,
                                   NCollection_List<TopoDS_Shape> LE)
    {
        NCollection_List<TopoDS_Shape>.Iterator itl = new();
        NE = new TopoDS_Edge();

        for (itl.Initialize(LE); itl.More(); itl.Next())
        {
            if (itl.Value().IsEqual(CE))
            {
                LE.Remove(itl);
                break;
            }
        }

        if (LE.Extent() > 1)
        {
            //--------------------------------------------------------------
            // Several edges possible.
            // - Test edges different from CE , Selection of edge
            // for which CV has U,V closer to the face
            // than corresponding to CE.
            // - If several edges give representation less than the tolerance.
            // discrimination on tangents.
            //--------------------------------------------------------------
            TopoDS_Face FForward = F;
            FForward.Orientation(TopAbs_FORWARD);

            Geom2d_Curve C = BRep_Tool.CurveOnSurface(CE, FForward, out double f, out double l)!;
            int k = 1, kmin = 0;
            double dist, distmin = 100 * BRep_Tool.Tolerance(CV);
            double u;
            if (CE.Orientation() == TopAbs_FORWARD)
            {
                u = l;
            }
            else
            {
                u = f;
            }

            Point2D P2, PV = C.Value(u);

            for (itl.Initialize(LE); itl.More(); itl.Next())
            {
                TopoDS_Edge E = TopoDS.Edge(itl.Value());
                if (!E.IsSame(CE))
                {
                    C = BRep_Tool.CurveOnSurface(E, FForward, out f, out l)!;
                    if (E.Orientation() == TopAbs_FORWARD)
                    {
                        u = f;
                    }
                    else
                    {
                        u = l;
                    }
                    P2 = C.Value(u);
                    dist = PV.Distance(P2);
                    if (dist <= distmin)
                    {
                        kmin = k;
                        distmin = dist;
                    }
                }
                k++;
            }

            if (kmin == 0)
            {
                return false;
            }

            k = 1;
            itl.Initialize(LE);
            while (k < kmin)
            {
                k++;
                itl.Next();
            }
            NE = TopoDS.Edge(itl.Value());
            LE.Remove(itl);
        }
        else if (LE.Extent() == 1)
        {
            NE = TopoDS.Edge(LE.First());
            LE.RemoveFirst();
        }
        else
        {
            return false;
        }
        return true;
    }

    // Keeps in the lists of new edges only the edges used in the wires
    private static void PurgeNewEdges(NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> NewEdges,
                                      NCollection_Map<TopoDS_Shape> UsedEdges)
    {
        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> it in NewEdges)
        {
            NCollection_List<TopoDS_Shape> LNE = it.Value;
            NCollection_List<TopoDS_Shape>.Iterator itL = new(LNE);

            while (itL.More())
            {
                TopoDS_Shape NE = itL.Value();
                if (!UsedEdges.Contains(NE))
                {
                    LNE.Remove(itL);
                }
                else
                {
                    itL.Next();
                }
            }
        }
    }

    // Adds E to the map vertex -> edges, first replacing its vertices by vertices of the map at the same place
    // (within theTolConf), recorded in VerticesForSubstitute; doubles the seams
    private static void StoreInMVE(in TopoDS_Face F,
                                   in TopoDS_Edge E,
                                   NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> MVE,
                                   ref bool YaCouture,
                                   NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> VerticesForSubstitute,
                                   double theTolConf)
    {
        TopoDS_Vertex V1, V2, V;
        Point3D P1, P;
        BRep_Builder BB = new();
        for (int iV = 1; iV <= MVE.Extent(); iV++)
        {
            V = TopoDS.Vertex(MVE.FindKey(iV));
            P = BRep_Tool.Pnt(V);
            NCollection_List<TopoDS_Shape> VList = new();
            TopoDS_Iterator VerExp = new(E);
            for (; VerExp.More(); VerExp.Next())
            {
                VList.Append(VerExp.Value());
            }
            NCollection_List<TopoDS_Shape>.Iterator itl = new(VList);
            for (; itl.More(); itl.Next())
            {
                V1 = TopoDS.Vertex(itl.Value());
                P1 = BRep_Tool.Pnt(V1);
                if (P.IsEqual(P1, theTolConf) && !V.IsSame(V1))
                {
                    V.Orientation(V1.Orientation());
                    if (VerticesForSubstitute.IsBound(V1))
                    {
                        TopoDS_Shape OldNewV = VerticesForSubstitute[V1];
                        if (!OldNewV.IsSame(V))
                        {
                            VerticesForSubstitute.Bind(OldNewV, V);
                            VerticesForSubstitute[V1] = V;
                        }
                    }
                    else
                    {
                        if (VerticesForSubstitute.IsBound(V))
                        {
                            TopoDS_Shape NewNewV = VerticesForSubstitute[V];
                            if (!NewNewV.IsSame(V1))
                            {
                                VerticesForSubstitute.Bind(V1, NewNewV);
                            }
                        }
                        else
                        {
                            VerticesForSubstitute.Bind(V1, V);
                            // OCCT changes the map's values while iterating it: the keys are collected first
                            NCollection_List<TopoDS_Shape> aKeys = new();
                            foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> mapit in VerticesForSubstitute)
                            {
                                if (mapit.Value.IsSame(V1))
                                {
                                    aKeys.Append(mapit.Key);
                                }
                            }
                            for (NCollection_List<TopoDS_Shape>.Iterator itk = new(aKeys); itk.More(); itk.Next())
                            {
                                VerticesForSubstitute[itk.Value()] = V;
                            }
                        }
                    }
                    E.Free(true);
                    BB.Remove(E, V1);
                    BB.Add(E, V);
                }
            }
        }

        TopExp.Vertices(E, out V1, out V2);
        if (V1.IsNull() && V2.IsNull())
        {
            YaCouture = false;
            return;
        }
        if (!MVE.Contains(V1))
        {
            MVE.Add(V1, new NCollection_List<TopoDS_Shape>());
        }
        MVE.ChangeFromKey(V1).Append(E);
        if (!V1.IsSame(V2))
        {
            if (!MVE.Contains(V2))
            {
                MVE.Add(V2, new NCollection_List<TopoDS_Shape>());
            }
            MVE.ChangeFromKey(V2).Append(E);
        }
        Geom_Surface? S = BRep_Tool.Surface(F, out Placement L);
        if (BRep_Tool.IsClosed(E, S, L))
        {
            MVE.ChangeFromKey(V2).Append(E.Reversed());
            if (!V1.IsSame(V2))
            {
                MVE.ChangeFromKey(V1).Append(E.Reversed());
            }
            YaCouture = true;
        }
    }

    /// <summary>Cuts the edges and builds the wires.</summary>
    public void Perform()
    {
        NCollection_List<TopoDS_Shape>.Iterator itl = new(), itl1 = new();
        TopoDS_Vertex V1, V2;
        bool YaCouture = false;

        //------------------------------------------------
        // Cut edges
        //------------------------------------------------
        for (itl.Initialize(myEdges); itl.More(); itl.Next())
        {
            TopoDS_Edge anEdge = TopoDS.Edge(itl.Value());
            NCollection_List<TopoDS_Shape> LCE = new();
            if (myVerOnEdges.Find(anEdge, out NCollection_List<TopoDS_Shape> pVertices))
            {
                CutEdge(anEdge, pVertices, LCE);
                myCutEdges.Bind(anEdge, LCE);
            }
        }
        //-----------------------------------
        // Construction map vertex => edges
        //-----------------------------------
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> MVE = new(TopTools_ShapeMapHasher.Instance);

        // add cut edges.
        NCollection_Map<TopoDS_Shape> Emap = new(TopTools_ShapeMapHasher.Instance);
        for (itl.Initialize(myEdges); itl.More(); itl.Next())
        {
            if (myCutEdges.Find(itl.Value(), out NCollection_List<TopoDS_Shape> pLCE))
            {
                for (itl1.Initialize(pLCE); itl1.More(); itl1.Next())
                {
                    TopoDS_Edge E = TopoDS.Edge(itl1.Value());
                    if (!Emap.Add(E))
                    {
                        continue;
                    }
                    StoreInMVE(myFace, E, MVE, ref YaCouture, myVerticesForSubstitute, myTolConf);
                }
            }
        }

        // add const edges
        // Sewn edges can be doubled or not in myConstEdges
        // => call only once StoreInMVE which should double them
        NCollection_Map<TopoDS_Shape> DejaVu = new(TopTools_ShapeMapHasher.Instance);
        for (itl.Initialize(myConstEdges); itl.More(); itl.Next())
        {
            TopoDS_Edge E = TopoDS.Edge(itl.Value());
            if (DejaVu.Add(E))
            {
                StoreInMVE(myFace, E, MVE, ref YaCouture, myVerticesForSubstitute, myTolConf);
            }
        }

        //-----------------------------------------------
        // Construction of wires and new faces.
        //----------------------------------------------
        TopoDS_Vertex VF, CV;
        TopoDS_Edge CE, NE, EF;
        BRep_Builder B = new();
        TopoDS_Wire NW = new();
        bool End;

        UpdateVEmap(MVE);

        NCollection_Map<TopoDS_Shape> UsedEdges = new(TopTools_ShapeMapHasher.Instance);

        while (MVE.Extent() > 0)
        {
            B.MakeWire(ref NW);
            //--------------------------------
            // Removal of hanging edges.
            //--------------------------------
            RemovePendingEdges(MVE);

            if (MVE.Extent() == 0)
            {
                break;
            }
            //--------------------------------
            // Start edge.
            //--------------------------------
            EF = CE = TopoDS.Edge(MVE.FindFromIndex(1).First());
            TopExp.Vertices(CE, out V1, out V2);
            //--------------------------------
            // VF vertex start of new wire
            //--------------------------------
            if (CE.Orientation() == TopAbs_FORWARD)
            {
                CV = VF = V1;
            }
            else
            {
                CV = VF = V2;
            }
            if (!MVE.Contains(CV))
            {
                continue;
            }
            NCollection_List<TopoDS_Shape> aListEdges = MVE.ChangeFromKey(CV);
            for (itl.Initialize(aListEdges); itl.More(); itl.Next())
            {
                if (itl.Value().IsEqual(CE))
                {
                    aListEdges.Remove(itl);
                    break;
                }
            }
            End = false;

            while (!End)
            {
                //-------------------------------
                // Construction of a wire.
                //-------------------------------
                TopExp.Vertices(CE, out V1, out V2);
                if (!CV.IsSame(V1))
                {
                    CV = V1;
                }
                else
                {
                    CV = V2;
                }

                B.Add(NW, CE);
                UsedEdges.Add(CE);

                if (!MVE.Contains(CV) || MVE.FindFromKey(CV).IsEmpty())
                {
                    End = true;
                }
                else
                {
                    End = !SelectEdge(myFace, CE, CV, out NE, MVE.ChangeFromKey(CV));
                    if (!End)
                    {
                        CE = NE;
                        if (MVE.FindFromKey(CV).IsEmpty())
                        {
                            MVE.RemoveKey(CV);
                        }
                    }
                }
            }
            //--------------------------------------------------
            // Add new wire to the set of wires
            //------------------------------------------------

            if (VF.IsSame(CV))
            {
                if (SamePnt2d(VF, EF, CE, myFace))
                {
                    NW.Closed(true);
                    myNewWires.Append(NW);
                }
                else if (BRep_Tool.Tolerance(VF) < myTolConf)
                {
                    BRep_Builder aBB = new();
                    aBB.UpdateVertex(VF, myTolConf);
                    if (SamePnt2d(VF, EF, CE, myFace))
                    {
                        NW.Closed(true);
                        myNewWires.Append(NW);
                    }
                }
            }
        }
        PurgeNewEdges(myCutEdges, UsedEdges);
    }

    /// <summary>
    /// Cuts the edge <paramref name="E"/> at the vertices <paramref name="VOnE"/> (FORWARD for a start, REVERSED for an
    /// end) into the edges <paramref name="NE"/>, dropping pieces shorter than 0.001 in the face's parametric space.
    /// </summary>
    public void CutEdge(in TopoDS_Edge E, NCollection_List<TopoDS_Shape> VOnE, NCollection_List<TopoDS_Shape> NE)
    {
        TopoDS_Shape aLocalE = E.Oriented(TopAbs_FORWARD);
        TopoDS_Edge WE = TopoDS.Edge(aLocalE);

        double U1, U2;
        TopoDS_Vertex V1, V2;
        NCollection_Sequence<TopoDS_Shape> SV = new();
        NCollection_List<TopoDS_Shape>.Iterator it = new(VOnE);
        BRep_Builder B = new();

        for (; it.More(); it.Next())
        {
            SV.Append(it.Value());
        }
        //--------------------------------
        // Parse vertices on the edge.
        //--------------------------------
        Bubble(WE, SV);

        int NbVer = SV.Length();
        //----------------------------------------------------------------
        // Construction of new edges.
        // Note :  vertices at the extremities of edges are not
        //         onligatorily in the list of vertices
        //----------------------------------------------------------------
        if (SV.IsEmpty())
        {
            NE.Append(E);
            return;
        }
        BRep_Tool.Range(WE, out double f, out double l);
        TopExp.Vertices(WE, out TopoDS_Vertex VF, out TopoDS_Vertex VL);

        if (NbVer == 2)
        {
            if (SV.Value(1).IsEqual(VF) && SV.Value(2).IsEqual(VL))
            {
                NE.Append(E);
                return;
            }
        }
        //----------------------------------------------------
        // Processing of closed edges
        // If a vertex of intersection is on the common vertex
        // it should appear at the beginning and end of SV.
        //----------------------------------------------------
        TopoDS_Vertex VCEI;

        if (!VF.IsNull() && VF.IsSame(VL))
        {
            VCEI = UpdateClosedEdge(WE, SV);
            if (!VCEI.IsNull())
            {
                TopoDS_Shape aLocalV = VCEI.Oriented(TopAbs_FORWARD);
                VF = TopoDS.Vertex(aLocalV);
                aLocalV = VCEI.Oriented(TopAbs_REVERSED);
                VL = TopoDS.Vertex(aLocalV);
            }
            SV.Prepend(VF);
            SV.Append(VL);
        }
        else
        {
            //-----------------------------------------
            // Eventually all extremities of the edge.
            //-----------------------------------------
            if (!VF.IsNull() && !VF.IsSame(SV.First()))
            {
                SV.Prepend(VF);
            }
            if (!VL.IsNull() && !VL.IsSame(SV.Last()))
            {
                SV.Append(VL);
            }
        }

        while (!SV.IsEmpty())
        {
            while (!SV.IsEmpty() && SV.First().Orientation() != TopAbs_FORWARD)
            {
                SV.Remove(1);
            }
            if (SV.IsEmpty())
            {
                break;
            }
            V1 = TopoDS.Vertex(SV.First());
            SV.Remove(1);
            if (SV.IsEmpty())
            {
                break;
            }
            if (SV.First().Orientation() == TopAbs_REVERSED)
            {
                V2 = TopoDS.Vertex(SV.First());
                SV.Remove(1);
                //-------------------------------------------
                // Copy the edge and restriction by V1 V2.
                //-------------------------------------------
                TopoDS_Shape NewEdge = WE.EmptyCopied();
                TopoDS_Shape aLocalEdge = V1.Oriented(TopAbs_FORWARD);
                B.Add(NewEdge, aLocalEdge);
                aLocalEdge = V2.Oriented(TopAbs_REVERSED);
                B.Add(TopoDS.Edge(NewEdge), aLocalEdge);
                if (V1.IsSame(VF))
                {
                    U1 = f;
                }
                else
                {
                    TopoDS_Shape aLocalV = V1.Oriented(TopAbs_INTERNAL);
                    U1 = BRep_Tool.Parameter(TopoDS.Vertex(aLocalV), WE);
                }
                if (V2.IsSame(VL))
                {
                    U2 = l;
                }
                else
                {
                    TopoDS_Shape aLocalV = V2.Oriented(TopAbs_INTERNAL);
                    U2 = BRep_Tool.Parameter(TopoDS.Vertex(aLocalV), WE);
                }
                B.Range(TopoDS.Edge(NewEdge), U1, U2);
                NE.Append(NewEdge.Oriented(E.Orientation()));
            }
        }

        // Remove edges with size <= tolerance
        double Tol = 0.001; // 5.e-05; //5.e-07;
        it.Initialize(NE);
        while (it.More())
        {
            // skl : I change "E" to "EE"
            TopoDS_Edge EE = TopoDS.Edge(it.Value());
            BRep_Tool.Range(EE, out double fpar, out double lpar);
            if (lpar - fpar <= global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                NE.Remove(it);
            }
            else
            {
                BRep_Tool.UVPoints(EE, myFace, out Point2D pf, out Point2D pl);
                if (pf.Distance(pl) <= Tol && !BRep_Tool.IsClosed(EE))
                {
                    NE.Remove(it);
                }
                else
                {
                    it.Next();
                }
            }
        }
    }

    /// <summary>The wires built (possibly none).</summary>
    public NCollection_List<TopoDS_Shape> NewWires() => myNewWires;

    /// <summary>Builds the faces the wires bound on the face (<see cref="BRepAlgo_FaceRestrictor"/>).</summary>
    public void WiresToFaces()
    {
        if (!myNewWires.IsEmpty())
        {
            BRepAlgo_FaceRestrictor FR = new();
            TopoDS_Shape aLocalS = myFace.Oriented(TopAbs_FORWARD);
            FR.Init(TopoDS.Face(aLocalS), false);
            NCollection_List<TopoDS_Shape>.Iterator it = new(myNewWires);
            for (; it.More(); it.Next())
            {
                FR.Add(TopoDS.Wire(it.Value()));
            }

            FR.InternalPerform();

            if (FR.IsDone())
            {
                TopAbs_Orientation OriF = myFace.Orientation();
                for (; FR.More(); FR.Next())
                {
                    myNewFaces.Append(FR.InternalCurrent().Oriented(OriF));
                }
            }
        }
    }

    /// <summary>The faces built by <see cref="WiresToFaces"/> (possibly none).</summary>
    public NCollection_List<TopoDS_Shape> NewFaces() => myNewFaces;

    /// <summary>The edges cut from the edge <paramref name="E"/> and used in the wires (possibly none).</summary>
    public NCollection_List<TopoDS_Shape> NewEdges(in TopoDS_Edge E) => myCutEdges[E];

    /// <summary>Fills <paramref name="VerVerMap"/> with the vertices and their substitutes.</summary>
    public void GetVerticesForSubstitute(NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> VerVerMap)
    {
        VerVerMap.Clear();
        foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> anItem in myVerticesForSubstitute)
        {
            VerVerMap.Bind(anItem.Key, anItem.Value);
        }
    }

    /// <summary>Sets the vertices and their substitutes (a copy of <paramref name="VerVerMap"/>).</summary>
    public void VerticesForSubstitute(NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> VerVerMap)
    {
        myVerticesForSubstitute = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(VerVerMap);
    }

    /// <summary>
    /// Merges the vertices of single edges in <paramref name="theVEmap"/> that are images of the same vertex (the image
    /// set by <see cref="SetImageVV"/>) into one, then rebuilds the map.
    /// </summary>
    public void UpdateVEmap(NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theVEmap)
    {
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> VerLver = new(TopTools_ShapeMapHasher.Instance);

        for (int ii = 1; ii <= theVEmap.Extent(); ii++)
        {
            TopoDS_Vertex aVertex = TopoDS.Vertex(theVEmap.FindKey(ii));
            NCollection_List<TopoDS_Shape> aElist = theVEmap.FindFromIndex(ii);
            if (aElist.Extent() == 1 && myImageVV.IsImage(aVertex))
            {
                TopoDS_Vertex aProVertex = TopoDS.Vertex(myImageVV.ImageFrom(aVertex));
                if (VerLver.Contains(aProVertex))
                {
                    NCollection_List<TopoDS_Shape> aVlist = VerLver.ChangeFromKey(aProVertex);
                    aVlist.Append(aVertex.Oriented(TopAbs_FORWARD));
                }
                else
                {
                    NCollection_List<TopoDS_Shape> aVlist = new();
                    aVlist.Append(aVertex.Oriented(TopAbs_FORWARD));
                    VerLver.Add(aProVertex, aVlist);
                }
            }
        }

        if (VerLver.IsEmpty())
        {
            return;
        }

        BRep_Builder aBB = new();
        for (int ii = 1; ii <= VerLver.Extent(); ii++)
        {
            NCollection_List<TopoDS_Shape> aVlist = VerLver.FindFromIndex(ii);
            if (aVlist.Extent() == 1)
            {
                continue;
            }

            double aMaxTol = 0.0;
            NCollection_Array1<Point3D> Points = new(1, aVlist.Extent());

            NCollection_List<TopoDS_Shape>.Iterator itl = new(aVlist);
            int jj = 0;
            for (; itl.More(); itl.Next())
            {
                TopoDS_Vertex aVertex = TopoDS.Vertex(itl.Value());
                double aTol = BRep_Tool.Tolerance(aVertex);
                aMaxTol = Max(aMaxTol, aTol);
                Point3D aPnt = BRep_Tool.Pnt(aVertex);
                Points[++jj] = aPnt;
            }

            GeomLib.AxeOfInertia(Points, out Frame3D anAxis, out _);
            Point3D aCentre = anAxis.Location();
            double aMaxDist = 0.0;
            for (jj = 1; jj <= Points.Upper(); jj++)
            {
                double aSqDist = aCentre.SquareDistance(Points[jj]);
                aMaxDist = Max(aMaxDist, aSqDist);
            }
            aMaxDist = Math.Sqrt(aMaxDist);
            aMaxTol = Max(aMaxTol, aMaxDist);

            // Find constant vertex
            TopoDS_Vertex aConstVertex = new();
            for (itl.Initialize(aVlist); itl.More(); itl.Next())
            {
                TopoDS_Vertex aVertex = TopoDS.Vertex(itl.Value());
                NCollection_List<TopoDS_Shape> aElist = theVEmap.FindFromKey(aVertex);
                TopoDS_Shape anEdge = aElist.First();
                NCollection_List<TopoDS_Shape>.Iterator itcedges = new(myConstEdges);
                for (; itcedges.More(); itcedges.Next())
                {
                    if (anEdge.IsSame(itcedges.Value()))
                    {
                        aConstVertex = aVertex;
                        break;
                    }
                }
                if (!aConstVertex.IsNull())
                {
                    break;
                }
            }
            if (aConstVertex.IsNull())
            {
                aConstVertex = TopoDS.Vertex(aVlist.First());
            }
            aBB.UpdateVertex(aConstVertex, aCentre, aMaxTol);

            for (itl.Initialize(aVlist); itl.More(); itl.Next())
            {
                TopoDS_Vertex aVertex = TopoDS.Vertex(itl.Value());
                if (aVertex.IsSame(aConstVertex))
                {
                    continue;
                }

                NCollection_List<TopoDS_Shape> aElist = theVEmap.FindFromKey(aVertex);
                TopoDS_Edge anEdge = TopoDS.Edge(aElist.First());
                anEdge.Orientation(TopAbs_FORWARD);
                TopExp.Vertices(anEdge, out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
                TopoDS_Vertex aVertexToRemove = aV1.IsSame(aVertex) ? aV1 : aV2;
                anEdge.Free(true);
                aBB.Remove(anEdge, aVertexToRemove);
                aBB.Add(anEdge, aConstVertex.Oriented(aVertexToRemove.Orientation()));
            }
        }

        NCollection_IndexedMap<TopoDS_Shape> Emap = new(TopTools_ShapeMapHasher.Instance);
        for (int ii = 1; ii <= theVEmap.Extent(); ii++)
        {
            NCollection_List<TopoDS_Shape> aElist = theVEmap.FindFromIndex(ii);
            NCollection_List<TopoDS_Shape>.Iterator itl = new(aElist);
            for (; itl.More(); itl.Next())
            {
                Emap.Add(itl.Value());
            }
        }

        theVEmap.Clear();
        for (int ii = 1; ii <= Emap.Extent(); ii++)
        {
            TopExp.MapShapesAndAncestors(Emap.FindKey(ii), TopAbs_VERTEX, TopAbs_EDGE, theVEmap);
        }
    }

    /// <summary>The largest distance between vertices that are merged.</summary>
    public double GetTolConf() => myTolConf;

    /// <summary>Sets the largest distance between vertices that are merged.</summary>
    public void SetTolConf(double theTolConf)
    {
        myTolConf = theTolConf;
    }
}
