// Port of: src/ModelingAlgorithms/TKBool/BRepAlgo/BRepAlgo_Loop.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepAlgo/BRepAlgo_Loop.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

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
    private Face myFace = new();
    private readonly NCollection_List<Shape> myConstEdges = new();
    private readonly NCollection_List<Shape> myEdges = new();
    private readonly NCollection_DataMap<Shape, NCollection_List<Shape>> myVerOnEdges = new(ShapeHasher.Instance);
    private readonly NCollection_List<Shape> myNewWires = new();
    private readonly NCollection_List<Shape> myNewFaces = new();
    private readonly NCollection_DataMap<Shape, NCollection_List<Shape>> myCutEdges = new(ShapeHasher.Instance);
    private NCollection_DataMap<Shape, Shape> myVerticesForSubstitute = new(ShapeHasher.Instance);
    private readonly BRepAlgo_Image myImageVV = new();
    private double myTolConf;

    /// <summary>An empty loop builder.</summary>
    public BRepAlgo_Loop()
    {
        myTolConf = 0.001;
    }

    /// <summary>Starts again on the face <paramref name="F"/>; the edges must have pcurves on it.</summary>
    public void Init(in Face F)
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
    private static void Bubble(in Edge E, NCollection_Sequence<Shape> Seq)
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
        Vertex V1, V2;

        while (Invert)
        {
            Invert = false;
            for (int i = 1; i < NbPoints; i++)
            {
                Shape aLocalV = Seq.Value(i).Oriented(Internal);
                V1 = ShapeCasts.Vertex(aLocalV);
                aLocalV = Seq.Value(i + 1).Oriented(Internal);
                V2 = ShapeCasts.Vertex(aLocalV);

                U1 = Tool.Parameter(V1, E);
                U2 = Tool.Parameter(V2, E);
                if (U2 < U1)
                {
                    Seq.Exchange(i, i + 1);
                    Invert = true;
                }
            }
        }
    }

    /// <summary>Adds the edge <paramref name="E"/>, to be cut at the vertices <paramref name="LV"/>.</summary>
    public void AddEdge(in Edge E, NCollection_List<Shape> LV)
    {
        myEdges.Append(E);
        myVerOnEdges.Bind(E, new NCollection_List<Shape>(LV));
    }

    /// <summary>Adds the constant edge <paramref name="E"/>, which can be in the result as it is.</summary>
    public void AddConstEdge(in Edge E)
    {
        myConstEdges.Append(E);
    }

    /// <summary>Adds the constant edges <paramref name="LE"/>.</summary>
    public void AddConstEdges(NCollection_List<Shape> LE)
    {
        NCollection_List<Shape>.Iterator itl = new(LE);
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
    private static Vertex UpdateClosedEdge(in Edge E, NCollection_Sequence<Shape> SV)
    {
        Vertex[] VB = [new(), new()];
        Vertex VRes = new();
        Point3D P, PC;
        bool OnStart = false, OnEnd = false;
        //// modified by jgv, 13.04.04 for OCC5634 ////
        Exploration.Vertices(E, out Vertex V1, out _);
        double Tol = Tool.Tolerance(V1);
        ///////////////////////////////////////////////

        if (SV.IsEmpty())
        {
            return VRes;
        }

        VB[0] = ShapeCasts.Vertex(SV.First());
        VB[1] = ShapeCasts.Vertex(SV.Last());
        PC = Tool.Pnt(V1);

        for (int i = 0; i < 2; i++)
        {
            P = Tool.Pnt(VB[i]);
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
    private static void RemovePendingEdges(NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> MVE)
    {
        //--------------------------------
        // Remove hanging edges.
        //--------------------------------
        NCollection_List<Shape>.Iterator itl = new();
        bool YaSupress = true;

        while (YaSupress)
        {
            YaSupress = false;
            NCollection_List<Shape> VToRemove = new();
            NCollection_Map<Shape> EToRemove = new(ShapeHasher.Instance);

            for (int iV = 1; iV <= MVE.Extent(); iV++)
            {
                Shape aVertex = MVE.FindKey(iV);
                NCollection_List<Shape> anEdges = MVE.FindFromIndex(iV);
                if (anEdges.IsEmpty())
                {
                    VToRemove.Append(aVertex);
                }
                if (anEdges.Extent() == 1)
                {
                    Edge E = ShapeCasts.Edge(anEdges.First());
                    Exploration.Vertices(E, out Vertex V1, out Vertex V2);
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
                        NCollection_List<Shape> LE = MVE.ChangeFromIndex(iV);
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
    private static bool SamePnt2d(in Vertex V, in Edge E1, in Edge E2, in Face F)
    {
        Point2D P1, P2;
        Shape aLocalF = F.Oriented(Forward);
        Face FF = ShapeCasts.Face(aLocalF);
        Curve2D C1 = Tool.CurveOnSurface(E1, FF, out double f1, out double l1)!;
        Curve2D C2 = Tool.CurveOnSurface(E2, FF, out double f2, out double l2)!;
        if (E1.Orientation() == Forward)
        {
            P1 = C1.Value(f1);
        }
        else
        {
            P1 = C1.Value(l1);
        }

        if (E2.Orientation() == Forward)
        {
            P2 = C2.Value(l2);
        }
        else
        {
            P2 = C2.Value(f2);
        }
        double Tol = 100 * Tool.Tolerance(V);
        double Dist = P1.Distance(P2);
        return Dist < Tol;
    }

    // Finds the edge NE connected to CE by the vertex CV in LE and removes it from LE; also removes CE from LE if it is
    // there with the same orientation
    private static bool SelectEdge(in Face F,
                                   in Edge CE,
                                   in Vertex CV,
                                   out Edge NE,
                                   NCollection_List<Shape> LE)
    {
        NCollection_List<Shape>.Iterator itl = new();
        NE = new Edge();

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
            Face FForward = F;
            FForward.Orientation(Forward);

            Curve2D C = Tool.CurveOnSurface(CE, FForward, out double f, out double l)!;
            int k = 1, kmin = 0;
            double dist, distmin = 100 * Tool.Tolerance(CV);
            double u;
            if (CE.Orientation() == Forward)
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
                Edge E = ShapeCasts.Edge(itl.Value());
                if (!E.IsSame(CE))
                {
                    C = Tool.CurveOnSurface(E, FForward, out f, out l)!;
                    if (E.Orientation() == Forward)
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
            NE = ShapeCasts.Edge(itl.Value());
            LE.Remove(itl);
        }
        else if (LE.Extent() == 1)
        {
            NE = ShapeCasts.Edge(LE.First());
            LE.RemoveFirst();
        }
        else
        {
            return false;
        }
        return true;
    }

    // Keeps in the lists of new edges only the edges used in the wires
    private static void PurgeNewEdges(NCollection_DataMap<Shape, NCollection_List<Shape>> NewEdges,
                                      NCollection_Map<Shape> UsedEdges)
    {
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> it in NewEdges)
        {
            NCollection_List<Shape> LNE = it.Value;
            NCollection_List<Shape>.Iterator itL = new(LNE);

            while (itL.More())
            {
                Shape NE = itL.Value();
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
    private static void StoreInMVE(in Face F,
                                   in Edge E,
                                   NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> MVE,
                                   ref bool YaCouture,
                                   NCollection_DataMap<Shape, Shape> VerticesForSubstitute,
                                   double theTolConf)
    {
        Vertex V1, V2, V;
        Point3D P1, P;
        BrepBuilder BB = new();
        for (int iV = 1; iV <= MVE.Extent(); iV++)
        {
            V = ShapeCasts.Vertex(MVE.FindKey(iV));
            P = Tool.Pnt(V);
            NCollection_List<Shape> VList = new();
            Iterator VerExp = new(E);
            for (; VerExp.More(); VerExp.Next())
            {
                VList.Append(VerExp.Value());
            }
            NCollection_List<Shape>.Iterator itl = new(VList);
            for (; itl.More(); itl.Next())
            {
                V1 = ShapeCasts.Vertex(itl.Value());
                P1 = Tool.Pnt(V1);
                if (P.IsEqual(P1, theTolConf) && !V.IsSame(V1))
                {
                    V.Orientation(V1.Orientation());
                    if (VerticesForSubstitute.IsBound(V1))
                    {
                        Shape OldNewV = VerticesForSubstitute[V1];
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
                            Shape NewNewV = VerticesForSubstitute[V];
                            if (!NewNewV.IsSame(V1))
                            {
                                VerticesForSubstitute.Bind(V1, NewNewV);
                            }
                        }
                        else
                        {
                            VerticesForSubstitute.Bind(V1, V);
                            // OCCT changes the map's values while iterating it: the keys are collected first
                            NCollection_List<Shape> aKeys = new();
                            foreach (KeyValuePair<Shape, Shape> mapit in VerticesForSubstitute)
                            {
                                if (mapit.Value.IsSame(V1))
                                {
                                    aKeys.Append(mapit.Key);
                                }
                            }
                            for (NCollection_List<Shape>.Iterator itk = new(aKeys); itk.More(); itk.Next())
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

        Exploration.Vertices(E, out V1, out V2);
        if (V1.IsNull && V2.IsNull)
        {
            YaCouture = false;
            return;
        }
        if (!MVE.Contains(V1))
        {
            MVE.Add(V1, new NCollection_List<Shape>());
        }
        MVE.ChangeFromKey(V1).Append(E);
        if (!V1.IsSame(V2))
        {
            if (!MVE.Contains(V2))
            {
                MVE.Add(V2, new NCollection_List<Shape>());
            }
            MVE.ChangeFromKey(V2).Append(E);
        }
        Surface? S = Tool.Surface(F, out Placement L);
        if (Tool.IsClosed(E, S, L))
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
        NCollection_List<Shape>.Iterator itl = new(), itl1 = new();
        Vertex V1, V2;
        bool YaCouture = false;

        //------------------------------------------------
        // Cut edges
        //------------------------------------------------
        for (itl.Initialize(myEdges); itl.More(); itl.Next())
        {
            Edge anEdge = ShapeCasts.Edge(itl.Value());
            NCollection_List<Shape> LCE = new();
            if (myVerOnEdges.Find(anEdge, out NCollection_List<Shape> pVertices))
            {
                CutEdge(anEdge, pVertices, LCE);
                myCutEdges.Bind(anEdge, LCE);
            }
        }
        //-----------------------------------
        // Construction map vertex => edges
        //-----------------------------------
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> MVE = new(ShapeHasher.Instance);

        // add cut edges.
        NCollection_Map<Shape> Emap = new(ShapeHasher.Instance);
        for (itl.Initialize(myEdges); itl.More(); itl.Next())
        {
            if (myCutEdges.Find(itl.Value(), out NCollection_List<Shape> pLCE))
            {
                for (itl1.Initialize(pLCE); itl1.More(); itl1.Next())
                {
                    Edge E = ShapeCasts.Edge(itl1.Value());
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
        NCollection_Map<Shape> DejaVu = new(ShapeHasher.Instance);
        for (itl.Initialize(myConstEdges); itl.More(); itl.Next())
        {
            Edge E = ShapeCasts.Edge(itl.Value());
            if (DejaVu.Add(E))
            {
                StoreInMVE(myFace, E, MVE, ref YaCouture, myVerticesForSubstitute, myTolConf);
            }
        }

        //-----------------------------------------------
        // Construction of wires and new faces.
        //----------------------------------------------
        Vertex VF, CV;
        Edge CE, NE, EF;
        BrepBuilder B = new();
        Wire NW = new();
        bool End;

        UpdateVEmap(MVE);

        NCollection_Map<Shape> UsedEdges = new(ShapeHasher.Instance);

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
            EF = CE = ShapeCasts.Edge(MVE.FindFromIndex(1).First());
            Exploration.Vertices(CE, out V1, out V2);
            //--------------------------------
            // VF vertex start of new wire
            //--------------------------------
            if (CE.Orientation() == Forward)
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
            NCollection_List<Shape> aListEdges = MVE.ChangeFromKey(CV);
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
                Exploration.Vertices(CE, out V1, out V2);
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
                else if (Tool.Tolerance(VF) < myTolConf)
                {
                    BrepBuilder aBB = new();
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
    public void CutEdge(in Edge E, NCollection_List<Shape> VOnE, NCollection_List<Shape> NE)
    {
        Shape aLocalE = E.Oriented(Forward);
        Edge WE = ShapeCasts.Edge(aLocalE);

        double U1, U2;
        Vertex V1, V2;
        NCollection_Sequence<Shape> SV = new();
        NCollection_List<Shape>.Iterator it = new(VOnE);
        BrepBuilder B = new();

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
        Tool.Range(WE, out double f, out double l);
        Exploration.Vertices(WE, out Vertex VF, out Vertex VL);

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
        Vertex VCEI;

        if (!VF.IsNull && VF.IsSame(VL))
        {
            VCEI = UpdateClosedEdge(WE, SV);
            if (!VCEI.IsNull)
            {
                Shape aLocalV = VCEI.Oriented(Forward);
                VF = ShapeCasts.Vertex(aLocalV);
                aLocalV = VCEI.Oriented(Reversed);
                VL = ShapeCasts.Vertex(aLocalV);
            }
            SV.Prepend(VF);
            SV.Append(VL);
        }
        else
        {
            //-----------------------------------------
            // Eventually all extremities of the edge.
            //-----------------------------------------
            if (!VF.IsNull && !VF.IsSame(SV.First()))
            {
                SV.Prepend(VF);
            }
            if (!VL.IsNull && !VL.IsSame(SV.Last()))
            {
                SV.Append(VL);
            }
        }

        while (!SV.IsEmpty())
        {
            while (!SV.IsEmpty() && SV.First().Orientation() != Forward)
            {
                SV.Remove(1);
            }
            if (SV.IsEmpty())
            {
                break;
            }
            V1 = ShapeCasts.Vertex(SV.First());
            SV.Remove(1);
            if (SV.IsEmpty())
            {
                break;
            }
            if (SV.First().Orientation() == Reversed)
            {
                V2 = ShapeCasts.Vertex(SV.First());
                SV.Remove(1);
                //-------------------------------------------
                // Copy the edge and restriction by V1 V2.
                //-------------------------------------------
                Shape NewEdge = WE.EmptyCopied();
                Shape aLocalEdge = V1.Oriented(Forward);
                B.Add(NewEdge, aLocalEdge);
                aLocalEdge = V2.Oriented(Reversed);
                B.Add(ShapeCasts.Edge(NewEdge), aLocalEdge);
                if (V1.IsSame(VF))
                {
                    U1 = f;
                }
                else
                {
                    Shape aLocalV = V1.Oriented(Internal);
                    U1 = Tool.Parameter(ShapeCasts.Vertex(aLocalV), WE);
                }
                if (V2.IsSame(VL))
                {
                    U2 = l;
                }
                else
                {
                    Shape aLocalV = V2.Oriented(Internal);
                    U2 = Tool.Parameter(ShapeCasts.Vertex(aLocalV), WE);
                }
                B.Range(ShapeCasts.Edge(NewEdge), U1, U2);
                NE.Append(NewEdge.Oriented(E.Orientation()));
            }
        }

        // Remove edges with size <= tolerance
        double Tol = 0.001; // 5.e-05; //5.e-07;
        it.Initialize(NE);
        while (it.More())
        {
            // skl : I change "E" to "EE"
            Edge EE = ShapeCasts.Edge(it.Value());
            Tool.Range(EE, out double fpar, out double lpar);
            if (lpar - fpar <= global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                NE.Remove(it);
            }
            else
            {
                Tool.UVPoints(EE, myFace, out Point2D pf, out Point2D pl);
                if (pf.Distance(pl) <= Tol && !Tool.IsClosed(EE))
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
    public NCollection_List<Shape> NewWires() => myNewWires;

    /// <summary>Builds the faces the wires bound on the face (<see cref="BRepAlgo_FaceRestrictor"/>).</summary>
    public void WiresToFaces()
    {
        if (!myNewWires.IsEmpty())
        {
            BRepAlgo_FaceRestrictor FR = new();
            Shape aLocalS = myFace.Oriented(Forward);
            FR.Init(ShapeCasts.Face(aLocalS), false);
            NCollection_List<Shape>.Iterator it = new(myNewWires);
            for (; it.More(); it.Next())
            {
                FR.Add(ShapeCasts.Wire(it.Value()));
            }

            FR.InternalPerform();

            if (FR.IsDone())
            {
                Orientation OriF = myFace.Orientation();
                for (; FR.More(); FR.Next())
                {
                    myNewFaces.Append(FR.InternalCurrent().Oriented(OriF));
                }
            }
        }
    }

    /// <summary>The faces built by <see cref="WiresToFaces"/> (possibly none).</summary>
    public NCollection_List<Shape> NewFaces() => myNewFaces;

    /// <summary>The edges cut from the edge <paramref name="E"/> and used in the wires (possibly none).</summary>
    public NCollection_List<Shape> NewEdges(in Edge E) => myCutEdges[E];

    /// <summary>Fills <paramref name="VerVerMap"/> with the vertices and their substitutes.</summary>
    public void GetVerticesForSubstitute(NCollection_DataMap<Shape, Shape> VerVerMap)
    {
        VerVerMap.Clear();
        foreach (KeyValuePair<Shape, Shape> anItem in myVerticesForSubstitute)
        {
            VerVerMap.Bind(anItem.Key, anItem.Value);
        }
    }

    /// <summary>Sets the vertices and their substitutes (a copy of <paramref name="VerVerMap"/>).</summary>
    public void VerticesForSubstitute(NCollection_DataMap<Shape, Shape> VerVerMap)
    {
        myVerticesForSubstitute = new NCollection_DataMap<Shape, Shape>(VerVerMap);
    }

    /// <summary>
    /// Merges the vertices of single edges in <paramref name="theVEmap"/> that are images of the same vertex (the image
    /// set by <see cref="SetImageVV"/>) into one, then rebuilds the map.
    /// </summary>
    public void UpdateVEmap(NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theVEmap)
    {
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> VerLver = new(ShapeHasher.Instance);

        for (int ii = 1; ii <= theVEmap.Extent(); ii++)
        {
            Vertex aVertex = ShapeCasts.Vertex(theVEmap.FindKey(ii));
            NCollection_List<Shape> aElist = theVEmap.FindFromIndex(ii);
            if (aElist.Extent() == 1 && myImageVV.IsImage(aVertex))
            {
                Vertex aProVertex = ShapeCasts.Vertex(myImageVV.ImageFrom(aVertex));
                if (VerLver.Contains(aProVertex))
                {
                    NCollection_List<Shape> aVlist = VerLver.ChangeFromKey(aProVertex);
                    aVlist.Append(aVertex.Oriented(Forward));
                }
                else
                {
                    NCollection_List<Shape> aVlist = new();
                    aVlist.Append(aVertex.Oriented(Forward));
                    VerLver.Add(aProVertex, aVlist);
                }
            }
        }

        if (VerLver.IsEmpty())
        {
            return;
        }

        BrepBuilder aBB = new();
        for (int ii = 1; ii <= VerLver.Extent(); ii++)
        {
            NCollection_List<Shape> aVlist = VerLver.FindFromIndex(ii);
            if (aVlist.Extent() == 1)
            {
                continue;
            }

            double aMaxTol = 0.0;
            NCollection_Array1<Point3D> Points = new(1, aVlist.Extent());

            NCollection_List<Shape>.Iterator itl = new(aVlist);
            int jj = 0;
            for (; itl.More(); itl.Next())
            {
                Vertex aVertex = ShapeCasts.Vertex(itl.Value());
                double aTol = Tool.Tolerance(aVertex);
                aMaxTol = Max(aMaxTol, aTol);
                Point3D aPnt = Tool.Pnt(aVertex);
                Points[++jj] = aPnt;
            }

            global::Kerf.Engine.Geometry.Library.GeomLib.AxeOfInertia(Points, out Frame3D anAxis, out _);
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
            Vertex aConstVertex = new();
            for (itl.Initialize(aVlist); itl.More(); itl.Next())
            {
                Vertex aVertex = ShapeCasts.Vertex(itl.Value());
                NCollection_List<Shape> aElist = theVEmap.FindFromKey(aVertex);
                Shape anEdge = aElist.First();
                NCollection_List<Shape>.Iterator itcedges = new(myConstEdges);
                for (; itcedges.More(); itcedges.Next())
                {
                    if (anEdge.IsSame(itcedges.Value()))
                    {
                        aConstVertex = aVertex;
                        break;
                    }
                }
                if (!aConstVertex.IsNull)
                {
                    break;
                }
            }
            if (aConstVertex.IsNull)
            {
                aConstVertex = ShapeCasts.Vertex(aVlist.First());
            }
            aBB.UpdateVertex(aConstVertex, aCentre, aMaxTol);

            for (itl.Initialize(aVlist); itl.More(); itl.Next())
            {
                Vertex aVertex = ShapeCasts.Vertex(itl.Value());
                if (aVertex.IsSame(aConstVertex))
                {
                    continue;
                }

                NCollection_List<Shape> aElist = theVEmap.FindFromKey(aVertex);
                Edge anEdge = ShapeCasts.Edge(aElist.First());
                anEdge.Orientation(Forward);
                Exploration.Vertices(anEdge, out Vertex aV1, out Vertex aV2);
                Vertex aVertexToRemove = aV1.IsSame(aVertex) ? aV1 : aV2;
                anEdge.Free(true);
                aBB.Remove(anEdge, aVertexToRemove);
                aBB.Add(anEdge, aConstVertex.Oriented(aVertexToRemove.Orientation()));
            }
        }

        NCollection_IndexedMap<Shape> Emap = new(ShapeHasher.Instance);
        for (int ii = 1; ii <= theVEmap.Extent(); ii++)
        {
            NCollection_List<Shape> aElist = theVEmap.FindFromIndex(ii);
            NCollection_List<Shape>.Iterator itl = new(aElist);
            for (; itl.More(); itl.Next())
            {
                Emap.Add(itl.Value());
            }
        }

        theVEmap.Clear();
        for (int ii = 1; ii <= Emap.Extent(); ii++)
        {
            Exploration.MapShapesAndAncestors(Emap.FindKey(ii), ShapeType.Vertex, ShapeType.Edge, theVEmap);
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
