// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeWire.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeWire.cxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeWire_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BRepLib_WireError;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Makes a wire from edges, added one at a time or as a list. An edge must share a vertex with the wire, or have a
/// vertex within the tolerance of one of the wire's (the edge is then copied onto the wire's vertex, which is moved
/// and enlarged to cover both); a list is connected whatever its order, coincident vertices merged.
/// </summary>
/// <remarks>
/// <para><see cref="Wire"/>, <see cref="Edge"/> and <see cref="Vertex"/> freeze the wire (design 4.2); ported code
/// reads the internal forms. Adding after a public read goes on with a new wire holding the same edges.</para>
/// <para>The caller's edges and vertices are frozen. Merging a vertex onto a wire vertex that must move or grow writes
/// into that vertex, so when it is frozen the maker first copies the wire built so far for write (design 4.6) and
/// goes on with the copy; <see cref="InternalHistory"/> then records the caller's edges and vertices as modified
/// into the copies. Writes that would change nothing are skipped.</para>
/// </remarks>
public class BRepLib_MakeWire : BRepLib_MakeShape
{
    private BRepLib_WireError myError;
    private TopoDS_Edge myEdge = new();
    private TopoDS_Vertex myVertex = new();
    private readonly NCollection_IndexedMap<TopoDS_Shape> myVertices = new(TopTools_ShapeMapHasher.Instance);
    private TopoDS_Vertex FirstVertex = new();
    private TopoDS_Vertex VF = new();
    private TopoDS_Vertex VL = new();
    // OCCT.NET: the history of the copies for write made so far (null if none).
    private BRepTools_History? myHistory;

    /// <summary>Makes an empty wire: <see cref="Error"/> is BRepLib_EmptyWire until an edge is added.</summary>
    public BRepLib_MakeWire()
    {
        myError = BRepLib_EmptyWire;
    }

    public BRepLib_MakeWire(in TopoDS_Edge E)
    {
        Add(E);
    }

    public BRepLib_MakeWire(in TopoDS_Edge E1, in TopoDS_Edge E2)
    {
        Add(E1);
        Add(E2);
    }

    public BRepLib_MakeWire(in TopoDS_Edge E1, in TopoDS_Edge E2, in TopoDS_Edge E3)
    {
        Add(E1);
        Add(E2);
        Add(E3);
    }

    public BRepLib_MakeWire(in TopoDS_Edge E1, in TopoDS_Edge E2, in TopoDS_Edge E3, in TopoDS_Edge E4)
    {
        Add(E1);
        Add(E2);
        Add(E3);
        Add(E4);
    }

    public BRepLib_MakeWire(in TopoDS_Wire W)
    {
        Add(W);
    }

    public BRepLib_MakeWire(in TopoDS_Wire W, in TopoDS_Edge E)
    {
        Add(W);
        Add(E);
    }

    /// <summary>Adds the edges of a wire, stopping at the first that can't be added.</summary>
    public void Add(in TopoDS_Wire W)
    {
        for (var it = new TopoDS_Iterator(W); it.More(); it.Next())
        {
            Add(TopoDS.Edge(it.Value()));
            if (myError != BRepLib_WireDone)
            {
                break;
            }
        }
    }

    /// <summary>
    /// Adds an edge connected to the wire by a shared vertex or a vertex within tolerance (the edge is then copied
    /// onto the wire's vertex).
    /// </summary>
    public void Add(in TopoDS_Edge E)
    {
        Add(E, true);
    }

    // PMN  19/03/1998  For the Problem of performance TopExp::Vertices are not used on wire
    // PMN  10/09/1998  In case if the wire is previously closed (or degenerated)
    //                  TopExp::Vertices is used to reduce the ambiguity.
    // IsCheckGeometryProximity flag : If true => check for the geometry proximity of vertices
    private void Add(in TopoDS_Edge E, bool IsCheckGeometryProximity)
    {
        bool forward = false;
        // to tell if it has been decided to add forward
        bool reverse = false;
        // to tell if it has been decided to add reversed
        bool init = false;
        // To know if it is necessary to calculate VL, VF
        var B = new BRep_Builder();
        var it = new TopoDS_Iterator();

        if (myEdge.IsNull())
        {
            init = true;
            // first edge, create the wire
            TopoDS_Wire aW = new();
            B.MakeWire(ref aW);
            myShape = aW;

            // set the edge
            myEdge = E;

            // add the vertices
            for (it.Initialize(myEdge); it.More(); it.Next())
            {
                myVertices.Add(it.Value());
            }
        }

        else
        {
            UnfreezeResult(); // OCCT.NET: frozen
            init = myShape.Closed(); // If it is closed no control
            TopoDS_Shape aLocalShape = E.Oriented(TopAbs_FORWARD);
            TopoDS_Edge EE = TopoDS.Edge(aLocalShape);

            // test the vertices of the edge

            bool connected = false;
            bool copyedge = false;

            if (myError != BRepLib_NonManifoldWire)
            {
                if (VF.IsNull() || VL.IsNull())
                {
                    myError = BRepLib_NonManifoldWire;
                }
            }

            for (it.Initialize(EE); it.More(); it.Next())
            {
                TopoDS_Vertex VE = TopoDS.Vertex(it.Value());

                // if the vertex is in the wire, ok for the connection
                if (myVertices.Contains(VE))
                {
                    connected = true;
                    myVertex = VE;
                    if (myError != BRepLib_NonManifoldWire)
                    {
                        // is it always so ?
                        if (VF.IsSame(VL))
                        {
                            // Orientation indetermined (in 3d) : Preserve the initial
                            if (!VF.IsSame(VE))
                            {
                                myError = BRepLib_NonManifoldWire;
                            }
                        }
                        else
                        {
                            if (VF.IsSame(VE))
                            {
                                if (VE.Orientation() == TopAbs_FORWARD)
                                {
                                    reverse = true;
                                }
                                else
                                {
                                    forward = true;
                                }
                            }
                            else if (VL.IsSame(VE))
                            {
                                if (VE.Orientation() == TopAbs_REVERSED)
                                {
                                    reverse = true;
                                }
                                else
                                {
                                    forward = true;
                                }
                            }
                            else
                            {
                                myError = BRepLib_NonManifoldWire;
                            }
                        }
                    }
                }
                else if (IsCheckGeometryProximity)
                {
                    // search if there is a similar vertex in the edge
                    gp_Pnt PE = BRep_Tool.Pnt(VE);

                    for (int i = 1; i <= myVertices.Extent(); i++)
                    {
                        TopoDS_Vertex VW = TopoDS.Vertex(myVertices.FindKey(i));
                        gp_Pnt PW = BRep_Tool.Pnt(VW);
                        double l = PE.Distance(PW);

                        if ((l < BRep_Tool.Tolerance(VE)) || (l < BRep_Tool.Tolerance(VW)))
                        {
                            copyedge = true;
                            if (myError != BRepLib_NonManifoldWire)
                            {
                                // is it always so ?
                                if (VF.IsSame(VL))
                                {
                                    // Orientation indetermined (in 3d) : Preserve the initial
                                    if (!VF.IsSame(VW))
                                    {
                                        myError = BRepLib_NonManifoldWire;
                                    }
                                }
                                else
                                {
                                    if (VF.IsSame(VW))
                                    {
                                        if (VE.Orientation() == TopAbs_FORWARD)
                                        {
                                            reverse = true;
                                        }
                                        else
                                        {
                                            forward = true;
                                        }
                                    }
                                    else if (VL.IsSame(VW))
                                    {
                                        if (VE.Orientation() == TopAbs_REVERSED)
                                        {
                                            reverse = true;
                                        }
                                        else
                                        {
                                            forward = true;
                                        }
                                    }
                                    else
                                    {
                                        myError = BRepLib_NonManifoldWire;
                                    }
                                }
                            }
                            break;
                        }
                    }
                    if (copyedge)
                    {
                        connected = true;
                    }
                }
            }

            if (!connected)
            {
                myError = BRepLib_DisconnectedWire;
                NotDone();
                return;
            }
            else
            {
                if (!copyedge)
                {
                    myEdge = EE;
                    for (it.Initialize(EE); it.More(); it.Next())
                    {
                        myVertices.Add(it.Value());
                    }
                }
                else
                {
                    // OCCT.NET: frozen. Copy the wire built so far for write if a frozen wire vertex must move or grow.
                    if (needsWritableVertices(EE))
                    {
                        copyForWrite();
                    }

                    // copy the edge
                    TopoDS_Shape Dummy = EE.EmptyCopied();
                    myEdge = TopoDS.Edge(Dummy);

                    for (it.Initialize(EE); it.More(); it.Next())
                    {
                        TopoDS_Vertex VE = TopoDS.Vertex(it.Value());
                        gp_Pnt PE = BRep_Tool.Pnt(VE);

                        bool newvertex = false;
                        for (int i = 1; i <= myVertices.Extent(); i++)
                        {
                            TopoDS_Vertex VW = TopoDS.Vertex(myVertices.FindKey(i));
                            gp_Pnt PW = BRep_Tool.Pnt(VW);
                            double l = PE.Distance(PW), tolE, tolW;
                            tolW = BRep_Tool.Tolerance(VW);
                            tolE = BRep_Tool.Tolerance(VE);

                            if ((l < tolE) || (l < tolW))
                            {
                                mergedVertex(PW, tolW, PE, tolE, l, out gp_Pnt PC, out double maxtol);

                                // OCCT.NET: frozen. A frozen VW here is one the update leaves as it is
                                // (needsWritableVertices made the others writable).
                                if (!VW.TShape()!.IsFrozen())
                                {
                                    B.UpdateVertex(VW, PC, maxtol);
                                }

                                newvertex = true;
                                myVertex = VW;
                                myVertex.Orientation(VE.Orientation());
                                B.Add(myEdge, myVertex);
                                transfert(B, EE, myEdge, VE, myVertex);
                                break;
                            }
                        }
                        if (!newvertex)
                        {
                            myVertices.Add(VE);
                            B.Add(myEdge, VE);
                            transfert(B, EE, myEdge, VE, VE);
                        }
                    }
                }
            }
            // Make a decision about the orientation of the edge
            // If there is an ambiguity (in 3d) preserve the orientation given at input
            // Case of ambiguity :
            // reverse and forward are false as nothing has been decided :
            //       closed wire, internal vertex ...
            // reverse and forward are true : closed or degenerated edge
            if (((forward == reverse) && (E.Orientation() == TopAbs_REVERSED)) || (reverse && !forward))
            {
                myEdge.Reverse();
            }
        }

        // add myEdge to myShape
        B.Add(myShape, myEdge);
        myShape.Closed(false);

        // Initialize VF, VL
        if (init)
        {
            TopExp.Vertices(TopoDS.Wire(myShape), out VF, out VL);
        }
        else
        {
            if (myError == BRepLib_WireDone)
            { // Update only
                TopoDS_Vertex VRef = new();
                TopExp.Vertices(myEdge, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
                if (V1.IsSame(myVertex))
                {
                    VRef = V2;
                }
                else if (V2.IsSame(myVertex))
                {
                    VRef = V1;
                }
                else
                {
                    myError = BRepLib_NonManifoldWire;
                }

                if (VF.IsSame(VL))
                {
                    // Particular case: it is required to control the orientation
                }
                else
                { // General case
                    if (VF.IsSame(myVertex))
                    {
                        VF = VRef;
                    }
                    else if (VL.IsSame(myVertex))
                    {
                        VL = VRef;
                    }
                    else
                    {
                        myError = BRepLib_NonManifoldWire;
                    }
                }
            }
            if (myError == BRepLib_NonManifoldWire)
            {
                VF = VL = new TopoDS_Vertex(); // nullify
            }
        }
        // Test myShape is closed
        if (!VF.IsNull() && !VL.IsNull() && VF.IsSame(VL))
        {
            myShape.Closed(true);
        }

        // OCCT resets the error here, so a NonManifoldWire found above is never reported (occt-issues.md); only VF
        // and VL, nullified, remember it.
        myError = BRepLib_WireDone;
        Done();
    }

    // The new point and tolerance of a wire vertex VW onto which an edge's vertex VE within tolerance is merged
    // (OCCT's code in Add, put in a function so that needsWritableVertices computes the same values).
    private static void mergedVertex(in gp_Pnt PW, double tolW, in gp_Pnt PE, double tolE, double l,
                                     out gp_Pnt PC, out double maxtol)
    {
        double cW, cE;
        maxtol = .5 * (tolW + tolE + l);
        if (maxtol > tolW && maxtol > tolE)
        {
            cW = (maxtol - tolE) / l;
            cE = 1.0 - cW;
        }
        else if (maxtol > tolW)
        {
            maxtol = tolE;
            cW = 0.0;
            cE = 1.0;
        }
        else
        {
            maxtol = tolW;
            cW = 1.0;
            cE = 0.0;
        }

        PC = new gp_Pnt(cW * PW.X() + cE * PE.X(),
                        cW * PW.Y() + cE * PE.Y(),
                        cW * PW.Z() + cE * PE.Z());
    }

    // OCCT.NET: frozen. True if merging the vertices of EE onto the wire's (as Add's copy loop does) would write into
    // a frozen wire vertex: move it, raise its tolerance, or give it a point representation (an INTERNAL or EXTERNAL
    // vertex). A frozen vertex of EE itself kept in the copied edge only receives its own tolerance (nothing
    // changes) unless it isn't FORWARD or REVERSED.
    private bool needsWritableVertices(in TopoDS_Edge EE)
    {
        for (var it = new TopoDS_Iterator(EE); it.More(); it.Next())
        {
            TopoDS_Vertex VE = TopoDS.Vertex(it.Value());
            gp_Pnt PE = BRep_Tool.Pnt(VE);
            bool matched = false;
            for (int i = 1; i <= myVertices.Extent(); i++)
            {
                TopoDS_Vertex VW = TopoDS.Vertex(myVertices.FindKey(i));
                gp_Pnt PW = BRep_Tool.Pnt(VW);
                double l = PE.Distance(PW);
                double tolW = BRep_Tool.Tolerance(VW);
                double tolE = BRep_Tool.Tolerance(VE);
                if ((l < tolE) || (l < tolW))
                {
                    matched = true;
                    if (VW.TShape()!.IsFrozen())
                    {
                        mergedVertex(PW, tolW, PE, tolE, l, out gp_Pnt PC, out double maxtol);
                        if (maxtol > tolW || !sameBits(PC, PW) || !isBoundary(VE) || tolE > maxtol)
                        {
                            return true;
                        }
                    }
                    break;
                }
            }
            if (!matched && VE.TShape()!.IsFrozen() && !isBoundary(VE))
            {
                return true;
            }
        }
        return false;
    }

    private static bool isBoundary(in TopoDS_Shape V)
        => V.Orientation() == TopAbs_FORWARD || V.Orientation() == TopAbs_REVERSED;

    private static bool sameBits(in gp_Pnt P1, in gp_Pnt P2)
        => P1.X().Equals(P2.X()) && P1.Y().Equals(P2.Y()) && P1.Z().Equals(P2.Z());

    // OCCT.NET: frozen. B.Transfert(Ein, Eout, Vin, Vout): sets on Eout the parameter of Vin on Ein for Vout, and raises
    // Vout's tolerance to Vin's. On a frozen Vout (a caller's vertex, FORWARD or REVERSED in Eout, whose tolerance is
    // already at least Vin's: see needsWritableVertices) only the edge's curve ranges change, as BRep_Builder's
    // UpdateVertex(V, Par, E, Tol) changes them; the vertex is left as it is, which is what the builder would leave.
    private static void transfert(BRep_Builder B, in TopoDS_Edge Ein, in TopoDS_Edge Eout, in TopoDS_Vertex Vin,
                                  in TopoDS_Vertex Vout)
    {
        if (!Vout.TShape()!.IsFrozen())
        {
            B.Transfert(Ein, Eout, Vin, Vout);
            return;
        }

        double tol = BRep_Tool.Tolerance(Vin);
        double parin = BRep_Tool.Parameter(Vin, Ein);
        if (Precision.IsPositiveInfinite(parin) || Precision.IsNegativeInfinite(parin))
        {
            throw new Standard_DomainError("BRep_Builder::Infinite parameter");
        }
        BRep_TEdge TE = (BRep_TEdge)Eout.TShape()!;

        TopAbs_Orientation ori = TopAbs_INTERNAL;
        var itv = new TopoDS_Iterator(Eout.Oriented(TopAbs_FORWARD));
        if (!itv.More() && TE.Degenerated())
        {
            ori = Vout.Orientation();
        }
        while (itv.More())
        {
            TopoDS_Shape Vcur = itv.Value();
            if (Vout.IsSame(Vcur))
            {
                ori = Vcur.Orientation();
                if (ori == Vout.Orientation())
                {
                    break;
                }
            }
            itv.Next();
        }

        if ((ori != TopAbs_FORWARD && ori != TopAbs_REVERSED) || tol > BRep_Tool.Tolerance(Vout))
        {
            // Would write into the frozen vertex: let the builder throw TopoDS_LockedShape.
            B.Transfert(Ein, Eout, Vin, Vout);
            return;
        }

        for (var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(TE.ChangeCurves()); itcr.More(); itcr.Next())
        {
            if (itcr.Value() is BRep_GCurve GC)
            {
                if (ori == TopAbs_FORWARD)
                {
                    GC.First(parin);
                }
                else
                {
                    GC.Last(parin);
                }
            }
        }
        TE.Modified(true);
    }

    // OCCT.NET: frozen. Replaces the wire built so far by a copy for write (design 4.6) and maps the maker's state
    // (the wire's vertices, VF, VL, the last edge and vertex) onto the copy, recording the copy in the history.
    private void copyForWrite()
    {
        TopoDS_Shape aCopy = BRepTools.CopyForWrite(myShape, out BRepTools_History aHistory);
        for (int i = 1; i <= myVertices.Extent(); i++)
        {
            myVertices.Substitute(i, imageOf(aHistory, myVertices.FindKey(i)));
        }
        VF = TopoDS.Vertex(imageOf(aHistory, VF));
        VL = TopoDS.Vertex(imageOf(aHistory, VL));
        FirstVertex = TopoDS.Vertex(imageOf(aHistory, FirstVertex));
        myVertex = TopoDS.Vertex(imageOf(aHistory, myVertex));
        myEdge = TopoDS.Edge(imageOf(aHistory, myEdge));
        myShape = aCopy;
        if (myHistory is null)
        {
            myHistory = aHistory;
        }
        else
        {
            myHistory.Merge(aHistory);
        }
    }

    // The copy of a shape of the wire, with the shape's orientation.
    private static TopoDS_Shape imageOf(BRepTools_History theHistory, in TopoDS_Shape theShape)
    {
        if (theShape.IsNull())
        {
            return theShape;
        }
        foreach (TopoDS_Shape aModified in theHistory.Modified(theShape))
        {
            return aModified.Oriented(theShape.Orientation());
        }
        return theShape;
    }

    /// <summary>
    /// Adds the edges of a list, in any order: coincident vertices of the list and the wire are merged into one
    /// (an existing wire vertex enlarged, or a new vertex), and the edges are added by connection. Not done if some
    /// edges couldn't be connected.
    /// </summary>
    public void Add(NCollection_List<TopoDS_Shape> L)
    {
        myError = BRepLib_WireDone;
        int aLSize = 0;
        int aRefSize = L.Size();
        if (!L.IsEmpty())
        {
            UnfreezeResult(); // OCCT.NET: frozen
            //
            var aGrVL = new NCollection_List<NCollection_List<TopoDS_Vertex>>();

            var aMapVE = new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);

            CollectCoincidentVertices(L, aGrVL);

            // OCCT.NET: frozen. Copy the wire for write if a frozen wire vertex must move or grow.
            if (needsWritableVertices(aGrVL))
            {
                copyForWrite(aGrVL);
            }

            var anO2NV = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);

            CreateNewVertices(aGrVL, anO2NV);

            var aNewEList = new NCollection_List<TopoDS_Shape>();

            CreateNewListOfEdges(L, anO2NV, aNewEList);
            //

            TopExp.MapShapesAndAncestors(myShape, TopAbs_VERTEX, TopAbs_EDGE, aMapVE);

            var aProcessedEdges = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            var anExp = new TopExp_Explorer();

            var anActEdges = new NCollection_List<TopoDS_Shape>();
            var aNeighEdges = new NCollection_List<TopoDS_Shape>();

            if (myEdge.IsNull())
            {
                // take the first edge from the list and add it
                TopoDS_Edge aFE = TopoDS.Edge(aNewEList.First());
                Add(aFE);
                aProcessedEdges.Add(aFE);
                anActEdges.Append(aFE);
                aLSize++;
            }
            else
            {
                // existing edges are already connected
                for (anExp.Init(myShape, TopAbs_EDGE); anExp.More(); anExp.Next())
                {
                    TopoDS_Shape aCSh = anExp.Current();
                    aProcessedEdges.Add(aCSh);
                    anActEdges.Append(aCSh);
                }
            }

            var anItL1 = new NCollection_List<TopoDS_Shape>.Iterator();
            var anItL2 = new NCollection_List<TopoDS_Shape>.Iterator();

            for (anItL1.Initialize(aNewEList); anItL1.More(); anItL1.Next())
            {
                TopExp.MapShapesAndAncestors(anItL1.Value(), TopAbs_VERTEX, TopAbs_EDGE, aMapVE);
            }

            while (!anActEdges.IsEmpty())
            {
                anItL2.Initialize(anActEdges);
                for (; anItL2.More(); anItL2.Next())
                {
                    TopoDS_Shape aCE = anItL2.Value();
                    anExp.Init(aCE, TopAbs_VERTEX);
                    for (; anExp.More(); anExp.Next())
                    {
                        TopoDS_Shape aCV = anExp.Current();
                        for (anItL1.Initialize(aMapVE.FindFromKey(aCV)); anItL1.More(); anItL1.Next())
                        {
                            TopoDS_Shape aNE = anItL1.Value(); // neighbor edge
                            if (!aProcessedEdges.Contains(aNE))
                            {
                                Add(TopoDS.Edge(aNE), false);
                                aNeighEdges.Append(aNE);
                                aProcessedEdges.Add(aNE);
                                aLSize++;
                            }
                        }
                    }
                }
                anActEdges.Clear();
                anActEdges.Append(aNeighEdges);
            }
        }
        if (aLSize == aRefSize)
        {
            Done();
        }
        else
        {
            NotDone();
            myError = BRepLib_DisconnectedWire;
        }
    }

    // OCCT.NET: frozen. True if CreateNewVertices would move or enlarge a frozen vertex of the wire.
    private bool needsWritableVertices(NCollection_List<NCollection_List<TopoDS_Vertex>> theGrVL)
    {
        for (var itLLV = new NCollection_List<NCollection_List<TopoDS_Vertex>>.Iterator(theGrVL); itLLV.More(); itLLV.Next())
        {
            TopoDS_Vertex aNewV = new();
            var aValList = new NCollection_List<TopoDS_Shape>();
            for (var itLV = new NCollection_List<TopoDS_Vertex>.Iterator(itLLV.Value()); itLV.More(); itLV.Next())
            {
                aValList.Append(itLV.Value());
                if (myVertices.Contains(itLV.Value()))
                {
                    aNewV = itLV.Value();
                }
            }
            if (aNewV.IsNull() || !aNewV.TShape()!.IsFrozen())
            {
                continue;
            }
            double aNewTol = 0;
            gp_Pnt aNewC = new();
            BRepLib.BoundingVertex(aValList, ref aNewC, ref aNewTol);
            if (aNewTol > BRep_Tool.Tolerance(aNewV) || !sameBits(aNewC, BRep_Tool.Pnt(aNewV)))
            {
                return true;
            }
        }
        return false;
    }

    // OCCT.NET: frozen. copyForWrite, then the wire's vertices in the groups replaced by their copies.
    private void copyForWrite(NCollection_List<NCollection_List<TopoDS_Vertex>> theGrVL)
    {
        TopoDS_Shape anOld = myShape;
        copyForWrite();
        BRepTools_History aHistory = myHistory!;
        var anOldVertices = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(anOld, anOldVertices);
        for (var itLLV = new NCollection_List<NCollection_List<TopoDS_Vertex>>.Iterator(theGrVL); itLLV.More(); itLLV.Next())
        {
            for (var itLV = new NCollection_List<TopoDS_Vertex>.Iterator(itLLV.ChangeValue()); itLV.More(); itLV.Next())
            {
                if (anOldVertices.Contains(itLV.Value()))
                {
                    itLV.ChangeValue() = TopoDS.Vertex(imageOf(aHistory, itLV.Value()));
                }
            }
        }
    }

    private sealed class BRepLib_BndBoxVertexSelector : NCollection_UBTree<int, Bnd_Box>.Selector
    {
        private readonly NCollection_IndexedMap<TopoDS_Shape> myMapOfShape; // vertices
        private gp_Pnt myP;
        private double myTolP;
        private int myVInd;
        private Bnd_Box myVBox = new();
        private readonly NCollection_List<int> myResultInd = new();

        public BRepLib_BndBoxVertexSelector(NCollection_IndexedMap<TopoDS_Shape> theMapOfShape)
        {
            myMapOfShape = theMapOfShape;
            myTolP = 0.0;
            myVInd = 0;
        }

        public override bool Reject(in Bnd_Box theBox) => theBox.IsOut(myVBox);

        public override bool Accept(in int theObj)
        {
            if (theObj > myMapOfShape.Extent())
            {
                return false;
            }

            TopoDS_Vertex aV = TopoDS.Vertex(myMapOfShape[theObj]);

            if (theObj == myVInd)
            {
                return false;
            }

            gp_Pnt aVPnt = BRep_Tool.Pnt(aV);

            double aTolV = BRep_Tool.Tolerance(aV);

            double aL = myP.SquareDistance(aVPnt);
            double aSTol = aTolV + myTolP;
            aSTol *= aSTol;

            if (aL <= aSTol)
            {
                myResultInd.Append(theObj);
                return true;
            }

            return false;
        }

        public void SetCurrentVertex(in gp_Pnt theP, double theTol, int theVInd)
        {
            myP = theP;
            // OCCT never resets myVBox, so the box grows with every vertex (occt-issues.md): more boxes pass Reject,
            // but Accept checks the distance, so only the time changes.
            myVBox.Add(myP);
            myVBox.Enlarge(theTol);
            myTolP = theTol;
            myVInd = theVInd;
        }

        public NCollection_List<int> GetResultInds() => myResultInd;

        public void ClearResInds() => myResultInd.Clear();
    }

    private void CollectCoincidentVertices(NCollection_List<TopoDS_Shape> theL,
                                           NCollection_List<NCollection_List<TopoDS_Vertex>> theGrVL)
    {
        var anAllV = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);

        TopExp.MapShapes(myShape, TopAbs_VERTEX, anAllV);

        var anItL = new NCollection_List<TopoDS_Shape>.Iterator(theL);
        for (; anItL.More(); anItL.Next())
        {
            TopExp.MapShapes(anItL.Value(), TopAbs_VERTEX, anAllV);
        }

        // aV2CV : vertex <-> its coincident vertices. OCCT's maps here use std::hash and operator== (IsEqual).
        var aV2CV = new NCollection_DataMap<TopoDS_Vertex, NCollection_Map<TopoDS_Vertex>>(EqualityComparer<TopoDS_Vertex>.Default);
        var aTree = new NCollection_UBTree<int, Bnd_Box>();
        var aTreeFiller = new NCollection_UBTreeFiller<int, Bnd_Box>(aTree);
        var aNonGroupedV = new NCollection_Map<TopoDS_Vertex>(EqualityComparer<TopoDS_Vertex>.Default);

        // add vertices from anAllV to treefiller
        for (int i = 1; i <= anAllV.Extent(); i++)
        {
            TopoDS_Shape aSh = anAllV[i];
            var aBB = new Bnd_Box();
            BRepBndLib.Add(aSh, ref aBB);
            aTreeFiller.Add(i, aBB);
        }

        aTreeFiller.Fill();
        var aSelector = new BRepLib_BndBoxVertexSelector(anAllV);

        int aNbColl = 0;
        var itI = new NCollection_List<int>.Iterator();
        for (int i = 1; i <= anAllV.Extent(); i++)
        {
            TopoDS_Vertex aV = TopoDS.Vertex(anAllV[i]);
            if (myVertices.Contains(aV))
            {
                continue;
            }
            aSelector.SetCurrentVertex(BRep_Tool.Pnt(aV), BRep_Tool.Tolerance(aV), i);
            aNbColl = aTree.Select(aSelector);
            if (aNbColl > 0)
            {
                NCollection_List<int> aResInds = aSelector.GetResultInds();
                NCollection_Map<TopoDS_Vertex> aVM = aV2CV.Bound(aV, new NCollection_Map<TopoDS_Vertex>(EqualityComparer<TopoDS_Vertex>.Default));
                for (itI.Initialize(aResInds); itI.More(); itI.Next())
                {
                    TopoDS_Vertex aCV = TopoDS.Vertex(anAllV[itI.Value()]);
                    aVM.Add(aCV);
                    if (myVertices.Contains(aCV))
                    {
                        if (aV2CV.IsBound(aCV))
                        {
                            aV2CV[aCV].Add(aV);
                        }
                        else
                        {
                            aV2CV.Bound(aCV, new NCollection_Map<TopoDS_Vertex>(EqualityComparer<TopoDS_Vertex>.Default)).Add(aV);
                            aNonGroupedV.Add(aCV);
                        }
                    }
                }
                if (!aVM.IsEmpty())
                {
                    aNonGroupedV.Add(aV); // vertexes to be grouped; store only coincident vertices
                }
            }
            aSelector.ClearResInds();
        }

        // group the coincident vertices
        var aStartV = new NCollection_List<TopoDS_Vertex>();
        var aCurrentV = new NCollection_List<TopoDS_Vertex>();
        var anOneGrV = new NCollection_List<TopoDS_Vertex>();
        var itLV = new NCollection_List<TopoDS_Vertex>.Iterator();
        bool IsStartNewGroup = true;
        while (!aNonGroupedV.IsEmpty() || !IsStartNewGroup)
        // exit only if there are no nongrouped vertices
        // and the last group are fully are constructed
        {
            if (IsStartNewGroup)
            {
                // start list of vertices is empty => append one from aNonGroupedV
                //  and remove it from it (i.e. mark as grouped)
                // OCCT takes the first vertex of its hash map (an order of addresses); the port takes its own map's
                // first (decision 18).
                TopoDS_Vertex aCurV = new();
                foreach (TopoDS_Vertex aV in aNonGroupedV)
                {
                    aCurV = aV;
                    break;
                }
                aStartV.Append(aCurV);
                aNonGroupedV.Remove(aCurV);
            }
            itLV.Initialize(aStartV);
            for (; itLV.More(); itLV.Next())
            {
                TopoDS_Vertex aSV = itLV.Value();
                anOneGrV.Append(aSV);
                foreach (TopoDS_Vertex aCV in aV2CV[aSV])
                {
                    if (aNonGroupedV.Contains(aCV))
                    {
                        aCurrentV.Append(aCV);
                        aNonGroupedV.Remove(aCV);
                    }
                }
            }
            aStartV.Clear();
            aStartV.Append(aCurrentV);
            IsStartNewGroup = aStartV.IsEmpty();
            if (IsStartNewGroup && !anOneGrV.IsEmpty())
            {
                // OCCT's Append copies the list here (the item overload), then clears anOneGrV.
                theGrVL.Append(new NCollection_List<TopoDS_Vertex>(anOneGrV));
                anOneGrV.Clear();
            }
        }
    }

    private void CreateNewVertices(NCollection_List<NCollection_List<TopoDS_Vertex>> theGrVL,
                                   NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> theO2NV)
    {
        // map [old vertex => new vertex]
        // note that already existing shape (i.e. the original ones)
        // shouldn't be modified on the topological level
        var itLLV = new NCollection_List<NCollection_List<TopoDS_Vertex>>.Iterator();
        var itLV = new NCollection_List<TopoDS_Vertex>.Iterator();
        var aBB = new BRep_Builder();
        itLLV.Initialize(theGrVL);
        for (; itLLV.More(); itLLV.Next())
        {
            TopoDS_Vertex aNewV = new();
            var aValList = new NCollection_List<TopoDS_Shape>();
            NCollection_List<TopoDS_Vertex> aVal = itLLV.Value();
            itLV.Initialize(aVal);
            double aNewTol = 0;
            gp_Pnt aNewC = new();
            for (; itLV.More(); itLV.Next())
            {
                TopoDS_Vertex aVV = itLV.Value();
                aValList.Append(aVV);
                if (myVertices.Contains(aVV))
                {
                    aNewV = aVV;
                }
            }
            BRepLib.BoundingVertex(aValList, ref aNewC, ref aNewTol);

            if (aNewV.IsNull())
            {
                // vertices from the original shape isn't found in this group
                // create the new vertex
                aNewV = new BRepLib_MakeVertex(aNewC).InternalVertex();
                aBB.UpdateVertex(aNewV, aNewTol);
            }
            else if (!aNewV.TShape()!.IsFrozen())
            {
                // update already existing vertex
                aBB.UpdateVertex(aNewV, new gp_Pnt(aNewC.XYZ()), aNewTol);
            }
            // OCCT.NET: frozen. A frozen existing vertex here is one the update leaves as it is (needsWritableVertices
            // made the others writable).

            // fill the map of old->new vertices
            itLV.Initialize(aVal);
            for (; itLV.More(); itLV.Next())
            {
                TopoDS_Vertex aVV = itLV.Value();
                theO2NV.Bind(aVV, aNewV);
            }
        }
    }

    private static void CreateNewListOfEdges(NCollection_List<TopoDS_Shape> theL,
                                             NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> theO2NV,
                                             NCollection_List<TopoDS_Shape> theNewEList)
    {
        // create the new list (theNewEList) from the input list L
        bool IsNewEdge;
        var aVList = new NCollection_List<TopoDS_Vertex>();
        var exp = new TopExp_Explorer();
        var aBB = new BRep_Builder();
        var anItL = new NCollection_List<TopoDS_Shape>.Iterator();
        for (anItL.Initialize(theL); anItL.More(); anItL.Next())
        {
            IsNewEdge = false;
            aVList.Clear();
            TopoDS_Edge aCE = TopoDS.Edge(anItL.Value());
            exp.Init(aCE, TopAbs_VERTEX);
            for (; exp.More(); exp.Next())
            {
                TopoDS_Vertex aVal = TopoDS.Vertex(exp.Current());
                if (theO2NV.IsBound(aVal))
                {
                    IsNewEdge = true;
                    // append the new vertex
                    aVList.Append(TopoDS.Vertex(theO2NV[aVal].Oriented(aVal.Orientation())));
                }
                else
                {
                    aVList.Append(aVal);
                }
            }
            if (IsNewEdge)
            {
                TopoDS_Shape NewE = aCE.EmptyCopied();
                var it = new NCollection_List<TopoDS_Vertex>.Iterator(aVList);
                for (; it.More(); it.Next())
                {
                    aBB.Add(NewE, it.Value());
                }
                theNewEList.Append(TopoDS.Edge(NewE));
            }
            else
            {
                theNewEList.Append(aCE);
            }
        }
    }

    /// <summary>Returns the construction status.</summary>
    public BRepLib_WireError Error()
    {
        return myError;
    }

    /// <summary>The wire (frozen).</summary>
    public TopoDS_Wire Wire()
    {
        return TopoDS.Wire(Shape());
    }

    /// <summary><see cref="Wire"/> without freezing, for ported code.</summary>
    internal TopoDS_Wire InternalWire()
    {
        return TopoDS.Wire(InternalShape());
    }

    /// <summary>The wire (OCCT's conversion operator; frozen).</summary>
    public static explicit operator TopoDS_Wire(BRepLib_MakeWire theMaker) => theMaker.Wire();

    /// <summary>The last edge added (frozen with the wire): a copy of the given edge when it was connected by proximity.</summary>
    public TopoDS_Edge Edge() => TopoDS.Edge(Published(myEdge));

    /// <summary><see cref="Edge"/> without freezing, for ported code.</summary>
    internal TopoDS_Edge InternalEdge() => myEdge;

    /// <summary>The last connecting vertex (frozen with the wire).</summary>
    public TopoDS_Vertex Vertex() => TopoDS.Vertex(Published(myVertex));

    /// <summary><see cref="Vertex"/> without freezing, for ported code.</summary>
    internal TopoDS_Vertex InternalVertex() => myVertex;

    /// <summary>
    /// OCCT.NET: the history of the copies for write the maker made when merging a vertex onto a frozen vertex of
    /// the wire: each edge and vertex of the wire at the time, modified into its copy. Null if no copy was made, in
    /// which case the wire holds the caller's edges themselves (or their copies made by OCCT's algorithm).
    /// </summary>
    internal BRepTools_History? InternalHistory() => myHistory;
}
