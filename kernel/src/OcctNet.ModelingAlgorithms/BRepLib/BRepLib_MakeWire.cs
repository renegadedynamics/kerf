// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeWire.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeWire.cxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeWire_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.BRepLib_WireError;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

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
    private Edge myEdge = new();
    private Vertex myVertex = new();
    private readonly NCollection_IndexedMap<Shape> myVertices = new(ShapeHasher.Instance);
    private Vertex FirstVertex = new();
    private Vertex VF = new();
    private Vertex VL = new();
    // OCCT.NET: the history of the copies for write made so far (null if none).
    private History? myHistory;

    /// <summary>Makes an empty wire: <see cref="Error"/> is BRepLib_EmptyWire until an edge is added.</summary>
    public BRepLib_MakeWire()
    {
        myError = BRepLib_EmptyWire;
    }

    public BRepLib_MakeWire(in Edge E)
    {
        Add(E);
    }

    public BRepLib_MakeWire(in Edge E1, in Edge E2)
    {
        Add(E1);
        Add(E2);
    }

    public BRepLib_MakeWire(in Edge E1, in Edge E2, in Edge E3)
    {
        Add(E1);
        Add(E2);
        Add(E3);
    }

    public BRepLib_MakeWire(in Edge E1, in Edge E2, in Edge E3, in Edge E4)
    {
        Add(E1);
        Add(E2);
        Add(E3);
        Add(E4);
    }

    public BRepLib_MakeWire(in Wire W)
    {
        Add(W);
    }

    public BRepLib_MakeWire(in Wire W, in Edge E)
    {
        Add(W);
        Add(E);
    }

    /// <summary>Adds the edges of a wire, stopping at the first that can't be added.</summary>
    public void Add(in Wire W)
    {
        for (var it = new Iterator(W); it.More(); it.Next())
        {
            Add(ShapeCasts.Edge(it.Value()));
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
    public void Add(in Edge E)
    {
        Add(E, true);
    }

    // PMN  19/03/1998  For the Problem of performance TopExp::Vertices are not used on wire
    // PMN  10/09/1998  In case if the wire is previously closed (or degenerated)
    //                  TopExp::Vertices is used to reduce the ambiguity.
    // IsCheckGeometryProximity flag : If true => check for the geometry proximity of vertices
    private void Add(in Edge E, bool IsCheckGeometryProximity)
    {
        bool forward = false;
        // to tell if it has been decided to add forward
        bool reverse = false;
        // to tell if it has been decided to add reversed
        bool init = false;
        // To know if it is necessary to calculate VL, VF
        var B = new BrepBuilder();
        var it = new Iterator();

        if (myEdge.IsNull)
        {
            init = true;
            // first edge, create the wire
            Wire aW = new();
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
            Shape aLocalShape = E.Oriented(Forward);
            Edge EE = ShapeCasts.Edge(aLocalShape);

            // test the vertices of the edge

            bool connected = false;
            bool copyedge = false;

            if (myError != BRepLib_NonManifoldWire)
            {
                if (VF.IsNull || VL.IsNull)
                {
                    myError = BRepLib_NonManifoldWire;
                }
            }

            for (it.Initialize(EE); it.More(); it.Next())
            {
                Vertex VE = ShapeCasts.Vertex(it.Value());

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
                                if (VE.Orientation() == Forward)
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
                                if (VE.Orientation() == Reversed)
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
                    Point3D PE = Tool.Pnt(VE);

                    for (int i = 1; i <= myVertices.Extent(); i++)
                    {
                        Vertex VW = ShapeCasts.Vertex(myVertices.FindKey(i));
                        Point3D PW = Tool.Pnt(VW);
                        double l = PE.Distance(PW);

                        if ((l < Tool.Tolerance(VE)) || (l < Tool.Tolerance(VW)))
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
                                        if (VE.Orientation() == Forward)
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
                                        if (VE.Orientation() == Reversed)
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
                    Shape Dummy = EE.EmptyCopied();
                    myEdge = ShapeCasts.Edge(Dummy);

                    for (it.Initialize(EE); it.More(); it.Next())
                    {
                        Vertex VE = ShapeCasts.Vertex(it.Value());
                        Point3D PE = Tool.Pnt(VE);

                        bool newvertex = false;
                        for (int i = 1; i <= myVertices.Extent(); i++)
                        {
                            Vertex VW = ShapeCasts.Vertex(myVertices.FindKey(i));
                            Point3D PW = Tool.Pnt(VW);
                            double l = PE.Distance(PW), tolE, tolW;
                            tolW = Tool.Tolerance(VW);
                            tolE = Tool.Tolerance(VE);

                            if ((l < tolE) || (l < tolW))
                            {
                                mergedVertex(PW, tolW, PE, tolE, l, out Point3D PC, out double maxtol);

                                // OCCT.NET: frozen. A frozen VW here is one the update leaves as it is
                                // (needsWritableVertices made the others writable).
                                if (!VW.TShape()!.IsFrozen)
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
            if (((forward == reverse) && (E.Orientation() == Reversed)) || (reverse && !forward))
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
            Exploration.Vertices(ShapeCasts.Wire(myShape), out VF, out VL);
        }
        else
        {
            if (myError == BRepLib_WireDone)
            { // Update only
                Vertex VRef = new();
                Exploration.Vertices(myEdge, out Vertex V1, out Vertex V2);
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
                VF = VL = new Vertex(); // nullify
            }
        }
        // Test myShape is closed
        if (!VF.IsNull && !VL.IsNull && VF.IsSame(VL))
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
    private static void mergedVertex(in Point3D PW, double tolW, in Point3D PE, double tolE, double l,
                                     out Point3D PC, out double maxtol)
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

        PC = new Point3D(cW * PW.X() + cE * PE.X(),
                        cW * PW.Y() + cE * PE.Y(),
                        cW * PW.Z() + cE * PE.Z());
    }

    // OCCT.NET: frozen. True if merging the vertices of EE onto the wire's (as Add's copy loop does) would write into
    // a frozen wire vertex: move it, raise its tolerance, or give it a point representation (an INTERNAL or EXTERNAL
    // vertex). A frozen vertex of EE itself kept in the copied edge only receives its own tolerance (nothing
    // changes) unless it isn't FORWARD or REVERSED.
    private bool needsWritableVertices(in Edge EE)
    {
        for (var it = new Iterator(EE); it.More(); it.Next())
        {
            Vertex VE = ShapeCasts.Vertex(it.Value());
            Point3D PE = Tool.Pnt(VE);
            bool matched = false;
            for (int i = 1; i <= myVertices.Extent(); i++)
            {
                Vertex VW = ShapeCasts.Vertex(myVertices.FindKey(i));
                Point3D PW = Tool.Pnt(VW);
                double l = PE.Distance(PW);
                double tolW = Tool.Tolerance(VW);
                double tolE = Tool.Tolerance(VE);
                if ((l < tolE) || (l < tolW))
                {
                    matched = true;
                    if (VW.TShape()!.IsFrozen)
                    {
                        mergedVertex(PW, tolW, PE, tolE, l, out Point3D PC, out double maxtol);
                        if (maxtol > tolW || !sameBits(PC, PW) || !isBoundary(VE) || tolE > maxtol)
                        {
                            return true;
                        }
                    }
                    break;
                }
            }
            if (!matched && VE.TShape()!.IsFrozen && !isBoundary(VE))
            {
                return true;
            }
        }
        return false;
    }

    private static bool isBoundary(in Shape V)
        => V.Orientation() == Forward || V.Orientation() == Reversed;

    private static bool sameBits(in Point3D P1, in Point3D P2)
        => P1.X().Equals(P2.X()) && P1.Y().Equals(P2.Y()) && P1.Z().Equals(P2.Z());

    // OCCT.NET: frozen. B.Transfert(Ein, Eout, Vin, Vout): sets on Eout the parameter of Vin on Ein for Vout, and raises
    // Vout's tolerance to Vin's. On a frozen Vout (a caller's vertex, FORWARD or REVERSED in Eout, whose tolerance is
    // already at least Vin's: see needsWritableVertices) only the edge's curve ranges change, as BRep_Builder's
    // UpdateVertex(V, Par, E, Tol) changes them; the vertex is left as it is, which is what the builder would leave.
    private static void transfert(BrepBuilder B, in Edge Ein, in Edge Eout, in Vertex Vin,
                                  in Vertex Vout)
    {
        if (!Vout.TShape()!.IsFrozen)
        {
            B.Transfert(Ein, Eout, Vin, Vout);
            return;
        }

        double tol = Tool.Tolerance(Vin);
        double parin = Tool.Parameter(Vin, Ein);
        if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(parin) || global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(parin))
        {
            throw new DomainErrorException("BRep_Builder::Infinite parameter");
        }
        BrepTEdge TE = (BrepTEdge)Eout.TShape()!;

        Orientation ori = Internal;
        var itv = new Iterator(Eout.Oriented(Forward));
        if (!itv.More() && TE.Degenerated())
        {
            ori = Vout.Orientation();
        }
        while (itv.More())
        {
            Shape Vcur = itv.Value();
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

        if ((ori != Forward && ori != Reversed) || tol > Tool.Tolerance(Vout))
        {
            // Would write into the frozen vertex: let the builder throw TopoDS_LockedShape.
            B.Transfert(Ein, Eout, Vin, Vout);
            return;
        }

        for (var itcr = new NCollection_List<CurveRepresentation>.Iterator(TE.ChangeCurves()); itcr.More(); itcr.Next())
        {
            if (itcr.Value() is GCurve GC)
            {
                if (ori == Forward)
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

    // OCCT.NET: frozen. Replaces the wire built so far by an exact copy for write (design 4.6) and maps the maker's state
    // (the wire's vertices, VF, VL, the last edge and vertex) onto the copy, recording the copy in the history.
    private void copyForWrite()
    {
        Shape aCopy = BrepTools.ExactCopyForWrite(myShape, out History aHistory);
        for (int i = 1; i <= myVertices.Extent(); i++)
        {
            myVertices.Substitute(i, imageOf(aHistory, myVertices.FindKey(i)));
        }
        VF = ShapeCasts.Vertex(imageOf(aHistory, VF));
        VL = ShapeCasts.Vertex(imageOf(aHistory, VL));
        FirstVertex = ShapeCasts.Vertex(imageOf(aHistory, FirstVertex));
        myVertex = ShapeCasts.Vertex(imageOf(aHistory, myVertex));
        myEdge = ShapeCasts.Edge(imageOf(aHistory, myEdge));
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
    private static Shape imageOf(History theHistory, in Shape theShape)
    {
        if (theShape.IsNull)
        {
            return theShape;
        }
        foreach (Shape aModified in theHistory.Modified(theShape))
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
    public void Add(NCollection_List<Shape> L)
    {
        myError = BRepLib_WireDone;
        int aLSize = 0;
        int aRefSize = L.Size();
        if (!L.IsEmpty())
        {
            UnfreezeResult(); // OCCT.NET: frozen
            //
            var aGrVL = new NCollection_List<NCollection_List<Vertex>>();

            var aMapVE = new NCollection_IndexedDataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);

            CollectCoincidentVertices(L, aGrVL);

            // OCCT.NET: frozen. Copy the wire for write if a frozen wire vertex must move or grow.
            if (needsWritableVertices(aGrVL))
            {
                copyForWrite(aGrVL);
            }

            var anO2NV = new NCollection_DataMap<Shape, Shape>(ShapeHasher.Instance);

            CreateNewVertices(aGrVL, anO2NV);

            var aNewEList = new NCollection_List<Shape>();

            CreateNewListOfEdges(L, anO2NV, aNewEList);
            //

            Exploration.MapShapesAndAncestors(myShape, global::Kerf.Topology.ShapeType.Vertex, global::Kerf.Topology.ShapeType.Edge, aMapVE);

            var aProcessedEdges = new NCollection_Map<Shape>(ShapeHasher.Instance);
            var anExp = new Explorer();

            var anActEdges = new NCollection_List<Shape>();
            var aNeighEdges = new NCollection_List<Shape>();

            if (myEdge.IsNull)
            {
                // take the first edge from the list and add it
                Edge aFE = ShapeCasts.Edge(aNewEList.First());
                Add(aFE);
                aProcessedEdges.Add(aFE);
                anActEdges.Append(aFE);
                aLSize++;
            }
            else
            {
                // existing edges are already connected
                for (anExp.Init(myShape, global::Kerf.Topology.ShapeType.Edge); anExp.More(); anExp.Next())
                {
                    Shape aCSh = anExp.Current();
                    aProcessedEdges.Add(aCSh);
                    anActEdges.Append(aCSh);
                }
            }

            var anItL1 = new NCollection_List<Shape>.Iterator();
            var anItL2 = new NCollection_List<Shape>.Iterator();

            for (anItL1.Initialize(aNewEList); anItL1.More(); anItL1.Next())
            {
                Exploration.MapShapesAndAncestors(anItL1.Value(), global::Kerf.Topology.ShapeType.Vertex, global::Kerf.Topology.ShapeType.Edge, aMapVE);
            }

            while (!anActEdges.IsEmpty())
            {
                anItL2.Initialize(anActEdges);
                for (; anItL2.More(); anItL2.Next())
                {
                    Shape aCE = anItL2.Value();
                    anExp.Init(aCE, global::Kerf.Topology.ShapeType.Vertex);
                    for (; anExp.More(); anExp.Next())
                    {
                        Shape aCV = anExp.Current();
                        for (anItL1.Initialize(aMapVE.FindFromKey(aCV)); anItL1.More(); anItL1.Next())
                        {
                            Shape aNE = anItL1.Value(); // neighbor edge
                            if (!aProcessedEdges.Contains(aNE))
                            {
                                Add(ShapeCasts.Edge(aNE), false);
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
    private bool needsWritableVertices(NCollection_List<NCollection_List<Vertex>> theGrVL)
    {
        for (var itLLV = new NCollection_List<NCollection_List<Vertex>>.Iterator(theGrVL); itLLV.More(); itLLV.Next())
        {
            Vertex aNewV = new();
            var aValList = new NCollection_List<Shape>();
            for (var itLV = new NCollection_List<Vertex>.Iterator(itLLV.Value()); itLV.More(); itLV.Next())
            {
                aValList.Append(itLV.Value());
                if (myVertices.Contains(itLV.Value()))
                {
                    aNewV = itLV.Value();
                }
            }
            if (aNewV.IsNull || !aNewV.TShape()!.IsFrozen)
            {
                continue;
            }
            double aNewTol = 0;
            Point3D aNewC = new();
            BRepLib.BoundingVertex(aValList, ref aNewC, ref aNewTol);
            if (aNewTol > Tool.Tolerance(aNewV) || !sameBits(aNewC, Tool.Pnt(aNewV)))
            {
                return true;
            }
        }
        return false;
    }

    // OCCT.NET: frozen. copyForWrite, then the wire's vertices in the groups replaced by their copies.
    private void copyForWrite(NCollection_List<NCollection_List<Vertex>> theGrVL)
    {
        Shape anOld = myShape;
        copyForWrite();
        History aHistory = myHistory!;
        var anOldVertices = new NCollection_Map<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(anOld, anOldVertices);
        for (var itLLV = new NCollection_List<NCollection_List<Vertex>>.Iterator(theGrVL); itLLV.More(); itLLV.Next())
        {
            for (var itLV = new NCollection_List<Vertex>.Iterator(itLLV.ChangeValue()); itLV.More(); itLV.Next())
            {
                if (anOldVertices.Contains(itLV.Value()))
                {
                    itLV.ChangeValue() = ShapeCasts.Vertex(imageOf(aHistory, itLV.Value()));
                }
            }
        }
    }

    private sealed class BRepLib_BndBoxVertexSelector : SpatialTree<int, Box>.Selector
    {
        private readonly NCollection_IndexedMap<Shape> myMapOfShape; // vertices
        private Point3D myP;
        private double myTolP;
        private int myVInd;
        private Box myVBox = new();
        private readonly NCollection_List<int> myResultInd = new();

        public BRepLib_BndBoxVertexSelector(NCollection_IndexedMap<Shape> theMapOfShape)
        {
            myMapOfShape = theMapOfShape;
            myTolP = 0.0;
            myVInd = 0;
        }

        public override bool Reject(in Box theBox) => theBox.IsOut(myVBox);

        public override bool Accept(in int theObj)
        {
            if (theObj > myMapOfShape.Extent())
            {
                return false;
            }

            Vertex aV = ShapeCasts.Vertex(myMapOfShape[theObj]);

            if (theObj == myVInd)
            {
                return false;
            }

            Point3D aVPnt = Tool.Pnt(aV);

            double aTolV = Tool.Tolerance(aV);

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

        public void SetCurrentVertex(in Point3D theP, double theTol, int theVInd)
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

    private void CollectCoincidentVertices(NCollection_List<Shape> theL,
                                           NCollection_List<NCollection_List<Vertex>> theGrVL)
    {
        var anAllV = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);

        Exploration.MapShapes(myShape, global::Kerf.Topology.ShapeType.Vertex, anAllV);

        var anItL = new NCollection_List<Shape>.Iterator(theL);
        for (; anItL.More(); anItL.Next())
        {
            Exploration.MapShapes(anItL.Value(), global::Kerf.Topology.ShapeType.Vertex, anAllV);
        }

        // aV2CV : vertex <-> its coincident vertices. OCCT's maps here use std::hash and operator== (IsEqual).
        var aV2CV = new NCollection_DataMap<Vertex, NCollection_Map<Vertex>>(EqualityComparer<Vertex>.Default);
        var aTree = new SpatialTree<int, Box>();
        var aTreeFiller = new SpatialTreeFiller<int, Box>(aTree);
        var aNonGroupedV = new NCollection_Map<Vertex>(EqualityComparer<Vertex>.Default);

        // add vertices from anAllV to treefiller
        for (int i = 1; i <= anAllV.Extent(); i++)
        {
            Shape aSh = anAllV[i];
            var aBB = new Box();
            BRepBndLib.Add(aSh, ref aBB);
            aTreeFiller.Add(i, aBB);
        }

        aTreeFiller.Fill();
        var aSelector = new BRepLib_BndBoxVertexSelector(anAllV);

        int aNbColl = 0;
        var itI = new NCollection_List<int>.Iterator();
        for (int i = 1; i <= anAllV.Extent(); i++)
        {
            Vertex aV = ShapeCasts.Vertex(anAllV[i]);
            if (myVertices.Contains(aV))
            {
                continue;
            }
            aSelector.SetCurrentVertex(Tool.Pnt(aV), Tool.Tolerance(aV), i);
            aNbColl = aTree.Select(aSelector);
            if (aNbColl > 0)
            {
                NCollection_List<int> aResInds = aSelector.GetResultInds();
                NCollection_Map<Vertex> aVM = aV2CV.Bound(aV, new NCollection_Map<Vertex>(EqualityComparer<Vertex>.Default));
                for (itI.Initialize(aResInds); itI.More(); itI.Next())
                {
                    Vertex aCV = ShapeCasts.Vertex(anAllV[itI.Value()]);
                    aVM.Add(aCV);
                    if (myVertices.Contains(aCV))
                    {
                        if (aV2CV.IsBound(aCV))
                        {
                            aV2CV[aCV].Add(aV);
                        }
                        else
                        {
                            aV2CV.Bound(aCV, new NCollection_Map<Vertex>(EqualityComparer<Vertex>.Default)).Add(aV);
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
        var aStartV = new NCollection_List<Vertex>();
        var aCurrentV = new NCollection_List<Vertex>();
        var anOneGrV = new NCollection_List<Vertex>();
        var itLV = new NCollection_List<Vertex>.Iterator();
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
                Vertex aCurV = new();
                foreach (Vertex aV in aNonGroupedV)
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
                Vertex aSV = itLV.Value();
                anOneGrV.Append(aSV);
                foreach (Vertex aCV in aV2CV[aSV])
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
                theGrVL.Append(new NCollection_List<Vertex>(anOneGrV));
                anOneGrV.Clear();
            }
        }
    }

    private void CreateNewVertices(NCollection_List<NCollection_List<Vertex>> theGrVL,
                                   NCollection_DataMap<Shape, Shape> theO2NV)
    {
        // map [old vertex => new vertex]
        // note that already existing shape (i.e. the original ones)
        // shouldn't be modified on the topological level
        var itLLV = new NCollection_List<NCollection_List<Vertex>>.Iterator();
        var itLV = new NCollection_List<Vertex>.Iterator();
        var aBB = new BrepBuilder();
        itLLV.Initialize(theGrVL);
        for (; itLLV.More(); itLLV.Next())
        {
            Vertex aNewV = new();
            var aValList = new NCollection_List<Shape>();
            NCollection_List<Vertex> aVal = itLLV.Value();
            itLV.Initialize(aVal);
            double aNewTol = 0;
            Point3D aNewC = new();
            for (; itLV.More(); itLV.Next())
            {
                Vertex aVV = itLV.Value();
                aValList.Append(aVV);
                if (myVertices.Contains(aVV))
                {
                    aNewV = aVV;
                }
            }
            BRepLib.BoundingVertex(aValList, ref aNewC, ref aNewTol);

            if (aNewV.IsNull)
            {
                // vertices from the original shape isn't found in this group
                // create the new vertex
                aNewV = new BRepLib_MakeVertex(aNewC).InternalVertex();
                aBB.UpdateVertex(aNewV, aNewTol);
            }
            else if (!aNewV.TShape()!.IsFrozen)
            {
                // update already existing vertex
                aBB.UpdateVertex(aNewV, new Point3D(aNewC.XYZ()), aNewTol);
            }
            // OCCT.NET: frozen. A frozen existing vertex here is one the update leaves as it is (needsWritableVertices
            // made the others writable).

            // fill the map of old->new vertices
            itLV.Initialize(aVal);
            for (; itLV.More(); itLV.Next())
            {
                Vertex aVV = itLV.Value();
                theO2NV.Bind(aVV, aNewV);
            }
        }
    }

    private static void CreateNewListOfEdges(NCollection_List<Shape> theL,
                                             NCollection_DataMap<Shape, Shape> theO2NV,
                                             NCollection_List<Shape> theNewEList)
    {
        // create the new list (theNewEList) from the input list L
        bool IsNewEdge;
        var aVList = new NCollection_List<Vertex>();
        var exp = new Explorer();
        var aBB = new BrepBuilder();
        var anItL = new NCollection_List<Shape>.Iterator();
        for (anItL.Initialize(theL); anItL.More(); anItL.Next())
        {
            IsNewEdge = false;
            aVList.Clear();
            Edge aCE = ShapeCasts.Edge(anItL.Value());
            exp.Init(aCE, global::Kerf.Topology.ShapeType.Vertex);
            for (; exp.More(); exp.Next())
            {
                Vertex aVal = ShapeCasts.Vertex(exp.Current());
                if (theO2NV.IsBound(aVal))
                {
                    IsNewEdge = true;
                    // append the new vertex
                    aVList.Append(ShapeCasts.Vertex(theO2NV[aVal].Oriented(aVal.Orientation())));
                }
                else
                {
                    aVList.Append(aVal);
                }
            }
            if (IsNewEdge)
            {
                Shape NewE = aCE.EmptyCopied();
                var it = new NCollection_List<Vertex>.Iterator(aVList);
                for (; it.More(); it.Next())
                {
                    aBB.Add(NewE, it.Value());
                }
                theNewEList.Append(ShapeCasts.Edge(NewE));
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
    public Wire Wire()
    {
        return ShapeCasts.Wire(Shape());
    }

    /// <summary><see cref="Wire"/> without freezing, for ported code.</summary>
    internal Wire InternalWire()
    {
        return ShapeCasts.Wire(InternalShape());
    }

    /// <summary>The wire (OCCT's conversion operator; frozen).</summary>
    public static explicit operator Wire(BRepLib_MakeWire theMaker) => theMaker.Wire();

    /// <summary>The last edge added (frozen with the wire): a copy of the given edge when it was connected by proximity.</summary>
    public Edge Edge() => ShapeCasts.Edge(Published(myEdge));

    /// <summary><see cref="Edge"/> without freezing, for ported code.</summary>
    internal Edge InternalEdge() => myEdge;

    /// <summary>The last connecting vertex (frozen with the wire).</summary>
    public Vertex Vertex() => ShapeCasts.Vertex(Published(myVertex));

    /// <summary><see cref="Vertex"/> without freezing, for ported code.</summary>
    internal Vertex InternalVertex() => myVertex;

    /// <summary>
    /// OCCT.NET: the history of the copies for write the maker made when merging a vertex onto a frozen vertex of
    /// the wire: each edge and vertex of the wire at the time, modified into its copy. Null if no copy was made, in
    /// which case the wire holds the caller's edges themselves (or their copies made by OCCT's algorithm).
    /// </summary>
    internal History? InternalHistory() => myHistory;
}
