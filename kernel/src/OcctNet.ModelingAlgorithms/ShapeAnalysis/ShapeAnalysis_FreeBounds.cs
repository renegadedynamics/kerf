// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_FreeBounds.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_FreeBounds.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_FreeBounds.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.ShapeExtend_Status;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// The free bounds of a shape: wires of the edges that only one face of the shape refers to (the outer boundaries of
/// faces and shells, as opposed to the edges between faces and the seams of closed faces), as a compound of closed
/// wires and a compound of open ones.
/// </summary>
/// <remarks>
/// <para>
/// Two kinds of shapes are analysed. A compound of faces, with a tolerance: the free edges are the ones sewing
/// (BRepBuilderAPI_Sewing) would leave free, and they are connected into wires where their ends are closer than the
/// tolerance. A compound of shells, without one: the free edges are the ones referenced by one face
/// (<see cref="ShapeAnalysis_Shell"/>), connected where they share a vertex. Closed sub-wires can be extracted from the
/// closed and open wires. The static methods connect edges or wires into wires, extract closed sub-wires and dispatch
/// wires into the two compounds.
/// </para>
/// <para>
/// A public working object (decision 21; OCCT uses it by value). It writes only into the wires and compounds it builds
/// (edges with replaced vertices are new edges): the shapes it is given may be frozen. Public results freeze what they
/// hand out (<see cref="GetClosedWires"/>, <see cref="GetOpenWires"/>, the wires of the static methods; docs/
/// phase3-design.md, section 4.2); ported code that goes on building uses the <c>Internal</c> forms. As in OCCT,
/// <see cref="ConnectEdgesToWires(NCollection_HSequence{Shape}, double, bool)"/> reverses, in the caller's
/// sequence, the edges it reversed, and <see cref="ConnectWiresToWires(NCollection_HSequence{Shape}, double,
/// bool)"/> the wires: the sequences are working collections.
/// </para>
/// </remarks>
public sealed class ShapeAnalysis_FreeBounds
{
    private Compound myWires = new();
    private Compound myEdges = new();
    private readonly double myTolerance;
    private readonly bool myShared;
    private readonly bool mySplitClosed;
    private readonly bool mySplitOpen;

    /// <summary>An empty analyser.</summary>
    public ShapeAnalysis_FreeBounds()
    {
    }

    /// <summary>
    /// The free bounds that sewing <paramref name="shape"/> (a compound of faces) with the tolerance
    /// <paramref name="toler"/> would leave: the free edges are connected where their ends are closer than the
    /// tolerance. With <paramref name="splitclosed"/> (<paramref name="splitopen"/>), closed sub-wires are extracted
    /// from the closed (open) wires.
    /// </summary>
    public ShapeAnalysis_FreeBounds(in Shape shape, double toler, bool splitclosed = false, bool splitopen = true)
        : this(shape, toler, splitclosed, splitopen, true)
    {
    }

    /// <summary>
    /// The analyser of the sewing form, without freezing the free edges sewing hands out (it works in place on
    /// unfrozen shapes): for ported code that goes on writing into the shape it analyses (ShapeFix_FreeBounds).
    /// </summary>
    internal static ShapeAnalysis_FreeBounds InternalSewn(in Shape shape, double toler, bool splitclosed, bool splitopen) =>
        new(shape, toler, splitclosed, splitopen, false);

    private ShapeAnalysis_FreeBounds(in Shape shape, double toler, bool splitclosed, bool splitopen, bool theToFreeze)
    {
        myTolerance = toler;
        myShared = false;
        mySplitClosed = splitclosed;
        mySplitOpen = splitopen;

        //
        // Extract free edges.
        //
        NCollection_HSequence<Shape> edges = sewnFreeEdges(shape, toler, theToFreeze);
        //
        // Chainage.
        //
        NCollection_HSequence<Shape> wires = InternalConnectEdgesToWires(edges, toler, false);
        InternalDispatchWires(wires, ref myWires, ref myEdges);
        SplitWires();
    }

    // The non-degenerated free edges of the faces of the shape after sewing them with the tolerance (OCCT's
    // constructor body, up to the chaining). Sewing copies frozen inputs itself and builds no sewn shape in this mode
    // (decision 599): nothing here needs a copy.
    // With theToFreeze false, the free edges aren't frozen (InternalFreeEdge).
    private static NCollection_HSequence<Shape> sewnFreeEdges(in Shape shape, double toler, bool theToFreeze)
    {
        var Sew = new BRepBuilderAPI_Sewing(toler, false, false);
        for (var S = new Iterator(shape); S.More(); S.Next())
        {
            Sew.Add(S.Value());
        }
        Sew.Perform();
        //
        // Extract free edges.
        //
        int nbedge = Sew.NbFreeEdges();
        var edges = new NCollection_HSequence<Shape>();
        Edge anEdge;
        for (int iedge = 1; iedge <= nbedge; iedge++)
        {
            anEdge = theToFreeze ? Sew.FreeEdge(iedge) // frozen: the caller's edges where sewing didn't cut them
                                 : Sew.InternalFreeEdge(iedge);
            if (!Tool.Degenerated(anEdge))
            {
                edges.Append(anEdge);
            }
        }
        return edges;
    }

    /// <summary>
    /// The actual free bounds of <paramref name="shape"/> (a compound of shells): the edges referenced by one face,
    /// connected where they share a vertex (INTERNAL edges aren't free with <paramref name="checkinternaledges"/>).
    /// With <paramref name="splitclosed"/> (<paramref name="splitopen"/>), closed sub-wires are extracted from the
    /// closed (open) wires.
    /// </summary>
    public ShapeAnalysis_FreeBounds(in Shape shape, bool splitclosed = false, bool splitopen = true,
                                    bool checkinternaledges = false)
    {
        myTolerance = 0.0;
        myShared = true;
        mySplitClosed = splitclosed;
        mySplitOpen = splitopen;

        Shell aTmpShell = new();
        BrepBuilder aB = new();
        aB.MakeShell(ref aTmpShell);
        for (var aExpFace = new Explorer(shape, ShapeType.Face); aExpFace.More(); aExpFace.Next())
        {
            aB.Add(aTmpShell, aExpFace.Current());
        }

        var sas = new ShapeAnalysis_Shell();
        sas.CheckOrientedShells(aTmpShell, true, checkinternaledges);

        if (sas.HasFreeEdges())
        {
            var see = new ShapeExtend_Explorer();
            NCollection_HSequence<Shape> edges = see.SeqFromCompound(sas.InternalFreeEdges(), false);

            NCollection_HSequence<Shape> wires = InternalConnectEdgesToWires(edges, Kerf.Engine.Foundation.Precision.Confusion(), true);
            InternalDispatchWires(wires, ref myWires, ref myEdges);
            SplitWires();
        }
    }

    /// <summary>The compound of the closed wires of free edges (a null compound if there are none). Frozen.</summary>
    public Compound GetClosedWires() => Published(myWires);

    /// <summary>The closed wires, as <see cref="GetClosedWires"/>, unfrozen (ported code).</summary>
    internal Compound InternalGetClosedWires() => myWires;

    /// <summary>The compound of the open wires of free edges (a null compound if there are none). Frozen.</summary>
    public Compound GetOpenWires() => Published(myEdges);

    /// <summary>The open wires, as <see cref="GetOpenWires"/>, unfrozen (ported code).</summary>
    internal Compound InternalGetOpenWires() => myEdges;

    /// <summary>
    /// Connects the unsorted edges into wires of maximal length: a wire stops growing when no edge connects at its
    /// head or tail. Connected where the edges share a vertex with <paramref name="shared"/>, else where their ends
    /// are closer than <paramref name="toler"/>. INTERNAL and EXTERNAL edges are ignored. An edge's orientation can
    /// change, and the edges reversed are reversed in <paramref name="edges"/> too. The wires are frozen.
    /// </summary>
    public static NCollection_HSequence<Shape> ConnectEdgesToWires(NCollection_HSequence<Shape> edges, double toler, bool shared) =>
        PublishedAll(InternalConnectEdgesToWires(edges, toler, shared));

    /// <summary>The connection, as <see cref="ConnectEdgesToWires(NCollection_HSequence{Shape}, double, bool)"/>, unfrozen (ported code).</summary>
    internal static NCollection_HSequence<Shape> InternalConnectEdgesToWires(NCollection_HSequence<Shape> edges, double toler, bool shared)
    {
        var iwires = new NCollection_HSequence<Shape>();
        var anEdgeIndices = new List<int>();
        BrepBuilder B = new();

        int i; // svv #1
        for (i = 1; i <= edges.Length(); i++)
        {
            Orientation anOrientation = edges.Value(i).Orientation();
            if (anOrientation == Internal || anOrientation == External)
            {
                continue;
            }

            Wire wire = new();
            B.MakeWire(ref wire);
            B.Add(wire, edges.Value(i));
            iwires.Append(wire);
            anEdgeIndices.Add(i);
        }

        NCollection_HSequence<Shape> wires = InternalConnectWiresToWires(iwires, toler, shared);

        for (i = 1; i <= iwires.Length(); i++)
        {
            if (iwires.Value(i).Orientation() == Reversed)
            {
                edges[anEdgeIndices[i - 1]].Reverse();
            }
        }

        return wires;
    }

    /// <summary>Connects the edges into wires (deprecated form of the function returning the sequence).</summary>
    [Obsolete("Use ConnectEdgesToWires() returning the sequence instead")]
    public static void ConnectEdgesToWires(NCollection_HSequence<Shape> edges, double toler, bool shared,
                                           out NCollection_HSequence<Shape> wires)
    {
        wires = ConnectEdgesToWires(edges, toler, shared);
    }

    /// <summary>
    /// Connects the unsorted wires into wires of maximal length, as
    /// <see cref="ConnectEdgesToWires(NCollection_HSequence{Shape}, double, bool)"/>. A wire's orientation can
    /// change, and the wires reversed are reversed in <paramref name="iwires"/> too. The wires are frozen.
    /// </summary>
    public static NCollection_HSequence<Shape> ConnectWiresToWires(NCollection_HSequence<Shape>? iwires, double toler, bool shared)
    {
        var map = new NCollection_DataMap<Shape, Shape>(ShapeHasher.Instance);
        return ConnectWiresToWires(iwires, toler, shared, map);
    }

    /// <summary>The connection, as <see cref="ConnectWiresToWires(NCollection_HSequence{Shape}, double, bool)"/>, unfrozen (ported code).</summary>
    internal static NCollection_HSequence<Shape> InternalConnectWiresToWires(NCollection_HSequence<Shape>? iwires, double toler, bool shared)
    {
        var map = new NCollection_DataMap<Shape, Shape>(ShapeHasher.Instance);
        return InternalConnectWiresToWires(iwires, toler, shared, map);
    }

    /// <summary>Connects the wires into wires (deprecated form of the function returning the sequence).</summary>
    [Obsolete("Use ConnectWiresToWires() returning the sequence instead")]
    public static void ConnectWiresToWires(NCollection_HSequence<Shape>? iwires, double toler, bool shared,
                                           out NCollection_HSequence<Shape> owires)
    {
        owires = ConnectWiresToWires(iwires, toler, shared);
    }

    /// <summary>
    /// Connects the wires, as <see cref="ConnectWiresToWires(NCollection_HSequence{Shape}, double, bool)"/>,
    /// and fills <paramref name="vertices"/> with the correspondence between the wires' original end vertices and the
    /// new vertices connecting them. The wires and the new vertices are frozen.
    /// </summary>
    public static NCollection_HSequence<Shape> ConnectWiresToWires(NCollection_HSequence<Shape>? iwires, double toler, bool shared,
                                                                          NCollection_DataMap<Shape, Shape> vertices)
    {
        NCollection_HSequence<Shape> owires = InternalConnectWiresToWires(iwires, toler, shared, vertices);
        foreach (KeyValuePair<Shape, Shape> aPair in vertices)
        {
            aPair.Value.TShape()?.FreezeGraph();
        }
        return PublishedAll(owires);
    }

    /// <summary>Connects the wires and fills the vertex map (deprecated form of the function returning the sequence).</summary>
    [Obsolete("Use ConnectWiresToWires() returning the sequence instead")]
    public static void ConnectWiresToWires(NCollection_HSequence<Shape>? iwires, double toler, bool shared,
                                           out NCollection_HSequence<Shape> owires,
                                           NCollection_DataMap<Shape, Shape> vertices)
    {
        owires = ConnectWiresToWires(iwires, toler, shared, vertices);
    }

    /// <summary>The connection with the vertex map, unfrozen (ported code).</summary>
    internal static NCollection_HSequence<Shape> InternalConnectWiresToWires(NCollection_HSequence<Shape>? iwires, double toler, bool shared,
                                                                                    NCollection_DataMap<Shape, Shape> vertices)
    {
        connectWiresToWiresImpl(iwires, toler, shared, out NCollection_HSequence<Shape> owires, vertices);
        return owires;
    }

    private static void connectWiresToWiresImpl(NCollection_HSequence<Shape>? iwires, double toler, bool shared,
                                                out NCollection_HSequence<Shape> owires,
                                                NCollection_DataMap<Shape, Shape> vertices)
    {
        if (iwires is null || iwires.Length() == 0)
        {
            owires = new NCollection_HSequence<Shape>(); // was left null, later crashed Append
            return;
        }
        var arrwires = new NCollection_HArray1<Shape>(1, iwires.Length());
        int i;
        for (i = 1; i <= arrwires.Length(); i++)
        {
            arrwires.SetValue(i, iwires.Value(i));
        }
        owires = new NCollection_HSequence<Shape>();
        double tolerance = Max(toler, Kerf.Engine.Foundation.Precision.Confusion());

        var aSel = new ShapeAnalysis_BoxBndTreeSelector(arrwires, shared);
        ShapeExtend_WireData? sewd = null;
        for (i = 1; i <= arrwires.Length(); i++)
        {
            var aWireData = new ShapeExtend_WireData(ShapeCasts.Wire(arrwires.Value(i)));
            if (aWireData.NbEdges() > 0)
            {
                if (sewd is null)
                {
                    sewd = aWireData;
                    aSel.LoadList(i);
                }
            }
            else
            {
                if (aWireData.NbNonManifoldEdges() > 0)
                {
                    owires.Append(aWireData.InternalWire());
                }
                aSel.LoadList(i);
            }
        }

        if (sewd is null)
        {
            return;
        }

        var saw = new ShapeAnalysis_Wire();
        saw.Load(sewd);
        saw.SetPrecision(tolerance);

        var aBBTree = new SpatialTree<int, Box>();
        var aTreeFiller = new SpatialTreeFiller<int, Box>(aBBTree);

        for (int inbW = 1; inbW <= arrwires.Length(); inbW++)
        {
            if (aSel.ContWire(inbW))
            {
                continue;
            }
            Wire trW = ShapeCasts.Wire(arrwires.Value(inbW));
            Box aBox = new();
            ShapeAnalysis.FindBounds(trW, out Vertex trV1, out Vertex trV2);
            Point3D trP1 = Tool.Pnt(trV1);
            Point3D trP2 = Tool.Pnt(trV2);
            aBox.Set(trP1);
            aBox.Add(trP2);
            aBox.SetGap(tolerance);
            aTreeFiller.Add(inbW, aBox);
        }

        aTreeFiller.Fill();
        int nsel;

        var sae = new ShapeAnalysis_Edge();
        bool done = false;

        while (!done)
        {
            bool found = false, tail = false, direct = false;
            int lwire = 0;
            aSel.SetStop();
            Box FVBox = new(), LVBox = new();
            Vertex Vf, Vl;
            Vf = sae.FirstVertex(sewd.Edge(1));
            Vl = sae.LastVertex(sewd.Edge(sewd.NbEdges()));

            Point3D pf, pl;
            pf = Tool.Pnt(Vf);
            pl = Tool.Pnt(Vl);
            FVBox.Set(pf);
            FVBox.SetGap(tolerance);
            LVBox.Set(pl);
            LVBox.SetGap(tolerance);

            aSel.DefineBoxes(FVBox, LVBox);

            if (shared)
            {
                aSel.DefineVertexes(Vf, Vl);
            }
            else
            {
                aSel.DefinePnt(pf, pl);
                aSel.SetTolerance(tolerance);
            }

            nsel = aBBTree.Select(aSel);

            if (nsel != 0 && !aSel.LastCheckStatus(ShapeExtend_FAIL))
            {
                found = true;
                lwire = aSel.GetNb();
                tail = aSel.LastCheckStatus(ShapeExtend_DONE1) || aSel.LastCheckStatus(ShapeExtend_DONE2);
                direct = aSel.LastCheckStatus(ShapeExtend_DONE1) || aSel.LastCheckStatus(ShapeExtend_DONE3);
                aSel.LoadList(lwire);
            }

            if (found)
            {
                if (!direct)
                {
                    arrwires[lwire].Reverse();
                }

                var acurwd = new ShapeExtend_WireData(ShapeCasts.Wire(arrwires.Value(lwire)), true, true);
                if (acurwd.NbEdges() == 0)
                {
                    continue;
                }
                sewd.Add(acurwd, (tail ? 0 : 1));
            }
            else
            {
                for (i = 1; i <= saw.NbEdges(); i++)
                {
                    if (saw.CheckConnected(i))
                    {
                        int n2 = i;
                        int n1 = (n2 > 1 ? n2 - 1 : saw.NbEdges());
                        Edge E1 = sewd.Edge(n1);
                        Edge E2 = sewd.Edge(n2);

                        Vertex Vprev, Vfol, V;
                        Vprev = sae.LastVertex(E1);
                        Vfol = sae.FirstVertex(E2);

                        if (saw.LastCheckStatus(ShapeExtend_DONE1))
                        {
                            V = Vprev;
                        }
                        else
                        {
                            var sbv = new ShapeBuild_Vertex();
                            V = sbv.InternalCombineVertex(Vprev, Vfol);
                        }
                        vertices.Bind(Vprev, V);
                        vertices.Bind(Vfol, V);

                        var sbe = new ShapeBuild_Edge();
                        if (saw.NbEdges() < 2)
                        {
                            sewd.Set(sbe.InternalCopyReplaceVertices(E2, V, V), n2);
                        }
                        else
                        {
                            sewd.Set(sbe.InternalCopyReplaceVertices(E2, V, new Vertex()), n2);
                            if (!saw.LastCheckStatus(ShapeExtend_DONE1))
                            {
                                sewd.Set(sbe.InternalCopyReplaceVertices(E1, new Vertex(), V), n1);
                            }
                        }
                    }
                }

                Wire wire = sewd.InternalWire();
                if (!saw.CheckConnected(1) && saw.LastCheckStatus(ShapeExtend_OK))
                {
                    wire.Closed(true);
                }

                owires.Append(wire);
                sewd.Clear();

                lwire = -1;
                for (i = 1; i <= arrwires.Length(); i++)
                {
                    if (!aSel.ContWire(i))
                    {
                        sewd.Add(ShapeCasts.Wire(arrwires.Value(i)));
                        aSel.LoadList(i);

                        if (sewd.NbEdges() > 0)
                        {
                            lwire = i;
                            break;
                        }
                        sewd.Clear();
                    }
                }

                if (lwire == -1)
                {
                    done = true;
                }
            }
        }

        for (i = 1; i <= iwires.Length(); i++)
        {
            iwires.SetValue(i, arrwires.Value(i));
        }
    }

    private static void SplitWire(in Wire wire, double toler, bool shared,
                                  out NCollection_HSequence<Shape> closed,
                                  out NCollection_HSequence<Shape> open)
    {
        closed = new NCollection_HSequence<Shape>();
        double tolerance = Max(toler, Kerf.Engine.Foundation.Precision.Confusion());

        BrepBuilder B = new();
        var sae = new ShapeAnalysis_Edge();

        var sewd = new ShapeExtend_WireData(wire);
        int nbedges = sewd.NbEdges();

        // ConnectedEdgeSequence - list of indices of connected edges to build a wire
        var ces = new NCollection_Sequence<int>();
        // statuses - array of flags describing the edge:
        // 0-free, 1-in CES, 2-already in wire,
        // 3-no closed wire can be produced starting at this edge
        var statuses = new NCollection_Array1<int>(1, nbedges);
        statuses.Init(0);

        // building closed wires
        int i; // svv #1
        for (i = 1; i <= nbedges; i++)
        {
            if (statuses.Value(i) == 0)
            {
                ces.Append(i);
                statuses.SetValue(i, 1); // putting into CES
                bool SearchBackward = true;

                for (;;)
                {
                    bool found;
                    Edge edge;
                    Vertex lvertex;
                    Point3D lpoint;

                    // searching for connection in ces
                    if (SearchBackward)
                    {
                        SearchBackward = false;
                        found = false;
                        edge = sewd.Edge(ces.Last());
                        lvertex = sae.LastVertex(edge);
                        lpoint = Tool.Pnt(lvertex);
                        int jb; // svv #1
                        for (jb = ces.Length(); (jb >= 1) && !found; jb--)
                        {
                            Vertex fv = sae.FirstVertex(sewd.Edge(ces.Value(jb)));
                            Point3D fp = Tool.Pnt(fv);
                            if ((shared && lvertex.IsSame(fv)) || (!shared && lpoint.IsEqual(fp, tolerance)))
                            {
                                found = true;
                            }
                        }

                        if (found)
                        {
                            jb++; // because of decreasing last iteration
                            // making closed wire
                            Wire wire1 = new();
                            B.MakeWire(ref wire1);
                            for (int cesindex = jb; cesindex <= ces.Length(); cesindex++)
                            {
                                B.Add(wire1, sewd.Edge(ces.Value(cesindex)));
                                statuses.SetValue(ces.Value(cesindex), 2);
                            }
                            wire1.Closed(true);
                            closed.Append(wire1);
                            ces.Remove(jb, ces.Length());
                            if (ces.IsEmpty())
                            {
                                break;
                            }
                        }
                    }

                    // searching for connection among free edges
                    found = false;
                    edge = sewd.Edge(ces.Last());
                    lvertex = sae.LastVertex(edge);
                    lpoint = Tool.Pnt(lvertex);
                    int j; // svv #1
                    for (j = 1; (j <= nbedges) && !found; j++)
                    {
                        if (statuses.Value(j) == 0)
                        {
                            Vertex fv = sae.FirstVertex(sewd.Edge(j));
                            Point3D fp = Tool.Pnt(fv);
                            if ((shared && lvertex.IsSame(fv)) || (!shared && lpoint.IsEqual(fp, tolerance)))
                            {
                                found = true;
                            }
                        }
                    }

                    if (found)
                    {
                        j--; // because of last iteration
                        ces.Append(j);
                        statuses.SetValue(j, 1); // putting into CES
                        SearchBackward = true;
                        continue;
                    }

                    // no edges found - mark the branch as open (use status 3)
                    statuses.SetValue(ces.Last(), 3);
                    ces.Remove(ces.Length());
                    if (ces.IsEmpty())
                    {
                        break;
                    }
                }
            }
        }

        // building open wires
        var edges = new NCollection_HSequence<Shape>();
        for (i = 1; i <= nbedges; i++)
        {
            if (statuses.Value(i) != 2)
            {
                edges.Append(sewd.Edge(i));
            }
        }

        open = InternalConnectEdgesToWires(edges, toler, shared);
    }

    /// <summary>
    /// Extracts the closed sub-wires of <paramref name="wires"/> into <paramref name="closed"/> and puts the open
    /// wires left into <paramref name="open"/> (both new sequences); connections by shared vertices with
    /// <paramref name="shared"/>, else by distance below <paramref name="toler"/>. The wires are frozen.
    /// </summary>
    public static void SplitWires(NCollection_HSequence<Shape> wires, double toler, bool shared,
                                  out NCollection_HSequence<Shape> closed, out NCollection_HSequence<Shape> open)
    {
        InternalSplitWires(wires, toler, shared, out closed, out open);
        PublishedAll(closed);
        PublishedAll(open);
    }

    /// <summary>The extraction, as <see cref="SplitWires(NCollection_HSequence{Shape}, double, bool, out NCollection_HSequence{Shape}, out NCollection_HSequence{Shape})"/>, unfrozen (ported code).</summary>
    internal static void InternalSplitWires(NCollection_HSequence<Shape> wires, double toler, bool shared,
                                            out NCollection_HSequence<Shape> closed, out NCollection_HSequence<Shape> open)
    {
        closed = new NCollection_HSequence<Shape>();
        open = new NCollection_HSequence<Shape>();

        for (int i = 1; i <= wires.Length(); i++)
        {
            SplitWire(ShapeCasts.Wire(wires.Value(i)), toler, shared, out NCollection_HSequence<Shape> tmpclosed,
                      out NCollection_HSequence<Shape> tmpopen);
            closed.Append(tmpclosed);
            open.Append(tmpopen);
        }
    }

    /// <summary>
    /// Adds the closed wires of <paramref name="wires"/> to <paramref name="closed"/> and the open ones to
    /// <paramref name="open"/>, making the compounds that are null. OCCT adds to the caller's compounds; a caller's
    /// compound is frozen, so the port replaces each by a new compound holding its items and the added wires. Both are
    /// frozen.
    /// </summary>
    public static void DispatchWires(NCollection_HSequence<Shape>? wires, ref Compound closed, ref Compound open)
    {
        // OCCT.NET: frozen: copy-on-write of the caller's compounds.
        closed = writable(closed);
        open = writable(open);
        InternalDispatchWires(wires, ref closed, ref open);
        Published(closed);
        Published(open);
    }

    /// <summary>The dispatch, as <see cref="DispatchWires"/>, adding in place to unfrozen compounds (ported code).</summary>
    internal static void InternalDispatchWires(NCollection_HSequence<Shape>? wires, ref Compound closed, ref Compound open)
    {
        BrepBuilder B = new();
        if (closed.IsNull)
        {
            B.MakeCompound(ref closed);
        }
        if (open.IsNull)
        {
            B.MakeCompound(ref open);
        }
        if (wires is null)
        {
            return;
        }

        for (int iw = 1; iw <= wires.Length(); iw++)
        {
            if (wires.Value(iw).Closed())
            {
                B.Add(closed, wires.Value(iw));
            }
            else
            {
                B.Add(open, wires.Value(iw));
            }
        }
    }

    // A compound that can be added to: the compound itself when it is null or unfrozen, else a new compound holding
    // its items.
    private static Compound writable(in Compound theCompound)
    {
        if (theCompound.IsNull || !theCompound.TShape()!.IsFrozen)
        {
            return theCompound;
        }
        Compound aNew = new();
        BrepBuilder B = new();
        B.MakeCompound(ref aNew);
        for (var it = new Iterator(theCompound, false, false); it.More(); it.Next())
        {
            B.Add(aNew, it.Value());
        }
        aNew.Location(theCompound.Location(), false);
        aNew.Orientation(theCompound.Orientation());
        return aNew;
    }

    //=======================================================================
    // function : SplitWires
    // purpose  : Splits compounds of closed (myWires) and open (myEdges) wires
    //           into small closed wires according to fields mySplitClosed and
    //           mySplitOpen and rebuilds compounds
    //=======================================================================

    private void SplitWires()
    {
        if (!mySplitClosed && !mySplitOpen)
        {
            return; // nothing to do
        }

        var see = new ShapeExtend_Explorer();
        NCollection_HSequence<Shape> closedwires, cw1, cw2, openwires, ow1, ow2;
        closedwires = see.SeqFromCompound(myWires, false);
        openwires = see.SeqFromCompound(myEdges, false);

        if (mySplitClosed)
        {
            InternalSplitWires(closedwires, myTolerance, myShared, out cw1, out ow1);
        }
        else
        {
            cw1 = closedwires;
            ow1 = new NCollection_HSequence<Shape>();
        }

        if (mySplitOpen)
        {
            InternalSplitWires(openwires, myTolerance, myShared, out cw2, out ow2);
        }
        else
        {
            cw2 = new NCollection_HSequence<Shape>();
            ow2 = openwires;
        }

        closedwires = cw1;
        closedwires.Append(cw2);
        openwires = ow1;
        openwires.Append(ow2);

        // szv#4:S4163:12Mar99 SGI warns
        Shape compWires = see.InternalCompoundFromSeq(closedwires);
        Shape compEdges = see.InternalCompoundFromSeq(openwires);
        myWires = ShapeCasts.Compound(compWires);
        myEdges = ShapeCasts.Compound(compEdges);
    }

    // Freezes the graph of a shape handed to a caller (docs/phase3-design.md, section 4.2).
    private static Compound Published(in Compound theShape)
    {
        theShape.TShape()?.FreezeGraph();
        return theShape;
    }

    // Freezes the graphs of the shapes of a sequence handed to a caller.
    private static NCollection_HSequence<Shape> PublishedAll(NCollection_HSequence<Shape> theShapes)
    {
        for (int i = 1; i <= theShapes.Length(); i++)
        {
            theShapes.Value(i).TShape()?.FreezeGraph();
        }
        return theShapes;
    }
}
