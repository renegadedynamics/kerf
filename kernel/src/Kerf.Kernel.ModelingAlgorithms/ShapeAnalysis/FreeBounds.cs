// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_FreeBounds.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_FreeBounds.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_FreeBounds.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Healing.Extend.ExtendStatus;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Healing.Analysis;

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
/// (<see cref="ShellAnalysis"/>), connected where they share a vertex. Closed sub-wires can be extracted from the
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
internal sealed class FreeBounds
{
    private Compound _wires = new();
    private Compound _edges = new();
    private readonly double _tolerance;
    private readonly bool _shared;
    private readonly bool _splitClosed;
    private readonly bool _splitOpen;

    /// <summary>An empty analyser.</summary>
    public FreeBounds()
    {
    }

    /// <summary>
    /// The free bounds that sewing <paramref name="shape"/> (a compound of faces) with the tolerance
    /// <paramref name="toler"/> would leave: the free edges are connected where their ends are closer than the
    /// tolerance. With <paramref name="splitclosed"/> (<paramref name="splitopen"/>), closed sub-wires are extracted
    /// from the closed (open) wires.
    /// </summary>
    public FreeBounds(in Shape shape, double toler, bool splitclosed = false, bool splitopen = true)
        : this(shape, toler, splitclosed, splitopen, true)
    {
    }

    /// <summary>
    /// The analyser of the sewing form, without freezing the free edges sewing hands out (it works in place on
    /// unfrozen shapes): for ported code that goes on writing into the shape it analyses (ShapeFix_FreeBounds).
    /// </summary>
    internal static FreeBounds InternalSewn(in Shape shape, double toler, bool splitclosed, bool splitopen) =>
        new(shape, toler, splitclosed, splitopen, false);

    private FreeBounds(in Shape shape, double toler, bool splitclosed, bool splitopen, bool toFreeze)
    {
        _tolerance = toler;
        _shared = false;
        _splitClosed = splitclosed;
        _splitOpen = splitopen;

        //
        // Extract free edges.
        //
        NCollection_HSequence<Shape> edges = SewnFreeEdges(shape, toler, toFreeze);
        //
        // Chainage.
        //
        NCollection_HSequence<Shape> wires = InternalConnectEdgesToWires(edges, toler, false);
        InternalDispatchWires(wires, ref _wires, ref _edges);
        SplitWires();
    }

    // The non-degenerated free edges of the faces of the shape after sewing them with the tolerance (OCCT's
    // constructor body, up to the chaining). Sewing copies frozen inputs itself and builds no sewn shape in this mode
    // (decision 599): nothing here needs a copy.
    // With theToFreeze false, the free edges aren't frozen (InternalFreeEdge).
    private static NCollection_HSequence<Shape> SewnFreeEdges(in Shape shape, double toler, bool toFreeze)
    {
        var sew = new Sewing(toler, false, false);
        for (var s = new Iterator(shape); s.More(); s.Next())
        {
            sew.Add(s.Value());
        }
        sew.Perform();
        //
        // Extract free edges.
        //
        int nbedge = sew.FreeEdgeCount;
        var edges = new NCollection_HSequence<Shape>();
        Edge edge;
        for (int iedge = 1; iedge <= nbedge; iedge++)
        {
            edge = toFreeze ? sew.FreeEdge(iedge) // frozen: the caller's edges where sewing didn't cut them
                                 : sew.InternalFreeEdge(iedge);
            if (!Tool.Degenerated(edge))
            {
                edges.Append(edge);
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
    public FreeBounds(in Shape shape, bool splitclosed = false, bool splitopen = true,
                                    bool checkinternaledges = false)
    {
        _tolerance = 0.0;
        _shared = true;
        _splitClosed = splitclosed;
        _splitOpen = splitopen;

        Shell tmpShell = new();
        BrepBuilder b = new();
        b.MakeShell(ref tmpShell);
        for (var expFace = new Explorer(shape, ShapeType.Face); expFace.More(); expFace.Next())
        {
            b.Add(tmpShell, expFace.Current());
        }

        var sas = new ShellAnalysis();
        sas.CheckOrientedShells(tmpShell, true, checkinternaledges);

        if (sas.HasFreeEdges)
        {
            var see = new ExtendExplorer();
            NCollection_HSequence<Shape> edges = see.SeqFromCompound(sas.InternalFreeEdges(), false);

            NCollection_HSequence<Shape> wires = InternalConnectEdgesToWires(edges, Kerf.Engine.Foundation.Precision.Confusion(), true);
            InternalDispatchWires(wires, ref _wires, ref _edges);
            SplitWires();
        }
    }

    /// <summary>The compound of the closed wires of free edges (a null compound if there are none). Frozen.</summary>
    public Compound GetClosedWires() => Published(_wires);

    /// <summary>The closed wires, as <see cref="GetClosedWires"/>, unfrozen (ported code).</summary>
    internal Compound InternalGetClosedWires() => _wires;

    /// <summary>The compound of the open wires of free edges (a null compound if there are none). Frozen.</summary>
    public Compound GetOpenWires() => Published(_edges);

    /// <summary>The open wires, as <see cref="GetOpenWires"/>, unfrozen (ported code).</summary>
    internal Compound InternalGetOpenWires() => _edges;

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
        var edgeIndices = new List<int>();
        BrepBuilder b = new();

        int i; // svv #1
        for (i = 1; i <= edges.Length(); i++)
        {
            Orientation orientation = edges.Value(i).Orientation;
            if (orientation == Internal || orientation == External)
            {
                continue;
            }

            Wire wire = new();
            b.MakeWire(ref wire);
            b.Add(wire, edges.Value(i));
            iwires.Append(wire);
            edgeIndices.Add(i);
        }

        NCollection_HSequence<Shape> wires = InternalConnectWiresToWires(iwires, toler, shared);

        for (i = 1; i <= iwires.Length(); i++)
        {
            if (iwires.Value(i).Orientation == Reversed)
            {
                edges[edgeIndices[i - 1]].Reverse();
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
        var map = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
        return ConnectWiresToWires(iwires, toler, shared, map);
    }

    /// <summary>The connection, as <see cref="ConnectWiresToWires(NCollection_HSequence{Shape}, double, bool)"/>, unfrozen (ported code).</summary>
    internal static NCollection_HSequence<Shape> InternalConnectWiresToWires(NCollection_HSequence<Shape>? iwires, double toler, bool shared)
    {
        var map = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
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
                                                                          Dictionary<Shape, Shape> vertices)
    {
        NCollection_HSequence<Shape> owires = InternalConnectWiresToWires(iwires, toler, shared, vertices);
        foreach (KeyValuePair<Shape, Shape> pair in vertices)
        {
            pair.Value.TShape()?.FreezeGraph();
        }
        return PublishedAll(owires);
    }

    /// <summary>Connects the wires and fills the vertex map (deprecated form of the function returning the sequence).</summary>
    [Obsolete("Use ConnectWiresToWires() returning the sequence instead")]
    public static void ConnectWiresToWires(NCollection_HSequence<Shape>? iwires, double toler, bool shared,
                                           out NCollection_HSequence<Shape> owires,
                                           Dictionary<Shape, Shape> vertices)
    {
        owires = ConnectWiresToWires(iwires, toler, shared, vertices);
    }

    /// <summary>The connection with the vertex map, unfrozen (ported code).</summary>
    internal static NCollection_HSequence<Shape> InternalConnectWiresToWires(NCollection_HSequence<Shape>? iwires, double toler, bool shared,
                                                                                    Dictionary<Shape, Shape> vertices)
    {
        ConnectWiresToWiresImpl(iwires, toler, shared, out NCollection_HSequence<Shape> owires, vertices);
        return owires;
    }

    private static void ConnectWiresToWiresImpl(NCollection_HSequence<Shape>? iwires, double toler, bool shared,
                                                out NCollection_HSequence<Shape> owires,
                                                Dictionary<Shape, Shape> vertices)
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

        var sel = new BoxBndTreeSelectorAnalysis(arrwires, shared);
        ExtendWireData? sewd = null;
        for (i = 1; i <= arrwires.Length(); i++)
        {
            var wireData = new ExtendWireData(ShapeCasts.Wire(arrwires.Value(i)));
            if (wireData.EdgeCount > 0)
            {
                if (sewd is null)
                {
                    sewd = wireData;
                    sel.LoadList(i);
                }
            }
            else
            {
                if (wireData.NonManifoldEdgeCount > 0)
                {
                    owires.Append(wireData.InternalWire());
                }
                sel.LoadList(i);
            }
        }

        if (sewd is null)
        {
            return;
        }

        var saw = new WireAnalysis();
        saw.Load(sewd);
        saw.SetPrecision(tolerance);

        var bbTree = new SpatialTree<int, Box>();
        var treeFiller = new SpatialTreeFiller<int, Box>(bbTree);

        for (int inbW = 1; inbW <= arrwires.Length(); inbW++)
        {
            if (sel.ContWire(inbW))
            {
                continue;
            }
            Wire trW = ShapeCasts.Wire(arrwires.Value(inbW));
            Box box = new();
            ShapeAnalysisAnalysis.FindBounds(trW, out Vertex trV1, out Vertex trV2);
            Point3D trP1 = Tool.Pnt(trV1);
            Point3D trP2 = Tool.Pnt(trV2);
            box.Set(trP1);
            box.Add(trP2);
            box.SetGap(tolerance);
            treeFiller.Add(inbW, box);
        }

        treeFiller.Fill();
        int nsel;

        var sae = new EdgeAnalysis();
        bool done = false;

        while (!done)
        {
            bool found = false, tail = false, direct = false;
            int lwire = 0;
            sel.SetStop();
            Box fvBox = new(), lvBox = new();
            Vertex vf, vl;
            vf = sae.FirstVertex(sewd.Edge(1));
            vl = sae.LastVertex(sewd.Edge(sewd.EdgeCount));

            Point3D pf, pl;
            pf = Tool.Pnt(vf);
            pl = Tool.Pnt(vl);
            fvBox.Set(pf);
            fvBox.SetGap(tolerance);
            lvBox.Set(pl);
            lvBox.SetGap(tolerance);

            sel.DefineBoxes(fvBox, lvBox);

            if (shared)
            {
                sel.DefineVertexes(vf, vl);
            }
            else
            {
                sel.DefinePnt(pf, pl);
                sel.SetTolerance(tolerance);
            }

            nsel = bbTree.Select(sel);

            if (nsel != 0 && !sel.LastCheckStatus(ExtendStatus.Fail))
            {
                found = true;
                lwire = sel.GetNb();
                tail = sel.LastCheckStatus(ExtendStatus.DONE1) || sel.LastCheckStatus(ExtendStatus.DONE2);
                direct = sel.LastCheckStatus(ExtendStatus.DONE1) || sel.LastCheckStatus(ExtendStatus.DONE3);
                sel.LoadList(lwire);
            }

            if (found)
            {
                if (!direct)
                {
                    arrwires[lwire].Reverse();
                }

                var acurwd = new ExtendWireData(ShapeCasts.Wire(arrwires.Value(lwire)), true, true);
                if (acurwd.EdgeCount == 0)
                {
                    continue;
                }
                sewd.Add(acurwd, (tail ? 0 : 1));
            }
            else
            {
                for (i = 1; i <= saw.EdgeCount; i++)
                {
                    if (saw.CheckConnected(i))
                    {
                        int n2 = i;
                        int n1 = (n2 > 1 ? n2 - 1 : saw.EdgeCount);
                        Edge e1 = sewd.Edge(n1);
                        Edge e2 = sewd.Edge(n2);

                        Vertex vprev, vfol, v;
                        vprev = sae.LastVertex(e1);
                        vfol = sae.FirstVertex(e2);

                        if (saw.LastCheckStatus(ExtendStatus.DONE1))
                        {
                            v = vprev;
                        }
                        else
                        {
                            var sbv = new VertexBuilder();
                            v = sbv.InternalCombineVertex(vprev, vfol);
                        }
                        vertices[vprev] = v;
                        vertices[vfol] = v;

                        var sbe = new BuildEdgeBuilder();
                        if (saw.EdgeCount < 2)
                        {
                            sewd.Set(sbe.InternalCopyReplaceVertices(e2, v, v), n2);
                        }
                        else
                        {
                            sewd.Set(sbe.InternalCopyReplaceVertices(e2, v, new Vertex()), n2);
                            if (!saw.LastCheckStatus(ExtendStatus.DONE1))
                            {
                                sewd.Set(sbe.InternalCopyReplaceVertices(e1, new Vertex(), v), n1);
                            }
                        }
                    }
                }

                Wire wire = sewd.InternalWire();
                if (!saw.CheckConnected(1) && saw.LastCheckStatus(ExtendStatus.OK))
                {
                    wire.Closed(true);
                }

                owires.Append(wire);
                sewd.Clear();

                lwire = -1;
                for (i = 1; i <= arrwires.Length(); i++)
                {
                    if (!sel.ContWire(i))
                    {
                        sewd.Add(ShapeCasts.Wire(arrwires.Value(i)));
                        sel.LoadList(i);

                        if (sewd.EdgeCount > 0)
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

        BrepBuilder b = new();
        var sae = new EdgeAnalysis();

        var sewd = new ExtendWireData(wire);
        int nbedges = sewd.EdgeCount;

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
                bool searchBackward = true;

                for (;;)
                {
                    bool found;
                    Edge edge;
                    Vertex lvertex;
                    Point3D lpoint;

                    // searching for connection in ces
                    if (searchBackward)
                    {
                        searchBackward = false;
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
                            b.MakeWire(ref wire1);
                            for (int cesindex = jb; cesindex <= ces.Length(); cesindex++)
                            {
                                b.Add(wire1, sewd.Edge(ces.Value(cesindex)));
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
                        searchBackward = true;
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
        closed = Writable(closed);
        open = Writable(open);
        InternalDispatchWires(wires, ref closed, ref open);
        Published(closed);
        Published(open);
    }

    /// <summary>The dispatch, as <see cref="DispatchWires"/>, adding in place to unfrozen compounds (ported code).</summary>
    internal static void InternalDispatchWires(NCollection_HSequence<Shape>? wires, ref Compound closed, ref Compound open)
    {
        BrepBuilder b = new();
        if (closed.IsNull)
        {
            b.MakeCompound(ref closed);
        }
        if (open.IsNull)
        {
            b.MakeCompound(ref open);
        }
        if (wires is null)
        {
            return;
        }

        for (int iw = 1; iw <= wires.Length(); iw++)
        {
            if (wires.Value(iw).ClosedFlag)
            {
                b.Add(closed, wires.Value(iw));
            }
            else
            {
                b.Add(open, wires.Value(iw));
            }
        }
    }

    // A compound that can be added to: the compound itself when it is null or unfrozen, else a new compound holding
    // its items.
    private static Compound Writable(in Compound compound)
    {
        if (compound.IsNull || !compound.TShape()!.IsFrozen)
        {
            return compound;
        }
        Compound aNew = new();
        BrepBuilder b = new();
        b.MakeCompound(ref aNew);
        for (var it = new Iterator(compound, false, false); it.More(); it.Next())
        {
            b.Add(aNew, it.Value());
        }
        aNew.Location(compound.Placement, false);
        aNew.Orient(compound.Orientation);
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
        if (!_splitClosed && !_splitOpen)
        {
            return; // nothing to do
        }

        var see = new ExtendExplorer();
        NCollection_HSequence<Shape> closedwires, cw1, cw2, openwires, ow1, ow2;
        closedwires = see.SeqFromCompound(_wires, false);
        openwires = see.SeqFromCompound(_edges, false);

        if (_splitClosed)
        {
            InternalSplitWires(closedwires, _tolerance, _shared, out cw1, out ow1);
        }
        else
        {
            cw1 = closedwires;
            ow1 = new NCollection_HSequence<Shape>();
        }

        if (_splitOpen)
        {
            InternalSplitWires(openwires, _tolerance, _shared, out cw2, out ow2);
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
        _wires = ShapeCasts.Compound(compWires);
        _edges = ShapeCasts.Compound(compEdges);
    }

    // Freezes the graph of a shape handed to a caller (docs/phase3-design.md, section 4.2).
    private static Compound Published(in Compound shape)
    {
        shape.TShape()?.FreezeGraph();
        return shape;
    }

    // Freezes the graphs of the shapes of a sequence handed to a caller.
    private static NCollection_HSequence<Shape> PublishedAll(NCollection_HSequence<Shape> shapes)
    {
        for (int i = 1; i <= shapes.Length(); i++)
        {
            shapes.Value(i).TShape()?.FreezeGraph();
        }
        return shapes;
    }
}
