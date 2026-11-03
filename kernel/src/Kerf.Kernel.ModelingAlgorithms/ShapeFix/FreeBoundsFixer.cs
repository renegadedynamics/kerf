// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_FreeBounds.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_FreeBounds.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_FreeBounds.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Healing.Fixes;

/// <summary>
/// Outputs the free bounds of a shape (the wires of edges referenced by one face), as
/// <see cref="FreeBounds"/> does, and reduces the number of open wires by connecting adjacent open wires
/// with a given tolerance into longer or closed wires. Where open wires are connected, their end vertices are replaced
/// by new connecting vertices, also in the edges of the shape that share them: the source shape is modified.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21; OCCT uses it by value). The connection writes into the edges of the shape it is given
/// (vertices removed and added). A shape holding frozen shapes (any shape a caller holds) is copied first
/// (<see cref="CopyAtEntry"/>, decision 583): the caller's shape never changes, and
/// <see cref="GetShape"/> is the modified copy with the caller's own sub-shapes wherever they are unchanged (the shape
/// itself when nothing changed). The results are frozen.
/// </para>
/// </remarks>
public sealed class FreeBoundsFixer
{
    private Compound _wires;
    private Compound _edges;
    private Shape    _shape;
    private readonly bool   _shared;
    private readonly double _sewToler;
    private readonly double _closeToler;
    private readonly bool   _splitClosed;
    private readonly bool   _splitOpen;

    /// <summary>Empty constructor.</summary>
    public FreeBoundsFixer()
    {
        _wires = new Compound();
        _edges = new Compound();
        _shape = new Shape();
    }

    /// <summary>
    /// Builds the forecast free bounds of <paramref name="shape"/> (a compound of faces) by sewing with
    /// <paramref name="sewtoler"/>, and connects the open wires with <paramref name="closetoler"/> (which must be
    /// greater than <paramref name="sewtoler"/>, else the open wires aren't connected).
    /// </summary>
    public FreeBoundsFixer(in Shape shape, double sewtoler, double closetoler, bool splitclosed, bool splitopen)
        : this()
    {
        _shared      = false;
        _sewToler    = sewtoler;
        _closeToler  = closetoler;
        _splitClosed = splitclosed;
        _splitOpen   = splitopen;
        _shape       = shape;
        Perform();
    }

    /// <summary>
    /// Builds the actual free bounds of <paramref name="shape"/> (a compound of shells) and connects the open wires
    /// with <paramref name="closetoler"/>.
    /// </summary>
    public FreeBoundsFixer(in Shape shape, double closetoler, bool splitclosed, bool splitopen)
        : this()
    {
        _shared      = true;
        _sewToler    = 0.0;
        _closeToler  = closetoler;
        _splitClosed = splitclosed;
        _splitOpen   = splitopen;
        _shape       = shape;
        Perform();
    }

    /// <summary>The compound of the closed wires of free edges, frozen.</summary>
    public Compound GetClosedWires() => Published(_wires);

    /// <summary>The compound of the open wires of free edges, frozen.</summary>
    public Compound GetOpenWires() => Published(_edges);

    /// <summary>The source shape, modified where open wires were connected; frozen.</summary>
    public Shape GetShape()
    {
        _shape.TShape()?.FreezeGraph();
        return _shape;
    }

    private static Compound Published(in Compound shape)
    {
        shape.TShape()?.FreezeGraph();
        return shape;
    }

    private bool Perform()
    {
        // OCCT.NET: the connection writes into the shape's edges; a frozen shape is copied (decision 583).
        var entry = new CopyAtEntry(_shape);
        _shape = entry.Shape();

        FreeBounds safb;
        if (_shared)
        {
            safb = new FreeBounds(_shape, _splitClosed, _splitOpen);
        }
        else
        {
            // OCCT.NET: the internal form, which doesn't freeze the free edges of the shape worked on
            safb = FreeBounds.InternalSewn(_shape, _sewToler, _splitClosed, _splitOpen);
        }
        _wires = safb.InternalGetClosedWires();
        _edges = safb.InternalGetOpenWires();

        if (_closeToler > _sewToler)
        {
            var see = new ExtendExplorer();
            NCollection_HSequence<Shape> open = see.SeqFromCompound(_edges, false);
            var vertices = new NCollection_DataMap<Shape, Shape>(ShapeHasher.Instance);
            NCollection_HSequence<Shape> newwires =
                FreeBounds.InternalConnectWiresToWires(open, _closeToler, _shared, vertices);
            _edges.Nullify();
            FreeBounds.InternalDispatchWires(newwires, ref _wires, ref _edges);

            for (var exp = new Explorer(_shape, ShapeType.Edge); exp.More(); exp.Next())
            {
                Edge edge = ShapeCasts.Edge(exp.Current());
                // OCCT removes and adds vertices of the edge while iterating over them (reading a freed list node);
                // the port iterates over the vertices the edge had first (occt-issues.md).
                var aVertices = new List<Shape>();
                for (var iter = new Iterator(edge); iter.More(); iter.Next())
                {
                    aVertices.Add(iter.Value());
                }
                foreach (Shape aV in aVertices)
                {
                    Vertex v = ShapeCasts.Vertex(aV);
                    var b = new BrepBuilder();
                    Vertex newV;
                    if (vertices.IsBound(v))
                    {
                        newV = ShapeCasts.Vertex(vertices.Find(v));
                        newV.Orientation(v.Orientation());
                        // OCCT removes and adds vertices of an edge that isn't free (it is in a wire): OCCT's release
                        // build skips the check, which raises TopoDS_FrozenShape in its debug build and in the port. The
                        // port lifts the flag around the edit, as ShapeFix_EdgeConnect does (occt-issues.md).
                        bool free = edge.Free();
                        edge.Free(true);
                        b.Remove(edge, v);
                        b.Add(edge, newV);
                        edge.Free(free);
                    }
                }
            }
        }

        // OCCT.NET: the caller's own sub-shapes back wherever they are unchanged
        _shape = entry.Finish(_shape, [_wires, _edges]);
        _wires = _wires.IsNull ? _wires : ShapeCasts.Compound(entry.Final(_wires));
        _edges = _edges.IsNull ? _edges : ShapeCasts.Compound(entry.Final(_edges));
        return true;
    }
}
