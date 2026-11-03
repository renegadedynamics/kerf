// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_FreeBounds.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_FreeBounds.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_FreeBounds.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// Outputs the free bounds of a shape (the wires of edges referenced by one face), as
/// <see cref="ShapeAnalysis_FreeBounds"/> does, and reduces the number of open wires by connecting adjacent open wires
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
public sealed class ShapeFix_FreeBounds
{
    private Compound myWires;
    private Compound myEdges;
    private Shape    myShape;
    private readonly bool   myShared;
    private readonly double mySewToler;
    private readonly double myCloseToler;
    private readonly bool   mySplitClosed;
    private readonly bool   mySplitOpen;

    /// <summary>Empty constructor.</summary>
    public ShapeFix_FreeBounds()
    {
        myWires = new Compound();
        myEdges = new Compound();
        myShape = new Shape();
    }

    /// <summary>
    /// Builds the forecast free bounds of <paramref name="shape"/> (a compound of faces) by sewing with
    /// <paramref name="sewtoler"/>, and connects the open wires with <paramref name="closetoler"/> (which must be
    /// greater than <paramref name="sewtoler"/>, else the open wires aren't connected).
    /// </summary>
    public ShapeFix_FreeBounds(in Shape shape, double sewtoler, double closetoler, bool splitclosed, bool splitopen)
        : this()
    {
        myShared      = false;
        mySewToler    = sewtoler;
        myCloseToler  = closetoler;
        mySplitClosed = splitclosed;
        mySplitOpen   = splitopen;
        myShape       = shape;
        Perform();
    }

    /// <summary>
    /// Builds the actual free bounds of <paramref name="shape"/> (a compound of shells) and connects the open wires
    /// with <paramref name="closetoler"/>.
    /// </summary>
    public ShapeFix_FreeBounds(in Shape shape, double closetoler, bool splitclosed, bool splitopen)
        : this()
    {
        myShared      = true;
        mySewToler    = 0.0;
        myCloseToler  = closetoler;
        mySplitClosed = splitclosed;
        mySplitOpen   = splitopen;
        myShape       = shape;
        Perform();
    }

    /// <summary>The compound of the closed wires of free edges, frozen.</summary>
    public Compound GetClosedWires() => Published(myWires);

    /// <summary>The compound of the open wires of free edges, frozen.</summary>
    public Compound GetOpenWires() => Published(myEdges);

    /// <summary>The source shape, modified where open wires were connected; frozen.</summary>
    public Shape GetShape()
    {
        myShape.TShape()?.FreezeGraph();
        return myShape;
    }

    private static Compound Published(in Compound theShape)
    {
        theShape.TShape()?.FreezeGraph();
        return theShape;
    }

    private bool Perform()
    {
        // OCCT.NET: the connection writes into the shape's edges; a frozen shape is copied (decision 583).
        var anEntry = new CopyAtEntry(myShape);
        myShape = anEntry.Shape();

        ShapeAnalysis_FreeBounds safb;
        if (myShared)
        {
            safb = new ShapeAnalysis_FreeBounds(myShape, mySplitClosed, mySplitOpen);
        }
        else
        {
            // OCCT.NET: the internal form, which doesn't freeze the free edges of the shape worked on
            safb = ShapeAnalysis_FreeBounds.InternalSewn(myShape, mySewToler, mySplitClosed, mySplitOpen);
        }
        myWires = safb.InternalGetClosedWires();
        myEdges = safb.InternalGetOpenWires();

        if (myCloseToler > mySewToler)
        {
            var see = new ShapeExtend_Explorer();
            NCollection_HSequence<Shape> open = see.SeqFromCompound(myEdges, false);
            var vertices = new NCollection_DataMap<Shape, Shape>(ShapeHasher.Instance);
            NCollection_HSequence<Shape> newwires =
                ShapeAnalysis_FreeBounds.InternalConnectWiresToWires(open, myCloseToler, myShared, vertices);
            myEdges.Nullify();
            ShapeAnalysis_FreeBounds.InternalDispatchWires(newwires, ref myWires, ref myEdges);

            for (var exp = new Explorer(myShape, ShapeType.Edge); exp.More(); exp.Next())
            {
                Edge Edge = ShapeCasts.Edge(exp.Current());
                // OCCT removes and adds vertices of the edge while iterating over them (reading a freed list node);
                // the port iterates over the vertices the edge had first (occt-issues.md).
                var aVertices = new List<Shape>();
                for (var iter = new Iterator(Edge); iter.More(); iter.Next())
                {
                    aVertices.Add(iter.Value());
                }
                foreach (Shape aV in aVertices)
                {
                    Vertex V = ShapeCasts.Vertex(aV);
                    var B = new BrepBuilder();
                    Vertex newV;
                    if (vertices.IsBound(V))
                    {
                        newV = ShapeCasts.Vertex(vertices.Find(V));
                        newV.Orientation(V.Orientation());
                        // OCCT removes and adds vertices of an edge that isn't free (it is in a wire): OCCT's release
                        // build skips the check, which raises TopoDS_FrozenShape in its debug build and in the port. The
                        // port lifts the flag around the edit, as ShapeFix_EdgeConnect does (occt-issues.md).
                        bool aFree = Edge.Free();
                        Edge.Free(true);
                        B.Remove(Edge, V);
                        B.Add(Edge, newV);
                        Edge.Free(aFree);
                    }
                }
            }
        }

        // OCCT.NET: the caller's own sub-shapes back wherever they are unchanged
        myShape = anEntry.Finish(myShape, [myWires, myEdges]);
        myWires = myWires.IsNull ? myWires : ShapeCasts.Compound(anEntry.Final(myWires));
        myEdges = myEdges.IsNull ? myEdges : ShapeCasts.Compound(anEntry.Final(myEdges));
        return true;
    }
}
