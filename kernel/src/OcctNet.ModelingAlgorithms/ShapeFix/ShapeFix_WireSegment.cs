// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_WireSegment.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_WireSegment.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>
/// An auxiliary class (data storage) of ShapeFix_ComposeShell: a segment of a wire (or a whole wire), held as a
/// <see cref="ShapeExtend_WireData"/>, with an orientation flag used for parity checking and the patch indices of its
/// edges.
/// </summary>
/// <remarks>
/// <para>
/// The orientation flag: FORWARD and REVERSED say that the segment was traversed once in that direction and must be
/// traversed once more in the opposite one; EXTERNAL that it was not yet traversed (not yet used as a boundary);
/// INTERNAL that it was traversed in both directions and is out of further work. Segments of the initial bounding
/// wires are created REVERSED (the outer wire) or FORWARD (inner wires), segments of splitting seams EXTERNAL.
/// </para>
/// <para>
/// A struct, as OCCT's class is used by value: a copy shares the wire data and the index sequences (OCCT's handles) and
/// copies the orientation and the vertex. It holds working objects, so it is not a value for callers: no equality.
/// Its mutators are internal (ShapeFix_ComposeShell's, unit 6H3). <c>default</c> holds nulls; write
/// <c>new ShapeFix_WireSegment()</c>, which makes an empty segment as OCCT's default constructor. It writes into no
/// shape. OCCT declares a constructor from a TopoDS_Wire that it never defines; it is left out.
/// </para>
/// </remarks>
public struct ShapeFix_WireSegment
{
    private ShapeExtend_WireData        myWire;
    private TopoDS_Vertex               myVertex;
    private TopAbs_Orientation          myOrient;
    private NCollection_HSequence<int>  myIUMin;
    private NCollection_HSequence<int>  myIUMax;
    private NCollection_HSequence<int>  myIVMin;
    private NCollection_HSequence<int>  myIVMax;

    private const int MININD = -32000;
    private const int MAXIND = 32000;

    /// <summary>Creates an empty segment.</summary>
    public ShapeFix_WireSegment()
    {
        Clear();
        myOrient = TopAbs_FORWARD;
    }

    /// <summary>Creates a segment and initializes it with a wire and an orientation.</summary>
    public ShapeFix_WireSegment(ShapeExtend_WireData wire, TopAbs_Orientation ori = TopAbs_EXTERNAL)
    {
        Load(wire);
        myOrient = ori;
    }

    /// <summary>Clears all fields.</summary>
    [System.Diagnostics.CodeAnalysis.MemberNotNull(nameof(myWire), nameof(myIUMin), nameof(myIUMax), nameof(myIVMin),
                                                   nameof(myIVMax))]
    internal void Clear()
    {
        myWire                = new ShapeExtend_WireData();
        myWire.ManifoldMode() = false;
        myIUMin               = new NCollection_HSequence<int>();
        myIUMax               = new NCollection_HSequence<int>();
        myIVMin               = new NCollection_HSequence<int>();
        myIVMax               = new NCollection_HSequence<int>();
        myVertex              = new TopoDS_Vertex();
    }

    /// <summary>Loads a wire (its edges are added to a new wire data, with indefinite patch indices).</summary>
    [System.Diagnostics.CodeAnalysis.MemberNotNull(nameof(myWire), nameof(myIUMin), nameof(myIUMax), nameof(myIVMin),
                                                   nameof(myIVMax))]
    internal void Load(ShapeExtend_WireData wire)
    {
        //  myWire = wire;
        Clear();
        myWire.ManifoldMode() = wire.ManifoldMode();
        for (int i = 1; i <= wire.NbEdges(); i++)
        {
            AddEdge(i, wire.Edge(i));
        }
    }

    /// <summary>Returns the wire.</summary>
    public readonly ShapeExtend_WireData WireData() => myWire;

    /// <summary>Sets the orientation flag.</summary>
    internal void Orientation(TopAbs_Orientation ori)
    {
        myOrient = ori;
    }

    /// <summary>Returns the orientation flag.</summary>
    public readonly TopAbs_Orientation Orientation() => myOrient;

    /// <summary>Returns the first vertex of the first edge of the wire (whatever <see cref="Orientation()"/>).</summary>
    public readonly TopoDS_Vertex FirstVertex()
    {
        var sae = new ShapeAnalysis_Edge();
        return sae.FirstVertex(myWire.Edge(1));
    }

    /// <summary>Returns the last vertex of the last edge of the wire (whatever <see cref="Orientation()"/>).</summary>
    public readonly TopoDS_Vertex LastVertex()
    {
        var sae = new ShapeAnalysis_Edge();
        return sae.LastVertex(myWire.Edge(myWire.NbEdges()));
    }

    /// <summary>True if FirstVertex() is LastVertex().</summary>
    public readonly bool IsClosed()
    {
        TopoDS_Vertex v;
        v = FirstVertex();
        return v.IsSame(LastVertex());
    }

    // WORK with EDGES and PATCH INDICES

    /// <summary>Returns the number of edges in the wire.</summary>
    public readonly int NbEdges() => myWire.NbEdges();

    /// <summary>Returns the edge of index <paramref name="i"/>.</summary>
    public readonly TopoDS_Edge Edge(int i) => myWire.Edge(i);

    /// <summary>Replaces the edge of index <paramref name="i"/>.</summary>
    internal readonly void SetEdge(int i, in TopoDS_Edge edge)
    {
        myWire.Set(edge, i);
    }

    /// <summary>
    /// Inserts an edge at index <paramref name="i"/> (at the end if 0) with indefinite patch indices.
    /// </summary>
    internal readonly void AddEdge(int i, in TopoDS_Edge edge)
    {
        AddEdge(i, edge, MININD, MAXIND, MININD, MAXIND);
    }

    /// <summary>Inserts an edge at index <paramref name="i"/> (at the end if 0) with the given patch indices.</summary>
    internal readonly void AddEdge(int i, in TopoDS_Edge edge, int iumin, int iumax, int ivmin, int ivmax)
    {
        myWire.Add(edge, i);
        if (i == 0)
        {
            myIUMin.Append(iumin);
            myIUMax.Append(iumax);
            myIVMin.Append(ivmin);
            myIVMax.Append(ivmax);
        }
        else
        {
            myIUMin.InsertBefore(i, iumin);
            myIUMax.InsertBefore(i, iumax);
            myIVMin.InsertBefore(i, ivmin);
            myIVMax.InsertBefore(i, ivmax);
        }
    }

    /// <summary>Sets the patch indices of edge <paramref name="i"/>.</summary>
    internal readonly void SetPatchIndex(int i, int iumin, int iumax, int ivmin, int ivmax)
    {
        myIUMin.SetValue(i, iumin);
        myIUMax.SetValue(i, iumax);
        myIVMin.SetValue(i, ivmin);
        myIVMax.SetValue(i, ivmax);
    }

    internal readonly void DefineIUMin(int i, int iumin)
    {
        if (myIUMin.Value(i) < iumin)
        {
            myIUMin.SetValue(i, iumin);
        }
    }

    internal readonly void DefineIUMax(int i, int iumax)
    {
        if (myIUMax.Value(i) > iumax)
        {
            myIUMax.SetValue(i, iumax);
        }
    }

    internal readonly void DefineIVMin(int i, int ivmin)
    {
        if (myIVMin.Value(i) < ivmin)
        {
            myIVMin.SetValue(i, ivmin);
        }
    }

    /// <summary>
    /// Modifies the minimal or maximal patch index of edge <paramref name="i"/> (the Define... methods), so that
    /// iumin &lt;= IUMin(i) &lt;= IUMax(i) &lt;= iumax.
    /// </summary>
    internal readonly void DefineIVMax(int i, int ivmax)
    {
        if (myIVMax.Value(i) > ivmax)
        {
            myIVMax.SetValue(i, ivmax);
        }
    }

    /// <summary>Returns the patch indices of edge <paramref name="i"/>.</summary>
    public readonly void GetPatchIndex(int i, out int iumin, out int iumax, out int ivmin, out int ivmax)
    {
        iumin = myIUMin.Value(i);
        iumax = myIUMax.Value(i);
        ivmin = myIVMin.Value(i);
        ivmax = myIVMax.Value(i);
    }

    /// <summary>Checks that the patch indices of edge <paramref name="i"/> satisfy IUMin(i) &lt;= IUMax(i) &lt;= IUMin(i)+1.</summary>
    public readonly bool CheckPatchIndex(int i)
    {
        int  dU = myIUMax.Value(i) - myIUMin.Value(i);
        int  dV = myIVMax.Value(i) - myIVMin.Value(i);
        bool ok = (dU == 0 || dU == 1) && (dV == 0 || dV == 1);
        return ok;
    }

    internal void SetVertex(in TopoDS_Vertex theVertex)
    {
        myVertex = theVertex;
        // SetVertex(theVertex, MININD, MAXIND, MININD, MAXIND);
    }

    public readonly TopoDS_Vertex GetVertex() => myVertex;

    public readonly bool IsVertex() => !myVertex.IsNull();
}
