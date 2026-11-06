// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_WireSegment.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_WireSegment.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Extend;
using Kerf.Topology;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Healing.Fixes;

/// <summary>
/// An auxiliary class (data storage) of ShapeFix_ComposeShell: a segment of a wire (or a whole wire), held as a
/// <see cref="ExtendWireData"/>, with an orientation flag used for parity checking and the patch indices of its
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
internal struct WireSegmentFixer
{
    private ExtendWireData        _wire;
    private Vertex               _vertex;
    private Orientation          _orient;
    private NCollection_HSequence<int>  _iuMin;
    private NCollection_HSequence<int>  _iuMax;
    private NCollection_HSequence<int>  _ivMin;
    private NCollection_HSequence<int>  _ivMax;

    private const int Minind = -32000;
    private const int Maxind = 32000;

    /// <summary>Creates an empty segment.</summary>
    public WireSegmentFixer()
    {
        Clear();
        _orient = Forward;
    }

    /// <summary>Creates a segment and initializes it with a wire and an orientation.</summary>
    public WireSegmentFixer(ExtendWireData wire, Orientation ori = External)
    {
        Load(wire);
        _orient = ori;
    }

    /// <summary>Clears all fields.</summary>
    [System.Diagnostics.CodeAnalysis.MemberNotNull(nameof(_wire), nameof(_iuMin), nameof(_iuMax), nameof(_ivMin),
                                                   nameof(_ivMax))]
    internal void Clear()
    {
        _wire                = new ExtendWireData();
        _wire.ManifoldMode() = false;
        _iuMin               = new NCollection_HSequence<int>();
        _iuMax               = new NCollection_HSequence<int>();
        _ivMin               = new NCollection_HSequence<int>();
        _ivMax               = new NCollection_HSequence<int>();
        _vertex              = new Vertex();
    }

    /// <summary>Loads a wire (its edges are added to a new wire data, with indefinite patch indices).</summary>
    [System.Diagnostics.CodeAnalysis.MemberNotNull(nameof(_wire), nameof(_iuMin), nameof(_iuMax), nameof(_ivMin),
                                                   nameof(_ivMax))]
    internal void Load(ExtendWireData wire)
    {
        //  myWire = wire;
        Clear();
        _wire.ManifoldMode() = wire.ManifoldMode();
        for (int i = 1; i <= wire.EdgeCount; i++)
        {
            AddEdge(i, wire.Edge(i));
        }
    }

    /// <summary>Returns the wire.</summary>
    public readonly ExtendWireData WireData() => _wire;

    /// <summary>Sets the orientation flag.</summary>
    internal void Orientation(Orientation ori)
    {
        _orient = ori;
    }

    /// <summary>Returns the orientation flag.</summary>
    public readonly Orientation Orientation() => _orient;

    /// <summary>Returns the first vertex of the first edge of the wire (whatever <see cref="Orientation()"/>).</summary>
    public readonly Vertex FirstVertex()
    {
        var sae = new EdgeAnalysis();
        return sae.FirstVertex(_wire.Edge(1));
    }

    /// <summary>Returns the last vertex of the last edge of the wire (whatever <see cref="Orientation()"/>).</summary>
    public readonly Vertex LastVertex()
    {
        var sae = new EdgeAnalysis();
        return sae.LastVertex(_wire.Edge(_wire.EdgeCount));
    }

    /// <summary>True if FirstVertex() is LastVertex().</summary>
    public readonly bool IsClosed()
    {
        Vertex v;
        v = FirstVertex();
        return v.IsSame(LastVertex());
    }

    // WORK with EDGES and PATCH INDICES

    /// <summary>Returns the number of edges in the wire.</summary>
    public readonly int EdgeCount => _wire.EdgeCount;

    /// <summary>Returns the edge of index <paramref name="i"/>.</summary>
    public readonly Edge Edge(int i) => _wire.Edge(i);

    /// <summary>Replaces the edge of index <paramref name="i"/>.</summary>
    internal readonly void SetEdge(int i, in Edge edge)
    {
        _wire.Set(edge, i);
    }

    /// <summary>
    /// Inserts an edge at index <paramref name="i"/> (at the end if 0) with indefinite patch indices.
    /// </summary>
    internal readonly void AddEdge(int i, in Edge edge)
    {
        AddEdge(i, edge, Minind, Maxind, Minind, Maxind);
    }

    /// <summary>Inserts an edge at index <paramref name="i"/> (at the end if 0) with the given patch indices.</summary>
    internal readonly void AddEdge(int i, in Edge edge, int iumin, int iumax, int ivmin, int ivmax)
    {
        _wire.Add(edge, i);
        if (i == 0)
        {
            _iuMin.Append(iumin);
            _iuMax.Append(iumax);
            _ivMin.Append(ivmin);
            _ivMax.Append(ivmax);
        }
        else
        {
            _iuMin.InsertBefore(i, iumin);
            _iuMax.InsertBefore(i, iumax);
            _ivMin.InsertBefore(i, ivmin);
            _ivMax.InsertBefore(i, ivmax);
        }
    }

    /// <summary>Sets the patch indices of edge <paramref name="i"/>.</summary>
    internal readonly void SetPatchIndex(int i, int iumin, int iumax, int ivmin, int ivmax)
    {
        _iuMin.SetValue(i, iumin);
        _iuMax.SetValue(i, iumax);
        _ivMin.SetValue(i, ivmin);
        _ivMax.SetValue(i, ivmax);
    }

    internal readonly void DefineIUMin(int i, int iumin)
    {
        if (_iuMin.Value(i) < iumin)
        {
            _iuMin.SetValue(i, iumin);
        }
    }

    internal readonly void DefineIUMax(int i, int iumax)
    {
        if (_iuMax.Value(i) > iumax)
        {
            _iuMax.SetValue(i, iumax);
        }
    }

    internal readonly void DefineIVMin(int i, int ivmin)
    {
        if (_ivMin.Value(i) < ivmin)
        {
            _ivMin.SetValue(i, ivmin);
        }
    }

    /// <summary>
    /// Modifies the minimal or maximal patch index of edge <paramref name="i"/> (the Define... methods), so that
    /// iumin &lt;= IUMin(i) &lt;= IUMax(i) &lt;= iumax.
    /// </summary>
    internal readonly void DefineIVMax(int i, int ivmax)
    {
        if (_ivMax.Value(i) > ivmax)
        {
            _ivMax.SetValue(i, ivmax);
        }
    }

    /// <summary>Returns the patch indices of edge <paramref name="i"/>.</summary>
    public readonly void GetPatchIndex(int i, out int iumin, out int iumax, out int ivmin, out int ivmax)
    {
        iumin = _iuMin.Value(i);
        iumax = _iuMax.Value(i);
        ivmin = _ivMin.Value(i);
        ivmax = _ivMax.Value(i);
    }

    /// <summary>Checks that the patch indices of edge <paramref name="i"/> satisfy IUMin(i) &lt;= IUMax(i) &lt;= IUMin(i)+1.</summary>
    public readonly bool CheckPatchIndex(int i)
    {
        int  dU = _iuMax.Value(i) - _iuMin.Value(i);
        int  dV = _ivMax.Value(i) - _ivMin.Value(i);
        bool ok = (dU == 0 || dU == 1) && (dV == 0 || dV == 1);
        return ok;
    }

    internal void SetVertex(in Vertex vertex)
    {
        _vertex = vertex;
        // SetVertex(theVertex, MININD, MAXIND, MININD, MAXIND);
    }

    public readonly Vertex GetVertex() => _vertex;

    public readonly bool IsVertex => !_vertex.IsNull;
}
