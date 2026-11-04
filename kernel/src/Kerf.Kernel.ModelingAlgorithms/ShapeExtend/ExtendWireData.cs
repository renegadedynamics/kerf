// Port of: src/ModelingAlgorithms/TKShHealing/ShapeExtend/ShapeExtend_WireData.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeExtend/ShapeExtend_WireData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Healing.Extend;

/// <summary>
/// A wire as an ordered list of edges, with direct access by index (from 1; a negative index gives the edge
/// reversed), its seams computed on demand, and the edges that aren't FORWARD or REVERSED (INTERNAL, EXTERNAL) kept
/// apart in manifold mode.
/// </summary>
/// <remarks>
/// A working object (decision 21) with OCCT's public mutators: it holds the caller's edges and changes only its own
/// list. <see cref="Wire"/> and <see cref="WireAPIMake"/> freeze the wire they return; ported code that goes on
/// building uses <see cref="InternalWire"/> and <see cref="InternalWireAPIMake"/>. <see cref="Reverse(in Face)"/>
/// swaps the pcurves of seam edges in place, so it is internal.
/// </remarks>
internal class ExtendWireData : Transient
{
    private NCollection_HSequence<Shape> _edges = null!;
    private NCollection_HSequence<Shape> _nonmanifoldEdges = null!;
    private NCollection_HSequence<int>? _seams;
    private readonly NCollection_PackedMapOfInteger _seamsCache = new NCollection_PackedMapOfInteger();
    private int _seamF;
    private int _seamR;
    private bool _manifoldMode;

    /// <summary>An empty wire.</summary>
    public ExtendWireData()
    {
        Clear();
    }

    /// <summary>
    /// The edges of the wire, as <see cref="Init(in Wire, bool, bool)"/> loads them.
    /// </summary>
    public ExtendWireData(in Wire wire, bool chained = true, bool manifoldMode = true)
    {
        Init(wire, chained, manifoldMode);
    }

    /// <summary>Copies the edges (and non-manifold edges and mode) of another wire data.</summary>
    public void Init(ExtendWireData other)
    {
        Clear();
        int i, nb = other.EdgeCount;
        for (i = 1; i <= nb; i++)
        {
            Add(other.Edge(i));
        }

        nb = other.NonManifoldEdgeCount;
        for (i = 1; i <= nb; i++)
        {
            Add(other.NonmanifoldEdge(i));
        }

        _manifoldMode = other.ManifoldMode();
    }

    /// <summary>
    /// Loads the edges of the wire in their stored order (reversed for a reversed wire). Returns false if they don't
    /// chain; then, if not <paramref name="chained"/>, they are loaded again in the order of BRepTools_WireExplorer.
    /// In manifold mode the INTERNAL and EXTERNAL edges are kept apart.
    /// </summary>
    public bool Init(in Wire wire, bool chained = true, bool manifoldMode = true)
    {
        Clear();
        _manifoldMode = manifoldMode;
        bool ok = true;
        Vertex vlast = new Vertex();
        for (var it = new Iterator(wire); it.More(); it.Next())
        {
            Edge e = ShapeCasts.Edge(it.Value());

            // protect against INTERNAL/EXTERNAL edges
            if ((e.Orientation != Reversed && e.Orientation != Forward))
            {
                _nonmanifoldEdges.Append(e);
                continue;
            }

            Vertex v1 = new Vertex(), v2 = new Vertex();
            for (var itv = new Iterator(e); itv.More(); itv.Next())
            {
                Vertex v = ShapeCasts.Vertex(itv.Value());
                if (v.Orientation == Forward)
                {
                    v1 = v;
                }
                else if (v.Orientation == Reversed)
                {
                    v2 = v;
                }
            }

            // chainage? Si pas bon et chained False on repart sur WireExplorer
            if (!vlast.IsNull && !vlast.IsSame(v1) && manifoldMode)
            {
                ok = false;
                if (!chained)
                {
                    break;
                }
            }

            vlast = v2;
            if (wire.Orientation == Reversed)
            {
                _edges.Prepend(e);
            }
            else
            {
                _edges.Append(e);
            }
        }

        if (!_manifoldMode)
        {
            int nb = _nonmanifoldEdges.Length();
            int i = 1;
            for (; i <= nb; i++)
            {
                _edges.Append(_nonmanifoldEdges.Value(i));
            }

            _nonmanifoldEdges.Clear();
        }

        //    refaire chainage ?  Par WireExplorer
        if (ok || chained)
        {
            return ok;
        }

        Clear();
        for (var we = new WireExplorer(wire); we.More(); we.Next())
        {
            _edges.Append(ShapeCasts.Edge(we.Current()));
        }

        return ok;
    }

    /// <summary>Clears the wire (and sets manifold mode).</summary>
    public void Clear()
    {
        _edges = new NCollection_HSequence<Shape>();
        _nonmanifoldEdges = new NCollection_HSequence<Shape>();
        _seamF = _seamR = -1;
        _seams = null;
        _manifoldMode = true;
    }

    /// <summary>
    /// Computes the seams: edges present twice, FORWARD and REVERSED. Done once, unless <paramref name="enforce"/>.
    /// </summary>
    public void ComputeSeams(bool enforce = true)
    {
        if (_seamF >= 0 && !enforce)
        {
            return;
        }

        _seams = new NCollection_HSequence<int>();
        _seamF = _seamR = 0;
        Shape s;
        int i, nb = EdgeCount;
        var me = new IndexedSet<Shape>(ShapeHasher.Instance);
        int[] se = new int[nb + 1];

        //  deux passes : d abord on mappe les Edges REVERSED
        //  Pour chacune, on note aussi son RANG dans la liste
        for (i = 1; i <= nb; i++)
        {
            s = Edge(i);
            if (s.Orientation == Reversed)
            {
                int num = me.Add(s) + 1;
                se[num] = i;
            }
        }

        //  ensuite on voit les Edges FORWARD qui y seraient deja -> on note leur n0
        //  c-a-d le n0 de la directe ET de la reverse
        for (i = 1; i <= nb; i++)
        {
            s = Edge(i);
            if (s.Orientation == Reversed)
            {
                continue;
            }

            int num = me.IndexOf(s) + 1;
            if (num <= 0)
            {
                continue;
            }

            if (_seamF == 0)
            {
                _seamF = i;
                _seamR = se[num];
            }
            else
            {
                _seams.Append(i);
                _seams.Append(se[num]);
            }
        }

        _seamsCache.Clear();
        for (i = 1; i <= _seams.Length(); i++)
        {
            _seamsCache.Add(_seams.Value(i));
        }
    }

    /// <summary>Makes edge <paramref name="num"/> the last one, by a circular shift (nothing for 0).</summary>
    public void SetLast(int num)
    {
        if (num == 0)
        {
            return;
        }

        int i, nb = EdgeCount;
        for (i = nb; i > num; i--)
        {
            Edge edge = ShapeCasts.Edge(_edges.Value(nb));
            _edges.Remove(nb);
            _edges.InsertBefore(1, edge);
        }

        _seamF = -1;
    }

    /// <summary>Makes the first degenerated edge the last one (see <see cref="SetLast"/>).</summary>
    public void SetDegeneratedLast()
    {
        int i, nb = EdgeCount;
        for (i = 1; i <= nb; i++)
        {
            if (Tool.Degenerated(Edge(i)))
            {
                SetLast(i);
                return;
            }
        }
    }

    /// <summary>
    /// Adds an edge at the end, or before edge <paramref name="atnum"/> if not 0. In manifold mode an INTERNAL or
    /// EXTERNAL edge goes to the non-manifold edges.
    /// </summary>
    public void Add(in Edge edge, int atnum = 0)
    {
        if (edge.Orientation != Reversed && edge.Orientation != Forward && _manifoldMode)
        {
            _nonmanifoldEdges.Append(edge);
            return;
        }

        if (edge.IsNull)
        {
            return;
        }

        if (atnum == 0)
        {
            _edges.Append(edge);
        }
        else
        {
            _edges.InsertBefore(atnum, edge);
        }

        _seamF = -1;
    }

    /// <summary>Adds the edges of a wire, in their stored order, as <see cref="Add(in Edge, int)"/>.</summary>
    public void Add(in Wire wire, int atnum = 0)
    {
        if (wire.IsNull)
        {
            return;
        }

        int n = atnum;
        var nmEdges = new NCollection_Sequence<Shape>();
        for (var it = new Iterator(wire); it.More(); it.Next())
        {
            Edge edge = ShapeCasts.Edge(it.Value());
            if (edge.Orientation != Reversed && edge.Orientation != Forward)
            {
                if (_manifoldMode)
                {
                    _nonmanifoldEdges.Append(edge);
                }
                else
                {
                    nmEdges.Append(edge);
                }

                continue;
            }

            if (n == 0)
            {
                _edges.Append(edge);
            }
            else
            {
                _edges.InsertBefore(n, edge);
                n++;
            }
        }

        int i = 1, nb = nmEdges.Length();
        for (; i <= nb; i++)
        {
            _edges.Append(nmEdges.Value(i));
        }

        _seamF = -1;
    }

    /// <summary>Adds the edges of another wire data, as <see cref="Add(in Edge, int)"/>.</summary>
    public void Add(ExtendWireData? wire, int atnum = 0)
    {
        if (wire is null)
        {
            return;
        }

        var nmEdges = new NCollection_Sequence<Shape>();
        int n = atnum;
        int i = 1;
        for (; i <= wire.EdgeCount; i++)
        {
            Edge e = wire.Edge(i);
            if (e.Orientation == Internal || e.Orientation == External)
            {
                nmEdges.Append(e);
                continue;
            }

            if (n == 0)
            {
                _edges.Append(wire.Edge(i));
            }
            else
            {
                _edges.InsertBefore(n, wire.Edge(i));
                n++;
            }
        }

        // non-manifold edges for non-manifold wire should be added at end
        for (i = 1; i <= nmEdges.Length(); i++)
        {
            _edges.Append(nmEdges.Value(i));
        }

        for (i = 1; i <= wire.NonManifoldEdgeCount; i++)
        {
            if (_manifoldMode)
            {
                _nonmanifoldEdges.Append(wire.NonmanifoldEdge(i));
            }
            else
            {
                // OCCT adds the i-th ordinary edge of the other wire here, not its i-th non-manifold edge
                if (n == 0)
                {
                    _edges.Append(wire.Edge(i));
                }
                else
                {
                    _edges.InsertBefore(n, wire.Edge(i));
                    n++;
                }
            }
        }

        _seamF = -1;
    }

    /// <summary>Adds an edge or the edges of a wire; other shapes are ignored.</summary>
    public void Add(in Shape shape, int atnum = 0)
    {
        if (shape.Type == global::Kerf.Topology.ShapeType.Edge)
        {
            Add(ShapeCasts.Edge(shape), atnum);
        }
        else if (shape.Type == global::Kerf.Topology.ShapeType.Wire)
        {
            Add(ShapeCasts.Wire(shape), atnum);
        }
    }

    /// <summary>
    /// Adds an edge: <paramref name="mode"/> 0 at the end, 1 reversed at the end, 2 at the start, 3 reversed at the
    /// start; nothing if negative.
    /// </summary>
    public void AddOriented(in Edge edge, int mode)
    {
        if (edge.IsNull || mode < 0)
        {
            return;
        }

        Edge e = edge;
        if (mode == 1 || mode == 3)
        {
            e.Reverse();
        }

        Add(e, mode / 2); // mode = 0,1 -> 0  mode = 2,3 -> 1
    }

    /// <summary>Adds the edges of a wire, as <see cref="AddOriented(in Edge, int)"/>.</summary>
    public void AddOriented(in Wire wire, int mode)
    {
        if (wire.IsNull || mode < 0)
        {
            return;
        }

        Wire w = wire;
        if (mode == 1 || mode == 3)
        {
            w.Reverse();
        }

        Add(w, mode / 2); // mode = 0,1 -> 0  mode = 2,3 -> 1
    }

    /// <summary>Adds an edge or the edges of a wire, as <see cref="AddOriented(in Edge, int)"/>.</summary>
    public void AddOriented(in Shape shape, int mode)
    {
        if (shape.Type == global::Kerf.Topology.ShapeType.Edge)
        {
            AddOriented(ShapeCasts.Edge(shape), mode);
        }
        else if (shape.Type == global::Kerf.Topology.ShapeType.Wire)
        {
            AddOriented(ShapeCasts.Wire(shape), mode);
        }
    }

    /// <summary>Removes edge <paramref name="num"/> (the last one for 0).</summary>
    public void Remove(int num = 0)
    {
        _edges.Remove(num > 0 ? num : EdgeCount);

        _seamF = -1;
    }

    /// <summary>
    /// Replaces edge <paramref name="num"/> (the last one for 0). In manifold mode an INTERNAL or EXTERNAL edge
    /// replaces the non-manifold edge <paramref name="num"/>, or is appended to them.
    /// </summary>
    public void Set(in Edge edge, int num = 0)
    {
        if (edge.Orientation != Reversed && edge.Orientation != Forward && _manifoldMode)
        {
            if (num <= _nonmanifoldEdges.Length())
            {
                _nonmanifoldEdges.SetValue(num, edge);
            }
            else
            {
                _nonmanifoldEdges.Append(edge);
            }
        }
        else
        {
            _edges.SetValue((num > 0 ? num : EdgeCount), edge);
        }

        _seamF = -1;
    }

    /// <summary>
    /// OCCT.NET: replaces the edge <paramref name="num"/> (of the manifold list) whatever its orientation, for
    /// ShapeFix_Wire's copy at the entry, which puts copies in place of a frozen wire's edges.
    /// </summary>
    internal void InternalSetEdge(in Edge edge, int num)
    {
        _edges.SetValue(num, edge);
        _seamF = -1;
    }

    /// <summary>Reverses the wire: the order of the edges and each edge's orientation.</summary>
    public void Reverse()
    {
        int i, nb = EdgeCount;

        // inverser les edges + les permuter pour inverser le wire
        for (i = 1; i <= nb / 2; i++)
        {
            Shape s1 = _edges.Value(i);
            s1.Reverse();
            Shape s2 = _edges.Value(nb + 1 - i);
            s2.Reverse();
            _edges.SetValue(i, s2);
            _edges.SetValue(nb + 1 - i, s1);
        }

        //  nb d edges impair : inverser aussi l edge du milieu (rang inchange)
        if (nb % 2 != 0)
        { //  test impair
            i = (nb + 1) / 2;
            Shape si = _edges.Value(i);
            si.Reverse();
            _edges.SetValue(i, si);
        }

        _seamF = -1;
    }

    // The file-static SwapSeam of ShapeExtend_WireData.cxx: swaps the two pcurves of a seam edge on the face, in place.
    private static void SwapSeam(in Shape s, in Face f)
    {
        Edge e = ShapeCasts.Edge(s);
        if (e.IsNull || f.IsNull)
        {
            return;
        }

        if (e.Orientation == Reversed)
        {
            return; // ne le faire qu une fois !
        }

        Face theface = f;
        theface.Orient(Forward);

        //: S4136  double Tol = BRep_Tool::Tolerance(theface);
        Curve2D? c2df, c2dr;
        double uff, ulf;

        // d abord FWD puis REV
        c2df = Tool.CurveOnSurface(e, theface, out uff, out ulf);
        e.Orient(Reversed);
        c2dr = Tool.CurveOnSurface(e, theface, out _, out _);
        if (c2df is null || c2dr is null)
        {
            return; //: q0
        }

        // On permute
        e.Orient(Forward);
        var b = new BrepBuilder();
        b.UpdateEdge(e, c2dr, c2df, theface, 0.0); //: S4136: Tol
        b.Range(e, theface, uff, ulf);
    }

    /// <summary>
    /// Reverses the wire, and on the face swaps the two pcurves of each seam edge, so that the seams keep their roles.
    /// </summary>
    /// <remarks>
    /// Internal: it writes into the seam edges, which throws on frozen ones (phase3-design 4.6). Without a face it is
    /// <see cref="Reverse()"/>.
    /// </remarks>
    internal void Reverse(in Face face)
    {
        Reverse();
        if (face.IsNull)
        {
            return;
        }

        //  ATTENTION aux coutures
        //  Une edge de couture est presente deux fois, FWD et REV
        //  Les inverser revient a permuter leur role ... donc ne rien faire
        //  Il faut donc aussi permuter leurs pcurves
        ComputeSeams(true);
        if (_seamF > 0)
        {
            SwapSeam(_edges.Value(_seamF), face);
        }

        if (_seamR > 0)
        {
            SwapSeam(_edges.Value(_seamR), face);
        }

        int nb = (_seams is null ? 0 : _seams.Length());
        for (int i = 1; i <= nb; i++)
        {
            SwapSeam(_edges.Value(_seams!.Value(i)), face);
        }

        _seamF = -1;
    }

    /// <summary>The count of edges (non-manifold edges apart).</summary>
    public int EdgeCount
    {
        get
        {
            return _edges.Length();
        }
    }

    /// <summary>The count of non-manifold edges kept apart.</summary>
    public int NonManifoldEdgeCount
    {
        get
        {
            return _nonmanifoldEdges.Length();
        }
    }

    /// <summary>The non-manifold edge <paramref name="num"/>; a null edge for a negative index.</summary>
    public Edge NonmanifoldEdge(int num)
    {
        Edge e = new Edge();
        if (num < 0)
        {
            return e;
        }

        return ShapeCasts.Edge(_nonmanifoldEdges.Value(num));
    }

    /// <summary>The non-manifold edges, as a new read-only list.</summary>
    /// <remarks>OCCT returns its own sequence; ported code reads it through <see cref="InternalNonmanifoldEdges"/>.</remarks>
    public IReadOnlyList<Shape> NonmanifoldEdges()
    {
        var array = new Shape[_nonmanifoldEdges.Length()];
        for (int i = 1; i <= array.Length; i++)
        {
            array[i - 1] = _nonmanifoldEdges.Value(i);
        }

        return ImmutableArrays.Own(array);
    }

    /// <summary>The wire data's own sequence of non-manifold edges, as OCCT's NonmanifoldEdges returns it.</summary>
    internal NCollection_HSequence<Shape> InternalNonmanifoldEdges()
    {
        return _nonmanifoldEdges;
    }

    /// <summary>Whether the INTERNAL and EXTERNAL edges are kept apart (true, the default) or with the others.</summary>
    public ref bool ManifoldMode()
    {
        return ref _manifoldMode;
    }

    /// <summary>Edge <paramref name="num"/>; for a negative index, edge -num reversed.</summary>
    public Edge Edge(int num)
    {
        if (num < 0)
        {
            Edge e = Edge(-num);
            e.Reverse();
            return e;
        }

        return ShapeCasts.Edge(_edges.Value(num));
    }

    /// <summary>
    /// The index of the edge (the same, with the same orientation if it is a seam), or 0 if it isn't in the wire.
    /// </summary>
    public int Index(in Edge edge)
    {
        for (int i = 1; i <= EdgeCount; i++)
        {
            if (Edge(i).IsSame(edge) && (Edge(i).Orientation == edge.Orientation || !IsSeam(i)))
            {
                return i;
            }
        }

        return 0;
    }

    /// <summary>Tells if edge <paramref name="num"/> is a seam (present twice, FORWARD and REVERSED).</summary>
    public bool IsSeam(int num)
    {
        if (_seamF < 0)
        {
            ComputeSeams();
        }

        if (_seamF == 0)
        {
            return false;
        }

        if (num == _seamF || num == _seamR)
        {
            return true;
        }

        // Use hash set for O(1) lookup instead of O(n) linear search
        return _seamsCache.Contains(num);
    }

    /// <summary>
    /// A new wire holding the edges in their order (and the non-manifold edges in manifold mode), closed if its ends
    /// are the same vertex. Frozen.
    /// </summary>
    public Wire Wire()
    {
        Wire w = InternalWire();
        w.TShape()!.FreezeGraph();
        return w;
    }

    /// <summary>As <see cref="Wire"/>, without freezing, for ported code that goes on building.</summary>
    internal Wire InternalWire()
    {
        Wire w = new Wire();
        var b = new BrepBuilder();
        b.MakeWire(ref w);
        int i, nb = EdgeCount;
        bool ismanifold = true;
        for (i = 1; i <= nb; i++)
        {
            Edge e = Edge(i);
            if (e.Orientation != Forward && e.Orientation != Reversed)
            {
                ismanifold = false;
            }

            b.Add(w, e);
        }

        if (ismanifold)
        {
            Exploration.Vertices(w, out Vertex vf, out Vertex vl);
            if (!vf.IsNull && !vl.IsNull && vf.IsSame(vl))
            {
                w.Closed(true);
            }
        }

        if (_manifoldMode)
        {
            nb = NonManifoldEdgeCount;
            for (i = 1; i <= nb; i++)
            {
                b.Add(w, NonmanifoldEdge(i));
            }
        }

        return w;
    }

    /// <summary>A new wire made by BRepBuilderAPI_MakeWire from the edges; a null wire if it fails. Frozen.</summary>
    public Wire WireAPIMake()
    {
        Wire w = InternalWireAPIMake();
        if (!w.IsNull)
        {
            w.TShape()!.FreezeGraph();
        }

        return w;
    }

    /// <summary>As <see cref="WireAPIMake"/>, without freezing, for ported code that goes on building.</summary>
    internal Wire InternalWireAPIMake()
    {
        Wire w = new Wire();
        var mw = new MakeWire();
        int i, nb = EdgeCount;
        for (i = 1; i <= nb; i++)
        {
            mw.Add(Edge(i));
        }

        if (_manifoldMode)
        {
            nb = NonManifoldEdgeCount;
            for (i = 1; i <= nb; i++)
            {
                mw.Add(NonmanifoldEdge(i));
            }
        }

        if (mw.IsDone)
        {
            w = mw.InternalWire();
        }

        return w;
    }
}
