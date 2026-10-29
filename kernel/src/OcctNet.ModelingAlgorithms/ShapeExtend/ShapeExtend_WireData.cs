// Port of: src/ModelingAlgorithms/TKShHealing/ShapeExtend/ShapeExtend_WireData.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeExtend/ShapeExtend_WireData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// A wire as an ordered list of edges, with direct access by index (from 1; a negative index gives the edge
/// reversed), its seams computed on demand, and the edges that aren't FORWARD or REVERSED (INTERNAL, EXTERNAL) kept
/// apart in manifold mode.
/// </summary>
/// <remarks>
/// A working object (decision 21) with OCCT's public mutators: it holds the caller's edges and changes only its own
/// list. <see cref="Wire"/> and <see cref="WireAPIMake"/> freeze the wire they return; ported code that goes on
/// building uses <see cref="InternalWire"/> and <see cref="InternalWireAPIMake"/>. <see cref="Reverse(in TopoDS_Face)"/>
/// swaps the pcurves of seam edges in place, so it is internal.
/// </remarks>
public class ShapeExtend_WireData : Standard_Transient
{
    private NCollection_HSequence<TopoDS_Shape> myEdges = null!;
    private NCollection_HSequence<TopoDS_Shape> myNonmanifoldEdges = null!;
    private NCollection_HSequence<int>? mySeams;
    private readonly NCollection_PackedMapOfInteger mySeamsCache = new NCollection_PackedMapOfInteger();
    private int mySeamF;
    private int mySeamR;
    private bool myManifoldMode;

    /// <summary>An empty wire.</summary>
    public ShapeExtend_WireData()
    {
        Clear();
    }

    /// <summary>
    /// The edges of the wire, as <see cref="Init(in TopoDS_Wire, bool, bool)"/> loads them.
    /// </summary>
    public ShapeExtend_WireData(in TopoDS_Wire wire, bool chained = true, bool theManifoldMode = true)
    {
        Init(wire, chained, theManifoldMode);
    }

    /// <summary>Copies the edges (and non-manifold edges and mode) of another wire data.</summary>
    public void Init(ShapeExtend_WireData other)
    {
        Clear();
        int i, nb = other.NbEdges();
        for (i = 1; i <= nb; i++)
        {
            Add(other.Edge(i));
        }

        nb = other.NbNonManifoldEdges();
        for (i = 1; i <= nb; i++)
        {
            Add(other.NonmanifoldEdge(i));
        }

        myManifoldMode = other.ManifoldMode();
    }

    /// <summary>
    /// Loads the edges of the wire in their stored order (reversed for a reversed wire). Returns false if they don't
    /// chain; then, if not <paramref name="chained"/>, they are loaded again in the order of BRepTools_WireExplorer.
    /// In manifold mode the INTERNAL and EXTERNAL edges are kept apart.
    /// </summary>
    public bool Init(in TopoDS_Wire wire, bool chained = true, bool theManifoldMode = true)
    {
        Clear();
        myManifoldMode = theManifoldMode;
        bool OK = true;
        TopoDS_Vertex Vlast = new TopoDS_Vertex();
        for (var it = new TopoDS_Iterator(wire); it.More(); it.Next())
        {
            TopoDS_Edge E = TopoDS.Edge(it.Value());

            // protect against INTERNAL/EXTERNAL edges
            if ((E.Orientation() != TopAbs_REVERSED && E.Orientation() != TopAbs_FORWARD))
            {
                myNonmanifoldEdges.Append(E);
                continue;
            }

            TopoDS_Vertex V1 = new TopoDS_Vertex(), V2 = new TopoDS_Vertex();
            for (var itv = new TopoDS_Iterator(E); itv.More(); itv.Next())
            {
                TopoDS_Vertex V = TopoDS.Vertex(itv.Value());
                if (V.Orientation() == TopAbs_FORWARD)
                {
                    V1 = V;
                }
                else if (V.Orientation() == TopAbs_REVERSED)
                {
                    V2 = V;
                }
            }

            // chainage? Si pas bon et chained False on repart sur WireExplorer
            if (!Vlast.IsNull() && !Vlast.IsSame(V1) && theManifoldMode)
            {
                OK = false;
                if (!chained)
                {
                    break;
                }
            }

            Vlast = V2;
            if (wire.Orientation() == TopAbs_REVERSED)
            {
                myEdges.Prepend(E);
            }
            else
            {
                myEdges.Append(E);
            }
        }

        if (!myManifoldMode)
        {
            int nb = myNonmanifoldEdges.Length();
            int i = 1;
            for (; i <= nb; i++)
            {
                myEdges.Append(myNonmanifoldEdges.Value(i));
            }

            myNonmanifoldEdges.Clear();
        }

        //    refaire chainage ?  Par WireExplorer
        if (OK || chained)
        {
            return OK;
        }

        Clear();
        for (var we = new BRepTools_WireExplorer(wire); we.More(); we.Next())
        {
            myEdges.Append(TopoDS.Edge(we.Current()));
        }

        return OK;
    }

    /// <summary>Clears the wire (and sets manifold mode).</summary>
    public void Clear()
    {
        myEdges = new NCollection_HSequence<TopoDS_Shape>();
        myNonmanifoldEdges = new NCollection_HSequence<TopoDS_Shape>();
        mySeamF = mySeamR = -1;
        mySeams = null;
        myManifoldMode = true;
    }

    /// <summary>
    /// Computes the seams: edges present twice, FORWARD and REVERSED. Done once, unless <paramref name="enforce"/>.
    /// </summary>
    public void ComputeSeams(bool enforce = true)
    {
        if (mySeamF >= 0 && !enforce)
        {
            return;
        }

        mySeams = new NCollection_HSequence<int>();
        mySeamF = mySeamR = 0;
        TopoDS_Shape S;
        int i, nb = NbEdges();
        var ME = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        int[] SE = new int[nb + 1];

        //  deux passes : d abord on mappe les Edges REVERSED
        //  Pour chacune, on note aussi son RANG dans la liste
        for (i = 1; i <= nb; i++)
        {
            S = Edge(i);
            if (S.Orientation() == TopAbs_REVERSED)
            {
                int num = ME.Add(S);
                SE[num] = i;
            }
        }

        //  ensuite on voit les Edges FORWARD qui y seraient deja -> on note leur n0
        //  c-a-d le n0 de la directe ET de la reverse
        for (i = 1; i <= nb; i++)
        {
            S = Edge(i);
            if (S.Orientation() == TopAbs_REVERSED)
            {
                continue;
            }

            int num = ME.FindIndex(S);
            if (num <= 0)
            {
                continue;
            }

            if (mySeamF == 0)
            {
                mySeamF = i;
                mySeamR = SE[num];
            }
            else
            {
                mySeams.Append(i);
                mySeams.Append(SE[num]);
            }
        }

        mySeamsCache.Clear();
        for (i = 1; i <= mySeams.Length(); i++)
        {
            mySeamsCache.Add(mySeams.Value(i));
        }
    }

    /// <summary>Makes edge <paramref name="num"/> the last one, by a circular shift (nothing for 0).</summary>
    public void SetLast(int num)
    {
        if (num == 0)
        {
            return;
        }

        int i, nb = NbEdges();
        for (i = nb; i > num; i--)
        {
            TopoDS_Edge edge = TopoDS.Edge(myEdges.Value(nb));
            myEdges.Remove(nb);
            myEdges.InsertBefore(1, edge);
        }

        mySeamF = -1;
    }

    /// <summary>Makes the first degenerated edge the last one (see <see cref="SetLast"/>).</summary>
    public void SetDegeneratedLast()
    {
        int i, nb = NbEdges();
        for (i = 1; i <= nb; i++)
        {
            if (BRep_Tool.Degenerated(Edge(i)))
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
    public void Add(in TopoDS_Edge edge, int atnum = 0)
    {
        if (edge.Orientation() != TopAbs_REVERSED && edge.Orientation() != TopAbs_FORWARD && myManifoldMode)
        {
            myNonmanifoldEdges.Append(edge);
            return;
        }

        if (edge.IsNull())
        {
            return;
        }

        if (atnum == 0)
        {
            myEdges.Append(edge);
        }
        else
        {
            myEdges.InsertBefore(atnum, edge);
        }

        mySeamF = -1;
    }

    /// <summary>Adds the edges of a wire, in their stored order, as <see cref="Add(in TopoDS_Edge, int)"/>.</summary>
    public void Add(in TopoDS_Wire wire, int atnum = 0)
    {
        if (wire.IsNull())
        {
            return;
        }

        int n = atnum;
        var aNMEdges = new NCollection_Sequence<TopoDS_Shape>();
        for (var it = new TopoDS_Iterator(wire); it.More(); it.Next())
        {
            TopoDS_Edge edge = TopoDS.Edge(it.Value());
            if (edge.Orientation() != TopAbs_REVERSED && edge.Orientation() != TopAbs_FORWARD)
            {
                if (myManifoldMode)
                {
                    myNonmanifoldEdges.Append(edge);
                }
                else
                {
                    aNMEdges.Append(edge);
                }

                continue;
            }

            if (n == 0)
            {
                myEdges.Append(edge);
            }
            else
            {
                myEdges.InsertBefore(n, edge);
                n++;
            }
        }

        int i = 1, nb = aNMEdges.Length();
        for (; i <= nb; i++)
        {
            myEdges.Append(aNMEdges.Value(i));
        }

        mySeamF = -1;
    }

    /// <summary>Adds the edges of another wire data, as <see cref="Add(in TopoDS_Edge, int)"/>.</summary>
    public void Add(ShapeExtend_WireData? wire, int atnum = 0)
    {
        if (wire is null)
        {
            return;
        }

        var aNMEdges = new NCollection_Sequence<TopoDS_Shape>();
        int n = atnum;
        int i = 1;
        for (; i <= wire.NbEdges(); i++)
        {
            TopoDS_Edge aE = wire.Edge(i);
            if (aE.Orientation() == TopAbs_INTERNAL || aE.Orientation() == TopAbs_EXTERNAL)
            {
                aNMEdges.Append(aE);
                continue;
            }

            if (n == 0)
            {
                myEdges.Append(wire.Edge(i));
            }
            else
            {
                myEdges.InsertBefore(n, wire.Edge(i));
                n++;
            }
        }

        // non-manifold edges for non-manifold wire should be added at end
        for (i = 1; i <= aNMEdges.Length(); i++)
        {
            myEdges.Append(aNMEdges.Value(i));
        }

        for (i = 1; i <= wire.NbNonManifoldEdges(); i++)
        {
            if (myManifoldMode)
            {
                myNonmanifoldEdges.Append(wire.NonmanifoldEdge(i));
            }
            else
            {
                // OCCT adds the i-th ordinary edge of the other wire here, not its i-th non-manifold edge
                if (n == 0)
                {
                    myEdges.Append(wire.Edge(i));
                }
                else
                {
                    myEdges.InsertBefore(n, wire.Edge(i));
                    n++;
                }
            }
        }

        mySeamF = -1;
    }

    /// <summary>Adds an edge or the edges of a wire; other shapes are ignored.</summary>
    public void Add(in TopoDS_Shape shape, int atnum = 0)
    {
        if (shape.ShapeType() == TopAbs_EDGE)
        {
            Add(TopoDS.Edge(shape), atnum);
        }
        else if (shape.ShapeType() == TopAbs_WIRE)
        {
            Add(TopoDS.Wire(shape), atnum);
        }
    }

    /// <summary>
    /// Adds an edge: <paramref name="mode"/> 0 at the end, 1 reversed at the end, 2 at the start, 3 reversed at the
    /// start; nothing if negative.
    /// </summary>
    public void AddOriented(in TopoDS_Edge edge, int mode)
    {
        if (edge.IsNull() || mode < 0)
        {
            return;
        }

        TopoDS_Edge E = edge;
        if (mode == 1 || mode == 3)
        {
            E.Reverse();
        }

        Add(E, mode / 2); // mode = 0,1 -> 0  mode = 2,3 -> 1
    }

    /// <summary>Adds the edges of a wire, as <see cref="AddOriented(in TopoDS_Edge, int)"/>.</summary>
    public void AddOriented(in TopoDS_Wire wire, int mode)
    {
        if (wire.IsNull() || mode < 0)
        {
            return;
        }

        TopoDS_Wire W = wire;
        if (mode == 1 || mode == 3)
        {
            W.Reverse();
        }

        Add(W, mode / 2); // mode = 0,1 -> 0  mode = 2,3 -> 1
    }

    /// <summary>Adds an edge or the edges of a wire, as <see cref="AddOriented(in TopoDS_Edge, int)"/>.</summary>
    public void AddOriented(in TopoDS_Shape shape, int mode)
    {
        if (shape.ShapeType() == TopAbs_EDGE)
        {
            AddOriented(TopoDS.Edge(shape), mode);
        }
        else if (shape.ShapeType() == TopAbs_WIRE)
        {
            AddOriented(TopoDS.Wire(shape), mode);
        }
    }

    /// <summary>Removes edge <paramref name="num"/> (the last one for 0).</summary>
    public void Remove(int num = 0)
    {
        myEdges.Remove(num > 0 ? num : NbEdges());

        mySeamF = -1;
    }

    /// <summary>
    /// Replaces edge <paramref name="num"/> (the last one for 0). In manifold mode an INTERNAL or EXTERNAL edge
    /// replaces the non-manifold edge <paramref name="num"/>, or is appended to them.
    /// </summary>
    public void Set(in TopoDS_Edge edge, int num = 0)
    {
        if (edge.Orientation() != TopAbs_REVERSED && edge.Orientation() != TopAbs_FORWARD && myManifoldMode)
        {
            if (num <= myNonmanifoldEdges.Length())
            {
                myNonmanifoldEdges.SetValue(num, edge);
            }
            else
            {
                myNonmanifoldEdges.Append(edge);
            }
        }
        else
        {
            myEdges.SetValue((num > 0 ? num : NbEdges()), edge);
        }

        mySeamF = -1;
    }

    /// <summary>Reverses the wire: the order of the edges and each edge's orientation.</summary>
    public void Reverse()
    {
        int i, nb = NbEdges();

        // inverser les edges + les permuter pour inverser le wire
        for (i = 1; i <= nb / 2; i++)
        {
            TopoDS_Shape S1 = myEdges.Value(i);
            S1.Reverse();
            TopoDS_Shape S2 = myEdges.Value(nb + 1 - i);
            S2.Reverse();
            myEdges.SetValue(i, S2);
            myEdges.SetValue(nb + 1 - i, S1);
        }

        //  nb d edges impair : inverser aussi l edge du milieu (rang inchange)
        if (nb % 2 != 0)
        { //  test impair
            i = (nb + 1) / 2;
            TopoDS_Shape SI = myEdges.Value(i);
            SI.Reverse();
            myEdges.SetValue(i, SI);
        }

        mySeamF = -1;
    }

    // The file-static SwapSeam of ShapeExtend_WireData.cxx: swaps the two pcurves of a seam edge on the face, in place.
    private static void SwapSeam(in TopoDS_Shape S, in TopoDS_Face F)
    {
        TopoDS_Edge E = TopoDS.Edge(S);
        if (E.IsNull() || F.IsNull())
        {
            return;
        }

        if (E.Orientation() == TopAbs_REVERSED)
        {
            return; // ne le faire qu une fois !
        }

        TopoDS_Face theface = F;
        theface.Orientation(TopAbs_FORWARD);

        //: S4136  double Tol = BRep_Tool::Tolerance(theface);
        Geom2d_Curve? c2df, c2dr;
        double uff, ulf;

        // d abord FWD puis REV
        c2df = BRep_Tool.CurveOnSurface(E, theface, out uff, out ulf);
        E.Orientation(TopAbs_REVERSED);
        c2dr = BRep_Tool.CurveOnSurface(E, theface, out _, out _);
        if (c2df is null || c2dr is null)
        {
            return; //: q0
        }

        // On permute
        E.Orientation(TopAbs_FORWARD);
        var B = new BRep_Builder();
        B.UpdateEdge(E, c2dr, c2df, theface, 0.0); //: S4136: Tol
        B.Range(E, theface, uff, ulf);
    }

    /// <summary>
    /// Reverses the wire, and on the face swaps the two pcurves of each seam edge, so that the seams keep their roles.
    /// </summary>
    /// <remarks>
    /// Internal: it writes into the seam edges, which throws on frozen ones (phase3-design 4.6). Without a face it is
    /// <see cref="Reverse()"/>.
    /// </remarks>
    internal void Reverse(in TopoDS_Face face)
    {
        Reverse();
        if (face.IsNull())
        {
            return;
        }

        //  ATTENTION aux coutures
        //  Une edge de couture est presente deux fois, FWD et REV
        //  Les inverser revient a permuter leur role ... donc ne rien faire
        //  Il faut donc aussi permuter leurs pcurves
        ComputeSeams(true);
        if (mySeamF > 0)
        {
            SwapSeam(myEdges.Value(mySeamF), face);
        }

        if (mySeamR > 0)
        {
            SwapSeam(myEdges.Value(mySeamR), face);
        }

        int nb = (mySeams is null ? 0 : mySeams.Length());
        for (int i = 1; i <= nb; i++)
        {
            SwapSeam(myEdges.Value(mySeams!.Value(i)), face);
        }

        mySeamF = -1;
    }

    /// <summary>The count of edges (non-manifold edges apart).</summary>
    public int NbEdges()
    {
        return myEdges.Length();
    }

    /// <summary>The count of non-manifold edges kept apart.</summary>
    public int NbNonManifoldEdges()
    {
        return myNonmanifoldEdges.Length();
    }

    /// <summary>The non-manifold edge <paramref name="num"/>; a null edge for a negative index.</summary>
    public TopoDS_Edge NonmanifoldEdge(int num)
    {
        TopoDS_Edge E = new TopoDS_Edge();
        if (num < 0)
        {
            return E;
        }

        return TopoDS.Edge(myNonmanifoldEdges.Value(num));
    }

    /// <summary>The non-manifold edges, as a new read-only list.</summary>
    /// <remarks>OCCT returns its own sequence; ported code reads it through <see cref="InternalNonmanifoldEdges"/>.</remarks>
    public Bento.Collections.IBentoList<TopoDS_Shape> NonmanifoldEdges()
    {
        var anArray = new TopoDS_Shape[myNonmanifoldEdges.Length()];
        for (int i = 1; i <= anArray.Length; i++)
        {
            anArray[i - 1] = myNonmanifoldEdges.Value(i);
        }

        return PackedList.Own(anArray);
    }

    /// <summary>The wire data's own sequence of non-manifold edges, as OCCT's NonmanifoldEdges returns it.</summary>
    internal NCollection_HSequence<TopoDS_Shape> InternalNonmanifoldEdges()
    {
        return myNonmanifoldEdges;
    }

    /// <summary>Whether the INTERNAL and EXTERNAL edges are kept apart (true, the default) or with the others.</summary>
    public ref bool ManifoldMode()
    {
        return ref myManifoldMode;
    }

    /// <summary>Edge <paramref name="num"/>; for a negative index, edge -num reversed.</summary>
    public TopoDS_Edge Edge(int num)
    {
        if (num < 0)
        {
            TopoDS_Edge E = Edge(-num);
            E.Reverse();
            return E;
        }

        return TopoDS.Edge(myEdges.Value(num));
    }

    /// <summary>
    /// The index of the edge (the same, with the same orientation if it is a seam), or 0 if it isn't in the wire.
    /// </summary>
    public int Index(in TopoDS_Edge edge)
    {
        for (int i = 1; i <= NbEdges(); i++)
        {
            if (Edge(i).IsSame(edge) && (Edge(i).Orientation() == edge.Orientation() || !IsSeam(i)))
            {
                return i;
            }
        }

        return 0;
    }

    /// <summary>Tells if edge <paramref name="num"/> is a seam (present twice, FORWARD and REVERSED).</summary>
    public bool IsSeam(int num)
    {
        if (mySeamF < 0)
        {
            ComputeSeams();
        }

        if (mySeamF == 0)
        {
            return false;
        }

        if (num == mySeamF || num == mySeamR)
        {
            return true;
        }

        // Use hash set for O(1) lookup instead of O(n) linear search
        return mySeamsCache.Contains(num);
    }

    /// <summary>
    /// A new wire holding the edges in their order (and the non-manifold edges in manifold mode), closed if its ends
    /// are the same vertex. Frozen.
    /// </summary>
    public TopoDS_Wire Wire()
    {
        TopoDS_Wire W = InternalWire();
        W.TShape()!.FreezeGraph();
        return W;
    }

    /// <summary>As <see cref="Wire"/>, without freezing, for ported code that goes on building.</summary>
    internal TopoDS_Wire InternalWire()
    {
        TopoDS_Wire W = new TopoDS_Wire();
        var B = new BRep_Builder();
        B.MakeWire(ref W);
        int i, nb = NbEdges();
        bool ismanifold = true;
        for (i = 1; i <= nb; i++)
        {
            TopoDS_Edge aE = Edge(i);
            if (aE.Orientation() != TopAbs_FORWARD && aE.Orientation() != TopAbs_REVERSED)
            {
                ismanifold = false;
            }

            B.Add(W, aE);
        }

        if (ismanifold)
        {
            TopExp.Vertices(W, out TopoDS_Vertex vf, out TopoDS_Vertex vl);
            if (!vf.IsNull() && !vl.IsNull() && vf.IsSame(vl))
            {
                W.Closed(true);
            }
        }

        if (myManifoldMode)
        {
            nb = NbNonManifoldEdges();
            for (i = 1; i <= nb; i++)
            {
                B.Add(W, NonmanifoldEdge(i));
            }
        }

        return W;
    }

    /// <summary>A new wire made by BRepBuilderAPI_MakeWire from the edges; a null wire if it fails. Frozen.</summary>
    public TopoDS_Wire WireAPIMake()
    {
        TopoDS_Wire W = InternalWireAPIMake();
        if (!W.IsNull())
        {
            W.TShape()!.FreezeGraph();
        }

        return W;
    }

    /// <summary>As <see cref="WireAPIMake"/>, without freezing, for ported code that goes on building.</summary>
    internal TopoDS_Wire InternalWireAPIMake()
    {
        TopoDS_Wire W = new TopoDS_Wire();
        var MW = new BRepBuilderAPI_MakeWire();
        int i, nb = NbEdges();
        for (i = 1; i <= nb; i++)
        {
            MW.Add(Edge(i));
        }

        if (myManifoldMode)
        {
            nb = NbNonManifoldEdges();
            for (i = 1; i <= nb; i++)
            {
                MW.Add(NonmanifoldEdge(i));
            }
        }

        if (MW.IsDone())
        {
            W = MW.InternalWire();
        }

        return W;
    }
}
