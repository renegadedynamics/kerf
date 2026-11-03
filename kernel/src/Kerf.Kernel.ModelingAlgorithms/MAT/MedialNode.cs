// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_Node.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_Node.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;
using static Kerf.Engine.Offsets.MedialAxis.MedialSide;

namespace Kerf.Engine.Offsets.MedialAxis;

/// <summary>A node of the bisecting locus: the end of arcs, at a distance from the contour.</summary>
/// <remarks>OCCT keeps its linked arc as a raw pointer; the port keeps a reference. The setters are internal.</remarks>
public sealed class MedialNode : Transient
{
    private int _nodeIndex;
    private readonly int _geomIndex;
    private MedialArc? _aLinkedArc;
    private readonly double _distance;

    public MedialNode(int geomIndex, MedialArc? linkedArc, double distance)
    {
        _nodeIndex = 0;
        _geomIndex = geomIndex;
        _distance = distance;
        _aLinkedArc = linkedArc;
    }

    public int GeomIndex() => _geomIndex;

    public int Index() => _nodeIndex;

    /// <summary>Fills S with the arcs ending at the node, turning left.</summary>
    public void LinkedArcs(NCollection_Sequence<MedialArc> s)
    {
        s.Clear();
        MedialNode me = this;
        MedialArc la = _aLinkedArc!;

        s.Append(la);

        if (la.HasNeighbour(me, MedialSide.Left))
        {
            MedialArc ca = la.Neighbour(me, MedialSide.Left)!;
            while (ca != la)
            {
                s.Append(ca);
                ca = ca.Neighbour(me, MedialSide.Left)!;
            }
        }
    }

    /// <summary>Fills S with the basic elements nearest the node.</summary>
    public void NearElts(NCollection_Sequence<MedialBasicElt> s)
    {
        s.Clear();

        MedialNode me = this;
        MedialArc la = _aLinkedArc!;

        s.Append(la.FirstElement());
        s.Append(la.SecondElement());

        if (la.HasNeighbour(me, MedialSide.Left))
        {
            MedialArc ca = la.Neighbour(me, MedialSide.Left)!;
            bool pair = false;

            //---------------------------------------------------------
            // Each BasicElt is visited 2 times. It is necessary to
            // check only once.
            //---------------------------------------------------------
            while (ca != la)
            {
                if (pair)
                {
                    s.Append(ca.FirstElement());
                    s.Append(ca.SecondElement());
                }
                else
                {
                    pair = true;
                }
                ca = ca.Neighbour(me, MedialSide.Left)!;
            }
        }
    }

    public double Distance() => _distance;

    /// <summary>True if the node has only one linked arc.</summary>
    public bool PendingNode()
    {
        MedialNode me = this;
        return !_aLinkedArc!.HasNeighbour(me, MedialSide.Left);
    }

    /// <summary>True if the node is on a basic element (its distance is 0).</summary>
    public bool OnBasicElt() => Distance() == 0.0;

    /// <summary>True if the node is at infinity.</summary>
    public bool Infinite() => Distance() == global::Kerf.Engine.Foundation.Precision.Infinite();

    internal void SetIndex(int anIndex) => _nodeIndex = anIndex;

    internal void SetLinkedArc(MedialArc? linkedArc) => _aLinkedArc = linkedArc;
}
