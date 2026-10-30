// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_Node.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_Node.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.MAT_Side;

namespace Occt;

/// <summary>A node of the bisecting locus: the end of arcs, at a distance from the contour.</summary>
/// <remarks>OCCT keeps its linked arc as a raw pointer; the port keeps a reference. The setters are internal.</remarks>
public sealed class MAT_Node : Standard_Transient
{
    private int nodeIndex;
    private readonly int geomIndex;
    private MAT_Arc? aLinkedArc;
    private readonly double distance;

    public MAT_Node(int GeomIndex, MAT_Arc? LinkedArc, double Distance)
    {
        nodeIndex = 0;
        geomIndex = GeomIndex;
        distance = Distance;
        aLinkedArc = LinkedArc;
    }

    public int GeomIndex() => geomIndex;

    public int Index() => nodeIndex;

    /// <summary>Fills S with the arcs ending at the node, turning left.</summary>
    public void LinkedArcs(NCollection_Sequence<MAT_Arc> S)
    {
        S.Clear();
        MAT_Node Me = this;
        MAT_Arc LA = aLinkedArc!;

        S.Append(LA);

        if (LA.HasNeighbour(Me, MAT_Left))
        {
            MAT_Arc CA = LA.Neighbour(Me, MAT_Left)!;
            while (CA != LA)
            {
                S.Append(CA);
                CA = CA.Neighbour(Me, MAT_Left)!;
            }
        }
    }

    /// <summary>Fills S with the basic elements nearest the node.</summary>
    public void NearElts(NCollection_Sequence<MAT_BasicElt> S)
    {
        S.Clear();

        MAT_Node Me = this;
        MAT_Arc LA = aLinkedArc!;

        S.Append(LA.FirstElement());
        S.Append(LA.SecondElement());

        if (LA.HasNeighbour(Me, MAT_Left))
        {
            MAT_Arc CA = LA.Neighbour(Me, MAT_Left)!;
            bool Pair = false;

            //---------------------------------------------------------
            // Each BasicElt is visited 2 times. It is necessary to
            // check only once.
            //---------------------------------------------------------
            while (CA != LA)
            {
                if (Pair)
                {
                    S.Append(CA.FirstElement());
                    S.Append(CA.SecondElement());
                }
                else
                {
                    Pair = true;
                }
                CA = CA.Neighbour(Me, MAT_Left)!;
            }
        }
    }

    public double Distance() => distance;

    /// <summary>True if the node has only one linked arc.</summary>
    public bool PendingNode()
    {
        MAT_Node Me = this;
        return !aLinkedArc!.HasNeighbour(Me, MAT_Left);
    }

    /// <summary>True if the node is on a basic element (its distance is 0).</summary>
    public bool OnBasicElt() => Distance() == 0.0;

    /// <summary>True if the node is at infinity.</summary>
    public bool Infinite() => Distance() == Precision.Infinite();

    internal void SetIndex(int anIndex) => nodeIndex = anIndex;

    internal void SetLinkedArc(MAT_Arc? LinkedArc) => aLinkedArc = LinkedArc;
}
