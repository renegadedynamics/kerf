// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_Arc.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_Arc.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using static Occt.MAT_Side;

namespace Occt;

/// <summary>An arc of the bisecting locus: the part of the bisector of two basic elements between two nodes, with
/// its neighbours on each side at each node.</summary>
/// <remarks>OCCT keeps the neighbouring arcs as raw pointers; the port keeps references. The setters are internal.
/// </remarks>
public sealed class MAT_Arc : Transient
{
    private int arcIndex;
    private int geomIndex;
    private MAT_BasicElt firstElement;
    private MAT_BasicElt secondElement;
    private MAT_Node? firstNode;
    private MAT_Node? secondNode;
    private MAT_Arc? firstArcLeft;
    private MAT_Arc? firstArcRight;
    private MAT_Arc? secondArcRight;
    private MAT_Arc? secondArcLeft;

    public MAT_Arc(int ArcIndex, int GeomIndex, MAT_BasicElt FirstElement, MAT_BasicElt SecondElement)
    {
        arcIndex = ArcIndex;
        geomIndex = GeomIndex;
        firstArcLeft = null;
        firstArcRight = null;
        secondArcRight = null;
        secondArcLeft = null;
        firstElement = FirstElement;
        secondElement = SecondElement;
    }

    /// <summary>The index of the arc in the graph.</summary>
    public int Index() => arcIndex;

    /// <summary>The index of the arc's geometry (its bisector) in the tool.</summary>
    public int GeomIndex() => geomIndex;

    /// <summary>One of the two basic elements equidistant from the arc.</summary>
    public MAT_BasicElt FirstElement() => firstElement;

    /// <summary>The other basic element equidistant from the arc.</summary>
    public MAT_BasicElt SecondElement() => secondElement;

    public MAT_Node FirstNode() => firstNode!;

    public MAT_Node SecondNode() => secondNode!;

    /// <summary>The node of the arc other than aNode; throws Standard_DomainError if aNode isn't one of its nodes.</summary>
    public MAT_Node TheOtherNode(MAT_Node aNode)
    {
        if (FirstNode() == aNode)
        {
            return SecondNode();
        }
        else if (SecondNode() == aNode)
        {
            return FirstNode();
        }
        else
        {
            throw new DomainErrorException("MAT_Arc::TheOtherNode");
        }
    }

    /// <summary>True if the arc has a neighbour on aSide at aNode.</summary>
    public bool HasNeighbour(MAT_Node aNode, MAT_Side aSide)
    {
        if (aSide == MAT_Left)
        {
            if (aNode == FirstNode())
            {
                return firstArcLeft is not null;
            }
            if (aNode == SecondNode())
            {
                return secondArcLeft is not null;
            }
        }
        else
        {
            if (aNode == FirstNode())
            {
                return firstArcRight is not null;
            }
            if (aNode == SecondNode())
            {
                return secondArcRight is not null;
            }
        }
        throw new DomainErrorException("MAT_Arc::HasNeighbour");
    }

    /// <summary>The neighbour of the arc on aSide at aNode (null if none).</summary>
    public MAT_Arc? Neighbour(MAT_Node aNode, MAT_Side aSide)
    {
        if (aSide == MAT_Left)
        {
            if (aNode == FirstNode())
            {
                return firstArcLeft;
            }
            if (aNode == SecondNode())
            {
                return secondArcLeft;
            }
        }
        else
        {
            if (aNode == FirstNode())
            {
                return firstArcRight;
            }
            if (aNode == SecondNode())
            {
                return secondArcRight;
            }
        }
        throw new DomainErrorException("MAT_Arc::Neighbour");
    }

    internal void SetIndex(int anInteger) => arcIndex = anInteger;

    internal void SetGeomIndex(int anInteger) => geomIndex = anInteger;

    internal void SetFirstElement(MAT_BasicElt aBasicElt) => firstElement = aBasicElt;

    internal void SetSecondElement(MAT_BasicElt aBasicElt) => secondElement = aBasicElt;

    internal void SetFirstNode(MAT_Node? aNode) => firstNode = aNode;

    internal void SetSecondNode(MAT_Node? aNode) => secondNode = aNode;

    internal void SetFirstArc(MAT_Side aSide, MAT_Arc? anArc)
    {
        if (aSide == MAT_Left)
        {
            firstArcLeft = anArc;
        }
        else
        {
            firstArcRight = anArc;
        }
    }

    internal void SetSecondArc(MAT_Side aSide, MAT_Arc? anArc)
    {
        if (aSide == MAT_Left)
        {
            secondArcLeft = anArc;
        }
        else
        {
            secondArcRight = anArc;
        }
    }

    internal void SetNeighbour(MAT_Side aSide, MAT_Node aNode, MAT_Arc? anArc)
    {
        if (aSide == MAT_Left)
        {
            if (aNode == FirstNode())
            {
                firstArcLeft = anArc;
            }
            else if (aNode == SecondNode())
            {
                secondArcLeft = anArc;
            }
            else
            {
                throw new DomainErrorException("MAT_Arc::SetNeighbour");
            }
        }
        else
        {
            if (aNode == FirstNode())
            {
                firstArcRight = anArc;
            }
            else if (aNode == SecondNode())
            {
                secondArcRight = anArc;
            }
            else
            {
                throw new DomainErrorException("MAT_Arc::SetNeighbour");
            }
        }
    }
}
