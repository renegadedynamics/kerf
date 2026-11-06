// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_Arc.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_Arc.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Kerf.Engine.Offsets.MedialAxis;

/// <summary>An arc of the bisecting locus: the part of the bisector of two basic elements between two nodes, with
/// its neighbours on each side at each node.</summary>
/// <remarks>OCCT keeps the neighbouring arcs as raw pointers; the port keeps references. The setters are internal.
/// </remarks>
internal sealed class MedialArc : Transient
{
    private int _arcIndex;
    private int _geomIndex;
    private MedialBasicElt _firstElement;
    private MedialBasicElt _secondElement;
    private MedialNode? _firstNode;
    private MedialNode? _secondNode;
    private MedialArc? _firstArcLeft;
    private MedialArc? _firstArcRight;
    private MedialArc? _secondArcRight;
    private MedialArc? _secondArcLeft;

    public MedialArc(int arcIndex, int geomIndex, MedialBasicElt firstElement, MedialBasicElt secondElement)
    {
        _arcIndex = arcIndex;
        _geomIndex = geomIndex;
        _firstArcLeft = null;
        _firstArcRight = null;
        _secondArcRight = null;
        _secondArcLeft = null;
        _firstElement = firstElement;
        _secondElement = secondElement;
    }

    /// <summary>The index of the arc in the graph.</summary>
    public int Index() => _arcIndex;

    /// <summary>The index of the arc's geometry (its bisector) in the tool.</summary>
    public int GeomIndex() => _geomIndex;

    /// <summary>One of the two basic elements equidistant from the arc.</summary>
    public MedialBasicElt FirstElement() => _firstElement;

    /// <summary>The other basic element equidistant from the arc.</summary>
    public MedialBasicElt SecondElement() => _secondElement;

    public MedialNode FirstNode() => _firstNode!;

    public MedialNode SecondNode() => _secondNode!;

    /// <summary>The node of the arc other than aNode; throws Standard_DomainError if aNode isn't one of its nodes.</summary>
    public MedialNode TheOtherNode(MedialNode aNode)
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
    public bool HasNeighbour(MedialNode aNode, MedialSide aSide)
    {
        if (aSide == MedialSide.Left)
        {
            if (aNode == FirstNode())
            {
                return _firstArcLeft is not null;
            }
            if (aNode == SecondNode())
            {
                return _secondArcLeft is not null;
            }
        }
        else
        {
            if (aNode == FirstNode())
            {
                return _firstArcRight is not null;
            }
            if (aNode == SecondNode())
            {
                return _secondArcRight is not null;
            }
        }
        throw new DomainErrorException("MAT_Arc::HasNeighbour");
    }

    /// <summary>The neighbour of the arc on aSide at aNode (null if none).</summary>
    public MedialArc? Neighbour(MedialNode aNode, MedialSide aSide)
    {
        if (aSide == MedialSide.Left)
        {
            if (aNode == FirstNode())
            {
                return _firstArcLeft;
            }
            if (aNode == SecondNode())
            {
                return _secondArcLeft;
            }
        }
        else
        {
            if (aNode == FirstNode())
            {
                return _firstArcRight;
            }
            if (aNode == SecondNode())
            {
                return _secondArcRight;
            }
        }
        throw new DomainErrorException("MAT_Arc::Neighbour");
    }

    internal void SetIndex(int anInteger) => _arcIndex = anInteger;

    internal void SetGeomIndex(int anInteger) => _geomIndex = anInteger;

    internal void SetFirstElement(MedialBasicElt aBasicElt) => _firstElement = aBasicElt;

    internal void SetSecondElement(MedialBasicElt aBasicElt) => _secondElement = aBasicElt;

    internal void SetFirstNode(MedialNode? aNode) => _firstNode = aNode;

    internal void SetSecondNode(MedialNode? aNode) => _secondNode = aNode;

    internal void SetFirstArc(MedialSide aSide, MedialArc? anArc)
    {
        if (aSide == MedialSide.Left)
        {
            _firstArcLeft = anArc;
        }
        else
        {
            _firstArcRight = anArc;
        }
    }

    internal void SetSecondArc(MedialSide aSide, MedialArc? anArc)
    {
        if (aSide == MedialSide.Left)
        {
            _secondArcLeft = anArc;
        }
        else
        {
            _secondArcRight = anArc;
        }
    }

    internal void SetNeighbour(MedialSide aSide, MedialNode aNode, MedialArc? anArc)
    {
        if (aSide == MedialSide.Left)
        {
            if (aNode == FirstNode())
            {
                _firstArcLeft = anArc;
            }
            else if (aNode == SecondNode())
            {
                _secondArcLeft = anArc;
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
                _firstArcRight = anArc;
            }
            else if (aNode == SecondNode())
            {
                _secondArcRight = anArc;
            }
            else
            {
                throw new DomainErrorException("MAT_Arc::SetNeighbour");
            }
        }
    }
}
