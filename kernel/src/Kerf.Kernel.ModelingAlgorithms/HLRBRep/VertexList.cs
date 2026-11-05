// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_VertexList.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_VertexList.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.HiddenLines.Algorithm;
using Kerf.Topology;
using Kerf.Engine;

namespace Kerf.Engine.HiddenLines;

/// <summary>
/// Iterates, by parameter, on the vertices of an edge and its interferences merged: each step is a boundary vertex,
/// an interference, or both.
/// </summary>
/// <remarks>
/// A working object, as OCCT's. It copies the tool and the iterator it is given, as OCCT does, so the caller's are
/// not moved.
/// </remarks>
internal sealed class VertexList
{
    private readonly NCollection_List<Interference>.Iterator _iterator;
    private readonly EdgeInterferenceTool _tool;
    private bool _fromEdge;
    private bool _fromInterf;

    public VertexList(EdgeInterferenceTool t, NCollection_List<Interference>.Iterator i)
    {
        _iterator = new NCollection_List<Interference>.Iterator(i);
        _tool = new EdgeInterferenceTool(t);
        _fromEdge = false;
        _fromInterf = false;
        _tool.InitVertices();
        Next();
    }

    public bool IsPeriodic => _tool.IsPeriodic;

    public bool More() => (_fromEdge || _fromInterf);

    public void Next()
    {
        if (_fromInterf)
        {
            _iterator.Next();
        }
        if (_fromEdge)
        {
            _tool.NextVertex();
        }
        _fromInterf = _iterator.More();
        _fromEdge = _tool.MoreVertices();
        if (_fromEdge && _fromInterf)
        {
            if (!_tool.SameVertexAndInterference(_iterator.Value()))
            {
                if (_tool.CurrentParameter() < _tool.ParameterOfInterference(_iterator.Value()))
                {
                    _fromInterf = false;
                }
                else
                {
                    _fromEdge = false;
                }
            }
        }
    }

    /// <summary>The current vertex: the boundary vertex if there is one, else the interference's intersection.</summary>
    /// <exception cref="NoSuchObjectException">The iteration is over.</exception>
    public AlgoIntersection Current()
    {
        if (_fromEdge)
        {
            return _tool.CurrentVertex();
        }
        else if (_fromInterf)
        {
            return _iterator.Value().Intersection();
        }
        else
        {
            throw new NoSuchObjectException("HLRBRep_VertexList::Current");
        }
    }

    public bool IsBoundary => _fromEdge;

    public bool IsInterference => _fromInterf;

    /// <exception cref="DomainErrorException">The current vertex is not a boundary.</exception>
    public Orientation Orientation()
    {
        if (_fromEdge)
        {
            return _tool.CurrentOrientation();
        }
        else
        {
            throw new DomainErrorException("HLRBRep_VertexList::Orientation");
        }
    }

    /// <exception cref="DomainErrorException">The current vertex is not an interference.</exception>
    public Orientation Transition()
    {
        if (_fromInterf)
        {
            return _iterator.Value().Transition();
        }
        else
        {
            throw new DomainErrorException("HLRBRep_VertexList::Transition");
        }
    }

    /// <exception cref="DomainErrorException">The current vertex is not an interference.</exception>
    public Orientation BoundaryTransition()
    {
        if (_fromInterf)
        {
            return _iterator.Value().BoundaryTransition();
        }
        else
        {
            throw new DomainErrorException("HLRBRep_VertexList::BoundaryTransition");
        }
    }
}
