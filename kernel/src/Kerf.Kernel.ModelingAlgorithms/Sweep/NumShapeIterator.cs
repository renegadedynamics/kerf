// Port of: src/ModelingAlgorithms/TKPrim/Sweep/Sweep_NumShapeIterator.hxx
// Port of: src/ModelingAlgorithms/TKPrim/Sweep/Sweep_NumShapeIterator.lxx
// Port of: src/ModelingAlgorithms/TKPrim/Sweep/Sweep_NumShapeIterator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Construction.Sweeping;

/// <summary>Iterates on the vertices of a <see cref="NumShape"/>.</summary>
internal sealed class NumShapeIterator
{
    private NumShape _numShape;
    private NumShape _currentNumShape;
    private int _currentRange;
    private bool _more;
    private Orientation _currentOrientation; // OCCT leaves it uninitialised until Init

    public NumShapeIterator()
    {
        _numShape = new NumShape(0, ShapeType.Shape);
        _currentNumShape = new NumShape(0, ShapeType.Shape);
        _currentRange = 0;
        _more = false;
    }

    /// <summary>Resets the iterator on the sub-shapes of <paramref name="aShape"/>.</summary>
    public void Init(in NumShape aShape)
    {
        _numShape = aShape;
        if (_numShape.Type() == ShapeType.Edge)
        {
            int nbvert = _numShape.Index();
            _more = (nbvert >= 1);
            if (_more)
            {
                _currentRange = 1;
                _currentNumShape = new NumShape(1, ShapeType.Vertex, _numShape.Closed(), false, false);
                if (nbvert == 1)
                {
                    if (_numShape.BegInfinite())
                    {
                        _currentOrientation = Reversed;
                    }
                    else
                    {
                        _currentOrientation = Forward;
                    }
                }
                else
                {
                    _currentOrientation = Forward;
                }
            }
        }
    }

    public bool More() => _more;

    public void Next()
    {
        _currentRange++;
        _more = _currentRange <= _numShape.Index();
        if (_more)
        {
            if (_numShape.Type() == ShapeType.Edge)
            {
                _currentNumShape = new NumShape(_currentRange, ShapeType.Vertex, _numShape.Closed(), false, false);
                _currentOrientation = Reversed;
            }
        }
    }

    public NumShape Value() => _currentNumShape;

    public Orientation Orientation() => _currentOrientation;
}
