// Port of: src/ModelingAlgorithms/TKPrim/Sweep/Sweep_NumShapeTool.hxx
// Port of: src/ModelingAlgorithms/TKPrim/Sweep/Sweep_NumShapeTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Construction.Sweeping;

/// <summary>Indexes the sub-shapes of a <see cref="NumShape"/>: the edge is 1, its vertices follow.</summary>
internal sealed class NumShapeTool
{
    private readonly NumShape _numShape;

    public NumShapeTool(in NumShape aShape)
    {
        _numShape = aShape;
    }

    public int NbShapes()
    {
        if (_numShape.Type() == ShapeType.Edge)
        {
            if (_numShape.Closed())
            {
                return _numShape.Index();
            }
            else
            {
                return _numShape.Index() + 1;
            }
        }
        else
        {
            return 1;
        }
    }

    public int Index(in NumShape aShape)
    {
        if (aShape.Type() == ShapeType.Edge)
        {
            return 1;
        }
        else
        {
            if (aShape.Closed())
            {
                return 2;
            }
            else
            {
                return (aShape.Index() + 1);
            }
        }
    }

    public NumShape Shape(int anIndex)
    {
        if (anIndex == 1)
        {
            return _numShape;
        }
        else
        {
            return new NumShape((anIndex - 1), ShapeType.Vertex, _numShape.Closed(), false, false);
        }
    }

    public ShapeType Type(in NumShape aShape) => aShape.Type();

    public Orientation Orientation(in NumShape aShape) => aShape.Orientation();

    public bool HasFirstVertex()
    {
        if (_numShape.Type() == ShapeType.Edge)
        {
            return !_numShape.BegInfinite();
        }
        return true;
    }

    public bool HasLastVertex()
    {
        if (_numShape.Type() == ShapeType.Edge)
        {
            return !_numShape.EndInfinite();
        }
        return true;
    }

    public NumShape FirstVertex()
    {
        if (_numShape.Type() == ShapeType.Edge)
        {
            if (HasFirstVertex())
            {
                return new NumShape(1, ShapeType.Vertex, _numShape.Closed(), false, false);
            }
            else
            {
                throw new ConstructionErrorException("infinite Shape");
            }
        }
        return _numShape;
    }

    public NumShape LastVertex()
    {
        if (_numShape.Type() == ShapeType.Edge)
        {
            if (HasLastVertex())
            {
                return new NumShape(NbShapes() - 1, ShapeType.Vertex, _numShape.Closed(), false, false);
            }
            else
            {
                throw new ConstructionErrorException("infinite Shape");
            }
        }
        return _numShape;
    }
}
