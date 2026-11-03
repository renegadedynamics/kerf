// Port of: src/ModelingAlgorithms/TKPrim/Sweep/Sweep_NumShapeTool.hxx
// Port of: src/ModelingAlgorithms/TKPrim/Sweep/Sweep_NumShapeTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>Indexes the sub-shapes of a <see cref="Sweep_NumShape"/>: the edge is 1, its vertices follow.</summary>
internal sealed class Sweep_NumShapeTool
{
    private readonly Sweep_NumShape myNumShape;

    public Sweep_NumShapeTool(in Sweep_NumShape aShape)
    {
        myNumShape = aShape;
    }

    public int NbShapes()
    {
        if (myNumShape.Type() == ShapeType.Edge)
        {
            if (myNumShape.Closed())
            {
                return myNumShape.Index();
            }
            else
            {
                return myNumShape.Index() + 1;
            }
        }
        else
        {
            return 1;
        }
    }

    public int Index(in Sweep_NumShape aShape)
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

    public Sweep_NumShape Shape(int anIndex)
    {
        if (anIndex == 1)
        {
            return myNumShape;
        }
        else
        {
            return new Sweep_NumShape((anIndex - 1), ShapeType.Vertex, myNumShape.Closed(), false, false);
        }
    }

    public ShapeType Type(in Sweep_NumShape aShape) => aShape.Type();

    public Orientation Orientation(in Sweep_NumShape aShape) => aShape.Orientation();

    public bool HasFirstVertex()
    {
        if (myNumShape.Type() == ShapeType.Edge)
        {
            return !myNumShape.BegInfinite();
        }
        return true;
    }

    public bool HasLastVertex()
    {
        if (myNumShape.Type() == ShapeType.Edge)
        {
            return !myNumShape.EndInfinite();
        }
        return true;
    }

    public Sweep_NumShape FirstVertex()
    {
        if (myNumShape.Type() == ShapeType.Edge)
        {
            if (HasFirstVertex())
            {
                return new Sweep_NumShape(1, ShapeType.Vertex, myNumShape.Closed(), false, false);
            }
            else
            {
                throw new ConstructionErrorException("infinite Shape");
            }
        }
        return myNumShape;
    }

    public Sweep_NumShape LastVertex()
    {
        if (myNumShape.Type() == ShapeType.Edge)
        {
            if (HasLastVertex())
            {
                return new Sweep_NumShape(NbShapes() - 1, ShapeType.Vertex, myNumShape.Closed(), false, false);
            }
            else
            {
                throw new ConstructionErrorException("infinite Shape");
            }
        }
        return myNumShape;
    }
}
