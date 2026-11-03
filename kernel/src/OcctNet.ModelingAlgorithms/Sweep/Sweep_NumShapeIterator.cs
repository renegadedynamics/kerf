// Port of: src/ModelingAlgorithms/TKPrim/Sweep/Sweep_NumShapeIterator.hxx
// Port of: src/ModelingAlgorithms/TKPrim/Sweep/Sweep_NumShapeIterator.lxx
// Port of: src/ModelingAlgorithms/TKPrim/Sweep/Sweep_NumShapeIterator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>Iterates on the vertices of a <see cref="Sweep_NumShape"/>.</summary>
internal sealed class Sweep_NumShapeIterator
{
    private Sweep_NumShape myNumShape;
    private Sweep_NumShape myCurrentNumShape;
    private int myCurrentRange;
    private bool myMore;
    private Orientation myCurrentOrientation; // OCCT leaves it uninitialised until Init

    public Sweep_NumShapeIterator()
    {
        myNumShape = new Sweep_NumShape(0, ShapeType.Shape);
        myCurrentNumShape = new Sweep_NumShape(0, ShapeType.Shape);
        myCurrentRange = 0;
        myMore = false;
    }

    /// <summary>Resets the iterator on the sub-shapes of <paramref name="aShape"/>.</summary>
    public void Init(in Sweep_NumShape aShape)
    {
        myNumShape = aShape;
        if (myNumShape.Type() == ShapeType.Edge)
        {
            int nbvert = myNumShape.Index();
            myMore = (nbvert >= 1);
            if (myMore)
            {
                myCurrentRange = 1;
                myCurrentNumShape = new Sweep_NumShape(1, ShapeType.Vertex, myNumShape.Closed(), false, false);
                if (nbvert == 1)
                {
                    if (myNumShape.BegInfinite())
                    {
                        myCurrentOrientation = Reversed;
                    }
                    else
                    {
                        myCurrentOrientation = Forward;
                    }
                }
                else
                {
                    myCurrentOrientation = Forward;
                }
            }
        }
    }

    public bool More() => myMore;

    public void Next()
    {
        myCurrentRange++;
        myMore = myCurrentRange <= myNumShape.Index();
        if (myMore)
        {
            if (myNumShape.Type() == ShapeType.Edge)
            {
                myCurrentNumShape = new Sweep_NumShape(myCurrentRange, ShapeType.Vertex, myNumShape.Closed(), false, false);
                myCurrentOrientation = Reversed;
            }
        }
    }

    public Sweep_NumShape Value() => myCurrentNumShape;

    public Orientation Orientation() => myCurrentOrientation;
}
