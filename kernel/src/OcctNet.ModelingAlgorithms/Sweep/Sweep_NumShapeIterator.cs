// Port of: src/ModelingAlgorithms/TKPrim/Sweep/Sweep_NumShapeIterator.hxx
// Port of: src/ModelingAlgorithms/TKPrim/Sweep/Sweep_NumShapeIterator.lxx
// Port of: src/ModelingAlgorithms/TKPrim/Sweep/Sweep_NumShapeIterator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>Iterates on the vertices of a <see cref="Sweep_NumShape"/>.</summary>
internal sealed class Sweep_NumShapeIterator
{
    private Sweep_NumShape myNumShape;
    private Sweep_NumShape myCurrentNumShape;
    private int myCurrentRange;
    private bool myMore;
    private TopAbs_Orientation myCurrentOrientation; // OCCT leaves it uninitialised until Init

    public Sweep_NumShapeIterator()
    {
        myNumShape = new Sweep_NumShape(0, TopAbs_SHAPE);
        myCurrentNumShape = new Sweep_NumShape(0, TopAbs_SHAPE);
        myCurrentRange = 0;
        myMore = false;
    }

    /// <summary>Resets the iterator on the sub-shapes of <paramref name="aShape"/>.</summary>
    public void Init(in Sweep_NumShape aShape)
    {
        myNumShape = aShape;
        if (myNumShape.Type() == TopAbs_EDGE)
        {
            int nbvert = myNumShape.Index();
            myMore = (nbvert >= 1);
            if (myMore)
            {
                myCurrentRange = 1;
                myCurrentNumShape = new Sweep_NumShape(1, TopAbs_VERTEX, myNumShape.Closed(), false, false);
                if (nbvert == 1)
                {
                    if (myNumShape.BegInfinite())
                    {
                        myCurrentOrientation = TopAbs_REVERSED;
                    }
                    else
                    {
                        myCurrentOrientation = TopAbs_FORWARD;
                    }
                }
                else
                {
                    myCurrentOrientation = TopAbs_FORWARD;
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
            if (myNumShape.Type() == TopAbs_EDGE)
            {
                myCurrentNumShape = new Sweep_NumShape(myCurrentRange, TopAbs_VERTEX, myNumShape.Closed(), false, false);
                myCurrentOrientation = TopAbs_REVERSED;
            }
        }
    }

    public Sweep_NumShape Value() => myCurrentNumShape;

    public TopAbs_Orientation Orientation() => myCurrentOrientation;
}
