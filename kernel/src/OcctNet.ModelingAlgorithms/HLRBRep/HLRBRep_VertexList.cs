// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_VertexList.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_VertexList.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Iterates, by parameter, on the vertices of an edge and its interferences merged: each step is a boundary vertex,
/// an interference, or both.
/// </summary>
/// <remarks>
/// A working object, as OCCT's. It copies the tool and the iterator it is given, as OCCT does, so the caller's are
/// not moved.
/// </remarks>
public sealed class HLRBRep_VertexList
{
    private readonly NCollection_List<HLRAlgo_Interference>.Iterator myIterator;
    private readonly HLRBRep_EdgeInterferenceTool myTool;
    private bool fromEdge;
    private bool fromInterf;

    public HLRBRep_VertexList(HLRBRep_EdgeInterferenceTool T, NCollection_List<HLRAlgo_Interference>.Iterator I)
    {
        myIterator = new NCollection_List<HLRAlgo_Interference>.Iterator(I);
        myTool = new HLRBRep_EdgeInterferenceTool(T);
        fromEdge = false;
        fromInterf = false;
        myTool.InitVertices();
        Next();
    }

    public bool IsPeriodic() => myTool.IsPeriodic();

    public bool More() => (fromEdge || fromInterf);

    public void Next()
    {
        if (fromInterf)
        {
            myIterator.Next();
        }
        if (fromEdge)
        {
            myTool.NextVertex();
        }
        fromInterf = myIterator.More();
        fromEdge = myTool.MoreVertices();
        if (fromEdge && fromInterf)
        {
            if (!myTool.SameVertexAndInterference(myIterator.Value()))
            {
                if (myTool.CurrentParameter() < myTool.ParameterOfInterference(myIterator.Value()))
                {
                    fromInterf = false;
                }
                else
                {
                    fromEdge = false;
                }
            }
        }
    }

    /// <summary>The current vertex: the boundary vertex if there is one, else the interference's intersection.</summary>
    /// <exception cref="Standard_NoSuchObject">The iteration is over.</exception>
    public HLRAlgo_Intersection Current()
    {
        if (fromEdge)
        {
            return myTool.CurrentVertex();
        }
        else if (fromInterf)
        {
            return myIterator.Value().Intersection();
        }
        else
        {
            throw new Standard_NoSuchObject("HLRBRep_VertexList::Current");
        }
    }

    public bool IsBoundary() => fromEdge;

    public bool IsInterference() => fromInterf;

    /// <exception cref="Standard_DomainError">The current vertex is not a boundary.</exception>
    public TopAbs_Orientation Orientation()
    {
        if (fromEdge)
        {
            return myTool.CurrentOrientation();
        }
        else
        {
            throw new Standard_DomainError("HLRBRep_VertexList::Orientation");
        }
    }

    /// <exception cref="Standard_DomainError">The current vertex is not an interference.</exception>
    public TopAbs_Orientation Transition()
    {
        if (fromInterf)
        {
            return myIterator.Value().Transition();
        }
        else
        {
            throw new Standard_DomainError("HLRBRep_VertexList::Transition");
        }
    }

    /// <exception cref="Standard_DomainError">The current vertex is not an interference.</exception>
    public TopAbs_Orientation BoundaryTransition()
    {
        if (fromInterf)
        {
            return myIterator.Value().BoundaryTransition();
        }
        else
        {
            throw new Standard_DomainError("HLRBRep_VertexList::BoundaryTransition");
        }
    }
}
