// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_ShapeWithState.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_ShapeWithState.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_State;

namespace Occt;

/// <summary>A shape's state and its parts IN, OUT and ON the other argument, for the legacy Booleans' classification.</summary>
/// <remarks>Storage of <see cref="TopOpeBRepDS_DataStructure"/>; the fillets don't use it.</remarks>
public sealed class TopOpeBRepDS_ShapeWithState
{
    private static readonly NCollection_List<TopoDS_Shape> theEmptyListOfShape = new();

    private readonly NCollection_List<TopoDS_Shape> myPartIn = new();
    private readonly NCollection_List<TopoDS_Shape> myPartOut = new();
    private readonly NCollection_List<TopoDS_Shape> myPartOn = new();
    private TopAbs_State myState;
    private bool myIsSplitted;

    /// <summary>An unknown state, no parts.</summary>
    public TopOpeBRepDS_ShapeWithState()
    {
        myState = TopAbs_UNKNOWN;
        myIsSplitted = false;
    }

    /// <summary>The parts in the state <paramref name="aState"/> (an empty list for UNKNOWN).</summary>
    /// <remarks>OCCT's empty list is a function-scope static; the port's is a static never written (callers only read it).</remarks>
    public NCollection_List<TopoDS_Shape> Part(TopAbs_State aState)
    {
        switch (aState)
        {
            case TopAbs_IN:
                return myPartIn;
            case TopAbs_OUT:
                return myPartOut;
            case TopAbs_ON:
                return myPartOn;
            default:
                return theEmptyListOfShape;
        }
    }

    internal void AddPart(in TopoDS_Shape aShape, TopAbs_State aState)
    {
        switch (aState)
        {
            case TopAbs_IN:
                myPartIn.Append(aShape);
                break;
            case TopAbs_OUT:
                myPartOut.Append(aShape);
                break;
            case TopAbs_ON:
                myPartOn.Append(aShape);
                break;
            default:
                break;
        }
    }

    internal void AddParts(NCollection_List<TopoDS_Shape> aListOfShape, TopAbs_State aState)
    {
        NCollection_List<TopoDS_Shape>.Iterator anIt = new(aListOfShape);

        switch (aState)
        {
            case TopAbs_IN:
                for (; anIt.More(); anIt.Next())
                {
                    myPartIn.Append(anIt.Value());
                }
                break;
            case TopAbs_OUT:
                for (; anIt.More(); anIt.Next())
                {
                    myPartOut.Append(anIt.Value());
                }
                break;
            case TopAbs_ON:
                for (; anIt.More(); anIt.Next())
                {
                    myPartOn.Append(anIt.Value());
                }
                break;
            default:
                break;
        }
    }

    internal void SetState(TopAbs_State aState)
    {
        myState = aState;
    }

    /// <summary>The state.</summary>
    public TopAbs_State State() => myState;

    internal void SetIsSplitted(bool aFlag)
    {
        myIsSplitted = aFlag;
    }

    /// <summary>Whether the shape is split.</summary>
    public bool IsSplitted() => myIsSplitted;
}
