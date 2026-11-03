// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_ShapeWithState.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_ShapeWithState.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using static Kerf.Topology.Containment;

namespace Occt;

/// <summary>A shape's state and its parts IN, OUT and ON the other argument, for the legacy Booleans' classification.</summary>
/// <remarks>Storage of <see cref="TopOpeBRepDS_DataStructure"/>; the fillets don't use it.</remarks>
public sealed class TopOpeBRepDS_ShapeWithState
{
    private static readonly NCollection_List<Shape> theEmptyListOfShape = new();

    private readonly NCollection_List<Shape> myPartIn = new();
    private readonly NCollection_List<Shape> myPartOut = new();
    private readonly NCollection_List<Shape> myPartOn = new();
    private Containment myState;
    private bool myIsSplitted;

    /// <summary>An unknown state, no parts.</summary>
    public TopOpeBRepDS_ShapeWithState()
    {
        myState = Unknown;
        myIsSplitted = false;
    }

    /// <summary>The parts in the state <paramref name="aState"/> (an empty list for UNKNOWN).</summary>
    /// <remarks>OCCT's empty list is a function-scope static; the port's is a static never written (callers only read it).</remarks>
    public NCollection_List<Shape> Part(Containment aState)
    {
        switch (aState)
        {
            case In:
                return myPartIn;
            case Out:
                return myPartOut;
            case On:
                return myPartOn;
            default:
                return theEmptyListOfShape;
        }
    }

    internal void AddPart(in Shape aShape, Containment aState)
    {
        switch (aState)
        {
            case In:
                myPartIn.Append(aShape);
                break;
            case Out:
                myPartOut.Append(aShape);
                break;
            case On:
                myPartOn.Append(aShape);
                break;
            default:
                break;
        }
    }

    internal void AddParts(NCollection_List<Shape> aListOfShape, Containment aState)
    {
        NCollection_List<Shape>.Iterator anIt = new(aListOfShape);

        switch (aState)
        {
            case In:
                for (; anIt.More(); anIt.Next())
                {
                    myPartIn.Append(anIt.Value());
                }
                break;
            case Out:
                for (; anIt.More(); anIt.Next())
                {
                    myPartOut.Append(anIt.Value());
                }
                break;
            case On:
                for (; anIt.More(); anIt.Next())
                {
                    myPartOn.Append(anIt.Value());
                }
                break;
            default:
                break;
        }
    }

    internal void SetState(Containment aState)
    {
        myState = aState;
    }

    /// <summary>The state.</summary>
    public Containment State() => myState;

    internal void SetIsSplitted(bool aFlag)
    {
        myIsSplitted = aFlag;
    }

    /// <summary>Whether the shape is split.</summary>
    public bool IsSplitted() => myIsSplitted;
}
