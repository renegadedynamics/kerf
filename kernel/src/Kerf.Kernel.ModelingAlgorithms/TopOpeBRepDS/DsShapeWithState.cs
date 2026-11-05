// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_ShapeWithState.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_ShapeWithState.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using Occt;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Booleans.Legacy.Data;

/// <summary>A shape's state and its parts IN, OUT and ON the other argument, for the legacy Booleans' classification.</summary>
/// <remarks>Storage of <see cref="DsDataStructure"/>; the fillets don't use it.</remarks>
internal sealed class DsShapeWithState
{
    private static readonly NCollection_List<Shape> EmptyListOfShape = new();

    private readonly NCollection_List<Shape> _partIn = new();
    private readonly NCollection_List<Shape> _partOut = new();
    private readonly NCollection_List<Shape> _partOn = new();
    private Containment _state;
    private bool _isSplitted;

    /// <summary>An unknown state, no parts.</summary>
    public DsShapeWithState()
    {
        _state = Unknown;
        _isSplitted = false;
    }

    /// <summary>The parts in the state <paramref name="aState"/> (an empty list for UNKNOWN).</summary>
    /// <remarks>OCCT's empty list is a function-scope static; the port's is a static never written (callers only read it).</remarks>
    public NCollection_List<Shape> Part(Containment aState)
    {
        switch (aState)
        {
            case In:
                return _partIn;
            case Out:
                return _partOut;
            case On:
                return _partOn;
            default:
                return EmptyListOfShape;
        }
    }

    internal void AddPart(in Shape aShape, Containment aState)
    {
        switch (aState)
        {
            case In:
                _partIn.Append(aShape);
                break;
            case Out:
                _partOut.Append(aShape);
                break;
            case On:
                _partOn.Append(aShape);
                break;
            default:
                break;
        }
    }

    internal void AddParts(NCollection_List<Shape> aListOfShape, Containment aState)
    {
        NCollection_List<Shape>.Iterator it = new(aListOfShape);

        switch (aState)
        {
            case In:
                for (; it.More(); it.Next())
                {
                    _partIn.Append(it.Value());
                }
                break;
            case Out:
                for (; it.More(); it.Next())
                {
                    _partOut.Append(it.Value());
                }
                break;
            case On:
                for (; it.More(); it.Next())
                {
                    _partOn.Append(it.Value());
                }
                break;
            default:
                break;
        }
    }

    internal void SetState(Containment aState)
    {
        _state = aState;
    }

    /// <summary>The state.</summary>
    public Containment State() => _state;

    internal void SetIsSplitted(bool aFlag)
    {
        _isSplitted = aFlag;
    }

    /// <summary>Whether the shape is split.</summary>
    public bool IsSplitted => _isSplitted;
}
