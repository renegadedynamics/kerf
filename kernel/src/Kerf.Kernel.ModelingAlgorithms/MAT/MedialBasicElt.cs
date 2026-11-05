// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_BasicElt.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_BasicElt.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine;

namespace Kerf.Engine.Offsets.MedialAxis;

/// <summary>A basic element of the bisecting locus: an element of the contour (a curve or a vertex), with the arcs of
/// the graph that start and end its zone.</summary>
/// <remarks>OCCT keeps the arcs as raw pointers (no reference count, to break cycles); the port keeps references. The
/// setters are internal (MAT_Graph and MAT2d build the graph).</remarks>
internal sealed class MedialBasicElt : Transient
{
    private MedialArc? _startLeftArc;
    private MedialArc? _endLeftArc;
    private int _index;
    private int _geomIndex;

    /// <summary>A basic element of index anInteger.</summary>
    public MedialBasicElt(int anInteger)
    {
        _startLeftArc = null;
        _endLeftArc = null;
        _index = anInteger;
        _geomIndex = 0;
    }

    /// <summary>The first arc of the zone of the element (null if none).</summary>
    public MedialArc? StartArc() => _startLeftArc;

    /// <summary>The last arc of the zone of the element (null if none).</summary>
    public MedialArc? EndArc() => _endLeftArc;

    public int Index() => _index;

    public int GeomIndex() => _geomIndex;

    internal void SetStartArc(MedialArc? anArc) => _startLeftArc = anArc;

    internal void SetEndArc(MedialArc? anArc) => _endLeftArc = anArc;

    internal void SetIndex(int anInteger) => _index = anInteger;

    internal void SetGeomIndex(int anInteger) => _geomIndex = anInteger;
}
