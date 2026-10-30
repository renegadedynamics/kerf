// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_BasicElt.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_BasicElt.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A basic element of the bisecting locus: an element of the contour (a curve or a vertex), with the arcs of
/// the graph that start and end its zone.</summary>
/// <remarks>OCCT keeps the arcs as raw pointers (no reference count, to break cycles); the port keeps references. The
/// setters are internal (MAT_Graph and MAT2d build the graph).</remarks>
public sealed class MAT_BasicElt : Standard_Transient
{
    private MAT_Arc? startLeftArc;
    private MAT_Arc? endLeftArc;
    private int index;
    private int geomIndex;

    /// <summary>A basic element of index anInteger.</summary>
    public MAT_BasicElt(int anInteger)
    {
        startLeftArc = null;
        endLeftArc = null;
        index = anInteger;
        geomIndex = 0;
    }

    /// <summary>The first arc of the zone of the element (null if none).</summary>
    public MAT_Arc? StartArc() => startLeftArc;

    /// <summary>The last arc of the zone of the element (null if none).</summary>
    public MAT_Arc? EndArc() => endLeftArc;

    public int Index() => index;

    public int GeomIndex() => geomIndex;

    internal void SetStartArc(MAT_Arc? anArc) => startLeftArc = anArc;

    internal void SetEndArc(MAT_Arc? anArc) => endLeftArc = anArc;

    internal void SetIndex(int anInteger) => index = anInteger;

    internal void SetGeomIndex(int anInteger) => geomIndex = anInteger;
}
