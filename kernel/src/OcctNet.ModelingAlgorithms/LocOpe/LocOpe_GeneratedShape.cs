// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_GeneratedShape.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_GeneratedShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Topology;

namespace Occt;

/// <summary>
/// A shape generated on a support shape, as <see cref="LocOpe_Generator"/> reads it: the edges it is glued along, the
/// edge each of their vertices generates, the face each generates, and its own faces.
/// </summary>
public abstract class LocOpe_GeneratedShape : Transient
{
    protected readonly NCollection_List<Shape> myGEdges = new();
    protected readonly NCollection_List<Shape> myList = new();

    public abstract NCollection_List<Shape> GeneratingEdges();

    /// <summary>Returns the edge created by the vertex <paramref name="V"/>. If none, must return a null shape.</summary>
    public abstract Edge Generated(in Vertex V);

    /// <summary>Returns the face created by the edge <paramref name="E"/>. If none, must return a null shape.</summary>
    public abstract Face Generated(in Edge E);

    /// <summary>Returns the list of correctly oriented generated faces.</summary>
    public abstract NCollection_List<Shape> OrientedFaces();
}
