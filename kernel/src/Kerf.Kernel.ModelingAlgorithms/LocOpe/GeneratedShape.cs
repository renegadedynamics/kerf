// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_GeneratedShape.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_GeneratedShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Features.Local;

/// <summary>
/// A shape generated on a support shape, as <see cref="LocalGenerator"/> reads it: the edges it is glued along, the
/// edge each of their vertices generates, the face each generates, and its own faces.
/// </summary>
public abstract class GeneratedShape : Transient
{
    protected readonly NCollection_List<Shape> _gEdges = new();
    protected readonly NCollection_List<Shape> _list = new();

    public abstract NCollection_List<Shape> GeneratingEdges();

    /// <summary>Returns the edge created by the vertex <paramref name="v"/>. If none, must return a null shape.</summary>
    public abstract Edge Generated(in Vertex v);

    /// <summary>Returns the face created by the edge <paramref name="e"/>. If none, must return a null shape.</summary>
    public abstract Face Generated(in Edge e);

    /// <summary>Returns the list of correctly oriented generated faces.</summary>
    public abstract NCollection_List<Shape> OrientedFaces();
}
