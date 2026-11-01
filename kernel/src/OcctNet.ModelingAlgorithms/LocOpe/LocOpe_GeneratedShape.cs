// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_GeneratedShape.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_GeneratedShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A shape generated on a support shape, as <see cref="LocOpe_Generator"/> reads it: the edges it is glued along, the
/// edge each of their vertices generates, the face each generates, and its own faces.
/// </summary>
public abstract class LocOpe_GeneratedShape : Standard_Transient
{
    protected readonly NCollection_List<TopoDS_Shape> myGEdges = new();
    protected readonly NCollection_List<TopoDS_Shape> myList = new();

    public abstract NCollection_List<TopoDS_Shape> GeneratingEdges();

    /// <summary>Returns the edge created by the vertex <paramref name="V"/>. If none, must return a null shape.</summary>
    public abstract TopoDS_Edge Generated(in TopoDS_Vertex V);

    /// <summary>Returns the face created by the edge <paramref name="E"/>. If none, must return a null shape.</summary>
    public abstract TopoDS_Face Generated(in TopoDS_Edge E);

    /// <summary>Returns the list of correctly oriented generated faces.</summary>
    public abstract NCollection_List<TopoDS_Shape> OrientedFaces();
}
