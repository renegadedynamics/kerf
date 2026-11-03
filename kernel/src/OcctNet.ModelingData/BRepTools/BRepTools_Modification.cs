// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_Modification.hxx
// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_Modification.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Meshing;

using Kerf.Geometry;

using Kerf.Topology;

namespace Occt;

/// <summary>
/// Describes a modification of the geometry of a shape for <see cref="BRepTools_Modifier"/>: the new surface of each
/// face, curve of each edge, point of each vertex, curve of each edge on each face, parameter of each vertex on each
/// edge, and continuity of each edge between two faces. A method returns false when the shape keeps its geometry.
/// </summary>
/// <remarks>
/// The outputs are <c>out</c> parameters (OCCT's references, which the modifier reads only when the method returns
/// true), except <see cref="NewParameter"/>'s, which are <c>ref</c>: OCCT's implementations leave them unchanged for a
/// null vertex, and BRepTools_TrsfModification relies on that.
/// </remarks>
public abstract class BRepTools_Modification : Transient
{
    /// <summary>
    /// True if the face has a new surface: then S is the surface, L its location, Tol the face's tolerance, RevWires
    /// true if the wires must be reversed, RevFace true if the face's orientation must be reversed.
    /// </summary>
    public abstract bool NewSurface(in TopoDS_Face F,
                                    out Geom_Surface? S,
                                    out Placement L,
                                    out double Tol,
                                    out bool RevWires,
                                    out bool RevFace);

    /// <summary>True if the face has a new triangulation T. The default is false.</summary>
    public virtual bool NewTriangulation(in TopoDS_Face F, out Triangulation? T)
    {
        T = null;
        return false;
    }

    /// <summary>
    /// True if the edge has a new 3D curve (null: none, the edge keeps no 3D curve): then C is the curve, L its
    /// location and Tol the edge's tolerance.
    /// </summary>
    public abstract bool NewCurve(in TopoDS_Edge E, out Geom_Curve? C, out Placement L, out double Tol);

    /// <summary>True if the edge has a new 3D polygon P. The default is false.</summary>
    public virtual bool NewPolygon(in TopoDS_Edge E, out Polygon3D? P)
    {
        P = null;
        return false;
    }

    /// <summary>True if the edge has a new polygon P on the face's triangulation. The default is false.</summary>
    public virtual bool NewPolygonOnTriangulation(in TopoDS_Edge E, in TopoDS_Face F, out PolygonOnTriangulation? P)
    {
        P = null;
        return false;
    }

    /// <summary>True if the vertex has a new point P with the tolerance Tol.</summary>
    public abstract bool NewPoint(in TopoDS_Vertex V, out Point3D P, out double Tol);

    /// <summary>
    /// True if the edge E has a new curve C on the face F, with the tolerance Tol. NewE and NewF are the new edge and
    /// face (NewF may be null).
    /// </summary>
    public abstract bool NewCurve2d(in TopoDS_Edge E,
                                    in TopoDS_Face F,
                                    in TopoDS_Edge NewE,
                                    in TopoDS_Face NewF,
                                    out Geom2d_Curve? C,
                                    out double Tol);

    /// <summary>
    /// True if the vertex has a new parameter P on the edge, with the tolerance Tol. P and Tol may be left unchanged
    /// when it returns false.
    /// </summary>
    public abstract bool NewParameter(in TopoDS_Vertex V, in TopoDS_Edge E, ref double P, ref double Tol);

    /// <summary>The continuity of NewE between NewF1 and NewF2, the new shapes of E, F1 and F2.</summary>
    public abstract Continuity Continuity(in TopoDS_Edge E,
                                             in TopoDS_Face F1,
                                             in TopoDS_Face F2,
                                             in TopoDS_Edge NewE,
                                             in TopoDS_Face NewF1,
                                             in TopoDS_Face NewF2);
}
