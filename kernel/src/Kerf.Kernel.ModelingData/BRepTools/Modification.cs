// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_Modification.hxx
// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_Modification.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Meshing;

using Kerf.Geometry;

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Topology.Tools;

/// <summary>
/// Describes a modification of the geometry of a shape for <see cref="Modifier"/>: the new surface of each
/// face, curve of each edge, point of each vertex, curve of each edge on each face, parameter of each vertex on each
/// edge, and continuity of each edge between two faces. A method returns false when the shape keeps its geometry.
/// </summary>
/// <remarks>
/// The outputs are <c>out</c> parameters (OCCT's references, which the modifier reads only when the method returns
/// true), except <see cref="NewParameter"/>'s, which are <c>ref</c>: OCCT's implementations leave them unchanged for a
/// null vertex, and BRepTools_TrsfModification relies on that.
/// </remarks>
internal abstract class Modification : Transient
{
    /// <summary>
    /// True if the face has a new surface: then S is the surface, L its location, Tol the face's tolerance, RevWires
    /// true if the wires must be reversed, RevFace true if the face's orientation must be reversed.
    /// </summary>
    public abstract bool NewSurface(in Face f,
                                    out Surface? s,
                                    out Placement l,
                                    out double tol,
                                    out bool revWires,
                                    out bool revFace);

    /// <summary>True if the face has a new triangulation T. The default is false.</summary>
    public virtual bool NewTriangulation(in Face f, out Triangulation? t)
    {
        t = null;
        return false;
    }

    /// <summary>
    /// True if the edge has a new 3D curve (null: none, the edge keeps no 3D curve): then C is the curve, L its
    /// location and Tol the edge's tolerance.
    /// </summary>
    public abstract bool NewCurve(in Edge e, out Curve3D? c, out Placement l, out double tol);

    /// <summary>True if the edge has a new 3D polygon P. The default is false.</summary>
    public virtual bool NewPolygon(in Edge e, out Polygon3D? p)
    {
        p = null;
        return false;
    }

    /// <summary>True if the edge has a new polygon P on the face's triangulation. The default is false.</summary>
    public virtual bool NewPolygonOnTriangulation(in Edge e, in Face f, out PolygonOnTriangulation? p)
    {
        p = null;
        return false;
    }

    /// <summary>True if the vertex has a new point P with the tolerance Tol.</summary>
    public abstract bool NewPoint(in Vertex v, out Point3D p, out double tol);

    /// <summary>
    /// True if the edge E has a new curve C on the face F, with the tolerance Tol. NewE and NewF are the new edge and
    /// face (NewF may be null).
    /// </summary>
    public abstract bool NewCurve2d(in Edge e,
                                    in Face f,
                                    in Edge newE,
                                    in Face newF,
                                    out Curve2D? c,
                                    out double tol);

    /// <summary>
    /// True if the vertex has a new parameter P on the edge, with the tolerance Tol. P and Tol may be left unchanged
    /// when it returns false.
    /// </summary>
    public abstract bool NewParameter(in Vertex v, in Edge e, ref double p, ref double tol);

    /// <summary>The continuity of NewE between NewF1 and NewF2, the new shapes of E, F1 and F2.</summary>
    public abstract Continuity Continuity(in Edge e,
                                             in Face f1,
                                             in Face f2,
                                             in Edge newE,
                                             in Face newF1,
                                             in Face newF2);
}
