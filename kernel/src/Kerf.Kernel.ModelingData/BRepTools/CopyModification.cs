// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_CopyModification.hxx
// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_CopyModification.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing;

using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Topology.Tools;

/// <summary>
/// The modification that copies a shape: every face, edge and vertex gets its own geometry back, copied if
/// <c>theCopyGeom</c>, and its triangulation and polygons if <c>theCopyMesh</c> (copied too if <c>theCopyGeom</c>).
/// </summary>
/// <remarks>
/// Without <c>theCopyGeom</c> the copy shares the original's curves, surfaces and meshes, which are frozen: that is
/// the structural copy of <see cref="BrepTools.CopyForWrite"/> (docs/phase3-design.md, section 4.6).
/// </remarks>
internal class CopyModification : Modification
{
    private readonly bool _copyGeom;
    private readonly bool _copyMesh;

    /// <summary>A copy modification; <paramref name="copyGeom"/> copies the geometry, <paramref name="copyMesh"/> the triangulations.</summary>
    public CopyModification(bool copyGeom = true, bool copyMesh = true)
    {
        _copyGeom = copyGeom;
        _copyMesh = copyMesh;
    }

    /// <summary>The face's surface, location and tolerance (the surface copied if the geometry is copied); always true.</summary>
    public override bool NewSurface(in Face face,
                                    out Surface? surf,
                                    out Placement loc,
                                    out double tol,
                                    out bool revWires,
                                    out bool revFace)
    {
        surf = Tool.Surface(face, out loc);
        tol = Tool.Tolerance(face);
        revWires = revFace = false;

        if (surf is not null && _copyGeom)
        {
            surf = (Surface)surf.Copy();
        }

        return true;
    }

    /// <summary>The face's triangulation (copied if the geometry is copied), if the mesh is copied or the face has no surface.</summary>
    public override bool NewTriangulation(in Face face, out Triangulation? tri)
    {
        tri = null;
        if (!_copyMesh && Tool.IsGeometric(face))
        {
            return false;
        }

        tri = Tool.Triangulation(face, out _);

        if (tri is null)
        {
            return false;
        }

        // mesh is copied if and only if the geometry need to be copied too
        if (_copyGeom)
        {
            tri = tri.Copy();
        }
        return true;
    }

    /// <summary>The edge's 3D curve, location and tolerance (the curve copied if the geometry is copied); always true.</summary>
    public override bool NewCurve(in Edge edge, out Curve3D? curve, out Placement loc, out double tol)
    {
        curve = Tool.Curve(edge, out loc, out _, out _);
        tol = Tool.Tolerance(edge);

        if (curve is not null && _copyGeom)
        {
            curve = (Curve3D)curve.Copy();
        }

        return true;
    }

    /// <summary>The edge's 3D polygon (copied if the geometry is copied), if the mesh is copied or the edge has no curve.</summary>
    public override bool NewPolygon(in Edge edge, out Polygon3D? poly)
    {
        poly = null;
        if (!_copyMesh && Tool.IsGeometric(edge))
        {
            return false;
        }

        poly = Tool.Polygon3D(edge, out _);

        if (poly is null)
        {
            return false;
        }

        // polygon is copied if and only if the geometry need to be copied too
        if (_copyGeom)
        {
            poly = poly.Copy();
        }
        return true;
    }

    /// <summary>
    /// The edge's polygon on the face's triangulation (copied if the geometry is copied), if the mesh is copied or the
    /// edge has no curve.
    /// </summary>
    public override bool NewPolygonOnTriangulation(in Edge edge, in Face face, out PolygonOnTriangulation? poly)
    {
        poly = null;
        if (!_copyMesh && Tool.IsGeometric(edge))
        {
            return false;
        }

        Triangulation? tria = Tool.Triangulation(face, out Placement loc);
        poly = Tool.PolygonOnTriangulation(edge, tria, loc);

        if (poly is null)
        {
            return false;
        }

        // polygon is copied if and only if the geometry need to be copied too
        if (_copyGeom)
        {
            poly = poly.Copy();
        }
        return true;
    }

    /// <summary>The vertex's point and tolerance; always true.</summary>
    public override bool NewPoint(in Vertex vertex, out Point3D pnt, out double tol)
    {
        pnt = Tool.Pnt(vertex);
        tol = Tool.Tolerance(vertex);
        return true;
    }

    /// <summary>The edge's pcurve on the face (copied if the geometry is copied) and its tolerance; always true.</summary>
    public override bool NewCurve2d(in Edge edge,
                                    in Face face,
                                    in Edge newEdge,
                                    in Face newFace,
                                    out Curve2D? curve,
                                    out double tol)
    {
        tol = Tool.Tolerance(edge);
        curve = Tool.CurveOnSurface(edge, face, out _, out _);

        if (curve is not null && _copyGeom)
        {
            curve = (Curve2D)curve.Copy();
        }

        return true;
    }

    /// <summary>The vertex's parameter on the edge and its tolerance; false for a null vertex.</summary>
    public override bool NewParameter(in Vertex vertex, in Edge edge, ref double pnt, ref double tol)
    {
        if (vertex.IsNull)
        {
            return false; // infinite edge may have Null vertex
        }

        tol = Tool.Tolerance(vertex);
        pnt = Tool.Parameter(vertex, edge);

        return true;
    }

    /// <summary>The edge's continuity between the original faces.</summary>
    public override Continuity Continuity(in Edge edge,
                                             in Face face1,
                                             in Face face2,
                                             in Edge newEdge,
                                             in Face newFace1,
                                             in Face newFace2)
    {
        return Tool.Continuity(edge, face1, face2);
    }
}
