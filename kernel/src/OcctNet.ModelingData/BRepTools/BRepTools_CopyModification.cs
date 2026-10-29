// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_CopyModification.hxx
// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_CopyModification.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The modification that copies a shape: every face, edge and vertex gets its own geometry back, copied if
/// <c>theCopyGeom</c>, and its triangulation and polygons if <c>theCopyMesh</c> (copied too if <c>theCopyGeom</c>).
/// </summary>
/// <remarks>
/// Without <c>theCopyGeom</c> the copy shares the original's curves, surfaces and meshes, which are frozen: that is
/// the structural copy of <see cref="BRepTools.CopyForWrite"/> (docs/phase3-design.md, section 4.6).
/// </remarks>
public class BRepTools_CopyModification : BRepTools_Modification
{
    private readonly bool myCopyGeom;
    private readonly bool myCopyMesh;

    /// <summary>A copy modification; <paramref name="theCopyGeom"/> copies the geometry, <paramref name="theCopyMesh"/> the triangulations.</summary>
    public BRepTools_CopyModification(bool theCopyGeom = true, bool theCopyMesh = true)
    {
        myCopyGeom = theCopyGeom;
        myCopyMesh = theCopyMesh;
    }

    /// <summary>The face's surface, location and tolerance (the surface copied if the geometry is copied); always true.</summary>
    public override bool NewSurface(in TopoDS_Face theFace,
                                    out Geom_Surface? theSurf,
                                    out TopLoc_Location theLoc,
                                    out double theTol,
                                    out bool theRevWires,
                                    out bool theRevFace)
    {
        theSurf = BRep_Tool.Surface(theFace, out theLoc);
        theTol = BRep_Tool.Tolerance(theFace);
        theRevWires = theRevFace = false;

        if (theSurf is not null && myCopyGeom)
        {
            theSurf = (Geom_Surface)theSurf.Copy();
        }

        return true;
    }

    /// <summary>The face's triangulation (copied if the geometry is copied), if the mesh is copied or the face has no surface.</summary>
    public override bool NewTriangulation(in TopoDS_Face theFace, out Poly_Triangulation? theTri)
    {
        theTri = null;
        if (!myCopyMesh && BRep_Tool.IsGeometric(theFace))
        {
            return false;
        }

        theTri = BRep_Tool.Triangulation(theFace, out _);

        if (theTri is null)
        {
            return false;
        }

        // mesh is copied if and only if the geometry need to be copied too
        if (myCopyGeom)
        {
            theTri = theTri.Copy();
        }
        return true;
    }

    /// <summary>The edge's 3D curve, location and tolerance (the curve copied if the geometry is copied); always true.</summary>
    public override bool NewCurve(in TopoDS_Edge theEdge, out Geom_Curve? theCurve, out TopLoc_Location theLoc, out double theTol)
    {
        theCurve = BRep_Tool.Curve(theEdge, out theLoc, out _, out _);
        theTol = BRep_Tool.Tolerance(theEdge);

        if (theCurve is not null && myCopyGeom)
        {
            theCurve = (Geom_Curve)theCurve.Copy();
        }

        return true;
    }

    /// <summary>The edge's 3D polygon (copied if the geometry is copied), if the mesh is copied or the edge has no curve.</summary>
    public override bool NewPolygon(in TopoDS_Edge theEdge, out Poly_Polygon3D? thePoly)
    {
        thePoly = null;
        if (!myCopyMesh && BRep_Tool.IsGeometric(theEdge))
        {
            return false;
        }

        thePoly = BRep_Tool.Polygon3D(theEdge, out _);

        if (thePoly is null)
        {
            return false;
        }

        // polygon is copied if and only if the geometry need to be copied too
        if (myCopyGeom)
        {
            thePoly = thePoly.Copy();
        }
        return true;
    }

    /// <summary>
    /// The edge's polygon on the face's triangulation (copied if the geometry is copied), if the mesh is copied or the
    /// edge has no curve.
    /// </summary>
    public override bool NewPolygonOnTriangulation(in TopoDS_Edge theEdge, in TopoDS_Face theFace, out Poly_PolygonOnTriangulation? thePoly)
    {
        thePoly = null;
        if (!myCopyMesh && BRep_Tool.IsGeometric(theEdge))
        {
            return false;
        }

        Poly_Triangulation? aTria = BRep_Tool.Triangulation(theFace, out TopLoc_Location aLoc);
        thePoly = BRep_Tool.PolygonOnTriangulation(theEdge, aTria, aLoc);

        if (thePoly is null)
        {
            return false;
        }

        // polygon is copied if and only if the geometry need to be copied too
        if (myCopyGeom)
        {
            thePoly = thePoly.Copy();
        }
        return true;
    }

    /// <summary>The vertex's point and tolerance; always true.</summary>
    public override bool NewPoint(in TopoDS_Vertex theVertex, out gp_Pnt thePnt, out double theTol)
    {
        thePnt = BRep_Tool.Pnt(theVertex);
        theTol = BRep_Tool.Tolerance(theVertex);
        return true;
    }

    /// <summary>The edge's pcurve on the face (copied if the geometry is copied) and its tolerance; always true.</summary>
    public override bool NewCurve2d(in TopoDS_Edge theEdge,
                                    in TopoDS_Face theFace,
                                    in TopoDS_Edge theNewEdge,
                                    in TopoDS_Face theNewFace,
                                    out Geom2d_Curve? theCurve,
                                    out double theTol)
    {
        theTol = BRep_Tool.Tolerance(theEdge);
        theCurve = BRep_Tool.CurveOnSurface(theEdge, theFace, out _, out _);

        if (theCurve is not null && myCopyGeom)
        {
            theCurve = (Geom2d_Curve)theCurve.Copy();
        }

        return true;
    }

    /// <summary>The vertex's parameter on the edge and its tolerance; false for a null vertex.</summary>
    public override bool NewParameter(in TopoDS_Vertex theVertex, in TopoDS_Edge theEdge, ref double thePnt, ref double theTol)
    {
        if (theVertex.IsNull())
        {
            return false; // infinite edge may have Null vertex
        }

        theTol = BRep_Tool.Tolerance(theVertex);
        thePnt = BRep_Tool.Parameter(theVertex, theEdge);

        return true;
    }

    /// <summary>The edge's continuity between the original faces.</summary>
    public override GeomAbs_Shape Continuity(in TopoDS_Edge theEdge,
                                             in TopoDS_Face theFace1,
                                             in TopoDS_Face theFace2,
                                             in TopoDS_Edge theNewEdge,
                                             in TopoDS_Face theNewFace1,
                                             in TopoDS_Face theNewFace2)
    {
        return BRep_Tool.Continuity(theEdge, theFace1, theFace2);
    }
}
