// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ShapeTool.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ShapeTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>The mesher's tools reading data from shapes and storing meshes into them.</summary>
/// <remarks>
/// <para>
/// OCCT's class is a <c>Standard_Transient</c> with only statics; the port's is a static class.
/// </para>
/// <para>
/// <b>Writing into shapes.</b> Only this class attaches meshes. <see cref="AddInFace"/>, <see cref="NullifyFace"/>,
/// <see cref="NullifyEdge(in TopoDS_Edge, Poly_Triangulation?, in TopLoc_Location)"/> and the <c>UpdateEdge</c>
/// overloads write into the face or edge they are given, as OCCT's do, through BRep_Builder's internal non-freezing
/// forms (decision 488): the mesher's triangulations and polygons stay unfrozen, so its post-processor can set each
/// triangulation's deflection and parameters in place, and they freeze with the result when it is published
/// (<c>FreezeGraph</c>). On a frozen face or edge they throw <see cref="Standard_ImmutableObject"/>: the mesher writes
/// only into an unfrozen shape, or into an exact copy of a frozen one (decision 489).
/// </para>
/// </remarks>
public static class BRepMesh_ShapeTool
{
    /// <summary>The maximum tolerance of the face, its edges and its vertices.</summary>
    public static double MaxFaceTolerance(in TopoDS_Face theFace)
    {
        double aMaxTolerance = BRep_Tool.Tolerance(theFace);

        double aTolerance = Max(MaxTolerance(theFace, TopAbs_EDGE),
                                MaxTolerance(theFace, TopAbs_VERTEX));

        return Max(aMaxTolerance, aTolerance);
    }

    /// <summary>
    /// Sets <paramref name="theMaxDimension"/> to the box's maximum dimension; leaves it unchanged for a void box.
    /// </summary>
    public static void BoxMaxDimension(in Bnd_Box theBox, ref double theMaxDimension)
    {
        if (theBox.IsVoid())
        {
            return;
        }

        theBox.Get(out double aMinX, out double aMinY, out double aMinZ, out double aMaxX, out double aMaxY, out double aMaxZ);

        theMaxDimension = Max(aMaxX - aMinX, Max(aMaxY - aMinY, aMaxZ - aMinZ));
    }

    /// <summary>
    /// Checks the SameParameter, SameRange and Degenerated flags of the discrete edge against the edge's geometry and
    /// updates the model: SameParameter and SameRange can only go from true to false, Degenerated only from false to
    /// true.
    /// </summary>
    public static void CheckAndUpdateFlags(IMeshData_Edge theEdge, IMeshData_PCurve thePCurve)
    {
        if (!theEdge.GetSameParam() && !theEdge.GetSameRange() && theEdge.GetDegenerated())
        {
            // Nothing to do worse.
            return;
        }

        TopoDS_Edge aEdge = theEdge.GetEdge();
        TopoDS_Face aFace = thePCurve.GetFace().GetFace();

        Range(aEdge, out Geom_Curve? aCurve, out double aFirstParam, out double aLastParam);
        if (aCurve is null)
        {
            theEdge.SetDegenerated(true);
            return;
        }

        BRepAdaptor_Curve aCurveOnSurf = new(aEdge, aFace);
        if (theEdge.GetSameParam() || theEdge.GetSameRange())
        {
            if (theEdge.GetSameRange())
            {
                double aDiffFirst = aCurveOnSurf.FirstParameter() - aFirstParam;
                double aDiffLast = aCurveOnSurf.LastParameter() - aLastParam;
                theEdge.SetSameRange(Math.Abs(aDiffFirst) < Precision.PConfusion()
                                     && Math.Abs(aDiffLast) < Precision.PConfusion());

                if (!theEdge.GetSameRange())
                {
                    theEdge.SetSameParam(false);
                }
            }
        }

        if (!theEdge.GetDegenerated() /* || theEdge->GetSameParam ()*/)
        {
            TopExp.Vertices(aEdge, out TopoDS_Vertex aStartVertex, out TopoDS_Vertex aEndVertex);
            if (aStartVertex.IsNull() || aEndVertex.IsNull())
            {
                theEdge.SetDegenerated(true);
                return;
            }

            if (aStartVertex.IsSame(aEndVertex))
            {
                const int aPointsNb = 20;
                double aVertexTolerance = BRep_Tool.Tolerance(aStartVertex);
                double aDu = (aLastParam - aFirstParam) / aPointsNb;

                aCurve.D0(aFirstParam, out gp_Pnt aPrevPnt);

                double aLength = 0.0;
                for (int i = 1; i <= aPointsNb; ++i)
                {
                    double aParameter = aFirstParam + i * aDu;
                    // Calculation of the length of the edge in 3D
                    // in order to check degenerativity
                    aCurve.D0(aParameter, out gp_Pnt aPnt);
                    aLength += aPrevPnt.Distance(aPnt);

                    if (aLength > aVertexTolerance /*&& !theEdge->GetSameParam()*/)
                    {
                        break;
                    }

                    aPrevPnt = aPnt;
                }

                theEdge.SetDegenerated(aLength < aVertexTolerance);
            }
        }
    }

    /// <summary>
    /// Stores the triangulation into the face, after moving its nodes by the inverse of the face's location.
    /// </summary>
    /// <remarks>
    /// Writes the nodes of <paramref name="theTriangulation"/> (unfrozen: the mesher's own) and attaches it to the face
    /// unfrozen (decision 488). OCCT passes the handle by reference without changing it; so does the port.
    /// </remarks>
    public static void AddInFace(in TopoDS_Face theFace, Poly_Triangulation theTriangulation)
    {
        TopLoc_Location aLoc = theFace.Location();
        if (!aLoc.IsIdentity())
        {
            gp_Trsf aTrsf = aLoc.Transformation();
            aTrsf.Invert();
            for (int aNodeIter = 1; aNodeIter <= theTriangulation.NbNodes(); ++aNodeIter)
            {
                gp_Pnt aNode = theTriangulation.Node(aNodeIter);
                aNode.Transform(aTrsf);
                theTriangulation.SetNode(aNodeIter, aNode);
            }
        }

        BRep_Builder aBuilder = new();
        aBuilder.UpdateFaceUnfrozen(theFace, theTriangulation); // OCCT.NET: the mesher's non-freezing attach (decision 488)
    }

    /// <summary>Removes the face's triangulation.</summary>
    public static void NullifyFace(in TopoDS_Face theFace)
    {
        BRep_Builder aBuilder = new();
        aBuilder.UpdateFaceUnfrozen(theFace, null);
    }

    /// <summary>Removes the edge's polygon on the triangulation of the face with the given location.</summary>
    public static void NullifyEdge(in TopoDS_Edge theEdge, Poly_Triangulation? theTriangulation, in TopLoc_Location theLocation)
    {
        UpdateEdge(theEdge, (Poly_PolygonOnTriangulation?)null, theTriangulation, theLocation);
    }

    /// <summary>Removes the edge's 3D polygon.</summary>
    public static void NullifyEdge(in TopoDS_Edge theEdge, in TopLoc_Location theLocation)
    {
        BRep_Builder aBuilder = new();
        aBuilder.UpdateEdgeUnfrozen(theEdge, (Poly_Polygon3D?)null, theLocation);
    }

    /// <summary>Stores the polygon on the triangulation of the face with the given location into the edge.</summary>
    public static void UpdateEdge(in TopoDS_Edge theEdge,
                                  Poly_PolygonOnTriangulation? thePolygon,
                                  Poly_Triangulation? theTriangulation,
                                  in TopLoc_Location theLocation)
    {
        BRep_Builder aBuilder = new();
        aBuilder.UpdateEdgeUnfrozen(theEdge, thePolygon, theTriangulation, theLocation);
    }

    /// <summary>
    /// Stores the polygons of the seam edge (for its forward and its reversed orientation) on the triangulation of the
    /// face with the given location into the edge.
    /// </summary>
    public static void UpdateEdge(in TopoDS_Edge theEdge,
                                  Poly_PolygonOnTriangulation? thePolygon1,
                                  Poly_PolygonOnTriangulation? thePolygon2,
                                  Poly_Triangulation? theTriangulation,
                                  in TopLoc_Location theLocation)
    {
        BRep_Builder aBuilder = new();
        aBuilder.UpdateEdgeUnfrozen(theEdge, thePolygon1, thePolygon2, theTriangulation, theLocation);
    }

    /// <summary>Stores the 3D polygon into the edge.</summary>
    public static void UpdateEdge(in TopoDS_Edge theEdge, Poly_Polygon3D? thePolygon)
    {
        BRep_Builder aBuilder = new();
        aBuilder.UpdateEdgeUnfrozen(theEdge, thePolygon, new TopLoc_Location());
    }

    /// <summary>The point moved by the location.</summary>
    public static gp_Pnt UseLocation(in gp_Pnt thePnt, in TopLoc_Location theLoc)
    {
        if (theLoc.IsIdentity())
        {
            return thePnt;
        }

        return thePnt.Transformed(theLoc.Transformation());
    }

    /// <summary>The UV points of the edge's ends on its pcurve on the face.</summary>
    public static bool UVPoints(in TopoDS_Edge theEdge,
                                in TopoDS_Face theFace,
                                out gp_Pnt2d theFirstPoint2d,
                                out gp_Pnt2d theLastPoint2d,
                                bool isConsiderOrientation = false)
    {
        theFirstPoint2d = new gp_Pnt2d();
        theLastPoint2d = new gp_Pnt2d();
        if (!Range(theEdge, theFace, out Geom2d_Curve? aCurve2d, out double aFirstParam, out double aLastParam,
                   isConsiderOrientation))
        {
            return false;
        }

        aCurve2d!.D0(aFirstParam, out theFirstPoint2d);
        aCurve2d.D0(aLastParam, out theLastPoint2d);
        return true;
    }

    /// <summary>The pcurve of the edge on the face and its range (ShapeAnalysis_Edge::PCurve).</summary>
    public static bool Range(in TopoDS_Edge theEdge,
                             in TopoDS_Face theFace,
                             out Geom2d_Curve? thePCurve,
                             out double theFirstParam,
                             out double theLastParam,
                             bool isConsiderOrientation = false)
    {
        ShapeAnalysis_Edge aSAEdge = new();
        return aSAEdge.PCurve(theEdge, theFace, out thePCurve, out theFirstParam, out theLastParam,
                              isConsiderOrientation);
    }

    /// <summary>The 3D curve of the edge, located, and its range (ShapeAnalysis_Edge::Curve3d).</summary>
    public static bool Range(in TopoDS_Edge theEdge,
                             out Geom_Curve? theCurve,
                             out double theFirstParam,
                             out double theLastParam,
                             bool isConsiderOrientation = false)
    {
        ShapeAnalysis_Edge aSAEdge = new();
        return aSAEdge.Curve3d(theEdge, out theCurve, out theFirstParam, out theLastParam, isConsiderOrientation);
    }

    // Returns maximum tolerance of face element of the specified type (OCCT's template over the shape type and the
    // tolerance extractor).
    private static double MaxTolerance(in TopoDS_Face theFace, TopAbs_ShapeEnum theShapeType)
    {
        double aMaxTolerance = RealFirst();
        TopExp_Explorer aExplorer = new(theFace, theShapeType);
        for (; aExplorer.More(); aExplorer.Next())
        {
            double aTolerance = theShapeType == TopAbs_EDGE
                                    ? BRep_Tool.Tolerance(TopoDS.Edge(aExplorer.Current()))
                                    : BRep_Tool.Tolerance(TopoDS.Vertex(aExplorer.Current()));
            if (aTolerance > aMaxTolerance)
            {
                aMaxTolerance = aTolerance;
            }
        }

        return aMaxTolerance;
    }
}
