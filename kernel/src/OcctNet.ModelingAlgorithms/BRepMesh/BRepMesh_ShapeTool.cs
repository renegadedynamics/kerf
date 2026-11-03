// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ShapeTool.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ShapeTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>The mesher's tools reading data from shapes and storing meshes into them.</summary>
/// <remarks>
/// <para>
/// OCCT's class is a <c>Standard_Transient</c> with only statics; the port's is a static class.
/// </para>
/// <para>
/// <b>Writing into shapes.</b> Only this class attaches meshes. <see cref="AddInFace"/>, <see cref="NullifyFace"/>,
/// <see cref="NullifyEdge(in Edge, Triangulation?, in Placement)"/> and the <c>UpdateEdge</c>
/// overloads write into the face or edge they are given, as OCCT's do, through BRep_Builder's internal non-freezing
/// forms (decision 488): the mesher's triangulations and polygons stay unfrozen, so its post-processor can set each
/// triangulation's deflection and parameters in place, and they freeze with the result when it is published
/// (<c>FreezeGraph</c>). On a frozen face or edge they throw <see cref="ImmutableObjectException"/>: the mesher writes
/// only into an unfrozen shape, or into an exact copy of a frozen one (decision 489).
/// </para>
/// </remarks>
public static class BRepMesh_ShapeTool
{
    /// <summary>The maximum tolerance of the face, its edges and its vertices.</summary>
    public static double MaxFaceTolerance(in Face theFace)
    {
        double aMaxTolerance = Tool.Tolerance(theFace);

        double aTolerance = Max(MaxTolerance(theFace, ShapeType.Edge),
                                MaxTolerance(theFace, ShapeType.Vertex));

        return Max(aMaxTolerance, aTolerance);
    }

    /// <summary>
    /// Sets <paramref name="theMaxDimension"/> to the box's maximum dimension; leaves it unchanged for a void box.
    /// </summary>
    public static void BoxMaxDimension(in Box theBox, ref double theMaxDimension)
    {
        if (theBox.IsVoid)
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

        Edge aEdge = theEdge.GetEdge();
        Face aFace = thePCurve.GetFace().GetFace();

        Range(aEdge, out Curve3D? aCurve, out double aFirstParam, out double aLastParam);
        if (aCurve is null)
        {
            theEdge.SetDegenerated(true);
            return;
        }

        EdgeCurveAdaptor aCurveOnSurf = new(aEdge, aFace);
        if (theEdge.GetSameParam() || theEdge.GetSameRange())
        {
            if (theEdge.GetSameRange())
            {
                double aDiffFirst = aCurveOnSurf.FirstParameter() - aFirstParam;
                double aDiffLast = aCurveOnSurf.LastParameter() - aLastParam;
                theEdge.SetSameRange(Math.Abs(aDiffFirst) < global::Kerf.Engine.Foundation.Precision.PConfusion()
                                     && Math.Abs(aDiffLast) < global::Kerf.Engine.Foundation.Precision.PConfusion());

                if (!theEdge.GetSameRange())
                {
                    theEdge.SetSameParam(false);
                }
            }
        }

        if (!theEdge.GetDegenerated() /* || theEdge->GetSameParam ()*/)
        {
            Exploration.Vertices(aEdge, out Vertex aStartVertex, out Vertex aEndVertex);
            if (aStartVertex.IsNull || aEndVertex.IsNull)
            {
                theEdge.SetDegenerated(true);
                return;
            }

            if (aStartVertex.IsSame(aEndVertex))
            {
                const int aPointsNb = 20;
                double aVertexTolerance = Tool.Tolerance(aStartVertex);
                double aDu = (aLastParam - aFirstParam) / aPointsNb;

                aCurve.D0(aFirstParam, out Point3D aPrevPnt);

                double aLength = 0.0;
                for (int i = 1; i <= aPointsNb; ++i)
                {
                    double aParameter = aFirstParam + i * aDu;
                    // Calculation of the length of the edge in 3D
                    // in order to check degenerativity
                    aCurve.D0(aParameter, out Point3D aPnt);
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
    public static void AddInFace(in Face theFace, Triangulation theTriangulation)
    {
        Placement aLoc = theFace.Location();
        if (!aLoc.IsIdentity)
        {
            Transform3D aTrsf = aLoc.Transformation();
            aTrsf.Invert();
            for (int aNodeIter = 1; aNodeIter <= theTriangulation.NodeCount; ++aNodeIter)
            {
                Point3D aNode = theTriangulation.Node(aNodeIter);
                aNode.Transform(aTrsf);
                theTriangulation.SetNode(aNodeIter, aNode);
            }
        }

        BrepBuilder aBuilder = new();
        aBuilder.UpdateFaceUnfrozen(theFace, theTriangulation); // OCCT.NET: the mesher's non-freezing attach (decision 488)
    }

    /// <summary>Removes the face's triangulation.</summary>
    public static void NullifyFace(in Face theFace)
    {
        BrepBuilder aBuilder = new();
        aBuilder.UpdateFaceUnfrozen(theFace, null);
    }

    /// <summary>Removes the edge's polygon on the triangulation of the face with the given location.</summary>
    public static void NullifyEdge(in Edge theEdge, Triangulation? theTriangulation, in Placement theLocation)
    {
        UpdateEdge(theEdge, (PolygonOnTriangulation?)null, theTriangulation, theLocation);
    }

    /// <summary>Removes the edge's 3D polygon.</summary>
    public static void NullifyEdge(in Edge theEdge, in Placement theLocation)
    {
        BrepBuilder aBuilder = new();
        aBuilder.UpdateEdgeUnfrozen(theEdge, (Polygon3D?)null, theLocation);
    }

    /// <summary>Stores the polygon on the triangulation of the face with the given location into the edge.</summary>
    public static void UpdateEdge(in Edge theEdge,
                                  PolygonOnTriangulation? thePolygon,
                                  Triangulation? theTriangulation,
                                  in Placement theLocation)
    {
        BrepBuilder aBuilder = new();
        aBuilder.UpdateEdgeUnfrozen(theEdge, thePolygon, theTriangulation, theLocation);
    }

    /// <summary>
    /// Stores the polygons of the seam edge (for its forward and its reversed orientation) on the triangulation of the
    /// face with the given location into the edge.
    /// </summary>
    public static void UpdateEdge(in Edge theEdge,
                                  PolygonOnTriangulation? thePolygon1,
                                  PolygonOnTriangulation? thePolygon2,
                                  Triangulation? theTriangulation,
                                  in Placement theLocation)
    {
        BrepBuilder aBuilder = new();
        aBuilder.UpdateEdgeUnfrozen(theEdge, thePolygon1, thePolygon2, theTriangulation, theLocation);
    }

    /// <summary>Stores the 3D polygon into the edge.</summary>
    public static void UpdateEdge(in Edge theEdge, Polygon3D? thePolygon)
    {
        BrepBuilder aBuilder = new();
        aBuilder.UpdateEdgeUnfrozen(theEdge, thePolygon, new Placement());
    }

    /// <summary>The point moved by the location.</summary>
    public static Point3D UseLocation(in Point3D thePnt, in Placement theLoc)
    {
        if (theLoc.IsIdentity)
        {
            return thePnt;
        }

        return thePnt.Transformed(theLoc.Transformation());
    }

    /// <summary>The UV points of the edge's ends on its pcurve on the face.</summary>
    public static bool UVPoints(in Edge theEdge,
                                in Face theFace,
                                out Point2D theFirstPoint2d,
                                out Point2D theLastPoint2d,
                                bool isConsiderOrientation = false)
    {
        theFirstPoint2d = new Point2D();
        theLastPoint2d = new Point2D();
        if (!Range(theEdge, theFace, out Curve2D? aCurve2d, out double aFirstParam, out double aLastParam,
                   isConsiderOrientation))
        {
            return false;
        }

        aCurve2d!.D0(aFirstParam, out theFirstPoint2d);
        aCurve2d.D0(aLastParam, out theLastPoint2d);
        return true;
    }

    /// <summary>The pcurve of the edge on the face and its range (ShapeAnalysis_Edge::PCurve).</summary>
    public static bool Range(in Edge theEdge,
                             in Face theFace,
                             out Curve2D? thePCurve,
                             out double theFirstParam,
                             out double theLastParam,
                             bool isConsiderOrientation = false)
    {
        ShapeAnalysis_Edge aSAEdge = new();
        return aSAEdge.PCurve(theEdge, theFace, out thePCurve, out theFirstParam, out theLastParam,
                              isConsiderOrientation);
    }

    /// <summary>The 3D curve of the edge, located, and its range (ShapeAnalysis_Edge::Curve3d).</summary>
    public static bool Range(in Edge theEdge,
                             out Curve3D? theCurve,
                             out double theFirstParam,
                             out double theLastParam,
                             bool isConsiderOrientation = false)
    {
        ShapeAnalysis_Edge aSAEdge = new();
        return aSAEdge.Curve3d(theEdge, out theCurve, out theFirstParam, out theLastParam, isConsiderOrientation);
    }

    // Returns maximum tolerance of face element of the specified type (OCCT's template over the shape type and the
    // tolerance extractor).
    private static double MaxTolerance(in Face theFace, ShapeType theShapeType)
    {
        double aMaxTolerance = RealFirst();
        Explorer aExplorer = new(theFace, theShapeType);
        for (; aExplorer.More(); aExplorer.Next())
        {
            double aTolerance = theShapeType == ShapeType.Edge
                                    ? Tool.Tolerance(ShapeCasts.Edge(aExplorer.Current()))
                                    : Tool.Tolerance(ShapeCasts.Vertex(aExplorer.Current()));
            if (aTolerance > aMaxTolerance)
            {
                aMaxTolerance = aTolerance;
            }
        }

        return aMaxTolerance;
    }
}
