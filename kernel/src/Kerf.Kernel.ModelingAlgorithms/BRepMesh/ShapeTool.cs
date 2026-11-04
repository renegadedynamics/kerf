// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ShapeTool.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ShapeTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Meshing;
using Kerf.Engine.Meshing.Model;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Meshing;

/// <summary>The mesher's tools reading data from shapes and storing meshes into them.</summary>
/// <remarks>
/// <para>
/// OCCT's class is a <c>Standard_Transient</c> with only statics; the port's is a static class.
/// </para>
/// <para>
/// <b>Writing into shapes.</b> Only this class attaches meshes. <see cref="AddInFace(in Face, Triangulation)"/>,
/// <see cref="NullifyFace"/>, <see cref="NullifyEdge(in Edge, Triangulation?, in Placement)"/> and the
/// <c>UpdateEdge</c> overloads write into the face or edge they are given, as OCCT's do, through BRep_Builder's
/// ordinary updates, which freeze the triangulations and polygons they attach (design 4.1). The mesher's
/// triangulations are finished before they are attached (<see cref="AddInFace(FaceModel, Triangulation)"/>; unit 10L5
/// retired decision 488's unfrozen attach). The mesher
/// writes into an unfrozen shape in place, and into a frozen one in a working overlay (unit 10H6, decision 489);
/// outside one, a frozen face or edge throws <see cref="LockedShapeException"/>.
/// </para>
/// </remarks>
internal static class ShapeTool
{
    /// <summary>The maximum tolerance of the face, its edges and its vertices.</summary>
    public static double MaxFaceTolerance(in Face face)
    {
        double maxTolerance = Tool.Tolerance(face);

        double tolerance = Max(MaxTolerance(face, ShapeType.Edge),
                                MaxTolerance(face, ShapeType.Vertex));

        return Max(maxTolerance, tolerance);
    }

    /// <summary>
    /// Sets <paramref name="maxDimension"/> to the box's maximum dimension; leaves it unchanged for a void box.
    /// </summary>
    public static void BoxMaxDimension(in Box box, ref double maxDimension)
    {
        if (box.IsVoid)
        {
            return;
        }

        box.Get(out double minX, out double minY, out double minZ, out double maxX, out double maxY, out double maxZ);

        maxDimension = Max(maxX - minX, Max(maxY - minY, maxZ - minZ));
    }

    /// <summary>
    /// Checks the SameParameter, SameRange and Degenerated flags of the discrete edge against the edge's geometry and
    /// updates the model: SameParameter and SameRange can only go from true to false, Degenerated only from false to
    /// true.
    /// </summary>
    public static void CheckAndUpdateFlags(EdgeModel edge, PCurveModel pCurve)
    {
        if (!edge.GetSameParam() && !edge.GetSameRange() && edge.GetDegenerated())
        {
            // Nothing to do worse.
            return;
        }

        Edge aEdge = edge.GetEdge();
        Face face = pCurve.GetFace().GetFace();

        Range(aEdge, out Curve3D? curve, out double firstParam, out double lastParam);
        if (curve is null)
        {
            edge.SetDegenerated(true);
            return;
        }

        EdgeCurveAdaptor curveOnSurf = new(aEdge, face);
        if (edge.GetSameParam() || edge.GetSameRange())
        {
            if (edge.GetSameRange())
            {
                double diffFirst = curveOnSurf.FirstParameter() - firstParam;
                double diffLast = curveOnSurf.LastParameter() - lastParam;
                edge.SetSameRange(Math.Abs(diffFirst) < global::Kerf.Engine.Foundation.Precision.PConfusion()
                                     && Math.Abs(diffLast) < global::Kerf.Engine.Foundation.Precision.PConfusion());

                if (!edge.GetSameRange())
                {
                    edge.SetSameParam(false);
                }
            }
        }

        if (!edge.GetDegenerated() /* || theEdge->GetSameParam ()*/)
        {
            Exploration.Vertices(aEdge, out Vertex startVertex, out Vertex endVertex);
            if (startVertex.IsNull || endVertex.IsNull)
            {
                edge.SetDegenerated(true);
                return;
            }

            if (startVertex.IsSame(endVertex))
            {
                const int pointsNb = 20;
                double vertexTolerance = Tool.Tolerance(startVertex);
                double du = (lastParam - firstParam) / pointsNb;

                curve.D0(firstParam, out Point3D prevPnt);

                double length = 0.0;
                for (int i = 1; i <= pointsNb; ++i)
                {
                    double parameter = firstParam + i * du;
                    // Calculation of the length of the edge in 3D
                    // in order to check degenerativity
                    curve.D0(parameter, out Point3D pnt);
                    length += prevPnt.DistanceTo(pnt);

                    if (length > vertexTolerance /*&& !theEdge->GetSameParam()*/)
                    {
                        break;
                    }

                    prevPnt = pnt;
                }

                edge.SetDegenerated(length < vertexTolerance);
            }
        }
    }

    /// <summary>
    /// Stores the triangulation into the face, after moving its nodes by the inverse of the face's location.
    /// </summary>
    /// <remarks>
    /// Writes the nodes of <paramref name="triangulation"/> (unfrozen: the caller's own) and attaches it frozen, as all
    /// geometry is attached (design 4.1). The mesher itself doesn't attach here: it hands its triangulations to the
    /// post-processor (<see cref="AddInFace(FaceModel, Triangulation)"/>), which attaches each once finished (unit
    /// 10L5). OCCT passes the handle by reference without changing it; so does the port.
    /// </remarks>
    public static void AddInFace(in Face face, Triangulation triangulation)
    {
        ToFaceDefinition(face, triangulation);
        AttachToFace(face, triangulation);
    }

    /// <summary>
    /// Kerf: the mesher's <see cref="AddInFace(in Face, Triangulation)"/>. Moves the nodes into the face's definition
    /// and leaves the triangulation with the face's model, for the post-processor to finish (its deflection and the
    /// parameters it was made with) and attach (unit 10L5): OCCT attaches it here and finishes it attached.
    /// </summary>
    public static void AddInFace(FaceModel face, Triangulation triangulation)
    {
        ToFaceDefinition(face.GetFace(), triangulation);
        face.MadeTriangulation = triangulation;
    }

    /// <summary>Kerf: attaches a finished triangulation, already in the face's definition, to the face (frozen).</summary>
    public static void AttachToFace(in Face face, Triangulation triangulation)
    {
        BrepBuilder builder = new();
        builder.UpdateFace(face, triangulation);
    }

    // The nodes moved by the inverse of the face's location
    private static void ToFaceDefinition(in Face face, Triangulation triangulation)
    {
        Placement loc = face.Placement;
        if (!loc.IsIdentity)
        {
            Transform3D trsf = loc.Transformation;
            trsf = trsf.Inverted();
            for (int nodeIter = 1; nodeIter <= triangulation.NodeCount; ++nodeIter)
            {
                Point3D node = triangulation.Node(nodeIter);
                node = node.Transformed(trsf);
                triangulation.SetNode(nodeIter, node);
            }
        }
    }

    /// <summary>Removes the face's triangulation.</summary>
    public static void NullifyFace(in Face face)
    {
        BrepBuilder builder = new();
        builder.UpdateFace(face, (Triangulation?)null);
    }

    /// <summary>Removes the edge's polygon on the triangulation of the face with the given location.</summary>
    public static void NullifyEdge(in Edge edge, Triangulation? triangulation, in Placement location)
    {
        UpdateEdge(edge, (PolygonOnTriangulation?)null, triangulation, location);
    }

    /// <summary>Removes the edge's 3D polygon.</summary>
    public static void NullifyEdge(in Edge edge, in Placement location)
    {
        BrepBuilder builder = new();
        builder.UpdateEdge(edge, (Polygon3D?)null, location);
    }

    /// <summary>Stores the polygon on the triangulation of the face with the given location into the edge.</summary>
    public static void UpdateEdge(in Edge edge,
                                  PolygonOnTriangulation? polygon,
                                  Triangulation? triangulation,
                                  in Placement location)
    {
        BrepBuilder builder = new();
        builder.UpdateEdge(edge, polygon, triangulation, location);
    }

    /// <summary>
    /// Stores the polygons of the seam edge (for its forward and its reversed orientation) on the triangulation of the
    /// face with the given location into the edge.
    /// </summary>
    public static void UpdateEdge(in Edge edge,
                                  PolygonOnTriangulation? polygon1,
                                  PolygonOnTriangulation? polygon2,
                                  Triangulation? triangulation,
                                  in Placement location)
    {
        BrepBuilder builder = new();
        builder.UpdateEdge(edge, polygon1, polygon2, triangulation, location);
    }

    /// <summary>Stores the 3D polygon into the edge.</summary>
    public static void UpdateEdge(in Edge edge, Polygon3D? polygon)
    {
        BrepBuilder builder = new();
        builder.UpdateEdge(edge, polygon, new Placement());
    }

    /// <summary>The point moved by the location.</summary>
    public static Point3D UseLocation(in Point3D pnt, in Placement loc)
    {
        if (loc.IsIdentity)
        {
            return pnt;
        }

        return pnt.Transformed(loc.Transformation);
    }

    /// <summary>The UV points of the edge's ends on its pcurve on the face.</summary>
    public static bool UVPoints(in Edge edge,
                                in Face face,
                                out Point2D firstPoint2d,
                                out Point2D lastPoint2d,
                                bool isConsiderOrientation = false)
    {
        firstPoint2d = new Point2D();
        lastPoint2d = new Point2D();
        if (!Range(edge, face, out Curve2D? curve2d, out double firstParam, out double lastParam,
                   isConsiderOrientation))
        {
            return false;
        }

        curve2d!.D0(firstParam, out firstPoint2d);
        curve2d.D0(lastParam, out lastPoint2d);
        return true;
    }

    /// <summary>The pcurve of the edge on the face and its range (ShapeAnalysis_Edge::PCurve).</summary>
    public static bool Range(in Edge edge,
                             in Face face,
                             out Curve2D? pCurve,
                             out double firstParam,
                             out double lastParam,
                             bool isConsiderOrientation = false)
    {
        EdgeAnalysis saEdge = new();
        return saEdge.PCurve(edge, face, out pCurve, out firstParam, out lastParam,
                              isConsiderOrientation);
    }

    /// <summary>The 3D curve of the edge, located, and its range (ShapeAnalysis_Edge::Curve3d).</summary>
    public static bool Range(in Edge edge,
                             out Curve3D? curve,
                             out double firstParam,
                             out double lastParam,
                             bool isConsiderOrientation = false)
    {
        EdgeAnalysis saEdge = new();
        return saEdge.Curve3d(edge, out curve, out firstParam, out lastParam, isConsiderOrientation);
    }

    // Returns maximum tolerance of face element of the specified type (OCCT's template over the shape type and the
    // tolerance extractor).
    private static double MaxTolerance(in Face face, ShapeType shapeType)
    {
        double maxTolerance = RealFirst();
        Explorer explorer = new(face, shapeType);
        for (; explorer.More(); explorer.Next())
        {
            double tolerance = shapeType == ShapeType.Edge
                                    ? Tool.Tolerance(ShapeCasts.Edge(explorer.Current()))
                                    : Tool.Tolerance(ShapeCasts.Vertex(explorer.Current()));
            if (tolerance > maxTolerance)
            {
                maxTolerance = tolerance;
            }
        }

        return maxTolerance;
    }
}
