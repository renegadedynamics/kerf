// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_SimpleOffset.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_SimpleOffset.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Offsets;

/// <summary>
/// The modification of the simple offset algorithm: the topology-preserving offset, without intersection.
/// </summary>
/// <remarks>
/// <para>The mapping scheme:</para>
/// <list type="bullet">
/// <item><description>each surface is mapped to its geometric offset surface;</description></item>
/// <item><description>for each edge, the pcurves are mapped to the same pcurves on the offset surfaces;</description></item>
/// <item><description>for each edge, the 3D curve is rebuilt from its pcurve on the first offset face;</description></item>
/// <item><description>each vertex is placed at the average of the ends of the offset edges sharing it;</description></item>
/// <item><description>the tolerances are updated to the resulting geometry.</description></item>
/// </list>
/// <para>
/// It reads the input shape only: everything is computed in the constructor, on new edges of its own. The maps are
/// keyed by the shapes with their orientations (OCCT's default hasher), so a shared edge has an image only in the
/// orientation in which it was first met.
/// </para>
/// </remarks>
internal sealed class SimpleOffset : Modification
{
    private struct NewFaceData
    {
        public Surface? OffsetS;
        public Placement L;
        public double myTol;
        public bool myRevWires;
        public bool myRevFace;
    }

    private struct NewEdgeData
    {
        public Curve3D? OffsetC; // Resulting curve.
        public Placement L;
        public double myTol;
    }

    private struct NewVertexData
    {
        public Point3D myP;
        public double myTol;
    }

    // Map of faces to new faces information.
    private readonly Dictionary<Face, NewFaceData> _faceInfo =
        new(EqualityComparer<Face>.Default); // the default hasher of OCCT (IsEqual)

    // Map of edges to new edges information.
    private readonly Dictionary<Edge, NewEdgeData> _edgeInfo =
        new(EqualityComparer<Edge>.Default); // the default hasher of OCCT (IsEqual)

    // Map of vertices to new vertices information.
    private readonly Dictionary<Vertex, NewVertexData> _vertexInfo =
        new(EqualityComparer<Vertex>.Default); // the default hasher of OCCT (IsEqual)

    // Offset value.
    private readonly double _offsetValue;

    // Tolerance.
    private readonly double _tolerance;

    //=================================================================================================

    /// <summary>
    /// Computes the offset of <paramref name="inputShape"/> by the signed distance
    /// <paramref name="offsetValue"/>; <paramref name="tolerance"/> is the tolerance for handling singular
    /// points.
    /// </summary>
    public SimpleOffset(in Shape inputShape, double offsetValue, double tolerance)
    {
        _offsetValue = offsetValue;
        _tolerance = tolerance;
        FillOffsetData(inputShape);
    }

    //=================================================================================================

    /// <summary>
    /// True if the face has been modified: <paramref name="s"/> is its offset surface, with the identity location.
    /// </summary>
    public override bool NewSurface(in Face f,
                                    out Surface? s,
                                    out Placement l,
                                    out double tol,
                                    out bool revWires,
                                    out bool revFace)
    {
        if (!_faceInfo.ContainsKey(f))
        {
            // Kerf: OCCT leaves the outputs unset
            s = null;
            l = new Placement();
            tol = 0.0;
            revWires = revFace = false;
            return false;
        }

        NewFaceData nfd = _faceInfo.GetValue(f);

        s = nfd.OffsetS;
        l = nfd.L;
        tol = nfd.myTol;
        revWires = nfd.myRevWires;
        revFace = nfd.myRevFace;

        return true;
    }

    //=================================================================================================

    /// <summary>True if the edge has been modified: <paramref name="c"/> is its new 3D curve.</summary>
    public override bool NewCurve(in Edge e, out Curve3D? c, out Placement l, out double tol)
    {
        if (!_edgeInfo.ContainsKey(e))
        {
            // Kerf: OCCT leaves the outputs unset
            c = null;
            l = new Placement();
            tol = 0.0;
            return false;
        }

        NewEdgeData ned = _edgeInfo.GetValue(e);

        c = ned.OffsetC;
        l = ned.L;
        tol = ned.myTol;

        return true;
    }

    //=================================================================================================

    /// <summary>True if the vertex has been modified: <paramref name="p"/> is its new point.</summary>
    public override bool NewPoint(in Vertex v, out Point3D p, out double tol)
    {
        if (!_vertexInfo.ContainsKey(v))
        {
            // Kerf: OCCT leaves the outputs unset
            p = new Point3D();
            tol = 0.0;
            return false;
        }

        NewVertexData nvd = _vertexInfo.GetValue(v);

        p = nvd.myP;
        tol = nvd.myTol;

        return true;
    }

    //=================================================================================================

    /// <summary>Always true: the edge keeps its pcurve on the face (on the offset surface).</summary>
    public override bool NewCurve2d(in Edge e,
                                    in Face f,
                                    in Edge newE,
                                    in Face newF,
                                    out Curve2D? c,
                                    out double tol)
    {
        // Use original pcurve.
        c = Tool.CurveOnSurface(e, f, out double aF, out double l);
        tol = Tool.Tolerance(e);

        if (_edgeInfo.ContainsKey(e))
        {
            tol = _edgeInfo.GetValue(e).myTol;
        }

        return true;
    }

    //=================================================================================================

    /// <summary>Always true: the vertex keeps its parameter on the edge.</summary>
    public override bool NewParameter(in Vertex v, in Edge e, ref double p, ref double tol)
    {
        // Use original parameter.
        p = Tool.Parameter(v, e);
        tol = Tool.Tolerance(v);

        if (_vertexInfo.ContainsKey(v))
        {
            tol = _vertexInfo.GetValue(v).myTol;
        }

        return true;
    }

    //=================================================================================================

    /// <summary>The continuity of the original edge between the original faces.</summary>
    public override Continuity Continuity(in Edge e,
                                             in Face f1,
                                             in Face f2,
                                             in Edge newE,
                                             in Face newF1,
                                             in Face newF2)
    {
        // Compute result using original continuity.
        return Tool.Continuity(e, f1, f2);
    }

    //=================================================================================================

    // Fills offset data.
    private void FillOffsetData(in Shape shape)
    {
        // Clears old data.
        _faceInfo.Clear();
        _edgeInfo.Clear();
        _vertexInfo.Clear();

        // Faces loop. Compute offset surface for each face.
        var expSF = new Explorer(shape, ShapeType.Face);
        for (; expSF.More(); expSF.Next())
        {
            Face currFace = ShapeCasts.Face(expSF.Current());
            FillFaceData(currFace);
        }

        // Iterate over edges to compute 3d curve.
        var edgeFaceMap =
            new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(shape, ShapeType.Edge, ShapeType.Face, edgeFaceMap);
        for (int idx = 1; idx <= edgeFaceMap.Count; ++idx)
        {
            Edge currEdge = ShapeCasts.Edge(edgeFaceMap.KeyAt(idx - 1));
            FillEdgeData(currEdge, edgeFaceMap, idx);
        }

        // Iterate over vertices to compute new vertex.
        var vertexEdgeMap =
            new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(shape, ShapeType.Vertex, ShapeType.Edge, vertexEdgeMap);
        for (int idx = 1; idx <= vertexEdgeMap.Count; ++idx)
        {
            Vertex currVertex = ShapeCasts.Vertex(vertexEdgeMap.KeyAt(idx - 1));
            FillVertexData(currVertex, vertexEdgeMap, idx);
        }
    }

    //=================================================================================================

    // Method to fill new face data for single face.
    private void FillFaceData(in Face face)
    {
        NewFaceData nfd = new();
        nfd.myRevWires = false;
        nfd.myRevFace = false;
        nfd.myTol = Tool.Tolerance(face);

        // Create offset surface.

        // Any existing transformation is applied to the surface.
        // New face will have null transformation.
        Surface s = Tool.Surface(face)!;
        s = global::Kerf.Engine.Offsets.BRepOffset.CollapseSingularities(s, face, _tolerance);

        // Take into account face orientation.
        double mult = 1.0;
        if (face.Orientation == Reversed)
        {
            mult = -1.0;
        }

        // aStatus is set by BRepOffset::Surface(), could be used to check result...
        nfd.OffsetS = global::Kerf.Engine.Offsets.BRepOffset.Surface(s, mult * _offsetValue, out OffsetStatus status, true);
        nfd.L = new Placement(); // Null transformation.

        // Save offset surface in map.
        _faceInfo[face] = nfd;
    }

    //=================================================================================================

    // Method to fill new edge data for single edge.
    private void FillEdgeData(in Edge edge,
                              IndexedMap<Shape, NCollection_List<Shape>> edgeFaceMap,
                              int idx)
    {
        NCollection_List<Shape> facesList = edgeFaceMap.ValueAt(idx - 1);

        if (facesList.Size() == 0)
        {
            return; // Free edges are skipped.
        }

        // Get offset surface.
        Face currFace = ShapeCasts.Face(facesList.First());

        if (!_faceInfo.ContainsKey(currFace))
        {
            return;
        }

        // No need to deal with transformation - it is applied in fill faces data method.
        NewFaceData nfd = _faceInfo.GetValue(currFace);
        Surface offsetSurf = nfd.OffsetS!;

        // Compute offset 3d curve.
        Curve2D c2d = Tool.CurveOnSurface(edge, currFace, out double f, out double l)!;

        var edgeMaker = new MakeEdge(c2d, offsetSurf, f, l);
        Edge newEdge = edgeMaker.InternalEdge();

        // Compute max tolerance. Vertex tolerance usage is taken from existing offset computation
        // algorithm. This piece of code significantly influences resulting performance.
        double tol = Tool.MaxTolerance(edge, ShapeType.Vertex);
        BrepLibrary.BuildCurves3d(newEdge, tol);

        NewEdgeData ned = new();
        ned.OffsetC = Tool.Curve(newEdge, out ned.L, out f, out l);

        // Iterate over adjacent faces for the current edge and compute max deviation.
        double edgeTol = 0.0;
        var iter = new NCollection_List<Shape>.Iterator(facesList);
        for (; ned.OffsetC is not null && iter.More(); iter.Next())
        {
            Face curFace = ShapeCasts.Face(iter.Value());

            if (!_faceInfo.ContainsKey(curFace))
            {
                continue;
            }

            // Create offset curve on surface.
            Curve2D c2dNew = Tool.CurveOnSurface(edge, curFace, out f, out l)!;
            Curve2DAdaptorBase hCurve2d = new CurveAdaptor2D(c2dNew, f, l);
            SurfaceAdaptorBase hSurface = new SurfaceAdaptor(_faceInfo.GetValue(curFace).OffsetS!);
            var curveOnSurf = new CurveOnSurface(hCurve2d, hSurface);

            // Extract 3d-curve (it is not null).
            CurveAdaptorBase curve3d = new CurveAdaptor(ned.OffsetC, f, l);

            // It is necessary to compute maximal deviation (tolerance).
            var validateEdge = new ValidateEdge(curve3d, curveOnSurf, true);
            validateEdge.Process();
            if (validateEdge.IsDone)
            {
                double maxTol1 = validateEdge.GetMaxDistance();
                edgeTol = Max(edgeTol, maxTol1);
            }
        }
        ned.myTol = Max(Tool.Tolerance(newEdge), edgeTol);

        // Save computed 3d curve in map.
        _edgeInfo[edge] = ned;
    }

    //=================================================================================================

    // Method to fill new vertex data for single vertex.
    private void FillVertexData(in Vertex vertex,
                                IndexedMap<Shape, NCollection_List<Shape>> vertexEdgeMap,
                                int idx)
    {
        // Algorithm:
        // Find adjacent edges for the given vertex.
        // Find corresponding end on the each adjacent edge.
        // Get offset points for founded end.
        // Set result vertex position as barycenter of founded points.

        Point3D currPnt = Tool.Pnt(vertex);

        NCollection_List<Shape> edgesList = vertexEdgeMap.ValueAt(idx - 1);

        if (edgesList.Size() == 0)
        {
            return; // Free verices are skipped.
        }

        // Array to store offset points.
        var offsetPointVec = new List<Point3D>();

        double maxEdgeTol = 0.0;

        // Iterate over adjacent edges.
        var iterEdges = new NCollection_List<Shape>.Iterator(edgesList);
        for (; iterEdges.More(); iterEdges.Next())
        {
            Edge currEdge = ShapeCasts.Edge(iterEdges.Value());

            if (!_edgeInfo.ContainsKey(currEdge))
            {
                continue; // Skip shared edges with wrong orientation.
            }

            // Find the closest bound.
            Curve3D? c3d = Tool.Curve(currEdge, out double f, out double l);

            // Protection from degenerated edges.
            if (c3d is null)
            {
                continue;
            }

            Point3D pntF = c3d.Value(f);
            Point3D pntL = c3d.Value(l);

            double sqDistF = pntF.SquareDistanceTo(currPnt);
            double sqDistL = pntL.SquareDistanceTo(currPnt);

            double minParam = f, maxParam = l;
            if (sqDistL < sqDistF)
            {
                // Square distance to last point is closer.
                minParam = l;
                maxParam = f;
            }

            // Compute point on offset edge.
            NewEdgeData ned = _edgeInfo.GetValue(currEdge);
            Curve3D offsetCurve = ned.OffsetC!;
            Point3D offsetPoint = offsetCurve.Value(minParam);
            offsetPointVec.Add(offsetPoint);

            // Handle situation when edge is closed.
            Exploration.Vertices(currEdge, out Vertex v1, out Vertex v2);
            if (v1.IsSame(v2))
            {
                Point3D offsetPointLast = offsetCurve.Value(maxParam);
                offsetPointVec.Add(offsetPointLast);
            }

            maxEdgeTol = Max(maxEdgeTol, ned.myTol);
        }

        // NCollection_DynamicArray starts from 0 by default.
        // It's better to use lower() and upper() in this case instead of direct indexes range.
        Point3D center = new(0.0, 0.0, 0.0);
        for (int i = 0; i <= (offsetPointVec.Count - 1); ++i)
        {
            center = center.WithXYZ(center.XYZ() + offsetPointVec[i].XYZ());
        }
        // OCCT: a vertex none of whose edges has an offset curve (only free or degenerated edges, or shared edges met
        // in the other orientation) divides by zero: its new point is NaN.
        center = center.WithXYZ(center.XYZ() / offsetPointVec.Count);

        // Compute max distance.
        double sqMaxDist = 0.0;
        for (int i = 0; i <= (offsetPointVec.Count - 1); ++i)
        {
            double sqDist = center.SquareDistanceTo(offsetPointVec[i]);
            if (sqDist > sqMaxDist)
            {
                sqMaxDist = sqDist;
            }
        }

        double resTol = Max(maxEdgeTol, Math.Sqrt(sqMaxDist));

        const double multCoeff = 1.001; // Avoid tolernace problems.
        NewVertexData nvd = new();
        nvd.myP = center;
        nvd.myTol = resTol * multCoeff;

        // Save computed vertex info.
        _vertexInfo[vertex] = nvd;
    }
}
