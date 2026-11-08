// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DelaunayDeflectionControlMeshAlgo.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;

namespace Kerf.Engine.Meshing;

/// <summary>
/// Extends the Delaunay node-insertion algorithm to control the deflection of the generated triangles from the
/// surface: triangles failing the check are split, for up to 11 passes.
/// </summary>
/// <remarks>
/// The Delabella instantiation of the OCCT template (over
/// <c>BRepMesh_CustomDelaunayBaseMeshAlgo&lt;BRepMesh_DelabellaBaseMeshAlgo&gt;</c>), written by a script from the
/// Watson generic's body with only the names changed (phase4-plan section 6, decision 202's approach).
/// </remarks>
internal class DelaunayDeflectionControlMeshAlgoOfDelabella<RangeSplitter> : DelaunayNodeInsertionMeshAlgoOfDelabella<RangeSplitter>
    where RangeSplitter : DefaultRangeSplitter, new()
{
    private double _maxSqDeflection;
    private double _sqMinSize;
    private bool _isAllDegenerated;
    // OCCT's map of the node pairs already checked, which it keeps from one pass to the next: here, a pass's own are marked
    // by their links' indices, and the map holds those of the passes before, made only when a pass inserts nodes (decision 23701)
    private HashSet<OrientedEdge>? _couplesMap;
    private LinkMarks? _passLinks;
    private NCollection_List<Point2D>? _controlNodes;
    private CircleTool? _circles;

    /// <summary>An algorithm.</summary>
    public DelaunayDeflectionControlMeshAlgoOfDelabella()
    {
        _maxSqDeflection = -1.0;
        _sqMinSize = -1.0;
        _isAllDegenerated = false;
        _circles = null;
    }

    /// <summary>Processes the generated mesh: inserts the surface nodes, then controls the deflection.</summary>
    protected override void PostProcessMesh(Delaun mesher, ProgressRange range)
    {
        using ProgressScope ps = new(range, "Post process mesh", 2);
        // Insert surface nodes.
        base.PostProcessMesh(mesher, ps.Next());
        if (!ps.More())
        {
            return;
        }

        if (GetParameters().ControlSurfaceDeflection
            && GetStructure().ElementsOfDomain().Extent() > 0)
        {
            OptimizeMesh(mesher, ps.Next());
        }
        else
        {
            ps.Next();
        }
    }

    /// <summary>Checks the deviation of the mesh from the surface and inserts nodes where it is too large.</summary>
    protected virtual void OptimizeMesh(Delaun mesher, ProgressRange range)
    {
        _sqMinSize = GetParameters().MinSize * GetParameters().MinSize;
        _couplesMap = null;
        _passLinks = new LinkMarks();
        _controlNodes = new NCollection_List<Point2D>();
        _circles = mesher.Circles();

        const int iterationsNb = 11;
        bool isInserted = true;
        using ProgressScope ps = new(range, "Iteration", iterationsNb);
        for (int pass = 1; pass <= iterationsNb && isInserted && !_isAllDegenerated; ++pass)
        {
            if (!ps.More())
            {
                return;
            }
            // Reset stop condition
            _maxSqDeflection = -1.0;
            _isAllDegenerated = true;
            _controlNodes.Clear();

            if (GetStructure().ElementsOfDomain().Extent() < 1)
            {
                break;
            }
            // Iterate on current triangles
            _passLinks.Reset(GetStructure().LinkCount);
            NCollection_PackedMapOfInteger.Iterator triangleIt = new(GetStructure().ElementsOfDomain());
            for (; triangleIt.More(); triangleIt.Next())
            {
                Triangle triangle = GetStructure().GetElement(triangleIt.Key());
                SplitTriangleGeometry(triangle);
            }

            if (!_controlNodes.IsEmpty())
            {
                KeepPassLinks();
            }

            isInserted = InsertNodes(_controlNodes, mesher, ps.Next());
        }

        _couplesMap = null;
        _passLinks = null;
        _controlNodes = null;

        if (!(_maxSqDeflection < 0.0))
        {
            GetDFace().SetDeflection(Math.Sqrt(_maxSqDeflection));
        }
    }

    // Adds the node pairs of the links this pass marked to those of the passes before, before the structure changes
    private void KeepPassLinks()
    {
        DataStructureOfDelaun structure = GetStructure();
        int linkCount = structure.LinkCount;
        _couplesMap ??= new HashSet<OrientedEdge>(linkCount);
        for (int index = 1; index <= linkCount; index++)
        {
            if (_passLinks!.IsMarked(index))
            {
                MeshEdge link = structure.GetLink(index);
                int first = link.FirstNode(), last = link.LastNode();
                _couplesMap.Add(first < last ? new OrientedEdge(first, last) : new OrientedEdge(last, first));
            }
        }
    }

    // Contains geometrical data related to node of triangle.
    private struct TriangleNodeInfo
    {
        public Xy Point2d;
        public Xyz Point;
        public bool isFrontierLink;
    }

    // A deviation functor.
    private interface IDeviation
    {
        double SquareDeviation(in Point3D point);
    }

    // Functor computing deflection of a point from surface.
    private readonly struct NormalDeviation(in Point3D theRefPnt, in Vector3D theNormal) : IDeviation
    {
        private readonly Point3D _refPnt = theRefPnt;
        private readonly Vector3D _normal = theNormal;

        public double SquareDeviation(in Point3D point)
        {
            double deflection = Math.Abs(_normal.Dot(new Vector3D(_refPnt, point)));
            return deflection * deflection;
        }
    }

    // Functor computing deflection of a point on triangle link from surface.
    private readonly struct LineDeviation(in Point3D thePnt1, in Point3D thePnt2) : IDeviation
    {
        private readonly Point3D _pnt1 = thePnt1;
        private readonly Point3D _pnt2 = thePnt2;

        public double SquareDeviation(in Point3D point)
        {
            return GeomTool.SquareDeflectionOfSegment(_pnt1, _pnt2, point);
        }
    }

    // Returns nodes info of the given triangle.
    private void GetTriangleInfo(in Triangle triangle,
                                 ReadOnlySpan<int> nodesIndices,
                                 Span<TriangleNodeInfo> info)
    {
        var e = triangle._edges;
        for (int i = 0; i < 3; ++i)
        {
            MeshVertex vertex = GetStructure().GetNode(nodesIndices[i]);
            info[i].Point2d = GetRangeSplitter().Scale(new Point2D(vertex.Coord()), false).XY();
            info[i].Point = GetNodesMap()[vertex.Location3d()].XYZ();
            info[i].isFrontierLink =
                (GetStructure().GetLink(e[i]).Movability() == DegreeOfFreedom.Frontier);
        }
    }

    // Check geometry of the given triangle. If triangle does not suit specified deflection, inserts new point.
    private void SplitTriangleGeometry(in Triangle triangle)
    {
        if (triangle.Movability() != DegreeOfFreedom.Deleted)
        {
            Span<int> nodexIndices = stackalloc int[3];
            GetStructure().ElementNodes(triangle, nodexIndices);

            Span<TriangleNodeInfo> nodesInfo = stackalloc TriangleNodeInfo[3];
            GetTriangleInfo(triangle, nodexIndices, nodesInfo);

            Vector3D normal = new();
            Span<Vector3D> linkVec = stackalloc Vector3D[3];
            if (ComputeTriangleGeometry(nodesInfo, linkVec, ref normal))
            {
                _isAllDegenerated = false;

                Xy center2d =
                    (nodesInfo[0].Point2d + nodesInfo[1].Point2d + nodesInfo[2].Point2d) / 3.0;

                UsePoint(center2d, new NormalDeviation(new Point3D(nodesInfo[0].Point), normal));
                SplitLinks(nodesInfo, nodexIndices, triangle._edges);
            }
        }
    }

    // Updates array of links vectors. Returns false on degenerative triangle.
    private bool ComputeTriangleGeometry(ReadOnlySpan<TriangleNodeInfo> nodesInfo,
                                         Span<Vector3D> links,
                                         ref Vector3D normal)
    {
        if (CheckTriangleForDegenerativityAndGetLinks(nodesInfo, links))
        {
            if (CheckTriangleArea2d(nodesInfo))
            {
                if (ComputeNormal(links[0], links[1], ref normal))
                {
                    return true;
                }
            }
        }

        return false;
    }

    // Updates array of links vectors. Returns false on degenerative triangle.
    private static bool CheckTriangleForDegenerativityAndGetLinks(ReadOnlySpan<TriangleNodeInfo> nodesInfo,
                                                                  Span<Vector3D> links)
    {
        const double minimalSqLength3d = 1.0e-12;
        for (int i = 0; i < 3; ++i)
        {
            links[i] = new Vector3D(nodesInfo[(i + 1) % 3].Point - nodesInfo[i].Point);
            if (links[i].SquareMagnitude < minimalSqLength3d)
            {
                return false;
            }
        }

        return true;
    }

    // Checks area of triangle in parametric space for degenerativity. Returns false on degenerative triangle.
    private static bool CheckTriangleArea2d(ReadOnlySpan<TriangleNodeInfo> nodesInfo)
    {
        Vector2D link2d1 = new(new Point2D(nodesInfo[0].Point2d), new Point2D(nodesInfo[1].Point2d));
        Vector2D link2d2 = new(new Point2D(nodesInfo[1].Point2d), new Point2D(nodesInfo[2].Point2d));

        const double minimalArea2d = 1.0e-9;
        return (Math.Abs(link2d1 ^ link2d2) > minimalArea2d);
    }

    // Computes normal using two link vectors. Returns true on success, false in case of normal of null magnitude.
    private static bool ComputeNormal(in Vector3D link1, in Vector3D link2, ref Vector3D normal)
    {
        Vector3D aNormal = link1 ^ link2;
        if (aNormal.SquareMagnitude > Constants.Resolution())
        {
            normal = aNormal.Normalized();
            return true;
        }

        return false;
    }

    // Computes deflection of midpoints of triangles links.
    private void SplitLinks(ReadOnlySpan<TriangleNodeInfo> nodesInfo, ReadOnlySpan<int> nodesIndices, Triangle.Int3 triangleEdges)
    {
        // Check deflection at triangle links
        for (int i = 0; i < 3; ++i)
        {
            if (nodesInfo[i].isFrontierLink)
            {
                continue;
            }

            int j = (i + 1) % 3;
            // Check if this link was already processed
            int firstVertex, lastVertex;
            if (nodesIndices[i] < nodesIndices[j])
            {
                firstVertex = nodesIndices[i];
                lastVertex = nodesIndices[j];
            }
            else
            {
                firstVertex = nodesIndices[j];
                lastVertex = nodesIndices[i];
            }

            if (_passLinks!.Add(Math.Abs(triangleEdges[i]))
                && (_couplesMap is null || !_couplesMap.Contains(new OrientedEdge(firstVertex, lastVertex))))
            {
                Xy midPnt2d = (nodesInfo[i].Point2d + nodesInfo[j].Point2d) / 2.0;

                if (!UsePoint(midPnt2d, new LineDeviation(new Point3D(nodesInfo[i].Point), new Point3D(nodesInfo[j].Point))))
                {
                    if (!RejectSplitLinksForMinSize(nodesInfo[i], nodesInfo[j], midPnt2d))
                    {
                        if (!CheckLinkEndsForAngularDeviation(nodesInfo[i], nodesInfo[j], midPnt2d))
                        {
                            _controlNodes!.Append(new Point2D(midPnt2d));
                        }
                    }
                }
            }
        }
    }

    // Checks that two links produced as the result of a split of the given link by the middle point fit MinSize
    // requirement.
    private bool RejectSplitLinksForMinSize(in TriangleNodeInfo nodeInfo1,
                                            in TriangleNodeInfo nodeInfo2,
                                            in Xy midPoint)
    {
        Point3D pnt = GetPoint3d(midPoint);
        return ((nodeInfo1.Point - pnt.XYZ()).SquareModulus() < _sqMinSize
                || (nodeInfo2.Point - pnt.XYZ()).SquareModulus() < _sqMinSize);
    }

    // Checks the given point (located between the given nodes) for specified angular deviation.
    private bool CheckLinkEndsForAngularDeviation(in TriangleNodeInfo nodeInfo1,
                                                  in TriangleNodeInfo nodeInfo2,
                                                  in Xy midPoint)
    {
        _ = midPoint;
        Direction3D norm1 = new(), norm2 = new();
        Surface surf = GetDFace().GetSurface().GeomSurfaceOriginal()!;

        if ((global::Kerf.Engine.Geometry.Library.GeomLib.NormEstim(surf, new Point2D(nodeInfo1.Point2d), global::Kerf.Engine.Foundation.Precision.Confusion(), ref norm1) == 0)
            && (global::Kerf.Engine.Geometry.Library.GeomLib.NormEstim(surf, new Point2D(nodeInfo2.Point2d), global::Kerf.Engine.Foundation.Precision.Confusion(), ref norm2) == 0))
        {
            double angle = norm1.AngleTo(norm2);
            if (angle > GetParameters().AngleInterior)
            {
                return false;
            }
        }

        return true;
    }

    // Returns 3d point corresponding to the given one in 2d space.
    private Point3D GetPoint3d(in Xy pnt2d)
    {
        GetDFace().GetSurface().D0(pnt2d.X(), pnt2d.Y(), out Point3D pnt);
        return pnt;
    }

    // Computes deflection of the given point and caches it for insertion in case if it overflows deflection.
    // Returns true if point has been cached for insertion.
    private bool UsePoint<DeflectionFunctor>(in Xy pnt2d, in DeflectionFunctor deflectionFunctor)
        where DeflectionFunctor : struct, IDeviation
    {
        Point3D pnt = GetPoint3d(pnt2d);
        if (!CheckDeflectionOfPointAndUpdateCache(pnt2d,
                                                  pnt,
                                                  deflectionFunctor.SquareDeviation(pnt)))
        {
            _controlNodes!.Append(new Point2D(pnt2d));
            return true;
        }

        return false;
    }

    // Checks the given point for specified linear deflection. Updates value of total mesh deflection.
    private bool CheckDeflectionOfPointAndUpdateCache(in Xy pnt2d, in Point3D pnt3d, double sqDeflection)
    {
        if (sqDeflection > _maxSqDeflection)
        {
            _maxSqDeflection = sqDeflection;
        }

        double aSqDeflection = GetDFace().GetDeflection() * GetDFace().GetDeflection();
        if (sqDeflection < aSqDeflection)
        {
            return true;
        }

        return RejectByMinSize(pnt2d, pnt3d);
    }

    // Checks distance between the given node and nodes of triangles shot by it for MinSize criteria.
    // This check is expected to roughly estimate and prevent generation of triangles with sides smaller than MinSize.
    private bool RejectByMinSize(in Xy pnt2d, in Point3D pnt3d)
    {
        NCollection_PackedMapOfInteger usedNodes = new();
        List<int> circlesList =
            _circles!.Select(GetRangeSplitter().Scale(new Point2D(pnt2d), true).XY());

        Span<int> nodes = stackalloc int[3];
        foreach (int circle in circlesList)
        {
            Triangle triangle = GetStructure().GetElement(circle);

            GetStructure().ElementNodes(triangle, nodes);

            for (int i = 0; i < 3; ++i)
            {
                if (!usedNodes.Contains(nodes[i]))
                {
                    usedNodes.Add(nodes[i]);
                    MeshVertex vertex = GetStructure().GetNode(nodes[i]);
                    Point3D point = GetNodesMap()[vertex.Location3d()];

                    if (pnt3d.SquareDistanceTo(point) < _sqMinSize)
                    {
                        return true;
                    }
                }
            }
        }

        return false;
    }
}
