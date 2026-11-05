// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_ProximityDistTool.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_ProximityDistTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2022-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bvh;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Kerf.Engine;
using BVH_PrjState = Kerf.Engine.Geometry.Bvh.Tools<double, Kerf.Engine.NCollection_Vec3<double>>.BVH_PrjStateInTriangle;
using BVH_Vec3d = Kerf.Engine.NCollection_Vec3<double>;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Measurement.Distance;

/// <summary>
/// The proximity distance from a first primitive set to a second: the largest of the smallest perpendicular
/// distances from the first set's vertices to the second set (or the smallest distance when there is no
/// perpendicular one). Results are approximate and depend on the tessellations.
/// </summary>
/// <remarks>A working object (decision 21), used by <see cref="ProximityValueTool"/>.</remarks>
internal class ProximityDistTool : BvhDistance<double, BVH_Vec3d, BVH_Vec3d, TriangleSet>
{
    /// <summary>The status of a proximity point: on the border or inside its shape.</summary>
    public enum ProxPnt_Status
    {
        ProxPnt_Status_BORDER,
        ProxPnt_Status_MIDDLE,
        ProxPnt_Status_UNKNOWN,
    }

    /// <summary>Where the projection giving a proximity point of the second shape lies on its triangle.</summary>
    public readonly struct PrjState
    {
        private readonly int _trgIdx; //!< Index of triangle on which the projection is located
        private readonly BVH_PrjState _prjState; //!< Position of a projection on the triangle (vertex, edge, inner)
        private readonly int _numberOfFirstNode; //!< The 1st vtx of the triangle edge on which the projection is located
        private readonly int _numberOfLastNode; //!< The 2nd vtx of the triangle edge on which the projection is located

        public PrjState()
        {
            _trgIdx = 0;
            _prjState = BVH_PrjState.PrjStateInTriangle_INNER;
            _numberOfFirstNode = 0;
            _numberOfLastNode = 0;
        }

        public PrjState(int trgIdx, BVH_PrjState prjState, int numberOfFirstNode, int numberOfLastNode)
        {
            _trgIdx = trgIdx;
            _prjState = prjState;
            _numberOfFirstNode = numberOfFirstNode;
            _numberOfLastNode = numberOfLastNode;
        }

        public int GetTrgIdx() => _trgIdx;

        public BVH_PrjState GetPrjState() => _prjState;

        public int GetNumberOfFirstNode() => _numberOfFirstNode;

        public int GetNumberOfLastNode() => _numberOfLastNode;
    }

    protected double _minDistance; //!< Minimal distance from point to BVH, could be not equal to myDistance
    protected BVH_Vec3d _minDistPoint; //!< Point on BVH providing the minimal distance
    protected BVH_Vec3d _extremaPoint; //!< Point on BVH providing the extrema
    protected double _proxDist; //!< Proximity distance
    protected BVH_Vec3d _pnt1, _pnt2; //!< Proximity points
    protected ProxPnt_Status _pntStatus1, _pntStatus2; //!< Proximity points' status

    private TriangleSet? _set1;
    private TriangleSet? _set2;
    private List<Shape> _shapeList1 = new();
    private List<Shape> _shapeList2 = new();
    private readonly int _nbSamples1; //!< Number of samples points on the first shape
    private bool _isProxVtx1FromAddSet; //!< Is the vertex of the proximity point of the 1st shape from the additional set
    private NCollection_LinearVector<BVH_Vec3d> _addVertices1 = new(); //!< Additional vertices on the 1st shape
    private List<ProxPnt_Status> _addStatus1 = new(); //!< Status of additional vertices on the 1st shape
    private int _proxVtxIdx1; //!< Vertex index from 1st BVH corresponding to proximity point of 1st shape
    private PrjState _proxPrjState = new(); //!< Projection state from the 2nd BVH giving the proximity point of the 2nd shape
    private PrjState _extPrjState = new(); //!< Projection state from the 2nd BVH giving the extrema
    private PrjState _minPrjState = new(); //!< Projection state from the 2nd BVH giving the minimal distance

    /// <summary>An uninitialized tool.</summary>
    public ProximityDistTool()
    {
        _minDistance = double.MaxValue;
        _proxDist = -1.0;
        _pntStatus1 = ProxPnt_Status.ProxPnt_Status_UNKNOWN;
        _pntStatus2 = ProxPnt_Status.ProxPnt_Status_UNKNOWN;
        _nbSamples1 = 0;
        _proxVtxIdx1 = -1;
    }

    /// <summary>A tool for the given element sets.</summary>
    public ProximityDistTool(TriangleSet set1,
                                         int nbSamples1,
                                         NCollection_LinearVector<BVH_Vec3d> addVertices1,
                                         List<ProxPnt_Status> addStatus1,
                                         TriangleSet set2,
                                         List<Shape> shapeList1,
                                         List<Shape> shapeList2)
    {
        _minDistance = double.MaxValue;
        _proxDist = -1.0;
        _pntStatus1 = ProxPnt_Status.ProxPnt_Status_UNKNOWN;
        _pntStatus2 = ProxPnt_Status.ProxPnt_Status_UNKNOWN;
        _nbSamples1 = nbSamples1;
        _proxVtxIdx1 = -1;
        LoadTriangleSets(set1, set2);
        LoadShapeLists(shapeList1, shapeList2);
        LoadAdditionalPointsFirstSet(addVertices1, addStatus1);
    }

    /// <summary>Loads the element sets.</summary>
    public void LoadTriangleSets(TriangleSet set1, TriangleSet set2)
    {
        _set1 = set1;
        _set2 = set2;
    }

    /// <summary>Loads the lists of sub-shapes (copies, as OCCT assigns by value).</summary>
    public void LoadShapeLists(List<Shape> shapeList1, List<Shape> shapeList2)
    {
        _shapeList1 = new List<Shape>(shapeList1);
        _shapeList2 = new List<Shape>(shapeList2);
    }

    /// <summary>Loads the additional vertices of the first set and their statuses (copies).</summary>
    public void LoadAdditionalPointsFirstSet(NCollection_LinearVector<BVH_Vec3d> addVertices1,
                                             List<ProxPnt_Status> addStatus1)
    {
        _addVertices1 = new NCollection_LinearVector<BVH_Vec3d>(addVertices1);
        _addStatus1 = new List<ProxPnt_Status>(addStatus1);
    }

    //=======================================================================
    // function : goThroughtSet1
    // purpose  : Goes through vertices from the 1st set
    //=======================================================================
    private void GoThroughtSet1(NCollection_LinearVector<BVH_Vec3d> vertices1, bool isAdditionalSet)
    {
        int vtxSize = vertices1.Size();
        int vtxStep = Max(_nbSamples1 <= 0 ? 1 : vtxSize / _nbSamples1, 1);
        for (int vtxIdx = 0; vtxIdx < vtxSize; vtxIdx += vtxStep)
        {
            _distance = double.MaxValue;
            _minDistance = double.MaxValue;
            _isDone = false;
            SetObject(vertices1[vtxIdx]);

            ComputeDistance();

            if (!IsDone && _proxDist < 0.0)
            {
                return;
            }

            if (IsDone && _distance > _proxDist)
            {
                _pnt1 = vertices1[vtxIdx];
                _pnt2 = _extremaPoint;
                _proxDist = _distance;
                _proxVtxIdx1 = vtxIdx;
                _isProxVtx1FromAddSet = isAdditionalSet;
                _proxPrjState = _extPrjState;
            }
        }
    }

    /// <summary>Searches for the proximity distance.</summary>
    public void Perform()
    {
        SetBVHSet(_set2);

        GoThroughtSet1(_set1!.GetVertices(), false);
        GoThroughtSet1(_addVertices1, true);

        _isDone = _proxDist > -1.0;

        if (_isDone)
        {
            DefineStatusProxPnt();
        }
    }

    private static double PointBoxSquareMaxDistance(in BVH_Vec3d point, in BVH_Vec3d cMin, in BVH_Vec3d cMax)
    {
        double dist = 0;
        for (int i = 0; i < 3; ++i)
        {
            if (point[i] <= 0.5 * (cMax[i] + cMin[i]))
            {
                double d = cMax[i] - point[i];
                d *= d;
                dist += d;
            }
            else
            {
                double d = point[i] - cMin[i];
                d *= d;
                dist += d;
            }
        }
        return dist;
    }

    /// <summary>Rejects a node by its bounding box.</summary>
    public override bool RejectNode(in BVH_Vec3d cornerMin, in BVH_Vec3d cornerMax, out double metric)
    {
        metric = Math.Sqrt(Tools<double, BVH_Vec3d>.PointBoxSquareDistance(_object, cornerMin, cornerMax));

        double maxMetric = Math.Sqrt(PointBoxSquareMaxDistance(_object, cornerMin, cornerMax));

        return metric > _distance || maxMetric < _proxDist;
    }

    /// <summary>Accepts a leaf element (a triangle of the second set).</summary>
    public override bool Accept(int trgIdx, in double metric)
    {
        BVH_PrjState bvh_PrjState = default;
        int numberOfFirstNode = -1;
        int numberOfLastNode = -1;

        _set2!.GetVertices(trgIdx, out BVH_Vec3d trgVert1, out BVH_Vec3d trgVert2, out BVH_Vec3d trgVert3);

        BVH_Vec3d nearestPnt = Tools<double, BVH_Vec3d>.PointTriangleProjection(_object,
                                                                                    trgVert1,
                                                                                    trgVert2,
                                                                                    trgVert3,
                                                                                    ref bvh_PrjState,
                                                                                    ref numberOfFirstNode,
                                                                                    ref numberOfLastNode);

        var prjState = new PrjState(trgIdx, bvh_PrjState, numberOfFirstNode, numberOfLastNode);
        BVH_Vec3d direct = _object - nearestPnt;
        double sqDistance = direct.Dot(direct);

        if (sqDistance > global::Kerf.Engine.Foundation.Precision.SquareConfusion()) // point belongs to triangle
        {
            BVH_Vec3d ab = trgVert2 - trgVert1;

            BVH_Vec3d norm;
            if (trgVert2.IsEqual(trgVert3)) // is this degenerate triangle (= segment)
            {
                BVH_Vec3d ap = _object - trgVert1;
                norm = BVH_Vec3d.Cross(BVH_Vec3d.Cross(ap, ab), ab);
            }
            else
            {
                BVH_Vec3d ac = trgVert3 - trgVert1;
                norm = BVH_Vec3d.Cross(ab, ac);
            }

            double normSqLen = norm.Dot(norm);

            // check if the distance is under perpendicular
            BVH_Vec3d crossCross = BVH_Vec3d.Cross(direct, norm);
            double crossCrossSqLen = crossCross.Dot(crossCross);
            if (crossCrossSqLen > global::Kerf.Engine.Foundation.Precision.SquareConfusion() * sqDistance * normSqLen)
            {
                // the distance is not under perpendicular
                if (_minDistance - Math.Sqrt(sqDistance) > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    _minDistance = Math.Sqrt(sqDistance);
                    _minDistPoint = nearestPnt;
                    _minPrjState = prjState;
                }

                return false;
            }
        }

        // the distance is under perpendicular
        if (_distance - Math.Sqrt(sqDistance) > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            _distance = Math.Sqrt(sqDistance);
            _extremaPoint = nearestPnt;
            _extPrjState = prjState;

            return true;
        }

        return false;
    }

    /// <summary>
    /// Computes the distance between the object and the BVH tree, falling back on the smallest non-perpendicular
    /// distance (hides BVH_Distance's, as OCCT's does).
    /// </summary>
    protected new double ComputeDistance()
    {
        _isDone = Select() > 0;

        if (!_isDone)
        {
            if (_minDistance < double.MaxValue)
            {
                _extremaPoint = _minDistPoint;
                _extPrjState = _minPrjState;
                _isDone = true;
            }

            _distance = _minDistance;
        }

        return _distance;
    }

    /// <summary>True if the node is on the border of the triangulation.</summary>
    public static bool IsNodeOnBorder(int nodeIdx, Triangulation tr)
    {
        var polyConnect = new Connect(tr);

        int contTrg; // index of triangle containing exploring node
        Span<int> contTrgNodes = stackalloc int[3];
        Span<int> adjTrg = stackalloc int[3];
        for (polyConnect.Initialize(nodeIdx); polyConnect.More(); polyConnect.Next())
        {
            contTrg = polyConnect.Value();

            tr.Triangle(contTrg).Get(out contTrgNodes[0], out contTrgNodes[1], out contTrgNodes[2]); // indices of nodes of the triangle

            polyConnect.Triangles(contTrg, out adjTrg[0], out adjTrg[1], out adjTrg[2]); // indices of adjacent triangles

            for (int j = 0; j < 3; j++)
            {
                int k = (j + 1) % 3;
                if (adjTrg[j] == 0) // free segment of triangle
                {
                    // aContTrgNodes[j], aContTrgNodes[k] are ends of free segment and it is a part of border
                    if (contTrgNodes[j] == nodeIdx || contTrgNodes[k] == nodeIdx)
                    {
                        return true;
                    }
                }
            }
        }

        return false;
    }

    /// <summary>True if the edge (given by the numbers of its nodes in the triangle) is on the border.</summary>
    public static bool IsEdgeOnBorder(int trgIdx, int firstEdgeNodeIdx, int secondEdgeNodeIdx, Triangulation tr)
    {
        var polyConnect = new Connect(tr);

        Span<int> adjTrg = stackalloc int[3];
        polyConnect.Triangles(trgIdx, out adjTrg[0], out adjTrg[1], out adjTrg[2]); // indices of adjacent triangles

        for (int j = 0; j < 3; j++)
        {
            int k = (j + 1) % 3;
            if (adjTrg[j] == 0) // free segment of triangle
            {
                // are ends of free segment and it is a part of border
                if (j == firstEdgeNodeIdx && k == secondEdgeNodeIdx)
                {
                    return true;
                }
            }
        }

        return false;
    }

    private void DefineStatusProxPnt1()
    {
        if (_isProxVtx1FromAddSet)
        {
            _pntStatus1 = _addStatus1[_proxVtxIdx1];
            return;
        }

        int faceID1 = _set1!.GetShapeIDOfVtx(_proxVtxIdx1);

        if (_shapeList1[faceID1].Type == ShapeType.Edge)
        {
            NCollection_LinearVector<BVH_Vec3d> vertices1 = _set1.GetVertices();
            int vtxSize = vertices1.Size();
            int lastIdx = vtxSize - 1;

            if ((vertices1[0] - vertices1[lastIdx]).Modulus() < global::Kerf.Engine.Foundation.Precision.Confusion()) // if closed
            {
                _pntStatus1 = ProxPnt_Status.ProxPnt_Status_MIDDLE;
                return;
            }

            if (_proxVtxIdx1 == 0 || _proxVtxIdx1 == lastIdx)
            {
                _pntStatus1 = ProxPnt_Status.ProxPnt_Status_BORDER;
            }
            else
            {
                _pntStatus1 = ProxPnt_Status.ProxPnt_Status_MIDDLE;
            }
        }
        else if (_shapeList1[faceID1].Type == ShapeType.Face)
        {
            int nodeIdx = _set1.GetVtxIdxInShape(_proxVtxIdx1) + 1;

            Face f = ShapeCasts.Face(_shapeList1[faceID1]);
            Triangulation tr = Tool.Triangulation(f, out _)!;

            if (IsNodeOnBorder(nodeIdx, tr))
            {
                _pntStatus1 = ProxPnt_Status.ProxPnt_Status_BORDER;
            }
            else
            {
                _pntStatus1 = ProxPnt_Status.ProxPnt_Status_MIDDLE;
            }
        }
    }

    private void DefineStatusProxPnt2()
    {
        int trgIdx = _proxPrjState.GetTrgIdx();
        int faceID2 = _set2!.GetFaceID(trgIdx);

        if (_shapeList2[faceID2].Type == ShapeType.Edge)
        {
            if (_proxPrjState.GetPrjState() == BVH_PrjState.PrjStateInTriangle_INNER)
            {
                return;
            }
            else
            {
                NCollection_LinearVector<BVH_Vec3d> vertices2 = _set2.GetVertices();
                int vtxSize = vertices2.Size();
                int lastIdx = vtxSize - 1;

                if ((vertices2[0] - vertices2[lastIdx]).Modulus() < global::Kerf.Engine.Foundation.Precision.Confusion()) // if closed
                {
                    _pntStatus2 = ProxPnt_Status.ProxPnt_Status_MIDDLE;
                    return;
                }

                _set2.GetVtxIndices(trgIdx, out NCollection_Array1<int> vtxIndicesOfTrg);

                int firstNodeNum = _proxPrjState.GetNumberOfFirstNode();
                int firstVtxIdx = vtxIndicesOfTrg[firstNodeNum];

                if (_proxPrjState.GetPrjState() == BVH_PrjState.PrjStateInTriangle_VERTEX)
                {
                    if (firstVtxIdx == 0 || firstVtxIdx == lastIdx)
                    {
                        _pntStatus2 = ProxPnt_Status.ProxPnt_Status_BORDER;
                    }
                    else
                    {
                        _pntStatus2 = ProxPnt_Status.ProxPnt_Status_MIDDLE;
                    }
                }
                else if (_proxPrjState.GetPrjState() == BVH_PrjState.PrjStateInTriangle_EDGE)
                {
                    int lastNodeNum = _proxPrjState.GetNumberOfLastNode();
                    int lastVtxIdx = vtxIndicesOfTrg[lastNodeNum];

                    // it could be border only in case projection is on a degenerated edge
                    if (firstVtxIdx == lastVtxIdx && (firstVtxIdx == 0 || firstVtxIdx == lastIdx))
                    {
                        _pntStatus2 = ProxPnt_Status.ProxPnt_Status_BORDER;
                    }
                    else
                    {
                        _pntStatus2 = ProxPnt_Status.ProxPnt_Status_MIDDLE;
                    }
                }
            }
        }
        else if (_shapeList2[faceID2].Type == ShapeType.Face)
        {
            if (_proxPrjState.GetPrjState() == BVH_PrjState.PrjStateInTriangle_INNER)
            {
                _pntStatus2 = ProxPnt_Status.ProxPnt_Status_MIDDLE;
            }
            else
            {
                Face f = ShapeCasts.Face(_shapeList2[faceID2]);
                Triangulation tr = Tool.Triangulation(f, out _)!;

                _set2.GetVtxIndices(trgIdx, out NCollection_Array1<int> vtxIndicesOfTrg);

                if (_proxPrjState.GetPrjState() == BVH_PrjState.PrjStateInTriangle_VERTEX)
                {
                    int nodeNum = _proxPrjState.GetNumberOfFirstNode();
                    int nodeIdx = _set2.GetVtxIdxInShape(vtxIndicesOfTrg[nodeNum]) + 1;

                    if (IsNodeOnBorder(nodeIdx, tr))
                    {
                        _pntStatus2 = ProxPnt_Status.ProxPnt_Status_BORDER;
                    }
                    else
                    {
                        _pntStatus2 = ProxPnt_Status.ProxPnt_Status_MIDDLE;
                    }
                }
                else if (_proxPrjState.GetPrjState() == BVH_PrjState.PrjStateInTriangle_EDGE)
                {
                    int trgIdxInShape = _set2.GetTrgIdxInShape(trgIdx) + 1;

                    if (IsEdgeOnBorder(trgIdxInShape,
                                       _proxPrjState.GetNumberOfFirstNode(),
                                       _proxPrjState.GetNumberOfLastNode(),
                                       tr))
                    {
                        _pntStatus2 = ProxPnt_Status.ProxPnt_Status_BORDER;
                    }
                    else
                    {
                        _pntStatus2 = ProxPnt_Status.ProxPnt_Status_MIDDLE;
                    }
                }
            }
        }
    }

    /// <summary>Defines the status of the proximity points.</summary>
    protected void DefineStatusProxPnt()
    {
        // define the status of proximity point from 1st BVH
        DefineStatusProxPnt1();

        // define the status of proximity point from 2nd BVH
        DefineStatusProxPnt2();
    }

    /// <summary>The points of the two sets giving the proximity distance.</summary>
    public void ProximityPoints(out BVH_Vec3d point1, out BVH_Vec3d point2)
    {
        point1 = _pnt1;
        point2 = _pnt2;
    }

    /// <summary>The statuses of the points giving the proximity distance.</summary>
    public void ProximityPointsStatus(out ProxPnt_Status pointStatus1, out ProxPnt_Status pointStatus2)
    {
        pointStatus1 = _pntStatus1;
        pointStatus2 = _pntStatus2;
    }

    /// <summary>The proximity distance.</summary>
    public double ProximityDistance() => _proxDist;
}
