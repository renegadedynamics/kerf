// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_ProximityValueTool.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_ProximityValueTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2022-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Measurement;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using BVH_Vec3d = Occt.NCollection_Vec3<double>;
using ProxPnt_Status = Kerf.Engine.Measurement.Distance.ProximityDistTool.ProxPnt_Status;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Measurement.Distance;

/// <summary>
/// The CellFilter inspector of gp_XYZ points in 3D space, used to find coincident points within a tolerance.
/// </summary>
public sealed class DistanceVertexInspector : ICellFilterInspector<int, Xyz>
{
    private double _tol;
    private bool _isNeedAdd;
    private readonly NCollection_DynamicArray<Xyz> _points = new();
    private Xyz _current;

    public static int Dimension => 3;

    public static double Coord(int i, Xyz pnt) => pnt.Coord(i + 1);

    public static Xyz Shift(in Xyz pnt, double tol) => new(pnt.X() + tol, pnt.Y() + tol, pnt.Z() + tol);

    /// <summary>An inspector with the square tolerance Precision::SquareConfusion().</summary>
    public DistanceVertexInspector()
    {
        _tol = global::Kerf.Engine.Foundation.Precision.SquareConfusion();
        _isNeedAdd = true;
    }

    /// <summary>Keeps a point to compare with.</summary>
    public void Add(in Xyz pnt) => _points.Append(pnt);

    /// <summary>Sets the (square) tolerance of the comparison of coordinates.</summary>
    public void SetTol(double tol) => _tol = tol;

    /// <summary>Sets the point to search coincidences for.</summary>
    public void SetCurrent(in Xyz curPnt)
    {
        _current = curPnt;
        _isNeedAdd = true;
    }

    /// <summary>True if the current point coincides with no kept point.</summary>
    public bool IsNeedAdd => _isNeedAdd;

    /// <summary>
    /// Compares the current point with the kept point <paramref name="target"/> (from 1).
    /// </summary>
    /// <remarks>
    /// OCCT: each call sets the flag again before comparing, so only the last target inspected decides
    /// (occt-issues.md).
    /// </remarks>
    public CellFilterAction Inspect(int target)
    {
        _isNeedAdd = true; // OCCT: reset for every target, so a coincidence found before a later target is forgotten

        Xyz pnt = _points.Value(target - 1);
        double dx, dy, dz;
        dx = _current.X() - pnt.X();
        dy = _current.Y() - pnt.Y();
        dz = _current.Z() - pnt.Z();

        if ((dx * dx <= _tol) && (dy * dy <= _tol) && (dz * dz <= _tol))
        {
            _isNeedAdd = false;
        }

        return CellFilterAction.CellFilter_Keep;
    }
}

/// <summary>
/// The proximity value from one BVH primitive set to another, solving the max(min) problem, for edge/edge and
/// face/face cases only: the coarser mesh is refined first. Results are approximate and depend on the tessellations.
/// </summary>
/// <remarks>A working object (decision 21), used by <see cref="ShapeProximity"/>. It only reads the meshes.</remarks>
public sealed class ProximityValueTool
{
    private TriangleSet? _set1;
    private TriangleSet? _set2;
    private NCollection_DynamicArray<Shape> _shapeList1 = new();
    private NCollection_DynamicArray<Shape> _shapeList2 = new();
    private Shape _shape1;
    private Shape _shape2;
    private readonly NCollection_LinearVector<BVH_Vec3d> _addVertices1 = new(); //!< Additional vertices on the 1st shape if its mesh is coarser.
    private readonly NCollection_LinearVector<BVH_Vec3d> _addVertices2 = new(); //!< Additional vertices on the 2nd shape if its mesh is coarser.
    private readonly NCollection_DynamicArray<ProxPnt_Status> _addStatus1 = new(); //!< Status of additional vertices on the 1st shape.
    private readonly NCollection_DynamicArray<ProxPnt_Status> _addStatus2 = new(); //!< Status of additional vertices on the 2nd shape.
    private bool _isInitS1; //!< Is the 1st shape initialized?
    private bool _isInitS2; //!< Is the 2nd shape initialized?
    private bool _isRefinementRequired1; //!< Flag about the need to refine the 1st shape.
    private bool _isRefinementRequired2; //!< Flag about the need to refine the 2nd shape.
    private int _nbNodes1; //!< Number of nodes in triangulation of the 1st shape.
    private int _nbNodes2; //!< Number of nodes in triangulation of the 2nd shape.
    private double _step1; //!< Step for getting vertices on the 1st shape.
    private double _step2; //!< Step for getting vertices on the 2nd shape.
    private readonly CellFilter<DistanceVertexInspector, int, Xyz> _cells = new();
    private DistanceVertexInspector _inspector = new();
    private ShapeType _shapeType1; //!< 1st shape type.
    private ShapeType _shapeType2; //!< 2nd shape type.
    private double _distance; //!< Distance
    private bool _isDone; //!< State of the algorithm
    private int _nbSamples1; //!< Number of samples points on the first shape
    private int _nbSamples2; //!< Number of samples points on the second shape
    private Point3D _pnt1, _pnt2; //!< Proximity points
    private ProxPnt_Status _pntStatus1, _pntStatus2; //!< Proximity points' status

    /// <summary>An uninitialized proximity tool.</summary>
    public ProximityValueTool()
    {
        _isRefinementRequired1 = false;
        _isRefinementRequired2 = false;
        _distance = double.MaxValue;
        _isDone = false;
        _nbSamples1 = 0;
        _nbSamples2 = 0;
        // Should be initialized later
        _isInitS1 = _isInitS2 = false;
    }

    /// <summary>A proximity tool for the given element sets and lists of sub-shapes.</summary>
    public ProximityValueTool(TriangleSet set1,
                                          TriangleSet set2,
                                          NCollection_DynamicArray<Shape> shapeList1,
                                          NCollection_DynamicArray<Shape> shapeList2)
    {
        _isRefinementRequired1 = false;
        _isRefinementRequired2 = false;
        _distance = double.MaxValue;
        _isDone = false;
        _nbSamples1 = 0;
        _nbSamples2 = 0;
        LoadShapeLists(shapeList1, shapeList2);
        LoadTriangleSets(set1, set2);
    }

    /// <summary>Loads the element sets.</summary>
    public void LoadTriangleSets(TriangleSet set1, TriangleSet set2)
    {
        _set1 = set1;
        _set2 = set2;

        MarkDirty();
    }

    /// <summary>True if the proximity test is completed.</summary>
    public bool IsDone => _isDone;

    /// <summary>Marks the results as outdated.</summary>
    public void MarkDirty() => _isDone = false;

    /// <summary>The computed distance.</summary>
    public double Distance() => _distance;

    /// <summary>The points of the two sets giving the proximity distance.</summary>
    public void ProximityPoints(out Point3D point1, out Point3D point2)
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

    //=======================================================================
    // function : calcEdgeRefinementStep
    // purpose  : Calculates the edge refinement step
    //=======================================================================
    private static double CalcEdgeRefinementStep(in Edge edge, int nbNodes)
    {
        if (nbNodes < 2)
        {
            return 0;
        }

        var bac = new EdgeCurveAdaptor(edge);
        double len = AbscissaPoint.Length(bac);
        return len / (double)(nbNodes - 1);
    }

    //=======================================================================
    // function : calcFaceRefinementStep
    // purpose  : Calculates the face refinement step as an approximate square
    // (Shape area / number triangles) * 2
    //=======================================================================
    private static double CalcFaceRefinementStep(in Face face, int nbTrg)
    {
        if (nbTrg < 1)
        {
            return 0;
        }

        var props = new Properties();
        MassPropertiesBuilder.SurfaceProperties(face, props);
        double area = props.Mass();
        return 2 * (area / (double)nbTrg);
    }

    //=======================================================================
    // function : getInfoForRefinement
    // purpose  : Gets shape data for further refinement
    //=======================================================================
    private static bool GetInfoForRefinement(in Shape shape,
                                             ref ShapeType shapeType,
                                             ref int nbNodes,
                                             ref double step)
    {
        if (shape.ShapeType() == ShapeType.Face)
        {
            shapeType = ShapeType.Face;
            Face f = ShapeCasts.Face(shape);

            Triangulation? triangulation = Tool.Triangulation(f, out _);

            if (triangulation is null)
            {
                return false;
            }

            nbNodes = triangulation.NodeCount;
            int nbTrg = triangulation.TriangleCount;
            step = CalcFaceRefinementStep(f, nbTrg);
        }
        else if (shape.ShapeType() == ShapeType.Edge)
        {
            shapeType = ShapeType.Edge;
            Edge e = ShapeCasts.Edge(shape);

            Polygon3D? polygon = Tool.Polygon3D(e, out _);

            if (polygon is null)
            {
                return false;
            }

            nbNodes = polygon.NodeCount;
            step = CalcEdgeRefinementStep(e, nbNodes);
        }
        else
        {
            return false;
        }

        if (step < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            return false;
        }

        return true;
    }

    /// <summary>Loads the lists of sub-shapes (copies, as OCCT assigns by value); the first of each is measured.</summary>
    public void LoadShapeLists(NCollection_DynamicArray<Shape> shapeList1, NCollection_DynamicArray<Shape> shapeList2)
    {
        _shapeList1 = new NCollection_DynamicArray<Shape>(shapeList1);
        _shapeList2 = new NCollection_DynamicArray<Shape>(shapeList2);

        _shape1 = shapeList1[0];
        _isInitS1 = GetInfoForRefinement(_shape1, ref _shapeType1, ref _nbNodes1, ref _step1);

        _shape2 = shapeList2[0];
        _isInitS2 = GetInfoForRefinement(_shape2, ref _shapeType2, ref _nbNodes2, ref _step2);

        MarkDirty();
    }

    /// <summary>The numbers of sample points of each shape; zero or less means every triangulation node.</summary>
    public void SetNbSamplePoints(int samples1 = 0, int samples2 = 0)
    {
        _nbSamples1 = samples1;
        _nbSamples2 = samples2;

        MarkDirty();
    }

    //=======================================================================
    // function : computeProximityValue
    // purpose  : Returns the computed proximity value from first BVH to another one
    //=======================================================================
    private static double ComputeProximityDist(TriangleSet set1,
                                               int nbSamples1,
                                               NCollection_LinearVector<BVH_Vec3d> addVertices1,
                                               NCollection_DynamicArray<ProxPnt_Status> addStatus1,
                                               TriangleSet set2,
                                               NCollection_DynamicArray<Shape> shapeList1,
                                               NCollection_DynamicArray<Shape> shapeList2,
                                               ref BVH_Vec3d point1,
                                               ref BVH_Vec3d point2,
                                               ref ProxPnt_Status pointStatus1,
                                               ref ProxPnt_Status pointStatus2)
    {
        var proxDistTool = new ProximityDistTool(set1,
                                                              nbSamples1,
                                                              addVertices1,
                                                              addStatus1,
                                                              set2,
                                                              shapeList1,
                                                              shapeList2);
        proxDistTool.Perform();

        if (!proxDistTool.IsDone)
        {
            return -1.0;
        }

        proxDistTool.ProximityPoints(out point1, out point2);
        proxDistTool.ProximityPointsStatus(out pointStatus1, out pointStatus2);

        return proxDistTool.ProximityDistance();
    }

    //=======================================================================
    // function : getEdgeAdditionalVertices
    // purpose  : Gets additional vertices and their statuses on the edge with the input step
    //=======================================================================
    private static bool GetEdgeAdditionalVertices(in Edge edge,
                                                  double step,
                                                  NCollection_LinearVector<BVH_Vec3d> addVertices,
                                                  NCollection_DynamicArray<ProxPnt_Status> addStatuses)
    {
        var bac = new EdgeCurveAdaptor(edge);

        if (!bac.Is3DCurve() || step < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            return false;
        }

        double len = AbscissaPoint.Length(bac);
        int nbSamplePoints = (int)(len / step) + 1;

        var gcPnts = new QuasiUniformAbscissa(bac, Max(3, nbSamplePoints));

        if (!gcPnts.IsDone)
        {
            return false;
        }

        int nbNodes = gcPnts.PointCount;
        for (int vertIdx = 2; vertIdx < nbNodes; ++vertIdx) // don't add extreme points
        {
            double par = gcPnts.Parameter(vertIdx);
            Point3D p = bac.Value(par);

            addVertices.Append(new BVH_Vec3d(p.X(), p.Y(), p.Z()));
            addStatuses.Append(ProxPnt_Status.ProxPnt_Status_MIDDLE);
        }

        return true;
    }

    //=======================================================================
    // function : doRecurTrgSplit
    // purpose  : Splits the triangle into two ones recursively, halving the longest side
    //           until the area of the current triangle > input step
    //=======================================================================
    private void DoRecurTrgSplit(ReadOnlySpan<Point3D> trg,
                                 ReadOnlySpan<ProxPnt_Status> edgesStatus,
                                 double tol,
                                 double step,
                                 NCollection_LinearVector<BVH_Vec3d> addVertices,
                                 NCollection_DynamicArray<ProxPnt_Status> addStatuses)
    {
        Xyz trgSide1 = trg[1].Coord() - trg[0].Coord();
        Xyz trgSide2 = trg[2].Coord() - trg[0].Coord();
        double trgArea = 0.5 * trgSide1.CrossMagnitude(trgSide2);

        if (trgArea - step < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            return;
        }

        ReadOnlySpan<double> d = [trg[0].Distance(trg[1]),
                                   trg[1].Distance(trg[2]),
                                   trg[2].Distance(trg[0])];

        int bisectedEdgeIdx = d[0] > d[1] ? (d[0] > d[2] ? 0 : 2) : (d[1] > d[2] ? 1 : 2);
        var centerOfMaxSide = new Point3D(trg[bisectedEdgeIdx].Coord());
        centerOfMaxSide.BaryCenter(0.5, trg[(bisectedEdgeIdx + 1) % 3], 0.5);

        var box = new Box();
        box.Add(centerOfMaxSide);
        box.Enlarge(tol);
        _inspector.SetCurrent(centerOfMaxSide.Coord());
        _cells.Inspect(box.CornerMin().XYZ(), box.CornerMax().XYZ(), ref _inspector);

        if (_inspector.IsNeedAdd) // is point aCenterOfMaxSide unique
        {
            var bisectingPnt = new BVH_Vec3d(centerOfMaxSide.X(), centerOfMaxSide.Y(), centerOfMaxSide.Z());
            addVertices.Append(bisectingPnt);
            addStatuses.Append(edgesStatus[bisectedEdgeIdx]);
            _inspector.Add(centerOfMaxSide.Coord());
            _cells.Add(addVertices.Size(), box.CornerMin().XYZ(), box.CornerMax().XYZ());
        }

        Span<Point3D> trg1 = [trg[0], trg[1], trg[2]];
        Span<Point3D> trg2 = [trg[0], trg[1], trg[2]];
        Span<ProxPnt_Status> edgesStatus1 = [edgesStatus[0], edgesStatus[1], edgesStatus[2]];
        Span<ProxPnt_Status> edgesStatus2 = [edgesStatus[0], edgesStatus[1], edgesStatus[2]];
        switch (bisectedEdgeIdx)
        {
            case 0:
                trg1[0] = trg2[1] = centerOfMaxSide;
                edgesStatus1[2] = edgesStatus2[1] = ProxPnt_Status.ProxPnt_Status_MIDDLE;
                break;
            case 1:
                trg1[1] = trg2[2] = centerOfMaxSide;
                edgesStatus1[0] = edgesStatus2[2] = ProxPnt_Status.ProxPnt_Status_MIDDLE;
                break;
            case 2:
                trg1[2] = trg2[0] = centerOfMaxSide;
                edgesStatus1[1] = edgesStatus2[0] = ProxPnt_Status.ProxPnt_Status_MIDDLE;
                break;
        }

        DoRecurTrgSplit(trg1, edgesStatus1, tol, step, addVertices, addStatuses);
        DoRecurTrgSplit(trg2, edgesStatus2, tol, step, addVertices, addStatuses);
    }

    private static double GetModelRange(in Placement location, Triangulation tr)
    {
        var box = new Box();
        tr.MinMax(ref box, location.Transformation());
        double aXm = 0.0, aYm = 0.0, aZm = 0.0, xm = 0.0, ym = 0.0, zm = 0.0;
        box.Get(out aXm, out aYm, out aZm, out xm, out ym, out zm);
        double range = xm - aXm;
        range = Max(range, ym - aYm);
        range = Max(range, zm - aZm);

        return range;
    }

    private static void GetNodesOfTrg(int triIdx, in Placement location, Triangulation tr, Span<Point3D> trg)
    {
        tr.Triangle(triIdx).Get(out int vtxIdx1, out int vtxIdx2, out int vtxIdx3);

        Point3D vtx1 = tr.Node(vtxIdx1);
        vtx1.Transform(location.Transformation());
        trg[0] = vtx1;

        Point3D vtx2 = tr.Node(vtxIdx2);
        vtx2.Transform(location.Transformation());
        trg[1] = vtx2;

        Point3D vtx3 = tr.Node(vtxIdx3);
        vtx3.Transform(location.Transformation());
        trg[2] = vtx3;
    }

    // Gets status of triangle edges - on the border or middle of the face
    private static void GetEdgesStatus(int triIdx, Triangulation tr, Span<ProxPnt_Status> edgesStatus1)
    {
        for (int j = 0; j < 3; j++)
        {
            int k = (j + 1) % 3;

            if (ProximityDistTool.IsEdgeOnBorder(triIdx, j, k, tr))
            {
                edgesStatus1[j] = ProxPnt_Status.ProxPnt_Status_BORDER;
            }
            else
            {
                edgesStatus1[j] = ProxPnt_Status.ProxPnt_Status_MIDDLE;
            }
        }
    }

    //=======================================================================
    // function : getFaceAdditionalVertices
    // purpose  : Gets additional vertices and their statuses on the face with the input step (triangle
    // square)
    //=======================================================================
    private bool GetFaceAdditionalVertices(in Face face,
                                           double step,
                                           NCollection_LinearVector<BVH_Vec3d> addVertices,
                                           NCollection_DynamicArray<ProxPnt_Status> addStatuses)
    {
        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();

        Triangulation? tr = Tool.Triangulation(face, out Placement location);

        if (tr is null)
        {
            return false;
        }

        _cells.Reset(Max(tol, GetModelRange(location, tr) / IntegerLast()));

        Span<Point3D> trg = stackalloc Point3D[3];
        Span<ProxPnt_Status> edgesStatus = stackalloc ProxPnt_Status[3];
        for (int triIdx = 1; triIdx <= tr.TriangleCount; ++triIdx)
        {
            GetNodesOfTrg(triIdx, location, tr, trg);
            GetEdgesStatus(triIdx, tr, edgesStatus);
            DoRecurTrgSplit(trg, edgesStatus, tol, step, addVertices, addStatuses);
        }

        return true;
    }

    //=======================================================================
    // function : getShapesVertices
    // purpose  : Gets additional vertices on shapes with refining a coarser one if it's needed
    //=======================================================================
    private bool GetShapesAdditionalVertices()
    {
        // estimate the density of meshes of shapes to add points to a coarcer one
        // target steps for refinement
        double step1 = _step1;
        double step2 = _step2;

        if ((_shapeType1 == ShapeType.Edge) && (_shapeType2 == ShapeType.Edge))
        {
            if (_nbSamples1 > _nbNodes1) // 1st edge needs refinement
            {
                step1 = CalcEdgeRefinementStep(ShapeCasts.Edge(_shape1), _nbSamples1);
                _isRefinementRequired1 = true;
            }

            if (_nbSamples2 > _nbNodes2) // 2nd edge needs refinement
            {
                step2 = CalcEdgeRefinementStep(ShapeCasts.Edge(_shape2), _nbSamples2);
                _isRefinementRequired2 = true;
            }

            if (step1 / step2 > 2.0) // 1st edge needs refinement
            {
                _isRefinementRequired1 = true;
                step1 = step2;
            }
            else if (step2 / step1 > 2.0) // 2nd edge needs refinement
            {
                _isRefinementRequired2 = true;
                step2 = step1;
            }

            if (_isRefinementRequired1)
            {
                if (!GetEdgeAdditionalVertices(ShapeCasts.Edge(_shape1), step1, _addVertices1, _addStatus1))
                {
                    return false;
                }
            }

            if (_isRefinementRequired2)
            {
                if (!GetEdgeAdditionalVertices(ShapeCasts.Edge(_shape2), step2, _addVertices2, _addStatus2))
                {
                    return false;
                }
            }
        }
        else if ((_shapeType1 == ShapeType.Face) && (_shapeType2 == ShapeType.Face))
        {
            if (step1 / step2 > 2) // 1st face needs refinement
            {
                _isRefinementRequired1 = true;
                step1 = _step2;
            }
            else if (step2 / step1 > 2.0) // 2nd face needs refinement
            {
                _isRefinementRequired2 = true;
                step2 = _step1;
            }

            if (_isRefinementRequired1)
            {
                return GetFaceAdditionalVertices(ShapeCasts.Face(_shape1), step1, _addVertices1, _addStatus1);
            }

            if (_isRefinementRequired2)
            {
                return GetFaceAdditionalVertices(ShapeCasts.Face(_shape2), step2, _addVertices2, _addStatus2);
            }
        }

        return true;
    }

    //=======================================================================
    // function : Perform
    // purpose  : Performs the computation of the proximity value
    //=======================================================================
    /// <summary>Computes the proximity value; when done, <paramref name="tolerance"/> receives it.</summary>
    public void Perform(ref double tolerance)
    {
        if (!_isInitS1 || !_isInitS2 || (_shapeType1 != _shapeType2))
        {
            return;
        }

        // get vertices on shapes with refining a coarser mesh if it's needed
        if (!GetShapesAdditionalVertices())
        {
            return;
        }

        // max(min) dist from the 1st shape to the 2nd one
        BVH_Vec3d p1_1 = default, p1_2 = default;
        ProxPnt_Status pointStatus1_1 = ProxPnt_Status.ProxPnt_Status_UNKNOWN;
        ProxPnt_Status pointStatus1_2 = ProxPnt_Status.ProxPnt_Status_UNKNOWN;

        double proximityDist1 = ComputeProximityDist(_set1!,
                                                      _nbSamples1,
                                                      _addVertices1,
                                                      _addStatus1,
                                                      _set2!,
                                                      _shapeList1,
                                                      _shapeList2,
                                                      ref p1_1,
                                                      ref p1_2,
                                                      ref pointStatus1_1,
                                                      ref pointStatus1_2);

        if (proximityDist1 < 0.0)
        {
            return;
        }

        // max(min) dist from the 2nd shape to t he 1st one
        BVH_Vec3d p2_1 = default, p2_2 = default;
        ProxPnt_Status pointStatus2_1 = ProxPnt_Status.ProxPnt_Status_UNKNOWN;
        ProxPnt_Status pointStatus2_2 = ProxPnt_Status.ProxPnt_Status_UNKNOWN;

        double proximityDist2 = ComputeProximityDist(_set2!,
                                                      _nbSamples2,
                                                      _addVertices2,
                                                      _addStatus2,
                                                      _set1!,
                                                      _shapeList2,
                                                      _shapeList1,
                                                      ref p2_2,
                                                      ref p2_1,
                                                      ref pointStatus2_2,
                                                      ref pointStatus2_1);

        if (proximityDist2 < 0.0)
        {
            return;
        }

        // min dist of the two max(min) dists
        if (proximityDist1 < proximityDist2)
        {
            _distance = proximityDist1;
            _pnt1 = new Point3D(p1_1.X(), p1_1.Y(), p1_1.Z());
            _pnt2 = new Point3D(p1_2.X(), p1_2.Y(), p1_2.Z());
            _pntStatus1 = pointStatus1_1;
            _pntStatus2 = pointStatus1_2;
        }
        else
        {
            _distance = proximityDist2;
            _pnt1 = new Point3D(p2_1.X(), p2_1.Y(), p2_1.Z());
            _pnt2 = new Point3D(p2_2.X(), p2_2.Y(), p2_2.Z());
            _pntStatus1 = pointStatus2_1;
            _pntStatus2 = pointStatus2_2;
        }

        _isDone = true;
        tolerance = _distance;
    }
}
