// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_SelfIntersection.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_SelfIntersection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2014-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Occt;
using BVH_Vec3d = Occt.NCollection_Vec3<double>;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Measurement.Distance;

/// <summary>
/// Detects self-intersections of a shape from its tessellation: pairs of overlapping triangles of different faces.
/// The result depends on the mesh (a mesh whose adjacent faces don't share vertices on their common edge gives false
/// overlaps), so this is a fast approximate test giving the faces that may overlap.
/// </summary>
/// <remarks>A working object (decision 21). It only reads the shape's triangulations.</remarks>
public class SelfIntersection : ElementFilter
{
    private double _tolerance;
    private bool _isInit;
    private readonly NCollection_DynamicArray<Shape> _faceList = new();
    private TriangleSet? _elementSet;
    private readonly OverlapTool _overlapTool = new();

    /// <summary>An uninitialized self-intersection tool.</summary>
    public SelfIntersection(double tolerance = 0.0)
    {
        _tolerance = tolerance;
        _isInit = false;
    }

    /// <summary>A self-intersection tool for the shape.</summary>
    public SelfIntersection(in Shape shape, double tolerance = 0.0)
    {
        _tolerance = tolerance;
        LoadShape(shape);
    }

    /// <summary>The tolerance of the self-intersection test.</summary>
    public double Tolerance() => _tolerance;

    /// <summary>Sets the tolerance of the self-intersection test.</summary>
    public void SetTolerance(double tolerance) => _tolerance = tolerance;

    /// <summary>True if the detection is completed.</summary>
    public bool IsDone => _overlapTool.IsDone;

    /// <summary>The overlapped faces (indices from 0), each with the faces it overlaps.</summary>
    public NCollection_DataMap<int, NCollection_PackedMapOfInteger> OverlapElements() => _overlapTool.OverlapSubShapes1();

    /// <summary>The face of the shape with the given index (from 0).</summary>
    public Face GetSubShape(int id) => ShapeCasts.Face(_faceList.Value(id));

    /// <summary>The set of all the face triangles of the shape.</summary>
    public TriangleSet? ElementSet() => _elementSet;

    /// <summary>Loads the shape.</summary>
    public bool LoadShape(in Shape shape)
    {
        _faceList.Clear();

        for (var iter = new Explorer(shape, ShapeType.Face); iter.More(); iter.Next())
        {
            _faceList.Append(ShapeCasts.Face(iter.Current()));
        }

        _elementSet ??= new TriangleSet();

        _isInit = _elementSet.Init(_faceList);

        if (_isInit)
        {
            _overlapTool.LoadTriangleSets(_elementSet, _elementSet);
        }

        return _isInit;
    }

    private static readonly BVH_Vec3d ZERO_VEC = new(0.0, 0.0, 0.0);

    // =======================================================================
    // function : ccw
    // purpose  : Check if triple is in counterclockwise order
    // =======================================================================
    private static bool Ccw(in BVH_Vec3d vertex0, in BVH_Vec3d vertex1, in BVH_Vec3d vertex2, int x, int y)
    {
        double sum =
            (vertex1[x] - vertex0[x]) * (vertex1[y] + vertex0[y])
            + (vertex2[x] - vertex1[x]) * (vertex2[y] + vertex1[y])
            + (vertex0[x] - vertex2[x]) * (vertex0[y] + vertex2[y]);

        return sum < 0.0;
    }

    // =======================================================================
    // function : rayInsideAngle
    // purpose  : Check the given ray is inside the angle
    // =======================================================================
    private static bool RayInsideAngle(in BVH_Vec3d direc, in BVH_Vec3d edge0, in BVH_Vec3d edge1, int x, int y)
    {
        bool ccw = Ccw(ZERO_VEC, edge0, edge1, x, y);

        return Ccw(ZERO_VEC, edge0, direc, x, y) == ccw
               && Ccw(ZERO_VEC, direc, edge1, x, y) == ccw;
    }

    private static void GetProjectionAxes(in BVH_Vec3d norm, out int axisX, out int axisY)
    {
        if (Math.Abs(norm[0]) > Math.Abs(norm[1]))
        {
            axisX = Math.Abs(norm[0]) > Math.Abs(norm[2]) ? 1 : 0;
            axisY = Math.Abs(norm[0]) > Math.Abs(norm[2]) ? 2 : 1;
        }
        else
        {
            axisX = Math.Abs(norm[1]) > Math.Abs(norm[2]) ? 0 : 0;
            axisY = Math.Abs(norm[1]) > Math.Abs(norm[2]) ? 2 : 1;
        }
    }

    /// <summary>Whether two triangles sharing only one vertex overlap there.</summary>
    protected FilterResult IsRegularSharedVertex(in BVH_Vec3d sharedVert,
                                                 in BVH_Vec3d trng0Vtxs1,
                                                 in BVH_Vec3d trng0Vtxs2,
                                                 in BVH_Vec3d trng1Vtxs1,
                                                 in BVH_Vec3d trng1Vtxs2)
    {
        ReadOnlySpan<BVH_Vec3d> trng0Edges = [(trng0Vtxs1 - sharedVert).Normalized(),
                                               (trng0Vtxs2 - sharedVert).Normalized()];

        ReadOnlySpan<BVH_Vec3d> trng1Edges = [(trng1Vtxs1 - sharedVert).Normalized(),
                                               (trng1Vtxs2 - sharedVert).Normalized()];

        BVH_Vec3d trng0Normal = BVH_Vec3d.Cross(trng0Edges[0], trng0Edges[1]);
        BVH_Vec3d trng1Normal = BVH_Vec3d.Cross(trng1Edges[0], trng1Edges[1]);

        BVH_Vec3d crossLine = BVH_Vec3d.Cross(trng0Normal, trng1Normal);

        int x;
        int y;

        if (crossLine.SquareModulus() < global::Kerf.Engine.Foundation.Precision.SquareConfusion()) // coplanar case
        {
            GetProjectionAxes(trng0Normal, out x, out y);

            if (RayInsideAngle(trng1Edges[0], trng0Edges[0], trng0Edges[1], x, y)
                || RayInsideAngle(trng1Edges[1], trng0Edges[0], trng0Edges[1], x, y)
                || RayInsideAngle(trng0Edges[0], trng1Edges[0], trng1Edges[1], x, y)
                || RayInsideAngle(trng0Edges[1], trng1Edges[0], trng1Edges[1], x, y))
            {
                return FilterResult.Overlap;
            }

            return FilterResult.NoCheck;
        }
        else // shared line should lie outside at least one triangle
        {
            GetProjectionAxes(trng0Normal, out x, out y);

            bool posOutTrgn0 = !RayInsideAngle(crossLine, trng0Edges[0], trng0Edges[1], x, y);

            bool negOutTrgn0 = !RayInsideAngle(-crossLine, trng0Edges[0], trng0Edges[1], x, y);

            // Standard_ASSERT_RAISE
            if (!(posOutTrgn0 || negOutTrgn0))
            {
                throw new ProgramErrorException("Failed to detect if shared vertex is regular or not");
            }

            if (posOutTrgn0 && negOutTrgn0)
            {
                return FilterResult.NoCheck;
            }

            GetProjectionAxes(trng1Normal, out x, out y);

            bool posOutTrgn1 = !RayInsideAngle(crossLine, trng1Edges[0], trng1Edges[1], x, y);

            bool negOutTrgn1 = !RayInsideAngle(-crossLine, trng1Edges[0], trng1Edges[1], x, y);

            // Standard_ASSERT_RAISE
            if (!(posOutTrgn1 || negOutTrgn1))
            {
                throw new ProgramErrorException("Failed to detect if shared vertex is regular or not");
            }

            if (posOutTrgn1 && negOutTrgn1)
            {
                return FilterResult.NoCheck;
            }

            return (posOutTrgn0 || posOutTrgn1) && (negOutTrgn0 || negOutTrgn1)
                       ? FilterResult.NoCheck
                       : FilterResult.Overlap;
        }
    }

    /// <summary>Whether two triangles sharing only one edge overlap there.</summary>
    protected FilterResult IsRegularSharedEdge(in BVH_Vec3d trng0Vtxs0,
                                               in BVH_Vec3d trng0Vtxs1,
                                               in BVH_Vec3d trng0Vtxs2,
                                               in BVH_Vec3d trng1Vtxs2)
    {
        BVH_Vec3d sharedEdge = (trng0Vtxs1 - trng0Vtxs0).Normalized();

        ReadOnlySpan<BVH_Vec3d> uniqueEdges = [(trng0Vtxs2 - trng0Vtxs0).Normalized(),
                                                (trng1Vtxs2 - trng0Vtxs0).Normalized()];

        BVH_Vec3d trng0Normal = BVH_Vec3d.Cross(sharedEdge, uniqueEdges[0]);
        BVH_Vec3d trng1Normal = BVH_Vec3d.Cross(sharedEdge, uniqueEdges[1]);

        BVH_Vec3d crossLine = BVH_Vec3d.Cross(trng0Normal, trng1Normal);

        if (crossLine.SquareModulus() > global::Kerf.Engine.Foundation.Precision.SquareConfusion()) // non-coplanar case
        {
            return FilterResult.NoCheck;
        }

        GetProjectionAxes(trng0Normal, out int x, out int y);

        return Ccw(ZERO_VEC, sharedEdge, uniqueEdges[0], x, y)
               != Ccw(ZERO_VEC, sharedEdge, uniqueEdges[1], x, y)
                   ? FilterResult.NoCheck
                   : FilterResult.Overlap;
    }

    /// <summary>Filters out correct adjacent mesh elements.</summary>
    public override FilterResult PreCheckElements(int index1, int index2)
    {
        if (_elementSet!.GetFaceID(index1) == _elementSet.GetFaceID(index2))
        {
            return FilterResult.NoCheck; // triangles are from the same face
        }

        Span<BVH_Vec3d> trng0Vtxs = stackalloc BVH_Vec3d[3];
        Span<BVH_Vec3d> trng1Vtxs = stackalloc BVH_Vec3d[3];

        _elementSet.GetVertices(index1, out trng0Vtxs[0], out trng0Vtxs[1], out trng0Vtxs[2]);
        _elementSet.GetVertices(index2, out trng1Vtxs[0], out trng1Vtxs[1], out trng1Vtxs[2]);

        Span<(int first, int second)> sharedVtxs = stackalloc (int, int)[3];
        int sharedVtxsSize = 0;

        for (int vertIdx1 = 0; vertIdx1 < 3; ++vertIdx1)
        {
            for (int vertIdx2 = 0; vertIdx2 < 3; ++vertIdx2)
            {
                if ((trng0Vtxs[vertIdx1] - trng1Vtxs[vertIdx2]).SquareModulus() < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                {
                    sharedVtxs[sharedVtxsSize++] = (vertIdx1, vertIdx2);

                    break; // go to next vertex of the 1st triangle
                }
            }
        }

        if (sharedVtxsSize == 2) // check shared edge
        {
            return IsRegularSharedEdge(trng0Vtxs[sharedVtxs[0].first],
                                       trng0Vtxs[sharedVtxs[1].first],
                                       trng0Vtxs[3 - sharedVtxs[0].first - sharedVtxs[1].first],
                                       trng1Vtxs[3 - sharedVtxs[0].second - sharedVtxs[1].second]);
        }
        else if (sharedVtxsSize == 1) // check shared vertex
        {
            (trng0Vtxs[0], trng0Vtxs[sharedVtxs[0].first]) = (trng0Vtxs[sharedVtxs[0].first], trng0Vtxs[0]);
            (trng1Vtxs[0], trng1Vtxs[sharedVtxs[0].second]) = (trng1Vtxs[sharedVtxs[0].second], trng1Vtxs[0]);

            return IsRegularSharedVertex(trng0Vtxs[0], trng0Vtxs[1], trng0Vtxs[2], trng1Vtxs[1], trng1Vtxs[2]);
        }

        return FilterResult.DoCheck;
    }

    /// <summary>Detects the self-intersections (once: nothing is done again until the shape is reloaded).</summary>
    public void Perform()
    {
        if (!_isInit || _overlapTool.IsDone)
        {
            return;
        }

        _overlapTool.SetElementFilter(this);

        _overlapTool.Perform(_tolerance);
    }
}
