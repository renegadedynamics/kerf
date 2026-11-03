// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_OverlapTool.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_OverlapTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2015-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Geometry.Bvh;
using Occt;
using BVH_Vec2d = Occt.NCollection_Vec2<double>;
using BVH_Vec3d = Occt.NCollection_Vec3<double>;

namespace Kerf.Engine.Measurement.Distance;

/// <summary>
/// Detects overlapping of two BVH primitive sets (triangle sets): the pairs of faces whose triangles intersect, or
/// come within a tolerance of each other. The two sets may be the same, to find self-intersections. Results are
/// approximate and depend on the quality of the tessellations.
/// </summary>
/// <remarks>
/// A working object (decision 21), used by <see cref="ShapeProximity"/> and
/// <see cref="SelfIntersection"/>; the result maps are its own. OCCT's debug output of the overlapped
/// triangles (OVERLAP_TOOL_OUTPUT_TRIANGLES) is compiled out, as in OCCT.
/// </remarks>
public class OverlapTool : PairTraverse<double, BVH_Vec3d, double>
{
    private TriangleSet? _set1;
    private TriangleSet? _set2;
    private ElementFilter? _filter;
    private readonly Dictionary<int, NCollection_PackedMapOfInteger> _overlapSubShapes1 = new();
    private readonly Dictionary<int, NCollection_PackedMapOfInteger> _overlapSubShapes2 = new();
    private bool _isDone;
    private double _tolerance;

    /// <summary>An uninitialized overlap tool.</summary>
    public OverlapTool()
    {
        _filter = null;
        _tolerance = 0.0;
        _isDone = false;
    }

    /// <summary>An overlap tool for the given element sets.</summary>
    public OverlapTool(TriangleSet set1, TriangleSet set2)
    {
        _filter = null;
        _tolerance = 0.0;
        LoadTriangleSets(set1, set2);
    }

    /// <summary>Loads the element sets.</summary>
    public void LoadTriangleSets(TriangleSet set1, TriangleSet set2)
    {
        _set1 = set1;
        _set2 = set2;

        // OCCT: the result maps aren't cleared, so a second Perform adds to the first one's pairs
        _isDone = false;
    }

    /// <summary>True if the overlap test is completed.</summary>
    public bool IsDone => _isDone;

    /// <summary>Marks the results as outdated.</summary>
    public void MarkDirty() => _isDone = false;

    /// <summary>The overlapped sub-shapes (faces) of the first set, each with the faces of the second it overlaps.</summary>
    public Dictionary<int, NCollection_PackedMapOfInteger> OverlapSubShapes1() => _overlapSubShapes1;

    /// <summary>The overlapped sub-shapes (faces) of the second set, each with the faces of the first it overlaps.</summary>
    public Dictionary<int, NCollection_PackedMapOfInteger> OverlapSubShapes2() => _overlapSubShapes2;

    /// <summary>Sets the filter for preliminary checking of pairs of mesh elements.</summary>
    public void SetElementFilter(ElementFilter? filter) => _filter = filter;

    //=================================================================================================

    /// <summary>A bounding triangular prism of a triangle.</summary>
    private struct BRepExtrema_BoundingPrism
    {
        public BVH_Vec3d[] Vertices;
        public BVH_Vec3d[] Edges;
        public BVH_Vec3d Normal;
        public BVH_Vec3d[] EdgeNormals;
        public bool IsInited;

        public BRepExtrema_BoundingPrism()
        {
            Vertices = new BVH_Vec3d[6];
            Edges = new BVH_Vec3d[3];
            EdgeNormals = new BVH_Vec3d[3];
            IsInited = false;
        }

        public BRepExtrema_BoundingPrism(in BVH_Vec3d vertex0, in BVH_Vec3d vertex1, in BVH_Vec3d vertex2, double deflect)
            : this()
        {
            Init(vertex0, vertex1, vertex2, deflect);
        }

        public void Init(in BVH_Vec3d vertex0, in BVH_Vec3d vertex1, in BVH_Vec3d vertex2, double deflect)
        {
            Edges[0] = vertex1 - vertex0;
            Edges[1] = vertex2 - vertex0;
            Edges[2] = vertex2 - vertex1;

            Normal = BVH_Vec3d.Cross(Edges[0], Edges[1]);

            EdgeNormals[0] = BVH_Vec3d.Cross(Edges[0], Normal);
            EdgeNormals[1] = BVH_Vec3d.Cross(Edges[1], Normal);
            EdgeNormals[2] = BVH_Vec3d.Cross(Edges[2], Normal);

            EdgeNormals[0] = EdgeNormals[0] * (1.0 / Max(EdgeNormals[0].Modulus(), global::Kerf.Engine.Foundation.Precision.Confusion()));
            EdgeNormals[1] = EdgeNormals[1] * (1.0 / Max(EdgeNormals[1].Modulus(), global::Kerf.Engine.Foundation.Precision.Confusion()));
            EdgeNormals[2] = EdgeNormals[2] * (1.0 / Max(EdgeNormals[2].Modulus(), global::Kerf.Engine.Foundation.Precision.Confusion()));

            BVH_Vec3d direct01 = EdgeNormals[0] - EdgeNormals[1];
            BVH_Vec3d direct02 = EdgeNormals[0] + EdgeNormals[2];
            BVH_Vec3d direct12 = EdgeNormals[2] - EdgeNormals[1];

            Vertices[0] = Vertices[3] = vertex0 + direct01 * (deflect / direct01.Dot(EdgeNormals[0]));
            Vertices[1] = Vertices[4] = vertex1 + direct02 * (deflect / direct02.Dot(EdgeNormals[2]));
            Vertices[2] = Vertices[5] = vertex2 + direct12 * (deflect / direct12.Dot(EdgeNormals[2]));

            BVH_Vec3d normOffset = Normal * (deflect / Max(Normal.Modulus(), global::Kerf.Engine.Foundation.Precision.Confusion()));

            for (int vertIdx = 0; vertIdx < 3; ++vertIdx)
            {
                Vertices[vertIdx + 0] = Vertices[vertIdx + 0] + normOffset;
                Vertices[vertIdx + 3] = Vertices[vertIdx + 3] - normOffset;
            }

            IsInited = true;
        }

        /// <summary>True if the two prisms are separated along the axis.</summary>
        public readonly bool Separated(in BRepExtrema_BoundingPrism prism, in BVH_Vec3d axis)
        {
            double min1 = double.MaxValue;
            double max1 = -double.MaxValue;

            double min2 = double.MaxValue;
            double max2 = -double.MaxValue;

            for (int vertIdx = 0; vertIdx < 6; ++vertIdx)
            {
                double proj1 = Vertices[vertIdx].Dot(axis);

                min1 = Min(min1, proj1);
                max1 = Max(max1, proj1);

                double proj2 = prism.Vertices[vertIdx].Dot(axis);

                min2 = Min(min2, proj2);
                max2 = Max(max2, proj2);

                if (min1 <= max2 && max1 >= min2)
                {
                    return false;
                }
            }

            return min1 > max2 || max1 < min2;
        }
    }

    private static double Sign(in BVH_Vec3d vertex0, in BVH_Vec3d vertex1, in BVH_Vec3d vertex2, int x, int y)
        => (vertex0[x] - vertex2[x]) * (vertex1[y] - vertex2[y])
           - (vertex1[x] - vertex2[x]) * (vertex0[y] - vertex2[y]);

    private static bool PointInTriangle(in BVH_Vec3d testPnt,
                                        in BVH_Vec3d trgVtx0,
                                        in BVH_Vec3d trgVtx1,
                                        in BVH_Vec3d trgVtx2,
                                        int x,
                                        int y)
    {
        bool sign0 = global::Kerf.Engine.Measurement.Distance.OverlapTool.Sign(testPnt, trgVtx0, trgVtx1, x, y) <= 0.0;
        bool sign1 = global::Kerf.Engine.Measurement.Distance.OverlapTool.Sign(testPnt, trgVtx1, trgVtx2, x, y) <= 0.0;
        bool sign2 = global::Kerf.Engine.Measurement.Distance.OverlapTool.Sign(testPnt, trgVtx2, trgVtx0, x, y) <= 0.0;

        return (sign0 == sign1) && (sign1 == sign2);
    }

    // =======================================================================
    // function : segmentsIntersected
    // purpose  : Checks if two line segments are intersected
    // =======================================================================
    private static bool SegmentsIntersected(in BVH_Vec2d originSeg0,
                                            in BVH_Vec2d originSeg1,
                                            in BVH_Vec2d directSeg0,
                                            in BVH_Vec2d directSeg1)
    {
        double det = -directSeg1.X() * directSeg0.Y() + directSeg0.X() * directSeg1.Y();

        if (Math.Abs(det) < DBL_EPSILON) // segments are parallel
        {
            BVH_Vec2d direct = directSeg0 * (1.0 / directSeg0.Modulus());

            double edge0Time0 = originSeg0.Dot(direct);
            double edge1Time0 = originSeg1.Dot(direct);

            double edge0Time1 = edge0Time0 + directSeg0.Dot(direct);
            double edge1Time1 = edge1Time0 + directSeg1.Dot(direct);

            double edge0Min = Min(edge0Time0, edge0Time1);
            double edge1Min = Min(edge1Time0, edge1Time1);
            double edge0Max = Max(edge0Time0, edge0Time1);
            double edge1Max = Max(edge1Time0, edge1Time1);

            if (Max(edge0Min, edge1Min) > Min(edge0Max, edge1Max))
            {
                return false;
            }

            var normal = new BVH_Vec2d(-direct.Y(), direct.X());

            return Math.Abs(originSeg0.Dot(normal) - originSeg1.Dot(normal)) < DBL_EPSILON;
        }

        BVH_Vec2d delta = originSeg0 - originSeg1;

        double u = (-directSeg0.Y() * delta.X() + directSeg0.X() * delta.Y()) / det;
        double v = (directSeg1.X() * delta.Y() - directSeg1.Y() * delta.X()) / det;

        return u >= 0.0 && u <= 1.0 && v >= 0.0 && v <= 1.0;
    }

    private const double DBL_EPSILON = 2.220446049250313E-16;

    // =======================================================================
    // function : trianglesIntersected
    // purpose  : Checks if two triangles are intersected
    //            ("A Fast Triangle-Triangle Intersection Test" by T. Moller)
    // =======================================================================
    private static bool TrianglesIntersected(in BVH_Vec3d trng0Vert0,
                                             in BVH_Vec3d trng0Vert1,
                                             in BVH_Vec3d trng0Vert2,
                                             in BVH_Vec3d trng1Vert0,
                                             in BVH_Vec3d trng1Vert1,
                                             in BVH_Vec3d trng1Vert2)
    {
        BVH_Vec3d trng1Normal = BVH_Vec3d.Cross(trng1Vert1 - trng1Vert0, trng1Vert2 - trng1Vert0).Normalized();

        double trng1PlaneDist = trng1Normal.Dot(-trng1Vert0);

        double distTrng0Vert0 = trng1Normal.Dot(trng0Vert0) + trng1PlaneDist;
        double distTrng0Vert1 = trng1Normal.Dot(trng0Vert1) + trng1PlaneDist;
        double distTrng0Vert2 = trng1Normal.Dot(trng0Vert2) + trng1PlaneDist;

        if ((distTrng0Vert0 < 0.0 && distTrng0Vert1 < 0.0 && distTrng0Vert2 < 0.0)
            || (distTrng0Vert0 > 0.0 && distTrng0Vert1 > 0.0 && distTrng0Vert2 > 0.0))
        {
            return false; // 1st triangle lies on one side of the 2nd triangle
        }

        if (Math.Abs(distTrng0Vert0) > global::Kerf.Engine.Foundation.Precision.Confusion()
            || Math.Abs(distTrng0Vert1) > global::Kerf.Engine.Foundation.Precision.Confusion()
            || Math.Abs(distTrng0Vert2) > global::Kerf.Engine.Foundation.Precision.Confusion()) // general 3D case
        {
            BVH_Vec3d trng0Normal = BVH_Vec3d.Cross(trng0Vert1 - trng0Vert0, trng0Vert2 - trng0Vert0).Normalized();

            double trng0PlaneDist = trng0Normal.Dot(-trng0Vert0);

            double distTrng1Vert0 = trng0Normal.Dot(trng1Vert0) + trng0PlaneDist;
            double distTrng1Vert1 = trng0Normal.Dot(trng1Vert1) + trng0PlaneDist;
            double distTrng1Vert2 = trng0Normal.Dot(trng1Vert2) + trng0PlaneDist;

            if ((distTrng1Vert0 < 0.0 && distTrng1Vert1 < 0.0 && distTrng1Vert2 < 0.0)
                || (distTrng1Vert0 > 0.0 && distTrng1Vert1 > 0.0 && distTrng1Vert2 > 0.0))
            {
                return false; // 2nd triangle lies on one side of the 1st triangle
            }

            BVH_Vec3d crossLine = BVH_Vec3d.Cross(trng0Normal, trng1Normal);

            double projTrng0Vert0 = trng0Vert0.Dot(crossLine);
            double projTrng0Vert1 = trng0Vert1.Dot(crossLine);
            double projTrng0Vert2 = trng0Vert2.Dot(crossLine);

            if (distTrng0Vert0 * distTrng0Vert1 > 0.0)
            {
                (distTrng0Vert1, distTrng0Vert2) = (distTrng0Vert2, distTrng0Vert1);
                (projTrng0Vert1, projTrng0Vert2) = (projTrng0Vert2, projTrng0Vert1);
            }
            else if (distTrng0Vert1 * distTrng0Vert2 > 0.0)
            {
                (distTrng0Vert1, distTrng0Vert0) = (distTrng0Vert0, distTrng0Vert1);
                (projTrng0Vert1, projTrng0Vert0) = (projTrng0Vert0, projTrng0Vert1);
            }

            double time1 = Math.Abs(distTrng0Vert0) <= DBL_EPSILON
                                ? projTrng0Vert0
                                : projTrng0Vert0
                                      + (projTrng0Vert1 - projTrng0Vert0) * distTrng0Vert0
                                            / (distTrng0Vert0 - distTrng0Vert1);
            double time2 = Math.Abs(distTrng0Vert2) <= DBL_EPSILON
                                ? projTrng0Vert2
                                : projTrng0Vert2
                                      + (projTrng0Vert1 - projTrng0Vert2) * distTrng0Vert2
                                            / (distTrng0Vert2 - distTrng0Vert1);

            double timeMin1 = Min(time1, time2);
            double timeMax1 = Max(time1, time2);

            double projTrng1Vert0 = trng1Vert0.Dot(crossLine);
            double projTrng1Vert1 = trng1Vert1.Dot(crossLine);
            double projTrng1Vert2 = trng1Vert2.Dot(crossLine);

            if (distTrng1Vert0 * distTrng1Vert1 > 0.0)
            {
                (distTrng1Vert1, distTrng1Vert2) = (distTrng1Vert2, distTrng1Vert1);
                (projTrng1Vert1, projTrng1Vert2) = (projTrng1Vert2, projTrng1Vert1);
            }
            else if (distTrng1Vert1 * distTrng1Vert2 > 0.0)
            {
                (distTrng1Vert1, distTrng1Vert0) = (distTrng1Vert0, distTrng1Vert1);
                (projTrng1Vert1, projTrng1Vert0) = (projTrng1Vert0, projTrng1Vert1);
            }

            time1 = Math.Abs(distTrng1Vert0) <= DBL_EPSILON
                         ? projTrng1Vert0
                         : projTrng1Vert0
                               + (projTrng1Vert1 - projTrng1Vert0) * distTrng1Vert0
                                     / (distTrng1Vert0 - distTrng1Vert1);
            time2 = Math.Abs(distTrng1Vert2) <= DBL_EPSILON
                         ? projTrng1Vert2
                         : projTrng1Vert2
                               + (projTrng1Vert1 - projTrng1Vert2) * distTrng1Vert2
                                     / (distTrng1Vert2 - distTrng1Vert1);

            double timeMin2 = Min(time1, time2);
            double timeMax2 = Max(time1, time2);

            time1 = Max(timeMin1, timeMin2);
            time2 = Min(timeMax1, timeMax2);

            return time1 <= time2; // intervals intersected --> triangles overlapped
        }
        else // triangles are co-planar
        {
            int x;
            int y;

            if (Math.Abs(trng1Normal[0]) > Math.Abs(trng1Normal[1]))
            {
                x = Math.Abs(trng1Normal[0]) > Math.Abs(trng1Normal[2]) ? 1 : 0;
                y = Math.Abs(trng1Normal[0]) > Math.Abs(trng1Normal[2]) ? 2 : 1;
            }
            else
            {
                x = Math.Abs(trng1Normal[1]) > Math.Abs(trng1Normal[2]) ? 0 : 0;
                y = Math.Abs(trng1Normal[1]) > Math.Abs(trng1Normal[2]) ? 2 : 1;
            }

            ReadOnlySpan<BVH_Vec2d> originSeg0 = [new BVH_Vec2d(trng0Vert0[x], trng0Vert0[y]),
                                                   new BVH_Vec2d(trng0Vert1[x], trng0Vert1[y]),
                                                   new BVH_Vec2d(trng0Vert2[x], trng0Vert2[y])];

            ReadOnlySpan<BVH_Vec2d> directSeg0 = [originSeg0[1] - originSeg0[0],
                                                   originSeg0[2] - originSeg0[1],
                                                   originSeg0[0] - originSeg0[2]];

            ReadOnlySpan<BVH_Vec2d> originSeg1 = [new BVH_Vec2d(trng1Vert0[x], trng1Vert0[y]),
                                                   new BVH_Vec2d(trng1Vert1[x], trng1Vert1[y]),
                                                   new BVH_Vec2d(trng1Vert2[x], trng1Vert2[y])];

            ReadOnlySpan<BVH_Vec2d> directSeg1 = [originSeg1[1] - originSeg1[0],
                                                   originSeg1[2] - originSeg1[1],
                                                   originSeg1[0] - originSeg1[2]];

            for (int trg0Edge = 0; trg0Edge < 3; ++trg0Edge)
            {
                for (int trg1Edge = 0; trg1Edge < 3; ++trg1Edge)
                {
                    if (SegmentsIntersected(originSeg0[trg0Edge],
                                            originSeg1[trg1Edge],
                                            directSeg0[trg0Edge],
                                            directSeg1[trg1Edge]))
                    {
                        return true; // edges intersected --> triangles overlapped
                    }
                }
            }

            if (PointInTriangle(trng1Vert0, trng0Vert0, trng0Vert1, trng0Vert2, x, y))
            {
                return true; // 1st triangle inside 2nd --> triangles overlapped
            }

            if (PointInTriangle(trng0Vert0, trng1Vert0, trng1Vert1, trng1Vert2, x, y))
            {
                return true; // 2nd triangle inside 1st --> triangles overlapped
            }
        }

        return false;
    }

    // =======================================================================
    // function : prismsIntersected
    // purpose  : Checks if two triangular prisms are intersected
    //            (test uses SAT - Separating Axis Theorem)
    // =======================================================================
    private static bool PrismsIntersected(in BRepExtrema_BoundingPrism prism1, in BRepExtrema_BoundingPrism prism2)
    {
        if (prism1.Separated(prism2, prism1.Normal))
        {
            return false;
        }

        if (prism1.Separated(prism2, prism2.Normal))
        {
            return false;
        }

        for (int idx = 0; idx < 3; ++idx)
        {
            if (prism1.Separated(prism2, prism1.EdgeNormals[idx]))
            {
                return false;
            }
        }

        for (int idx = 0; idx < 3; ++idx)
        {
            if (prism1.Separated(prism2, prism2.EdgeNormals[idx]))
            {
                return false;
            }
        }

        for (int idx1 = 0; idx1 < 4; ++idx1)
        {
            BVH_Vec3d edge1 = (idx1 == 3) ? prism1.Normal : prism1.Edges[idx1];

            for (int idx2 = 0; idx2 < 4; ++idx2)
            {
                BVH_Vec3d edge2 = (idx2 == 3) ? prism2.Normal : prism2.Edges[idx2];

                if (prism1.Separated(prism2, BVH_Vec3d.Cross(edge1, edge2)))
                {
                    return false;
                }
            }
        }

        return true;
    }

    // =======================================================================
    // function : overlapBoxes
    // purpose  : Checks if two boxes (AABBs) are overlapped
    // =======================================================================
    private static bool OverlapBoxes(in BVH_Vec3d boxMin1,
                                     in BVH_Vec3d boxMax1,
                                     in BVH_Vec3d boxMin2,
                                     in BVH_Vec3d boxMax2,
                                     double tolerance)
    {
        // Check for overlap
        return boxMin1.X() <= boxMax2.X() + tolerance
               && boxMax1.X() >= boxMin2.X() - tolerance
               && boxMin1.Y() <= boxMax2.Y() + tolerance
               && boxMax1.Y() >= boxMin2.Y() - tolerance
               && boxMin1.Z() <= boxMax2.Z() + tolerance
               && boxMax1.Z() >= boxMin2.Z() - tolerance;
    }

    private static NCollection_PackedMapOfInteger GetSetOfFaces(Dictionary<int, NCollection_PackedMapOfInteger> faces,
                                                                int faceIdx)
    {
        if (!faces.ContainsKey(faceIdx))
        {
            faces[faceIdx] = new NCollection_PackedMapOfInteger();
        }

        return faces.GetValue(faceIdx);
    }

    //=================================================================================================

    /// <summary>The narrow phase of the overlap test: exact intersection.</summary>
    protected void IntersectTrianglesExact(int trgIdx1, int trgIdx2)
    {
        int faceIdx1 = _set1!.GetFaceID(trgIdx1);

        _set1.GetVertices(trgIdx1, out BVH_Vec3d trg1Vert1, out BVH_Vec3d trg1Vert2, out BVH_Vec3d trg1Vert3);

        bool isInSet = _overlapSubShapes1.ContainsKey(faceIdx1);

        int faceIdx2 = _set2!.GetFaceID(trgIdx2);

        if (isInSet && _overlapSubShapes1.GetValue(faceIdx1).Contains(faceIdx2))
        {
            return;
        }

        ElementFilter.FilterResult result =
            _filter is null ? ElementFilter.FilterResult.DoCheck
                             : _filter.PreCheckElements(trgIdx1, trgIdx2);

        if (result == ElementFilter.FilterResult.Overlap)
        {
            GetSetOfFaces(_overlapSubShapes1, faceIdx1).Add(faceIdx2);
            GetSetOfFaces(_overlapSubShapes2, faceIdx2).Add(faceIdx1);
        }
        else if (result == ElementFilter.FilterResult.DoCheck)
        {
            _set2.GetVertices(trgIdx2, out BVH_Vec3d trg2Vert1, out BVH_Vec3d trg2Vert2, out BVH_Vec3d trg2Vert3);

            if (TrianglesIntersected(trg1Vert1, trg1Vert2, trg1Vert3, trg2Vert1, trg2Vert2, trg2Vert3))
            {
                GetSetOfFaces(_overlapSubShapes1, faceIdx1).Add(faceIdx2);
                GetSetOfFaces(_overlapSubShapes2, faceIdx2).Add(faceIdx1);
            }
        }
    }

    /// <summary>The narrow phase of the overlap test: intersection with a non-zero tolerance.</summary>
    protected void IntersectTrianglesToler(int trgIdx1, int trgIdx2, double toler)
    {
        int faceIdx1 = _set1!.GetFaceID(trgIdx1);

        _set1.GetVertices(trgIdx1, out BVH_Vec3d trg1Vert1, out BVH_Vec3d trg1Vert2, out BVH_Vec3d trg1Vert3);

        var prism1 = new BRepExtrema_BoundingPrism(); // not initialized

        bool isInSet = _overlapSubShapes1.ContainsKey(faceIdx1);

        int faceIdx2 = _set2!.GetFaceID(trgIdx2);

        if (isInSet && _overlapSubShapes1.GetValue(faceIdx1).Contains(faceIdx2))
        {
            return;
        }

        ElementFilter.FilterResult result =
            _filter is null ? ElementFilter.FilterResult.DoCheck
                             : _filter.PreCheckElements(trgIdx1, trgIdx2);

        if (result == ElementFilter.FilterResult.Overlap)
        {
            GetSetOfFaces(_overlapSubShapes1, faceIdx1).Add(faceIdx2);
            GetSetOfFaces(_overlapSubShapes2, faceIdx2).Add(faceIdx1);
        }
        else if (result == ElementFilter.FilterResult.DoCheck)
        {
            if (!prism1.IsInited)
            {
                prism1.Init(trg1Vert1, trg1Vert2, trg1Vert3, toler);
            }

            _set2.GetVertices(trgIdx2, out BVH_Vec3d trg2Vert1, out BVH_Vec3d trg2Vert2, out BVH_Vec3d trg2Vert3);

            var prism2 = new BRepExtrema_BoundingPrism(trg2Vert1, trg2Vert2, trg2Vert3, toler);

            if (PrismsIntersected(prism1, prism2))
            {
                GetSetOfFaces(_overlapSubShapes1, faceIdx1).Add(faceIdx2);
                GetSetOfFaces(_overlapSubShapes2, faceIdx2).Add(faceIdx1);
            }
        }
    }

    //=======================================================================
    // function : Perform
    // purpose  : Performs search for overlapped faces
    //=======================================================================
    /// <summary>Searches for overlapped mesh elements, within the tolerance when it isn't zero.</summary>
    public void Perform(double tolerance = 0.0)
    {
        _tolerance = tolerance;

        _isDone = (Select(_set1!.BVH(), _set2!.BVH()) > 0);
    }

    /// <summary>Rejects a pair of nodes whose boxes don't overlap (within the tolerance).</summary>
    public override bool RejectNode(in BVH_Vec3d cornerMin1,
                                    in BVH_Vec3d cornerMax1,
                                    in BVH_Vec3d cornerMin2,
                                    in BVH_Vec3d cornerMax2,
                                    out double metric)
    {
        metric = 0.0; // OCCT leaves the metric unwritten
        return !OverlapBoxes(cornerMin1, cornerMax1, cornerMin2, cornerMax2, _tolerance);
    }

    /// <summary>Tests a pair of leaf elements (triangles).</summary>
    public override bool Accept(int trgIdx1, int trgIdx2)
    {
        if (_tolerance == 0.0)
        {
            IntersectTrianglesExact(trgIdx1, trgIdx2);
        }
        else
        {
            IntersectTrianglesToler(trgIdx1, trgIdx2, _tolerance);
        }
        return true;
    }
}
