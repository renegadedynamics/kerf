// Port of: src/FoundationClasses/TKMath/BVH/BVH_Tools.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Numerics;

namespace Kerf.Engine.Geometry.Bvh;

/// <summary>Static methods on points, triangles, rays and bounding boxes.</summary>
internal static class Tools<T, TVec>
    where T : unmanaged, IFloatingPointIeee754<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    public enum BVH_PrjStateInTriangle
    {
        PrjStateInTriangle_VERTEX,
        PrjStateInTriangle_EDGE,
        PrjStateInTriangle_INNER,
    }

    // Box-Box square distance

    /// <summary>The square distance between two boxes (zero if either is invalid).</summary>
    public static T BoxBoxSquareDistance(in Box<T, TVec> box1, in Box<T, TVec> box2)
    {
        if (!box1.IsValid || !box2.IsValid)
        {
            return T.Zero;
        }

        return BoxBoxSquareDistance(box1.CornerMin(), box1.CornerMax(), box2.CornerMin(), box2.CornerMax());
    }

    /// <summary>The square distance between two boxes given by their corners.</summary>
    public static T BoxBoxSquareDistance(in TVec cMin1, in TVec cMax1, in TVec cMin2, in TVec cMax2)
    {
        T dist = T.Zero;
        for (int i = 0; i < TVec.Length(); ++i)
        {
            if (cMin1[i] > cMax2[i])
            {
                T d = cMin1[i] - cMax2[i];
                dist += d * d;
            }
            else if (cMax1[i] < cMin2[i])
            {
                T d = cMin2[i] - cMax1[i];
                dist += d * d;
            }
        }

        return dist;
    }

    // Point-Box square distance

    /// <summary>The square distance between a point and a box (zero if the box is invalid).</summary>
    public static T PointBoxSquareDistance(in TVec point, in Box<T, TVec> box)
    {
        if (!box.IsValid)
        {
            return T.Zero;
        }

        return PointBoxSquareDistance(point, box.CornerMin(), box.CornerMax());
    }

    /// <summary>The square distance between a point and a box given by its corners.</summary>
    public static T PointBoxSquareDistance(in TVec point, in TVec cMin, in TVec cMax)
    {
        T dist = T.Zero;
        for (int i = 0; i < TVec.Length(); ++i)
        {
            if (point[i] < cMin[i])
            {
                T d = cMin[i] - point[i];
                dist += d * d;
            }
            else if (point[i] > cMax[i])
            {
                T d = point[i] - cMax[i];
                dist += d * d;
            }
        }

        return dist;
    }

    // Point-Box projection

    /// <summary>The projection of a point on a box (the point itself if the box is invalid).</summary>
    public static TVec PointBoxProjection(in TVec point, in Box<T, TVec> box)
    {
        if (!box.IsValid)
        {
            return point;
        }

        return PointBoxProjection(point, box.CornerMin(), box.CornerMax());
    }

    /// <summary>The projection of a point on a box given by its corners.</summary>
    public static TVec PointBoxProjection(in TVec point, in TVec cMin, in TVec cMax)
        => point.CwiseMax(cMin).CwiseMin(cMax);

    // Internal helpers for point-triangle projection (OCCT's pointer outputs are refs, written only when asked)

    private static void SetVertexState(bool toSet, ref BVH_PrjStateInTriangle prjState, ref int firstNode, ref int lastNode, int vertexIndex)
    {
        if (toSet)
        {
            prjState = BVH_PrjStateInTriangle.PrjStateInTriangle_VERTEX;
            firstNode = vertexIndex;
            lastNode = vertexIndex;
        }
    }

    private static void SetEdgeState(bool toSet, ref BVH_PrjStateInTriangle prjState, ref int firstNode, ref int lastNode, int startVertex, int endVertex)
    {
        if (toSet)
        {
            prjState = BVH_PrjStateInTriangle.PrjStateInTriangle_EDGE;
            firstNode = startVertex;
            lastNode = endVertex;
        }
    }

    private static TVec ProjectToEdge(in TVec edgeStart, in TVec edge, T dot1, T dot2)
    {
        T t = dot1 / (dot1 + dot2);
        return edgeStart + edge * t;
    }

    // Point-Triangle square distance

    /// <summary>The point of a triangle nearest to the given point (Voronoi region tests).</summary>
    public static TVec PointTriangleProjection(in TVec point, in TVec node0, in TVec node1, in TVec node2)
    {
        BVH_PrjStateInTriangle state = default;
        int first = 0, last = 0;
        return PointTriangleProjection(false, point, node0, node1, node2, ref state, ref first, ref last);
    }

    /// <summary>
    /// The point of a triangle nearest to the given point, with where it lies: a vertex (first and last node the
    /// vertex), an edge (its nodes) or inside (the node numbers are left unchanged).
    /// </summary>
    public static TVec PointTriangleProjection(in TVec point,
                                               in TVec node0,
                                               in TVec node1,
                                               in TVec node2,
                                               ref BVH_PrjStateInTriangle prjState,
                                               ref int numberOfFirstNode,
                                               ref int numberOfLastNode)
        => PointTriangleProjection(true, point, node0, node1, node2, ref prjState, ref numberOfFirstNode, ref numberOfLastNode);

    private static TVec PointTriangleProjection(bool hasState,
                                                in TVec point,
                                                in TVec node0,
                                                in TVec node1,
                                                in TVec node2,
                                                ref BVH_PrjStateInTriangle prjState,
                                                ref int numberOfFirstNode,
                                                ref int numberOfLastNode)
    {
        // Compute edge vectors
        TVec ab = node1 - node0;
        TVec ac = node2 - node0;
        TVec bc = node2 - node1;

        // Compute point-to-vertex vectors
        TVec ap = point - node0;
        TVec bp = point - node1;
        TVec cp = point - node2;

        // Compute dot products for Voronoi region tests
        T aBdotAP = ab.Dot(ap);
        T aCdotAP = ac.Dot(ap);

        // Check if P is in vertex region outside A
        if (aBdotAP <= T.Zero && aCdotAP <= T.Zero)
        {
            SetVertexState(hasState, ref prjState, ref numberOfFirstNode, ref numberOfLastNode, 0);
            return node0;
        }

        T bAdotBP = -ab.Dot(bp);
        T bCdotBP = bc.Dot(bp);

        // Check if P is in vertex region outside B
        if (bAdotBP <= T.Zero && bCdotBP <= T.Zero)
        {
            SetVertexState(hasState, ref prjState, ref numberOfFirstNode, ref numberOfLastNode, 1);
            return node1;
        }

        T cBdotCP = -bc.Dot(cp);
        T cAdotCP = -ac.Dot(cp);

        // Check if P is in vertex region outside C
        if (cAdotCP <= T.Zero && cBdotCP <= T.Zero)
        {
            SetVertexState(hasState, ref prjState, ref numberOfFirstNode, ref numberOfLastNode, 2);
            return node2;
        }

        // Compute barycentric coordinates for edge/interior tests
        T aCdotBP = ac.Dot(bp);
        T vc = aBdotAP * aCdotBP + bAdotBP * aCdotAP;

        // Check if P is in edge region of AB
        if (vc <= T.Zero && aBdotAP > T.Zero && bAdotBP > T.Zero)
        {
            SetEdgeState(hasState, ref prjState, ref numberOfFirstNode, ref numberOfLastNode, 0, 1);
            return ProjectToEdge(node0, ab, aBdotAP, bAdotBP);
        }

        T aBdotCP = ab.Dot(cp);
        T va = bAdotBP * cAdotCP - aBdotCP * aCdotBP;

        // Check if P is in edge region of BC
        if (va <= T.Zero && bCdotBP > T.Zero && cBdotCP > T.Zero)
        {
            SetEdgeState(hasState, ref prjState, ref numberOfFirstNode, ref numberOfLastNode, 1, 2);
            return ProjectToEdge(node1, bc, bCdotBP, cBdotCP);
        }

        T vb = aBdotCP * aCdotAP + aBdotAP * cAdotCP;

        // Check if P is in edge region of CA
        if (vb <= T.Zero && aCdotAP > T.Zero && cAdotCP > T.Zero)
        {
            SetEdgeState(hasState, ref prjState, ref numberOfFirstNode, ref numberOfLastNode, 2, 0);
            return ProjectToEdge(node0, ac, aCdotAP, cAdotCP);
        }

        // P is inside triangle - compute barycentric coordinates
        T norm = va + vb + vc;

        // Handle degenerate triangle (zero or near-zero area)
        if (norm <= BvhTools.Epsilon<T>() * (T.Abs(va) + T.Abs(vb) + T.Abs(vc)))
        {
            SetVertexState(hasState, ref prjState, ref numberOfFirstNode, ref numberOfLastNode, 0);
            return (node0 + node1 + node2) / T.CreateTruncating(3);
        }

        if (hasState)
        {
            prjState = BVH_PrjStateInTriangle.PrjStateInTriangle_INNER;
        }

        return (node0 * va + node1 * vb + node2 * vc) / norm;
    }

    /// <summary>The square distance between a point and a triangle.</summary>
    public static T PointTriangleSquareDistance(in TVec point, in TVec node0, in TVec node1, in TVec node2)
    {
        TVec proj = PointTriangleProjection(point, node0, node1, node2);
        TVec pp = proj - point;
        return pp.Dot(pp);
    }

    // Ray-Box intersection

    /// <summary>The times the ray enters and leaves the box, using the ray's reciprocal direction.</summary>
    public static bool RayBoxIntersection(in Ray<T, TVec> ray, in Box<T, TVec> box, ref T timeEnter, ref T timeLeave)
    {
        if (!box.IsValid)
        {
            return false;
        }

        return RayBoxIntersection(ray, box.CornerMin(), box.CornerMax(), ref timeEnter, ref timeLeave);
    }

    /// <summary>
    /// The times the ray enters and leaves the box, using the ray's reciprocal direction. Rays parallel to a slab
    /// (infinite reciprocal) are handled explicitly to avoid NaN from 0 * inf. The times are written only on a hit.
    /// </summary>
    public static bool RayBoxIntersection(in Ray<T, TVec> ray, in TVec boxCMin, in TVec boxCMax, ref T timeEnter, ref T timeLeave)
    {
        T aTimeEnter = T.MinValue;
        T aTimeLeave = T.MaxValue;

        TVec origin = ray.Origin;
        TVec invDirect = ray.InvDirect;
        for (int i = 0; i < TVec.Length(); ++i)
        {
            // Handle parallel rays (infinite inverse direction) to avoid NaN from 0*inf
            if (T.IsInfinity(invDirect[i]))
            {
                if (origin[i] < boxCMin[i] || origin[i] > boxCMax[i])
                {
                    return false;
                }

                continue;
            }

            T t1 = (boxCMin[i] - origin[i]) * invDirect[i];
            T t2 = (boxCMax[i] - origin[i]) * invDirect[i];

            aTimeEnter = BvhTools.Max(aTimeEnter, BvhTools.Min(t1, t2));
            aTimeLeave = BvhTools.Min(aTimeLeave, BvhTools.Max(t1, t2));

            if (aTimeEnter > aTimeLeave)
            {
                return false;
            }
        }

        // Check if intersection is behind the ray origin
        if (aTimeLeave < T.Zero)
        {
            return false;
        }

        timeEnter = aTimeEnter;
        timeLeave = aTimeLeave;
        return true;
    }

    /// <summary>The times a ray given by origin and direction enters and leaves the box.</summary>
    public static bool RayBoxIntersection(in TVec rayOrigin, in TVec rayDirection, in Box<T, TVec> box, ref T timeEnter, ref T timeLeave)
    {
        if (!box.IsValid)
        {
            return false;
        }

        return RayBoxIntersection(rayOrigin, rayDirection, box.CornerMin(), box.CornerMax(), ref timeEnter, ref timeLeave);
    }

    /// <summary>
    /// The times a ray given by origin and direction enters and leaves the box (single pass with early exit). The
    /// times are written only on a hit.
    /// </summary>
    public static bool RayBoxIntersection(in TVec rayOrigin, in TVec rayDirection, in TVec boxCMin, in TVec boxCMax, ref T timeEnter, ref T timeLeave)
    {
        T aTimeEnter = T.MinValue;
        T aTimeLeave = T.MaxValue;

        for (int i = 0; i < TVec.Length(); ++i)
        {
            if (rayDirection[i] == T.Zero)
            {
                // Ray is parallel to this axis slab - check if origin is within bounds
                if (rayOrigin[i] < boxCMin[i] || rayOrigin[i] > boxCMax[i])
                {
                    return false; // Ray misses the slab entirely
                }

                // Ray is within the slab, doesn't constrain the intersection interval
                continue;
            }

            // Compute intersection distances for this axis
            T t1 = (boxCMin[i] - rayOrigin[i]) / rayDirection[i];
            T t2 = (boxCMax[i] - rayOrigin[i]) / rayDirection[i];

            // Ensure aT1 <= aT2 (handle negative direction)
            T tMin = BvhTools.Min(t1, t2);
            T tMax = BvhTools.Max(t1, t2);

            // Update intersection interval
            aTimeEnter = BvhTools.Max(aTimeEnter, tMin);
            aTimeLeave = BvhTools.Min(aTimeLeave, tMax);

            // Early exit if no intersection
            if (aTimeEnter > aTimeLeave)
            {
                return false;
            }
        }

        // Check if intersection is behind the ray origin
        if (aTimeLeave < T.Zero)
        {
            return false;
        }

        timeEnter = aTimeEnter;
        timeLeave = aTimeLeave;
        return true;
    }
}
