// Port of: src/FoundationClasses/TKMath/BVH/BVH_Tools.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Numerics;

namespace Occt;

/// <summary>Static methods on points, triangles, rays and bounding boxes.</summary>
public static class BVH_Tools<T, TVec>
    where T : unmanaged, IFloatingPointIeee754<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    public enum BVH_PrjStateInTriangle
    {
        BVH_PrjStateInTriangle_VERTEX,
        BVH_PrjStateInTriangle_EDGE,
        BVH_PrjStateInTriangle_INNER,
    }

    // Box-Box square distance

    /// <summary>The square distance between two boxes (zero if either is invalid).</summary>
    public static T BoxBoxSquareDistance(in BVH_Box<T, TVec> theBox1, in BVH_Box<T, TVec> theBox2)
    {
        if (!theBox1.IsValid() || !theBox2.IsValid())
        {
            return T.Zero;
        }

        return BoxBoxSquareDistance(theBox1.CornerMin(), theBox1.CornerMax(), theBox2.CornerMin(), theBox2.CornerMax());
    }

    /// <summary>The square distance between two boxes given by their corners.</summary>
    public static T BoxBoxSquareDistance(in TVec theCMin1, in TVec theCMax1, in TVec theCMin2, in TVec theCMax2)
    {
        T aDist = T.Zero;
        for (int i = 0; i < TVec.Length(); ++i)
        {
            if (theCMin1[i] > theCMax2[i])
            {
                T d = theCMin1[i] - theCMax2[i];
                aDist += d * d;
            }
            else if (theCMax1[i] < theCMin2[i])
            {
                T d = theCMin2[i] - theCMax1[i];
                aDist += d * d;
            }
        }

        return aDist;
    }

    // Point-Box square distance

    /// <summary>The square distance between a point and a box (zero if the box is invalid).</summary>
    public static T PointBoxSquareDistance(in TVec thePoint, in BVH_Box<T, TVec> theBox)
    {
        if (!theBox.IsValid())
        {
            return T.Zero;
        }

        return PointBoxSquareDistance(thePoint, theBox.CornerMin(), theBox.CornerMax());
    }

    /// <summary>The square distance between a point and a box given by its corners.</summary>
    public static T PointBoxSquareDistance(in TVec thePoint, in TVec theCMin, in TVec theCMax)
    {
        T aDist = T.Zero;
        for (int i = 0; i < TVec.Length(); ++i)
        {
            if (thePoint[i] < theCMin[i])
            {
                T d = theCMin[i] - thePoint[i];
                aDist += d * d;
            }
            else if (thePoint[i] > theCMax[i])
            {
                T d = thePoint[i] - theCMax[i];
                aDist += d * d;
            }
        }

        return aDist;
    }

    // Point-Box projection

    /// <summary>The projection of a point on a box (the point itself if the box is invalid).</summary>
    public static TVec PointBoxProjection(in TVec thePoint, in BVH_Box<T, TVec> theBox)
    {
        if (!theBox.IsValid())
        {
            return thePoint;
        }

        return PointBoxProjection(thePoint, theBox.CornerMin(), theBox.CornerMax());
    }

    /// <summary>The projection of a point on a box given by its corners.</summary>
    public static TVec PointBoxProjection(in TVec thePoint, in TVec theCMin, in TVec theCMax)
        => thePoint.cwiseMax(theCMin).cwiseMin(theCMax);

    // Internal helpers for point-triangle projection (OCCT's pointer outputs are refs, written only when asked)

    private static void SetVertexState(bool theToSet, ref BVH_PrjStateInTriangle thePrjState, ref int theFirstNode, ref int theLastNode, int theVertexIndex)
    {
        if (theToSet)
        {
            thePrjState = BVH_PrjStateInTriangle.BVH_PrjStateInTriangle_VERTEX;
            theFirstNode = theVertexIndex;
            theLastNode = theVertexIndex;
        }
    }

    private static void SetEdgeState(bool theToSet, ref BVH_PrjStateInTriangle thePrjState, ref int theFirstNode, ref int theLastNode, int theStartVertex, int theEndVertex)
    {
        if (theToSet)
        {
            thePrjState = BVH_PrjStateInTriangle.BVH_PrjStateInTriangle_EDGE;
            theFirstNode = theStartVertex;
            theLastNode = theEndVertex;
        }
    }

    private static TVec ProjectToEdge(in TVec theEdgeStart, in TVec theEdge, T theDot1, T theDot2)
    {
        T aT = theDot1 / (theDot1 + theDot2);
        return theEdgeStart + theEdge * aT;
    }

    // Point-Triangle square distance

    /// <summary>The point of a triangle nearest to the given point (Voronoi region tests).</summary>
    public static TVec PointTriangleProjection(in TVec thePoint, in TVec theNode0, in TVec theNode1, in TVec theNode2)
    {
        BVH_PrjStateInTriangle aState = default;
        int aFirst = 0, aLast = 0;
        return PointTriangleProjection(false, thePoint, theNode0, theNode1, theNode2, ref aState, ref aFirst, ref aLast);
    }

    /// <summary>
    /// The point of a triangle nearest to the given point, with where it lies: a vertex (first and last node the
    /// vertex), an edge (its nodes) or inside (the node numbers are left unchanged).
    /// </summary>
    public static TVec PointTriangleProjection(in TVec thePoint,
                                               in TVec theNode0,
                                               in TVec theNode1,
                                               in TVec theNode2,
                                               ref BVH_PrjStateInTriangle thePrjState,
                                               ref int theNumberOfFirstNode,
                                               ref int theNumberOfLastNode)
        => PointTriangleProjection(true, thePoint, theNode0, theNode1, theNode2, ref thePrjState, ref theNumberOfFirstNode, ref theNumberOfLastNode);

    private static TVec PointTriangleProjection(bool theHasState,
                                                in TVec thePoint,
                                                in TVec theNode0,
                                                in TVec theNode1,
                                                in TVec theNode2,
                                                ref BVH_PrjStateInTriangle thePrjState,
                                                ref int theNumberOfFirstNode,
                                                ref int theNumberOfLastNode)
    {
        // Compute edge vectors
        TVec aAB = theNode1 - theNode0;
        TVec aAC = theNode2 - theNode0;
        TVec aBC = theNode2 - theNode1;

        // Compute point-to-vertex vectors
        TVec aAP = thePoint - theNode0;
        TVec aBP = thePoint - theNode1;
        TVec aCP = thePoint - theNode2;

        // Compute dot products for Voronoi region tests
        T aABdotAP = aAB.Dot(aAP);
        T aACdotAP = aAC.Dot(aAP);

        // Check if P is in vertex region outside A
        if (aABdotAP <= T.Zero && aACdotAP <= T.Zero)
        {
            SetVertexState(theHasState, ref thePrjState, ref theNumberOfFirstNode, ref theNumberOfLastNode, 0);
            return theNode0;
        }

        T aBAdotBP = -aAB.Dot(aBP);
        T aBCdotBP = aBC.Dot(aBP);

        // Check if P is in vertex region outside B
        if (aBAdotBP <= T.Zero && aBCdotBP <= T.Zero)
        {
            SetVertexState(theHasState, ref thePrjState, ref theNumberOfFirstNode, ref theNumberOfLastNode, 1);
            return theNode1;
        }

        T aCBdotCP = -aBC.Dot(aCP);
        T aCAdotCP = -aAC.Dot(aCP);

        // Check if P is in vertex region outside C
        if (aCAdotCP <= T.Zero && aCBdotCP <= T.Zero)
        {
            SetVertexState(theHasState, ref thePrjState, ref theNumberOfFirstNode, ref theNumberOfLastNode, 2);
            return theNode2;
        }

        // Compute barycentric coordinates for edge/interior tests
        T aACdotBP = aAC.Dot(aBP);
        T aVC = aABdotAP * aACdotBP + aBAdotBP * aACdotAP;

        // Check if P is in edge region of AB
        if (aVC <= T.Zero && aABdotAP > T.Zero && aBAdotBP > T.Zero)
        {
            SetEdgeState(theHasState, ref thePrjState, ref theNumberOfFirstNode, ref theNumberOfLastNode, 0, 1);
            return ProjectToEdge(theNode0, aAB, aABdotAP, aBAdotBP);
        }

        T aABdotCP = aAB.Dot(aCP);
        T aVA = aBAdotBP * aCAdotCP - aABdotCP * aACdotBP;

        // Check if P is in edge region of BC
        if (aVA <= T.Zero && aBCdotBP > T.Zero && aCBdotCP > T.Zero)
        {
            SetEdgeState(theHasState, ref thePrjState, ref theNumberOfFirstNode, ref theNumberOfLastNode, 1, 2);
            return ProjectToEdge(theNode1, aBC, aBCdotBP, aCBdotCP);
        }

        T aVB = aABdotCP * aACdotAP + aABdotAP * aCAdotCP;

        // Check if P is in edge region of CA
        if (aVB <= T.Zero && aACdotAP > T.Zero && aCAdotCP > T.Zero)
        {
            SetEdgeState(theHasState, ref thePrjState, ref theNumberOfFirstNode, ref theNumberOfLastNode, 2, 0);
            return ProjectToEdge(theNode0, aAC, aACdotAP, aCAdotCP);
        }

        // P is inside triangle - compute barycentric coordinates
        T aNorm = aVA + aVB + aVC;

        // Handle degenerate triangle (zero or near-zero area)
        if (aNorm <= BVH.Epsilon<T>() * (T.Abs(aVA) + T.Abs(aVB) + T.Abs(aVC)))
        {
            SetVertexState(theHasState, ref thePrjState, ref theNumberOfFirstNode, ref theNumberOfLastNode, 0);
            return (theNode0 + theNode1 + theNode2) / T.CreateTruncating(3);
        }

        if (theHasState)
        {
            thePrjState = BVH_PrjStateInTriangle.BVH_PrjStateInTriangle_INNER;
        }

        return (theNode0 * aVA + theNode1 * aVB + theNode2 * aVC) / aNorm;
    }

    /// <summary>The square distance between a point and a triangle.</summary>
    public static T PointTriangleSquareDistance(in TVec thePoint, in TVec theNode0, in TVec theNode1, in TVec theNode2)
    {
        TVec aProj = PointTriangleProjection(thePoint, theNode0, theNode1, theNode2);
        TVec aPP = aProj - thePoint;
        return aPP.Dot(aPP);
    }

    // Ray-Box intersection

    /// <summary>The times the ray enters and leaves the box, using the ray's reciprocal direction.</summary>
    public static bool RayBoxIntersection(in BVH_Ray<T, TVec> theRay, in BVH_Box<T, TVec> theBox, ref T theTimeEnter, ref T theTimeLeave)
    {
        if (!theBox.IsValid())
        {
            return false;
        }

        return RayBoxIntersection(theRay, theBox.CornerMin(), theBox.CornerMax(), ref theTimeEnter, ref theTimeLeave);
    }

    /// <summary>
    /// The times the ray enters and leaves the box, using the ray's reciprocal direction. Rays parallel to a slab
    /// (infinite reciprocal) are handled explicitly to avoid NaN from 0 * inf. The times are written only on a hit.
    /// </summary>
    public static bool RayBoxIntersection(in BVH_Ray<T, TVec> theRay, in TVec theBoxCMin, in TVec theBoxCMax, ref T theTimeEnter, ref T theTimeLeave)
    {
        T aTimeEnter = T.MinValue;
        T aTimeLeave = T.MaxValue;

        TVec anOrigin = theRay.Origin;
        TVec anInvDirect = theRay.InvDirect;
        for (int i = 0; i < TVec.Length(); ++i)
        {
            // Handle parallel rays (infinite inverse direction) to avoid NaN from 0*inf
            if (T.IsInfinity(anInvDirect[i]))
            {
                if (anOrigin[i] < theBoxCMin[i] || anOrigin[i] > theBoxCMax[i])
                {
                    return false;
                }

                continue;
            }

            T aT1 = (theBoxCMin[i] - anOrigin[i]) * anInvDirect[i];
            T aT2 = (theBoxCMax[i] - anOrigin[i]) * anInvDirect[i];

            aTimeEnter = BVH.Max(aTimeEnter, BVH.Min(aT1, aT2));
            aTimeLeave = BVH.Min(aTimeLeave, BVH.Max(aT1, aT2));

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

        theTimeEnter = aTimeEnter;
        theTimeLeave = aTimeLeave;
        return true;
    }

    /// <summary>The times a ray given by origin and direction enters and leaves the box.</summary>
    public static bool RayBoxIntersection(in TVec theRayOrigin, in TVec theRayDirection, in BVH_Box<T, TVec> theBox, ref T theTimeEnter, ref T theTimeLeave)
    {
        if (!theBox.IsValid())
        {
            return false;
        }

        return RayBoxIntersection(theRayOrigin, theRayDirection, theBox.CornerMin(), theBox.CornerMax(), ref theTimeEnter, ref theTimeLeave);
    }

    /// <summary>
    /// The times a ray given by origin and direction enters and leaves the box (single pass with early exit). The
    /// times are written only on a hit.
    /// </summary>
    public static bool RayBoxIntersection(in TVec theRayOrigin, in TVec theRayDirection, in TVec theBoxCMin, in TVec theBoxCMax, ref T theTimeEnter, ref T theTimeLeave)
    {
        T aTimeEnter = T.MinValue;
        T aTimeLeave = T.MaxValue;

        for (int i = 0; i < TVec.Length(); ++i)
        {
            if (theRayDirection[i] == T.Zero)
            {
                // Ray is parallel to this axis slab - check if origin is within bounds
                if (theRayOrigin[i] < theBoxCMin[i] || theRayOrigin[i] > theBoxCMax[i])
                {
                    return false; // Ray misses the slab entirely
                }

                // Ray is within the slab, doesn't constrain the intersection interval
                continue;
            }

            // Compute intersection distances for this axis
            T aT1 = (theBoxCMin[i] - theRayOrigin[i]) / theRayDirection[i];
            T aT2 = (theBoxCMax[i] - theRayOrigin[i]) / theRayDirection[i];

            // Ensure aT1 <= aT2 (handle negative direction)
            T aTMin = BVH.Min(aT1, aT2);
            T aTMax = BVH.Max(aT1, aT2);

            // Update intersection interval
            aTimeEnter = BVH.Max(aTimeEnter, aTMin);
            aTimeLeave = BVH.Min(aTimeLeave, aTMax);

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

        theTimeEnter = aTimeEnter;
        theTimeLeave = aTimeLeave;
        return true;
    }
}
