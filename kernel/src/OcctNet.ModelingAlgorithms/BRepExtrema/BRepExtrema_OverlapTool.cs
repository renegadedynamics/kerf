// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_OverlapTool.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_OverlapTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2015-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bvh;
using BVH_Vec2d = Occt.NCollection_Vec2<double>;
using BVH_Vec3d = Occt.NCollection_Vec3<double>;

namespace Occt;

/// <summary>
/// Detects overlapping of two BVH primitive sets (triangle sets): the pairs of faces whose triangles intersect, or
/// come within a tolerance of each other. The two sets may be the same, to find self-intersections. Results are
/// approximate and depend on the quality of the tessellations.
/// </summary>
/// <remarks>
/// A working object (decision 21), used by <see cref="BRepExtrema_ShapeProximity"/> and
/// <see cref="BRepExtrema_SelfIntersection"/>; the result maps are its own. OCCT's debug output of the overlapped
/// triangles (OVERLAP_TOOL_OUTPUT_TRIANGLES) is compiled out, as in OCCT.
/// </remarks>
public class BRepExtrema_OverlapTool : PairTraverse<double, BVH_Vec3d, double>
{
    private BRepExtrema_TriangleSet? mySet1;
    private BRepExtrema_TriangleSet? mySet2;
    private BRepExtrema_ElementFilter? myFilter;
    private readonly NCollection_DataMap<int, NCollection_PackedMapOfInteger> myOverlapSubShapes1 = new();
    private readonly NCollection_DataMap<int, NCollection_PackedMapOfInteger> myOverlapSubShapes2 = new();
    private bool myIsDone;
    private double myTolerance;

    /// <summary>An uninitialized overlap tool.</summary>
    public BRepExtrema_OverlapTool()
    {
        myFilter = null;
        myTolerance = 0.0;
        myIsDone = false;
    }

    /// <summary>An overlap tool for the given element sets.</summary>
    public BRepExtrema_OverlapTool(BRepExtrema_TriangleSet theSet1, BRepExtrema_TriangleSet theSet2)
    {
        myFilter = null;
        myTolerance = 0.0;
        LoadTriangleSets(theSet1, theSet2);
    }

    /// <summary>Loads the element sets.</summary>
    public void LoadTriangleSets(BRepExtrema_TriangleSet theSet1, BRepExtrema_TriangleSet theSet2)
    {
        mySet1 = theSet1;
        mySet2 = theSet2;

        // OCCT: the result maps aren't cleared, so a second Perform adds to the first one's pairs
        myIsDone = false;
    }

    /// <summary>True if the overlap test is completed.</summary>
    public bool IsDone() => myIsDone;

    /// <summary>Marks the results as outdated.</summary>
    public void MarkDirty() => myIsDone = false;

    /// <summary>The overlapped sub-shapes (faces) of the first set, each with the faces of the second it overlaps.</summary>
    public NCollection_DataMap<int, NCollection_PackedMapOfInteger> OverlapSubShapes1() => myOverlapSubShapes1;

    /// <summary>The overlapped sub-shapes (faces) of the second set, each with the faces of the first it overlaps.</summary>
    public NCollection_DataMap<int, NCollection_PackedMapOfInteger> OverlapSubShapes2() => myOverlapSubShapes2;

    /// <summary>Sets the filter for preliminary checking of pairs of mesh elements.</summary>
    public void SetElementFilter(BRepExtrema_ElementFilter? theFilter) => myFilter = theFilter;

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

        public BRepExtrema_BoundingPrism(in BVH_Vec3d theVertex0, in BVH_Vec3d theVertex1, in BVH_Vec3d theVertex2, double theDeflect)
            : this()
        {
            Init(theVertex0, theVertex1, theVertex2, theDeflect);
        }

        public void Init(in BVH_Vec3d theVertex0, in BVH_Vec3d theVertex1, in BVH_Vec3d theVertex2, double theDeflect)
        {
            Edges[0] = theVertex1 - theVertex0;
            Edges[1] = theVertex2 - theVertex0;
            Edges[2] = theVertex2 - theVertex1;

            Normal = BVH_Vec3d.Cross(Edges[0], Edges[1]);

            EdgeNormals[0] = BVH_Vec3d.Cross(Edges[0], Normal);
            EdgeNormals[1] = BVH_Vec3d.Cross(Edges[1], Normal);
            EdgeNormals[2] = BVH_Vec3d.Cross(Edges[2], Normal);

            EdgeNormals[0] = EdgeNormals[0] * (1.0 / Max(EdgeNormals[0].Modulus(), global::Kerf.Engine.Foundation.Precision.Confusion()));
            EdgeNormals[1] = EdgeNormals[1] * (1.0 / Max(EdgeNormals[1].Modulus(), global::Kerf.Engine.Foundation.Precision.Confusion()));
            EdgeNormals[2] = EdgeNormals[2] * (1.0 / Max(EdgeNormals[2].Modulus(), global::Kerf.Engine.Foundation.Precision.Confusion()));

            BVH_Vec3d aDirect01 = EdgeNormals[0] - EdgeNormals[1];
            BVH_Vec3d aDirect02 = EdgeNormals[0] + EdgeNormals[2];
            BVH_Vec3d aDirect12 = EdgeNormals[2] - EdgeNormals[1];

            Vertices[0] = Vertices[3] = theVertex0 + aDirect01 * (theDeflect / aDirect01.Dot(EdgeNormals[0]));
            Vertices[1] = Vertices[4] = theVertex1 + aDirect02 * (theDeflect / aDirect02.Dot(EdgeNormals[2]));
            Vertices[2] = Vertices[5] = theVertex2 + aDirect12 * (theDeflect / aDirect12.Dot(EdgeNormals[2]));

            BVH_Vec3d aNormOffset = Normal * (theDeflect / Max(Normal.Modulus(), global::Kerf.Engine.Foundation.Precision.Confusion()));

            for (int aVertIdx = 0; aVertIdx < 3; ++aVertIdx)
            {
                Vertices[aVertIdx + 0] = Vertices[aVertIdx + 0] + aNormOffset;
                Vertices[aVertIdx + 3] = Vertices[aVertIdx + 3] - aNormOffset;
            }

            IsInited = true;
        }

        /// <summary>True if the two prisms are separated along the axis.</summary>
        public readonly bool Separated(in BRepExtrema_BoundingPrism thePrism, in BVH_Vec3d theAxis)
        {
            double aMin1 = double.MaxValue;
            double aMax1 = -double.MaxValue;

            double aMin2 = double.MaxValue;
            double aMax2 = -double.MaxValue;

            for (int aVertIdx = 0; aVertIdx < 6; ++aVertIdx)
            {
                double aProj1 = Vertices[aVertIdx].Dot(theAxis);

                aMin1 = Min(aMin1, aProj1);
                aMax1 = Max(aMax1, aProj1);

                double aProj2 = thePrism.Vertices[aVertIdx].Dot(theAxis);

                aMin2 = Min(aMin2, aProj2);
                aMax2 = Max(aMax2, aProj2);

                if (aMin1 <= aMax2 && aMax1 >= aMin2)
                {
                    return false;
                }
            }

            return aMin1 > aMax2 || aMax1 < aMin2;
        }
    }

    private static double sign(in BVH_Vec3d theVertex0, in BVH_Vec3d theVertex1, in BVH_Vec3d theVertex2, int theX, int theY)
        => (theVertex0[theX] - theVertex2[theX]) * (theVertex1[theY] - theVertex2[theY])
           - (theVertex1[theX] - theVertex2[theX]) * (theVertex0[theY] - theVertex2[theY]);

    private static bool pointInTriangle(in BVH_Vec3d theTestPnt,
                                        in BVH_Vec3d theTrgVtx0,
                                        in BVH_Vec3d theTrgVtx1,
                                        in BVH_Vec3d theTrgVtx2,
                                        int theX,
                                        int theY)
    {
        bool aSign0 = sign(theTestPnt, theTrgVtx0, theTrgVtx1, theX, theY) <= 0.0;
        bool aSign1 = sign(theTestPnt, theTrgVtx1, theTrgVtx2, theX, theY) <= 0.0;
        bool aSign2 = sign(theTestPnt, theTrgVtx2, theTrgVtx0, theX, theY) <= 0.0;

        return (aSign0 == aSign1) && (aSign1 == aSign2);
    }

    // =======================================================================
    // function : segmentsIntersected
    // purpose  : Checks if two line segments are intersected
    // =======================================================================
    private static bool segmentsIntersected(in BVH_Vec2d theOriginSeg0,
                                            in BVH_Vec2d theOriginSeg1,
                                            in BVH_Vec2d theDirectSeg0,
                                            in BVH_Vec2d theDirectSeg1)
    {
        double aDet = -theDirectSeg1.X() * theDirectSeg0.Y() + theDirectSeg0.X() * theDirectSeg1.Y();

        if (Math.Abs(aDet) < DBL_EPSILON) // segments are parallel
        {
            BVH_Vec2d aDirect = theDirectSeg0 * (1.0 / theDirectSeg0.Modulus());

            double aEdge0Time0 = theOriginSeg0.Dot(aDirect);
            double aEdge1Time0 = theOriginSeg1.Dot(aDirect);

            double aEdge0Time1 = aEdge0Time0 + theDirectSeg0.Dot(aDirect);
            double aEdge1Time1 = aEdge1Time0 + theDirectSeg1.Dot(aDirect);

            double aEdge0Min = Min(aEdge0Time0, aEdge0Time1);
            double aEdge1Min = Min(aEdge1Time0, aEdge1Time1);
            double aEdge0Max = Max(aEdge0Time0, aEdge0Time1);
            double aEdge1Max = Max(aEdge1Time0, aEdge1Time1);

            if (Max(aEdge0Min, aEdge1Min) > Min(aEdge0Max, aEdge1Max))
            {
                return false;
            }

            var aNormal = new BVH_Vec2d(-aDirect.Y(), aDirect.X());

            return Math.Abs(theOriginSeg0.Dot(aNormal) - theOriginSeg1.Dot(aNormal)) < DBL_EPSILON;
        }

        BVH_Vec2d aDelta = theOriginSeg0 - theOriginSeg1;

        double aU = (-theDirectSeg0.Y() * aDelta.X() + theDirectSeg0.X() * aDelta.Y()) / aDet;
        double aV = (theDirectSeg1.X() * aDelta.Y() - theDirectSeg1.Y() * aDelta.X()) / aDet;

        return aU >= 0.0 && aU <= 1.0 && aV >= 0.0 && aV <= 1.0;
    }

    private const double DBL_EPSILON = 2.220446049250313E-16;

    // =======================================================================
    // function : trianglesIntersected
    // purpose  : Checks if two triangles are intersected
    //            ("A Fast Triangle-Triangle Intersection Test" by T. Moller)
    // =======================================================================
    private static bool trianglesIntersected(in BVH_Vec3d theTrng0Vert0,
                                             in BVH_Vec3d theTrng0Vert1,
                                             in BVH_Vec3d theTrng0Vert2,
                                             in BVH_Vec3d theTrng1Vert0,
                                             in BVH_Vec3d theTrng1Vert1,
                                             in BVH_Vec3d theTrng1Vert2)
    {
        BVH_Vec3d aTrng1Normal = BVH_Vec3d.Cross(theTrng1Vert1 - theTrng1Vert0, theTrng1Vert2 - theTrng1Vert0).Normalized();

        double aTrng1PlaneDist = aTrng1Normal.Dot(-theTrng1Vert0);

        double aDistTrng0Vert0 = aTrng1Normal.Dot(theTrng0Vert0) + aTrng1PlaneDist;
        double aDistTrng0Vert1 = aTrng1Normal.Dot(theTrng0Vert1) + aTrng1PlaneDist;
        double aDistTrng0Vert2 = aTrng1Normal.Dot(theTrng0Vert2) + aTrng1PlaneDist;

        if ((aDistTrng0Vert0 < 0.0 && aDistTrng0Vert1 < 0.0 && aDistTrng0Vert2 < 0.0)
            || (aDistTrng0Vert0 > 0.0 && aDistTrng0Vert1 > 0.0 && aDistTrng0Vert2 > 0.0))
        {
            return false; // 1st triangle lies on one side of the 2nd triangle
        }

        if (Math.Abs(aDistTrng0Vert0) > global::Kerf.Engine.Foundation.Precision.Confusion()
            || Math.Abs(aDistTrng0Vert1) > global::Kerf.Engine.Foundation.Precision.Confusion()
            || Math.Abs(aDistTrng0Vert2) > global::Kerf.Engine.Foundation.Precision.Confusion()) // general 3D case
        {
            BVH_Vec3d aTrng0Normal = BVH_Vec3d.Cross(theTrng0Vert1 - theTrng0Vert0, theTrng0Vert2 - theTrng0Vert0).Normalized();

            double aTrng0PlaneDist = aTrng0Normal.Dot(-theTrng0Vert0);

            double aDistTrng1Vert0 = aTrng0Normal.Dot(theTrng1Vert0) + aTrng0PlaneDist;
            double aDistTrng1Vert1 = aTrng0Normal.Dot(theTrng1Vert1) + aTrng0PlaneDist;
            double aDistTrng1Vert2 = aTrng0Normal.Dot(theTrng1Vert2) + aTrng0PlaneDist;

            if ((aDistTrng1Vert0 < 0.0 && aDistTrng1Vert1 < 0.0 && aDistTrng1Vert2 < 0.0)
                || (aDistTrng1Vert0 > 0.0 && aDistTrng1Vert1 > 0.0 && aDistTrng1Vert2 > 0.0))
            {
                return false; // 2nd triangle lies on one side of the 1st triangle
            }

            BVH_Vec3d aCrossLine = BVH_Vec3d.Cross(aTrng0Normal, aTrng1Normal);

            double aProjTrng0Vert0 = theTrng0Vert0.Dot(aCrossLine);
            double aProjTrng0Vert1 = theTrng0Vert1.Dot(aCrossLine);
            double aProjTrng0Vert2 = theTrng0Vert2.Dot(aCrossLine);

            if (aDistTrng0Vert0 * aDistTrng0Vert1 > 0.0)
            {
                (aDistTrng0Vert1, aDistTrng0Vert2) = (aDistTrng0Vert2, aDistTrng0Vert1);
                (aProjTrng0Vert1, aProjTrng0Vert2) = (aProjTrng0Vert2, aProjTrng0Vert1);
            }
            else if (aDistTrng0Vert1 * aDistTrng0Vert2 > 0.0)
            {
                (aDistTrng0Vert1, aDistTrng0Vert0) = (aDistTrng0Vert0, aDistTrng0Vert1);
                (aProjTrng0Vert1, aProjTrng0Vert0) = (aProjTrng0Vert0, aProjTrng0Vert1);
            }

            double aTime1 = Math.Abs(aDistTrng0Vert0) <= DBL_EPSILON
                                ? aProjTrng0Vert0
                                : aProjTrng0Vert0
                                      + (aProjTrng0Vert1 - aProjTrng0Vert0) * aDistTrng0Vert0
                                            / (aDistTrng0Vert0 - aDistTrng0Vert1);
            double aTime2 = Math.Abs(aDistTrng0Vert2) <= DBL_EPSILON
                                ? aProjTrng0Vert2
                                : aProjTrng0Vert2
                                      + (aProjTrng0Vert1 - aProjTrng0Vert2) * aDistTrng0Vert2
                                            / (aDistTrng0Vert2 - aDistTrng0Vert1);

            double aTimeMin1 = Min(aTime1, aTime2);
            double aTimeMax1 = Max(aTime1, aTime2);

            double aProjTrng1Vert0 = theTrng1Vert0.Dot(aCrossLine);
            double aProjTrng1Vert1 = theTrng1Vert1.Dot(aCrossLine);
            double aProjTrng1Vert2 = theTrng1Vert2.Dot(aCrossLine);

            if (aDistTrng1Vert0 * aDistTrng1Vert1 > 0.0)
            {
                (aDistTrng1Vert1, aDistTrng1Vert2) = (aDistTrng1Vert2, aDistTrng1Vert1);
                (aProjTrng1Vert1, aProjTrng1Vert2) = (aProjTrng1Vert2, aProjTrng1Vert1);
            }
            else if (aDistTrng1Vert1 * aDistTrng1Vert2 > 0.0)
            {
                (aDistTrng1Vert1, aDistTrng1Vert0) = (aDistTrng1Vert0, aDistTrng1Vert1);
                (aProjTrng1Vert1, aProjTrng1Vert0) = (aProjTrng1Vert0, aProjTrng1Vert1);
            }

            aTime1 = Math.Abs(aDistTrng1Vert0) <= DBL_EPSILON
                         ? aProjTrng1Vert0
                         : aProjTrng1Vert0
                               + (aProjTrng1Vert1 - aProjTrng1Vert0) * aDistTrng1Vert0
                                     / (aDistTrng1Vert0 - aDistTrng1Vert1);
            aTime2 = Math.Abs(aDistTrng1Vert2) <= DBL_EPSILON
                         ? aProjTrng1Vert2
                         : aProjTrng1Vert2
                               + (aProjTrng1Vert1 - aProjTrng1Vert2) * aDistTrng1Vert2
                                     / (aDistTrng1Vert2 - aDistTrng1Vert1);

            double aTimeMin2 = Min(aTime1, aTime2);
            double aTimeMax2 = Max(aTime1, aTime2);

            aTime1 = Max(aTimeMin1, aTimeMin2);
            aTime2 = Min(aTimeMax1, aTimeMax2);

            return aTime1 <= aTime2; // intervals intersected --> triangles overlapped
        }
        else // triangles are co-planar
        {
            int anX;
            int anY;

            if (Math.Abs(aTrng1Normal[0]) > Math.Abs(aTrng1Normal[1]))
            {
                anX = Math.Abs(aTrng1Normal[0]) > Math.Abs(aTrng1Normal[2]) ? 1 : 0;
                anY = Math.Abs(aTrng1Normal[0]) > Math.Abs(aTrng1Normal[2]) ? 2 : 1;
            }
            else
            {
                anX = Math.Abs(aTrng1Normal[1]) > Math.Abs(aTrng1Normal[2]) ? 0 : 0;
                anY = Math.Abs(aTrng1Normal[1]) > Math.Abs(aTrng1Normal[2]) ? 2 : 1;
            }

            ReadOnlySpan<BVH_Vec2d> aOriginSeg0 = [new BVH_Vec2d(theTrng0Vert0[anX], theTrng0Vert0[anY]),
                                                   new BVH_Vec2d(theTrng0Vert1[anX], theTrng0Vert1[anY]),
                                                   new BVH_Vec2d(theTrng0Vert2[anX], theTrng0Vert2[anY])];

            ReadOnlySpan<BVH_Vec2d> aDirectSeg0 = [aOriginSeg0[1] - aOriginSeg0[0],
                                                   aOriginSeg0[2] - aOriginSeg0[1],
                                                   aOriginSeg0[0] - aOriginSeg0[2]];

            ReadOnlySpan<BVH_Vec2d> aOriginSeg1 = [new BVH_Vec2d(theTrng1Vert0[anX], theTrng1Vert0[anY]),
                                                   new BVH_Vec2d(theTrng1Vert1[anX], theTrng1Vert1[anY]),
                                                   new BVH_Vec2d(theTrng1Vert2[anX], theTrng1Vert2[anY])];

            ReadOnlySpan<BVH_Vec2d> aDirectSeg1 = [aOriginSeg1[1] - aOriginSeg1[0],
                                                   aOriginSeg1[2] - aOriginSeg1[1],
                                                   aOriginSeg1[0] - aOriginSeg1[2]];

            for (int aTrg0Edge = 0; aTrg0Edge < 3; ++aTrg0Edge)
            {
                for (int aTrg1Edge = 0; aTrg1Edge < 3; ++aTrg1Edge)
                {
                    if (segmentsIntersected(aOriginSeg0[aTrg0Edge],
                                            aOriginSeg1[aTrg1Edge],
                                            aDirectSeg0[aTrg0Edge],
                                            aDirectSeg1[aTrg1Edge]))
                    {
                        return true; // edges intersected --> triangles overlapped
                    }
                }
            }

            if (pointInTriangle(theTrng1Vert0, theTrng0Vert0, theTrng0Vert1, theTrng0Vert2, anX, anY))
            {
                return true; // 1st triangle inside 2nd --> triangles overlapped
            }

            if (pointInTriangle(theTrng0Vert0, theTrng1Vert0, theTrng1Vert1, theTrng1Vert2, anX, anY))
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
    private static bool prismsIntersected(in BRepExtrema_BoundingPrism thePrism1, in BRepExtrema_BoundingPrism thePrism2)
    {
        if (thePrism1.Separated(thePrism2, thePrism1.Normal))
        {
            return false;
        }

        if (thePrism1.Separated(thePrism2, thePrism2.Normal))
        {
            return false;
        }

        for (int anIdx = 0; anIdx < 3; ++anIdx)
        {
            if (thePrism1.Separated(thePrism2, thePrism1.EdgeNormals[anIdx]))
            {
                return false;
            }
        }

        for (int anIdx = 0; anIdx < 3; ++anIdx)
        {
            if (thePrism1.Separated(thePrism2, thePrism2.EdgeNormals[anIdx]))
            {
                return false;
            }
        }

        for (int anIdx1 = 0; anIdx1 < 4; ++anIdx1)
        {
            BVH_Vec3d aEdge1 = (anIdx1 == 3) ? thePrism1.Normal : thePrism1.Edges[anIdx1];

            for (int anIdx2 = 0; anIdx2 < 4; ++anIdx2)
            {
                BVH_Vec3d aEdge2 = (anIdx2 == 3) ? thePrism2.Normal : thePrism2.Edges[anIdx2];

                if (thePrism1.Separated(thePrism2, BVH_Vec3d.Cross(aEdge1, aEdge2)))
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
    private static bool overlapBoxes(in BVH_Vec3d theBoxMin1,
                                     in BVH_Vec3d theBoxMax1,
                                     in BVH_Vec3d theBoxMin2,
                                     in BVH_Vec3d theBoxMax2,
                                     double theTolerance)
    {
        // Check for overlap
        return theBoxMin1.X() <= theBoxMax2.X() + theTolerance
               && theBoxMax1.X() >= theBoxMin2.X() - theTolerance
               && theBoxMin1.Y() <= theBoxMax2.Y() + theTolerance
               && theBoxMax1.Y() >= theBoxMin2.Y() - theTolerance
               && theBoxMin1.Z() <= theBoxMax2.Z() + theTolerance
               && theBoxMax1.Z() >= theBoxMin2.Z() - theTolerance;
    }

    private static NCollection_PackedMapOfInteger getSetOfFaces(NCollection_DataMap<int, NCollection_PackedMapOfInteger> theFaces,
                                                                int theFaceIdx)
    {
        if (!theFaces.IsBound(theFaceIdx))
        {
            theFaces.Bind(theFaceIdx, new NCollection_PackedMapOfInteger());
        }

        return theFaces.ChangeFind(theFaceIdx);
    }

    //=================================================================================================

    /// <summary>The narrow phase of the overlap test: exact intersection.</summary>
    protected void intersectTrianglesExact(int theTrgIdx1, int theTrgIdx2)
    {
        int aFaceIdx1 = mySet1!.GetFaceID(theTrgIdx1);

        mySet1.GetVertices(theTrgIdx1, out BVH_Vec3d aTrg1Vert1, out BVH_Vec3d aTrg1Vert2, out BVH_Vec3d aTrg1Vert3);

        bool aIsInSet = myOverlapSubShapes1.IsBound(aFaceIdx1);

        int aFaceIdx2 = mySet2!.GetFaceID(theTrgIdx2);

        if (aIsInSet && myOverlapSubShapes1.Find(aFaceIdx1).Contains(aFaceIdx2))
        {
            return;
        }

        BRepExtrema_ElementFilter.FilterResult aResult =
            myFilter is null ? BRepExtrema_ElementFilter.FilterResult.DoCheck
                             : myFilter.PreCheckElements(theTrgIdx1, theTrgIdx2);

        if (aResult == BRepExtrema_ElementFilter.FilterResult.Overlap)
        {
            getSetOfFaces(myOverlapSubShapes1, aFaceIdx1).Add(aFaceIdx2);
            getSetOfFaces(myOverlapSubShapes2, aFaceIdx2).Add(aFaceIdx1);
        }
        else if (aResult == BRepExtrema_ElementFilter.FilterResult.DoCheck)
        {
            mySet2.GetVertices(theTrgIdx2, out BVH_Vec3d aTrg2Vert1, out BVH_Vec3d aTrg2Vert2, out BVH_Vec3d aTrg2Vert3);

            if (trianglesIntersected(aTrg1Vert1, aTrg1Vert2, aTrg1Vert3, aTrg2Vert1, aTrg2Vert2, aTrg2Vert3))
            {
                getSetOfFaces(myOverlapSubShapes1, aFaceIdx1).Add(aFaceIdx2);
                getSetOfFaces(myOverlapSubShapes2, aFaceIdx2).Add(aFaceIdx1);
            }
        }
    }

    /// <summary>The narrow phase of the overlap test: intersection with a non-zero tolerance.</summary>
    protected void intersectTrianglesToler(int theTrgIdx1, int theTrgIdx2, double theToler)
    {
        int aFaceIdx1 = mySet1!.GetFaceID(theTrgIdx1);

        mySet1.GetVertices(theTrgIdx1, out BVH_Vec3d aTrg1Vert1, out BVH_Vec3d aTrg1Vert2, out BVH_Vec3d aTrg1Vert3);

        var aPrism1 = new BRepExtrema_BoundingPrism(); // not initialized

        bool aIsInSet = myOverlapSubShapes1.IsBound(aFaceIdx1);

        int aFaceIdx2 = mySet2!.GetFaceID(theTrgIdx2);

        if (aIsInSet && myOverlapSubShapes1.Find(aFaceIdx1).Contains(aFaceIdx2))
        {
            return;
        }

        BRepExtrema_ElementFilter.FilterResult aResult =
            myFilter is null ? BRepExtrema_ElementFilter.FilterResult.DoCheck
                             : myFilter.PreCheckElements(theTrgIdx1, theTrgIdx2);

        if (aResult == BRepExtrema_ElementFilter.FilterResult.Overlap)
        {
            getSetOfFaces(myOverlapSubShapes1, aFaceIdx1).Add(aFaceIdx2);
            getSetOfFaces(myOverlapSubShapes2, aFaceIdx2).Add(aFaceIdx1);
        }
        else if (aResult == BRepExtrema_ElementFilter.FilterResult.DoCheck)
        {
            if (!aPrism1.IsInited)
            {
                aPrism1.Init(aTrg1Vert1, aTrg1Vert2, aTrg1Vert3, theToler);
            }

            mySet2.GetVertices(theTrgIdx2, out BVH_Vec3d aTrg2Vert1, out BVH_Vec3d aTrg2Vert2, out BVH_Vec3d aTrg2Vert3);

            var aPrism2 = new BRepExtrema_BoundingPrism(aTrg2Vert1, aTrg2Vert2, aTrg2Vert3, theToler);

            if (prismsIntersected(aPrism1, aPrism2))
            {
                getSetOfFaces(myOverlapSubShapes1, aFaceIdx1).Add(aFaceIdx2);
                getSetOfFaces(myOverlapSubShapes2, aFaceIdx2).Add(aFaceIdx1);
            }
        }
    }

    //=======================================================================
    // function : Perform
    // purpose  : Performs search for overlapped faces
    //=======================================================================
    /// <summary>Searches for overlapped mesh elements, within the tolerance when it isn't zero.</summary>
    public void Perform(double theTolerance = 0.0)
    {
        myTolerance = theTolerance;

        myIsDone = (Select(mySet1!.BVH(), mySet2!.BVH()) > 0);
    }

    /// <summary>Rejects a pair of nodes whose boxes don't overlap (within the tolerance).</summary>
    public override bool RejectNode(in BVH_Vec3d theCornerMin1,
                                    in BVH_Vec3d theCornerMax1,
                                    in BVH_Vec3d theCornerMin2,
                                    in BVH_Vec3d theCornerMax2,
                                    out double theMetric)
    {
        theMetric = 0.0; // OCCT leaves the metric unwritten
        return !overlapBoxes(theCornerMin1, theCornerMax1, theCornerMin2, theCornerMax2, myTolerance);
    }

    /// <summary>Tests a pair of leaf elements (triangles).</summary>
    public override bool Accept(int theTrgIdx1, int theTrgIdx2)
    {
        if (myTolerance == 0.0)
        {
            intersectTrianglesExact(theTrgIdx1, theTrgIdx2);
        }
        else
        {
            intersectTrianglesToler(theTrgIdx1, theTrgIdx2, myTolerance);
        }
        return true;
    }
}
