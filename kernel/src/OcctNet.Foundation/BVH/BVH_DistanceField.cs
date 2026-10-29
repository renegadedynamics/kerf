// Port of: src/FoundationClasses/TKMath/BVH/BVH_DistanceField.hxx
// Port of: src/FoundationClasses/TKMath/BVH/BVH_DistanceField.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2014-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Numerics;

namespace Occt;

/// <summary>
/// Builds a 3D distance field from a set of BVH triangulations: a grid of the distances from each voxel's centre to
/// the geometry, signed (negative inside) if asked.
/// </summary>
/// <remarks>For three or four dimensions (OCCT asserts N = 3 or 4 statically); the port checks at construction.</remarks>
public class BVH_DistanceField<T, TVec>
    where T : unmanaged, IFloatingPointIeee754<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    /// <summary>Array of voxels.</summary>
    protected T[] myVoxelData;

    /// <summary>Size of single voxel.</summary>
    protected TVec myVoxelSize;

    /// <summary>Minimum corner of voxel grid.</summary>
    protected TVec myCornerMin;

    /// <summary>Maximum corner of voxel grid.</summary>
    protected TVec myCornerMax;

    protected int myDimensionX;   //!< Size of voxel grid in X dimension
    protected int myDimensionY;   //!< Size of voxel grid in Y dimension
    protected int myDimensionZ;   //!< Size of voxel grid in Z dimension
    protected int myMaximumSize;  //!< Size of voxel grid in maximum dimension
    protected bool myComputeSign; //!< Enables/disables signing of distance field
    protected bool myIsParallel;

    /// <summary>Creates an empty 3D distance field.</summary>
    public BVH_DistanceField(int theMaximumSize, bool theComputeSign)
    {
        Standard_DimensionError_Raise_if(TVec.Length() != 3 && TVec.Length() != 4, "BVH_DistanceField: N must be 3 or 4");
        myDimensionX = 0;
        myDimensionY = 0;
        myDimensionZ = 0;
        myMaximumSize = theMaximumSize;
        myComputeSign = theComputeSign;
        myIsParallel = false;

        myVoxelData = new T[myMaximumSize * myMaximumSize * myMaximumSize];
    }

    /// <summary>The parallel flag.</summary>
    public bool IsParallel() => myIsParallel;

    /// <summary>Sets the flag allowing parallel execution.</summary>
    public void SetParallel(bool isParallel) => myIsParallel = isParallel;

    /// <summary>The packed voxel data.</summary>
    public ReadOnlySpan<T> PackedData() => myVoxelData;

    /// <summary>The distance value of the given voxel.</summary>
    public ref T Voxel(int theX, int theY, int theZ) => ref myVoxelData[theX + (theY + theZ * myDimensionY) * myDimensionX];

    public int DimensionX() => myDimensionX;

    public int DimensionY() => myDimensionY;

    public int DimensionZ() => myDimensionZ;

    /// <summary>The size of a single voxel.</summary>
    public TVec VoxelSize() => myVoxelSize;

    /// <summary>The minimum corner of the voxel grid.</summary>
    public TVec CornerMin() => myCornerMin;

    /// <summary>The maximum corner of the voxel grid.</summary>
    public TVec CornerMax() => myCornerMax;

    /// <summary>Builds the distance field of the given geometry; false if it is empty.</summary>
    public bool Build(BVH_Geometry<T, TVec> theGeometry)
    {
        if (theGeometry.Size() == 0)
        {
            return false;
        }

        TVec aGlobalBoxSize = theGeometry.Box().Size();

        T aMaxBoxSide = BVH.Max(BVH.Max(aGlobalBoxSize[0], aGlobalBoxSize[1]), aGlobalBoxSize[2]);

        myDimensionX = int.CreateTruncating(T.CreateTruncating(myMaximumSize) * aGlobalBoxSize[0] / aMaxBoxSide);
        myDimensionY = int.CreateTruncating(T.CreateTruncating(myMaximumSize) * aGlobalBoxSize[1] / aMaxBoxSide);
        myDimensionZ = int.CreateTruncating(T.CreateTruncating(myMaximumSize) * aGlobalBoxSize[2] / aMaxBoxSide);

        myDimensionX = Min(myMaximumSize, Max(myDimensionX, 16));
        myDimensionY = Min(myMaximumSize, Max(myDimensionY, 16));
        myDimensionZ = Min(myMaximumSize, Max(myDimensionZ, 16));

        TVec aGlobalBoxMin = theGeometry.Box().CornerMin();
        TVec aGlobalBoxMax = theGeometry.Box().CornerMax();

        const int aVoxelOffset = 2;

        T anOffset = T.CreateTruncating(aVoxelOffset);
        Span<T> aCornerMin = NCollection_VecTools.ChangeData<T, TVec>(ref myCornerMin);
        Span<T> aCornerMax = NCollection_VecTools.ChangeData<T, TVec>(ref myCornerMax);
        Span<T> aVoxelSize = NCollection_VecTools.ChangeData<T, TVec>(ref myVoxelSize);

        aCornerMin[0] = aGlobalBoxMin[0] - anOffset * aGlobalBoxSize[0] / T.CreateTruncating(myDimensionX - 2 * aVoxelOffset);
        aCornerMin[1] = aGlobalBoxMin[1] - anOffset * aGlobalBoxSize[1] / T.CreateTruncating(myDimensionY - 2 * aVoxelOffset);
        aCornerMin[2] = aGlobalBoxMin[2] - anOffset * aGlobalBoxSize[2] / T.CreateTruncating(myDimensionZ - 2 * aVoxelOffset);

        aCornerMax[0] = aGlobalBoxMax[0] + anOffset * aGlobalBoxSize[0] / T.CreateTruncating(myDimensionX - 2 * aVoxelOffset);
        aCornerMax[1] = aGlobalBoxMax[1] + anOffset * aGlobalBoxSize[1] / T.CreateTruncating(myDimensionY - 2 * aVoxelOffset);
        aCornerMax[2] = aGlobalBoxMax[2] + anOffset * aGlobalBoxSize[2] / T.CreateTruncating(myDimensionZ - 2 * aVoxelOffset);

        aVoxelSize[0] = (aCornerMax[0] - aCornerMin[0]) / T.CreateTruncating(myDimensionX);
        aVoxelSize[1] = (aCornerMax[1] - aCornerMin[1]) / T.CreateTruncating(myDimensionY);
        aVoxelSize[2] = (aCornerMax[2] - aCornerMin[2]) / T.CreateTruncating(myDimensionZ);

        // OSD_Parallel::For(0, myDimensionZ, BVH_ParallelDistanceFieldBuilder, !IsParallel())
        OSD_Parallel.For(0, myDimensionZ, theIndex => BuildSlices(theGeometry, theIndex, theIndex + 1), !IsParallel());

        return true;
    }

    /// <summary>Builds the distance field for the given Z slices.</summary>
    protected void BuildSlices(BVH_Geometry<T, TVec> theGeometry, int theStartSlice, int theFinalSlice)
    {
        T aHalf = T.CreateTruncating(0.5);
        for (int aZ = theStartSlice; aZ < theFinalSlice; ++aZ)
        {
            for (int aY = 0; aY < myDimensionY; ++aY)
            {
                for (int aX = 0; aX < myDimensionX; ++aX)
                {
                    TVec aCenter = default;
                    Span<T> aCenterData = NCollection_VecTools.ChangeData<T, TVec>(ref aCenter);

                    aCenterData[0] = myCornerMin[0] + myVoxelSize[0] * (T.CreateTruncating(aX) + aHalf);
                    aCenterData[1] = myCornerMin[1] + myVoxelSize[1] * (T.CreateTruncating(aY) + aHalf);
                    aCenterData[2] = myCornerMin[2] + myVoxelSize[2] * (T.CreateTruncating(aZ) + aHalf);

                    bool isOutside = true;
                    T aDistance = T.Sqrt(BVH_DistanceFieldTools<T, TVec>.SquareDistanceToGeomerty(theGeometry, aCenter, ref isOutside));

                    Voxel(aX, aY, aZ) = (!myComputeSign || isOutside) ? aDistance : -aDistance;
                }
            }
        }
    }
}

/// <summary>The helpers of <see cref="BVH_DistanceField{T, TVec}"/> (OCCT's <c>BVH</c> namespace in BVH_DistanceField.lxx).</summary>
public static class BVH_DistanceFieldTools<T, TVec>
    where T : unmanaged, IFloatingPointIeee754<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    // BVH_DOT3
    private static T Dot3(in TVec A, in TVec B) => A[0] * B[0] + A[1] * B[1] + A[2] * B[2];

    /// <summary>The square distance from a point to a box (zero inside).</summary>
    public static T DistanceToBox(in TVec thePnt, in TVec theMin, in TVec theMax)
    {
        T aNearestX = BVH.Min(BVH.Max(thePnt[0], theMin[0]), theMax[0]);
        T aNearestY = BVH.Min(BVH.Max(thePnt[1], theMin[1]), theMax[1]);
        T aNearestZ = BVH.Min(BVH.Max(thePnt[2], theMin[2]), theMax[2]);

        if (aNearestX == thePnt[0] && aNearestY == thePnt[1] && aNearestZ == thePnt[2])
        {
            return T.Zero;
        }

        aNearestX -= thePnt[0];
        aNearestY -= thePnt[1];
        aNearestZ -= thePnt[2];

        return aNearestX * aNearestX + aNearestY * aNearestY + aNearestZ * aNearestZ;
    }

    /// <summary>The vector from the nearest point of a triangle to the given point.</summary>
    public static TVec DirectionToNearestPoint(in TVec thePoint, in TVec theVertA, in TVec theVertB, in TVec theVertC)
    {
        TVec aAB = theVertB - theVertA;
        TVec aAC = theVertC - theVertA;
        TVec aAP = thePoint - theVertA;

        T aABdotAP = Dot3(aAB, aAP);
        T aACdotAP = Dot3(aAC, aAP);

        if (aABdotAP <= T.Zero && aACdotAP <= T.Zero)
        {
            return aAP;
        }

        TVec aBC = theVertC - theVertB;
        TVec aBP = thePoint - theVertB;

        T aBAdotBP = -Dot3(aAB, aBP);
        T aBCdotBP = Dot3(aBC, aBP);

        if (aBAdotBP <= T.Zero && aBCdotBP <= T.Zero)
        {
            return aBP;
        }

        TVec aCP = thePoint - theVertC;

        T aCBdotCP = -Dot3(aBC, aCP);
        T aCAdotCP = -Dot3(aAC, aCP);

        if (aCAdotCP <= T.Zero && aCBdotCP <= T.Zero)
        {
            return aCP;
        }

        T aACdotBP = Dot3(aAC, aBP);

        T aVC = aABdotAP * aACdotBP + aBAdotBP * aACdotAP;

        if (aVC <= T.Zero && aABdotAP >= T.Zero && aBAdotBP >= T.Zero)
        {
            return aAP - aAB * (aABdotAP / (aABdotAP + aBAdotBP));
        }

        T aABdotCP = Dot3(aAB, aCP);

        T aVA = aBAdotBP * aCAdotCP - aABdotCP * aACdotBP;

        if (aVA <= T.Zero && aBCdotBP >= T.Zero && aCBdotCP >= T.Zero)
        {
            return aBP - aBC * (aBCdotBP / (aBCdotBP + aCBdotCP));
        }

        T aVB = aABdotCP * aACdotAP + aABdotAP * aCAdotCP;

        if (aVB <= T.Zero && aACdotAP >= T.Zero && aCAdotCP >= T.Zero)
        {
            return aAP - aAC * (aACdotAP / (aACdotAP + aCAdotCP));
        }

        T aNorm = T.One / (aVA + aVB + aVC);

        T aU = aVA * aNorm;
        T aV = aVB * aNorm;

        return thePoint - (theVertA * aU + theVertB * aV + theVertC * (T.One - aU - aV));
    }

    /// <summary>The square distance from a point to a set, with whether the point is outside.</summary>
    public abstract class SquareDistanceToPoint<TSet> : BVH_Distance<T, TVec, TVec, TSet>
        where TSet : class
    {
        protected bool myIsOutside;

        protected SquareDistanceToPoint()
        {
            myIsOutside = true;
        }

        public bool IsOutside() => myIsOutside;

        /// <summary>Rejects the node by its (square) distance from the point.</summary>
        public override bool RejectNode(in TVec theCMin, in TVec theCMax, out T theMetric)
        {
            theMetric = DistanceToBox(myObject, theCMin, theCMax);
            return theMetric > myDistance;
        }

        /// <summary>Never stops the selection.</summary>
        public override bool Stop() => false;
    }

    /// <summary>The square distance from a point to a triangulation.</summary>
    public sealed class PointTriangulationSquareDistance : SquareDistanceToPoint<BVH_Triangulation<T, TVec>>
    {
        public override bool Accept(int theIndex, in T theMetric)
        {
            NCollection_Vec4<int> aTriangle = myBVHSet!.Elements[theIndex];

            TVec aVertex0 = myBVHSet.Vertices[aTriangle.x()];
            TVec aVertex1 = myBVHSet.Vertices[aTriangle.y()];
            TVec aVertex2 = myBVHSet.Vertices[aTriangle.z()];

            TVec aDirection = DirectionToNearestPoint(myObject, aVertex0, aVertex1, aVertex2);

            T aDistance = Dot3(aDirection, aDirection);

            if (aDistance < myDistance)
            {
                myDistance = aDistance;

                TVec aTrgEdge0 = aVertex1 - aVertex0;
                TVec aTrgEdge1 = aVertex2 - aVertex0;

                TVec aTrgNormal = default;
                Span<T> aNormal = NCollection_VecTools.ChangeData<T, TVec>(ref aTrgNormal);

                aNormal[0] = aTrgEdge0[1] * aTrgEdge1[2] - aTrgEdge0[2] * aTrgEdge1[1];
                aNormal[1] = aTrgEdge0[2] * aTrgEdge1[0] - aTrgEdge0[0] * aTrgEdge1[2];
                aNormal[2] = aTrgEdge0[0] * aTrgEdge1[1] - aTrgEdge0[1] * aTrgEdge1[0];

                myIsOutside = Dot3(aTrgNormal, aDirection) > T.Zero;

                return true;
            }

            return false;
        }
    }

    /// <summary>The square distance from a point to an object, which must be a triangulation.</summary>
    public static T SquareDistanceToObject(BVH_Object<T, TVec>? theObject, in TVec thePnt, ref bool theIsOutside)
    {
        T aMinDistance = T.MaxValue;

        if (theObject is not BVH_Triangulation<T, TVec> aTriangulation)
        {
            // Standard_ASSERT_RETURN: "Error: Unsupported BVH object (non triangulation)"
            return aMinDistance;
        }

        BVH_Tree<T, TVec>? aBVH = aTriangulation.BVH();

        if (aBVH is null)
        {
            return T.Zero; // OCCT: returns false (zero), a zero distance, though aMinDistance was meant
        }

        PointTriangulationSquareDistance aDistTool = new();
        aDistTool.SetObject(thePnt);
        aDistTool.SetBVHSet(aTriangulation);
        aDistTool.ComputeDistance();
        theIsOutside = aDistTool.IsOutside();
        return aDistTool.Distance();
    }

    /// <summary>The square distance from a point to a geometry of triangulations.</summary>
    public sealed class PointGeometrySquareDistance : SquareDistanceToPoint<BVH_Geometry<T, TVec>>
    {
        public override bool Accept(int theIndex, in T theMetric)
        {
            bool isOutside = true;
            T aDistance = SquareDistanceToObject(myBVHSet!.Objects()[theIndex], myObject, ref isOutside);

            if (aDistance < myDistance)
            {
                myDistance = aDistance;
                myIsOutside = isOutside;
                return true;
            }

            return false;
        }
    }

    /// <summary>The square distance from a point to a geometry (OCCT's name, with its typo).</summary>
    public static T SquareDistanceToGeomerty(BVH_Geometry<T, TVec> theGeometry, in TVec thePnt, ref bool theIsOutside)
    {
        BVH_Tree<T, TVec>? aBVH = theGeometry.BVH();

        if (aBVH is null)
        {
            return T.Zero; // OCCT: returns false (zero)
        }

        PointGeometrySquareDistance aDistTool = new();
        aDistTool.SetObject(thePnt);
        aDistTool.SetBVHSet(theGeometry);
        aDistTool.ComputeDistance();
        theIsOutside = aDistTool.IsOutside();
        return aDistTool.Distance();
    }
}
