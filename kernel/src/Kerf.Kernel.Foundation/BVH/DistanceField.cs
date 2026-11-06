// Port of: src/FoundationClasses/TKMath/BVH/BVH_DistanceField.hxx
// Port of: src/FoundationClasses/TKMath/BVH/BVH_DistanceField.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2014-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Numerics;

namespace Kerf.Engine.Geometry.Bvh;

/// <summary>
/// Builds a 3D distance field from a set of BVH triangulations: a grid of the distances from each voxel's centre to
/// the geometry, signed (negative inside) if asked.
/// </summary>
/// <remarks>For three or four dimensions (OCCT asserts N = 3 or 4 statically); the port checks at construction.</remarks>
internal class DistanceField<T, TVec>
    where T : unmanaged, IFloatingPointIeee754<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    /// <summary>Array of voxels.</summary>
    protected T[] _voxelData;

    /// <summary>Size of single voxel.</summary>
    protected TVec _voxelSize;

    /// <summary>Minimum corner of voxel grid.</summary>
    protected TVec _cornerMin;

    /// <summary>Maximum corner of voxel grid.</summary>
    protected TVec _cornerMax;

    protected int _dimensionX;   //!< Size of voxel grid in X dimension
    protected int _dimensionY;   //!< Size of voxel grid in Y dimension
    protected int _dimensionZ;   //!< Size of voxel grid in Z dimension
    protected int _maximumSize;  //!< Size of voxel grid in maximum dimension
    protected bool _computeSign; //!< Enables/disables signing of distance field
    protected bool _isParallel;

    /// <summary>Creates an empty 3D distance field.</summary>
    public DistanceField(int maximumSize, bool computeSign)
    {
        Standard_DimensionError_Raise_if(TVec.Length() != 3 && TVec.Length() != 4, "BVH_DistanceField: N must be 3 or 4");
        _dimensionX = 0;
        _dimensionY = 0;
        _dimensionZ = 0;
        _maximumSize = maximumSize;
        _computeSign = computeSign;
        _isParallel = false;

        _voxelData = new T[_maximumSize * _maximumSize * _maximumSize];
    }

    /// <summary>The parallel flag.</summary>
    public bool IsParallel => _isParallel;

    /// <summary>Sets the flag allowing parallel execution.</summary>
    public void SetParallel(bool isParallel) => _isParallel = isParallel;

    /// <summary>The packed voxel data.</summary>
    public ReadOnlySpan<T> PackedData() => _voxelData;

    /// <summary>The distance value of the given voxel.</summary>
    public ref T Voxel(int x, int y, int z) => ref _voxelData[x + (y + z * _dimensionY) * _dimensionX];

    public int DimensionX() => _dimensionX;

    public int DimensionY() => _dimensionY;

    public int DimensionZ() => _dimensionZ;

    /// <summary>The size of a single voxel.</summary>
    public TVec VoxelSize() => _voxelSize;

    /// <summary>The minimum corner of the voxel grid.</summary>
    public TVec CornerMin() => _cornerMin;

    /// <summary>The maximum corner of the voxel grid.</summary>
    public TVec CornerMax() => _cornerMax;

    /// <summary>Builds the distance field of the given geometry; false if it is empty.</summary>
    public bool Build(BvhGeometry<T, TVec> geometry)
    {
        if (geometry.Size() == 0)
        {
            return false;
        }

        TVec globalBoxSize = geometry.Box().Size();

        T maxBoxSide = BvhTools.Max(BvhTools.Max(globalBoxSize[0], globalBoxSize[1]), globalBoxSize[2]);

        _dimensionX = int.CreateTruncating(T.CreateTruncating(_maximumSize) * globalBoxSize[0] / maxBoxSide);
        _dimensionY = int.CreateTruncating(T.CreateTruncating(_maximumSize) * globalBoxSize[1] / maxBoxSide);
        _dimensionZ = int.CreateTruncating(T.CreateTruncating(_maximumSize) * globalBoxSize[2] / maxBoxSide);

        _dimensionX = Min(_maximumSize, Max(_dimensionX, 16));
        _dimensionY = Min(_maximumSize, Max(_dimensionY, 16));
        _dimensionZ = Min(_maximumSize, Max(_dimensionZ, 16));

        TVec globalBoxMin = geometry.Box().CornerMin();
        TVec globalBoxMax = geometry.Box().CornerMax();

        const int voxelOffset = 2;

        T offset = T.CreateTruncating(voxelOffset);
        Span<T> cornerMin = NCollection_VecTools.ChangeData<T, TVec>(ref _cornerMin);
        Span<T> cornerMax = NCollection_VecTools.ChangeData<T, TVec>(ref _cornerMax);
        Span<T> voxelSize = NCollection_VecTools.ChangeData<T, TVec>(ref _voxelSize);

        cornerMin[0] = globalBoxMin[0] - offset * globalBoxSize[0] / T.CreateTruncating(_dimensionX - 2 * voxelOffset);
        cornerMin[1] = globalBoxMin[1] - offset * globalBoxSize[1] / T.CreateTruncating(_dimensionY - 2 * voxelOffset);
        cornerMin[2] = globalBoxMin[2] - offset * globalBoxSize[2] / T.CreateTruncating(_dimensionZ - 2 * voxelOffset);

        cornerMax[0] = globalBoxMax[0] + offset * globalBoxSize[0] / T.CreateTruncating(_dimensionX - 2 * voxelOffset);
        cornerMax[1] = globalBoxMax[1] + offset * globalBoxSize[1] / T.CreateTruncating(_dimensionY - 2 * voxelOffset);
        cornerMax[2] = globalBoxMax[2] + offset * globalBoxSize[2] / T.CreateTruncating(_dimensionZ - 2 * voxelOffset);

        voxelSize[0] = (cornerMax[0] - cornerMin[0]) / T.CreateTruncating(_dimensionX);
        voxelSize[1] = (cornerMax[1] - cornerMin[1]) / T.CreateTruncating(_dimensionY);
        voxelSize[2] = (cornerMax[2] - cornerMin[2]) / T.CreateTruncating(_dimensionZ);

        // Kerf: the trees are built here, not by the first slice to ask: building one reorders its elements, under any
        // other slice reading them (decision 19804). OCCT builds them on the slices' threads
        _ = geometry.BVH();
        foreach (global::Kerf.Engine.Geometry.Bvh.Object<T, TVec>? anObject in geometry.Objects())
        {
            _ = (anObject as Triangulation<T, TVec>)?.BVH();
        }

        // OSD_Parallel::For(0, myDimensionZ, BVH_ParallelDistanceFieldBuilder, !IsParallel())
        OsdParallel.For(0, _dimensionZ, index => BuildSlices(geometry, index, index + 1), !IsParallel);

        return true;
    }

    /// <summary>Builds the distance field for the given Z slices.</summary>
    protected void BuildSlices(BvhGeometry<T, TVec> geometry, int startSlice, int finalSlice)
    {
        T half = T.CreateTruncating(0.5);
        for (int z = startSlice; z < finalSlice; ++z)
        {
            for (int y = 0; y < _dimensionY; ++y)
            {
                for (int x = 0; x < _dimensionX; ++x)
                {
                    TVec center = default;
                    Span<T> centerData = NCollection_VecTools.ChangeData<T, TVec>(ref center);

                    centerData[0] = _cornerMin[0] + _voxelSize[0] * (T.CreateTruncating(x) + half);
                    centerData[1] = _cornerMin[1] + _voxelSize[1] * (T.CreateTruncating(y) + half);
                    centerData[2] = _cornerMin[2] + _voxelSize[2] * (T.CreateTruncating(z) + half);

                    bool isOutside = true;
                    T distance = T.Sqrt(DistanceFieldTools<T, TVec>.SquareDistanceToGeomerty(geometry, center, ref isOutside));

                    Voxel(x, y, z) = (!_computeSign || isOutside) ? distance : -distance;
                }
            }
        }
    }
}

/// <summary>The helpers of <see cref="DistanceField{T, TVec}"/> (OCCT's <c>BVH</c> namespace in BVH_DistanceField.lxx).</summary>
internal static class DistanceFieldTools<T, TVec>
    where T : unmanaged, IFloatingPointIeee754<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    // BVH_DOT3
    private static T Dot3(in TVec a, in TVec b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

    /// <summary>The square distance from a point to a box (zero inside).</summary>
    public static T DistanceToBox(in TVec pnt, in TVec min, in TVec max)
    {
        T nearestX = BvhTools.Min(BvhTools.Max(pnt[0], min[0]), max[0]);
        T nearestY = BvhTools.Min(BvhTools.Max(pnt[1], min[1]), max[1]);
        T nearestZ = BvhTools.Min(BvhTools.Max(pnt[2], min[2]), max[2]);

        if (nearestX == pnt[0] && nearestY == pnt[1] && nearestZ == pnt[2])
        {
            return T.Zero;
        }

        nearestX -= pnt[0];
        nearestY -= pnt[1];
        nearestZ -= pnt[2];

        return nearestX * nearestX + nearestY * nearestY + nearestZ * nearestZ;
    }

    /// <summary>The vector from the nearest point of a triangle to the given point.</summary>
    public static TVec DirectionToNearestPoint(in TVec point, in TVec vertA, in TVec vertB, in TVec vertC)
    {
        TVec ab = vertB - vertA;
        TVec ac = vertC - vertA;
        TVec ap = point - vertA;

        T aBdotAP = Dot3(ab, ap);
        T aCdotAP = Dot3(ac, ap);

        if (aBdotAP <= T.Zero && aCdotAP <= T.Zero)
        {
            return ap;
        }

        TVec bc = vertC - vertB;
        TVec bp = point - vertB;

        T bAdotBP = -Dot3(ab, bp);
        T bCdotBP = Dot3(bc, bp);

        if (bAdotBP <= T.Zero && bCdotBP <= T.Zero)
        {
            return bp;
        }

        TVec cp = point - vertC;

        T cBdotCP = -Dot3(bc, cp);
        T cAdotCP = -Dot3(ac, cp);

        if (cAdotCP <= T.Zero && cBdotCP <= T.Zero)
        {
            return cp;
        }

        T aCdotBP = Dot3(ac, bp);

        T vc = aBdotAP * aCdotBP + bAdotBP * aCdotAP;

        if (vc <= T.Zero && aBdotAP >= T.Zero && bAdotBP >= T.Zero)
        {
            return ap - ab * (aBdotAP / (aBdotAP + bAdotBP));
        }

        T aBdotCP = Dot3(ab, cp);

        T va = bAdotBP * cAdotCP - aBdotCP * aCdotBP;

        if (va <= T.Zero && bCdotBP >= T.Zero && cBdotCP >= T.Zero)
        {
            return bp - bc * (bCdotBP / (bCdotBP + cBdotCP));
        }

        T vb = aBdotCP * aCdotAP + aBdotAP * cAdotCP;

        if (vb <= T.Zero && aCdotAP >= T.Zero && cAdotCP >= T.Zero)
        {
            return ap - ac * (aCdotAP / (aCdotAP + cAdotCP));
        }

        T norm = T.One / (va + vb + vc);

        T u = va * norm;
        T v = vb * norm;

        return point - (vertA * u + vertB * v + vertC * (T.One - u - v));
    }

    /// <summary>The square distance from a point to a set, with whether the point is outside.</summary>
    public abstract class SquareDistanceToPoint<TSet> : BvhDistance<T, TVec, TVec, TSet>
        where TSet : class
    {
        protected bool _isOutside;

        protected SquareDistanceToPoint()
        {
            _isOutside = true;
        }

        public bool IsOutside => _isOutside;

        /// <summary>Rejects the node by its (square) distance from the point.</summary>
        public override bool RejectNode(in TVec cMin, in TVec cMax, out T metric)
        {
            metric = DistanceToBox(_object, cMin, cMax);
            return metric > _distance;
        }

        /// <summary>Never stops the selection.</summary>
        public override bool Stop() => false;
    }

    /// <summary>The square distance from a point to a triangulation.</summary>
    public sealed class PointTriangulationSquareDistance : SquareDistanceToPoint<Triangulation<T, TVec>>
    {
        public override bool Accept(int index, in T metric)
        {
            NCollection_Vec4<int> triangle = _bvhSet!.Elements[index];

            TVec vertex0 = _bvhSet.Vertices[triangle.X()];
            TVec vertex1 = _bvhSet.Vertices[triangle.Y()];
            TVec vertex2 = _bvhSet.Vertices[triangle.Z()];

            TVec direction = DirectionToNearestPoint(_object, vertex0, vertex1, vertex2);

            T distance = Dot3(direction, direction);

            if (distance < _distance)
            {
                _distance = distance;

                TVec trgEdge0 = vertex1 - vertex0;
                TVec trgEdge1 = vertex2 - vertex0;

                TVec trgNormal = default;
                Span<T> normal = NCollection_VecTools.ChangeData<T, TVec>(ref trgNormal);

                normal[0] = trgEdge0[1] * trgEdge1[2] - trgEdge0[2] * trgEdge1[1];
                normal[1] = trgEdge0[2] * trgEdge1[0] - trgEdge0[0] * trgEdge1[2];
                normal[2] = trgEdge0[0] * trgEdge1[1] - trgEdge0[1] * trgEdge1[0];

                _isOutside = Dot3(trgNormal, direction) > T.Zero;

                return true;
            }

            return false;
        }
    }

    /// <summary>The square distance from a point to an object, which must be a triangulation.</summary>
    public static T SquareDistanceToObject(global::Kerf.Engine.Geometry.Bvh.Object<T, TVec>? theObject, in TVec pnt, ref bool isOutside)
    {
        T minDistance = T.MaxValue;

        if (theObject is not Triangulation<T, TVec> triangulation)
        {
            // Standard_ASSERT_RETURN: "Error: Unsupported BVH object (non triangulation)"
            return minDistance;
        }

        Tree<T, TVec>? bvh = triangulation.BVH();

        if (bvh is null)
        {
            return T.Zero; // OCCT: returns false (zero), a zero distance, though aMinDistance was meant
        }

        PointTriangulationSquareDistance distTool = new();
        distTool.SetObject(pnt);
        distTool.SetBVHSet(triangulation);
        distTool.ComputeDistance();
        isOutside = distTool.IsOutside;
        return distTool.Distance();
    }

    /// <summary>The square distance from a point to a geometry of triangulations.</summary>
    public sealed class PointGeometrySquareDistance : SquareDistanceToPoint<BvhGeometry<T, TVec>>
    {
        public override bool Accept(int index, in T metric)
        {
            bool isOutside = true;
            T distance = SquareDistanceToObject(_bvhSet!.Objects()[index], _object, ref isOutside);

            if (distance < _distance)
            {
                _distance = distance;
                _isOutside = isOutside;
                return true;
            }

            return false;
        }
    }

    /// <summary>The square distance from a point to a geometry (OCCT's name, with its typo).</summary>
    public static T SquareDistanceToGeomerty(BvhGeometry<T, TVec> geometry, in TVec pnt, ref bool isOutside)
    {
        Tree<T, TVec>? bvh = geometry.BVH();

        if (bvh is null)
        {
            return T.Zero; // OCCT: returns false (zero)
        }

        PointGeometrySquareDistance distTool = new();
        distTool.SetObject(pnt);
        distTool.SetBVHSet(geometry);
        distTool.ComputeDistance();
        isOutside = distTool.IsOutside;
        return distTool.Distance();
    }
}
