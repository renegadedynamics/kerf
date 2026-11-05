// Port of: src/FoundationClasses/TKMath/BVH/BVH_BinnedBuilder.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;
using System.Numerics;
using static Kerf.Engine.Geometry.Bvh.Constants;

namespace Kerf.Engine.Geometry.Bvh;

/// <summary>The parameters of a single bin (slice of an AABB).</summary>
/// <remarks><c>default(BVH_Bin)</c> has a valid box at the origin; write <c>new()</c>, which makes an empty bin.</remarks>
internal struct Bin<T, TVec>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    public int Count;             //!< Number of primitives in the bin
    public Box<T, TVec> Box;  //!< AABB of primitives in the bin

    /// <summary>An empty bin.</summary>
    public Bin()
    {
        Count = 0;
        Box = new Box<T, TVec>();
    }
}

/// <summary>
/// Builds a BVH tree with the binned SAH algorithm. The number of bins trades build time for tree quality (32 to 48
/// is best; 4 to 8 is only 10-20% worse). Several threads can only be used with thread-safe primitive sets.
/// </summary>
/// <remarks>OCCT's template argument <c>Bins</c> is the constructor's last argument.</remarks>
internal class BinnedBuilder<T, TVec> : QueueBuilder<T, TVec>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    /// <summary>The number of bins (OCCT's template argument <c>Bins</c>).</summary>
    protected readonly int _bins;

    private readonly bool _useMainAxis; //!< Defines whether to search for the best split or use the widest axis

    /// <summary>A split plane candidate.</summary>
    protected struct BVH_SplitPlane
    {
        public Bin<T, TVec> LftVoxel;
        public Bin<T, TVec> RghVoxel;

        public BVH_SplitPlane()
        {
            LftVoxel = new Bin<T, TVec>();
            RghVoxel = new Bin<T, TVec>();
        }
    }

    /// <summary>Creates a binned SAH BVH builder.</summary>
    public BinnedBuilder(int leafNodeSize = BVH_Constants_LeafNodeSizeDefault,
                             int maxTreeDepth = BVH_Constants_MaxTreeDepth,
                             bool doMainSplits = false,
                             int numOfThreads = 1,
                             int bins = BVH_Constants_NbBinsOptimal)
        : base(leafNodeSize, maxTreeDepth, numOfThreads)
    {
        _bins = bins;
        _useMainAxis = doMainSplits;
    }

    /// <summary>Arranges the node's primitives into bins along the axis.</summary>
    protected virtual void GetSubVolumes(global::Kerf.Engine.Geometry.Bvh.ISet<T, TVec> set,
                                         Tree<T, TVec> bvh,
                                         int node,
                                         Span<Bin<T, TVec>> bins,
                                         int axis)
    {
        T min = BvhTools.VecComp<T, TVec>(bvh.MinPoint(node), axis);
        T max = BvhTools.VecComp<T, TVec>(bvh.MaxPoint(node), axis);
        T inverseStep = T.CreateTruncating(_bins) / (max - min);
        for (int idx = bvh.BegPrimitive(node); idx <= bvh.EndPrimitive(node); ++idx)
        {
            Box<T, TVec> box = set.Box(idx);
            int binIndex = BvhTools.IntFloor((set.Center(idx, axis) - min) * inverseStep);
            if (binIndex < 0)
            {
                binIndex = 0;
            }
            else if (binIndex >= _bins)
            {
                binIndex = _bins - 1;
            }

            bins[binIndex].Count++;
            bins[binIndex].Box.Combine(box);
        }
    }

    /// <summary>Splits the given BVH node.</summary>
    protected override BVH_ChildNodes BuildNode(global::Kerf.Engine.Geometry.Bvh.ISet<T, TVec> set, Tree<T, TVec> bvh, int node)
    {
        int nodeBegPrimitive = bvh.BegPrimitive(node);
        int nodeEndPrimitive = bvh.EndPrimitive(node);
        int nodeNbPrimitives = bvh.NbPrimitives(node);
        if (nodeNbPrimitives <= _leafNodeSize)
        {
            return new BVH_ChildNodes(); // node does not require partitioning
        }

        Box<T, TVec> aabb = new(bvh.MinPoint(node), bvh.MaxPoint(node));
        TVec size = aabb.Size();

        // Parameters for storing best split
        int minSplitAxis = -1;
        int minSplitIndex = 0;
        int minSplitNumLft = 0;
        int minSplitNumRgh = 0;

        Box<T, TVec> minSplitBoxLft = new();
        Box<T, TVec> minSplitBoxRgh = new();

        double minSplitCost = double.MaxValue;
        int mainAxis = BvhTools.MainAxis<T, TVec>(size);

        Span<Bin<T, TVec>> binVector = stackalloc Bin<T, TVec>[_bins];
        Span<BVH_SplitPlane> splitPlanes = stackalloc BVH_SplitPlane[_bins + 1];

        // Find best split
        for (int axis = _useMainAxis ? mainAxis : 0;
             axis <= (_useMainAxis ? mainAxis : Min(TVec.Length() - 1, 2));
             ++axis)
        {
            if (double.CreateTruncating(BvhTools.VecComp<T, TVec>(size, axis)) <= BvhTools.THE_NODE_MIN_SIZE)
            {
                continue;
            }

            // C++ default-constructs the arrays on each iteration
            binVector.Fill(new Bin<T, TVec>());
            splitPlanes.Fill(new BVH_SplitPlane());

            GetSubVolumes(set, bvh, node, binVector, axis);

            for (int lftSplit = 1, rghSplit = _bins - 1; lftSplit < _bins; ++lftSplit, --rghSplit)
            {
                splitPlanes[lftSplit].LftVoxel.Count =
                    splitPlanes[lftSplit - 1].LftVoxel.Count + binVector[lftSplit - 1].Count;

                splitPlanes[rghSplit].RghVoxel.Count =
                    splitPlanes[rghSplit + 1].RghVoxel.Count + binVector[rghSplit + 0].Count;

                splitPlanes[lftSplit].LftVoxel.Box = splitPlanes[lftSplit - 1].LftVoxel.Box;
                splitPlanes[rghSplit].RghVoxel.Box = splitPlanes[rghSplit + 1].RghVoxel.Box;

                splitPlanes[lftSplit].LftVoxel.Box.Combine(binVector[lftSplit - 1].Box);
                splitPlanes[rghSplit].RghVoxel.Box.Combine(binVector[rghSplit + 0].Box);
            }

            // Choose the best split (with minimum SAH cost)
            double parentArea = double.CreateTruncating(aabb.Area());
            for (int split = 1; split < _bins; ++split)
            {
                // SAH evaluation with proper normalization by parent surface area
                double lftArea = double.CreateTruncating(splitPlanes[split].LftVoxel.Box.Area());
                double rghArea = double.CreateTruncating(splitPlanes[split].RghVoxel.Box.Area());
                double cost = (lftArea / parentArea) * splitPlanes[split].LftVoxel.Count
                               + (rghArea / parentArea) * splitPlanes[split].RghVoxel.Count;

                if (cost <= minSplitCost)
                {
                    minSplitCost = cost;
                    minSplitAxis = axis;
                    minSplitIndex = split;
                    minSplitBoxLft = splitPlanes[split].LftVoxel.Box;
                    minSplitBoxRgh = splitPlanes[split].RghVoxel.Box;
                    minSplitNumLft = splitPlanes[split].LftVoxel.Count;
                    minSplitNumRgh = splitPlanes[split].RghVoxel.Count;
                }
            }
        }

        bvh.SetInner(node);
        int middle;
        if (minSplitNumLft == 0 || minSplitNumRgh == 0
            || minSplitAxis == -1) // case of objects with the same center
        {
            minSplitBoxLft.Clear();
            minSplitBoxRgh.Clear();

            middle = Max(nodeBegPrimitive + 1, (int)((nodeBegPrimitive + nodeEndPrimitive) / 2.0f));

            minSplitNumLft = middle - nodeBegPrimitive;
            for (int index = nodeBegPrimitive; index < middle; ++index)
            {
                minSplitBoxLft.Combine(set.Box(index));
            }

            minSplitNumRgh = nodeEndPrimitive - middle + 1;
            for (int index = nodeEndPrimitive; index >= middle; --index)
            {
                minSplitBoxRgh.Combine(set.Box(index));
            }
        }
        else
        {
            middle = BvhTools.SplitPrimitives(set, aabb, nodeBegPrimitive, nodeEndPrimitive,
                                          minSplitIndex - 1, minSplitAxis, _bins);
        }

        return new BVH_ChildNodes(minSplitBoxLft,
                                  minSplitBoxRgh,
                                  new BVH_PrimitiveRange(nodeBegPrimitive, middle - 1),
                                  new BVH_PrimitiveRange(middle, nodeEndPrimitive));
    }
}

internal static partial class BvhTools
{
    /// <summary>Partitions the node's primitives about the given bin along the axis; returns the first of the right part.</summary>
    public static int SplitPrimitives<T, TVec>(global::Kerf.Engine.Geometry.Bvh.ISet<T, TVec> set,
                                               in Box<T, TVec> box,
                                               int beg,
                                               int end,
                                               int bin,
                                               int axis,
                                               int bins)
        where T : unmanaged, INumber<T>, IMinMaxValue<T>
        where TVec : unmanaged, INCollection_Vec<T, TVec>
    {
        T min = VecComp<T, TVec>(box.CornerMin(), axis);
        T max = VecComp<T, TVec>(box.CornerMax(), axis);

        T inverseStep = T.CreateTruncating(bins) / (max - min);

        int lftIdx = beg;
        int rghIdx = end;

        do
        {
            while (IntFloor((set.Center(lftIdx, axis) - min) * inverseStep) <= bin
                   && lftIdx < end)
            {
                ++lftIdx;
            }

            while (IntFloor((set.Center(rghIdx, axis) - min) * inverseStep) > bin
                   && rghIdx > beg)
            {
                --rghIdx;
            }

            if (lftIdx <= rghIdx)
            {
                if (lftIdx != rghIdx)
                {
                    set.Swap(lftIdx, rghIdx);
                }

                ++lftIdx;
                --rghIdx;
            }
        } while (lftIdx <= rghIdx);

        return lftIdx;
    }

    /// <summary>The axis of the largest size (<c>BVH::BVH_AxisSelector</c>).</summary>
    public static int MainAxis<T, TVec>(in TVec size)
        where T : unmanaged, INumber<T>
        where TVec : unmanaged, INCollection_Vec<T, TVec>
    {
        if (TVec.Length() == 2)
        {
            return size[0] > size[1] ? 0 : 1;
        }

        if (size[1] > size[0])
        {
            return size[1] > size[2] ? 1 : 2;
        }
        else
        {
            return size[2] > size[0] ? 2 : 0;
        }
    }
}
