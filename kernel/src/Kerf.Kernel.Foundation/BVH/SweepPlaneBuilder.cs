// Port of: src/FoundationClasses/TKMath/BVH/BVH_SweepPlaneBuilder.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Numerics;
using static Kerf.Engine.Geometry.Bvh.Constants;

namespace Kerf.Engine.Geometry.Bvh;

/// <summary>Builds a BVH tree with the sweep plane SAH algorithm.</summary>
internal class SweepPlaneBuilder<T, TVec> : QueueBuilder<T, TVec>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    /// <summary>Creates a sweep plane SAH BVH builder.</summary>
    public SweepPlaneBuilder(int leafNodeSize = BVH_Constants_LeafNodeSizeDefault,
                                 int maxTreeDepth = BVH_Constants_MaxTreeDepth,
                                 int numOfThreads = 1)
        : base(leafNodeSize, maxTreeDepth, numOfThreads)
    {
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

        // Parameters for storing best split
        int minSplitAxis = -1;
        int minSplitIndex = 0;

        NCollection_Array1<double> lftSet = new(1, nodeNbPrimitives - 1);
        NCollection_Array1<double> rghSet = new(1, nodeNbPrimitives - 1);
        double minSplitCost = double.MaxValue;

        int n = TVec.Length();

        // Find best split
        for (int axis = 0; axis < (n < 4 ? n : 3); ++axis)
        {
            T nodeSize = BvhTools.VecComp<T, TVec>(bvh.MaxPoint(node), axis)
                          - BvhTools.VecComp<T, TVec>(bvh.MinPoint(node), axis);
            if (double.CreateTruncating(nodeSize) <= BvhTools.THE_NODE_MIN_SIZE)
            {
                continue;
            }

            new QuickSorter<T, TVec>(axis).Perform(set, nodeBegPrimitive, nodeEndPrimitive);

            Box<T, TVec> lftBox = new();
            Box<T, TVec> rghBox = new();

            // Sweep from left
            for (int index = 1; index < nodeNbPrimitives; ++index)
            {
                lftBox.Combine(set.Box(index + nodeBegPrimitive - 1));

                lftSet[index] = double.CreateTruncating(lftBox.Area());
            }

            // Sweep from right
            for (int index = 1; index < nodeNbPrimitives; ++index)
            {
                rghBox.Combine(set.Box(nodeEndPrimitive - index + 1));

                rghSet[index] = double.CreateTruncating(rghBox.Area());
            }

            // Find best split using simplified SAH
            for (int nbLft = 1, nbRgh = nodeNbPrimitives - 1; nbLft < nodeNbPrimitives; ++nbLft, --nbRgh)
            {
                double cost = (lftSet[nbLft] /* / aNodeArea */) * nbLft
                               + (rghSet[nbRgh] /* / aNodeArea */) * nbRgh;

                if (cost < minSplitCost)
                {
                    minSplitCost = cost;
                    minSplitAxis = axis;
                    minSplitIndex = nbLft;
                }
            }
        }

        if (minSplitAxis == -1)
        {
            return new BVH_ChildNodes(); // failed to find split axis
        }

        bvh.SetInner(node);

        if (minSplitAxis != (n < 4 ? n - 1 : 2))
        {
            new QuickSorter<T, TVec>(minSplitAxis).Perform(set, nodeBegPrimitive, nodeEndPrimitive);
        }

        Box<T, TVec> minSplitBoxLft = new();
        Box<T, TVec> minSplitBoxRgh = new();

        // Compute bounding boxes for selected split plane
        for (int index = nodeBegPrimitive; index < minSplitIndex + nodeBegPrimitive; ++index)
        {
            minSplitBoxLft.Combine(set.Box(index));
        }

        for (int index = nodeEndPrimitive; index >= minSplitIndex + nodeBegPrimitive; --index)
        {
            minSplitBoxRgh.Combine(set.Box(index));
        }

        int middle = nodeBegPrimitive + minSplitIndex;

        return new BVH_ChildNodes(minSplitBoxLft,
                                  minSplitBoxRgh,
                                  new BVH_PrimitiveRange(nodeBegPrimitive, middle - 1),
                                  new BVH_PrimitiveRange(middle, nodeEndPrimitive));
    }
}
