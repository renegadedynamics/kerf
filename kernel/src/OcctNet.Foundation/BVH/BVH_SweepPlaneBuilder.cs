// Port of: src/FoundationClasses/TKMath/BVH/BVH_SweepPlaneBuilder.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Numerics;
using static Occt.BVH_Constants;

namespace Occt;

/// <summary>Builds a BVH tree with the sweep plane SAH algorithm.</summary>
public class BVH_SweepPlaneBuilder<T, TVec> : BVH_QueueBuilder<T, TVec>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    /// <summary>Creates a sweep plane SAH BVH builder.</summary>
    public BVH_SweepPlaneBuilder(int theLeafNodeSize = BVH_Constants_LeafNodeSizeDefault,
                                 int theMaxTreeDepth = BVH_Constants_MaxTreeDepth,
                                 int theNumOfThreads = 1)
        : base(theLeafNodeSize, theMaxTreeDepth, theNumOfThreads)
    {
    }

    /// <summary>Splits the given BVH node.</summary>
    protected override BVH_ChildNodes buildNode(BVH_Set<T, TVec> theSet, BVH_Tree<T, TVec> theBVH, int theNode)
    {
        int aNodeBegPrimitive = theBVH.BegPrimitive(theNode);
        int aNodeEndPrimitive = theBVH.EndPrimitive(theNode);
        int aNodeNbPrimitives = theBVH.NbPrimitives(theNode);

        if (aNodeNbPrimitives <= myLeafNodeSize)
        {
            return new BVH_ChildNodes(); // node does not require partitioning
        }

        // Parameters for storing best split
        int aMinSplitAxis = -1;
        int aMinSplitIndex = 0;

        NCollection_Array1<double> aLftSet = new(1, aNodeNbPrimitives - 1);
        NCollection_Array1<double> aRghSet = new(1, aNodeNbPrimitives - 1);
        double aMinSplitCost = double.MaxValue;

        int N = TVec.Length();

        // Find best split
        for (int anAxis = 0; anAxis < (N < 4 ? N : 3); ++anAxis)
        {
            T aNodeSize = BVH.VecComp<T, TVec>(theBVH.MaxPoint(theNode), anAxis)
                          - BVH.VecComp<T, TVec>(theBVH.MinPoint(theNode), anAxis);
            if (double.CreateTruncating(aNodeSize) <= BVH.THE_NODE_MIN_SIZE)
            {
                continue;
            }

            new BVH_QuickSorter<T, TVec>(anAxis).Perform(theSet, aNodeBegPrimitive, aNodeEndPrimitive);

            BVH_Box<T, TVec> aLftBox = new();
            BVH_Box<T, TVec> aRghBox = new();

            // Sweep from left
            for (int anIndex = 1; anIndex < aNodeNbPrimitives; ++anIndex)
            {
                aLftBox.Combine(theSet.Box(anIndex + aNodeBegPrimitive - 1));

                aLftSet[anIndex] = double.CreateTruncating(aLftBox.Area());
            }

            // Sweep from right
            for (int anIndex = 1; anIndex < aNodeNbPrimitives; ++anIndex)
            {
                aRghBox.Combine(theSet.Box(aNodeEndPrimitive - anIndex + 1));

                aRghSet[anIndex] = double.CreateTruncating(aRghBox.Area());
            }

            // Find best split using simplified SAH
            for (int aNbLft = 1, aNbRgh = aNodeNbPrimitives - 1; aNbLft < aNodeNbPrimitives; ++aNbLft, --aNbRgh)
            {
                double aCost = (aLftSet[aNbLft] /* / aNodeArea */) * aNbLft
                               + (aRghSet[aNbRgh] /* / aNodeArea */) * aNbRgh;

                if (aCost < aMinSplitCost)
                {
                    aMinSplitCost = aCost;
                    aMinSplitAxis = anAxis;
                    aMinSplitIndex = aNbLft;
                }
            }
        }

        if (aMinSplitAxis == -1)
        {
            return new BVH_ChildNodes(); // failed to find split axis
        }

        theBVH.SetInner(theNode);

        if (aMinSplitAxis != (N < 4 ? N - 1 : 2))
        {
            new BVH_QuickSorter<T, TVec>(aMinSplitAxis).Perform(theSet, aNodeBegPrimitive, aNodeEndPrimitive);
        }

        BVH_Box<T, TVec> aMinSplitBoxLft = new();
        BVH_Box<T, TVec> aMinSplitBoxRgh = new();

        // Compute bounding boxes for selected split plane
        for (int anIndex = aNodeBegPrimitive; anIndex < aMinSplitIndex + aNodeBegPrimitive; ++anIndex)
        {
            aMinSplitBoxLft.Combine(theSet.Box(anIndex));
        }

        for (int anIndex = aNodeEndPrimitive; anIndex >= aMinSplitIndex + aNodeBegPrimitive; --anIndex)
        {
            aMinSplitBoxRgh.Combine(theSet.Box(anIndex));
        }

        int aMiddle = aNodeBegPrimitive + aMinSplitIndex;

        return new BVH_ChildNodes(aMinSplitBoxLft,
                                  aMinSplitBoxRgh,
                                  new BVH_PrimitiveRange(aNodeBegPrimitive, aMiddle - 1),
                                  new BVH_PrimitiveRange(aMiddle, aNodeEndPrimitive));
    }
}
