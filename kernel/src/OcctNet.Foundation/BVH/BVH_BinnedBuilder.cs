// Port of: src/FoundationClasses/TKMath/BVH/BVH_BinnedBuilder.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Numerics;
using static Occt.BVH_Constants;

namespace Occt;

/// <summary>The parameters of a single bin (slice of an AABB).</summary>
/// <remarks><c>default(BVH_Bin)</c> has a valid box at the origin; write <c>new()</c>, which makes an empty bin.</remarks>
public struct BVH_Bin<T, TVec>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    public int Count;             //!< Number of primitives in the bin
    public BVH_Box<T, TVec> Box;  //!< AABB of primitives in the bin

    /// <summary>An empty bin.</summary>
    public BVH_Bin()
    {
        Count = 0;
        Box = new BVH_Box<T, TVec>();
    }
}

/// <summary>
/// Builds a BVH tree with the binned SAH algorithm. The number of bins trades build time for tree quality (32 to 48
/// is best; 4 to 8 is only 10-20% worse). Several threads can only be used with thread-safe primitive sets.
/// </summary>
/// <remarks>OCCT's template argument <c>Bins</c> is the constructor's last argument.</remarks>
public class BVH_BinnedBuilder<T, TVec> : BVH_QueueBuilder<T, TVec>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    /// <summary>The number of bins (OCCT's template argument <c>Bins</c>).</summary>
    protected readonly int Bins;

    private readonly bool myUseMainAxis; //!< Defines whether to search for the best split or use the widest axis

    /// <summary>A split plane candidate.</summary>
    protected struct BVH_SplitPlane
    {
        public BVH_Bin<T, TVec> LftVoxel;
        public BVH_Bin<T, TVec> RghVoxel;

        public BVH_SplitPlane()
        {
            LftVoxel = new BVH_Bin<T, TVec>();
            RghVoxel = new BVH_Bin<T, TVec>();
        }
    }

    /// <summary>Creates a binned SAH BVH builder.</summary>
    public BVH_BinnedBuilder(int theLeafNodeSize = BVH_Constants_LeafNodeSizeDefault,
                             int theMaxTreeDepth = BVH_Constants_MaxTreeDepth,
                             bool theDoMainSplits = false,
                             int theNumOfThreads = 1,
                             int theBins = BVH_Constants_NbBinsOptimal)
        : base(theLeafNodeSize, theMaxTreeDepth, theNumOfThreads)
    {
        Bins = theBins;
        myUseMainAxis = theDoMainSplits;
    }

    /// <summary>Arranges the node's primitives into bins along the axis.</summary>
    protected virtual void getSubVolumes(BVH_Set<T, TVec> theSet,
                                         BVH_Tree<T, TVec> theBVH,
                                         int theNode,
                                         Span<BVH_Bin<T, TVec>> theBins,
                                         int theAxis)
    {
        T aMin = BVH.VecComp<T, TVec>(theBVH.MinPoint(theNode), theAxis);
        T aMax = BVH.VecComp<T, TVec>(theBVH.MaxPoint(theNode), theAxis);
        T anInverseStep = T.CreateTruncating(Bins) / (aMax - aMin);
        for (int anIdx = theBVH.BegPrimitive(theNode); anIdx <= theBVH.EndPrimitive(theNode); ++anIdx)
        {
            BVH_Box<T, TVec> aBox = theSet.Box(anIdx);
            int aBinIndex = BVH.IntFloor((theSet.Center(anIdx, theAxis) - aMin) * anInverseStep);
            if (aBinIndex < 0)
            {
                aBinIndex = 0;
            }
            else if (aBinIndex >= Bins)
            {
                aBinIndex = Bins - 1;
            }

            theBins[aBinIndex].Count++;
            theBins[aBinIndex].Box.Combine(aBox);
        }
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

        BVH_Box<T, TVec> anAABB = new(theBVH.MinPoint(theNode), theBVH.MaxPoint(theNode));
        TVec aSize = anAABB.Size();

        // Parameters for storing best split
        int aMinSplitAxis = -1;
        int aMinSplitIndex = 0;
        int aMinSplitNumLft = 0;
        int aMinSplitNumRgh = 0;

        BVH_Box<T, TVec> aMinSplitBoxLft = new();
        BVH_Box<T, TVec> aMinSplitBoxRgh = new();

        double aMinSplitCost = double.MaxValue;
        int aMainAxis = BVH.MainAxis<T, TVec>(aSize);

        Span<BVH_Bin<T, TVec>> aBinVector = stackalloc BVH_Bin<T, TVec>[Bins];
        Span<BVH_SplitPlane> aSplitPlanes = stackalloc BVH_SplitPlane[Bins + 1];

        // Find best split
        for (int anAxis = myUseMainAxis ? aMainAxis : 0;
             anAxis <= (myUseMainAxis ? aMainAxis : Min(TVec.Length() - 1, 2));
             ++anAxis)
        {
            if (double.CreateTruncating(BVH.VecComp<T, TVec>(aSize, anAxis)) <= BVH.THE_NODE_MIN_SIZE)
            {
                continue;
            }

            // C++ default-constructs the arrays on each iteration
            aBinVector.Fill(new BVH_Bin<T, TVec>());
            aSplitPlanes.Fill(new BVH_SplitPlane());

            getSubVolumes(theSet, theBVH, theNode, aBinVector, anAxis);

            for (int aLftSplit = 1, aRghSplit = Bins - 1; aLftSplit < Bins; ++aLftSplit, --aRghSplit)
            {
                aSplitPlanes[aLftSplit].LftVoxel.Count =
                    aSplitPlanes[aLftSplit - 1].LftVoxel.Count + aBinVector[aLftSplit - 1].Count;

                aSplitPlanes[aRghSplit].RghVoxel.Count =
                    aSplitPlanes[aRghSplit + 1].RghVoxel.Count + aBinVector[aRghSplit + 0].Count;

                aSplitPlanes[aLftSplit].LftVoxel.Box = aSplitPlanes[aLftSplit - 1].LftVoxel.Box;
                aSplitPlanes[aRghSplit].RghVoxel.Box = aSplitPlanes[aRghSplit + 1].RghVoxel.Box;

                aSplitPlanes[aLftSplit].LftVoxel.Box.Combine(aBinVector[aLftSplit - 1].Box);
                aSplitPlanes[aRghSplit].RghVoxel.Box.Combine(aBinVector[aRghSplit + 0].Box);
            }

            // Choose the best split (with minimum SAH cost)
            double aParentArea = double.CreateTruncating(anAABB.Area());
            for (int aSplit = 1; aSplit < Bins; ++aSplit)
            {
                // SAH evaluation with proper normalization by parent surface area
                double aLftArea = double.CreateTruncating(aSplitPlanes[aSplit].LftVoxel.Box.Area());
                double aRghArea = double.CreateTruncating(aSplitPlanes[aSplit].RghVoxel.Box.Area());
                double aCost = (aLftArea / aParentArea) * aSplitPlanes[aSplit].LftVoxel.Count
                               + (aRghArea / aParentArea) * aSplitPlanes[aSplit].RghVoxel.Count;

                if (aCost <= aMinSplitCost)
                {
                    aMinSplitCost = aCost;
                    aMinSplitAxis = anAxis;
                    aMinSplitIndex = aSplit;
                    aMinSplitBoxLft = aSplitPlanes[aSplit].LftVoxel.Box;
                    aMinSplitBoxRgh = aSplitPlanes[aSplit].RghVoxel.Box;
                    aMinSplitNumLft = aSplitPlanes[aSplit].LftVoxel.Count;
                    aMinSplitNumRgh = aSplitPlanes[aSplit].RghVoxel.Count;
                }
            }
        }

        theBVH.SetInner(theNode);
        int aMiddle;
        if (aMinSplitNumLft == 0 || aMinSplitNumRgh == 0
            || aMinSplitAxis == -1) // case of objects with the same center
        {
            aMinSplitBoxLft.Clear();
            aMinSplitBoxRgh.Clear();

            aMiddle = Max(aNodeBegPrimitive + 1, (int)((aNodeBegPrimitive + aNodeEndPrimitive) / 2.0f));

            aMinSplitNumLft = aMiddle - aNodeBegPrimitive;
            for (int anIndex = aNodeBegPrimitive; anIndex < aMiddle; ++anIndex)
            {
                aMinSplitBoxLft.Combine(theSet.Box(anIndex));
            }

            aMinSplitNumRgh = aNodeEndPrimitive - aMiddle + 1;
            for (int anIndex = aNodeEndPrimitive; anIndex >= aMiddle; --anIndex)
            {
                aMinSplitBoxRgh.Combine(theSet.Box(anIndex));
            }
        }
        else
        {
            aMiddle = BVH.SplitPrimitives(theSet, anAABB, aNodeBegPrimitive, aNodeEndPrimitive,
                                          aMinSplitIndex - 1, aMinSplitAxis, Bins);
        }

        return new BVH_ChildNodes(aMinSplitBoxLft,
                                  aMinSplitBoxRgh,
                                  new BVH_PrimitiveRange(aNodeBegPrimitive, aMiddle - 1),
                                  new BVH_PrimitiveRange(aMiddle, aNodeEndPrimitive));
    }
}

public static partial class BVH
{
    /// <summary>Partitions the node's primitives about the given bin along the axis; returns the first of the right part.</summary>
    public static int SplitPrimitives<T, TVec>(BVH_Set<T, TVec> theSet,
                                               in BVH_Box<T, TVec> theBox,
                                               int theBeg,
                                               int theEnd,
                                               int theBin,
                                               int theAxis,
                                               int theBins)
        where T : unmanaged, INumber<T>, IMinMaxValue<T>
        where TVec : unmanaged, INCollection_Vec<T, TVec>
    {
        T aMin = VecComp<T, TVec>(theBox.CornerMin(), theAxis);
        T aMax = VecComp<T, TVec>(theBox.CornerMax(), theAxis);

        T anInverseStep = T.CreateTruncating(theBins) / (aMax - aMin);

        int aLftIdx = theBeg;
        int aRghIdx = theEnd;

        do
        {
            while (IntFloor((theSet.Center(aLftIdx, theAxis) - aMin) * anInverseStep) <= theBin
                   && aLftIdx < theEnd)
            {
                ++aLftIdx;
            }

            while (IntFloor((theSet.Center(aRghIdx, theAxis) - aMin) * anInverseStep) > theBin
                   && aRghIdx > theBeg)
            {
                --aRghIdx;
            }

            if (aLftIdx <= aRghIdx)
            {
                if (aLftIdx != aRghIdx)
                {
                    theSet.Swap(aLftIdx, aRghIdx);
                }

                ++aLftIdx;
                --aRghIdx;
            }
        } while (aLftIdx <= aRghIdx);

        return aLftIdx;
    }

    /// <summary>The axis of the largest size (<c>BVH::BVH_AxisSelector</c>).</summary>
    public static int MainAxis<T, TVec>(in TVec theSize)
        where T : unmanaged, INumber<T>
        where TVec : unmanaged, INCollection_Vec<T, TVec>
    {
        if (TVec.Length() == 2)
        {
            return theSize[0] > theSize[1] ? 0 : 1;
        }

        if (theSize[1] > theSize[0])
        {
            return theSize[1] > theSize[2] ? 1 : 2;
        }
        else
        {
            return theSize[2] > theSize[0] ? 2 : 0;
        }
    }
}
