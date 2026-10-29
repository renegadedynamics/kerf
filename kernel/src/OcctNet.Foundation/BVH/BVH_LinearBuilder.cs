// Port of: src/FoundationClasses/TKMath/BVH/BVH_LinearBuilder.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Numerics;
using System.Runtime.CompilerServices;
using static Occt.BVH_Constants;

namespace Occt;

/// <summary>
/// Fast BVH construction with the LBVH approach: spatial Morton codes reduce the construction to a radix sort
/// (O(N)). The trees are of lower quality than SAH builders give, but are built over an order of magnitude faster.
/// See C. Lauterbach, M. Garland, S. Sengupta, D. Luebke, and D. Manocha, Fast BVH construction on GPUs,
/// Eurographics, 2009.
/// </summary>
public class BVH_LinearBuilder<T, TVec> : BVH_Builder<T, TVec>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    /// <summary>Creates a binned LBVH builder.</summary>
    public BVH_LinearBuilder(int theLeafNodeSize = BVH_Constants_LeafNodeSizeDefault,
                             int theMaxTreeDepth = BVH_Constants_MaxTreeDepth)
        : base(theLeafNodeSize, theMaxTreeDepth)
    {
    }

    /// <summary>The index of the first element (from <paramref name="theStart"/>) with the given bit set.</summary>
    protected int lowerBound(NCollection_Array1<BVH_EncodedLink> theEncodedLinks, int theStart, int theFinal, int theDigit)
    {
        int aNbPrims = theFinal - theStart;
        uint aBit = 1U << theDigit;
        while (aNbPrims > 0)
        {
            int aStep = aNbPrims / 2;
            if ((theEncodedLinks.Value(theStart + aStep).first & aBit) != 0)
            {
                aNbPrims = aStep;
            }
            else
            {
                theStart += aStep + 1;
                aNbPrims -= aStep + 1;
            }
        }

        return theStart;
    }

    /// <summary>Emits the hierarchy from the sorted Morton codes; returns the index of the node made.</summary>
    protected int emitHierachy(BVH_Tree<T, TVec> theBVH,
                               NCollection_Array1<BVH_EncodedLink> theEncodedLinks,
                               int theDigit,
                               int theShift,
                               int theStart,
                               int theFinal)
    {
        if (theFinal - theStart > myLeafNodeSize)
        {
            int aPosition = theDigit < 0 ? (theStart + theFinal) / 2
                                         : lowerBound(theEncodedLinks, theStart, theFinal, theDigit);
            if (aPosition == theStart || aPosition == theFinal)
            {
                return emitHierachy(theBVH, theEncodedLinks, theDigit - 1, theShift, theStart, theFinal);
            }

            // Build inner node
            int aNode = theBVH.AddInnerNode(0, 0);
            int aRghNode = theShift + aPosition - theStart;

            int aLftChild = emitHierachy(theBVH, theEncodedLinks, theDigit - 1, theShift, theStart, aPosition);
            int aRghChild = emitHierachy(theBVH, theEncodedLinks, theDigit - 1, aRghNode, aPosition, theFinal);

            theBVH.NodeInfoBuffer()[aNode][1] = aLftChild;
            theBVH.NodeInfoBuffer()[aNode][2] = aRghChild;

            return aNode;
        }
        else
        {
            // Build leaf node
            return theBVH.AddLeafNode(theShift, theShift + theFinal - theStart - 1);
        }
    }

    /// <summary>Builds the BVH.</summary>
    public override void Build(BVH_Set<T, TVec> theSet, BVH_Tree<T, TVec> theBVH, in BVH_Box<T, TVec> theBox)
    {
        int aSetSize = theSet.Size();
        if (theBVH is null || aSetSize == 0)
        {
            return;
        }

        theBVH.Clear();

        // Step 0 -- Initialize parameter of virtual grid
        BVH_RadixSorter<T, TVec> aRadixSorter = new(theBox);
        aRadixSorter.SetParallel(IsParallel());

        // Step 1 - Perform radix sorting of primitive set
        aRadixSorter.Perform(theSet);

        // Step 2 -- Emitting BVH hierarchy from sorted Morton codes
        emitHierachy(theBVH, aRadixSorter.EncodedLinks(), 29, 0, 0, theSet.Size());

        // Step 3 -- Compute bounding boxes of BVH nodes
        theBVH.MinPointBuffer().Resize(theBVH.NodeInfoBuffer().Size());
        theBVH.MaxPointBuffer().Resize(theBVH.NodeInfoBuffer().Size());

        StrongBox<int> aHeight = new(0);

        BVH.BoundData<T, TVec> aBoundData = new(theSet, theBVH, 0, 0, aHeight);
        BVH.UpdateBoundTask<T, TVec> aBoundTask = new(IsParallel());

        aBoundTask.Invoke(aBoundData);

        updateDepth(theBVH, aHeight.Value);
    }
}

public static partial class BVH
{
    /// <summary>Calculates the bounding boxes of the given tree's nodes from the node down; returns the height.</summary>
    public static int UpdateBounds<T, TVec>(BVH_Set<T, TVec> theSet, BVH_Tree<T, TVec> theTree, int theNode = 0)
        where T : unmanaged, INumber<T>, IMinMaxValue<T>
        where TVec : unmanaged, INCollection_Vec<T, TVec>
    {
        NCollection_Vec4<int> aData = theTree.NodeInfoBuffer()[theNode];
        if (aData.x() == 0)
        {
            int aLftChild = theTree.NodeInfoBuffer()[theNode].y();
            int aRghChild = theTree.NodeInfoBuffer()[theNode].z();

            int aLftDepth = UpdateBounds(theSet, theTree, aLftChild);
            int aRghDepth = UpdateBounds(theSet, theTree, aRghChild);

            TVec aLftMinPoint = theTree.MinPointBuffer()[aLftChild];
            TVec aLftMaxPoint = theTree.MaxPointBuffer()[aLftChild];
            TVec aRghMinPoint = theTree.MinPointBuffer()[aRghChild];
            TVec aRghMaxPoint = theTree.MaxPointBuffer()[aRghChild];

            CwiseMin<T, TVec>(ref aLftMinPoint, aRghMinPoint);
            CwiseMax<T, TVec>(ref aLftMaxPoint, aRghMaxPoint);

            theTree.MinPointBuffer()[theNode] = aLftMinPoint;
            theTree.MaxPointBuffer()[theNode] = aLftMaxPoint;

            return Math.Max(aLftDepth, aRghDepth) + 1;
        }
        else
        {
            ref TVec aMinPoint = ref theTree.MinPointBuffer()[theNode];
            ref TVec aMaxPoint = ref theTree.MaxPointBuffer()[theNode];

            for (int aPrimIdx = aData.y(); aPrimIdx <= aData.z(); ++aPrimIdx)
            {
                BVH_Box<T, TVec> aBox = theSet.Box(aPrimIdx);

                if (aPrimIdx == aData.y())
                {
                    aMinPoint = aBox.CornerMin();
                    aMaxPoint = aBox.CornerMax();
                }
                else
                {
                    CwiseMin<T, TVec>(ref aMinPoint, aBox.CornerMin());
                    CwiseMax<T, TVec>(ref aMaxPoint, aBox.CornerMax());
                }
            }
        }

        return 0;
    }

    /// <summary>The data of a bounds update task (OCCT's <c>int* myHeight</c> is a shared box).</summary>
    public readonly struct BoundData<T, TVec>(BVH_Set<T, TVec> theSet, BVH_Tree<T, TVec> theBVH, int theNode, int theLevel, StrongBox<int> theHeight)
        where T : unmanaged, INumber<T>, IMinMaxValue<T>
        where TVec : unmanaged, INCollection_Vec<T, TVec>
    {
        public readonly BVH_Set<T, TVec> mySet = theSet;        //!< Set of geometric objects
        public readonly BVH_Tree<T, TVec> myBVH = theBVH;       //!< BVH tree built over the set
        public readonly int myNode = theNode;                   //!< BVH node to update bounding box
        public readonly int myLevel = theLevel;                 //!< Level of the processed BVH node
        public readonly StrongBox<int> myHeight = theHeight;    //!< Height of the processed BVH node
    }

    /// <summary>A task for updating bounds, in parallel for the top levels when allowed.</summary>
    public sealed class UpdateBoundTask<T, TVec>(bool isParallel)
        where T : unmanaged, INumber<T>, IMinMaxValue<T>
        where TVec : unmanaged, INCollection_Vec<T, TVec>
    {
        private readonly bool myIsParallel = isParallel;

        /// <summary>Executes the task (OCCT's <c>operator()</c>).</summary>
        public void Invoke(BoundData<T, TVec> theData)
        {
            if (theData.myBVH.IsOuter(theData.myNode) || theData.myLevel > 2)
            {
                theData.myHeight.Value = UpdateBounds(theData.mySet, theData.myBVH, theData.myNode);
            }
            else
            {
                StrongBox<int> aLftHeight = new(0);
                StrongBox<int> aRghHeight = new(0);

                int aLftChild = theData.myBVH.NodeInfoBuffer()[theData.myNode].y();
                int aRghChild = theData.myBVH.NodeInfoBuffer()[theData.myNode].z();

                NCollection_DynamicArray<BoundData<T, TVec>> aList = new(2);

                if (!theData.myBVH.IsOuter(aLftChild))
                {
                    aList.Append(new BoundData<T, TVec>(theData.mySet, theData.myBVH, aLftChild, theData.myLevel + 1, aLftHeight));
                }
                else
                {
                    aLftHeight.Value = UpdateBounds(theData.mySet, theData.myBVH, aLftChild);
                }

                if (!theData.myBVH.IsOuter(aRghChild))
                {
                    aList.Append(new BoundData<T, TVec>(theData.mySet, theData.myBVH, aRghChild, theData.myLevel + 1, aRghHeight));
                }
                else
                {
                    aRghHeight.Value = UpdateBounds(theData.mySet, theData.myBVH, aRghChild);
                }

                if (aList.Size() > 0)
                {
                    // OSD_Parallel::ForEach(..., !myIsParallel)
                    UpdateBoundTask<T, TVec> aTask = new(myIsParallel);
                    OSD_Parallel.ForEach(aList, aTask.Invoke, !myIsParallel);
                }

                TVec aLftMinPoint = theData.myBVH.MinPointBuffer()[aLftChild];
                TVec aLftMaxPoint = theData.myBVH.MaxPointBuffer()[aLftChild];
                TVec aRghMinPoint = theData.myBVH.MinPointBuffer()[aRghChild];
                TVec aRghMaxPoint = theData.myBVH.MaxPointBuffer()[aRghChild];

                CwiseMin<T, TVec>(ref aLftMinPoint, aRghMinPoint);
                CwiseMax<T, TVec>(ref aLftMaxPoint, aRghMaxPoint);

                theData.myBVH.MinPointBuffer()[theData.myNode] = aLftMinPoint;
                theData.myBVH.MaxPointBuffer()[theData.myNode] = aLftMaxPoint;

                theData.myHeight.Value = Math.Max(aLftHeight.Value, aRghHeight.Value) + 1;
            }
        }
    }
}
