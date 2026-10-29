// Port of: src/FoundationClasses/TKMath/BVH/BVH_QueueBuilder.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Numerics;
using System.Runtime.CompilerServices;

namespace Occt;

/// <summary>
/// An abstract BVH builder based on a work queue. It can build with a fixed number of threads (best with the number
/// of CPU cores plus one); in parallel mode, the primitive set must be thread-safe (Swap, Box, Center).
/// </summary>
public abstract class BVH_QueueBuilder<T, TVec> : BVH_Builder<T, TVec>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    protected int myNumOfThreads; //!< Number of threads used to build BVH

    /// <summary>Creates a new queue-based builder.</summary>
    protected BVH_QueueBuilder(int theLeafNodeSize, int theMaxTreeDepth, int theNumOfThreads = 1)
        : base(theLeafNodeSize, theMaxTreeDepth)
    {
        myNumOfThreads = theNumOfThreads;
    }

    /// <summary>The range of primitives belonging to a BVH node.</summary>
    protected struct BVH_PrimitiveRange
    {
        public int Start;
        public int Final;

        /// <summary>An uninitialized range (-1, -1), as OCCT's default arguments.</summary>
        public BVH_PrimitiveRange()
        {
            Start = -1;
            Final = -1;
        }

        public BVH_PrimitiveRange(int theStart, int theFinal = -1)
        {
            Start = theStart;
            Final = theFinal;
        }

        /// <summary>The total number of primitives.</summary>
        public readonly int Size() => Final - Start + 1;

        /// <summary>True if the range is initialized.</summary>
        public readonly bool IsValid() => Start != -1;
    }

    [InlineArray(2)]
    protected struct BVH_BoxPair
    {
        private BVH_Box<T, TVec> myElement0;
    }

    [InlineArray(2)]
    protected struct BVH_RangePair
    {
        private BVH_PrimitiveRange myElement0;
    }

    /// <summary>The parameters of the constructed child nodes.</summary>
    protected struct BVH_ChildNodes
    {
        /// <summary>Bounding boxes of child nodes.</summary>
        public BVH_BoxPair Boxes;

        /// <summary>Primitive ranges of child nodes.</summary>
        public BVH_RangePair Ranges;

        /// <summary>Uninitialized child nodes (invalid boxes and ranges).</summary>
        public BVH_ChildNodes()
        {
            Boxes[0] = new BVH_Box<T, TVec>();
            Boxes[1] = new BVH_Box<T, TVec>();
            Ranges[0] = new BVH_PrimitiveRange();
            Ranges[1] = new BVH_PrimitiveRange();
        }

        public BVH_ChildNodes(in BVH_Box<T, TVec> theLftBox,
                              in BVH_Box<T, TVec> theRghBox,
                              in BVH_PrimitiveRange theLftRange,
                              in BVH_PrimitiveRange theRghRange)
        {
            Boxes[0] = theLftBox;
            Boxes[1] = theRghBox;
            Ranges[0] = theLftRange;
            Ranges[1] = theRghRange;
        }

        /// <summary>The number of primitives in the given child.</summary>
        public readonly int NbPrims(int theChild) => Ranges[theChild].Size();

        /// <summary>True if the parameters are initialized.</summary>
        public readonly bool IsValid() => Ranges[0].IsValid() && Ranges[1].IsValid();
    }

    /// <summary>Wrapper for BVH build data.</summary>
    protected sealed class BVH_TypedBuildTool : BVH_BuildTool
    {
        private readonly BVH_Set<T, TVec> mySet;                  //!< Primitive set to build BVH
        private readonly BVH_Tree<T, TVec> myBVH;                 //!< Output BVH tree for the set
        private readonly BVH_BuildQueue myBuildQueue;
        private readonly BVH_QueueBuilder<T, TVec> myAlgo;       //!< Queue based BVH builder to use

        public BVH_TypedBuildTool(BVH_Set<T, TVec> theSet,
                                  BVH_Tree<T, TVec> theBVH,
                                  BVH_BuildQueue theBuildQueue,
                                  BVH_QueueBuilder<T, TVec> theAlgo)
        {
            mySet = theSet;
            myBVH = theBVH;
            myBuildQueue = theBuildQueue;
            myAlgo = theAlgo ?? throw new Standard_ProgramError("Error! BVH builder should be queue based");
        }

        /// <summary>Splits the given BVH node.</summary>
        public override void Perform(int theNode)
        {
            BVH_ChildNodes aChildren = myAlgo.buildNode(mySet, myBVH, theNode);
            myAlgo.addChildren(myBVH, myBuildQueue, theNode, aChildren);
        }
    }

    /// <summary>Splits the given BVH node.</summary>
    protected abstract BVH_ChildNodes buildNode(BVH_Set<T, TVec> theSet, BVH_Tree<T, TVec> theBVH, int theNode);

    /// <summary>Processes the child nodes of the split BVH node.</summary>
    protected virtual void addChildren(BVH_Tree<T, TVec> theBVH, BVH_BuildQueue theBuildQueue, int theNode, in BVH_ChildNodes theSubNodes)
    {
        Span<int> aChildren = [-1, -1];
        if (!theSubNodes.IsValid())
        {
            return;
        }

        // Add child nodes
        lock (theBuildQueue.myMutex)
        {
            for (int anIdx = 0; anIdx < 2; ++anIdx)
            {
                aChildren[anIdx] = theBVH.AddLeafNode(theSubNodes.Boxes[anIdx],
                                                      theSubNodes.Ranges[anIdx].Start,
                                                      theSubNodes.Ranges[anIdx].Final);
            }

            updateDepth(theBVH, theBVH.Level(theNode) + 1);
        }

        // Set parameters of child nodes and generate new tasks
        for (int anIdx = 0; anIdx < 2; ++anIdx)
        {
            int aChildIndex = aChildren[anIdx];

            theBVH.Level(aChildIndex) = theBVH.Level(theNode) + 1;

            theBVH.Child(anIdx == 0 ? 0 : 1, theNode) = aChildIndex;

            // Check to see if the child node must be split
            bool isLeaf = theSubNodes.NbPrims(anIdx) <= myLeafNodeSize
                          || theBVH.Level(aChildIndex) >= myMaxTreeDepth;

            if (!isLeaf)
            {
                theBuildQueue.Enqueue(aChildIndex);
            }
        }
    }

    /// <summary>Builds the BVH, with <c>myNumOfThreads</c> threads.</summary>
    public override void Build(BVH_Set<T, TVec> theSet, BVH_Tree<T, TVec> theBVH, in BVH_Box<T, TVec> theBox)
    {
        if (theBVH is null) // Standard_ASSERT_RETURN: "Error! BVH tree to construct is NULL"
        {
            return;
        }

        theBVH.Clear();
        int aSetSize = theSet.Size();
        if (aSetSize == 0)
        {
            return;
        }

        int aRoot = theBVH.AddLeafNode(theBox, 0, aSetSize - 1);
        if (theSet.Size() == 1)
        {
            return;
        }

        BVH_BuildQueue aBuildQueue = new();
        aBuildQueue.Enqueue(aRoot);

        BVH_TypedBuildTool aBuildTool = new(theSet, theBVH, aBuildQueue, this);

        // OCCT.NET: a runtime without threads (single-threaded WebAssembly) builds in the calling thread (decision 487)
        if (myNumOfThreads > 1 && !OSD_Parallel.IsSerial())
        {
            // Reserve the maximum possible number of nodes in the BVH
            theBVH.Reserve(2 * aSetSize - 1);

            NCollection_DynamicArray<BVH_BuildThread> aThreads = new();

            // Run BVH build threads
            for (int aThreadIndex = 0; aThreadIndex < myNumOfThreads; ++aThreadIndex)
            {
                aThreads.Append(new BVH_BuildThread(aBuildTool, aBuildQueue));
                aThreads.Last().Run();
            }

            // Wait until all threads finish their work
            for (int aThreadIndex = 0; aThreadIndex < myNumOfThreads; ++aThreadIndex)
            {
                aThreads.ChangeValue(aThreadIndex).Wait();
            }

            // Free unused memory (Reserve never shrinks, so this does nothing, in OCCT too)
            theBVH.Reserve(theBVH.Length());
        }
        else
        {
            BVH_BuildThread aThread = new(aBuildTool, aBuildQueue);

            // Execute thread function inside current thread
            aThread.execute();
        }
    }
}
