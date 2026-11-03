// Port of: src/FoundationClasses/TKMath/BVH/BVH_QueueBuilder.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace Kerf.Engine.Geometry.Bvh;

/// <summary>
/// An abstract BVH builder based on a work queue. It can build with a fixed number of threads (best with the number
/// of CPU cores plus one); in parallel mode, the primitive set must be thread-safe (Swap, Box, Center).
/// </summary>
public abstract class QueueBuilder<T, TVec> : Builder<T, TVec>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    protected int _numOfThreads; //!< Number of threads used to build BVH

    /// <summary>Creates a new queue-based builder.</summary>
    protected QueueBuilder(int leafNodeSize, int maxTreeDepth, int numOfThreads = 1)
        : base(leafNodeSize, maxTreeDepth)
    {
        _numOfThreads = numOfThreads;
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

        public BVH_PrimitiveRange(int start, int final = -1)
        {
            Start = start;
            Final = final;
        }

        /// <summary>The total number of primitives.</summary>
        public readonly int Size() => Final - Start + 1;

        /// <summary>True if the range is initialized.</summary>
        public readonly bool IsValid => Start != -1;
    }

    [InlineArray(2)]
    protected struct BVH_BoxPair
    {
        private Box<T, TVec> _element0;
    }

    [InlineArray(2)]
    protected struct BVH_RangePair
    {
        private BVH_PrimitiveRange _element0;
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
            Boxes[0] = new Box<T, TVec>();
            Boxes[1] = new Box<T, TVec>();
            Ranges[0] = new BVH_PrimitiveRange();
            Ranges[1] = new BVH_PrimitiveRange();
        }

        public BVH_ChildNodes(in Box<T, TVec> lftBox,
                              in Box<T, TVec> rghBox,
                              in BVH_PrimitiveRange lftRange,
                              in BVH_PrimitiveRange rghRange)
        {
            Boxes[0] = lftBox;
            Boxes[1] = rghBox;
            Ranges[0] = lftRange;
            Ranges[1] = rghRange;
        }

        /// <summary>The number of primitives in the given child.</summary>
        public readonly int NbPrims(int child) => Ranges[child].Size();

        /// <summary>True if the parameters are initialized.</summary>
        public readonly bool IsValid => Ranges[0].IsValid && Ranges[1].IsValid;
    }

    /// <summary>Wrapper for BVH build data.</summary>
    protected sealed class BVH_TypedBuildTool : BuildTool
    {
        private readonly global::Kerf.Engine.Geometry.Bvh.ISet<T, TVec> _set;                  //!< Primitive set to build BVH
        private readonly Tree<T, TVec> _bvh;                 //!< Output BVH tree for the set
        private readonly BuildQueue _buildQueue;
        private readonly QueueBuilder<T, TVec> _algo;       //!< Queue based BVH builder to use

        public BVH_TypedBuildTool(global::Kerf.Engine.Geometry.Bvh.ISet<T, TVec> set,
                                  Tree<T, TVec> bvh,
                                  BuildQueue buildQueue,
                                  QueueBuilder<T, TVec> algo)
        {
            _set = set;
            _bvh = bvh;
            _buildQueue = buildQueue;
            _algo = algo ?? throw new ProgramErrorException("Error! BVH builder should be queue based");
        }

        /// <summary>Splits the given BVH node.</summary>
        public override void Perform(int node)
        {
            BVH_ChildNodes children = _algo.BuildNode(_set, _bvh, node);
            _algo.AddChildren(_bvh, _buildQueue, node, children);
        }
    }

    /// <summary>Splits the given BVH node.</summary>
    protected abstract BVH_ChildNodes BuildNode(global::Kerf.Engine.Geometry.Bvh.ISet<T, TVec> set, Tree<T, TVec> bvh, int node);

    /// <summary>Processes the child nodes of the split BVH node.</summary>
    protected virtual void AddChildren(Tree<T, TVec> bvh, BuildQueue buildQueue, int node, in BVH_ChildNodes subNodes)
    {
        Span<int> children = [-1, -1];
        if (!subNodes.IsValid)
        {
            return;
        }

        // Add child nodes
        lock (buildQueue._mutex)
        {
            for (int idx = 0; idx < 2; ++idx)
            {
                children[idx] = bvh.AddLeafNode(subNodes.Boxes[idx],
                                                      subNodes.Ranges[idx].Start,
                                                      subNodes.Ranges[idx].Final);
            }

            UpdateDepth(bvh, bvh.Level(node) + 1);
        }

        // Set parameters of child nodes and generate new tasks
        for (int idx = 0; idx < 2; ++idx)
        {
            int childIndex = children[idx];

            bvh.Level(childIndex) = bvh.Level(node) + 1;

            bvh.Child(idx == 0 ? 0 : 1, node) = childIndex;

            // Check to see if the child node must be split
            bool isLeaf = subNodes.NbPrims(idx) <= _leafNodeSize
                          || bvh.Level(childIndex) >= _maxTreeDepth;

            if (!isLeaf)
            {
                buildQueue.Enqueue(childIndex);
            }
        }
    }

    /// <summary>Builds the BVH, with <c>myNumOfThreads</c> threads.</summary>
    public override void Build(global::Kerf.Engine.Geometry.Bvh.ISet<T, TVec> set, Tree<T, TVec> bvh, in Box<T, TVec> box)
    {
        if (bvh is null) // Standard_ASSERT_RETURN: "Error! BVH tree to construct is NULL"
        {
            return;
        }

        bvh.Clear();
        int setSize = set.Size();
        if (setSize == 0)
        {
            return;
        }

        int root = bvh.AddLeafNode(box, 0, setSize - 1);
        if (set.Size() == 1)
        {
            return;
        }

        BuildQueue buildQueue = new();
        buildQueue.Enqueue(root);

        BVH_TypedBuildTool buildTool = new(set, bvh, buildQueue, this);

        // OCCT.NET: a runtime without threads (single-threaded WebAssembly) builds in the calling thread (decision 487)
        if (_numOfThreads > 1 && !OsdParallel.IsSerial)
        {
            // Reserve the maximum possible number of nodes in the BVH
            bvh.Reserve(2 * setSize - 1);

            List<BuildThread> threads = new();

            // Run BVH build threads
            for (int threadIndex = 0; threadIndex < _numOfThreads; ++threadIndex)
            {
                threads.Add(new BuildThread(buildTool, buildQueue));
                threads[^1].Run();
            }

            // Wait until all threads finish their work
            for (int threadIndex = 0; threadIndex < _numOfThreads; ++threadIndex)
            {
                threads[threadIndex].Wait();
            }

            // Free unused memory (Reserve never shrinks, so this does nothing, in OCCT too)
            bvh.Reserve(bvh.Length());
        }
        else
        {
            BuildThread thread = new(buildTool, buildQueue);

            // Execute thread function inside current thread
            thread.Execute();
        }
    }
}
