// Port of: src/FoundationClasses/TKMath/BVH/BVH_Traverse.hxx
// Port of: src/FoundationClasses/TKMath/BVH/BVH_Traverse.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2019-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;
using System.Numerics;
using System.Runtime.CompilerServices;
using static Kerf.Engine.Geometry.Bvh.Constants;

namespace Kerf.Engine.Geometry.Bvh;

/// <summary>
/// The base of the BVH traversals: selection of the elements of a BVH tree by rules defined in the derived class.
/// </summary>
/// <remarks>
/// <para>The rules:</para>
/// <list type="bullet">
/// <item><c>RejectNode</c>: rejects a node by its bounding box, and computes the node's metric, so the descent can
/// take the best branch first. It is applied to inner and outer nodes.</item>
/// <item><c>Accept</c>: accepts an element, by its index in the tree; the element itself is reached through the
/// set the tree is built on. Usually the logic of the whole operation.</item>
/// <item><see cref="IsMetricBetter"/>: true if the left metric is better than the right one.</item>
/// <item><see cref="RejectMetric"/>: rejects a node by its metric, compared with the global one.</item>
/// <item><see cref="Stop"/>: stops the descent when the necessary elements are found.</item>
/// <item><c>AcceptMetric</c> (single tree only): accepts a whole branch by its metric, without further checks.</item>
/// </list>
/// <para>
/// A traversal either sets the BVH set (<c>SetBVHSet</c>) and calls <c>Select()</c>, which takes the tree from the set
/// and lets <c>Accept</c> reach the elements; or leaves the set out and calls <c>Select(tree)</c> on any tree. In the
/// port, the set type is a type argument as in OCCT; the generic types without it stand for OCCT's <c>void</c>.
/// </para>
/// </remarks>
/// <typeparam name="TMetric">The type of the metric used to descend the tree in the best order.</typeparam>
public abstract class BaseTraverse<TMetric>
{
    protected BaseTraverse()
    {
    }

    /// <summary>True if the first metric is better than the second (the default keeps the left-to-right descent).</summary>
    public virtual bool IsMetricBetter(in TMetric left, in TMetric right) => true;

    /// <summary>True if the node with this metric should be rejected (the default rejects nothing).</summary>
    public virtual bool RejectMetric(in TMetric metric) => false;

    /// <summary>True if the descent should stop (the default never stops).</summary>
    public virtual bool Stop() => false;

    /// <summary>The tree of a set: OCCT calls <c>BVH()</c> on whatever set type is given.</summary>
    private protected static Tree<T, TVec>? TreeOf<T, TVec>(object? set)
        where T : unmanaged, INumber<T>, IMinMaxValue<T>
        where TVec : unmanaged, INCollection_Vec<T, TVec>
        => set switch
        {
            null => null,
            PrimitiveSet<T, TVec> primitiveSet => primitiveSet.BVH(),
            BvhGeometry<T, TVec> geometry => geometry.BVH(),
            _ => throw new TypeMismatchException("BVH_Traverse: the set has no BVH() (not a BVH_PrimitiveSet or BVH_Geometry)"),
        };
}

/// <summary>
/// Traversal of a single binary tree; see <see cref="BaseTraverse{TMetric}"/> for the rules to define.
/// </summary>
/// <typeparam name="T">The element type.</typeparam>
/// <typeparam name="TVec">The vector type (dimension).</typeparam>
/// <typeparam name="TSet">The type of the set holding the tree, to reach the elements by index.</typeparam>
/// <typeparam name="TMetric">The type of the metric.</typeparam>
public abstract class Traverse<T, TVec, TSet, TMetric> : BaseTraverse<TMetric>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
    where TSet : class
    where TMetric : new()
{
    protected TSet? _bvhSet;

    protected Traverse()
    {
        _bvhSet = null;
    }

    /// <summary>Sets the BVH set containing the tree.</summary>
    public void SetBVHSet(TSet? bvhSet) => _bvhSet = bvhSet;

    /// <summary>True if the whole branch with this metric may be accepted without further checks (the default: no).</summary>
    public virtual bool AcceptMetric(in TMetric metric) => false;

    /// <summary>Rejects a node by its bounding box, computing its metric; true if the node should be rejected.</summary>
    public abstract bool RejectNode(in TVec cornerMin, in TVec cornerMax, out TMetric metric);

    /// <summary>
    /// Accepts a leaf element; the metric of the parent leaf node is passed to allow accepting it unconditionally.
    /// True if the element has been accepted.
    /// </summary>
    public abstract bool Accept(int index, in TMetric metric);

    /// <summary>Selects the elements of the tree of the set (which must be set); returns the number accepted.</summary>
    public int Select()
    {
        if (_bvhSet is not null)
        {
            Tree<T, TVec>? bvh = TreeOf<T, TVec>(_bvhSet);
            return Select(bvh);
        }

        return 0;
    }

    /// <summary>A node to process, with its metric.</summary>
    protected struct BVH_NodeInStack
    {
        public int NodeID;      //!< Id of the node in the BVH tree
        public TMetric Metric;  //!< Metric computed for the node

        public BVH_NodeInStack(int nodeID, TMetric metric)
        {
            NodeID = nodeID;
            Metric = metric;
        }
    }

    [InlineArray(BVH_Constants_MaxTreeDepth)]
    private struct NodeStack
    {
        private BVH_NodeInStack _element0;
    }

    /// <summary>Selects the elements of the given tree; returns the number accepted.</summary>
    public int Select(Tree<T, TVec>? bvh)
    {
        if (bvh is null)
        {
            return 0;
        }

        NCollection_LinearVector<NCollection_Vec4<int>> bvhNodes = bvh.NodeInfoBuffer();
        if (bvhNodes.IsEmpty())
        {
            return 0;
        }

        // Create stack
        NodeStack stack = default;

        BVH_NodeInStack node = new(0, new TMetric()); // Currently processed node, starting with the root node
        BVH_NodeInStack prevNode = node;             // Previously processed node

        int head = -1;      // End of the stack
        int nbAccepted = 0; // Counter for accepted elements

        for (;;)
        {
            NCollection_Vec4<int> data = bvhNodes[node.NodeID];

            if (data.X() == 0)
            {
                // Inner node:
                // - check the metric of the node
                // - test the children of the node

                if (!AcceptMetric(node.Metric))
                {
                    // Test the left branch
                    bool isGoodLft = !RejectNode(bvh.MinPoint(data.Y()), bvh.MaxPoint(data.Y()), out TMetric metricLft);
                    if (Stop())
                    {
                        return nbAccepted;
                    }

                    // Test the right branch
                    bool isGoodRgh = !RejectNode(bvh.MinPoint(data.Z()), bvh.MaxPoint(data.Z()), out TMetric metricRgh);
                    if (Stop())
                    {
                        return nbAccepted;
                    }

                    if (isGoodLft && isGoodRgh)
                    {
                        // Chose the branch with the best metric to be processed next,
                        // put the other branch in the stack
                        if (!(head < BVH_Constants_MaxTreeDepth - 1))
                        {
                            throw new ProgramErrorException("Error! BVH stack overflow");
                        }

                        if (IsMetricBetter(metricLft, metricRgh))
                        {
                            node = new BVH_NodeInStack(data.Y(), metricLft);
                            stack[++head] = new BVH_NodeInStack(data.Z(), metricRgh);
                        }
                        else
                        {
                            node = new BVH_NodeInStack(data.Z(), metricRgh);
                            stack[++head] = new BVH_NodeInStack(data.Y(), metricLft);
                        }
                    }
                    else if (isGoodLft || isGoodRgh)
                    {
                        node = isGoodLft ? new BVH_NodeInStack(data.Y(), metricLft)
                                          : new BVH_NodeInStack(data.Z(), metricRgh);
                    }
                }
                else
                {
                    // Both children will be accepted
                    // Take one for processing, put the other into stack
                    if (!(head < BVH_Constants_MaxTreeDepth - 1))
                    {
                        throw new ProgramErrorException("Error! BVH stack overflow");
                    }

                    node = new BVH_NodeInStack(data.Y(), node.Metric);
                    stack[++head] = new BVH_NodeInStack(data.Z(), node.Metric);
                }
            }
            else
            {
                // Leaf node - apply the leaf node operation to each element
                for (int iN = data.Y(); iN <= data.Z(); ++iN)
                {
                    if (Accept(iN, node.Metric))
                    {
                        ++nbAccepted;
                    }

                    if (Stop())
                    {
                        return nbAccepted;
                    }
                }
            }

            if (node.NodeID == prevNode.NodeID)
            {
                if (head < 0)
                {
                    return nbAccepted;
                }

                // Remove the nodes with bad metric from the stack
                node = stack[head--];
                while (RejectMetric(node.Metric))
                {
                    if (head < 0)
                    {
                        return nbAccepted;
                    }

                    node = stack[head--];
                }
            }

            prevNode = node;
        }
    }
}

/// <summary>A single-tree traversal without a set (OCCT's <c>BVHSetType = void</c>): use <c>Select(tree)</c>.</summary>
public abstract class Traverse<T, TVec, TMetric> : Traverse<T, TVec, object, TMetric>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
    where TMetric : new()
{
}

/// <summary>
/// Parallel traversal of two binary trees, selecting pairs of elements; see <see cref="BaseTraverse{TMetric}"/>
/// for the rules to define.
/// </summary>
public abstract class PairTraverse<T, TVec, TSet, TMetric> : BaseTraverse<TMetric>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
    where TSet : class
    where TMetric : new()
{
    protected TSet? _bvhSet1;
    protected TSet? _bvhSet2;

    protected PairTraverse()
    {
        _bvhSet1 = null;
        _bvhSet2 = null;
    }

    /// <summary>Sets the BVH sets containing the trees.</summary>
    public void SetBVHSets(TSet? bvhSet1, TSet? bvhSet2)
    {
        _bvhSet1 = bvhSet1;
        _bvhSet2 = bvhSet2;
    }

    /// <summary>Rejects a pair of nodes by their bounding boxes, computing their metric; true if rejected.</summary>
    public abstract bool RejectNode(in TVec cornerMin1, in TVec cornerMax1, in TVec cornerMin2, in TVec cornerMax2, out TMetric metric);

    /// <summary>Accepts a pair of leaf elements; true if accepted.</summary>
    public abstract bool Accept(int index1, int index2);

    /// <summary>Selects pairs of elements of the trees of the two sets (which must be set); returns the number accepted.</summary>
    public int Select()
    {
        if (_bvhSet1 is not null && _bvhSet2 is not null)
        {
            Tree<T, TVec>? bvh1 = TreeOf<T, TVec>(_bvhSet1);
            Tree<T, TVec>? bvh2 = TreeOf<T, TVec>(_bvhSet2);
            return Select(bvh1, bvh2);
        }

        return 0;
    }

    /// <summary>A pair of nodes to process, with its metric.</summary>
    protected struct BVH_PairNodesInStack
    {
        public int NodeID1;     //!< Id of the node in the first BVH tree
        public int NodeID2;     //!< Id of the node in the second BVH tree
        public TMetric Metric;  //!< Metric computed for the pair of nodes

        public BVH_PairNodesInStack(int nodeID1, int nodeID2, TMetric metric)
        {
            NodeID1 = nodeID1;
            NodeID2 = nodeID2;
            Metric = metric;
        }
    }

    [InlineArray(3 * BVH_Constants_MaxTreeDepth)]
    private struct PairStack
    {
        private BVH_PairNodesInStack _element0;
    }

    [InlineArray(4)]
    private struct PairQuad
    {
        private BVH_PairNodesInStack _element0;
    }

    /// <summary>Selects pairs of elements of the two given trees; returns the number accepted.</summary>
    public int Select(Tree<T, TVec>? bvh1, Tree<T, TVec>? bvh2)
    {
        if (bvh1 is null || bvh2 is null)
        {
            return 0;
        }

        NCollection_LinearVector<NCollection_Vec4<int>> bvhNodes1 = bvh1.NodeInfoBuffer();
        NCollection_LinearVector<NCollection_Vec4<int>> bvhNodes2 = bvh2.NodeInfoBuffer();
        if (bvhNodes1.IsEmpty() || bvhNodes2.IsEmpty())
        {
            return 0;
        }

        // On each iteration we can add max four new pairs of nodes to process.
        // One of these pairs goes directly to processing, while others
        // are put in the stack. So the max number of pairs in the stack is
        // the max tree depth multiplied by 3.
        const int maxNbPairsInStack = 3 * BVH_Constants_MaxTreeDepth;

        // Stack of pairs of nodes to process
        PairStack stack = default;

        // Currently processed pair, starting with the root nodes
        BVH_PairNodesInStack node = new(0, 0, new TMetric());

        // Previously processed pair
        BVH_PairNodesInStack prevNode = node;

        // End of the stack
        int head = -1;

        // Counter for accepted elements
        int nbAccepted = 0;

        for (;;)
        {
            NCollection_Vec4<int> data1 = bvhNodes1[node.NodeID1];
            NCollection_Vec4<int> data2 = bvhNodes2[node.NodeID2];

            if (data1.X() != 0 && data2.X() != 0)
            {
                // Outer/Outer - both nodes are leaves
                // Check if the leaf node bounding boxes overlap before testing elements
                bool isRejected = RejectNode(bvh1.MinPoint(node.NodeID1),
                                             bvh1.MaxPoint(node.NodeID1),
                                             bvh2.MinPoint(node.NodeID2),
                                             bvh2.MaxPoint(node.NodeID2),
                                             out _);
                if (!isRejected)
                {
                    // Bounding boxes overlap, test all element pairs
                    for (int iN1 = data1.Y(); iN1 <= data1.Z(); ++iN1)
                    {
                        for (int iN2 = data2.Y(); iN2 <= data2.Z(); ++iN2)
                        {
                            if (Accept(iN1, iN2))
                            {
                                ++nbAccepted;
                            }

                            if (Stop())
                            {
                                return nbAccepted;
                            }
                        }
                    }
                }
            }
            else
            {
                PairQuad pairs = default;
                int nbPairs = 0;

                if (data1.X() == 0 && data2.X() == 0)
                {
                    // Inner/Inner
                    pairs[nbPairs++] = new BVH_PairNodesInStack(data1.Y(), data2.Y(), default!);
                    pairs[nbPairs++] = new BVH_PairNodesInStack(data1.Y(), data2.Z(), default!);
                    pairs[nbPairs++] = new BVH_PairNodesInStack(data1.Z(), data2.Y(), default!);
                    pairs[nbPairs++] = new BVH_PairNodesInStack(data1.Z(), data2.Z(), default!);
                }
                else if (data1.X() == 0)
                {
                    // Inner/Outer
                    pairs[nbPairs++] = new BVH_PairNodesInStack(data1.Y(), node.NodeID2, default!);
                    pairs[nbPairs++] = new BVH_PairNodesInStack(data1.Z(), node.NodeID2, default!);
                }
                else if (data2.X() == 0)
                {
                    // Outer/Inner
                    pairs[nbPairs++] = new BVH_PairNodesInStack(node.NodeID1, data2.Y(), default!);
                    pairs[nbPairs++] = new BVH_PairNodesInStack(node.NodeID1, data2.Z(), default!);
                }

                PairQuad keptPairs = default;
                int nbKept = 0;

                // Compute metrics for the nodes
                for (int iPair = 0; iPair < nbPairs; ++iPair)
                {
                    bool isPairRejected = RejectNode(bvh1.MinPoint(pairs[iPair].NodeID1),
                                                     bvh1.MaxPoint(pairs[iPair].NodeID1),
                                                     bvh2.MinPoint(pairs[iPair].NodeID2),
                                                     bvh2.MaxPoint(pairs[iPair].NodeID2),
                                                     out pairs[iPair].Metric);
                    if (!isPairRejected)
                    {
                        // Put the item into the sorted array of pairs
                        int iSort = nbKept;
                        while (iSort > 0 && IsMetricBetter(pairs[iPair].Metric, keptPairs[iSort - 1].Metric))
                        {
                            keptPairs[iSort] = keptPairs[iSort - 1];
                            --iSort;
                        }

                        keptPairs[iSort] = pairs[iPair];
                        ++nbKept;
                    }
                }

                if (nbKept > 0)
                {
                    node = keptPairs[0];

                    for (int iPair = 1; iPair < nbKept; ++iPair)
                    {
                        if (!(head < maxNbPairsInStack - 1))
                        {
                            throw new ProgramErrorException("Error! BVH pair stack overflow");
                        }

                        stack[++head] = keptPairs[iPair];
                    }
                }
            }

            if (node.NodeID1 == prevNode.NodeID1 && node.NodeID2 == prevNode.NodeID2)
            {
                // No pairs to add
                if (head < 0)
                {
                    return nbAccepted;
                }

                // Remove the pairs of nodes with bad metric from the stack
                node = stack[head--];
                while (RejectMetric(node.Metric))
                {
                    if (head < 0)
                    {
                        return nbAccepted;
                    }

                    node = stack[head--];
                }
            }

            prevNode = node;
        }
    }
}

/// <summary>A pair traversal without sets (OCCT's <c>BVHSetType = void</c>): use <c>Select(tree1, tree2)</c>.</summary>
public abstract class PairTraverse<T, TVec, TMetric> : PairTraverse<T, TVec, object, TMetric>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
    where TMetric : new()
{
}
