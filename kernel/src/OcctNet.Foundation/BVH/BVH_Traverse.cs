// Port of: src/FoundationClasses/TKMath/BVH/BVH_Traverse.hxx
// Port of: src/FoundationClasses/TKMath/BVH/BVH_Traverse.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2019-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Numerics;
using System.Runtime.CompilerServices;
using static Occt.BVH_Constants;

namespace Occt;

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
public abstract class BVH_BaseTraverse<TMetric>
{
    protected BVH_BaseTraverse()
    {
    }

    /// <summary>True if the first metric is better than the second (the default keeps the left-to-right descent).</summary>
    public virtual bool IsMetricBetter(in TMetric theLeft, in TMetric theRight) => true;

    /// <summary>True if the node with this metric should be rejected (the default rejects nothing).</summary>
    public virtual bool RejectMetric(in TMetric theMetric) => false;

    /// <summary>True if the descent should stop (the default never stops).</summary>
    public virtual bool Stop() => false;

    /// <summary>The tree of a set: OCCT calls <c>BVH()</c> on whatever set type is given.</summary>
    private protected static BVH_Tree<T, TVec>? TreeOf<T, TVec>(object? theSet)
        where T : unmanaged, INumber<T>, IMinMaxValue<T>
        where TVec : unmanaged, INCollection_Vec<T, TVec>
        => theSet switch
        {
            null => null,
            BVH_PrimitiveSet<T, TVec> aPrimitiveSet => aPrimitiveSet.BVH(),
            BVH_Geometry<T, TVec> aGeometry => aGeometry.BVH(),
            _ => throw new Standard_TypeMismatch("BVH_Traverse: the set has no BVH() (not a BVH_PrimitiveSet or BVH_Geometry)"),
        };
}

/// <summary>
/// Traversal of a single binary tree; see <see cref="BVH_BaseTraverse{TMetric}"/> for the rules to define.
/// </summary>
/// <typeparam name="T">The element type.</typeparam>
/// <typeparam name="TVec">The vector type (dimension).</typeparam>
/// <typeparam name="TSet">The type of the set holding the tree, to reach the elements by index.</typeparam>
/// <typeparam name="TMetric">The type of the metric.</typeparam>
public abstract class BVH_Traverse<T, TVec, TSet, TMetric> : BVH_BaseTraverse<TMetric>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
    where TSet : class
    where TMetric : new()
{
    protected TSet? myBVHSet;

    protected BVH_Traverse()
    {
        myBVHSet = null;
    }

    /// <summary>Sets the BVH set containing the tree.</summary>
    public void SetBVHSet(TSet? theBVHSet) => myBVHSet = theBVHSet;

    /// <summary>True if the whole branch with this metric may be accepted without further checks (the default: no).</summary>
    public virtual bool AcceptMetric(in TMetric theMetric) => false;

    /// <summary>Rejects a node by its bounding box, computing its metric; true if the node should be rejected.</summary>
    public abstract bool RejectNode(in TVec theCornerMin, in TVec theCornerMax, out TMetric theMetric);

    /// <summary>
    /// Accepts a leaf element; the metric of the parent leaf node is passed to allow accepting it unconditionally.
    /// True if the element has been accepted.
    /// </summary>
    public abstract bool Accept(int theIndex, in TMetric theMetric);

    /// <summary>Selects the elements of the tree of the set (which must be set); returns the number accepted.</summary>
    public int Select()
    {
        if (myBVHSet is not null)
        {
            BVH_Tree<T, TVec>? aBVH = TreeOf<T, TVec>(myBVHSet);
            return Select(aBVH);
        }

        return 0;
    }

    /// <summary>A node to process, with its metric.</summary>
    protected struct BVH_NodeInStack
    {
        public int NodeID;      //!< Id of the node in the BVH tree
        public TMetric Metric;  //!< Metric computed for the node

        public BVH_NodeInStack(int theNodeID, TMetric theMetric)
        {
            NodeID = theNodeID;
            Metric = theMetric;
        }
    }

    [InlineArray(BVH_Constants_MaxTreeDepth)]
    private struct NodeStack
    {
        private BVH_NodeInStack myElement0;
    }

    /// <summary>Selects the elements of the given tree; returns the number accepted.</summary>
    public int Select(BVH_Tree<T, TVec>? theBVH)
    {
        if (theBVH is null)
        {
            return 0;
        }

        NCollection_LinearVector<NCollection_Vec4<int>> aBVHNodes = theBVH.NodeInfoBuffer();
        if (aBVHNodes.IsEmpty())
        {
            return 0;
        }

        // Create stack
        NodeStack aStack = default;

        BVH_NodeInStack aNode = new(0, new TMetric()); // Currently processed node, starting with the root node
        BVH_NodeInStack aPrevNode = aNode;             // Previously processed node

        int aHead = -1;      // End of the stack
        int aNbAccepted = 0; // Counter for accepted elements

        for (;;)
        {
            NCollection_Vec4<int> aData = aBVHNodes[aNode.NodeID];

            if (aData.x() == 0)
            {
                // Inner node:
                // - check the metric of the node
                // - test the children of the node

                if (!AcceptMetric(aNode.Metric))
                {
                    // Test the left branch
                    bool isGoodLft = !RejectNode(theBVH.MinPoint(aData.y()), theBVH.MaxPoint(aData.y()), out TMetric aMetricLft);
                    if (Stop())
                    {
                        return aNbAccepted;
                    }

                    // Test the right branch
                    bool isGoodRgh = !RejectNode(theBVH.MinPoint(aData.z()), theBVH.MaxPoint(aData.z()), out TMetric aMetricRgh);
                    if (Stop())
                    {
                        return aNbAccepted;
                    }

                    if (isGoodLft && isGoodRgh)
                    {
                        // Chose the branch with the best metric to be processed next,
                        // put the other branch in the stack
                        if (!(aHead < BVH_Constants_MaxTreeDepth - 1))
                        {
                            throw new Standard_ProgramError("Error! BVH stack overflow");
                        }

                        if (IsMetricBetter(aMetricLft, aMetricRgh))
                        {
                            aNode = new BVH_NodeInStack(aData.y(), aMetricLft);
                            aStack[++aHead] = new BVH_NodeInStack(aData.z(), aMetricRgh);
                        }
                        else
                        {
                            aNode = new BVH_NodeInStack(aData.z(), aMetricRgh);
                            aStack[++aHead] = new BVH_NodeInStack(aData.y(), aMetricLft);
                        }
                    }
                    else if (isGoodLft || isGoodRgh)
                    {
                        aNode = isGoodLft ? new BVH_NodeInStack(aData.y(), aMetricLft)
                                          : new BVH_NodeInStack(aData.z(), aMetricRgh);
                    }
                }
                else
                {
                    // Both children will be accepted
                    // Take one for processing, put the other into stack
                    if (!(aHead < BVH_Constants_MaxTreeDepth - 1))
                    {
                        throw new Standard_ProgramError("Error! BVH stack overflow");
                    }

                    aNode = new BVH_NodeInStack(aData.y(), aNode.Metric);
                    aStack[++aHead] = new BVH_NodeInStack(aData.z(), aNode.Metric);
                }
            }
            else
            {
                // Leaf node - apply the leaf node operation to each element
                for (int iN = aData.y(); iN <= aData.z(); ++iN)
                {
                    if (Accept(iN, aNode.Metric))
                    {
                        ++aNbAccepted;
                    }

                    if (Stop())
                    {
                        return aNbAccepted;
                    }
                }
            }

            if (aNode.NodeID == aPrevNode.NodeID)
            {
                if (aHead < 0)
                {
                    return aNbAccepted;
                }

                // Remove the nodes with bad metric from the stack
                aNode = aStack[aHead--];
                while (RejectMetric(aNode.Metric))
                {
                    if (aHead < 0)
                    {
                        return aNbAccepted;
                    }

                    aNode = aStack[aHead--];
                }
            }

            aPrevNode = aNode;
        }
    }
}

/// <summary>A single-tree traversal without a set (OCCT's <c>BVHSetType = void</c>): use <c>Select(tree)</c>.</summary>
public abstract class BVH_Traverse<T, TVec, TMetric> : BVH_Traverse<T, TVec, object, TMetric>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
    where TMetric : new()
{
}

/// <summary>
/// Parallel traversal of two binary trees, selecting pairs of elements; see <see cref="BVH_BaseTraverse{TMetric}"/>
/// for the rules to define.
/// </summary>
public abstract class BVH_PairTraverse<T, TVec, TSet, TMetric> : BVH_BaseTraverse<TMetric>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
    where TSet : class
    where TMetric : new()
{
    protected TSet? myBVHSet1;
    protected TSet? myBVHSet2;

    protected BVH_PairTraverse()
    {
        myBVHSet1 = null;
        myBVHSet2 = null;
    }

    /// <summary>Sets the BVH sets containing the trees.</summary>
    public void SetBVHSets(TSet? theBVHSet1, TSet? theBVHSet2)
    {
        myBVHSet1 = theBVHSet1;
        myBVHSet2 = theBVHSet2;
    }

    /// <summary>Rejects a pair of nodes by their bounding boxes, computing their metric; true if rejected.</summary>
    public abstract bool RejectNode(in TVec theCornerMin1, in TVec theCornerMax1, in TVec theCornerMin2, in TVec theCornerMax2, out TMetric theMetric);

    /// <summary>Accepts a pair of leaf elements; true if accepted.</summary>
    public abstract bool Accept(int theIndex1, int theIndex2);

    /// <summary>Selects pairs of elements of the trees of the two sets (which must be set); returns the number accepted.</summary>
    public int Select()
    {
        if (myBVHSet1 is not null && myBVHSet2 is not null)
        {
            BVH_Tree<T, TVec>? aBVH1 = TreeOf<T, TVec>(myBVHSet1);
            BVH_Tree<T, TVec>? aBVH2 = TreeOf<T, TVec>(myBVHSet2);
            return Select(aBVH1, aBVH2);
        }

        return 0;
    }

    /// <summary>A pair of nodes to process, with its metric.</summary>
    protected struct BVH_PairNodesInStack
    {
        public int NodeID1;     //!< Id of the node in the first BVH tree
        public int NodeID2;     //!< Id of the node in the second BVH tree
        public TMetric Metric;  //!< Metric computed for the pair of nodes

        public BVH_PairNodesInStack(int theNodeID1, int theNodeID2, TMetric theMetric)
        {
            NodeID1 = theNodeID1;
            NodeID2 = theNodeID2;
            Metric = theMetric;
        }
    }

    [InlineArray(3 * BVH_Constants_MaxTreeDepth)]
    private struct PairStack
    {
        private BVH_PairNodesInStack myElement0;
    }

    [InlineArray(4)]
    private struct PairQuad
    {
        private BVH_PairNodesInStack myElement0;
    }

    /// <summary>Selects pairs of elements of the two given trees; returns the number accepted.</summary>
    public int Select(BVH_Tree<T, TVec>? theBVH1, BVH_Tree<T, TVec>? theBVH2)
    {
        if (theBVH1 is null || theBVH2 is null)
        {
            return 0;
        }

        NCollection_LinearVector<NCollection_Vec4<int>> aBVHNodes1 = theBVH1.NodeInfoBuffer();
        NCollection_LinearVector<NCollection_Vec4<int>> aBVHNodes2 = theBVH2.NodeInfoBuffer();
        if (aBVHNodes1.IsEmpty() || aBVHNodes2.IsEmpty())
        {
            return 0;
        }

        // On each iteration we can add max four new pairs of nodes to process.
        // One of these pairs goes directly to processing, while others
        // are put in the stack. So the max number of pairs in the stack is
        // the max tree depth multiplied by 3.
        const int aMaxNbPairsInStack = 3 * BVH_Constants_MaxTreeDepth;

        // Stack of pairs of nodes to process
        PairStack aStack = default;

        // Currently processed pair, starting with the root nodes
        BVH_PairNodesInStack aNode = new(0, 0, new TMetric());

        // Previously processed pair
        BVH_PairNodesInStack aPrevNode = aNode;

        // End of the stack
        int aHead = -1;

        // Counter for accepted elements
        int aNbAccepted = 0;

        for (;;)
        {
            NCollection_Vec4<int> aData1 = aBVHNodes1[aNode.NodeID1];
            NCollection_Vec4<int> aData2 = aBVHNodes2[aNode.NodeID2];

            if (aData1.x() != 0 && aData2.x() != 0)
            {
                // Outer/Outer - both nodes are leaves
                // Check if the leaf node bounding boxes overlap before testing elements
                bool isRejected = RejectNode(theBVH1.MinPoint(aNode.NodeID1),
                                             theBVH1.MaxPoint(aNode.NodeID1),
                                             theBVH2.MinPoint(aNode.NodeID2),
                                             theBVH2.MaxPoint(aNode.NodeID2),
                                             out _);
                if (!isRejected)
                {
                    // Bounding boxes overlap, test all element pairs
                    for (int iN1 = aData1.y(); iN1 <= aData1.z(); ++iN1)
                    {
                        for (int iN2 = aData2.y(); iN2 <= aData2.z(); ++iN2)
                        {
                            if (Accept(iN1, iN2))
                            {
                                ++aNbAccepted;
                            }

                            if (Stop())
                            {
                                return aNbAccepted;
                            }
                        }
                    }
                }
            }
            else
            {
                PairQuad aPairs = default;
                int aNbPairs = 0;

                if (aData1.x() == 0 && aData2.x() == 0)
                {
                    // Inner/Inner
                    aPairs[aNbPairs++] = new BVH_PairNodesInStack(aData1.y(), aData2.y(), default!);
                    aPairs[aNbPairs++] = new BVH_PairNodesInStack(aData1.y(), aData2.z(), default!);
                    aPairs[aNbPairs++] = new BVH_PairNodesInStack(aData1.z(), aData2.y(), default!);
                    aPairs[aNbPairs++] = new BVH_PairNodesInStack(aData1.z(), aData2.z(), default!);
                }
                else if (aData1.x() == 0)
                {
                    // Inner/Outer
                    aPairs[aNbPairs++] = new BVH_PairNodesInStack(aData1.y(), aNode.NodeID2, default!);
                    aPairs[aNbPairs++] = new BVH_PairNodesInStack(aData1.z(), aNode.NodeID2, default!);
                }
                else if (aData2.x() == 0)
                {
                    // Outer/Inner
                    aPairs[aNbPairs++] = new BVH_PairNodesInStack(aNode.NodeID1, aData2.y(), default!);
                    aPairs[aNbPairs++] = new BVH_PairNodesInStack(aNode.NodeID1, aData2.z(), default!);
                }

                PairQuad aKeptPairs = default;
                int aNbKept = 0;

                // Compute metrics for the nodes
                for (int iPair = 0; iPair < aNbPairs; ++iPair)
                {
                    bool isPairRejected = RejectNode(theBVH1.MinPoint(aPairs[iPair].NodeID1),
                                                     theBVH1.MaxPoint(aPairs[iPair].NodeID1),
                                                     theBVH2.MinPoint(aPairs[iPair].NodeID2),
                                                     theBVH2.MaxPoint(aPairs[iPair].NodeID2),
                                                     out aPairs[iPair].Metric);
                    if (!isPairRejected)
                    {
                        // Put the item into the sorted array of pairs
                        int iSort = aNbKept;
                        while (iSort > 0 && IsMetricBetter(aPairs[iPair].Metric, aKeptPairs[iSort - 1].Metric))
                        {
                            aKeptPairs[iSort] = aKeptPairs[iSort - 1];
                            --iSort;
                        }

                        aKeptPairs[iSort] = aPairs[iPair];
                        ++aNbKept;
                    }
                }

                if (aNbKept > 0)
                {
                    aNode = aKeptPairs[0];

                    for (int iPair = 1; iPair < aNbKept; ++iPair)
                    {
                        if (!(aHead < aMaxNbPairsInStack - 1))
                        {
                            throw new Standard_ProgramError("Error! BVH pair stack overflow");
                        }

                        aStack[++aHead] = aKeptPairs[iPair];
                    }
                }
            }

            if (aNode.NodeID1 == aPrevNode.NodeID1 && aNode.NodeID2 == aPrevNode.NodeID2)
            {
                // No pairs to add
                if (aHead < 0)
                {
                    return aNbAccepted;
                }

                // Remove the pairs of nodes with bad metric from the stack
                aNode = aStack[aHead--];
                while (RejectMetric(aNode.Metric))
                {
                    if (aHead < 0)
                    {
                        return aNbAccepted;
                    }

                    aNode = aStack[aHead--];
                }
            }

            aPrevNode = aNode;
        }
    }
}

/// <summary>A pair traversal without sets (OCCT's <c>BVHSetType = void</c>): use <c>Select(tree1, tree2)</c>.</summary>
public abstract class BVH_PairTraverse<T, TVec, TMetric> : BVH_PairTraverse<T, TVec, object, TMetric>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
    where TMetric : new()
{
}
