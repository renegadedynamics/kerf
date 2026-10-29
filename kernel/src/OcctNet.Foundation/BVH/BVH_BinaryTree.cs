// Port of: src/FoundationClasses/TKMath/BVH/BVH_BinaryTree.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Numerics;

namespace Occt;

/// <summary>
/// A binary BVH tree: OCCT's <c>BVH_Tree&lt;T, N, BVH_BinaryTree&gt;</c>, which is also its default
/// <c>BVH_Tree&lt;T, N&gt;</c>. The quad tree is <see cref="BVH_QuadTree{T, TVec}"/>; OCCT's arity tag types are
/// not needed.
/// </summary>
public class BVH_Tree<T, TVec> : BVH_TreeBase<T, TVec>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    /// <summary>Creates a new empty tree.</summary>
    public BVH_Tree()
    {
    }

    /// <summary>Sets the node type to 'outer' (leaf).</summary>
    public void SetOuter(int theNodeIndex) => NodeInfo(theNodeIndex, 0) = 1;

    /// <summary>Sets the node type to 'inner'.</summary>
    public void SetInner(int theNodeIndex) => NodeInfo(theNodeIndex, 0) = 0;

    /// <summary>The index of the K-th child (0 or 1) of the given inner node (OCCT's <c>Child&lt;K&gt;</c>).</summary>
    public ref int Child(int theK, int theNodeIndex) => ref NodeInfo(theNodeIndex, theK + 1);

    /// <summary>The index of the K-th child (0 or 1) of the given inner node, for writing.</summary>
    public ref int ChangeChild(int theK, int theNodeIndex) => ref NodeInfo(theNodeIndex, theK + 1);

    // Adding and removing nodes

    /// <summary>Removes all nodes from the tree.</summary>
    public void Clear()
    {
        myDepth = 0;
        myMinPointBuffer.Clear();
        myMaxPointBuffer.Clear();
        myNodeInfoBuffer.Clear();
    }

    /// <summary>Reserves storage for the given number of nodes.</summary>
    public void Reserve(int theNbNodes)
    {
        myMinPointBuffer.Reserve(theNbNodes);
        myMaxPointBuffer.Reserve(theNbNodes);
        myNodeInfoBuffer.Reserve(theNbNodes);
    }

    /// <summary>Adds a new leaf node; returns its index.</summary>
    public int AddLeafNode(in TVec theMinPoint, in TVec theMaxPoint, int theBegElem, int theEndElem)
    {
        myMinPointBuffer.Append(theMinPoint);
        myMaxPointBuffer.Append(theMaxPoint);
        myNodeInfoBuffer.Append(new NCollection_Vec4<int>(1, theBegElem, theEndElem, 0));
        return myNodeInfoBuffer.Size() - 1;
    }

    /// <summary>Adds a new inner node; returns its index.</summary>
    public int AddInnerNode(in TVec theMinPoint, in TVec theMaxPoint, int theLftChild, int theRghChild)
    {
        myMinPointBuffer.Append(theMinPoint);
        myMaxPointBuffer.Append(theMaxPoint);
        myNodeInfoBuffer.Append(new NCollection_Vec4<int>(0, theLftChild, theRghChild, 0));
        return myNodeInfoBuffer.Size() - 1;
    }

    /// <summary>Adds a new leaf node; returns its index.</summary>
    public int AddLeafNode(in BVH_Box<T, TVec> theAABB, int theBegElem, int theEndElem)
        => AddLeafNode(theAABB.CornerMin(), theAABB.CornerMax(), theBegElem, theEndElem);

    /// <summary>Adds a new inner node; returns its index.</summary>
    public int AddInnerNode(in BVH_Box<T, TVec> theAABB, int theLftChild, int theRghChild)
        => AddInnerNode(theAABB.CornerMin(), theAABB.CornerMax(), theLftChild, theRghChild);

    /// <summary>Adds a new leaf node without bounds (the point buffers are not extended); returns its index.</summary>
    public int AddLeafNode(int theBegElem, int theEndElem)
    {
        myNodeInfoBuffer.Append(new NCollection_Vec4<int>(1, theBegElem, theEndElem, 0));
        return myNodeInfoBuffer.Size() - 1;
    }

    /// <summary>Adds a new inner node without bounds (the point buffers are not extended); returns its index.</summary>
    public int AddInnerNode(int theLftChild, int theRghChild)
    {
        myNodeInfoBuffer.Append(new NCollection_Vec4<int>(0, theLftChild, theRghChild, 0));
        return myNodeInfoBuffer.Size() - 1;
    }

    // Methods specific to binary BVH

    /// <summary>
    /// The value of the surface area heuristic (SAH), to compare the quality of trees built for the same objects by
    /// different methods.
    /// </summary>
    public T EstimateSAH()
    {
        T aSAH = T.Zero;
        EstimateSAH(this, 0, T.One, ref aSAH);
        return aSAH;
    }

    /// <summary>
    /// Collapses the tree into a quad tree and returns it: every second level of this tree is kept and the rest are
    /// discarded.
    /// </summary>
    public BVH_QuadTree<T, TVec> CollapseToQuadTree()
    {
        BVH_QuadTree<T, TVec> aQBVH = new();

        if (Length() == 0)
        {
            return aQBVH;
        }

        Queue<(int Node, int Level)> aQueue = new();
        aQueue.Enqueue((0, 0));

        for (int aNbNodes = 1; aQueue.Count != 0;)
        {
            (int Node, int Level) aNode = aQueue.Peek();

            aQBVH.myMinPointBuffer.Append(myMinPointBuffer.Value(aNode.Node));
            aQBVH.myMaxPointBuffer.Append(myMaxPointBuffer.Value(aNode.Node));

            NCollection_Vec4<int> aNodeInfo;
            if (IsOuter(aNode.Node)) // is leaf node
            {
                aNodeInfo = new NCollection_Vec4<int>(1 /* leaf flag */,
                                                      BegPrimitive(aNode.Node),
                                                      EndPrimitive(aNode.Node),
                                                      aNode.Level /* level */);
            }
            else
            {
                NCollection_DynamicArray<int> aGrandChildNodes = new();

                int aLftChild = Child(0, aNode.Node);
                int aRghChild = Child(1, aNode.Node);
                if (IsOuter(aLftChild)) // is leaf node
                {
                    aGrandChildNodes.Append(aLftChild);
                }
                else
                {
                    aGrandChildNodes.Append(Child(0, aLftChild));
                    aGrandChildNodes.Append(Child(1, aLftChild));
                }

                if (IsOuter(aRghChild)) // is leaf node
                {
                    aGrandChildNodes.Append(aRghChild);
                }
                else
                {
                    aGrandChildNodes.Append(Child(0, aRghChild));
                    aGrandChildNodes.Append(Child(1, aRghChild));
                }

                for (int aNodeIdx = 0; aNodeIdx < aGrandChildNodes.Length(); ++aNodeIdx)
                {
                    aQueue.Enqueue((aGrandChildNodes.Value(aNodeIdx), aNode.Level + 1));
                }

                aNodeInfo = new NCollection_Vec4<int>(0 /* inner flag */,
                                                      aNbNodes,
                                                      aGrandChildNodes.Length() - 1,
                                                      aNode.Level /* level */);

                aQBVH.myDepth = Max(aQBVH.myDepth, aNode.Level + 1);

                aNbNodes += aGrandChildNodes.Length();
            }

            aQBVH.myNodeInfoBuffer.Append(aNodeInfo);
            aQueue.Dequeue(); // node processing completed
        }

        return aQBVH;
    }

    /// <summary>Recursive calculation of the surface area heuristic (<c>BVH::EstimateSAH</c>).</summary>
    private static void EstimateSAH(BVH_Tree<T, TVec> theTree, int theNode, T theProb, ref T theSAH)
    {
        BVH_Box<T, TVec> aBox = new(theTree.MinPoint(theNode), theTree.MaxPoint(theNode));

        if (theTree.IsOuter(theNode))
        {
            theSAH += theProb * T.CreateTruncating(theTree.EndPrimitive(theNode) - theTree.BegPrimitive(theNode) + 1);
        }
        else
        {
            theSAH += theProb * T.CreateTruncating(2.0);

            BVH_Box<T, TVec> aLftBox = new(theTree.MinPoint(theTree.Child(0, theNode)),
                                          theTree.MaxPoint(theTree.Child(0, theNode)));

            if (theProb > T.Zero)
            {
                EstimateSAH(theTree, theTree.Child(0, theNode), theProb * aLftBox.Area() / aBox.Area(), ref theSAH);
            }

            BVH_Box<T, TVec> aRghBox = new(theTree.MinPoint(theTree.Child(1, theNode)),
                                          theTree.MaxPoint(theTree.Child(1, theNode)));

            if (theProb > T.Zero)
            {
                EstimateSAH(theTree, theTree.Child(1, theNode), theProb * aRghBox.Area() / aBox.Area(), ref theSAH);
            }
        }
    }
}
