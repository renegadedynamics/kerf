// Port of: src/FoundationClasses/TKMath/BVH/BVH_BinaryTree.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Numerics;

namespace Kerf.Engine.Geometry.Bvh;

/// <summary>
/// A binary BVH tree: OCCT's <c>BVH_Tree&lt;T, N, BVH_BinaryTree&gt;</c>, which is also its default
/// <c>BVH_Tree&lt;T, N&gt;</c>. The quad tree is <see cref="QuadTree{T, TVec}"/>; OCCT's arity tag types are
/// not needed.
/// </summary>
internal class Tree<T, TVec> : TreeBase<T, TVec>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    /// <summary>Creates a new empty tree.</summary>
    public Tree()
    {
    }

    /// <summary>Sets the node type to 'outer' (leaf).</summary>
    public void SetOuter(int nodeIndex) => NodeInfo(nodeIndex, 0) = 1;

    /// <summary>Sets the node type to 'inner'.</summary>
    public void SetInner(int nodeIndex) => NodeInfo(nodeIndex, 0) = 0;

    /// <summary>The index of the K-th child (0 or 1) of the given inner node (OCCT's <c>Child&lt;K&gt;</c>).</summary>
    public ref int Child(int k, int nodeIndex) => ref NodeInfo(nodeIndex, k + 1);

    /// <summary>The index of the K-th child (0 or 1) of the given inner node, for writing.</summary>
    public ref int ChangeChild(int k, int nodeIndex) => ref NodeInfo(nodeIndex, k + 1);

    // Adding and removing nodes

    /// <summary>Removes all nodes from the tree.</summary>
    public void Clear()
    {
        _depth = 0;
        _minPointBuffer.Clear();
        _maxPointBuffer.Clear();
        _nodeInfoBuffer.Clear();
    }

    /// <summary>Reserves storage for the given number of nodes.</summary>
    public void Reserve(int nbNodes)
    {
        _minPointBuffer.Reserve(nbNodes);
        _maxPointBuffer.Reserve(nbNodes);
        _nodeInfoBuffer.Reserve(nbNodes);
    }

    /// <summary>Adds a new leaf node; returns its index.</summary>
    public int AddLeafNode(in TVec minPoint, in TVec maxPoint, int begElem, int endElem)
    {
        _minPointBuffer.Append(minPoint);
        _maxPointBuffer.Append(maxPoint);
        _nodeInfoBuffer.Append(new NCollection_Vec4<int>(1, begElem, endElem, 0));
        return _nodeInfoBuffer.Size() - 1;
    }

    /// <summary>Adds a new inner node; returns its index.</summary>
    public int AddInnerNode(in TVec minPoint, in TVec maxPoint, int lftChild, int rghChild)
    {
        _minPointBuffer.Append(minPoint);
        _maxPointBuffer.Append(maxPoint);
        _nodeInfoBuffer.Append(new NCollection_Vec4<int>(0, lftChild, rghChild, 0));
        return _nodeInfoBuffer.Size() - 1;
    }

    /// <summary>Adds a new leaf node; returns its index.</summary>
    public int AddLeafNode(in Box<T, TVec> aabb, int begElem, int endElem)
        => AddLeafNode(aabb.CornerMin(), aabb.CornerMax(), begElem, endElem);

    /// <summary>Adds a new inner node; returns its index.</summary>
    public int AddInnerNode(in Box<T, TVec> aabb, int lftChild, int rghChild)
        => AddInnerNode(aabb.CornerMin(), aabb.CornerMax(), lftChild, rghChild);

    /// <summary>Adds a new leaf node without bounds (the point buffers are not extended); returns its index.</summary>
    public int AddLeafNode(int begElem, int endElem)
    {
        _nodeInfoBuffer.Append(new NCollection_Vec4<int>(1, begElem, endElem, 0));
        return _nodeInfoBuffer.Size() - 1;
    }

    /// <summary>Adds a new inner node without bounds (the point buffers are not extended); returns its index.</summary>
    public int AddInnerNode(int lftChild, int rghChild)
    {
        _nodeInfoBuffer.Append(new NCollection_Vec4<int>(0, lftChild, rghChild, 0));
        return _nodeInfoBuffer.Size() - 1;
    }

    // Methods specific to binary BVH

    /// <summary>
    /// The value of the surface area heuristic (SAH), to compare the quality of trees built for the same objects by
    /// different methods.
    /// </summary>
    public T EstimateSAH()
    {
        T sah = T.Zero;
        EstimateSAH(this, 0, T.One, ref sah);
        return sah;
    }

    /// <summary>
    /// Collapses the tree into a quad tree and returns it: every second level of this tree is kept and the rest are
    /// discarded.
    /// </summary>
    public QuadTree<T, TVec> CollapseToQuadTree()
    {
        QuadTree<T, TVec> qbvh = new();

        if (Length() == 0)
        {
            return qbvh;
        }

        Queue<(int Node, int Level)> queue = new();
        queue.Enqueue((0, 0));

        for (int nbNodes = 1; queue.Count != 0;)
        {
            (int Node, int Level) node = queue.Peek();

            qbvh._minPointBuffer.Append(_minPointBuffer.Value(node.Node));
            qbvh._maxPointBuffer.Append(_maxPointBuffer.Value(node.Node));

            NCollection_Vec4<int> nodeInfo;
            if (IsOuter(node.Node)) // is leaf node
            {
                nodeInfo = new NCollection_Vec4<int>(1 /* leaf flag */,
                                                      BegPrimitive(node.Node),
                                                      EndPrimitive(node.Node),
                                                      node.Level /* level */);
            }
            else
            {
                List<int> grandChildNodes = new();

                int lftChild = Child(0, node.Node);
                int rghChild = Child(1, node.Node);
                if (IsOuter(lftChild)) // is leaf node
                {
                    grandChildNodes.Add(lftChild);
                }
                else
                {
                    grandChildNodes.Add(Child(0, lftChild));
                    grandChildNodes.Add(Child(1, lftChild));
                }

                if (IsOuter(rghChild)) // is leaf node
                {
                    grandChildNodes.Add(rghChild);
                }
                else
                {
                    grandChildNodes.Add(Child(0, rghChild));
                    grandChildNodes.Add(Child(1, rghChild));
                }

                for (int nodeIdx = 0; nodeIdx < grandChildNodes.Count; ++nodeIdx)
                {
                    queue.Enqueue((grandChildNodes[nodeIdx], node.Level + 1));
                }

                nodeInfo = new NCollection_Vec4<int>(0 /* inner flag */,
                                                      nbNodes,
                                                      grandChildNodes.Count - 1,
                                                      node.Level /* level */);

                qbvh._depth = Max(qbvh._depth, node.Level + 1);

                nbNodes += grandChildNodes.Count;
            }

            qbvh._nodeInfoBuffer.Append(nodeInfo);
            queue.Dequeue(); // node processing completed
        }

        return qbvh;
    }

    /// <summary>Recursive calculation of the surface area heuristic (<c>BVH::EstimateSAH</c>).</summary>
    private static void EstimateSAH(Tree<T, TVec> tree, int node, T prob, ref T sah)
    {
        Box<T, TVec> box = new(tree.MinPoint(node), tree.MaxPoint(node));

        if (tree.IsOuter(node))
        {
            sah += prob * T.CreateTruncating(tree.EndPrimitive(node) - tree.BegPrimitive(node) + 1);
        }
        else
        {
            sah += prob * T.CreateTruncating(2.0);

            Box<T, TVec> lftBox = new(tree.MinPoint(tree.Child(0, node)),
                                          tree.MaxPoint(tree.Child(0, node)));

            if (prob > T.Zero)
            {
                EstimateSAH(tree, tree.Child(0, node), prob * lftBox.Area() / box.Area(), ref sah);
            }

            Box<T, TVec> rghBox = new(tree.MinPoint(tree.Child(1, node)),
                                          tree.MaxPoint(tree.Child(1, node)));

            if (prob > T.Zero)
            {
                EstimateSAH(tree, tree.Child(1, node), prob * rghBox.Area() / box.Area(), ref sah);
            }
        }
    }
}
