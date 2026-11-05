// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_EBTree.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2002-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Occt;

namespace Kerf.Engine.Collections;

/// <summary>
/// An unbalanced binary tree of overlapping bounding volumes from which objects can also be removed: a
/// <see cref="SpatialTree{TheObjType, TheBndType}"/> that also maps each object to its leaf.
/// </summary>
/// <remarks>
/// The objects are the keys of a map, so each object can be in the tree once, compared with
/// the constructor's comparer (OCCT's hasher; the type's default equality when null). OCCT's map holds pointers to
/// the nodes and fixes them where Gemmate and Kill move a node's content to another node; the port's nodes are objects
/// moved the same way (the content of a node is copied into a new child), so the same fix-ups are made. The tree can't
/// be copied; the C++ move constructor and move assignment are the constructor taking a tree and <see cref="Move"/>.
/// </remarks>
internal class IndexedSpatialTree<TheObjType, TheBndType> : SpatialTree<TheObjType, TheBndType>
    where TheObjType : notnull
    where TheBndType : struct, ISpatialTreeBox<TheBndType>
{
    private readonly Dictionary<TheObjType, TreeNode> _objNodeMap; // map of object to node

    /// <summary>An empty tree.</summary>
    public IndexedSpatialTree(IEqualityComparer<TheObjType>? comparer = null)
    {
        _objNodeMap = new Dictionary<TheObjType, TreeNode>(comparer);
    }

    /// <summary>Takes the nodes and map of <paramref name="other"/>, which is left empty (C++'s move constructor).</summary>
    public IndexedSpatialTree(IndexedSpatialTree<TheObjType, TheBndType> other)
        : base(other)
    {
        _objNodeMap = new Dictionary<TheObjType, TreeNode>(other._objNodeMap,other._objNodeMap.Comparer);
        other._objNodeMap.Clear();
    }

    /// <summary>
    /// Clears this tree and takes the nodes and map of <paramref name="other"/>, which is left empty (C++'s move
    /// assignment).
    /// </summary>
    public IndexedSpatialTree<TheObjType, TheBndType> Move(IndexedSpatialTree<TheObjType, TheBndType> other)
    {
        if (!ReferenceEquals(this, other))
        {
            base.Move(other);
            _objNodeMap.Clear();
            foreach (KeyValuePair<TheObjType, TreeNode> item in other._objNodeMap)
            {
                _objNodeMap[item.Key] = item.Value;
            }
            other._objNodeMap.Clear();
        }
        return this;
    }

    /// <summary>
    /// Adds the object with its bounding volume, and maps it to its leaf. False if the tree already contains it.
    /// </summary>
    public override bool Add(in TheObjType obj, in TheBndType bnd)
    {
        bool result = false;
        if (!Contains(obj))
        {
            // Add object in the tree using parent method
            base.Add(obj, bnd);

            // Update the map
            TreeNode newNode = ChangeLastNode();
            _objNodeMap[obj] = newNode;
            // If the new node is not the root (has a parent) check the neighbour node.
            // Gemmate moved old content to child(0), so the map entry points to a stale node.
            if (!newNode.IsRoot)
            {
                TreeNode neiNode = newNode.ChangeParent().ChangeChild(0);
                if (neiNode.IsLeaf)
                {
                    if (_objNodeMap.ContainsKey(neiNode.Object()))
                    {
                        _objNodeMap[neiNode.Object()] = neiNode;
                    }
                }
            }
            result = true;
        }
        return result;
    }

    /// <summary>Removes the object and updates the tree. False if the tree doesn't contain it.</summary>
    public bool Remove(in TheObjType obj)
    {
        if (!_objNodeMap.TryGetValue(obj, out TreeNode? pNode))
        {
            return false;
        }

        if (pNode.IsRoot)
        {
            // it is the root, so clear all the tree
            Clear();
        }
        else
        {
            // it is a child of some parent,
            // so kill the child that contains theObj
            // and update bounding boxes of all ancestors
            _objNodeMap.Remove(obj);
            TreeNode pParent = pNode.ChangeParent();
            pParent.Kill(ReferenceEquals(pNode, pParent.Child(0)) ? 0 : 1);
            if (pParent.IsLeaf)
            {
                // The parent node became a leaf (absorbed surviving child),
                // so update the map entry to point to the new node.
                if (_objNodeMap.ContainsKey(pParent.Object()))
                {
                    _objNodeMap[pParent.Object()] = pParent;
                }
            }
            while (!pParent.IsRoot)
            {
                pParent = pParent.ChangeParent();
                pParent.ChangeBnd() = pParent.Child(0).Bnd();
                TheBndType child1 = pParent.Child(1).Bnd();
                pParent.ChangeBnd().Add(child1);
            }
        }
        return true;
    }

    /// <summary>True if the tree contains the object.</summary>
    public bool Contains(in TheObjType obj) => _objNodeMap.ContainsKey(obj);

    /// <summary>The leaf holding the object; throws <see cref="NoSuchObjectException"/> if there is none.</summary>
    public TreeNode FindNode(in TheObjType obj) => _objNodeMap.GetValue(obj);

    /// <summary>Clears the tree.</summary>
    public override void Clear()
    {
        _objNodeMap.Clear();
        base.Clear();
    }
}
