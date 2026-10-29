// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_EBTree.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2002-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// An unbalanced binary tree of overlapping bounding volumes from which objects can also be removed: a
/// <see cref="NCollection_UBTree{TheObjType, TheBndType}"/> that also maps each object to its leaf.
/// </summary>
/// <remarks>
/// The objects are the keys of a map, so each object can be in the tree once, compared with
/// the constructor's comparer (OCCT's hasher; the type's default equality when null). OCCT's map holds pointers to
/// the nodes and fixes them where Gemmate and Kill move a node's content to another node; the port's nodes are objects
/// moved the same way (the content of a node is copied into a new child), so the same fix-ups are made. The tree can't
/// be copied; the C++ move constructor and move assignment are the constructor taking a tree and <see cref="Move"/>.
/// </remarks>
public class NCollection_EBTree<TheObjType, TheBndType> : NCollection_UBTree<TheObjType, TheBndType>
    where TheObjType : notnull
    where TheBndType : struct, INCollection_UBTreeBnd<TheBndType>
{
    private readonly NCollection_DataMap<TheObjType, TreeNode> myObjNodeMap; // map of object to node

    /// <summary>An empty tree.</summary>
    public NCollection_EBTree(IEqualityComparer<TheObjType>? theComparer = null)
    {
        myObjNodeMap = new NCollection_DataMap<TheObjType, TreeNode>(theComparer);
    }

    /// <summary>Takes the nodes and map of <paramref name="theOther"/>, which is left empty (C++'s move constructor).</summary>
    public NCollection_EBTree(NCollection_EBTree<TheObjType, TheBndType> theOther)
        : base(theOther)
    {
        myObjNodeMap = new NCollection_DataMap<TheObjType, TreeNode>(theOther.myObjNodeMap);
        theOther.myObjNodeMap.Clear();
    }

    /// <summary>
    /// Clears this tree and takes the nodes and map of <paramref name="theOther"/>, which is left empty (C++'s move
    /// assignment).
    /// </summary>
    public NCollection_EBTree<TheObjType, TheBndType> Move(NCollection_EBTree<TheObjType, TheBndType> theOther)
    {
        if (!ReferenceEquals(this, theOther))
        {
            base.Move(theOther);
            myObjNodeMap.Clear();
            foreach (KeyValuePair<TheObjType, TreeNode> anItem in theOther.myObjNodeMap)
            {
                myObjNodeMap.Bind(anItem.Key, anItem.Value);
            }
            theOther.myObjNodeMap.Clear();
        }
        return this;
    }

    /// <summary>
    /// Adds the object with its bounding volume, and maps it to its leaf. False if the tree already contains it.
    /// </summary>
    public override bool Add(in TheObjType theObj, in TheBndType theBnd)
    {
        bool result = false;
        if (!Contains(theObj))
        {
            // Add object in the tree using parent method
            base.Add(theObj, theBnd);

            // Update the map
            TreeNode aNewNode = ChangeLastNode();
            myObjNodeMap.Bind(theObj, aNewNode);
            // If the new node is not the root (has a parent) check the neighbour node.
            // Gemmate moved old content to child(0), so the map entry points to a stale node.
            if (!aNewNode.IsRoot())
            {
                TreeNode aNeiNode = aNewNode.ChangeParent().ChangeChild(0);
                if (aNeiNode.IsLeaf())
                {
                    if (myObjNodeMap.IsBound(aNeiNode.Object()))
                    {
                        myObjNodeMap.Bind(aNeiNode.Object(), aNeiNode);
                    }
                }
            }
            result = true;
        }
        return result;
    }

    /// <summary>Removes the object and updates the tree. False if the tree doesn't contain it.</summary>
    public bool Remove(in TheObjType theObj)
    {
        if (!myObjNodeMap.Find(theObj, out TreeNode pNode))
        {
            return false;
        }

        if (pNode.IsRoot())
        {
            // it is the root, so clear all the tree
            Clear();
        }
        else
        {
            // it is a child of some parent,
            // so kill the child that contains theObj
            // and update bounding boxes of all ancestors
            myObjNodeMap.UnBind(theObj);
            TreeNode pParent = pNode.ChangeParent();
            pParent.Kill(ReferenceEquals(pNode, pParent.Child(0)) ? 0 : 1);
            if (pParent.IsLeaf())
            {
                // The parent node became a leaf (absorbed surviving child),
                // so update the map entry to point to the new node.
                if (myObjNodeMap.IsBound(pParent.Object()))
                {
                    myObjNodeMap.Bind(pParent.Object(), pParent);
                }
            }
            while (!pParent.IsRoot())
            {
                pParent = pParent.ChangeParent();
                pParent.ChangeBnd() = pParent.Child(0).Bnd();
                TheBndType aChild1 = pParent.Child(1).Bnd();
                pParent.ChangeBnd().Add(aChild1);
            }
        }
        return true;
    }

    /// <summary>True if the tree contains the object.</summary>
    public bool Contains(in TheObjType theObj) => myObjNodeMap.IsBound(theObj);

    /// <summary>The leaf holding the object; throws <see cref="Standard_NoSuchObject"/> if there is none.</summary>
    public TreeNode FindNode(in TheObjType theObj) => myObjNodeMap.Find(theObj);

    /// <summary>Clears the tree.</summary>
    public override void Clear()
    {
        myObjNodeMap.Clear();
        base.Clear();
    }
}
