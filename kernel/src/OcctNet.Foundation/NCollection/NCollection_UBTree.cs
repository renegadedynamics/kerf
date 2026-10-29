// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_UBTree.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2002-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// What <see cref="NCollection_UBTree{TheObjType, TheBndType}"/> requires of its bounding type: OCCT's template
/// parameter <c>TheBndType</c>, which must have <c>Add</c>, <c>IsOut</c> and <c>SquareExtent</c>.
/// </summary>
/// <remarks>
/// The bounding types are value types whose <c>Add</c> is an internal mutator; they implement <see cref="Add"/>
/// explicitly. Called through the interface on a boxed copy it changes only that copy; the tree calls it on its own
/// nodes' values through the constraint, which changes them in place as C++ does.
/// </remarks>
public interface INCollection_UBTreeBnd<TheBndType> where TheBndType : struct, INCollection_UBTreeBnd<TheBndType>
{
    /// <summary>Enlarges this bounding volume to contain <paramref name="theOther"/>.</summary>
    void Add(in TheBndType theOther);

    /// <summary>True if this bounding volume and <paramref name="theOther"/> don't intersect.</summary>
    bool IsOut(in TheBndType theOther);

    /// <summary>The square of the volume's diagonal (extent).</summary>
    double SquareExtent();
}

/// <summary>
/// An unbalanced binary tree of bounding volumes: each object is stored with its bounding volume in a leaf, and every
/// branch has the union of its children's volumes. Objects are found with a <see cref="Selector"/> rejecting branches
/// by their volume.
/// </summary>
/// <remarks>
/// OCCT's nodes are allocated in pairs from an allocator and linked by pointers; the port's nodes are objects (the
/// allocator arguments are dropped, rules section 9). The tree can't be copied, as in OCCT; the C++ move constructor
/// and move assignment are the constructor taking a tree and <see cref="Move"/>, which take the other tree's nodes and
/// leave it empty.
/// </remarks>
public class NCollection_UBTree<TheObjType, TheBndType> where TheBndType : struct, INCollection_UBTreeBnd<TheBndType>
{
    /// <summary>The minimal interface of a selector.</summary>
    public abstract class Selector
    {
        /// <summary>
        /// Set by <see cref="Accept"/> when the selection process is to be stopped.
        /// </summary>
        protected bool myStop;

        protected Selector()
        {
            myStop = false;
        }

        /// <summary>True if the bounding volume does not conform to the selection conditions.</summary>
        /// <remarks>Const in OCCT; OCCT's own selectors cast that away, so it isn't readonly here.</remarks>
        public abstract bool Reject(in TheBndType theBnd);

        /// <summary>
        /// Confirms the object, called when its bounding volume conforms to the conditions (see <see cref="Reject"/>);
        /// keeps a record of accepted objects. True if the object is accepted.
        /// </summary>
        public abstract bool Accept(in TheObjType theObj);

        /// <summary>Checked after each call to Accept(): true stops the selection.</summary>
        public bool Stop() => myStop;
    }

    /// <summary>
    /// A node of the tree. Initially the tree is one leaf; a node grows to a branch holding two children: one with the
    /// node's previous content, the other with a new object and its bounding volume.
    /// </summary>
    public sealed class TreeNode
    {
        private TheBndType myBnd; // bounding geometry
        private TheObjType myObject; // the object
        private TreeNode[]? myChildren; // 2 children forming a b-tree
        private TreeNode? myParent; // the parent node

        internal TreeNode(in TheObjType theObj, in TheBndType theBnd)
        {
            myBnd = theBnd;
            myObject = theObj;
            myChildren = null;
            myParent = null;
        }

        private TreeNode()
        {
            myBnd = new TheBndType();
            myObject = NCollection_Default<TheObjType>.Value;
            myChildren = null;
            myParent = null;
        }

        public bool IsLeaf() => myChildren is null;

        public bool IsRoot() => myParent is null;

        public TheBndType Bnd() => myBnd;

        internal ref TheBndType ChangeBnd() => ref myBnd;

        public TheObjType Object() => myObject;

        public TreeNode Child(int i) => myChildren![i];

        internal TreeNode ChangeChild(int i) => myChildren![i];

        public TreeNode Parent() => myParent!;

        internal TreeNode ChangeParent() => myParent!;

        /// <summary>
        /// Makes this node a branch holding its previous content in the first child and theObj in the second.
        /// </summary>
        /// <param name="theNewBnd">new bounding volume comprising both child nodes.</param>
        /// <param name="theObj">added object.</param>
        /// <param name="theBnd">bounding volume of theObj.</param>
        internal void Gemmate(in TheBndType theNewBnd, in TheObjType theObj, in TheBndType theBnd)
        {
            TreeNode[] children = [new TreeNode(), new TreeNode()];
            // children[0] = *this
            children[0].myBnd = myBnd;
            children[0].myObject = myObject;
            children[0].myChildren = myChildren;
            children[0].myParent = myParent;
            children[1].myObject = theObj;
            children[1].myBnd = theBnd;
            children[0].myParent = children[1].myParent = this;
            if (!IsLeaf())
            {
                myChildren![0].myParent = children[0];
                myChildren[1].myParent = children[0];
            }
            myChildren = children;
            myBnd = theNewBnd;
            myObject = NCollection_Default<TheObjType>.Value; // nullify myObject
        }

        /// <summary>Kills the i-th child; this node takes the content of the other child.</summary>
        internal void Kill(int i)
        {
            if (!IsLeaf())
            {
                TreeNode[] oldChildren = myChildren!;
                int iopp = 1 - i;
                myBnd = oldChildren[iopp].myBnd;
                myObject = oldChildren[iopp].myObject;
                myChildren = oldChildren[iopp].myChildren;
                if (!IsLeaf())
                {
                    myChildren![0].myParent = this;
                    myChildren[1].myParent = this;
                }
                // OCCT destroys the other child and frees the killed branch: the garbage collector does
            }
        }
    }

    private TreeNode? myRoot; // root of the tree
    private TreeNode? myLastNode; // the last added node

    /// <summary>An empty tree.</summary>
    public NCollection_UBTree()
    {
        myRoot = null;
        myLastNode = null;
    }

    /// <summary>Takes the nodes of <paramref name="theOther"/>, which is left empty (C++'s move constructor).</summary>
    public NCollection_UBTree(NCollection_UBTree<TheObjType, TheBndType> theOther)
    {
        myRoot = theOther.myRoot;
        myLastNode = theOther.myLastNode;
        theOther.myRoot = null;
        theOther.myLastNode = null;
    }

    /// <summary>Clears this tree and takes the nodes of <paramref name="theOther"/>, which is left empty (C++'s move assignment).</summary>
    public NCollection_UBTree<TheObjType, TheBndType> Move(NCollection_UBTree<TheObjType, TheBndType> theOther)
    {
        if (!ReferenceEquals(this, theOther))
        {
            Clear();
            myRoot = theOther.myRoot;
            myLastNode = theOther.myLastNode;
            theOther.myRoot = null;
            theOther.myLastNode = null;
        }
        return this;
    }

    /// <summary>Adds an object with its bounding volume. Always true.</summary>
    public virtual bool Add(in TheObjType theObj, in TheBndType theBnd)
    {
        if (IsEmpty())
        {
            myRoot = new TreeNode(theObj, theBnd);
            myLastNode = myRoot;
            return true;
        }

        TreeNode pBranch = myRoot!;
        bool isOutOfBranch = pBranch.Bnd().IsOut(theBnd);

        for (;;)
        {
            if (isOutOfBranch || pBranch.IsLeaf())
            {
                TheBndType aNewBnd = theBnd;
                TheBndType aBranchBnd = pBranch.Bnd();
                aNewBnd.Add(aBranchBnd);
                pBranch.Gemmate(aNewBnd, theObj, theBnd);
                myLastNode = pBranch.ChangeChild(1);
                break;
            }

            pBranch.ChangeBnd().Add(theBnd);

            int iBest = 0;
            Span<bool> isOut = [pBranch.Child(0).Bnd().IsOut(theBnd), pBranch.Child(1).Bnd().IsOut(theBnd)];
            if (isOut[0] != isOut[1])
            {
                iBest = (isOut[0] ? 1 : 0);
            }
            else
            {
                TheBndType aUnion0 = theBnd, aUnion1 = theBnd;
                TheBndType aChild0 = pBranch.Child(0).Bnd(), aChild1 = pBranch.Child(1).Bnd();
                aUnion0.Add(aChild0);
                aUnion1.Add(aChild1);
                double d1 = aUnion0.SquareExtent();
                double d2 = aUnion1.SquareExtent();
                if (d1 > d2)
                {
                    iBest = 1;
                }
            }

            isOutOfBranch = isOut[iBest];
            pBranch = pBranch.ChangeChild(iBest);
        }
        return true;
    }

    /// <summary>Searches the tree for all objects conforming to the selector. Returns the number of objects accepted.</summary>
    public virtual int Select(Selector theSelector) => IsEmpty() ? 0 : Select(Root(), theSelector);

    /// <summary>Clears the contents of the tree.</summary>
    public virtual void Clear()
    {
        myRoot = null;
        myLastNode = null; // OCCT leaves it dangling
    }

    public bool IsEmpty() => myRoot is null;

    /// <summary>The root node of the tree.</summary>
    public TreeNode Root() => myRoot!;

    /// <summary>The last added node.</summary>
    protected TreeNode ChangeLastNode() => myLastNode!;

    /// <summary>Searches the branch for all objects conforming to the selector. Returns the number of objects accepted.</summary>
    protected int Select(TreeNode theBranch, Selector theSelector)
    {
        const int THE_INIT_STACK_SIZE = 64;
        TreeNode[] aStack = new TreeNode[THE_INIT_STACK_SIZE];
        int aTop = 0;
        int nSel = 0;

        aStack[aTop++] = theBranch;

        while (aTop > 0)
        {
            TreeNode aNode = aStack[--aTop];

            if (theSelector.Reject(aNode.Bnd()))
            {
                continue;
            }

            if (aNode.IsLeaf())
            {
                if (theSelector.Accept(aNode.Object()))
                {
                    nSel++;
                }
                if (theSelector.Stop())
                {
                    break;
                }
            }
            else
            {
                if (aTop + 2 > aStack.Length)
                {
                    Array.Resize(ref aStack, aStack.Length * 2);
                }
                aStack[aTop++] = aNode.Child(1);
                aStack[aTop++] = aNode.Child(0);
            }
        }
        return nSel;
    }
}
