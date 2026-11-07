// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_UBTree.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2002-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Collections;

/// <summary>
/// What <see cref="SpatialTree{TheObjType, TheBndType}"/> requires of its bounding type: OCCT's template
/// parameter <c>TheBndType</c>, which must have <c>Add</c>, <c>IsOut</c> and <c>SquareExtent</c>.
/// </summary>
/// <remarks>
/// The bounding types are value types whose <c>Add</c> is an internal mutator; they implement <see cref="Add"/>
/// explicitly. Called through the interface on a boxed copy it changes only that copy; the tree calls it on its own
/// nodes' values through the constraint, which changes them in place as C++ does.
/// </remarks>
internal interface ISpatialTreeBox<TheBndType> where TheBndType : struct, ISpatialTreeBox<TheBndType>
{
    /// <summary>Enlarges this bounding volume to contain <paramref name="other"/>.</summary>
    void Add(in TheBndType other);

    /// <summary>True if this bounding volume and <paramref name="other"/> don't intersect.</summary>
    bool IsOut(in TheBndType other);

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
internal class SpatialTree<TheObjType, TheBndType> where TheBndType : struct, ISpatialTreeBox<TheBndType>
{
    /// <summary>The minimal interface of a selector.</summary>
    public abstract class Selector
    {
        /// <summary>
        /// Set by <see cref="Accept"/> when the selection process is to be stopped.
        /// </summary>
        protected bool _stop;

        protected Selector()
        {
            _stop = false;
        }

        /// <summary>True if the bounding volume does not conform to the selection conditions.</summary>
        /// <remarks>Const in OCCT; OCCT's own selectors cast that away, so it isn't readonly here.</remarks>
        public abstract bool Reject(in TheBndType bnd);

        /// <summary>
        /// Confirms the object, called when its bounding volume conforms to the conditions (see <see cref="Reject"/>);
        /// keeps a record of accepted objects. True if the object is accepted.
        /// </summary>
        public abstract bool Accept(in TheObjType obj);

        /// <summary>Checked after each call to Accept(): true stops the selection.</summary>
        public bool Stop() => _stop;
    }

    /// <summary>
    /// A node of the tree. Initially the tree is one leaf; a node grows to a branch holding two children: one with the
    /// node's previous content, the other with a new object and its bounding volume.
    /// </summary>
    public sealed class TreeNode
    {
        private TheBndType _bnd; // bounding geometry
        private TheObjType _object; // the object
        private TreeNode[]? _children; // 2 children forming a b-tree
        private TreeNode? _parent; // the parent node

        internal TreeNode(in TheObjType obj, in TheBndType bnd)
        {
            _bnd = bnd;
            _object = obj;
            _children = null;
            _parent = null;
        }

        private TreeNode()
        {
            _bnd = new TheBndType();
            _object = Default<TheObjType>.Value;
            _children = null;
            _parent = null;
        }

        public bool IsLeaf => _children is null;

        public bool IsRoot => _parent is null;

        public TheBndType Bnd() => _bnd;

        internal ref TheBndType ChangeBnd() => ref _bnd;

        public TheObjType Object() => _object;

        public TreeNode Child(int i) => _children![i];

        internal TreeNode ChangeChild(int i) => _children![i];

        public TreeNode Parent() => _parent!;

        internal TreeNode ChangeParent() => _parent!;

        /// <summary>
        /// Makes this node a branch holding its previous content in the first child and theObj in the second.
        /// </summary>
        /// <param name="newBnd">new bounding volume comprising both child nodes.</param>
        /// <param name="obj">added object.</param>
        /// <param name="bnd">bounding volume of theObj.</param>
        internal void Gemmate(in TheBndType newBnd, in TheObjType obj, in TheBndType bnd)
        {
            TreeNode[] children = [new TreeNode(), new TreeNode()];
            // children[0] = *this
            children[0]._bnd = _bnd;
            children[0]._object = _object;
            children[0]._children = _children;
            children[0]._parent = _parent;
            children[1]._object = obj;
            children[1]._bnd = bnd;
            children[0]._parent = children[1]._parent = this;
            if (!IsLeaf)
            {
                _children![0]._parent = children[0];
                _children[1]._parent = children[0];
            }
            _children = children;
            _bnd = newBnd;
            _object = Default<TheObjType>.Value; // nullify myObject
        }

        /// <summary>Kills the i-th child; this node takes the content of the other child.</summary>
        internal void Kill(int i)
        {
            if (!IsLeaf)
            {
                TreeNode[] oldChildren = _children!;
                int iopp = 1 - i;
                _bnd = oldChildren[iopp]._bnd;
                _object = oldChildren[iopp]._object;
                _children = oldChildren[iopp]._children;
                if (!IsLeaf)
                {
                    _children![0]._parent = this;
                    _children[1]._parent = this;
                }
                // OCCT destroys the other child and frees the killed branch: the garbage collector does
            }
        }
    }

    private TreeNode? _root; // root of the tree
    private TreeNode? _lastNode; // the last added node

    /// <summary>An empty tree.</summary>
    public SpatialTree()
    {
        _root = null;
        _lastNode = null;
    }

    /// <summary>Takes the nodes of <paramref name="other"/>, which is left empty (C++'s move constructor).</summary>
    public SpatialTree(SpatialTree<TheObjType, TheBndType> other)
    {
        _root = other._root;
        _lastNode = other._lastNode;
        other._root = null;
        other._lastNode = null;
    }

    /// <summary>Clears this tree and takes the nodes of <paramref name="other"/>, which is left empty (C++'s move assignment).</summary>
    public SpatialTree<TheObjType, TheBndType> Move(SpatialTree<TheObjType, TheBndType> other)
    {
        if (!ReferenceEquals(this, other))
        {
            Clear();
            _root = other._root;
            _lastNode = other._lastNode;
            other._root = null;
            other._lastNode = null;
        }
        return this;
    }

    /// <summary>Adds an object with its bounding volume. Always true.</summary>
    public virtual bool Add(in TheObjType obj, in TheBndType bnd)
    {
        if (IsEmpty)
        {
            _root = new TreeNode(obj, bnd);
            _lastNode = _root;
            return true;
        }

        TreeNode pBranch = _root!;
        bool isOutOfBranch = pBranch.Bnd().IsOut(bnd);

        for (;;)
        {
            if (isOutOfBranch || pBranch.IsLeaf)
            {
                TheBndType newBnd = bnd;
                TheBndType branchBnd = pBranch.Bnd();
                newBnd.Add(branchBnd);
                pBranch.Gemmate(newBnd, obj, bnd);
                _lastNode = pBranch.ChangeChild(1);
                break;
            }

            pBranch.ChangeBnd().Add(bnd);

            int iBest = 0;
            Span<bool> isOut = [pBranch.Child(0).Bnd().IsOut(bnd), pBranch.Child(1).Bnd().IsOut(bnd)];
            if (isOut[0] != isOut[1])
            {
                iBest = (isOut[0] ? 1 : 0);
            }
            else
            {
                TheBndType union0 = bnd, union1 = bnd;
                TheBndType child0 = pBranch.Child(0).Bnd(), child1 = pBranch.Child(1).Bnd();
                union0.Add(child0);
                union1.Add(child1);
                double d1 = union0.SquareExtent();
                double d2 = union1.SquareExtent();
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
    public virtual int Select(Selector selector) => IsEmpty ? 0 : Select(Root(), selector);

    /// <summary>Clears the contents of the tree.</summary>
    public virtual void Clear()
    {
        _root = null;
        _lastNode = null; // OCCT leaves it dangling
    }

    public bool IsEmpty => _root is null;

    /// <summary>The root node of the tree.</summary>
    public TreeNode Root() => _root!;

    /// <summary>The last added node.</summary>
    protected TreeNode ChangeLastNode() => _lastNode!;

    /// <summary>Searches the branch for all objects conforming to the selector. Returns the number of objects accepted.</summary>
    protected int Select(TreeNode branch, Selector selector)
    {
        // On the stack: a mesher's check selects once for each segment (11Q, decision 1874)
        InitialStack initial = default;
        Span<TreeNode> stack = initial;
        int top = 0;
        int nSel = 0;

        stack[top++] = branch;

        while (top > 0)
        {
            TreeNode node = stack[--top];

            if (selector.Reject(node.Bnd()))
            {
                continue;
            }

            if (node.IsLeaf)
            {
                if (selector.Accept(node.Object()))
                {
                    nSel++;
                }
                if (selector.Stop())
                {
                    break;
                }
            }
            else
            {
                if (top + 2 > stack.Length)
                {
                    var larger = new TreeNode[stack.Length * 2];
                    stack.CopyTo(larger);
                    stack = larger;
                }
                stack[top++] = node.Child(1);
                stack[top++] = node.Child(0);
            }
        }
        return nSel;
    }

    [System.Runtime.CompilerServices.InlineArray(64)]
    private struct InitialStack
    {
        private TreeNode _first;
    }
}
