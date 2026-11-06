// Port of: src/FoundationClasses/TKMath/BVH/BVH_Tree.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Numerics;

namespace Kerf.Engine.Geometry.Bvh;

/// <summary>A non-generic base for <see cref="TreeBase{T, TVec}"/> (just to have a named base class).</summary>
/// <remarks>DumpJson and DumpNode are not ported.</remarks>
internal abstract class TreeBaseTransient : Transient
{
    protected TreeBaseTransient()
    {
    }
}

/// <summary>
/// The parameters of a bounding volume hierarchy (BVH): a tree of nodes, each with the axis-aligned bounding box of
/// the objects below it, serialized into arrays.
/// </summary>
/// <remarks>
/// Each node's data record is a vector of four integers: x is 1 for a leaf (outer) node and 0 for an inner node; for
/// a leaf, y and z are the first and last primitive; for an inner node of a binary tree, y and z are the children;
/// w is the level. As in OCCT, the node accessors return references, so builders write through them.
/// </remarks>
internal class TreeBase<T, TVec> : TreeBaseTransient
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    /// <summary>Array of node data records.</summary>
    internal NCollection_LinearVector<NCollection_Vec4<int>> _nodeInfoBuffer = new();

    /// <summary>Array of node minimum points.</summary>
    internal NCollection_LinearVector<TVec> _minPointBuffer = new();

    /// <summary>Array of node maximum points.</summary>
    internal NCollection_LinearVector<TVec> _maxPointBuffer = new();

    /// <summary>Current depth of BVH tree (set by builder).</summary>
    internal int _depth;

    /// <summary>Creates a new empty BVH tree.</summary>
    public TreeBase()
    {
        _depth = 0;
    }

    /// <summary>The depth (height) of the tree.</summary>
    public int Depth() => _depth;

    /// <summary>The total number of nodes.</summary>
    public int Length() => _nodeInfoBuffer.Size();

    /// <summary>The minimum point of the given node.</summary>
    public ref TVec MinPoint(int nodeIndex) => ref _minPointBuffer.ChangeValue(nodeIndex);

    /// <summary>The maximum point of the given node.</summary>
    public ref TVec MaxPoint(int nodeIndex) => ref _maxPointBuffer.ChangeValue(nodeIndex);

    /// <summary>The index of the first primitive of the given leaf node.</summary>
    public ref int BegPrimitive(int nodeIndex) => ref NodeInfo(nodeIndex, 1);

    /// <summary>The index of the last primitive of the given leaf node.</summary>
    public ref int EndPrimitive(int nodeIndex) => ref NodeInfo(nodeIndex, 2);

    /// <summary>The number of primitives in the given leaf node.</summary>
    public int NbPrimitives(int nodeIndex) => EndPrimitive(nodeIndex) - BegPrimitive(nodeIndex) + 1;

    /// <summary>The level (depth) of the given node.</summary>
    public ref int Level(int nodeIndex) => ref NodeInfo(nodeIndex, 3);

    /// <summary>True if the given node is outer (a leaf).</summary>
    public bool IsOuter(int nodeIndex) => _nodeInfoBuffer.Value(nodeIndex).X() != 0;

    /// <summary>The array of node data records.</summary>
    public NCollection_LinearVector<NCollection_Vec4<int>> NodeInfoBuffer() => _nodeInfoBuffer;

    /// <summary>The array of node minimum points.</summary>
    public NCollection_LinearVector<TVec> MinPointBuffer() => _minPointBuffer;

    /// <summary>The array of node maximum points.</summary>
    public NCollection_LinearVector<TVec> MaxPointBuffer() => _maxPointBuffer;

    /// <summary>A component of a node's data record (OCCT's <c>ChangeValue(i).y()</c> and the like).</summary>
    internal ref int NodeInfo(int nodeIndex, int component)
        => ref NCollection_VecTools.Change<int, NCollection_Vec4<int>>(ref _nodeInfoBuffer.ChangeValue(nodeIndex), component);
}
