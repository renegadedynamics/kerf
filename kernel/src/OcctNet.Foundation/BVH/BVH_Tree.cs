// Port of: src/FoundationClasses/TKMath/BVH/BVH_Tree.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Numerics;

namespace Occt;

/// <summary>A non-generic base for <see cref="BVH_TreeBase{T, TVec}"/> (just to have a named base class).</summary>
/// <remarks>DumpJson and DumpNode are not ported.</remarks>
public abstract class BVH_TreeBaseTransient : Standard_Transient
{
    protected BVH_TreeBaseTransient()
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
public class BVH_TreeBase<T, TVec> : BVH_TreeBaseTransient
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    /// <summary>Array of node data records.</summary>
    internal NCollection_LinearVector<NCollection_Vec4<int>> myNodeInfoBuffer = new();

    /// <summary>Array of node minimum points.</summary>
    internal NCollection_LinearVector<TVec> myMinPointBuffer = new();

    /// <summary>Array of node maximum points.</summary>
    internal NCollection_LinearVector<TVec> myMaxPointBuffer = new();

    /// <summary>Current depth of BVH tree (set by builder).</summary>
    internal int myDepth;

    /// <summary>Creates a new empty BVH tree.</summary>
    public BVH_TreeBase()
    {
        myDepth = 0;
    }

    /// <summary>The depth (height) of the tree.</summary>
    public int Depth() => myDepth;

    /// <summary>The total number of nodes.</summary>
    public int Length() => myNodeInfoBuffer.Size();

    /// <summary>The minimum point of the given node.</summary>
    public ref TVec MinPoint(int theNodeIndex) => ref myMinPointBuffer.ChangeValue(theNodeIndex);

    /// <summary>The maximum point of the given node.</summary>
    public ref TVec MaxPoint(int theNodeIndex) => ref myMaxPointBuffer.ChangeValue(theNodeIndex);

    /// <summary>The index of the first primitive of the given leaf node.</summary>
    public ref int BegPrimitive(int theNodeIndex) => ref NodeInfo(theNodeIndex, 1);

    /// <summary>The index of the last primitive of the given leaf node.</summary>
    public ref int EndPrimitive(int theNodeIndex) => ref NodeInfo(theNodeIndex, 2);

    /// <summary>The number of primitives in the given leaf node.</summary>
    public int NbPrimitives(int theNodeIndex) => EndPrimitive(theNodeIndex) - BegPrimitive(theNodeIndex) + 1;

    /// <summary>The level (depth) of the given node.</summary>
    public ref int Level(int theNodeIndex) => ref NodeInfo(theNodeIndex, 3);

    /// <summary>True if the given node is outer (a leaf).</summary>
    public bool IsOuter(int theNodeIndex) => myNodeInfoBuffer.Value(theNodeIndex).x() != 0;

    /// <summary>The array of node data records.</summary>
    public NCollection_LinearVector<NCollection_Vec4<int>> NodeInfoBuffer() => myNodeInfoBuffer;

    /// <summary>The array of node minimum points.</summary>
    public NCollection_LinearVector<TVec> MinPointBuffer() => myMinPointBuffer;

    /// <summary>The array of node maximum points.</summary>
    public NCollection_LinearVector<TVec> MaxPointBuffer() => myMaxPointBuffer;

    /// <summary>A component of a node's data record (OCCT's <c>ChangeValue(i).y()</c> and the like).</summary>
    internal ref int NodeInfo(int theNodeIndex, int theComponent)
        => ref NCollection_VecTools.Change<int, NCollection_Vec4<int>>(ref myNodeInfoBuffer.ChangeValue(theNodeIndex), theComponent);
}
