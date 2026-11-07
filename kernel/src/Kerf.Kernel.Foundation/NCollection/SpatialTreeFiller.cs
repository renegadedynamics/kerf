// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_UBTreeFiller.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2002-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation.Std;
using System.Globalization;

namespace Kerf.Engine.Collections;

/// <summary>
/// Fills an <see cref="SpatialTree{TheObjType, TheBndType}"/> in random order, which gives a better balanced
/// tree than adding the objects in their natural order.
/// </summary>
/// <remarks>
/// The shuffle uses <see cref="Mt19937"/> with C++'s default seed, so the tree is OCCT's. OCCT's destructor fills
/// the tree with the objects not yet added; C# has no destructor, so the filler is <see cref="IDisposable"/> and
/// <see cref="Dispose"/> does it: ported code writes <c>using var aFiller = new ...</c> where the C++ relies on the
/// end of the scope.
/// </remarks>
internal sealed class SpatialTreeFiller<TheObjType, TheBndType> : IDisposable
    where TheBndType : struct, ISpatialTreeBox<TheBndType>
{
    /// <summary>An object with its bounding volume.</summary>
    public struct ObjBnd
    {
        public TheObjType myObj;
        public TheBndType myBnd;

        public ObjBnd(in TheObjType obj, in TheBndType bnd)
        {
            myObj = obj;
            myBnd = bnd;
        }

        public ObjBnd()
        {
            myObj = Default<TheObjType>.Value;
            myBnd = new TheBndType();
        }
    }

    private readonly SpatialTree<TheObjType, TheBndType> _tree;
    private readonly List<ObjBnd> _seqPtr;
    private DefaultMt19937 _randGen; // random number generator
    private readonly bool _isFullRandom;

    /// <summary>A filler of <paramref name="tree"/>.</summary>
    /// <param name="tree">the tree to fill.</param>
    /// <param name="isFullRandom">
    /// Takes effect when the number of items is large (order of 50,000). When true, the maximal randomisation gives a
    /// better balanced tree; when false, the tree is worse balanced but filled faster.
    /// </param>
    /// <param name="capacity">How many objects are to come, if that's known.</param>
    public SpatialTreeFiller(SpatialTree<TheObjType, TheBndType> tree, bool isFullRandom = true, int capacity = 0)
    {
        _tree = tree;
        _seqPtr = new List<ObjBnd>(capacity);
        _isFullRandom = isFullRandom;
    }

    /// <summary>Adds an object to the buffer of the filler.</summary>
    public void Add(in TheObjType obj, in TheBndType bnd)
    {
        _seqPtr.Add(new ObjBnd(obj, bnd));
    }

    /// <summary>
    /// Fills the tree with the buffered objects, in random order, and clears the buffer. Returns the number of objects
    /// added to the tree.
    /// </summary>
    public int Fill()
    {
        int i, nbAdd = _seqPtr.Count;
        if (_isFullRandom)
        {
            for (i = nbAdd; i > 0; i--)
            {
                if ((i & 1023) == 0)
                {
                    Cancellation.ThrowIfRequested();
                }

                int ind = (int)(_randGen.Invoke() % (uint)i);
                ObjBnd objBnd = _seqPtr[ind];
                _tree.Add(objBnd.myObj, objBnd.myBnd);
                _seqPtr[ind] = _seqPtr[i - 1];
            }
        }
        else
        {
            for (i = nbAdd; i > 0; i--)
            {
                if ((i & 1023) == 0)
                {
                    Cancellation.ThrowIfRequested();
                }

                int ind = i - (int)(_randGen.Invoke() % (uint)i) - 1;
                ObjBnd objBnd = _seqPtr[ind];
                _tree.Add(objBnd.myObj, objBnd.myBnd);
                _seqPtr[ind] = _seqPtr[i - 1];
            }
        }
        _seqPtr.Clear();
        return nbAdd;
    }

    /// <summary>Removes the buffered objects, so that they are not added to the tree.</summary>
    public void Reset() => _seqPtr.Clear();

    /// <summary>
    /// Writes the number of leaves of the filled tree and its balance to <paramref name="stream"/>. Returns the
    /// number of leaves.
    /// </summary>
    public int CheckTree(TextWriter stream)
    {
        int number = 0;
        double len = CheckNode(_tree.Root(), 0, ref number);
        double num = (double)number;
        double len1 = Math.Sqrt(len / num);
        double len0 = Math.Log(num) / Math.Log(2.0);
        // Sprintf(buf, "Checking UBTree:%8d leaves, balance =%7.2f", ...)
        stream.Write(string.Format(CultureInfo.InvariantCulture, "Checking UBTree:{0,8} leaves, balance ={1,7:F2}",
            number, len1 / len0));
        stream.Write('\n');
        return number;
    }

    /// <summary>Fills the tree with the objects not yet added (OCCT's destructor).</summary>
    public void Dispose()
    {
        if (_seqPtr.Count > 0)
        {
            Fill();
        }
    }

    private static double CheckNode(SpatialTree<TheObjType, TheBndType>.TreeNode node, int length,
        ref int number)
    {
        double aLength;
        if (!node.IsLeaf)
        {
            aLength = (CheckNode(node.Child(0), length + 1, ref number)
                       + CheckNode(node.Child(1), length + 1, ref number));
        }
        else
        {
            number++;
            aLength = length * length;
        }
        return aLength;
    }
}
