// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_UBTreeFiller.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2002-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;

namespace Occt;

/// <summary>
/// Fills an <see cref="NCollection_UBTree{TheObjType, TheBndType}"/> in random order, which gives a better balanced
/// tree than adding the objects in their natural order.
/// </summary>
/// <remarks>
/// The shuffle uses <see cref="std_mt19937"/> with C++'s default seed, so the tree is OCCT's. OCCT's destructor fills
/// the tree with the objects not yet added; C# has no destructor, so the filler is <see cref="IDisposable"/> and
/// <see cref="Dispose"/> does it: ported code writes <c>using var aFiller = new ...</c> where the C++ relies on the
/// end of the scope.
/// </remarks>
public sealed class NCollection_UBTreeFiller<TheObjType, TheBndType> : IDisposable
    where TheBndType : struct, INCollection_UBTreeBnd<TheBndType>
{
    /// <summary>An object with its bounding volume.</summary>
    public struct ObjBnd
    {
        public TheObjType myObj;
        public TheBndType myBnd;

        public ObjBnd(in TheObjType theObj, in TheBndType theBnd)
        {
            myObj = theObj;
            myBnd = theBnd;
        }

        public ObjBnd()
        {
            myObj = NCollection_Default<TheObjType>.Value;
            myBnd = new TheBndType();
        }
    }

    private readonly NCollection_UBTree<TheObjType, TheBndType> myTree;
    private readonly NCollection_DynamicArray<ObjBnd> mySeqPtr;
    private readonly std_mt19937 myRandGen; // random number generator
    private readonly bool myIsFullRandom;

    /// <summary>A filler of <paramref name="theTree"/>.</summary>
    /// <param name="theTree">the tree to fill.</param>
    /// <param name="isFullRandom">
    /// Takes effect when the number of items is large (order of 50,000). When true, the maximal randomisation gives a
    /// better balanced tree; when false, the tree is worse balanced but filled faster.
    /// </param>
    public NCollection_UBTreeFiller(NCollection_UBTree<TheObjType, TheBndType> theTree, bool isFullRandom = true)
    {
        myTree = theTree;
        mySeqPtr = new NCollection_DynamicArray<ObjBnd>(256);
        myRandGen = new std_mt19937(5489u /* == std::mt19937::default_seed */);
        myIsFullRandom = isFullRandom;
    }

    /// <summary>Adds an object to the buffer of the filler.</summary>
    public void Add(in TheObjType theObj, in TheBndType theBnd)
    {
        mySeqPtr.Append(new ObjBnd(theObj, theBnd));
    }

    /// <summary>
    /// Fills the tree with the buffered objects, in random order, and clears the buffer. Returns the number of objects
    /// added to the tree.
    /// </summary>
    public int Fill()
    {
        int i, nbAdd = mySeqPtr.Length();
        if (myIsFullRandom)
        {
            for (i = nbAdd; i > 0; i--)
            {
                int ind = (int)(myRandGen.Invoke() % (uint)i);
                ObjBnd aObjBnd = mySeqPtr[ind];
                myTree.Add(aObjBnd.myObj, aObjBnd.myBnd);
                mySeqPtr[ind] = mySeqPtr[i - 1];
            }
        }
        else
        {
            for (i = nbAdd; i > 0; i--)
            {
                int ind = i - (int)(myRandGen.Invoke() % (uint)i) - 1;
                ObjBnd aObjBnd = mySeqPtr[ind];
                myTree.Add(aObjBnd.myObj, aObjBnd.myBnd);
                mySeqPtr[ind] = mySeqPtr[i - 1];
            }
        }
        mySeqPtr.Clear();
        return nbAdd;
    }

    /// <summary>Removes the buffered objects, so that they are not added to the tree.</summary>
    public void Reset() => mySeqPtr.Clear();

    /// <summary>
    /// Writes the number of leaves of the filled tree and its balance to <paramref name="theStream"/>. Returns the
    /// number of leaves.
    /// </summary>
    public int CheckTree(TextWriter theStream)
    {
        int aNumber = 0;
        double aLen = checkNode(myTree.Root(), 0, ref aNumber);
        double num = (double)aNumber;
        double aLen1 = Math.Sqrt(aLen / num);
        double aLen0 = Math.Log(num) / Math.Log(2.0);
        // Sprintf(buf, "Checking UBTree:%8d leaves, balance =%7.2f", ...)
        theStream.Write(string.Format(CultureInfo.InvariantCulture, "Checking UBTree:{0,8} leaves, balance ={1,7:F2}",
            aNumber, aLen1 / aLen0));
        theStream.Write('\n');
        return aNumber;
    }

    /// <summary>Fills the tree with the objects not yet added (OCCT's destructor).</summary>
    public void Dispose()
    {
        if (mySeqPtr.Length() > 0)
        {
            Fill();
        }
    }

    private static double checkNode(NCollection_UBTree<TheObjType, TheBndType>.TreeNode theNode, int theLength,
        ref int theNumber)
    {
        double aLength;
        if (!theNode.IsLeaf())
        {
            aLength = (checkNode(theNode.Child(0), theLength + 1, ref theNumber)
                       + checkNode(theNode.Child(1), theLength + 1, ref theNumber));
        }
        else
        {
            theNumber++;
            aLength = theLength * theLength;
        }
        return aLength;
    }
}
