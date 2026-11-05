// Port of: src/FoundationClasses/TKernel/GTests/NCollection_UBTree_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// The file's NCollection_EBTree tests (its NCollection_UBTree tests are in NCollection_UBTreeTests.cs), and additions.

using Kerf.Engine.Collections;

namespace Kerf.Kernel.Foundation.Tests;

public class NCollection_EBTreeTest
{
    [Fact]
    public void AddContainsRemove()
    {
        IndexedSpatialTree<int, TestBnd1D> aTree = new();

        Assert.True(aTree.IsEmpty);

        aTree.Add(1, new TestBnd1D(0.0, 2.0));
        aTree.Add(2, new TestBnd1D(3.0, 5.0));
        aTree.Add(3, new TestBnd1D(4.0, 6.0));

        Assert.True(aTree.Contains(1));
        Assert.True(aTree.Contains(2));
        Assert.True(aTree.Contains(3));
        Assert.False(aTree.Contains(4));

        // Duplicate add should return false
        Assert.False(aTree.Add(1, new TestBnd1D(0.0, 2.0)));

        // Remove object 2
        Assert.True(aTree.Remove(2));
        Assert.False(aTree.Contains(2));
        Assert.True(aTree.Contains(1));
        Assert.True(aTree.Contains(3));

        // Remove non-existent
        Assert.False(aTree.Remove(4));
    }

    [Fact]
    public void RemoveRoot()
    {
        IndexedSpatialTree<int, TestBnd1D> aTree = new();

        aTree.Add(1, new TestBnd1D(0.0, 5.0));
        Assert.True(aTree.Contains(1));

        // Removing the only element (root) should clear the tree
        Assert.True(aTree.Remove(1));
        Assert.True(aTree.IsEmpty);
    }

    [Fact]
    public void SelectAfterRemove()
    {
        IndexedSpatialTree<int, TestBnd1D> aTree = new();

        aTree.Add(1, new TestBnd1D(0.0, 2.0));
        aTree.Add(2, new TestBnd1D(3.0, 5.0));
        aTree.Add(3, new TestBnd1D(4.0, 6.0));

        // Remove object 2
        aTree.Remove(2);

        // Select overlapping [3.5, 5.5] -- only object 3 should remain
        TestSelector1D aSelector = new(new TestBnd1D(3.5, 5.5));
        int nSel = aTree.Select(aSelector);
        Assert.Equal(1, nSel);
        Assert.Single(aSelector.Results());
        Assert.Equal(3, aSelector.Results()[0]);
    }

    [Fact]
    public void FindNode()
    {
        IndexedSpatialTree<int, TestBnd1D> aTree = new();

        aTree.Add(1, new TestBnd1D(0.0, 2.0));
        aTree.Add(2, new TestBnd1D(3.0, 5.0));

        SpatialTree<int, TestBnd1D>.TreeNode aNode = aTree.FindNode(1);
        Assert.True(aNode.IsLeaf);
        Assert.Equal(1, aNode.Object());
    }

    [Fact]
    public void RemoveAll()
    {
        IndexedSpatialTree<int, TestBnd1D> aTree = new();

        aTree.Add(1, new TestBnd1D(0.0, 2.0));
        aTree.Add(2, new TestBnd1D(3.0, 5.0));
        aTree.Add(3, new TestBnd1D(4.0, 6.0));

        Assert.True(aTree.Remove(1));
        Assert.True(aTree.Remove(2));
        Assert.True(aTree.Remove(3));
        Assert.True(aTree.IsEmpty);
    }

    [Fact]
    public void MoveConstructor()
    {
        IndexedSpatialTree<int, TestBnd1D> aSrc = new();
        aSrc.Add(1, new TestBnd1D(0.0, 2.0));
        aSrc.Add(2, new TestBnd1D(3.0, 5.0));
        aSrc.Add(3, new TestBnd1D(4.0, 6.0));

        IndexedSpatialTree<int, TestBnd1D> aDst = new(aSrc);

        // Source is empty
        Assert.True(aSrc.IsEmpty);
        Assert.False(aSrc.Contains(1));

        // Destination has all objects
        Assert.False(aDst.IsEmpty);
        Assert.True(aDst.Contains(1));
        Assert.True(aDst.Contains(2));
        Assert.True(aDst.Contains(3));

        TestSelector1D aSelector = new(new TestBnd1D(0.0, 10.0));
        int nSel = aDst.Select(aSelector);
        Assert.Equal(3, nSel);
    }

    [Fact]
    public void MoveAssignment()
    {
        IndexedSpatialTree<int, TestBnd1D> aSrc = new();
        aSrc.Add(1, new TestBnd1D(0.0, 2.0));
        aSrc.Add(2, new TestBnd1D(3.0, 5.0));

        IndexedSpatialTree<int, TestBnd1D> aDst = new();
        aDst.Add(10, new TestBnd1D(100.0, 200.0));

        aDst.Move(aSrc);

        Assert.True(aSrc.IsEmpty);
        Assert.False(aDst.IsEmpty);
        Assert.True(aDst.Contains(1));
        Assert.True(aDst.Contains(2));
        Assert.False(aDst.Contains(10));

        TestSelector1D aSelector = new(new TestBnd1D(0.0, 10.0));
        int nSel = aDst.Select(aSelector);
        Assert.Equal(2, nSel);
    }

    // Additions

    /// <summary>
    /// Random adds and removes against a brute-force list: the selection, each object's leaf (its own object and
    /// volume), and every branch's volume being the union of its children's.
    /// </summary>
    [Theory]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void RandomAddRemove_MatchesBruteForce(int theSeed)
    {
        Random aRandom = new(theSeed);
        IndexedSpatialTree<int, TestBnd1D> aTree = new();
        Dictionary<int, TestBnd1D> aReference = [];
        for (int anIter = 0; anIter < 2000; ++anIter)
        {
            int anObj = aRandom.Next(200);
            if (aRandom.NextDouble() < 0.6)
            {
                double aMin = aRandom.NextDouble() * 100.0;
                TestBnd1D aBnd = new(aMin, aMin + aRandom.NextDouble() * 5.0);
                bool isAdded = aTree.Add(anObj, aBnd);
                Assert.Equal(!aReference.ContainsKey(anObj), isAdded);
                aReference.TryAdd(anObj, aBnd);
            }
            else
            {
                Assert.Equal(aReference.Remove(anObj), aTree.Remove(anObj));
            }

            if (anIter % 50 == 0)
            {
                checkTree(aTree, aReference, aRandom);
            }
        }
        checkTree(aTree, aReference, aRandom);
    }

    private static void checkTree(IndexedSpatialTree<int, TestBnd1D> theTree, Dictionary<int, TestBnd1D> theReference,
                                  Random theRandom)
    {
        Assert.Equal(theReference.Count == 0, theTree.IsEmpty);
        foreach ((int anObj, TestBnd1D aBnd) in theReference)
        {
            Assert.True(theTree.Contains(anObj));
            SpatialTree<int, TestBnd1D>.TreeNode aNode = theTree.FindNode(anObj);
            Assert.True(aNode.IsLeaf);
            Assert.Equal(anObj, aNode.Object());
            Assert.Equal(aBnd.Min, aNode.Bnd().Min);
            Assert.Equal(aBnd.Max, aNode.Bnd().Max);
        }
        if (!theTree.IsEmpty)
        {
            Assert.Equal(theReference.Count, checkBranch(theTree.Root()));
        }

        double aMin = theRandom.NextDouble() * 100.0;
        TestBnd1D aQuery = new(aMin, aMin + 10.0);
        TestSelector1D aSelector = new(aQuery);
        theTree.Select(aSelector);
        List<int> anExpected = [.. theReference.Where(theItem => !aQuery.IsOut(theItem.Value)).Select(theItem => theItem.Key)];
        Assert.Equal(anExpected.Order(), aSelector.Results().Order());
    }

    // Checks that each branch's volume is the union of its children's; returns the number of leaves.
    private static int checkBranch(SpatialTree<int, TestBnd1D>.TreeNode theNode)
    {
        if (theNode.IsLeaf)
        {
            return 1;
        }
        Assert.Same(theNode, theNode.Child(0).Parent());
        Assert.Same(theNode, theNode.Child(1).Parent());
        TestBnd1D aUnion = theNode.Child(0).Bnd();
        aUnion.Add(theNode.Child(1).Bnd());
        Assert.Equal(aUnion.Min, theNode.Bnd().Min);
        Assert.Equal(aUnion.Max, theNode.Bnd().Max);
        return checkBranch(theNode.Child(0)) + checkBranch(theNode.Child(1));
    }
}
