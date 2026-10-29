// Port of: src/FoundationClasses/TKernel/GTests/NCollection_UBTree_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// The file's NCollection_EBTree tests are not ported: NCollection_EBTree (UBTree with removal) is not ported yet.

namespace OcctNet.Foundation.Tests;

/// <summary>Simple 1D interval bounding type for testing: Add, IsOut, SquareExtent.</summary>
internal struct TestBnd1D : INCollection_UBTreeBnd<TestBnd1D>
{
    public double Min;
    public double Max;

    public TestBnd1D()
    {
        Min = 0.0;
        Max = 0.0;
    }

    public TestBnd1D(double theMin, double theMax)
    {
        Min = theMin;
        Max = theMax;
    }

    public void Add(in TestBnd1D theOther)
    {
        if (theOther.Min < Min)
        {
            Min = theOther.Min;
        }
        if (theOther.Max > Max)
        {
            Max = theOther.Max;
        }
    }

    public readonly bool IsOut(in TestBnd1D theOther) => theOther.Min > Max || theOther.Max < Min;

    public readonly double SquareExtent()
    {
        double d = Max - Min;
        return d * d;
    }
}

/// <summary>Selector that collects all objects overlapping a given query interval.</summary>
internal sealed class TestSelector1D(TestBnd1D theQuery) : NCollection_UBTree<int, TestBnd1D>.Selector
{
    private readonly List<int> myResults = [];

    public override bool Reject(in TestBnd1D theBnd) => theQuery.IsOut(theBnd);

    public override bool Accept(in int theObj)
    {
        myResults.Add(theObj);
        return true;
    }

    public List<int> Results() => myResults;
}

/// <summary>Selector that stops after accepting a given number of objects.</summary>
internal sealed class TestStopSelector1D(TestBnd1D theQuery, int theMaxAccept) : NCollection_UBTree<int, TestBnd1D>.Selector
{
    private int myAccepted;
    private readonly List<int> myResults = [];

    public override bool Reject(in TestBnd1D theBnd) => theQuery.IsOut(theBnd);

    public override bool Accept(in int theObj)
    {
        myResults.Add(theObj);
        myAccepted++;
        if (myAccepted >= theMaxAccept)
        {
            myStop = true;
        }
        return true;
    }

    public List<int> Results() => myResults;
}

public class NCollection_UBTreeTest
{
    [Fact]
    public void EmptyTree()
    {
        NCollection_UBTree<int, TestBnd1D> aTree = new();

        Assert.True(aTree.IsEmpty());

        TestSelector1D aSelector = new(new TestBnd1D(0.0, 10.0));
        int nSel = aTree.Select(aSelector);
        Assert.Equal(0, nSel);
    }

    [Fact]
    public void AddAndSelect_SingleObject()
    {
        NCollection_UBTree<int, TestBnd1D> aTree = new();

        aTree.Add(1, new TestBnd1D(0.0, 5.0));
        Assert.False(aTree.IsEmpty());

        // Query overlapping
        TestSelector1D aSelector = new(new TestBnd1D(3.0, 7.0));
        int nSel = aTree.Select(aSelector);
        Assert.Equal(1, nSel);
        Assert.Single(aSelector.Results());
        Assert.Equal(1, aSelector.Results()[0]);

        // Query not overlapping
        TestSelector1D aSelector2 = new(new TestBnd1D(6.0, 10.0));
        int nSel2 = aTree.Select(aSelector2);
        Assert.Equal(0, nSel2);
    }

    [Fact]
    public void AddAndSelect_MultipleObjects()
    {
        NCollection_UBTree<int, TestBnd1D> aTree = new();

        // Add intervals: [0,2], [3,5], [4,6], [8,10]
        aTree.Add(1, new TestBnd1D(0.0, 2.0));
        aTree.Add(2, new TestBnd1D(3.0, 5.0));
        aTree.Add(3, new TestBnd1D(4.0, 6.0));
        aTree.Add(4, new TestBnd1D(8.0, 10.0));

        // Query [3.5, 5.5] should overlap with objects 2 and 3
        TestSelector1D aSelector = new(new TestBnd1D(3.5, 5.5));
        int nSel = aTree.Select(aSelector);
        Assert.Equal(2, nSel);

        List<int> aResults = [.. aSelector.Results()];
        aResults.Sort();
        Assert.Equal(2, aResults.Count);
        Assert.Equal(2, aResults[0]);
        Assert.Equal(3, aResults[1]);

        // Query [0, 10] should overlap all
        TestSelector1D aSelector2 = new(new TestBnd1D(0.0, 10.0));
        int nSel2 = aTree.Select(aSelector2);
        Assert.Equal(4, nSel2);
    }

    [Fact]
    public void StopSelector()
    {
        NCollection_UBTree<int, TestBnd1D> aTree = new();

        for (int i = 0; i < 10; ++i)
        {
            aTree.Add(i, new TestBnd1D(i, i + 1));
        }

        // Query overlaps all, but stop after 3
        TestStopSelector1D aSelector = new(new TestBnd1D(0.0, 11.0), 3);
        int nSel = aTree.Select(aSelector);
        Assert.Equal(3, nSel);
        Assert.Equal(3, aSelector.Results().Count);
    }

    [Fact]
    public void Clear()
    {
        NCollection_UBTree<int, TestBnd1D> aTree = new();

        aTree.Add(1, new TestBnd1D(0.0, 5.0));
        aTree.Add(2, new TestBnd1D(3.0, 7.0));

        Assert.False(aTree.IsEmpty());

        aTree.Clear();
        Assert.True(aTree.IsEmpty());

        TestSelector1D aSelector = new(new TestBnd1D(0.0, 10.0));
        Assert.Equal(0, aTree.Select(aSelector));
    }

    [Fact]
    public void ManyObjects()
    {
        NCollection_UBTree<int, TestBnd1D> aTree = new();

        const int N = 100;
        for (int i = 0; i < N; ++i)
        {
            aTree.Add(i, new TestBnd1D(i, i + 1));
        }

        // Query that overlaps all
        TestSelector1D aSelector = new(new TestBnd1D(0.0, N + 1));
        int nSel = aTree.Select(aSelector);
        Assert.Equal(N, nSel);
    }

    [Fact]
    public void DeepUnbalancedTree()
    {
        // Insert sorted non-overlapping intervals to create a deeply unbalanced tree.
        // Each new interval is "out" of all previous bounds, forcing the tree
        // to grow on one side -- producing depth ~ N.
        NCollection_UBTree<int, TestBnd1D> aTree = new();

        const int N = 200; // well over the initial stack size of 64
        for (int i = 0; i < N; ++i)
        {
            double aLo = i * 10.0;
            aTree.Add(i, new TestBnd1D(aLo, aLo + 1.0));
        }

        // Select all -- should find all N objects despite deep recursion
        TestSelector1D aSelector = new(new TestBnd1D(0.0, N * 10.0));
        int nSel = aTree.Select(aSelector);
        Assert.Equal(N, nSel);
    }

    [Fact]
    public void MoveConstructor()
    {
        NCollection_UBTree<int, TestBnd1D> aSrc = new();
        aSrc.Add(1, new TestBnd1D(0.0, 2.0));
        aSrc.Add(2, new TestBnd1D(3.0, 5.0));
        aSrc.Add(3, new TestBnd1D(4.0, 6.0));

        NCollection_UBTree<int, TestBnd1D> aDst = new(aSrc);

        // Source is empty
        Assert.True(aSrc.IsEmpty());

        // Destination has all objects
        Assert.False(aDst.IsEmpty());
        TestSelector1D aSelector = new(new TestBnd1D(0.0, 10.0));
        int nSel = aDst.Select(aSelector);
        Assert.Equal(3, nSel);
    }

    [Fact]
    public void MoveAssignment()
    {
        NCollection_UBTree<int, TestBnd1D> aSrc = new();
        aSrc.Add(1, new TestBnd1D(0.0, 2.0));
        aSrc.Add(2, new TestBnd1D(3.0, 5.0));

        NCollection_UBTree<int, TestBnd1D> aDst = new();
        aDst.Add(10, new TestBnd1D(100.0, 200.0));

        aDst.Move(aSrc);

        Assert.True(aSrc.IsEmpty());
        Assert.False(aDst.IsEmpty());

        TestSelector1D aSelector = new(new TestBnd1D(0.0, 10.0));
        int nSel = aDst.Select(aSelector);
        Assert.Equal(2, nSel);
    }

    // Additions: the filler, the tree's shape and std_mt19937

    [Fact]
    public void Filler_AddsAllObjectsInRandomOrder()
    {
        NCollection_UBTree<int, TestBnd1D> aTree = new();
        using (NCollection_UBTreeFiller<int, TestBnd1D> aFiller = new(aTree))
        {
            for (int i = 0; i < 1000; ++i)
            {
                aFiller.Add(i, new TestBnd1D(i, i + 0.5));
            }
            Assert.Equal(1000, aFiller.Fill());
            Assert.Equal(0, aFiller.Fill());
            StringWriter aStream = new();
            Assert.Equal(1000, aFiller.CheckTree(aStream));
            Assert.StartsWith("Checking UBTree:    1000 leaves, balance =", aStream.ToString(), StringComparison.Ordinal);
        }
        TestSelector1D aSelector = new(new TestBnd1D(10.2, 20.1));
        Assert.Equal(11, aTree.Select(aSelector));

        // Dispose fills the tree with what Fill has not added, as OCCT's destructor
        NCollection_UBTree<int, TestBnd1D> aTree2 = new();
        using (NCollection_UBTreeFiller<int, TestBnd1D> aFiller = new(aTree2, false))
        {
            aFiller.Add(1, new TestBnd1D(0, 1));
            aFiller.Add(2, new TestBnd1D(2, 3));
        }
        Assert.Equal(2, aTree2.Select(new TestSelector1D(new TestBnd1D(-1, 4))));
    }

    [Fact]
    public void Gemmate_KeepsParentLinks()
    {
        NCollection_UBTree<int, TestBnd1D> aTree = new();
        for (int i = 0; i < 20; ++i)
        {
            aTree.Add(i, new TestBnd1D(i % 7, i % 7 + 2));
        }
        int aLeaves = 0;
        Stack<NCollection_UBTree<int, TestBnd1D>.TreeNode> aStack = new();
        aStack.Push(aTree.Root());
        Assert.True(aTree.Root().IsRoot());
        while (aStack.Count > 0)
        {
            var aNode = aStack.Pop();
            if (aNode.IsLeaf())
            {
                aLeaves++;
                continue;
            }
            for (int c = 0; c < 2; c++)
            {
                Assert.Same(aNode, aNode.Child(c).Parent());
                Assert.False(aNode.Bnd().IsOut(aNode.Child(c).Bnd()));
                aStack.Push(aNode.Child(c));
            }
        }
        Assert.Equal(20, aLeaves);
    }

    [Fact]
    public void Mt19937_MatchesTheStandardSequence()
    {
        // The C++ standard requires the 10000th value of a default-constructed std::mt19937 to be 4123659995
        std_mt19937 aGen = new();
        uint aValue = 0;
        for (int i = 0; i < 10000; ++i)
        {
            aValue = aGen.Invoke();
        }
        Assert.Equal(4123659995u, aValue);
    }
}
