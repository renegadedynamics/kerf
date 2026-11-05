// Port of: src/FoundationClasses/TKernel/GTests/NCollection_Map_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Only the file's test of NCollection_MapAlgo (OCC24271_BooleanOperations) is ported here, and additions.

namespace Kerf.Kernel.Foundation.Tests;

public class NCollection_MapTest
{
    [Fact]
    public void OCC24271_BooleanOperations()
    {
        const int aLeftLower = 1;
        const int aLeftUpper = 10;
        const int aRightLower = 5;
        const int aRightUpper = 15;

        HashSet<int> aMapLeft = new();
        for (int aKeyIter = aLeftLower; aKeyIter <= aLeftUpper; ++aKeyIter)
        {
            aMapLeft.Add(aKeyIter);
        }

        HashSet<int> aMapRight = new();
        for (int aKeyIter = aRightLower; aKeyIter <= aRightUpper; ++aKeyIter)
        {
            aMapRight.Add(aKeyIter);
        }

        Assert.False(NCollection_MapAlgo.Contains(aMapLeft, aMapRight));
        Assert.False(NCollection_MapAlgo.Contains(aMapRight, aMapLeft));

        HashSet<int> aMapUnion = new();
        NCollection_MapAlgo.Union(aMapUnion, aMapLeft, aMapRight);
        Assert.Equal(aRightUpper - aLeftLower + 1, aMapUnion.Count);
        for (int aKeyIter = aLeftLower; aKeyIter <= aRightUpper; ++aKeyIter)
        {
            Assert.Contains(aKeyIter, aMapUnion);
        }

        HashSet<int> aMapSect = new();
        NCollection_MapAlgo.Intersection(aMapSect, aMapLeft, aMapRight);
        Assert.Equal(aLeftUpper - aRightLower + 1, aMapSect.Count);
        for (int aKeyIter = aRightLower; aKeyIter <= aLeftUpper; ++aKeyIter)
        {
            Assert.Contains(aKeyIter, aMapSect);
        }
        Assert.True(NCollection_MapAlgo.Contains(aMapLeft, aMapSect));
        Assert.True(NCollection_MapAlgo.Contains(aMapRight, aMapSect));

        HashSet<int> aMapSubsLR = new();
        NCollection_MapAlgo.Subtraction(aMapSubsLR, aMapLeft, aMapRight);
        Assert.Equal(aRightLower - aLeftLower, aMapSubsLR.Count);
        for (int aKeyIter = aLeftLower; aKeyIter < aRightLower; ++aKeyIter)
        {
            Assert.Contains(aKeyIter, aMapSubsLR);
        }

        HashSet<int> aMapSubsRL = new();
        NCollection_MapAlgo.Subtraction(aMapSubsRL, aMapRight, aMapLeft);
        Assert.Equal(aRightUpper - aLeftUpper, aMapSubsRL.Count);
        for (int aKeyIter = aLeftUpper + 1; aKeyIter <= aRightUpper; ++aKeyIter)
        {
            Assert.Contains(aKeyIter, aMapSubsRL);
        }

        HashSet<int> aMapDiff = new();
        NCollection_MapAlgo.Difference(aMapDiff, aMapLeft, aMapRight);
        Assert.Equal(aRightLower - aLeftLower + aRightUpper - aLeftUpper, aMapDiff.Count);
        for (int aKeyIter = aLeftLower; aKeyIter < aRightLower; ++aKeyIter)
        {
            Assert.Contains(aKeyIter, aMapDiff);
        }
        for (int aKeyIter = aLeftUpper + 1; aKeyIter <= aRightUpper; ++aKeyIter)
        {
            Assert.Contains(aKeyIter, aMapDiff);
        }

        // OCCT's Exchange: the port's maps are HashSets now (unit 10J), exchanged by their references
        HashSet<int> aMapSwap = new();
        (aMapSwap, aMapSect) = (aMapSect, aMapSwap);
        for (int aKeyIter = aRightLower; aKeyIter <= aLeftUpper; ++aKeyIter)
        {
            Assert.Contains(aKeyIter, aMapSwap);
        }
        Assert.True(aMapSect.Count == 0);
    }
}

/// <summary>Additions: every operation of NCollection_MapAlgo, with the result map as an operand, against LINQ.</summary>
public class NCollection_MapAlgoTest
{
    private static HashSet<int> Map(params int[] theKeys)
    {
        HashSet<int> aMap = new();
        foreach (int aKey in theKeys)
        {
            aMap.Add(aKey);
        }
        return aMap;
    }

    private static int[] Sorted(HashSet<int> theMap) => [.. theMap.Order()];

    [Fact]
    public void Unite_ChangesAndReportsChange()
    {
        HashSet<int> aMap = Map(1, 2, 3);
        Assert.True(NCollection_MapAlgo.Unite(aMap, Map(3, 4)));
        Assert.Equal([1, 2, 3, 4], Sorted(aMap));
        Assert.False(NCollection_MapAlgo.Unite(aMap, Map(1, 4)));
        Assert.False(NCollection_MapAlgo.Unite(aMap, aMap));
        Assert.Equal([1, 2, 3, 4], Sorted(aMap));
    }

    [Fact]
    public void Unite_KeepsInsertionOrder()
    {
        HashSet<int> aMap = Map(7, 3);
        NCollection_MapAlgo.Unite(aMap, Map(9, 1, 3, 5));
        Assert.Equal([7, 3, 9, 1, 5], aMap.ToArray());
    }

    [Fact]
    public void Union_WithResultAsOperand()
    {
        HashSet<int> aLeft = Map(1, 2);
        HashSet<int> aRight = Map(2, 3);
        NCollection_MapAlgo.Union(aLeft, aLeft, aRight);
        Assert.Equal([1, 2, 3], Sorted(aLeft));

        HashSet<int> aRight2 = Map(5);
        NCollection_MapAlgo.Union(aRight2, Map(4), aRight2);
        Assert.Equal([4, 5], Sorted(aRight2));

        HashSet<int> aResult = Map(99);
        HashSet<int> aSame = Map(6, 7);
        NCollection_MapAlgo.Union(aResult, aSame, aSame);
        Assert.Equal([6, 7], Sorted(aResult));
    }

    [Fact]
    public void HasIntersection()
    {
        Assert.True(NCollection_MapAlgo.HasIntersection(Map(1, 2, 3), Map(3, 9)));
        Assert.False(NCollection_MapAlgo.HasIntersection(Map(1, 2, 3), Map(4, 5, 6, 7)));
        Assert.False(NCollection_MapAlgo.HasIntersection(Map(), Map(1)));
    }

    [Fact]
    public void Intersection_And_Intersect()
    {
        HashSet<int> aLeft = Map(1, 2, 3, 4);
        NCollection_MapAlgo.Intersection(aLeft, aLeft, Map(2, 4, 6));
        Assert.Equal([2, 4], Sorted(aLeft));

        HashSet<int> aRight = Map(2, 4, 6);
        NCollection_MapAlgo.Intersection(aRight, Map(4, 6, 8), aRight);
        Assert.Equal([4, 6], Sorted(aRight));

        HashSet<int> aMap = Map(1, 2, 3);
        Assert.True(NCollection_MapAlgo.Intersect(aMap, Map(2, 3, 4)));
        Assert.Equal([2, 3], Sorted(aMap));
        Assert.False(NCollection_MapAlgo.Intersect(aMap, Map(2, 3)));
        Assert.False(NCollection_MapAlgo.Intersect(aMap, aMap));
        Assert.False(NCollection_MapAlgo.Intersect(Map(), Map(1)));
    }

    [Fact]
    public void Subtract_And_Subtraction()
    {
        HashSet<int> aMap = Map(1, 2, 3);
        Assert.True(NCollection_MapAlgo.Subtract(aMap, Map(2, 9)));
        Assert.Equal([1, 3], Sorted(aMap));
        Assert.False(NCollection_MapAlgo.Subtract(aMap, Map(9)));
        Assert.True(NCollection_MapAlgo.Subtract(aMap, aMap));
        Assert.True(aMap.Count == 0);
        Assert.False(NCollection_MapAlgo.Subtract(aMap, aMap));

        HashSet<int> aRight = Map(2, 3);
        NCollection_MapAlgo.Subtraction(aRight, Map(1, 2, 3, 4), aRight);
        Assert.Equal([1, 4], Sorted(aRight));

        HashSet<int> aLeft = Map(1, 2, 3, 4);
        NCollection_MapAlgo.Subtraction(aLeft, aLeft, Map(1, 4));
        Assert.Equal([2, 3], Sorted(aLeft));
    }

    [Fact]
    public void Difference_And_Differ()
    {
        HashSet<int> aLeft = Map(1, 2, 3);
        NCollection_MapAlgo.Difference(aLeft, aLeft, Map(3, 4));
        Assert.Equal([1, 2, 4], Sorted(aLeft));

        HashSet<int> aRight = Map(3, 4);
        NCollection_MapAlgo.Difference(aRight, Map(1, 3), aRight);
        Assert.Equal([1, 4], Sorted(aRight));

        HashSet<int> aSame = Map(1);
        HashSet<int> aResult = Map(5);
        NCollection_MapAlgo.Difference(aResult, aSame, aSame);
        Assert.True(aResult.Count == 0);

        HashSet<int> aMap = Map(1, 2);
        Assert.True(NCollection_MapAlgo.Differ(aMap, Map(2, 3, 4)));
        Assert.Equal([1, 3, 4], Sorted(aMap));
        Assert.True(NCollection_MapAlgo.Differ(aMap, aMap));
        Assert.True(aMap.Count == 0);
        Assert.False(NCollection_MapAlgo.Differ(aMap, aMap));
    }

    /// <summary>OCCT judges "changed" by the count: exchanging one key for another reports no change.</summary>
    [Fact]
    public void Differ_ReportsByCount()
    {
        HashSet<int> aMap = Map(1, 2);
        Assert.False(NCollection_MapAlgo.Differ(aMap, Map(2, 3)));
        Assert.Equal([1, 3], Sorted(aMap));
    }

    [Fact]
    public void Contains_And_IsEqual()
    {
        HashSet<int> aMap = Map(1, 2, 3);
        Assert.True(NCollection_MapAlgo.Contains(aMap, Map(1, 3)));
        Assert.True(NCollection_MapAlgo.Contains(aMap, Map()));
        Assert.True(NCollection_MapAlgo.Contains(aMap, aMap));
        Assert.False(NCollection_MapAlgo.Contains(aMap, Map(1, 4)));
        Assert.False(NCollection_MapAlgo.Contains(Map(1), Map(1, 2)));
        Assert.True(NCollection_MapAlgo.IsEqual(aMap, Map(3, 2, 1)));
        Assert.True(NCollection_MapAlgo.IsEqual(aMap, aMap));
        Assert.False(NCollection_MapAlgo.IsEqual(aMap, Map(1, 2)));
        Assert.False(NCollection_MapAlgo.IsEqual(aMap, Map(1, 2, 4)));
    }

    [Fact]
    public void ReferenceKeys_UseTheMapsComparer()
    {
        object a = new(), b = new(), c = new();
        HashSet<object> aMap = new(ReferenceEqualityComparer.Instance);
        aMap.Add(a);
        aMap.Add(b);
        HashSet<object> anOther = new(ReferenceEqualityComparer.Instance);
        anOther.Add(b);
        anOther.Add(c);
        NCollection_MapAlgo.Intersection(aMap, aMap, anOther);
        Assert.Equal([b], aMap.ToArray());
        NCollection_MapAlgo.Unite(aMap, anOther);
        Assert.Equal([b, c], aMap.ToArray());
    }

    [Fact]
    public void PackedMapAlgo_Unite()
    {
        NCollection_PackedMapOfInteger aMap = new();
        NCollection_PackedMapOfInteger anOther = new();
        foreach (int aKey in new[] { 5, 70, 33, 1000, -4 })
        {
            anOther.Add(aKey);
        }

        // An empty map takes a node-for-node copy, with the other map's iteration order
        Assert.True(NCollection_PackedMapAlgo.Unite(aMap, anOther));
        Assert.Equal(keys(anOther), keys(aMap));
        Assert.False(NCollection_PackedMapAlgo.Unite(aMap, anOther));
        Assert.False(NCollection_PackedMapAlgo.Unite(aMap, aMap));
        Assert.False(NCollection_PackedMapAlgo.Unite(aMap, new NCollection_PackedMapOfInteger()));

        NCollection_PackedMapOfInteger aThird = new();
        aThird.Add(5);
        aThird.Add(6);
        Assert.True(NCollection_PackedMapAlgo.Unite(aMap, aThird));
        Assert.Equal(6, aMap.Extent());
        Assert.True(aMap.Contains(6));
    }

    private static List<int> keys(NCollection_PackedMapOfInteger theMap)
    {
        List<int> aKeys = [];
        for (NCollection_PackedMapOfInteger.Iterator anIter = new(theMap); anIter.More(); anIter.Next())
        {
            aKeys.Add(anIter.Key());
        }
        return aKeys;
    }
}
