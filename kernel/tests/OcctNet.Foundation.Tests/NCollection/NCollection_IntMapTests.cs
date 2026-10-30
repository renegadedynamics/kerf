// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

/// <summary>
/// <see cref="NCollection_IntMap"/>: OCCT's <c>NCollection_Map&lt;int&gt;</c>, iterating in OCCT's order (the order
/// of <see cref="NCollection_IntDataMap{TValue}"/>, which shares NCollection_BaseMap's buckets; decision 6901). The
/// pave filler's native recordings check it through the Boolean component (BOPAlgo_PaveFillerNativeTests).
/// </summary>
public class NCollection_IntMapTests
{
    private static List<int> Keys(NCollection_IntMap theMap)
    {
        List<int> aKeys = [];
        for (NCollection_IntMap.Iterator anIt = new(theMap); anIt.More(); anIt.Next())
        {
            aKeys.Add(anIt.Key());
        }
        return aKeys;
    }

    [Fact]
    public void Keys_iterate_by_bucket_then_newest_first_in_a_chain()
    {
        NCollection_IntMap aMap = new(10);
        foreach (int aKey in new[] { 5, 106, 3, 207, 100, 4 })
        {
            Assert.True(aMap.Add(aKey));
        }
        Assert.False(aMap.Add(106));
        Assert.Equal(101, aMap.NbBuckets());
        Assert.Equal([3, 4, 207, 106, 5, 100], Keys(aMap));
        Assert.Equal([3, 4, 207, 106, 5, 100], [.. aMap]); // foreach gives the same order
    }

    [Fact]
    public void Initial_buckets_change_the_order()
    {
        // OCCT's aMI(100, ...): the first insertion resizes to NextPrimeForMap(0) = 101, larger than 100, so such a
        // map orders as a default one; 1000 buckets are kept.
        NCollection_IntMap aMap100 = new(100), aMap1 = new();
        foreach (int aKey in new[] { 150, 49, 250, 1 })
        {
            aMap100.Add(aKey);
            aMap1.Add(aKey);
        }
        Assert.Equal(101, aMap100.NbBuckets());
        Assert.Equal(101, aMap1.NbBuckets());
        Assert.Equal(Keys(aMap1), Keys(aMap100));

        NCollection_IntMap aMap1000 = new(1000);
        foreach (int aKey in new[] { 150, 49, 250, 1 })
        {
            aMap1000.Add(aKey);
        }
        Assert.Equal(1000, aMap1000.NbBuckets());
        Assert.Equal([1, 49, 150, 250], Keys(aMap1000));
        Assert.Equal([1, 250, 49, 150], Keys(aMap1)); // buckets 2, 49 (250 % 101 + 1), then 50 with 49 newest before 150
    }

    [Fact]
    public void Same_order_as_IntDataMap_under_random_operations()
    {
        Random aRandom = new(1234);
        NCollection_IntMap aMap = new();
        NCollection_IntDataMap<bool> aRef = new();
        for (int i = 0; i < 20000; ++i)
        {
            int aKey = aRandom.Next(-500, 5000);
            switch (aRandom.Next(10))
            {
                case < 7:
                    Assert.Equal(aRef.Bind(aKey, true), aMap.Add(aKey));
                    break;
                case < 9:
                    Assert.Equal(aRef.UnBind(aKey), aMap.Remove(aKey));
                    break;
                default:
                    if (aRandom.Next(50) == 0)
                    {
                        aRef.Clear();
                        aMap.Clear();
                    }
                    break;
            }
            Assert.Equal(aRef.Extent(), aMap.Extent());
        }
        List<int> aRefKeys = [];
        for (NCollection_IntDataMap<bool>.Iterator anIt = new(aRef); anIt.More(); anIt.Next())
        {
            aRefKeys.Add(anIt.Key());
        }
        Assert.Equal(aRefKeys, Keys(aMap));
        Assert.Equal(aRef.NbBuckets(), aMap.NbBuckets());
    }

    [Fact]
    public void Copy_Assign_Exchange_and_Clear()
    {
        NCollection_IntMap aMap = new();
        foreach (int aKey in new[] { 7, 108, 2, 209, 50 })
        {
            aMap.Add(aKey);
        }
        NCollection_IntMap aCopy = new(aMap);
        Assert.Equal(aMap.NbBuckets(), aCopy.NbBuckets());
        Assert.Equal(Keys(aMap), Keys(aCopy));
        Assert.True(aCopy.Contains(209) && !aCopy.Contains(3));

        NCollection_IntMap anAssigned = new();
        anAssigned.Add(1000);
        anAssigned.Assign(aMap);
        Assert.Equal(Keys(aMap), Keys(anAssigned));

        NCollection_IntMap anOther = new();
        anOther.Add(1);
        anOther.Exchange(aCopy);
        Assert.Equal([1], Keys(aCopy));
        Assert.Equal(Keys(aMap), Keys(anOther));

        long aBuckets = aMap.NbBuckets();
        aMap.Clear();
        Assert.True(aMap.IsEmpty());
        Assert.Equal(aBuckets, aMap.NbBuckets()); // Clear() keeps the buckets
        Assert.Empty(Keys(aMap));
        Assert.Empty([.. aMap]);
        Assert.False(aMap.Remove(7));
    }
}
