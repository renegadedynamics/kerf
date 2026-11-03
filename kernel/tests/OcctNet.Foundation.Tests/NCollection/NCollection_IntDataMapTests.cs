// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace OcctNet.Foundation.Tests;

/// <summary>
/// <see cref="NCollection_IntDataMap{TValue}"/>: OCCT's DataMap of int keys, iterating in OCCT's order. The expected
/// orders are derived from NCollection_DataMap/BaseMap's code: identity hash, bucket <c>hash % NbBuckets + 1</c>, new
/// keys at the head of their chain, 101 buckets (the first of NCollection_Primes) at the first insertion, growth to the
/// next prime when there are more keys than buckets.
/// </summary>
public class NCollection_IntDataMapTests
{
    private static List<int> Keys<T>(NCollection_IntDataMap<T> theMap)
    {
        List<int> aKeys = [];
        for (NCollection_IntDataMap<T>.Iterator anIt = new(theMap); anIt.More(); anIt.Next())
        {
            aKeys.Add(anIt.Key());
        }

        return aKeys;
    }

    [Fact]
    public void Keys_iterate_by_bucket_then_newest_first_in_a_chain()
    {
        NCollection_IntDataMap<bool> aMap = new(10);
        foreach (int aKey in new[] { 5, 106, 3, 207, 100, 4 })
        {
            Assert.True(aMap.Bind(aKey, true));
        }

        Assert.Equal(101, aMap.NbBuckets());
        // Buckets: 3 -> 4, 4 -> 5, 5/106/207 -> 6 (207 newest), 100 -> 101.
        Assert.Equal([3, 4, 207, 106, 5, 100], Keys(aMap));
    }

    [Fact]
    public void Negative_keys_hash_as_size_t()
    {
        NCollection_IntDataMap<int> aMap = new();
        aMap.Bind(-1, 1);
        aMap.Bind(0, 2);
        aMap.Bind(1, 3);
        // (size_t)-1 % 101 = 18446744073709551615 % 101 = 78, so -1 is in bucket 79, after 0 (1) and 1 (2).
        Assert.Equal(78UL, 18446744073709551615UL % 101UL);
        Assert.Equal([0, 1, -1], Keys(aMap));
    }

    [Fact]
    public void Bind_existing_key_replaces_and_UnBind_keeps_the_others_order()
    {
        NCollection_IntDataMap<bool> aMap = new(10);
        aMap.Bind(7, true);
        aMap.Bind(108, true);
        aMap.Bind(2, false);
        Assert.False(aMap.Bind(7, false));
        Assert.False(aMap[7]);
        Assert.Equal(3, aMap.Extent());

        Assert.True(aMap.UnBind(108));
        Assert.False(aMap.UnBind(108));
        Assert.Equal([2, 7], Keys(aMap));
        Assert.Throws<NoSuchObjectException>(() => aMap.Find(108));
    }

    [Fact]
    public void Growth_rehashes_to_the_next_prime()
    {
        NCollection_IntDataMap<int> aMap = new();
        for (int i = 0; i < 102; ++i)
        {
            aMap.Bind(1009 - i, i);
        }

        // The 102nd insertion found 101 keys in 101 buckets: not more, so no growth yet.
        Assert.Equal(101, aMap.NbBuckets());
        aMap.Bind(5000, 0);
        Assert.Equal(1009, aMap.NbBuckets());
        List<int> aKeys = Keys(aMap);
        Assert.Equal(103, aKeys.Count);
        // With 1009 buckets every key < 1009 has its own bucket; 1009 % 1009 = 0 goes first.
        Assert.Equal(1009, aKeys[0]);
        Assert.Equal(908, aKeys[1]);
    }

    [Fact]
    public void Clear_keeps_the_buckets()
    {
        NCollection_IntDataMap<bool> aMap = new(10);
        for (int i = 0; i < 300; ++i)
        {
            aMap.Bind(i, true);
        }

        long aNbBuckets = aMap.NbBuckets();
        aMap.Clear();
        Assert.True(aMap.IsEmpty());
        Assert.Empty(Keys(aMap));
        aMap.Bind(1009 + 1, true);
        aMap.Bind(1, true);
        Assert.Equal(aNbBuckets, aMap.NbBuckets());
        Assert.Equal(1009, aNbBuckets);
        // Both in bucket 2 of 1009: the newest first.
        Assert.Equal([1, 1010], Keys(aMap));
    }

    [Fact]
    public void Bound_TryBound_and_the_enumerator_follow_OCCTs_order()
    {
        NCollection_IntDataMap<int> aMap = new(10);
        aMap.Bound(5, 1) += 10;
        aMap.TryBound(5, 99) += 100;  // already bound: the item is kept
        aMap.TryBound(106, 2);
        aMap.Bound(3, 3);
        Assert.Equal(111, aMap.Find(5));
        Assert.Equal([3, 106, 5], aMap.Keys.ToList());
        Assert.Equal([(3, 3), (106, 2), (5, 111)], aMap.Select(kv => (kv.Key, kv.Value)).ToList());
    }
}
