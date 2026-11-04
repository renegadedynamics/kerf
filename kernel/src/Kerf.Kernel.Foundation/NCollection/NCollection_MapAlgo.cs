// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_MapAlgo.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2024-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Boolean operations on <see cref="HashSet{TKey}"/>: union, intersection, subtraction, symmetric difference,
/// inclusion and equality.
/// </summary>
/// <remarks>
/// OCCT's functions are templates over the map type; here they are generic over the key. The result map may be one
/// of the operands, as in OCCT; where OCCT exchanges the result's content into a temporary map (<c>Exchange</c>), the
/// port copies it into a new map with the same comparer and clears the result. <c>Assign</c> is clearing and adding the
/// other map's keys in its order.
/// </remarks>
internal static class NCollection_MapAlgo
{
    /// <summary>
    /// Makes <paramref name="map"/> the union of the two maps (boolean OR). <paramref name="map"/> may be one of
    /// the operands.
    /// </summary>
    public static void Union<TKey>(HashSet<TKey> map, HashSet<TKey> leftMap, HashSet<TKey> rightMap)
        where TKey : notnull
    {
        if (ReferenceEquals(leftMap, rightMap))
        {
            Assign(map, leftMap);
            return;
        }
        if (!ReferenceEquals(map, leftMap) && !ReferenceEquals(map, rightMap))
        {
            map.Clear();
        }
        if (!ReferenceEquals(map, leftMap))
        {
            foreach (TKey key in leftMap)
            {
                map.Add(key);
            }
        }
        if (!ReferenceEquals(map, rightMap))
        {
            foreach (TKey key in rightMap)
            {
                map.Add(key);
            }
        }
    }

    /// <summary>Adds the keys of <paramref name="otherMap"/> to <paramref name="map"/>. True if it changed.</summary>
    public static bool Unite<TKey>(HashSet<TKey> map, HashSet<TKey> otherMap) where TKey : notnull
    {
        if (ReferenceEquals(map, otherMap))
        {
            return false;
        }
        int oldExtent = map.Count;
        Union(map, map, otherMap);
        return oldExtent != map.Count;
    }

    /// <summary>True if the two maps have a common key.</summary>
    public static bool HasIntersection<TKey>(HashSet<TKey> map, HashSet<TKey> otherMap) where TKey : notnull
    {
        HashSet<TKey> map1 = map;
        HashSet<TKey> map2 = otherMap;
        if (otherMap.Count < map.Count)
        {
            map1 = otherMap;
            map2 = map;
        }
        foreach (TKey key in map1)
        {
            if (map2.Contains(key))
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>
    /// Makes <paramref name="map"/> the intersection of the two maps (boolean AND). <paramref name="map"/> may be
    /// one of the operands.
    /// </summary>
    public static void Intersection<TKey>(HashSet<TKey> map, HashSet<TKey> leftMap,
                                          HashSet<TKey> rightMap) where TKey : notnull
    {
        if (ReferenceEquals(leftMap, rightMap))
        {
            Assign(map, leftMap);
            return;
        }
        if (ReferenceEquals(map, leftMap))
        {
            HashSet<TKey> copy = Exchanged(map);
            Intersection(map, copy, rightMap);
            return;
        }
        else if (ReferenceEquals(map, rightMap))
        {
            HashSet<TKey> copy = Exchanged(map);
            Intersection(map, leftMap, copy);
            return;
        }
        map.Clear();
        if (leftMap.Count < rightMap.Count)
        {
            foreach (TKey key in leftMap)
            {
                if (rightMap.Contains(key))
                {
                    map.Add(key);
                }
            }
        }
        else
        {
            foreach (TKey key in rightMap)
            {
                if (leftMap.Contains(key))
                {
                    map.Add(key);
                }
            }
        }
    }

    /// <summary>Keeps in <paramref name="map"/> only the keys also in <paramref name="otherMap"/>. True if it changed.</summary>
    public static bool Intersect<TKey>(HashSet<TKey> map, HashSet<TKey> otherMap) where TKey : notnull
    {
        if (ReferenceEquals(map, otherMap) || map.Count == 0)
        {
            return false;
        }
        int oldExtent = map.Count;
        Intersection(map, map, otherMap);
        return oldExtent != map.Count;
    }

    /// <summary>Removes from <paramref name="map"/> the keys of <paramref name="otherMap"/>. True if it changed.</summary>
    public static bool Subtract<TKey>(HashSet<TKey> map, HashSet<TKey> otherMap) where TKey : notnull
    {
        if (ReferenceEquals(map, otherMap))
        {
            if (map.Count == 0)
            {
                return false;
            }
            map.Clear();
            return true;
        }
        int oldExtent = map.Count;
        foreach (TKey key in otherMap)
        {
            map.Remove(key);
        }
        return oldExtent != map.Count;
    }

    /// <summary>
    /// Makes <paramref name="map"/> the keys of the left map that aren't in the right one. <paramref name="map"/>
    /// may be one of the operands.
    /// </summary>
    public static void Subtraction<TKey>(HashSet<TKey> map, HashSet<TKey> leftMap,
                                         HashSet<TKey> rightMap) where TKey : notnull
    {
        if (ReferenceEquals(map, leftMap))
        {
            Subtract(map, rightMap);
            return;
        }
        else if (ReferenceEquals(map, rightMap))
        {
            HashSet<TKey> copy = Exchanged(map);
            Subtraction(map, leftMap, copy);
            return;
        }
        Assign(map, leftMap);
        Subtract(map, rightMap);
    }

    /// <summary>
    /// Makes <paramref name="map"/> the symmetric difference of the two maps (boolean XOR). <paramref name="map"/>
    /// may be one of the operands.
    /// </summary>
    public static void Difference<TKey>(HashSet<TKey> map, HashSet<TKey> leftMap,
                                        HashSet<TKey> rightMap) where TKey : notnull
    {
        if (ReferenceEquals(leftMap, rightMap))
        {
            map.Clear();
            return;
        }
        else if (ReferenceEquals(map, leftMap))
        {
            HashSet<TKey> copy = Exchanged(map);
            Difference(map, copy, rightMap);
            return;
        }
        else if (ReferenceEquals(map, rightMap))
        {
            HashSet<TKey> copy = Exchanged(map);
            Difference(map, leftMap, copy);
            return;
        }
        map.Clear();
        foreach (TKey key in leftMap)
        {
            if (!rightMap.Contains(key))
            {
                map.Add(key);
            }
        }
        foreach (TKey key in rightMap)
        {
            if (!leftMap.Contains(key))
            {
                map.Add(key);
            }
        }
    }

    /// <summary>
    /// Makes <paramref name="map"/> the symmetric difference of itself and <paramref name="otherMap"/>. True if
    /// it changed.
    /// </summary>
    /// <remarks>
    /// As OCCT, "changed" is judged by the number of keys: exchanging as many keys as are added keeps the count and
    /// returns false.
    /// </remarks>
    public static bool Differ<TKey>(HashSet<TKey> map, HashSet<TKey> otherMap) where TKey : notnull
    {
        if (ReferenceEquals(map, otherMap))
        {
            if (map.Count == 0)
            {
                return false;
            }
            map.Clear();
            return true;
        }
        int oldExtent = map.Count;
        Difference(map, map, otherMap);
        return oldExtent != map.Count;
    }

    /// <summary>True if <paramref name="map"/> contains every key of <paramref name="otherMap"/>.</summary>
    public static bool Contains<TKey>(HashSet<TKey> map, HashSet<TKey> otherMap) where TKey : notnull
    {
        if (ReferenceEquals(map, otherMap) || otherMap.Count == 0)
        {
            return true;
        }
        else if (map.Count < otherMap.Count)
        {
            return false;
        }
        foreach (TKey key in otherMap)
        {
            if (!map.Contains(key))
            {
                return false;
            }
        }
        return true;
    }

    /// <summary>True if the two maps have exactly the same keys.</summary>
    public static bool IsEqual<TKey>(HashSet<TKey> map, HashSet<TKey> otherMap) where TKey : notnull
    {
        if (ReferenceEquals(map, otherMap))
        {
            return true;
        }
        return map.Count == otherMap.Count && Contains(map, otherMap);
    }

    // theMap.Assign(theOther)
    private static void Assign<TKey>(HashSet<TKey> map, HashSet<TKey> other) where TKey : notnull
    {
        if (ReferenceEquals(map, other))
        {
            return;
        }
        map.Clear();
        foreach (TKey key in other)
        {
            map.Add(key);
        }
    }

    // MapType aCopy; theMap.Exchange(aCopy): the content moves to the returned map and theMap is left empty.
    private static HashSet<TKey> Exchanged<TKey>(HashSet<TKey> map) where TKey : notnull
    {
        HashSet<TKey> copy = new(map,map.Comparer);
        map.Clear();
        return copy;
    }
}
