// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_MapAlgo.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2024-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Boolean operations on <see cref="NCollection_Map{TKey}"/>: union, intersection, subtraction, symmetric difference,
/// inclusion and equality.
/// </summary>
/// <remarks>
/// OCCT's functions are templates over the map type; here they are generic over the key. The result map may be one
/// of the operands, as in OCCT; where OCCT exchanges the result's content into a temporary map (<c>Exchange</c>), the
/// port copies it into a new map with the same comparer and clears the result. <c>Assign</c> is clearing and adding the
/// other map's keys in its order.
/// </remarks>
public static class NCollection_MapAlgo
{
    /// <summary>
    /// Makes <paramref name="theMap"/> the union of the two maps (boolean OR). <paramref name="theMap"/> may be one of
    /// the operands.
    /// </summary>
    public static void Union<TKey>(NCollection_Map<TKey> theMap, NCollection_Map<TKey> theLeftMap, NCollection_Map<TKey> theRightMap)
        where TKey : notnull
    {
        if (ReferenceEquals(theLeftMap, theRightMap))
        {
            Assign(theMap, theLeftMap);
            return;
        }
        if (!ReferenceEquals(theMap, theLeftMap) && !ReferenceEquals(theMap, theRightMap))
        {
            theMap.Clear();
        }
        if (!ReferenceEquals(theMap, theLeftMap))
        {
            foreach (TKey aKey in theLeftMap)
            {
                theMap.Add(aKey);
            }
        }
        if (!ReferenceEquals(theMap, theRightMap))
        {
            foreach (TKey aKey in theRightMap)
            {
                theMap.Add(aKey);
            }
        }
    }

    /// <summary>Adds the keys of <paramref name="theOtherMap"/> to <paramref name="theMap"/>. True if it changed.</summary>
    public static bool Unite<TKey>(NCollection_Map<TKey> theMap, NCollection_Map<TKey> theOtherMap) where TKey : notnull
    {
        if (ReferenceEquals(theMap, theOtherMap))
        {
            return false;
        }
        int anOldExtent = theMap.Extent();
        Union(theMap, theMap, theOtherMap);
        return anOldExtent != theMap.Extent();
    }

    /// <summary>True if the two maps have a common key.</summary>
    public static bool HasIntersection<TKey>(NCollection_Map<TKey> theMap, NCollection_Map<TKey> theOtherMap) where TKey : notnull
    {
        NCollection_Map<TKey> aMap1 = theMap;
        NCollection_Map<TKey> aMap2 = theOtherMap;
        if (theOtherMap.Size() < theMap.Size())
        {
            aMap1 = theOtherMap;
            aMap2 = theMap;
        }
        foreach (TKey aKey in aMap1)
        {
            if (aMap2.Contains(aKey))
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>
    /// Makes <paramref name="theMap"/> the intersection of the two maps (boolean AND). <paramref name="theMap"/> may be
    /// one of the operands.
    /// </summary>
    public static void Intersection<TKey>(NCollection_Map<TKey> theMap, NCollection_Map<TKey> theLeftMap,
                                          NCollection_Map<TKey> theRightMap) where TKey : notnull
    {
        if (ReferenceEquals(theLeftMap, theRightMap))
        {
            Assign(theMap, theLeftMap);
            return;
        }
        if (ReferenceEquals(theMap, theLeftMap))
        {
            NCollection_Map<TKey> aCopy = exchanged(theMap);
            Intersection(theMap, aCopy, theRightMap);
            return;
        }
        else if (ReferenceEquals(theMap, theRightMap))
        {
            NCollection_Map<TKey> aCopy = exchanged(theMap);
            Intersection(theMap, theLeftMap, aCopy);
            return;
        }
        theMap.Clear();
        if (theLeftMap.Extent() < theRightMap.Extent())
        {
            foreach (TKey aKey in theLeftMap)
            {
                if (theRightMap.Contains(aKey))
                {
                    theMap.Add(aKey);
                }
            }
        }
        else
        {
            foreach (TKey aKey in theRightMap)
            {
                if (theLeftMap.Contains(aKey))
                {
                    theMap.Add(aKey);
                }
            }
        }
    }

    /// <summary>Keeps in <paramref name="theMap"/> only the keys also in <paramref name="theOtherMap"/>. True if it changed.</summary>
    public static bool Intersect<TKey>(NCollection_Map<TKey> theMap, NCollection_Map<TKey> theOtherMap) where TKey : notnull
    {
        if (ReferenceEquals(theMap, theOtherMap) || theMap.IsEmpty())
        {
            return false;
        }
        int anOldExtent = theMap.Extent();
        Intersection(theMap, theMap, theOtherMap);
        return anOldExtent != theMap.Extent();
    }

    /// <summary>Removes from <paramref name="theMap"/> the keys of <paramref name="theOtherMap"/>. True if it changed.</summary>
    public static bool Subtract<TKey>(NCollection_Map<TKey> theMap, NCollection_Map<TKey> theOtherMap) where TKey : notnull
    {
        if (ReferenceEquals(theMap, theOtherMap))
        {
            if (theMap.IsEmpty())
            {
                return false;
            }
            theMap.Clear();
            return true;
        }
        int anOldExtent = theMap.Extent();
        foreach (TKey aKey in theOtherMap)
        {
            theMap.Remove(aKey);
        }
        return anOldExtent != theMap.Extent();
    }

    /// <summary>
    /// Makes <paramref name="theMap"/> the keys of the left map that aren't in the right one. <paramref name="theMap"/>
    /// may be one of the operands.
    /// </summary>
    public static void Subtraction<TKey>(NCollection_Map<TKey> theMap, NCollection_Map<TKey> theLeftMap,
                                         NCollection_Map<TKey> theRightMap) where TKey : notnull
    {
        if (ReferenceEquals(theMap, theLeftMap))
        {
            Subtract(theMap, theRightMap);
            return;
        }
        else if (ReferenceEquals(theMap, theRightMap))
        {
            NCollection_Map<TKey> aCopy = exchanged(theMap);
            Subtraction(theMap, theLeftMap, aCopy);
            return;
        }
        Assign(theMap, theLeftMap);
        Subtract(theMap, theRightMap);
    }

    /// <summary>
    /// Makes <paramref name="theMap"/> the symmetric difference of the two maps (boolean XOR). <paramref name="theMap"/>
    /// may be one of the operands.
    /// </summary>
    public static void Difference<TKey>(NCollection_Map<TKey> theMap, NCollection_Map<TKey> theLeftMap,
                                        NCollection_Map<TKey> theRightMap) where TKey : notnull
    {
        if (ReferenceEquals(theLeftMap, theRightMap))
        {
            theMap.Clear();
            return;
        }
        else if (ReferenceEquals(theMap, theLeftMap))
        {
            NCollection_Map<TKey> aCopy = exchanged(theMap);
            Difference(theMap, aCopy, theRightMap);
            return;
        }
        else if (ReferenceEquals(theMap, theRightMap))
        {
            NCollection_Map<TKey> aCopy = exchanged(theMap);
            Difference(theMap, theLeftMap, aCopy);
            return;
        }
        theMap.Clear();
        foreach (TKey aKey in theLeftMap)
        {
            if (!theRightMap.Contains(aKey))
            {
                theMap.Add(aKey);
            }
        }
        foreach (TKey aKey in theRightMap)
        {
            if (!theLeftMap.Contains(aKey))
            {
                theMap.Add(aKey);
            }
        }
    }

    /// <summary>
    /// Makes <paramref name="theMap"/> the symmetric difference of itself and <paramref name="theOtherMap"/>. True if
    /// it changed.
    /// </summary>
    /// <remarks>
    /// As OCCT, "changed" is judged by the number of keys: exchanging as many keys as are added keeps the count and
    /// returns false.
    /// </remarks>
    public static bool Differ<TKey>(NCollection_Map<TKey> theMap, NCollection_Map<TKey> theOtherMap) where TKey : notnull
    {
        if (ReferenceEquals(theMap, theOtherMap))
        {
            if (theMap.IsEmpty())
            {
                return false;
            }
            theMap.Clear();
            return true;
        }
        int anOldExtent = theMap.Extent();
        Difference(theMap, theMap, theOtherMap);
        return anOldExtent != theMap.Extent();
    }

    /// <summary>True if <paramref name="theMap"/> contains every key of <paramref name="theOtherMap"/>.</summary>
    public static bool Contains<TKey>(NCollection_Map<TKey> theMap, NCollection_Map<TKey> theOtherMap) where TKey : notnull
    {
        if (ReferenceEquals(theMap, theOtherMap) || theOtherMap.IsEmpty())
        {
            return true;
        }
        else if (theMap.Extent() < theOtherMap.Extent())
        {
            return false;
        }
        foreach (TKey aKey in theOtherMap)
        {
            if (!theMap.Contains(aKey))
            {
                return false;
            }
        }
        return true;
    }

    /// <summary>True if the two maps have exactly the same keys.</summary>
    public static bool IsEqual<TKey>(NCollection_Map<TKey> theMap, NCollection_Map<TKey> theOtherMap) where TKey : notnull
    {
        if (ReferenceEquals(theMap, theOtherMap))
        {
            return true;
        }
        return theMap.Extent() == theOtherMap.Extent() && Contains(theMap, theOtherMap);
    }

    // theMap.Assign(theOther)
    private static void Assign<TKey>(NCollection_Map<TKey> theMap, NCollection_Map<TKey> theOther) where TKey : notnull
    {
        if (ReferenceEquals(theMap, theOther))
        {
            return;
        }
        theMap.Clear();
        foreach (TKey aKey in theOther)
        {
            theMap.Add(aKey);
        }
    }

    // MapType aCopy; theMap.Exchange(aCopy): the content moves to the returned map and theMap is left empty.
    private static NCollection_Map<TKey> exchanged<TKey>(NCollection_Map<TKey> theMap) where TKey : notnull
    {
        NCollection_Map<TKey> aCopy = new(theMap);
        theMap.Clear();
        return aCopy;
    }
}
