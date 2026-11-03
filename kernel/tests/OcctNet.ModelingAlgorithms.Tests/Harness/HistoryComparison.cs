// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// Decision 581's last step (plan section 2.5, item 3): for every argument sub-shape, Modified, Generated and
// IsDeleted on both sides, compared through the canonical match of the results.
//
// How later units use it: with the native arguments, result and history (read from the oracle's BRep text and its
// recorded history) and the port's, call HistoryComparison.Compare(...) and assert that the returned list of
// differences is empty (or call AssertSame). The arguments of the two sides may be different objects (read twice):
// they are matched by their canonical keys too.

using Kerf.Engine.Topology;

using Kerf.Topology;

namespace OcctNet.TestHarness;

/// <summary>Compares two histories through the canonical keys of the arguments and the results.</summary>
internal static class HistoryComparison
{
    /// <summary>
    /// The differences between the two histories: for each canonical key of the arguments' vertices, edges, faces and
    /// solids, the multisets of keys of the shapes modified and generated from the sub-shapes with that key, and
    /// their removed flags. Empty if the histories agree.
    /// </summary>
    public static List<string> Compare(IReadOnlyList<Shape> theNativeArguments, Shape theNativeResult,
                                       History? theNativeHistory,
                                       IReadOnlyList<Shape> thePortArguments, Shape thePortResult,
                                       History? thePortHistory)
    {
        var aDifferences = new List<string>();
        if (theNativeHistory is null || thePortHistory is null)
        {
            if ((theNativeHistory is null) != (thePortHistory is null))
            {
                aDifferences.Add($"history: native {(theNativeHistory is null ? "none" : "present")}, port {(thePortHistory is null ? "none" : "present")}");
            }
            return aDifferences;
        }

        Dictionary<string, Entry> aNative = Entries(theNativeArguments, theNativeResult, theNativeHistory);
        Dictionary<string, Entry> aPort = Entries(thePortArguments, thePortResult, thePortHistory);
        foreach (string aKey in aNative.Keys.Union(aPort.Keys).Order(StringComparer.Ordinal))
        {
            if (!aNative.TryGetValue(aKey, out Entry? aN) || !aPort.TryGetValue(aKey, out Entry? aP))
            {
                aDifferences.Add($"argument sub-shape [{aKey}] only in the {(aNative.ContainsKey(aKey) ? "native" : "port")} arguments");
                continue;
            }
            if (!aN.Modified.SequenceEqual(aP.Modified, StringComparer.Ordinal))
            {
                aDifferences.Add($"{aN.Description} [{aKey}]: Modified native {aN.Modified.Count} shape(s), port {aP.Modified.Count}, or different shapes");
            }
            if (!aN.Generated.SequenceEqual(aP.Generated, StringComparer.Ordinal))
            {
                aDifferences.Add($"{aN.Description} [{aKey}]: Generated native {aN.Generated.Count} shape(s), port {aP.Generated.Count}, or different shapes");
            }
            if (!aN.Removed.SequenceEqual(aP.Removed))
            {
                aDifferences.Add($"{aN.Description} [{aKey}]: IsDeleted native {string.Join(",", aN.Removed)}, port {string.Join(",", aP.Removed)}");
            }
        }
        return aDifferences;
    }

    /// <summary>Asserts that the histories agree (<see cref="Compare"/>), listing the differences otherwise.</summary>
    public static void AssertSame(IReadOnlyList<Shape> theNativeArguments, Shape theNativeResult,
                                  History? theNativeHistory,
                                  IReadOnlyList<Shape> thePortArguments, Shape thePortResult,
                                  History? thePortHistory)
    {
        List<string> aDifferences = Compare(theNativeArguments, theNativeResult, theNativeHistory,
                                            thePortArguments, thePortResult, thePortHistory);
        Assert.True(aDifferences.Count == 0, "histories differ:\n  " + string.Join("\n  ", aDifferences.Take(30)));
    }

    private sealed class Entry
    {
        public string Description = "";
        public List<string> Modified = [];
        public List<string> Generated = [];
        public List<bool> Removed = [];
    }

    // Argument key -> the history of all argument sub-shapes with that key, as sorted result keys.
    private static Dictionary<string, Entry> Entries(IReadOnlyList<Shape> theArguments, Shape theResult,
                                                     History theHistory)
    {
        CanonicalKeys aResultKeys = CanonicalKeys.Build(theResult);
        var anArgumentMap = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        var aKeysOf = new List<CanonicalKeys>();
        foreach (Shape anArgument in theArguments)
        {
            CanonicalKeys aKeys = CanonicalKeys.Build(anArgument);
            int aBefore = anArgumentMap.Extent();
            Exploration.MapShapes(anArgument, anArgumentMap);
            for (int i = aBefore + 1; i <= anArgumentMap.Extent(); ++i)
            {
                aKeysOf.Add(aKeys);
            }
        }

        var anEntries = new Dictionary<string, Entry>(StringComparer.Ordinal);
        for (int i = 1; i <= anArgumentMap.Extent(); ++i)
        {
            Shape aShape = anArgumentMap.FindKey(i);
            if (!History.IsSupportedType(aShape))
            {
                continue;
            }
            CanonicalKeys aKeys = aKeysOf[i - 1];
            string aKey = aKeys.Key(aShape);
            if (!anEntries.TryGetValue(aKey, out Entry? anEntry))
            {
                anEntries[aKey] = anEntry = new Entry { Description = aKeys.Describe(aKey) };
            }
            anEntry.Modified.AddRange(theHistory.Modified(aShape).Select(aResultKeys.Key));
            anEntry.Generated.AddRange(theHistory.Generated(aShape).Select(aResultKeys.Key));
            anEntry.Removed.Add(theHistory.IsRemoved(aShape));
        }
        foreach (Entry anEntry in anEntries.Values)
        {
            anEntry.Modified.Sort(StringComparer.Ordinal);
            anEntry.Generated.Sort(StringComparer.Ordinal);
            anEntry.Removed.Sort();
        }
        return anEntries;
    }
}
