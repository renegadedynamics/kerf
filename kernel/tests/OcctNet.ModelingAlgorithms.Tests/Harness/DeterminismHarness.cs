// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// Decision 580 (decision 490's rule for Booleans): a parallel run gives the serial run's result, bit for bit, at
// every degree of parallelism and on every repeat.
//
// How later units use it: wrap the operation in a function that builds a fresh algorithm, runs it with
// SetRunParallel(true) and returns its result (a shape, a shape and its history, or any text such as a DS dump), and
// call DeterminismHarness.AssertSerialEqualsParallel(run). The reference run is serial (OSD_Parallel's
// ForceSerialScope makes every OSD_Parallel loop a plain loop); the others run at each degree of parallelism
// (MaxDegreeOfParallelismScope), several times. Both scopes are AsyncLocal, so tests running in parallel don't
// interfere.

using System.Globalization;
using System.Text;

namespace OcctNet.TestHarness;

/// <summary>Runs an operation serially and in parallel, repeatedly, and requires the same output every time.</summary>
internal static class DeterminismHarness
{
    /// <summary>The default degrees of parallelism: 1, 2, 4 and the number of logical processors.</summary>
    public static int[] DefaultDegrees() => [.. new[] { 1, 2, 4, Environment.ProcessorCount }.Distinct()];

    /// <summary>
    /// Runs <paramref name="theRun"/> serially as the reference, then <paramref name="theRepeats"/> times at each
    /// degree of parallelism (<see cref="DefaultDegrees"/> if none given), and fails at the first output that differs
    /// from the reference (with its first differing line). Returns the reference output.
    /// </summary>
    public static string AssertSerialEqualsParallel(Func<string> theRun, int theRepeats = 3, params int[] theDegrees)
    {
        string aReference;
        using (OSD_Parallel.ForceSerialScope())
        {
            aReference = theRun();
        }
        int[] aDegrees = theDegrees.Length > 0 ? theDegrees : DefaultDegrees();
        foreach (int aDegree in aDegrees)
        {
            for (int aRepeat = 1; aRepeat <= theRepeats; ++aRepeat)
            {
                string anOutput;
                using (OSD_Parallel.MaxDegreeOfParallelismScope(aDegree))
                {
                    anOutput = theRun();
                }
                if (!string.Equals(aReference, anOutput, StringComparison.Ordinal))
                {
                    Assert.Fail($"run {aRepeat} at degree of parallelism {aDegree} differs from the serial run: {FirstDifference(aReference, anOutput)}");
                }
            }
        }
        return aReference;
    }

    /// <summary>As <see cref="AssertSerialEqualsParallel(Func{string}, int, int[])"/>, comparing the result's BRep text.</summary>
    public static TopoDS_Shape AssertSerialEqualsParallel(Func<TopoDS_Shape> theRun, int theRepeats = 3, params int[] theDegrees)
    {
        TopoDS_Shape aReference = new();
        AssertSerialEqualsParallel(() =>
        {
            TopoDS_Shape aShape = theRun();
            if (aReference.IsNull())
            {
                aReference = aShape;
            }
            return BRepTextComparison.Write(aShape);
        }, theRepeats, theDegrees);
        return aReference;
    }

    /// <summary>
    /// As <see cref="AssertSerialEqualsParallel(Func{string}, int, int[])"/>, comparing the result's BRep text and its
    /// history over the arguments (<see cref="HistoryText"/>).
    /// </summary>
    public static void AssertSerialEqualsParallel(IReadOnlyList<TopoDS_Shape> theArguments,
                                                  Func<(TopoDS_Shape Result, BRepTools_History? History)> theRun,
                                                  int theRepeats = 3, params int[] theDegrees)
    {
        AssertSerialEqualsParallel(() =>
        {
            (TopoDS_Shape aResult, BRepTools_History? aHistory) = theRun();
            return BRepTextComparison.Write(aResult) + "\n" + HistoryText(theArguments, aResult, aHistory);
        }, theRepeats, theDegrees);
    }

    /// <summary>
    /// A history as text, in a stable order: for each vertex, edge, face and solid of the arguments (TopExp.MapShapes
    /// order over the arguments in turn), the indices of its modified and generated shapes in the result's
    /// TopExp.MapShapes numbering (0 for a shape not in the result), and whether it is removed.
    /// </summary>
    public static string HistoryText(IReadOnlyList<TopoDS_Shape> theArguments, TopoDS_Shape theResult, BRepTools_History? theHistory)
    {
        if (theHistory is null)
        {
            return "no history";
        }
        var aResultMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        if (!theResult.IsNull())
        {
            TopExp.MapShapes(theResult, aResultMap);
        }
        var anArgumentMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        foreach (TopoDS_Shape anArgument in theArguments)
        {
            TopExp.MapShapes(anArgument, anArgumentMap);
        }
        var aText = new StringBuilder();
        for (int i = 1; i <= anArgumentMap.Extent(); ++i)
        {
            TopoDS_Shape aShape = anArgumentMap.FindKey(i);
            if (!BRepTools_History.IsSupportedType(aShape))
            {
                continue;
            }
            aText.Append(CultureInfo.InvariantCulture, $"{i} {aShape.ShapeType()} M[");
            aText.AppendJoin(' ', theHistory.Modified(aShape).Select(s => aResultMap.FindIndex(s)));
            aText.Append("] G[");
            aText.AppendJoin(' ', theHistory.Generated(aShape).Select(s => aResultMap.FindIndex(s)));
            aText.Append(theHistory.IsRemoved(aShape) ? "] removed\n" : "]\n");
        }
        return aText.ToString();
    }

    /// <summary>The first differing line of two texts, as "line n: 'a' / 'b'".</summary>
    public static string FirstDifference(string theA, string theB)
    {
        string[] aLinesA = theA.Split('\n');
        string[] aLinesB = theB.Split('\n');
        int aNb = Math.Max(aLinesA.Length, aLinesB.Length);
        for (int i = 0; i < aNb; ++i)
        {
            string? a = i < aLinesA.Length ? aLinesA[i] : null;
            string? b = i < aLinesB.Length ? aLinesB[i] : null;
            if (!string.Equals(a, b, StringComparison.Ordinal))
            {
                return $"line {i + 1}: '{a ?? "<end>"}' / '{b ?? "<end>"}'";
            }
        }
        return "no difference";
    }
}
