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

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;
using System.Globalization;
using System.Text;

namespace Kerf.Kernel.TestHarness;

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
        using (OsdParallel.ForceSerialScope())
        {
            aReference = theRun();
        }
        int[] aDegrees = theDegrees.Length > 0 ? theDegrees : DefaultDegrees();
        foreach (int aDegree in aDegrees)
        {
            for (int aRepeat = 1; aRepeat <= theRepeats; ++aRepeat)
            {
                string anOutput;
                using (OsdParallel.MaxDegreeOfParallelismScope(aDegree))
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
    public static Shape AssertSerialEqualsParallel(Func<Shape> theRun, int theRepeats = 3, params int[] theDegrees)
    {
        Shape aReference = new();
        AssertSerialEqualsParallel(() =>
        {
            Shape aShape = theRun();
            if (aReference.IsNull)
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
    public static void AssertSerialEqualsParallel(IReadOnlyList<Shape> theArguments,
                                                  Func<(Shape Result, History? History)> theRun,
                                                  int theRepeats = 3, params int[] theDegrees)
    {
        AssertSerialEqualsParallel(() =>
        {
            (Shape aResult, History? aHistory) = theRun();
            return BRepTextComparison.Write(aResult) + "\n" + HistoryText(theArguments, aResult, aHistory);
        }, theRepeats, theDegrees);
    }

    /// <summary>
    /// A history as text, in a stable order: for each vertex, edge, face and solid of the arguments (TopExp.MapShapes
    /// order over the arguments in turn), the indices of its modified and generated shapes in the result's
    /// TopExp.MapShapes numbering (0 for a shape not in the result), and whether it is removed.
    /// </summary>
    public static string HistoryText(IReadOnlyList<Shape> theArguments, Shape theResult, History? theHistory)
    {
        if (theHistory is null)
        {
            return "no history";
        }
        var aResultMap = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        if (!theResult.IsNull)
        {
            Exploration.MapShapes(theResult, aResultMap);
        }
        var anArgumentMap = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        foreach (Shape anArgument in theArguments)
        {
            Exploration.MapShapes(anArgument, anArgumentMap);
        }
        var aText = new StringBuilder();
        for (int i = 1; i <= anArgumentMap.Extent(); ++i)
        {
            Shape aShape = anArgumentMap.FindKey(i);
            if (!History.IsSupportedType(aShape))
            {
                continue;
            }
            aText.Append(CultureInfo.InvariantCulture, $"{i} {Kerf.Engine.Foundation.OcctTypeNames.Of(aShape.ShapeType())} M[");
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
