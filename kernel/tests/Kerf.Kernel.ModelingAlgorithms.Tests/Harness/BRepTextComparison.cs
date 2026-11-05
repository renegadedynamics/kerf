// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// Decision 581, step 1: Boolean (and healing, sewing) results are first compared as BRep bytes, with the Free flag
// masked and counted (plan section 2.3: an unsplit argument face shared into the result keeps the Free flag it was
// frozen with, where native clears it).
//
// How later units use it: write the port's result with BRepTextComparison.Write(shape), read native's BRep text from
// the oracle (or a recorded file), and call Compare(native, port). A case is "bytes-identical" when
// IsIdenticalMaskingFree holds; FreeFlagDifferences is reported apart. Otherwise fall back to CanonicalShapeComparer.

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;

namespace Kerf.Kernel.TestHarness;

/// <summary>The result of comparing two BRep texts.</summary>
internal sealed class BRepTextDiff
{
    /// <summary>True if the texts are the same, byte for byte.</summary>
    public bool IsIdentical { get; init; }

    /// <summary>True if the texts are the same once each TShape's Free flag is masked.</summary>
    public bool IsIdenticalMaskingFree { get; init; }

    /// <summary>The number of TShapes whose Free flag differs (counted when the rest of their entry lines up).</summary>
    public int FreeFlagDifferences { get; init; }

    /// <summary>The 1-based number of the first differing line (Free masked), or 0.</summary>
    public int FirstDifferenceLine { get; init; }

    /// <summary>The first differing line of each text (Free masked), or null.</summary>
    public string? NativeLine { get; init; }

    /// <inheritdoc cref="NativeLine"/>
    public string? PortLine { get; init; }

    public override string ToString() =>
        IsIdentical ? "BRep bytes identical"
        : IsIdenticalMaskingFree ? $"BRep bytes identical but for {FreeFlagDifferences} Free flag(s)"
        : $"BRep texts differ at line {FirstDifferenceLine}: native \"{NativeLine}\", port \"{PortLine}\""
          + (FreeFlagDifferences > 0 ? $" ({FreeFlagDifferences} Free flag(s) differ too)" : "");
}

/// <summary>BRep text of shapes, and its comparison with the Free flag masked (decision 581).</summary>
internal static class BRepTextComparison
{
    /// <summary>The shape's BRep text, as <see cref="BrepTools.Write(in Shape, TextWriter, ProgressRange)"/> writes it.</summary>
    public static string Write(Shape theShape)
    {
        using var aWriter = new StringWriter(System.Globalization.CultureInfo.InvariantCulture);
        BrepTools.Write(theShape, aWriter);
        return aWriter.ToString();
    }

    /// <summary>Compares two BRep texts: bytes, then with every TShape's Free flag masked.</summary>
    public static BRepTextDiff Compare(string theNative, string thePort)
    {
        if (string.Equals(theNative, thePort, StringComparison.Ordinal))
        {
            return new BRepTextDiff { IsIdentical = true, IsIdenticalMaskingFree = true };
        }

        string[] aNative = SplitLines(theNative);
        string[] aPort = SplitLines(thePort);
        List<int> aNativeFlags = FlagLines(aNative);
        List<int> aPortFlags = FlagLines(aPort);

        // Free flags that differ, counted on the flag lines that line up (same entry, same line number)
        int aNbFree = 0;
        var aPortFlagSet = new HashSet<int>(aPortFlags);
        foreach (int aLine in aNativeFlags)
        {
            if (aPortFlagSet.Contains(aLine) && aLine < aPort.Length && aNative[aLine][0] != aPort[aLine][0]
                && aNative[aLine].AsSpan(1).SequenceEqual(aPort[aLine].AsSpan(1)))
            {
                ++aNbFree;
            }
        }

        string[] aNativeMasked = Masked(aNative, aNativeFlags);
        string[] aPortMasked = Masked(aPort, aPortFlags);
        int aNb = Math.Max(aNativeMasked.Length, aPortMasked.Length);
        for (int i = 0; i < aNb; ++i)
        {
            string? aN = i < aNativeMasked.Length ? aNativeMasked[i] : null;
            string? aP = i < aPortMasked.Length ? aPortMasked[i] : null;
            if (!string.Equals(aN, aP, StringComparison.Ordinal))
            {
                return new BRepTextDiff
                {
                    FreeFlagDifferences = aNbFree,
                    FirstDifferenceLine = i + 1,
                    NativeLine = i < aNative.Length ? aNative[i] : null,
                    PortLine = i < aPort.Length ? aPort[i] : null,
                };
            }
        }
        return new BRepTextDiff { IsIdenticalMaskingFree = true, FreeFlagDifferences = aNbFree };
    }

    /// <summary>The text with the Free flag of each TShape replaced by 'x'.</summary>
    public static string MaskFreeFlags(string theText)
    {
        string[] aLines = SplitLines(theText);
        return string.Join("\n", Masked(aLines, FlagLines(aLines)));
    }

    private static string[] SplitLines(string theText) => theText.Replace("\r\n", "\n", StringComparison.Ordinal).Split('\n');

    private static string[] Masked(string[] theLines, List<int> theFlagLines)
    {
        string[] aCopy = (string[])theLines.Clone();
        foreach (int aLine in theFlagLines)
        {
            aCopy[aLine] = "x" + aCopy[aLine][1..];
        }
        return aCopy;
    }

    /// <summary>
    /// The indices of the flag lines of the TShapes section (TopTools_ShapeSet.Write): after "TShapes n", each entry is
    /// its type line, its geometry, a line of seven '0'/'1' flags (Free, Modified, Checked, Orientable, Closed,
    /// Infinite, Convex), then its sub-shape references, ending with '*'.
    /// </summary>
    private static List<int> FlagLines(string[] theLines)
    {
        var aResult = new List<int>();
        int i = Array.FindIndex(theLines, l => l.StartsWith("TShapes ", StringComparison.Ordinal));
        if (i < 0)
        {
            return aResult;
        }
        int aNbShapes = int.Parse(theLines[i].AsSpan(8).Trim(), System.Globalization.CultureInfo.InvariantCulture);
        ++i;
        for (int aShape = 0; aShape < aNbShapes && i < theLines.Length; ++aShape)
        {
            ++i; // the type line
            // the geometry, up to the flags line
            while (i < theLines.Length && !IsFlagsLine(theLines[i]))
            {
                ++i;
            }
            if (i >= theLines.Length)
            {
                break;
            }
            aResult.Add(i);
            ++i;
            // the sub-shapes, up to the line ending with '*'
            while (i < theLines.Length && !theLines[i].TrimEnd().EndsWith('*'))
            {
                ++i;
            }
            ++i;
        }
        return aResult;
    }

    private static bool IsFlagsLine(string theLine)
    {
        if (theLine.Length != 7)
        {
            return false;
        }
        foreach (char c in theLine)
        {
            if (c != '0' && c != '1')
            {
                return false;
            }
        }
        return true;
    }
}
