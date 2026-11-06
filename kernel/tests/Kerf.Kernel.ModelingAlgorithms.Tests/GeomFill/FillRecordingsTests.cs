// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The port against native OCCT's GeomFill fillings recorded by tools/fillrec (<see cref="FillRecordings"/>): every
/// token of every case, bit for bit.
/// </summary>
public class FillRecordingsTests(ITestOutputHelper output)
{
    public static IEnumerable<object[]> Kinds() =>
        new[] { "BSC4", "BSC3", "BSC2", "BZC4", "BZC3", "BZC2", "COONS", "CURVED", "STRETCH", "CURVED2", "CF" }
            .Select(k => new object[] { k });

    [Theory]
    [MemberData(nameof(Kinds))]
    public void Port_matches_native_bit_for_bit(string theKind)
    {
        List<FillRecordings.Case> aCases = FillRecordings.Cases.Where(c => c.Kind == theKind).ToList();
        Assert.NotEmpty(aCases);
        int aTokens = 0, aBad = 0, anIntended = 0;
        foreach (FillRecordings.Case aCase in aCases)
        {
            List<string> aPort = FillRecordings.Run(aCase);
            aTokens += aCase.Native.Length;
            bool aSame = aPort.Count == aCase.Native.Length;
            for (int i = 0; aSame && i < aPort.Count; i++)
            {
                aSame = aPort[i] == aCase.Native[i];
            }
            if (!aSame && IsCoonsWeights(aPort, aCase.Native))
            {
                anIntended++;
            }
            else if (!aSame)
            {
                aBad++;
                output.WriteLine($"case {aCases.IndexOf(aCase)}: {aPort.Count} tokens, native {aCase.Native.Length}");
                for (int i = 0; i < Math.Max(aPort.Count, aCase.Native.Length); i++)
                {
                    string p = i < aPort.Count ? aPort[i] : "-";
                    string n = i < aCase.Native.Length ? aCase.Native[i] : "-";
                    if (p != n)
                    {
                        output.WriteLine($"  port   {p[..Math.Min(p.Length, 300)]}");
                        output.WriteLine($"  native {n[..Math.Min(n.Length, 300)]}");
                    }
                }
            }
        }
        output.WriteLine($"{theKind}: {aCases.Count} cases, {aTokens} tokens, {aBad} cases differ, {anIntended} intended differences");
        Assert.Equal(0, aBad);
    }

    // Plan 3.2's third level (occt-issues.md, unit 5L3's GeomFill_Coons::Init row; Kerf unit 10K2, decision 1479): a
    // rational Coons filling's weights follow its poles, where native swapped the left and right boundaries' weights;
    // only the weights (wt, swt) and the derivatives evaluated with them (sd1) differ, the poles are native's.
    private static bool IsCoonsWeights(List<string> thePort, string[] theNative)
    {
        static string Key(string theToken) => theToken[..Math.Max(0, theToken.IndexOf(':'))];
        return thePort.Count == theNative.Length
               && Enumerable.Range(0, thePort.Count).All(i => thePort[i] == theNative[i]
                                                             || (Key(thePort[i]) == Key(theNative[i]) && Key(thePort[i]) is "wt" or "swt" or "sd1")
                                                             || IsCoonsValue(thePort[i], theNative[i]));
    }

    // Decision 1815: native's CoonsAlgPatch.Value evaluates two boundaries at the wrong parameter. A "pv" token holds, per
    // point, the value, D1U, D1V and DUV, three numbers each: only the values may differ.
    private static bool IsCoonsValue(string thePort, string theNative)
    {
        if (!thePort.StartsWith("pv:", StringComparison.Ordinal) || !theNative.StartsWith("pv:", StringComparison.Ordinal))
        {
            return false;
        }

        string[] aP = thePort[3..].Split(','), aN = theNative[3..].Split(',');
        return aP.Length == aN.Length && Enumerable.Range(0, aP.Length).All(i => aP[i] == aN[i] || i % 12 < 3);
    }
}
