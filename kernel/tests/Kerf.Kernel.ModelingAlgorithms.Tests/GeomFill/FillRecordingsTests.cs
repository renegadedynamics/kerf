// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

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
        int aTokens = 0, aBad = 0;
        foreach (FillRecordings.Case aCase in aCases)
        {
            List<string> aPort = FillRecordings.Run(aCase);
            aTokens += aCase.Native.Length;
            bool aSame = aPort.Count == aCase.Native.Length;
            for (int i = 0; aSame && i < aPort.Count; i++)
            {
                aSame = aPort[i] == aCase.Native[i];
            }
            if (!aSame)
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
        output.WriteLine($"{theKind}: {aCases.Count} cases, {aTokens} tokens, {aBad} cases differ");
        Assert.Equal(0, aBad);
    }
}
