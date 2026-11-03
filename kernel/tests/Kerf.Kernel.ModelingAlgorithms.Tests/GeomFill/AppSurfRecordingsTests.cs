// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The port against native OCCT's surfaces through sections recorded by tools/appsurfrec (<see cref="AppSurfRecordings"/>):
/// GeomFill_Profiler, GeomFill_Generator and the four ways of GeomFill_AppSurf, every token of every case, bit for bit.
/// </summary>
public class AppSurfRecordingsTests(ITestOutputHelper output)
{
    public static IEnumerable<object[]> Kinds() =>
        new[] { "GEN", "APP", "APPSP", "APPSM", "APPMAXP" }.Select(k => new object[] { k });

    [Theory]
    [MemberData(nameof(Kinds))]
    public void Port_matches_native_bit_for_bit(string theKind)
    {
        List<AppSurfRecordings.Case> aCases = AppSurfRecordings.Cases.Where(c => c.Kind == theKind).ToList();
        Assert.NotEmpty(aCases);
        int aTokens = 0, aBad = 0, aDone = 0, anAttributed = 0;
        foreach (AppSurfRecordings.Case aCase in aCases)
        {
            List<string> aPort = AppSurfRecordings.Run(aCase);
            aTokens += aCase.Native.Length;
            aDone += aCase.Native.Contains("done:1") || aCase.Native.Any(t => t.StartsWith("sdeg:", StringComparison.Ordinal)) ? 1 : 0;
            bool aSame = aPort.Count == aCase.Native.Length;
            for (int i = 0; aSame && i < aPort.Count; i++)
            {
                aSame = aPort[i] == aCase.Native[i];
            }
            if (!aSame && IsTwoPointSmoothing(aCase, aPort))
            {
                anAttributed++;
                continue;
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
        output.WriteLine($"{theKind}: {aCases.Count} cases ({aDone} with a surface), {aTokens} tokens, {aBad} cases differ, "
                         + $"{anAttributed} attributed");
        Assert.Equal(0, aBad);
    }

    // The smoothing of two sections: AppDef_Variational::InitCriterionEstimations calls EstTangent(myFirstPoint + 2)
    // on a line of two points (occt-issues.md, AppDef). Native reads past the points and throws Standard_OutOfRange
    // further on; the port's view of the points checks its range first and throws Standard_RangeError (its base).
    private static bool IsTwoPointSmoothing(AppSurfRecordings.Case theCase, List<string> thePort)
    {
        int n = theCase.Native.Length;
        if (theCase.Kind != "APPSM" || theCase.Inputs[0] != 2 || thePort.Count != n
            || theCase.Native[n - 1] != "exc:!Standard_OutOfRange" || thePort[n - 1] != "exc:!Standard_RangeError")
        {
            return false;
        }
        for (int i = 0; i < n - 1; i++)
        {
            if (thePort[i] != theCase.Native[i])
            {
                return false;
            }
        }
        return true;
    }
}
