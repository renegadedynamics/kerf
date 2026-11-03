// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Text;
using Xunit.Abstractions;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The fair curves against native OCCT's results recorded by tools/fairrec (<see cref="FairCurveRecordings"/>):
/// FairCurve_Batten and FairCurve_MinimalVariation on random points, angles, constraint orders, slidings, heights,
/// slopes, physical ratios and curvatures, computed and then edited and computed again; Draw's battencurve and
/// minvarcurve with random sequences of its editing commands; and the energies' values, gradients and hessians on
/// random poles. Every result bit for bit: the returned flags and analysis codes, the getters, the sliding of
/// reference, the curves' poles, knots and multiplicities, exceptions. NaNs compare equal whatever their payload.
/// </summary>
public class FairCurve_NativeTests(ITestOutputHelper theOutput)
{
    public static IEnumerable<object[]> Kinds() => FairCurveRecordings.Kinds();

    private static bool SameValue(string theA, string theB)
    {
        if (theA == theB)
        {
            return true;
        }
        string[] a = theA.Split(',');
        string[] b = theB.Split(',');
        if (a.Length != b.Length)
        {
            return false;
        }
        for (int i = 0; i < a.Length; i++)
        {
            if (a[i] == b[i])
            {
                continue;
            }
            if (!long.TryParse(a[i], NumberStyles.HexNumber, CultureInfo.InvariantCulture, out long x)
                || !long.TryParse(b[i], NumberStyles.HexNumber, CultureInfo.InvariantCulture, out long y)
                || !double.IsNaN(BitConverter.Int64BitsToDouble(x)) || !double.IsNaN(BitConverter.Int64BitsToDouble(y)))
            {
                return false;
            }
        }
        return true;
    }

    /// <summary>The first token that differs, or null.</summary>
    private static string? Diff(string[] theNative, string[] thePort)
    {
        for (int i = 0; i < Math.Max(theNative.Length, thePort.Length); i++)
        {
            string n = i < theNative.Length ? theNative[i] : "<none>";
            string p = i < thePort.Length ? thePort[i] : "<none>";
            int kn = n.IndexOf(':');
            int kp = p.IndexOf(':');
            if (kn < 0 || kp < 0 || n[..kn] != p[..kp] || !SameValue(n[(kn + 1)..], p[(kp + 1)..]))
            {
                return $"token {i}: native {Shorten(n)} port {Shorten(p)}";
            }
        }
        return null;
    }

    private static string Shorten(string theTok) => theTok.Length > 400 ? theTok[..400] + "..." : theTok;

    [Theory]
    [MemberData(nameof(Kinds))]
    public void Port_MatchesNative(string theKind)
    {
        StringBuilder aReport = new();
        int aNbDiff = 0, aNbCases = 0;
        Dictionary<string, int> aCodes = [];
        foreach (FairCurveRecordings.Case aCase in FairCurveRecordings.Cases.Where(c => c.Kind == theKind))
        {
            aNbCases++;
            string[] aPort = FairCurveRecordings.Run(aCase);
            foreach (string aTok in aCase.Native)
            {
                // the outcomes of the computations (c0, c1, c2, opN) and the exceptions, for the report
                int k = aTok.IndexOf(':');
                string aName = aTok[..k];
                if (aName is "c0" or "c1" or "c2" || aName.StartsWith("op", StringComparison.Ordinal) && !aName.Contains('.')
                    || aTok.Contains('!'))
                {
                    string aKey = aTok.Contains('!') ? aTok[(aTok.IndexOf('!'))..] : aTok[(k + 1)..];
                    aCodes[aKey] = aCodes.GetValueOrDefault(aKey) + 1;
                }
            }
            string? aDiff = Diff(aCase.Native, aPort);
            if (aDiff is not null)
            {
                aNbDiff++;
                aReport.AppendLine(CultureInfo.InvariantCulture,
                                   $"{theKind} [{string.Join(' ', aCase.Inputs.Select(v => v.ToString("R", CultureInfo.InvariantCulture)))}] {aDiff}");
            }
        }
        theOutput.WriteLine($"{theKind}: {aNbCases} cases, {aNbDiff} differ; outcomes "
                            + string.Join(", ", aCodes.OrderBy(p => p.Key, StringComparer.Ordinal).Select(p => $"{p.Key} x{p.Value}")));
        theOutput.WriteLine(aReport.ToString());
        Assert.True(aNbDiff == 0, aReport.ToString());
    }
}
