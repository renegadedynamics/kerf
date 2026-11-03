// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Constraints;
using System.Globalization;
using System.Text;
using Xunit.Abstractions;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// The GccAna solvers against native OCCT's results recorded by tools/gccrec (<see cref="GccAnaRecordings"/>): every
/// constructor of every class on 120 random configurations, every result bit for bit (solution count, solutions,
/// qualifiers, tangency, centre and intersection data, IsTheSame, exceptions). NaNs compare equal whatever their
/// payload. The results OCCT leaves uninitialised (docs/occt-issues.md, unit 5K1) are masked: native returns whatever
/// memory holds, the port zeros.
/// </summary>
public class GccAna_NativeTests(ITestOutputHelper theOutput)
{
    public static IEnumerable<object[]> CtorNames() => GccAnaRecordings.CtorNames();

    /// <summary>
    /// The accessors whose values OCCT leaves unset on some paths, per constructor: native reads uninitialised array
    /// elements, the port .NET's zeros (decision 781).
    /// </summary>
    private static readonly Dictionary<string, string[]> theUninitialised = new()
    {
        // pararg1 written at the bisector's index, not NbrSol; pararg2 unset on the boundary path
        ["2TanOn.CPl"] = ["t1", "t2"],
        // TheSame2(1) unset when the solution is the first circle but not the second
        ["2TanRad.CC"] = ["t2", "s2"],
        // the second solution's qualifiers written into the first's
        ["Lin2d2Tan.CC"] = ["q"],
        // the loop writes solution 1's data for every solution
        ["TanOnRad.Pc"] = ["q", "s1", "t1", "c3"],
        // qualifier1 never set
        ["TanPer.Pl"] = ["q"],
        ["TanPer.Pc"] = ["q"],
        ["TanPer.Cl"] = ["q"],
        ["TanPer.Cc"] = ["q"],
    };

    /// <summary>
    /// Circ2d3Tan's three lines with the third unqualified and the first two enclosed and unqualified (either way
    /// round) filter the solutions with cote3(2), which OCCT never sets (it sets cote3(1) twice).
    /// </summary>
    private static bool IsUninitialisedCase(GccAnaRecordings.Case theCase)
    {
        if (theCase.Ctor != "3Tan.LLL")
        {
            return false;
        }
        Qualifier q1 = (Qualifier)(int)theCase.Inputs[4];
        Qualifier q2 = (Qualifier)(int)theCase.Inputs[9];
        Qualifier q3 = (Qualifier)(int)theCase.Inputs[14];
        return q3 == Qualifier.Unqualified
               && ((q1 == Qualifier.Enclosed && q2 == Qualifier.Unqualified)
                   || (q1 == Qualifier.Unqualified && q2 == Qualifier.Enclosed));
    }

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
    internal static string? Diff(string[] theNative, string[] thePort, string[] theMasked)
    {
        for (int i = 0; i < Math.Max(theNative.Length, thePort.Length); i++)
        {
            string n = i < theNative.Length ? theNative[i] : "<none>";
            string p = i < thePort.Length ? thePort[i] : "<none>";
            int kn = n.IndexOf(':');
            int kp = p.IndexOf(':');
            if (kn >= 0 && kp >= 0 && n[..kn] == p[..kp] && theMasked.Contains(n[..kn]))
            {
                continue;
            }
            if (kn < 0 || kp < 0 || n[..kn] != p[..kp] || !SameValue(n[(kn + 1)..], p[(kp + 1)..]))
            {
                return $"token {i}: native {n} port {p}";
            }
        }
        return null;
    }

    [Theory]
    [MemberData(nameof(CtorNames))]
    public void Port_MatchesNative(string theCtor)
    {
        StringBuilder aReport = new();
        int aNbDiff = 0, aNbCases = 0, aNbSkipped = 0;
        string[] aMasked = theUninitialised.GetValueOrDefault(theCtor, []);
        foreach (GccAnaRecordings.Case aCase in GccAnaRecordings.Cases.Where(c => c.Ctor == theCtor))
        {
            aNbCases++;
            string[] aPort = GccAnaRecordings.Run(aCase);
            if (IsUninitialisedCase(aCase))
            {
                aNbSkipped++;
                continue;
            }
            string? aDiff = Diff(aCase.Native, aPort, aMasked);
            if (aDiff is not null)
            {
                aNbDiff++;
                aReport.AppendLine(CultureInfo.InvariantCulture,
                                   $"{theCtor} [{string.Join(' ', aCase.Inputs.Select(v => v.ToString("R", CultureInfo.InvariantCulture)))}] {aDiff}");
            }
        }
        theOutput.WriteLine($"{theCtor}: {aNbCases} cases, {aNbSkipped} skipped, {aNbDiff} differ"
                            + (aMasked.Length > 0 ? $" (masked: {string.Join(' ', aMasked)})" : ""));
        theOutput.WriteLine(aReport.ToString());
        Assert.True(aNbDiff == 0, aReport.ToString());
    }
}
