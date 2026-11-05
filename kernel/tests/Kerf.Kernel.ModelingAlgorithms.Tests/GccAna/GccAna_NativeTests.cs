// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Constraints;
using Kerf.Geometry;
using System.Globalization;
using System.Text;
using Xunit.Abstractions;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

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
    /// The constructors Kerf corrected (docs/occt-issues.md's triage, batch K4), with the decision: their cases compare
    /// at plan 3.2's third level. A case whose results differ from native's is an intended difference when the port's
    /// solutions obey every law (<see cref="GccLaws"/>) and include each of native's that obeys them; native's other
    /// solutions are the bugs fixed.
    /// </summary>
    private static readonly Dictionary<string, string> theCorrected = new()
    {
        ["3Tan.CCC"] = "decision 1540", ["3Tan.CCL"] = "decision 1540", ["3Tan.CLL"] = "decision 1540",
        ["3Tan.LLL"] = "decision 1540", ["3Tan.CCP"] = "decision 1540", ["3Tan.CLP"] = "decision 1540",
        ["3Tan.LLP"] = "decision 1540",
        ["2TanOn.CCl"] = "decision 1541", ["2TanOn.CLl"] = "decision 1541", ["2TanOn.LLl"] = "decision 1541",
        ["2TanOn.CPl"] = "decision 1541", ["2TanOn.LPl"] = "decision 1541", ["2TanOn.PPl"] = "decision 1541",
        ["2TanOn.CCc"] = "decision 1541", ["2TanOn.CLc"] = "decision 1541", ["2TanOn.CPc"] = "decision 1541",
        ["2TanOn.LLc"] = "decision 1541", ["2TanOn.LPc"] = "decision 1541", ["2TanOn.PPc"] = "decision 1541",
        ["TanOnRad.Cl"] = "decision 1543", ["TanOnRad.Ll"] = "decision 1543", ["TanOnRad.Pl"] = "decision 1543",
        ["TanOnRad.Cc"] = "decision 1543", ["TanOnRad.Lc"] = "decision 1543", ["TanOnRad.Pc"] = "decision 1543",
        ["Lin2d2Tan.CC"] = "decision 1544", ["Lin2d2Tan.CP"] = "decision 1544", ["TanObl.P"] = "decision 1544",
        ["TanObl.C"] = "decision 1544", ["TanPer.Pc"] = "decision 1544", ["TanPer.Cl"] = "decision 1544",
        ["TanPer.Cc"] = "decision 1544", ["TanPer.Pl"] = "decision 1544",
        ["2TanRad.CC"] = "decision 1542", ["2TanRad.LL"] = "decision 1542", ["2TanRad.CP"] = "decision 1542",
    };

    /// <summary>The law tolerance for a case solved with the given tolerance.</summary>
    private static double LawTolerance(double theTol) => Math.Max(1.0e-6, 10 * theTol);

    /// <summary>True if the port's results obey the laws and keep every solution of native's that does.</summary>
    internal static bool IsIntendedDifference(GccAnaRecordings.Case theCase, out string theWhy)
    {
        theWhy = "";
        if (GccAnaRecordings.LineLaws(theCase) is GccAnaRecordings.LineLawCase aLineLaw)
        {
            return IsIntendedLineDifference(theCase, aLineLaw, out theWhy);
        }
        if (GccAnaRecordings.Laws(theCase) is not GccAnaRecordings.LawCase aLaw)
        {
            theWhy = "no laws for " + theCase.Ctor;
            return false;
        }
        double aTol = LawTolerance(aLaw.Tolerance);
        List<string> aBroken = GccLaws.Check(aLaw.Solve, aLaw.Args, aLaw.Locus, aLaw.Radius, aTol);
        if (aBroken.Count > 0)
        {
            theWhy = "the port breaks a law: " + aBroken[0];
            return false;
        }
        List<Circle2D> aPort = [];
        try
        {
            GccLaws.Circles S = aLaw.Solve();
            aPort = Enumerable.Range(1, S.Count()).Select(S.Solution).ToList();
        }
        catch (BadQualifierException)
        {
            // a qualifier no argument may have, which the laws accept
        }
        foreach (Circle2D aNative in GccAnaRecordings.NativeCircles(theCase))
        {
            if (GccLaws.IsSolution(aNative, aLaw.Args, aLaw.Locus, aLaw.Radius, aTol)
                && !aPort.Any(p => p.Location.DistanceTo(aNative.Location) <= aTol && Math.Abs(p.Radius - aNative.Radius) <= aTol))
            {
                theWhy = FormattableString.Invariant($"native's solution ({aNative.Location.X}, {aNative.Location.Y}) r {aNative.Radius} is lost");
                return false;
            }
        }
        return true;
    }

    /// <summary>As <see cref="IsIntendedDifference"/> for a line solver: lines compare as lines, whatever their sense.</summary>
    private static bool IsIntendedLineDifference(GccAnaRecordings.Case theCase, GccAnaRecordings.LineLawCase theLaw, out string theWhy)
    {
        theWhy = "";
        double aTol = LawTolerance(theLaw.Tolerance);
        List<string> aBroken = GccLaws.CheckLines(theLaw.Solve, theLaw.Args, theLaw.Direction, theLaw.Second, aTol);
        if (aBroken.Count > 0)
        {
            theWhy = "the port breaks a law: " + aBroken[0];
            return false;
        }
        List<Line2D> aPort = [];
        try
        {
            GccLaws.Lines S = theLaw.Solve();
            aPort = Enumerable.Range(1, S.Count()).Select(S.Solution).ToList();
        }
        catch (BadQualifierException)
        {
            // a qualifier no argument may have, which the laws accept
        }
        foreach (Line2D aNative in GccAnaRecordings.NativeLines(theCase))
        {
            bool isSolution = theLaw.Args.All(a => GccLaws.Touches(aNative, a, aTol)) && theLaw.Direction?.Invoke(aNative) is null;
            if (isSolution && !aPort.Any(p => Math.Abs(p.Direction.XY() ^ aNative.Direction.XY()) <= aTol && p.DistanceTo(aNative.Location) <= aTol))
            {
                theWhy = FormattableString.Invariant($"native's solution through ({aNative.Location.X}, {aNative.Location.Y}) is lost");
                return false;
            }
        }
        return true;
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
        int aNbDiff = 0, aNbCases = 0, aNbIntended = 0;
        string[] aMasked = theUninitialised.GetValueOrDefault(theCtor, []);
        foreach (GccAnaRecordings.Case aCase in GccAnaRecordings.Cases.Where(c => c.Ctor == theCtor))
        {
            aNbCases++;
            string[] aPort = GccAnaRecordings.Run(aCase);
            string? aDiff = Diff(aCase.Native, aPort, aMasked);
            string aWhy = "";
            if (aDiff is not null && theCorrected.ContainsKey(theCtor) && IsIntendedDifference(aCase, out aWhy))
            {
                aNbIntended++;
                continue;
            }
            if (aDiff is not null)
            {
                aNbDiff++;
                aReport.AppendLine(CultureInfo.InvariantCulture,
                                   $"{theCtor} [{string.Join(' ', aCase.Inputs.Select(v => v.ToString("R", CultureInfo.InvariantCulture)))}] {aDiff} {aWhy}");
            }
        }
        theOutput.WriteLine($"{theCtor}: {aNbCases} cases, {aNbIntended} intended differences"
                            + (theCorrected.TryGetValue(theCtor, out string? aReason) ? $" ({aReason})" : "") + $", {aNbDiff} differ"
                            + (aMasked.Length > 0 ? $" (masked: {string.Join(' ', aMasked)})" : ""));
        theOutput.WriteLine(aReport.ToString());
        Assert.True(aNbDiff == 0, aReport.ToString());
    }
}
