// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using System.IO.Compression;
using System.Text;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The recordings of tools/dsrec: for each case (a native fillet or chamfer on a box, cylinder, cone, sphere, torus or
/// Boolean result), the input shape as BRep text and native OCCT's legacy data structure as
/// <c>ChFi3d_Builder::Compute</c> hands it to <c>TopOpeBRepBuild_HBuilder::Perform</c>, in
/// <see cref="TextDump"/>'s format.
/// </summary>
/// <remarks>
/// For units 7N1 and 7N2: run the port's builder on <see cref="DsCase.Input"/> with the case's edges and radii (see
/// tools/dsrec/main.cpp), write its data structure with <c>TopOpeBRepDS_TextDump.Write(DS, input)</c> and compare
/// with <see cref="DsCase.DsText"/>. For unit 7R: <c>TopOpeBRepDS_TextDump.Read(DsText, input)</c> gives native's data
/// structure to rebuild from.
/// </remarks>
internal static class DsRecordings
{
    private static readonly Lazy<Dictionary<string, DsCase>> theCases = new(Load);

    /// <summary>The cases by name.</summary>
    internal static Dictionary<string, DsCase> Cases => theCases.Value;

    private static Dictionary<string, DsCase> Load()
    {
        using Stream aRes = typeof(DsRecordings).Assembly.GetManifestResourceStream("TopOpeBRepDS.dsrec.txt.gz")
                            ?? throw new InvalidOperationException("TopOpeBRepDS.dsrec.txt.gz");
        using GZipStream aGz = new(aRes, CompressionMode.Decompress);
        using StreamReader aReader = new(aGz, Encoding.ASCII);
        string[] aLines = aReader.ReadToEnd().Replace("\r\n", "\n").Split('\n');

        Dictionary<string, DsCase> aCases = new();
        int k = 0;
        while (k < aLines.Length && aLines[k].StartsWith("case ", StringComparison.Ordinal))
        {
            string aName = aLines[k++][5..];
            string[] aStatus = aLines[k++].Split(' ');
            bool aDone = aStatus[2] == "1";
            int aCalls = int.Parse(aStatus[4], System.Globalization.CultureInfo.InvariantCulture);
            bool anUnchanged = aCalls > 0 && aStatus[6] == "1";
            string aBRep = "", aDs = "";
            // the input is recorded for every case since unit 7N2, the data structure when Perform was reached
            if (k < aLines.Length && aLines[k].StartsWith("brep ", StringComparison.Ordinal))
            {
                int nB = int.Parse(aLines[k++][5..], System.Globalization.CultureInfo.InvariantCulture);
                aBRep = string.Join("\n", aLines, k, nB) + "\n";
                k += nB;
            }
            if (k < aLines.Length && aLines[k].StartsWith("ds ", StringComparison.Ordinal))
            {
                int nD = int.Parse(aLines[k++][3..], System.Globalization.CultureInfo.InvariantCulture);
                aDs = nD == 0 ? "" : string.Join("\n", aLines, k, nD) + "\n";
                k += nD;
            }
            if (aLines[k++] != "end-case")
            {
                throw new InvalidDataException("dsrec: end-case expected after " + aName);
            }
            aCases[aName] = new DsCase(aName, aDone, aCalls, anUnchanged, aBRep, aDs, aStatus);
        }
        return aCases;
    }

    /// <summary>
    /// The differences between two dumps, token by token: the words must be equal; two numbers may differ by
    /// <paramref name="theTol"/> relative to the larger (at least 1), or by a whole number of periods 2π (a periodic
    /// curve's trim). Each difference is "line: expected | actual"; the comparison stops at the first ten.
    /// </summary>
    internal static List<string> Differences(string theExpected, string theActual, double theTol)
    {
        List<string> aDiffs = [];
        string[] e = theExpected.Split('\n'), a = theActual.Split('\n');
        if (e.Length != a.Length)
        {
            aDiffs.Add("line counts " + e.Length + " | " + a.Length);
        }
        for (int i = 0; i < Math.Min(e.Length, a.Length) && aDiffs.Count < 10; i++)
        {
            if (e[i] == a[i])
            {
                continue;
            }
            string[] te = e[i].Split(' '), ta = a[i].Split(' ');
            bool aSame = te.Length == ta.Length;
            for (int k = 0; aSame && k < te.Length; k++)
            {
                aSame = te[k] == ta[k] || NumbersClose(te[k], ta[k], theTol);
            }
            if (!aSame)
            {
                aDiffs.Add((i + 1) + ": " + e[i] + " | " + a[i]);
            }
        }
        return aDiffs;
    }

    private static bool NumbersClose(string theE, string theA, double theTol)
    {
        System.Globalization.CultureInfo c = System.Globalization.CultureInfo.InvariantCulture;
        if (!double.TryParse(theE, System.Globalization.NumberStyles.Float, c, out double x)
            || !double.TryParse(theA, System.Globalization.NumberStyles.Float, c, out double y))
        {
            return false;
        }
        double aScale = Math.Max(1.0, Math.Max(Math.Abs(x), Math.Abs(y)));
        double d = Math.Abs(x - y);
        if (d <= theTol * aScale)
        {
            return true;
        }
        double aPeriods = Math.Round(d / (2.0 * Math.PI));
        return aPeriods >= 1.0 && Math.Abs(d - aPeriods * 2.0 * Math.PI) <= theTol * aScale * 10.0;
    }
}

/// <summary>A case of <see cref="DsRecordings"/>.</summary>
/// <param name="Name">The case's name.</param>
/// <param name="Done">Whether native's maker was done.</param>
/// <param name="PerformCalls">How many times the builder's Perform ran (0: the data structure was never complete).</param>
/// <param name="UnchangedByPerform">Whether the dump after Build equals the dump before Perform.</param>
/// <param name="BRepText">The input shape, as BRepTools::Write writes it.</param>
/// <param name="DsText">The data structure before Perform.</param>
/// <param name="StatusTokens">The case's status line (unit 7N2 adds a fillet's diagnostics: "contours n faulty k i..
/// statuses s.. faultyvertices v hasresult h"; "exception" when Build threw).</param>
internal sealed record DsCase(string Name, bool Done, int PerformCalls, bool UnchangedByPerform, string BRepText, string DsText,
                             string[] StatusTokens)
{
    /// <summary>The integers following <paramref name="theKey"/> in the status line, up to the next word.</summary>
    internal List<int> StatusInts(string theKey)
    {
        List<int> r = [];
        int i = Array.IndexOf(StatusTokens, theKey);
        if (i < 0)
        {
            return r;
        }
        for (i++; i < StatusTokens.Length && int.TryParse(StatusTokens[i], out int v); i++)
        {
            r.Add(v);
        }
        return r;
    }

    /// <summary>The input shape.</summary>
    internal Shape Input
    {
        get
        {
            BrepTools.Read(out Shape aShape, new StringReader(BRepText));
            return aShape;
        }
    }
}
