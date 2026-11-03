// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Parser;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using System.Globalization;
using Kerf.Kernel.DataExchange.Tests.StepGen;
using Xunit.Abstractions;

namespace Kerf.Kernel.DataExchange.Tests.GeomToStep;

/// <summary>
/// StepToGeom and GeomToStep against native OCCT's (tools/geomsteprec, unit 8D): every geometric entity of the
/// corpus's STEP files converted into Geom and Geom2d, and the constructed cases and data/occ's geometry converted
/// into STEP entities, written and converted back; each with the default factors and with inches and degrees.
/// </summary>
[Collection(ProcessWideMessages.Name)]
public sealed class GeomStepNativeTests(ITestOutputHelper output)
{
    /// <summary>A block of the recording: a file (or the cases) and, per factor set, its line count, hash and samples.</summary>
    private sealed record Block(string Name, bool ReadFail, Dictionary<string, (int NbLines, string Hash, List<string> Lines)> Tags);

    private static List<Block> Recording(string theResource)
    {
        using StreamReader aReader = StepDataDump.Resource(theResource);
        var aBlocks = new List<Block>();
        Block? aBlock = null;
        for (string? aLine = aReader.ReadLine(); aLine is not null; aLine = aReader.ReadLine())
        {
            if (aLine.StartsWith("FILE ", StringComparison.Ordinal) || aLine.StartsWith("CASES ", StringComparison.Ordinal))
            {
                aBlock = new Block(aLine.StartsWith('F') ? aLine[5..] : "(cases)", false, []);
                aBlocks.Add(aBlock);
            }
            else if (aLine == "READFAIL")
            {
                aBlocks[^1] = aBlock = aBlock! with { ReadFail = true };
            }
            else if (aLine.StartsWith("TAG ", StringComparison.Ordinal))
            {
                string[] aParts = aLine.Split(' ');
                aBlock!.Tags[aParts[1]] = (int.Parse(aParts[2], CultureInfo.InvariantCulture), aParts[3], []);
            }
            else if (aLine.StartsWith("L ", StringComparison.Ordinal))
            {
                int aSpace = aLine.IndexOf(' ', 2);
                aBlock!.Tags[aLine[2..aSpace]].Lines.Add(aLine[(aSpace + 1)..]);
            }
        }

        return aBlocks;
    }

    private static readonly (string Tag, Factors Factors)[] TheFactors =
        [("mm", GeomStepRecorder.Millimetres), ("in", GeomStepRecorder.Inches)];

    // The port's lines of a block that differs, for diagnosis (compare with geomsteprec's "full" output)
    private static string Keep(string theName, string theTag, List<string> theLines)
    {
        string aDir = Path.Combine(Path.GetTempPath(), "geomsteprec-port");
        Directory.CreateDirectory(aDir);
        string aPath = Path.Combine(aDir, theName.Replace('/', '_') + "." + theTag + ".txt");
        File.WriteAllLines(aPath, theLines);
        return aPath;
    }

    // Null when the port's lines are native's; otherwise the first difference among the recorded samples
    private static string? Compare(string theName, string theTag, (int NbLines, string Hash, List<string> Lines) theNative, List<string> thePort)
    {
        if (thePort.Count == theNative.NbLines && StepFileDump.Hash(thePort) == theNative.Hash)
        {
            return null;
        }

        string aFirst = "";
        // the samples are the first lines (cut at 200 characters) and the exceptions: the first differing first line
        for (int i = 0; i < Math.Min(10, Math.Min(theNative.Lines.Count, thePort.Count)); i++)
        {
            string aPort = thePort[i].Length > 200 && theNative.Lines.Count < theNative.NbLines ? thePort[i][..200] : thePort[i];
            if (aPort != theNative.Lines[i])
            {
                aFirst = $"\n  native: {theNative.Lines[i]}\n  port:   {aPort}";
                break;
            }
        }

        return $"{theName} {theTag}: {thePort.Count} lines (native {theNative.NbLines}), port's in {Keep(theName, theTag, thePort)}{aFirst}";
    }

    [Fact]
    public void StepToGeom_converts_the_corpus_as_native()
    {
        var aProtocol = GeomStepRecorder.TheProtocol.Value;
        if (aProtocol is null)
        {
            return; // OCCT's source not fetched
        }

        global::Kerf.Engine.Healing.ShapeAlgo.Init(); // as STEPControl_Controller does natively (MakeSurface's C0 offset surfaces)
        List<Block> aBlocks = Recording("GeomToStep.geomsteprec-s2g.txt.gz");
        Assert.Equal(338, aBlocks.Count);
        int aRun = 0, aLines = 0;
        var aDiffs = new List<string>();
        var aKinds = new Dictionary<string, int>(StringComparer.Ordinal);
        foreach (Block aBlock in aBlocks)
        {
            string? aPath = StepFileDump.CorpusPath(aBlock.Name);
            if (aPath is null)
            {
                continue;
            }

            var aModel = new StepModel();
            aModel.SetProtocol(aProtocol);
            int aStatus;
            using (FileStream aStream = File.OpenRead(aPath))
            {
                aStatus = StepFileReader.Read(aPath, aStream, aModel, aProtocol);
            }

            Assert.Equal(aBlock.ReadFail, aStatus != 0);
            if (aStatus != 0)
            {
                continue;
            }

            foreach (var (aTag, aFactors) in TheFactors)
            {
                List<string> aPort = GeomStepRecorder.ModelLines(aModel, aFactors);
                aLines += aPort.Count;
                foreach (string aLine in aPort)
                {
                    // "<num> <method> <result>": per method, the results that are geometry, null or an exception
                    string[] aParts = aLine.Split(' ', 4);
                    string aKey = aParts[1] + (aParts[2] is "null" or "EXC" or "false" ? " " + aParts[2] : "");
                    aKinds[aKey] = aKinds.GetValueOrDefault(aKey) + 1;
                }

                if (Compare(aBlock.Name, aTag, aBlock.Tags[aTag], aPort) is { } aDiff)
                {
                    aDiffs.Add(aDiff);
                }
            }

            aRun++;
        }

        output.WriteLine($"{aRun} of {aBlocks.Count} files compared ({aLines} conversions), {aDiffs.Count} different");
        output.WriteLine(string.Join(", ", aKinds.OrderBy(k => k.Key, StringComparer.Ordinal).Select(k => $"{k.Key} {k.Value}")));
        foreach (string aDiff in aDiffs.Take(30))
        {
            output.WriteLine(aDiff);
        }

        Assert.Empty(aDiffs);
    }

    [Fact]
    public void GeomToStep_converts_the_cases_and_shapes_as_native()
    {
        var aProtocol = GeomStepRecorder.TheProtocol.Value;
        if (aProtocol is null)
        {
            return; // OCCT's source not fetched
        }

        global::Kerf.Engine.Healing.ShapeAlgo.Init();
        List<Block> aBlocks = Recording("GeomToStep.geomsteprec-g2s.txt.gz");
        List<string> aCases = Cases();
        int aRun = 0, aLines = 0;
        var aDiffs = new List<string>();
        var anIntended = new SortedDictionary<string, int>(StringComparer.Ordinal);
        foreach (Block aBlock in aBlocks)
        {
            Shape aShape = new Shape();
            if (aBlock.Name != "(cases)")
            {
                string? aPath = StepFileDump.CorpusPath(aBlock.Name);
                if (aPath is null)
                {
                    continue;
                }

                bool isRead = BrepTools.Read(out aShape, aPath);
                Assert.Equal(aBlock.ReadFail, !isRead);
                if (!isRead)
                {
                    continue;
                }
            }

            foreach (var (aTag, aFactors) in TheFactors)
            {
                List<string> aPort = aBlock.Name == "(cases)"
                                         ? GeomStepRecorder.CasesLines(aCases, aProtocol, aFactors)
                                         : GeomStepRecorder.ShapeLines(aShape, aProtocol, aFactors);
                aLines += aPort.Count;
                if (aBlock.Name == "(cases)")
                {
                    // the cases are recorded in full: line by line
                    // by case (its lines start with its key, "k12 ..."), the intended differences apart
                    var aNative = aBlock.Tags[aTag].Lines;
                    var aNativeCases = aNative.GroupBy(l => l.Split(' ')[0]).ToDictionary(g => g.Key, g => g.ToList());
                    var aPortCases = aPort.GroupBy(l => l.Split(' ')[0]).ToDictionary(g => g.Key, g => g.ToList());
                    Assert.Equal(aNativeCases.Keys.Order(), aPortCases.Keys.Order());
                    foreach (var (aKey, aN) in aNativeCases)
                    {
                        List<string> aP = aPortCases[aKey];
                        if (aN.SequenceEqual(aP))
                        {
                            continue;
                        }
                        if (IntendedCaseDifference(aN, aP, aTag) is { } aReason)
                        {
                            anIntended[aReason] = anIntended.GetValueOrDefault(aReason) + 1;
                            continue;
                        }
                        int i = Enumerable.Range(0, Math.Max(aN.Count, aP.Count)).First(i => i >= aN.Count || i >= aP.Count || aN[i] != aP[i]);
                        aDiffs.Add($"cases {aTag} {aKey}:\n  native: {(i < aN.Count ? aN[i] : null)}\n  port:   {(i < aP.Count ? aP[i] : null)}");
                    }
                }
                else if (Compare(aBlock.Name, aTag, aBlock.Tags[aTag], aPort) is { } aDiff)
                {
                    // a shape's conversions are recorded by their hash: a set holding the kinds whose conversion
                    // changed on purpose (the cases above compare those line by line) is taken as intended
                    string[] aChanged = ["Geom2d_TrimmedCurve", "Geom2d_BezierCurve", "Geom_ConicalSurface", "Geom_RectangularTrimmedSurface"];
                    if (aPort.Any(l => l.Split(' ') is [_, var aType] && aChanged.Contains(aType)))
                    {
                        anIntended[$"shapes holding geometry converted otherwise ({aTag})"] =
                            anIntended.GetValueOrDefault($"shapes holding geometry converted otherwise ({aTag})") + 1;
                    }
                    else
                    {
                        aDiffs.Add(aDiff);
                    }
                }
            }

            aRun++;
        }

        output.WriteLine($"{aRun} of {aBlocks.Count} sets compared ({aCases.Count} cases; {aLines} lines), {aDiffs.Count} different");
        foreach (var (aReason, aCount) in anIntended)
        {
            output.WriteLine($"intended: {aCount} {aReason}");
        }
        foreach (string aDiff in aDiffs.Take(40))
        {
            output.WriteLine(aDiff);
        }

        Assert.True(aRun >= 2, "the cases and at least one shape");
        Assert.Empty(aDiffs);
    }

    // Plan 3.2's third level for a case (Kerf unit 10K3; GeomToStepTests checks each conversion against what reads back):
    // the reason its lines differ on purpose, or null.
    private static string? IntendedCaseDifference(List<string> theNative, List<string> thePort, string theTag)
    {
        string aType = theNative[0].Split(' ')[1];
        // occt-issues.md, GeomToStep_MakeCurve (Geom2d; decision 1507): a 2D trimmed curve is written as its basis,
        // where native threw
        if (aType == "Geom2d_TrimmedCurve" && theNative.Any(l => l.EndsWith(" curve EXC", StringComparison.Ordinal)))
        {
            return "2D trimmed curves written (decision 1507)";
        }
        // occt-issues.md, GeomToStep_MakeBoundedCurve (Geom2d; decision 1508): a rational 2D Bezier keeps its weights
        if (aType == "Geom2d_BezierCurve" && thePort.Any(l => l.Contains("RATIONAL_B_SPLINE_CURVE", StringComparison.Ordinal))
            && !theNative.Any(l => l.Contains("RATIONAL_B_SPLINE_CURVE", StringComparison.Ordinal)))
        {
            return "rational 2D Bezier curves with their weights (decision 1508)";
        }
        // occt-issues.md, GeomToStep_MakeRectangularTrimmedSurface, MakeConicalSurface (decision 1510): angles in the
        // plane angle unit, where native wrote degrees, or a cone's radians, whatever the unit
        if (aType is "Geom_ConicalSurface" or "Geom_RectangularTrimmedSurface")
        {
            return $"angles in the plane angle unit ({theTag}; decision 1510)";
        }
        return null;
    }

    /// <summary>tools/geomsteprec/cases.txt, embedded.</summary>
    internal static List<string> Cases()
    {
        using Stream aStream = typeof(GeomStepNativeTests).Assembly.GetManifestResourceStream("GeomToStep.cases.txt")
                               ?? throw new InvalidOperationException("GeomToStep.cases.txt");
        using var aReader = new StreamReader(aStream);
        var aCases = new List<string>();
        for (string? aLine = aReader.ReadLine(); aLine is not null; aLine = aReader.ReadLine())
        {
            if (aLine.Length > 0)
            {
                aCases.Add(aLine);
            }
        }

        return aCases;
    }
}
