// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using OcctNet.DataExchange.Tests.StepGen;
using Xunit.Abstractions;

namespace OcctNet.DataExchange.Tests.GeomToStep;

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

    private static readonly (string Tag, StepData_Factors Factors)[] TheFactors =
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

        ShapeAlgo.Init(); // as STEPControl_Controller does natively (MakeSurface's C0 offset surfaces)
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

            var aModel = new StepData_StepModel();
            aModel.SetProtocol(aProtocol);
            int aStatus;
            using (FileStream aStream = File.OpenRead(aPath))
            {
                aStatus = StepFile_Read.Read(aPath, aStream, aModel, aProtocol);
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

        ShapeAlgo.Init();
        List<Block> aBlocks = Recording("GeomToStep.geomsteprec-g2s.txt.gz");
        List<string> aCases = Cases();
        int aRun = 0, aLines = 0;
        var aDiffs = new List<string>();
        foreach (Block aBlock in aBlocks)
        {
            TopoDS_Shape aShape = new TopoDS_Shape();
            if (aBlock.Name != "(cases)")
            {
                string? aPath = StepFileDump.CorpusPath(aBlock.Name);
                if (aPath is null)
                {
                    continue;
                }

                bool isRead = BRepTools.Read(out aShape, aPath);
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
                    var aNative = aBlock.Tags[aTag].Lines;
                    for (int i = 0; i < Math.Max(aNative.Count, aPort.Count); i++)
                    {
                        string? aN = i < aNative.Count ? aNative[i] : null, aP = i < aPort.Count ? aPort[i] : null;
                        if (aN != aP)
                        {
                            aDiffs.Add($"cases {aTag} line {i + 1}:\n  native: {aN}\n  port:   {aP}");
                        }
                    }
                }
                else if (Compare(aBlock.Name, aTag, aBlock.Tags[aTag], aPort) is { } aDiff)
                {
                    aDiffs.Add(aDiff);
                }
            }

            aRun++;
        }

        output.WriteLine($"{aRun} of {aBlocks.Count} sets compared ({aCases.Count} cases; {aLines} lines), {aDiffs.Count} different");
        foreach (string aDiff in aDiffs.Take(40))
        {
            output.WriteLine(aDiff);
        }

        Assert.True(aRun >= 2, "the cases and at least one shape");
        Assert.Empty(aDiffs);
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
