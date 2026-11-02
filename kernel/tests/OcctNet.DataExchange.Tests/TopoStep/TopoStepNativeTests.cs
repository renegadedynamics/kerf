// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;

namespace OcctNet.DataExchange.Tests.TopoStep;

/// <summary>
/// StepToTopoDS and TopoDSToStep against native OCCT's (tools/toporec, unit 8E): every shape representation item of
/// the corpus's STEP files translated into a BRep (its text hashed), and data/occ's shapes converted into STEP entities
/// (their written lines hashed), with the default factors and with inches and degrees.
/// </summary>
[Collection(ProcessWideMessages.Name)]
public sealed class TopoStepNativeTests(ITestOutputHelper output)
{
    private sealed record Block(string Name, bool ReadFail, List<string> Lines);

    private static List<Block> Recording(string theResource)
    {
        using StreamReader aReader = StepDataDump.Resource(theResource);
        var aBlocks = new List<Block>();
        for (string? aLine = aReader.ReadLine(); aLine is not null; aLine = aReader.ReadLine())
        {
            if (aLine.StartsWith("FILE ", StringComparison.Ordinal))
            {
                aBlocks.Add(new Block(aLine[5..], false, []));
            }
            else if (aLine == "READFAIL")
            {
                aBlocks[^1] = aBlocks[^1] with { ReadFail = true };
            }
            else
            {
                aBlocks[^1].Lines.Add(aLine);
            }
        }

        return aBlocks;
    }

    /// <summary>The differences between native's lines and the port's, keyed by the line's head (its first fields).</summary>
    private static IEnumerable<(string Native, string Port)> Diffs(List<string> theNative, List<string> thePort)
    {
        for (int i = 0; i < Math.Max(theNative.Count, thePort.Count); i++)
        {
            string aN = i < theNative.Count ? theNative[i] : "(none)", aP = i < thePort.Count ? thePort[i] : "(none)";
            if (aN != aP)
            {
                yield return (aN, aP);
            }
        }
    }

    // Where the port's texts of a differing file go, for diagnosis (compare with toporec's dump)
    private static string KeepDir()
    {
        string aDir = Path.Combine(Path.GetTempPath(), "toporec-port");
        Directory.CreateDirectory(aDir);
        return aDir;
    }

    private static string Safe(string theName) => theName.Replace('/', '_').Replace(':', '_');

    /// <summary>
    /// The items whose BRep differs from native's, each attributed: none. (tr9_r1001-ec.stp's solid #12 was, until
    /// ShapeFix_EdgeProjAux put its reversed pcurves into the edge as OCCT does, decision 13401.)
    /// </summary>
    private static readonly HashSet<string> KnownDifferences = [];

    [Fact]
    public void StepToTopoDS_translates_the_corpus_as_native()
    {
        StepData_Protocol aProtocol = TopoStepRecorder.Protocol();
        ShapeAlgo.Init(); // as STEPControl_Controller does natively
        List<Block> aBlocks = Recording("TopoStep.toporec-s2t.txt.gz");
        Assert.Equal(338, aBlocks.Count);
        int aRun = 0, aItems = 0, aSame = 0, aAttributed = 0;
        var aDiffs = new List<string>();
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

            var aTexts = new Dictionary<int, string>();
            List<string> aPort = TopoStepRecorder.ModelLines(aModel, aTexts);
            aItems += aBlock.Lines.Count;
            var aFileDiffs = Diffs(aBlock.Lines, aPort).ToList();
            aSame += aBlock.Lines.Count - aFileDiffs.Count;
            foreach (var (aN, aP) in aFileDiffs)
            {
                string[] aParts = aP.Split(' ');
                if (aParts.Length > 1 && KnownDifferences.Contains(aBlock.Name + " " + aParts[1]) && aN.Split(' ')[1] == aParts[1])
                {
                    aAttributed++;
                    continue;
                }

                aDiffs.Add($"{aBlock.Name}\n  native: {aN}\n  port:   {aP}");
                if (aParts.Length > 1 && int.TryParse(aParts[1], out int aNum) && aTexts.TryGetValue(aNum, out string? aText))
                {
                    File.WriteAllText(Path.Combine(KeepDir(), Safe(aBlock.Name) + "." + aNum + ".brep"), aText);
                }
            }

            aRun++;
        }

        output.WriteLine($"{aRun} of {aBlocks.Count} files compared, {aItems} items, {aSame} identical, {aAttributed} attributed, "
                         + $"{aDiffs.Count} different");
        foreach (string aDiff in aDiffs.Take(60))
        {
            output.WriteLine(aDiff);
        }

        Assert.Empty(aDiffs);
        Assert.Equal(aRun == aBlocks.Count ? KnownDifferences.Count : aAttributed, aAttributed);
    }

    [Fact]
    public void TopoDSToStep_converts_the_shapes_as_native()
    {
        ShapeAlgo.Init();
        List<Block> aBlocks = Recording("TopoStep.toporec-t2s.txt.gz");
        int aRun = 0, aConv = 0, aSame = 0;
        var aDiffs = new List<string>();
        foreach (Block aBlock in aBlocks)
        {
            string? aPath = StepFileDump.CorpusPath(aBlock.Name);
            if (aPath is null)
            {
                continue;
            }

            bool isRead = BRepTools.Read(out TopoDS_Shape aShape, aPath);
            Assert.Equal(aBlock.ReadFail, !isRead);
            if (!isRead)
            {
                continue;
            }

            var aDump = new List<string>();
            List<string> aPort = TopoStepRecorder.ShapeLines(aShape, aDump);
            aConv += aBlock.Lines.Count;
            var aFileDiffs = Diffs(aBlock.Lines, aPort).ToList();
            aSame += aBlock.Lines.Count - aFileDiffs.Count;
            foreach (var (aN, aP) in aFileDiffs)
            {
                aDiffs.Add($"{aBlock.Name}\n  native: {aN}\n  port:   {aP}");
            }

            if (aFileDiffs.Count > 0)
            {
                File.WriteAllLines(Path.Combine(KeepDir(), Safe(aBlock.Name) + ".txt"), aDump);
            }

            aRun++;
        }

        output.WriteLine($"{aRun} of {aBlocks.Count} shapes compared, {aConv} conversions, {aSame} identical, {aDiffs.Count} different");
        foreach (string aDiff in aDiffs.Take(60))
        {
            output.WriteLine(aDiff);
        }

        Assert.True(aRun >= 30, "data/occ's shapes");
        Assert.Empty(aDiffs);
    }
}
