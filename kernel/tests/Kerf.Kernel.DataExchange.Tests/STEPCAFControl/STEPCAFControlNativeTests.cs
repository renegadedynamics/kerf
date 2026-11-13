// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Documents;
using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Exchange.Step;
using Kerf.Engine.Exchange.Step.Caf;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using System.Security.Cryptography;
using System.Text;
using Kerf.Kernel.DataExchange.Tests.StepGen;
using Xunit.Abstractions;

namespace Kerf.Kernel.DataExchange.Tests.STEPCAFControl;

/// <summary>
/// STEPCAFControl's reader and writer over XdeDoc_Document against native OCCT's over XCAF documents, through
/// tools/stepcafrec's recordings (unit 8G2): the corpus's STEP files read and the documents dumped (labels, shapes,
/// names, colours, visibility, layers, validation properties, the length unit); documents built from data/occ's shapes
/// and three assemblies, and the corpus's files read, written (the STEP text hashed with its time, author, host and
/// per-process counters masked, as stepctlrec masks it).
/// </summary>
[Collection(ProcessWideMessages.Name)]
public sealed class STEPCAFControlNativeTests(ITestOutputHelper output)
{
    private sealed record Block(string Name, List<string> Lines);

    private static List<Block> Recording(string theResource, string thePrefix)
    {
        // the recorder writes names in UTF-8
        Stream aRes = typeof(STEPCAFControlNativeTests).Assembly.GetManifestResourceStream(theResource)
                      ?? throw new InvalidOperationException(theResource);
        using var aReader = new StreamReader(new System.IO.Compression.GZipStream(aRes, System.IO.Compression.CompressionMode.Decompress), Encoding.UTF8);
        var aBlocks = new List<Block>();
        for (string? aLine = aReader.ReadLine(); aLine is not null; aLine = aReader.ReadLine())
        {
            if (aLine.StartsWith(thePrefix, StringComparison.Ordinal))
            {
                aBlocks.Add(new Block(aLine, []));
            }
            else
            {
                aBlocks[^1].Lines.Add(aLine);
            }
        }
        return aBlocks;
    }

    /// <summary>What the recorder writes for one file read.</summary>
    internal static List<string> ReadLines(string thePath)
    {
        var aLines = new List<string>();
        try
        {
            var aDoc = new ExchangeDocument { AutoNaming = true };
            var aReader = new CafReader();
            ReturnStatus aStatus = aReader.ReadFile(thePath);
            bool isOk = aStatus == ReturnStatus.RetDone && aReader.Transfer(aDoc);
            aLines.Add($"R {(int)aStatus} {(isOk ? 1 : 0)}");
            if (isOk)
            {
                aLines.AddRange(XdeDocDump.Dump(aDoc));
            }
        }
        catch (Exception anExc) when (Refusal.IsFailure(anExc))
        {
            aLines.Add("EXC " + Refusal.Unwrapped(anExc).Message);
        }
        return aLines;
    }

    [Fact]
    public void ReadsTheCorpusAsNative()
    {
        // The healing at poles as OCCT 8.0.1 has it, so that the native recordings replay (Kerf decisions 2424, 2425)
        using IDisposable aNativeHealing = global::Kerf.Engine.Healing.Analysis.NativeHealingAtPoles.Scope();
        List<string> aFiles = StepRoundTripTests.CorpusFiles();
        if (aFiles.Count == 0)
        {
            return; // OCCT's dataset not fetched
        }
        List<Block> aBlocks = Recording("STEPCAFControl.stepcafrec-read.txt.gz", "F ");
        Assert.Equal(aFiles.Count, aBlocks.Count);
        int aSame = 0;
        var aDifferent = new List<string>();
        string? aDump = Environment.GetEnvironmentVariable("KERF_STEPCAF_DUMP");
        object aLock = new();
        Parallel.For(0, aFiles.Count, new ParallelOptions { MaxDegreeOfParallelism = Environment.ProcessorCount / 2 + 1 }, i =>
        {
            Assert.EndsWith("/" + Path.GetFileName(aFiles[i]), aBlocks[i].Name, StringComparison.Ordinal);
            List<string> aPort = ReadLines(aFiles[i]);
            lock (aLock)
            {
                if (aPort.SequenceEqual(aBlocks[i].Lines, StringComparer.Ordinal))
                {
                    aSame++;
                    return;
                }
                if (aDump is not null)
                {
                    File.WriteAllLines(Path.Combine(aDump, Path.GetFileName(aFiles[i]) + ".native.txt"), aBlocks[i].Lines);
                    File.WriteAllLines(Path.Combine(aDump, Path.GetFileName(aFiles[i]) + ".port.txt"), aPort);
                }
                int aIndex = 0;
                while (aIndex < aPort.Count && aIndex < aBlocks[i].Lines.Count && aPort[aIndex] == aBlocks[i].Lines[aIndex])
                {
                    aIndex++;
                }
                string aNative = aIndex < aBlocks[i].Lines.Count ? aBlocks[i].Lines[aIndex] : "(end)";
                string aPortLine = aIndex < aPort.Count ? aPort[aIndex] : "(end)";
                aDifferent.Add($"{Path.GetFileName(aFiles[i])}: line {aIndex} native '{aNative}' port '{aPortLine}'");
            }
        });
        output.WriteLine($"{aFiles.Count} files read: {aSame} as native");
        foreach (string aLine in aDifferent.Order(StringComparer.Ordinal))
        {
            output.WriteLine(aLine);
        }
        Assert.DoesNotContain(aDifferent, d => !KnownReadDifferences.Contains(d.Split(':')[0]));
        Assert.Equal(aFiles.Count - KnownReadDifferences.Count, aSame);
    }

    // Files whose documents differ from native's, each attributed (decision 1279): native's GD&T reading
    // (ReadGDTs, not ported) adds sub-shape labels for the faces and edges its tolerances and datums refer to
    private static readonly HashSet<string> KnownReadDifferences = new(StringComparer.Ordinal)
    {
        "bug29241.stp", "bug32745_pmi1.stp", "bug33661_nist_ftc_08_asme1_ap242-e1-tg.stp", "nist_ftc_08_asme1_ap242-2.stp",
        "trj12_gd1-id-214.stp",
        // a product name whose decoding reads past its end in native (bytes of other memory in the name: occt-issues.md)
        "bug31670_russian.stp",
        // a sphere's and a torus's patches whose outer boundary curves run clockwise, which native reads as the rest of
        // each surface, or the far side of a rod's hemispherical end (decision 2446, OCCT #1652)
        "trj4_k1_geo-pe-203.stp", "trj4_k1_geo-pe-214.stp",
    };

    private static string WriteLine(ExchangeDocument theDoc, string theName, string theSchema, string? theDump)
    {
        string aHead = $"W {SafeName(theName)} {theSchema} ";
        try
        {
            var aWriter = new CafWriter();
            aWriter.Writer().WS()!.Statics().SetCVal("write.step.schema", theSchema);
            if (!aWriter.Transfer(theDoc, ModelType.AsIs))
            {
                return aHead + "0";
            }
            var aStream = new MemoryStream();
            aWriter.WriteStream(aStream);
            string aText = STEPControl.STEPControlNativeTests.Mask(Unwrap(global::Kerf.Tests.Shared.NativeStepText.Of(Encoding.Latin1.GetString(aStream.ToArray()))));
            byte[] aBytes = Encoding.Latin1.GetBytes(aText);
            if (theDump is not null)
            {
                File.WriteAllText(Path.Combine(theDump, $"{SafeName(theName)}.{theSchema}.stp"), aText, Encoding.Latin1);
            }
            return aHead + $"1 {aWriter.Writer().Model().EntityCount} {Convert.ToHexStringLower(SHA256.HashData(aBytes))} {aBytes.Length} "
                   + Convert.ToHexStringLower(SHA256.HashData(Encoding.Latin1.GetBytes(Canonical(aText))));
        }
        catch (Exception anExc) when (Refusal.IsFailure(anExc))
        {
            return aHead + "EXC " + Refusal.Unwrapped(anExc).Message;
        }
    }

    // stepcafrec's Unwrap: each line break followed by an indentation removed (the masked identifiers change where the
    // writer breaks the lines)
    internal static string Unwrap(string theText) => System.Text.RegularExpressions.Regex.Replace(theText, "\n +", "");

    // stepcafrec's Canonical: the lines with the entity numbers erased, sorted (the order of the styles'
    // representations follows a map keyed by addresses in OCCT: decision 1280)
    internal static string Canonical(string theText)
    {
        List<string> aLines = theText.Split('\n').Select(l => System.Text.RegularExpressions.Regex.Replace(l, "#[0-9]+", "#")).ToList();
        aLines.Sort(StringComparer.Ordinal);
        var aBuilder = new StringBuilder();
        foreach (string aLine in aLines)
        {
            aBuilder.Append(aLine).Append('\n');
        }
        return aBuilder.ToString();
    }

    private static string SafeName(string theName)
    {
        var aBuilder = new StringBuilder(theName);
        for (int i = 0; i < aBuilder.Length; i++)
        {
            if (aBuilder[i] is '/' or '\\' or ':' or ' ')
            {
                aBuilder[i] = '_';
            }
        }
        return aBuilder.ToString();
    }

    [Fact]
    public void WritesDocumentsAsNative()
    {
        // The healing at poles as OCCT 8.0.1 has it, so that the native recordings replay (Kerf decisions 2424, 2425)
        using IDisposable aNativeHealing = global::Kerf.Engine.Healing.Analysis.NativeHealingAtPoles.Scope();
        string aDir = Path.Combine(StepRoundTripTests.Metadata("OcctSourceDir"), "data", "occ");
        if (!Directory.Exists(aDir))
        {
            return;
        }
        string[] aSchemas = ["AP203", "AP214IS", "AP242DIS"];
        var aCases = new List<(string Line, Func<string> Port)>();
        List<Block> aBlocks = Recording("STEPCAFControl.stepcafrec-write.txt.gz", "W ");
        string? aDump = Environment.GetEnvironmentVariable("KERF_STEPCAF_DUMP");
        int anIndex = 0;
        var aPaths = new Dictionary<string, string>();
        var aJobs = new List<Func<string>>();
        foreach (string aFile in Directory.GetFiles(aDir, "*.brep").Order(StringComparer.Ordinal))
        {
            string aShort = Path.GetFileName(aFile);
            aPaths[aShort] = aFile;
            int aPartIndex = anIndex++;
            foreach (string aSchema in aSchemas)
            {
                aJobs.Add(() =>
                {
                    Assert.True(BrepTools.Read(out Shape aShape, aFile));
                    return WriteLine(XdeDocDump.MakePartDoc(aShape, aShort, aPartIndex), "part src/data/occ/" + aShort, aSchema, aDump);
                });
            }
        }
        foreach (string aName in new[] { "Ball.brep", "Pump_Nut.brep", "bottle.brep" })
        {
            foreach (string aSchema in aSchemas)
            {
                aJobs.Add(() =>
                {
                    Assert.True(BrepTools.Read(out Shape aShape, aPaths[aName]));
                    return WriteLine(XdeDocDump.MakeAssemblyDoc(aShape), "assembly " + aName, aSchema, aDump);
                });
            }
        }
        foreach (string aFile in StepRoundTripTests.CorpusFiles())
        {
            string aName = "step " + (aFile.StartsWith(Path.Combine(StepRoundTripTests.Metadata("OcctTestDataDir"), "step"), StringComparison.Ordinal) ? "testdata/step/" : "src/data/step/") + Path.GetFileName(aFile);
            aJobs.Add(() =>
            {
                var aDoc = new ExchangeDocument { AutoNaming = true };
                bool isOk;
                try
                {
                    var aReader = new CafReader();
                    isOk = aReader.ReadFile(aFile) == ReturnStatus.RetDone && aReader.Transfer(aDoc);
                }
                catch (Exception caught) when (Refusal.IsFailure(caught))
                {
                    isOk = false;
                }
                return isOk ? WriteLine(aDoc, aName, "AP214IS", aDump) : $"W {SafeName(aName)} AP214IS -";
            });
        }
        Assert.Equal(aBlocks.Count, aJobs.Count);
        string[] aPort = new string[aJobs.Count];
        Parallel.For(0, aJobs.Count, new ParallelOptions { MaxDegreeOfParallelism = Environment.ProcessorCount / 2 + 1 },
                     i => aPort[i] = aJobs[i]());
        int aSame = 0, aCanonical = 0;
        var aDifferent = new List<string>();
        for (int i = 0; i < aJobs.Count; i++)
        {
            string[] aN = aBlocks[i].Name.Split(' '), aP = aPort[i].Split(' ');
            if (aPort[i] == aBlocks[i].Name)
            {
                aSame++;
            }
            else if (aN.Length == 8 && aP.Length == 8 && aN[4] == aP[4] && aN[7] == aP[7])
            {
                aCanonical++; // the same entities and records, in another order
            }
            else
            {
                aDifferent.Add($"{aBlocks[i].Name.Split(' ')[1]} {aBlocks[i].Name.Split(' ')[2]}: native '{aBlocks[i].Name}' port '{aPort[i]}'");
            }
        }
        output.WriteLine($"{aJobs.Count} writes: {aSame} identical to native, {aCanonical} canonically identical");
        foreach (string aLine in aDifferent)
        {
            output.WriteLine(aLine);
        }
        Assert.DoesNotContain(aDifferent, d => !KnownWriteDifferences.Contains(d.Split(':')[0]));
        Assert.Equal(aJobs.Count - KnownWriteDifferences.Count, aSame + aCanonical);
        // The cube tessellated by CATIA (decision 2309): native's 71 entities and 5,295 characters, its strips' triangles
        // written back with their nodes in the order they are now read in
        Assert.Contains("step_testdata_step_bug33638.stp AP214IS: native 'W step_testdata_step_bug33638.stp AP214IS 1 71 "
                        + "6969b10d8da40cae7d7523fbc02db8391e134283435e49befb4992e42acd0e57 5295 c22d462c832e9e49f9a8d9adead94983f6d78a3587f14fe24b910bd5be0da69f' "
                        + "port 'W step_testdata_step_bug33638.stp AP214IS 1 71 "
                        + "b07d71cbe9fc58711073d6a4a13a7bc2507f5bc842c93ab306b152f93350e689 5295 c2780847daf9a562764599e2aa45511ea06e24dfd793f4947704b2b124716097'",
                        aDifferent);
    }

    // Writes that differ from native's, each attributed: the AP203 assemblies, where native leaves out the AP203
    // management entities of the last assembly occurrences (as STEPControl's, decision 1272 of unit 8F); files whose
    // GD&T (decision 1279) or materials native reads and writes back; the name native reads past its end; a file of
    // triangle strips, wound as their lists are (decision 2309)
    private static readonly HashSet<string> KnownWriteDifferences = new(StringComparer.Ordinal)
    {
        "step_testdata_step_bug33638.stp AP214IS",
        "assembly_Ball.brep AP203", "assembly_Pump_Nut.brep AP203", "assembly_bottle.brep AP203",
        "step_testdata_step_bug32745_pmi1.stp AP214IS", "step_testdata_step_bug33569.stp AP214IS",
        "step_testdata_step_bug33661_nist_ftc_08_asme1_ap242-e1-tg.stp AP214IS",
        "step_testdata_step_nist_ftc_08_asme1_ap242-2.stp AP214IS", "step_testdata_step_trj12_gd1-id-214.stp AP214IS",
        "step_testdata_step_as1-oc-214-mat.stp AP214IS", "step_testdata_step_bug28345_30338.stp AP214IS",
        "step_testdata_step_bug31670_russian.stp AP214IS",
        // decision 2446, as the reads
        "step_testdata_step_trj4_k1_geo-pe-203.stp AP214IS", "step_testdata_step_trj4_k1_geo-pe-214.stp AP214IS",
    };
}
