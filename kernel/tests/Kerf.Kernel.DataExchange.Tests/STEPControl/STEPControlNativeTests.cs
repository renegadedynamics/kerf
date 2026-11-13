// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Exchange.Step;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Security.Cryptography;
using System.Text;
using Kerf.Kernel.DataExchange.Tests.StepGen;
using Xunit.Abstractions;

namespace Kerf.Kernel.DataExchange.Tests.STEPControl;

/// <summary>
/// STEPControl's reader and writer against native OCCT's, through tools/stepctlrec's recordings (unit 8F; the
/// in-process oracle, tests/Kerf.Kernel.Oracle.Tests/STEPControl, compares the shapes read and the AP214 texts written):
/// the corpus's STEP files read (statuses, roots, shapes, the model's and the transfer's check lists, each distinct
/// message of the transfer counted) and data/occ's shapes and three assemblies written in each schema (AP203, AP214IS,
/// AP242DIS) and model type (the STEP text hashed with its time, author, host and per-process counters masked).
/// </summary>
[Collection(ProcessWideMessages.Name)]
public sealed class STEPControlNativeTests(ITestOutputHelper output)
{
    private sealed record Block(string Name, List<string> Lines);

    private static List<Block> Recording(string theResource, string thePrefix)
    {
        using StreamReader aReader = StepDataDump.Resource(theResource);
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

    private static void CountChecks(CheckIterator theList, string theTag, List<string> theOut, SortedDictionary<string, int>? theMessages)
    {
        int aNbItems = 0, aNbF = 0, aNbW = 0;
        for (theList.Start(); theList.More(); theList.Next())
        {
            FrameworkCheck aCheck = theList.Value()!;
            if (aCheck.FailCount + aCheck.WarningCount == 0)
            {
                continue;
            }
            aNbItems++;
            aNbF += aCheck.FailCount;
            aNbW += aCheck.WarningCount;
            if (theMessages is not null)
            {
                for (int i = 1; i <= aCheck.FailCount; i++)
                {
                    string aKey = "F " + aCheck.CFail(i);
                    theMessages[aKey] = theMessages.GetValueOrDefault(aKey) + 1;
                }
                for (int i = 1; i <= aCheck.WarningCount; i++)
                {
                    string aKey = "W " + aCheck.CWarning(i);
                    theMessages[aKey] = theMessages.GetValueOrDefault(aKey) + 1;
                }
            }
        }
        theOut.Add($"{theTag} {aNbItems} {aNbF} {aNbW}");
    }

    /// <summary>What the recorder writes for one file read.</summary>
    private static List<string> ReadLines(string thePath)
    {
        var aLines = new List<string>();
        try
        {
            var aReader = new StepReader();
            ReturnStatus aStatus = aReader.ReadFile(thePath);
            if (aStatus != ReturnStatus.RetDone)
            {
                aLines.Add($"R {(int)aStatus} 0 0 0");
                return aLines;
            }
            int aNbRoots = aReader.NbRootsForTransfer();
            int aNbTr = aReader.TransferRoots();
            aLines.Add($"R {(int)aStatus} {aNbRoots} {aNbTr} {aReader.ShapeCount}");
            CountChecks(aReader.WS()!.ModelCheckList(), "L", aLines, null);
            var aMessages = new SortedDictionary<string, int>(StringComparer.Ordinal);
            CountChecks(aReader.WS()!.TransferReader()!.TransientProcess()!.CheckList(false), "T", aLines, aMessages);
            foreach ((string aMessage, int aCount) in aMessages)
            {
                aLines.Add($"M {aCount} {aMessage}");
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
        List<Block> aBlocks = Recording("STEPControl.stepctlrec-read.txt.gz", "F ");
        Assert.Equal(aFiles.Count, aBlocks.Count);
        int aSame = 0;
        var aDifferent = new List<string>();
        for (int i = 0; i < aFiles.Count; i++)
        {
            Assert.EndsWith("/" + Path.GetFileName(aFiles[i]), aBlocks[i].Name, StringComparison.Ordinal);
            List<string> aPort = ReadLines(aFiles[i]);
            if (aPort.SequenceEqual(aBlocks[i].Lines, StringComparer.Ordinal))
            {
                aSame++;
                continue;
            }
            string aFirst = aBlocks[i].Lines.Zip(aPort).FirstOrDefault(p => p.First != p.Second) is var (aN, aP) && aN is not null
                ? $"native '{aN}' port '{aP}'"
                : $"native {aBlocks[i].Lines.Count} lines, port {aPort.Count}";
            aDifferent.Add($"{Path.GetFileName(aFiles[i])}: {aFirst}");
        }
        output.WriteLine($"{aFiles.Count} files read: {aSame} as native");
        foreach (string aLine in aDifferent)
        {
            output.WriteLine(aLine);
        }
        Assert.Equal(334, aSame);
        Assert.DoesNotContain(aDifferent, d => !KnownReadDifferences.Contains(d.Split(':')[0]));
        Assert.Contains("bug32314_corrupted.stp: native 'L 58 178 0' port 'L 58 204 0'", aDifferent);
    }

    // Files whose checks differ from native's, each attributed (decision 1272)
    private static readonly HashSet<string> KnownReadDifferences = new(StringComparer.Ordinal)
    {
        // native throws Standard_OutOfRange ("NCollection_Sequence::Value") while the recorder walks the model's check
        // list after the transfer; the port's list is complete (1 entity, 2 fails): occt-issues.md, 8F
        "bug25013_25013faces.stp",
        // corrupted throughout, with references to entities that aren't in it: unresolved in the port, where native binds
        // each to the entity at the record of that number (decision 2308), so the load's checks count 204 warnings for
        // native's 178, on the same 58 entities
        "bug32314_corrupted.stp",
        // a sphere's and a torus's patches whose outer boundary curves run clockwise, which native reads as the rest of
        // each surface, or the far side of a rod's hemispherical end (decision 2446, OCCT #1652)
        "trj4_k1_geo-pe-203.stp", "trj4_k1_geo-pe-214.stp",
    };

    /// <summary>The STEP text with its varying parts masked, as tools/stepctlrec masks it.</summary>
    internal static string Mask(string theText)
    {
        theText = global::Kerf.Tests.Shared.NativeStepText.Of(theText);
        string[] aMasked = ["FILE_NAME", "CALENDAR_DATE", "LOCAL_TIME", "COORDINATED_UNIVERSAL_TIME_OFFSET", "PERSON", "ORGANIZATION"];
        var aRes = new StringBuilder();
        int aPos = 0;
        while (aPos < theText.Length)
        {
            int anEnd = theText.IndexOf(";\n", aPos, StringComparison.Ordinal);
            int aNext = anEnd < 0 ? theText.Length : anEnd + 2;
            string aRecord = theText[aPos..aNext];
            aPos = aNext;
            int aTypeBeg = 0;
            if (aRecord.Length > 0 && aRecord[0] == '#')
            {
                int anEq = aRecord.IndexOf(" = ", StringComparison.Ordinal);
                aTypeBeg = anEq < 0 ? aRecord.Length : anEq + 3;
            }
            int aParen = aTypeBeg < aRecord.Length ? aRecord.IndexOf('(', aTypeBeg) : -1;
            string aType = aParen < 0 ? "" : aRecord[aTypeBeg..aParen];
            if (aMasked.Contains(aType))
            {
                aRes.Append(aRecord[..aParen]).Append("(*);\n");
                continue;
            }
            if (aType == "NEXT_ASSEMBLY_USAGE_OCCURRENCE" && aParen + 1 < aRecord.Length && aRecord[aParen + 1] == '\'')
            {
                int aQuote = aRecord.IndexOf('\'', aParen + 2);
                aRecord = aRecord[..(aParen + 2)] + "#" + aRecord[aQuote..];
            }
            aRes.Append(aRecord);
        }
        string aText = aRes.ToString();
        const string aPrefix = "Open CASCADE STEP translator 8.0 ";
        var aMaskedText = new StringBuilder();
        int aFrom = 0;
        for (int aHit = aText.IndexOf(aPrefix, StringComparison.Ordinal); aHit >= 0; aHit = aText.IndexOf(aPrefix, aFrom, StringComparison.Ordinal))
        {
            int anIndex = aHit + aPrefix.Length;
            int aStop = anIndex;
            while (aStop < aText.Length && (char.IsAsciiDigit(aText[aStop]) || aText[aStop] == '.'))
            {
                aStop++;
            }
            aMaskedText.Append(aText, aFrom, anIndex - aFrom).Append('#');
            aFrom = aStop;
        }
        aMaskedText.Append(aText, aFrom, aText.Length - aFrom);
        return aMaskedText.ToString();
    }

    private static Shape Assembly(in Shape theShape)
    {
        var aBuilder = new BrepBuilder();
        var aTrsf = new Transform3D();
        aTrsf = aTrsf.WithRotation(new Axis3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 0.5);
        aTrsf = aTrsf.WithTranslationPart(new Vector3D(100.0, 50.0, 0.0));
        var anAssembly = new Compound();
        aBuilder.MakeCompound(ref anAssembly);
        aBuilder.Add(anAssembly, theShape);
        aBuilder.Add(anAssembly, theShape.Moved(new Placement(aTrsf)));
        var aTop = new Compound();
        aBuilder.MakeCompound(ref aTop);
        var aTrsf2 = new Transform3D();
        aTrsf2 = aTrsf2.WithTranslation(new Vector3D(0.0, 0.0, 300.0));
        aBuilder.Add(aTop, anAssembly);
        aBuilder.Add(aTop, anAssembly.Moved(new Placement(aTrsf2)));
        return aTop;
    }

    private static readonly (ModelType Mode, string Name)[] Modes =
    [
        (ModelType.AsIs, "AsIs"),
        (ModelType.ManifoldSolidBrep, "ManifoldSolidBrep"),
        (ModelType.BrepWithVoids, "BrepWithVoids"),
        (ModelType.FacetedBrep, "FacetedBrep"),
        (ModelType.ShellBasedSurfaceModel, "ShellBasedSurfaceModel"),
        (ModelType.GeometricCurveSet, "GeometricCurveSet"),
    ];

    [Fact]
    public void WritesEachSchemaAndModelTypeAsNative()
    {
        string aDir = Path.Combine(StepRoundTripTests.Metadata("OcctSourceDir"), "data", "occ");
        if (!Directory.Exists(aDir))
        {
            return;
        }
        var aShapes = new List<(string, Shape)>();
        foreach (string aFile in Directory.GetFiles(aDir, "*.brep").Order(StringComparer.Ordinal))
        {
            Assert.True(BrepTools.Read(out Shape aShape, aFile));
            aShapes.Add(("src_data_occ_" + Path.GetFileName(aFile), aShape));
        }
        foreach (string aName in new[] { "Ball.brep", "Pump_Nut.brep", "bottle.brep" })
        {
            Assert.True(BrepTools.Read(out Shape aShape, Path.Combine(aDir, aName)));
            aShapes.Add(("assembly_of_" + aName, Assembly(aShape)));
        }

        List<Block> aBlocks = Recording("STEPControl.stepctlrec-write.txt.gz", "W ");
        string? aDump = Environment.GetEnvironmentVariable("KERF_STEPCTL_DUMP");
        int aCase = 0, aSame = 0, aVoids = 0;
        var aDifferent = new List<string>();
        List<string> aManifold = [];
        foreach ((string aName, Shape aShape) in aShapes)
        {
            string aBefore = BRepTextOf(aShape);
            foreach (string aSchema in new[] { "AP203", "AP214IS", "AP242DIS" })
            {
                foreach ((ModelType aMode, string aModeName) in Modes)
                {
                    var aLines = new List<string>();
                    string aHead = $"W {aName} {aSchema} {aModeName} ";
                    try
                    {
                        var aWriter = new Writer();
                        aWriter.WS()!.Statics().SetCVal("write.step.schema", aSchema);
                        ReturnStatus aStatus = aWriter.Transfer(aShape, aMode);
                        if (aStatus != ReturnStatus.RetDone)
                        {
                            aLines.Add(aHead + (int)aStatus);
                        }
                        else
                        {
                            var aStream = new MemoryStream();
                            aWriter.WriteStream(aStream);
                            string aText = Mask(Encoding.Latin1.GetString(aStream.ToArray()));
                            byte[] aBytes = Encoding.Latin1.GetBytes(aText);
                            aLines.Add(aHead + $"{(int)aStatus} {aWriter.Model().EntityCount} {Convert.ToHexStringLower(SHA256.HashData(aBytes))} {aBytes.Length}");
                            CountChecks(aWriter.WS()!.TransferWriteCheckList(), "C", aLines, null);
                            if (aDump is not null)
                            {
                                File.WriteAllText(Path.Combine(aDump, $"{aName}.{aSchema}.{aModeName}.stp"), aText, Encoding.Latin1);
                            }
                        }
                    }
                    catch (Exception anExc) when (Refusal.IsFailure(anExc))
                    {
                        aLines.Add(aHead + "EXC " + Refusal.Unwrapped(anExc).Message);
                    }
                    Block aNative = aBlocks[aCase++];
                    var aNativeLines = new List<string> { aNative.Name };
                    aNativeLines.AddRange(aNative.Lines);
                    if (aMode == ModelType.ManifoldSolidBrep)
                    {
                        aManifold = aLines.Select(l => l.Replace(" ManifoldSolidBrep ", " BrepWithVoids ", StringComparison.Ordinal)).ToList();
                    }
                    if (aLines.SequenceEqual(aNativeLines, StringComparer.Ordinal))
                    {
                        aSame++;
                    }
                    // Attributed (decision 1963): native's writer refuses BrepWithVoids as unknown (2); the port's writes
                    // these shapes, which have no voids, as the manifold solid B-reps of that mode, byte for byte
                    else if (aMode == ModelType.BrepWithVoids && aNativeLines.Count == 1 && aNativeLines[0] == aHead + "2"
                             && aLines.SequenceEqual(aManifold, StringComparer.Ordinal))
                    {
                        aVoids++;
                    }
                    // ... and refuses what has no solid (3), as for its faceted modes
                    else if (aMode == ModelType.BrepWithVoids && aNativeLines.Count == 1 && aNativeLines[0] == aHead + "2"
                             && aLines.Count == 1 && aLines[0] == aHead + "3" && !new Explorer(aShape, ShapeType.Solid).More())
                    {
                        aVoids++;
                    }
                    // Attributed (decision 1962): native's faceted B-rep of a solid reads a tessellated shell it has just
                    // found to be null, so every solid failed (3); the port's writes a solid of planes
                    else if (aMode == ModelType.FacetedBrep && aNativeLines.Count == 1 && aNativeLines[0] == aHead + "3"
                             && aLines[0].StartsWith(aHead + "1 ", StringComparison.Ordinal) && AllPlanes(aShape))
                    {
                        aVoids++;
                    }
                    else
                    {
                        aDifferent.Add($"{aName} {aSchema} {aModeName}: native '{string.Join(" | ", aNativeLines)}' port '{string.Join(" | ", aLines)}'");
                    }
                }
            }
            Assert.Equal(aBefore, BRepTextOf(aShape)); // the writer never changes its input
        }
        Assert.Equal(aBlocks.Count, aCase);
        output.WriteLine($"{aCase} writes: {aSame} as native, {aVoids} B-reps with voids and faceted ones attributed");
        foreach (string aLine in aDifferent)
        {
            output.WriteLine(aLine);
        }
        Assert.Equal(708, aSame + aVoids);
        Assert.DoesNotContain(aDifferent, d => !KnownWriteDifferences.Contains(d.Split(':')[0]));
    }

    // Writes that differ from native's, each attributed (decision 1272): the AP203 assemblies, where native leaves
    // out the AP203 management entities (security classification, its officer, date and approval) of the last two
    // assembly occurrences, those of the second instance of the sub-assembly, which the port writes; the same
    // assemblies are identical in AP214 and AP242 (cause not found: occt-issues.md, 8F)
    private static readonly HashSet<string> KnownWriteDifferences = new(StringComparer.Ordinal)
    {
        "assembly_of_Ball.brep AP203 AsIs", "assembly_of_Ball.brep AP203 ManifoldSolidBrep",
        "assembly_of_Ball.brep AP203 ShellBasedSurfaceModel", "assembly_of_Ball.brep AP203 GeometricCurveSet",
        "assembly_of_Pump_Nut.brep AP203 AsIs", "assembly_of_Pump_Nut.brep AP203 ManifoldSolidBrep",
        "assembly_of_Pump_Nut.brep AP203 ShellBasedSurfaceModel", "assembly_of_Pump_Nut.brep AP203 GeometricCurveSet",
        "assembly_of_bottle.brep AP203 AsIs", "assembly_of_bottle.brep AP203 ManifoldSolidBrep",
        "assembly_of_bottle.brep AP203 ShellBasedSurfaceModel", "assembly_of_bottle.brep AP203 GeometricCurveSet",
    };

    private static bool AllPlanes(in Shape theShape)
    {
        for (var anExp = new Explorer(theShape, ShapeType.Face); anExp.More(); anExp.Next())
        {
            if (Tool.Surface(ShapeCasts.Face(anExp.Current())) is not PlaneSurface)
            {
                return false;
            }
        }
        return true;
    }

    private static string BRepTextOf(in Shape theShape)
    {
        using var aWriter = new StringWriter();
        BrepTools.Write(theShape, aWriter);
        return aWriter.ToString();
    }
}
