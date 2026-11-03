// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Foundation.Resources;
using System.Globalization;
using System.IO.Compression;
using Xunit.Abstractions;

namespace OcctNet.DataExchange.Tests;

/// <summary>
/// StepData against native OCCT's (tools/stepdatarec, unit 8B): the corpus's STEP files read into a model (header
/// entities, undefined entities, labels, checks), every text decoded, and the model written back; the decoding of
/// crafted texts under every code page; Resource_Unicode's code tables against .NET's encodings.
/// </summary>
[Collection(ProcessWideMessages.Name)]
public class StepDataNativeTests(ITestOutputHelper output)
{
    [Fact]
    public void The_corpus_reads_and_writes_as_native()
    {
        List<StepDataDump.Entry> anEntries = StepDataDump.Recording("StepData.stepdatarec-corpus.txt.gz");
        Assert.Equal(340, anEntries.Count);
        int aRun = 0, aTexts = 0, aLines = 0;
        var aDiffs = new List<string>();
        foreach (StepDataDump.Entry anEntry in anEntries)
        {
            string? aPath = StepFileDump.CorpusPath(anEntry.Name);
            if (aPath is null)
            {
                continue;
            }

            using FileStream aStream = File.OpenRead(aPath);
            Assert.Equal(anEntry.Size, aStream.Length);
            (List<string> aRead, List<string> aWrite) = StepDataDump.ReadAndWrite(aStream);
            if (StepDataDump.Compare(anEntry.Name, "READ", anEntry.Read, aRead) is { } aReadDiff)
            {
                aDiffs.Add(aReadDiff);
            }

            if (StepDataDump.Compare(anEntry.Name, "WRITE", anEntry.Write, aWrite) is { } aWriteDiff)
            {
                aDiffs.Add(aWriteDiff);
            }

            aRun++;
            aTexts += aRead.Count(l => l.StartsWith("T ", StringComparison.Ordinal));
            aLines += aWrite.Count;
        }

        output.WriteLine($"{aRun} of {anEntries.Count} files compared ({aTexts} texts decoded, {aLines} lines written), {aDiffs.Count} different");
        foreach (string aDiff in aDiffs.Take(20))
        {
            output.WriteLine(aDiff);
        }

        Assert.Empty(aDiffs);
    }

    [Fact]
    public void Texts_decode_as_native_under_every_code_page()
    {
        List<string> aCases = TextCases();
        using StreamReader aRecording = StepDataDump.Resource("StepData.stepdatarec-text.txt.gz");
        int aCompared = 0, aSame = 0, anAttributed = 0;
        var aDiffs = new List<string>();
        for (int aCase = 1; aCase <= aCases.Count; aCase++)
        {
            for (int aPage = 0; aPage <= (int)FormatType.FormatType_Big5; aPage++)
            {
                string aNative = aRecording.ReadLine()!;
                string aPort = string.Create(CultureInfo.InvariantCulture, $"{aCase} {aPage} ") + Decode(aCases[aCase - 1], (FormatType)aPage);
                aCompared++;
                if (aNative == aPort)
                {
                    aSame++;
                }
                else if (IsPastTheEnd(aCases[aCase - 1], (FormatType)aPage))
                {
                    anAttributed++;
                }
                else
                {
                    aDiffs.Add($"native '{aNative}', port '{aPort}'");
                }
            }
        }

        output.WriteLine($"{aCompared} decodings compared ({aCases.Count} texts): {aSame} identical, {anAttributed} attributed, {aDiffs.Count} different");
        foreach (string aDiff in aDiffs.Take(30))
        {
            output.WriteLine(aDiff);
        }

        Assert.Empty(aDiffs);
    }

    /// <summary>
    /// The decodings attributed, not compared: those of the system locale's code page (native's depends on the process's
    /// locale); and the texts where OCCT reads past the string's end: a UTF-8 lead byte announcing more bytes than are left before
    /// the closing quote makes OCCT's decoder read the quote and the terminating zero as continuation bytes, then go on
    /// into whatever memory follows (the port reads zeros there).
    /// </summary>
    private static bool IsPastTheEnd(string theText, FormatType thePage)
    {
        if (thePage == FormatType.FormatType_SystemLocale)
        {
            return true; // native converts from the process's C locale, the port reads UTF-8 (decision 1191)
        }

        if (thePage != FormatType.FormatType_UTF8)
        {
            return false;
        }

        // The bytes after the opening quote, the closing quote included (what cleanText decodes)
        string aBytes = theText[1..];
        for (int i = 0; i < aBytes.Length;)
        {
            int aLead = aBytes[i];
            int aMore = aLead < 0xC0 ? 0 : aLead < 0xE0 ? 1 : aLead < 0xF0 ? 2 : aLead < 0xF8 ? 3 : aLead < 0xFC ? 4 : 5;
            if (i + aMore >= aBytes.Length)
            {
                return true;
            }

            i += aMore + 1;
        }

        return false;
    }

    internal static List<string> TextCases()
    {
        using Stream aRes = typeof(StepDataNativeTests).Assembly.GetManifestResourceStream("StepData.text-cases.txt")!;
        using var aReader = new StreamReader(aRes);
        var aCases = new List<string>();
        string? aLine;
        while ((aLine = aReader.ReadLine()) is not null)
        {
            aCases.Add(System.Text.Encoding.Latin1.GetString(Convert.FromHexString(aLine)));
        }

        return aCases;
    }

    /// <summary>StepData_StepReaderData::ReadString on one quoted text, as the recorder's "text" mode writes it.</summary>
    internal static string Decode(string theText, FormatType thePage)
    {
        try
        {
            var aData = new StepReaderData(0, 1, 1, thePage);
            aData.SetRecord(1, "#1", "X", 1);
            aData.AddStepParam(1, theText, ParamType.ParamText);
            aData.InitParams(1);
            TCollection_HAsciiString? aVal = null;
            aData.ReadString(1, 1, "t", new FrameworkCheck(), ref aVal);
            string aRes = StepDataDump.Str(aVal);
            for (int i = 1; i <= aData.GlobalCheck().WarningCount; i++)
            {
                aRes += " W " + StepFileDump.Esc(aData.GlobalCheck().CWarning(i, true));
            }

            return aRes;
        }
        catch (FailureException anEx)
        {
            return "THROW " + global::Kerf.Engine.Foundation.OcctTypeNames.Of(anEx.GetType()) + " " + StepFileDump.Esc(anEx.Message);
        }
    }

    /// <summary>OCCT's code tables (the entries its conversions reach), as the recorder dumps them.</summary>
    private static List<List<(int Index, uint Value)>> NativeTables()
    {
        using Stream aRes = typeof(StepDataNativeTests).Assembly.GetManifestResourceStream("StepData.stepdatarec-tables.gz")!;
        using var aGz = new GZipStream(aRes, CompressionMode.Decompress);
        using var aReader = new BinaryReader(aGz);
        var aTables = new List<List<(int, uint)>>();
        for (int aTable = 0; aTable < 5; aTable++)
        {
            int aCount = aReader.ReadInt32();
            var anEntries = new List<(int, uint)>(aCount);
            for (int i = 0; i < aCount; i++)
            {
                anEntries.Add((aReader.ReadInt32(), aReader.ReadUInt32()));
            }

            aTables.Add(anEntries);
        }

        return aTables;
    }

    private static uint PortValue(UnicodeTables.Table theTable, int theIndex) => theTable switch
    {
        UnicodeTables.Table.CodePages =>
            UnicodeTables.CodePage(FormatType.FormatType_CP1250 + theIndex / 128)[theIndex % 128],
        UnicodeTables.Table.Sjis => UnicodeTables.SjisUni(theIndex),
        UnicodeTables.Table.Gb => UnicodeTables.GbUni(theIndex),
        UnicodeTables.Table.Gbk => UnicodeTables.GbkUni(theIndex),
        _ => UnicodeTables.Big5Uni(theIndex),
    };

    [Fact]
    public void The_code_tables_are_dotnet_encodings_corrected_to_native()
    {
        List<List<(int Index, uint Value)>> aNative = NativeTables();
        var aDiff = new List<List<(int, uint)>>();
        for (int aTable = 0; aTable < 5; aTable++)
        {
            var aKind = (UnicodeTables.Table)aTable;
            uint[] aNet = UnicodeTables.NetTable(aKind);
            var aFixes = new List<(int, uint)>();
            int aWrong = 0;
            foreach ((int anIndex, uint aValue) in aNative[aTable])
            {
                if (aNet[anIndex] != aValue)
                {
                    aFixes.Add((anIndex, aValue));
                }

                if (PortValue(aKind, anIndex) != aValue)
                {
                    aWrong++;
                }
            }

            aDiff.Add(aFixes);
            output.WriteLine($"{aKind}: {aNative[aTable].Count} entries, .NET differs on {aFixes.Count}, the corrected table on {aWrong}");
        }

        // The corrections file (src/OcctNet.DataExchange/Resource/Resource_UnicodeTables.diff.gz) is regenerated with
        // OCCTNET_WRITE_UNICODE_DIFF=<path>
        if (Environment.GetEnvironmentVariable("OCCTNET_WRITE_UNICODE_DIFF") is { Length: > 0 } aPath)
        {
            using FileStream aFile = File.Create(aPath);
            using var aGz = new GZipStream(aFile, CompressionLevel.SmallestSize);
            using var aWriter = new BinaryWriter(aGz);
            foreach (List<(int Index, uint Value)> aFixes in aDiff)
            {
                aWriter.Write(aFixes.Count);
                foreach ((int anIndex, uint aValue) in aFixes)
                {
                    aWriter.Write(anIndex);
                    aWriter.Write(aValue);
                }
            }
        }

        for (int aTable = 0; aTable < 5; aTable++)
        {
            var aKind = (UnicodeTables.Table)aTable;
            foreach ((int anIndex, uint aValue) in aNative[aTable])
            {
                Assert.True(PortValue(aKind, anIndex) == aValue, $"{aKind}[{anIndex}]: native {aValue:X}, port {PortValue(aKind, anIndex):X}");
            }
        }
    }
}
