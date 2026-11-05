// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Exchange.Step.Parser;
using Kerf.Engine.Foundation;
using System.Globalization;
using System.IO.Compression;
using System.Reflection;
using System.Security.Cryptography;
using System.Text;

namespace Kerf.Kernel.DataExchange.Tests;

/// <summary>
/// What the STEP parser leaves in <see cref="ReadData"/>, written as tools/steprec writes native OCCT's
/// (see its main.cpp): the parse's result, the counts, every record and argument as StepFile_Read walks them, the
/// errors.
/// </summary>
internal static class StepFileDump
{
    internal static List<string> Dump(Stream theStream, ParserReadLimits? theLimits = null, ProgressRange theRange = default)
    {
        var aLines = new List<string>();
        ReadData aData = new();
        string aResult;
        string? aFatal = null;
        try
        {
            Scanner aScanner = new(aData, theStream, theLimits);
            StepParser aParser = new(aScanner, theRange);
            aResult = aParser.Parse().ToString(CultureInfo.InvariantCulture);
        }
        catch (Exception anEx) when (Refusal.IsFailure(anEx))
        {
            aResult = "FATAL";
            aFatal = Refusal.Of(anEx)!.GetMessageString();
        }

        aLines.Add("RESULT " + aResult);
        if (aFatal is not null)
        {
            aLines.Add("FATAL " + Esc(aFatal));
        }

        aData.GetFileNbR(out int nbhead, out int nbrec, out int nbpar);
        aLines.Add(string.Create(CultureInfo.InvariantCulture, $"NB {nbhead} {nbrec} {nbpar}"));
        for (int nr = 1; nr <= nbrec; nr++)
        {
            if (!aData.GetRecordDescription(out string? ident, out string? typrec, out int nbarg))
            {
                aLines.Add("R (none)");
                break;
            }

            aLines.Add(string.Create(CultureInfo.InvariantCulture, $"R {Esc(ident)} {Esc(typrec)} {nbarg}"));
            if (nbarg > 0)
            {
                while (aData.GetArgDescription(out ParamType typa, out string? val))
                {
                    aLines.Add(string.Create(CultureInfo.InvariantCulture, $"A {(int)typa} {Esc(val)}"));
                }
            }

            aData.NextRecord();
        }

        foreach (string anError in aData.InternalErrors())
        {
            aLines.Add("ERR " + Esc(anError));
        }

        return aLines;
    }

    internal static string Esc(string? theText)
    {
        if (theText is null)
        {
            return "(null)";
        }

        var aRes = new StringBuilder(theText.Length);
        foreach (char c in theText)
        {
            if (c == '\0')
            {
                break;
            }

            if (c == '\\')
            {
                aRes.Append("\\\\");
            }
            else if (c < 0x20 || c == 0x7F)
            {
                aRes.Append(CultureInfo.InvariantCulture, $"\\x{(int)c:X2}");
            }
            else
            {
                aRes.Append(c);
            }
        }

        return aRes.ToString();
    }

    /// <summary>The SHA-256 of the dump's bytes (Latin-1, one byte per char), lines ended by '\n', as steprec's.</summary>
    internal static string Hash(List<string> theLines)
    {
        var aAll = new StringBuilder();
        foreach (string aLine in theLines)
        {
            aAll.Append(aLine).Append('\n');
        }

        return Convert.ToHexStringLower(SHA256.HashData(Encoding.Latin1.GetBytes(aAll.ToString())));
    }

    /// <summary>One input of a recording: its name, size, the dump's line count and hash, and its lines kept.</summary>
    internal sealed record Entry(string Name, long Size, int NbLines, string Hash, List<string> Lines);

    internal static List<Entry> Recording(string theResource)
    {
        using Stream aRes = typeof(StepFileDump).Assembly.GetManifestResourceStream(theResource)
                            ?? throw new InvalidOperationException(theResource);
        using GZipStream aGz = new(aRes, CompressionMode.Decompress);
        using StreamReader aReader = new(aGz, Encoding.Latin1);
        var anEntries = new List<Entry>();
        string? aLine;
        while ((aLine = aReader.ReadLine()) is not null)
        {
            // FILE <name> <bytes>, DUMP <lines> <hash>, the lines kept, END
            int aSpace = aLine.LastIndexOf(' ');
            string aName = aLine[5..aSpace];
            long aSize = long.Parse(aLine[(aSpace + 1)..], CultureInfo.InvariantCulture);
            string[] aDump = aReader.ReadLine()!.Split(' ');
            var aLines = new List<string>();
            while ((aLine = aReader.ReadLine()) != "END")
            {
                aLines.Add(aLine!);
            }

            anEntries.Add(new Entry(aName, aSize, int.Parse(aDump[1], CultureInfo.InvariantCulture), aDump[2], aLines));
        }

        return anEntries;
    }

    /// <summary>
    /// The differences between the port's dump and a recorded entry: none when the hashes agree; else the first
    /// differing line the recording kept, or the counts.
    /// </summary>
    internal static string? Compare(Entry theEntry, List<string> theDump)
    {
        if (theDump.Count == theEntry.NbLines && Hash(theDump) == theEntry.Hash)
        {
            return null;
        }

        bool isWhole = theEntry.NbLines == theEntry.Lines.Count;
        int aKept = isWhole ? theEntry.Lines.Count : Math.Min(60, theEntry.Lines.Count);
        for (int i = 0; i < aKept; i++)
        {
            if (i >= theDump.Count || theDump[i] != theEntry.Lines[i])
            {
                return string.Create(CultureInfo.InvariantCulture,
                    $"{theEntry.Name}: line {i + 1}: native '{theEntry.Lines[i]}', port '{(i < theDump.Count ? theDump[i] : "<end>")}'");
            }
        }

        if (!isWhole)
        {
            // The native ERR lines kept after the first 60
            List<string> aNativeErrs = theEntry.Lines.Skip(60).ToList();
            List<string> aPortErrs = theDump.Skip(60).Where(l => l.StartsWith("ERR ", StringComparison.Ordinal)).ToList();
            if (!aNativeErrs.SequenceEqual(aPortErrs))
            {
                return $"{theEntry.Name}: errors differ: native {aNativeErrs.Count}, port {aPortErrs.Count}: '{aNativeErrs.FirstOrDefault()}' / '{aPortErrs.FirstOrDefault()}'";
            }
        }

        return string.Create(CultureInfo.InvariantCulture,
            $"{theEntry.Name}: {theDump.Count} lines (native {theEntry.NbLines}), hash differs");
    }

    internal static string Metadata(string theKey) =>
        typeof(StepFileDump).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == theKey).Value!;

    /// <summary>The path of a corpus input: testdata/... in OCCT's dataset, src/... in OCCT's source; null if absent.</summary>
    internal static string? CorpusPath(string theName)
    {
        string? aPath = null;
        if (theName.StartsWith("testdata/", StringComparison.Ordinal))
        {
            aPath = Path.Combine(Metadata("OcctTestDataDir"), theName["testdata/".Length..]);
        }
        else if (theName.StartsWith("src/", StringComparison.Ordinal))
        {
            aPath = Path.Combine(Metadata("OcctSourceDir"), theName["src/".Length..]);
        }

        return aPath is not null && File.Exists(aPath) ? aPath : null;
    }

    /// <summary>The inputs of a pack (tools/steprec's format: "name\nlength\nbytes" each).</summary>
    internal static List<(string Name, byte[] Data)> Pack(string theResource)
    {
        using Stream aRes = typeof(StepFileDump).Assembly.GetManifestResourceStream(theResource)
                            ?? throw new InvalidOperationException(theResource);
        using GZipStream aGz = new(aRes, CompressionMode.Decompress);
        using MemoryStream aMem = new();
        aGz.CopyTo(aMem);
        return ParsePack(aMem.ToArray());
    }

    internal static List<(string Name, byte[] Data)> ParsePack(byte[] theBytes)
    {
        var aRes = new List<(string, byte[])>();
        int aPos = 0;
        while (aPos < theBytes.Length)
        {
            int anEol = Array.IndexOf(theBytes, (byte)'\n', aPos);
            string aName = Encoding.Latin1.GetString(theBytes, aPos, anEol - aPos);
            int anEol2 = Array.IndexOf(theBytes, (byte)'\n', anEol + 1);
            int aLen = int.Parse(Encoding.Latin1.GetString(theBytes, anEol + 1, anEol2 - anEol - 1), CultureInfo.InvariantCulture);
            aRes.Add((aName, theBytes.AsSpan(anEol2 + 1, aLen).ToArray()));
            aPos = anEol2 + 1 + aLen;
        }

        return aRes;
    }
}
