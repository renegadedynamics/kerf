// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Parser;
using Kerf.Engine.Foundation;
using System.Globalization;
using System.IO.Compression;
using System.Text;

namespace Kerf.Kernel.DataExchange.Tests;

/// <summary>
/// The dumps tools/stepdatarec writes (unit 8B), written by the port: a STEP file read into a model with the header
/// protocol only (data entities are undefined entities), its texts decoded, then the model written back.
/// </summary>
internal static class StepDataDump
{
    private static readonly Lazy<StepProtocol> s_headerOnly = new(() =>
    {
        global::Kerf.Engine.Exchange.Step.Header.RWHeaderSection.Init();
        var aProtocol = new FileProtocol();
        aProtocol.Add(global::Kerf.Engine.Exchange.Step.Header.HeaderSection.Protocol());
        return aProtocol;
    });

    /// <summary>The protocol of the dumps: a FileProtocol of HeaderSection's protocol.</summary>
    internal static StepProtocol HeaderOnlyProtocol() => s_headerOnly.Value;

    /// <summary>A decoded text (OCCT's UTF-8) as the recorder writes it: its bytes, escaped, quoted.</summary>
    internal static string Str(TCollection_HAsciiString? theStr) =>
        theStr is null ? "(null)" : "'" + StepFileDump.Esc(StepWriter.ToFileText(theStr.ToCString())) + "'";

    private static string Strs(NCollection_HArray1<TCollection_HAsciiString>? theStrs)
    {
        if (theStrs is null)
        {
            return "(null)";
        }

        var aRes = new StringBuilder("(");
        for (int i = theStrs.Lower(); i <= theStrs.Upper(); i++)
        {
            aRes.Append(i > theStrs.Lower() ? "," : "").Append(Str(theStrs.Value(i)));
        }

        return aRes.Append(')').ToString();
    }

    // An undefined entity: TYPE(params) [& NEXT(...)], references as the model's numbers; raw texts as the file's bytes
    private static string Und(UndefinedEntity theUnd, StepModel theModel)
    {
        var aRes = new StringBuilder(StepFileDump.Esc(theUnd.StepType())).Append('(');
        UndefinedContent aCont = theUnd.UndefinedContent();
        for (int i = 1; i <= aCont.ParamCount; i++)
        {
            if (i > 1)
            {
                aRes.Append(',');
            }

            ParamType aType = aCont.ParamType(i);
            aRes.Append(((int)aType).ToString(CultureInfo.InvariantCulture)).Append(':');
            if (aType == ParamType.ParamSub)
            {
                aRes.Append(Und((UndefinedEntity)aCont.ParamEntity(i)!, theModel));
            }
            else if (aCont.IsParamEntity(i))
            {
                aRes.Append('#').Append(theModel.Number(aCont.ParamEntity(i)).ToString(CultureInfo.InvariantCulture));
            }
            else
            {
                aRes.Append(StepFileDump.Esc(aCont.ParamValue(i)?.ToCString()));
            }
        }

        aRes.Append(')');
        if (theUnd.Next() is { } aNext)
        {
            aRes.Append(" & ").Append(Und(aNext, theModel));
        }

        return aRes.ToString();
    }

    private static string Entity(Transient? theEnt, StepModel theModel) => theEnt switch
    {
        null => "(null)",
        UndefinedEntity anUnd => "U " + Und(anUnd, theModel),
        global::Kerf.Engine.Exchange.Step.Header.FileName aFn => "FILE_NAME " + Str(aFn.Name()) + " " + Str(aFn.TimeStamp()) + " " + Strs(aFn.Author()) + " "
                                      + Strs(aFn.Organization()) + " " + Str(aFn.PreprocessorVersion()) + " "
                                      + Str(aFn.OriginatingSystem()) + " " + Str(aFn.Authorisation()),
        global::Kerf.Engine.Exchange.Step.Header.FileDescription aFd => "FILE_DESCRIPTION " + Strs(aFd.Description()) + " " + Str(aFd.ImplementationLevel()),
        global::Kerf.Engine.Exchange.Step.Header.FileSchema aFs => "FILE_SCHEMA " + Strs(aFs.SchemaIdentifiers()),
        _ => "? " + theEnt.DynamicTypeName(),
    };

    private static void Checks(List<string> theLines, string thePrefix, FrameworkCheck? theCheck)
    {
        if (theCheck is null)
        {
            return;
        }

        for (int i = 1; i <= theCheck.FailCount; i++)
        {
            theLines.Add(thePrefix + " F " + StepFileDump.Esc(theCheck.CFail(i, true)));
        }

        for (int i = 1; i <= theCheck.WarningCount; i++)
        {
            theLines.Add(thePrefix + " W " + StepFileDump.Esc(theCheck.CWarning(i, true)));
        }
    }

    /// <summary>StepFile_Read's work as the recorder does it, keeping the reader data to decode its texts.</summary>
    internal static (List<string> Read, List<string> Write) ReadAndWrite(Stream theStream)
    {
        var aRead = new List<string>();
        var aWrite = new List<string>();
        StepProtocol aProtocol = HeaderOnlyProtocol();
        var aModel = new StepModel();
        var aFileDataModel = new ReadData();
        try
        {
            if (new StepParser(new Scanner(aFileDataModel, theStream)).Parse() != 0)
            {
                aRead.Add("RESULT 1");
                return (aRead, aWrite);
            }
        }
        catch (FailureException)
        {
            aRead.Add("RESULT FATAL");
            return (aRead, aWrite);
        }

        aRead.Add("RESULT 0");
        aFileDataModel.GetFileNbR(out int nbhead, out int nbrec, out int nbpar);
        var undirec = new StepReaderData(nbhead, nbrec, nbpar, aModel.SourceCodePage());
        for (int nr = 1; nr <= nbrec; nr++)
        {
            aFileDataModel.GetRecordDescription(out string? ident, out string? typrec, out int nbarg);
            undirec.SetRecord(nr, ident!, typrec!, nbarg);
            if (nbarg > 0)
            {
                while (aFileDataModel.GetArgDescription(out ParamType typa, out string? val))
                {
                    undirec.AddStepParam(nr, val!, typa);
                }
            }

            undirec.InitParams(nr);
            aFileDataModel.NextRecord();
        }

        aFileDataModel.ErrorHandle(undirec.GlobalCheck());
        aFileDataModel.ClearRecorder(1);
        var readtool = new StepReaderTool(undirec, aProtocol);
        readtool.SetErrorHandle(true);
        readtool.PrepareHeader(null);
        readtool.Prepare(null);
        readtool.LoadModel(aModel);
        if (aModel.Protocol() is null)
        {
            aModel.SetProtocol(aProtocol);
        }

        EntityIterator aHeader = aModel.Header();
        aRead.Add("HEADER " + aHeader.NbEntities().ToString(CultureInfo.InvariantCulture));
        for (aHeader.Start(); aHeader.More(); aHeader.Next())
        {
            aRead.Add("H " + Entity(aHeader.Value(), aModel));
        }

        int nb = aModel.EntityCount;
        aRead.Add("NB " + nb.ToString(CultureInfo.InvariantCulture));
        for (int i = 1; i <= nb; i++)
        {
            aRead.Add(string.Create(CultureInfo.InvariantCulture, $"E {i} #{aModel.IdentLabel(aModel.Value(i))} ") + Entity(aModel.Value(i), aModel));
            if (aModel.IsReportEntity(i))
            {
                Checks(aRead, "C " + i.ToString(CultureInfo.InvariantCulture), aModel.Check(i, true));
            }
        }

        Checks(aRead, "G", aModel.GlobalCheck());

        // Every text parameter, decoded (cleanText through ReadString)
        for (int num = 1; num <= undirec.RecordCount; num++)
        {
            for (int nump = 1; nump <= undirec.NbParams(num); nump++)
            {
                if (undirec.ParamType(num, nump) != ParamType.ParamText)
                {
                    continue;
                }

                var ach = new FrameworkCheck();
                TCollection_HAsciiString? aVal = null;
                string aPrefix = string.Create(CultureInfo.InvariantCulture, $"T {num} {nump} ");
                try
                {
                    undirec.ReadString(num, nump, "t", ach, ref aVal);
                    aRead.Add(aPrefix + Str(aVal));
                }
                catch (FailureException anEx)
                {
                    aRead.Add(aPrefix + "THROW " + StepFileDump.Esc(anEx.Message) + " " + StepFileDump.Esc(undirec.ParamCValue(num, nump)));
                }
            }
        }

        Checks(aRead, "GT", undirec.GlobalCheck());
        aFileDataModel.ClearRecorder(2);
        readtool.Clear();

        // The model written back
        var aWriter = new StepWriter(aModel);
        aWriter.SendModel(aProtocol);
        for (int i = 1; i <= aWriter.LineCount; i++)
        {
            aWrite.Add(aWriter.Line(i).ToCString());
        }

        CheckIterator aChecks = aWriter.CheckList();
        for (aChecks.Start(); aChecks.More(); aChecks.Next())
        {
            Checks(aWrite, "WC " + aChecks.Number().ToString(CultureInfo.InvariantCulture), aChecks.Value());
        }

        return (aRead, aWrite);
    }

    /// <summary>A dump as the recording keeps it: its count, hash, and kept lines (the first ones, the checks, the exceptions).</summary>
    internal sealed record Part(int NbLines, string Hash, List<string> Lines);

    /// <summary>One input of the corpus recording.</summary>
    internal sealed record Entry(string Name, long Size, Part Read, Part Write);

    internal const int Keep = 40;

    /// <summary>The lines the recorder keeps of a dump beyond its first ones.</summary>
    internal static bool IsKept(string theLine) =>
        theLine.StartsWith("G ", StringComparison.Ordinal) || theLine.StartsWith("GT ", StringComparison.Ordinal)
        || theLine.StartsWith("C ", StringComparison.Ordinal) || theLine.StartsWith("WC ", StringComparison.Ordinal)
        || theLine.StartsWith("EXCEPTION ", StringComparison.Ordinal) || theLine.Contains(" THROW ", StringComparison.Ordinal);

    internal static List<Entry> Recording(string theResource)
    {
        using StreamReader aReader = Resource(theResource);
        var anEntries = new List<Entry>();
        string? aLine = aReader.ReadLine();
        while (aLine is not null)
        {
            // FILE <name> <bytes>, READ <lines> <hash>, kept lines, WRITE <lines> <hash>, kept lines, END
            int aSpace = aLine.LastIndexOf(' ');
            string aName = aLine[5..aSpace];
            long aSize = long.Parse(aLine[(aSpace + 1)..], CultureInfo.InvariantCulture);
            aLine = aReader.ReadLine()!;
            Part? aRead = null, aWrite = null;
            while (aLine != "END")
            {
                string[] aHead = aLine.Split(' ');
                var aLines = new List<string>();
                while ((aLine = aReader.ReadLine()) != "END" && !aLine!.StartsWith("WRITE ", StringComparison.Ordinal))
                {
                    aLines.Add(aLine);
                }

                var aPart = new Part(int.Parse(aHead[1], CultureInfo.InvariantCulture), aHead[2], aLines);
                if (aHead[0] == "READ")
                {
                    aRead = aPart;
                }
                else
                {
                    aWrite = aPart;
                }
            }

            anEntries.Add(new Entry(aName, aSize, aRead!, aWrite!));
            aLine = aReader.ReadLine();
        }

        return anEntries;
    }

    internal static StreamReader Resource(string theResource)
    {
        Stream aRes = typeof(StepDataDump).Assembly.GetManifestResourceStream(theResource)
                      ?? throw new InvalidOperationException(theResource);
        return new StreamReader(new GZipStream(aRes, CompressionMode.Decompress), Encoding.Latin1);
    }

    /// <summary>The differences between the port's dump and a recorded part: null when they agree.</summary>
    internal static string? Compare(string theName, string theTag, Part theNative, List<string> theDump)
    {
        if (theDump.Count == theNative.NbLines && StepFileDump.Hash(theDump) == theNative.Hash)
        {
            return null;
        }

        List<string> aKept = theDump.Where((l, i) => i < Keep || IsKept(l)).ToList();
        for (int i = 0; i < Math.Min(aKept.Count, theNative.Lines.Count); i++)
        {
            if (aKept[i] != theNative.Lines[i])
            {
                return $"{theName} {theTag}: kept line {i + 1}: native '{theNative.Lines[i]}', port '{aKept[i]}'";
            }
        }

        return string.Create(CultureInfo.InvariantCulture,
            $"{theName} {theTag}: {theDump.Count} lines (native {theNative.NbLines}), hash differs");
    }
}
