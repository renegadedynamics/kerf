// Port of: src/DataExchange/TKDESTEP/StepFile/StepFile_Read.hxx
// Port of: src/DataExchange/TKDESTEP/StepFile/StepFile_Read.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Foundation;

namespace Kerf.Engine.Exchange.Step.Parser;

/// <summary>
/// OCCT's free functions <c>StepFile_Read</c> (reads a STEP file or stream into a model) and
/// <c>StepFile_Interrupt</c> (traces a parse error).
/// </summary>
/// <remarks>
/// Reentrant: OCCT's global mutex, which serialised the transfer of the parse into the reader data, is left out
/// (decision 1177). The parse takes a progress range (cancellation) and limits for hostile input (decisions 1175,
/// 1176), which OCCT's function does not have.
/// </remarks>
internal static class StepFileReader
{
    /// <summary>Prints the error message: as a fail if <paramref name="isFail"/>, else as a trace.</summary>
    public static void StepFile_Interrupt(string? errorMessage, bool isFail = true)
    {
        if (errorMessage is null)
        {
            return;
        }
        using Messenger.StreamBuffer sout = isFail ? global::Kerf.Engine.Foundation.Message.SendFail() : global::Kerf.Engine.Foundation.Message.SendTrace();
        sout.Append("**** ERR StepFile : ").Append(errorMessage).Append("    ****").Append('\n');
    }

    /// <summary>
    /// Working function reading STEP file or stream. theName is the name of the file or stream; theIStream the stream
    /// to read (if null, the file theName is opened). Returns 0 on success, -1 if stream fails, 1 in case of parsing
    /// error (or cancellation, or a limit reached).
    /// </summary>
    public static int Read(string name, Stream? iStream, StepModel stepModel, StepProtocol protocol,
                           ProgressRange range = default, ParserReadLimits? limits = null)
    {
        FileRecognizer nulRecog = null;
        return Read(name, iStream, stepModel, protocol, nulRecog, nulRecog, range, limits);
    }

    private static int Read(string name, Stream? iStream, StepModel stepModel, StepProtocol protocol,
                            FileRecognizer recogHeader, FileRecognizer recogData,
                            ProgressRange range, ParserReadLimits? limits)
    {
        // if stream is not provided, open file stream here
        Stream streamPtr = iStream;
        FileStream fileStream = null;
        try
        {
            if (streamPtr is null)
            {
                try
                {
                    fileStream = new FileStream(name, FileMode.Open, FileAccess.Read, FileShare.Read);
                    streamPtr = fileStream;
                }
                catch (Exception exception) when (exception is IOException or UnauthorizedAccessException or ArgumentException or NotSupportedException)
                {
                    streamPtr = null;
                }
            }
            if (streamPtr is null || !streamPtr.CanRead)
            {
                return -1;
            }
            return ReadStream(name, streamPtr, stepModel, protocol, recogHeader, recogData, range, limits);
        }
        finally
        {
            fileStream?.Dispose();
        }
    }

    private static int ReadStream(string name, Stream aStreamPtr, StepModel stepModel, StepProtocol protocol,
                                  FileRecognizer recogHeader, FileRecognizer recogData,
                                  ProgressRange range, ParserReadLimits? limits)
    {
        // OCCT_DEBUG's chronometer (CHRONOMESURE) left out
        using Messenger.StreamBuffer sout = global::Kerf.Engine.Foundation.Message.SendTrace();
        sout.Append("      ...    Step File Reading : '").Append(name).Append("'");

        ReadData fileDataModel = new ReadData();
        try
        {
            int letat = 0;
            Scanner scanner = new Scanner(fileDataModel, aStreamPtr, limits);
            StepParser parser = new StepParser(scanner, range);
            letat = parser.Parse();
            if (letat != 0)
            {
                StepFile_Interrupt(fileDataModel.GetLastError(), true);
                return 1;
            }
        }
        catch (Exception exception) when (Refusal.IsFailure(exception))
        {
            using Messenger.StreamBuffer fail = global::Kerf.Engine.Foundation.Message.SendFail();
            fail.Append(" ...  Exception Raised while reading Step File : '").Append(name).Append("':\n")
                .Append(global::Kerf.Engine.Foundation.OcctTypeNames.Of(Refusal.Unwrapped(exception).GetType())).Append(": ").Append(Refusal.Unwrapped(exception).Message).Append("    ...");
            return 1;
        }

        sout.Append("      ...    STEP File   Read    ...\n");

        // Kerf: OCCT locks a global mutex here (decision 1177)
        fileDataModel.GetFileNbR(out int nbhead, out int nbrec, out int nbpar); // renvoi par lex/yacc
        fileDataModel.WalkOnce();
        StepReaderData undirec = new StepReaderData(nbhead, nbrec, nbpar, stepModel.SourceCodePage()); // creation tableau de records
        for (int nr = 1; nr <= nbrec; nr++)
        {
            fileDataModel.GetRecordDescription(out string ident, out string typrec, out int nbarg);
            undirec.SetRecord(nr, ident, typrec, nbarg);

            if (nbarg > 0)
            {
                while (fileDataModel.GetArgDescription(out ParamType typa, out string val))
                {
                    undirec.AddStepParam(nr, val, typa);
                }
            }
            undirec.InitParams(nr);
            fileDataModel.NextRecord();
        }

        fileDataModel.ErrorHandle(undirec.GlobalCheck());
        int failsCount = undirec.GlobalCheck().FailCount;
        if (failsCount > 0)
        {
            using Messenger.StreamBuffer info = global::Kerf.Engine.Foundation.Message.SendInfo();
            info.Append("**** ERR StepFile : Incorrect Syntax : Fails Count : ").Append(failsCount).Append(" ****");
        }

        fileDataModel.ClearRecorder(1);

        sout.Append("      ... Step File loaded  ...\n");
        sout.Append("   ").Append(undirec.RecordCount).Append(" records (entities,sub-lists,scopes), ").Append(nbpar).Append(" parameters");

        //   Analyse : par StepReaderTool

        StepReaderTool readtool = new StepReaderTool(undirec, protocol);
        readtool.SetErrorHandle(true);

        readtool.PrepareHeader(recogHeader); // Header. reco nul -> pour Protocol
        readtool.Prepare(recogData);         // Data.   reco nul -> pour Protocol

        sout.Append("      ... Parameters prepared ...\n");

        readtool.LoadModel(stepModel);
        if (stepModel.Protocol() is null)
        {
            stepModel.SetProtocol(protocol);
        }
        fileDataModel.ClearRecorder(2);
        failsCount = undirec.GlobalCheck().FailCount - failsCount;
        if (failsCount > 0)
        {
            using Messenger.StreamBuffer info = global::Kerf.Engine.Foundation.Message.SendInfo();
            info.Append("*** ERR StepReaderData : Unresolved Reference : Fails Count : ").Append(failsCount).Append(" ***");
        }
        readtool.Clear();
        undirec = null;

        sout.Append("      ...   Objects analysed  ...\n");
        int n = stepModel.EntityCount;
        sout.Append("  STEP Loading done : ").Append(n).Append(" Entities");
        return 0;
    }
}
