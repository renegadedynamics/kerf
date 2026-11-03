// Port of: src/DataExchange/TKDESTEP/StepFile/StepFile_Read.hxx
// Port of: src/DataExchange/TKDESTEP/StepFile/StepFile_Read.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// OCCT's free functions <c>StepFile_Read</c> (reads a STEP file or stream into a model) and
/// <c>StepFile_Interrupt</c> (traces a parse error).
/// </summary>
/// <remarks>
/// Reentrant: OCCT's global mutex, which serialised the transfer of the parse into the reader data, is left out
/// (decision 1177). The parse takes a progress range (cancellation) and limits for hostile input (decisions 1175,
/// 1176), which OCCT's function does not have.
/// </remarks>
public static class StepFile_Read
{
    /// <summary>Prints the error message: as a fail if <paramref name="theIsFail"/>, else as a trace.</summary>
    public static void StepFile_Interrupt(string? theErrorMessage, bool theIsFail = true)
    {
        if (theErrorMessage is null)
        {
            return;
        }
        using Messenger.StreamBuffer sout = theIsFail ? global::Kerf.Engine.Foundation.Message.SendFail() : global::Kerf.Engine.Foundation.Message.SendTrace();
        sout.Append("**** ERR StepFile : ").Append(theErrorMessage).Append("    ****").Append('\n');
    }

    /// <summary>
    /// Working function reading STEP file or stream. theName is the name of the file or stream; theIStream the stream
    /// to read (if null, the file theName is opened). Returns 0 on success, -1 if stream fails, 1 in case of parsing
    /// error (or cancellation, or a limit reached).
    /// </summary>
    public static int Read(string theName, Stream? theIStream, StepData_StepModel theStepModel, StepData_Protocol theProtocol,
                           ProgressRange theRange = default, StepFile_ReadLimits? theLimits = null)
    {
        StepData_FileRecognizer aNulRecog = null;
        return Read(theName, theIStream, theStepModel, theProtocol, aNulRecog, aNulRecog, theRange, theLimits);
    }

    private static int Read(string theName, Stream? theIStream, StepData_StepModel theStepModel, StepData_Protocol theProtocol,
                            StepData_FileRecognizer theRecogHeader, StepData_FileRecognizer theRecogData,
                            ProgressRange theRange, StepFile_ReadLimits? theLimits)
    {
        // if stream is not provided, open file stream here
        Stream aStreamPtr = theIStream;
        FileStream aFileStream = null;
        try
        {
            if (aStreamPtr is null)
            {
                try
                {
                    aFileStream = new FileStream(theName, FileMode.Open, FileAccess.Read, FileShare.Read);
                    aStreamPtr = aFileStream;
                }
                catch (Exception anException) when (anException is IOException or UnauthorizedAccessException or ArgumentException or NotSupportedException)
                {
                    aStreamPtr = null;
                }
            }
            if (aStreamPtr is null || !aStreamPtr.CanRead)
            {
                return -1;
            }
            return ReadStream(theName, aStreamPtr, theStepModel, theProtocol, theRecogHeader, theRecogData, theRange, theLimits);
        }
        finally
        {
            aFileStream?.Dispose();
        }
    }

    private static int ReadStream(string theName, Stream aStreamPtr, StepData_StepModel theStepModel, StepData_Protocol theProtocol,
                                  StepData_FileRecognizer theRecogHeader, StepData_FileRecognizer theRecogData,
                                  ProgressRange theRange, StepFile_ReadLimits? theLimits)
    {
        // OCCT_DEBUG's chronometer (CHRONOMESURE) left out
        using Messenger.StreamBuffer sout = global::Kerf.Engine.Foundation.Message.SendTrace();
        sout.Append("      ...    Step File Reading : '").Append(theName).Append("'");

        StepFile_ReadData aFileDataModel = new StepFile_ReadData();
        try
        {
            int aLetat = 0;
            StepFile_Scanner aScanner = new StepFile_Scanner(aFileDataModel, aStreamPtr, theLimits);
            StepFile_Parser aParser = new StepFile_Parser(aScanner, theRange);
            aLetat = aParser.parse();
            if (aLetat != 0)
            {
                StepFile_Interrupt(aFileDataModel.GetLastError(), true);
                return 1;
            }
        }
        catch (FailureException anException)
        {
            using Messenger.StreamBuffer aFail = global::Kerf.Engine.Foundation.Message.SendFail();
            aFail.Append(" ...  Exception Raised while reading Step File : '").Append(theName).Append("':\n")
                .Append(global::Kerf.Engine.Foundation.OcctTypeNames.Of(anException.GetType())).Append(": ").Append(anException.Message).Append("    ...");
            return 1;
        }

        sout.Append("      ...    STEP File   Read    ...\n");

        // OCCT.NET: OCCT locks a global mutex here (decision 1177)
        aFileDataModel.GetFileNbR(out int nbhead, out int nbrec, out int nbpar); // renvoi par lex/yacc
        StepData_StepReaderData undirec = new StepData_StepReaderData(nbhead, nbrec, nbpar, theStepModel.SourceCodePage()); // creation tableau de records
        for (int nr = 1; nr <= nbrec; nr++)
        {
            aFileDataModel.GetRecordDescription(out string ident, out string typrec, out int nbarg);
            undirec.SetRecord(nr, ident, typrec, nbarg);

            if (nbarg > 0)
            {
                while (aFileDataModel.GetArgDescription(out Interface_ParamType typa, out string val))
                {
                    undirec.AddStepParam(nr, val, typa);
                }
            }
            undirec.InitParams(nr);
            aFileDataModel.NextRecord();
        }

        aFileDataModel.ErrorHandle(undirec.GlobalCheck());
        int anFailsCount = undirec.GlobalCheck().NbFails();
        if (anFailsCount > 0)
        {
            using Messenger.StreamBuffer anInfo = global::Kerf.Engine.Foundation.Message.SendInfo();
            anInfo.Append("**** ERR StepFile : Incorrect Syntax : Fails Count : ").Append(anFailsCount).Append(" ****");
        }

        aFileDataModel.ClearRecorder(1);

        sout.Append("      ... Step File loaded  ...\n");
        sout.Append("   ").Append(undirec.NbRecords()).Append(" records (entities,sub-lists,scopes), ").Append(nbpar).Append(" parameters");

        //   Analyse : par StepReaderTool

        StepData_StepReaderTool readtool = new StepData_StepReaderTool(undirec, theProtocol);
        readtool.SetErrorHandle(true);

        readtool.PrepareHeader(theRecogHeader); // Header. reco nul -> pour Protocol
        readtool.Prepare(theRecogData);         // Data.   reco nul -> pour Protocol

        sout.Append("      ... Parameters prepared ...\n");

        readtool.LoadModel(theStepModel);
        if (theStepModel.Protocol() is null)
        {
            theStepModel.SetProtocol(theProtocol);
        }
        aFileDataModel.ClearRecorder(2);
        anFailsCount = undirec.GlobalCheck().NbFails() - anFailsCount;
        if (anFailsCount > 0)
        {
            using Messenger.StreamBuffer anInfo = global::Kerf.Engine.Foundation.Message.SendInfo();
            anInfo.Append("*** ERR StepReaderData : Unresolved Reference : Fails Count : ").Append(anFailsCount).Append(" ***");
        }
        readtool.Clear();
        undirec = null;

        sout.Append("      ...   Objects analysed  ...\n");
        int n = theStepModel.NbEntities();
        sout.Append("  STEP Loading done : ").Append(n).Append(" Entities");
        return 0;
    }
}
