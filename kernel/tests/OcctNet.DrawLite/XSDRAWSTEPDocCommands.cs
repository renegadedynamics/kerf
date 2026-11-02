// Port of: src/Draw/TKXSDRAWSTEP/XSDRAWSTEP/XSDRAWSTEP.cxx (ReadStep, WriteStep)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2023-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using static Occt.IFSelect_ReturnStatus;
using static Occt.STEPControl_StepModelType;

namespace OcctNet.DrawLite;

/// <summary>
/// XSDRAWSTEP's document commands (unit 8G2): <c>ReadStep</c> and <c>WriteStep</c>, over Draw-lite's documents
/// (XdeDoc_Document, decision 1256) through STEPCAFControl's reader and writer.
/// </summary>
/// <remarks>
/// They work on the interpreter's session (XSDRAWCommands.Session) and its parameter set, which get STEPCAFControl's
/// controller as DRAWEXE's (XSDRAWSTEP::Factory records it). The main file is read from Draw-lite's files (the overlay
/// first) through the reader's stream form, under its own name, so that external references are looked for beside it
/// on the disk; the written files (the main one and, in the multifile mode, the external ones) go to the overlay. A new
/// document is made as DDocStd's commands make one (DDocStdCommands.NewDocument: the interpreter's auto-naming). The
/// progress indicators and XSDRAW::CollectActiveWorkSessions are left out.
/// </remarks>
internal static class XSDRAWSTEPDocCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "ReadStep", ReadStep);
        DrawCxx.Add(theInterpreter, "WriteStep", WriteStep);
    }

    private static int ReadStep(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int theNbArgs = argv.Count;
        XSControl_WorkSession aSession = XSDRAWCommands.Session(theDI);

        string? aDocumentName = null;
        string aFilePath = "", aModeStr = "";
        bool toTestStream = false;

        for (int anArgIter = 1; anArgIter < theNbArgs; ++anArgIter)
        {
            string anArgCase = argv[anArgIter].ToLowerInvariant();
            if (anArgCase == "-stream")
            {
                toTestStream = true;
            }
            else if (aDocumentName is null)
            {
                aDocumentName = argv[anArgIter];
            }
            else if (aFilePath.Length == 0)
            {
                aFilePath = argv[anArgIter];
            }
            else if (aModeStr.Length == 0)
            {
                aModeStr = argv[anArgIter];
            }
            else
            {
                di.Append("Syntax error at '").Append(argv[anArgIter]).Append("'");
                return 1;
            }
        }
        _ = toTestStream; // OCCT: the stream form; Draw-lite reads through it always

        bool isFileMode = XSDRAWCommands.FileAndVar(aSession, aFilePath, aDocumentName, "STEP", out string aFileName, out _);

        if (isFileMode)
        {
            di.Append(" File STEP to read : ").Append(aFileName).Append("\n");
        }
        else
        {
            di.Append(" Model taken from the session : ").Append(aFileName).Append("\n");
        }

        var aReader = new STEPCAFControl_Reader(aSession, isFileMode);
        if (aModeStr.Length > 0)
        {
            bool aMode = true;
            foreach (char c in aModeStr)
            {
                switch (c)
                {
                    case '-':
                        aMode = false;
                        break;
                    case '+':
                        aMode = true;
                        break;
                    case 'c':
                        aReader.SetColorMode(aMode);
                        break;
                    case 'n':
                        aReader.SetNameMode(aMode);
                        break;
                    case 'l':
                        aReader.SetLayerMode(aMode);
                        break;
                    case 'v':
                        aReader.SetPropsMode(aMode);
                        break;
                    case 'm':
                        aReader.SetMetaMode(aMode);
                        break;
                    default:
                        di.Append("Syntax error at '").Append(aModeStr).Append("'\n");
                        return 1;
                }
            }
        }

        IFSelect_ReturnStatus aReadStat = IFSelect_RetVoid;
        if (isFileMode)
        {
            if (theDI.Files.TryReadBytes(aFileName, out byte[] aBytes))
            {
                using var aStream = new MemoryStream(aBytes);
                aReadStat = aReader.ReadStream(aFileName, aStream);
            }
            else
            {
                aReadStat = IFSelect_RetError;
            }
        }
        else if (aSession.NbStartingEntities() > 0)
        {
            aReadStat = IFSelect_RetDone;
        }

        if (aReadStat != IFSelect_RetDone)
        {
            if (isFileMode)
            {
                di.Append("Could not read file ").Append(aFileName).Append(" , abandon\n");
            }
            else
            {
                di.Append("No model loaded\n");
            }
            return 1;
        }

        XdeDoc_Document? aDocument = DDocStdCommands.GetDocument(theDI, aDocumentName!, false);
        if (aDocument is null)
        {
            aDocument = DDocStdCommands.NewDocument(theDI, aDocumentName!);
        }

        (NCollection_DataMap<string, string> aParameters, ShapeProcess.OperationsFlags aFlags) =
            XSAlgo_ShapeProcessor.ReadProcessingData("read.step.resource.name", "read.step.sequence", aSession.Statics());
        aReader.SetShapeFixParameters(aParameters);
        aReader.SetShapeProcessFlags(aFlags);

        if (!aReader.Transfer(aDocument))
        {
            di.Append("Cannot read any relevant data from the STEP file\n");
            return 1;
        }

        di.Append("Document saved with name ").Append(aDocumentName!);
        return 0;
    }

    // A session's model written to Draw-lite's overlay, as STEPControl_Writer::Write writes it (the header applied)
    private static IFSelect_ReturnStatus Send(DrawInterpreter theDI, XSControl_WorkSession theWS, string theFileName)
    {
        if (theWS.Model() is not StepData_StepModel aModel)
        {
            return IFSelect_RetFail;
        }
        new APIHeaderSection_MakeHeader().Apply(aModel);
        var aStream = new MemoryStream();
        IFSelect_ReturnStatus aStatus = theWS.SendAll(theFileName, false, aStream);
        if (aStatus == IFSelect_RetDone || aStream.Length > 0)
        {
            theDI.Files.WriteBytes(theFileName, aStream.ToArray());
        }
        return aStatus;
    }

    // STEPCAFControl_Writer::Write to the overlay: the main file, then the external files beside it
    internal static IFSelect_ReturnStatus Write(DrawInterpreter theDI, STEPCAFControl_Writer theWriter, string theFileName)
    {
        IFSelect_ReturnStatus aStatus = Send(theDI, theWriter.Writer().WS()!, theFileName);
        if (aStatus != IFSelect_RetDone)
        {
            return aStatus;
        }
        string aDir = Path.GetDirectoryName(theFileName) ?? "";
        foreach (KeyValuePair<string, STEPCAFControl_ExternFile> anExtFileIter in theWriter.ExternFiles())
        {
            STEPCAFControl_ExternFile anExtFile = anExtFileIter.Value;
            if (anExtFile.GetWriteStatus() != IFSelect_RetVoid)
            {
                continue;
            }
            string aName = anExtFile.GetName().ToCString();
            string aFileName = aDir.Length == 0 || Path.IsPathRooted(aName) ? aName : aDir + "/" + aName;
            IFSelect_ReturnStatus anExtStatus = Send(theDI, anExtFile.GetWS(), aFileName);
            anExtFile.SetWriteStatus(anExtStatus);
            if (anExtStatus != IFSelect_RetDone)
            {
                aStatus = anExtStatus;
            }
        }
        return aStatus;
    }

    private static int WriteStep(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int theNbArgs = argv.Count;
        XSControl_WorkSession aSession = XSDRAWCommands.Session(theDI);
        var aWriter = new STEPCAFControl_Writer(aSession, true);

        XdeDoc_Document? aDocument = null;
        string aDocumentName = "", aFilePath = "";
        STEPControl_StepModelType aMode = STEPControl_AsIs;
        bool hasModeArg = false, toTestStream = false;
        string aMultiFilePrefix = "", aLabelName = "";
        XdeDoc_Label aLabel = default;

        for (int anArgIter = 1; anArgIter < theNbArgs; ++anArgIter)
        {
            string anArgCase = argv[anArgIter].ToLowerInvariant();
            if (anArgCase == "-stream")
            {
                toTestStream = true;
            }
            else if (aDocumentName.Length == 0)
            {
                aDocument = DDocStdCommands.GetDocument(theDI, argv[anArgIter]);
                if (aDocument is null)
                {
                    di.Append("Syntax error: '").Append(argv[anArgIter]).Append("' is not a document");
                    return 1;
                }
                aDocumentName = argv[anArgIter];
            }
            else if (aFilePath.Length == 0)
            {
                aFilePath = argv[anArgIter];
            }
            else if (!hasModeArg)
            {
                hasModeArg = true;
                switch (anArgCase.Length > 0 ? anArgCase[0] : '\0')
                {
                    case 'a':
                    case '0':
                        aMode = STEPControl_AsIs;
                        break;
                    case 'f':
                    case '1':
                        aMode = STEPControl_FacetedBrep;
                        break;
                    case 's':
                    case '2':
                        aMode = STEPControl_ShellBasedSurfaceModel;
                        break;
                    case 'm':
                    case '3':
                        aMode = STEPControl_ManifoldSolidBrep;
                        break;
                    case 'w':
                    case '4':
                        aMode = STEPControl_GeometricCurveSet;
                        break;
                    default:
                        di.Append("Syntax error: mode '").Append(argv[anArgIter]).Append("' is incorrect [give fsmw]");
                        return 1;
                }

                bool wrmode = true;
                foreach (char c in anArgCase)
                {
                    switch (c)
                    {
                        case '-':
                            wrmode = false;
                            break;
                        case '+':
                            wrmode = true;
                            break;
                        case 'c':
                            aWriter.SetColorMode(wrmode);
                            break;
                        case 'n':
                            aWriter.SetNameMode(wrmode);
                            break;
                        case 'l':
                            aWriter.SetLayerMode(wrmode);
                            break;
                        case 'v':
                            aWriter.SetPropsMode(wrmode);
                            break;
                    }
                }
            }
            else if (aMultiFilePrefix.Length == 0 && !anArgCase.Contains(':', StringComparison.Ordinal))
            {
                aMultiFilePrefix = argv[anArgIter];
            }
            else if (aLabel.IsNull())
            {
                aLabel = DDocStdCommands.Label(aDocument!, argv[anArgIter]);
                if (aLabel.IsNull())
                {
                    di.Append("Syntax error: No label for entry '").Append(argv[anArgIter]).Append("'");
                    return 1;
                }
                aLabelName = argv[anArgIter];
            }
            else
            {
                di.Append("Syntax error: unknown argument '").Append(argv[anArgIter]).Append("'");
                return 1;
            }
        }

        if (aFilePath.Length == 0)
        {
            di.Append("Syntax error: wrong number of arguments");
            return 1;
        }

        bool isFileMode = XSDRAWCommands.FileAndVar(aSession, aFilePath, aDocumentName, "STEP", out _, out _);

        (NCollection_DataMap<string, string> aParameters, ShapeProcess.OperationsFlags aFlags) =
            XSAlgo_ShapeProcessor.ReadProcessingData("write.step.resource.name", "write.step.sequence", aSession.Statics());
        aWriter.SetShapeFixParameters(aParameters);
        aWriter.SetShapeProcessFlags(aFlags);

        string? aMulti = aMultiFilePrefix.Length > 0 ? aMultiFilePrefix : null;
        if (!aLabel.IsNull())
        {
            di.Append("Translating label ").Append(aLabelName).Append(" of document ").Append(aDocumentName).Append(" to STEP\n");
            if (!aWriter.Transfer(aLabel, aMode, aMulti))
            {
                di.Append("Error: the label of document cannot be translated or gives no result");
                return 1;
            }
        }
        else
        {
            di.Append("Translating document ").Append(aDocumentName).Append(" to STEP\n");
            if (!aWriter.Transfer(aDocument!, aMode, aMulti))
            {
                di.Append("Error: The document cannot be translated or gives no result\n");
            }
        }

        if (!isFileMode)
        {
            di.Append("Document has been translated into the session");
            return 0;
        }

        di.Append("Writing STEP file ").Append(aFilePath).Append("\n");

        IFSelect_ReturnStatus aStat;
        if (toTestStream)
        {
            var aStream = new MemoryStream();
            aStat = aWriter.WriteStream(aStream);
            if (aStat == IFSelect_RetDone)
            {
                theDI.Files.WriteBytes(aFilePath, aStream.ToArray());
            }
        }
        else
        {
            aStat = Write(theDI, aWriter, aFilePath);
        }

        switch (aStat)
        {
            case IFSelect_RetVoid:
                di.Append("Error: no file written");
                break;
            case IFSelect_RetDone:
                di.Append("File ").Append(aFilePath).Append(" written\n");
                break;
            default:
                di.Append("Error on writing file");
                break;
        }
        return 0;
    }
}
