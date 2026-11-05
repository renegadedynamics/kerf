// Port of: src/Draw/TKXSDRAWSTEP/XSDRAWSTEP/XSDRAWSTEP.cxx (stepread, testreadstep, steptrans, stepwrite,
//          testwritestep, stepfileunits)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2023-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Exchange.Step;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Exchange.Step.Header;
using Kerf.Engine.Exchange.Step.Import;
using Kerf.Engine.Foundation;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Exchange.Framework.Selection.ReturnStatus;
using static Kerf.Engine.Exchange.Step.ModelType;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// XSDRAWSTEP's commands without a document (unit 8F): <c>stepread</c>, <c>testreadstep</c>, <c>steptrans</c>,
/// <c>stepwrite</c>, <c>testwritestep</c>, <c>stepfileunits</c>. The document commands (ReadStep, WriteStep) are unit
/// 8G2's (XSDRAWSTEPDocCommands); <c>countexpected</c> and <c>dumpassembly</c> (STEPSelections' counter and assembly
/// explorer) are left out.
/// </summary>
/// <remarks>
/// They work on the interpreter's session (XSDRAWCommands.Session) and its parameter set (decision 1274): the readers
/// and writers made on the fly (testreadstep, testwritestep, stepfileunits), which OCCT makes on sessions of their own
/// reading the process-wide statics, are given sessions on the interpreter's set. Files are read from Draw-lite's
/// files (its in-memory overlay first) through the readers' stream forms, and written to the overlay through the
/// writers' stream forms, the same bytes. testreadstep and testwritestep run their files one after the other (OCCT: in
/// parallel, OSD_Parallel::For). The interactive modes of stepread (std::cin) read nothing, as DRAWEXE without input
/// does: the command ends after listing the roots. The progress indicators are left out.
/// </remarks>
internal static class XSDRAWSTEPCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "stepwrite", stepwrite);
        DrawCxx.Add(theInterpreter, "testwritestep", testwrite);
        DrawCxx.Add(theInterpreter, "stepread", stepread);
        DrawCxx.Add(theInterpreter, "testreadstep", testreadstep);
        DrawCxx.Add(theInterpreter, "steptrans", steptrans);
        DrawCxx.Add(theInterpreter, "stepfileunits", stepfileunits);
    }

    // A reader's read of a Draw-lite file: from the overlay or the disk, through the stream form
    private static ReturnStatus ReadFile(DrawInterpreter theDI, StepReader theReader, string theFile,
                                                  in Parameters? theParams = null)
    {
        if (!theDI.Files.TryReadBytes(theFile, out byte[] aBytes))
        {
            return ReturnStatus.RetError; // the file can not be opened: StepFile_Read gives -1
        }
        using var aStream = new MemoryStream(aBytes);
        return theParams is { } aParams ? theReader.ReadStream(theFile, aParams, aStream) : theReader.ReadStream(theFile, aStream);
    }

    private static int stepread(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int theNbArgs = argv.Count;
        if (theNbArgs < 3)
        {
            di.Append("Use: stepread  [file] [f or r (type of model full or reduced)]\n");
            return 1;
        }

        global::Kerf.Engine.Exchange.Framework.ExchangeWorkSession aSession = XSDRAWCommands.Session(theDI);
        var sr = new StepReader(aSession, false);
        bool modfic = XSDRAWCommands.FileAndVar(aSession, argv[1], argv[2], "STEP", out string fnom, out string rnom);
        if (modfic)
        {
            di.Append(" File STEP to read : ").Append(fnom).Append("\n");
        }
        else
        {
            di.Append(" Model taken from the session : ").Append(fnom).Append("\n");
        }
        di.Append(" -- Names of variables BREP-DRAW prefixed by : ").Append(rnom).Append("\n");
        ReturnStatus readstat = ReturnStatus.RetVoid;

        bool fromtcl = false;
        bool aFullMode = false;
        int k = 3;
        if (theNbArgs > k)
        {
            if (argv[k].StartsWith('f') || argv[3].StartsWith('F'))
            {
                aFullMode = true;
                k++;
            }
            else if (argv[k].StartsWith('r') || argv[3].StartsWith('R'))
            {
                aFullMode = false;
                k++;
            }
            else
            {
                fromtcl = true;
            }
        }
        if (!fromtcl)
        {
            fromtcl = theNbArgs > k;
        }
        theDI.Output.Write(aFullMode ? "Full model for translation with additional info will be used \n"
                                     : "Reduced model for translation without additional info will be used \n");

        sr.WS()!.SetModeStat(aFullMode);

        if (modfic)
        {
            readstat = ReadFile(theDI, sr, fnom);
        }
        else if (aSession.StartingEntityCount > 0)
        {
            readstat = ReturnStatus.RetDone;
        }

        if (readstat != ReturnStatus.RetDone)
        {
            if (modfic)
            {
                di.Append("Could not read file ").Append(fnom).Append(" , abandon\n");
            }
            else
            {
                di.Append("No model loaded\n");
            }
            return 1;
        }

        sr.SetSystemLengthUnit(XSDRAWCommands.GetLengthUnit(theDI));
        (Dictionary<string, string> aParameters, global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags aFlags) =
            global::Kerf.Engine.Exchange.Framework.ShapeProcessor.ReadProcessingData("read.step.resource.name", "read.step.sequence", aSession.Statics());
        sr.SetShapeFixParameters(aParameters);
        sr.SetShapeProcessFlags(aFlags);

        //   nom = "." -> fichier deja lu
        int i, num, nbs, modepri = 1;
        if (fromtcl)
        {
            modepri = 4;
        }
        while (modepri != 0)
        {
            num = sr.NbRootsForTransfer();
            if (!fromtcl)
            {
                di.Append("NbRootsForTransfer=").Append(num).Append(" :\n");
                for (i = 1; i <= num; i++)
                {
                    di.Append("Root.").Append(i).Append(", Ent. ");
                    var aTmpStream = new StringWriter();
                    sr.Model()!.Print(sr.RootForTransfer(i), aTmpStream);
                    di.Append(aTmpStream.ToString());
                    di.Append(" Type:").Append(sr.RootForTransfer(i)!.DynamicTypeName()).Append("\n");
                }

                theDI.Output.Write("Mode (0 End, 1 root n0 1, 2 one root/n0, 3 one entity/n0, 4 Selection) : ");
                modepri = 0; // std::cin >> modepri: no input, extraction fails and gives 0
            }

            if (modepri == 0)
            {
                di.Append("End Reading STEP\n");
                return 0;
            }
            if (modepri == 4)
            {
                NCollection_HSequence<Transient>? list;

                //  Selection, nommee ou via tcl. tcl : raccourcis admis
                //   * donne xst-transferrable-roots
                modepri = 0; // d ioffice une seule passe
                if (argv[k] == "*")
                {
                    di.Append("Transferrable Roots : ");
                    list = aSession.GiveList("xst-transferrable-roots");
                }
                else
                {
                    di.Append("List given by ").Append(argv[k]);
                    if (theNbArgs > k + 1)
                    {
                        di.Append(" ").Append(argv[k + 1]);
                    }
                    di.Append(" : ");
                    list = aSession.GiveList(argv[k], theNbArgs > k + 1 ? argv[k + 1] : "");
                }
                if (list is null)
                {
                    di.Append("No list defined. Give a selection name or * for all transferrable roots\n");
                    continue;
                }

                int ill, nbl = list.Length();
                di.Append("Nb entities selected : ").Append(nbl).Append("\n");
                if (nbl == 0)
                {
                    continue;
                }

                for (ill = 1; ill <= nbl; ill++)
                {
                    num = sr.Model()!.Number(list.Value(ill));
                    if (num == 0)
                    {
                        continue;
                    }
                    if (!sr.TransferOne(num))
                    {
                        di.Append("Transfer entity n0 ").Append(num).Append(" : no result\n");
                    }
                    else
                    {
                        nbs = sr.ShapeCount;
                        string shname = $"{rnom}_{nbs}";
                        di.Append("Transfer entity n0 ").Append(num).Append(" OK  -> DRAW Shape: ").Append(shname).Append("\n");
                        di.Append("Now, ").Append(nbs).Append(" Shapes produced\n");
                        Shape sh = sr.Shape(nbs);
                        theDI.SetShape(shname, sh);
                    }
                }
            }
            else
            {
                di.Append("Unknown mode n0 ").Append(modepri).Append("\n");
            }
        }
        return 0;
    }

    private static int testreadstep(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int theNbArgs = argv.Count;
        if (theNbArgs < 3)
        {
            di.Append("ERROR in ").Append(argv[0]).Append("Wrong Number of Arguments.\n");
            di.Append(" Usage : ").Append(argv[0]).Append(" file_name shape_name [-stream]\n");
            di.Append(" Option -stream forces usage of API accepting stream\n");
            return 1;
        }

        bool useStream = theNbArgs > 3 && string.Equals(argv[theNbArgs - 1], "-stream", StringComparison.OrdinalIgnoreCase);
        string aShName = useStream ? argv[theNbArgs - 2] : argv[theNbArgs - 1];
        int aSize = useStream ? (theNbArgs - 3) : (theNbArgs - 2);
        var aFileNames = new string[aSize];
        for (int anInd = 1; anInd <= aSize; ++anInd)
        {
            aFileNames[anInd - 1] = argv[anInd];
        }
        StepController.Init();
        global::Kerf.Engine.Exchange.Framework.StaticSet aStatics = XSDRAWCommands.Session(theDI).Statics();
        var aParameters = new Parameters();
        aParameters.InitFromStatic(aStatics);
        int aNbSubShape = 0;
        var aShapesMap = new List<(string, Shape)>();
        for (int theIndex = 0; theIndex < aSize; theIndex++)
        {
            var aReader = new StepReader(new global::Kerf.Engine.Exchange.Framework.ExchangeWorkSession(aStatics));
            (Dictionary<string, string> aFixParameters, global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags aFlags) =
                global::Kerf.Engine.Exchange.Framework.ShapeProcessor.ReadProcessingData("read.step.resource.name", "read.step.sequence", aStatics);
            aReader.SetShapeFixParameters(aFixParameters);
            aReader.SetShapeProcessFlags(aFlags);
            aReader.SetSystemLengthUnit(global::Kerf.Engine.Exchange.Framework.UnitsMethods.GetCasCadeLengthUnit(aStatics));
            ReturnStatus aReadStat;
            if (useStream)
            {
                // OCCT names the stream after the file (OSD_Path::FolderAndFileFromPath)
                string aFileNameShort = aFileNames[theIndex].Replace('\\', '/');
                aFileNameShort = aFileNameShort[(aFileNameShort.LastIndexOf('/') + 1)..];
                aReadStat = theDI.Files.TryReadBytes(aFileNames[theIndex], out byte[] aBytes)
                    ? aReader.ReadStream(aFileNameShort, aParameters, new MemoryStream(aBytes))
                    : aReader.ReadStream(aFileNameShort, aParameters, new MemoryStream());
            }
            else
            {
                aReadStat = ReadFile(theDI, aReader, aFileNames[theIndex], aParameters);
            }
            if (aReadStat == ReturnStatus.RetDone)
            {
                aReader.TransferRoots();
                string aName = aShName;
                if (aSize > 1)
                {
                    aName += theIndex;
                }
                aShapesMap.Add((aName, aReader.OneShape()));
                aNbSubShape += aReader.ShapeCount;
            }
            else
            {
                di.Append("Error: Problem with reading shape by file: ").Append("[").Append(aFileNames[theIndex]).Append("]");
            }
        }
        foreach ((string aName, Shape aShape) in aShapesMap)
        {
            theDI.SetShape(aName, aShape);
        }
        di.Append("Count of shapes produced : ").Append(aNbSubShape).Append("\n");
        return 0;
    }

    private static int steptrans(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count < 5)
        {
            di.Append("give shape-name new-shape + entity-n0 entity-n0: AXIS2\n");
            return 1;
        }
        global::Kerf.Engine.Exchange.Framework.ExchangeWorkSession aWS = XSDRAWCommands.Session(theDI);
        Shape shape = theDI.GetShape(argv[1]);
        if (shape.IsNull)
        {
            di.Append("Not a shape : ").Append(argv[1]).Append("\n");
            return 1;
        }
        StepAxis2Placement3d? ax1 = null, ax2 = null;
        int n1 = 0, n2 = 0;
        n1 = Functions.GiveEntityNumber(aWS, argv[3]);
        if (argv.Count > 4)
        {
            n2 = Functions.GiveEntityNumber(aWS, argv[4]);
        }
        if (n1 > 0)
        {
            ax1 = aWS.StartingEntity(n1) as StepAxis2Placement3d;
        }
        if (n2 > 0)
        {
            ax2 = aWS.StartingEntity(n2) as StepAxis2Placement3d;
        }
        var aFactors = new Factors();
        var mktrans = new MakeTransformed();
        if (mktrans.Compute(ax1!, ax2!, aFactors))
        {
            var loc = new Placement(mktrans.Transformation());
            theDI.SetShape(argv[2], shape.Moved(loc));
            di.Append("Transformed Shape as ").Append(argv[2]).Append("\n");
        }
        else
        {
            di.Append("No transformation computed\n");
        }
        return 0;
    }

    private static int stepwrite(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int theNbArgs = argv.Count;
        global::Kerf.Engine.Exchange.Framework.ExchangeWorkSession aWS = XSDRAWCommands.Session(theDI);
        if (aWS.NormAdaptor() is not StepController)
        {
            StepController.SelectFor(aWS); // OCCT: aWS->SelectNorm("STEP") (decision 1264)
        }
        if (theNbArgs < 3)
        {
            di.Append("Error: Give mode[1-4] and Shape name + optional file. Mode possible\n");
            di.Append("f ou 1 : FacettedBRep        s ou 2 : ShellBasedSurfaceModel\n")
              .Append("m ou 3 : ManifoldSolidBrep   w ou 4 : GeometricCurveSet/WireFrame\n");
            return 1;
        }
        ModelType mode;
        switch (argv[1].Length > 0 ? argv[1][0] : '\0')
        {
            case 'a':
            case '0':
                mode = ModelType.AsIs;
                break;
            case 'f':
            case '1':
                mode = ModelType.FacetedBrep;
                break;
            case 's':
            case '2':
                mode = ModelType.ShellBasedSurfaceModel;
                break;
            case 'm':
            case '3':
                mode = ModelType.ManifoldSolidBrep;
                break;
            case 'w':
            case '4':
                mode = ModelType.GeometricCurveSet;
                break;
            default:
                di.Append("Error: 1st arg = mode, incorrect [give fsmw]\n");
                return 1;
        }
        if (aWS.NormAdaptor()!.ActorWrite() is ActorWrite ActWrite)
        {
            ActWrite.SetGroupMode(aWS.Statics().IVal("write.step.assembly"));
        }

        Shape shape = theDI.GetShape(argv[2]);
        var sw = new Writer(aWS, false);
        global::Kerf.Engine.Exchange.Framework.InterfaceModel? stepmodel = sw.Model();
        int nbavant = stepmodel is null ? 0 : stepmodel.EntityCount;

        (Dictionary<string, string> aParameters, global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags aFlags) =
            global::Kerf.Engine.Exchange.Framework.ShapeProcessor.ReadProcessingData("write.step.resource.name", "write.step.sequence", aWS.Statics());
        sw.SetShapeFixParameters(aParameters);
        sw.SetShapeProcessFlags(aFlags);
        ReturnStatus stat = sw.Transfer(shape, mode, true);
        if (stat == ReturnStatus.RetDone)
        {
            di.Append("Translation: OK\n");
        }
        else
        {
            di.Append("Error: translation failed, status = ").Append((int)stat).Append("\n");
        }

        //   Que s est-il passe
        stepmodel = sw.Model();
        int nbapres = stepmodel is null ? 0 : stepmodel.EntityCount;
        if (nbavant > 0)
        {
            di.Append("Beware : Model not empty before transferring\n");
        }
        if (nbapres <= nbavant)
        {
            di.Append("Beware : No data produced by this transfer\n");
        }
        if (nbapres == 0)
        {
            di.Append("No data to write\n");
            return 0;
        }

        if (theNbArgs <= 3)
        {
            di.Append(" Now, to write a file, command : writeall filename\n");
            return 0;
        }
        string nomfic = argv[3];
        stat = Write(theDI, sw, nomfic);
        switch (stat)
        {
            case ReturnStatus.RetVoid:
                di.Append("Error: No file written\n");
                return 1;
            case ReturnStatus.RetDone:
                di.Append("File ").Append(nomfic).Append(" written\n");
                break;
            case ReturnStatus.RetStop:
                di.Append("Error on writing file: no space on disk or destination is write protected\n");
                return 1;
            default:
                di.Append("Error: File ").Append(nomfic).Append(" written with fail messages\n");
                break;
        }
        // XSDRAW::CollectActiveWorkSessions (the sessions XDE's commands write through) left out
        return 0;
    }

    // STEPControl_Writer::Write to Draw-lite's overlay: the header applied, then the session's SendAll to a stream
    internal static ReturnStatus Write(DrawInterpreter theDI, Writer theWriter, string theFileName)
    {
        StepModel? aModel = theWriter.Model();
        if (aModel is null)
        {
            return ReturnStatus.RetFail;
        }
        new MakeHeader().Apply(aModel);
        var aStream = new MemoryStream();
        ReturnStatus aStatus = theWriter.WS()!.SendAll(theFileName, false, aStream);
        if (aStatus == ReturnStatus.RetDone || aStream.Length > 0)
        {
            theDI.Files.WriteBytes(theFileName, aStream.ToArray());
        }
        return aStatus;
    }

    private static int testwrite(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int theNbArgs = argv.Count;
        if (theNbArgs < 3)
        {
            di.Append("ERROR in ").Append(argv[0]).Append("Wrong Number of Arguments.\n");
            di.Append(" Usage : ").Append(argv[0]).Append(" file_name shape_name [-stream]\n");
            di.Append(" Option -stream forces usage of API accepting stream\n");
            return 1;
        }

        bool useStream = theNbArgs > 3 && string.Equals(argv[theNbArgs - 1], "-stream", StringComparison.OrdinalIgnoreCase);
        string aShName = useStream ? argv[theNbArgs - 2] : argv[theNbArgs - 1];
        int aSize = useStream ? (theNbArgs - 3) : (theNbArgs - 2);
        var aFileNames = new string[aSize];
        for (int anInd = 1; anInd <= aSize; ++anInd)
        {
            aFileNames[anInd - 1] = argv[anInd];
        }
        Shape aShape = theDI.GetShape(aShName);
        if (aShape.IsNull)
        {
            di.Append("Syntax error: wrong number of arguments");
            return 1;
        }

        global::Kerf.Engine.Exchange.Framework.StaticSet aStatics = XSDRAWCommands.Session(theDI).Statics();
        var aParameters = new Parameters();
        aParameters.InitFromStatic(aStatics);

        for (int theIndex = 0; theIndex < aSize; theIndex++)
        {
            var aWriter = new Writer(new global::Kerf.Engine.Exchange.Framework.ExchangeWorkSession(aStatics));
            (Dictionary<string, string> aFixParameters, global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags aFlags) =
                global::Kerf.Engine.Exchange.Framework.ShapeProcessor.ReadProcessingData("write.step.resource.name", "write.step.sequence", aStatics);
            aWriter.SetShapeFixParameters(aFixParameters);
            aWriter.SetShapeProcessFlags(aFlags);

            if (aWriter.Transfer(aShape, ModelType.AsIs, aParameters) != ReturnStatus.RetDone)
            {
                di.Append("Error: Can't transfer input shape");
                continue;
            }
            ReturnStatus aStat;
            if (useStream)
            {
                var aStream = new MemoryStream();
                aStat = aWriter.WriteStream(aStream);
                theDI.Files.WriteBytes(aFileNames[theIndex], aStream.ToArray());
            }
            else
            {
                aStat = Write(theDI, aWriter, aFileNames[theIndex]);
            }
            if (aStat != ReturnStatus.RetDone)
            {
                di.Append("Error on writing file: ").Append("[").Append(aFileNames[theIndex]).Append("]");
                continue;
            }
        }
        di.Append("File(s) are Written");
        return 0;
    }

    private static int stepfileunits(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count < 2)
        {
            di.Append("Error: Invalid number of parameters. Should be: getfileunits name_file\n");
            return 1;
        }
        var aStepReader = new StepReader(new global::Kerf.Engine.Exchange.Framework.ExchangeWorkSession(XSDRAWCommands.Session(theDI).Statics()));
        ReturnStatus readstat = ReadFile(theDI, aStepReader, argv[1]);
        if (readstat != ReturnStatus.RetDone)
        {
            di.Append("No model loaded\n");
            return 1;
        }

        var anUnitLengthNames = new NCollection_Sequence<string>();
        var anUnitAngleNames = new NCollection_Sequence<string>();
        var anUnitSolidAngleNames = new NCollection_Sequence<string>();
        aStepReader.FileUnits(anUnitLengthNames, anUnitAngleNames, anUnitSolidAngleNames);

        int i = 1;
        di.Append("=====================================================\n");
        di.Append("LENGTH Unit\n");
        for (; i <= anUnitLengthNames.Length(); i++)
        {
            di.Append(anUnitLengthNames.Value(i)).Append("\n");
        }

        di.Append("=====================================================\n");
        di.Append("Angle Unit\n");
        for (i = 1; i <= anUnitAngleNames.Length(); i++)
        {
            di.Append(anUnitAngleNames.Value(i)).Append("\n");
        }

        di.Append("=====================================================\n");
        di.Append("Solid Angle Unit\n");
        for (i = 1; i <= anUnitSolidAngleNames.Length(); i++)
        {
            di.Append(anUnitSolidAngleNames.Value(i)).Append("\n");
        }

        return 0;
    }
}
