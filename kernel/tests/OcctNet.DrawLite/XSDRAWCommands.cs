// Port of: src/Draw/TKXSDRAW/XSDRAW/XSDRAW.cxx (the session, FileAndVar, GetLengthUnit)
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_Functions.cxx (xload, xread, writeall, writeent, writent,
//          listtypes, data, entity, param)
// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_Functions.cxx (newmodel, tpstat)
// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_FuncShape.cxx (tpdraw, tpcompound, FileAndVar)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using System.Runtime.CompilerServices;
using System.Text;
using Occt;
using static Occt.IFSelect_ReturnStatus;

namespace OcctNet.DrawLite;

/// <summary>
/// The data exchange session of an interpreter (XSDRAW's) and the IFSelect and XSControl commands the corpus uses
/// (unit 8F, decisions 1221, 1274): <c>param</c>, <c>data</c>, <c>entity</c>, <c>xload</c>/<c>xread</c>,
/// <c>writeall</c>, <c>writeent</c>/<c>writent</c>, <c>listtypes</c>, <c>newmodel</c>, <c>tpstat</c>, <c>tpdraw</c>,
/// <c>tpcompound</c>.
/// </summary>
/// <remarks>
/// <para>
/// OCCT keeps one process-wide session (XSDRAW::Session, held by an IFSelect_SessionPilot) whose statics are the
/// process-wide <c>Interface_Static</c>; Draw-lite keeps one per interpreter, with its own parameter set, in which
/// the standard, XSAlgo's, STEP's and STEPCAFControl's statics are declared when the session is made (as DRAWEXE
/// declares them when it loads the plugin). The commands are written on the session directly (decision 1221): no
/// session pilot. As XSTEPDRAWRUN gives the default messenger a printer to the Tcl result while such a command runs,
/// Draw-lite gives the command's flow a messenger printing to its result (decision 1275): the libraries' messages
/// (StepSelect_WorkLibrary's "Write ... Done") reach the result as in DRAWEXE, each followed by a new line, and the
/// command's own message comes when it returns, after them. Files are Draw-lite's (its in-memory overlay for those
/// written).
/// </para>
/// </remarks>
internal static class XSDRAWCommands
{
    private static readonly ConditionalWeakTable<DrawInterpreter, XSControl_WorkSession> theSessions = new();

    public static void Register(DrawInterpreter theInterpreter)
    {
        Pilot(theInterpreter, "xload", xload);
        Pilot(theInterpreter, "xread", xload);
        Pilot(theInterpreter, "writeall", writeall);
        Pilot(theInterpreter, "writeent", writeent);
        Pilot(theInterpreter, "writent", writeent);
        Pilot(theInterpreter, "listtypes", listtypes);
        Pilot(theInterpreter, "data", data);
        Pilot(theInterpreter, "entity", entity);
        Pilot(theInterpreter, "param", param);
        Pilot(theInterpreter, "newmodel", newmodel);
        Pilot(theInterpreter, "xnorm", xnorm);
        Pilot(theInterpreter, "tpstat", tpstat);
        Pilot(theInterpreter, "tpdraw", tpdraw);
        Pilot(theInterpreter, "tpcompound", tpcompound);
    }

    // XSTEPDRAWRUN: while the command runs, the default messenger prints to the Tcl result (Draw_Printer: each message
    // and a new line, from Message_Info up), here through a messenger of the command's flow (decision 1275)
    internal static void Pilot(DrawInterpreter theInterpreter, string theName, DrawCxxCommand theCommand)
        => DrawCxx.Add(theInterpreter, theName, (theDI, di, argv) =>
        {
            global::Kerf.Engine.Foundation.Messenger? anOld = global::Kerf.Engine.Foundation.Message.SetFlowMessenger(new global::Kerf.Engine.Foundation.Messenger(new ResultPrinter(di)));
            try
            {
                return theCommand(theDI, di, argv);
            }
            finally
            {
                global::Kerf.Engine.Foundation.Message.SetFlowMessenger(anOld);
            }
        });

    /// <summary>Draw_Printer: a message and a new line appended to the command's result.</summary>
    private sealed class ResultPrinter : global::Kerf.Engine.Foundation.Printer
    {
        private readonly DrawResult myResult;

        public ResultPrinter(DrawResult theResult)
        {
            myResult = theResult;
            _traceLevel = global::Kerf.Engine.Foundation.Gravity.Info;
        }

        protected override void SendCore(string theString, global::Kerf.Engine.Foundation.Gravity theGravity)
        {
            if (theGravity < _traceLevel)
            {
                return;
            }
            myResult.Append(theString).Append("\n");
        }
    }

    /// <summary>XSDRAW::Session: the interpreter's session, made the first time with the statics of STEP declared.</summary>
    public static XSControl_WorkSession Session(DrawInterpreter theDI) => theSessions.GetValue(theDI, _ => NewSession());

    private static XSControl_WorkSession NewSession()
    {
        var aSession = new XSControl_WorkSession();
        aSession.SetVars(new XSControl_Vars());
        Interface_StaticSet aStatics = aSession.Statics();
        // XSDRAW::LoadSession calls XSAlgo::Init; XSDRAWSTEP::Factory STEPCAFControl_Controller::Init, which records
        // the STEP controller and declares its statics (decision 1274)
        XSAlgo.Init(aStatics);
        STEPCAFControl_Controller.Init();
        new STEPCAFControl_Controller().InitStatics(aStatics);
        return aSession;
    }

    /// <summary>XSDRAW::GetLengthUnit without a document: the session's <c>xstep.cascade.unit</c>, in millimetres.</summary>
    public static double GetLengthUnit(DrawInterpreter theDI) => UnitsMethods.GetCasCadeLengthUnit(Session(theDI).Statics());

    /// <summary>XSControl_FuncShape::FileAndVar: the file (else the session's loaded one) and the variable's name.</summary>
    public static bool FileAndVar(XSControl_WorkSession session, string? file, string? var, string def, out string resfile, out string resvar)
    {
        bool iafic = true;
        if (file is not null && (file.Length == 0 || file == "."))
        {
            iafic = false;
        }
        resfile = iafic ? file ?? "" : session.LoadedFile();

        if (!string.IsNullOrEmpty(var) && var != ".")
        {
            resvar = var;
        }
        else if (resfile.Length == 0)
        {
            resvar = def;
        }
        else
        {
            // TCollection_AsciiString's 1-based searches (-1 when not found)
            int nomdeb = resfile.LastIndexOf('/') + 1;
            if (nomdeb <= 0)
            {
                nomdeb = resfile.LastIndexOf('\\') + 1; // for NT
                if (nomdeb == 0)
                {
                    nomdeb = -1;
                }
            }
            if (nomdeb < 0)
            {
                nomdeb = 0;
            }
            int nomfin = resfile.LastIndexOf('.') + 1;
            if (nomfin == 0)
            {
                nomfin = -1;
            }
            if (nomfin < nomdeb)
            {
                nomfin = resfile.Length + 1;
            }
            resvar = resfile.Substring(nomdeb, nomfin - 1 - nomdeb);
        }
        return iafic;
    }

    // A message of the command (Message::SendInfo's stream buffer), appended with a new line as Draw_Printer does
    private static void Send(DrawResult di, StringBuilder theMessage)
    {
        if (theMessage.Length > 0)
        {
            di.Append(theMessage.ToString()).Append("\n");
            theMessage.Clear();
        }
    }

    private static int Status(IFSelect_ReturnStatus theStatus) => theStatus is IFSelect_ReturnStatus.IFSelect_RetError or IFSelect_ReturnStatus.IFSelect_RetFail ? 1 : 0;

    // IFSelect_SessionPilot::CommandPart: the words from the given one on
    private static string CommandPart(IReadOnlyList<string> argv, int numword) =>
        numword < argv.Count ? string.Join(" ", argv.Skip(numword)) : "";

    // IFSelect_SessionPilot::Number: an entity's number from its label (OCCT prints a refused label to cout)
    private static int Number(DrawInterpreter theDI, XSControl_WorkSession WS, string val)
    {
        int num = WS.NumberFromLabel(val);
        if (num < 0)
        {
            theDI.Output.WriteLine(" Label:" + val + " ->" + (-num) + " ent.s, refus");
        }
        return num;
    }

    private static string Arg(IReadOnlyList<string> argv, int num) => num < argv.Count ? argv[num] : "";

    // fun3: Read File -> Load Model (the file read from Draw-lite's files)
    private static int xload(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        XSControl_WorkSession WS = Session(theDI);
        int argc = argv.Count;
        string arg1 = Arg(argv, 1);
        var sout = new StringBuilder();
        if (argc < 2)
        {
            sout.Append("Read/Load : give file name !").Append('\n');
            Send(di, sout);
            return 1;
        }
        if (WS.Protocol() is null)
        {
            sout.Append("Protocol not defined").Append('\n');
            Send(di, sout);
            return 1;
        }
        if (WS.WorkLibrary() is null)
        {
            sout.Append("WorkLibrary not defined").Append('\n');
            Send(di, sout);
            return 1;
        }

        IFSelect_ReturnStatus status = theDI.Files.TryReadBytes(arg1, out byte[] aBytes)
            ? WS.ReadStream(arg1, new MemoryStream(aBytes))
            : WS.ReadFile(arg1);
        // status: 0 OK, 1 read error, 2 Fail(try/catch),
        //          -1 file not found, -2 reading done but empty result
        switch (status)
        {
            case IFSelect_ReturnStatus.IFSelect_RetVoid:
                sout.Append("file:").Append(arg1).Append(" gives empty result").Append('\n');
                break;
            case IFSelect_ReturnStatus.IFSelect_RetError:
                sout.Append("file:").Append(arg1).Append(" could not be opened").Append('\n');
                break;
            case IFSelect_ReturnStatus.IFSelect_RetDone:
                sout.Append("file:").Append(arg1).Append(" read").Append('\n');
                break;
            case IFSelect_ReturnStatus.IFSelect_RetFail:
                sout.Append("file:").Append(arg1).Append(" : error while reading").Append('\n');
                break;
            case IFSelect_ReturnStatus.IFSelect_RetStop:
                sout.Append("file:").Append(arg1).Append(" : EXCEPTION while reading").Append('\n');
                break;
            default:
                sout.Append("file:").Append(arg1).Append(" could not be read").Append('\n');
                break;
        }
        Send(di, sout);
        if (status != IFSelect_ReturnStatus.IFSelect_RetDone)
        {
            return Status(status);
        }
        //      sout<<" - clearing list of already written files"<<std::endl;
        WS.BeginSentFiles(true);
        return Status(status);
    }

    // fun4: Write All (to Draw-lite's file overlay)
    private static int writeall(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        XSControl_WorkSession WS = Session(theDI);
        var sout = new StringBuilder();
        if (argv.Count < 2)
        {
            sout.Append("Write All : give file name !").Append('\n');
            Send(di, sout);
            return 1;
        }
        var aStream = new MemoryStream();
        IFSelect_ReturnStatus aStatus = WS.SendAll(argv[1], false, aStream);
        if (aStream.Length > 0 || aStatus == IFSelect_ReturnStatus.IFSelect_RetDone)
        {
            theDI.Files.WriteBytes(argv[1], aStream.ToArray());
        }
        return Status(aStatus);
    }

    // fun6: Write Entite(s)
    private static int writeent(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        XSControl_WorkSession WS = Session(theDI);
        int argc = argv.Count;
        var sout = new StringBuilder();
        if (argc < 3)
        {
            sout.Append("Write Entitie(s) : give file name + n0s entitie(s)!").Append('\n');
            Send(di, sout);
            return 1;
        }
        int ko = 0;
        var sp = new IFSelect_SelectPointed();
        for (int ia = 2; ia < argc; ia++)
        {
            int id = Number(theDI, WS, argv[ia]);
            if (id > 0)
            {
                global::Kerf.Engine.Foundation.Transient? item = WS.StartingEntity(id);
                if (sp.Add(item))
                {
                    sout.Append("Added:no.").Append(id).Append('\n');
                }
                else
                {
                    sout.Append(" Fail Add n0.").Append(id).Append('\n');
                    ko++;
                }
            }
            else
            {
                sout.Append("Not an entity number:").Append(argv[ia]).Append('\n');
                ko++;
            }
        }
        if (ko > 0)
        {
            sout.Append(ko).Append(" bad arguments, abandon").Append('\n');
            Send(di, sout);
            return 1;
        }
        var aStream = new MemoryStream();
        IFSelect_ReturnStatus aStatus = WS.SendSelected(argv[1], sp, false, aStream);
        if (aStream.Length > 0 || aStatus == IFSelect_ReturnStatus.IFSelect_RetDone)
        {
            theDI.Files.WriteBytes(argv[1], aStream.ToArray());
        }
        Send(di, sout); // the command's message, sent when it returns: after the library's
        return Status(aStatus);
    }

    // fun9: List Types, through IFSelect_SessionPilot::ExecuteCounter
    private static int listtypes(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        XSControl_WorkSession WS = Session(theDI);
        IFSelect_Signature? signtype = WS.SignType() ?? new IFSelect_SignType();
        var counter = new IFSelect_SignCounter(signtype, false);
        counter.Clear();
        if (argv.Count <= 1)
        {
            counter.AddModel(WS.Model());
        }
        else
        {
            //   we request a givelist
            NCollection_HSequence<global::Kerf.Engine.Foundation.Transient>? list = WS.GiveList(CommandPart(argv, 1));
            if (list is null)
            {
                theDI.Output.WriteLine("Nothing selected from : " + CommandPart(argv, 1));
                return 1;
            }
            counter.AddWithGraph(list, WS.Graph());
        }
        var aText = new StringWriter();
        counter.PrintList(aText, WS.Model(), IFSelect_PrintCount.IFSelect_ListByItem);
        di.Append(aText.ToString()).Append("\n");
        return 0;
    }

    // fun11: DumpModel (Data)
    private static int data(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        XSControl_WorkSession WS = Session(theDI);
        string arg1 = Arg(argv, 1);
        int niv = 0;
        var sout = new StringBuilder();
        switch (arg1.Length > 0 ? arg1[0] : '\0')
        {
            case '?':
                sout.Append("? for this help, else give a listing mode (first letter suffices) :\n")
                    .Append(" general    General Statistics\n roots    Roots\n")
                    .Append(" entities   All Entities\n")
                    .Append(" listfails  CheckList (fails)    per entity\n")
                    .Append(" messages   CheckList (complete) per entity\n")
                    .Append(" fails      CheckList (fails)    per message (counting)\n")
                    .Append(" check      CheckList (complete) per message (counting)\n")
                    .Append(" totalcheck CheckList (complete) per message (listing n0 ents)\n")
                    .Append(" FAILS      CheckList (fails)    per message (listing complete)\n")
                    .Append(" TOTALCHECK CheckList (complete) per message (listing complete)").Append('\n');
                Send(di, sout);
                return 0;
            case 'g':
                niv = 0;
                break;
            case 'r':
                niv = 1;
                break;
            case 'e':
                niv = 2;
                break;
            case 'l':
                niv = 3;
                break;
            case 'm':
                niv = 4;
                break;
            case 'c':
                niv = 5;
                break;
            case 't':
                niv = 6;
                break;
            case 'T':
                niv = 7;
                break;
            case 'f':
                niv = 8;
                break;
            case 'F':
                niv = 10;
                break;
            default:
                sout.Append("Unknown Mode .  data simply for help").Append('\n');
                Send(di, sout);
                return 1;
        }
        WS.TraceDumpModel(niv); // one message, through the messenger
        return 0;
    }

    // fundumpent: entity n0 or id [+ level]
    private static int entity(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        XSControl_WorkSession WS = Session(theDI);
        int argc = argv.Count;
        IFSelect_WorkLibrary WL = WS.WorkLibrary()!;
        int levdef = 0, levmax = 10, level;
        WL.DumpLevels(out levdef, out levmax);
        var sout = new StringBuilder();
        if (argc < 2 || (argc == 2 && levmax < 0))
        {
            sout.Append("Give n0 or id of entity");
            if (levmax < 0)
            {
                sout.Append("  and dump level").Append('\n');
            }
            else
            {
                sout.Append("  + optional, dump level in [0 - ").Append(levmax).Append("] , default = ").Append(levdef).Append('\n');
            }
            for (level = 0; level <= levmax; level++)
            {
                string help = WL.DumpHelp(level);
                if (help.Length > 0)
                {
                    sout.Append(level).Append(" : ").Append(help).Append('\n');
                }
            }
            Send(di, sout);
            return 1;
        }

        string arg1 = argv[1];
        string arg2 = Arg(argv, 2);
        int num = Number(theDI, WS, arg1);
        if (num == 0)
        {
            return 1;
        }
        level = levdef;
        if (argc > 2)
        {
            level = global::Kerf.Engine.Foundation.Standard.Atoi(arg2);
        }
        global::Kerf.Engine.Foundation.Transient? ent = WS.StartingEntity(num);
        if (ent is null)
        {
            sout.Append("No entity with given id ").Append(arg1).Append(" (").Append(num).Append(") is found in the current model").Append('\n');
        }
        else
        {
            sout.Append("  --   DUMP  Entity n0 ").Append(num).Append("  level ").Append(level).Append('\n');
            var aText = new StringWriter();
            WL.DumpEntity(WS.Model(), WS.Protocol(), ent, aText, level);

            Interface_CheckIterator chl = WS.CheckOne(ent);
            if (!chl.IsEmpty(false))
            {
                chl.Print(aText, WS.Model(), false);
            }
            sout.Append(aText.ToString());
        }
        Send(di, sout);
        return 0;
    }

    // fun27: Param(Value) on the interpreter's parameter set
    private static int param(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        XSControl_WorkSession WS = Session(theDI);
        Interface_StaticSet aStatics = WS.Statics();
        int argc = argv.Count;
        string arg1 = Arg(argv, 1);
        string arg2 = Arg(argv, 2);
        if (arg2 == "\"\"")
        {
            arg2 = "";
        }
        var sout = new StringBuilder();
        if (argc < 2 || (argc == 3 && arg1 == "-p"))
        {
            NCollection_HSequence<TCollection_HAsciiString> li = aStatics.Items();
            int i, nb = li.Length(), aPatternNb = 0;
            if (argc == 3)
            {
                for (i = 1; i <= nb; i++)
                {
                    if (li.Value(i)!.String().StartsWith(arg2, StringComparison.Ordinal))
                    {
                        aPatternNb++;
                    }
                }
            }
            else
            {
                aPatternNb = nb;
            }
            sout.Append(" List of parameters : ").Append(aPatternNb).Append(" items : ").Append('\n');
            for (i = 1; i <= nb; i++)
            {
                if (argc == 3 && !li.Value(i)!.String().StartsWith(arg2, StringComparison.Ordinal))
                {
                    continue;
                }
                sout.Append(li.Value(i)!.String());
                sout.Append(" : ").Append(aStatics.CVal(li.Value(i)!.ToCString())).Append('\n');
            }
            Send(di, sout);
            return 0;
        }
        else if (global::Kerf.Engine.Foundation.Standard.Atoi(arg1) > 0)
        {
            int use = global::Kerf.Engine.Foundation.Standard.Atoi(arg1);
            WS.TraceStatics(use);
        }
        else
        {
            if (argc > 2)
            {
                sout.Append("     FORMER STATUS of Static Parameter ").Append(arg1).Append('\n');
            }
            else
            {
                sout.Append("     ACTUAL STATUS of Static Parameter ").Append(arg1).Append('\n');
            }
            if (!aStatics.IsPresent(arg1))
            {
                sout.Append(" Parameter ").Append(arg1).Append(" undefined").Append('\n');
                Send(di, sout);
                return 1;
            }
            if (!aStatics.IsSet(arg1))
            {
                sout.Append(" Parameter ").Append(arg1).Append(" not valued").Append('\n');
            }
            else if (argc == 2)
            {
                var aText = new StringWriter();
                aStatics.Static(arg1)!.Print(aText);
                sout.Append(aText.ToString());
            }
            else
            {
                sout.Append(" Value : ").Append(aStatics.CVal(arg1)).Append('\n');
            }

            if (argc == 2)
            {
                sout.Append("To modify, param name_param new_val").Append('\n');
            }
            else
            {
                if (arg2.Length != 0)
                {
                    sout.Append(" New demanded value : ").Append(arg2);
                }
                else
                {
                    sout.Append(" New demanded value : not valued");
                }
                if (aStatics.SetCVal(arg1, arg2))
                {
                    sout.Append("   OK").Append('\n');
                    Send(di, sout);
                    return 0;
                }
                else
                {
                    sout.Append(" , refused").Append('\n');
                    Send(di, sout);
                    return 1;
                }
            }
        }
        Send(di, sout);
        return 0;
    }

    // XSControl_newmodel
    // XSControl_xnorm. The norms recorded are STEP's (OCCT.NET: the session gets a controller of its own, decision
    // 1264, rather than the one recorded process-wide); the others aren't ported
    private static int xnorm(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        XSControl_WorkSession WS = Session(theDI);
        XSControl_Controller? control = WS.NormAdaptor();
        var sout = new StringBuilder();
        sout.Append(argv.Count == 1 ? "Current Norm. xnorm newnorm to change" : "Current Norm :").Append('\n');
        if (control is null)
        {
            sout.Append("no norm currently defined").Append('\n');
        }
        else
        {
            sout.Append("  Long  Name (complete) : ").Append(control.Name(false)).Append('\n')
                .Append("  Short name (resource) : ").Append(control.Name(true)).Append('\n');
        }
        if (argv.Count == 1)
        {
            Send(di, sout);
            return 0; // IFSelect_RetVoid
        }

        if (XSControl_Controller.Recorded(argv[1]) is not STEPControl_Controller)
        {
            sout.Append(" No norm named : ").Append(argv[1]).Append('\n');
            Send(di, sout);
            return 1;
        }
        STEPControl_Controller.SelectFor(WS);
        sout.Append("new norm : ").Append(WS.NormAdaptor()!.Name()).Append('\n');
        Send(di, sout);
        return 0;
    }

    private static int newmodel(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        XSControl_WorkSession WS = Session(theDI);
        // OCCT's Draw session has the STEP norm once a STEP command ran; Draw-lite's selects it here if none is
        if (WS.NormAdaptor() is null)
        {
            STEPControl_Controller.SelectFor(WS);
        }
        if (WS.NewModel() is not null)
        {
            return 0;
        }
        di.Append("No new Model produced\n").Append("\n");
        return 0;
    }

    // XSControl_tpstat
    private static int tpstat(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        XSControl_WorkSession WS = Session(theDI);
        int argc = argv.Count;
        string arg1 = Arg(argv, 1);
        Transfer_TransientProcess? TP = WS.TransferReader()?.TransientProcess();
        var sout = new StringBuilder();
        if (TP is null)
        {
            sout.Append("No Transfer Read").Append('\n');
            Send(di, sout);
            return 1;
        }
        //        ****    tpstat        ****

        int mod1 = -1;
        int mod2 = 0;
        //  g : general  c : check (compte) C (liste)   f : fails(compte)  F (liste)
        //  resultats racines :  n : n0s entites  s : status  b : binders
        //    t : count by type  r : count by result   l : list(type-result)
        //   *n *s *b *t *r *l : same on everything
        //   ?n  etc.. : same on abnormal results
        //   ?  short for help

        if (argc > 1)
        {
            char a2 = arg1.Length > 1 ? arg1[1] : '\0';
            if (a2 == '\0')
            {
                a2 = '!';
            }
            switch (arg1.Length > 0 ? arg1[0] : '\0')
            {
                case 'g':
                    mod1 = 0;
                    break;
                case 'c':
                    mod1 = 4;
                    mod2 = 4;
                    break;
                case 'C':
                    mod1 = 4;
                    mod2 = 2;
                    break;
                case 'f':
                    mod1 = 5;
                    mod2 = 4;
                    break;
                case 'F':
                    mod1 = 5;
                    mod2 = 2;
                    break;
                case '*':
                    mod1 = 2;
                    break;
                case '?':
                    mod1 = 3;
                    break;
                default:
                    mod1 = 1;
                    if (argc > 2)
                    {
                        mod1 = 2;
                    }
                    a2 = arg1.Length > 0 ? arg1[0] : '\0';
                    break;
            }
            if (mod1 < 1 || mod1 > 3)
            {
                a2 = '!';
            }
            switch (a2)
            {
                case 'n':
                    mod2 = 0;
                    break;
                case 's':
                    mod2 = 1;
                    break;
                case 'b':
                    mod2 = 2;
                    break;
                case 't':
                    mod2 = 3;
                    break;
                case 'r':
                    mod2 = 4;
                    break;
                case 'l':
                    mod2 = 5;
                    break;
                case 'L':
                    mod2 = 6;
                    break;
                case '!':
                    break;
                case '?':
                    mod1 = -1;
                    break;
                default:
                    mod1 = -2;
                    break;
            }
        }
        //  A present help eventuel
        if (mod1 < -1)
        {
            sout.Append("Unknown Mode").Append('\n');
        }
        if (mod1 < 0)
        {
            sout.Append("Modes available :\n")
                .Append("g : general    c : checks (count)  C (list)\n")
                .Append("               f : fails  (count)  F (list)\n")
                .Append("  n : numbers of transferred entities (on TRANSFER ROOTS)\n")
                .Append("  s : their status (type entity-result , presence checks)\n")
                .Append("  b : detail of binders\n")
                .Append("  t : count per entity type    r : per type/status result\n")
                .Append("  l : count per couple  type entity/result\n")
                .Append("  L : list  per couple  type entity/result\n")
                .Append("  *n  *s  *b  *t  *r  *l  *L : idem on ALL recorded items\n")
                .Append("  ?n  ?s  ?b  ?t ... : idem on abnormal items\n")
                .Append("  n select : n applied on a selection   idem for  s b t r l").Append('\n');
            Send(di, sout);
            return mod1 < -1 ? 1 : 0;
        }

        sout.Append("TransferRead :");
        if (TP.Model() != WS.Model())
        {
            sout.Append("Model differs from the session");
        }
        NCollection_HSequence<global::Kerf.Engine.Foundation.Transient>? list = IFSelect_Functions.GiveList(WS, CommandPart(argv, 2));
        // PrintStatsOnList sends its own message (the process's messenger) before the command's
        var aStats = new StringWriter();
        XSControl_TransferReader.PrintStatsOnList(TP, list, mod1, mod2, aStats);
        if (aStats.GetStringBuilder().Length > 0)
        {
            di.Append(aStats.ToString()).Append("\n");
        }
        Send(di, sout);
        return 0;
    }

    // XSControl_tpdraw
    private static int tpdraw(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        XSControl_WorkSession WS = Session(theDI);
        int argc = argv.Count;
        string arg1 = Arg(argv, 1), arg2 = Arg(argv, 2), arg3 = Arg(argv, 3);
        Transfer_TransientProcess? TP = WS.TransferReader()?.TransientProcess();
        var sout = new StringBuilder();
        if (TP is null)
        {
            sout.Append("No Transfer Read").Append('\n');
            Send(di, sout);
            return 1;
        }
        //        ****    tpdraw        ****
        if (argc < 2)
        {
            sout.Append("Donner [mode facultatif : item ou root] , NUMERO , nom DRAW facultatif").Append('\n');
            sout.Append("  mode si present : item ou root, sinon n0 d entite modele").Append('\n');
            sout.Append("  INTEGER NUMBER : of entity, of transfer item or of transfer root\n").Append("    or * to say all").Append('\n');
            Send(di, sout);
            return 1;
        }
        int mode = 0, num = 0;
        if (arg1.StartsWith('i'))
        {
            mode = 1;
        }
        else if (arg1.StartsWith('r'))
        {
            mode = 2;
        }
        bool tout = false;
        if (mode == 0)
        {
            if (arg1.StartsWith('*'))
            {
                tout = true;
            }
            else
            {
                num = IFSelect_Functions.GiveEntityNumber(WS, arg1);
            }
        }
        else
        {
            if (arg2.StartsWith('*'))
            {
                tout = true;
            }
            else
            {
                num = IFSelect_Functions.GiveEntityNumber(WS, arg2);
            }
        }

        int nbvar = 0;
        Transfer_Binder? binder = null;
        global::Kerf.Engine.Foundation.Transient? ent;
        Shape sh = new();
        string nomvar;

        int n1, n2, i, max = 0, index = 0;
        Interface_InterfaceModel? model = TP.Model();
        if (model is null)
        {
            if (mode == 0)
            {
                sout.Append("No model, specify n0 of transfer item").Append('\n');
                Send(di, sout);
                return 1;
            }
        }
        if (mode == 0)
        {
            sout.Append("Model entity");
            max = model!.NbEntities();
        }
        if (mode == 1)
        {
            sout.Append("Transfer item");
            max = TP.NbMapped();
        }
        if (mode == 2)
        {
            sout.Append("Transfer root");
            max = TP.NbRoots();
        }
        if (tout)
        {
            n1 = 1;
            n2 = max;
            sout.Append(", listing from 1 to ").Append(max).Append('\n');
        }
        else if (num <= 0 || num > max)
        {
            sout.Append(" - Num=").Append(num).Append(" hors limite (de 1 a ").Append(max).Append(")").Append('\n');
            Send(di, sout);
            return 1;
        }
        else
        {
            n1 = n2 = num;
            nbvar = -1; // nbvar : 1st simple shape = no n0
            sout.Append(", n0 ").Append(num).Append('\n');
        }

        for (i = n1; i <= n2; i++)
        {
            if (mode == 0)
            {
                ent = model!.Value(i);
                num = i;
                index = TP.MapIndex(ent);
            }
            else if (mode == 1)
            {
                ent = TP.Mapped(i);
                num = model is null ? 0 : model.Number(ent);
                index = i;
            }
            else if (mode == 2)
            {
                ent = TP.Root(i);
                num = model is null ? 0 : model.Number(ent);
                index = TP.MapIndex(ent);
            }

            if (index > 0)
            {
                binder = TP.MapItem(index);
            }
            if (binder is null)
            {
                index = 0;
            }
            if (index == 0)
            {
                if (!tout)
                {
                    sout.Append("Entite n0 ").Append(num).Append(" : non repertoriee").Append('\n');
                }
                continue;
            }
            if (!binder!.HasResult())
            {
                if (!tout)
                {
                    sout.Append("Entity n0 ").Append(num).Append(" : no result").Append('\n');
                }
                continue;
            }
            sh = TransferBRep.ShapeResult(binder);
            if (!sh.IsNull)
            {
                nbvar++;
                if (tout)
                {
                    sout.Append("[ ").Append(i).Append(" ]:");
                }
                if (num == 0)
                {
                    sout.Append(" pas dans le modele");
                }
                else
                {
                    sout.Append(" ent.n0 ").Append(num);
                }
                sout.Append(", item transfert n0 ").Append(index);
                if (nbvar == 0)
                {
                    nomvar = argc > 3 && mode > 0 ? arg3 : argc > 2 && mode == 0 ? arg2 : $"tp_{i}";
                }
                else
                {
                    nomvar = argc > 3 && mode > 0 ? $"{arg3}_{nbvar}" : argc > 2 && mode == 0 ? $"{arg2}_{nbvar}" : $"tp_{i}";
                }
                sout.Append(" -> 1 DRAW Shape: ").Append(nomvar).Append('\n');
                theDI.SetShape(nomvar, sh);
                continue;
            }
            if (binder is TransferBRep_ShapeListBinder slb)
            {
                int nbs = slb.NbShapes();
                if (tout)
                {
                    sout.Append("[ ").Append(i).Append(" ]:");
                }
                if (num == 0)
                {
                    sout.Append(" pas dans le modele");
                }
                else
                {
                    sout.Append(" ent.n0 ").Append(num);
                }
                sout.Append(", item transfert n0 ").Append(index);
                sout.Append(" -> ").Append(nbs).Append(" DRAW Shapes :");
                for (int j = 1; j <= nbs; j++)
                {
                    sh = slb.Shape(j);
                    if (nbvar < 0)
                    {
                        nbvar = 0;
                    }
                    nbvar++;
                    if (sh.IsNull)
                    {
                        sout.Append(" (no Shape recorded)").Append('\n');
                        continue;
                    }
                    nomvar = argc > 3 && mode > 0 ? $"{arg3}_{nbvar}" : argc > 2 && mode == 0 ? $"{arg2}_{nbvar}" : $"tp_{i}_{nbvar}";
                    sout.Append(' ').Append(nomvar);
                    theDI.SetShape(nomvar, sh);
                }
                sout.Append('\n');
                continue;
            }
            if (binder is Transfer_SimpleBinderOfTransient trb)
            {
                global::Kerf.Engine.Foundation.Transient? resu = trb.Result();
                if (resu is null)
                {
                    sout.Append("Entity n0 ").Append(num).Append(" : no result").Append('\n');
                    continue;
                }
                sout.Append("Entite n0 ").Append(num).Append(" : resultat ").Append(resu.DynamicTypeName());
                // OCCT binds a geometric result to a Draw variable (XSControl_Vars::Set): Draw-lite reports it only
                sout.Append('\n');
                continue;
            }

            if (sh.IsNull)
            {
                if (!tout)
                {
                    sout.Append("Entite n0 ").Append(num).Append(" : resultat pas une Shape mais ").Append(binder.ResultTypeName()).Append('\n');
                }
            }
        }

        if (sh.IsNull)
        {
            sout.Append(" (No Shape)").Append('\n');
        }
        Send(di, sout);
        return 0;
    }

    // XSControl_tpcompound
    private static int tpcompound(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        XSControl_WorkSession WS = Session(theDI);
        int argc = argv.Count;
        Transfer_TransientProcess? TP = WS.TransferReader()?.TransientProcess();
        var sout = new StringBuilder();
        if (TP is null)
        {
            sout.Append("No Transfer Read").Append('\n');
            Send(di, sout);
            return 1;
        }
        //        ****    tpcompound        ****
        if (argc < 2)
        {
            sout.Append("Give a NAME for the Compound  + optional givelist, else roots are taken").Append('\n');
            Send(di, sout);
            return 1;
        }
        NCollection_HSequence<Shape>? list;
        if (argc == 2)
        {
            list = TransferBRep.Shapes(TP);
        }
        else
        {
            NCollection_HSequence<global::Kerf.Engine.Foundation.Transient>? lise = IFSelect_Functions.GiveList(WS, CommandPart(argv, 2));
            if (lise is null)
            {
                sout.Append("Not a valid entity list : ").Append(CommandPart(argv, 2)).Append('\n');
                Send(di, sout);
                return 1;
            }
            list = TransferBRep.Shapes(TP, lise);
            sout.Append(lise.Length()).Append(" Entities, ");
        }
        if (list is null)
        {
            sout.Append("No Shape listed").Append('\n');
            Send(di, sout);
            return 1;
        }
        int nb = list.Length();
        sout.Append(nb).Append(" Shape(s) listed").Append('\n');
        var C = new Compound();
        var B = new BrepBuilder();
        B.MakeCompound(ref C);
        for (int i = 1; i <= nb; i++)
        {
            B.Add(C, list.Value(i));
        }
        theDI.SetShape(argv[1], C);
        Send(di, sout);
        return 0;
    }
}
