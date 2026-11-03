// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Header;
using Kerf.Engine.Exchange.Step.Parser;
using Kerf.Engine.Foundation;
using System.Text;
using OcctNet.DataExchange.Tests.StepGen;

namespace OcctNet.DataExchange.Tests.XSControl;

/// <summary>
/// A STEP work library for the tests, until unit 8F ports StepSelect_WorkLibrary: reading as it does
/// (StepFile_Read into the model given), writing the model with StepData_StepWriter (to the context's stream, or its
/// file), dumping an entity with StepData_StepDumper's level-free form (its label and type).
/// </summary>
internal sealed class TestStepLibrary : WorkLibrary
{
    public override int ReadFile(string name, ref InterfaceModel? model, Protocol? protocol,
                                 ProgressRange theProgress = default)
    {
        if (protocol is not StepProtocol stepro)
        {
            return 1;
        }
        return StepFileReader.Read(name, null, (StepModel)model!, stepro, theProgress);
    }

    public override int ReadStream(string theName, Stream theIStream, ref InterfaceModel? model,
                                   Protocol? protocol, ProgressRange theProgress = default)
    {
        if (protocol is not StepProtocol stepro)
        {
            return 1;
        }
        return StepFileReader.Read(theName, theIStream, (StepModel)model!, stepro, theProgress);
    }

    public override bool WriteFile(ContextWrite ctx)
    {
        if (ctx.Model() is not StepModel stepmodel || ctx.Protocol() is not StepProtocol stepro)
        {
            return false;
        }
        var SW = new StepWriter(stepmodel);
        SW.SendModel(stepro);
        if (ctx.OStream() is { } aStream)
        {
            return SW.Print(aStream);
        }
        using var aFile = File.Create(ctx.FileName());
        return SW.Print(aFile);
    }

    public override void DumpEntity(InterfaceModel? model, Protocol? protocol, Transient? entity,
                                    TextWriter S, int level)
    {
        S.Write(model!.StringLabel(entity)!.ToCString() + " " + entity!.DynamicTypeName());
    }
}

/// <summary>A STEP controller for the tests (unit 8F ports STEPControl_Controller), recorded once as "TestSTEP".</summary>
internal sealed class TestStepController : Controller
{
    public static readonly TestStepController Instance = Create();

    private static TestStepController Create()
    {
        var aCtl = new TestStepController();
        aCtl.AutoRecord();
        return aCtl;
    }

    private TestStepController() : base("TestSTEP", "tstep")
    {
        _adaptorLibrary = new TestStepLibrary();
        _adaptorProtocol = StepRoundTripTests.Protocol();
    }

    public override InterfaceModel? NewModel()
    {
        var aModel = new StepModel();
        aModel.SetProtocol(_adaptorProtocol);
        return aModel;
    }

    /// <summary>Reads a file as STEPControl_Reader::ReadFile does: a new STEP model with its header, then the library.</summary>
    public static ReturnStatus ReadFile(ExchangeWorkSession WS, string thePath)
    {
        var aModel = new StepModel();
        // STEPControl_Reader: the model's parameters from the statics (DESTEP's, declared by STEPControl_Controller in
        // unit 8F): here their defaults, with the UTF-8 code page
        aModel.SetSourceCodePage(aModel.InternalParameters.ReadCodePage);
        new MakeHeader().Apply(aModel);
        InterfaceModel? aRead = aModel;
        int stat;
        try
        {
            stat = WS.WorkLibrary()!.ReadFile(thePath, ref aRead, WS.Protocol());
        }
        catch (FailureException)
        {
            return ReturnStatus.RetFail;
        }
        ReturnStatus status = stat == 0 ? ReturnStatus.RetDone
                                     : stat < 0 ? ReturnStatus.RetError : ReturnStatus.RetFail;
        if (status != ReturnStatus.RetDone)
        {
            return status;
        }
        WS.SetModel(aModel);
        WS.SetLoadedFile(thePath);
        WS.InitTransferReader(4);
        return status;
    }
}

/// <summary>What tools/xscontrolrec writes for a STEP file (its format, line for line).</summary>
internal static class XSControlDump
{
    private static void DumpList(List<string> O, string theTag, InterfaceModel M, NCollection_HSequence<Transient>? L)
    {
        if (L is null)
        {
            O.Add(theTag + " null");
            return;
        }
        long aSum = 0;
        var aFirst = new StringBuilder();
        for (int i = 1; i <= L.Length(); i++)
        {
            int n = M.Number(L.Value(i));
            aSum = (aSum + (long)i * n) % 1000000007L;
            if (i <= 20)
            {
                aFirst.Append(' ').Append(n);
            }
        }
        O.Add($"{theTag} {L.Length()} {aSum}{aFirst}");
    }

    private static void DumpCounter(List<string> O, string theTag, SignatureList C)
    {
        NCollection_HSequence<TCollection_HAsciiString> aList = C.List()!;
        O.Add($"{theTag} {aList.Length()} {C.NullCount}");
        for (int i = 1; i <= aList.Length(); i++)
        {
            O.Add($"  {C.NbTimes(aList.Value(i)!.ToCString())} {aList.Value(i)!.ToCString()}");
        }
    }

    private static void Record(List<string> O, string thePath, bool theFull)
    {
        _ = TestStepController.Instance;
        var WS = new ExchangeWorkSession();
        Assert.True(WS.SelectNorm("TestSTEP"));
        WS.SetModeStat(theFull);
        ReturnStatus aStat = TestStepController.ReadFile(WS, thePath);
        InterfaceModel? M = WS.Model();
        int nb = M is null ? 0 : M.EntityCount;
        O.Add($"{(theFull ? "full" : "lite")} st {(int)aStat} n {nb}");
        if (aStat != ReturnStatus.RetDone || nb == 0)
        {
            return;
        }
        WS.AddNamedItem("rec-roots", new SelectModelRoots());
        WS.AddNamedItem("rec-shared", new SelectShared());
        WS.AddNamedItem("rec-sharing", new SelectSharing());
        WS.AddNamedItem("rec-type", new SelectionSignType(true));
        if (theFull)
        {
            DumpList(O, "roots", M!, WS.GiveList("rec-roots"));
            DumpList(O, "shared", M!, WS.GiveList("rec-shared rec-roots"));
            DumpList(O, "sharing1", M!, WS.GiveList("rec-sharing", "1"));
            var aCat = new SignCounter(new SignCategory(), false, false);
            aCat.AddModel(M);
            DumpCounter(O, "categories", aCat);
            return;
        }
        DumpList(O, "roots", M!, WS.GiveList("rec-roots"));
        var aTypes = new SignCounter(new SelectionSignType(false), false, false);
        aTypes.AddModel(M);
        DumpCounter(O, "types", aTypes);
        var aValid = new SignCounter(new SignValidity(), false, false);
        aValid.AddModel(M);
        DumpCounter(O, "validity", aValid);
        CheckIterator aChl = WS.ModelCheckList();
        int aNbItems = 0, aNbFails = 0, aNbWarns = 0;
        for (aChl.Start(); aChl.More(); aChl.Next())
        {
            aNbItems++;
            aNbFails += aChl.Value()!.FailCount;
            aNbWarns += aChl.Value()!.WarningCount;
        }
        O.Add($"checks {aNbItems} {aNbFails} {aNbWarns}");
        for (aChl.Start(); aChl.More(); aChl.Next())
        {
            FrameworkCheck aCh = aChl.Value()!;
            O.Add($"  {aChl.Number()} {aCh.FailCount} {aCh.WarningCount} {(aCh.FailCount > 0 ? aCh.CFail(1) : aCh.CWarning(1))}");
        }
        int aMid = (nb + 1) / 2;
        TCollection_HAsciiString? aLab = M!.StringLabel(M.Value(aMid));
        O.Add($"label {aMid} {aLab?.ToCString() ?? ""} {(aLab is null ? 0 : WS.NumberFromLabel(aLab.ToCString()))}");
        NCollection_HSequence<Transient>? aCp = WS.GiveList("rec-type(CartesianPoint)");
        O.Add($"sel-cp {(aCp is null ? -1 : aCp.Length())}");
    }

    /// <summary>The dump of one file, as xscontrolrec writes it (its "F" line apart).</summary>
    public static List<string> Dump(string thePath)
    {
        var O = new List<string>();
        try
        {
            Record(O, thePath, false);
            Record(O, thePath, true);
        }
        catch (FailureException anExc)
        {
            O.Add("exception " + anExc.Message);
        }
        return O;
    }
}
