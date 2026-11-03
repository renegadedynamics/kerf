// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Header;
using System.Reflection;
using System.Text;
using static Kerf.Engine.Exchange.Framework.Selection.ReturnStatus;

namespace Kerf.Kernel.DataExchange.Tests.XSControl;

/// <summary>
/// The work session on STEP models (unit 8A3), through the tests' STEP controller and library: reading, sending to a
/// stream (the StepData writer's bytes), the STEP header's editor, the reader's path OCCT's STEPControl_Reader avoids.
/// </summary>
[Collection(ProcessWideMessages.Name)]
public class XSControlStepTests
{
    private static string? DataStep(string theName)
    {
        string aDir = typeof(XSControlStepTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>()
                                                .Single(a => a.Key == "OcctSourceDir").Value!;
        string aPath = Path.Combine(aDir, "data", "step", theName);
        return File.Exists(aPath) ? aPath : null;
    }

    private static ExchangeWorkSession Read(string thePath)
    {
        _ = TestStepController.Instance;
        var WS = new ExchangeWorkSession();
        Assert.True(WS.SelectNorm("TestSTEP"));
        Assert.Equal(ReturnStatus.RetDone, TestStepController.ReadFile(WS, thePath));
        return WS;
    }

    [Fact]
    public void SendAllWritesTheModel()
    {
        string? aPath = DataStep("screw.step");
        if (aPath is null)
        {
            return;
        }
        ExchangeWorkSession WS = Read(aPath);
        var aModel = (StepModel)WS.Model()!;
        var anExpected = new MemoryStream();
        var SW = new StepWriter(aModel);
        SW.SendModel((StepProtocol)WS.Protocol()!);
        Assert.True(SW.Print(anExpected));
        var anOut = new MemoryStream();
        Assert.Equal(ReturnStatus.RetDone, WS.SendAll("screw-out.step", false, anOut));
        Assert.Equal(anExpected.ToArray(), anOut.ToArray());
        // SendSelected: a new model with the selected entities and what they share
        var pointed = new SelectPointed();
        pointed.SetEntity(aModel.Value(1));
        anOut = new MemoryStream();
        Assert.Equal(ReturnStatus.RetDone, WS.SendSelected("part.step", pointed, false, anOut));
        string aText = Encoding.UTF8.GetString(anOut.ToArray());
        Assert.StartsWith("ISO-10303-21;", aText, StringComparison.Ordinal);
        // DumpEntity goes to the library
        var aDump = new StringWriter();
        WS.DumpEntity(aModel.Value(1), 0, aDump);
        Assert.Contains("Dumping Entity n0 1", aDump.ToString());
    }

    [Fact]
    public void TheHeaderEditor()
    {
        string? aPath = DataStep("screw.step");
        if (aPath is null)
        {
            return;
        }
        ExchangeWorkSession WS = Read(aPath);
        var aModel = (StepModel)WS.Model()!;
        var anEditor = new EditHeader();
        Assert.Equal("Step Header", anEditor.Label());
        EditForm aForm = anEditor.Form(false)!;
        Assert.True(aForm.LoadModel(aModel));
        string? aName = new MakeHeader(aModel).Name()?.ToCString();
        Assert.Equal(aName, aForm.OriginalValue(1)!.ToCString());
        Assert.True(aForm.Modify(1, new TCollection_HAsciiString("renamed")));
        Assert.True(aForm.Modify(6, new TCollection_HAsciiString("OCCT.NET")));
        Assert.True(aForm.ApplyData(aForm.Entity(), aModel));
        Assert.Equal("renamed", new MakeHeader(aModel).Name()!.ToCString());
        Assert.Equal("OCCT.NET", new MakeHeader(aModel).OriginatingSystem()!.ToCString());
        // another kind of model: not loaded
        Assert.False(aForm.LoadModel(new Interface.ToyModel()));
    }

    [Fact]
    public void ReadingThroughTheSessionMakesAModel()
    {
        // The session gives the library no model (IFSelect_WorkSession::ReadFile): OCCT 8's StepSelect_WorkLibrary
        // read into it and failed (RetFail), so XSControl_Reader::ReadFile failed on STEP where STEPControl_Reader,
        // which makes the model itself, read. The library makes one for its protocol (occt-issues.md,
        // IFSelect_WorkSession::ReadFile with StepSelect_WorkLibrary; Kerf decision 16822): the generic reader reads
        // the file, with as many entities as STEPControl_Reader.
        string? aPath = DataStep("screw.step");
        if (aPath is null)
        {
            return;
        }
        var aWS = new ExchangeWorkSession();
        Assert.True(global::Kerf.Engine.Exchange.Step.StepController.SelectFor(aWS));
        var aReader = new FrameworkReader(aWS, false);
        Assert.IsType<global::Kerf.Engine.Exchange.Step.Selections.StepWorkLibrary>(aWS.WorkLibrary());
        Assert.NotNull(aWS.Protocol());
        Assert.Equal(RetDone, aReader.ReadFile(aPath));
        var aStepReader = new global::Kerf.Engine.Exchange.Step.StepReader();
        Assert.Equal(RetDone, aStepReader.ReadFile(aPath));
        Assert.Equal(aStepReader.WS()!.StartingEntityCount, aReader.WS()!.StartingEntityCount);
        Assert.True(aReader.WS()!.StartingEntityCount > 0);
    }

    [Fact]
    public void SessionsAreIndependent()
    {
        string? aPath = DataStep("screw.step");
        if (aPath is null)
        {
            return;
        }
        // sessions read in parallel: no process-wide state but the recorded controllers
        int[] aCounts = new int[4];
        Parallel.For(0, aCounts.Length, i =>
        {
            ExchangeWorkSession WS = Read(aPath);
            WS.SetModeStat(i % 2 == 0);
            aCounts[i] = WS.GiveList("xst-model-roots")!.Length();
        });
        Assert.All(aCounts, n => Assert.Equal(aCounts[0], n));
    }
}
