// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Reflection;
using System.Text;
using static Occt.IFSelect_ReturnStatus;

namespace OcctNet.DataExchange.Tests.XSControl;

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

    private static XSControl_WorkSession Read(string thePath)
    {
        _ = TestStepController.Instance;
        var WS = new XSControl_WorkSession();
        Assert.True(WS.SelectNorm("TestSTEP"));
        Assert.Equal(IFSelect_RetDone, TestStepController.ReadFile(WS, thePath));
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
        XSControl_WorkSession WS = Read(aPath);
        var aModel = (StepData_StepModel)WS.Model()!;
        var anExpected = new MemoryStream();
        var SW = new StepData_StepWriter(aModel);
        SW.SendModel((StepData_Protocol)WS.Protocol()!);
        Assert.True(SW.Print(anExpected));
        var anOut = new MemoryStream();
        Assert.Equal(IFSelect_RetDone, WS.SendAll("screw-out.step", false, anOut));
        Assert.Equal(anExpected.ToArray(), anOut.ToArray());
        // SendSelected: a new model with the selected entities and what they share
        var pointed = new IFSelect_SelectPointed();
        pointed.SetEntity(aModel.Value(1));
        anOut = new MemoryStream();
        Assert.Equal(IFSelect_RetDone, WS.SendSelected("part.step", pointed, false, anOut));
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
        XSControl_WorkSession WS = Read(aPath);
        var aModel = (StepData_StepModel)WS.Model()!;
        var anEditor = new APIHeaderSection_EditHeader();
        Assert.Equal("Step Header", anEditor.Label());
        IFSelect_EditForm aForm = anEditor.Form(false)!;
        Assert.True(aForm.LoadModel(aModel));
        string? aName = new APIHeaderSection_MakeHeader(aModel).Name()?.ToCString();
        Assert.Equal(aName, aForm.OriginalValue(1)!.ToCString());
        Assert.True(aForm.Modify(1, new TCollection_HAsciiString("renamed")));
        Assert.True(aForm.Modify(6, new TCollection_HAsciiString("OCCT.NET")));
        Assert.True(aForm.ApplyData(aForm.Entity(), aModel));
        Assert.Equal("renamed", new APIHeaderSection_MakeHeader(aModel).Name()!.ToCString());
        Assert.Equal("OCCT.NET", new APIHeaderSection_MakeHeader(aModel).OriginatingSystem()!.ToCString());
        // another kind of model: not loaded
        Assert.False(aForm.LoadModel(new Interface.ToyModel()));
    }

    [Fact]
    public void ReadingThroughTheSessionNeedsAModel()
    {
        // OCCT's StepSelect_WorkLibrary reads into the model it is given: XSControl_Reader::ReadFile (through
        // IFSelect_WorkSession::ReadFile) gives none, and the read fails on a null model, caught as a signal (RetFail).
        // STEPControl_Reader makes the model itself (unit 8F, as TestStepController.ReadFile here).
        string? aPath = DataStep("screw.step");
        if (aPath is null)
        {
            return;
        }
        _ = TestStepController.Instance;
        var aReader = new XSControl_Reader("TestSTEP");
        Assert.Equal(IFSelect_RetFail, aReader.ReadFile(aPath));
        Assert.Equal(0, aReader.WS()!.NbStartingEntities());
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
            XSControl_WorkSession WS = Read(aPath);
            WS.SetModeStat(i % 2 == 0);
            aCounts[i] = WS.GiveList("xst-model-roots")!.Length();
        });
        Assert.All(aCounts, n => Assert.Equal(aCounts[0], n));
    }
}
