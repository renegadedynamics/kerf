// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using System.Text;
using OcctNet.DataExchange.Tests.IFSelect;
using OcctNet.DataExchange.Tests.Interface;
using OcctNet.DataExchange.Tests.Transfer;
using static Occt.IFSelect_PrintCount;
using static Occt.IFSelect_ReturnStatus;
using static Occt.Interface_CheckStatus;

namespace OcctNet.DataExchange.Tests.XSControl;

/// <summary>
/// XSControl on the toy norm (unit 8A3): a controller recorded once, sessions customised by it (their named items,
/// their parameters in their own parameter set), the transfer reader and writer, the reader and writer, the
/// selections and signatures on transfers.
/// </summary>
[Collection(ProcessWideMessages.Name)]
public class XSControlTests
{
    private const string ToyFile = "A B C\nB C\nC\nD E\nE D\nF\n?H\n";

    private static string Names(NCollection_HSequence<Transient>? theList) =>
        theList is null ? "null" : string.Join(" ", Enumerable.Range(1, theList.Length()).Select(i => theList.Value(i)!.ToString()));

    private static XSControl_WorkSession Session(Interface_StaticSet? theStatics = null)
    {
        _ = ToyController.Instance;
        XSControl_WorkSession WS = theStatics is null ? new XSControl_WorkSession() : new XSControl_WorkSession(theStatics);
        Assert.True(WS.SelectNorm("toy"));
        return WS;
    }

    private static XSControl_WorkSession Loaded(string theText = ToyFile)
    {
        XSControl_WorkSession WS = Session();
        using var aStream = new MemoryStream(Encoding.UTF8.GetBytes(theText));
        Assert.Equal(IFSelect_RetDone, WS.ReadStream("toy.txt", aStream));
        WS.InitTransferReader(4);
        return WS;
    }

    [Fact]
    public void ControllersAreRecordedOnce()
    {
        ToyController aCtl = ToyController.Instance;
        Assert.Same(aCtl, XSControl_Controller.Recorded("toy"));
        Assert.Same(aCtl, XSControl_Controller.Recorded("ToyNorm"));
        Assert.Null(XSControl_Controller.Recorded("no-such-norm"));
        aCtl.Record("toy"); // again: nothing changes
        Assert.Same(aCtl, XSControl_Controller.Recorded("toy"));
        Assert.Equal("ToyNorm", aCtl.Name());
        Assert.Equal("toy", aCtl.Name(true));
        Assert.True(aCtl.ModeWriteBounds(out int aMin, out int aMax));
        Assert.Equal((0, 1), (aMin, aMax));
        Assert.True(aCtl.IsModeWrite(1));
        Assert.False(aCtl.IsModeWrite(2));
        Assert.Equal("upper case", aCtl.ModeWriteHelp(1));
        Assert.Equal("", aCtl.ModeWriteHelp(0));
        Assert.False(new XSControl_WorkSession().SelectNorm("no-such-norm"));
    }

    [Fact]
    public void SelectNormCustomisesTheSession()
    {
        XSControl_WorkSession WS = Session();
        Assert.Same(ToyController.Instance, WS.NormAdaptor());
        Assert.Equal("ToyNorm", WS.SelectedNorm());
        Assert.Equal("toy", WS.SelectedNorm(true));
        Assert.IsType<ToyLibrary>(WS.WorkLibrary());
        Assert.Same(ToyNorm.Protocol, WS.Protocol());
        Assert.IsType<IFSelect_SelectModelEntities>(WS.NamedItem("xst-model-all"));
        Assert.IsType<IFSelect_SelectModelRoots>(WS.NamedItem("xst-model-roots"));
        Assert.IsType<XSControl_SelectForTransfer>(WS.NamedItem("xst-transferrable-roots")); // not a "STEP" norm
        Assert.IsType<XSControl_SelectForTransfer>(WS.NamedItem("xst-transferrable-all"));
        Assert.IsType<XSControl_SignTransferStatus>(WS.NamedItem("xst-transfer-status"));
        Assert.IsType<XSControl_ConnectedShapes>(WS.NamedItem("xst-connected-faces"));
        Assert.IsType<IFSelect_SignCounter>(WS.NamedItem("xst-types"));
        Assert.IsType<IFSelect_DispPerCount>(WS.NamedItem("xst-disp-count"));
        Assert.Equal(5, ((IFSelect_DispPerCount)WS.NamedItem("xst-disp-count")!).CountValue());
        Assert.Equal(10, ((IFSelect_DispPerFiles)WS.NamedItem("xst-disp-files")!).CountValue());
        Assert.Same(WS.NamedItem("xst-long-type"), WS.SignType());
        Assert.Equal(4, WS.ShareOut()!.NbDispatches()); // the dispatches are active
        // the parameters: the controller's statics are declared in the session's set
        Assert.Equal(3, WS.Statics().IVal("toy.param"));
        var params_ = (IFSelect_ParamEditor)WS.NamedItem("xst-params-edit")!;
        Assert.Equal(5, params_.NbValues()); // read/write precision mode and value, toy.param
        Assert.Equal(1, params_.NameNumber("read.precision.mode"));
        Assert.Equal(5, params_.NameNumber("toy.param"));
        Assert.IsType<IFSelect_EditForm>(WS.NamedItem("xst-params-read"));   // uses 2 and 5
        Assert.IsType<IFSelect_EditForm>(WS.NamedItem("xst-params-write"));  // use 6
        Assert.Null(WS.NamedItem("xst-params-general"));
        var all = (IFSelect_ParamEditor)WS.NamedItem("xst-static-params-edit")!;
        Assert.Equal(WS.Statics().Items().Length(), all.NbValues());
        // editing a parameter edits the session's set, not another session's
        var form = (IFSelect_EditForm)WS.NamedItem("xst-params-all")!;
        Assert.True(form.LoadData());
        Assert.True(form.Modify(5, new TCollection_HAsciiString("8")));
        Assert.True(form.Apply());
        Assert.Equal(8, WS.Statics().IVal("toy.param"));
        Assert.Equal(3, Session().Statics().IVal("toy.param"));
        // selecting the same norm again keeps the items
        int nb = WS.MaxIdent();
        Assert.True(WS.SelectNorm("ToyNorm"));
        Assert.Equal(nb, WS.MaxIdent());
        // a session given a parameter set: the norm's statics are declared in it
        var aSet = new Interface_StaticSet();
        XSControl_WorkSession WS2 = Session(aSet);
        Assert.Same(aSet, WS2.Statics());
        Assert.True(aSet.IsPresent("toy.param"));
        Assert.Same(aSet, WS2.TransferReader()!.Statics());
    }

    [Fact]
    public void ReadsAndTransfers()
    {
        XSControl_WorkSession WS = Loaded();
        Assert.Equal(7, WS.NbStartingEntities());
        XSControl_TransferReader TR = WS.TransferReader()!;
        Assert.Same(WS.Model(), TR.Model());
        Assert.IsType<ToyActor>(TR.Actor());
        // the selections of transfers: those the actor recognizes
        Assert.Equal("A F H", Names(WS.GiveList("xst-transferrable-roots")));
        Assert.Equal(7, WS.GiveList("xst-transferrable-all")!.Length());
        // transfer one, then the roots
        Assert.Equal(1, WS.TransferReadOne(WS.StartingEntity(2)));
        Assert.True(TR.IsRecorded(WS.StartingEntity(2)));
        Assert.Equal("b", ((TCollection_HAsciiString)TR.TransientResult(WS.StartingEntity(2))!).ToCString());
        Assert.Equal("b", ((TCollection_HAsciiString)WS.Result(WS.StartingEntity(2), 0)!).ToCString());
        Assert.IsType<Transfer_ResultFromModel>(WS.Result(WS.StartingEntity(2), 20));
        Assert.Equal("B", Names(TR.RecordedList()));
        Assert.Equal(0, WS.TransferReadRoots()); // the toy's results are no shapes: none counted
        Assert.Equal("A B F H", Names(TR.RecordedList())); // the roots (the toy actor doesn't go deeper) and B
        Assert.True(TR.HasResult(WS.StartingEntity(1)));
        Assert.Equal(1, TR.FinalEntityNumber(WS.StartingEntity(1)));
        Assert.Same(WS.StartingEntity(2), TR.EntityFromResult(TR.TransientProcess()!.Find(WS.StartingEntity(2)), 1));
        Assert.Same(WS.StartingEntity(2), TR.EntityFromResult(TR.TransientResult(WS.StartingEntity(2)), 1));
        // the transfer status signature
        var status = (XSControl_SignTransferStatus)WS.NamedItem("xst-transfer-status")!;
        Assert.Equal("Result:TCollection_HAsciiString", status.Value(WS.StartingEntity(1), WS.Model()));
        Assert.Equal("Result:TCollection_HAsciiString/Warning", status.Value(WS.StartingEntity(7), WS.Model()));
        // checks: H warns
        Assert.True(TR.HasChecks(WS.StartingEntity(7), false));
        Assert.False(TR.HasChecks(WS.StartingEntity(7), true));
        Assert.Equal("H", Names(TR.CheckedList(WS.Model(), Interface_CheckWarning, true)));
        Interface_CheckIterator chl = TR.CheckList(WS.Model());
        Assert.Equal("XSControl : CheckList complete Model", chl.Name());
        Assert.False(chl.IsEmpty(false));
        Assert.False(TR.LastCheckList().IsEmpty(false));
        var aText = new StringWriter();
        TR.PrintStats(aText, 10, 0);
        Assert.Contains("Nb Recorded : 4", aText.ToString());
        // the statistics of the transient process (Draw's tpstat): every mode runs
        var aPrinter = new OcctNet.DataExchange.Tests.Transfer.StringPrinter();
        TR.TransientProcess()!.Messenger().AddPrinter(aPrinter);
        for (int what = 0; what <= 5; what++)
        {
            for (int mode = 0; mode <= 6; mode++)
            {
                XSControl_TransferReader.PrintStatsProcess(TR.TransientProcess(), what, mode);
            }
        }
        XSControl_TransferReader.PrintStatsOnList(TR.TransientProcess(), WS.GiveList("(1,2)"), 2, 5);
        Assert.Contains("Statistics on Transfer Process (Read)", aPrinter.Text.ToString());
        Assert.Contains("Percentages according Transfer Status", aPrinter.Text.ToString());
        TR.TransientProcess()!.Messenger().RemovePrinter(aPrinter);
        var aStatus = new StringWriter();
        Assert.True(WS.PrintTransferStatus(1, false, aStatus));
        Assert.Contains("Transfer Read item n0.1", aStatus.ToString());
        Assert.False(WS.PrintTransferStatus(1, true, aStatus)); // nothing written
        // skipping, clearing a result
        Assert.True(TR.Skip(WS.StartingEntity(6)));
        Assert.True(TR.IsSkipped(WS.StartingEntity(6)));
        Assert.True(TR.ClearResult(WS.StartingEntity(1), -1));
        Assert.False(TR.IsMarked(WS.StartingEntity(1)));
        // the context of the transfers goes to the transient process at the next transfer
        WS.SetAllContext(new NCollection_DataMap<string, Transient>());
        TR.SetContext("ctx", new TCollection_HAsciiString("v"));
        Assert.True(TR.BeginTransfer());
        Transient? aCtx = null;
        Assert.True(TR.TransientProcess()!.GetContext("ctx", typeof(TCollection_HAsciiString), ref aCtx));
        // TransferClear on the model clears the process
        TR.TransferClear(WS.Model());
        Assert.Equal(0, TR.TransientProcess()!.NbMapped());
    }

    [Fact]
    public void ReaderAndWriter()
    {
        _ = ToyController.Instance;
        var reader = new XSControl_Reader("toy");
        using (var aStream = new MemoryStream(Encoding.UTF8.GetBytes(ToyFile)))
        {
            Assert.Equal(IFSelect_RetDone, reader.ReadStream("toy.txt", aStream));
        }
        Assert.Equal(7, reader.Model()!.NbEntities());
        Assert.Equal(3, reader.NbRootsForTransfer()); // A, F, H: the roots the actor recognizes
        Assert.Same(reader.Model()!.Value(6), reader.RootForTransfer(2));
        Assert.Null(reader.RootForTransfer(4));
        Assert.Equal("A F H", Names(reader.GiveList()));
        Assert.Equal(0, reader.GiveList("xst-shared", reader.GiveList("(1)"))!.Length()); // not in full mode: none
        Assert.Equal(3, reader.TransferRoots());
        Assert.Equal(3, reader.NbShapes());
        Assert.True(reader.Shape(1).IsNull); // the toy's results are no shapes (null shapes are kept)
        Assert.True(reader.OneShape().IsNull);
        Assert.True(reader.TransferOne(2));
        Assert.Equal(4, reader.NbShapes());
        Assert.False(reader.TransferEntity(null));
        reader.ClearShapes();
        Assert.Equal(1, reader.TransferList(reader.GiveList("(3)")));
        reader.GetStatsTransfer(null, out int nbMapped, out int nbWithResult, out int nbWithFail);
        Assert.Equal((5, 5, 0), (nbMapped, nbWithResult, nbWithFail)); // A F H, B, C
        var aText = new StringWriter();
        reader.PrintCheckTransfer(aText, false, IFSelect_ItemsByEntity);
        Assert.Contains("warns H", aText.ToString());
        // the shape fix parameters go to the actor
        Assert.True(reader.GetShapeFixParameters().IsEmpty() == false); // set from the defaults at the first transfer
        Assert.True(reader.GetShapeProcessFlags().Item2);

        // the writer: a transient transferred through the controller's actor into a new model, then written
        var writer = new XSControl_Writer("toy");
        XSControl_WorkSession WS = writer.WS()!;
        Assert.Equal(0, writer.Model()!.NbEntities());
        Assert.True(WS.TransferWriter()!.RecognizeTransient(new TCollection_HAsciiString("x")));
        Assert.Equal(IFSelect_RetDone, WS.TransferWriter()!.TransferWriteTransient(writer.Model(), new TCollection_HAsciiString("x")));
        Assert.Equal(IFSelect_RetVoid, WS.TransferWriter()!.TransferWriteTransient(null, new TCollection_HAsciiString("y")));
        Assert.Equal(1, writer.Model()!.NbEntities());
        Assert.Equal("X", writer.Model()!.Value(1)!.ToString());
        WS.ComputeGraph(true);
        var anOut = new MemoryStream();
        Assert.Equal(IFSelect_RetDone, writer.WriteStream(anOut));
        Assert.Equal("X\n", Encoding.UTF8.GetString(anOut.ToArray()));
        Assert.True(WS.TransferWriteCheckList().IsEmpty(false));
        // a null shape: void
        Assert.Equal(IFSelect_RetVoid, writer.TransferShape(new Shape()));
        // a new model clears the transfers
        Assert.NotNull(WS.NewModel());
        Assert.Equal(0, WS.TransferWriter()!.FinderProcess()!.NbMapped());
    }

    [Fact]
    public void VarsAndUtils()
    {
        var vars = new XSControl_Vars();
        vars.Set("a", new TCollection_HAsciiString("x"));
        Assert.IsType<TCollection_HAsciiString>(vars.Get("a"));
        Assert.Null(vars.Get("b"));
        vars.SetPoint("p", new Point3D(1, 2, 3));
        Point3D p = default;
        Assert.True(vars.GetPoint("p", ref p));
        Assert.Equal(2.0, p.Y());
        Assert.False(vars.GetPoint("a", ref p));
        Shape aBox = new BRepPrimAPI_MakeBox(1, 2, 3).Shape();
        vars.SetShape("s", aBox);
        Assert.True(vars.GetShape("s").IsSame(aBox));
        Assert.True(vars.GetShape("a").IsNull);

        var xu = new XSControl_Utils();
        Assert.Equal("TCollection_HAsciiString", xu.TypeName(new TCollection_HAsciiString("x")));
        Assert.Equal("HAsciiString", xu.TypeName(new TCollection_HAsciiString("x"), true));
        Assert.Equal(ShapeType.Solid, xu.ShapeType(aBox, true));
        Shape faces = xu.SortedCompound(aBox, ShapeType.Face, true, false);
        Assert.Equal(ShapeType.Compound, faces.ShapeType());
        Assert.Equal(ShapeType.Face, xu.ShapeType(faces, true));
        Assert.True(xu.BinderShape(xu.ShapeBinder(aBox)).IsSame(aBox));
        Assert.True(xu.BinderShape(xu.ShapeBinder(aBox, false)).IsSame(aBox));
        NCollection_HSequence<TCollection_HAsciiString> seq = xu.NewSeqCStr();
        xu.AppendCStr(seq, "a");
        xu.AppendCStr(seq, "b");
        Assert.Equal(2, xu.SeqLength(seq));
        Assert.Equal("b", xu.CStrValue(seq, 2));
        Assert.Equal("", xu.CStrValue(seq, 3));
        Assert.Equal("1999-12-31:23-59-58", xu.DateString(1999, 12, 31, 23, 59, 58));
        // the faces connected to a face, through the transfer of the toy... a direct use of AdjacentEntities needs a
        // transient process of shapes: bind the box's faces as results of toy entities
        var TP = new Transfer_TransientProcess();
        ToyModel M = ToyNorm.Model(new ToyEntity("f1"), new ToyEntity("f2"));
        TP.SetModel(M);
        var aFaces = new List<Shape>();
        for (var anExp = new Explorer(aBox, ShapeType.Face); anExp.More(); anExp.Next())
        {
            aFaces.Add(anExp.Current());
        }
        Occt.TransferBRep.SetShapeResult(TP, M.Value(1), aFaces[0]);
        Occt.TransferBRep.SetShapeResult(TP, M.Value(2), aFaces[2]);
        NCollection_HSequence<Transient>? adj = XSControl_ConnectedShapes.AdjacentEntities(aFaces[0], TP, ShapeType.Face);
        Assert.Equal("f1 f2", Names(adj)); // the box's faces 1 and 3 share an edge
    }
}
