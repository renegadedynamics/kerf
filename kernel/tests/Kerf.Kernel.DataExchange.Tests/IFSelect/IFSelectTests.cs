// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Exchange.Framework.Graph;
using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Exchange.Step.Header;
using Kerf.Engine.Foundation;
using System.Text;
using Kerf.Kernel.DataExchange.Tests.Interface;
using static Kerf.Engine.Exchange.Framework.Selection.EditValue;
using static Kerf.Engine.Exchange.Framework.Selection.PrintCount;
using static Kerf.Engine.Exchange.Framework.Selection.ReturnStatus;

namespace Kerf.Kernel.DataExchange.Tests.IFSelect;

/// <summary>
/// IFSelect on the toy norm (unit 8A3): the work session (reading, the graph, items and names, GiveList, checks,
/// sending), selections, signatures and counters, dispatches, the share-out and the model copier with modifiers,
/// editors and parameters. The toy file: A shares B and C, B shares C, D and E share each other, F is alone, G fails
/// its check and H warns.
/// </summary>
[Collection(ProcessWideMessages.Name)]
public class IFSelectTests
{
    internal const string ToyFile = "A B C\nB C\nC\nD E\nE D\nF\n!G\n?H\n";

    private static string Names(NCollection_HSequence<Transient>? theList) =>
        theList is null ? "null" : string.Join(" ", Enumerable.Range(1, theList.Length()).Select(i => theList.Value(i)!.ToString()));

    // OCCT's selections deducing lists from a graph (shared, sharing, complete results) need its statuses: a session
    // computes them in "full mode" only (SetModeStat, as Draw's stepread with "f")
    internal static WorkSession Session(string theText = ToyFile, bool theModeStat = true)
    {
        var WS = new WorkSession();
        WS.SetModeStat(theModeStat);
        WS.SetLibrary(new ToyLibrary());
        WS.SetProtocol(ToyNorm.ProtocolInstance);
        using var aStream = new MemoryStream(Encoding.UTF8.GetBytes(theText));
        Assert.Equal(ReturnStatus.RetDone, WS.ReadStream("toy.txt", aStream));
        return WS;
    }

    [Fact]
    public void ReadsThroughTheLibrary()
    {
        var WS = new WorkSession();
        using var aStream = new MemoryStream(Encoding.UTF8.GetBytes(ToyFile));
        Assert.Equal(ReturnStatus.RetVoid, WS.ReadStream("x", aStream)); // no library
        WS.SetLibrary(new ToyLibrary());
        Assert.Equal(ReturnStatus.RetVoid, WS.ReadStream("x", aStream)); // no protocol
        WS = Session();
        Assert.Equal("toy.txt", WS.LoadedFile());
        Assert.True(WS.HasModel);
        Assert.Equal(8, WS.StartingEntityCount);
        Assert.True(WS.IsLoaded());
        Assert.Equal("B", WS.StartingEntity(2)!.ToString());
        Assert.Null(WS.StartingEntity(9));
        Assert.Equal(3, WS.StartingNumber(WS.StartingEntity(3)));
        Assert.Equal(8, WS.Graph().Size());
        Assert.Equal("B C", Names(WS.Shareds(WS.StartingEntity(1))));
        Assert.Equal("A B", Names(WS.Sharings(WS.StartingEntity(3))));
        Assert.Equal("A", WS.EntityName(WS.StartingEntity(1))!.ToCString());
        Assert.Equal("#A", WS.EntityLabel(WS.StartingEntity(1))!.ToCString());
        // a missing file: the library's status -1 is RetError
        Assert.Equal(ReturnStatus.RetError, WS.ReadFile(Path.Combine(Path.GetTempPath(), "no-such-toy-file.txt")));
    }

    [Fact]
    public void NumbersFromLabels()
    {
        WorkSession WS = Session();
        Assert.Equal(3, WS.NumberFromLabel("3"));
        Assert.Equal(6, WS.NumberFromLabel("#F")); // the labels, case-insensitive, not exact
        Assert.Equal(6, WS.NumberFromLabel("#f"));
        Assert.Equal(0, WS.NumberFromLabel("#")); // no label ends so
        Assert.Equal(0, WS.NumberFromLabel("zz"));
    }

    [Fact]
    public void ItemsAndNames()
    {
        var WS = new WorkSession();
        var sel = new SelectModelRoots();
        var par = new IntParam();
        Assert.Equal(1, WS.AddNamedItem("roots", sel));
        Assert.Equal(2, WS.AddItem(par));
        Assert.Equal(0, WS.AddNamedItem("#x", par)); // reserved names
        Assert.Equal(0, WS.AddNamedItem("!x", par));
        Assert.Equal(2, WS.MaxIdent());
        Assert.Same(sel, WS.NamedItem("roots"));
        Assert.Same(par, WS.NamedItem("#2"));
        Assert.Null(WS.NamedItem("nothing"));
        Assert.Equal(1, WS.NameIdent("roots"));
        Assert.Equal(2, WS.NameIdent("#2"));
        Assert.True(WS.HasName(sel));
        Assert.False(WS.HasName(par));
        Assert.Equal("roots", WS.Name(sel)!.ToCString());
        // naming an item already there: the name replaces its attribute
        Assert.Equal(2, WS.AddNamedItem("count", par));
        Assert.Equal("count", WS.Name(par)!.ToCString());
        Assert.Equal("Selection:Root (not shared) Entities from Model", WS.ItemLabel(1)!.ToCString());
        Assert.Equal("Integer:0", WS.ItemLabel(2)!.ToCString());
        Assert.Equal([1], Seq(WS.ItemIdents(typeof(AbstractSelection))));
        Assert.Equal(["roots"], StrSeq(WS.ItemNames(typeof(AbstractSelection))));
        Assert.Equal(1, WS.NextIdentForLabel("Selection", 0, 1));
        Assert.Equal(2, WS.NextIdentForLabel("Integer:0", 0, 0));
        Assert.Equal(0, WS.NextIdentForLabel("Integer", 2, 2));
        // removing: the item stays in the map, marked removed
        Assert.True(WS.RemoveNamedItem("roots"));
        Assert.Null(WS.NamedItem("roots"));
        Assert.Equal(0, WS.ItemIdent(sel));
        Assert.Null(WS.Item(1));
        Assert.Equal(2, WS.MaxIdent());
        Assert.False(WS.RemoveItem(sel)); // already removed
        // RemoveName keeps the item, unnamed; OCCT re-adds it to its indexed map, which keeps the old attribute
        Assert.True(WS.RemoveName("count"));
        Assert.Equal(2, WS.ItemIdent(par));
        Assert.Null(WS.NamedItem("count"));
        Assert.True(WS.HasName(par));
        WS.ClearItems();
        Assert.Equal(0, WS.MaxIdent());
    }

    private static int[] Seq(NCollection_HSequence<int>? theSeq) =>
        Enumerable.Range(1, theSeq!.Length()).Select(i => theSeq.Value(i)).ToArray();

    private static string[] StrSeq(NCollection_HSequence<TCollection_HAsciiString>? theSeq) =>
        Enumerable.Range(1, theSeq!.Length()).Select(i => theSeq.Value(i)!.ToCString()).ToArray();

    [Fact]
    public void Parameters()
    {
        var WS = new WorkSession();
        IntParam? ip = WS.NewIntParam("ip");
        Assert.True(WS.SetIntValue(ip, 7));
        Assert.Equal(7, WS.IntValue(ip));
        Assert.False(WS.SetIntValue(new IntParam(), 1)); // not an item
        TCollection_HAsciiString? tp = WS.NewTextParam("tp");
        Assert.True(WS.SetTextValue(tp, "hello"));
        Assert.Equal("hello", WS.TextValue(tp));
        Assert.Equal("Text:hello", WS.ItemLabel(WS.ItemIdent(tp))!.ToCString());
        // a parameter bound to a static of the session's parameter set
        Assert.True(WS.Statics().Init("test", "test.int", 'i', "4"));
        Assert.IsType<IntParam>(WS.NewParamFromStatic("test.int", "pm"));
        var pm = (IntParam)WS.NamedItem("pm")!;
        Assert.Equal("test.int", pm.StaticName());
        Assert.Equal(4, pm.Value());
        pm.SetValue(1);
        Assert.Equal(1, WS.Statics().IVal("test.int"));
        // an enumeration: its text value
        Assert.IsType<TCollection_HAsciiString>(WS.NewParamFromStatic("read.precision.mode", "pe"));
        // a text static: its value itself (shared)
        var tv = (TCollection_HAsciiString)WS.NewParamFromStatic("read.precision.val", "pv")!;
        Assert.Equal(WS.Statics().CVal("read.precision.val"), tv.ToCString());
        Assert.Null(WS.NewParamFromStatic("no.such.static"));
        // another session's parameters are its own
        Assert.Equal(new StaticSet().IVal("read.precision.mode"), new WorkSession().Statics().IVal("read.precision.mode"));
    }

    [Fact]
    public void Selections()
    {
        WorkSession WS = Session();
        EntityGraph G = WS.Graph();
        Assert.Equal("A F G H", ToyNorm.Names(new SelectModelRoots().RootResult(G)));
        Assert.Equal("A B C D E F G H", ToyNorm.Names(new SelectModelEntities().UniqueResult(G)));
        var roots = new SelectModelRoots();
        var shared = new SelectShared();
        shared.SetInput(roots);
        Assert.Equal("B C", ToyNorm.Names(shared.UniqueResult(G)));
        var pointed = new SelectPointed();
        Assert.True(pointed.Add(WS.StartingEntity(3)));
        Assert.False(pointed.Add(WS.StartingEntity(3)));
        Assert.True(pointed.Add(WS.StartingEntity(4)));
        Assert.Equal(2, pointed.ItemCount);
        var sharing = new SelectSharing();
        sharing.SetInput(pointed);
        Assert.Equal("A B E", ToyNorm.Names(sharing.UniqueResult(G)));
        Assert.Equal("Sharing Entities (one level)", sharing.Label());
        Assert.True(pointed.Toggle(WS.StartingEntity(5)));
        Assert.False(pointed.Toggle(WS.StartingEntity(5)));
        Assert.Equal(1, pointed.Rank(WS.StartingEntity(3)));
        // the alternate input of a deduction: used once
        shared.Alternate()!.SetEntity(WS.StartingEntity(2));
        Assert.True(shared.HasAlternate());
        Assert.Equal("C", ToyNorm.Names(shared.UniqueResult(G)));
        Assert.False(shared.HasAlternate());
        Assert.Equal("B C", ToyNorm.Names(shared.UniqueResult(G)));
        // an extraction by signature
        var byname = new SelectSignature(new SelectionSignType(), "ToyEntity");
        byname.SetInput(new SelectModelEntities());
        Assert.Equal(8, byname.UniqueResult(G).NbEntities());
        byname.SetDirect(false);
        Assert.Equal(0, byname.UniqueResult(G).NbEntities());
        Assert.Equal("Removed: Dynamic Type matching ToyEntity", byname.Label());
        // CompleteResult: what the selection's result shares
        Assert.Equal("A B C F G H", ToyNorm.Names(roots.CompleteResult(G)));
        // the sources of a selection
        SelectionIterator it = new(shared);
        Assert.True(it.More());
        Assert.Same(roots, it.Value());
    }

    [Fact]
    public void SignaturesAndCounters()
    {
        WorkSession WS = Session();
        InterfaceModel M = WS.Model()!;
        Transient A = WS.StartingEntity(1)!;
        Assert.Equal("ToyEntity", new SelectionSignType().Value(A, M));
        Assert.Equal("ToyEntity", new SelectionSignType(true).Value(A, M)); // the part after the first '_' (none here)
        Assert.Equal("HAsciiString", new SelectionSignType(true).Value(new TCollection_HAsciiString("x"), M));
        Assert.True(new SignAncestor().Matches(A, M, "Standard_Transient", true));
        Assert.False(new SignAncestor().Matches(A, M, "Interface_Check", true));
        Assert.Equal("OK", SignValidity.CVal(A, M));
        // G fails its check, but the model's semantic checks of G and H are the same object, H's (see Checks)
        Assert.Equal("Data-Warning", SignValidity.CVal(WS.StartingEntity(7), M));
        Assert.Equal("Data-Warning", SignValidity.CVal(WS.StartingEntity(8), M));
        Assert.True(Signature.MatchValue("ToyEntity", "Ent", false));
        Assert.False(Signature.MatchValue("ToyEntity", "Ent", true));
        Assert.Throws<OutOfRangeException>(() => Signature.MatchValue("ToyEntity", "", false)); // OCCT
        Assert.Equal("12", Signature.IntValue(12));
        // a counter of validities
        var cnt = new SignCounter(new SignValidity(), true, true);
        cnt.AddModel(M);
        Assert.Equal("Validity", cnt.Name());
        Assert.Equal(6, cnt.NbTimes("OK"));
        Assert.Equal(2, cnt.NbTimes("Data-Warning"));
        Assert.Equal(["OK", "Data-Warning"], StrSeq(cnt.List()));
        Assert.Equal("G H", Names(cnt.Entities("Data-Warning")));
        var aText = new StringWriter();
        cnt.PrintCount(aText);
        Assert.Equal(" Count\tValidity\n -----\t-----------\n     6\tOK\n     2\tData-Warning\n    Nb Total:8  for 2 items\n",
                     aText.ToString().Replace("\r", ""));
        // a counter of the shared entities of each entity (GraphCounter)
        var gc = new GraphCounter();
        var shared = new SelectShared();
        gc.SetApplied(shared);
        gc.AddWithGraph(new SelectModelEntities().UniqueResult(WS.Graph()).Content(), WS.Graph());
        Assert.Equal(["2", "1", "0"], StrSeq(gc.List()));
        Assert.Equal(1, gc.NbTimes("2"));
        // ComputedSign: the signature of one entity
        Assert.Equal("OK", cnt.ComputedSign(A, WS.Graph()));
        // AddModel on an empty counter doesn't fill its map (each entity of a model is unique); ComputedSign (in
        // "sign only" mode) put A in it without counting it
        Assert.False(cnt.AddEntity(A, M));
        Assert.True(cnt.AddEntity(WS.StartingEntity(2), M));
        Assert.Equal(7, cnt.NbTimes("OK"));
        // the session's counters: from a list, or (without one) from the counter's selection
        Assert.True(WS.ComputeCounterFromList(cnt, WS.GiveList("(7,8)")));
        Assert.Equal(2, cnt.NbTimes("Data-Warning"));
        Assert.Equal(0, cnt.NbTimes("OK"));
        Assert.False(WS.ComputeCounterFromList(cnt, null)); // no selection
        var byroots = new SignCounter(new SignValidity());
        byroots.SetSelection(new SelectModelRoots());
        Assert.True(WS.ComputeCounter(byroots));
        Assert.Equal(2, byroots.NbTimes("OK")); // the roots: A F G H, of which G and H warn
        Assert.Equal(2, byroots.NbTimes("Data-Warning"));
    }

    [Fact]
    public void GiveList()
    {
        WorkSession WS = Session();
        WS.AddNamedItem("roots", new SelectModelRoots());
        WS.AddNamedItem("shared", new SelectShared());
        WS.AddNamedItem("sharing", new SelectSharing());
        WS.AddNamedItem("type", new SelectionSignType());
        Assert.Equal("A C", Names(WS.GiveList("(1,3)")));
        Assert.Equal("A C", Names(WS.GiveList("(1, 3,1)"))); // each once
        Assert.Equal("D", Names(WS.GiveList("4")));
        Assert.Equal("A F G H", Names(WS.GiveList("roots")));
        Assert.Equal("B C", Names(WS.GiveList("shared roots"))); // the shareds of the roots
        Assert.Equal("C", Names(WS.GiveList("shared", "2")));    // the shareds of entity 2
        Assert.Equal("A B", Names(WS.GiveList("sharing", "#C")));
        Assert.Equal("A B C D E F G H", Names(WS.GiveList("type(Toy)"))); // a signature with its text
        Assert.Null(WS.GiveList(""));
        Assert.Equal("A F G H", Names(WS.GiveList(" roots")));
        Assert.Null(WS.GiveList("nothing")); // neither an entity nor a selection
        Assert.Equal("A F G H", Names(WS.GiveList(WS.NamedItem("roots"))));
        Assert.Equal("A B C D E F G H", Names(WS.GiveList(WS.Model())));
        NCollection_HSequence<Transient>? aRoots = WS.GiveList("roots");
        Assert.Same(aRoots, WS.GiveList(aRoots)); // a list (a transient): itself
        Assert.Equal("B C", Names(WS.GiveListFromList("shared", aRoots)));
        Assert.Equal("C", Names(WS.GiveList(WS.StartingEntity(3))));
        Assert.Equal("D", Names(WS.GiveList(new TCollection_HAsciiString("4"))));
        Assert.Equal("B C", Names(WS.GiveListFromList("shared", WS.StartingEntity(1))));
        Assert.Equal("D", Names(Functions.GiveList(WS, "4")));
        Assert.Same(WS.StartingEntity(6), Functions.GiveEntity(WS, "#F"));
        Assert.Equal(0, Functions.GiveEntityNumber(WS, ""));
        // GiveListCombined builds no list: OCCT dereferences a null handle (occt-issues.md, 8A3)
        Assert.Throws<NullReferenceException>(() => WS.GiveListCombined(WS.GiveList("roots"), WS.GiveList("shared roots"), 1));
    }

    [Fact]
    public void Checks()
    {
        // OCCT's Interface_CheckTool::VerifyCheckList (the session's semantic checks, ComputeCheck) records one check
        // object for every entity with messages, cleared and refilled for each entity: they all end with the last
        // entity's messages (occt-issues.md, 8A3). Here G's fail is lost: G and H both have H's warning.
        WorkSession WS = Session();
        CheckIterator chl = WS.ModelCheckList();
        Assert.Equal("Model Complete Check List", chl.Name());
        Assert.True(chl.IsEmpty(true));
        Assert.False(chl.IsEmpty(false));
        var counter = new CheckCounter(true);
        counter.Analyse(chl, WS.Model(), true, false);
        Assert.Equal("Check Model Complete Check List", counter.Name());
        string[] aMess = StrSeq(counter.List());
        Assert.Single(aMess);
        Assert.StartsWith("W:", aMess[0], StringComparison.Ordinal);
        Assert.EndsWith("warns: H", aMess[0], StringComparison.Ordinal);
        Assert.Equal("G H", Names(counter.Entities(aMess[0])));
        CheckIterator one = WS.CheckOne(WS.StartingEntity(7));
        Assert.Equal("Data Check (One Entity)", one.Name());
        Assert.Equal("warns: H", one.CCheck(7)!.CWarning(1));
        Assert.True(WS.CheckOne(WS.StartingEntity(1)).IsEmpty(false));
        var aText = new StringWriter();
        WS.PrintCheckList(aText, chl, false, PrintCount.CountByItem);
        Assert.Contains("warns: H", aText.ToString());
        // the last entity's messages only: B's fail is cleared by C's (empty) check, kept when B is last
        WorkSession WS1 = Session("A B\n!B\nC\n");
        Assert.Equal("OK", SignValidity.CVal(WS1.StartingEntity(2), WS1.Model()));
        Assert.True(WS1.ModelCheckList().IsEmpty(false));
        WorkSession WS3 = Session("A B\n!B\n");
        Assert.Equal("Data-Error", SignValidity.CVal(WS3.StartingEntity(2), WS3.Model()));
        Assert.False(WS3.ModelCheckList().IsEmpty(true));
        Assert.Equal(1, WS1.QueryParent(WS1.StartingEntity(1), WS1.StartingEntity(2)));
        Assert.Equal(-1, WS1.QueryParent(WS1.StartingEntity(2), WS1.StartingEntity(1)));
        Assert.Equal(0, WS1.QueryParent(WS1.StartingEntity(2), WS1.StartingEntity(2)));
        // without the graph's statuses (not in full mode), the shared and sharing selections give nothing (OCCT)
        WorkSession WSl = Session(ToyFile, false);
        var sh = new SelectShared();
        sh.SetInput(new SelectModelRoots());
        Assert.Equal(0, sh.UniqueResult(WSl.Graph()).NbEntities());
        // a session without data
        var WS0 = new WorkSession();
        Assert.Equal("DATA NOT AVAILABLE FOR CHECK", WS0.ModelCheckList().CCheck(0)!.CFail(1));
        // the propagation of the checks to the sharing entities
        WorkSession WS2 = Session("A B\n!B\nC\n");
        Assert.Equal(1, WS2.QueryParent(WS2.StartingEntity(1), WS2.StartingEntity(2)));
        Assert.Equal(-1, WS2.QueryParent(WS2.StartingEntity(2), WS2.StartingEntity(1)));
        Assert.Equal(0, WS2.QueryParent(WS2.StartingEntity(2), WS2.StartingEntity(2)));
    }

    [Fact]
    public void Dispatches()
    {
        WorkSession WS = Session();
        EntityGraph G = WS.Graph();
        var roots = new SelectModelRoots();
        var one = new DispPerOne();
        one.SetFinalSelection(roots);
        Assert.Equal("One File per Input Entity", one.Label());
        var packs = new SubPartsIterator(G, false);
        one.Packets(G, packs);
        Assert.Equal(4, packs.PartCount);
        var count = new DispPerCount();
        var two = new IntParam();
        two.SetValue(2);
        count.SetCount(two);
        count.SetFinalSelection(roots);
        Assert.Equal("One File per 2 Entities", count.Label());
        packs = new SubPartsIterator(G, false);
        count.Packets(G, packs);
        Assert.Equal(2, packs.PartCount);
        var files = new DispPerFiles();
        files.SetCount(two);
        files.SetFinalSelection(roots);
        Assert.Equal("Maximum 2 Files", files.Label());
        packs = new SubPartsIterator(G, false);
        files.Packets(G, packs);
        Assert.Equal(2, packs.PartCount);
        var sign = new DispPerSignature();
        sign.SetSignCounter(new SignCounter(new SignValidity()));
        sign.SetFinalSelection(new SelectModelEntities());
        Assert.Equal("One File per Signature Validity", sign.Label());
        packs = new SubPartsIterator(G, false);
        sign.Packets(G, packs);
        Assert.Equal(2, packs.PartCount); // OK and Data-Warning (G's check is H's, see Checks)
        // the dispatches of a session: in its share-out when added as items
        WS.AddNamedItem("one", one);
        Assert.Equal(1, WS.ShareOut()!.DispatchCount);
        Assert.Equal(1, WS.DispatchRank(one));
        Assert.True(WS.SetFileRoot(one, "part"));
        Assert.Equal("part", WS.FileRoot(one)!.ToCString());
        Assert.Equal("part_3", WS.ShareOut()!.FileName(1, 3));
        Assert.Equal("part_03", WS.ShareOut()!.FileName(1, 3, 12));
        WS.SetFilePrefix("pre-");
        WS.SetFileExtension(".toy");
        Assert.Equal("pre-part.toy", WS.ShareOut()!.FileName(1, 1)); // a single packet: no suffix
        Assert.Equal("pre-Default_1.toy", WS.ShareOut()!.FileName(2, 1)); // no such dispatch: the default root, numbered
        Assert.Equal("pre-x.toy", WS.GiveFileComplete("x"));
        Assert.Equal("x", WS.GiveFileRoot("dir/x.step"));
        Assert.True(WS.SetActive(one, false));
        Assert.Equal(0, WS.ShareOut()!.DispatchCount);
        // GiveDispatch: by name, or with a parameter
        WS.AddNamedItem("count", count);
        WS.AddNamedItem("validity", new SignValidity());
        WS.AddNamedItem("bysign", sign);
        Assert.Same(count, Functions.GiveDispatch(WS, "count"));
        Assert.Same(count, Functions.GiveDispatch(WS, "count(5)"));
        Assert.Equal(5, count.CountValue());
        Assert.Null(Functions.GiveDispatch(WS, "count(0)"));
        Assert.Same(sign, Functions.GiveDispatch(WS, "bysign(validity)"));
        Assert.Null(Functions.GiveDispatch(WS, "bysign(nothing)"));
        // OCCT: an unknown name without parentheses raises (TCollection_AsciiString::SetValue(0, ...))
        Assert.Throws<OutOfRangeException>(() => Functions.GiveDispatch(WS, "unknown"));
    }

    [Fact]
    public void SendAllWithModifiers()
    {
        WorkSession WS = Session("A B\nB\nC\n");
        var aStream = new MemoryStream();
        Assert.Equal(ReturnStatus.RetDone, WS.SendAll("out.toy", false, aStream));
        Assert.Equal("A\nB\nC\n", Encoding.UTF8.GetString(aStream.ToArray()));
        Assert.True(WS.LastRunCheckList().IsEmpty(true)); // (SendAll asks for its global check: not empty)
        // a model modifier on the roots, a file modifier on all, applied by the share-out
        var rename = new ToyRename();
        var roots = new SelectModelRoots();
        WS.AddNamedItem("roots", roots);
        WS.AddNamedItem("rename", rename);
        WS.AddNamedItem("filemod", new ToyFileMod());
        Assert.True(WS.SetItemSelection(rename, roots));
        Assert.Same(roots, WS.ItemSelection(rename));
        Assert.True(WS.SetAppliedModifier(rename, WS.ShareOut()));
        Assert.True(WS.SetAppliedModifier((GeneralModifier)WS.NamedItem("filemod")!, WS.ShareOut()));
        Assert.Same(WS.ShareOut(), WS.UsesAppliedModifier(rename));
        Assert.Equal(1, WS.NbFinalModifiers(true));
        Assert.Equal(1, WS.NbFinalModifiers(false));
        aStream = new MemoryStream();
        Assert.Equal(ReturnStatus.RetDone, WS.SendAll("out.toy", false, aStream));
        // SendAll works on the session's own model: the modifier renames its roots (A, C)
        Assert.Equal("mod: FileMod (all)\nA*\nB\nC*\n", Encoding.UTF8.GetString(aStream.ToArray()));
        Assert.Equal("A*", WS.StartingEntity(1)!.ToString());
        // SendSelected copies the selected entities (and what they share) into a new model
        var pointed = new SelectPointed();
        pointed.SetEntity(WS.StartingEntity(2));
        WS.AddNamedItem("pointed", pointed);
        Assert.True(WS.ResetAppliedModifier(rename));
        aStream = new MemoryStream();
        Assert.Equal(ReturnStatus.RetDone, WS.SendSelected("sel.toy", pointed, false, aStream));
        Assert.Equal("mod: FileMod (all)\nB\n", Encoding.UTF8.GetString(aStream.ToArray()));
        // nothing selected: void
        pointed.Clear();
        Assert.Equal(ReturnStatus.RetVoid, WS.SendSelected("sel.toy", pointed, false, new MemoryStream()));
        // a library failure: RetStop (and, as OCCT, the last run's check list is not updated)
        Assert.Equal(ReturnStatus.RetStop, WS.SendAll(""));
        // ClearFinalModifiers removes them as items
        WS.ClearFinalModifiers();
        Assert.Equal(0, WS.NbFinalModifiers(false));
        Assert.Null(WS.NamedItem("filemod"));
    }

    [Fact]
    public void EditorsAndForms()
    {
        var statics = new StaticSet();
        statics.Standards();
        NCollection_HSequence<TCollection_HAsciiString> list = new();
        list.Append(new TCollection_HAsciiString("read.precision.mode"));
        list.Append(new TCollection_HAsciiString("write.precision.val"));
        list.Append(new TCollection_HAsciiString("no.such.static"));
        ParamEditor ed = ParamEditor.StaticEditor(statics, list, "Some statics")!;
        Assert.Equal("Some statics", ed.Label());
        Assert.Equal(2, ed.ValueCount);
        Assert.Equal(1, ed.NameNumber("read.precision.mode"));
        Assert.Equal(2, ed.NameNumber("2"));
        Assert.Equal(0, ed.NameNumber("x"));
        Assert.Equal(EditValue.Editable, ed.EditMode(1));
        EditForm form = ed.Form(false)!;
        Assert.True(form.LoadData());
        Assert.Equal(statics.CVal("write.precision.val"), form.OriginalValue(2)!.ToCString());
        Assert.False(form.Modify(2, new TCollection_HAsciiString("abc"))); // not a real
        Assert.True(form.Modify(2, new TCollection_HAsciiString("0.25")));
        Assert.True(form.IsModified(2));
        Assert.Equal("0.25", form.EditedValue(2)!.ToCString());
        Assert.True(form.Apply());
        Assert.Equal(0.25, statics.RVal("write.precision.val"));
        Assert.False(form.IsModified(2)); // cleared after Apply
        var aText = new StringWriter();
        form.PrintValues(aText, 1, true);
        Assert.Contains("write.precision.val", aText.ToString());
        aText = new StringWriter();
        ed.PrintDefs(aText);
        Assert.Contains("Editable", aText.ToString());
        // a read-only form can't be modified
        EditForm ro = ed.Form(true)!;
        Assert.False(ro.Modify(1, new TCollection_HAsciiString("1")));
        // a constant text
        ed.AddConstantText("v", "const");
        Assert.Equal(3, ed.ValueCount);
        Assert.Equal(EditValue.EditRead, ed.EditMode(3));
        // the STEP header's editor recognizes no time stamp (OCCT: its separators fall through, occt-issues.md, 8A3)
        var header = new EditHeader();
        Assert.Equal(10, header.ValueCount);
        Assert.False(header.TypedValue(2)!.Satisfies(new TCollection_HAsciiString("2024-01-02T03:04:05")));
        Assert.True(header.TypedValue(1)!.Satisfies(new TCollection_HAsciiString("any name")));
    }
}
