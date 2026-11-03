// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Text;
using OcctNet.DataExchange.Tests.Interface;
using static Occt.Interface_CheckStatus;
using static Occt.Interface_ParamType;
using static Occt.Transfer_StatusExec;
using static Occt.Transfer_StatusResult;
using static OcctNet.DataExchange.Tests.Interface.ToyNorm;

namespace OcctNet.DataExchange.Tests.Transfer;

/// <summary>A printer keeping what a messenger prints (every gravity).</summary>
internal sealed class StringPrinter : Printer
{
    public readonly StringBuilder Text = new();

    public StringPrinter() => _traceLevel = Gravity.Trace;

    protected override void SendCore(string theString, Gravity theGravity) => Text.Append(theString).Append('\n');
}

/// <summary>
/// The toy norm's actor: a ToyEntity becomes its name in lower case (a transient result); with <see cref="Nested"/>, it
/// first transfers what the entity shares (binding the entity beforehand, so cycles are found as OCCT's actors do);
/// <see cref="Throw"/> names an entity whose transfer throws.
/// </summary>
internal sealed class ToyActor : Transfer_ActorOfTransientProcess
{
    public bool Nested;
    public string? Throw;

    public override bool Recognize(Transient? start) => start is ToyEntity;

    public override Transfer_Binder? Transfer(Transient? start, Transfer_TransientProcess? TP,
                                              ProgressRange theProgress = default)
    {
        var anEnt = (ToyEntity)start!;
        if (anEnt.Name == Throw)
        {
            throw new FailureException("boom " + anEnt.Name);
        }
        if (Nested)
        {
            TP!.FindElseBind(anEnt);
            foreach (ToyEntity aRef in anEnt.Refs)
            {
                TP.Transferring(aRef);
            }
        }
        if (anEnt.Warns)
        {
            TP!.AddWarning(anEnt, "warns " + anEnt.Name);
        }
        return TransientResult(new TCollection_HAsciiString(anEnt.Name.ToLowerInvariant()));
    }
}

/// <summary>
/// Transfer's binders, processes, iterators and tools on the toy norm: A shares B and C, B shares C, D and E share
/// each other (a cycle), F is alone.
/// </summary>
[Collection(ProcessWideMessages.Name)]
public class TransferTests
{
    private readonly ToyEntity A = new("A"), B = new("B"), C = new("C"), D = new("D"), E = new("E"), F = new("F");
    private readonly ToyModel myModel;

    public TransferTests()
    {
        A.Ref(B, C);
        B.Ref(C);
        D.Ref(E);
        E.Ref(D);
        myModel = Model(A, B, C, D, E, F);
    }

    private static string Res(Transfer_Binder? theBinder) =>
        (theBinder as Transfer_SimpleBinderOfTransient)?.Result() is TCollection_HAsciiString aStr ? aStr.ToCString() : "-";

    [Fact]
    public void Binders_chain_their_next_results_once_each()
    {
        var b1 = new Transfer_SimpleBinderOfTransient();
        var b2 = new Transfer_SimpleBinderOfTransient();
        var b3 = new Transfer_VoidBinder();
        Assert.False(b1.HasResult());
        Assert.Equal(Transfer_StatusVoid, b1.Status());
        Assert.Equal("(void)", b1.ResultTypeName());
        Assert.Equal(typeof(Transient), b1.ResultType());
        b1.SetResult(new TCollection_HAsciiString("x"));
        Assert.Equal(Transfer_StatusDone, b1.StatusExec());
        Assert.Equal("TCollection_HAsciiString", b1.ResultTypeName());

        b1.AddResult(b2);
        b1.AddResult(b3);
        b1.AddResult(b1); // itself: ignored
        Assert.Same(b2, b1.NextResult());
        Assert.Same(b3, b2.NextResult());
        Assert.Null(b3.NextResult());
        Assert.False(b1.IsMultiple()); // b2 and b3 have no result
        b2.SetResult(new TCollection_HAsciiString("y"));
        Assert.True(b1.IsMultiple());

        // a used result can't change
        b1.SetAlreadyUsed();
        Assert.Equal(Transfer_StatusUsed, b1.Status());
        Assert.Throws<Transfer_TransferFailure>(() => b1.SetResult(null));

        // fails and warnings, and Merge
        var b4 = new Transfer_VoidBinder();
        b4.AddWarning("w");
        Assert.Equal(Transfer_StatusInitial, b4.StatusExec());
        b4.AddFail("f", "orig");
        Assert.Equal(Transfer_StatusError, b4.StatusExec());
        var b5 = new Transfer_SimpleBinderOfTransient();
        b5.Merge(b4);
        Assert.Equal(Transfer_StatusError, b5.StatusExec());
        Assert.Equal(1, b5.Check().NbFails());
        Assert.Equal(1, b5.Check().NbWarnings());
        Assert.Equal(typeof(Transfer_VoidBinder), b4.ResultType());
        Assert.Equal("(void)", b4.ResultTypeName());

        // GetTypedResult searches the chain
        Transient? aRes = null;
        Assert.True(Transfer_SimpleBinderOfTransient.GetTypedResult(b3, typeof(Transient), ref aRes) == false);
        Assert.True(Transfer_SimpleBinderOfTransient.GetTypedResult(b1, typeof(TCollection_HAsciiString), ref aRes));
        Assert.Equal("x", ((TCollection_HAsciiString)aRes!).ToCString());
        Assert.False(Transfer_SimpleBinderOfTransient.GetTypedResult(b1, typeof(Interface_IntVal), ref aRes));
        Assert.Equal("x", ((TCollection_HAsciiString)aRes!).ToCString()); // not touched
    }

    [Fact]
    public void Multiple_list_and_integer_binders()
    {
        var aMulti = new Transfer_MultipleBinder();
        Assert.False(aMulti.IsMultiple());
        Assert.Equal(0, aMulti.NbResults());
        Assert.Equal(0, aMulti.MultipleResult().Length());
        aMulti.AddResult(A);
        Assert.False(aMulti.IsMultiple());
        aMulti.AddResult(B);
        Assert.True(aMulti.IsMultiple());
        Assert.Same(B, aMulti.ResultValue(2));
        Assert.Equal("(list)", aMulti.ResultTypeName());

        var aList = new Transfer_TransientListBinder();
        aList.AddResult(A);
        Assert.False(aList.IsMultiple());
        aList.AddResult(B);
        aList.SetResult(2, C);
        Assert.True(aList.IsMultiple());
        Assert.Same(C, aList.Transient(2));
        Assert.Equal("list(Standard_Transient)", aList.ResultTypeName());
        Assert.Equal(typeof(Transfer_TransientListBinder), aList.ResultType());

        var anInt = new Transfer_BinderOfTransientInteger();
        Assert.Equal(0, anInt.Integer());
        anInt.SetInteger(7);
        anInt.SetResult(A);
        Assert.Equal(7, anInt.Integer());
        Assert.Equal("ToyEntity", anInt.ResultTypeName());
    }

    [Fact]
    public void A_process_binds_finds_and_rebinds()
    {
        var TP = new Transfer_TransientProcess();
        Assert.Null(TP.Find(A));
        Assert.False(TP.IsBound(A));
        Assert.Throws<Transfer_TransferFailure>(() => TP.IsAlreadyUsed(A));

        // a void binder is replaced, its check kept
        TP.AddWarning(A, "early");
        Assert.IsType<Transfer_VoidBinder>(TP.Find(A));
        TP.BindTransient(A, new TCollection_HAsciiString("a"));
        Assert.True(TP.IsBound(A));
        Assert.Equal("a", Res(TP.Find(A)));
        Assert.Equal(1, TP.Find(A)!.Check().NbWarnings());
        Assert.Equal(1, TP.NbMapped());
        Assert.Same(A, TP.Mapped(1));
        Assert.Equal(1, TP.MapIndex(A));

        // rebinding keeps the former check; a used result refuses
        TP.Rebind(A, new Transfer_ActorOfTransientProcess().TransientResult(new TCollection_HAsciiString("a2")));
        Assert.Equal("a2", Res(TP.Find(A)));
        Assert.Equal(1, TP.Check(A)!.NbWarnings());
        TP.Find(A)!.SetAlreadyUsed();
        Assert.True(TP.IsAlreadyUsed(A));
        Assert.Throws<Transfer_TransferFailure>(() => TP.Bind(A, new Transfer_VoidBinder()));

        // FindElseBind, FindTransient, FindTypedTransient
        Transfer_Binder aVoid = TP.FindElseBind(B);
        Assert.IsType<Transfer_VoidBinder>(aVoid);
        Assert.Same(aVoid, TP.FindElseBind(B));
        Assert.Null(TP.FindTransient(B));
        Transient? aVal = null;
        Assert.True(TP.FindTypedTransient(A, typeof(TCollection_HAsciiString), ref aVal));
        Assert.Null(TP.Check(F));

        // multiple results
        TP.BindMultiple(C);
        TP.AddMultiple(C, D);
        TP.AddMultiple(C, E);
        Assert.Equal(2, ((Transfer_MultipleBinder)TP.Find(C)!).NbResults());
        Assert.Throws<Transfer_TransferFailure>(() => TP.BindMultiple(A));
        Assert.Throws<Transfer_TransferFailure>(() => TP.AddMultiple(F, D));
        Assert.Throws<Transfer_TransferFailure>(() => TP.AddMultiple(A, D));

        // roots
        TP.SetRoot(A);
        TP.SetRoot(F); // not mapped: ignored
        Assert.Equal(1, TP.NbRoots());
        Assert.Same(A, TP.Root(1));
        Assert.Equal(1, TP.RootIndex(A));
        Assert.Equal(0, TP.RootIndex(B));
        Assert.Throws<OutOfRangeException>(() => TP.Root(2));

        // OCCT's Unbind keeps the binder in the map (thelastbnd is the former binder), but drops the root
        Assert.True(TP.Unbind(A));
        Assert.NotNull(TP.Find(A));
        Assert.Equal(0, TP.NbRoots());
        Assert.False(TP.Unbind(F));
        Assert.True(TP.Unbind(B)); // a void binder: true, nothing done
        TP.Clean(); // no null binder: nothing to do
        Assert.Equal(3, TP.NbMapped());
        TP.Clear();
        Assert.Equal(0, TP.NbMapped());
    }

    [Fact]
    public void Fails_and_warnings_are_traced_by_level()
    {
        var aPrinter = new StringPrinter();
        var TP = new Transfer_TransientProcess();
        TP.SetMessenger(new Messenger(aPrinter));
        Assert.Equal(0, TP.TraceLevel());
        TP.AddFail(A, "silent");
        Assert.Equal("", aPrinter.Text.ToString());

        TP.SetTraceLevel(3);
        TP.SetModel(myModel);
        TP.AddFail(B, "bad", "orig");
        TP.AddWarning(C, "soso");
        string aText = aPrinter.Text.ToString();
        Assert.Contains("Entity #B", aText); // PrintTrace with the model: the label
        Assert.Contains("    --> Fail : bad [from: orig]", aText);
        Assert.Contains("    --> Warning : soso", aText);
        Assert.Equal(Transfer_StatusError, TP.Find(B)!.StatusExec());

        // Mend prefixes the fails (OCCT's Interface_Check::Mend adds them back as fails)
        TP.Mend(B, "M");
        Assert.Equal(1, TP.Check(B)!.NbFails());
        Assert.Equal("M : bad", TP.Check(B)!.CFail(1));

        // the messages as Message_Msg: the value, and the original when edited
        var aMsg = new Msg();
        aMsg.Set("value %d");
        aMsg.Arg(5);
        TP.SendWarning(D, aMsg);
        Assert.Equal("value 5", TP.Check(D)!.CWarning(1));
        Assert.Equal("value %d", TP.Check(D)!.CWarning(1, false));
    }

    [Fact]
    public void Transferring_runs_the_actors_and_records_roots()
    {
        var TP = new Transfer_TransientProcess(myModel.NbEntities());
        TP.SetModel(myModel);
        Assert.False(TP.Recognize(A));
        Assert.Null(TP.Transferring(A)); // no actor: no result
        var anActor = new ToyActor { Nested = true };
        TP.SetActor(anActor);
        Assert.Same(anActor, TP.Actor());
        Assert.True(TP.Recognize(A));
        Assert.False(TP.Recognize(new ToyUnknown()));
        TP.SetRootManagement(true);

        Transfer_Binder? aBinder = TP.Transferring(A);
        Assert.Equal("a", Res(aBinder));
        Assert.Equal("b", Res(TP.Find(B)));
        Assert.Equal("c", Res(TP.Find(C)));
        // OCCT's root management: each transfer that ends at the level of the first one done becomes a root, the
        // nested ones included (C, then B, then A)
        Assert.Equal(3, TP.NbRoots());
        Assert.Same(C, TP.Root(1));
        Assert.Same(A, TP.Root(3));
        Assert.Equal(0, TP.NestingLevel());

        // a second request uses the result
        Assert.Same(aBinder, TP.Transferring(A));
        Assert.Equal(Transfer_StatusUsed, aBinder!.Status());

        // RootResult and CompleteResult
        Transfer_IteratorOfProcessForTransient aRoots = TP.RootResult(true);
        Assert.Equal(3, aRoots.Number());
        aRoots.Start();
        Assert.True(aRoots.More());
        Assert.Same(C, aRoots.Starting());
        Transfer_IteratorOfProcessForTransient anAll = TP.CompleteResult();
        Assert.Equal(3, anAll.Number());
        Assert.False(anAll.HasStarting());
        Assert.Throws<NoSuchObjectException>(() => anAll.Starting());
        Assert.Equal(0, TP.AbnormalResult().Number());
        Assert.True(TP.CheckList(false).IsEmpty(false));

        // OCCT's per-entity results lost their scopes: always empty
        Assert.Equal(0, TP.ResultOne(A, 2, true).Number());
        Assert.True(TP.CheckListOne(A, 0, false).IsEmpty(false));
        Assert.True(TP.IsCheckListEmpty(A, 0, false));
        Assert.False(TP.IsCheckListEmpty(F, 0, false)); // not mapped
        TP.RemoveResult(A, 0); // nothing done
        Assert.True(TP.IsBound(A));
    }

    [Fact]
    public void A_transfer_met_running_is_a_loop()
    {
        var aPrinter = new StringPrinter();
        var TP = new Transfer_TransientProcess();
        TP.SetMessenger(new Messenger(aPrinter));
        TP.SetModel(myModel);
        TP.SetActor(new ToyActor { Nested = true });

        // a transfer met while it runs is marked looping, and gives no result
        var aRunning = new Transfer_VoidBinder();
        TP.Bind(D, aRunning);
        aRunning.SetStatusExec(Transfer_StatusRun);
        Assert.Same(aRunning, TP.Transferring(D));
        Assert.Equal(Transfer_StatusLoop, aRunning.StatusExec());
        Assert.Equal(1, TP.AbnormalResult().Number());
        Interface_CheckIterator aList = TP.CheckList(true);
        aList.Start();
        Assert.Equal("Transfer in Abnormal Status (!= Initial or Done)", aList.Value()!.CFail(1));
        Assert.Equal(4, aList.Number()); // CheckNum: the number in the model

        // a looping one is the head of a dead loop
        TP.SetTraceLevel(1);
        Transfer_Binder? aHead = TP.Transferring(D);
        Assert.Same(aRunning, aHead);
        Assert.Equal("Transfer: Head of Dead Loop", aHead!.Check().CFail(2));
        Assert.Contains("*** Dead Loop : Finding head of Loop :", aPrinter.Text.ToString());

        // OCCT's actors meet a cycle while their entity runs: the inner transfer is marked looping, then the outer one
        // binds its result over it (the worst status is kept only over a void binder)
        var TP2 = new Transfer_TransientProcess();
        TP2.SetActor(new ToyActor { Nested = true });
        Assert.Equal("d", Res(TP2.Transferring(D)));
        Assert.Equal(Transfer_StatusDone, TP2.Find(D)!.StatusExec());
        Assert.Equal(Transfer_StatusDone, TP2.Find(E)!.StatusExec());
    }

    [Fact]
    public void Exceptions_are_caught_under_error_handling()
    {
        var aPrinter = new StringPrinter();
        var TP = new Transfer_TransientProcess();
        TP.SetMessenger(new Messenger(aPrinter));
        TP.SetTraceLevel(1);
        TP.SetActor(new ToyActor { Throw = "B" });
        Assert.True(TP.ErrorHandle());
        Assert.Null(TP.Transferring(B));
        Transfer_Binder aBinder = TP.Find(B)!;
        Assert.IsType<Transfer_VoidBinder>(aBinder);
        Assert.Equal("Transfer stopped by exception raising", aBinder.Check().CFail(1));
        Assert.Contains("*** Raised : boom B", aPrinter.Text.ToString());
        // a binder in error status: the next transfer throws
        Assert.Throws<Transfer_TransferFailure>(() => TP.Transferring(B));

        TP.SetErrorHandle(false);
        Assert.Throws<FailureException>(() => TP.Transferring(new ToyEntity("B")));
    }

    [Fact]
    public void A_cancelled_transfer_gives_no_binder()
    {
        using var aSource = new CancellationTokenSource();
        aSource.Cancel();
        var anIndicator = new ProgressIndicator(aSource.Token, null);
        var TP = new Transfer_TransientProcess();
        TP.SetActor(new ToyActor());
        Assert.Null(TP.Transferring(A, anIndicator.Start()));
        Assert.False(TP.Transfer(A, anIndicator.Start()));
        Assert.True(TP.Transfer(A));
    }

    [Fact]
    public void Actors_form_a_list_with_the_last_ones_at_the_end()
    {
        var aDefault = new Transfer_ActorOfTransientProcess();
        aDefault.SetLast();
        var a1 = new ToyActor();
        var a2 = new ToyActor();
        var TP = new Transfer_TransientProcess();
        TP.SetActor(aDefault);
        TP.SetActor(a1); // before the last one
        Assert.Same(a1, TP.Actor());
        Assert.Same(aDefault, a1.Next());
        TP.SetActor(a2); // a1 is not last: a2 goes after it, before the default
        Assert.Same(a2, a1.Next());
        Assert.Same(aDefault, a2.Next());
        TP.SetActor(null);
        Assert.Null(TP.Actor());
        Assert.Null(aDefault.Transferring(A, TP));
        Assert.Null(aDefault.TransientResult(null));
    }

    [Fact]
    public void The_iterator_selects_binders()
    {
        var anIter = new Transfer_TransferIterator();
        var b1 = new Transfer_SimpleBinderOfTransient();
        b1.SetResult(new TCollection_HAsciiString("x"));
        var b2 = new Transfer_VoidBinder();
        var b3 = new Transfer_MultipleBinder();
        b3.AddResult(A);
        b3.AddResult(B);
        anIter.AddItem(b1);
        anIter.AddItem(b2);
        anIter.AddItem(b3);
        Assert.Equal(3, anIter.Number());
        Assert.Throws<NoSuchObjectException>(() => new Transfer_TransferIterator().Value());

        var aTypes = new List<string>();
        for (anIter.Start(); anIter.More(); anIter.Next())
        {
            aTypes.Add(global::Kerf.Engine.Foundation.OcctTypeNames.Of(anIter.Value().GetType()) + ":" + anIter.HasResult() + anIter.HasUniqueResult() + anIter.HasTransientResult());
        }
        Assert.Equal(["Transfer_SimpleBinderOfTransient:TrueTrueTrue", "Transfer_VoidBinder:FalseFalseTrue",
                      "Transfer_MultipleBinder:FalseFalseFalse"], aTypes);
        anIter.Start();
        Assert.Equal("x", ((TCollection_HAsciiString)anIter.TransientResult()!).ToCString());
        Assert.Equal(Transfer_StatusDone, anIter.Status());
        Assert.False(anIter.HasFails());
        Assert.False(anIter.HasWarnings());

        var aCopy = new Transfer_TransferIterator(anIter);
        aCopy.SelectUnique(false); // keep only the multiple ones
        Assert.Equal(1, aCopy.Number());
        aCopy.Start();
        Assert.Same(b3, aCopy.Value());

        var anIter2 = new Transfer_TransferIterator();
        anIter2.AddItem(b1);
        anIter2.AddItem(b2);
        anIter2.SelectBinder(typeof(Transfer_VoidBinder), false);
        Assert.Equal(1, anIter2.Number());

        // OCCT's SelectResult compares each item with the current item's result type
        var anIter3 = new Transfer_TransferIterator();
        anIter3.AddItem(b1);
        anIter3.AddItem(b1);
        anIter3.SelectResult(typeof(TCollection_HAsciiString), true);
        Assert.Equal(2, anIter3.Number());
    }

    [Fact]
    public void The_process_iterator_filters_on_starting_objects()
    {
        var anIter = new Transfer_IteratorOfProcessForTransient(true);
        anIter.Add(new Transfer_VoidBinder(), A);
        anIter.Add(new Transfer_VoidBinder(), B);
        anIter.Add(new Transfer_VoidBinder(), C);
        Assert.Throws<NoSuchObjectException>(() => anIter.Add(new Transfer_VoidBinder()));
        var aList = new NCollection_HSequence<Transient>();
        aList.Append(B);
        anIter.Filter(aList);
        Assert.Equal(1, anIter.Number());
        anIter.Start();
        Assert.Same(B, anIter.Starting());
        anIter.Filter(aList, false);
        Assert.Equal(2, anIter.Number());
    }

    [Fact]
    public void The_transient_process_reads_its_model()
    {
        C.Fails = true;
        var TP = new Transfer_TransientProcess();
        Assert.True(TP.IsDataLoaded(A));
        Assert.False(TP.IsDataFail(A));
        Assert.Equal(0, TP.CheckNum(A));
        var aText = new StringWriter();
        TP.PrintTrace(A, aText);
        Assert.Equal("Entity Type:ToyEntity", aText.ToString());

        TP.SetGraph(new Interface_HGraph(myModel, Protocol));
        Assert.True(TP.HasGraph());
        Assert.Same(myModel, TP.Model());
        Assert.Equal(3, TP.CheckNum(C));
        Assert.True(TP.IsDataLoaded(A));
        Assert.False(TP.IsDataFail(A));
        Assert.Equal("C", Sorted(TP.TypedSharings(C, typeof(ToyEntity)))); // OCCT: the entity itself, of the type
        TP.SetGraph(null);
        Assert.Null(TP.Model());
        Assert.Equal(0, TP.TypedSharings(C, typeof(ToyEntity)).NbEntities());

        // contexts
        Transient? aCtx = null;
        Assert.False(TP.GetContext("x", null, ref aCtx));
        TP.SetContext("x", A);
        Assert.True(TP.GetContext("x", typeof(ToyEntity), ref aCtx));
        Assert.Same(A, aCtx);
        Assert.False(TP.GetContext("x", typeof(TCollection_HAsciiString), ref aCtx));
        Assert.Null(aCtx);
        Assert.True(TP.GetContext("x", null, ref aCtx));
        Assert.Equal(1, TP.Context().Extent());
        Assert.Equal(0, TP.RootsForTransfer().Length());

        // the statistics
        TP.SetModel(myModel);
        TP.SetActor(new ToyActor { Nested = true });
        TP.SetRootManagement(true);
        TP.Transferring(A);
        TP.AddFail(F, "f");
        var aStats = new StringWriter();
        TP.PrintStats(1, aStats);
        Assert.Equal("\n*******************************************************************\n"
                     + "********                 Basic Statistics                  ********\n"
                     + "****        Nb Entities         : 6\n"
                     + "****        Nb Final Results    : 3\n"
                     + "****                  Errors on :    1 Entities\n"
                     + "*******************************************************************\n", aStats.ToString());
    }

    [Fact]
    public void Finders_and_the_finder_process()
    {
        var m1 = new Transfer_TransientMapper(A);
        var m2 = new Transfer_TransientMapper(A);
        var m3 = new Transfer_TransientMapper(B);
        Assert.True(m1.Equates(m2));
        Assert.False(m1.Equates(m3));
        Assert.False(m1.Equates(null));
        Assert.Equal(m1.GetHashCode(), m2.GetHashCode());
        Assert.Equal(typeof(ToyEntity), m1.ValueType());
        Assert.Equal("ToyEntity", m1.ValueTypeName());
        Assert.True(Transfer_FindHasher.Instance.Equals(m1, m2));
        Assert.False(Transfer_FindHasher.Instance.Equals(null, m2));

        var FP = new Transfer_FinderProcess();
        FP.SetModel(myModel);
        Assert.Same(myModel, FP.Model());
        Assert.NotSame(m1, FP.TransientMapper(A)); // not mapped: a new one
        FP.BindTransient(m1, new TCollection_HAsciiString("a"));
        Assert.Same(m1, FP.TransientMapper(A)); // mapped: the mapped one
        Assert.Equal("a", Res(FP.Find(m2))); // found through another mapper of the same object
        Assert.Equal(1, FP.NbMapped());
        FP.BindTransient(m3, new TCollection_HAsciiString("b"));
        m3.SetIntegerAttribute("n", 3);
        Assert.Equal(2, FP.NextMappedWithAttribute("n", 0));
        Assert.Equal(0, FP.NextMappedWithAttribute("n", 2));
        var aText = new StringWriter();
        FP.PrintTrace(m3, aText);
        Assert.Equal(" Type:ToyEntity", aText.ToString());
        TCollection_HAsciiString? aStr = null;
        Assert.True(FP.FindTypedTransient(m2, typeof(TCollection_HAsciiString), ref aStr));
        Assert.Equal("a", aStr!.ToCString());

        // the finder actor transfers the mapped transient
        var anActor = new ToyFinderActor();
        FP.SetActor(anActor);
        var m4 = new Transfer_TransientMapper(C);
        Assert.Equal("C!", Res(FP.Transferring(m4)));
        Assert.Equal(0, anActor.ModeTrans());
        anActor.ModeTrans() = 2;
        Assert.Equal(2, anActor.ModeTrans());
        var aStats = new StringWriter();
        FP.PrintStats(1, aStats);
        Assert.Contains("****        Nb Final Results    : 0\n", aStats.ToString());
    }

    private sealed class ToyFinderActor : Transfer_ActorOfFinderProcess
    {
        public override Transient? TransferTransient(Transient? ent, Transfer_FinderProcess? FP,
                                                              ProgressRange theProgress = default) =>
            ent is ToyEntity anEnt ? new TCollection_HAsciiString(anEnt.Name + "!") : null;
    }

    [Fact]
    public void Finder_attributes()
    {
        var m = new Transfer_TransientMapper(A);
        Assert.Equal(Interface_ParamVoid, m.AttributeType("i"));
        Assert.False(m.RemoveAttribute("i"));
        m.SetIntegerAttribute("i", 4);
        m.SetRealAttribute("r", 2.5);
        m.SetStringAttribute("s", "text");
        m.SetAttribute("t", B);
        Assert.Equal(Interface_ParamInteger, m.AttributeType("i"));
        Assert.Equal(Interface_ParamReal, m.AttributeType("r"));
        Assert.Equal(Interface_ParamText, m.AttributeType("s"));
        Assert.Equal(Interface_ParamIdent, m.AttributeType("t"));
        Assert.True(m.GetIntegerAttribute("i", out int i) && i == 4);
        Assert.False(m.GetIntegerAttribute("r", out i));
        Assert.Equal(0, i);
        Assert.True(m.GetRealAttribute("r", out double r) && r == 2.5);
        Assert.Equal(0.0, m.RealAttribute("i"));
        Assert.True(m.GetStringAttribute("s", out string s) && s == "text");
        Assert.Equal("", m.StringAttribute("i"));
        Assert.True(m.GetAttribute("t", typeof(ToyEntity), out Transient? t) && t == B);
        Assert.False(m.GetAttribute("t", typeof(Interface_IntVal), out t));
        Assert.Null(t);

        // copied attributes are new objects for the basic types, shared for the others
        var m2 = new Transfer_TransientMapper(B);
        m2.GetAttributes(m, "", true);
        Assert.Equal(4, m2.IntegerAttribute("i"));
        Assert.NotSame(m.Attribute("i"), m2.Attribute("i"));
        Assert.Same(m.Attribute("t"), m2.Attribute("t"));
        var m3 = new Transfer_TransientMapper(C);
        m3.GetAttributes(m, "r", false);
        Assert.Equal(1, m3.AttrList().Extent());
        Assert.Same(m.Attribute("r"), m3.Attribute("r"));
        // OCCT's SameAttributes copies the list (C++'s map assignment)
        var m4 = new Transfer_TransientMapper(D);
        m4.SameAttributes(m);
        m.RemoveAttribute("i");
        Assert.Equal(4, m4.IntegerAttribute("i"));
    }

    [Fact]
    public void TransferOutput_and_TransferInput()
    {
        // roots of the model: A and F (D and E share each other)
        var anOut = new Transfer_TransferOutput(new ToyActor { Nested = true }, myModel);
        Assert.Same(myModel, anOut.Model());
        anOut.TransferRoots();
        Transfer_TransientProcess TP = anOut.TransientProcess();
        Assert.Equal(2, TP.NbRoots());
        Assert.Same(A, TP.Root(1));
        Assert.Same(F, TP.Root(2));
        Assert.Equal("a", Res(TP.Find(A)));
        Assert.Throws<Transfer_TransferFailure>(() => anOut.Transfer(new ToyEntity("Z")));
        anOut.Transfer(D);
        TP.AddFail(F, "f");
        Assert.Equal("F", Names(anOut.ListForStatus(false, false)));
        Assert.Equal("A", Names(anOut.ListForStatus(true)));
        Interface_InterfaceModel aStatusModel = anOut.ModelForStatus(Protocol, true)!;
        Assert.Equal("A B C", Names(aStatusModel.Entities()));

        // the graph form sets the model
        var anOut2 = new Transfer_TransferOutput(new Transfer_TransientProcess(), myModel);
        anOut2.TransientProcess().SetActor(new ToyActor());
        anOut2.TransferRoots(new Interface_Graph(myModel, Protocol));
        Assert.Same(myModel, anOut2.TransientProcess().Model());
        Assert.Equal(2, anOut2.TransientProcess().NbRoots());

        // results back into a model: the entities of the transient results
        var aTP = new Transfer_TransientProcess();
        aTP.SetRootManagement(true);
        aTP.BindTransient(A, B);
        aTP.SetRoot(A);
        var aMulti = new Transfer_MultipleBinder();
        aMulti.AddResult(C);
        aMulti.AddResult(D);
        aTP.Bind(F, aMulti);
        aTP.Bind(E, new Transfer_VoidBinder());
        var anIn = new Transfer_TransferInput();
        Assert.Equal("B C D", Names(anIn.Entities(aTP.CompleteResult())));
        var aNew = new ToyModel();
        anIn.FillModel(aTP, aNew);
        Assert.Equal("B C D", Names(aNew.Entities()));
        var aNew2 = new ToyModel();
        anIn.FillModel(aTP, aNew2, Protocol, true);
        Assert.Equal("B C", Names(aNew2.Entities())); // the root's result and what it shares
        var aList = new Transfer_IteratorOfProcessForTransient(false);
        aList.Add(new TransferBRep_ShapeBinder());
        Assert.Throws<Transfer_TransferFailure>(() => anIn.Entities(aList));

        var aFP = new Transfer_FinderProcess();
        aFP.BindTransient(new Transfer_TransientMapper(A), C);
        var aNew3 = new ToyModel();
        anIn.FillModel(aFP, aNew3);
        Assert.Equal("C", Names(aNew3.Entities()));
    }

    [Fact]
    public void Dispatch_copies_a_model_through_its_actors()
    {
        var aDispatch = new Transfer_ActorDispatch(myModel, Protocol);
        // a specific actor: F becomes a new entity "F*"
        aDispatch.AddActor(new RenameActor());
        Transfer_TransferDispatch aTool = aDispatch.TransferDispatch();
        aTool.TransferEntity(A);
        aTool.TransferEntity(F);
        Transfer_TransientProcess TP = aTool.TransientProcess();
        var aCopyOfA = (ToyEntity)TP.FindTransient(A)!;
        Assert.Equal("A'", aCopyOfA.Name);
        Assert.Equal("B'", aCopyOfA.Refs[0].Name);
        Assert.Equal("F*", ((ToyEntity)TP.FindTransient(F)!).Name);

        var aControl = (Transfer_DispatchControl)aTool.Control()!;
        Assert.Same(myModel, aControl.StartingModel());
        Assert.True(aControl.Search(A, out Transient? aRes));
        Assert.Same(aCopyOfA, aRes);
        aControl.Bind(D, E);
        Assert.Same(E, TP.FindTransient(D));
        aControl.Clear();
        Assert.Equal(0, TP.NbMapped());
        Assert.False(aControl.Search(A, out aRes));

        var aDispatch2 = new Transfer_ActorDispatch(myModel);
        aDispatch2.TransferDispatch().TransferEntity(C);
        Assert.Equal("C'", ((ToyEntity)aDispatch2.TransferDispatch().TransientProcess().FindTransient(C)!).Name);
        var aDispatch3 = new Transfer_ActorDispatch(myModel, new Interface_GeneralLib(Protocol));
        Assert.Null(aDispatch3.TransferDispatch().TransientProcess().Find(C));
    }

    private sealed class RenameActor : Transfer_ActorOfTransientProcess
    {
        public override bool Recognize(Transient? start) => start is ToyEntity { Name: "F" };

        public override Transient? TransferTransient(Transient? ent, Transfer_TransientProcess? TP,
                                                              ProgressRange theProgress = default) => new ToyEntity("F*");
    }

    [Fact]
    public void Results_from_a_model()
    {
        B.Warns = true;
        var TP = new Transfer_TransientProcess();
        TP.SetModel(myModel);
        TP.SetActor(new ToyActor());
        TP.Transferring(B);
        TP.AddFail(C, "f");
        var aRes = new Transfer_ResultFromModel();
        Assert.False(aRes.Fill(TP, A)); // not transferred
        Assert.False(aRes.HasResult());
        Assert.True(aRes.Fill(TP, B));
        Assert.True(aRes.HasResult());
        Assert.Same(myModel, aRes.Model());
        Assert.Equal(2, aRes.MainNumber());
        Assert.Equal("#B", aRes.MainLabel());
        aRes.SetFileName("file.toy");
        Assert.Equal("file.toy", aRes.FileName());
        Assert.Same(aRes.MainResult(), aRes.ResultFromKey(B));
        Assert.Null(aRes.ResultFromKey(C));

        // a sub-result
        var aSub = new Transfer_ResultFromTransient();
        aSub.SetStart(C);
        aSub.SetBinder(TP.Find(C));
        aRes.MainResult()!.AddSubResult(aSub);
        aRes.MainResult()!.AddSubResult(null);
        Assert.Equal(1, aRes.MainResult()!.NbSubResults());
        Assert.Same(aSub, aRes.ResultFromKey(C));
        Assert.Null(aRes.MainResult()!.SubResult(2));
        Assert.Equal(2, aRes.Results(2).Length());
        Assert.Equal(3, aRes.Results(1).Length()); // OCCT: the main result twice
        Assert.Equal(1, aRes.Results(0).Length());
        Assert.Equal("B", Names(new Interface_EntityIterator(aRes.TransferredList())));
        Assert.Equal("C", Names(new Interface_EntityIterator(aRes.CheckedList(Interface_CheckFail, false))));
        Assert.Equal("", Names(new Interface_EntityIterator(aRes.CheckedList(Interface_CheckFail, true))));
        Interface_CheckIterator aList = aRes.CheckList(false);
        Assert.Equal(Interface_CheckFail, aList.Status());
        Assert.Equal(Interface_CheckOK, aRes.CheckList(true, 0).Status()); // B's warning only
        Assert.Equal(Interface_CheckFail, aRes.CheckStatus());
        Assert.Equal(Interface_CheckFail, aRes.ComputeCheckStatus(false));
        Assert.Equal(Interface_CheckOK, new Transfer_ResultFromTransient().CheckStatus());
        Assert.Equal(0, new Transfer_ResultFromTransient().Check().NbFails());

        // Strip 10 keeps the status and the label, drops the model and the subs
        aRes.Strip(10);
        Assert.Null(aRes.Model());
        Assert.Equal(0, aRes.MainResult()!.NbSubResults());
        Assert.Null(aRes.MainResult()!.Start());
        Assert.Equal("#B", aRes.MainLabel());
        Assert.Equal(Interface_CheckFail, aRes.CheckStatus());
        aRes.Strip(11);
        Assert.Null(aRes.MainResult());
    }
}
