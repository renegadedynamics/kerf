// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Exchange.Framework.Graph;
using Kerf.Engine.Exchange.Framework.Monitoring;
using Kerf.Engine.Foundation;
using static Kerf.Kernel.DataExchange.Tests.Interface.ToyNorm;

namespace Kerf.Kernel.DataExchange.Tests.Interface;

/// <summary>
/// The model, graph, share, check and copy tools of Interface and the graph tools of IFGraph on a toy norm: A shares
/// B and C, B shares C, D and E share each other (a cycle), F is alone, G shares X, which is not in the model.
/// </summary>
[Collection(ProcessWideMessages.Name)]
public class GraphTests
{
    private readonly ToyEntity A = new("A"), B = new("B"), C = new("C"), D = new("D"), E = new("E"), F = new("F"),
        G = new("G"), X = new("X");

    private readonly ToyModel myModel;

    public GraphTests()
    {
        A.Ref(B, C);
        B.Ref(C);
        D.Ref(E);
        E.Ref(D);
        G.Ref(X);
        myModel = Model(A, B, C, D, E, F, G);
    }

    [Fact]
    public void The_model_numbers_its_entities_in_order()
    {
        Assert.Equal(7, myModel.EntityCount);
        Assert.Equal(3, myModel.Number(C));
        Assert.Same(D, myModel.Value(4));
        Assert.Equal(0, myModel.Number(X));
        Assert.False(myModel.Contains(X));
        Assert.Equal("A B C D E F G", Names(myModel.Entities()));
        Assert.Equal(5, myModel.NextNumberForLabel("#e"));
        Assert.Equal(0, myModel.NextNumberForLabel("#Z"));
        Assert.Equal(6, myModel.NextNumberForLabel("F", 0, false));
        Assert.Equal("ToyEntity", myModel.TypeName(A)); // the GTool's sign: the class name past its first "_"
        Assert.Equal("Entity", InterfaceModel.ClassName("Toy_Entity"));
        var aText = new StringWriter();
        myModel.Print(B, aText);
        Assert.Equal("2:#B", aText.ToString());

        myModel.ReverseOrders(4);
        Assert.Equal("A B C D G F E", Names(myModel.Entities()));
        myModel.ReverseOrders(4);
        Assert.Equal("A B C D E F G", Names(myModel.Entities()));
    }

    [Fact]
    public void AddWithRefs_adds_an_entity_and_what_it_shares()
    {
        var aModel = new ToyModel();
        aModel.AddWithRefs(A, ToyNorm.ProtocolInstance);
        Assert.Equal("A B C", Names(aModel.Entities()));
        Assert.Same(ToyNorm.ProtocolInstance, aModel.Protocol());
        aModel.AddWithRefs(D);
        Assert.Equal("A B C D E", Names(aModel.Entities()));
    }

    [Fact]
    public void The_graph_finds_shared_and_sharing_entities()
    {
        var aGraph = new EntityGraph(myModel);
        Assert.Equal(7, aGraph.Size());
        Assert.Equal("B C", Names(aGraph.Shareds(A)));
        Assert.Equal("A B", Names(aGraph.Sharings(C)));
        Assert.Equal("A F G", Names(aGraph.RootEntities()));
        Assert.True(aGraph.HasShareErrors(G));
        Assert.False(aGraph.HasShareErrors(A));
        Assert.Equal("C", Names(aGraph.TypedSharings(C, typeof(ToyEntity)))); // C is of the type itself
        // OCCT: Name() returns nothing for an entity of the model (its test is inverted)
        Assert.Null(aGraph.Name(A));

        aGraph.GetFromEntity(A, true, 5);
        Assert.True(aGraph.IsPresent(B));
        Assert.Equal(5, aGraph.Status(3));
        Assert.False(aGraph.IsPresent(D));
        aGraph.GetFromEntity(B, false, 7, 1, true);
        Assert.Equal(6, aGraph.Status(2)); // already present: cumulated
        aGraph.RemoveItem(2);
        Assert.False(aGraph.IsPresent(2));
        aGraph.ResetStatus();
        Assert.False(aGraph.IsPresent(1));

        var aCopy = new EntityGraph(aGraph);
        Assert.Equal("B C", Names(aCopy.Shareds(A)));
        aCopy.GetFromModel();
        Assert.True(aCopy.IsPresent(7));
        Assert.False(aGraph.IsPresent(7)); // the copy has its own statuses
    }

    [Fact]
    public void The_share_tools_find_roots_and_everything_shared()
    {
        var aShare = new ShareTool(myModel, ToyNorm.ProtocolInstance);
        Assert.Equal("A F G", Names(aShare.RootEntities()));
        // OCCT: IsShared tells whether the entity shares others
        Assert.True(aShare.IsShared(A));
        Assert.False(aShare.IsShared(C));
        Assert.Equal("A B", Names(aShare.Sharings(C)));
        Assert.Equal(2, aShare.NbTypedSharings(C, typeof(ToyEntity)));
        Assert.Equal("A B C", Names(aShare.All(A)));
        Assert.Equal("C B A", Names(aShare.All(A, false)));

        var aFlags = new ShareFlags(myModel, ToyNorm.ProtocolInstance);
        Assert.Equal("A F G", Names(aFlags.RootEntities()));
        Assert.Equal(3, aFlags.RootCount);
        Assert.Same(F, aFlags.Root(2));
        Assert.True(aFlags.IsShared(E));
    }

    [Fact]
    public void IFGraph_finds_closures_parts_and_cycles()
    {
        var aGraph = new EntityGraph(myModel);
        Assert.Equal("A B C", Sorted(Evaluated(new AllShared(aGraph, A))));
        // OCCT: AllConnected::GetFromEntity returns at once for an entity not yet present (its test is inverted), so
        // it finds nothing, and the connected components are all empty
        Assert.Equal("", Sorted(Evaluated(new AllConnected(aGraph, C))));
        var aParts = new ConnectedComponants(aGraph, true);
        var aFound = new List<string>();
        for (aParts.Start(); aParts.More(); aParts.Next())
        {
            aFound.Add(Sorted(aParts.Entities()));
        }
        Assert.All(aFound, p => Assert.Equal("", p));

        var aCycles = new Cycles(aGraph, true);
        aFound.Clear();
        for (aCycles.Start(); aCycles.More(); aCycles.Next())
        {
            aFound.Add(Sorted(aCycles.Entities()));
        }
        // OCCT 8's StrongComponants puts each entity in a part of its own (no cycle detection), so Cycles finds none
        Assert.Empty(aFound);

        var aStrong = new StrongComponants(aGraph, true);
        int aSingles = 0;
        for (aStrong.Start(); aStrong.More(); aStrong.Next())
        {
            if (aStrong.IsSingle)
            {
                aSingles++;
            }
        }
        Assert.Equal(7, aSingles);
        Assert.Equal(7, aStrong.PartCount);

        // OCCT: GetFromEntity iterates an AllShared it doesn't evaluate, so it adds nothing; from evaluated contents:
        var aCumul = new Cumulate(aGraph);
        aCumul.GetFromEntity(A);
        Assert.Equal(-1, aCumul.NbTimes(A));
        aCumul.GetFromIter(Evaluated(new AllShared(aGraph, A)));
        aCumul.GetFromIter(Evaluated(new AllShared(aGraph, B)));
        Assert.Equal(2, aCumul.NbTimes(C));
        Assert.Equal(1, aCumul.NbTimes(A));
        Assert.Equal("B C", Sorted(aCumul.Overlapped()));
        Assert.Equal("D E F G", Sorted(aCumul.Forgotten()));
        Assert.Equal(2, aCumul.HighestNbTimes());

        var aCompare = new Compare(aGraph);
        aCompare.GetFromIter(Evaluated(new AllShared(aGraph, A)), true); // GetFromEntity: as Cumulate's
        aCompare.GetFromIter(Evaluated(new AllShared(aGraph, B)), false);
        Assert.Equal("B C", Sorted(aCompare.Common()));
        Assert.Equal("A", Sorted(aCompare.FirstOnly()));
        Assert.Equal("", Sorted(aCompare.SecondOnly()));

        var aSources = new ExternalSources(aGraph);
        aSources.GetFromEntity(B);
        Assert.Equal("A", Sorted(Evaluated(aSources)));
        Assert.False(aSources.IsEmpty());

        var aRoots = new SCRoots(aGraph, true);
        aFound.Clear();
        for (aRoots.Start(); aRoots.More(); aRoots.Next())
        {
            aFound.Add(Sorted(aRoots.Entities()));
        }
        aFound.Sort(StringComparer.Ordinal);
        Assert.Equal(["A", "D", "F", "G"], aFound); // a root takes what it shares: B, C under A; E under D
    }

    [Fact]
    public void The_check_tool_lists_fails_and_warnings()
    {
        B.Fails = true;
        C.Warns = true;
        var aTool = new CheckTool(myModel, ToyNorm.ProtocolInstance);
        CheckIterator aComplete = aTool.CompleteCheckList();
        var aNums = new List<int>();
        for (aComplete.Start(); aComplete.More(); aComplete.Next())
        {
            aNums.Add(aComplete.Number());
        }
        Assert.Equal([2, 3], aNums); // OCCT: the lists check without try/catch, which skips the shared items unknown from the model (G)
        Assert.Equal("fails: B", aComplete.Check(2)!.CFail(1));
        Assert.Equal("** Shared Items unknown from the containing Model", aTool.Check(7)!.CFail(1)); // with try/catch
        Assert.Equal(FrameworkCheckStatus.CheckFail, aComplete.Status());
        Assert.True(aComplete.Complies(FrameworkCheckStatus.CheckMessage));

        CheckIterator aFails = aTool.CheckList();
        Assert.Equal(1, CountChecks(aFails));
        CheckIterator aWarnings = aTool.WarningCheckList();
        Assert.Equal(1, CountChecks(aWarnings));
        // OCCT: WarningCheckList adds its working check and then clears it for the next entities, so the check recorded
        // for C ends with the last entity set (F, the one before G) and no message
        Assert.Same(F, aWarnings.Check(3)!.Entity());
        Assert.Equal(0, aWarnings.Check(3)!.WarningCount);
        EngineAssert.Throws<CheckFailureException>(() => aTool.CheckSuccess());
        Assert.Empty(Names(aTool.UnknownEntities()));

        var aText = new StringWriter();
        aComplete.Print(aText, myModel, false);
        Assert.Contains("fails: B", aText.ToString());
        Assert.Contains("warns: C", aText.ToString());

        CheckIterator anExtract = aComplete.Extract("warns", -1, FrameworkCheckStatus.CheckWarning);
        Assert.Equal(1, CountChecks(anExtract));
        // OCCT: Interface_Check::Remove looks for fails among the warnings, so a fail is never removed
        Assert.False(aComplete.Remove("fails: B", 0, FrameworkCheckStatus.CheckFail));
        Assert.True(aComplete.Remove("warns: C", 0, FrameworkCheckStatus.CheckWarning));
        Assert.Equal(0, aComplete.Check(3)!.WarningCount); // the check stays, empty
        Assert.Equal(2, CountChecks(aComplete));

        myModel.FillSemanticChecks(aTool.CompleteCheckList());
        Assert.True(myModel.HasSemanticChecks);
        Assert.Equal(DataState.DataFail, myModel.EntityState(2));
        Assert.Equal(DataState.DataWarning, myModel.EntityState(3));
        Assert.Equal(DataState.StateOK, myModel.EntityState(1));
    }

    // A graph content is evaluated by Begin (or Evaluate), not by Start: as in OCCT
    private static GraphContent Evaluated(GraphContent theContent)
    {
        theContent.Evaluate();
        return theContent;
    }

    private static int CountChecks(CheckIterator theList)
    {
        int aCount = 0;
        for (theList.Start(); theList.More(); theList.Next())
        {
            aCount++;
        }
        return aCount;
    }

    [Fact]
    public void The_copy_tool_copies_entities_and_their_references()
    {
        var aTool = new CopyTool(myModel, ToyNorm.ProtocolInstance);
        var aCopyA = (ToyEntity)aTool.Transferred(A)!;
        Assert.Equal("A'", aCopyA.Name);
        Assert.Equal("B' C'", string.Join(" ", aCopyA.Refs.Select(r => r.Name)));
        Assert.Same(aCopyA.Refs[1], aCopyA.Refs[0].Refs[0]); // C copied once
        Assert.True(aTool.Search(B, out Transient? aCopyB));
        Assert.Same(aCopyA.Refs[0], aCopyB);

        var aNewModel = new ToyModel();
        aTool.FillModel(aNewModel);
        Assert.Equal("A' B' C'", Names(aNewModel.Entities()));
        // OCCT: the nesting level counts after the copy, so the entities copied from a root's copy are roots too
        Assert.Equal("C' B' C' A'", Names(aTool.RootResult()));

        // the copy tool from a model alone takes the model's protocol (OCCT: the process-wide active protocol)
        var aTool2 = new CopyTool(myModel);
        Assert.Equal("D'", ((ToyEntity)aTool2.Transferred(D)!).Name);
        var anOrphan = new ToyModel();
        EngineAssert.Throws<InterfaceErrorException>(() => new CopyTool(anOrphan));
    }

    [Fact]
    public void Categories_and_the_GTool_go_through_the_general_module()
    {
        var aCat = new Category(ToyNorm.ProtocolInstance);
        var aShare = new ShareTool(myModel, ToyNorm.ProtocolInstance);
        Assert.Equal(2, aCat.CatNum(A, aShare));
        aCat.Compute(myModel, aShare);
        Assert.Equal(2, aCat.Num(7));
        Assert.Equal(0, aCat.Num(8));
        Assert.Equal("Structure", Category.Name(2));
        Assert.Equal("unspecified", Category.Name(9));

        var aGTool = new GTool(ToyNorm.ProtocolInstance);
        Assert.True(aGTool.Select(A, out GeneralModule? aModule, out int aCN));
        Assert.IsType<ToyGeneralModule>(aModule);
        Assert.Equal(1, aCN);
        Assert.True(aGTool.Select(B, out GeneralModule? aModule2, out aCN)); // from the cache
        Assert.Same(aModule, aModule2);
        Assert.False(aGTool.Select(new MonitorIntVal(), out _, out aCN));
        Assert.Equal("Class Name", aGTool.SignName());
        Assert.Equal("ToyEntity", aGTool.SignValue(A, myModel));
    }

    [Fact]
    public void Reports_are_kept_by_entity_number()
    {
        var aCheck = new FrameworkCheck(B);
        aCheck.AddWarning("read with a warning");
        var aRep = new ReportEntity(aCheck, B);
        Assert.True(myModel.SetReportEntity(2, aRep));
        Assert.True(myModel.IsReportEntity(2));
        Assert.Equal(DataState.LoadWarning, myModel.EntityState(2));
        Assert.Same(aRep, myModel.ReportEntity(2));
        Assert.Equal(2, myModel.Number(aRep)); // a report numbers as its concerned entity
        Assert.Equal("B", Names(new EntityIterator(myModel.Reports().Content()) is var it ? Concerned(it) : it));
        Assert.True(myModel.ClearReportEntity(2));
        Assert.False(myModel.IsReportEntity(2));
    }

    private static EntityIterator Concerned(EntityIterator theReports)
    {
        var aRes = new EntityIterator();
        for (theReports.Start(); theReports.More(); theReports.Next())
        {
            aRes.AddItem(((ReportEntity)theReports.Value()!).Concerned());
        }
        return aRes;
    }
}
