// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static OcctNet.DataExchange.Tests.Interface.ToyNorm;

namespace OcctNet.DataExchange.Tests.Interface;

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
        Assert.Equal(7, myModel.NbEntities());
        Assert.Equal(3, myModel.Number(C));
        Assert.Same(D, myModel.Value(4));
        Assert.Equal(0, myModel.Number(X));
        Assert.False(myModel.Contains(X));
        Assert.Equal("A B C D E F G", Names(myModel.Entities()));
        Assert.Equal(5, myModel.NextNumberForLabel("#e"));
        Assert.Equal(0, myModel.NextNumberForLabel("#Z"));
        Assert.Equal(6, myModel.NextNumberForLabel("F", 0, false));
        Assert.Equal("ToyEntity", myModel.TypeName(A)); // the GTool's sign: the class name past its first "_"
        Assert.Equal("Entity", Interface_InterfaceModel.ClassName("Toy_Entity"));
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
        aModel.AddWithRefs(A, Protocol);
        Assert.Equal("A B C", Names(aModel.Entities()));
        Assert.Same(Protocol, aModel.Protocol());
        aModel.AddWithRefs(D);
        Assert.Equal("A B C D E", Names(aModel.Entities()));
    }

    [Fact]
    public void The_graph_finds_shared_and_sharing_entities()
    {
        var aGraph = new Interface_Graph(myModel);
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

        var aCopy = new Interface_Graph(aGraph);
        Assert.Equal("B C", Names(aCopy.Shareds(A)));
        aCopy.GetFromModel();
        Assert.True(aCopy.IsPresent(7));
        Assert.False(aGraph.IsPresent(7)); // the copy has its own statuses
    }

    [Fact]
    public void The_share_tools_find_roots_and_everything_shared()
    {
        var aShare = new Interface_ShareTool(myModel, Protocol);
        Assert.Equal("A F G", Names(aShare.RootEntities()));
        // OCCT: IsShared tells whether the entity shares others
        Assert.True(aShare.IsShared(A));
        Assert.False(aShare.IsShared(C));
        Assert.Equal("A B", Names(aShare.Sharings(C)));
        Assert.Equal(2, aShare.NbTypedSharings(C, typeof(ToyEntity)));
        Assert.Equal("A B C", Names(aShare.All(A)));
        Assert.Equal("C B A", Names(aShare.All(A, false)));

        var aFlags = new Interface_ShareFlags(myModel, Protocol);
        Assert.Equal("A F G", Names(aFlags.RootEntities()));
        Assert.Equal(3, aFlags.NbRoots());
        Assert.Same(F, aFlags.Root(2));
        Assert.True(aFlags.IsShared(E));
    }

    [Fact]
    public void IFGraph_finds_closures_parts_and_cycles()
    {
        var aGraph = new Interface_Graph(myModel);
        Assert.Equal("A B C", Sorted(Evaluated(new IFGraph_AllShared(aGraph, A))));
        // OCCT: AllConnected::GetFromEntity returns at once for an entity not yet present (its test is inverted), so
        // it finds nothing, and the connected components are all empty
        Assert.Equal("", Sorted(Evaluated(new IFGraph_AllConnected(aGraph, C))));
        var aParts = new IFGraph_ConnectedComponants(aGraph, true);
        var aFound = new List<string>();
        for (aParts.Start(); aParts.More(); aParts.Next())
        {
            aFound.Add(Sorted(aParts.Entities()));
        }
        Assert.All(aFound, p => Assert.Equal("", p));

        var aCycles = new IFGraph_Cycles(aGraph, true);
        aFound.Clear();
        for (aCycles.Start(); aCycles.More(); aCycles.Next())
        {
            aFound.Add(Sorted(aCycles.Entities()));
        }
        // OCCT 8's StrongComponants puts each entity in a part of its own (no cycle detection), so Cycles finds none
        Assert.Empty(aFound);

        var aStrong = new IFGraph_StrongComponants(aGraph, true);
        int aSingles = 0;
        for (aStrong.Start(); aStrong.More(); aStrong.Next())
        {
            if (aStrong.IsSingle())
            {
                aSingles++;
            }
        }
        Assert.Equal(7, aSingles);
        Assert.Equal(7, aStrong.NbParts());

        // OCCT: GetFromEntity iterates an AllShared it doesn't evaluate, so it adds nothing; from evaluated contents:
        var aCumul = new IFGraph_Cumulate(aGraph);
        aCumul.GetFromEntity(A);
        Assert.Equal(-1, aCumul.NbTimes(A));
        aCumul.GetFromIter(Evaluated(new IFGraph_AllShared(aGraph, A)));
        aCumul.GetFromIter(Evaluated(new IFGraph_AllShared(aGraph, B)));
        Assert.Equal(2, aCumul.NbTimes(C));
        Assert.Equal(1, aCumul.NbTimes(A));
        Assert.Equal("B C", Sorted(aCumul.Overlapped()));
        Assert.Equal("D E F G", Sorted(aCumul.Forgotten()));
        Assert.Equal(2, aCumul.HighestNbTimes());

        var aCompare = new IFGraph_Compare(aGraph);
        aCompare.GetFromIter(Evaluated(new IFGraph_AllShared(aGraph, A)), true); // GetFromEntity: as Cumulate's
        aCompare.GetFromIter(Evaluated(new IFGraph_AllShared(aGraph, B)), false);
        Assert.Equal("B C", Sorted(aCompare.Common()));
        Assert.Equal("A", Sorted(aCompare.FirstOnly()));
        Assert.Equal("", Sorted(aCompare.SecondOnly()));

        var aSources = new IFGraph_ExternalSources(aGraph);
        aSources.GetFromEntity(B);
        Assert.Equal("A", Sorted(Evaluated(aSources)));
        Assert.False(aSources.IsEmpty());

        var aRoots = new IFGraph_SCRoots(aGraph, true);
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
        var aTool = new Interface_CheckTool(myModel, Protocol);
        Interface_CheckIterator aComplete = aTool.CompleteCheckList();
        var aNums = new List<int>();
        for (aComplete.Start(); aComplete.More(); aComplete.Next())
        {
            aNums.Add(aComplete.Number());
        }
        Assert.Equal([2, 3], aNums); // OCCT: the lists check without try/catch, which skips the shared items unknown from the model (G)
        Assert.Equal("fails: B", aComplete.Check(2)!.CFail(1));
        Assert.Equal("** Shared Items unknown from the containing Model", aTool.Check(7)!.CFail(1)); // with try/catch
        Assert.Equal(Interface_CheckStatus.Interface_CheckFail, aComplete.Status());
        Assert.True(aComplete.Complies(Interface_CheckStatus.Interface_CheckMessage));

        Interface_CheckIterator aFails = aTool.CheckList();
        Assert.Equal(1, CountChecks(aFails));
        Interface_CheckIterator aWarnings = aTool.WarningCheckList();
        Assert.Equal(1, CountChecks(aWarnings));
        // OCCT: WarningCheckList adds its working check and then clears it for the next entities, so the check recorded
        // for C ends with the last entity set (F, the one before G) and no message
        Assert.Same(F, aWarnings.Check(3)!.Entity());
        Assert.Equal(0, aWarnings.Check(3)!.NbWarnings());
        Assert.Throws<Interface_CheckFailure>(() => aTool.CheckSuccess());
        Assert.Empty(Names(aTool.UnknownEntities()));

        var aText = new StringWriter();
        aComplete.Print(aText, myModel, false);
        Assert.Contains("fails: B", aText.ToString());
        Assert.Contains("warns: C", aText.ToString());

        Interface_CheckIterator anExtract = aComplete.Extract("warns", -1, Interface_CheckStatus.Interface_CheckWarning);
        Assert.Equal(1, CountChecks(anExtract));
        // OCCT: Interface_Check::Remove looks for fails among the warnings, so a fail is never removed
        Assert.False(aComplete.Remove("fails: B", 0, Interface_CheckStatus.Interface_CheckFail));
        Assert.True(aComplete.Remove("warns: C", 0, Interface_CheckStatus.Interface_CheckWarning));
        Assert.Equal(0, aComplete.Check(3)!.NbWarnings()); // the check stays, empty
        Assert.Equal(2, CountChecks(aComplete));

        myModel.FillSemanticChecks(aTool.CompleteCheckList());
        Assert.True(myModel.HasSemanticChecks());
        Assert.Equal(Interface_DataState.Interface_DataFail, myModel.EntityState(2));
        Assert.Equal(Interface_DataState.Interface_DataWarning, myModel.EntityState(3));
        Assert.Equal(Interface_DataState.Interface_StateOK, myModel.EntityState(1));
    }

    // A graph content is evaluated by Begin (or Evaluate), not by Start: as in OCCT
    private static Interface_GraphContent Evaluated(Interface_GraphContent theContent)
    {
        theContent.Evaluate();
        return theContent;
    }

    private static int CountChecks(Interface_CheckIterator theList)
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
        var aTool = new Interface_CopyTool(myModel, Protocol);
        var aCopyA = (ToyEntity)aTool.Transferred(A)!;
        Assert.Equal("A'", aCopyA.Name);
        Assert.Equal("B' C'", string.Join(" ", aCopyA.Refs.Select(r => r.Name)));
        Assert.Same(aCopyA.Refs[1], aCopyA.Refs[0].Refs[0]); // C copied once
        Assert.True(aTool.Search(B, out Standard_Transient? aCopyB));
        Assert.Same(aCopyA.Refs[0], aCopyB);

        var aNewModel = new ToyModel();
        aTool.FillModel(aNewModel);
        Assert.Equal("A' B' C'", Names(aNewModel.Entities()));
        // OCCT: the nesting level counts after the copy, so the entities copied from a root's copy are roots too
        Assert.Equal("C' B' C' A'", Names(aTool.RootResult()));

        // the copy tool from a model alone takes the model's protocol (OCCT: the process-wide active protocol)
        var aTool2 = new Interface_CopyTool(myModel);
        Assert.Equal("D'", ((ToyEntity)aTool2.Transferred(D)!).Name);
        var anOrphan = new ToyModel();
        Assert.Throws<Interface_InterfaceError>(() => new Interface_CopyTool(anOrphan));
    }

    [Fact]
    public void Categories_and_the_GTool_go_through_the_general_module()
    {
        var aCat = new Interface_Category(Protocol);
        var aShare = new Interface_ShareTool(myModel, Protocol);
        Assert.Equal(2, aCat.CatNum(A, aShare));
        aCat.Compute(myModel, aShare);
        Assert.Equal(2, aCat.Num(7));
        Assert.Equal(0, aCat.Num(8));
        Assert.Equal("Structure", Interface_Category.Name(2));
        Assert.Equal("unspecified", Interface_Category.Name(9));

        var aGTool = new Interface_GTool(Protocol);
        Assert.True(aGTool.Select(A, out Interface_GeneralModule? aModule, out int aCN));
        Assert.IsType<ToyGeneralModule>(aModule);
        Assert.Equal(1, aCN);
        Assert.True(aGTool.Select(B, out Interface_GeneralModule? aModule2, out aCN)); // from the cache
        Assert.Same(aModule, aModule2);
        Assert.False(aGTool.Select(new MoniTool_IntVal(), out _, out aCN));
        Assert.Equal("Class Name", aGTool.SignName());
        Assert.Equal("ToyEntity", aGTool.SignValue(A, myModel));
    }

    [Fact]
    public void Reports_are_kept_by_entity_number()
    {
        var aCheck = new Interface_Check(B);
        aCheck.AddWarning("read with a warning");
        var aRep = new Interface_ReportEntity(aCheck, B);
        Assert.True(myModel.SetReportEntity(2, aRep));
        Assert.True(myModel.IsReportEntity(2));
        Assert.Equal(Interface_DataState.Interface_LoadWarning, myModel.EntityState(2));
        Assert.Same(aRep, myModel.ReportEntity(2));
        Assert.Equal(2, myModel.Number(aRep)); // a report numbers as its concerned entity
        Assert.Equal("B", Names(new Interface_EntityIterator(myModel.Reports().Content()) is var it ? Concerned(it) : it));
        Assert.True(myModel.ClearReportEntity(2));
        Assert.False(myModel.IsReportEntity(2));
    }

    private static Interface_EntityIterator Concerned(Interface_EntityIterator theReports)
    {
        var aRes = new Interface_EntityIterator();
        for (theReports.Start(); theReports.More(); theReports.Next())
        {
            aRes.AddItem(((Interface_ReportEntity)theReports.Value()!).Concerned());
        }
        return aRes;
    }
}
