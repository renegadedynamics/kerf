// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Exchange.Framework.Monitoring;
using Kerf.Engine.Foundation;
using static Kerf.Engine.Exchange.Framework.FrameworkCheckStatus;
using static Kerf.Engine.Exchange.Framework.ParamType;

namespace Kerf.Kernel.DataExchange.Tests.Interface;

/// <summary>Interface's lists, iterators, checks, parameters and the library (LibCtl) of modules, class by class.</summary>
[Collection(ProcessWideMessages.Name)]
public class InterfaceTests
{
    [Fact]
    public void EntityIterator_iterates_selects_and_types()
    {
        var anIter = new EntityIterator();
        Assert.Equal(0, anIter.NbEntities());
        Assert.False(anIter.More());
        ToyEntity a = new("a"), b = new("b");
        var u = new ToyUnknown();
        anIter.AddItem(a);
        anIter.AddItem(null); // ignored
        anIter.GetOneItem(u);
        anIter.AddItem(b);
        Assert.Equal(3, anIter.NbEntities());
        Assert.Equal(2, anIter.NbTyped(typeof(ToyEntity)));
        Assert.Equal(2, anIter.Typed(typeof(ToyEntity)).NbEntities());
        var aSeen = new List<Transient>();
        for (anIter.Start(); anIter.More(); anIter.Next())
        {
            aSeen.Add(anIter.Value()!);
        }
        Assert.Equal([a, u, b], aSeen);
        EngineAssert.Throws<NoSuchObjectException>(() => anIter.Value());
        anIter.SelectType(typeof(ToyEntity), false);
        Assert.Equal(1, anIter.NbEntities());
        Assert.Same(u, anIter.Content()!.Value(1));

        var aList = new NCollection_HSequence<Transient>();
        aList.Append(a);
        var aShared = new EntityIterator(aList); // over the list, not a copy
        aList.Append(b);
        Assert.Equal(2, aShared.NbEntities());
    }

    [Fact]
    public void EntityList_keeps_one_entity_alone_then_clusters()
    {
        var aList = new EntityList();
        Assert.True(aList.IsEmpty);
        var anEnts = Enumerable.Range(1, 9).Select(i => new ToyEntity(i.ToString())).ToArray();
        aList.Append(anEnts[0]);
        Assert.Same(anEnts[0], aList.Value(1));
        for (int i = 1; i < 9; i++)
        {
            aList.Append(anEnts[i]);
        }
        Assert.Equal(9, aList.NbEntities());
        Assert.Same(anEnts[6], aList.Value(7));
        Assert.Equal(9, aList.NbTypedEntities(typeof(ToyEntity)));
        EngineAssert.Throws<InterfaceErrorException>(() => aList.TypedEntity(typeof(ToyEntity)));
        Assert.Same(anEnts[0], aList.TypedEntity(typeof(ToyEntity), 4)); // OCCT counts each cluster backwards
        aList.SetValue(2, anEnts[8]);
        Assert.Same(anEnts[8], aList.Value(2));
        aList.Remove(9);
        Assert.Equal(8, aList.NbEntities());
        var anIter = new EntityIterator();
        aList.FillIterator(anIter);
        Assert.Equal(8, anIter.NbEntities());
        // OCCT's EntityCluster::Remove(num) runs out of its array for an inner rank: fixed (occt-issues.md, 8A1)
        aList.Remove(2);
        Assert.Equal(7, aList.NbEntities());
        Assert.Same(anEnts[2], aList.Value(2));
        EngineAssert.Throws<NullObjectException>(() => aList.Append(null));
    }

    [Fact]
    public void IntList_records_references_per_entity()
    {
        var aList = new IntList(5);
        aList.SetNumber(2);
        aList.Add(4);
        Assert.Equal(1, aList.Length());
        Assert.Equal(4, aList.Value(1));
        aList.Add(5);
        aList.Add(1);
        Assert.Equal(3, aList.Length());
        Assert.Equal([4, 5, 1], new[] { aList.Value(1), aList.Value(2), aList.Value(3) });
        aList.SetNumber(3);
        aList.Add(2);
        aList.SetNumber(2);
        Assert.Equal(3, aList.Length());
        IntList aCopy = aList.List(3, true);
        Assert.Equal(1, aCopy.Length());
        Assert.Equal(2, aCopy.Value(1));
        aList.SetNumber(1);
        Assert.Equal(0, aList.Length());
        aList.SetRedefined(true);
        Assert.True(aList.IsRedefined());
        Assert.False(aList.Remove(1)); // not implemented in OCCT
        aList.Clear();
        aList.SetNumber(2);
        Assert.Equal(0, aList.Length());
    }

    [Fact]
    public void Check_collects_and_edits_messages()
    {
        var aCheck = new FrameworkCheck();
        Assert.Equal(FrameworkCheckStatus.CheckOK, aCheck.Status());
        aCheck.AddWarning("a warning", "original warning");
        Assert.Equal(FrameworkCheckStatus.CheckWarning, aCheck.Status());
        aCheck.AddFail("a fail");
        aCheck.AddFail(""); // ignored
        Assert.Equal(FrameworkCheckStatus.CheckFail, aCheck.Status());
        Assert.Equal(1, aCheck.FailCount);
        Assert.Equal("original warning", aCheck.CWarning(1, false));
        Assert.True(aCheck.Complies(FrameworkCheckStatus.CheckMessage));
        Assert.False(aCheck.Complies(FrameworkCheckStatus.CheckNoFail));
        Assert.True(aCheck.Complies(new TCollection_HAsciiString("a warning"), 0, FrameworkCheckStatus.CheckWarning));
        Assert.True(aCheck.Complies(new TCollection_HAsciiString("warn"), -1, FrameworkCheckStatus.CheckWarning));
        Assert.True(aCheck.Mend("FM", 1)); // the fail is marked "Mended"
        Assert.Equal(1, aCheck.FailCount);
        Assert.Equal("Mended : a fail", aCheck.CFail(1));
        Assert.True(aCheck.Mend("CF", 1));
        Assert.Equal(0, aCheck.FailCount);
        aCheck.AddFail("a fail");
        var aText = new StringWriter();
        aCheck.Print(aText, 2);
        Assert.Contains("a fail", aText.ToString());
        var anOther = new FrameworkCheck();
        anOther.GetMessages(aCheck);
        Assert.Equal(1, anOther.WarningCount);
        Assert.Equal(1, anOther.FailCount);
        aCheck.ClearWarnings();
        aCheck.ClearFails();
        Assert.Equal(FrameworkCheckStatus.CheckOK, aCheck.Status());

        // a message from the loaded resources
        StaticSet.LoadMessages();
        var aMsg = new Msg("XSTEP_21");
        aMsg.Arg(3);
        aCheck.SendFail(aMsg);
        Assert.Equal(1, aCheck.FailCount);
    }

    [Fact]
    public void CheckIterator_merges_and_finds_checks()
    {
        var aList = new CheckIterator("checks");
        Assert.Equal("checks", aList.Name());
        var c1 = new FrameworkCheck();
        c1.AddFail("f1");
        var c2 = new FrameworkCheck();
        c2.AddWarning("w2");
        aList.Add(c1, 1);
        aList.Add(c2, 2);
        aList.Add(new FrameworkCheck(), 3); // empty: not added
        var c1b = new FrameworkCheck();
        c1b.AddWarning("w1");
        aList.Add(c1b, 1); // cumulated with the check of 1
        Assert.Equal(1, aList.Check(1)!.WarningCount);
        Assert.False(aList.IsEmpty(true));
        Assert.Equal(FrameworkCheckStatus.CheckFail, aList.Status());
        CheckIterator aWarnings = aList.Extract(FrameworkCheckStatus.CheckWarning);
        Assert.Same(c2, aWarnings.Check(2));
        Assert.Equal(0, aList.Check(9)!.FailCount); // an empty check for a number without one
        aList.CCheck(9)!.AddFail("on the spot");
        Assert.Equal(1, aList.Check(9)!.FailCount);
        var aMerged = new CheckIterator();
        aMerged.Merge(aList);
        Assert.Equal(1, aMerged.Check(2)!.WarningCount);
        var aText = new StringWriter();
        aList.Print(aText, false);
        Assert.Contains("f1", aText.ToString());
        Assert.Contains("on the spot", aText.ToString());
    }

    [Fact]
    public void UndefinedContent_keeps_literals_and_entities()
    {
        var aContent = new UndefinedContent();
        var anEnt = new ToyEntity("e");
        aContent.AddLiteral(ParamType.ParamInteger, new TCollection_HAsciiString("12"));
        aContent.AddEntity(ParamType.ParamIdent, anEnt);
        aContent.AddLiteral(ParamType.ParamText, new TCollection_HAsciiString("'txt'"));
        Assert.Equal(3, aContent.ParamCount);
        Assert.Equal(2, aContent.LiteralCount);
        Assert.True(aContent.IsParamEntity(2));
        Assert.Same(anEnt, aContent.ParamEntity(2));
        Assert.Equal("'txt'", aContent.ParamValue(3)!.ToCString());
        Assert.Equal(ParamType.ParamText, aContent.ParamType(3));
        Assert.True(aContent.ParamData(2, out ParamType aType, out Transient? aRef, out _));
        Assert.Equal(ParamType.ParamIdent, aType);
        Assert.Same(anEnt, aRef);
        aContent.SetLiteral(2, ParamType.ParamReal, new TCollection_HAsciiString("1.5"));
        Assert.False(aContent.IsParamEntity(2));
        Assert.Equal("1.5", aContent.ParamValue(2)!.ToCString());
        aContent.RemoveParam(1);
        Assert.Equal(2, aContent.ParamCount);
        Assert.Equal("1.5", aContent.ParamValue(1)!.ToCString());
    }

    [Fact]
    public void ParamSet_lists_parameters_and_their_texts()
    {
        var aSet = new ParamSet(4);
        Assert.Equal(1, aSet.Append("abcdef", 3, ParamType.ParamText, 0));
        Assert.Equal(2, aSet.Append("#12", -1, ParamType.ParamIdent, 12));
        Assert.Equal("abc", aSet.Param(1).CValue());
        Assert.Equal(12, aSet.Param(2).EntityNumber());
        var aFP = new FileParameter();
        aFP.Init("3.5", ParamType.ParamReal);
        aSet.Append(aFP);
        aFP.Init("changed", ParamType.ParamText); // the set keeps a copy
        Assert.Equal("3.5", aSet.Param(3).CValue());
        aSet.ChangeParam(3).SetEntityNumber(7); // changed in place
        Assert.Equal(7, aSet.Param(3).EntityNumber());
        ParamList aSub = aSet.Params(2, 2);
        Assert.Equal(2, aSub.Length());
        Assert.Equal("#12", aSub.Value(1).CValue());
        Assert.Equal(3, aSet.Params(0, 0).Length());
    }

    [Fact]
    public void HAsciiString_edits_and_searches()
    {
        var s = new TCollection_HAsciiString("  Hello World  ");
        s.LeftAdjust();
        s.RightAdjust();
        Assert.Equal("Hello World", s.ToCString());
        Assert.Equal(7, s.Search("World"));
        Assert.Equal(-1, s.Search("xyz"));
        Assert.Equal(7, s.Location(new TCollection_HAsciiString("Wor"), 1, 11));
        Assert.Equal("World", s.Token(" ", 2).ToCString());
        Assert.Equal("ello", s.SubString(2, 5).ToCString());
        s.UpperCase();
        Assert.Equal("HELLO WORLD", s.ToCString());
        Assert.True(s.IsSameString(new TCollection_HAsciiString("hello world"), false));
        Assert.Equal(12, new TCollection_HAsciiString("12abc").IntegerValue());
        Assert.True(new TCollection_HAsciiString("1.5e3").IsRealValue());
        Assert.Equal("1e+20", new TCollection_HAsciiString(1.0e20).ToCString());
        EngineAssert.Throws<NullObjectException>(() => s.IsSameString(null));
        EngineAssert.Throws<OutOfRangeException>(() => s.Value(0));
    }

    [Fact]
    public void LibCtl_selects_modules_through_protocols_and_their_resources()
    {
        var aLib = new GeneralLib(ToyNorm.ProtocolInstance);
        Assert.True(aLib.Select(new ToyEntity("a"), out GeneralModule? aModule, out int aCN));
        Assert.IsType<ToyGeneralModule>(aModule);
        Assert.Equal(1, aCN);
        Assert.False(aLib.Select(new MonitorIntVal(), out aModule, out aCN));
        Assert.Null(aModule);
        Assert.Equal(0, aCN);
        int aCount = 0;
        for (aLib.Start(); aLib.More(); aLib.Next())
        {
            Assert.Same(ToyNorm.ProtocolInstance, aLib.Protocol());
            aCount++;
        }
        Assert.Equal(1, aCount);

        // a protocol using the toy one as a resource finds the toy module through it
        var aLib2 = new GeneralLib(new ToyUserProtocol());
        Assert.True(aLib2.Select(new ToyEntity("b"), out aModule, out _));
        Assert.IsType<ToyGeneralModule>(aModule);
        Assert.False(new GeneralLib((Protocol?)null).Select(new ToyEntity("c"), out _, out _));
        var aCopy = new GeneralLib(aLib2);
        Assert.True(aCopy.Select(new ToyEntity("d"), out _, out _));
    }

    private sealed class ToyUserProtocol : Protocol
    {
        public override int ResourceCount => 1;

        public override Protocol? Resource(int num) => ToyNorm.ProtocolInstance;

        public override int TypeNumber(Type? atype) => 0;

        public override InterfaceModel? NewModel() => new ToyModel();

        public override bool IsSuitableModel(InterfaceModel? model) => false;

        public override Transient? UnknownEntity() => null;

        public override bool IsUnknownEntity(Transient? ent) => false;
    }

    [Fact]
    public void FloatWriter_writes_STEPs_reals_as_the_general_formats_do()
    {
        // "%14.12lf" means the same as "%14.12f" and goes through Sprintf, so every real is compared with C's rules as ported
        var aRandom = new Random(11);
        var aValues = new List<double>
        {
            0.0, -0.0, 1.0, -1.0, 0.1, -0.1, 1000.0, -1000.0, 999.9999999999999, 0.09999999999999999, 1e-7, 1.5e-7, 1e100, -1e100, 1e-100, 1e300, 1.7976931348623157e308,
            5e-324, 2.2250738585072014e-308, double.NaN, double.PositiveInfinity, double.NegativeInfinity, 123456.789, 0.5, 10.0, 100.0, 1e15, 1e16,
        };
        for (int i = 0; i < 200000; i++)
        {
            aValues.Add(BitConverter.Int64BitsToDouble(aRandom.NextInt64()));
            aValues.Add((aRandom.NextDouble() - 0.5) * Math.Pow(10, aRandom.Next(-20, 20)));
            aValues.Add(aRandom.Next(-5000, 5000) / 8.0);
        }

        foreach (bool aSuppress in new[] { true, false })
        {
            foreach (double aValue in aValues)
            {
                FloatWriter.Convert(aValue, out string aQuick, aSuppress, 0.1, 1000.0, "%14.12E", "%14.12f");
                FloatWriter.Convert(aValue, out string aGeneral, aSuppress, 0.1, 1000.0, "%14.12lE", "%14.12lf");
                Assert.Equal(aGeneral, aQuick);
            }
        }
    }

    [Fact]
    public void FloatWriter_and_LineBuffer_write_as_STEP_does()
    {
        var aWriter = new FloatWriter(12); // StepData_StepWriter's
        Assert.Equal(2, aWriter.Write(0.0, out string aText));
        Assert.Equal("0.", aText);
        aWriter.Write(999.5, out aText);
        Assert.Equal("999.5", aText);
        aWriter.Write(1000.0, out aText); // past the range [0.1, 1000)
        Assert.Equal("1.E+03", aText);
        aWriter.Write(0.5, out aText);
        Assert.Equal("0.5", aText);
        aWriter.Write(1.0e-7, out aText);
        Assert.Equal("1.E-07", aText);
        aWriter.Write(-123456.5, out aText);
        Assert.Equal("-1.234565E+05", aText);

        var aBuf = new LineBuffer(20);
        aBuf.SetInitial(2);
        aBuf.Add("abc");
        Assert.Equal(5, aBuf.Length());
        Assert.Equal("  abc", aBuf.Moved().ToCString());
        EngineAssert.Throws<OutOfRangeException>(() => aBuf.SetInitial(20));
    }

    [Fact]
    public void SignLabel_and_SignType_name_entities()
    {
        ToyModel aModel = ToyNorm.Model(new ToyEntity("lab"));
        var aSign = new SignLabel();
        Assert.Equal("Entity Label", aSign.Name());
        Assert.Equal("#lab", aSign.Text(aModel.Value(1), aModel));
        Assert.Equal("Entity", SignType.ClassName("Toy_Entity"));
        Assert.Equal("ToyEntity", SignType.ClassName("ToyEntity"));
    }
}
