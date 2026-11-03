// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Exchange.Framework.Monitoring;
using Kerf.Engine.Exchange.Tools;
using Kerf.Engine.Foundation;
using static Kerf.Engine.Exchange.Framework.ParamType;
using static Kerf.Engine.Exchange.Framework.Monitoring.MonitorValueType;

namespace OcctNet.DataExchange.Tests.Interface;

/// <summary>
/// The parameter object (<see cref="StaticSet"/>, OCCT's process-wide statics), MoniTool's typed values and
/// attribute lists, and the file reading framework (FileReaderData, FileReaderTool, ReaderLib) on a toy file.
/// </summary>
[Collection(ProcessWideMessages.Name)]
public class StaticAndReaderTests
{
    [Fact]
    public void A_set_of_statics_is_independent_of_its_copies()
    {
        var aSet = new StaticSet();
        Assert.False(aSet.IsPresent("read.precision.mode"));
        Assert.Equal(0, aSet.IVal("read.precision.mode")); // absent: 0, as OCCT
        aSet.Standards();
        aSet.Standards(); // once per set
        Assert.Equal(1, aSet.IVal("write.surfacecurve.mode"));
        Assert.Equal(0.001, aSet.RVal("read.precision.val"));

        var aCopy = new StaticSet(aSet);
        Assert.True(aCopy.SetCVal("read.precision.mode", "User"));
        Assert.True(aCopy.SetRVal("read.precision.val", 0.01));
        Assert.Equal(1, aCopy.IVal("read.precision.mode"));
        Assert.Equal(0, aSet.IVal("read.precision.mode"));
        Assert.Equal(0.001, aSet.RVal("read.precision.val"));
        Assert.Equal(aSet.Items().Length(), aCopy.Items().Length());
        // the copy keeps the definitions, and later edits of either don't reach the other
        aCopy.Init("XSTEP", "read.precision.mode", '&', "eval Extra");
        Assert.Equal("Extra", aCopy.CDef("read.precision.mode", "enum 2"));
        Assert.Equal("", aSet.CDef("read.precision.mode", "enum 2"));
        Assert.False(aCopy.StartInit("Interface_Static::Standards"));
        Assert.True(aCopy.StartInit("STEPControl_Controller"));
        Assert.True(aSet.StartInit("STEPControl_Controller"));

        var aMap = new NCollection_DataMap<string, string>();
        aCopy.FillMap(aMap);
        Assert.Equal("User", aMap.Find("read.precision.mode"));
        Assert.Equal("0.01", aMap.Find("read.precision.val"));

        // families: those beginning with '$' are listed only by "$"
        aSet.Init("$local", "my.local", 't', "x");
        Assert.Equal(0, aSet.Items(0, "XSTEP8A1").Length());
        Assert.Equal(1, aSet.Items(0, "$").Length());
        Assert.DoesNotContain("my.local", Names(aSet.Items()));
        Assert.True(aSet.Update("my.local"));
        Assert.True(aSet.IsUpdated("my.local"));
        Assert.Equal(0, aSet.Items(100, "$").Length()); // non-updated ones only

        // wild-cards are copied as the copy's own
        Static aWild = aSet.Static("read.precision.val")!;
        aSet.Init("XSTEP", "read.precision.alias", 'r', "");
        aSet.Static("read.precision.alias")!.SetWild(aWild);
        Assert.True(aSet.IsSet("read.precision.alias", false));
        Assert.False(aSet.IsSet("read.precision.alias"));
        var aCopy2 = new StaticSet(aSet);
        Assert.Same(aCopy2.Static("read.precision.val"), aCopy2.Static("read.precision.alias")!.Wild());
    }

    private static List<string> Names(NCollection_HSequence<TCollection_HAsciiString> theList) =>
        theList.Select(s => s.ToCString()).ToList();

    [Fact]
    public void Typed_values_check_interpret_and_hold_objects()
    {
        var anInt = new TypedValue("count", MonitorValueType.ValueInteger, "3");
        Assert.True(anInt.IsSetValue());
        Assert.Equal(3, anInt.IntegerValue());
        anInt.SetIntegerLimit(false, 1);
        Assert.False(anInt.SetCStringValue("0"));
        Assert.True(anInt.SetIntegerValue(7));
        Assert.Equal("7", anInt.CStringValue());
        Assert.Equal("Integer  >= 1", anInt.Definition());
        TCollection_HAsciiString aShared = anInt.HStringValue()!;
        anInt.SetCStringValue("8");
        Assert.Equal("8", aShared.ToCString()); // the value's string is shared and changed in place

        var aText = new TypedValue("name");
        aText.SetMaxLength(3);
        Assert.False(aText.SetCStringValue("long"));
        aText.SetSatisfies(v => v.Search("x") < 0, "NoX");
        Assert.False(aText.Satisfies(new TCollection_HAsciiString("ax")));
        Assert.Equal("NoX", aText.SatisfiesName());

        var anEnum = TypedValue.FromLib("Logical")!;
        Assert.True(anEnum.SetCStringValue("Unknown"));
        Assert.Equal(0, anEnum.IntegerValue());
        Assert.Equal("1", anEnum.Interpret(new TCollection_HAsciiString("True"), false)!.ToCString());
        Assert.Equal("False", anEnum.Interpret(new TCollection_HAsciiString("-1"), true)!.ToCString());
        anEnum.SetInterpret((tv, v, native) => new TCollection_HAsciiString(native ? "N" : "C"));
        Assert.Equal("N", anEnum.Interpret(new TCollection_HAsciiString("True"), true)!.ToCString());
        Assert.NotSame(TypedValue.Lib("Logical"), TypedValue.FromLib("Logical"));
        Assert.Equal(["Integer", "Real", "Text", "Transient", "Boolean", "Logical"], TypedValue.LibList().ToList());

        var anObj = new TypedValue("obj", MonitorValueType.ValueIdent);
        anObj.SetObjectType(typeof(ToyEntity));
        Assert.False(anObj.SetObjectValue(new ToyUnknown()));
        Assert.True(anObj.SetObjectValue(new ToyEntity("o")));
        Assert.Equal("ToyEntity", anObj.ObjectTypeName());
        Assert.Throws<ConstructionErrorException>(() => new TypedValue("bad", MonitorValueType.ValueHexa));
        Assert.Throws<ConstructionErrorException>(() => anObj.SetIntegerLimit(true, 1));

        var aStatic = new Static("fam", "copy.of.logical", ParamType.ParamEnum);
        Assert.Equal(ParamType.ParamEnum, aStatic.Type());
        Assert.Equal("fam", aStatic.Family());
    }

    [Fact]
    public void Attribute_lists_and_statistics()
    {
        var aList = new AttributeList();
        aList.SetIntegerAttribute("n", 4);
        aList.SetRealAttribute("r", 2.5);
        aList.SetStringAttribute("s", "text");
        aList.SetAttribute("e", new ToyEntity("e"));
        Assert.Equal(4, aList.IntegerAttribute("n"));
        Assert.True(aList.GetRealAttribute("r", out double aReal));
        Assert.Equal(2.5, aReal);
        Assert.Equal("text", aList.StringAttribute("s"));
        Assert.Equal(MonitorValueType.ValueIdent, aList.AttributeType("e"));
        Assert.Equal(MonitorValueType.ValueVoid, aList.AttributeType("none"));
        Assert.False(aList.GetAttribute("e", typeof(ToyUnknown), out _));
        var aCopy = new AttributeList();
        aCopy.GetAttributes(aList, "", true);
        aList.SetIntegerAttribute("n", 5);
        Assert.Equal(4, aCopy.IntegerAttribute("n"));
        Assert.True(aList.RemoveAttribute("n"));
        Assert.Equal(0, aList.IntegerAttribute("n"));

        var aStat = new Stat("work");
        int anId = aStat.Open(10);
        aStat.Add(4);
        Assert.Equal(40.0, aStat.Percent(1));
        aStat.Close(anId);
        Assert.Equal(0, aStat.Level());

        var anElem = new TransientElem(aCopy.AttrList().Find("e"));
        Assert.True(anElem.Equates(new TransientElem(anElem.Value())));
        Assert.False(anElem.Equates(new TransientElem(new ToyEntity("x"))));
        Assert.Equal("ToyEntity", anElem.ValueTypeName());
    }

    // A toy file: one record per line, "NAME ref1 ref2..." with references as record numbers ("#2")
    private sealed class ToyReaderData : FileReaderData
    {
        public ToyReaderData(string[] theLines) : base(theLines.Length, theLines.Sum(l => l.Split(' ').Length))
        {
            for (int i = 1; i <= theLines.Length; i++)
            {
                foreach (string aWord in theLines[i - 1].Split(' '))
                {
                    if (aWord.StartsWith('#'))
                    {
                        AddParam(i, aWord, global::Kerf.Engine.Exchange.Framework.ParamType.ParamIdent, int.Parse(aWord[1..], System.Globalization.CultureInfo.InvariantCulture));
                    }
                    else
                    {
                        AddParam(i, aWord, global::Kerf.Engine.Exchange.Framework.ParamType.ParamText);
                    }
                }
                InitParams(i); // the count of parameters at the end of record i
            }
        }

        public override int FindNextRecord(int num) => num < RecordCount ? num + 1 : 0;
    }

    private sealed class ToyReaderModule : ReaderModule
    {
        public override int CaseNum(FileReaderData? data, int num) =>
            data!.ParamCValue(num, 1) == "BAD" ? 0 : 1;

        public override void Read(int casenum, FileReaderData? data, int num, ref FrameworkCheck? ach, Transient? ent)
        {
            var anEnt = (ToyEntity)ent!;
            anEnt.Name = data!.ParamCValue(num, 1)!;
            for (int i = 2; i <= data.NbParams(num); i++)
            {
                if (data.ParamEntity(num, i) is ToyEntity aRef)
                {
                    anEnt.Refs.Add(aRef);
                }
                else
                {
                    ach!.AddWarning("unresolved reference");
                }
            }
        }
    }

    private sealed class ToyReaderTool : FileReaderTool
    {
        private readonly GeneralLib myGLib = new(ToyNorm.ProtocolInstance);
        private readonly ReaderLib myRLib = new(ToyNorm.ProtocolInstance);

        public ToyReaderTool(ToyReaderData theData) => SetData(theData, ToyNorm.ProtocolInstance);

        public override bool Recognize(int num, ref FrameworkCheck? ach, ref Transient? ent) =>
            RecognizeByLib(num, myGLib, myRLib, ref ach, ref ent);

        public override void BeginRead(InterfaceModel? amodel)
        {
        }

        public override bool AnalyseRecord(int num, Transient? anent, ref FrameworkCheck? acheck)
        {
            if (anent is ToyEntity)
            {
                myRModule.Read(1, Data(), num, ref acheck, anent);
            }
            return true;
        }

        private static readonly ToyReaderModule myRModule = ToyReaderRegistration.Module;
    }

    private static class ToyReaderRegistration
    {
        public static readonly ToyReaderModule Module = Register();

        private static ToyReaderModule Register()
        {
            var aModule = new ToyReaderModule();
            ReaderLib.SetGlobal(aModule, ToyNorm.ProtocolInstance);
            return aModule;
        }
    }

    [Fact]
    public void The_file_reader_tool_loads_records_into_a_model()
    {
        _ = ToyReaderRegistration.Module;
        var aData = new ToyReaderData(["P #2 #3", "Q #3", "R", "BAD #1", "S #9"]);
        Assert.Equal(5, aData.NbEntities());
        Assert.Equal(3, aData.NbParams(1));
        Assert.Equal("#2", aData.ParamCValue(1, 2));
        Assert.Equal(2, aData.ParamNumber(1, 2));
        Assert.Equal(ParamType.ParamIdent, aData.ParamType(1, 2));

        var aTool = new ToyReaderTool(aData);
        aTool.SetEntities();
        Assert.IsType<ToyEntity>(aData.BoundEntity(1));
        Assert.IsType<ToyUnknown>(aData.BoundEntity(4)); // not recognised: the protocol's unknown entity
        Assert.Null(aData.BoundEntity(9));

        var aModel = new ToyModel();
        aModel.SetProtocol(ToyNorm.ProtocolInstance);
        aTool.LoadModel(aModel);
        Assert.Equal(5, aModel.EntityCount);
        var aP = (ToyEntity)aModel.Value(1)!;
        Assert.Equal("P", aP.Name);
        Assert.Equal("Q R", string.Join(" ", aP.Refs.Select(r => r.Name)));
        Assert.IsType<ToyUnknown>(aModel.Value(4));
        Assert.True(aModel.IsReportEntity(4)); // with an empty check: a report, not "unknown" (no content)
        Assert.Equal(DataState.LoadWarning, aModel.EntityState(5)); // #9: unresolved
        Assert.Equal(DataState.StateOK, aModel.EntityState(2));
    }
}
