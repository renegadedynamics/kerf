// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Text;
using static Occt.StepData_Logical;
using static Occt.UnitsMethods_LengthUnit;

namespace OcctNet.DataExchange.Tests;

/// <summary>Unit tests of StepData's classes, the header packages, StepFile_Read and UnitsMethods (unit 8B).</summary>
[Collection(ProcessWideMessages.Name)]
public class StepDataTests
{
    private const string Header =
        "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('d'),'2;1');\nFILE_NAME('n','t',('a'),('o'),'p','s','x');\n" +
        "FILE_SCHEMA(('CONFIG_CONTROL_DESIGN'));\nENDSEC;\n";

    private static Stream Bytes(string theText) => new MemoryStream(Encoding.Latin1.GetBytes(theText));

    /// <summary>A file's data, read and prepared as StepFile_Read does (entities bound, references resolved).</summary>
    private static StepData_StepReaderData Prepared(string theData)
    {
        var aFileData = new StepFile_ReadData();
        Assert.Equal(0, new StepFile_Parser(new StepFile_Scanner(aFileData, Bytes(Header + "DATA;\n" + theData + "ENDSEC;\nEND-ISO-10303-21;\n"))).parse());
        aFileData.GetFileNbR(out int nbhead, out int nbrec, out int nbpar);
        var aData = new StepData_StepReaderData(nbhead, nbrec, nbpar);
        for (int nr = 1; nr <= nbrec; nr++)
        {
            aFileData.GetRecordDescription(out string? ident, out string? typrec, out int nbarg);
            aData.SetRecord(nr, ident!, typrec!, nbarg);
            while (nbarg > 0 && aFileData.GetArgDescription(out Interface_ParamType typa, out string? val))
            {
                aData.AddStepParam(nr, val!, typa);
            }

            aData.InitParams(nr);
            aFileData.NextRecord();
        }

        var aTool = new StepData_StepReaderTool(aData, StepDataDump.HeaderOnlyProtocol());
        aTool.PrepareHeader(null);
        aTool.Prepare();
        return aData;
    }

    private static int Record(StepData_StepReaderData theData, int theIdent)
    {
        for (int i = theData.FindNextRecord(0); i != 0; i = theData.FindNextRecord(i))
        {
            if (theData.RecordIdent(i) == theIdent)
            {
                return i;
            }
        }

        throw new InvalidOperationException("#" + theIdent);
    }

    [Fact]
    public void The_read_methods_read_their_parameters_and_report_as_OCCT()
    {
        StepData_StepReaderData aData = Prepared(
            "#1=POINT('p',(1.,2.5,-3.E1),7,2.6,.T.,.U.,.RED.,*,$,#2,'\\X2\\00E9\\X0\\t\\S\\a');\n#2=THING(());\n");
        int aNum = Record(aData, 1);
        var ach = new Interface_Check();
        double x = 0, y = 0, z = 0, r = 0;
        int i = 0, e = 0;
        bool b = false;
        StepData_Logical l = StepData_Logical.StepData_LFalse;
        TCollection_HAsciiString? s = null;
        Transient? ent = null;

        Assert.True(aData.CheckNbParams(aNum, 11, ach, "point"));
        Assert.True(aData.ReadString(aNum, 1, "name", ach, ref s));
        Assert.Equal("p", s!.ToCString());
        Assert.True(aData.ReadXYZ(aNum, 2, "coords", ach, ref x, ref y, ref z));
        Assert.Equal((1.0, 2.5, -30.0), (x, y, z));
        Assert.True(aData.ReadInteger(aNum, 3, "i", ach, ref i));
        Assert.Equal(7, i);
        Assert.False(aData.ReadInteger(aNum, 4, "r", ach, ref i)); // a real is rounded, with a fail
        Assert.Equal(3, i);
        Assert.True(aData.ReadReal(aNum, 3, "i", ach, ref r)); // an integer reads as a real
        Assert.Equal(7.0, r);
        Assert.True(aData.ReadBoolean(aNum, 5, "b", ach, ref b));
        Assert.True(b);
        Assert.True(aData.ReadLogical(aNum, 6, "l", ach, ref l));
        Assert.Equal(StepData_Logical.StepData_LUnknown, l);
        Assert.True(aData.ReadEnum(aNum, 7, "e", ach, new StepData_EnumTool(".GREEN.", ".RED."), ref e));
        Assert.Equal(1, e);
        Assert.True(aData.CheckDerived(aNum, 8, "d", ach));
        Assert.False(aData.ReadEntity(aNum, 9, "void", ach, typeof(Transient), ref ent));
        Assert.True(aData.ReadEntity(aNum, 10, "ref", ach, typeof(StepData_UndefinedEntity), ref ent));
        Assert.Same(aData.BoundEntity(Record(aData, 2)), ent);
        StepData_UndefinedEntity? und = null;
        Assert.True(aData.ReadEntity(aNum, 10, "ref", ach, typeof(StepData_UndefinedEntity), ref und));
        Assert.True(aData.ReadString(aNum, 11, "text", ach, ref s));
        Assert.Equal("étá", s!.ToCString()); // \X2\, then \S\ of 'a' (0x61 | 0x80, ISO 8859-1)

        Assert.Equal(new[] { "Parameter n0.4 (r) was rounded", "Parameter n0.9 (void) not an Entity" },
                     Enumerable.Range(1, ach.NbFails()).Select(n => ach.CFail(n)).ToArray());
        int numsub = -1;
        Assert.True(aData.ReadSubList(Record(aData, 2), 1, "empty", ach, ref numsub));
        Assert.Equal("Parameter n0.1 (empty) is an empty LIST", ach.CWarning(1));
    }

    [Fact]
    public void Unresolved_references_and_complex_types_are_reported_in_the_global_check()
    {
        StepData_StepReaderData aData = Prepared("#1=A(#9);\n#2=(B(1)C(2)A(3));\n");
        Interface_Check aCheck = aData.GlobalCheck();
        Assert.Contains("Unresolved Reference, Ent.Id.#1 Param.n0 1 (Id.#9)", Enumerable.Range(1, aCheck.NbFails()).Select(n => aCheck.CFail(n)));
        Assert.Equal("Complex Type incorrect : C / A ... ", aCheck.CWarning(1));
        int aNum = Record(aData, 2);
        Assert.True(aData.IsComplex(aNum));
        var aTypes = new NCollection_Sequence<string>();
        aData.ComplexType(aNum, aTypes);
        Assert.Equal(new[] { "B", "C", "A" }, aTypes.ToArray());
        var ach = new Interface_Check();
        int n = 0;
        Assert.False(aData.NamedForComplex("A", aNum, ref n, ach)); // found, but not in alphabetic order
        Assert.Equal("Complex Record n0." + aNum + ", member type A not in alphabetic order", ach.CWarning(1));
    }

    [Fact]
    public void A_file_reads_into_a_model_with_its_header_and_writes_back()
    {
        var aModel = new StepData_StepModel();
        string aText = Header + "DATA;\n#10=PRODUCT('cafÃ©','',(#20),.T.);\n#20=CONTEXT($);\nENDSEC;\nEND-ISO-10303-21;\n";
        Assert.Equal(0, StepFile_Read.Read("mem", Bytes(aText), aModel, StepDataDump.HeaderOnlyProtocol()));
        Assert.Equal(2, aModel.NbEntities());
        Assert.Equal(10, aModel.IdentLabel(aModel.Value(1)));
        var aHeader = new APIHeaderSection_MakeHeader(aModel);
        Assert.True(aHeader.IsDone());
        Assert.Equal("CONFIG_CONTROL_DESIGN", aHeader.SchemaIdentifiersValue(1)!.ToCString());
        Assert.Equal("x", aHeader.Authorisation()!.ToCString());

        var aWriter = new StepData_StepWriter(aModel);
        aWriter.SendModel(StepDataDump.HeaderOnlyProtocol());
        var anOut = new MemoryStream();
        Assert.True(aWriter.Print(anOut));
        string aWritten = Encoding.Latin1.GetString(anOut.ToArray());
        Assert.Contains("#1 = PRODUCT('cafÃ©','',/* (SUB) */(#2),.T.);\n#2 = CONTEXT($);\n", aWritten); // the file's bytes again
        Assert.StartsWith("ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('d'),'2;1');\n", aWritten, StringComparison.Ordinal);

        Assert.Equal(-1, StepFile_Read.Read(Path.Combine(Path.GetTempPath(), "no-such-file-8b.stp"), null, new StepData_StepModel(), StepDataDump.HeaderOnlyProtocol()));
    }

    [Fact]
    public void Reading_stops_on_cancellation_and_on_limits()
    {
        // The parser looks at the progress every 4,096 tokens
        string aText = Header + "DATA;\n" + string.Concat(Enumerable.Range(1, 3000).Select(n => $"#{n}=A({n});\n")) + "ENDSEC;\nEND-ISO-10303-21;\n";
        using var aSource = new CancellationTokenSource();
        aSource.Cancel();
        Assert.Equal(1, StepFile_Read.Read("mem", Bytes(aText), new StepData_StepModel(), StepDataDump.HeaderOnlyProtocol(),
                                           new ProgressIndicator(aSource.Token).Start()));
        Assert.Equal(1, StepFile_Read.Read("mem", Bytes(aText), new StepData_StepModel(), StepDataDump.HeaderOnlyProtocol(),
                                           default, new StepFile_ReadLimits { MaxRecords = 2 }));
    }

    [Fact]
    public void Model_texts_are_written_as_UTF8_and_long_texts_are_cut_as_OCCT_cuts_them()
    {
        StepData_StepModel aModel = new APIHeaderSection_MakeHeader().NewModel(StepDataDump.HeaderOnlyProtocol());
        var aWriter = new StepData_StepWriter(aModel);
        aWriter.SendIdent(1);
        aWriter.StartEntity("NAMED");
        aWriter.Send("été π");
        aWriter.Send(new string('x', 100));
        aWriter.SendEnum("RED");
        aWriter.SendLogical(StepData_Logical.StepData_LUnknown);
        aWriter.Send(1.5);
        aWriter.EndEntity();
        string[] aLines = Enumerable.Range(1, aWriter.NbLines()).Select(n => aWriter.Line(n).ToCString()).ToArray();
        Assert.Equal("#1 =NAMED('Ã©tÃ© Ï\u0080',", aLines[0]); // UTF-8 bytes, one char per byte
        Assert.Equal("'" + new string('x', 71), aLines[1]);
        Assert.Equal(new string('x', 29) + "',.RED.,.U.,1.5);", aLines[2]);
        Assert.All(aLines, l => Assert.True(l.Length <= 72));
    }

    [Fact]
    public void The_header_is_made_dumped_and_copied()
    {
        var aMake = new APIHeaderSection_MakeHeader(4);
        aMake.AddSchemaIdentifier(new TCollection_HAsciiString("AP242"));
        Assert.Equal("Open CASCADE Manifold Solid Brep Model", aMake.Name()!.ToCString());
        Assert.Equal("Open CASCADE STEP processor 8.0", aMake.PreprocessorVersion()!.ToCString());
        Assert.Equal(2, aMake.NbSchemaIdentifiers());
        StepData_StepModel aModel = aMake.NewModel(StepDataDump.HeaderOnlyProtocol());
        Assert.Equal(3, aModel.Header().NbEntities());
        // An empty schema name is taken from the protocol: none here (a FileProtocol's is "")
        Assert.Equal("", aModel.HeaderEntity(typeof(HeaderSection_FileSchema)) is HeaderSection_FileSchema fs ? fs.SchemaIdentifiersValue(1)!.ToCString() : null);

        var aDump = new StringWriter();
        aModel.DumpHeader(aDump);
        Assert.Contains(" --   Dumped with Protocol : HeaderSection_Protocol   --", aDump.ToString());
        Assert.Contains("FILE_NAME('Open CASCADE Manifold Solid Brep Model',", aDump.ToString());

        var aCopy = new StepData_StepModel();
        aCopy.GetFromAnother(aModel);
        var aCopied = new APIHeaderSection_MakeHeader(aCopy);
        Assert.True(aCopied.IsDone());
        Assert.NotSame(aMake.FnValue(), aCopied.FnValue());
        Assert.Equal(aMake.TimeStamp()!.ToCString(), aCopied.TimeStamp()!.ToCString());
    }

    private sealed class ColourSelect : StepData_SelectType
    {
        public override int CaseNum(Transient? ent) => ent is StepData_UndefinedEntity ? 1 : 0;

        public override int CaseMem(StepData_SelectMember? ent) => ent is not null && (ent.Name() == "RGB" || ent.Name() == "") ? 1 : 0;
    }

    [Fact]
    public void Select_types_and_members_behave_as_OCCT()
    {
        var aSelect = new ColourSelect();
        var aMember = new StepData_SelectInt();
        Assert.Throws<TypeMismatchException>(() => aSelect.SetValue(new TCollection_HAsciiString("x")));
        aSelect.SetValue(new StepData_UndefinedEntity());
        Assert.Equal(1, aSelect.CaseNumber());
        aSelect.SetReal(0.5, "RGB");
        Assert.Equal(0.5, aSelect.Real());
        Assert.Equal("RGB", aSelect.SelectName());
        Assert.IsType<StepData_SelectNamed>(aSelect.Value());
        Assert.Throws<TypeMismatchException>(() => aSelect.SetInteger(3, "HSV"));

        // OCCT stores every logical as 0 (occt-issues.md, 8B)
        aMember.SetLogical(StepData_Logical.StepData_LTrue);
        Assert.Equal(StepData_Logical.StepData_LFalse, aMember.Logical());
        Assert.Equal(Interface_ParamType.Interface_ParamLogical, aMember.ParamType());
    }

    [Fact]
    public void Fields_hold_values_lists_and_copies()
    {
        var aField = new StepData_Field();
        Assert.False(aField.IsSet());
        aField.SetInteger();
        aField.SetList(3);
        aField.SetInteger(2, 5);
        Assert.Equal((1, 3, 1, 5), (aField.Arity(), aField.Length(), aField.Kind(), aField.Integer(2)));
        aField.SetReal(1, 2.5); // a list of integers takes no real: nothing changes (OCCT)
        Assert.Equal(0.0, aField.Real(1));

        var aNamed = new StepData_SelectNamed();
        aNamed.SetName("LENGTH");
        aNamed.SetReal(3.0);
        var aSelect = new StepData_Field();
        aSelect.SetSelectMember(aNamed);
        var aCopy = new StepData_Field(aSelect, true); // OCCT recurses without end here; the port copies the member's value
        Assert.NotSame(aNamed, aCopy.Transient());
        Assert.Equal(3.0, aCopy.Real());
        Assert.Equal("", ((StepData_SelectNamed)aCopy.Transient()).Name()); // OCCT names the copy with its own empty name

        var aList = new StepData_Field();
        aList.Set(new NCollection_HArray1<double>(1, 2, 4.0));
        Assert.Equal((5, 2, 4.0), (aList.Kind(), aList.Length(), aList.Real(2)));
    }

    [Fact]
    public void Enum_tools_and_descriptors_translate_as_OCCT()
    {
        var aTool = new StepData_EnumTool(".RED.", "GREEN", "BLUE YELLOW", "$");
        // OCCT: the second term of a definition keeps the blank before it
        Assert.Equal((0, 1, 2, 3, -1), (aTool.Value(".RED."), aTool.Value(".GREEN."), aTool.Value(".BLUE."), aTool.Value(". YELLOW."), aTool.Value(".YELLOW.")));
        Assert.Equal(".GREEN.", aTool.Text(1));
        Assert.Equal("", aTool.Text(9));
        Assert.Equal("$.", aTool.Text(4)); // OCCT adds a dot to "$" too, so the null value is never found
        Assert.Equal(-1, aTool.NullValue());

        var aDescr = new StepData_ESDescr("THING");
        aDescr.SetNbFields(2);
        var aReal = new StepData_PDescr();
        aReal.SetReal();
        aDescr.SetField(1, "size", aReal);
        aDescr.SetField(2, "name", new StepData_PDescr());
        Assert.Equal((2, "name"), (aDescr.Rank("name"), aDescr.Name(2)));
        Assert.True(aDescr.Field(1)!.IsReal());
        var aProtocol = new StepData_Protocol();
        aProtocol.AddDescr(aDescr, 5);
        var aSimple = (StepData_Simple)aDescr.NewEntity();
        Assert.Equal(5, aProtocol.CaseNumber(aSimple));
        Assert.Same(aDescr, aProtocol.Descr(5));
        Assert.Same(aDescr, aProtocol.ESDescr("THING"));
        aSimple.CField("size").SetReal(2.0);
        Assert.Equal(2.0, aSimple.Field("size").Real());
    }

    [Fact]
    public void Free_form_entities_reorder_their_members()
    {
        var aB = new StepData_FreeFormEntity();
        aB.SetStepType("B");
        var anA = new StepData_FreeFormEntity();
        anA.SetStepType("A");
        aB.SetNext(anA);
        StepData_FreeFormEntity? anEnt = aB;
        Assert.True(StepData_FreeFormEntity.Reorder(ref anEnt));
        Assert.Same(aB, anEnt); // OCCT returns the last member of the new chain
        Assert.Equal(new[] { "A", "B" }, anA.TypeList().ToArray());
    }

    [Fact]
    public void Units_convert_as_OCCT()
    {
        Assert.Equal(UnitsMethods_LengthUnit.UnitsMethods_LengthUnit_Inch, UnitsMethods.GetLengthUnitByFactorValue(25.4));
        Assert.Equal(UnitsMethods_LengthUnit.UnitsMethods_LengthUnit_Meter, UnitsMethods.GetLengthUnitByFactorValue(1.0, UnitsMethods_LengthUnit.UnitsMethods_LengthUnit_Meter));
        Assert.Equal("stat.mile", UnitsMethods.DumpLengthUnit(1609344.0));
        Assert.Equal(UnitsMethods_LengthUnit.UnitsMethods_LengthUnit_Centimeter, UnitsMethods.LengthUnitFromString("CM", false));
        Assert.Equal(UnitsMethods_LengthUnit.UnitsMethods_LengthUnit_Undefined, UnitsMethods.LengthUnitFromString("CM", true));
        Assert.Equal(0.001, UnitsMethods.GetLengthUnitScale(UnitsMethods_LengthUnit.UnitsMethods_LengthUnit_Millimeter, UnitsMethods_LengthUnit.UnitsMethods_LengthUnit_Meter));
        Assert.Equal(1.0, UnitsMethods.GetCasCadeLengthUnit(null));
        var aStatics = new Interface_StaticSet();
        aStatics.Init("XSTEP", "xstep.cascade.unit", 'e', "");
        aStatics.Init("XSTEP", "xstep.cascade.unit", '&', "enum 1");
        aStatics.Init("XSTEP", "xstep.cascade.unit", '&', "eval INCH");
        aStatics.Init("XSTEP", "xstep.cascade.unit", '&', "eval MM");
        aStatics.SetIVal("xstep.cascade.unit", 1);
        Assert.Equal(25.4, UnitsMethods.GetCasCadeLengthUnit(aStatics));
        Assert.Equal(0.0254, UnitsMethods.GetCasCadeLengthUnit(aStatics, UnitsMethods_LengthUnit.UnitsMethods_LengthUnit_Meter), 15);

        StepData_Factors aFactors = new StepData_Factors().InitializedFactors(2.0, 0.5, 1.0).WithCascadeUnit(10.0);
        Assert.Equal((2.0, 0.5, 2.0, 0.5, 10.0), (aFactors.LengthFactor(), aFactors.PlaneAngleFactor(), aFactors.FactorRadianDegree(),
                                                  aFactors.FactorDegreeRadian(), aFactors.CascadeUnit()));
        Assert.Equal(1.0, new StepData_Factors().CascadeUnit());
    }

    [Fact]
    public void Code_pages_decode_through_dotnet_with_OCCTs_corrections()
    {
        Assert.Equal("€", Resource_Unicode.ConvertFormatToUnicode(Resource_FormatType.Resource_FormatType_CP1252, "\u0080"));
        Assert.Equal("Ж", Resource_Unicode.ConvertFormatToUnicode(Resource_FormatType.Resource_FormatType_CP1251, "Æ"));
        Assert.Equal("あ", Resource_Unicode.ConvertFormatToUnicode(Resource_FormatType.Resource_FormatType_SJIS, "\u0082 "));
        Assert.Equal("啊", Resource_Unicode.ConvertFormatToUnicode(Resource_FormatType.Resource_FormatType_GB, "°¡"));
        Assert.Equal("café", Resource_Unicode.ConvertFormatToUnicode(Resource_FormatType.Resource_FormatType_UTF8, "cafÃ©"));
        // Not UTF-8 (a code point beyond U+10FFFF): one char per byte
        Assert.Equal("ÿÿ", Resource_Unicode.ConvertFormatToUnicode(Resource_FormatType.Resource_FormatType_UTF8, "ÿÿ"));
        // OCCT's GBK and Big5 converters insert an ASCII byte after a lead byte "at 0", which throws
        Assert.Throws<OutOfRangeException>(() => Resource_Unicode.ConvertFormatToUnicode(Resource_FormatType.Resource_FormatType_Big5, "¤" + "0"));
    }
}
