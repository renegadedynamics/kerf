// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Text;
using static Occt.Interface_ParamType;

namespace OcctNet.DataExchange.Tests;

/// <summary>
/// StepFile_Scanner, StepFile_Parser and StepFile_ReadData by hand (unit 8P): what a small file leaves, the scanner's
/// odd rules, error messages, the limits for hostile input, cancellation, and reentrancy.
/// </summary>
public class StepFileParserTests
{
    private const string Head = "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('d'),'2;1');\nENDSEC;\nDATA;\n";
    private const string Tail = "ENDSEC;\nEND-ISO-10303-21;\n";

    private static (int Result, StepFile_ReadData Data, StepFile_Parser Parser) Parse(string theText,
        StepFile_ReadLimits? theLimits = null, ProgressRange theRange = default)
    {
        StepFile_ReadData aData = new();
        StepFile_Parser aParser = new(new StepFile_Scanner(aData, new MemoryStream(Encoding.Latin1.GetBytes(theText)), theLimits), theRange);
        return (aParser.parse(), aData, aParser);
    }

    private static List<string> Lines(string theText) => StepFileDump.Dump(new MemoryStream(Encoding.Latin1.GetBytes(theText)));

    [Fact]
    public void A_small_file_gives_its_records_and_arguments()
    {
        var (aResult, aData, _) = Parse(Head + "#1=CARTESIAN_POINT('',(0.,1.E+00,-2.5));\n#2=VERTEX_POINT('',#1);\n" + Tail);
        Assert.Equal(0, aResult);
        aData.GetFileNbR(out int aNbHead, out int aNbRec, out int aNbPar);
        Assert.Equal(2, aNbHead); // FILE_DESCRIPTION and its sub-list
        Assert.Equal(5, aNbRec);
        Assert.Equal(10, aNbPar);
        Assert.Null(aData.GetLastError());

        // The header's sub-list comes before its entity, as in OCCT
        Assert.True(aData.GetRecordDescription(out string? anIdent, out string? aType, out int aNbArg));
        Assert.Equal(("$1", "/* (SUB) */", 1), (anIdent, aType, aNbArg));
        Assert.True(aData.GetArgDescription(out Interface_ParamType anArgType, out string? aValue));
        Assert.Equal((Interface_ParamType.Interface_ParamText, "'d'"), (anArgType, aValue));
        Assert.False(aData.GetArgDescription(out _, out _));
        aData.NextRecord();
        aData.GetRecordDescription(out anIdent, out aType, out _);
        Assert.Equal(("#0", "FILE_DESCRIPTION"), (anIdent, aType));
        aData.NextRecord();
        aData.GetRecordDescription(out anIdent, out aType, out _);
        Assert.Equal(("$1", "/* (SUB) */"), (anIdent, aType));
        var anArgs = new List<(Interface_ParamType, string?)>();
        while (aData.GetArgDescription(out anArgType, out aValue))
        {
            anArgs.Add((anArgType, aValue));
        }

        Assert.Equal([(Interface_ParamType.Interface_ParamReal, "0."), (Interface_ParamType.Interface_ParamReal, "1.E+00"), (Interface_ParamType.Interface_ParamReal, "-2.5")], anArgs);
        aData.NextRecord();
        aData.GetRecordDescription(out anIdent, out aType, out _);
        Assert.Equal(("#1", "CARTESIAN_POINT"), (anIdent, aType));
        Assert.True(aData.GetArgDescription(out _, out _));
        Assert.True(aData.GetArgDescription(out anArgType, out aValue));
        Assert.Equal((Interface_ParamType.Interface_ParamSub, "$1"), (anArgType, aValue));
        aData.NextRecord();
        aData.GetRecordDescription(out anIdent, out aType, out _);
        Assert.Equal(("#2", "VERTEX_POINT"), (anIdent, aType));
        aData.GetArgDescription(out _, out _);
        aData.GetArgDescription(out anArgType, out aValue);
        Assert.Equal((Interface_ParamType.Interface_ParamIdent, "#1"), (anArgType, aValue));
        aData.NextRecord();
        Assert.False(aData.GetRecordDescription(out _, out _, out _));
    }

    [Theory]
    // The number rules: 1E3 is a name (rule 40 is longer than 18), 12E3 a real (20 ties 40 and comes first)
    [InlineData("1E3", "A 0 1E3")]
    [InlineData("12E3", "A 2 12E3")]
    [InlineData("-", "A 1 -")]
    [InlineData("...", "A 2 ...")]
    [InlineData("T.", "A 6 T.")]
    [InlineData("1.", "A 2 1.")]
    [InlineData("\"0f\"", "A 9 \"0f\"")]
    [InlineData("'a' \t,'b')", "A 5 'a' \\x09,'b'")]
    [InlineData("'a\0b'", "A 5 'a")]
    public void The_scanner_reproduces_OCCTs_rules(string theArg, string theFirstArg)
    {
        List<string> aLines = Lines(Head + "#1=FOO(" + theArg + ");\n" + Tail);
        Assert.Contains(theFirstArg, aLines);
    }

    [Fact]
    public void Syntax_errors_are_reported_on_the_next_line_as_in_OCCT()
    {
        // OCCT's message adds 1 to flex's line counter, which starts at 1: the error on line 6 is reported on line 7
        var (aResult, aData, _) = Parse(Head + "#1=FOO(1;\n#2=BAR(2);\n" + Tail);
        Assert.Equal(0, aResult);
        Assert.Equal("Undefined Parsing: Line 7: Incorrect syntax: unexpected ';', expecting ')' or ','", aData.InternalErrors().First());
    }

    [Fact]
    public void An_unexpected_end_is_an_abort_after_a_header_and_an_accept_without_one()
    {
        Assert.Equal(1, Parse(Head + "#1=FOO(1);\n").Result);
        // stepf3: an empty header, then the error token at the end of the input
        Assert.Equal(0, Parse("ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n#1=FOO(1);\n").Result);
    }

    [Fact]
    public void A_token_of_16384_bytes_throws_as_OCCTs_scanner_does()
    {
        // The string with its quotes, and the ')' the scanner looks at to end it: 16,383 bytes at most
        string aShort = Head + "#1=A('" + new string('x', StepFile_Scanner.MaxToken - 3) + "');\n" + Tail;
        Assert.Equal(0, Parse(aShort).Result);
        string aLong = Head + "#1=A('" + new string('x', StepFile_Scanner.MaxToken - 2) + "');\n" + Tail;
        FailureException anEx = Assert.Throws<FailureException>(() => Parse(aLong));
        Assert.Equal("input buffer overflow, can't enlarge buffer because scanner uses REJECT", anEx.Message);
    }

    [Fact]
    public void The_record_limit_stops_the_parse()
    {
        string aText = Head + string.Concat(Enumerable.Range(1, 100).Select(i => $"#{i}=A({i});\n")) + Tail;
        var (aResult, aData, aParser) = Parse(aText, new StepFile_ReadLimits { MaxRecords = 50 });
        Assert.Equal(1, aResult);
        Assert.Equal("record count", aParser.ExceededLimit);
        Assert.Contains("Limit reached: record count above 50", aData.GetLastError());
        Assert.Equal(0, Parse(aText, new StepFile_ReadLimits { MaxRecords = 102 }).Result);
    }

    [Fact]
    public void The_depth_limit_stops_the_parse()
    {
        string aText = Head + "#1=A(" + new string('(', 30) + "1" + new string(')', 30) + ");\n" + Tail;
        Assert.Equal(0, Parse(aText, new StepFile_ReadLimits { MaxDepth = 31 }).Result);
        var (aResult, _, aParser) = Parse(aText, new StepFile_ReadLimits { MaxDepth = 30 });
        Assert.Equal(1, aResult);
        Assert.Equal("nesting depth", aParser.ExceededLimit);

        // Far deeper than OCCT's corpus needs: lists are read in a loop, not a recursion
        string aDeep = Head + "#1=A(" + new string('(', 9000) + "1" + new string(')', 9000) + ");\n" + Tail;
        Assert.Equal(0, Parse(aDeep).Result);
    }

    [Fact]
    public void The_file_size_limit_stops_the_parse()
    {
        string aText = Head + string.Concat(Enumerable.Range(1, 30000).Select(i => $"#{i}=A({i});\n")) + Tail;
        var (aResult, _, aParser) = Parse(aText, new StepFile_ReadLimits { MaxFileSize = 100_000 });
        Assert.Equal(1, aResult);
        Assert.Equal("file size", aParser.ExceededLimit);
        Assert.Equal(0, Parse(aText, new StepFile_ReadLimits { MaxFileSize = aText.Length }).Result);
    }

    [Fact]
    public void Cancellation_stops_the_parse_and_progress_is_reported()
    {
        string aText = Head + string.Concat(Enumerable.Range(1, 30000).Select(i => $"#{i}=A({i});\n")) + Tail;
        using var aSource = new CancellationTokenSource();
        aSource.Cancel();
        var (aResult, aData, aParser) = Parse(aText, theRange: new ProgressIndicator(aSource.Token).Start());
        Assert.Equal(1, aResult);
        Assert.True(aParser.IsUserBreak);
        Assert.Contains("Reading cancelled", aData.GetLastError());

        var anIndicator = new ProgressIndicator();
        Assert.Equal(0, Parse(aText, theRange: anIndicator.Start()).Result);
        Assert.Equal(1.0, anIndicator.GetPosition(), 1e-9);
    }

    [Fact]
    public void Files_parse_in_parallel_as_serially()
    {
        List<(string Name, byte[] Data)> anInputs = StepFileDump.Pack("StepFile.cases.pack.gz");
        string[] aSerial = anInputs.Select(i => StepFileDump.Hash(StepFileDump.Dump(new MemoryStream(i.Data)))).ToArray();
        string[] aParallel = new string[anInputs.Count];
        Parallel.For(0, anInputs.Count, i => aParallel[i] = StepFileDump.Hash(StepFileDump.Dump(new MemoryStream(anInputs[i].Data))));
        Assert.Equal(aSerial, aParallel);
    }

    [Fact]
    public void The_print_modes_write_the_records()
    {
        StepFile_ReadData aData = new();
        var aWriter = new StringWriter();
        aData.PrintOutput = aWriter;
        aData.SetModePrint(2);
        Assert.Equal(2, aData.GetModePrint());
        new StepFile_Parser(new StepFile_Scanner(aData, new MemoryStream(Encoding.Latin1.GetBytes(Head + "#1=A(1,'x');\n" + Tail)))).parse();
        string aText = aWriter.ToString();
        Assert.Contains("Record no : 3 -- Ident : #1  Type : A  Nb.Arg.s : 1", aText);
        // OCCT: its two-letter codes of the argument types predate Interface_ParamType's order: a text prints "nd"
        Assert.Contains("  - Arg.1[In] : 1  - Arg.2[nd] : 'x'", aText);
    }

    [Fact]
    public void ClearRecorder_forgets_the_records()
    {
        var (_, aData, _) = Parse(Head + "#1=A(1);\n" + Tail);
        aData.ClearRecorder(3);
        aData.GetFileNbR(out _, out int aNbRec, out _);
        Assert.Equal(3, aNbRec); // the counters stay, as in OCCT
        Assert.False(aData.GetRecordDescription(out _, out _, out _));
    }
}
