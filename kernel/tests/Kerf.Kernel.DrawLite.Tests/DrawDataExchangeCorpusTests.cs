// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Text.RegularExpressions;
using Kerf.Kernel.DrawLite;
using Xunit.Abstractions;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// Unit 8W's commands (XSDRAWDE's ReadFile, readfile, WriteFile and writefile; XSControl's xnorm; Tcl's exec, binary
/// format and encodings) on hand-made shapes and on OCCT's corpus, and phase 8's third milestone: the data-exchange
/// cases, run as OCCT's harness runs them and judged by OCCT's log check.
/// </summary>
public partial class DrawDataExchangeCorpusTests(ITestOutputHelper output)
{
    private static string TestsDirectory => Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests");

    /// <summary>The Draw and Tcl commands unit 8W added.</summary>
    internal static readonly string[] Unit8WCommands = ["ReadFile", "readfile", "WriteFile", "writefile", "xnorm", "exec", "binary"];

    /// <summary>Draw-lite before unit 8W: without its commands.</summary>
    internal static void Before8W(DrawInterpreter theInterpreter)
    {
        foreach (string aCommand in Unit8WCommands)
        {
            theInterpreter.Eval($"rename {aCommand} {{}}");
        }
    }

    private static DrawInterpreter NewInterpreter(TextWriter? theOutput = null)
    {
        var anInterpreter = new DrawInterpreter(theOutput);
        anInterpreter.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        return anInterpreter;
    }

    [Fact]
    public void WriteFile_and_ReadFile_pick_the_format_by_the_file_and_take_the_configuration()
    {
        DrawInterpreter aDI = NewInterpreter();
        aDI.Eval("box b 10 20 30");

        // a shape through STEP, in the configuration's unit
        aDI.Eval("writefile b /tmp/p8w/b.stp");
        Assert.Contains("ISO-10303-21", aDI.Eval("set f [open /tmp/p8w/b.stp]; set t [read $f]; close $f; set t"), StringComparison.Ordinal);
        aDI.Eval("readfile r /tmp/p8w/b.stp");
        Assert.Contains("SOLID     : 1", aDI.Eval("nbshapes r"), StringComparison.Ordinal);
        aDI.Eval("readfile rm /tmp/p8w/b.stp -conf \"global.general.length.unit : 1000\"");
        Assert.Contains("Mass :", aDI.Eval("vprops rm"), StringComparison.Ordinal);
        Assert.Equal(6.0e-6, double.Parse(Regex.Match(aDI.Eval("vprops rm"), @"Mass :\s*(\S+)").Groups[1].Value,
                                           System.Globalization.CultureInfo.InvariantCulture), 12);

        // a document through STEP: the document's unit is the configuration's
        aDI.Eval("XNewDoc D");
        aDI.Eval("XAddShape D b");
        aDI.Eval("WriteFile D /tmp/p8w/d.step");
        aDI.Eval("ReadFile R /tmp/p8w/d.step");
        Assert.Equal("mm", aDI.Eval("XGetLengthUnit R").Trim());
        aDI.Eval("ReadFile M /tmp/p8w/d.step -conf {global.general.length.unit : 1000}");
        Assert.Equal("m", aDI.Eval("XGetLengthUnit M").Trim());

        // BRep: binary by default, text with the provider's parameter (without Draw's header line)
        aDI.Eval("writefile b /tmp/p8w/b.brep");
        Assert.StartsWith("\nOpen CASCADE Topology", aDI.Eval("set f [open /tmp/p8w/b.brep]; fconfigure $f -translation binary; set t [read $f]; close $f; set t"), StringComparison.Ordinal);
        aDI.Eval("readfile rb /tmp/p8w/b.brep");
        Assert.Contains("FACE      : 6", aDI.Eval("nbshapes rb"), StringComparison.Ordinal);
        aDI.Eval("writefile b /tmp/p8w/t.brep -conf {provider.BREP.OCC.write.binary : 0}");
        string aText = aDI.Eval("set f [open /tmp/p8w/t.brep]; set t [read $f]; close $f; set t");
        Assert.Contains("CASCADE Topology V", aText, StringComparison.Ordinal);
        Assert.DoesNotContain("DBRep_DrawableShape", aText, StringComparison.Ordinal);

        // a format without a provider in the port, an unknown one, a missing file
        Assert.Contains("no provider for the format", aDI.Eval("catch {writefile b /tmp/p8w/b.igs} e; set e"), StringComparison.Ordinal);
        Assert.Contains("no provider is registered", aDI.Eval("catch {writefile b /tmp/p8w/b.xyz} e; set e"), StringComparison.Ordinal);
        Assert.Equal("1", aDI.Eval("catch {readfile x /tmp/p8w/none.stp}"));
        Assert.Contains("incorrect document", aDI.Eval("catch {WriteFile nodoc /tmp/p8w/n.stp} e; set e"), StringComparison.Ordinal);
    }

    [Fact]
    public void Tcl_encodings_convert_strings_and_channels_write_bytes()
    {
        DrawInterpreter aDI = NewInterpreter();
        // byte strings decoded and encoded
        Assert.Equal("Тест", aDI.Eval("encoding convertfrom utf-8 \"\\xD0\\xa2\\xD0\\xB5\\xD1\\x81\\xD1\\x82\""));
        Assert.Equal("国标", aDI.Eval("encoding convertfrom unicode \"\\xFD\\x56\\x07\\x68\""));
        Assert.Equal("Ð¢", aDI.Eval("encoding convertto utf-8 \"Т\""));
        Assert.Equal("À", aDI.Eval("encoding convertto cp1251 \"А\""));
        // invalid UTF-8: the bytes stand for themselves, as Tcl 8.6 reads them
        Assert.Equal("aéb", aDI.Eval("encoding convertfrom utf-8 \"a\\xE9b\""));
        Assert.Contains("unknown encoding", aDI.Eval("catch {encoding convertfrom nosuch x} e; set e"), StringComparison.Ordinal);

        // a channel writes its text in its encoding; a channel reads in the system encoding (utf-8) unless told
        aDI.Eval("set f [open /tmp/p8w/e.txt w]; fconfigure $f -translation lf -encoding cp1251; puts -nonewline $f \"АБ\"; close $f");
        Assert.Equal("2", aDI.Eval("file size /tmp/p8w/e.txt"));
        Assert.Equal("АБ", aDI.Eval("set f [open /tmp/p8w/e.txt]; fconfigure $f -encoding cp1251; set t [read $f]; close $f; set t"));
        aDI.Eval("set f [open /tmp/p8w/u.txt w]; puts -nonewline $f \"А\"; close $f");
        Assert.Equal("2", aDI.Eval("file size /tmp/p8w/u.txt"));
        Assert.Equal("А", aDI.Eval("set f [open /tmp/p8w/u.txt]; set t [read $f]; close $f; set t"));
        Assert.Equal("cp1251", aDI.Eval("set f [open /tmp/p8w/e.txt]; fconfigure $f -encoding cp1251; set e [fconfigure $f -encoding]; close $f; set e"));

        // binary format, as STL's binary files are written by de_mesh/stl_read/D1
        Assert.Equal("\u0001\0\0\0\0\0\u0080?", aDI.Eval("binary format if 1 1.0"));
        Assert.Equal("ab\0\0", aDI.Eval("binary format a4 ab"));
        Assert.Equal(12, aDI.Eval("binary format f3 {0 0 1}").Length);
    }

    [Fact]
    public void Exec_removes_overlay_files_and_runs_nothing()
    {
        DrawInterpreter aDI = NewInterpreter();
        aDI.Eval("set f [open /tmp/p8w/x.txt w]; puts $f x; close $f");
        Assert.Equal("", aDI.Eval("exec chmod 777 /tmp/p8w/x.txt"));
        Assert.Equal("", aDI.Eval("exec rm /tmp/p8w/x.txt"));
        Assert.Equal("0", aDI.Eval("file exists /tmp/p8w/x.txt"));
        Assert.Equal("1", aDI.Eval("catch {exec rm /tmp/p8w/x.txt}"));
        Assert.Equal("", aDI.Eval("exec rm -f /tmp/p8w/x.txt"));
        Assert.Contains("couldn't execute", aDI.Eval("catch {exec ls} e; set e"), StringComparison.Ordinal);

        // xnorm: STEP's norm selected for the session
        Assert.Contains("new norm : STEP", aDI.Eval("xnorm STEP"), StringComparison.Ordinal);
        Assert.Contains("No norm named : IGES", aDI.Eval("catch {xnorm IGES} e; set e"), StringComparison.Ordinal);
    }

    [Fact]
    public void Corpus_cases_using_unit_8Ws_commands_run_after_8W()
    {
        Assert.True(Directory.Exists(TestsDirectory), $"No OCCT source at {TestsDirectory}: run tools/get-occt.ps1");
        var aCorpus = new DrawCorpus(TestsDirectory);
        DrawInterpreter anInterpreter = NewInterpreter();
        DrawInterpreter aBefore = NewInterpreter();
        Before8W(aBefore);

        List<DrawCaseScan> aScans = aCorpus.Cases().AsParallel().AsOrdered()
                                           .Select(c => aCorpus.Scan(c, anInterpreter))
                                           .Where(s => Unit8WCommands.Any(s.Commands.Contains))
                                           .ToList();
        List<DrawTestCase> aRunnable = aScans.Where(s => s.IsRunnable).Select(s => s.Case).ToList();
        List<DrawTestCase> aGained = aRunnable.Where(c => !aCorpus.Scan(c, aBefore).IsRunnable).ToList();
        output.WriteLine($"{aScans.Count} cases name unit 8W's commands; runnable {aRunnable.Count}, made runnable by 8W: {aGained.Count}");

        List<DrawCaseRun> aRuns = aGained.AsParallel().WithDegreeOfParallelism(Math.Max(1, Environment.ProcessorCount / 2))
                                         .Select(c => aCorpus.RunAndCheck(c, DrawResourcesTests.ResourcesDirectory, DrawCorpusTests.DataPath,
                                                                          TimeSpan.FromSeconds(DrawCorpusTests.CorpusTimeLimitSeconds), null))
                                         .OrderBy(r => r.Case.Id, StringComparer.Ordinal)
                                         .ToList();
        output.WriteLine("after 8W: " + Counts(aRuns));
        foreach (DrawCaseRun aRun in aRuns)
        {
            output.WriteLine($"  {aRun.Case.Id}: {aRun.Status}");
            if (!aRun.Passed)
            {
                output.WriteLine($"      {FirstFailure(aRun)}");
            }
        }

        foreach (IGrouping<string, string> aMissing in aScans.Where(s => !s.IsRunnable).SelectMany(s => s.Missing).GroupBy(c => c)
                                                            .OrderByDescending(g => g.Count()).ThenBy(g => g.Key, StringComparer.Ordinal).Take(20))
        {
            output.WriteLine($"  missing {aMissing.Key,-20} {aMissing.Count()}");
        }

        // 42 cases made runnable (de_wrapper's BRep, STEP, STL and glTF grids and the bugs cases removing their files
        // with exec): OK 22, BAD 1 (OCCT's TODO), SKIPPED 18 (data not public), FAILED 1, attributed
        Assert.Equal(42, aGained.Count);
        Assert.Equal(22, aRuns.Count(r => r.Status == "OK"));
        Assert.Equal(18, aRuns.Count(r => r.Status.StartsWith("SKIPPED", StringComparison.Ordinal)));
        Assert.All(aRuns, r => Assert.True(r.Passed || r.Status.StartsWith("SKIPPED", StringComparison.Ordinal) || KnownFailures.ContainsKey(r.Case.Id),
                                           $"{r.Case.Id}: {r.Status}"));
    }

    /// <summary>Cases that fail in Draw-lite, each attributed.</summary>
    internal static readonly Dictionary<string, string> KnownFailures = new(StringComparer.Ordinal)
    {
        // reads an XBF file (OCAF's binary storage) with readfile: Draw-lite has no XBF provider (OCAF storage)
        ["de_wrapper/xcaf/A4"] = "OCAF storage (XBF)",
    };

    private static string Counts(IEnumerable<DrawCaseRun> theRuns)
    {
        string aCounts = string.Join(", ", theRuns.GroupBy(r => r.Status.Split(' ')[0])
                                                  .OrderBy(g => g.Key, StringComparer.Ordinal)
                                                  .Select(g => $"{g.Key} {g.Count()}"));
        return aCounts.Length > 0 ? aCounts : "none";
    }

    private static string FirstFailure(DrawCaseRun theRun)
    {
        string[] aLines = theRun.Log.Split('\n');
        return aLines.FirstOrDefault(l => !l.StartsWith("TODO", StringComparison.Ordinal)
                                          && (l.Contains("rror", StringComparison.Ordinal) || l.Contains("xception", StringComparison.Ordinal)
                                              || l.Contains("Faulty", StringComparison.Ordinal)))
               ?? aLines.Reverse().Skip(1).FirstOrDefault() ?? "";
    }
}
