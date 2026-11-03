// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Kernel.DrawLite;

namespace Kerf.Kernel.DrawLite.Tests;

public class DrawInterpreterTests
{
    [Fact]
    public void Substitutes_variables_commands_and_quotes()
    {
        var anOutput = new StringWriter();
        var anInterpreter = new DrawInterpreter(anOutput);

        anInterpreter.Eval("""
            set a 1
            set b "x$a y"
            set c {literal $a [not run]}
            set d [set b]2
            puts "$b|$c|$d|${a}"
            """);

        Assert.Equal("x1 y|literal $a [not run]|x1 y2|1", anOutput.ToString().TrimEnd());
    }

    [Fact]
    public void Handles_comments_semicolons_and_continuations()
    {
        var anInterpreter = new DrawInterpreter();

        string aResult = anInterpreter.Eval("""
            # a comment \
              continued
            set a 1; set b 2
            set c \
                3
            set c
            """);

        Assert.Equal("3", aResult);
        Assert.True(anInterpreter.TryGetVariable("b", out string aB) && aB == "2");
    }

    [Fact]
    public void A_lone_dollar_is_literal()
        => Assert.Equal("cost $ 5$", new DrawInterpreter().Eval("set a \"cost $ 5$\""));

    [Fact]
    public void Viewer_commands_do_nothing()
        => Assert.Equal("", new DrawInterpreter().Eval("pload TOPTEST\ncheckview -display result -2d -path ${imagedir}.png".Replace("${imagedir}", "x")));

    [Fact]
    public void Unknown_commands_are_reported_with_their_line()
    {
        var anError = Assert.Throws<DrawCommandNotFoundException>(() => new DrawInterpreter().Eval("set a 1\nblend1 r a b"));

        Assert.Equal("blend1", anError.Command);
        Assert.Equal(2, anError.Line);
    }

    [Fact]
    public void Registered_commands_receive_substituted_arguments()
    {
        var anInterpreter = new DrawInterpreter();
        anInterpreter.Register("sum", (_, args) => args.Skip(1).Sum(double.Parse).ToString(System.Globalization.CultureInfo.InvariantCulture));

        Assert.Equal("6", anInterpreter.Eval("set x 3\nsum 1 2 $x"));
    }

    [Fact]
    public void Parses_tcl_words_strictly()
    {
        Assert.Equal("{]}", Tcl.Eval("set x [list \"]\"]"));
        Assert.Equal("a;b", Tcl.Eval("set x [string map {, ;} a,b]"));
        Assert.Equal("{a b}", Tcl.Eval("set x [list [list a b]]"));
        Assert.Equal("extra characters after close-quote", Tcl.Error("set x \"a\"b"));
        Assert.Equal("extra characters after close-brace", Tcl.Error("set x {a}b"));
        Assert.Equal("missing close-brace", Tcl.Error("set x {a"));
        Assert.Equal("missing close-bracket", Tcl.Error("set x [list a"));
        Assert.Equal("missing \"", Tcl.Error("set x \"a"));
        Assert.Equal("a\"b", Tcl.Eval("set x a\"b"));
        Assert.Equal("#not-a-comment", Tcl.Eval("set x #not-a-comment"));
    }

    [Fact]
    public void Errors_carry_the_line_and_file()
    {
        string aPath = Path.GetTempFileName();
        try
        {
            File.WriteAllText(aPath, "set a 1\n\nnosuch 2\n");
            var anError = Assert.Throws<DrawCommandNotFoundException>(() => new DrawInterpreter().Source(aPath.Replace('\\', '/')));
            Assert.Equal(3, anError.Line);
            Assert.Equal(aPath.Replace('\\', '/'), anError.File);
        }
        finally
        {
            File.Delete(aPath);
        }
    }

    [Fact]
    public void An_unknown_procedure_receives_calls_of_missing_commands()
        => Assert.Equal("stub:blend1 r 1", new DrawInterpreter().Eval("proc unknown {args} {return \"stub:$args\"}; blend1 r 1"));

    [Fact]
    public void Commands_registered_from_csharp_can_set_variables_in_the_calling_proc()
    {
        var anInterpreter = new DrawInterpreter();
        anInterpreter.Register("setfive", static (i, a) =>
        {
            i.SetVariable(a[1], "5");
            return "";
        });
        Assert.Equal("5 0", anInterpreter.Eval("proc p {} {setfive v; return $v}; list [p] [info exists v]"));
    }

    [Fact]
    public void Subcommands_may_be_abbreviated()
    {
        var anInterpreter = new DrawInterpreter();
        Assert.Equal("1 3 1 a.b", anInterpreter.Eval("set x 1; list [info exist x] [string len abc] [string is int 5] [file ta /d/a.b]"));
        Assert.StartsWith("ambiguous subcommand \"t\": must be bytelength, cat, compare,", Assert.ThrowsAny<DrawException>(() => anInterpreter.Eval("string t abc")).Reason);
        Assert.StartsWith("unknown or ambiguous subcommand \"zz\"", Assert.ThrowsAny<DrawException>(() => anInterpreter.Eval("info zz")).Reason);
        Assert.Equal("c b a", anInterpreter.Eval("lsort -decr {a c b}"));
    }

    [Fact]
    public void A_deadline_stops_a_runaway_script_and_cannot_be_caught()
    {
        var anInterpreter = new DrawInterpreter { Deadline = DateTime.UtcNow.AddMilliseconds(200) };
        Assert.Throws<DrawTimeoutException>(() => anInterpreter.Eval("while 1 { catch { while 1 {} } }"));
    }

    [Fact]
    public void Deep_recursion_within_tcl_limits_works()
    {
        Assert.Equal("900", new DrawInterpreter().Eval("proc d {n} {if {$n == 0} {return 0}; expr {1 + [d [expr {$n - 1}]]}}; d 900"));
        Assert.Contains("too many nested evaluations", Assert.ThrowsAny<DrawException>(() => new DrawInterpreter().Eval("set s {eval $s}; eval $s")).Reason);
    }

    [Fact]
    public void Corpus_cases_run_as_the_harness_runs_them()
    {
        string aRoot = Directory.CreateTempSubdirectory("drawlite-corpus").FullName;
        try
        {
            string aGrid = Path.Combine(aRoot, "group", "grid");
            Directory.CreateDirectory(aGrid);
            File.WriteAllText(Path.Combine(aRoot, "group", "begin"), "set log group-begin\n");
            File.WriteAllText(Path.Combine(aGrid, "begin"), "lappend log grid-begin\n");
            File.WriteAllText(Path.Combine(aGrid, "case1"), "lappend log \"$groupname/$gridname/$casename\" [file tail [info script]]\n");
            File.WriteAllText(Path.Combine(aGrid, "end"), "lappend log grid-end\n");
            var aCorpus = new DrawCorpus(aRoot);
            DrawTestCase aCase = Assert.Single(aCorpus.Cases());
            var anInterpreter = new DrawInterpreter();
            aCorpus.Run(aCase, anInterpreter);
            Assert.True(anInterpreter.TryGetVariable("log", out string aLog));
            Assert.Equal("group-begin grid-begin group/grid/case1 case1 grid-end", aLog);
        }
        finally
        {
            Directory.Delete(aRoot, recursive: true);
        }
    }
}
