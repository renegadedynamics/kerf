// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Kernel.DrawLite;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>file, open/gets/puts/close, source, clock and puts.</summary>
public class TclSystemTests
{
    [Theory]
    [InlineData("file join a b c", "a/b/c")]
    [InlineData("file join a /b c", "/b/c")]
    [InlineData("file join a/ b", "a/b")]
    [InlineData("file join {} a", "a")]
    [InlineData("file tail /x/y.brep", "y.brep")]
    [InlineData("file tail y.brep", "y.brep")]
    [InlineData("file tail /x/y/", "y")]
    [InlineData("file dirname /x/y", "/x")]
    [InlineData("file dirname y", ".")]
    [InlineData("file dirname /y", "/")]
    [InlineData("file dirname a/b/c", "a/b")]
    [InlineData("file extension a/b.c.brep", ".brep")]
    [InlineData("file extension a/b", "")]
    [InlineData("file rootname a/b.brep", "a/b")]
    [InlineData("file rootname [file tail /d/bug123.stp]", "bug123")]
    [InlineData("file split /a/b", "/ a b")]
    [InlineData("file split a/b", "a b")]
    public void File_name_operations(string theScript, string theExpected) => Assert.Equal(theExpected, Tcl.Eval(theScript));

    [Fact]
    public void File_queries_read_the_disk()
    {
        string aPath = Path.GetTempFileName();
        try
        {
            File.WriteAllText(aPath, "line1\nline2\n");
            string aTcl = aPath.Replace('\\', '/');
            Assert.Equal("1 1 0 12", Tcl.Eval($"list [file exists {{{aTcl}}}] [file isfile {{{aTcl}}}] [file isdirectory {{{aTcl}}}] [file size {{{aTcl}}}]"));
            Assert.Equal("1", Tcl.Eval($"file isdirectory [file dirname {{{aTcl}}}]"));
            Assert.Equal("line1|line2|", Tcl.Eval($$"""
                set f [open {{{aTcl}}}]
                set r {}
                while {[gets $f line] >= 0} { append r $line| }
                close $f
                set r
                """));
            Assert.Equal("line1\nline2", Tcl.Eval($"set f [open {{{aTcl}}} r]; set t [read -nonewline $f]; close $f; set t"));
            Assert.Equal(aTcl.ToLowerInvariant(), Tcl.Eval($"file normalize {{{aTcl}}}").ToLowerInvariant());
        }
        finally
        {
            File.Delete(aPath);
        }
    }

    [Fact]
    public void Writes_go_to_memory_and_never_to_disk()
    {
        string aPath = Path.Combine(Path.GetTempPath(), $"drawlite-{Guid.NewGuid():N}", "out.txt").Replace('\\', '/');
        var anInterpreter = new DrawInterpreter();
        Assert.Equal("hello\nworld|1|1", anInterpreter.Eval($$"""
            file mkdir [file dirname {{{aPath}}}]
            set f [open {{{aPath}}} w]
            puts $f hello
            puts -nonewline $f world
            close $f
            set f [open {{{aPath}}}]
            set t [read $f]
            close $f
            return "$t|[file exists {{{aPath}}}]|[file isdirectory [file dirname {{{aPath}}}]]"
            """));
        Assert.False(File.Exists(aPath));
        Assert.False(Directory.Exists(Path.GetDirectoryName(aPath)));
        Assert.Equal("0", anInterpreter.Eval($"file delete {{{aPath}}}; file exists {{{aPath}}}"));
    }

    [Fact]
    public void Opening_a_missing_file_is_an_error()
        => Assert.StartsWith("couldn't open \"/no/such/file.txt\"", Tcl.Error("open /no/such/file.txt"));

    [Fact]
    public void Source_runs_a_file_and_return_ends_it()
    {
        string aPath = Path.GetTempFileName();
        try
        {
            File.WriteAllText(aPath, "set a 1\r\nproc twice {x} {expr {2 * $x}}\r\nreturn [info script]\r\nset a 2\r\n");
            string aTcl = aPath.Replace('\\', '/');
            var anInterpreter = new DrawInterpreter();
            Assert.Equal(aTcl, anInterpreter.Eval($"source {{{aTcl}}}"));
            Assert.Equal("1 6 {}", anInterpreter.Eval("list $a [twice 3] [info script]"));
        }
        finally
        {
            File.Delete(aPath);
        }
    }

    [Fact]
    public void Glob_lists_files()
    {
        string aDirectory = Directory.CreateTempSubdirectory("drawlite").FullName;
        try
        {
            File.WriteAllText(Path.Combine(aDirectory, "a.brep"), "");
            File.WriteAllText(Path.Combine(aDirectory, "b.txt"), "");
            Directory.CreateDirectory(Path.Combine(aDirectory, "sub"));
            string aTcl = aDirectory.Replace('\\', '/');
            Assert.Equal("a.brep", Tcl.Eval($"glob -tails -directory {{{aTcl}}} *.brep"));
            Assert.Equal("sub", Tcl.Eval($"file tail [glob -directory {{{aTcl}}} -type d *]"));
            Assert.Equal("", Tcl.Eval($"glob -nocomplain -directory {{{aTcl}}} *.stp"));
            Assert.StartsWith("no files matched", Tcl.Error($"glob -directory {{{aTcl}}} *.stp"));
        }
        finally
        {
            Directory.Delete(aDirectory, recursive: true);
        }
    }

    [Fact]
    public void Clock_gives_times()
    {
        long aSeconds = long.Parse(Tcl.Eval("clock seconds"), System.Globalization.CultureInfo.InvariantCulture);
        Assert.InRange(aSeconds, DateTimeOffset.UtcNow.ToUnixTimeSeconds() - 5, DateTimeOffset.UtcNow.ToUnixTimeSeconds() + 5);
        Assert.Equal("1", Tcl.Eval("expr {[clock milliseconds] > [clock seconds]}"));
        Assert.Equal("1", Tcl.Eval("set a [clock clicks]; set b [clock clicks]; expr {$b >= $a}"));
        Assert.Equal("1970-01-01T00h00m00s", Tcl.Eval("clock format 0 -format {%Y-%m-%dT%Hh%Mm%Ss} -gmt 1"));
    }

    [Fact]
    public void Puts_writes_to_the_output()
    {
        Assert.Equal("a\nbc\nd\n", Tcl.Output("puts a; puts -nonewline b; puts c; puts stdout d"));
        Assert.Equal("err\n", Tcl.Output("puts stderr err"));
        Assert.Equal("x\n", Tcl.Output("dputs -red -intense x"));
    }
}
