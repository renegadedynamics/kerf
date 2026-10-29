// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.DrawLite.Tests;

/// <summary>string, format, scan, regexp, regsub and subst.</summary>
public class TclStringTests
{
    [Theory]
    [InlineData("string length abc", "3")]
    [InlineData("string length {}", "0")]
    [InlineData("string equal abc abc", "1")]
    [InlineData("string equal abc abd", "0")]
    [InlineData("string equal -nocase ABC abc", "1")]
    [InlineData("string equal -length 2 abx aby", "1")]
    [InlineData("string compare a b", "-1")]
    [InlineData("string compare b a", "1")]
    [InlineData("string compare a a", "0")]
    [InlineData("string compare -nocase A a", "0")]
    [InlineData("string first b abcb", "1")]
    [InlineData("string first b abcb 2", "3")]
    [InlineData("string first z abc", "-1")]
    [InlineData("string last b abcb", "3")]
    [InlineData("string last b abcb 2", "1")]
    [InlineData("string index abc 1", "b")]
    [InlineData("string index abc end", "c")]
    [InlineData("string index abc end-1", "b")]
    [InlineData("string index abc 5", "")]
    [InlineData("string range abcdef 1 3", "bcd")]
    [InlineData("string range abcdef 2 end", "cdef")]
    [InlineData("string range abc 2 1", "")]
    [InlineData("string range abc -5 0", "a")]
    [InlineData("string range abc 0 end-1", "ab")]
    [InlineData("string tolower ABC", "abc")]
    [InlineData("string toupper abc", "ABC")]
    [InlineData("string toupper abc 1 1", "aBc")]
    [InlineData("string totitle hELLO", "Hello")]
    [InlineData("string trim {  a b  }", "a b")]
    [InlineData("string trim \"\\n x \\t\"", "x")]
    [InlineData("string trimleft xxaxx x", "axx")]
    [InlineData("string trimright xxaxx x", "xxa")]
    [InlineData("string trim 007 0", "7")]
    [InlineData("string repeat ab 3", "ababab")]
    [InlineData("string replace abcdef 1 2 XY", "aXYdef")]
    [InlineData("string replace abc 1 1", "ac")]
    [InlineData("string replace abc 5 6 x", "abc")]
    [InlineData("string cat a b c", "abc")]
    [InlineData("string reverse abc", "cba")]
    [InlineData("string map {a 1 b 2} abcab", "12c12")]
    [InlineData("string map {abc X a Y} abca", "XY")]
    [InlineData("string map -nocase {A x} aA", "xx")]
    [InlineData("string map {\\\\ /} {a\\b}", "a/b")]
    public void String_subcommands(string theScript, string theExpected) => Assert.Equal(theExpected, Tcl.Eval(theScript));

    [Theory]
    [InlineData("a*", "abc", "1")]
    [InlineData("*c", "abc", "1")]
    [InlineData("a?c", "abc", "1")]
    [InlineData("a?c", "ac", "0")]
    [InlineData("[a-c]?", "bz", "1")]
    [InlineData("[xyz]", "b", "0")]
    [InlineData("a\\*", "a*", "1")]
    [InlineData("a\\*", "ab", "0")]
    [InlineData("*", "", "1")]
    [InlineData("??", "a", "0")]
    [InlineData("*.brep", "shape.brep", "1")]
    [InlineData("*/*", "dir/file", "1")]
    public void String_match_globs(string thePattern, string theText, string theExpected)
        => Assert.Equal(theExpected, Tcl.Eval($"string match {{{thePattern}}} {{{theText}}}"));

    [Theory]
    [InlineData("integer 42", "1")]
    [InlineData("integer { 42 }", "1")]
    [InlineData("integer -7", "1")]
    [InlineData("integer 0x1A", "1")]
    [InlineData("integer 4.2", "0")]
    [InlineData("integer {}", "1")]
    [InlineData("integer -strict {}", "0")]
    [InlineData("integer 99999999999", "0")]
    [InlineData("wide 99999999999", "1")]
    [InlineData("double 1e5", "1")]
    [InlineData("double 42", "1")]
    [InlineData("double abc", "0")]
    [InlineData("double -strict {}", "0")]
    [InlineData("digit 123", "1")]
    [InlineData("digit 12a", "0")]
    [InlineData("alpha abc", "1")]
    [InlineData("alnum ab1", "1")]
    [InlineData("boolean yes", "1")]
    [InlineData("boolean false", "1")]
    [InlineData("boolean 1", "1")]
    [InlineData("boolean 2", "0")]
    [InlineData("boolean 0.5", "0")]
    [InlineData("boolean maybe", "0")]
    [InlineData("true 1", "1")]
    [InlineData("true on", "1")]
    [InlineData("false yes", "0")]
    [InlineData("false off", "1")]
    [InlineData("space \" \\t\"", "1")]
    [InlineData("upper ABC", "1")]
    [InlineData("lower aBc", "0")]
    [InlineData("list {a {b}}", "1")]
    [InlineData("list \"a \\{\"", "0")]
    [InlineData("xdigit 0aF", "1")]
    [InlineData("wordchar a_1", "1")]
    [InlineData("wordchar a-1", "0")]
    public void String_is(string theArgs, string theExpected) => Assert.Equal(theExpected, Tcl.Eval($"string is {theArgs}"));

    [Fact]
    public void String_is_reports_where_it_failed() => Assert.Equal("0 2", Tcl.Eval("list [string is digit -failindex i 12a] $i"));

    [Theory]
    [InlineData("%d", "42", "42")]
    [InlineData("%5d", "42", "   42")]
    [InlineData("%-5d|", "42", "42   |")]
    [InlineData("%05d", "42", "00042")]
    [InlineData("%05d", "-42", "-0042")]
    [InlineData("%+d", "5", "+5")]
    [InlineData("% d", "5", " 5")]
    [InlineData("%i", "0x10", "16")]
    [InlineData("%x", "255", "ff")]
    [InlineData("%X", "255", "FF")]
    [InlineData("%#x", "255", "0xff")]
    [InlineData("%o", "8", "10")]
    [InlineData("%b", "5", "101")]
    [InlineData("%c", "65", "A")]
    [InlineData("%s", "abc", "abc")]
    [InlineData("%6s|", "abc", "   abc|")]
    [InlineData("%-6s|", "abc", "abc   |")]
    [InlineData("%.2s", "abc", "ab")]
    [InlineData("%f", "1.5", "1.500000")]
    [InlineData("%f", "2", "2.000000")]
    [InlineData("%.2f", "3.14159", "3.14")]
    [InlineData("%8.3f", "3.14159", "   3.142")]
    [InlineData("%-8.2f|", "-1.5", "-1.50   |")]
    [InlineData("%08.2f", "-1.5", "-0001.50")]
    [InlineData("%.0f", "1.75", "2")]
    [InlineData("%e", "12345.678", "1.234568e+04")]
    [InlineData("%E", "0.00012", "1.200000E-04")]
    [InlineData("%.3e", "0", "0.000e+00")]
    [InlineData("%g", "0.0001", "0.0001")]
    [InlineData("%g", "0.00001", "1e-05")]
    [InlineData("%g", "123456789", "1.23457e+08")]
    [InlineData("%g", "100000", "100000")]
    [InlineData("%g", "1000000", "1e+06")]
    [InlineData("%.3g", "3.14159", "3.14")]
    [InlineData("%g", "1.5", "1.5")]
    [InlineData("%g", "0", "0")]
    [InlineData("%#g", "1.5", "1.50000")]
    [InlineData("%.10g", "0.1", "0.1")]
    [InlineData("%.17g", "0.1", "0.10000000000000001")]
    [InlineData("%5.1f%%", "99.44", " 99.4%")]
    public void Format_conversions(string theFormat, string theValue, string theExpected)
        => Assert.Equal(theExpected, Tcl.Eval($"format {{{theFormat}}} {{{theValue}}}"));

    [Fact]
    public void Format_handles_positions_star_and_errors()
    {
        Assert.Equal("b a", Tcl.Eval("format {%2$s %1$s} a b"));
        Assert.Equal("   42", Tcl.Eval("format %*d 5 42"));
        Assert.Equal("x=1 y=2.50", Tcl.Eval("format \"x=%d y=%.2f\" 1 2.5"));
        Assert.Equal("100%", Tcl.Eval("format 100%%"));
        Assert.Equal("expected integer but got \"1.5\"", Tcl.Error("format %d 1.5"));
        Assert.Equal("not enough arguments for all format specifiers", Tcl.Error("format %s%s a"));
    }

    [Fact]
    public void Scan_converts_and_assigns()
    {
        Assert.Equal("12 abc 3.5", Tcl.Eval("scan {12 abc 3.5} {%d %s %f}"));
        Assert.Equal("2 12 34", Tcl.Eval("set n [scan {12 34} {%d %d} a b]; list $n $a $b"));
        Assert.Equal("{}", Tcl.Eval("scan x %d"));
        Assert.Equal("-1", Tcl.Eval("scan {} %d a"));
        Assert.Equal("0", Tcl.Eval("scan x %d a"));
        Assert.Equal("31", Tcl.Eval("scan 0x1f %x"));
        Assert.Equal("255", Tcl.Eval("scan ff %x"));
        Assert.Equal("abc 123", Tcl.Eval("scan abc123 {%[a-z]%d}"));
        Assert.Equal("97", Tcl.Eval("scan a %c"));
        Assert.Equal("1500.0", Tcl.Eval("scan 1.5e3 %g"));
        Assert.Equal("7 8", Tcl.Eval("scan {7,8} {%d,%d}"));
        Assert.Equal("12", Tcl.Eval("scan {   12xyz} %d"));
    }

    [Theory]
    [InlineData("regexp {^a(b+)c$} abbc m s; list $m $s", "abbc bb")]
    [InlineData("regexp {x} abc", "0")]
    [InlineData("regexp -nocase {ABC} xabcx", "1")]
    [InlineData("regexp -all {a} banana", "3")]
    [InlineData("regexp -inline {(\\d+)-(\\d+)} 10-20", "10-20 10 20")]
    [InlineData("regexp -all -inline {\\d+} a1b22c333", "1 22 333")]
    [InlineData("regexp -indices {b+} abbc m; set m", "1 2")]
    [InlineData("regexp {(a)|(b)} b m g1 g2; list $m $g1 $g2", "b {} b")]
    [InlineData("regexp {Mass +: +([-0-9.+eE]+)} \"Mass :  12.5\\n\" full m; set m", "12.5")]
    [InlineData("regexp {^-[^0-9]} -vertex", "1")]
    [InlineData("regexp {^-[^0-9]} -5", "0")]
    [InlineData("regexp {^[[:digit:]]+$} 123", "1")]
    [InlineData("regexp {[[:alpha:]]+} 12abc3 m; set m", "abc")]
    [InlineData("regexp {a$} \"a\\n\"", "0")]
    [InlineData("regexp {a.b} \"a\\nb\"", "1")]
    [InlineData("regexp -line {^b} \"a\\nb\"", "1")]
    [InlineData("regexp {^b} \"a\\nb\"", "0")]
    [InlineData("regexp {\\mfoo\\M} {a foo b}", "1")]
    [InlineData("regexp {\\mfoo\\M} afoob", "0")]
    [InlineData("regexp -start 2 {a} aaba", "1")]
    [InlineData("regexp -start 2 {a} aab", "0")]
    [InlineData("regexp {\\-[not]*equal} {-notequal a}", "1")]
    [InlineData("regexp -- {-x} a-x", "1")]
    [InlineData("regexp {(\\d+)\\s+(\\d+)} {nb 3 4} all a b; list $a $b", "3 4")]
    [InlineData("set m untouched; regexp {z} abc m; set m", "untouched")]
    [InlineData("regexp {VERTEX +: +([-0-9.+eE]+)} \"Number of shapes in s\\n VERTEX    : 8\\n EDGE      : 12\" full n; set n", "8")]
    public void Regexp(string theScript, string theExpected) => Assert.Equal(theExpected, Tcl.Eval(theScript));

    [Theory]
    [InlineData("regsub {a} banana A", "bAnana")]
    [InlineData("regsub -all {a} banana A", "bAnAnA")]
    [InlineData("regsub -all {(a)(n)} banana {\\2\\1}", "bnanaa")]
    [InlineData("regsub -all {n} banana {<&>}", "ba<n>a<n>a")]
    [InlineData("regsub -all {n} banana {\\&}", "ba&a&a")]
    [InlineData("regsub {x} abc y", "abc")]
    [InlineData("list [regsub -all a banana o res] $res", "3 bonono")]
    [InlineData("regsub -all {\\s+} {a  b   c} { }", "a b c")]
    [InlineData("regsub -all {x*} abc -", "-a-b-c-")]
    [InlineData("regsub -nocase {B} abc X", "aXc")]
    [InlineData("regsub {^} abc >", ">abc")]
    public void Regsub(string theScript, string theExpected) => Assert.Equal(theExpected, Tcl.Eval(theScript));

    [Fact]
    public void Subst_substitutes_variables_commands_and_backslashes()
    {
        Assert.Equal("a 1 2 \n", Tcl.Eval("set x 1; subst {a $x [set y 2] \\n}"));
        Assert.Equal("1 [set y]", Tcl.Eval("set x 1; subst -nocommands {$x [set y]}"));
        Assert.Equal("$x", Tcl.Eval("set x 1; subst -novariables {$x}"));
        Assert.Equal("a\\nb", Tcl.Eval("subst -nobackslashes {a\\nb}"));
    }

    [Fact]
    public void Backslash_escapes_in_words()
    {
        Assert.Equal("a\tb", Tcl.Eval("set x a\\tb"));
        Assert.Equal("A", Tcl.Eval("set x \\x41"));
        Assert.Equal("\u00e9", Tcl.Eval("set x \\u00e9"));
        Assert.Equal("A", Tcl.Eval("set x \\101"));
        Assert.Equal("[x]$", Tcl.Eval("set x \\[x\\]\\$"));
        Assert.Equal("a b", Tcl.Eval("set x \"a\\\n    b\""));
        Assert.Equal("a b", Tcl.Eval("set x {a\\\n    b}"));
    }
}
