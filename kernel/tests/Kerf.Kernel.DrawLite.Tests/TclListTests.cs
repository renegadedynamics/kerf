// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Kernel.DrawLite;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>Lists: Tcl's quoting rules, parsing, and the list commands.</summary>
public class TclListTests
{
    [Theory]
    [InlineData(new[] { "a", "b", "c" }, "a b c")]
    [InlineData(new[] { "a b", "c" }, "{a b} c")]
    [InlineData(new[] { "", "x" }, "{} x")]
    [InlineData(new[] { "a{" }, "a\\{")]
    [InlineData(new[] { "{a" }, "\\{a")]
    [InlineData(new[] { "a}" }, "a\\}")]
    [InlineData(new[] { "a\\" }, "a\\\\")]
    [InlineData(new[] { "a b\\" }, "a\\ b\\\\")]
    [InlineData(new[] { "$x" }, "{$x}")]
    [InlineData(new[] { "\"" }, "{\"}")]
    [InlineData(new[] { "#a", "b" }, "{#a} b")]
    [InlineData(new[] { "b", "#a" }, "b #a")]
    [InlineData(new[] { "a\nb" }, "{a\nb}")]
    [InlineData(new[] { "a {b c}" }, "{a {b c}}")]
    [InlineData(new[] { "[x]" }, "{[x]}")]
    [InlineData(new[] { ";" }, "{;}")]
    [InlineData(new[] { "{a}b" }, "{{a}b}")]
    [InlineData(new[] { "}{" }, "\\}\\{")]
    public void Elements_are_quoted_as_tcl_does(string[] theElements, string theExpected)
    {
        string aList = TclList.Format(theElements);
        Assert.Equal(theExpected, aList);
        Assert.Equal(theElements, TclList.Parse(aList));
    }

    [Theory]
    [InlineData("a b c", new[] { "a", "b", "c" })]
    [InlineData("  a\t b\n c  ", new[] { "a", "b", "c" })]
    [InlineData("{a b} c", new[] { "a b", "c" })]
    [InlineData("\"a b\" c", new[] { "a b", "c" })]
    [InlineData("a\\ b c", new[] { "a b", "c" })]
    [InlineData("{a {b c}} d", new[] { "a {b c}", "d" })]
    [InlineData("{} {}", new[] { "", "" })]
    [InlineData("\"a\\tb\"", new[] { "a\tb" })]
    [InlineData("{a\\}b}", new[] { "a\\}b" })]
    [InlineData("", new string[0])]
    public void Strings_parse_as_lists(string theList, string[] theExpected) => Assert.Equal(theExpected, TclList.Parse(theList));

    [Fact]
    public void Malformed_lists_are_errors()
    {
        Assert.Equal("unmatched open brace in list", Tcl.Error("llength \"a {b\""));
        Assert.Equal("list element in braces followed by \"b\" instead of space", Tcl.Error("llength {{a}b}"));
        Assert.Equal("unmatched open quote in list", Tcl.Error("llength {\"a b}"));
    }

    [Theory]
    [InlineData("list a {b c} {}", "a {b c} {}")]
    [InlineData("llength {a {b c} \"d e\"}", "3")]
    [InlineData("llength {}", "0")]
    [InlineData("lindex {a b c} 0", "a")]
    [InlineData("lindex {a b c} end", "c")]
    [InlineData("lindex {a b c} end-1", "b")]
    [InlineData("lindex {a b c} end-0", "c")]
    [InlineData("lindex {a b c} 0+1", "b")]
    [InlineData("lindex {a b c} 5", "")]
    [InlineData("lindex {a b c} -1", "")]
    [InlineData("lindex {a {b c} d} 1 0", "b")]
    [InlineData("lindex {a {b c} d} {1 1}", "c")]
    [InlineData("lindex {a b c}", "a b c")]
    [InlineData("lindex {{a b}}  0", "a b")]
    [InlineData("lrange {a b c d e} 1 end-1", "b c d")]
    [InlineData("lrange {a b c} 0 end", "a b c")]
    [InlineData("lrange {a b} 5 6", "")]
    [InlineData("lrange {a {b c} d} 1 1", "{b c}")]
    [InlineData("linsert {a b c} 1 x y", "a x y b c")]
    [InlineData("linsert {a b c} end x", "a b c x")]
    [InlineData("linsert {a b c} end-1 x", "a b x c")]
    [InlineData("lreplace {a b c d} 1 2 x", "a x d")]
    [InlineData("lreplace {a b c} 0 0", "b c")]
    [InlineData("lreplace {a b c} end end", "a b")]
    [InlineData("lreplace {a b c} 1 0 x", "a x b c")]
    [InlineData("lreverse {1 2 {3 4}}", "{3 4} 2 1")]
    [InlineData("lrepeat 2 a b", "a b a b")]
    [InlineData("concat \"a b\" {c d} \" e \"", "a b c d e")]
    [InlineData("concat", "")]
    [InlineData("concat {a b} {} c", "a b c")]
    [InlineData("join {a b c} ,", "a,b,c")]
    [InlineData("join {{a b} c}", "a b c")]
    [InlineData("split a,b,,c ,", "a b {} c")]
    [InlineData("split abc {}", "a b c")]
    [InlineData("split {a b  c}", "a b {} c")]
    [InlineData("split {} ,", "")]
    [InlineData("split \"a\\nb\" \\n", "a b")]
    public void List_commands(string theScript, string theExpected) => Assert.Equal(theExpected, Tcl.Eval(theScript));

    [Theory]
    [InlineData("lsearch {a b c} b", "1")]
    [InlineData("lsearch {a b c} z", "-1")]
    [InlineData("lsearch {abc def} d*", "1")]
    [InlineData("lsearch -exact {a* b} a*", "0")]
    [InlineData("lsearch -exact {ab a*} a*", "1")]
    [InlineData("lsearch -all {a b a} a", "0 2")]
    [InlineData("lsearch -inline {apple banana} b*", "banana")]
    [InlineData("lsearch -all -inline -regexp {a1 b2 c3} {[ab]\\d}", "a1 b2")]
    [InlineData("lsearch -not {a a b} a", "2")]
    [InlineData("lsearch -nocase {A B} b", "1")]
    [InlineData("lsearch -integer {1 02 3} 2", "1")]
    [InlineData("lsearch -start 1 {a b a} a", "2")]
    [InlineData("lsearch -index 1 {{a 1} {b 2}} 2", "1")]
    public void Lsearch(string theScript, string theExpected) => Assert.Equal(theExpected, Tcl.Eval(theScript));

    [Theory]
    [InlineData("lsort {c a b}", "a b c")]
    [InlineData("lsort {10 9 100}", "10 100 9")]
    [InlineData("lsort -integer {10 9 100}", "9 10 100")]
    [InlineData("lsort -real {1.5 -2 1e1}", "-2 1.5 1e1")]
    [InlineData("lsort -decreasing {a c b}", "c b a")]
    [InlineData("lsort -unique {b a b c a}", "a b c")]
    [InlineData("lsort -dictionary {a10 a9 A1 b}", "A1 a9 a10 b")]
    [InlineData("lsort -dictionary -unique {2 1 2}", "1 2")]
    [InlineData("lsort -nocase {B a C}", "a B C")]
    [InlineData("lsort -index 1 {{a 2} {b 1}}", "{b 1} {a 2}")]
    [InlineData("lsort -index end -integer {{x 10} {y 9}}", "{y 9} {x 10}")]
    [InlineData("lsort -indices {c a b}", "1 2 0")]
    [InlineData("lsort -command {apply_cmp} {3 1 2}", "1 2 3")]
    public void Lsort(string theScript, string theExpected)
        => Assert.Equal(theExpected, Tcl.Eval("proc apply_cmp {a b} {expr {$a - $b}}\n" + theScript));

    [Fact]
    public void Lsort_rejects_non_numbers_for_numeric_sorts()
        => Assert.Equal("expected integer but got \"x\"", Tcl.Error("lsort -integer {1 x}"));

    [Fact]
    public void Lappend_lassign_and_lset_change_variables()
    {
        Assert.Equal("a {b c}", Tcl.Eval("lappend l a {b c}"));
        Assert.Equal("a {b c} d", Tcl.Eval("set l {a {b c}}; lappend l d; set l"));
        Assert.Equal("", Tcl.Eval("lappend fresh"));
        Assert.Equal("3|1|2", Tcl.Eval("set rest [lassign {1 2 3} a b]; return $rest|$a|$b"));
        Assert.Equal("|1|", Tcl.Eval("set rest [lassign {1} a b]; return $rest|$a|$b"));
        Assert.Equal("a x c", Tcl.Eval("set l {a b c}; lset l 1 x"));
        Assert.Equal("a {b y} c", Tcl.Eval("set l {a {b c} c}; lset l 1 1 y; set l"));
        Assert.Equal("new", Tcl.Eval("set l {a b}; lset l new"));
    }

    [Fact]
    public void A_list_built_in_a_loop_round_trips()
    {
        Assert.Equal("5 {a b} c\\{ {} {$x}", Tcl.Eval("""
            set l {}
            foreach e [list {a b} "c\{" {} {$x}] { lappend l $e }
            concat [llength [linsert $l 0 x]] $l
            """));
    }

    /// <summary>
    /// A list lappend extends in place where its result isn't used (decision 27803) reads as lappend's own: in a loop's
    /// body, after a command, in an if's branch, through upvar, read part way, set again, an element of an array.
    /// </summary>
    [Fact]
    public void A_list_extended_in_place_reads_as_lappend_made_it()
    {
        const string Elements = """[list {a b} "c\{" {} {$x} #h "{d}e"]""";
        string aMade = Tcl.Eval($"set r {{}}; foreach e {Elements} {{ set r [lappend r $e] }}; set r");
        Assert.Equal("""{a b} c\{ {} {$x} #h {{d}e}""", aMade);
        Assert.Equal(aMade, Tcl.Eval($"set l {{}}; foreach e {Elements} {{ lappend l $e }}; set l"));
        Assert.Equal(aMade, Tcl.Eval($"set l {{}}; foreach e {Elements} {{ lappend l $e; set y 1 }}; set l"));
        Assert.Equal(aMade, Tcl.Eval($"set l {{}}; foreach e {Elements} {{ if {{1}} {{ lappend l $e }} }}; set l"));
        Assert.Equal(aMade, Tcl.Eval($"proc p {{v}} {{ upvar $v w; foreach e {Elements} {{ lappend w $e }} }}; set l {{}}; p l; set l"));
        Assert.Equal("1 2 3 4 {1 2 3 4}", Tcl.Eval("set l {}; set n {}; foreach e {1 2 3 4} { lappend l $e; lappend n [llength $l] }; concat $n [list $l]"));
        Assert.Equal("x y", Tcl.Eval("set l {}; foreach e {1 2} { lappend l $e }; set l x; foreach e {y} { lappend l $e }; set l"));
        Assert.Equal("1 2", Tcl.Eval("foreach e {1 2} { lappend a(k) $e }; set a(k)"));
        Assert.Equal("{1 2} 3", Tcl.Eval("set l {}; foreach e {{1 2} 3} { lappend l $e }; set l"));
        Assert.Equal(2, int.Parse(Tcl.Eval("set l {}; foreach e {1 2} { lappend l $e }; llength $l"), System.Globalization.CultureInfo.InvariantCulture));
    }
}
