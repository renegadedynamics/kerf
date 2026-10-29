// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.DrawLite.Tests;

/// <summary>Control flow, procedures, frames (upvar, uplevel, global) and return codes.</summary>
public class TclControlTests
{
    [Theory]
    [InlineData("if {1} {set r a} else {set r b}", "a")]
    [InlineData("if {0} {set r a} else {set r b}", "b")]
    [InlineData("if 0 {set r a} elseif {2 > 1} then {set r c} else {set r b}", "c")]
    [InlineData("if {0} {set r a}", "")]
    [InlineData("if {\"abc\" == \"abc\"} {set r eq}", "eq")]
    [InlineData("if {yes} {set r y}", "y")]
    [InlineData("if {0} {set r a} {set r implicit-else}", "implicit-else")]
    public void If_chooses_a_branch(string theScript, string theExpected) => Assert.Equal(theExpected, Tcl.Eval(theScript));

    [Fact]
    public void If_rejects_a_non_boolean_condition()
        => Assert.Equal("expected boolean value but got \"maybe\"", Tcl.Error("if {\"maybe\"} {set r 1}"));

    [Fact]
    public void While_and_for_loop_with_break_and_continue()
    {
        Assert.Equal("0 1 2 4 5", Tcl.Eval("""
            set r {}
            for {set i 0} {$i < 10} {incr i} {
                if {$i == 3} continue
                if {$i > 5} break
                lappend r $i
            }
            set r
            """));
        Assert.Equal("10", Tcl.Eval("set i 0; while {$i < 10} {incr i}; set i"));
        Assert.Equal("3", Tcl.Eval("set i 0; while 1 {incr i; if {$i == 3} break}; set i"));
    }

    [Fact]
    public void Foreach_takes_several_variables_and_lists()
    {
        Assert.Equal("a1 b2 c3", Tcl.Eval("set r {}; foreach x {a b c} y {1 2 3} {lappend r $x$y}; join $r"));
        Assert.Equal("a-b c-d e-", Tcl.Eval("set r {}; foreach {x y} {a b c d e} {lappend r $x-$y}; join $r"));
        Assert.Equal("{a b} c", Tcl.Eval("set r {}; foreach x {{a b} c} {lappend r $x}; set r"));
        Assert.Equal("", Tcl.Eval("foreach x {} {error never}"));
    }

    [Fact]
    public void Break_inside_foreach_inside_a_proc_only_leaves_the_loop()
    {
        Assert.Equal("b|after", Tcl.Eval("""
            proc find {list} {
                set found ""
                foreach x $list {
                    if {$x eq "b"} { set found $x; break }
                }
                return "$found|after"
            }
            find {a b c}
            """));
    }

    [Fact]
    public void Lmap_collects_results() => Assert.Equal("2 4 6", Tcl.Eval("lmap x {1 2 3} {expr {$x * 2}}"));

    [Fact]
    public void Break_outside_a_loop_is_an_error()
    {
        Assert.Equal("invoked \"break\" outside of a loop", Tcl.Error("proc p {} {break}; p"));
        Assert.Equal("invoked \"break\" outside of a loop", Tcl.Error("break"));
    }

    [Fact]
    public void Procs_have_their_own_variables_and_defaults_and_args()
    {
        Assert.Equal("1 2 {}", Tcl.Eval("proc p {a {b 2} args} {list $a $b $args}; p 1"));
        Assert.Equal("1 5 {6 7}", Tcl.Eval("proc p {a {b 2} args} {list $a $b $args}; p 1 5 6 7"));
        Assert.Equal("outer", Tcl.Eval("set v outer; proc p {} {set v inner}; p; set v"));
        Assert.Equal("can't read \"v\": no such variable", Tcl.Error("set v outer; proc p {} {set v}; p"));
        Assert.Equal("wrong # args: should be \"p a ?b? ?arg ...?\"", Tcl.Error("proc p {a {b 2} args} {}; p"));
        Assert.Equal("wrong # args: should be \"p a\"", Tcl.Error("proc p {a} {}; p 1 2"));
        Assert.Equal("", Tcl.Eval("proc p {} {}; p"));
        Assert.Equal("last", Tcl.Eval("proc p {} {set x first; set x last}; p"));
    }

    [Fact]
    public void Recursion_works() => Assert.Equal("120", Tcl.Eval("proc f {n} {if {$n <= 1} {return 1}; expr {$n * [f [expr {$n - 1}]]}}; f 5"));

    [Fact]
    public void Runaway_recursion_is_an_error_not_a_crash()
        => Assert.Contains("too many nested evaluations", Tcl.Error("proc f {} {f}; f"));

    [Fact]
    public void Global_links_to_global_variables()
    {
        Assert.Equal("2", Tcl.Eval("set g 1; proc p {} {global g; incr g}; p; set g"));
        Assert.Equal("new", Tcl.Eval("proc p {} {global h; set h new}; p; set h"));
        Assert.Equal("7", Tcl.Eval("set g 7; proc p {} {set ::g}; p"));
        Assert.Equal("8", Tcl.Eval("proc p {} {set ::g 8}; p; set g"));
    }

    [Fact]
    public void Upvar_links_to_the_callers_variables()
    {
        Assert.Equal("11", Tcl.Eval("proc inc {name} {upvar $name v; incr v}; set x 10; inc x; set x"));
        Assert.Equal("set-by-callee", Tcl.Eval("proc p {name} {upvar 1 $name v; set v set-by-callee}; p fresh; set fresh"));
        Assert.Equal("1 0", Tcl.Eval("proc p {} {upvar nosuch v; info exists v}; list [proc q {} {upvar x x; info exists x}; set x 1; q] [p]"));
        Assert.Equal("deep", Tcl.Eval("""
            proc a {} { set local 1; b; return $local }
            proc b {} { upvar 1 local l; set l deep; c }
            proc c {} { upvar #0 top t; set t [uplevel 2 {set local}] }
            a
            set top
            """));
        // upvar of the same name, as CheckCommands.tcl does: upvar ${shape} ${shape}
        Assert.Equal("s1", Tcl.Eval("proc p {shape} {upvar ${shape} ${shape}; set ${shape}}; set res s1; p res"));
        Assert.Equal("arr", Tcl.Eval("proc p {n} {upvar $n a; set a(k) arr}; p z; set z(k)"));
    }

    [Fact]
    public void Uplevel_runs_in_the_callers_frame()
    {
        Assert.Equal("5", Tcl.Eval("proc setx {} {uplevel {set x 5}}; proc p {} {setx; return $x}; p"));
        Assert.Equal("glob", Tcl.Eval("proc p {} {uplevel #0 {set y glob}}; proc q {} {p}; q; set y"));
        Assert.Equal("3", Tcl.Eval("proc p {} {uplevel 1 set z 3}; p; set z"));
        Assert.Equal("1 2", Tcl.Eval("proc p {} {list [info level] [q]}; proc q {} {info level}; p"));
        Assert.Equal("p a b", Tcl.Eval("proc p {x y} {info level 0}; p a b"));
        Assert.Equal("bad level \"5\"", Tcl.Error("proc p {} {uplevel 5 {set x}}; p"));
    }

    [Fact]
    public void Return_codes_and_catch()
    {
        Assert.Equal("1 oops", Tcl.Eval("set c [catch {error oops} msg]; list $c $msg"));
        Assert.Equal("0 fine", Tcl.Eval("set c [catch {set x fine} msg]; list $c $msg"));
        Assert.Equal("2 val", Tcl.Eval("set c [catch {return val} msg]; list $c $msg"));
        Assert.Equal("3", Tcl.Eval("catch {break}"));
        Assert.Equal("4", Tcl.Eval("catch {continue}"));
        Assert.Equal("1 {invalid command name \"nosuchcommand\"}", Tcl.Eval("set c [catch {nosuchcommand 1 2} msg]; list $c $msg"));
        Assert.Equal("1 failed", Tcl.Eval("proc p {} {return -code error failed}; set c [catch {p} msg]; list $c $msg"));
        Assert.Equal("1 {can't read \"nope\": no such variable}", Tcl.Eval("set c [catch {set nope} msg]; list $c $msg"));
        Assert.Equal("MYCODE", Tcl.Eval("catch {error msg info MYCODE}; set errorCode"));
        Assert.Equal("1", Tcl.Eval("catch {error bad} m opts; dict get $opts -code"));
        Assert.Equal("1", Tcl.Eval("proc p {} {return -code break}; set r {}; foreach x {1 2} {lappend r $x; p; lappend r never}; set r"));
        Assert.Equal("early", Tcl.Eval("proc p {} {foreach x {1 2} {return early}; return late}; p"));
        Assert.Equal("v", Tcl.Eval("return -level 0 v"));
    }

    [Fact]
    public void A_return_at_the_top_level_ends_the_script()
        => Assert.Equal("done", Tcl.Eval("set a 1\nreturn done\nerror never"));

    [Fact]
    public void Exit_stops_the_script()
    {
        var anInterpreter = new DrawInterpreter();
        Assert.Equal("", anInterpreter.Eval("set a 1\nexit 3\nset a 2"));
        Assert.True(anInterpreter.HasExited);
        Assert.Equal(3, anInterpreter.ExitCode);
        Assert.True(anInterpreter.TryGetVariable("a", out string aValue) && aValue == "1");
    }

    [Fact]
    public void Error_inside_a_proc_unwinds_to_catch()
        => Assert.Equal("1 inner 1", Tcl.Eval("proc a {} {b}; proc b {} {error inner}; set c [catch a m]; list $c $m [info exists m]"));

    [Fact]
    public void Try_runs_handlers_and_finally()
    {
        Assert.Equal("caught boom|fin", Tcl.Eval("set f {}; set r [try {error boom} on error {m} {set x \"caught $m\"} finally {set f fin}]; return \"$r|$f\""));
        Assert.Equal("ok", Tcl.Eval("try {set x ok} on error {m} {set x bad}"));
        Assert.Equal("trapped", Tcl.Eval("try {throw {MY ERR} msg} trap {MY} {} {set x trapped}"));
    }

    [Fact]
    public void Eval_concatenates_its_arguments()
    {
        Assert.Equal("3", Tcl.Eval("set cmd {expr 1 +}; eval $cmd 2"));
        Assert.Equal("a b c", Tcl.Eval("set l {a b}; eval lappend l c"));
        Assert.Equal("x y", Tcl.Eval("set args {x y}; eval list $args"));
    }

    [Fact]
    public void Switch_matches_exactly_by_glob_or_by_regexp()
    {
        Assert.Equal("two", Tcl.Eval("switch b {a {set r one} b {set r two} default {set r other}}"));
        Assert.Equal("other", Tcl.Eval("switch z {a {set r one} default {set r other}}"));
        Assert.Equal("fall", Tcl.Eval("switch b {a - b {set r fall} c {set r no}}"));
        Assert.Equal("glob", Tcl.Eval("switch -glob abc {a* {set r glob} default {set r no}}"));
        Assert.Equal("re", Tcl.Eval("switch -regexp -- x42 {{^x[0-9]+$} {set r re}}"));
        Assert.Equal("", Tcl.Eval("switch q {a {set r one}}"));
        Assert.Equal("args", Tcl.Eval("switch -exact b a {set r one} b {set r args}"));
        Assert.Equal("0", Tcl.Eval("switch -- 1 {0 {format 1} 1 {format 0}}"));
    }

    [Fact]
    public void Rename_moves_and_deletes_commands()
    {
        Assert.Equal("hi", Tcl.Eval("proc p {} {return hi}; rename p q; q"));
        Assert.Equal("invalid command name \"p\"", Tcl.Error("proc p {} {}; rename p q; p"));
        Assert.Equal("0", Tcl.Eval("proc p {} {}; rename p {}; llength [info commands p]"));
        Assert.Equal("wrapped:x", Tcl.Eval("""
            rename puts puts-saved
            proc puts {args} { global log; append log "wrapped:[lindex $args end]" }
            puts x
            rename puts {}
            rename puts-saved puts
            set log
            """));
    }

    [Fact]
    public void Info_reports_procs_variables_and_levels()
    {
        Assert.Equal("a {b 2} args", Tcl.Eval("proc p {a {b 2} args} {return x}; set r {}; foreach x [info args p] {if {[info default p $x d]} {lappend r [list $x $d]} else {lappend r $x}}; set r"));
        Assert.Equal("return x", Tcl.Eval("proc p {a} {return x}; info body p"));
        Assert.Equal("p", Tcl.Eval("proc p {} {}; info procs p"));
        Assert.Equal("set", Tcl.Eval("info commands set"));
        Assert.Equal("", Tcl.Eval("info commands nosuch"));
        Assert.Equal("0", Tcl.Eval("info level"));
        Assert.Equal("1 0", Tcl.Eval("set a 1; list [info exists a] [info exists b]"));
        Assert.Equal("1 0", Tcl.Eval("set a(x) 1; list [info exists a(x)] [info exists a(y)]"));
        Assert.Equal("x y", Tcl.Eval("proc p {x} {set y 1; lsort [info vars]}; p 0"));
        Assert.Equal("1", Tcl.Eval("set zz 1; expr {[lsearch [info globals] zz] >= 0}"));
        Assert.Equal("8.6", Tcl.Eval("info tclversion"));
        Assert.Equal("1 0", Tcl.Eval("""list [info complete {set a {b}}] [info complete "set a \{b"]"""));
    }

    [Fact]
    public void Time_measures_a_script()
    {
        Assert.Matches(@"^[0-9.e+-]+ microseconds per iteration$", Tcl.Eval("time {set x 1} 10"));

        // one iteration: an integer, as Tcl's (expr then divides it as an integer)
        Assert.Matches(@"^[0-9]+ microseconds per iteration$", Tcl.Eval("time {set x 1}"));
        Assert.Equal("0", Tcl.Eval("expr [lindex [time {set x 1}] 0]/1000000"));
    }

    [Fact]
    public void Variables_scalar_array_unset_incr_append()
    {
        Assert.Equal("5", Tcl.Eval("incr n 5"));
        Assert.Equal("3", Tcl.Eval("set n 5; incr n -2"));
        Assert.Equal("expected integer but got \"1.5\"", Tcl.Error("set n 1.5; incr n"));
        Assert.Equal("abc", Tcl.Eval("append s a b c"));
        Assert.Equal("0", Tcl.Eval("set a 1; unset a; info exists a"));
        Assert.Equal("can't unset \"a\": no such variable", Tcl.Error("unset a"));
        Assert.Equal("", Tcl.Eval("unset -nocomplain a b"));
        Assert.Equal("can't read \"arr\": variable is array", Tcl.Error("set arr(1) x; set arr"));
        Assert.Equal("can't set \"s(1)\": variable isn't array", Tcl.Error("set s 1; set s(1) x"));
        Assert.Equal("can't read \"arr(2)\": no such element in array", Tcl.Error("set arr(1) x; set arr(2)"));
        Assert.Equal("v", Tcl.Eval("set i 3; set arr(3) v; set arr($i)"));
        Assert.Equal("w", Tcl.Eval("set arr(a,b) w; set i a; set j b; set arr($i,$j)"));
        Assert.Equal("q", Tcl.Eval("set arr(k1) q; set arr([string trim { k1 }])"));
        Assert.Equal("y", Tcl.Eval("set {a b} y; set {a b}"));
        Assert.Equal("y", Tcl.Eval("set {a b} y; set x ${a b}"));
    }

    [Fact]
    public void Expansion_splices_a_list_into_the_command()
    {
        Assert.Equal("3", Tcl.Eval("set l {a b c}; llength [list {*}$l]"));
        Assert.Equal("1 2 3 4", Tcl.Eval("list {*}{1 2} {*}[list 3 4]"));
        Assert.Equal("0", Tcl.Eval("llength [list {*}{}]"));
    }

    [Fact]
    public void Namespace_qualified_names_are_global()
    {
        Assert.Equal("1", Tcl.Eval("proc ::p {} {return 1}; p"));
        Assert.Equal("1", Tcl.Eval("proc p {} {return 1}; ::p"));
        Assert.Equal("::", Tcl.Eval("namespace current"));
        Assert.Equal("2", Tcl.Eval("proc p {} {variable vv 2; return $vv}; p; set vv"));
    }
}
