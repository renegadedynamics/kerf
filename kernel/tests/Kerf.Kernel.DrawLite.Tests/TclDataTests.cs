// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>Arrays and dicts.</summary>
public class TclDataTests
{
    [Fact]
    public void Arrays()
    {
        Assert.Equal("1 2", Tcl.Eval("array set a {x 1 y 2}; list $a(x) $a(y)"));
        Assert.Equal("x y", Tcl.Eval("array set a {x 1 y 2}; lsort [array names a]"));
        Assert.Equal("x1", Tcl.Eval("array set a {x1 1 y 2}; array names a x*"));
        Assert.Equal("1 0", Tcl.Eval("set a(1) v; list [array exists a] [array exists b]"));
        Assert.Equal("2", Tcl.Eval("set a(1) v; set a(2) w; array size a"));
        Assert.Equal("0", Tcl.Eval("array size nosuch"));
        Assert.Equal("1 v", Tcl.Eval("set a(1) v; array get a"));
        Assert.Equal("", Tcl.Eval("array get nosuch"));
        Assert.Equal("", Tcl.Eval("array get Draw_Groups \"TOPOLOGY Check commands\""));
        Assert.Equal("0", Tcl.Eval("set a(1) v; array unset a; array exists a"));
        Assert.Equal("b", Tcl.Eval("array set a {a 1 b 2}; array unset a a; array names a"));
        Assert.Equal("1", Tcl.Eval("array set e {}; array exists e"));
        Assert.Equal("x 1", Tcl.Eval("proc p {} {global g; set g(x) 1}; p; array get g"));
        Assert.Equal("{a b} 1", Tcl.Eval("set a(a\\ b) 1; array get a"));
        Assert.Equal("1", Tcl.Eval("set a(1) v; unset a(1); array size a; expr {[array size a] == 0}"));
    }

    [Fact]
    public void Env_and_tcl_platform_are_arrays()
    {
        Assert.Equal("1", Tcl.Eval("array exists env"));
        Assert.Contains(Tcl.Eval("set tcl_platform(platform)"), new[] { "windows", "unix" });
        Assert.Equal("1", Tcl.Eval("info exists ::tcl_platform(os)"));
    }

    [Fact]
    public void Dicts()
    {
        Assert.Equal("a 1 b 2", Tcl.Eval("dict create a 1 b 2"));
        Assert.Equal("2", Tcl.Eval("dict get {a 1 b 2} b"));
        Assert.Equal("x", Tcl.Eval("dict get {a {b x}} a b"));
        Assert.Equal("key \"z\" not known in dictionary", Tcl.Error("dict get {a 1} z"));
        Assert.Equal("1 0", Tcl.Eval("list [dict exists {a 1} a] [dict exists {a 1} z]"));
        Assert.Equal("a b", Tcl.Eval("dict keys {a 1 b 2}"));
        Assert.Equal("1 2", Tcl.Eval("dict values {a 1 b 2}"));
        Assert.Equal("2", Tcl.Eval("dict size {a 1 b 2}"));
        Assert.Equal("a 3 b 2", Tcl.Eval("set d {a 1 b 2}; dict set d a 3"));
        Assert.Equal("a {x 1}", Tcl.Eval("dict set d a x 1"));
        Assert.Equal("b 2", Tcl.Eval("set d {a 1 b 2}; dict unset d a"));
        Assert.Equal("a 15", Tcl.Eval("set d {a 10}; dict incr d a 5"));
        Assert.Equal("n 1", Tcl.Eval("dict incr d n"));
        Assert.Equal("a xyz", Tcl.Eval("set d {a x}; dict append d a y z"));
        Assert.Equal("a {x y}", Tcl.Eval("set d {a x}; dict lappend d a y"));
        Assert.Equal("a=1 b=2", Tcl.Eval("set r {}; dict for {k v} {a 1 b 2} {lappend r $k=$v}; join $r"));
        Assert.Equal("a 1 b 3 c 4", Tcl.Eval("dict merge {a 1 b 2} {b 3 c 4}"));
        Assert.Equal("b 2", Tcl.Eval("dict remove {a 1 b 2} a"));
        Assert.Equal("a 9 b 2", Tcl.Eval("dict replace {a 1 b 2} a 9"));
        Assert.Equal("a 1", Tcl.Eval("dict filter {a 1 b 2} key a*"));
        Assert.Equal("missing value to go with key", Tcl.Error("dict get {a} a"));
    }
}
