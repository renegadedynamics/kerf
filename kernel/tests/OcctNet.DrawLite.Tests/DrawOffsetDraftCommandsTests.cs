// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.DrawLite;

namespace OcctNet.DrawLite.Tests;

/// <summary>
/// Unit 7G's Draw commands: the 3D offset commands (thickshell, mkoffsetshape, offsetshape, offsetcompshape,
/// offsetparameter, offsetload, offsetonface, offsetperform, offsetshapesimple), the draft commands (depouille,
/// ndepouille, draft), the rolling ball and nproject, and the Tcl parser's backslash-newline between words.
/// </summary>
public class DrawOffsetDraftCommandsTests
{
    private static DrawInterpreter NewInterpreter()
    {
        var anInterpreter = new DrawInterpreter();
        anInterpreter.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        return anInterpreter;
    }

    private static double Volume(DrawInterpreter theDI, string theName)
    {
        string aText = theDI.Eval($"vprops {theName}");
        System.Text.RegularExpressions.Match aMatch = System.Text.RegularExpressions.Regex.Match(aText, @"Mass :\s+([-0-9.eE+]+)");
        Assert.True(aMatch.Success, aText);
        return double.Parse(aMatch.Groups[1].Value, System.Globalization.CultureInfo.InvariantCulture);
    }

    [Fact]
    public void Offsetparameter_keeps_its_values_per_interpreter()
    {
        DrawInterpreter aDI = NewInterpreter();
        string aDefault = aDI.Eval("offsetparameter");
        Assert.Contains("--> TheInter  : Partial", aDefault);
        Assert.Contains("--> TheJoin   : Arc", aDefault);
        Assert.Contains("--> Internal Edges : Keep", aDefault);
        aDI.Eval("offsetparameter 1e-7 c i r");
        string aSet = aDI.Eval("offsetparameter");
        Assert.Contains("Complete", aSet);
        Assert.Contains("Intersection", aSet);
        Assert.Contains("Remove", aSet);
        // another interpreter starts from OCCT's defaults
        Assert.Contains("Partial", NewInterpreter().Eval("offsetparameter"));
    }

    [Fact]
    public void Offsetload_and_offsetperform_offset_and_hollow_a_box()
    {
        DrawInterpreter aDI = NewInterpreter();
        aDI.Eval("box b 10 10 10; explode b f");
        aDI.Eval("offsetparameter 1e-7 c i");
        aDI.Eval("offsetload b 1");
        Assert.Equal("", aDI.Eval("offsetperform r"));
        Assert.Equal(12.0 * 12.0 * 12.0, Volume(aDI, "r"), 1.0e-6);
        // a closing face: a thick solid
        aDI.Eval("offsetload b -1 b_1");
        aDI.Eval("offsetperform h");
        Assert.Equal(1000.0 - 8.0 * 9.0 * 8.0, Volume(aDI, "h"), 1.0e-6);
        // the history of offsetperform
        Assert.Contains("1", aDI.Eval("nbshapes h"));
        // offsetshape reports the state
        Assert.Contains("OK. Offset performed successfully.", aDI.Eval("offsetshape s b 1"));
        Assert.Contains("OK", aDI.Eval("thickshell t b_1 1"));
        Assert.Equal("", aDI.Eval("mkoffsetshape m b 1 1e-7 1 0 i"));
        Assert.Equal(12.0 * 12.0 * 12.0, Volume(aDI, "m"), 1.0e-6);
        Assert.Equal("", aDI.Eval("offsetshapesimple o b_1 1 solid"));
        Assert.Contains("Null offset value", aDI.Eval("offsetshapesimple o b_1 0"));
    }

    [Fact]
    public void Depouille_drafts_a_box_side_and_reports_bad_faces()
    {
        DrawInterpreter aDI = NewInterpreter();
        aDI.Eval("box b 10 10 10; explode b f");
        Assert.Equal("", aDI.Eval("depouille r b 0 0 1 b_2 5 0 0 0 0 0 1"));
        double aDelta = 0.5 * 10.0 * 10.0 * Math.Tan(5.0 * Math.PI / 180.0) * 10.0;
        Assert.Equal(aDelta, Math.Abs(Volume(aDI, "r") - 1000.0), 1.0e-6);
        Assert.Equal("", aDI.Eval("ndepouille n b 0 0 1 b_2 1 5 0 0 0 0 0 1"));
        // a sphere can't be drafted: bugdep holds it
        aDI.Eval("psphere s 5; explode s f");
        Assert.ThrowsAny<Exception>(() => aDI.Eval("depouille r s 0 0 1 s_1 5 0 0 0 0 0 1"));
        Assert.Contains("FACE", aDI.Eval("whatis bugdep"));
    }

    [Fact]
    public void Draft_builds_a_skin_from_a_wire()
    {
        DrawInterpreter aDI = NewInterpreter();
        aDI.Eval("polyline w 0 0 0 10 0 0 10 10 0 0 10 0 0 0 0");
        Assert.Contains("External Draft", aDI.Eval("draft r w 0 0 1 5 8"));
        Assert.Contains("4", aDI.Eval("nbshapes DraftShell"));
        Assert.Contains("Internal Draft", aDI.Eval("draft r w 0 0 1 5 8 -IN -Ri -Internal"));
    }

    [Fact]
    public void Rollingball_and_nproject_run()
    {
        DrawInterpreter aDI = NewInterpreter();
        aDI.Eval("box b 10 10 10; explode b f");
        Assert.Equal("", aDI.Eval("rollingball r b -2 @ b_1 b_3"));
        Assert.Equal("", aDI.Eval("brollingball r b -2 @ b_1 b_3"));
        Assert.Contains(" Indexes of the 1th Branch", aDI.Eval("trollingball t b -2 @ b_1 b_3"));
        Assert.ThrowsAny<Exception>(() => aDI.Eval("rollingball r b 2 @ b_1"));

        aDI.Eval("circle c 5 5 13 0 0 1 3; mkedge e c");
        Assert.Equal("", aDI.Eval("nproject p e b"));
        Assert.Contains("EDGE", aDI.Eval("nbshapes p"));
    }

    [Fact]
    public void A_backslash_newline_ends_a_bare_word()
    {
        DrawInterpreter aDI = NewInterpreter();
        Assert.Equal("3", aDI.Eval("llength [list a b\\\n   c]"));
        // within quotes it is a space of the word
        Assert.Equal("3", aDI.Eval("string length \"a\\\n   b\""));
    }
}
