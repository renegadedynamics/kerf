// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Kernel.DrawLite;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// Unit 6L's Boolean commands on hand-made shapes: BOPTest's BOPCommands, PartitionCommands, APICommands,
/// <c>bsimplify</c>, UtilityCommands, TolerCommands and DebugCommands, with the session's history; and frozen inputs.
/// </summary>
public class DrawBopCommandsTests
{
    private static DrawInterpreter Load()
    {
        var anInterpreter = new DrawInterpreter(new StringWriter { NewLine = "\n" });
        anInterpreter.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        return anInterpreter;
    }

    private static double Volume(DrawInterpreter i, string theShape)
    {
        string aText = i.Eval($"vprops {theShape}");
        string aMass = aText.Split('\n').First(l => l.Contains("Mass", StringComparison.Ordinal));
        return double.Parse(aMass[(aMass.IndexOf(':') + 1)..].Trim(), System.Globalization.CultureInfo.InvariantCulture);
    }

    [Fact]
    public void Two_shape_Booleans_give_the_volumes_of_the_overlap()
    {
        DrawInterpreter i = Load();
        i.Eval("box b1 10 10 10; box b2 5 5 5 10 10 10");
        Assert.Equal("", i.Eval("bcommon r b1 b2"));
        Assert.Equal(125.0, Volume(i, "r"), 1e-6);
        i.Eval("bfuse r b1 b2");
        Assert.Equal(1875.0, Volume(i, "r"), 1e-6);
        i.Eval("bcut r b1 b2");
        Assert.Equal(875.0, Volume(i, "r"), 1e-6);
        i.Eval("btuc r b1 b2");
        Assert.Equal(875.0, Volume(i, "r"), 1e-6);
        Assert.Equal(" use bx r s1 s2\n", i.Eval("bcut r b1"));
        Assert.Equal(" null shapes are not allowed \n", i.Eval("bcut r b1 nothing"));
        // bsection: the section edges
        Assert.Equal("", i.Eval("bsection s b1 b2"));
        Assert.Equal("6", i.Eval("llength [explode s e]"));
        i.Eval("bsection s2 b1 b2 -n2d -na");
        Assert.Equal("use bsection r s1 s2 [-n2d/-n2d1/-n2d2] [-na] [tol]\n", i.Eval("bsection s b1"));
    }

    [Fact]
    public void Bop_prepares_the_filler_that_the_bop_commands_build_on()
    {
        DrawInterpreter i = Load();
        i.Eval("box b1 10 10 10; box b2 5 5 5 10 10 10");
        Assert.Equal(" prepare PaveFiller first\n", i.Eval("bopcommon r"));
        Assert.Equal("", i.Eval("bop b1 b2"));
        i.Eval("bopcommon r");
        Assert.Equal(125.0, Volume(i, "r"), 1e-6);
        i.Eval("bopfuse r");
        Assert.Equal(1875.0, Volume(i, "r"), 1e-6);
        i.Eval("bopcut r");
        Assert.Equal(875.0, Volume(i, "r"), 1e-6);
        i.Eval("boptuc r");
        Assert.Equal(875.0, Volume(i, "r"), 1e-6);
        i.Eval("bopsection r");
        Assert.Equal("6", i.Eval("llength [explode r e]"));
        Assert.Equal(" use bop s1 s2 \n", i.Eval("bop b1"));
    }

    [Fact]
    public void Partition_commands_build_on_bfillds()
    {
        DrawInterpreter i = Load();
        i.Eval("box b1 10 10 10; box b2 5 5 5 10 10 10");
        Assert.Equal("Prepare PaveFiller first\n", i.Eval("bclear; bbuild r"));
        i.Eval("baddobjects b1; baddtools b2; bfillds");
        i.Eval("bbuild gf");
        Assert.Equal("3", i.Eval("llength [explode gf so]"));
        i.Eval("bbop r 0");
        Assert.Equal(125.0, Volume(i, "r"), 1e-6);
        i.Eval("bbop r fuse");
        Assert.Equal(1875.0, Volume(i, "r"), 1e-6);
        i.Eval("bbop r cut21");
        Assert.Equal(875.0, Volume(i, "r"), 1e-6);
        i.Eval("bbop r section");
        Assert.Equal("6", i.Eval("llength [explode r e]"));
        Assert.Equal("Invalid operation type\n", i.Eval("bbop r 7"));
        Assert.Equal("Invalid operation type\n", i.Eval("bbop r junk"));
        i.Eval("bsplit sp");
        Assert.Equal("2", i.Eval("llength [explode sp so]"));
        // buildbop classifies the splits of the General Fuse
        i.Eval("bbuild gf");
        i.Eval("buildbop rb -o b1 -t b2 -op common");
        Assert.Equal(125.0, Volume(i, "rb"), 1e-6);
        Assert.Equal("Error: operation has not been specified", Assert.ThrowsAny<DrawException>(() => i.Eval("buildbop rb -o b1")).Reason);
        Assert.Equal("Error: junk invalid key", Assert.ThrowsAny<DrawException>(() => i.Eval("buildbop rb junk")).Reason);
    }

    [Fact]
    public void Api_commands_and_bsimplify()
    {
        DrawInterpreter i = Load();
        i.Eval("box b1 10 10 10; box b2 10 0 0 10 10 10");
        i.Eval("baddobjects b1; baddtools b2");
        i.Eval("bapibop r fuse");
        Assert.Equal("10", i.Eval("llength [explode r f]"));
        // with unification the shared side goes and the faces in a plane merge
        i.Eval("bsimplify -e 1 -f 1");
        Assert.Contains("Unify Faces: Yes", i.Eval("boptions"), StringComparison.Ordinal);
        i.Eval("bapibop r fuse");
        Assert.Equal("6", i.Eval("llength [explode r f]"));
        i.Eval("bapibop r common");
        i.Eval("bapibuild r");
        i.Eval("bapisplit r");
        Assert.ThrowsAny<DrawException>(() => i.Eval("bsimplify -e"));
        Assert.Equal("Wrong key option.\n", Assert.ThrowsAny<DrawException>(() => i.Eval("bsimplify -x 1")).Reason);
        Assert.Equal("Invalid operation type\n", i.Eval("bapibop r 5"));
    }

    [Fact]
    public void The_history_of_a_Boolean_answers_for_its_arguments()
    {
        DrawInterpreter i = Load();
        i.Eval("box b1 10 10 10; box b2 5 5 5 10 10 10; explode b1 f");
        i.Eval("setfillhistory 1");
        i.Eval("bfuse r b1 b2");
        i.Eval("savehistory h");
        Assert.Contains("Not deleted", i.Eval("isdeleted h b1_1"), StringComparison.Ordinal);
        // the top face of b1 (b1_6, z = 10) is split by b2
        Assert.NotEqual("The shape has not been modified", i.Eval("modified m h b1_6").Trim());
    }

    [Fact]
    public void Bopcurves_intersects_two_faces()
    {
        DrawInterpreter i = Load();
        i.Eval("plane p1 0 0 0 0 0 1; mkface f1 p1 -10 10 -10 10");
        i.Eval("plane p2 0 0 0 1 0 0; mkface f2 p2 -10 10 -10 10");
        string aText = i.Eval("bopcurves f1 f2 -2d");
        Assert.StartsWith("Tolerance Reached=", aText, StringComparison.Ordinal);
        Assert.Contains("1 curve(s) found.\nc_1 (c2d1_1, c2d2_1) \n", aText, StringComparison.Ordinal);
        Assert.Contains("Wrong key.", Assert.ThrowsAny<DrawException>(() => i.Eval("bopcurves f1 f2 -x")).Reason);
        i.Eval("plane p3 0 0 5 0 0 1; mkface f3 p3 -10 10 -10 10");
        Assert.Equal(" has no 3d curves\n has no 3d points\n", i.Eval("bopcurves f1 f3"));
    }

    [Fact]
    public void Utility_commands_make_faces_and_pcurves()
    {
        DrawInterpreter i = Load();
        i.Eval("polyline w 0 0 0 10 0 0 10 10 0 0 10 0 0 0 0");
        i.Eval("explode w e; compound w_1 w_2 w_3 w_4 ce");
        Assert.Equal("", i.Eval("edgestofaces f ce"));
        Assert.Equal("1", i.Eval("llength [explode f f]"));
        i.Eval("edgestowire ow ce");
        Assert.Equal("", i.Eval("buildpcurvesonplane f"));
        Assert.Equal("no edges\n", Assert.ThrowsAny<DrawException>(() => i.Eval("edgestofaces f nothing")).Reason);
        // attachpcurve: the face's own edge gives its pcurve to a copy
        i.Eval("explode f f; explode f_1 e; tcopy f_1_1 enew");
        Assert.Equal("PCurve is attached successfully\n", i.Eval("attachpcurve f_1_1 enew f_1"));
    }

    [Fact]
    public void Toler_commands_reduce_tolerances_in_place()
    {
        DrawInterpreter i = Load();
        i.Eval("box b 10 10 10; settolerance b 0.01");
        i.Eval("btolx b");
        Assert.Contains("Face   : Min ", i.Eval("maxtolerance b"), StringComparison.Ordinal);
        Assert.Equal("", i.Eval("breducetolerance b"));
        Assert.Equal("", i.Eval("bopaddpcs b"));
        Assert.Equal(" Null shape is not allowed\n", Assert.ThrowsAny<DrawException>(() => i.Eval("btolx nothing")).Reason);
    }

    [Fact]
    public void Debug_commands_read_the_data_structure()
    {
        DrawInterpreter i = Load();
        i.Eval("box b1 10 10 10; box b2 5 5 5 10 10 10");
        Assert.Equal(" prepare PaveFiller first\n", Assert.ThrowsAny<DrawException>(() => i.Eval("bclear; bopds")).Reason);
        i.Eval("baddobjects b1; baddtools b2; bfillds");
        i.Eval("bopds f");
        Assert.Equal(" Index: 2\n", i.Eval("explode b1 f; bopindex b1_1"));
        Assert.Contains(" FACE ", i.Eval("whatis z2"), StringComparison.Ordinal);
        Assert.Contains("F/F: (z", i.Eval("bopiterator f f"), StringComparison.Ordinal);
        Assert.StartsWith("F/F: 6 interference(s) found\n", i.Eval("bopinterf f f"), StringComparison.Ordinal);
        Assert.StartsWith("z", i.Eval("bopnews v"), StringComparison.Ordinal);
        Assert.Contains("section edges", i.Eval("bopsc"), StringComparison.Ordinal);
        Assert.StartsWith("Edge ", i.Eval("bopsp"), StringComparison.Ordinal);
        Assert.Contains(" Rank: 0", i.Eval("bopwho 1"), StringComparison.Ordinal);
        Assert.Contains("has no SD shape", i.Eval("bopsd 1"), StringComparison.Ordinal);
        i.Eval("boppb; bopcb");
        i.Eval("bbuild r");
        Assert.Contains("images found", i.Eval("bopimage b1_6"), StringComparison.Ordinal);
        i.Eval("explode r f");
        Assert.Contains("origin", i.Eval("boporigin r_1"), StringComparison.Ordinal);
        Assert.Equal(" shape has no sd shape\n", i.Eval("bopfsd b1_1"));
        // the area builders
        i.Eval("box b 10 10 10; explode b f; compound b_1 b_2 b_3 b_4 b_5 b_6 c");
        Assert.Equal(" s_1\n 1 solids were built\n", i.Eval("bopbsolid s c"));
    }

    [Fact]
    public void Commands_leave_frozen_inputs_unchanged()
    {
        DrawInterpreter i = Load();
        i.Eval("box b1 10 10 10; box b2 5 5 5 10 10 10; tcopy b1 c1; tcopy b2 c2");
        i.Eval("bnondestructive 0; bfuzzyvalue 0.1");
        foreach (string aCommand in new[] { "bcommon r b1 b2", "bfuse r b1 b2", "bcut r b1 b2", "bsection r b1 b2", "bop b1 b2",
                                            "bopfuse r", "bopsection r", "baddobjects b1", "baddtools b2", "bfillds", "bbuild r",
                                            "bbop r 1", "bsplit r", "bapibop r 1", "bapibuild r", "bapisplit r" })
        {
            i.Eval(aCommand);
        }
        // the inputs are what they were: tcopy's copies compare equal by geometry and tolerance
        Assert.Equal(i.Eval("maxtolerance c1").Replace("c1", "b1"), i.Eval("maxtolerance b1"));
        Assert.Equal(i.Eval("maxtolerance c2").Replace("c2", "b2"), i.Eval("maxtolerance b2"));
        Assert.Equal(i.Eval("nbshapes c1").Replace("c1", "b1"), i.Eval("nbshapes b1"));
        Assert.Equal(1000.0, Volume(i, "b1"), 1e-9);
    }
}
