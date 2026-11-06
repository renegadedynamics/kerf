// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Text.RegularExpressions;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// Decision 1816: a clean doesn't spoil a good Boolean. Native OCCT 8.0.1, given these same results, returns invalid
/// shapes, with a volume up to 17% out: 1071.85 for 1055.58, 3.97e6 for 4.05e6, 184369 for 222817.
/// </summary>
public class DrawUnifySpoilsNothingTests
{
    private static DrawInterpreter NewInterpreter()
    {
        var anInterpreter = new DrawInterpreter(new StringWriter { NewLine = "\n" });
        anInterpreter.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        anInterpreter.SetVariable("env(CSF_TestDataPath)", DrawCorpusTests.DataPath);
        return anInterpreter;
    }

    private static double Volume(string theProps) =>
        double.Parse(Regex.Match(theProps, @"Mass :\s+(\S+)").Groups[1].Value, CultureInfo.InvariantCulture);

    private static void CheckTheClean(DrawInterpreter theDI)
    {
        // the command writes into the shape it's given, as OCCT's does
        theDI.Eval("tcopy r before");
        double aVolume = Volume(theDI.Eval("vprops before"));
        theDI.Eval("unifysamedom u r");
        Assert.Contains("This shape seems to be valid", theDI.Eval("checkshape u"), StringComparison.Ordinal);
        Assert.Equal(aVolume, Volume(theDI.Eval("vprops u")), aVolume * 1e-5);
    }

    // Two faces of one cone with their seams apart can't be one face without moving a seam.
    [Theory]
    [InlineData(90)]
    [InlineData(180)]
    [InlineData(270)]
    public void Cones_with_their_seams_apart(int theAngle)
    {
        DrawInterpreter aDI = NewInterpreter();
        aDI.Eval("pcone b1 8 4 8");
        aDI.Eval("pcone b2 4 2 4");
        aDI.Eval("ttranslate b2 0 0 8");
        aDI.Eval($"trotate b2 0 0 2 0 0 1 {theAngle}");
        aDI.Eval("bop b1 b2");
        aDI.Eval("bopfuse r");
        CheckTheClean(aDI);
        Assert.Contains("FACE      : 4", aDI.Eval("nbshapes u"), StringComparison.Ordinal);
    }

    // A merge whose wires can't be built must leave no pcurves behind (H8, N1); pole edges merged on a reversed face must
    // go the same way round (I5). boolean/bfuse_complex.
    [Theory]
    [InlineData("CTO902_cts20487a.rle", "cts20487b.rle")]
    [InlineData("CTO904_pro14893a.rle", "pro14893b.rle")]
    [InlineData("cts20552-part.rle", "cts20552-tool.rle")]
    public void Fuses_from_the_corpus(string theA, string theB)
    {
        DrawInterpreter aDI = NewInterpreter();
        aDI.Eval($"restore [locate_data_file {theA}] a");
        aDI.Eval($"restore [locate_data_file {theB}] b");
        aDI.Eval("bfuse r a b");
        CheckTheClean(aDI);
    }
}
