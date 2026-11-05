// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Kernel.DrawLite;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// <see cref="DrawCapture"/>, and the captures tools/borec records from (unit 6G), written when the environment
/// variable OCCTNET_BOREC_CAPTURE names a directory.
/// </summary>
public class DrawCaptureTests
{
    [Fact]
    public void A_case_s_calls_are_captured_with_their_shapes()
    {
        var aCorpus = new DrawCorpus(Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests"));
        DrawTestCase aCase = aCorpus.Cases().First(c => c is { Group: "boolean", Grid: "volumemaker", Name: "A1" });
        List<DrawCapture.Call> aCalls = DrawCapture.Capture(aCorpus, aCase, ["mkvolume", "checkprops"], TimeSpan.FromSeconds(60));
        DrawCapture.Call aCall = Assert.Single(aCalls, c => c.Words[0] == "mkvolume");
        Assert.Equal(["mkvolume", "result", "f1", "f2"], aCall.Words);
        Assert.Equal(["f1", "f2"], aCall.Shapes.Keys.Order());
        Assert.Contains("CASCADE Topology V3", aCall.Shapes["f1"], StringComparison.Ordinal);

        var aWriter = new StringWriter { NewLine = "\n" };
        DrawCapture.Write(aWriter, "boolean/volumemaker/A1", [aCall]);
        Assert.StartsWith("case boolean/volumemaker/A1\ncall 4\nw mkvolume\nw result\nw f1\nw f2\nshape f1 ", aWriter.ToString(), StringComparison.Ordinal);
    }
}
