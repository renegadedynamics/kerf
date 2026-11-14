// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using Kerf.Kernel.DrawLite;
using Kerf.Measurement;
using Kerf.Topology;
using Xunit.Abstractions;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// The public area and volume of the corpus's swept and offset faces, right to the tolerance they're asked at (unit 15E15,
/// decisions 2455 and 2456). Each case's <c>result</c> as its script makes it, measured at the default 1e-6 and against the
/// same measure at 1e-10. Before, the offsets' faces were integrated across their basis's knots and the sweeps' estimate agreed
/// with itself far from the integral: bug5805_21's area 3e-5 out, L7's volume 1e-3 out, both with estimates under 1e-8.
/// </summary>
public class DrawMassConfirmTests(ITestOutputHelper output)
{
    private static readonly MassOptions Reference = new() { Tolerance = 1e-10 };

    private static Shape Result(string id)
    {
        var corpus = new DrawCorpus(Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests"));
        DrawTestCase testCase = corpus.Cases().Single(c => $"{c.Group}/{c.Grid}/{c.Name}" == id);
        var interpreter = new DrawInterpreter(TextWriter.Null);
        interpreter.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        corpus.Run(testCase, interpreter);
        return interpreter.GetShape("result");
    }

    [Theory]
    [InlineData("bugs/modalg_2/bug5805_21", 244.193862)]
    [InlineData("bugs/modalg_2/bug5805_45", 744.174404)]
    [InlineData("bugs/modalg_2/bug534_1", 11452.37397)]
    public void A_swept_face_s_area_is_right_to_its_tolerance(string id, double area)
    {
        Shape result = Result(id);
        double measured = result.IntegrateArea(), reference = result.IntegrateArea(Reference);
        output.WriteLine(string.Create(CultureInfo.InvariantCulture, $"{id}: {measured:R} at 1e-6, {reference:R} at 1e-10"));
        Assert.Equal(reference, measured, 1e-6 * reference);
        Assert.Equal(area, measured, 1e-6 * area);
    }

    [Theory]
    [InlineData("offset/faces_type_i/L7")]
    [InlineData("offset/faces_type_i/M2")]
    public void An_offset_s_volume_is_right_to_its_tolerance(string id)
    {
        Shape result = Result(id);
        double measured = result.IntegrateVolume(), reference = result.IntegrateVolume(Reference);
        output.WriteLine(string.Create(CultureInfo.InvariantCulture, $"{id}: {measured:R} at 1e-6, {reference:R} at 1e-10"));
        Assert.Equal(reference, measured, 1e-6 * Math.Abs(reference));
    }
}
