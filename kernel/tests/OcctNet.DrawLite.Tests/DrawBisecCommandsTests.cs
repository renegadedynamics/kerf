// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;
using OcctNet.DrawLite;
using Xunit.Abstractions;

namespace OcctNet.DrawLite.Tests;

/// <summary>
/// Unit 5M's <c>bisec</c> (GeometryTest_CurveCommands.cxx): the bisectors of 2D lines, circles and points through
/// GccAna, their names and kinds, its messages, and the corpus (no case of OCCT's tests uses it).
/// </summary>
public class DrawBisecCommandsTests(ITestOutputHelper output)
{
    private static DrawInterpreter New() => new(new StringWriter { NewLine = "\n" });

    private static T Geometry<T>(DrawInterpreter theDI, string theName) =>
        Assert.IsAssignableFrom<T>(Assert.IsType<DrawGeometry>(theDI.GetDrawable(theName)).Geometry);

    private static string Error(DrawInterpreter theDI, string theScript) =>
        Assert.ThrowsAny<DrawException>(() => theDI.Eval(theScript)).Reason;

    [Fact]
    public void Bisec_names_its_solutions_as_Draw()
    {
        DrawInterpreter i = New();
        i.Eval("line l1 0 0 1 0; line l2 0 0 0 1; circle c1 0 0 1; circle c2 5 0 2; point p1 0 2; point p2 2 2");

        // two lines: two bisecting lines, always named r_1, r_2
        Assert.Equal("There are 2 Solutions.\n", i.Eval("bisec r l1 l2"));
        Line2D aL = Geometry<LineCurve2D>(i, "r_1").Lin2d();
        Assert.Equal(Math.Abs(aL.Direction().X()), Math.Abs(aL.Direction().Y()), 1e-15);
        Geometry<LineCurve2D>(i, "r_2");

        // a line and a circle: parabolas
        string aRes = i.Eval("bisec s l1 c2");
        Assert.StartsWith("There are ", aRes, StringComparison.Ordinal);
        Geometry<ParabolaCurve2D>(i, "s_1");

        // two circles apart: hyperbolas (several solutions)
        Assert.StartsWith("There are ", i.Eval("bisec h c1 c2"), StringComparison.Ordinal);
        Geometry<Curve2D>(i, "h_1");

        // a line and a point: one parabola, named as the result
        Assert.Equal("There is 1 Solution.\n", i.Eval("bisec q l1 p1"));
        Geometry<ParabolaCurve2D>(i, "q");
        Assert.Equal("There is 1 Solution.\n", i.Eval("bisec q2 p1 l1"));
        Geometry<ParabolaCurve2D>(i, "q2");

        // a circle and a point outside it
        Assert.StartsWith("There ", i.Eval("bisec e c1 p2"), StringComparison.Ordinal);

        // two points: their perpendicular bisector
        Assert.Equal("There is 1 Solution.\n", i.Eval("bisec m p1 p2"));
        Line2D aM = Geometry<LineCurve2D>(i, "m").Lin2d();
        Assert.Equal(1.0, aM.Location().X(), 1e-15);
    }

    [Fact]
    public void Bisec_messages_as_Draw()
    {
        DrawInterpreter i = New();
        i.Eval("line l1 0 0 1 0; point p1 1 1; ellipse e 0 0 3 1");
        Assert.Equal("", Error(i, "bisec r l1"));
        Assert.Equal(" Bisec has failed !!\n", Error(i, "bisec r p1 p1"));
        Assert.Equal(" args must be line/circle/point line/circle/point\n", Error(i, "bisec r l1 e"));
        Assert.Equal(" args must be line/circle/point line/circle/point\n", Error(i, "bisec r nothing p1"));
        Assert.Equal(" the second arg must be line/circle/point \n", Error(i, "bisec r p1 nothing"));
        // OCCT doesn't return after this message: no solution follows it
        Assert.Equal(" the second arg must be line/circle/point \nThere is 0 Solution.\n", i.Eval("bisec r l1 nothing"));
        // an ellipse and a point: nothing is computed
        Assert.Equal("There is 0 Solution.\n", i.Eval("bisec r e p1"));
    }

    [Fact]
    public void Corpus_has_no_case_using_bisec()
    {
        string aTests = Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests");
        Assert.True(Directory.Exists(aTests), $"No OCCT source at {aTests}: run tools/get-occt.ps1");
        var aCorpus = new DrawCorpus(aTests);
        var anInterpreter = new DrawInterpreter();
        anInterpreter.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        List<DrawCaseScan> aScans = aCorpus.Cases().AsParallel().AsOrdered()
                                           .Select(c => aCorpus.Scan(c, anInterpreter))
                                           .Where(s => s.Commands.Contains("bisec"))
                                           .ToList();
        output.WriteLine($"{aScans.Count} cases use bisec");
        Assert.Empty(aScans);
    }
}
