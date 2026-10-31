// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Text.RegularExpressions;
using OcctNet.DrawLite;

namespace OcctNet.DrawLite.Tests;

/// <summary>
/// Unit 5W's commands on hand-made shapes and curves: lofts (<c>thrusections</c>, <c>gener</c>, <c>pruled</c>), pipe
/// shells (<c>mksweep</c> and the other sweep commands), <c>middlepath</c>, <c>geompipe</c>, projections (<c>prj</c>,
/// <c>cprj</c>), the continuity commands, <c>ruled</c>, <c>partuyau</c>, qualified circles (<c>qcircle</c>,
/// <c>qline</c>, <c>circ2d3Tan</c>) and the interpolations. Draw variables hold frozen shapes: every command leaves its
/// inputs as they were.
/// </summary>
public class DrawConstructionCommandsTests5W
{
    private static DrawInterpreter NewInterpreter()
    {
        var anInterpreter = new DrawInterpreter(new StringWriter { NewLine = "\n" });
        anInterpreter.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        return anInterpreter;
    }

    // sprops', vprops' or lprops' mass, to their 6 significant digits
    private static double Mass(DrawInterpreter theDI, string theCommand)
        => double.Parse(Regex.Match(theDI.Eval(theCommand), @"Mass :\s+(\S+)").Groups[1].Value, CultureInfo.InvariantCulture);

    private static void AssertNear(double theExpected, double theMass) => Assert.Equal(theExpected, theMass, Math.Abs(theExpected) * 1e-5);

    // a Tcl error's message ends with the command's result
    private static void AssertError(DrawInterpreter theDI, string theCommand, string theResult)
        => Assert.EndsWith(theResult, Assert.ThrowsAny<DrawException>(() => theDI.Eval(theCommand)).Message, StringComparison.Ordinal);

    private static void AssertValid(DrawInterpreter theDI, string theName)
        => Assert.Contains("This shape seems to be valid", theDI.Eval("checkshape " + theName), StringComparison.Ordinal);

    // the text of each variable, to check that commands leave their inputs as they were
    private static string[] Dumps(DrawInterpreter theDI, params string[] theNames) => [.. theNames.Select(n => theDI.Eval("dump " + n))];

    [Fact]
    public void Thrusections_lofts_wires_and_vertices_and_reports_failures()
    {
        DrawInterpreter aDI = NewInterpreter();
        aDI.Eval("circle c1 0 0 0 1");
        aDI.Eval("circle c2 0 0 1 1");
        aDI.Eval("circle c3 0 0 2 2");
        aDI.Eval("mkedge e1 c1; wire w1 e1; mkedge e2 c2; wire w2 e2; mkedge e3 c3; wire w3 e3");
        string[] aBefore = Dumps(aDI, "w1", "w2", "w3");

        // a ruled solid between two equal circles is a cylinder
        Assert.Equal("", aDI.Eval("thrusections r 1 1 w1 w2"));
        AssertNear(Math.PI, Mass(aDI, "vprops r"));
        AssertValid(aDI, "r");
        // smooth through three, a shell, without the check, and safe
        Assert.Equal("", aDI.Eval("thrusections r 0 0 w1 w2 w3"));
        Assert.Contains("SHELL", aDI.Eval("whatis r"), StringComparison.Ordinal);
        Assert.Equal("", aDI.Eval("thrusections -N r 1 0 w1 w2 w3 -safe"));
        AssertValid(aDI, "r");
        // a cone to a vertex
        aDI.Eval("vertex v 0 0 3");
        // DBRep::Get complains on the output ("v is not a wire but a vertex") before the vertex is read
        Assert.Equal("", aDI.Eval("thrusections r 1 1 w1 v"));
        AssertNear(Math.PI, Mass(aDI, "vprops r"));
        // the loft's history
        Assert.Equal("", aDI.Eval("savehistory h"));
        Assert.Equal("", aDI.Eval("generated g h e1"));
        Assert.Contains("FACE", aDI.Eval("whatis g"), StringComparison.Ordinal);

        // an open profile with a closed one: no result, OCCT's message
        aDI.Eval("polyline p 0 0 5 1 0 5 1 1 5");
        Assert.Equal("The input profiles should be all closed or all opened\n", aDI.Eval("thrusections q 0 1 w1 p"));
        Assert.Equal(0, aDI.Eval("isdraw q") == "1" ? 1 : 0);
        Assert.ThrowsAny<DrawException>(() => aDI.Eval("thrusections r 0 1 w1"));
        Assert.Equal(aBefore, Dumps(aDI, "w1", "w2", "w3"));
    }

    [Fact]
    public void Gener_and_pruled_join_wires_and_edges()
    {
        DrawInterpreter aDI = NewInterpreter();
        aDI.Eval("polyline p1 0 0 0 1 0 0 1 1 0 0 1 0 0 0 0");
        aDI.Eval("polyline p2 0 0 2 1 0 2 1 1 2 0 1 2 0 0 2");
        string[] aBefore = Dumps(aDI, "p1", "p2");
        Assert.Equal("", aDI.Eval("gener r p1 p2"));
        AssertNear(8.0, Mass(aDI, "sprops r"));
        AssertValid(aDI, "r");
        Assert.Equal("", aDI.Eval("pruled r p1 p2"));
        AssertNear(8.0, Mass(aDI, "sprops r"));

        // two edges make a face; an edge with a wire, a shell
        aDI.Eval("line l 0 0 0 1 0 0; mkedge a l 0 3; line m 0 2 1 1 0 0; mkedge b m 0 3");
        Assert.Equal("", aDI.Eval("pruled f a b"));
        Assert.Contains("FACE", aDI.Eval("whatis f"), StringComparison.Ordinal);
        AssertNear(3 * Math.Sqrt(5), Mass(aDI, "sprops f"));
        aDI.Eval("wire wb b");
        Assert.Equal("", aDI.Eval("pruled s a wb"));
        Assert.Contains("SHELL", aDI.Eval("whatis s"), StringComparison.Ordinal);
        Assert.ThrowsAny<DrawException>(() => aDI.Eval("pruled s a"));
        Assert.ThrowsAny<DrawException>(() => aDI.Eval("gener s p1"));
        Assert.Equal(aBefore, Dumps(aDI, "p1", "p2"));
    }

    [Fact]
    public void Sweep_commands_build_and_simulate_pipe_shells()
    {
        DrawInterpreter aDI = NewInterpreter();
        Assert.StartsWith("setsweep options", aDI.Eval("setsweep"), StringComparison.Ordinal);
        Assert.StartsWith("addsweep wire/vertex", aDI.Eval("addsweep"), StringComparison.Ordinal);
        Assert.StartsWith("build sweep result", aDI.Eval("buildsweep"), StringComparison.Ordinal);
        AssertError(aDI, "buildsweep r", "You have forgotten the <<mksweep>> command  !\n");

        aDI.Eval("polyline sp 0 0 0 0 0 10 0 10 10");
        aDI.Eval("circle c 0 0 0 1; mkedge e c; wire w e");
        aDI.Eval("circle c2 0 0 10 2; mkedge e2 c2; wire w2 e2");
        string[] aBefore = Dumps(aDI, "sp", "w", "w2");
        aDI.Eval("mksweep sp");
        AssertError(aDI, "buildsweep r", "You have forgotten the <<addsweep>> command  !\n");
        aDI.Eval("addsweep w");
        // a right corner, a solid
        Assert.Equal("", aDI.Eval("buildsweep r -C -S"));
        Assert.Contains("SOLID", aDI.Eval("whatis r"), StringComparison.Ordinal);
        AssertValid(aDI, "r");
        Assert.StartsWith("Tolerance on surfaces = ", aDI.Eval("errorsweep"), StringComparison.Ordinal);
        Assert.Equal("", aDI.Eval("savehistory h"));
        Assert.Equal("", aDI.Eval("generated g h e"));
        Assert.Contains("COMPOUND", aDI.Eval("whatis g"), StringComparison.Ordinal);

        // a second section at the corner, then removed again
        aDI.Eval("vertex v 0 0 10");
        aDI.Eval("addsweep w2 v");
        aDI.Eval("simulsweep s 3");
        Assert.Contains("WIRE", aDI.Eval("whatis s_3"), StringComparison.Ordinal);
        aDI.Eval("deletesweep w2");
        // the modes
        foreach (string aMode in new[] { "-FR", "-CF", "-DT", "-CN 1 0 0", "-FX 0 0 1", "-FX 0 0 1 1 0 0", "-DM 8", "-SM 50" })
        {
            Assert.Equal("", aDI.Eval("setsweep " + aMode));
        }
        AssertError(aDI, "setsweep -XX", "The option -XX is unknown !\n");
        AssertError(aDI, "setsweep -DM 20", " -DM paramvalue must be [1; 14]\n");

        // a law of radii along a straight spine
        aDI.Eval("polyline st 0 0 0 0 0 10");
        aDI.Eval("mksweep st");
        aDI.Eval("addsweep w 0 1 1 2");
        Assert.Equal("", aDI.Eval("buildsweep r -S"));
        // a frustum of radii 1 and 2, height 10
        AssertNear(Math.PI * 10 * (1 + 2 + 4) / 3, Mass(aDI, "vprops r"));

        // mksweep -C merges the spine's tangent edges first
        Assert.Contains("To correct input spine", Assert.ThrowsAny<DrawException>(() => aDI.Eval("mksweep sp -X 1 1")).Message,
                        StringComparison.Ordinal);
        Assert.Equal("", aDI.Eval("mksweep sp -C 5 0.1"));
        Assert.Equal(aBefore, Dumps(aDI, "sp", "w", "w2"));
    }

    [Fact]
    public void Middlepath_geompipe_and_projections()
    {
        DrawInterpreter aDI = NewInterpreter();
        // the middle path of a box between two opposite faces (bug23367_1)
        aDI.Eval("box p 10 100 10");
        aDI.Eval("explode p f");
        string[] aBefore = Dumps(aDI, "p", "p_3", "p_4");
        Assert.Equal("", aDI.Eval("middlepath mp p p_3 p_4"));
        AssertNear(100.0, Mass(aDI, "lprops mp"));

        // geompipe: a circle of radius 1 swept along a Bezier curve, guided by a circle
        aDI.Eval("circle cg 0 0 0 0 0 1 1; mkedge gc cg");
        aDI.Eval("beziercurve bz 3 0 0 0 0 1 5 0 0 10; mkedge be bz");
        Assert.Equal("", aDI.Eval("geompipe gp gc be 1"));
        Assert.Contains("FACE", aDI.Eval("whatis gp"), StringComparison.Ordinal);

        // prj and cprj: a square projected on a box's faces
        aDI.Eval("box b -5 -5 -5 10 10 10");
        aDI.Eval("polyline w -2 -2 20 2 -2 20 2 2 20 -2 2 20 -2 -2 20");
        string[] aBoxBefore = Dumps(aDI, "b", "w");
        Assert.Equal("r_1 r_2 \n", aDI.Eval("prj r w b 0 0 1"));
        AssertNear(16.0, Mass(aDI, "lprops r_1"));
        Assert.Equal("c_1 c_2 \n", aDI.Eval("cprj c w b 0 0 40"));
        AssertError(aDI, "prj r nothing b 0 0 1", "Null input shapes\n");
        Assert.ThrowsAny<DrawException>(() => aDI.Eval("prj r w b 0 0"));
        Assert.Equal(aBefore, Dumps(aDI, "p", "p_3", "p_4"));
        Assert.Equal(aBoxBefore, Dumps(aDI, "b", "w"));
    }

    [Fact]
    public void Continuity_commands_report_LocalAnalysis()
    {
        DrawInterpreter aDI = NewInterpreter();
        aDI.Eval("circle c 0 0 0 1");
        aDI.Eval("line l 1 0 0 0 1 0");
        string aC1 = aDI.Eval("curveCcontinuity 1 c 0 l 0");
        Assert.StartsWith(" Continuity Status : C1 \n C0Value = 0\n", aC1, StringComparison.Ordinal);
        Assert.Contains("G1Angle = ", aDI.Eval("curveGcontinuity 1 c 0 l 0"), StringComparison.Ordinal);
        // out of the second curve's range: Draw's error
        aDI.Eval("trim lt l 0 1");
        Assert.ThrowsAny<DrawException>(() => aDI.Eval("curveCcontinuity 1 c 0 lt 2"));
        // an order the command doesn't know: Message::SendFail's line, then an error
        Assert.ThrowsAny<DrawException>(() => aDI.Eval("curveCcontinuity 5 c 0 l 0"));

        aDI.Eval("plane p 0 0 0 0 0 1");
        aDI.Eval("cylinder cy 0 0 0 1");
        Assert.Contains("C0Value = ", aDI.Eval("surfaceCcontinuity 0 p 1 0 cy 0 0"), StringComparison.Ordinal);
        Assert.Contains("Continuity Status : ", aDI.Eval("surfaceGcontinuity 1 p 1 0 cy 0 0"), StringComparison.Ordinal);

        Assert.Equal("c has CN continuity.\n", aDI.Eval("getcurvcontinuity c"));
        aDI.Eval("2dbeziercurve b2 3 0 0 1 1 2 0");
        Assert.Equal("b2 has CN continuity.\n", aDI.Eval("getcurvcontinuity b2"));
        Assert.Equal("cy has CN continuity.\n", aDI.Eval("getsurfcontinuity cy"));
        AssertError(aDI, "getsurfcontinuity c", "Argument is not a surface!\n");
    }

    [Fact]
    public void Ruled_and_partuyau_make_surfaces()
    {
        DrawInterpreter aDI = NewInterpreter();
        aDI.Eval("circle c1 0 0 0 1");
        aDI.Eval("circle c2 0 0 5 1");
        Assert.Equal("", aDI.Eval("ruled r c1 c2"));
        Assert.Contains("a surface", aDI.Eval("whatis r"), StringComparison.Ordinal);
        AssertError(aDI, "ruled r c1 x", " C2 is not a Curve ==> Command failed\n");

        aDI.Eval("line l 0 0 0 0 0 1; trim lt l 0 10");
        Assert.StartsWith("Accuracy of approximation = ", aDI.Eval("partuyau t lt 2"), StringComparison.Ordinal);
        Assert.Contains("a surface", aDI.Eval("whatis t"), StringComparison.Ordinal);
    }

    [Fact]
    public void Qualified_curves_and_circ2d3Tan_solve_as_Draw()
    {
        DrawInterpreter aDI = NewInterpreter();
        // GccEnt::PositionFromString reads the qualifier without the dash that the command's help shows
        aDI.Eval("qcircle q0 0 0 1 -enclosed");
        Assert.Contains("Position :UNQUALIFIED", aDI.Eval("dump q0"), StringComparison.Ordinal);
        aDI.Eval("qcircle q1 0 0 1 enclosed");
        aDI.Eval("qline l1 0 -2 1 0 -unqualified");
        Assert.Contains("qualified 2d Circle", aDI.Eval("whatis q1"), StringComparison.Ordinal);
        Assert.Contains("Position :ENCLOSED", aDI.Eval("dump q1"), StringComparison.Ordinal);
        Assert.Equal("1", aDI.Eval("2dcvalue q1 0 x y; dval x"));

        // the circle through three points
        aDI.Eval("point p1 1 0; point p2 0 1; point p3 -1 0");
        string aSolutions = aDI.Eval("circ2d3Tan c p1 p2 p3");
        Assert.StartsWith("circle: c_1, qualifiers: NOQUALIFIER, NOQUALIFIER, NOQUALIFIER\n", aSolutions, StringComparison.Ordinal);
        Assert.Contains("    c_1_tp_1 (", aSolutions, StringComparison.Ordinal);
        Assert.Contains("2d curve", aDI.Eval("whatis c_1"), StringComparison.Ordinal);
        // an unqualified circle, a line and a point
        aDI.Eval("qcircle q2 0 0 1; point p4 0 3");
        Assert.StartsWith("circle: c_1, qualifiers: ", aDI.Eval("circ2d3Tan c q2 l1 p4"), StringComparison.Ordinal);
        // an enclosed circle and a line outside it with a point on the circle: no solution
        Assert.Equal("", aDI.Eval("circ2d3Tan c q1 l1 p1"));
        Assert.Throws<DrawException>(() => aDI.Eval("circ2d3Tan c p1 p2"));
    }

    [Fact]
    public void Interpolations_go_through_their_points()
    {
        DrawInterpreter aDI = NewInterpreter();
        Assert.Equal("1 ", aDI.Eval("tanginterpol c 1 0 0 0 1 1 0 2 0 0"));
        Assert.Contains("a 3d curve", aDI.Eval("whatis c"), StringComparison.Ordinal);
        Assert.Equal("3 ", aDI.Eval("tanginterpol c 3 0 0 0 1 1 0 2 0 0 1 0 0"));
        Assert.Equal("p ", aDI.Eval("tanginterpol c p 3 0 0 0 1 1 0 2 0 0"));

        string aFile = Path.Combine(Path.GetTempPath(), $"occtnet-5w-{Guid.NewGuid():N}.txt");
        try
        {
            File.WriteAllText(aFile, "3\n3d\n0 0 0\n1 1 0\n2 0 0\n");
            Assert.Equal("", aDI.Eval($"interpol i {{{aFile.Replace('\\', '/')}}}"));
            Assert.Contains("a 3d curve", aDI.Eval("whatis i"), StringComparison.Ordinal);
            File.WriteAllText(aFile, "3 2d 0 0 1 1 2 0");
            aDI.Eval($"interpol j {{{aFile.Replace('\\', '/')}}}");
            Assert.Contains("2d curve", aDI.Eval("whatis j"), StringComparison.Ordinal);
        }
        finally
        {
            File.Delete(aFile);
        }
        Assert.Equal("give a name to your curve !\n", aDI.Eval("interpol"));
        Assert.ThrowsAny<DrawException>(() => aDI.Eval("interpol k /no/such/file.txt"));
    }
}
