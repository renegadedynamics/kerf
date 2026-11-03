// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using System.Globalization;
using System.Text.RegularExpressions;
using Occt;
using Kerf.Kernel.DrawLite;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// Unit 7W's commands (phase7-plan.md section 1.6, the cheap gaps): Draw's <c>dversion</c>, <c>meminfo</c>,
/// <c>lastrep</c>, <c>dmode</c>, <c>XProgress</c> and <c>removeinternals</c>; the geometry commands <c>proj</c>,
/// <c>to2d</c>, <c>to3d</c>, <c>project</c>, <c>projonplane</c>, <c>crvpoints</c>, <c>crvtpoints</c>, <c>2dproj</c>,
/// <c>2dextrema</c>, <c>2dintersect</c>, <c>2dintanalytical</c>, <c>intconcon</c>, <c>incdeg</c>, <c>insertpole</c>,
/// <c>setweight</c>, <c>convert</c>, <c>segsur</c>; the topology commands <c>validrange</c>, <c>shape</c>,
/// <c>subshape</c>, <c>xbounds</c>, <c>xclassify</c>, <c>BRepIntCS</c>, <c>concatC0wire</c>; and the history that
/// <c>prism</c> and <c>revol</c> now record (bugs/modalg_7/bug29586).
/// </summary>
public class DrawCheapGapsCommandsTests
{
    private static DrawInterpreter New() => new(new StringWriter { NewLine = "\n" });

    private static T Geometry<T>(DrawInterpreter theDI, string theName)
        => Assert.IsAssignableFrom<T>(Assert.IsType<DrawGeometry>(theDI.GetDrawable(theName)).Geometry);

    private static double Number(DrawInterpreter theDI, string theName)
    {
        Assert.True(theDI.TryGetNumber(theName, out double aValue), theName);
        return aValue;
    }

    private static double Real(string theText) => double.Parse(theText, CultureInfo.InvariantCulture);

    private static string Error(DrawInterpreter theDI, string theScript) => Assert.ThrowsAny<DrawException>(() => theDI.Eval(theScript)).Reason;

    [Fact]
    public void Revol_and_prism_record_their_history_as_bug29586_checks()
    {
        DrawInterpreter i = New();
        // bugs/modalg_7/bug29586: a revolution of a cut of two discs; the generated edges of the cut's vertices
        i.Eval("circle c1 0 -5 0 1 0 0 10; circle c2 0 5 0 1 0 0 10; mkedge e1 c1; mkedge e2 c2; wire w1 e1; wire w2 e2");
        i.Eval("mkplane f1 w1; mkplane f2 w2; bcut f12 f1 f2; revol r f12 0 0 0 0 0 1 180");
        i.Eval("savehistory h; explode f12 v");
        Assert.Contains("EDGE      : 2", i.Eval("generated g1 h f12_1; nbshapes g1"), StringComparison.Ordinal);
        Assert.Contains("EDGE      : 1", i.Eval("generated g2 h f12_2; nbshapes g2"), StringComparison.Ordinal);
        Assert.Contains("EDGE      : 2", i.Eval("generated g3 h f12_3; nbshapes g3"), StringComparison.Ordinal);

        // a prism of a face: its edges generate the side faces
        i.Eval("box b 1 1 1; explode b f; prism p b_1 0 0 5; savehistory hp; explode b_1 e");
        Assert.Contains("FACE      : 1", i.Eval("generated g hp b_1_1; nbshapes g"), StringComparison.Ordinal);
        Assert.Contains("SOLID     : 1", i.Eval("generated g hp b_1; nbshapes g"), StringComparison.Ordinal);
    }

    [Fact]
    public void Dversion_meminfo_lastrep_dmode_and_XProgress()
    {
        DrawInterpreter i = New();
        string aVersion = i.Eval("dversion");
        Assert.StartsWith("Open CASCADE Technology 8.0.1\n", aVersion, StringComparison.Ordinal);
        Assert.Contains("Exceptions enabled\n", aVersion, StringComparison.Ordinal);
        Assert.Matches(@"\nArchitecture: [^\n]+\nOS: [^\n]+\n$", aVersion);
        Assert.Equal(OperatingSystem.IsWindows(), aVersion.Contains("OS: Windows", StringComparison.Ordinal));

        // the counters in OSD_MemInfo's order, whatever the words' order
        string[] aMem = i.Eval("meminfo h virt").TrimEnd().Split(' ');
        Assert.Equal(2, aMem.Length);
        Assert.True(Real(aMem[0]) > Real(aMem[1]));
        Assert.Contains("  Heap memory:     ", i.Eval("meminfo"), StringComparison.Ordinal);
        Assert.Equal("Syntax error at 'bogus'!\n", Error(i, "meminfo h bogus"));

        // no view: the pick DRAWEXE starts with
        Assert.Equal("", i.Eval("lastrep id x y b"));
        Assert.Equal(0, Number(i, "id"));
        Assert.Equal(0, Number(i, "x"));
        Assert.Equal(0, Number(i, "b"));
        i.Eval("lastrep id x y z b");
        Assert.True(double.IsNaN(Number(i, "x")));
        Assert.Equal("", Error(i, "lastrep id x y"));
        Assert.Equal("Too many args", Error(i, "lastrep id x y z b c"));

        // DrawTrSurf_Params: any word sets the uniform mode
        Assert.Equal("d", i.Eval("dmode"));
        Assert.Equal("u", i.Eval("dmode D"));
        Assert.Equal("30", i.Eval("discr"));
        Assert.Equal("50", i.Eval("discr 50"));
        Assert.Equal("0.01", i.Eval("defle"));

        Assert.Equal("Progress Indicator defaults: tcl mode is OFF, console mode is OFF, graphical mode is OFF\n", i.Eval("XProgress"));
        Assert.Equal("Progress Indicator defaults: tcl mode is ON, console mode is ON, graphical mode is OFF\n", i.Eval("XProgress +t +c x"));
        Assert.Equal("Progress Indicator defaults: tcl mode is OFF, console mode is ON, graphical mode is OFF\n", i.Eval("XProgress -t"));
        // an interpreter of its own
        Assert.Contains("tcl mode is OFF, console mode is OFF", New().Eval("XProgress"), StringComparison.Ordinal);
    }

    [Fact]
    public void Removeinternals_removes_the_internal_sub_shapes_in_every_variable_sharing_them()
    {
        DrawInterpreter i = New();
        i.Eval("box b 1 1 1; explode b f; vertex v 0.5 0.5 1; orientation v I; compound b_1 v c; compound c d");
        Assert.Contains("VERTEX    : 5", i.Eval("nbshapes c"), StringComparison.Ordinal);
        string aBox = i.Eval("dump b_1");
        Assert.Equal("", i.Eval("removeinternals c 1"));
        Assert.Contains("VERTEX    : 4", i.Eval("nbshapes c"), StringComparison.Ordinal);
        // the compound holding c sees the change, as in OCCT; the face doesn't change
        Assert.Contains("VERTEX    : 4", i.Eval("nbshapes d"), StringComparison.Ordinal);
        Assert.Equal(aBox, i.Eval("dump b_1"));
        Assert.Equal("nois a null shape\n", Error(i, "removeinternals no"));
    }

    [Fact]
    public void Proj_projects_points_on_curves_and_surfaces()
    {
        DrawInterpreter i = New();
        i.Eval("circle c 0 0 0 10");
        Assert.Equal("ext_1  parameter 1 = 0\next_2  parameter 2 = 3.1415926535897931\n", i.Eval("proj c 20 0 0"));
        Assert.Equal(10, Geometry<TrimmedCurve3D>(i, "ext_1").LastParameter(), 1e-12);
        Assert.Equal("ext_1  Point on curve  parameter 1 = 0\next_2  parameter 2 = 3.1415926535897931\n", i.Eval("proj c 10 0 0"));
        Assert.Equal(new Point3D(10, 0, 0), Geometry<Point3D>(i, "ext_1"));

        i.Eval("sphere s 5");
        Assert.Equal("ext_1  Parameters: 0 0\next_2  Parameters: 3.1415926535897931 -0\n", i.Eval("proj s 10 0 0"));
        Assert.Equal("ext_1  Parameters: 0 0\n", i.Eval("proj s 10 0 0 0.1 0.1"));
        Assert.Equal("", Error(i, "proj c 1 2"));
    }

    [Fact]
    public void To2d_to3d_project_and_projonplane()
    {
        DrawInterpreter i = New();
        i.Eval("circle c 1 2 0 10; to2d c2 c; to3d c3 c2");
        Assert.Equal(new Point2D(1, 2), Geometry<CircleCurve2D>(i, "c2").Location());
        Assert.Equal(new Point3D(1, 2, 0), Geometry<CircleCurve3D>(i, "c3").Location());
        i.Eval("cylinder cy 10");
        Assert.Equal("", Error(i, "to2d x c cy"));

        // a circle on a cylinder projects on a line of the cylinder's parameters
        i.Eval("circle cc 0 0 3 10; project lp cc cy");
        Assert.Equal(new Point2D(0, 3), Geometry<LineCurve2D>(i, "lp").Location());
        string aVerif = i.Eval("project lp cc cy -v 3");
        Assert.Equal(5, Regex.Matches(aVerif, @" Parameter = \S+\tDistance = 0\n").Count);
        Assert.EndsWith(" **** Distance Maximale : 0\n", aVerif, StringComparison.Ordinal);
        Assert.Equal(" n must be positive\n", i.Eval("project lp cc cy -v 0"));
        Assert.StartsWith("project result2d c3d surf", i.Eval("project"), StringComparison.Ordinal);

        i.Eval("plane p 0 0 0 0 0 1; circle cz 0 0 4 0 1 0 2; projonplane pp cz p 0");
        Assert.Equal(0, Geometry<Curve3D>(i, "pp").Value(0.3).Z(), 1e-12);
        Assert.Equal(" The surface must be a plane\n", Error(i, "projonplane pp cz cy"));
    }

    [Fact]
    public void Crvpoints_and_crvtpoints_discretise_curves_and_wires()
    {
        DrawInterpreter i = New();
        i.Eval("bsplinecurve bs 3 2 0 4 1 4 0 0 0 1 1 0 0 1 2 1 0 1 3 0 0 1");
        Assert.Equal("Nb points : 11\nMax defl: 0.009478998464963197 0 0.125 1\n", i.Eval("crvpoints r bs 0.01"));
        BSplineCurve3D aPolyline = Geometry<BSplineCurve3D>(i, "r");
        Assert.Equal(1, aPolyline.Degree());
        Assert.Equal(11, aPolyline.PoleCount);
        Assert.StartsWith("Nb points : 19\nMax defl: ", i.Eval("crvtpoints r bs 0.01 0.1"), StringComparison.Ordinal);

        i.Eval("polyline w 0 0 0 1 0 0 1 1 0");
        Assert.Equal("Nb points : 3\nMax defl: 0 0 0 0\n", i.Eval("crvpoints r w 0.01"));
        Assert.Equal("", Error(i, "crvpoints r nothing 0.01"));
    }

    [Fact]
    public void The_2d_analysis_and_intersection_commands()
    {
        DrawInterpreter i = New();
        i.Eval("circle ca 0 0 1; circle cb 1 0 1; line l2 0 0.5 1 0");
        Assert.Equal("Intersection point 1 : 0.5 0.8660254037844386\n"
                     + "parameter on the fist: 1.0471975511965979 parameter on the second: 2.0943951023931957\n"
                     + "Intersection point 2 : 0.5 -0.8660254037844386\n"
                     + "parameter on the fist: 5.2359877559829879 parameter on the second: 4.1887902047863905\n",
                     i.Eval("2dintanalytical ca cb"));
        Assert.Equal("Syntax error: 'l2' is not a circle", Error(i, "2dintanalytical ca l2"));
        // the line is the second conic: implicit, so no parameter on it
        string aConCon = i.Eval("intconcon ca l2");
        Assert.StartsWith("Intersection point 1 : 0.8660254037844386 0.5\nparameter on the fist: 0.52359877559829893\n", aConCon, StringComparison.Ordinal);

        string anInter = i.Eval("2dintersect ca l2 -state");
        Assert.Equal(2, Regex.Matches(anInter, "Intersection type: INTERSECTION\n").Count);
        Assert.Contains("Intersection point 1 : ", anInter, StringComparison.Ordinal);
        Assert.NotEmpty(i.Eval("2dintersect ca l2 -tol 1e-7"));
        Assert.Equal("Syntax error: wrong number of arguments", Error(i, "2dintersect -state"));
        Assert.Equal("Syntax error at 'cb'", Error(i, "2dintersect ca l2 cb"));

        // the circle's extrema with the line: the two points at distance 0
        string anExtrema = i.Eval("2dextrema ca l2");
        Assert.Matches(@"^dist 1: \S+  ext_1\ndist 2: \S+  ext_2\n$", anExtrema);
        Assert.Equal("Syntax error: wrong number of arguments", Error(i, "2dextrema ca"));
        Assert.Equal("ext_1 ext_2 ", i.Eval("2dproj ca 2 0"));
        Assert.Equal([1.0, 3.0], new[] { "ext_1", "ext_2" }.Select(e => Math.Round(Geometry<TrimmedCurve2D>(i, e).LastParameter(), 12)).Order());
        Assert.Equal("Syntax error: 'no' is NULL", Error(i, "2dproj no 2 0"));
    }

    [Fact]
    public void Incdeg_insertpole_setweight_convert_and_segsur_bind_the_changed_copy()
    {
        DrawInterpreter i = New();
        i.Eval("beziercurve bz 3 0 0 0 1 1 0 2 0 0; copy bz keep");
        i.Eval("incdeg bz 4");
        Assert.Equal(4, Geometry<BezierCurve3D>(i, "bz").Degree());
        Assert.Equal(2, Geometry<BezierCurve3D>(i, "keep").Degree());
        i.Eval("insertpole bz 1 0.5 0.5 0");
        Assert.Equal(new Point3D(0.5, 0.5, 0), Geometry<BezierCurve3D>(i, "bz").Pole(2));
        i.Eval("setweight bz 2 2");
        Assert.Equal(2, Geometry<BezierCurve3D>(i, "bz").Weight(2));
        Assert.Equal("insertpole needs a bezier curve", Error(i, "insertpole nothing 1 0 0 0"));

        i.Eval("2dbeziercurve b2 3 0 0 1 1 2 0; insertpole b2 1 0.5 0.5 3");
        Assert.Equal(3, Geometry<BezierCurve2D>(i, "b2").Weight(2));

        i.Eval("circle c 0 0 0 10; convert cc c");
        BSplineCurve3D aConverted = Geometry<BSplineCurve3D>(i, "cc");
        Assert.True(aConverted.IsPeriodic);
        Assert.Equal(6, aConverted.PoleCount);
        i.Eval("convert cp c po");
        Assert.False(Geometry<BSplineCurve3D>(i, "cp").IsRational);

        i.Eval("sphere s 5; convert ss s; segsur ss 0 1 0 0.5");
        BSplineSurface aSegment = Geometry<BSplineSurface>(i, "ss");
        aSegment.Bounds(out double u1, out double u2, out double v1, out double v2);
        Assert.Equal((0.0, 1.0, 0.0, 0.5), (u1, u2, v1, v2));
        i.Eval("setweight ss 1 1 3");
        Assert.Equal(3, Geometry<BSplineSurface>(i, "ss").Weight(1, 1));
        Assert.Equal("", Error(i, "setweight ss 1 3"));
        Assert.Equal("", Error(i, "setweight ss 1"));
    }

    [Fact]
    public void Validrange_shape_subshape_xbounds_xclassify_BRepIntCS_and_concatC0wire()
    {
        DrawInterpreter i = New();
        i.Eval("box b 1 2 3; explode b e");
        // the edge's range without its vertices' tolerance spheres
        string[] aRange = i.Eval("validrange b_1").Split(' ');
        Assert.True(Real(aRange[0]) > 0 && Real(aRange[0]) < 1e-6);
        Assert.True(Real(aRange[1]) < 3 && Real(aRange[1]) > 3 - 1e-6);
        Assert.Equal("", i.Eval("validrange b_1 u1 u2"));
        Assert.Equal(Real(aRange[0]), Number(i, "u1"));

        i.Eval("shape em c");
        Assert.Contains("COMPOUND", i.Eval("dtyp em"), StringComparison.Ordinal);
        i.Eval("shape ee so");
        Assert.Contains("SOLID", i.Eval("dtyp ee"), StringComparison.Ordinal);
        Assert.Equal("", Error(i, "shape x bogus"));

        Assert.Equal("b_2", i.Eval("subshape b f 2"));
        Assert.Contains("FACE", i.Eval("dtyp b_2"), StringComparison.Ordinal);
        Assert.Equal("b_1", i.Eval("subshape b 1"));
        Assert.Contains("SHELL", i.Eval("dtyp b_1"), StringComparison.Ordinal);
        Assert.Equal("b_5", i.Eval("subshape b v 5"));

        i.Eval("explode b f");
        Assert.Equal("0\n3\n-2\n0\n", i.Eval("xbounds b_1"));
        Assert.Equal(" shapeb must be a face\n", i.Eval("xbounds b"));
        Assert.Equal("state is: OUT\n", i.Eval("xclassify b"));
        Assert.Equal(" Shape type must be SOLID\n", i.Eval("xclassify b_1"));

        i.Eval("line ln 0.5 1 -5 0 0 1");
        Assert.Equal("Point 1 : 0.5 1 0\nPoint 2 : 0.5 1 3\n", i.Eval("BRepIntCS ln b res"));
        Assert.Contains("VERTEX    : 2", i.Eval("nbshapes res"), StringComparison.Ordinal);
        Assert.Equal(new Point3D(0.5, 1, 3), Geometry<Point3D>(i, "brics_2"));
        i.Eval("line far 50 50 0 0 0 1");
        Assert.Equal("Points of intersections are not found\n", i.Eval("BRepIntCS far b"));
        Assert.Equal("Point 1 : 0.5 1 0\nPoint 2 : 0.5 1 3\n", i.Eval("BRepIntCS ln far b"));

        i.Eval("polyline pw 0 0 0 1 0 0 1 1 0; concatC0wire cw pw");
        Assert.Contains("EDGE", i.Eval("dtyp cw"), StringComparison.Ordinal);
        Assert.Equal(2, Real(Regex.Match(i.Eval("lprops cw"), @"Mass :\s+(\S+)").Groups[1].Value), 1e-9);
    }
}
