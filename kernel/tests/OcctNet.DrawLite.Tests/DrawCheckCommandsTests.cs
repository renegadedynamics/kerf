// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Topology;
using System.Text.RegularExpressions;
using Occt;
using OcctNet.DrawLite;

namespace OcctNet.DrawLite.Tests;

/// <summary>
/// The checking commands of unit 3L2a on OCCT's data/occ files: checkshape, checksection, computetolerance (from
/// BRepTest_CheckCommands.cxx), bounding, isbbinterf, gbounding, getcoords (BRepTest_BasicCommands.cxx), bclassify,
/// b2dclassify (BOPTest_LowCommands.cxx), point (GeomliteTest), dchrono, chrono and dlog (Draw_BasicCommands.cxx), and
/// the command log. The texts are Draw's, derived from those sources; the values are those the oracle tests
/// (DrawLiteChecksOracleTests) find equal to native OCCT's.
/// </summary>
public class DrawCheckCommandsTests
{
    private static string DataOcc => Path.Combine(DrawResourcesTests.OcctSourceDirectory, "data", "occ").Replace('\\', '/');

    private static string TestData => DrawResourcesTests.OcctTestDataDirectory.Replace('\\', '/').TrimEnd('/');

    private static (DrawInterpreter Interpreter, StringWriter Output) Load()
    {
        var anOutput = new StringWriter { NewLine = "\n" };
        var anInterpreter = new DrawInterpreter(anOutput);
        anInterpreter.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        return (anInterpreter, anOutput);
    }

    private static string Error(DrawInterpreter theDI, string theScript) => Assert.ThrowsAny<DrawException>(() => theDI.Eval(theScript)).Reason;

    // -------------------------------------------------------------------------------------------------------------
    // checkshape

    [Fact]
    public void Checkshape_reports_a_valid_shape_in_each_style()
    {
        (DrawInterpreter i, _) = Load();
        i.Eval($"restore {DataOcc}/solid.brep s");
        Assert.Equal("This shape seems to be valid", i.Eval("checkshape s"));
        Assert.Equal("This shape seems to be valid", i.Eval("checkshape -top s"));
        Assert.Equal("This shape seems to be valid", i.Eval("checkshape s -parallel -exact"));
        Assert.Equal("This shape seems to be valid", i.Eval("checkshape s -SHORT"));
        Assert.Equal(" -- The Shape s looks OK", i.Eval("checkshape s res"));
        Assert.Equal(" -- The Shape s looks OK", i.Eval("checkshape -top s res -short"));
        Assert.Equal("0", i.Eval("isdraw faulty_1"));
    }

    private const string TopContextual = "On Shape faulty_1 :\nBRepCheck_SelfIntersectingWire\n"
                                         + "On Shape faulty_2 :\nBRepCheck_UnorientableShape\n"
                                         + "Shape faulty_1  on shape faulty_2 :\nBRepCheck_SelfIntersectingWire\n"
                                         + "\nFaulty shapes in variables faulty_1 to faulty_2 \n\n";

    [Fact]
    public void Checkshape_dumps_the_faulty_shapes_in_context_and_binds_them()
    {
        // Top.brep, the one invalid file of data/occ: a face whose wire intersects itself
        (DrawInterpreter i, _) = Load();
        i.Eval($"restore {DataOcc}/Top.brep s");
        Assert.Equal(TopContextual, i.Eval("checkshape s"));
        Assert.StartsWith("shape WIRE", i.Eval("dtyp faulty_1"), StringComparison.Ordinal);
        Assert.StartsWith("shape FACE", i.Eval("dtyp faulty_2"), StringComparison.Ordinal);

        // the parallel analysis prints the same; -short only says so; the topology alone is valid
        Assert.Equal(TopContextual, i.Eval("checkshape s -parallel"));
        Assert.Equal("This shape has faulty shapes", i.Eval("checkshape s -short"));
        Assert.Equal("This shape seems to be valid", i.Eval("checkshape -top s"));
    }

    [Fact]
    public void Checkshape_structural_dump_counts_the_problems_and_makes_compounds()
    {
        (DrawInterpreter i, _) = Load();
        i.Eval($"restore {DataOcc}/Top.brep s");
        Assert.Equal(" -- The Shape s has problems :\n"
                     + "  Check                                    Count\n"
                     + " ------------------------------------------------\n"
                     + "  Self Intersecting Wire ................... 1\n"
                     + "  Unorientable Shape ....................... 1\n"
                     + " ------------------------------------------------\n"
                     + "*** Shapes with problems : 2\n"
                     + "WIRE\t:  1 Items -> compound named r_w\n"
                     + "FACE\t:  1 Items -> compound named r_f\n",
                     i.Eval("checkshape s r"));
        Assert.Equal(ShapeType.Compound, i.GetShape("r_w").ShapeType());
        Assert.Equal(ShapeType.Face, new Iterator(i.GetShape("r_f")).Value().ShapeType());
        Assert.Equal("0", i.Eval("isdraw faulty_1"));
    }

    [Fact]
    public void Checkshape_usage_and_errors_are_Draws()
    {
        (DrawInterpreter i, _) = Load();
        Assert.StartsWith("\nUsage : checkshape [-top] shape [result] [-short] [-parallel] [-exact]\n\nWhere :\n", i.Eval("checkshape"),
                          StringComparison.Ordinal);
        Assert.Equal("Invalid number of args!!!\nNo args to have help.\n", Error(i, "checkshape -top"));
        Assert.Equal("Invalid number of args!!!\nNo args to have help.\n", Error(i, "checkshape a b c d e f g"));
        Assert.Equal("nothing is not a topological shape!!!\n", Error(i, "checkshape nothing"));
        i.Eval($"restore {DataOcc}/face.brep f");
        Assert.Equal("Syntax error at '-fast'", Error(i, "checkshape f -FAST"));
    }

    // -------------------------------------------------------------------------------------------------------------
    // checksection, computetolerance

    [Fact]
    public void Checksection_counts_and_binds_the_lone_vertices()
    {
        (DrawInterpreter i, _) = Load();
        i.Eval($"restore {DataOcc}/face.brep f");
        Assert.Equal(" nb alone Vertices : 0\n\n", i.Eval("checksection f"));
        Assert.Equal(" nb alone Vertices : 0\nSection is OK\n\n", i.Eval("checksection f -r 0"));
        Assert.Equal(" nb alone Vertices : 0\nError: 2 vertices are expected but 0 are found.\n\n", i.Eval("checksection f -r 2"));
        Assert.Equal("Error: Wrong option-x\n nb alone Vertices : 0\n\n", i.Eval("checksection f -x"));

        // a single open edge: both its vertices are alone
        i.Eval($"restore {DataOcc}/edge.brep e");
        Assert.Equal(" nb alone Vertices : 2\nSection is OK\nalone_1 alone_2 \n", i.Eval("checksection e -r 2"));
        Assert.StartsWith("shape VERTEX", i.Eval("dtyp alone_2"), StringComparison.Ordinal);
        Assert.Equal("checksection shape [-r <ref_val>]\n", Error(i, "checksection"));
    }

    [Fact]
    public void Computetolerance_prints_each_edge_once()
    {
        (DrawInterpreter i, _) = Load();
        i.Eval($"restore {DataOcc}/face.brep f");
        string aText = i.Eval("computetolerance f");
        Assert.Matches(new Regex(@"^(Tolerance de [0-9A-F]{16} : [-+.0-9e]+\n){6}\n$"), aText);
        i.Eval("explode f e");
        Assert.Matches(new Regex(@"^Tolerance de [0-9A-F]{16} : [-+.0-9e]+\n$"), i.Eval("computetolerance f_1"));
        Assert.Equal("Usage: computetolerance shape\n", Error(i, "computetolerance"));
    }

    // -------------------------------------------------------------------------------------------------------------
    // bounding, isbbinterf, gbounding, getcoords

    [Fact]
    public void Bounding_prints_saves_and_parses_as_Draw()
    {
        (DrawInterpreter i, StringWriter anOutput) = Load();
        i.Eval($"restore {DataOcc}/face.brep f");
        Assert.Equal("97.071609567319626 95.770603892871094 -1.7000001 340.39899437555101 159.17060399287112 1.7000001\n", i.Eval("bounding f"));
        Assert.Equal("Axes-aligned bounding box\nX-range: 97.071609567319626 340.39899437555101\nY-range: 95.770603892871094 159.17060399287112\n"
                     + "Z-range: -1.7000001 1.7000001\n", i.Eval("bounding f -dump -nodraw"));

        // -save binds Draw numbers and prints nothing
        Assert.Equal("", i.Eval("bounding f -save x0 y0 z0 x1 y1 z1"));
        Assert.Equal("97.071609567319626", i.Eval("dval x0"));
        Assert.Equal("1.7000001", i.Eval("dval z1"));

        // the box given by its corners
        Assert.Equal("Axes-aligned bounding box\nX-range: 0 1\nY-range: -2 2\nZ-range: 3 4.5\n", i.Eval("bounding 0 -2 3 1 2 4.5abc -print"));

        // OBB
        Assert.StartsWith("Oriented bounding box\nCenter: 218.73530197143532 127.47060394287111 0\n", i.Eval("bounding f -obb"), StringComparison.Ordinal);

        // errors go to Message::SendFail
        anOutput.GetStringBuilder().Clear();
        Assert.Equal("", Error(i, "bounding"));
        Assert.Equal("Syntax error: input is not specified (neither shape nor coordinates)\n", anOutput.ToString());
        anOutput.GetStringBuilder().Clear();
        Error(i, "bounding f -what");
        Assert.Equal("Syntax error at argument '-what'\n", anOutput.ToString());
        anOutput.GetStringBuilder().Clear();
        Error(i, "bounding 0 0 0 1 1 1 -obb");
        Assert.Equal("Syntax error: Options -obb, -optimal and -extToler cannot be used for explicitly defined AABB\n", anOutput.ToString());
        anOutput.GetStringBuilder().Clear();
        Error(i, "bounding f -obb -save a b c d e g");
        Assert.Equal("Error: Option -save works only with axes-aligned boxes\n", anOutput.ToString());

        // -shape makes the box as a solid (unit 3M); what needs unported code says so
        i.Eval("bounding f -shape b");
        Assert.StartsWith("b is a shape SOLID", i.Eval("whatis b"));
        Assert.Equal("bounding -dumpJson (Bnd_Box::DumpJson) is not ported yet\n", Error(i, "bounding f -dumpJson"));
    }

    [Fact]
    public void Bounding_of_an_empty_compound_is_void()
    {
        (DrawInterpreter i, _) = Load();
        i.Eval("compound c");
        Assert.Equal("Void box.\n", i.Eval("bounding c"));
        Assert.Equal("Void box.\n", i.Eval("bounding c -obb"));
        Assert.Equal("Void box.\n", i.Eval("bounding c -shape b"));
        Assert.True(i.GetShape("b").IsNull);
    }

    [Fact]
    public void Isbbinterf_gbounding_and_getcoords()
    {
        (DrawInterpreter i, _) = Load();
        i.Eval($"restore {DataOcc}/face.brep f; restore {DataOcc}/solid.brep s; restore {DataOcc}/wing.brep w");
        Assert.Equal("The shapes are interfered by AABB.\n", i.Eval("isbbinterf s s"));
        Assert.Equal("The shapes are NOT interfered by AABB.\n", i.Eval("isbbinterf f w"));
        Assert.Equal("The shapes are NOT interfered by OBB.\n", i.Eval("isbbinterf f w -o"));
        Assert.Equal("Use: isbbinterf shape1 shape2 [-o].\n", Error(i, "isbbinterf f"));

        i.Eval($"brestore {TestData}/brep/bug12800_c.draw c");
        Assert.Equal(6, i.Eval("gbounding c").Split(' ').Length);
        Assert.Equal(6, i.Eval("gbounding c -o").Split(' ').Length);
        Assert.Equal("Wrong argument \n", Error(i, "gbounding f"));

        i.Eval("explode f v");
        Assert.Matches(new Regex(@"^f_1 \(x,y,z\) : \S+ \S+ \S+\n$"), i.Eval("getcoords f_1 f"));
    }

    // -------------------------------------------------------------------------------------------------------------
    // bclassify, b2dclassify, point

    [Fact]
    public void Bclassify_and_b2dclassify_print_the_state()
    {
        (DrawInterpreter i, _) = Load();
        i.Eval($"restore {DataOcc}/solid.brep s");
        List<double> aBox = i.Eval("bounding s").Split(' ').Select(x => double.Parse(x, System.Globalization.CultureInfo.InvariantCulture)).ToList();
        Assert.Equal("The point is OUT of shape\n", i.Eval($"point p {aBox[3] + 1} 0 0; bclassify s p"));
        // without a point of that name, Draw's default point (8, 9, 10), which is inside
        Assert.Equal("The point is IN shape\n", i.Eval("bclassify s nothing"));
        Assert.Equal(" Shape type must be SOLID\n", Error(i, $"restore {DataOcc}/face.brep f; bclassify f p"));
        Assert.Equal(" Null Shape is not allowed\n", Error(i, "bclassify nothing p"));
        Assert.Equal(" use bclassify Solid Point [Tolerance=1.e-7]\n", Error(i, "bclassify s"));

        // a 2D point in the face's parametric space; a 3D point leaves the default (8, 9)
        i.Eval($"restore {DataOcc}/face.brep f");
        Assert.Equal("The point is OUT of shape\n", i.Eval("point q -1e6 -1e6; b2dclassify f q"));
        Assert.Matches(new Regex("^The point is (IN|OUT of|ON) shape\n$"), i.Eval("b2dclassify f p 1e-7 1 0.2"));
        Assert.Equal(" Shape type must be FACE\n", Error(i, "b2dclassify s q"));
        Assert.Equal(" use b2dclassify Face Point2d [Tol] [UseBox] [GapCheckTol]\n", Error(i, "b2dclassify f"));
        Assert.Equal("", Error(i, "point p 1"));
    }

    // -------------------------------------------------------------------------------------------------------------
    // dchrono, dlog and the command log

    [Fact]
    public void Dchrono_shows_as_OSD_Timer_and_chrono_writes_to_the_output()
    {
        (DrawInterpreter i, StringWriter anOutput) = Load();
        i.Eval("dchrono t reset; dchrono t start; dchrono t stop");
        Assert.Matches(new Regex(@"^Elapsed time: 0 Hours 0 Minutes [-+.0-9e]+ Seconds\nCPU user time: [-+.0-9e]+ seconds\nCPU system time: [-+.0-9e]+ seconds\n$"),
                       i.Eval("dchrono t show"));
        Assert.Equal("chronometer", i.Eval("dtyp t"));
        Assert.Matches(new Regex(@"^[-+.0-9e]+ [-+.0-9e]+$"), i.Eval("dchrono t -elapsed -cpu"));
        Assert.Equal("", i.Eval("chrono t show"));
        Assert.StartsWith("Elapsed time: ", anOutput.ToString(), StringComparison.Ordinal);
        Assert.Equal("Chronometers activated.\n", i.Eval("dchrono 1"));
        Assert.Equal("Syntax error at 'go'\n", Error(i, "dchrono t go"));

        // the harness's own timing pattern (heal/checkshape/begin)
        Assert.Equal("1", i.Eval("regexp {Elapsed time: +([-0-9.+eE]+) Hours +([-0-9.+eE]+) Minutes +([-0-9.+eE]+) Seconds} [dchrono t show] full h m s"));
    }

    [Fact]
    public void The_command_log_has_each_commands_words_and_result()
    {
        (DrawInterpreter i, StringWriter anOutput) = Load();
        i.Eval($"restore {DataOcc}/face.brep f");
        Assert.Equal("off", i.Eval("dlog status"));
        anOutput.GetStringBuilder().Clear();
        i.Eval("dlog on; dlog reset; checkshape f; set x 1; puts hello; catch {checkshape nothing}");
        Assert.Equal("checkshape f \nThis shape seems to be valid\nhello\ncheckshape nothing \nnothing is not a topological shape!!!\n\n",
                     anOutput.ToString());
        Assert.Equal(anOutput.ToString(), i.Eval("dlog get"));
        i.Eval("dlog add {a line}; dlog off");
        Assert.EndsWith("a line\n", anOutput.ToString(), StringComparison.Ordinal);
        Assert.Equal("off", i.Eval("dlog status"));
        anOutput.GetStringBuilder().Clear();
        i.Eval("checkshape f");
        Assert.Equal("", anOutput.ToString());
        Assert.Equal("", Error(i, "dlog"));
    }
}
