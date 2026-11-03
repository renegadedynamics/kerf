// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.DrawLite;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// Draw's shape and variable commands (unit 3L1) on OCCT's data/occ files. The text each prints is Draw's, derived from
/// DBRep.cxx, Draw_VariableCommands.cxx, BRepTest_BasicCommands.cxx, BRepTest_GPropCommands.cxx and
/// SWDRAW_ShapeAnalysis.cxx; the values are those the oracle tests (DrawLiteShapesOracleTests) find bit-identical to
/// native OCCT's.
/// </summary>
public class DrawShapeCommandsTests
{
    private static string DataOcc => Tcl(Path.Combine(DrawResourcesTests.OcctSourceDirectory, "data", "occ"));

    private static string TestData => Tcl(DrawResourcesTests.OcctTestDataDirectory).TrimEnd('/');

    private static string Tcl(string thePath) => thePath.Replace('\\', '/');

    private static (DrawInterpreter Interpreter, StringWriter Output) Load()
    {
        var anOutput = new StringWriter { NewLine = "\n" };
        var anInterpreter = new DrawInterpreter(anOutput);
        anInterpreter.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        return (anInterpreter, anOutput);
    }

    private static string Error(DrawInterpreter theDI, string theScript) => Assert.ThrowsAny<DrawException>(() => theDI.Eval(theScript)).Reason;

    private const string FaceExtent = " VERTEX    : 6\n EDGE      : 6\n WIRE      : 3\n FACE      : 1\n SHELL     : 0\n SOLID     : 0\n"
                                      + " COMPSOLID : 0\n COMPOUND  : 0\n SHAPE     : 16\n";

    // -------------------------------------------------------------------------------------------------------------
    // Variables

    [Fact]
    public void Restore_binds_a_frozen_shape_to_the_variable()
    {
        (DrawInterpreter i, _) = Load();
        Assert.Equal("f", i.Eval($"restore {DataOcc}/face.brep f"));
        Assert.Equal("1", i.Eval("isdraw f"));
        Assert.Equal("0", i.Eval("isdraw nothing"));
        Assert.True(i.GetShape("f").TShape()!.IsFrozen);

        // as in Draw, the Tcl variable's value is the name given to Draw::Set: restore's upvar'ed "n"
        Assert.Equal("n", i.Eval("set f"));
        Assert.Equal("n", i.Eval("dname f"));
        Assert.Equal("shape FACE REVERSED Modified Orientable", i.Eval("dtyp f"));
        Assert.Equal("f is a shape FACE REVERSED Modified Orientable\n", i.Eval("whatis f"));

        // the file name's root is the default name
        Assert.Equal("face", i.Eval($"restore {DataOcc}/face.brep"));
        Assert.Equal("1", i.Eval("isdraw face"));
    }

    [Fact]
    public void Writing_or_unsetting_the_variable_drops_the_drawable()
    {
        (DrawInterpreter i, _) = Load();
        i.Eval($"restore {DataOcc}/face.brep f; restore {DataOcc}/face.brep g");
        i.Eval("set f 1");
        Assert.Equal("0", i.Eval("isdraw f"));
        i.Eval("unset g");
        Assert.Equal("0", i.Eval("isdraw g"));
    }

    [Fact]
    public void Drawables_follow_Tcls_frames_and_upvar()
    {
        (DrawInterpreter i, _) = Load();
        i.Eval($"restore {DataOcc}/face.brep f");
        // a procedure sees the caller's variable through upvar, and its own explode results are local
        Assert.Equal("1 6 0", i.Eval("proc p {name} { upvar $name s; set n [llength [explode s e]]; list [isdraw s] $n [isdraw nothing] }; p f"));
        Assert.Equal("0", i.Eval("isdraw s_1"));
        Assert.Equal("0", i.Eval("proc q {} { isdraw f }; q"));
        Assert.Equal("1", i.Eval("proc r {} { global f; isdraw f }; r"));

        // array elements hold drawables too
        i.Eval("explode f w; copy f_1 a(1)");
        Assert.Equal("shape WIRE REVERSED Modified Orientable Closed", i.Eval("dtyp a(1)"));
        i.Eval("set a(1) x");
        Assert.Equal("0", i.Eval("isdraw a(1)"));
    }

    [Fact]
    public void Draw_numbers_and_Atof()
    {
        (DrawInterpreter i, _) = Load();
        Assert.Equal("5", i.Eval("dset a 1 b 2+3"));
        Assert.Equal("numeric", i.Eval("dtyp a"));
        Assert.Equal("11", i.Eval("dval a+b*2"));
        Assert.Equal("-5", i.Eval("dval -(a+b)+2/2"));
        Assert.Equal("1.5707963267948966", i.Eval("dval pi/2"));
        Assert.Equal("9.9999999999999995e-08", i.Eval("dval 1e-7"));
        Assert.Equal("0.90929742682568171", i.Eval("dval sin(2)"));
        Assert.Equal("0", i.Eval("dval unknown"));
        Assert.Equal("1", i.Eval("isdraw pi"));
        Assert.Equal("1", i.Eval("isprot pi"));
        Assert.Equal("can't set \"pi\": variable is protected", Error(i, "set pi 3"));
        Assert.Equal("a ", i.Eval("protect a"));
        Assert.Equal("1", i.Eval("isprot a"));
        Assert.Equal("a ", i.Eval("unprotect a"));
        Assert.Equal(1.7976931348623157e308, i.Atof("pinf"));
    }

    [Fact]
    public void Copy_and_renamevar()
    {
        (DrawInterpreter i, _) = Load();
        i.Eval($"restore {DataOcc}/face.brep f");
        Assert.Equal("", i.Eval("copy f g"));
        Assert.Equal("g", i.Eval("dname g"));
        Assert.Equal(i.GetShape("f"), i.GetShape("g"));
        i.Eval("renamevar g h");
        Assert.Equal("0", i.Eval("isdraw g"));
        Assert.Equal("h", i.Eval("dname h"));
    }

    [Fact]
    public void Dump_prints_the_shape_set()
    {
        (DrawInterpreter i, _) = Load();
        i.Eval($"restore {DataOcc}/face.brep f");
        string aDump = i.Eval("dump f");
        Assert.StartsWith("\n\n*********** Dump of f *************\nShape : 16, REVERSED, location : 3\n\nDump of 16 TShapes", aDump);
    }

    // -------------------------------------------------------------------------------------------------------------
    // Files

    [Fact]
    public void Brestore_errors_are_Draws()
    {
        (DrawInterpreter i, _) = Load();
        Assert.Equal("Error: cannot open file for reading: '/no/such.brep'", Error(i, "brestore /no/such.brep a"));
        Assert.Equal("", Error(i, "brestore a"));
        Assert.Equal("Error: cannot read the file '/no/such.brep'", Error(i, "readbrep /no/such.brep a"));
        Assert.Equal("Syntax error: wrong number of arguments", Error(i, "readbrep a"));
    }

    [Fact]
    public void Geometry_files_restore_as_DrawTrSurf_drawables()
    {
        (DrawInterpreter i, _) = Load();
        i.Eval($"brestore {TestData}/brep/bug12800_c.draw c");
        Assert.Equal(" a 3d curve", i.Eval("dtyp c"));
        Assert.Equal("c is a  a 3d curve\n", i.Eval("whatis c"));
        i.Eval($"brestore {TestData}/brep/bug12800_s.draw su");
        Assert.Equal("a surface", i.Eval("dtyp su"));
        i.Eval($"brestore {TestData}/brep/bug26609_p.draw p");
        Assert.Equal("point", i.Eval("dtyp p"));
        Assert.Equal("\n\n*********** Dump of p *************\nPoint : -1.357216400000000e+01, 3.000000000000000e+02, 6.000000000000000e+01\n",
                     i.Eval("dump p"));

        // saved and restored through the overlay
        Assert.Equal("p", i.Eval("bsave p C:/virtual/p.draw"));
        i.Eval("brestore C:/virtual/p.draw p2");
        Assert.Equal(i.Eval("dump p").Replace("of p ", "of p2 "), i.Eval("dump p2"));
        i.Eval("bsave su C:/virtual/su.draw; brestore C:/virtual/su.draw su2");
        Assert.Equal(i.Eval("dump su").Replace("of su ", "of su2 "), i.Eval("dump su2"));
    }

    [Fact]
    public void Shapes_and_numbers_round_trip_through_every_format()
    {
        (DrawInterpreter i, _) = Load();
        i.Eval($"restore {DataOcc}/face.brep f");
        string aProps = i.Eval("sprops f -full");

        Assert.Equal("f", i.Eval("bsave f C:/virtual/f.brep"));
        Assert.Equal("f1", i.Eval("brestore C:/virtual/f.brep f1"));
        Assert.Equal("f", i.Eval("writebrep f C:/virtual/f2.brep"));
        Assert.Equal("f2", i.Eval("readbrep C:/virtual/f2.brep f2"));
        Assert.Equal("f", i.Eval("binsave f C:/virtual/f.bin"));
        Assert.Equal("f3", i.Eval("binrestore C:/virtual/f.bin f3"));
        Assert.Equal("f", i.Eval("writebrep f C:/virtual/f4.bin -binary -version 2"));
        Assert.Equal("f4", i.Eval("readbrep C:/virtual/f4.bin f4"));
        Assert.Equal("f", i.Eval("writebrep f C:/virtual/f5.brep -version 1 -notriangles"));
        Assert.Equal("f5", i.Eval("brestore C:/virtual/f5.brep f5"));
        foreach (string aName in new[] { "f1", "f2", "f3", "f4", "f5" })
        {
            Assert.Equal($"Number of shapes in {aName}\n" + FaceExtent, i.Eval($"nbshapes {aName}"));
            Assert.Equal(aProps, i.Eval($"sprops {aName} -full"));
            Assert.True(i.GetShape(aName).TShape()!.IsFrozen);
        }

        // the text file starts as Draw's (DBRep_DrawableShape, then the shape set), bsave's ends with Draw's "0"
        Assert.StartsWith("DBRep_DrawableShape\n\nCASCADE Topology V3, (c) Open Cascade\nLocations ", i.Eval("read [open C:/virtual/f2.brep]"));
        // the shape (reversed, the first TShape counted from the end, at location 3), then Draw's end, as data/occ's files
        Assert.EndsWith("*\n\n-1 3 \n0\n\n", i.Eval("read [open C:/virtual/f.brep]"));

        i.Eval("dset n 2.5; bsave n C:/virtual/n");
        Assert.Equal("Draw_Number\n         2.500000000000000e+00\n\n0\n\n", i.Eval("read [open C:/virtual/n]"));
        i.Eval("brestore C:/virtual/n n2");
        Assert.Equal("2.5", i.Eval("dval n2"));

        Assert.Equal("Syntax error: unknown format version", Error(i, "writebrep f C:/virtual/x -version 7"));
        Assert.Equal("Error: vertex normals require ascii format version 3 or later", Error(i, "writebrep f C:/virtual/x -version 1 -normals"));
        Assert.Equal("Error: vertex normals require binary format version 4 or later", Error(i, "binsave f C:/virtual/x -version 3 -normals"));
        Assert.Equal("Syntax error: nothing is not a shape", Error(i, "writebrep nothing C:/virtual/x"));
        Assert.Equal("Syntax error: wrong number of arguments", Error(i, "writebrep f"));
        Assert.Equal("Syntax error: '1' is not a drawable", Error(i, "bsave 1 C:/virtual/x"));
    }

    // -------------------------------------------------------------------------------------------------------------
    // Topology

    [Fact]
    public void Nbshapes_numshapes_and_countshapes()
    {
        (DrawInterpreter i, _) = Load();
        i.Eval($"restore {DataOcc}/face.brep f");
        Assert.Equal("Number of shapes in f\n" + FaceExtent, i.Eval("nbshapes f"));
        Assert.Equal("Number of shapes in f\n" + FaceExtent, i.Eval("nbshapes f -t"));
        Assert.Equal("Number of shapes in f\n" + FaceExtent + "\n", i.Eval("numshapes f"));
        Assert.Equal("Number of shapes in f\n VERTEX     : 6\n EDGE       : 6\n WIRE       : 3\n FACE       : 1\n SHELL      : 0\n"
                     + " SOLID      : 0\n COMPSOLID  : 0\n COMPOUND   : 0\n SHAPE      : 16\n\n", i.Eval("countshapes f"));
        Assert.Equal("", i.Eval("nbshapes nothing"));
        Assert.Equal("", Error(i, "nbshapes"));
    }

    [Fact]
    public void Explode_names_the_sub_shapes()
    {
        (DrawInterpreter i, _) = Load();
        i.Eval($"restore {DataOcc}/face.brep f");
        Assert.Equal("f_1 f_2 f_3", i.Eval("explode f"));
        Assert.Equal("shape WIRE REVERSED Modified Orientable Closed", i.Eval("dtyp f_1"));
        Assert.Equal("f_1 f_2 f_3 f_4 f_5 f_6", i.Eval("explode f e"));
        Assert.Equal("f_1 f_2 f_3 f_4 f_5 f_6", i.Eval("explode f V"));
        // the shape itself is never one of its sub-shapes
        Assert.Equal("", i.Eval("explode f F"));
        Assert.Equal("", i.Eval("explode f So"));
        Assert.Equal("", Error(i, "explode f S"));
        Assert.Equal("", Error(i, "explode f Z"));
        Assert.Equal("", i.Eval("explode nothing f"));

        // nexplode orders by the centre of mass
        Assert.Equal("f_1 f_2 f_3 f_4 f_5 f_6", i.Eval("nexplode f e"));
        string[] aKeys = [.. Enumerable.Range(1, 6).Select(k => i.Eval($"lprops f_{k} x y z; expr [dval x]*999 + [dval y]*99 + [dval z]*0.9"))];
        Assert.Equal(aKeys.Select(double.Parse).Order(), aKeys.Select(double.Parse));
        Assert.Equal("", Error(i, "nexplode f w"));

        // OCCT sizes nexplode's array one short when the shape is of the type (occt-issues.md)
        Assert.StartsWith("An exception was caught Standard_OutOfRange", Error(i, "nexplode f_1 e"));

        // exwire: the edges of a wire in order, named without a result
        i.Eval("explode f w");
        Assert.Equal("", i.Eval("exwire f_1"));
        Assert.Equal("shape EDGE REVERSED Modified Orientable", i.Eval("dtyp f_1_1"));
    }

    [Fact]
    public void Compound_add_and_emptycopy_bind_new_shapes()
    {
        (DrawInterpreter i, _) = Load();
        i.Eval($"restore {DataOcc}/face.brep f; restore {DataOcc}/solid.brep s");
        Assert.Equal("", i.Eval("compound f s nothing c"));
        Assert.Equal("shape COMPOUND FORWARD Free Modified", i.Eval("dtyp c"));
        Assert.Contains(" SHAPE     : 60\n", i.Eval("nbshapes c"));
        Shape aBefore = i.GetShape("c");

        // add what where: a new compound with one more child; the old one is unchanged
        i.Eval("compound c2; add f c2; add s c2");
        Assert.Equal(2, i.GetShape("c2").ChildCount);
        i.Eval("add f c");
        Assert.Equal(3, i.GetShape("c").ChildCount);
        Assert.Equal(2, aBefore.ChildCount);
        Assert.True(i.GetShape("c").TShape()!.IsFrozen);

        // a sub-shape isn't free: OCCT's builder refuses it
        i.Eval("explode f e");
        Assert.Equal("An exception was caught TopoDS_FrozenShape: TopoDS_Builder::Add", Error(i, "add f f_1"));

        Assert.Equal("", i.Eval("emptycopy e c"));
        Assert.Equal(0, i.GetShape("e").ChildCount);
        Assert.Equal("", i.Eval("emptycopy f"));
        Assert.Equal(0, i.GetShape("f").ChildCount);
    }

    [Fact]
    public void Orientation_commands()
    {
        (DrawInterpreter i, _) = Load();
        i.Eval($"restore {DataOcc}/face.brep f");
        i.Eval("orientation f F");
        Assert.Equal("shape FACE FORWARD Modified Orientable", i.Eval("dtyp f"));
        i.Eval("orientation f I");
        Assert.Equal("shape FACE INTERNAL Modified Orientable", i.Eval("dtyp f"));
        i.Eval("complement f");
        Assert.Equal("shape FACE EXTERNAL Modified Orientable", i.Eval("dtyp f"));
        i.Eval("orientation f R; treverse f");
        Assert.Equal("shape FACE FORWARD Modified Orientable", i.Eval("dtyp f"));

        // invert reverses the sub-shapes in a new TShape
        i.Eval($"restore {DataOcc}/solid.brep s");
        Assert.Contains("Mass :          259982\n", i.Eval("vprops s"));
        i.Eval("invert s");
        Assert.Contains("Mass :         -259982\n", i.Eval("vprops s"));
        Assert.Equal("shape SOLID FORWARD Free Modified", i.Eval("dtyp s"));
    }

    [Fact]
    public void Setflags_binds_a_copy_with_the_flags()
    {
        (DrawInterpreter i, _) = Load();
        i.Eval($"restore {DataOcc}/face.brep f");
        Shape aBefore = i.GetShape("f");
        i.Eval("setflags f free -orientable closed convex infinite");
        Assert.Equal("shape FACE REVERSED Free Modified Closed Infinite Convex", i.Eval("dtyp f"));
        i.Eval("explode f v");
        Assert.Equal("shape VERTEX REVERSED Free Modified Closed Infinite Convex", i.Eval("dtyp f_1"));
        Assert.True(i.GetShape("f").TShape()!.IsFrozen);

        // the shape bound before is unchanged
        i.SetShape("g", aBefore);
        Assert.Equal("shape FACE REVERSED Modified Orientable", i.Eval("dtyp g"));
        Assert.Contains(" SHAPE     : 16\n", i.Eval("nbshapes f"));
        Assert.Equal("", Error(i, "setflags f"));
    }

    // -------------------------------------------------------------------------------------------------------------
    // Properties and tolerances

    [Fact]
    public void Props_print_as_BRepTest_GPropCommands()
    {
        (DrawInterpreter i, _) = Load();
        i.Eval($"restore {DataOcc}/face.brep f");
        Assert.Equal("\n\nMass :         12999.1\n\nCenter of gravity : \nX =         218.674\nY =         127.348\nZ =               0\n\n"
                     + "Matrix of Inertia : \n    3.85625e+06         2107.09               0\n        2107.09     5.40722e+07               0\n"
                     + "              0               0     5.79285e+07\n\nMoments : \nIX =     5.79285e+07\nIY =     5.40722e+07\nIZ =     3.85625e+06\n\n",
                     i.Eval("sprops f"));
        Assert.Equal("\n\nMass : 12999.114857512877\n\nCenter of gravity : \nX = 218.67417217792138\nY = 127.34834435584233\nZ = 0\n\n"
                     + "Matrix of Inertia :\n3856250.3618266638    2107.093628667295    0\n2107.093628667295    54072241.348124117    0\n"
                     + "0    0    57928491.709950805\n\nMoments :\nIX = 57928491.709950805\nIY = 54072241.436539054\nIZ = 3856250.2734117294\n\n",
                     i.Eval("sprops f -full"));
        Assert.StartsWith("\n\nMass :         12999.1\n\nRelative error of mass computation :     1.09902e-16\n\nCenter of gravity : \n", i.Eval("sprops f 1e-4"));
        Assert.StartsWith("\n\nMass :         674.159\n\nCenter of gravity : \n", i.Eval("lprops f 1e-4"));
        Assert.StartsWith("\n\nMass :               0\n\n", i.Eval("vprops f"));

        // x y z: the centre as Draw numbers
        i.Eval("lprops f x y z");
        Assert.Equal("218.97104247014002 127.9420849402799 0", i.Eval("list [dval x] [dval y] [dval z]"));
        Assert.Equal("x", i.Eval("set x"));

        Assert.Equal("", i.Eval("vprops nothing"));
        Assert.StartsWith("Use: sprops shape [epsilon] [c[losed]] [x y z] [-skip] [-full] [-tri]\n", Error(i, "sprops"));
        Assert.Equal("", Error(i, "sprops f 1e-20"));
    }

    [Fact]
    public void Vpropsgk_prints_at_precision_15()
    {
        (DrawInterpreter i, _) = Load();
        i.Eval($"restore {DataOcc}/solid.brep s");
        string aText = i.Eval("vpropsgk s 1e-3 0 0 3");
        Assert.Matches(@"^\n\nMass : {9}259982\.[0-9]+\n\nRelative error of mass computation : +[-0-9.e]+\n\nCenter of gravity : \nX = +118\.674", aText);
        Assert.Contains("Moments : \nIX = ", aText);
        Assert.Equal("Wrong arguments\n", Error(i, "vpropsgk s 1"));
    }

    [Fact]
    public void Tolerance_and_maxtolerance()
    {
        (DrawInterpreter i, _) = Load();
        i.Eval($"restore {DataOcc}/face.brep f");
        const string aTol = "MAX=9.9999999999999995e-08 AVG=9.9999999999999995e-08 MIN=9.9999999999999995e-08\n";
        Assert.Equal($"Tolerance {aTol}FACE    : {aTol}EDGE    : {aTol}VERTEX  : {aTol}", i.Eval("tolerance f"));
        Assert.Equal("\n## Tolerances on the shape f  (nbFaces:1  nbEdges:6 nbVtx:6)\n\n    Face   : Min 1.00000e-07    Max  1.00000e-07 \n "
                     + "\n    Edge   : Min 1.00000e-07    Max  1.00000e-07 \n \n    Vertex : Min 1.00000e-07    Max  1.00000e-07 \n ",
                     i.Eval("maxtolerance f"));
        Assert.Equal("Analysing Edges gives 6 Shapes over tol=1e-08 , named tol_1 to tol_6\n", i.Eval("tolerance f e 1e-8"));
        Assert.Equal("shape EDGE REVERSED Modified Orientable", i.Eval("dtyp tol_1"));
        Assert.Equal("Analysing Faces gives 1 Shapes between tol1=1e-08 and tol2=1 , named tol_1\n", i.Eval("tolerance f f 1e-8 1"));
        // OCCT tests vertices against the upper bound with >= (occt-issues.md): the 12 vertex occurrences are "below"
        Assert.Equal("Analysing all sub-shapes gives 12 Shapes below tol=1e-08 , named tol_1 to tol_12\n", i.Eval("tolerance f 0 1e-8"));
        Assert.Equal("Shape unknown : x\n", Error(i, "tolerance x"));
        Assert.Equal("", Error(i, "maxtolerance x"));
    }

    [Fact]
    public void OCCTs_check_procedures_run_on_the_commands()
    {
        (DrawInterpreter i, StringWriter aLog) = Load();
        i.Eval($"restore {DataOcc}/solid.brep s");
        i.Eval("checknbshapes s -vertex 8 -edge 14 -wire 11 -face 8 -shell 1 -solid 1 -compsolid 0 -compound 0 -shape 43");
        i.Eval("checkprops s -v 259982 -s 39481.4 -l 2936.64");
        Assert.Equal("1.5707963267949001", i.Eval("checkmaxtol s -ref 1.5707963267949"));
        string aText = aLog.ToString();
        Assert.DoesNotContain("Error", aText);
        Assert.Contains("OK :  is GOOD because number of SHAPE entities is equal to number of expected SHAPE entities", aText);

        aLog.GetStringBuilder().Clear();
        i.Eval("checkprops s -v 1000");
        Assert.Contains("Error : The volume of result shape is 259982, expected 1000", aLog.ToString());
    }

    // -------------------------------------------------------------------------------------------------------------
    // Transformations and locations

    [Fact]
    public void Transformations_move_scale_and_mirror()
    {
        (DrawInterpreter i, StringWriter aLog) = Load();
        i.Eval($"restore {DataOcc}/solid.brep s");
        Shape anOriginal = i.GetShape("s");

        // a rigid move only locates the shape
        i.Eval("ttranslate s 1 2 3");
        Assert.True(i.GetShape("s").IsPartner(anOriginal));
        Assert.Contains("X =         119.674\nY =         29.3483\nZ =              13\n", i.Eval("vprops s"));
        i.Eval("trotate s 0 0 0 0 0 1 90");
        Assert.True(i.GetShape("s").IsPartner(anOriginal));
        i.Eval("reset s");
        // as in OCCT, an elementary location of the identity (TopLoc_Location(gp_Trsf()))
        Assert.Equal(TransformForm.Identity, i.GetShape("s").Location().Transformation().Form());

        // tmove sets the location of another shape
        i.Eval($"restore {DataOcc}/solid.brep m; ttranslate m 5 0 0; tmove s m");
        Assert.Equal(i.GetShape("m").Location().Transformation().TranslationPart(), i.GetShape("s").Location().Transformation().TranslationPart());

        // a scaling or a mirror rebuilds the geometry
        i.Eval("reset s; tscale s 0 0 0 2");
        Assert.False(i.GetShape("s").IsPartner(anOriginal));
        Assert.Contains("Mass :     2.07986e+06\n", i.Eval("vprops s"));
        i.Eval("tmirror s 0 0 0 1 0 0");
        Assert.Contains("X =        -237.348\n", i.Eval("vprops s"));

        // -copy rebuilds a rigid move too
        i.Eval($"restore {DataOcc}/solid.brep c; ttranslate c 1 0 0 -copy");
        Assert.False(i.GetShape("c").IsPartner(anOriginal));

        // b and f forms move by locations, f without checking the scale
        i.Eval($"restore {DataOcc}/solid.brep b");
        Shape aB = i.GetShape("b");
        i.Eval("bscale b 0 0 0 1");
        Assert.True(i.GetShape("b").IsPartner(aB));
        aLog.GetStringBuilder().Clear();
        Assert.Equal("", i.Eval("bscale b 0 0 0 2"));
        Assert.Equal("Operation is not done: scale is not a valid transformation - scale = 2\n", aLog.ToString());
        Assert.Equal("", i.Eval("fscale b 0 0 0 2"));
        Assert.Equal(2.0, i.GetShape("b").Location().Transformation().ScaleFactor());

        aLog.GetStringBuilder().Clear();
        Assert.Equal("", Error(i, "ttranslate nothing 1 2 3"));
        Assert.Equal("Error: nothing is not a valid shape\n", aLog.ToString());
        Assert.Equal("", Error(i, "ttranslate s 1"));
        Assert.Equal("An exception was caught Standard_ConstructionError: gp_Trsf::SetScaleFactor", Error(i, "tscale s 0 0 0 0"));
    }

    [Fact]
    public void Tcopy_copies_the_shapes()
    {
        (DrawInterpreter i, StringWriter aLog) = Load();
        i.Eval($"restore {DataOcc}/face.brep f; restore {DataOcc}/solid.brep s");
        Assert.Equal("f2 ", i.Eval("tcopy f f2"));
        Assert.False(i.GetShape("f2").IsPartner(i.GetShape("f")));
        Assert.Equal(i.Eval("sprops f -full"), i.Eval("sprops f2 -full"));

        // OCCT steps through the pairs one word at a time (occt-issues.md): f is copied to f3, then f3 to s
        Assert.Equal("f3 s ", i.Eval("tcopy f f3 s s2"));
        Assert.Equal("0", i.Eval("isdraw s2"));
        Assert.Equal("shape FACE REVERSED Free Modified Orientable", i.Eval("dtyp s"));
        Assert.Equal("g ", i.Eval("tcopy -n -m f g"));
        Assert.Equal("", Error(i, "tcopy f"));
        Assert.StartsWith("Use: tcopy [-n(ogeom)] [-m(esh)] shape1 copy1 [shape2 copy2 [...]]\n", aLog.ToString());
    }

    [Fact]
    public void Purgeloc_and_checkloc()
    {
        (DrawInterpreter i, _) = Load();
        i.Eval($"restore {DataOcc}/solid.brep s");
        Assert.Equal("There are no problematic shapes\n", i.Eval("checkloc s"));
        Assert.Equal("All problematic locations are purged \n", i.Eval("purgeloc p s"));
        Assert.Equal("1", i.Eval("isdraw p"));

        // a scaled location is a problem, and purgeloc removes it
        i.Eval("fscale s 0 0 0 2");
        Assert.Equal("s_1 \n", i.Eval("checkloc s"));
        Assert.Equal("All problematic locations are purged \n", i.Eval("purgeloc p s"));
        Assert.Equal("There are no problematic shapes\n", i.Eval("checkloc p"));
        Assert.Equal("", Error(i, "checkloc nothing"));
    }

    // -------------------------------------------------------------------------------------------------------------
    // Display commands

    [Fact]
    public void Display_settings_print_Draws_values()
    {
        (DrawInterpreter i, StringWriter aLog) = Load();
        Assert.Equal("Current number of isos : 2\n", i.Eval("isos"));
        Assert.Equal("New current number of isos : 5\n", i.Eval("isos 5"));
        Assert.Equal("Current number of isos : 5\n", i.Eval("isos"));
        Assert.Equal("Current number of points : 30\n", i.Eval("discretisation"));
        Assert.Equal("", i.Eval("discretisation 50; triangles; polygons; vconn"));
        Assert.Equal("Current number of points : 50\n", i.Eval("discretisation"));
        Assert.Equal(" wireframe\n", i.Eval("hlr"));
        i.Eval("hlr hid");
        Assert.Equal(" HLR Rg1Lines HiddenLines\nAngle of discretization : 35 degrees\n\n", i.Eval("hlr"));
        Assert.Equal("Angle of discretization : 10 degrees\n", i.Eval("hlr ang 10"));
        Assert.Equal("", i.Eval("vori"));
        Assert.Equal("\nrouge  FORWARD\nbleu   REVERSED\nrose   EXTERNAL\norange INTERNAL\n", aLog.ToString());
    }
}
