// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.HiddenLines.Presentation;
using Kerf.Engine.Meshing;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using Occt;
using OcctNet.DrawLite;

namespace OcctNet.DrawLite.Tests;

/// <summary>
/// Unit 4L2's commands: incmesh (MeshTest.cxx) with its options, the rebinding of the variables that share the meshed
/// sub-shapes (decision 495), tessellate, the mesh factory's commands (mpnames, mpsetdefaultname, mpgetdefaultname,
/// mpincmesh, mpparallel), and the viewer commands that mesh what they display (vdisplay, vsetdispmode, vdefaults).
/// The texts are Draw's, derived from those sources; the meshes are compared with BRepMesh_IncrementalMesh's with the
/// parameters the options stand for, which the oracle (DrawLiteIncMeshOracleTests) compares with native OCCT's.
/// </summary>
[Collection(MeshDefaultsCollection.Name)]
public class DrawMeshingCommandsTests
{
    private static string DataOcc => Path.Combine(DrawResourcesTests.OcctSourceDirectory, "data", "occ").Replace('\\', '/');

    private static (DrawInterpreter Interpreter, StringWriter Output) Load(string? theFile = null)
    {
        var anOutput = new StringWriter { NewLine = "\n" };
        var anInterpreter = new DrawInterpreter(anOutput);
        anInterpreter.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        if (theFile is not null)
        {
            anInterpreter.Eval($"restore {DataOcc}/{theFile} s; tclean s");
        }

        return (anInterpreter, anOutput);
    }

    private static string Error(DrawInterpreter theDI, string theScript) => Assert.ThrowsAny<DrawException>(() => theDI.Eval(theScript)).Reason;

    private static string Text(in Shape theShape)
    {
        var aWriter = new StringWriter { NewLine = "\n" };
        BrepTools.Write(theShape, aWriter);
        return aWriter.ToString();
    }

    private static Shape Meshed(in Shape theShape, ToolParameters theParameters, ToolContext? theContext = null)
    {
        var aMesher = new IncrementalMesh();
        aMesher.SetShape(theShape);
        aMesher.ChangeParameters() = theParameters;
        aMesher.Perform(theContext ?? new MeshContext());
        return aMesher.Shape();
    }

    private static ToolParameters Parameters(double theDeflection)
        => new() { Deflection = theDeflection, DeflectionInterior = theDeflection };

    [Fact]
    public void Incmesh_meshes_the_shape_and_reports_the_statuses()
    {
        (DrawInterpreter i, _) = Load("solid.brep");
        Shape anInput = i.GetShape("s");
        Assert.Equal("Incremental Mesh, multi-threading OFF\nMeshing statuses: NoError", i.Eval("incmesh s 0.1"));
        Assert.Equal("\nThis shape contains 8 faces.\n"
                     + "                    492 triangles.\n"
                     + "                    500 nodes.\n"
                     + "                    92 polygons on triangulation.\n"
                     + "Maximal deflection 0.048152733278053722\n"
                     + "Meshing deflection 0.10000000000000001\n"
                     + "Meshing angular deflection 28.647889756541161\n"
                     + "Meshing min size 0.010000000000000002\n\n",
                     i.Eval("trinfo s"));

        // a copy is meshed (the input is frozen), and bound to the name
        Assert.False(i.GetShape("s").IsSame(anInput));
        Assert.Equal(Text(Meshed(anInput, Parameters(0.1))), Text(i.GetShape("s")));

        // meshing it again finely enough changes nothing (the meshes are reused); -decrease is needed for a coarser mesh
        Shape aMeshed = i.GetShape("s");
        Assert.EndsWith("Meshing statuses: Reused ",i.Eval("incmesh s 0.5"), StringComparison.Ordinal);
        Assert.True(i.GetShape("s").IsSame(aMeshed));
        i.Eval("incmesh s 0.5 -decrease");
        Assert.False(i.GetShape("s").IsSame(aMeshed));

        // the command log: the words, then the result
        i.DoLog = true;
        var aLog = (StringWriter)i.Output;
        aLog.GetStringBuilder().Clear();
        i.Eval("incmesh s 0.5 -parallel");
        Assert.StartsWith("incmesh s 0.5 -parallel \nIncremental Mesh, multi-threading ON\nMeshing statuses: ", aLog.ToString(), StringComparison.Ordinal);
    }

    [Fact]
    public void Each_option_sets_its_parameter()
    {
        (DrawInterpreter i, _) = Load("CrankArm.brep");
        Shape anInput = i.GetShape("s");
        var aCases = new (string Options, ToolParameters Parameters, ToolContext? Context)[]
        {
            ("0.5", Parameters(0.5), null),
            ("0.5 -a 10", Parameters(0.5) with { Angle = 10 * Math.PI / 180 }, null),
            ("-angular 10 0.5", Parameters(0.5) with { Angle = 10 * Math.PI / 180 }, null),
            ("0.01 -relative", Parameters(0.01) with { Relative = true }, null),
            ("0.01 -relative 1 -norelative", Parameters(0.01), null),
            ("0.5 -parallel", Parameters(0.5) with { InParallel = true }, null),
            ("0.5 -min 0.2", Parameters(0.5) with { MinSize = 0.2 }, null),
            ("0.5 -di 0.1", Parameters(0.5) with { DeflectionInterior = 0.1 }, null),
            ("0.5 -ai 15", Parameters(0.5) with { AngleInterior = 15 * Math.PI / 180 }, null),
            ("0.5 -surf_def_all", Parameters(0.5) with { EnableControlSurfaceDeflectionAllSurfaces = true }, null),
            ("0.5 -surf_def_off", Parameters(0.5) with { ControlSurfaceDeflection = false }, null),
            ("0.5 -surf_def_off 0", Parameters(0.5), null),
            ("0.5 -int_vert_off", Parameters(0.5) with { InternalVerticesMode = false }, null),
            ("0.5 -force_face_def", Parameters(0.5) with { ForceFaceDeflection = true }, null),
            ("0.5 -adjust_min", Parameters(0.5) with { AdjustMinSize = true }, null),
            ("0.5 -decrease", Parameters(0.5) with { AllowQualityDecrease = true }, null),
            ("0.5 -algo delabella", Parameters(0.5) with { MeshAlgo = MeshAlgoType.MeshAlgoType_Delabella },
                new MeshContext(MeshAlgoType.MeshAlgoType_Delabella)),
            ("0.5 -algo 0", Parameters(0.5) with { MeshAlgo = MeshAlgoType.MeshAlgoType_Watson }, null),
            ("0.5 -algo default", Parameters(0.5), null),
        };

        string aDefault = Text(Meshed(anInput, Parameters(0.5)));
        int aDifferent = 0;
        foreach ((string anOptions, ToolParameters aParameters, ToolContext? aContext) in aCases)
        {
            i.Eval("tclean s");
            i.Eval("incmesh s " + anOptions);
            string anExpected = Text(Meshed(anInput, aParameters, aContext));
            Assert.True(anExpected == Text(i.GetShape("s")), "incmesh s " + anOptions);
            aDifferent += anExpected != aDefault ? 1 : 0;
        }

        // most options change CrankArm's mesh
        Assert.True(aDifferent >= 9, $"{aDifferent} options change the mesh");

        // -prs: AIS's default deflection (relative to the bounding box, 20 degrees); a deflection given is the coefficient
        i.Eval("tclean s; incmesh s -prs");
        var aDrawer = new Drawer();
        double aPrsDeflection = PresentationToolTriangulatedShape.GetDeflection(anInput, aDrawer);
        Assert.Equal(Text(Meshed(anInput, Parameters(aPrsDeflection) with { Angle = 20 * Math.PI / 180 })), Text(i.GetShape("s")));
    }

    [Fact]
    public void Incmesh_errors_are_Draws()
    {
        (DrawInterpreter i, StringWriter o) = Load("solid.brep");
        Assert.Equal("Syntax error: wrong number of arguments", Error(i, "incmesh s"));
        Assert.Equal("Syntax error at 'foo'", Error(i, "incmesh s 0.1 -algo FOO"));
        Assert.Equal("Syntax error: invalid input parameter '0'", Error(i, "incmesh s 0.1 -a 0"));
        Assert.Equal("Syntax error: invalid input parameter '1e-9'", Error(i, "incmesh s 0.1 -min 1e-9"));
        Assert.Equal("Syntax error: null shapes are not allowed here 'x'\n", Error(i, "incmesh s 0.1 x"));
        Assert.Equal("", Error(i, "incmesh 0.1 -relative"));
        Assert.Equal("Syntax error: wrong number of arguments.\n", o.ToString());
    }

    [Fact]
    public void Variables_that_share_the_meshed_sub_shapes_see_the_meshes()
    {
        (DrawInterpreter i, _) = Load("solid.brep");
        i.Eval("explode s f; compound s_1 s_2 c; tcopy s t");
        i.Eval("proc inner {} { global s; upvar #0 s_3 f3; incmesh s 0.1; return [trinfo f3] }");
        string aFace3 = i.Eval("inner");
        Assert.DoesNotContain("empty", aFace3, StringComparison.Ordinal);

        // the exploded faces are the meshed shape's own, the compound holds them, the copy is untouched
        Shape aMeshed = i.GetShape("s");
        var aFaces = new List<Shape>();
        for (var anExp = new Explorer(aMeshed, ShapeType.Face); anExp.More(); anExp.Next())
        {
            aFaces.Add(anExp.Current());
        }

        for (int k = 1; k <= 8; k++)
        {
            Assert.True(i.GetShape($"s_{k}").IsEqual(aFaces[k - 1]), $"s_{k}");
        }

        Assert.Contains("188 triangles", i.Eval("trinfo c"), StringComparison.Ordinal);
        Assert.Contains("8 empty faces", i.Eval("trinfo t"), StringComparison.Ordinal);

        // several shapes: their compound is meshed, each name rebound
        i.Eval($"restore {DataOcc}/solid.brep a; tclean a; restore {DataOcc}/Room.brep b; tclean b");
        Assert.Equal("Incremental Mesh, multi-threading OFF\nMeshing statuses: NoError", i.Eval("incmesh a b 0.5"));
        Assert.DoesNotContain("empty", i.Eval("trinfo a"), StringComparison.Ordinal);
        Assert.DoesNotContain("empty", i.Eval("trinfo b"), StringComparison.Ordinal);
    }

    [Fact]
    public void Tessellate_builds_a_regular_triangulation()
    {
        (DrawInterpreter i, StringWriter o) = Load();
        i.Eval("plane p 0 0 0 0 0 1; mkface f p -10 10 -5 5; tessellate r f 5 4");
        Assert.Equal("\nThis shape contains 1 faces.\n                    40 triangles.\n                    30 nodes.\n"
                     + "                    4 polygons on triangulation.\nMaximal deflection 0\n\n", i.Eval("trinfo r"));
        Assert.Equal("", i.Eval("tricheck r"));
        Assert.Equal("200 199.99999999999997\n", i.Eval("triarea r"));

        // a closed surface: the seam's two polygons
        i.Eval("cylinder cy 3; trim cyt cy 0 2*pi 0 10; tessellate r2 cyt 8 4");
        Assert.Contains("64 triangles.\n                    45 nodes.\n                    3 polygons", i.Eval("trinfo r2"), StringComparison.Ordinal);
        Assert.Equal("", i.Eval("tricheck r2"));

        Assert.Equal("", Error(i, "tessellate r f 0 4"));
        Assert.Equal("", Error(i, "tessellate r x 1 1"));
        Assert.Equal("", Error(i, "tessellate r cy 1 1"));
        Assert.Equal("", Error(i, "tessellate r f"));
        Assert.Equal("Error: Arguments nbu and nbv must be both greater than 0\nError: x is not a face\n"
                     + "Error: surface has infinite parametric range, aborting\n", o.ToString()[..o.ToString().IndexOf("Builds", StringComparison.Ordinal)]);
    }

    [Fact]
    public void The_mesh_factory_commands_print_with_printf()
    {
        (DrawInterpreter i, StringWriter o) = Load("solid.brep");
        i.Eval("mpnames; mpgetdefaultname; mpsetdefaultname XBRepMesh; mpgetdefaultname; mpsetdefaultname foo; mpsetdefaultname FastDiscret");
        Assert.Equal(" *available algorithms:\n  FastDiscret\n  XBRepMesh\n *default name: FastDiscret\n *ready\n *default name: XBRepMesh\n"
                     + " *algorithm 'foo' not found\n *ready\n", o.ToString());
        o.GetStringBuilder().Clear();
        i.Eval("mpnames x; mpgetdefaultname x; mpsetdefaultname; mpincmesh s; mpincmesh x 1");
        Assert.Equal(" use mpnames\n use mpgetdefaultname\n use mpsetdefaultname name\n use mpincmesh shape deflection [angle]\n"
                     + " null shape is not allowed here\n", o.ToString());

        // mpincmesh: the factory's algorithm (BRepMesh_IncrementalMesh), the name rebound to the meshed copy
        Shape anInput = i.GetShape("s");
        i.Eval("explode s f; mpincmesh s 0.1 0.3");
        Assert.Equal(Text(Meshed(anInput, Parameters(0.1) with { Angle = 0.3 })), Text(i.GetShape("s")));
        Assert.DoesNotContain("empty", i.Eval("trinfo s_1"), StringComparison.Ordinal);

        // mpparallel sets the mesher's process-wide default
        o.GetStringBuilder().Clear();
        try
        {
            i.Eval("mpparallel; mpparallel 1");
            Assert.True(IncrementalMesh.IsParallelDefault);
        }
        finally
        {
            i.Eval("mpparallel 0");
        }

        Assert.Equal("Incremental Mesh, multi-threading OFF\nIncremental Mesh, multi-threading ON\nIncremental Mesh, multi-threading OFF\n", o.ToString());
    }

    [Fact]
    public void Vdisplay_meshes_as_AIS_does()
    {
        (DrawInterpreter i, StringWriter o) = Load("CrankArm.brep");
        Shape anInput = i.GetShape("s");
        double aDeflection = PresentationToolTriangulatedShape.GetDeflection(anInput, new Drawer());
        string anExpected = Text(new IncrementalMesh(anInput, aDeflection, false, 20 * Math.PI / 180).Shape());

        // no viewer: vdefaults fails as in DRAWEXE; vinit makes the context
        Assert.Equal("", Error(i, "vdefaults"));
        Assert.Equal("Error: no active viewer\n", o.ToString());
        i.Eval("vinit View1");
        Assert.Equal("DeflType:           relative\nDeviationCoeff:     0.001\nAngularDeflection:  20\nAutoTriangulation:  on\n", i.Eval("vdefaults"));

        // wireframe: the selection meshes the shape (StdSelect_BRepSelectionTool::Load)
        i.Eval("explode s f; vdisplay s");
        Assert.Equal(anExpected, Text(i.GetShape("s")));
        Assert.DoesNotContain("empty", i.Eval("trinfo s_1"), StringComparison.Ordinal);

        // displayed again: echoed; the same shape is not meshed again
        Assert.Equal("Display s\n", i.Eval("vdisplay s"));
        Assert.Equal("", i.Eval("vdisplay s -noecho"));

        // without selection, the wireframe mode doesn't mesh; the shaded mode does (StdPrs_ShadedShape)
        i.Eval("vclear; tclean s; vdisplay -noselect s");
        Assert.Contains("empty faces", i.Eval("trinfo s"), StringComparison.Ordinal);
        i.Eval("vsetdispmode s 1");
        Assert.Equal(anExpected, Text(i.GetShape("s")));

        // the context's display mode for the objects without their own
        i.Eval("vclear; tclean s; vsetdispmode 1; vdisplay -noselect s");
        Assert.Equal(anExpected, Text(i.GetShape("s")));

        // no automatic triangulation, by the defaults or by the object
        i.Eval("vclear; tclean s; vdefaults -autoTriang off; vdisplay -dispMode 1 s");
        Assert.Contains("empty faces", i.Eval("trinfo s"), StringComparison.Ordinal);
        i.Eval("vdefaults -autoTriang 1; vclear; vdisplay s -noautotriang");
        Assert.Contains("empty faces", i.Eval("trinfo s"), StringComparison.Ordinal);

        // an absolute deflection and another angle
        i.Eval("vclear; vdefaults -absDefl 0.5 -angDefl 10; vdisplay s");
        Assert.Equal(Text(new IncrementalMesh(anInput, 0.5, false, 10 * Math.PI / 180).Shape()), Text(i.GetShape("s")));
        Assert.Equal("DeflType:           absolute\nAbsoluteDeflection: 0.5\nAngularDeflection:  10\nAutoTriangulation:  on\n", i.Eval("vdefaults"));
        Assert.Equal("Syntax error: unknown argument '-FOO'\n", Error(i, "vdefaults -foo") + o.ToString()[^38..]);

        // vclose of the last view drops the context
        i.Eval("vclose");
        Assert.Equal("", Error(i, "vsetdispmode 1"));
    }

    [Fact]
    public void Shapes_meshed_by_the_viewer_stay_shared()
    {
        (DrawInterpreter i, _) = Load("solid.brep");

        // the mesh group's shading sequence
        i.Eval("vinit View1; vdisplay -noupdate -redisplay s; vsetdispmode s 1");
        string aMeshed = Text(i.GetShape("s"));
        Assert.DoesNotContain("empty", i.Eval("trinfo s"), StringComparison.Ordinal);

        // tclean writes into the displayed shape as well: displayed again unchanged, it isn't meshed again (as in DRAWEXE)
        i.Eval("tclean s");
        Assert.Equal("Display s\n", i.Eval("vdisplay s"));
        Assert.Contains("8 empty faces", i.Eval("trinfo s"), StringComparison.Ordinal);

        // a new shape under the name is redisplayed: meshed
        i.Eval($"restore {DataOcc}/solid.brep s; tclean s; vdisplay s");
        Assert.Equal(aMeshed, Text(i.GetShape("s")));
    }
}

/// <summary>
/// The tests that change or read the mesher's process-wide defaults (mpparallel, mpsetdefaultname). The collection
/// runs on its own, after every parallel test, because corpus cases and other tests may call mpparallel too.
/// </summary>
[CollectionDefinition(Name, DisableParallelization = true)]
public sealed class MeshDefaultsCollection
{
    public const string Name = "Mesh defaults";
}
