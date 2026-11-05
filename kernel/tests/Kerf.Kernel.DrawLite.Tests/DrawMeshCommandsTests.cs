// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Kerf.Engine;
using Kerf.Kernel.DrawLite;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// The mesh readers of unit 4L1 on data/occ files that carry triangulations: tclean (DBRep.cxx), trinfo, triepoints,
/// trmergenodes, correctnormals (MeshTest.cxx), triarea, tricheck and mpparallel (MeshTest_PluginCommands.cxx), and
/// DrawResources' checktrinfo over trinfo. The texts are Draw's, derived from those sources; the values are those the
/// oracle tests (DrawLiteMeshOracleTests) find equal to native OCCT's.
/// </summary>
[Collection(MeshDefaultsCollection.Name)]
public class DrawMeshCommandsTests
{
    private static string DataOcc => Path.Combine(DrawResourcesTests.OcctSourceDirectory, "data", "occ").Replace('\\', '/');

    private static (DrawInterpreter Interpreter, StringWriter Output) Load(string? theFile = null)
    {
        var anOutput = new StringWriter { NewLine = "\n" };
        var anInterpreter = new DrawInterpreter(anOutput);
        anInterpreter.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        if (theFile is not null)
        {
            anInterpreter.Eval($"restore {DataOcc}/{theFile} s");
        }

        return (anInterpreter, anOutput);
    }

    private static string Error(DrawInterpreter theDI, string theScript) => Assert.ThrowsAny<DrawException>(() => theDI.Eval(theScript)).Reason;

    private const string SolidInfo = "\nThis shape contains 8 faces.\n"
                                     + "                    248 triangles.\n"
                                     + "                    256 nodes.\n"
                                     + "                    92 polygons on triangulation.\n"
                                     + "Maximal deflection 1.95\n\n";

    [Fact]
    public void Trinfo_counts_the_faces_triangles_nodes_and_polygons()
    {
        (DrawInterpreter i, StringWriter o) = Load("solid.brep");
        Assert.Equal(SolidInfo, i.Eval("trinfo s"));

        // -lods: the triangulations of each face
        Assert.Equal(SolidInfo[..^1] + "Number of triangulation LODs [1]\nLOD #0. NbTris: 248.\n        Types: Poly_Triangulation (8).\n\n",
                     i.Eval("trinfo s -LODS"));

        // a deflection printed at %.17g, and the empty faces after tclean
        i.Eval($"restore {DataOcc}/Room.brep r");
        Assert.Contains("Maximal deflection 1.6000000007999999\n", i.Eval("trinfo r"), StringComparison.Ordinal);
        i.Eval("tclean r");
        Assert.Equal("\nThis shape contains 5 faces.\n                    5 empty faces.\n                    0 triangles.\n"
                     + "                    0 nodes.\n                    4 polygons on triangulation.\nMaximal deflection 0\n\n",
                     i.Eval("trinfo r"));

        // errors: Message::SendFail without a shape, the result with a name that isn't one
        Assert.Equal("", Error(i, "trinfo"));
        Assert.Equal("Syntax error: not enough arguments\n", o.ToString());
        Assert.Equal("x is not a shape\n", Error(i, "trinfo x"));
    }

    [Fact]
    public void Checktrinfo_reads_trinfo()
    {
        (DrawInterpreter i, StringWriter o) = Load("solid.brep");
        i.Eval("checktrinfo s -tri 248 -nod 256 -defl 1.95");
        Assert.DoesNotContain("Error", o.ToString(), StringComparison.Ordinal);
        Assert.Contains("Check of Number of triangles OK: value = 248, expected = 248", o.ToString(), StringComparison.Ordinal);
        i.Eval("checktrinfo s -tri 250");
        Assert.Contains("Error: Number of triangles = 248 is not equal to expected 250", o.ToString(), StringComparison.Ordinal);
    }

    [Fact]
    public void Triarea_sums_the_triangles_and_measures_the_surfaces()
    {
        (DrawInterpreter i, StringWriter o) = Load("solid.brep");
        Assert.Equal("39383.211678668878 39481.415022205372\n", i.Eval("triarea s"));
        Assert.Equal("640000 640000\n", i.Eval($"restore {DataOcc}/Room.brep r; triarea r"));

        // with an epsilon, BRepGProp's adaptive integration
        i.Eval($"restore {DataOcc}/CrankArm.brep c");
        Assert.Equal("24734.817085511248 24859.014274456538\n", i.Eval("triarea c 1e-4"));

        // a shape with no triangulation at all: 0 for the triangles and nothing printed
        i.Eval("tclean s");
        Assert.Equal("0 39481.415022205372\n", i.Eval("triarea s"));
        Assert.Equal("", o.ToString());

        // a face without triangulation among others is reported on the output
        i.Eval($"restore {DataOcc}/solid.brep s; explode s f; tclean s_1; compound s_1 s_2 c");
        Assert.Equal("", i.Eval("tricheck s_2"));
        i.Eval("triarea c");
        Assert.Equal("face 1 has no triangulation\n", o.ToString());

        Assert.Equal("", Error(i, "triarea"));
        Assert.Equal("", Error(i, "triarea x"));
    }

    [Fact]
    public void Tricheck_reports_small_triangles_free_links_and_faces_without_triangulation()
    {
        (DrawInterpreter i, _) = Load("CrankArm.brep");
        Assert.Equal("", i.Eval("tricheck s"));
        Assert.Equal("triangles with null area (in pairs: face / triangle): \n{40 5} {46 5} \n"
                     + "Free_links 0 Cross_face_errors 0 Async_edges 0 Free_nodes 0 Small triangles 2\n",
                     i.Eval("tricheck s -small"));

        // the faces without triangulation come first (MeshTest_CheckTopology::Perform), and the check of the meshes
        // inside the faces stops at the first face without polygons on its triangulation
        i.Eval("tclean s");
        string aCheck = i.Eval("tricheck s");
        Assert.StartsWith("face 1 has no triangulation\nface 2 has no triangulation\n", aCheck, StringComparison.Ordinal);
        Assert.EndsWith("face 53 has no triangulation\n", aCheck, StringComparison.Ordinal);

        // one face made of Room's triangulations, merged: each wall is a separate quad whose 4 sides are free links,
        // and none of its nodes is on a polygon of the face's (absent) edges
        i.Eval($"restore {DataOcc}/Room.brep r");
        Assert.Equal("Old, Triangles: 10, Nodes: 20\nNew, Triangles: 10, Nodes: 20\n", i.Eval("trmergenodes r -oneFace f"));
        string aFree = i.Eval("tricheck f");
        Assert.StartsWith("free links of face 1\n", aFree, StringComparison.Ordinal);
        Assert.EndsWith("Free_links 20 Cross_face_errors 0 Async_edges 0 Free_nodes 0 Small triangles 0\n", aFree, StringComparison.Ordinal);

        Assert.Equal("", Error(i, "tricheck"));
        Assert.Equal("", Error(i, "tricheck x"));
    }

    [Fact]
    public void Tclean_removes_the_meshes_on_a_copy()
    {
        (DrawInterpreter i, _) = Load("solid.brep");
        i.Eval("explode s f; tcopy s t");
        Shape aBefore = i.GetShape("s");
        Assert.Equal("", i.Eval("tclean s t"));
        Shape anAfter = i.GetShape("s");

        // the name is bound to a cleaned copy (decision 480); the faces exploded before are rebound to the copy's, as
        // in DRAWEXE where tclean writes into them (decision 495)
        Assert.False(anAfter.IsSame(aBefore));
        Assert.Contains("8 empty faces", i.Eval("trinfo s"), StringComparison.Ordinal);
        Assert.Contains("8 empty faces", i.Eval("trinfo t"), StringComparison.Ordinal);
        Assert.Contains("1 empty faces", i.Eval("trinfo s_1"), StringComparison.Ordinal);
        Assert.True(i.GetShape("s_1").IsSame(FirstFace(anAfter)));
        Assert.Equal("This shape seems to be valid", i.Eval("checkshape s"));

        // -geom drops the surfaces and curves as well
        i.Eval($"restore {DataOcc}/solid.brep g; tclean -geom g");
        Assert.Null(Tool.Surface(ShapeCasts.Face(FirstFace(i.GetShape("g")))));

        Assert.Equal("Syntax error : NULL shape 'x'", Error(i, "tclean x"));
        Assert.Equal("Syntax error: wrong usage of arguments: do not use 'force' and 'geom' flags together", Error(i, "tclean -force -geom s"));
        Assert.Equal("", Error(i, "tclean -force"));
        Assert.Equal("", Error(i, "tclean"));
    }

    private static Shape FirstFace(in Shape theShape) => new Explorer(theShape, ShapeType.Face).Current();

    [Fact]
    public void Triepoints_binds_a_vertex_at_each_node_of_the_edges_polygons()
    {
        (DrawInterpreter i, _) = Load("Room.brep");
        string aNames = i.Eval("triepoints s");
        Assert.Equal(24, TclList.Parse(aNames).Count);
        Assert.StartsWith("s_1_1 s_1_2 ", aNames, StringComparison.Ordinal);
        Assert.Equal("1", i.Eval("isdraw s_1_1"));
        Assert.Contains("s_1_1 is a shape VERTEX", i.Eval("whatis s_1_1"), StringComparison.Ordinal);

        // one edge: the names are shape_node
        i.Eval("explode s e");
        Assert.Equal("s_1_1 s_1_2", i.Eval("triepoints s_1"));

        // names that aren't shapes are skipped
        Assert.Equal("", i.Eval("triepoints x"));
        Assert.Equal("", Error(i, "triepoints"));
    }

    [Fact]
    public void Trmergenodes_merges_the_nodes_of_each_face()
    {
        (DrawInterpreter i, _) = Load("solid.brep");
        Shape aBefore = i.GetShape("s");
        Assert.Equal("Old, Triangles: 248, Nodes: 256\nNew, Triangles: 248, Nodes: 252\n", i.Eval("trmergenodes s"));
        Assert.False(i.GetShape("s").IsSame(aBefore));
        Assert.Contains("252 nodes.", i.Eval("trinfo s"), StringComparison.Ordinal);

        // nothing to merge: the shape stays bound
        i.Eval($"restore {DataOcc}/Room.brep r");
        Shape aRoom = i.GetShape("r");
        Assert.Equal("Old, Triangles: 10, Nodes: 20\nNew, Triangles: 10, Nodes: 20\n", i.Eval("trmergenodes r -angle 10 -tolerance 0.1"));
        Assert.True(i.GetShape("r").IsSame(aRoom));
        Assert.Equal("Old, Triangles: 10, Nodes: 20\nNew, Triangles: 10, Nodes: 20\n", i.Eval("trmergenodes r -force"));
        Assert.False(i.GetShape("r").IsSame(aRoom));

        Assert.Equal("Syntax error: angle should be within [0,90] range", Error(i, "trmergenodes r -angle 100"));
        Assert.Equal("Syntax error: tolerance should be within >=0", Error(i, "trmergenodes r -tolerance -1"));
        Assert.Equal("Syntax error at '-what'", Error(i, "trmergenodes r -what"));
        Assert.Equal("Syntax error: 'x' is not a shape", Error(i, "trmergenodes x"));
        Assert.Equal("Syntax error: not enough arguments", Error(i, "trmergenodes"));
    }

    [Fact]
    public void Correctnormals_computes_and_aligns_the_normals()
    {
        (DrawInterpreter i, _) = Load("Room.brep");
        Shape aBefore = i.GetShape("s");
        Assert.Equal("Some corrections in source shape have been made!\n", i.Eval("correctnormals s"));
        Assert.False(i.GetShape("s").IsSame(aBefore));
        Assert.True(Tool.Triangulation(ShapeCasts.Face(FirstFace(i.GetShape("s"))), out _)!.HasNormals);

        // the normals are there now and agree: nothing to do, the shape stays bound
        Shape aCorrected = i.GetShape("s");
        Assert.Equal("Normals have not been changed!\n", i.Eval("correctnormals s"));
        Assert.True(i.GetShape("s").IsSame(aCorrected));

        // OCCT reads argv[1] unchecked; an empty name is no shape
        Assert.Equal("Normals have not been changed!\n", i.Eval("correctnormals"));
    }

    [Fact]
    public void Mpparallel_shows_and_sets_the_flag()
    {
        (DrawInterpreter i, StringWriter o) = Load();
        Assert.Equal("", i.Eval("mpparallel"));
        i.Eval("mpparallel 1");
        i.Eval("mpparallel 0");
        Assert.Equal("Incremental Mesh, multi-threading OFF\nIncremental Mesh, multi-threading ON\nIncremental Mesh, multi-threading OFF\n", o.ToString());
    }

    [Fact]
    public void The_mesh_commands_are_logged_and_know_their_sources()
    {
        (DrawInterpreter i, StringWriter o) = Load("Room.brep");
        i.DoLog = true;
        i.Eval("triarea s");
        Assert.Equal("triarea s \n640000 640000\n\n", o.ToString());
        Assert.Equal("src/Draw/TKTopTest/MeshTest/MeshTest_PluginCommands.cxx", i.Eval("set Draw_Files(tricheck)"));
        Assert.Equal("src/Draw/TKDraw/DBRep/DBRep.cxx", i.Eval("set Draw_Files(tclean)"));
    }
}
