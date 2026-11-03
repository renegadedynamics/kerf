// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Kerf.Kernel.DrawLite;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// Unit 6L's healing commands on hand-made shapes: <c>sewing</c> and <c>fastsewing</c> (BRepTest_SurfaceCommands),
/// SWDRAW's ShapeAnalysis, ShapeFix, ShapeTool and ShapeUpgrade commands; their text as SWDRAW writes it, and the
/// frozen shapes of the Draw variables (the in-place tools work on a copy and rebind the variables).
/// </summary>
public class DrawHealCommandsTests
{
    private static DrawInterpreter Load()
    {
        var anInterpreter = new DrawInterpreter(new StringWriter { NewLine = "\n" });
        anInterpreter.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        return anInterpreter;
    }

    /// <summary>A box b, and its six faces as separate copies f1 ... f6 (no shared edges).</summary>
    private static DrawInterpreter LoadLooseFaces()
    {
        DrawInterpreter i = Load();
        i.Eval("box b 10 10 10; explode b f");
        for (int k = 1; k <= 6; k++)
        {
            i.Eval($"tcopy b_{k} f{k}");
        }
        return i;
    }

    [Fact]
    public void Sewing_and_fastsewing_make_a_shell_of_loose_faces()
    {
        DrawInterpreter i = LoadLooseFaces();
        Assert.Equal("", i.Eval("sewing r f1 f2 f3 f4 f5 f6"));
        Assert.Contains("SHELL", i.Eval("whatis r"), StringComparison.Ordinal);
        Assert.Contains("EDGE      : 12", i.Eval("nbshapes r"), StringComparison.Ordinal);
        // the tolerance word, and switches
        i.Eval("sewing r2 0.1 f1 f2 f3 -p +n");
        Assert.Contains("EDGE      : 10", i.Eval("nbshapes r2"), StringComparison.Ordinal);
        Assert.StartsWith("Use: sewing result [tolerance]", Assert.ThrowsAny<DrawException>(() => i.Eval("sewing r")).Reason,
                          StringComparison.Ordinal);
        Assert.Equal("Error! min tolerance can't exceed working tolerance\n",
                     Assert.ThrowsAny<DrawException>(() => i.Eval("sewing r 0.01 f1 f2 +mint 1")).Reason);
        Assert.Equal("Error! max tolerance can't be less than working tolerance\n",
                     Assert.ThrowsAny<DrawException>(() => i.Eval("sewing r 0.01 f1 f2 +maxt 0.001")).Reason);

        // fastsewing takes faces with their natural bounds on finite surfaces only
        Assert.Equal("Face is not added. See statuses.\nError: There are some problems while adding (32)\n"
                     + "Error: There are some problems while performing (96)\n", i.Eval("fastsewing fs f1"));
        i.Eval("beziersurf s1 2 2 0 0 0 1 0 0 0 1 0 1 1 0; mkface bf1 s1");
        i.Eval("beziersurf s2 2 2 1 0 0 2 0 0 1 1 0 2 1 0; mkface bf2 s2");
        Assert.Equal("", i.Eval("fastsewing fs -tol 1.e-4 bf1 bf2"));
        Assert.Contains("FACE      : 2", i.Eval("nbshapes fs"), StringComparison.Ordinal);
        Assert.Contains("EDGE      : 7", i.Eval("nbshapes fs"), StringComparison.Ordinal);
        Assert.Equal("Use: fastsewing result [-tol <value>] <list_of_faces>\n", Assert.ThrowsAny<DrawException>(() => i.Eval("fastsewing fs")).Reason);
    }

    [Fact]
    public void Free_bound_commands_split_the_free_edges_into_wires()
    {
        DrawInterpreter i = LoadLooseFaces();
        i.Eval("sewing sh f1 f2 f3 f4 f5");
        Assert.Equal("sh_c - closed wires\nsh_o - open wires\n", i.Eval("freebounds sh 0"));
        Assert.Contains("EDGE      : 4", i.Eval("nbshapes sh_c"), StringComparison.Ordinal);
        Assert.Contains("WIRE      : 0", i.Eval("nbshapes sh_o"), StringComparison.Ordinal);
        Assert.Equal("sh_c - closed wires\nsh_o - open wires\n", i.Eval("freebounds sh 0.1 1 1"));
        // freebounds of a compound of loose faces with a sewing tolerance
        i.Eval("compound f1 f2 lf");
        Assert.Equal("lf_c - closed wires\nlf_o - open wires\n", i.Eval("freebounds lf 0.1"));

        string aProps = i.Eval("fbprops sh");
        Assert.StartsWith("\n \tArea mm2\tLength mm\tRatio L/W\tWidth mm\tNb notch\nClosed bounds properties\n 1\t   100.00000\t    40.00000\t",
                          aProps, StringComparison.Ordinal);
        Assert.EndsWith("sh_c - closed wires,  sh_o - closed wires \n", aProps, StringComparison.Ordinal);
        Assert.Equal("sh_c - closed wires\nsh_o - open wires\n", i.Eval("fbclose sh 0 0.1"));
        Assert.ThrowsAny<DrawException>(() => i.Eval("freebounds sh"));
    }

    [Fact]
    public void Statshape_counts_the_contents_of_a_box()
    {
        DrawInterpreter i = Load();
        i.Eval("box b 10 10 10");
        Assert.Equal("Count    Item\n-----    ----\n24\t EDGE (Oriented)\n12\t EDGE (Shared)\n6\t FACE\n1\t SHELL\n1\t SOLID\n"
                     + "48\t VERTEX (Oriented)\n8\t VERTEX (Shared)\n6\t WIRE\n",
                     i.Eval("statshape b"));
        Assert.Equal("Shape unknown : x\n", Assert.ThrowsAny<DrawException>(() => i.Eval("statshape x")).Reason);
        // a cylinder's face has a seam; the particular cases are saved under the prefix
        i.Eval("pcylinder c 5 10");
        Assert.Contains("1\tWire(s) with several seam edges\n", i.Eval("statshape c pfx trc3d"), StringComparison.Ordinal);
    }

    [Fact]
    public void Edge_and_wire_analysis_commands_report_as_SWDRAW()
    {
        DrawInterpreter i = Load();
        i.Eval("box b 10 10 10; explode b f; explode b_1 w; renamevar b_1_1 w1; explode b_1 e");
        Assert.Equal("Edge seems OK.\n", i.Eval("checkedge b_1_1"));
        Assert.Equal("Edge seems OK.\n", i.Eval("checkedge b_1_1 b_1"));
        // BRep_Tool computes a pcurve on any plane, so an edge off the plane fails the vertices' check
        Assert.StartsWith("Vertices of the edge don't coincide with start/end points of 2d-curve", i.Eval("checkedge b_1_1 b_2"),
                          StringComparison.Ordinal);
        Assert.Equal("Not self-intersecting wire.\n", i.Eval("checkselfintersection w1"));
        Assert.Equal("A null shape or not a wire is used.\n", Assert.ThrowsAny<DrawException>(() => i.Eval("checkselfintersection b_1")).Reason);
        Assert.Equal("Area = 100\n", i.Eval("getareacontour w1"));
        Assert.StartsWith("Edges tolerance computed by 371 points: \nMAX=", i.Eval("comptol b"), StringComparison.Ordinal);
        Assert.StartsWith("Edges tolerance computed by 10 points: \nMAX=", i.Eval("comptol b 10 pre"), StringComparison.Ordinal);
        Assert.Contains("EDGE", i.Eval("whatis pre_edge_tol"), StringComparison.Ordinal);
        i.Eval("plane p 0 0 0 0 0 1; mkface pf p 0 10 0 10");
        Assert.Equal(" Point UV  U = 2  V = 3\n => proj  X = 2  Y = 3  Z = 0\n", i.Eval("projface pf 2 3"));
        Assert.Contains(" Found 1 Points\n", i.Eval("projface pf 1 2 3"), StringComparison.Ordinal);
        Assert.StartsWith("Edge b_1_1 Params from 0 to 10\nPrecision (BRepBuilderAPI) : ", i.Eval("projcurve b_1_1 1 0 1"), StringComparison.Ordinal);
        Assert.StartsWith("Point:\n", i.Eval("projpcurve b_1_1 b_1 1e-7 0 0 1"), StringComparison.Ordinal);
        Assert.Equal("Point is IN\n", i.Eval("checkfclass2d pf 2 3"));
        Assert.Equal("Point is OUT\n", i.Eval("checkfclass2d pf 20 3"));
        Assert.Equal("", i.Eval("anaedges b"));
        Assert.StartsWith("TopoDS_Iterator(EDGE)  donne 4 Edges dont 4 distinctes\n", i.Eval("expwire w1"), StringComparison.Ordinal);
        Assert.StartsWith("Wire - 1 : \n", i.Eval("connectedges cw b_1"), StringComparison.Ordinal);
        Assert.Contains("WIRE      : 1", i.Eval("nbshapes cw"), StringComparison.Ordinal);
        Assert.Equal("   Type (True) : SOLID   Type  (via Compound) : SOLID\n", i.Eval("sortcompound b"));
    }

    [Fact]
    public void Canonical_recognition_finds_a_cylinder_and_a_circle()
    {
        DrawInterpreter i = Load();
        i.Eval("pcylinder c 5 10; explode c f; explode c_1 e");
        Assert.StartsWith("Gap = ", i.Eval("getanasurf s c_1 cyl"), StringComparison.Ordinal);
        Assert.Contains("CylindricalSurface", i.Eval("dump s"), StringComparison.Ordinal);
        Assert.Equal("Cannot get required surface\n", i.Eval("getanasurf s2 c_1 pln"));
        Assert.StartsWith("Gap = ", i.Eval("getanacurve cc c_1_1 cir"), StringComparison.Ordinal);
        Assert.Equal("Cannot get required curve\n", i.Eval("getanacurve cl c_1_1 lin"));
        Assert.Equal("Wrong shape type, shape can be wire or an edge \n", Assert.ThrowsAny<DrawException>(() => i.Eval("getanacurve cl c_1")).Reason);
    }

    [Fact]
    public void Fix_commands_leave_a_valid_box_alone()
    {
        DrawInterpreter i = Load();
        i.Eval("box b 10 10 10");
        Shape aBox = i.GetShape("b");
        Assert.Equal("", i.Eval("fixshape r b"));
        Assert.True(i.GetShape("r").IsSame(aBox));
        Assert.Equal("", i.Eval("fixshape r2 b 1e-7 1 -l +o *h"));
        Assert.StartsWith("Use: fixshape result shape", Assert.ThrowsAny<DrawException>(() => i.Eval("fixshape r")).Reason, StringComparison.Ordinal);
        Assert.Equal("", i.Eval("fixwgaps r3 b"));
        Assert.Equal("", i.Eval("fixsmall r4 b 0.1"));
        Assert.Equal("", i.Eval("fixsmalledges r5 b 0.1"));
        Assert.True(i.GetShape("r5").IsSame(aBox));
        Assert.Equal("", i.Eval("fixsmallfaces r6 b"));
        Assert.True(i.GetShape("r6").IsSame(aBox));
        Assert.Equal("ShapeFix_Face n a rien trouve a redire\n", i.Eval("reface b r7"));
        Assert.Equal("\n", i.Eval("edgesameparam b"));
        // the variable keeps the input
        Assert.True(i.GetShape("b").IsSame(aBox));
    }

    [Fact]
    public void Fixshape_closes_a_wire_with_a_gap()
    {
        DrawInterpreter i = Load();
        i.Eval("polyline w 0 0 0 10 0 0 10 10 0 0 10 0 0 0.001 0");
        i.Eval("mkplane f w 1");
        i.Eval("fixshape r f 0.01");
        Assert.Contains("This shape seems to be valid", i.Eval("checkshape r"), StringComparison.Ordinal);
        Assert.Contains("EDGE      : 4", i.Eval("nbshapes r"), StringComparison.Ordinal);
    }

    [Fact]
    public void Settolerance_writes_into_the_variables_sharing_the_shape_and_never_into_the_frozen_input()
    {
        DrawInterpreter i = Load();
        i.Eval("box b 10 10 10; explode b v");
        Shape aBox = i.GetShape("b");
        Shape aVertex = i.GetShape("b_1");
        Assert.Equal("Setting Tolerance to 0.01\n", i.Eval("settolerance b 0.01"));
        // DRAWEXE writes into the box's vertices, which b_1 holds; Draw-lite rebinds the variables to the changed copy
        Assert.Equal(0.01, Tool.Tolerance(ShapeCasts.Vertex(i.GetShape("b_1"))));
        Assert.Equal(0.01, Tool.Tolerance(ShapeCasts.Edge(new Explorer(i.GetShape("b"), ShapeType.Edge).Current())));
        Assert.True(ShapeCasts.Vertex(new Explorer(i.GetShape("b"), ShapeType.Vertex).Current()).IsSame(i.GetShape("b_1")));
        // the frozen shapes the variables held are untouched
        Assert.Equal(1.0e-7, Tool.Tolerance(ShapeCasts.Vertex(aVertex)));
        Assert.Equal(1.0e-7, Tool.Tolerance(ShapeCasts.Edge(new Explorer(aBox, ShapeType.Edge).Current())));
        Assert.Equal("Maximum Tolerance to 0.001\n", i.Eval("settolerance b < 0.001"));
        Assert.Equal(0.001, Tool.Tolerance(ShapeCasts.Vertex(i.GetShape("b_1"))));
        Assert.Equal("Minimum Tolerance to 0.0050000000000000001\n", i.Eval("settolerance b v > 0.005"));
        Assert.Equal(0.005, Tool.Tolerance(ShapeCasts.Vertex(i.GetShape("b_1"))));
        Assert.Equal("Tolerance Limited between 1.0000000000000001e-05 and 0.002\n", i.Eval("settolerance b e 1e-5 0.002"));
        Assert.Equal(0.001, Tool.Tolerance(ShapeCasts.Edge(new Explorer(i.GetShape("b"), ShapeType.Edge).Current())));
    }

    [Fact]
    public void Shape_tools_build_solids_remove_locations_and_reshape()
    {
        DrawInterpreter i = LoadLooseFaces();
        i.Eval("sewing sh f1 f2 f3 f4 f5 f6");
        i.Eval("ssolid sh so");
        Assert.Contains("SOLID", i.Eval("whatis so"), StringComparison.Ordinal);
        Assert.Contains("Not a Shell", Assert.ThrowsAny<DrawException>(() => i.Eval("ssolid f1 so2")).Reason);
        Assert.Equal(" Already a Solide ! nothing done\n", i.Eval("ssolid b so3"));

        Assert.Equal("", i.Eval("reshape rs b -remove b_1"));
        Assert.Contains("FACE      : 5", i.Eval("nbshapes rs"), StringComparison.Ordinal);
        Assert.Equal("", i.Eval("reshape rs2 b -replace b_1 f2 -until face"));
        Assert.Equal("Error: unknown shape type 'x'\n", Assert.ThrowsAny<DrawException>(() => i.Eval("reshape rs b -until x")).Reason);
        Assert.Equal("Error: invalid syntax at -foo\n", Assert.ThrowsAny<DrawException>(() => i.Eval("reshape rs b -FOO")).Reason);
        Assert.Equal("", i.Eval("samerange b"));

        i.Eval("ttranslate b 1 2 3");
        Assert.Equal("", i.Eval("removeloc r b"));
        Assert.True(i.GetShape("r").Location().IsIdentity);
        Assert.Contains("FACE      : 6", i.Eval("nbshapes r"), StringComparison.Ordinal);

        i.Eval("copytranslate ct b 10 0 0");
        Assert.Contains("SOLID", i.Eval("whatis ct"), StringComparison.Ordinal);
    }
}
