// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Occt;
using OcctNet.DrawLite;

namespace OcctNet.DrawLite.Tests;

/// <summary>
/// The construction, distance and modification commands of unit 3L2b (BRepTest_CurveCommands, _SurfaceCommands,
/// _ExtremaCommands and _BasicCommands): the shapes each binds and Draw's texts, derived from those sources; the
/// values are those DrawLiteConstructionOracleTests finds equal to native OCCT's.
/// </summary>
public class DrawConstructionCommandsTests
{
    private static string DataOcc => Path.Combine(DrawResourcesTests.OcctSourceDirectory, "data", "occ").Replace('\\', '/');

    private static (DrawInterpreter Interpreter, StringWriter Output) Load()
    {
        var anOutput = new StringWriter { NewLine = "\n" };
        var anInterpreter = new DrawInterpreter(anOutput);
        anInterpreter.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        return (anInterpreter, anOutput);
    }

    private static string Error(DrawInterpreter theDI, string theScript) => Assert.ThrowsAny<DrawException>(() => theDI.Eval(theScript)).Reason;

    private static int Count(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, theType, aMap);
        return aMap.Extent();
    }

    private static double Number(DrawInterpreter theDI, string theName)
    {
        Assert.True(theDI.TryGetNumber(theName, out double aValue), theName);
        return aValue;
    }

    [Fact]
    public void Vertices_edges_polylines_and_wires()
    {
        (DrawInterpreter i, _) = Load();
        i.Eval("vertex v1 0 0 0; vertex v2 3 4 0; edge e v1 v2; polyvertex pv v1 v2");
        Assert.Equal(new Point3D(3, 4, 0), BRep_Tool.Pnt(TopoDS.Vertex(i.GetShape("v2"))));
        TopoDS_Edge e = TopoDS.Edge(i.GetShape("e"));
        Assert.Equal(5.0, BRep_Tool.Curve(e, out double f, out double l)!.Value(l).Distance(new Point3D()) - f, 12);
        Assert.Equal(1, Count(i.GetShape("pv"), TopAbs_ShapeEnum.TopAbs_EDGE));

        // vertex on an edge's curve, at a point
        i.Eval("vertex v3 2.5 e; point p 1 2 3; vertex v4 p");
        Assert.Equal(new Point3D(1.5, 2, 0), BRep_Tool.Pnt(TopoDS.Vertex(i.GetShape("v3"))));
        Assert.Equal(new Point3D(1, 2, 3), BRep_Tool.Pnt(TopoDS.Vertex(i.GetShape("v4"))));
        Assert.Equal("", i.Eval("vertex v5 1 nothing"));
        Assert.Equal("0", i.Eval("isdraw v5"));
        Assert.Equal("Standard_NullObject", Error(i, "vertex v6 nothing")["An exception was caught ".Length..]);

        i.Eval("polyline p 0 0 0 1 0 0 1 1 0 0 1 0 0 0 0");
        TopoDS_Shape p = i.GetShape("p");
        Assert.Equal(TopAbs_ShapeEnum.TopAbs_WIRE, p.ShapeType());
        Assert.Equal(4, Count(p, TopAbs_ShapeEnum.TopAbs_EDGE));
        Assert.Equal(4, Count(p, TopAbs_ShapeEnum.TopAbs_VERTEX));
        Assert.Equal("", Error(i, "polyline q 0 0 0 1 0"));

        // wire: sorted edges, -unsorted, the errors in the result
        i.Eval("vertex a 0 0 0; vertex b 1 0 0; vertex c 1 1 0; vertex d 5 5 5; edge e1 a b; edge e2 b c; edge e3 c d; edge far d a");
        i.Eval("wire w e1 e2 e3");
        Assert.Equal(3, Count(i.GetShape("w"), TopAbs_ShapeEnum.TopAbs_EDGE));
        i.Eval("wire wu -unsorted e3 e1 e2");
        Assert.Equal(3, Count(i.GetShape("wu"), TopAbs_ShapeEnum.TopAbs_EDGE));
        i.Eval("line l 0 5 0 1 0 0; mkedge el l 0 1");
        Assert.Equal("Wire not done with an error:\nBRepBuilderAPI_DisconnectedWire\n", i.Eval("wire bad e1 el"));
        Assert.Equal("0", i.Eval("isdraw bad"));
        Assert.Equal("Wire not done with an error:\nBRepBuilderAPI_EmptyWire\n", i.Eval("wire empty nothing"));
    }

    [Fact]
    public void Mkedge_from_3d_and_2d_curves_and_polygons()
    {
        (DrawInterpreter i, _) = Load();
        i.Eval("line l 0 0 0 1 0 0; mkedge e l; mkedge e1 l 2 5");
        BRep_Tool.Range(TopoDS.Edge(i.GetShape("e1")), out double f, out double l);
        Assert.Equal((2.0, 5.0), (f, l));
        Assert.True(BRep_Tool.Degenerated(TopoDS.Edge(i.GetShape("e1"))) == false);

        // between vertices, with parameters
        i.Eval("vertex v1 1 0 0; vertex v2 4 0 0; mkedge e2 l v1 v2; mkedge e3 l v1 1 v2 4");
        BRep_Tool.Range(TopoDS.Edge(i.GetShape("e2")), out f, out l);
        Assert.Equal((1.0, 4.0), (f, l));
        BRep_Tool.Range(TopoDS.Edge(i.GetShape("e3")), out f, out l);
        Assert.Equal((1.0, 4.0), (f, l));

        // a 2D curve alone (on the XY plane), and on a surface
        i.Eval("line l2 0 0 1 1; mkedge p2 l2 0 2; cylinder cy 1; mkedge s2 l2 cy 0 1");
        BRep_Tool.CurveOnSurface(TopoDS.Edge(i.GetShape("s2")), out Geom2d_Curve? aC, out Geom_Surface? aS, out _, out _, out _);
        Assert.NotNull(aC);
        Assert.IsType<Geom_CylindricalSurface>(aS);
        Assert.Equal(TopAbs_ShapeEnum.TopAbs_EDGE, i.GetShape("p2").ShapeType());

        Assert.Equal("nothing is not a curve or polygon 3d\n", Error(i, "mkedge x nothing"));
        Assert.Equal("", Error(i, "mkedge x l 1 2 3"));
    }

    [Fact]
    public void Curves_points_and_pcurves_from_shapes()
    {
        (DrawInterpreter i, _) = Load();
        i.Eval("line l 0 0 0 1 0 0; mkedge e l 0 1; mkcurve c e; vertex v 3 4 0; mkpoint p v");
        Assert.Equal("\n\n*********** Dump of c *************\nTrimmed curve\nParameters : 0 1\nBasis curve :\nLine\n  Origin :0, 0, 0 \n  Axis   :1, 0, 0 \n\n",
                     i.Eval("dump c"));
        Assert.Equal("\n\n*********** Dump of p *************\nPoint : 3.000000000000000e+00, 4.000000000000000e+00, 0.000000000000000e+00\n",
                     i.Eval("dump p"));
        // an edge made on a surface from a 2D curve has no 3D curve
        Assert.Equal("e2 has no 3d curve\n", Error(i, "line l2 0 0 1 0; cylinder cy 1; mkedge e2 l2 cy 0 1; mkcurve c2 e2"));

        // the pcurves of a planar face, and one by edge and face
        i.Eval("polyline w 0 0 0 1 0 0 1 1 0 0 1 0 0 0 0; mkplane f w; pcurve f");
        for (int k = 1; k <= 4; k++)
        {
            Assert.IsAssignableFrom<Geom2d_Curve>(((DrawGeometry)i.GetDrawable($"f_{k}")!).Geometry);
        }

        // a planar face's pcurves are computed, not stored: mk2dcurve finds them only with the face
        i.Eval("explode f e; pcurve pc f_1 f; mk2dcurve m f_1 f");
        Assert.IsType<Geom2d_TrimmedCurve>(((DrawGeometry)i.GetDrawable("m")!).Geometry);
        Assert.Equal("f_1 has no 2d curve\n", Error(i, "mk2dcurve m1 f_1"));
        i.Eval("cylinder cy2 1; mkface cf cy2 0 1 0 1; explode cf e; mk2dcurve m1 cf_1; mk2dcurve m2 cf_1 1");
        Assert.IsType<Geom2d_TrimmedCurve>(((DrawGeometry)i.GetDrawable("m1")!).Geometry);
        Assert.IsType<Geom2d_TrimmedCurve>(((DrawGeometry)i.GetDrawable("m2")!).Geometry);
        Assert.Equal("e has no 2d curve\n", Error(i, "mk2dcurve m3 e"));
        Assert.Equal("f_1 has no 2d curve on cf\n", Error(i, "mk2dcurve m4 f_1 cf"));
        Assert.Equal("", i.Eval("pcurve -mute f"));
    }

    [Fact]
    public void Faces_shells_planes_and_surfaces()
    {
        (DrawInterpreter i, StringWriter o) = Load();
        i.Eval("plane pl; mkface f pl 0 1 0 2; mkshell sh pl 0 1 0 2; mksurface s f");
        Assert.Equal(TopAbs_ShapeEnum.TopAbs_FACE, i.GetShape("f").ShapeType());
        Assert.Equal(TopAbs_ShapeEnum.TopAbs_SHELL, i.GetShape("sh").ShapeType());
        BRepTools.UVBounds(TopoDS.Face(i.GetShape("f")), out double u1, out double u2, out double v1, out double v2);
        Assert.Equal((0.0, 1.0, 0.0, 2.0), (u1, u2, v1, v2));
        Assert.IsType<Geom_Plane>(((DrawGeometry)i.GetDrawable("s")!).Geometry);

        // a face bounded by a wire, an infinite face, a shell of a whole sphere
        i.Eval("polyline w 0 0 0 1 0 0 1 1 0 0 1 0 0 0 0; mkface fw pl w; mkface fi pl; sphere sp 1; mkshell ss sp");
        Assert.Equal(1, Count(i.GetShape("fw"), TopAbs_ShapeEnum.TopAbs_WIRE));
        Assert.Equal(TopAbs_ShapeEnum.TopAbs_SHELL, i.GetShape("ss").ShapeType());
        Assert.Equal("", Error(i, "mkface x nothing"));
        Assert.Equal("nothing is not a surface\n", o.ToString());

        // mkplane and its statuses
        i.Eval("mkplane pf w");
        Assert.IsType<Geom_Plane>(BRep_Tool.Surface(TopoDS.Face(i.GetShape("pf"))));
        i.Eval("polyline q 0 0 0 1 0 0 1 1 1 0 1 0 0 0 0");
        Assert.Equal("Error. mkplane has been finished with \"Not Planar\" status.\n", i.Eval("mkplane g q"));
        Assert.Equal("0", i.Eval("isdraw g"));
    }

    [Fact]
    public void Profile_builds_faces_and_wires()
    {
        (DrawInterpreter i, _) = Load();
        Assert.StartsWith("profile name [code values] [code values] ...\n", i.Eval("profile"), StringComparison.Ordinal);

        // a 2 x 1 rectangle closed by the last segment
        i.Eval("profile r X 2 Y 1 X -2");
        TopoDS_Shape r = i.GetShape("r");
        Assert.Equal(TopAbs_ShapeEnum.TopAbs_FACE, r.ShapeType());
        Assert.Equal(4, Count(r, TopAbs_ShapeEnum.TopAbs_EDGE));
        var aProps = new GProp_GProps();
        BRepGProp.SurfaceProperties(r, aProps);
        Assert.Equal(2.0, aProps.Mass(), 12);

        // an open wire raised to z = 1; an arc; the codes' errors
        i.Eval("profile w O 0 0 1 X 1 Y 1 WW");
        Assert.Equal(TopAbs_ShapeEnum.TopAbs_WIRE, i.GetShape("w").ShapeType());
        Assert.Equal(2, Count(i.GetShape("w"), TopAbs_ShapeEnum.TopAbs_EDGE));
        i.Eval("profile c F 1 0 C 1 180 X -2");
        aProps = new GProp_GProps();
        BRepGProp.SurfaceProperties(i.GetShape("c"), aProps);
        // a half disc of radius 1 and the triangle (1, 0), (1, 2), (-1, 2)
        Assert.Equal(2 + Math.PI / 2, aProps.Mass(), 12);
        Assert.Equal("profile : unknown code Q", Error(i, "profile x Q 1"));
        Assert.Equal("profile : bad number of arguments", Error(i, "profile x X"));
        Assert.Equal("profile: The F instruction must precede all moves", Error(i, "profile x X 1 F 1 1"));
        Assert.Equal("profile : null direction", Error(i, "profile x P 0 0 0 1 0 0"));
        // with the direction along Y, a vertical can't be met: OCCT prints the index of the code
        Assert.Equal("Profile : cannot intersect, arg 5", Error(i, "profile x D 0 1 IX 1"));
    }

    [Fact]
    public void Distmini_binds_the_value_and_the_solutions()
    {
        (DrawInterpreter i, _) = Load();
        i.Eval("vertex v1 0 0 0; vertex v2 3 4 0");
        Assert.Equal("\"distmini\" command returns:\nd_val d \nOutput is complete.\n", i.Eval("distmini d v1 v2"));
        Assert.Equal(5.0, Number(i, "d_val"));
        Assert.Equal(TopAbs_ShapeEnum.TopAbs_EDGE, i.GetShape("d").ShapeType());

        // two parallel segments: the two ends are solutions; touching shapes give vertices
        i.Eval("line l 0 0 0 1 0 0; line l2 0 1 0 1 0 0; mkedge e1 l 0 1; mkedge e2 l2 0 1");
        Assert.Equal("\"distmini\" command returns:\nx_val x x2 \nOutput is complete.\n", i.Eval("distmini x e1 e2 1e-7 -parallel"));
        Assert.Equal(1.0, Number(i, "x_val"));
        i.Eval("vertex m 0.5 0 0");
        i.Eval("distmini t e1 m");
        Assert.Equal(0.0, Number(i, "t_val"));
        Assert.Equal(TopAbs_ShapeEnum.TopAbs_VERTEX, i.GetShape("t").ShapeType());

        Assert.Equal("Syntax error at '-bad'", Error(i, "distmini y e1 e2 -BAD"));
        Assert.Equal("", Error(i, "distmini y e1"));
    }

    [Fact]
    public void Proximity_and_self_intersection_need_meshes()
    {
        (DrawInterpreter i, StringWriter o) = Load();
        i.Eval("line l 0 0 0 1 0 0; mkedge e l 0 1");
        Assert.Equal("", Error(i, "dist e e"));
        Assert.Equal("", Error(i, "proximity e e"));
        Assert.Equal("Error: Failed to perform proximity test\n", o.ToString());
        Assert.Equal("", Error(i, "proximity e e -tol 1 -value"));
        Assert.EndsWith("Error: Proximity value could not be computed if the tolerance is set\n", o.ToString(), StringComparison.Ordinal);
        Assert.Equal("", Error(i, "proximity e"));
        Assert.EndsWith("Usage: proximity Shape1 Shape2 [-tol <value> | -value] [-profile]\n", o.ToString(), StringComparison.Ordinal);

        i.Eval($"restore {DataOcc}/solid.brep s");
        Assert.Equal("Compound of overlapped sub-faces: s_overlapped\n", i.Eval("selfintersect s"));
        Assert.Equal(TopAbs_ShapeEnum.TopAbs_COMPOUND, i.GetShape("s_overlapped").ShapeType());
    }

    [Fact]
    public void Modifications_bind_new_shapes()
    {
        (DrawInterpreter i, _) = Load();
        i.Eval($"restore {DataOcc}/solid.brep s");
        TopoDS_Shape s = i.GetShape("s");

        i.Eval("nurbsconvert n s; scalexyz sc s 1 2 3; deform df s 1 2 3");
        foreach (string aName in new[] { "n", "sc", "df" })
        {
            Assert.Equal(Count(s, TopAbs_ShapeEnum.TopAbs_FACE), Count(i.GetShape(aName), TopAbs_ShapeEnum.TopAbs_FACE));
        }

        Assert.Equal("", Error(i, "nurbsconvert n"));
        Assert.Equal("Syntax error: wrong number of arguments", Error(i, "deform df s 1 2"));
        Assert.Equal("Syntax error: 'nothing' is not a valid shape", Error(i, "deform df nothing 1 2 3"));

        // with a result name the input is left; without, the name is bound to the changed copy
        i.Eval("sameparameter r s 1e-5; fsameparameter r2 s; updatetolerance u s; updatetolerance u2 s 1");
        Assert.True(i.GetShape("s").IsSame(s));
        i.Eval("sameparameter s; updatetolerance s 1; encoderegularity s 5; solidorientation s");
        Assert.False(i.GetShape("s").IsSame(s));
        Assert.Equal(Count(s, TopAbs_ShapeEnum.TopAbs_EDGE), Count(i.GetShape("s"), TopAbs_ShapeEnum.TopAbs_EDGE));
        Assert.StartsWith("Use sameparameter [result] shape [toler]\n", Error(i, "sameparameter"), StringComparison.Ordinal);

        // compare, issubshape, wexplo, findplane
        i.Eval("explode s f");
        Assert.Equal("same shapes\nequal shapes\n", i.Eval("compare s s"));
        Assert.Equal("shapes are not same\n", i.Eval("compare s s_1"));
        Assert.Equal("s_1 is sub-shape of s. Index in the shape: 1.\n", i.Eval("issubshape s_1 s"));
        Assert.Equal("s is NOT sub-shape of s_1.\n", i.Eval("issubshape s s_1"));
        Assert.Equal("null shapes\n", i.Eval("issubshape nothing s"));

        i.Eval("polyline w 0 0 0 1 0 0 1 1 0 0 1 0 0 0 0; wexplo w");
        Assert.Equal(TopAbs_ShapeEnum.TopAbs_EDGE, i.GetShape("WEDGE_4").ShapeType());
        Assert.Equal(" a plane is found \n", i.Eval("findplane w pl"));
        Assert.IsType<Geom_Plane>(((DrawGeometry)i.GetDrawable("pl")!).Geometry);
    }

    [Fact]
    public void Gcarc_length_and_getsourcefile()
    {
        (DrawInterpreter i, _) = Load();
        i.Eval("point p1 0 0 0; point p2 1 1 0; point p3 2 0 0; point p4 0 1 0");

        // OCCT's gcarc returns 1 once it has bound the curve: an error with an empty message
        Assert.Equal("", Error(i, "gcarc s seg p1 p3"));
        Assert.Equal("The length s is 2\n", i.Eval("length s"));
        Assert.Equal("", Error(i, "gcarc a cir p1 p2 p3"));
        Assert.Equal($"The length a is {StreamUtilities.FormatReal(GCPnts_AbscissaPoint.Length(new GeomAdaptor_Curve(DrawTrSurf_Curve(i, "a")), 1e-7), 17)}\n",
                     i.Eval("length a"));
        // the half circle through the three points
        Assert.Equal(Math.PI, GCPnts_AbscissaPoint.Length(new GeomAdaptor_Curve(DrawTrSurf_Curve(i, "a"))), 6);
        Assert.Equal("", Error(i, "gcarc t cir p1 p1 p4 p3"));
        Assert.IsType<Geom_TrimmedCurve>(DrawTrSurf_Curve(i, "t"));
        Assert.StartsWith("give a name for arc and the type seg or cir then\n", i.Eval("gcarc x cir p1 p2"), StringComparison.Ordinal);
        Assert.Equal("xis not a curve\n", Error(i, "length x"));

        Assert.Equal("pload                src/Draw/TKDraw/Draw/Draw_PloadCommands.cxx", i.Eval("getsourcefile pload"));
        Assert.Equal("distmini             src/Draw/TKTopTest/BRepTest/BRepTest_ExtremaCommands.cxx", i.Eval("getsourcefile distmini"));
    }

    [Fact]
    public void Extrema_of_curves_and_surfaces()
    {
        (DrawInterpreter i, _) = Load();
        i.Eval("line l1 0 0 0 1 0 0; line l2 0 0 1 0 1 0; line l3 0 1 0 1 0 0; plane p 0 0 5; circle c 0 0 0 1");
        Assert.Equal("ext_1 ", i.Eval("extrema l1 l2"));
        Assert.Equal(1.0, GCPnts_AbscissaPoint.Length(new GeomAdaptor_Curve(DrawTrSurf_Curve(i, "ext_1"))), 15);
        Assert.Equal("Infinite number of extremas, distance = 1\n", i.Eval("extrema l1 l3"));
        Assert.Equal("Infinite number of extremas, distance = 4\n", i.Eval("extrema l2 p 1"));
        Assert.Equal("Extrema 1 is point : 0 0 5\next_1_2 ext_1_3 prm_1_1 prm_3_1 prm_4_1 ", i.Eval("line l5 0 0 0 0 0 1; extrema l5 p 1"));
        Assert.Equal(5.0, Number(i, "prm_1_1"));
        Assert.Equal("Infinite number of extremas, distance = 5\n", i.Eval("extrema c p"));
        Assert.Equal("Extrema 1 is point : 0 0 0\n", i.Eval("line l4 0 0 0 0 0 1; extrema l1 l4"));
        Assert.Equal("", Error(i, "extrema l1 nothing"));
    }

    [Fact]
    public void Geometry_transformations_bind_transformed_copies()
    {
        (DrawInterpreter i, _) = Load();
        i.Eval("line l 0 0 0 1 0 0; point p 1 2 3; circle c 0 0 0 1; point q 1 2");
        Assert.Equal("l p ", i.Eval("translate l p 0 0 5"));
        Assert.Equal(new Point3D(0, 0, 5), ((Geom_Line)((DrawGeometry)i.GetDrawable("l")!).Geometry).Lin().Location());
        Assert.Equal(new Point3D(1, 2, 8), (Point3D)((DrawGeometry)i.GetDrawable("p")!).Geometry);

        // pmirror and pscale leave the point's coordinates in the list of names, as OCCT does
        Assert.Equal("p 0 0 0 ", i.Eval("pmirror p 0 0 0"));
        Assert.Equal(new Point3D(-1, -2, -8), (Point3D)((DrawGeometry)i.GetDrawable("p")!).Geometry);
        Assert.Equal("c 0 0 0 ", i.Eval("pscale c 0 0 0 2"));
        Assert.Equal(2.0, ((Geom_Circle)((DrawGeometry)i.GetDrawable("c")!).Geometry).Radius());
        Assert.Equal("c ", i.Eval("rotate c 0 0 0 1 0 0 90"));
        Assert.Equal("c ", i.Eval("lmirror c 0 0 0 0 0 1"));
        Assert.Equal("c ", i.Eval("smirror c 0 0 0 0 0 1"));
        Assert.Equal("", Error(i, "translate c 1 2"));

        Assert.Equal("q ", i.Eval("2dtranslate q 1 1"));
        Assert.Equal(new Point2D(2, 3), (Point2D)((DrawGeometry)i.GetDrawable("q")!).Geometry);
        i.Eval("line l2 0 0 1 0; 2drotate l2 0 0 90; 2dpmirror q 0 0; 2dlmirror q 0 0 1 0; 2dpscale q 0 0 2");
        Assert.Equal(new Point2D(-4, 6), (Point2D)((DrawGeometry)i.GetDrawable("q")!).Geometry);
        Assert.Equal(0.0, ((Geom2d_Line)((DrawGeometry)i.GetDrawable("l2")!).Geometry).Direction().X(), 15);
    }

    [Fact]
    public void Lappend_extends_canonical_lists_and_their_older_versions()
    {
        // lappend extends a list it made without formatting it again (OCCT's _check_log builds a long list so)
        (DrawInterpreter i, _) = Load();
        i.Eval("set l {}; for {set k 0} {$k < 2000} {incr k} { lappend l \"line $k\" {} {#x} }");
        Assert.Equal("6000", i.Eval("llength $l"));
        Assert.Equal("{line 1999} {} #x", i.Eval("lrange $l end-2 end"));
        Assert.EndsWith(" {line 1999} {} #x", i.Eval("set l"), StringComparison.Ordinal);
        Assert.Equal("{#x} y", i.Eval("lrange $l end end; list #x y"));
        Assert.Equal("{line 0} {} #x {line 1}", i.Eval("lrange $l 0 3"));

        // an older value of the list extended again keeps its own elements
        i.Eval("set a [list aaaaaaaaaa bbbbbbbbbb cccccccccc]; set b $a; lappend a x; lappend b y z");
        Assert.Equal("aaaaaaaaaa bbbbbbbbbb cccccccccc x", i.Eval("set a"));
        Assert.Equal("aaaaaaaaaa bbbbbbbbbb cccccccccc y z", i.Eval("set b"));
        Assert.Equal("4 5", i.Eval("list [llength $a] [llength $b]"));

        // a value that isn't canonical is read and written again, as Tcl does
        i.Eval("set c \"aaaaaaaaaa   bbbbbbbbbb  cccccccccc\"; lappend c d");
        Assert.Equal("aaaaaaaaaa bbbbbbbbbb cccccccccc d", i.Eval("set c"));
    }

    private static Geom_Curve DrawTrSurf_Curve(DrawInterpreter theDI, string theName)
        => Assert.IsAssignableFrom<Geom_Curve>(((DrawGeometry)theDI.GetDrawable(theName)!).Geometry);

    [Fact]
    public void Regularity_isoedges_and_projections()
    {
        (DrawInterpreter i, _) = Load();
        i.Eval("plane pl; mkface f pl 0 1 0 1; explode f e");
        Assert.Equal("Regularity of edge : C0\n", i.Eval("getedgeregularity f_1 f"));
        Assert.Equal("", Error(i, "getedgeregularity f_1"));

        i.Eval("uisoedge ui f 0.5 0 1; visoedge vi f 0.25 0 1");
        Assert.NotNull(BRep_Tool.CurveOnSurface(TopoDS.Edge(i.GetShape("ui")), TopoDS.Face(i.GetShape("f")), out _, out _));
        Assert.NotNull(BRep_Tool.Curve(TopoDS.Edge(i.GetShape("vi")), out _, out _));

        i.Eval("point p 0.5 0.25 3");
        Assert.Equal("proj dist = 3\nuvproj = 0.5 0.25\npproj = 0.5 0.25 0\n", i.Eval("projponf f p"));
        Assert.Equal("proj dist = 3\nuvproj = 0.5 0.25\npproj = 0.5 0.25 0\n", i.Eval("projponf f p -min -t"));
        Assert.Equal("not a face\n", i.Eval("projponf f_1 p"));
        Assert.StartsWith("Project point on the face.\n", Error(i, "projponf f"), StringComparison.Ordinal);
    }
}
