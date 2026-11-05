// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using Kerf.Kernel.DrawLite;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// Unit 3M's Draw commands: box (every form), wedge, pcylinder, pcone, psphere, ptorus, halfspace
/// (BRepTest_PrimitiveCommands.cxx, BRepTest_TopologyCommands.cxx), prism and revol (BRepTest_SweepCommands.cxx):
/// the shapes they bind (counts and global properties against known answers), their texts and their errors.
/// </summary>
public class DrawPrimitiveCommandsTests
{
    private static (DrawInterpreter Interpreter, StringWriter Output) Load()
    {
        var anOutput = new StringWriter { NewLine = "\n" };
        var anInterpreter = new DrawInterpreter(anOutput);
        anInterpreter.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        return (anInterpreter, anOutput);
    }

    private static string Error(DrawInterpreter theDI, string theScript) => Assert.ThrowsAny<DrawException>(() => theDI.Eval(theScript)).Reason;

    private static int Count(in Shape theShape, ShapeType theType)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, theType, aMap);
        return aMap.Count;
    }

    private static (int V, int E, int F) Counts(DrawInterpreter theDI, string theName)
    {
        Shape aShape = theDI.GetShape(theName);
        Assert.False(aShape.IsNull);
        Assert.True(aShape.TShape()!.IsFrozen);
        return (Count(aShape, ShapeType.Vertex), Count(aShape, ShapeType.Edge), Count(aShape, ShapeType.Face));
    }

    private static double Volume(DrawInterpreter theDI, string theName)
    {
        var aProps = new Properties();
        MassPropertiesBuilder.VolumeProperties(theDI.GetShape(theName), aProps);
        return aProps.Mass();
    }

    private static double Area(DrawInterpreter theDI, string theName)
    {
        var aProps = new Properties();
        MassPropertiesBuilder.SurfaceProperties(theDI.GetShape(theName), aProps);
        return aProps.Mass();
    }

    private static void BindPlane(DrawInterpreter theDI, string theName, CoordinateSystem theAxes)
        => theDI.SetDrawable(theName, DrawGeometry.Of(new PlaneSurface(theAxes)));

    [Fact]
    public void Box_in_each_form()
    {
        (DrawInterpreter i, StringWriter o) = Load();
        Assert.Equal("", i.Eval("box b 1 2 3"));
        Assert.Equal((8, 12, 6), Counts(i, "b"));
        Assert.Equal(6.0, Volume(i, "b"), 1e-12);
        Assert.StartsWith("b is a shape SOLID", i.Eval("whatis b"));

        // origin and size
        i.Eval("box b 1 1 1 2 3 4");
        Assert.Equal(24.0, Volume(i, "b"), 1e-12);
        i.Eval("bounding b -save x0 y0 z0 x1 y1 z1");
        Assert.Equal(1.0, double.Parse(i.Eval("dval x0"), System.Globalization.CultureInfo.InvariantCulture), 1e-6);

        // a negative size extends from the corner the other way
        i.Eval("box b -1 2 -3");
        Assert.Equal(6.0, Volume(i, "b"), 1e-12);

        // -min, -size, -max (case-insensitive options)
        i.Eval("box b -min 1 2 3 -size 2 2 2");
        Assert.Equal(8.0, Volume(i, "b"), 1e-12);
        i.Eval("box b -MIN 1 2 3 -max 2 4 6");
        Assert.Equal(6.0, Volume(i, "b"), 1e-12);

        // -dir and -xdir: the axes of the box
        i.Eval("box b -dir 1 0 0 -xdir 0 1 0 -size 1 2 3");
        Assert.Equal(6.0, Volume(i, "b"), 1e-12);
        i.Eval("box b 0 0 0 1 1 1 -dir 0 0 1 -xdir 1 1 0");
        Assert.Equal(1.0, Volume(i, "b"), 1e-12);

        // Draw expressions
        i.Eval("dset s 2");
        i.Eval("box b s s*2 s+1");
        Assert.Equal(24.0, Volume(i, "b"), 1e-12);
        Assert.Equal("", o.ToString());
    }

    [Fact]
    public void Box_preview_is_a_vertex_an_edge_a_face_or_a_box()
    {
        (DrawInterpreter i, _) = Load();
        i.Eval("box p 0 0 0 -preview");
        Assert.StartsWith("p is a shape VERTEX", i.Eval("whatis p"));
        i.Eval("box p 1 0 0 -preview");
        Assert.StartsWith("p is a shape EDGE", i.Eval("whatis p"));
        i.Eval("box p 1 2 0 -preview");
        Assert.StartsWith("p is a shape FACE", i.Eval("whatis p"));
        Assert.Equal(2.0, Area(i, "p"), 1e-12);
        i.Eval("box p -min 1 1 1 -max 1 3 4 -preview");
        Assert.StartsWith("p is a shape FACE", i.Eval("whatis p"));
        Assert.Equal(6.0, Area(i, "p"), 1e-12);
        i.Eval("box p 1 2 3 -preview");
        Assert.StartsWith("p is a shape SOLID", i.Eval("whatis p"));
        Assert.Equal(6.0, Volume(i, "p"), 1e-12);
    }

    [Fact]
    public void Box_errors()
    {
        (DrawInterpreter i, StringWriter o) = Load();
        Assert.Equal("", Error(i, "box b 1 2 x y"));
        Assert.Equal("Syntax error\n", o.ToString());

        // a flat box: BRepPrim_GWedge's degenerated shell
        Assert.Equal("An exception was caught Standard_DomainError", Error(i, "box b 0 1 1"));
    }

    [Fact]
    public void Wedge_in_each_form()
    {
        (DrawInterpreter i, _) = Load();
        i.Eval("wedge w 10 10 10 5");
        Assert.Equal((8, 12, 6), Counts(i, "w"));
        Assert.Equal(10 * 10 * (10 + 5) / 2.0, Volume(i, "w"), 1e-9);

        i.Eval("wedge w 10 10 10 0");
        Assert.Equal((6, 9, 5), Counts(i, "w"));
        Assert.Equal(500.0, Volume(i, "w"), 1e-9);

        i.Eval("wedge w 20 10 20 5 5 15 15");
        Assert.Equal((8, 12, 6), Counts(i, "w"));

        i.Eval("wedge w 1 2 3 0 0 1 1 0 0 10 10 10 5");
        Assert.Equal(750.0, Volume(i, "w"), 1e-9);
        i.Eval("wedge w 1 2 3 0 0 1 1 0 0 20 10 20 5 5 15 15");
        Assert.Equal((8, 12, 6), Counts(i, "w"));

        Assert.Equal("", Error(i, "wedge w 1 2 3"));
    }

    [Fact]
    public void Pcylinder_pcone_psphere_ptorus()
    {
        (DrawInterpreter i, _) = Load();
        i.Eval("pcylinder c 2 3");
        Assert.Equal((2, 3, 3), Counts(i, "c"));
        Assert.Equal(Math.PI * 4 * 3, Volume(i, "c"), 1e-9);
        i.Eval("pcylinder c 2 3 90");
        Assert.Equal(Math.PI * 4 * 3 / 4, Volume(i, "c"), 1e-9);
        Assert.Equal(5, Counts(i, "c").F);

        BindPlane(i, "pl", new CoordinateSystem(new Point3D(1, 2, 3), new Direction3D(0, 1, 0), new Direction3D(1, 0, 0)));
        i.Eval("pcylinder c pl 2 3");
        Assert.Equal(Math.PI * 4 * 3, Volume(i, "c"), 1e-9);
        i.Eval("pcylinder c pl 2 3 180");
        Assert.Equal(Math.PI * 4 * 3 / 2, Volume(i, "c"), 1e-9);
        Assert.Equal("", Error(i, "pcylinder c 2 3 4 5"));
        Assert.Equal("", Error(i, "pcylinder c"));

        i.Eval("pcone k 5 2 10");
        Assert.Equal(Math.PI * 10 / 3 * (25 + 10 + 4), Volume(i, "k"), 1e-9);
        i.Eval("pcone k 5 0 10");
        Assert.Equal(2, Counts(i, "k").F);
        i.Eval("pcone k 5 2 10 180");
        Assert.Equal(Math.PI * 10 / 3 * (25 + 10 + 4) / 2, Volume(i, "k"), 1e-9);
        i.Eval("pcone k pl 5 2 10");
        Assert.Equal(Math.PI * 10 / 3 * (25 + 10 + 4), Volume(i, "k"), 1e-9);
        i.Eval("pcone k pl 5 2 10 90");
        Assert.Equal(Math.PI * 10 / 3 * (25 + 10 + 4) / 4, Volume(i, "k"), 1e-9);
        Assert.Equal("An exception was caught Standard_DomainError: cone with two identic radii", Error(i, "pcone k 2 2 10"));

        i.Eval("psphere s 2");
        Assert.Equal(4.0 / 3 * Math.PI * 8, Volume(i, "s"), 1e-9);
        i.Eval("psphere s 2 90");
        Assert.Equal(4.0 / 3 * Math.PI * 8 / 4, Volume(i, "s"), 1e-9);
        i.Eval("psphere s 2 0 90");
        Assert.Equal(4.0 / 3 * Math.PI * 8 / 2, Volume(i, "s"), 1e-9);
        i.Eval("psphere s 2 0 90 180");
        Assert.Equal(4.0 / 3 * Math.PI * 8 / 4, Volume(i, "s"), 1e-9);
        i.Eval("psphere s pl 2");
        Assert.Equal(4.0 / 3 * Math.PI * 8, Volume(i, "s"), 1e-9);
        i.Eval("psphere s pl 2 90");
        Assert.Equal(4.0 / 3 * Math.PI * 8 / 4, Volume(i, "s"), 1e-9);
        i.Eval("psphere s pl 2 -90 0 90");
        Assert.Equal(4.0 / 3 * Math.PI * 8 / 8, Volume(i, "s"), 1e-9);

        double aTorus = 2 * Math.PI * Math.PI * 10 * 4;
        i.Eval("ptorus t 10 2");
        Assert.Equal((1, 2, 1), Counts(i, "t"));
        Assert.Equal(aTorus, Volume(i, "t"), 1e-6);
        i.Eval("ptorus t 10 2 90");
        Assert.Equal(aTorus / 4, Volume(i, "t"), 1e-6);
        i.Eval("ptorus t pl 10 2");
        Assert.Equal(aTorus, Volume(i, "t"), 1e-6);
        i.Eval("ptorus t pl 10 2 180");
        Assert.Equal(aTorus / 2, Volume(i, "t"), 1e-6);
        i.Eval("ptorus t 10 2 -90 90 360");
        Assert.True(Volume(i, "t") > 0);
        i.Eval("ptorus t pl 10 2 -90 90 90");
        Assert.Equal(5, Counts(i, "t").F);
        // OCCT dereferences the null plane of the longest form; the port's null dereference ends the command
        Assert.Equal("ptorus: DrawTrSurf::Get: not a plane", Error(i, "ptorus t 1 2 3 4 5 6"));
    }

    [Fact]
    public void Halfspace_of_a_face_and_of_a_shell()
    {
        (DrawInterpreter i, StringWriter o) = Load();
        i.Eval("box b 1 1 1");
        i.Eval("explode b f");
        i.Eval("halfspace h b_1 5 0.5 0.5");
        Assert.StartsWith("h is a shape SOLID", i.Eval("whatis h"));
        Assert.Equal(1, Counts(i, "h").F);

        i.Eval("explode b sh");
        i.Eval("halfspace h b_1 0.5 0.5 0.5");
        Assert.Equal(6, Counts(i, "h").F);

        Assert.Equal("b must be a face or a shell\n", Error(i, "halfspace h b 0 0 0"));
        Assert.Equal("b_1 is not a FACE but a SHELL\nb is not a FACE but a SOLID\nb is not a SHELL but a SOLID\n", o.ToString());
        Assert.Equal("", Error(i, "halfspace h b 0 0"));
    }

    [Fact]
    public void Prism_and_revol()
    {
        (DrawInterpreter i, _) = Load();
        i.Eval("box b 1 1 1");
        i.Eval("explode b f");
        // the face at x = 0, extruded along x
        i.Eval("prism p b_1 -2 0 0");
        Assert.StartsWith("p is a shape SOLID", i.Eval("whatis p"));
        Assert.Equal(2.0, Math.Abs(Volume(i, "p")), 1e-12);
        i.Eval("prism p b_1 -2 0 0 Copy");
        Assert.Equal(2.0, Math.Abs(Volume(i, "p")), 1e-12);
        i.Eval("prism p b_1 1 0 0 Inf");
        Assert.True(i.GetShape("p").InfiniteFlag || Counts(i, "p").F == 4);
        i.Eval("prism p b_1 1 0 0 semiinf");
        Assert.StartsWith("p is a shape SOLID", i.Eval("whatis p"));
        Assert.Equal("", Error(i, "prism p b_1 1 0"));
        Assert.Equal("", Error(i, "prism p nothing 1 0 0"));

        // a unit square at x in [2, 3] turned around Z: a tube
        i.Eval("box q 2 0 0 1 1 1");
        i.Eval("explode q f");
        i.Eval("revol r q_3 0 0 0 0 0 1 360");
        Assert.Equal(Math.PI * (9 - 4), Math.Abs(Volume(i, "r")), 1e-9);
        i.Eval("revol r q_3 0 0 0 0 0 1 90 copy");
        Assert.Equal(Math.PI * (9 - 4) / 4, Math.Abs(Volume(i, "r")), 1e-9);
        Assert.Equal("", Error(i, "revol r q_3 0 0 0 0 0 1"));

        // an edge crossing the axis: Draw's message, no result
        i.Eval("box c -1 -1 0 2 2 1");
        for (Explorer anExp = new(i.GetShape("c"), ShapeType.Edge); anExp.More(); anExp.Next())
        {
            // the edge x = -1, z = 0 along Y, through the X axis
            var aBox = Box.Void();
            Bounds.Add(anExp.Current(), ref aBox, false);
            aBox.Get(out _, out double y0, out _, out double x1, out double y1, out double z1);
            if (x1 < -0.5 && z1 < 0.5 && y1 - y0 > 1)
            {
                i.SetShape("ce", anExp.Current());
            }
        }

        Assert.Equal("Revol not done \n", i.Eval("revol r2 ce 0 0 0 1 0 0 90"));
        Assert.Equal("0", i.Eval("isdraw r2"));
    }
}
