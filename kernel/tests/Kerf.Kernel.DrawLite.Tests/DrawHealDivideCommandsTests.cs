// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using Kerf.Kernel.DrawLite;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// Unit 7C's Draw commands of SWDRAW_ShapeUpgrade.cxx (the divide and convert tools) and SWDRAW_ShapeCustom.cxx, on
/// primitives: their counts, messages and argument errors, and the inputs left as they were.
/// </summary>
public class DrawHealDivideCommandsTests
{
    private static DrawInterpreter Load()
    {
        var anInterpreter = new DrawInterpreter(new StringWriter { NewLine = "\n" });
        anInterpreter.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        return anInterpreter;
    }

    private static int Count(in Shape theShape, ShapeType theType)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, theType, aMap);
        return aMap.Count;
    }

    private static void Geometry<T>(DrawInterpreter theDI, string theName)
        => Assert.IsAssignableFrom<T>(Assert.IsType<DrawGeometry>(theDI.GetDrawable(theName)).Geometry);

    private static string Error(DrawInterpreter theDI, string theScript) =>
        Assert.ThrowsAny<DrawException>(() => theDI.Eval(theScript)).Reason;

    [Fact]
    public void The_divide_commands_split_and_print_their_statuses()
    {
        DrawInterpreter i = Load();
        i.Eval("pcylinder c 10 20");
        Shape aCylinder = i.GetShape("c");
        int aFaces = Count(aCylinder, ShapeType.Face);

        // the lateral face spans 360 degrees: split into 4 (95 degrees at most each)
        Assert.Contains("Status: DONE2", i.Eval("DT_SplitAngle r c"), StringComparison.Ordinal);
        Assert.Equal(aFaces + 3, Count(i.GetShape("r"), ShapeType.Face));
        i.Eval("DT_SplitAngle r c 180");
        Assert.Equal(aFaces + 1, Count(i.GetShape("r"), ShapeType.Face));
        Assert.Equal("Status: OK\n", i.Eval("DT_ShapeDivide r c 0.01"));
        Assert.Equal(aFaces, Count(i.GetShape("r"), ShapeType.Face));

        // closed faces split in two
        i.Eval("DT_ClosedSplit r c");
        Assert.Equal(aFaces + 1, Count(i.GetShape("r"), ShapeType.Face));

        // a face split by number
        i.Eval("plane p 0 0 0 0 0 1; mkface f p 0 10 0 10");
        i.Eval("DT_SplitByNumber r f 4");
        Assert.Equal(4, Count(i.GetShape("r"), ShapeType.Face));
        i.Eval("DT_SplitByNumber r f 2 3");
        Assert.Equal(6, Count(i.GetShape("r"), ShapeType.Face));
        Assert.Contains("Incorrect number of parts", Error(i, "DT_SplitByNumber r f 0"), StringComparison.Ordinal);
        Assert.Contains("Incorrect numbers in U or V", Error(i, "DT_SplitByNumber r f 2 0"), StringComparison.Ordinal);
        i.Eval("DT_SplitByArea r f 30");
        Assert.True(Count(i.GetShape("r"), ShapeType.Face) > 1);

        // splitface by U and V values
        string aSplit = i.Eval("splitface r f u 5 v 3 7");
        Assert.Contains("Splitting by U: , 5", aSplit, StringComparison.Ordinal);
        Assert.Contains("Splitting by V: 3, 7", aSplit, StringComparison.Ordinal);
        Assert.Equal(6, Count(i.GetShape("r"), ShapeType.Face));
        // the plane is infinite: the range goes 100 beyond the face's bounds
        Assert.Contains("Splitting by U: , 50", i.Eval("splitface r f u 50"), StringComparison.Ordinal);
        Assert.Contains("Values should be inside range of surface; skipped\nNo splitting defined",
                        Error(i, "splitface r f u 150"), StringComparison.Ordinal);

        // the arguments
        Assert.Contains("bad number of arguments", Error(i, "DT_SplitAngle r"), StringComparison.Ordinal);
        Assert.Contains("Unknown shape", Error(i, "DT_ShapeDivide r nothing"), StringComparison.Ordinal);
        Assert.Contains("bad number of arguments", Error(i, "DT_ShapeConvert r c 1"), StringComparison.Ordinal);

        // the input is unchanged
        Assert.True(ReferenceEquals(aCylinder.TShape(), i.GetShape("c").TShape()));
        Assert.Equal(aFaces, Count(i.GetShape("c"), ShapeType.Face));
    }

    [Fact]
    public void The_convert_commands_convert_to_Bezier_and_BSpline()
    {
        DrawInterpreter i = Load();
        i.Eval("pcylinder c 10 20");

        // the planes become Bezier surfaces, the cylinder stays (an elementary surface)
        Assert.Equal("Status: DONE1\n", i.Eval("DT_ShapeConvert r c 1 1"));
        Assert.Contains("Number of Bezier surfaces with degree more then 0 - 2", i.Eval("expshape r 0 0"), StringComparison.Ordinal);
        Assert.Contains("Number of Planes - 0\nNumber of other surfaces - 1", i.Eval("expshape r 0 0"), StringComparison.Ordinal);

        // converted to a revolution first, the cylinder becomes Bezier too
        Assert.Equal("ConvertToRevolution -> Result : \nStatus: DONE1\n", i.Eval("DT_ShapeConvertRev r c 1 1"));
        Assert.Contains("Number of Bezier surfaces with degree more then 0 - 3", i.Eval("expshape r 0 0"), StringComparison.Ordinal);
        Assert.Contains("Number of other surfaces - 0", i.Eval("expshape r 0 0"), StringComparison.Ordinal);

        i.Eval("DT_ToBspl r c");
        string anExp = i.Eval("expshape r 0 0");
        Assert.Contains("Number of Planes - 2", anExp, StringComparison.Ordinal);
        Assert.Contains("Number of other surfaces - 0", anExp, StringComparison.Ordinal);
        Assert.Contains("Use: DT_ToBspl result shape", Error(i, "DT_ToBspl r"), StringComparison.Ordinal);
    }

    [Fact]
    public void The_ShapeCustom_commands_report_as_Draw()
    {
        DrawInterpreter i = Load();
        i.Eval("pcylinder c 10 20");
        Shape aCylinder = i.GetShape("c");

        string anExp = i.Eval("expshape c 3 10");
        // the seam is counted at each of its two occurrences in the face (OCCT explores the face's edges)
        Assert.StartsWith("Number of seam edges - 2\n", anExp, StringComparison.Ordinal);
        Assert.Contains("Number of Planes - 2\nNumber of other surfaces - 1\n", anExp, StringComparison.Ordinal);
        Assert.Contains("Invalid argument Cont3e", Error(i, "expshape c 3 10 C7"), StringComparison.Ordinal);
        Assert.Contains("Incorrect number of arguments. Must be 3", Error(i, "expshape c 3"), StringComparison.Ordinal);

        Assert.Equal("No modif\n", i.Eval("directfaces r c"));
        Assert.Equal("ConvertToRevolution -> Result : r\n", i.Eval("convtorevol r c"));
        Assert.Equal("", i.Eval("scaleshape r c 2"));
        Assert.Matches(@"Mass :\s+6283\.19", i.Eval("vprops c"));
        Assert.Matches(@"Mass :\s+50265\.5", i.Eval("vprops r"));
        Assert.Contains("Must be 4", Error(i, "scaleshape r c"), StringComparison.Ordinal);

        i.Eval("bsplres r c 1e-3 1e-3 9 100 C1 C1 1 1");
        Assert.True(Count(i.GetShape("r"), ShapeType.Face) == 3);
        Assert.Contains("Must be 10", Error(i, "bsplres r c 1e-3"), StringComparison.Ordinal);

        Assert.True(ReferenceEquals(aCylinder.TShape(), i.GetShape("c").TShape()));
    }

    [Fact]
    public void The_curve_commands_split_and_offset_curves()
    {
        DrawInterpreter i = Load();
        // C0 B-splines of degree 1 and three spans
        i.Eval("2dbsplinecurve c2 1 4 0 2 1 1 2 1 3 2 0 0 1 1 0 1 1 1 1 2 1 1");
        // degree 1 with an inner knot of multiplicity 1 is C0 at each inner knot: three pieces
        string aPieces = i.Eval("DT_SplitCurve2d c2 1e-7 0");
        Assert.Equal("c2_1 c2_2 c2_3", aPieces);
        Geometry<BSplineCurve2D>(i, "c2_2");

        // an offset of a C0 curve is refused (Standard_ConstructionError), as in OCCT
        Assert.Contains("Offset on C0 curve", Error(i, "offset2dcurve o2 c2 1"), StringComparison.Ordinal);
        i.Eval("circle c2c 0 0 3; offset2dcurve o2 c2c 1");
        Geometry<OffsetCurve2D>(i, "o2");
        i.Eval("circle ci 0 0 0 5; point d 0 0 1; offsetcurve oc ci 2 d");
        Geometry<OffsetCurve3D>(i, "oc");
        Assert.Contains("result + curve + offset + Dir", Error(i, "offsetcurve oc ci 2"), StringComparison.Ordinal);

        i.Eval("bsplinecurve c3 1 4 0 2 1 1 2 1 3 2 0 0 0 1 1 0 0 1 1 1 0 1 2 1 0 1");
        Assert.Equal("c3_1 c3_2 c3_3", i.Eval("DT_SplitCurve c3 1e-7 0"));

        string aSurf = i.Eval("plane ps; trim ts ps 0 4 0 4; DT_SplitSurface rs ts 1e-7 1");
        Assert.Contains("single surf", aSurf, StringComparison.Ordinal);
        Assert.Contains("nb GlobalU ; nb GlobalV=5 5 0 1 2 3 4 0 1 2 3 4\n", aSurf, StringComparison.Ordinal);
        Assert.EndsWith("rs_4_4", aSurf, StringComparison.Ordinal);
    }

    [Fact]
    public void RemoveIntWires_removes_the_small_holes()
    {
        DrawInterpreter i = Load();
        i.Eval("box b 10 10 10; pcylinder cy 1 10; ttranslate cy 5 5 0; bcut s b cy");
        string aRes = i.Eval("RemoveIntWires r 10 s");
        // the hole's two circles, and the cylindrical face left with no wire
        Assert.Equal("2 internal wires were removed\n1 small faces were removed\n", aRes);
        Assert.Equal(6, Count(i.GetShape("r"), ShapeType.Face));
        Assert.Contains("Invalid type of first shape", Error(i, "explode s e; RemoveIntWires r 10 s_1"), StringComparison.Ordinal);
        Assert.Contains("bad number of arguments", Error(i, "RemoveIntWires r 10"), StringComparison.Ordinal);
    }
}
