// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Hashing;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// The port's own tests of GeomTools: round trips of random geometry of every kind, the readable dumps, the sets,
/// unknown types and malformed text. Byte-for-byte comparisons with native OCCT are in the oracle tests.
/// </summary>
public class GeomTools_PortTests
{
    private static string Write(global::Kerf.Engine.Exchange.Brep.CurveSet theSet)
    {
        var aWriter = new StringWriter();
        theSet.Write(aWriter);
        return aWriter.ToString();
    }

    private static string Write(global::Kerf.Engine.Exchange.Brep.Curve2dSet theSet)
    {
        var aWriter = new StringWriter();
        theSet.Write(aWriter);
        return aWriter.ToString();
    }

    private static string Write(global::Kerf.Engine.Exchange.Brep.SurfaceSet theSet)
    {
        var aWriter = new StringWriter();
        theSet.Write(aWriter);
        return aWriter.ToString();
    }

    // kinds whose text holds no direction, which reading normalises again: their text comes back exactly
    private static bool HasNoDirection(object theGeometry)
        => theGeometry is BezierCurve3D or BSplineCurve3D or BezierSurface or BSplineSurface
                          or BezierCurve2D or BSplineCurve2D;

    [Fact]
    public void Random_curves_of_every_kind_round_trip()
    {
        var aRand = new Random(20261029);
        var aSet = new global::Kerf.Engine.Exchange.Brep.CurveSet();
        var aCurves = new List<Curve3D>();
        for (int i = 0; i < 550; i++)
        {
            Curve3D aCurve = GeomHash_PortTests.Curve(aRand, i % 11);
            aCurves.Add(aCurve);
            Assert.Equal(i + 1, aSet.Add(aCurve));
        }

        Assert.Equal(1, aSet.Add(aCurves[0]));
        string aText = Write(aSet);
        Assert.StartsWith("Curves 550\n", aText, StringComparison.Ordinal);

        var aRead = new global::Kerf.Engine.Exchange.Brep.CurveSet();
        aRead.Read(new StringReader(aText));
        var aHasher = new CurveHasher();
        for (int i = 1; i <= aCurves.Count; i++)
        {
            Curve3D? aCurve = aRead.Curve(i);
            Assert.NotNull(aCurve);
            Assert.True(aCurve.IsFrozen);
            Assert.Equal(aCurves[i - 1].DynamicType(), aCurve.DynamicType());
            Assert.True(aHasher.Equals(aCurves[i - 1], aCurve), $"curve {i}, {aCurve.DynamicTypeName()}");
            Assert.Equal(i, aRead.Index(aCurve));

            var aWriter = new StringWriter();
            StreamUtilities.Precision(aWriter, 17);
            global::Kerf.Engine.Exchange.Brep.GeomTools.Write(aCurves[i - 1], aWriter);
            string anOne = aWriter.ToString();
            aWriter = new StringWriter();
            StreamUtilities.Precision(aWriter, 17);
            global::Kerf.Engine.Exchange.Brep.GeomTools.Write(aCurve, aWriter);
            if (HasNoDirection(aCurve))
            {
                Assert.Equal(anOne, aWriter.ToString());
            }

            global::Kerf.Engine.Exchange.Brep.GeomTools.Read(out Curve3D? aSingle, new StringReader(anOne));
            Assert.NotNull(aSingle);
            Assert.False(aSingle.IsFrozen);
            Assert.True(aHasher.Equals(aCurves[i - 1], aSingle));
        }

        Assert.Null(aRead.Curve(0));
        Assert.Null(aRead.Curve(551));
    }

    [Fact]
    public void Random_2d_curves_of_every_kind_round_trip()
    {
        var aRand = new Random(20261030);
        var aSet = new global::Kerf.Engine.Exchange.Brep.Curve2dSet();
        var aCurves = new List<Curve2D>();
        for (int i = 0; i < 500; i++)
        {
            Curve2D aCurve = GeomHash_PortTests.Curve2d(aRand, i % 10);
            aCurves.Add(aCurve);
            aSet.Add(aCurve);
        }

        string aText = Write(aSet);
        Assert.StartsWith("Curve2ds 500\n", aText, StringComparison.Ordinal);

        var aRead = new global::Kerf.Engine.Exchange.Brep.Curve2dSet();
        aRead.Read(new StringReader(aText));
        var aHasher = new CurveHasher2D();
        for (int i = 1; i <= aCurves.Count; i++)
        {
            Curve2D? aCurve = aRead.Curve2d(i);
            Assert.NotNull(aCurve);
            Assert.True(aCurve.IsFrozen);
            Assert.Equal(aCurves[i - 1].DynamicType(), aCurve.DynamicType());
            Assert.True(aHasher.Equals(aCurves[i - 1], aCurve), $"curve {i}, {aCurve.DynamicTypeName()}");
            if (HasNoDirection(aCurve))
            {
                var aSingle = new global::Kerf.Engine.Exchange.Brep.Curve2dSet();
                aSingle.Add(aCurve);
                var anOriginal = new global::Kerf.Engine.Exchange.Brep.Curve2dSet();
                anOriginal.Add(aCurves[i - 1]);
                Assert.Equal(Write(anOriginal), Write(aSingle));
            }
        }
    }

    [Fact]
    public void Random_surfaces_of_every_kind_round_trip()
    {
        var aRand = new Random(20261031);
        var aSet = new global::Kerf.Engine.Exchange.Brep.SurfaceSet();
        var aSurfaces = new List<Surface>();
        for (int i = 0; i < 600; i++)
        {
            Surface aSurface = GeomHash_PortTests.Surface(aRand, i % 12);
            aSurfaces.Add(aSurface);
            aSet.Add(aSurface);
        }

        string aText = Write(aSet);
        Assert.StartsWith("Surfaces 600\n", aText, StringComparison.Ordinal);

        var aRead = new global::Kerf.Engine.Exchange.Brep.SurfaceSet();
        aRead.Read(new StringReader(aText));
        var aHasher = new SurfaceHasher();
        for (int i = 1; i <= aSurfaces.Count; i++)
        {
            Surface? aSurface = aRead.Surface(i);
            Assert.NotNull(aSurface);
            Assert.True(aSurface.IsFrozen);
            Assert.Equal(aSurfaces[i - 1].DynamicType(), aSurface.DynamicType());
            Assert.True(aHasher.Equals(aSurfaces[i - 1], aSurface), $"surface {i}, {aSurface.DynamicTypeName()}");
            if (HasNoDirection(aSurface))
            {
                var aWriter = new StringWriter();
                StreamUtilities.Precision(aWriter, 17);
                global::Kerf.Engine.Exchange.Brep.GeomTools.Write(aSurface, aWriter);
                var anOriginal = new StringWriter();
                StreamUtilities.Precision(anOriginal, 17);
                global::Kerf.Engine.Exchange.Brep.GeomTools.Write(aSurfaces[i - 1], anOriginal);
                Assert.Equal(anOriginal.ToString(), aWriter.ToString());
            }
        }
    }

    [Fact]
    public void Left_handed_axes_are_read_back_left_handed()
    {
        var anAx3 = new CoordinateSystem(new Point3D(1, 2, 3), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0));
        anAx3 = anAx3.YReversed();
        var aCylinder = new CylindricalSurface(anAx3, 2.0);
        var aWriter = new StringWriter();
        global::Kerf.Engine.Exchange.Brep.GeomTools.Write(aCylinder, aWriter);
        Assert.Equal("2 1 2 3 0 0 1 1 0 -0 0 -1 -0 2\n", aWriter.ToString());

        global::Kerf.Engine.Exchange.Brep.GeomTools.Read(out Surface? aRead, new StringReader(aWriter.ToString()));
        var aCylinder2 = Assert.IsType<CylindricalSurface>(aRead);
        Assert.False(aCylinder2.Position.IsDirect);
        Assert.Equal(-1.0, aCylinder2.Position.YDirection.Y);
    }

    [Fact]
    public void Write_uses_the_writer_precision_and_restores_it()
    {
        var aCircle = new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 1.0 / 3.0);
        var aWriter = new StringWriter();
        global::Kerf.Engine.Exchange.Brep.GeomTools.Write(aCircle, aWriter);
        Assert.Equal("2 0 0 0 0 0 1 1 0 -0 -0 1 0 0.333333\n", aWriter.ToString());

        var aSet = new global::Kerf.Engine.Exchange.Brep.CurveSet();
        aSet.Add(aCircle);
        aWriter = new StringWriter();
        aSet.Write(aWriter);
        Assert.Equal("Curves 1\n2 0 0 0 0 0 1 1 0 -0 -0 1 0 0.33333333333333331\n", aWriter.ToString());
        Assert.Equal(6, StreamUtilities.Precision(aWriter));
    }

    [Fact]
    public void Dumps_are_OCCT_s()
    {
        var aLine = new LineCurve3D(new Point3D(1, 2, 3), new Direction3D(0, 0, 1));
        var aWriter = new StringWriter();
        global::Kerf.Engine.Exchange.Brep.GeomTools.Dump(aLine, aWriter);
        Assert.Equal("Line\n  Origin :1, 2, 3 \n  Axis   :0, 0, 1 \n\n", aWriter.ToString());

        var aPoles = new NCollection_Array1<Point3D>(1, 2);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(1, 0, 0);
        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots[1] = 0;
        aKnots[2] = 1;
        var aMults = new NCollection_Array1<int>(1, 2);
        aMults[1] = 2;
        aMults[2] = 2;
        var aSpline = new BSplineCurve3D(aPoles, aKnots, aMults, 1);
        var aSet = new global::Kerf.Engine.Exchange.Brep.CurveSet();
        aSet.Add(aLine);
        aSet.Add(aSpline);
        aWriter = new StringWriter();
        aSet.Dump(aWriter);
        Assert.Equal("\n -------\nDump of 2 Curves \n -------\n\n"
                     + "   1 : Line\n  Origin :1, 2, 3 \n  Axis   :0, 0, 1 \n\n"
                     + "   2 : BSplineCurve\n  Degree 1, 2 Poles, 2  Knots\nPoles :\n\n   1 : 0, 0, 0 \n   2 : 1, 0, 0 \n"
                     + "Knots :\n\n   1 :  0 2\n   2 :  1 2\n\n",
                     aWriter.ToString());

        var aSphere = new SphericalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 2.0);
        aWriter = new StringWriter();
        global::Kerf.Engine.Exchange.Brep.GeomTools.Dump(aSphere, aWriter);
        Assert.Equal("SphericalSurface\n  Center :0, 0, 0 \n  Axis   :0, 0, 1 \n  XAxis  :1, 0, -0 \n  YAxis  :-0, 1, 0 \n"
                     + "  Radius :2\n\n", aWriter.ToString());

        var aCircle2d = new CircleCurve2D(new Axis2D(new Point2D(1, 1), new Direction2D(1, 0)), 0.5);
        aWriter = new StringWriter();
        global::Kerf.Engine.Exchange.Brep.GeomTools.Dump(aCircle2d, aWriter);
        Assert.Equal("Circle\n  Center :1, 1 \n  XAxis  :1, 0 \n  YAxis  :-0, 1 \n  Radius :0.5\n\n", aWriter.ToString());
    }

    [Fact]
    public void Sets_index_and_null_as_OCCT_does()
    {
        var aLine = new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        var aCurves = new global::Kerf.Engine.Exchange.Brep.CurveSet();
        Assert.Equal(0, aCurves.Add(null));
        Assert.Equal(1, aCurves.Add(aLine));
        Assert.Equal(1, aCurves.Add(aLine));
        Assert.Equal(2, aCurves.Add((Curve3D)aLine.Copy()));
        Assert.Equal(0, aCurves.Index(null));
        Assert.Equal(0, aCurves.Index(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0))));
        Assert.False(aLine.IsFrozen);
        aCurves.Clear();
        Assert.Null(aCurves.Curve(1));

        // the 2D and surface sets add a null handle, once
        var aCurves2d = new global::Kerf.Engine.Exchange.Brep.Curve2dSet();
        Assert.Equal(1, aCurves2d.Add(null));
        Assert.Equal(1, aCurves2d.Add(null));
        Assert.Equal(1, aCurves2d.Index(null));
        Assert.Null(aCurves2d.Curve2d(1));
        var aSurfaces = new global::Kerf.Engine.Exchange.Brep.SurfaceSet();
        Assert.Equal(0, aSurfaces.Index(null));
        Assert.Equal(1, aSurfaces.Add(null));
        Assert.Equal(1, aSurfaces.Index(null));
    }

    [Fact]
    public void Curves_that_cannot_be_built_are_one_null_entry()
    {
        // a null direction (gp_Dir throws), a good line, a circle with a negative radius, a good circle
        const string aText = "Curves 4\n1 0 0 0 0 0 0 \n1 1 2 3 0 0 1 \n2 0 0 0 0 0 1 1 0 0 0 1 0 -1\n2 0 0 0 0 0 1 1 0 0 0 1 0 1\n";
        var aSet = new global::Kerf.Engine.Exchange.Brep.CurveSet();
        aSet.Read(new StringReader(aText));

        // OCCT's map holds the null handle once: four curves read, three entries
        Assert.Null(aSet.Curve(1));
        Assert.IsType<LineCurve3D>(aSet.Curve(2));
        Assert.IsType<CircleCurve3D>(aSet.Curve(3));
        Assert.Null(aSet.Curve(4));

        // a trimmed curve of a curve that can't be read
        global::Kerf.Engine.Exchange.Brep.GeomTools.Read(out Curve3D? aTrimmed, new StringReader("8 0 1\n1 0 0 0 0 0 0 \n"));
        Assert.Null(aTrimmed);
    }

    [Fact]
    public void Truncated_and_foreign_text_reads_as_OCCT_reads_it()
    {
        // the stream ends in the middle of a B-spline: the missing numbers are 0, and the curve can't be built
        global::Kerf.Engine.Exchange.Brep.GeomTools.Read(out Curve3D? aCurve, new StringReader("7 0 0  1 2 2  0 0 0  1 0"));
        Assert.Null(aCurve);

        // not a table of curves: nothing is read
        var aSet = new global::Kerf.Engine.Exchange.Brep.CurveSet();
        aSet.Read(new StringReader("Surfaces 1\n1 0 0 0 0 0 1 1 0 0 0 1 0 \n"));
        Assert.Null(aSet.Curve(1));

        // GetReal reads the numeric prefix of a word, and 0 for none
        var aReader = new StringReader("1.5abc xyz 2e3");
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(aReader, out double aValue);
        Assert.Equal(1.5, aValue);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(aReader, out aValue);
        Assert.Equal(0.0, aValue);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(aReader, out aValue);
        Assert.Equal(2000.0, aValue);
        Assert.True(StreamUtilities.Eof(aReader));
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(aReader, out aValue);
        Assert.Equal(0.0, aValue);

        // the sets read OCCT's files with CRLF line ends
        var aSurfaces = new global::Kerf.Engine.Exchange.Brep.SurfaceSet();
        aSurfaces.Read(new StringReader("Surfaces 1\r\n1 0 0 0 0 0 1 1 0 0 0 1 0 \r\n"));
        Assert.IsType<PlaneSurface>(aSurfaces.Surface(1));
    }

    private sealed class ReadingHandler : global::Kerf.Engine.Exchange.Brep.UndefinedTypeHandler
    {
        public int Printed;

        public override void PrintCurve(Curve3D C, TextWriter OS, bool compact = false)
        {
            Printed++;
            OS.Write("42 ");
            StreamUtilities.Write(OS, ((TestLine)C).Origin().X);
            OS.Write("\n");
        }

        public override TextReader ReadCurve(int ctype, TextReader IS, ref Curve3D? C)
        {
            if (ctype == 42)
            {
                global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out double anX);
                C = new TestLine(new Point3D(anX, 0, 0), new Vector3D(1, 0, 0));
            }

            return IS;
        }
    }

    [Fact]
    public void Unknown_types_go_to_the_handler()
    {
        var aTestLine = new TestLine(new Point3D(7, 0, 0), new Vector3D(1, 0, 0));
        var aWriter = new StringWriter();
        global::Kerf.Engine.Exchange.Brep.GeomTools.Dump(aTestLine, aWriter);
        Assert.Equal("****** UNKNOWN CURVE TYPE ******\n", aWriter.ToString());
        aWriter = new StringWriter();
        global::Kerf.Engine.Exchange.Brep.GeomTools.Dump(new TestSaddle(), aWriter);
        Assert.Equal("****** UNKNOWN SURFACE TYPE ******\n", aWriter.ToString());

        global::Kerf.Engine.Exchange.Brep.GeomTools.Read(out Curve3D? anUnknown, new StringReader("42 7\n"));
        Assert.Null(anUnknown);

        global::Kerf.Engine.Exchange.Brep.UndefinedTypeHandler aDefault = global::Kerf.Engine.Exchange.Brep.GeomTools.GetUndefinedTypeHandler();
        var aHandler = new ReadingHandler();
        try
        {
            global::Kerf.Engine.Exchange.Brep.GeomTools.SetUndefinedTypeHandler(aHandler);
            global::Kerf.Engine.Exchange.Brep.GeomTools.SetUndefinedTypeHandler(null);
            Assert.Same(aHandler, global::Kerf.Engine.Exchange.Brep.GeomTools.GetUndefinedTypeHandler());

            var aSet = new global::Kerf.Engine.Exchange.Brep.CurveSet();
            aSet.Add(aTestLine);
            aWriter = new StringWriter();
            aSet.Write(aWriter);
            Assert.Equal("Curves 1\n42 7\n", aWriter.ToString());
            Assert.Equal(1, aHandler.Printed);

            var aRead = new global::Kerf.Engine.Exchange.Brep.CurveSet();
            aRead.Read(new StringReader(aWriter.ToString()));
            Assert.Equal(7.0, Assert.IsType<TestLine>(aRead.Curve(1)).Origin().X);
        }
        finally
        {
            global::Kerf.Engine.Exchange.Brep.GeomTools.SetUndefinedTypeHandler(aDefault);
        }
    }
}
