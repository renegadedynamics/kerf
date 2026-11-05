// Port of: src/ModelingAlgorithms/TKGeomAlgo/GTests/Intf_Tool_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;

using Kerf.Engine.Intersection.Polygons;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>OCCT's Intf_Tool GTests, then known configurations.</summary>
public class Intf_ToolTest
{
    private static Box2d Box2d(double x, double y, double X, double Y)
    {
        var aBox = new Box2d();
        aBox.Update(x, y, X, Y);
        return aBox;
    }

    private static Box Box(double x, double y, double z, double X, double Y, double Z)
    {
        var aBox = new Box();
        aBox.Update(x, y, z, X, Y, Z);
        return aBox;
    }

    private static void CheckSegments(PolygonTool theTool, bool theCheckFinite)
    {
        int aNbSeg = theTool.SegmentCount;
        Assert.InRange(aNbSeg, 0, 6);
        for (int i = 1; i <= aNbSeg; ++i)
        {
            double aBegin = theTool.BeginParam(i);
            double anEnd = theTool.EndParam(i);
            if (theCheckFinite)
            {
                Assert.True(double.IsFinite(aBegin) || aBegin == -global::Kerf.Engine.Foundation.Precision.Infinite(), $"Segment {i} begin={aBegin}");
                Assert.True(double.IsFinite(anEnd) || anEnd == global::Kerf.Engine.Foundation.Precision.Infinite(), $"Segment {i} end={anEnd}");
            }
            Assert.True(aBegin <= anEnd, $"Segment {i} begin > end");
        }
    }

    // Regression test for bugs #75/#76: Hypr2dBox must produce valid segments
    // with finite begin/end parameters and not overflow fixed-size arrays.
    [Fact]
    public void Hypr2dBox_ProducesValidSegments()
    {
        Axis2D anAxis = new(new Point2D(0, 0), new Direction2D(1, 0));
        Hyperbola2D aHypr = new(anAxis, 2.0, 1.0);
        Box2d aDomain = Box2d(-5.0, -5.0, 5.0, 5.0);

        Box2d aResult = new();
        var aTool = new PolygonTool();
        aTool.Hypr2dBox(aHypr, aDomain, ref aResult);

        CheckSegments(aTool, true);
    }

    // Regression test for bugs #75/#76: Parab2dBox must produce valid segments.
    [Fact]
    public void Parab2dBox_ProducesValidSegments()
    {
        Axis2D anAxis = new(new Point2D(0, 0), new Direction2D(1, 0));
        Parabola2D aParab = new(anAxis, 1.0);
        Box2d aDomain = Box2d(-5.0, -5.0, 5.0, 5.0);

        Box2d aResult = new();
        var aTool = new PolygonTool();
        aTool.Parab2dBox(aParab, aDomain, ref aResult);

        CheckSegments(aTool, true);
    }

    // Regression test for bugs #75/#76: ParabBox (3D) must produce valid segments.
    [Fact]
    public void ParabBox_ProducesValidSegments()
    {
        Parabola3D aParab = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 1.0);
        Box aDomain = Box(-5.0, -5.0, -5.0, 5.0, 5.0, 5.0);

        Box aResult = new();
        var aTool = new PolygonTool();
        aTool.ParabBox(aParab, aDomain, ref aResult);

        CheckSegments(aTool, false);
    }

    // HyprBox (3D) produces valid segments.
    [Fact]
    public void HyprBox_ProducesValidSegments()
    {
        Hyperbola3D aHypr = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 2.0, 1.0);
        Box aDomain = Box(-5.0, -5.0, -5.0, 5.0, 5.0, 5.0);

        Box aResult = new();
        var aTool = new PolygonTool();
        aTool.HyprBox(aHypr, aDomain, ref aResult);

        Assert.InRange(aTool.SegmentCount, 0, 6);
    }

    // Small domain that doesn't intersect the curve should produce zero segments.
    [Fact]
    public void Hypr2dBox_NoIntersection_ZeroSegments()
    {
        Axis2D anAxis = new(new Point2D(100, 100), new Direction2D(1, 0));
        Hyperbola2D aHypr = new(anAxis, 0.1, 0.05);
        Box2d aDomain = Box2d(-1.0, -1.0, 1.0, 1.0);

        Box2d aResult = new();
        var aTool = new PolygonTool();
        aTool.Hypr2dBox(aHypr, aDomain, ref aResult);

        Assert.Equal(0, aTool.SegmentCount);
    }

    // Known configurations (not in OCCT's GTests)

    [Fact]
    public void Lin2dBox_DiagonalLine_OneSegmentWithBoxParameters()
    {
        // y = x through the origin, unit direction: enters the box [-1,2]x[-1,2] at (-1,-1), leaves at (2,2).
        double aS = Math.Sqrt(0.5);
        Line2D aLin = new(new Point2D(0, 0), new Direction2D(1, 1));
        Box2d aDomain = Box2d(-1.0, -1.0, 2.0, 2.0);
        Box2d aResult = new();
        var aTool = new PolygonTool();
        aTool.Lin2dBox(aLin, aDomain, ref aResult);

        Assert.Equal(1, aTool.SegmentCount);
        Assert.Equal(-1.0 / aS, aTool.BeginParam(1), 1e-12);
        Assert.Equal(2.0 / aS, aTool.EndParam(1), 1e-12);
        aResult.Get(out double x, out double y, out double X, out double Y);
        Assert.Equal(-1.0, x, 1e-12);
        Assert.Equal(-1.0, y, 1e-12);
        Assert.Equal(2.0, X, 1e-12);
        Assert.Equal(2.0, Y, 1e-12);
        EngineAssert.Throws<global::Kerf.Engine.Foundation.OutOfRangeException>(() => aTool.BeginParam(2));
        EngineAssert.Throws<global::Kerf.Engine.Foundation.OutOfRangeException>(() => aTool.EndParam(0));
    }

    [Fact]
    public void Lin2dBox_ReversedHorizontalLine()
    {
        // Line y = 0.5 going towards -X from (10, 0.5): parameters 7 (x = 3) to 10 (x = 0).
        Line2D aLin = new(new Point2D(10, 0.5), new Direction2D(-1, 0));
        Box2d aDomain = Box2d(0.0, 0.0, 3.0, 1.0);
        Box2d aResult = new();
        var aTool = new PolygonTool();
        aTool.Lin2dBox(aLin, aDomain, ref aResult);

        Assert.Equal(1, aTool.SegmentCount);
        Assert.Equal(7.0, aTool.BeginParam(1), 1e-12);
        Assert.Equal(10.0, aTool.EndParam(1), 1e-12);
        aResult.Get(out double x, out double y, out double X, out double Y);
        Assert.Equal(0.0, x, 1e-12);
        Assert.Equal(0.5, y, 1e-12);
        Assert.Equal(3.0, X, 1e-12);
        Assert.Equal(0.5, Y, 1e-12);
    }

    [Fact]
    public void Lin2dBox_ParallelLineOutsideBox_NoSegment()
    {
        Line2D aLin = new(new Point2D(0, 5), new Direction2D(1, 0));
        Box2d aDomain = Box2d(-1.0, -1.0, 1.0, 1.0);
        Box2d aResult = new();
        var aTool = new PolygonTool();
        aTool.Lin2dBox(aLin, aDomain, ref aResult);

        Assert.Equal(0, aTool.SegmentCount);
        Assert.True(aResult.IsVoid);
    }

    [Fact]
    public void Lin2dBox_WholeAndVoidDomains()
    {
        Line2D aLin = new(new Point2D(1, 2), new Direction2D(0, 1));
        var aTool = new PolygonTool();

        Box2d aWhole = new();
        aWhole.SetWhole();
        Box2d aResult = new();
        aTool.Lin2dBox(aLin, aWhole, ref aResult);
        Assert.Equal(1, aTool.SegmentCount);
        Assert.Equal(-global::Kerf.Engine.Foundation.Precision.Infinite(), aTool.BeginParam(1));
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Infinite(), aTool.EndParam(1));

        aTool.Lin2dBox(aLin, new Box2d(), ref aResult);
        Assert.Equal(0, aTool.SegmentCount);
        Assert.True(aResult.IsVoid);
    }

    [Fact]
    public void LinBox_LineThroughBox_OneSegment()
    {
        // Along Z through (0.5, 0.5): the unit cube is crossed between parameters 1 and 2 from z = -1.
        Line3D aLin = new(new Point3D(0.5, 0.5, -1.0), new Direction3D(0, 0, 1));
        Box aDomain = Box(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
        Box aResult = new();
        var aTool = new PolygonTool();
        aTool.LinBox(aLin, aDomain, ref aResult);

        Assert.Equal(1, aTool.SegmentCount);
        Assert.Equal(1.0, aTool.BeginParam(1), 1e-12);
        Assert.Equal(2.0, aTool.EndParam(1), 1e-12);
        aResult.Get(out double x, out double y, out double z, out double X, out double Y, out double Z);
        Assert.Equal(0.5, x, 1e-12);
        Assert.Equal(0.5, y, 1e-12);
        Assert.Equal(0.0, z, 1e-12);
        Assert.Equal(0.5, X, 1e-12);
        Assert.Equal(0.5, Y, 1e-12);
        Assert.Equal(1.0, Z, 1e-12);
    }

    [Fact]
    public void LinBox_LineMissingBox_ZeroSegments()
    {
        // Parallel to Z, outside the box in X: no segment.
        Line3D aLin = new(new Point3D(5.0, 0.5, 0.0), new Direction3D(0, 0, 1));
        Box aDomain = Box(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
        Box aResult = new();
        var aTool = new PolygonTool();
        aTool.LinBox(aLin, aDomain, ref aResult);

        Assert.Equal(0, aTool.SegmentCount);
        Assert.True(aResult.IsVoid);
    }

    [Fact]
    public void Parab2dBox_SegmentPointsInsideDomainAndBox()
    {
        // y^2 = 4 x (focal 1, axis X): the parabola's points between the segment bounds lie in the domain and in
        // the result box.
        Parabola2D aParab = new(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 1.0);
        Box2d aDomain = Box2d(-5.0, -5.0, 5.0, 5.0);
        Box2d aResult = new();
        var aTool = new PolygonTool();
        aTool.Parab2dBox(aParab, aDomain, ref aResult);

        Assert.Equal(1, aTool.SegmentCount);
        double aBegin = aTool.BeginParam(1);
        double anEnd = aTool.EndParam(1);
        // Enters and leaves through x = 5, at y = -sqrt(20) and y = sqrt(20) (the parameter is y).
        Assert.Equal(-Math.Sqrt(20.0), aBegin, 1e-9);
        Assert.Equal(Math.Sqrt(20.0), anEnd, 1e-9);
        for (int i = 0; i <= 20; i++)
        {
            Point2D aP = global::Kerf.Engine.Geometry.ElCLib.Value(aBegin + (anEnd - aBegin) * i / 20.0, aParab);
            Assert.False(aDomain.Enlarged(1e-9).IsOut(aP));
            Assert.False(aResult.Enlarged(1e-9).IsOut(aP));
        }
    }

    [Fact]
    public void Hypr2dBox_DomainAroundVertexOnly()
    {
        // The domain contains the vertex (2, 0) of the hyperbola but is crossed by the branch: one segment around
        // parameter 0.
        Hyperbola2D aHypr = new(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 2.0, 1.0);
        Box2d aDomain = Box2d(1.0, -1.0, 3.0, 1.0);
        Box2d aResult = new();
        var aTool = new PolygonTool();
        aTool.Hypr2dBox(aHypr, aDomain, ref aResult);

        Assert.Equal(1, aTool.SegmentCount);
        Assert.True(aTool.BeginParam(1) < 0.0);
        Assert.True(aTool.EndParam(1) > 0.0);
        Assert.Equal(-Math.Asinh(1.0), aTool.BeginParam(1), 1e-9);
        Assert.Equal(Math.Asinh(1.0), aTool.EndParam(1), 1e-9);
        Assert.False(aResult.IsOut(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, aHypr)));
    }

    [Fact]
    public void ParabBox_SegmentPointsInsideDomain()
    {
        // In its plane XOY with axis X: y^2 = 4 x; the box [-5, 5]^3 cuts it at x = 5, y = -sqrt(20) and y = sqrt(20).
        Parabola3D aParab = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 1.0);
        Box aDomain = Box(-5.0, -5.0, -5.0, 5.0, 5.0, 5.0);
        Box aResult = new();
        var aTool = new PolygonTool();
        aTool.ParabBox(aParab, aDomain, ref aResult);

        Assert.Equal(1, aTool.SegmentCount);
        Assert.Equal(-Math.Sqrt(20.0), aTool.BeginParam(1), 1e-9);
        Assert.Equal(Math.Sqrt(20.0), aTool.EndParam(1), 1e-9);
        for (int i = 0; i <= 20; i++)
        {
            Point3D aP = global::Kerf.Engine.Geometry.ElCLib.Value(-Math.Sqrt(20.0) + 2.0 * Math.Sqrt(20.0) * i / 20.0, aParab);
            Assert.False(aResult.Enlarged(1e-9).IsOut(aP));
        }
    }
}
