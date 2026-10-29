// Port of: src/ModelingAlgorithms/TKGeomAlgo/GTests/Intf_Tool_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>OCCT's Intf_Tool GTests, then known configurations.</summary>
public class Intf_ToolTest
{
    private static Bnd_Box2d Box2d(double x, double y, double X, double Y)
    {
        var aBox = new Bnd_Box2d();
        aBox.Update(x, y, X, Y);
        return aBox;
    }

    private static Bnd_Box Box(double x, double y, double z, double X, double Y, double Z)
    {
        var aBox = new Bnd_Box();
        aBox.Update(x, y, z, X, Y, Z);
        return aBox;
    }

    private static void CheckSegments(Intf_Tool theTool, bool theCheckFinite)
    {
        int aNbSeg = theTool.NbSegments();
        Assert.InRange(aNbSeg, 0, 6);
        for (int i = 1; i <= aNbSeg; ++i)
        {
            double aBegin = theTool.BeginParam(i);
            double anEnd = theTool.EndParam(i);
            if (theCheckFinite)
            {
                Assert.True(double.IsFinite(aBegin) || aBegin == -Precision.Infinite(), $"Segment {i} begin={aBegin}");
                Assert.True(double.IsFinite(anEnd) || anEnd == Precision.Infinite(), $"Segment {i} end={anEnd}");
            }
            Assert.True(aBegin <= anEnd, $"Segment {i} begin > end");
        }
    }

    // Regression test for bugs #75/#76: Hypr2dBox must produce valid segments
    // with finite begin/end parameters and not overflow fixed-size arrays.
    [Fact]
    public void Hypr2dBox_ProducesValidSegments()
    {
        gp_Ax2d anAxis = new(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0));
        gp_Hypr2d aHypr = new(anAxis, 2.0, 1.0);
        Bnd_Box2d aDomain = Box2d(-5.0, -5.0, 5.0, 5.0);

        Bnd_Box2d aResult = new();
        var aTool = new Intf_Tool();
        aTool.Hypr2dBox(aHypr, aDomain, ref aResult);

        CheckSegments(aTool, true);
    }

    // Regression test for bugs #75/#76: Parab2dBox must produce valid segments.
    [Fact]
    public void Parab2dBox_ProducesValidSegments()
    {
        gp_Ax2d anAxis = new(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0));
        gp_Parab2d aParab = new(anAxis, 1.0);
        Bnd_Box2d aDomain = Box2d(-5.0, -5.0, 5.0, 5.0);

        Bnd_Box2d aResult = new();
        var aTool = new Intf_Tool();
        aTool.Parab2dBox(aParab, aDomain, ref aResult);

        CheckSegments(aTool, true);
    }

    // Regression test for bugs #75/#76: ParabBox (3D) must produce valid segments.
    [Fact]
    public void ParabBox_ProducesValidSegments()
    {
        gp_Parab aParab = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 1.0);
        Bnd_Box aDomain = Box(-5.0, -5.0, -5.0, 5.0, 5.0, 5.0);

        Bnd_Box aResult = new();
        var aTool = new Intf_Tool();
        aTool.ParabBox(aParab, aDomain, ref aResult);

        CheckSegments(aTool, false);
    }

    // HyprBox (3D) produces valid segments.
    [Fact]
    public void HyprBox_ProducesValidSegments()
    {
        gp_Hypr aHypr = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 2.0, 1.0);
        Bnd_Box aDomain = Box(-5.0, -5.0, -5.0, 5.0, 5.0, 5.0);

        Bnd_Box aResult = new();
        var aTool = new Intf_Tool();
        aTool.HyprBox(aHypr, aDomain, ref aResult);

        Assert.InRange(aTool.NbSegments(), 0, 6);
    }

    // Small domain that doesn't intersect the curve should produce zero segments.
    [Fact]
    public void Hypr2dBox_NoIntersection_ZeroSegments()
    {
        gp_Ax2d anAxis = new(new gp_Pnt2d(100, 100), new gp_Dir2d(1, 0));
        gp_Hypr2d aHypr = new(anAxis, 0.1, 0.05);
        Bnd_Box2d aDomain = Box2d(-1.0, -1.0, 1.0, 1.0);

        Bnd_Box2d aResult = new();
        var aTool = new Intf_Tool();
        aTool.Hypr2dBox(aHypr, aDomain, ref aResult);

        Assert.Equal(0, aTool.NbSegments());
    }

    // Known configurations (not in OCCT's GTests)

    [Fact]
    public void Lin2dBox_DiagonalLine_OneSegmentWithBoxParameters()
    {
        // y = x through the origin, unit direction: enters the box [-1,2]x[-1,2] at (-1,-1), leaves at (2,2).
        double aS = Math.Sqrt(0.5);
        gp_Lin2d aLin = new(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 1));
        Bnd_Box2d aDomain = Box2d(-1.0, -1.0, 2.0, 2.0);
        Bnd_Box2d aResult = new();
        var aTool = new Intf_Tool();
        aTool.Lin2dBox(aLin, aDomain, ref aResult);

        Assert.Equal(1, aTool.NbSegments());
        Assert.Equal(-1.0 / aS, aTool.BeginParam(1), 1e-12);
        Assert.Equal(2.0 / aS, aTool.EndParam(1), 1e-12);
        aResult.Get(out double x, out double y, out double X, out double Y);
        Assert.Equal(-1.0, x, 1e-12);
        Assert.Equal(-1.0, y, 1e-12);
        Assert.Equal(2.0, X, 1e-12);
        Assert.Equal(2.0, Y, 1e-12);
        Assert.Throws<Standard_OutOfRange>(() => aTool.BeginParam(2));
        Assert.Throws<Standard_OutOfRange>(() => aTool.EndParam(0));
    }

    [Fact]
    public void Lin2dBox_ReversedHorizontalLine()
    {
        // Line y = 0.5 going towards -X from (10, 0.5): parameters 7 (x = 3) to 10 (x = 0).
        gp_Lin2d aLin = new(new gp_Pnt2d(10, 0.5), new gp_Dir2d(-1, 0));
        Bnd_Box2d aDomain = Box2d(0.0, 0.0, 3.0, 1.0);
        Bnd_Box2d aResult = new();
        var aTool = new Intf_Tool();
        aTool.Lin2dBox(aLin, aDomain, ref aResult);

        Assert.Equal(1, aTool.NbSegments());
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
        gp_Lin2d aLin = new(new gp_Pnt2d(0, 5), new gp_Dir2d(1, 0));
        Bnd_Box2d aDomain = Box2d(-1.0, -1.0, 1.0, 1.0);
        Bnd_Box2d aResult = new();
        var aTool = new Intf_Tool();
        aTool.Lin2dBox(aLin, aDomain, ref aResult);

        Assert.Equal(0, aTool.NbSegments());
        Assert.True(aResult.IsVoid());
    }

    [Fact]
    public void Lin2dBox_WholeAndVoidDomains()
    {
        gp_Lin2d aLin = new(new gp_Pnt2d(1, 2), new gp_Dir2d(0, 1));
        var aTool = new Intf_Tool();

        Bnd_Box2d aWhole = new();
        aWhole.SetWhole();
        Bnd_Box2d aResult = new();
        aTool.Lin2dBox(aLin, aWhole, ref aResult);
        Assert.Equal(1, aTool.NbSegments());
        Assert.Equal(-Precision.Infinite(), aTool.BeginParam(1));
        Assert.Equal(Precision.Infinite(), aTool.EndParam(1));

        aTool.Lin2dBox(aLin, new Bnd_Box2d(), ref aResult);
        Assert.Equal(0, aTool.NbSegments());
        Assert.True(aResult.IsVoid());
    }

    [Fact]
    public void LinBox_LineThroughBox_OneSegment()
    {
        // Along Z through (0.5, 0.5): the unit cube is crossed between parameters 1 and 2 from z = -1.
        gp_Lin aLin = new(new gp_Pnt(0.5, 0.5, -1.0), new gp_Dir(0, 0, 1));
        Bnd_Box aDomain = Box(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
        Bnd_Box aResult = new();
        var aTool = new Intf_Tool();
        aTool.LinBox(aLin, aDomain, ref aResult);

        Assert.Equal(1, aTool.NbSegments());
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
        gp_Lin aLin = new(new gp_Pnt(5.0, 0.5, 0.0), new gp_Dir(0, 0, 1));
        Bnd_Box aDomain = Box(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
        Bnd_Box aResult = new();
        var aTool = new Intf_Tool();
        aTool.LinBox(aLin, aDomain, ref aResult);

        Assert.Equal(0, aTool.NbSegments());
        Assert.True(aResult.IsVoid());
    }

    [Fact]
    public void Parab2dBox_SegmentPointsInsideDomainAndBox()
    {
        // y^2 = 4 x (focal 1, axis X): the parabola's points between the segment bounds lie in the domain and in
        // the result box.
        gp_Parab2d aParab = new(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 1.0);
        Bnd_Box2d aDomain = Box2d(-5.0, -5.0, 5.0, 5.0);
        Bnd_Box2d aResult = new();
        var aTool = new Intf_Tool();
        aTool.Parab2dBox(aParab, aDomain, ref aResult);

        Assert.Equal(1, aTool.NbSegments());
        double aBegin = aTool.BeginParam(1);
        double anEnd = aTool.EndParam(1);
        // Enters and leaves through x = 5, at y = -sqrt(20) and y = sqrt(20) (the parameter is y).
        Assert.Equal(-Math.Sqrt(20.0), aBegin, 1e-9);
        Assert.Equal(Math.Sqrt(20.0), anEnd, 1e-9);
        for (int i = 0; i <= 20; i++)
        {
            gp_Pnt2d aP = ElCLib.Value(aBegin + (anEnd - aBegin) * i / 20.0, aParab);
            Assert.False(aDomain.Enlarged(1e-9).IsOut(aP));
            Assert.False(aResult.Enlarged(1e-9).IsOut(aP));
        }
    }

    [Fact]
    public void Hypr2dBox_DomainAroundVertexOnly()
    {
        // The domain contains the vertex (2, 0) of the hyperbola but is crossed by the branch: one segment around
        // parameter 0.
        gp_Hypr2d aHypr = new(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 2.0, 1.0);
        Bnd_Box2d aDomain = Box2d(1.0, -1.0, 3.0, 1.0);
        Bnd_Box2d aResult = new();
        var aTool = new Intf_Tool();
        aTool.Hypr2dBox(aHypr, aDomain, ref aResult);

        Assert.Equal(1, aTool.NbSegments());
        Assert.True(aTool.BeginParam(1) < 0.0);
        Assert.True(aTool.EndParam(1) > 0.0);
        Assert.Equal(-Math.Asinh(1.0), aTool.BeginParam(1), 1e-9);
        Assert.Equal(Math.Asinh(1.0), aTool.EndParam(1), 1e-9);
        Assert.False(aResult.IsOut(ElCLib.Value(0.0, aHypr)));
    }

    [Fact]
    public void ParabBox_SegmentPointsInsideDomain()
    {
        // In its plane XOY with axis X: y^2 = 4 x; the box [-5, 5]^3 cuts it at x = 5, y = -sqrt(20) and y = sqrt(20).
        gp_Parab aParab = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 1.0);
        Bnd_Box aDomain = Box(-5.0, -5.0, -5.0, 5.0, 5.0, 5.0);
        Bnd_Box aResult = new();
        var aTool = new Intf_Tool();
        aTool.ParabBox(aParab, aDomain, ref aResult);

        Assert.Equal(1, aTool.NbSegments());
        Assert.Equal(-Math.Sqrt(20.0), aTool.BeginParam(1), 1e-9);
        Assert.Equal(Math.Sqrt(20.0), aTool.EndParam(1), 1e-9);
        for (int i = 0; i <= 20; i++)
        {
            gp_Pnt aP = ElCLib.Value(-Math.Sqrt(20.0) + 2.0 * Math.Sqrt(20.0) * i / 20.0, aParab);
            Assert.False(aResult.Enlarged(1e-9).IsOut(aP));
        }
    }
}
