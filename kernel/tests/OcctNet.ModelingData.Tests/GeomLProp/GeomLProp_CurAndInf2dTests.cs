// Port of: src/ModelingData/TKGeomBase/GTests/GeomLProp_CurAndInf2d_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Occt.LProp_CIType;

namespace OcctNet.ModelingData.Tests;

public class GeomLProp_CurAndInf2dTest
{
    private readonly Geom2d_Circle myCircle;
    private readonly Geom2d_Ellipse myEllipse;
    private readonly Geom2d_Hyperbola myHyperbola;
    private readonly Geom2d_Parabola myParabola;

    public GeomLProp_CurAndInf2dTest()
    {
        // Circle centered at origin, radius 5
        var aCirc = new Circle2D(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 5.0);
        myCircle = new Geom2d_Circle(aCirc);

        // Ellipse centered at origin, major radius 10, minor radius 3
        var anElips = new Ellipse2D(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 10.0, 3.0);
        myEllipse = new Geom2d_Ellipse(anElips);

        var aHypr = new Hyperbola2D(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 6.0, 3.0);
        myHyperbola = new Geom2d_Hyperbola(aHypr);

        var aParab = new Parabola2D(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 2.0);
        myParabola = new Geom2d_Parabola(aParab);
    }

    [Fact]
    public void Circle_Perform_NoInflections()
    {
        // A circle has constant curvature: no inflections or extrema
        var aAnalyzer = new GeomLProp_CurAndInf2d();
        aAnalyzer.Perform(myCircle);

        Assert.True(aAnalyzer.IsDone());
        Assert.Equal(0, aAnalyzer.NbPoints());
    }

    [Fact]
    public void Circle_PerformInf_NoInflections()
    {
        var aAnalyzer = new GeomLProp_CurAndInf2d();
        aAnalyzer.PerformInf(myCircle);

        Assert.True(aAnalyzer.IsDone());
        Assert.Equal(0, aAnalyzer.NbPoints());
    }

    [Fact]
    public void Circle_PerformCurExt_NoExtrema()
    {
        var aAnalyzer = new GeomLProp_CurAndInf2d();
        aAnalyzer.PerformCurExt(myCircle);

        Assert.True(aAnalyzer.IsDone());
        Assert.Equal(0, aAnalyzer.NbPoints());
    }

    [Fact]
    public void Ellipse_PerformCurExt_HasExtrema()
    {
        // An ellipse has 4 curvature extrema:
        // 2 maxima (at ends of minor axis) and 2 minima (at ends of major axis)
        var aAnalyzer = new GeomLProp_CurAndInf2d();
        aAnalyzer.PerformCurExt(myEllipse);

        Assert.True(aAnalyzer.IsDone());
        Assert.Equal(4, aAnalyzer.NbPoints());
    }

    [Fact]
    public void Ellipse_PerformCurExt_Types()
    {
        var aAnalyzer = new GeomLProp_CurAndInf2d();
        aAnalyzer.PerformCurExt(myEllipse);

        Assert.True(aAnalyzer.IsDone());
        Assert.Equal(4, aAnalyzer.NbPoints());

        // Count min and max curvature points
        int aNbMin = 0;
        int aNbMax = 0;
        for (int i = 1; i <= aAnalyzer.NbPoints(); ++i)
        {
            LProp_CIType aType = aAnalyzer.Type(i);
            if (aType == LProp_MinCur)
            {
                ++aNbMin;
            }
            else if (aType == LProp_MaxCur)
            {
                ++aNbMax;
            }
        }

        Assert.Equal(2, aNbMin);
        Assert.Equal(2, aNbMax);
    }

    [Fact]
    public void Ellipse_PerformCurExt_Parameters()
    {
        var aAnalyzer = new GeomLProp_CurAndInf2d();
        aAnalyzer.PerformCurExt(myEllipse);

        Assert.True(aAnalyzer.IsDone());
        Assert.Equal(4, aAnalyzer.NbPoints());

        // Parameters should be sorted in increasing order
        for (int i = 1; i < aAnalyzer.NbPoints(); ++i)
        {
            Assert.True(aAnalyzer.Parameter(i) < aAnalyzer.Parameter(i + 1));
        }
    }

    [Fact]
    public void Ellipse_PerformInf_NoInflections()
    {
        // An ellipse is a convex curve, so it has no inflection points
        var aAnalyzer = new GeomLProp_CurAndInf2d();
        aAnalyzer.PerformInf(myEllipse);

        Assert.True(aAnalyzer.IsDone());
        Assert.Equal(0, aAnalyzer.NbPoints());
    }

    [Fact]
    public void Ellipse_Perform_CombinedResult()
    {
        // Perform computes both inflections and curvature extrema
        var aAnalyzer = new GeomLProp_CurAndInf2d();
        aAnalyzer.Perform(myEllipse);

        Assert.True(aAnalyzer.IsDone());
        // Ellipse: 4 extrema, 0 inflections
        Assert.Equal(4, aAnalyzer.NbPoints());
    }

    [Fact]
    public void Ellipse_CurvatureExtremaAtExpectedParameters()
    {
        // For an ellipse with major axis along X:
        // Curvature extrema at U = 0, PI/2, PI, 3*PI/2
        var aAnalyzer = new GeomLProp_CurAndInf2d();
        aAnalyzer.PerformCurExt(myEllipse);

        Assert.True(aAnalyzer.IsDone());
        Assert.Equal(4, aAnalyzer.NbPoints());

        double[] anExpectedParams = [0.0, Math.PI / 2.0, Math.PI, 3.0 * Math.PI / 2.0];
        for (int i = 1; i <= 4; ++i)
        {
            Assert.Equal(anExpectedParams[i - 1], aAnalyzer.Parameter(i), 1e-6);
        }
    }

    [Fact]
    public void Hyperbola_PerformCurExt_VertexOnly()
    {
        var aAnalyzer = new GeomLProp_CurAndInf2d();
        aAnalyzer.PerformCurExt(myHyperbola);

        Assert.True(aAnalyzer.IsDone());
        Assert.Equal(1, aAnalyzer.NbPoints());
        Assert.Equal(0.0, aAnalyzer.Parameter(1), global::Kerf.Engine.Foundation.Precision.PConfusion());
        Assert.Equal(LProp_MinCur, aAnalyzer.Type(1));
    }

    [Fact]
    public void Hyperbola_PerformInf_NoInflections()
    {
        var aAnalyzer = new GeomLProp_CurAndInf2d();
        aAnalyzer.PerformInf(myHyperbola);

        Assert.True(aAnalyzer.IsDone());
        Assert.Equal(0, aAnalyzer.NbPoints());
    }

    [Fact]
    public void Parabola_PerformCurExt_VertexOnly()
    {
        var aAnalyzer = new GeomLProp_CurAndInf2d();
        aAnalyzer.PerformCurExt(myParabola);

        Assert.True(aAnalyzer.IsDone());
        Assert.Equal(1, aAnalyzer.NbPoints());
        Assert.Equal(0.0, aAnalyzer.Parameter(1), global::Kerf.Engine.Foundation.Precision.PConfusion());
        Assert.Equal(LProp_MinCur, aAnalyzer.Type(1));
    }

    [Fact]
    public void PerformInf_ClearsPreviousExtrema()
    {
        var aAnalyzer = new GeomLProp_CurAndInf2d();
        aAnalyzer.PerformCurExt(myEllipse);

        Assert.True(aAnalyzer.IsDone());
        Assert.Equal(4, aAnalyzer.NbPoints());

        aAnalyzer.PerformInf(myEllipse);

        Assert.True(aAnalyzer.IsDone());
        Assert.Equal(0, aAnalyzer.NbPoints());
    }
}
