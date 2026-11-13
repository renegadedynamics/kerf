// Port of: src/ModelingData/TKGeomBase/GTests/GeomLProp_CurAndInf2d_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Properties;
using Kerf.Geometry;
using static Kerf.Engine.Geometry.Properties.CIType;

namespace Kerf.Kernel.ModelingData.Tests;

public class GeomLProp_CurAndInf2dTest
{
    private readonly CircleCurve2D myCircle;
    private readonly EllipseCurve2D myEllipse;
    private readonly HyperbolaCurve2D myHyperbola;
    private readonly ParabolaCurve2D myParabola;

    public GeomLProp_CurAndInf2dTest()
    {
        // Circle centered at origin, radius 5
        var aCirc = new Circle2D(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 5.0);
        myCircle = new CircleCurve2D(aCirc);

        // Ellipse centered at origin, major radius 10, minor radius 3
        var anElips = new Ellipse2D(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 10.0, 3.0);
        myEllipse = new EllipseCurve2D(anElips);

        var aHypr = new Hyperbola2D(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 6.0, 3.0);
        myHyperbola = new HyperbolaCurve2D(aHypr);

        var aParab = new Parabola2D(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 2.0);
        myParabola = new ParabolaCurve2D(aParab);
    }

    [Fact]
    public void Circle_Perform_NoInflections()
    {
        // A circle has constant curvature: no inflections or extrema
        var aAnalyzer = new CurAndInf2d();
        aAnalyzer.Perform(myCircle);

        Assert.True(aAnalyzer.IsDone);
        Assert.Equal(0, aAnalyzer.PointCount);
    }

    [Fact]
    public void Circle_PerformInf_NoInflections()
    {
        var aAnalyzer = new CurAndInf2d();
        aAnalyzer.PerformInf(myCircle);

        Assert.True(aAnalyzer.IsDone);
        Assert.Equal(0, aAnalyzer.PointCount);
    }

    [Fact]
    public void Circle_PerformCurExt_NoExtrema()
    {
        var aAnalyzer = new CurAndInf2d();
        aAnalyzer.PerformCurExt(myCircle);

        Assert.True(aAnalyzer.IsDone);
        Assert.Equal(0, aAnalyzer.PointCount);
    }

    [Fact]
    public void Ellipse_PerformCurExt_HasExtrema()
    {
        // An ellipse has 4 curvature extrema:
        // 2 maxima (at ends of minor axis) and 2 minima (at ends of major axis)
        var aAnalyzer = new CurAndInf2d();
        aAnalyzer.PerformCurExt(myEllipse);

        Assert.True(aAnalyzer.IsDone);
        Assert.Equal(4, aAnalyzer.PointCount);
    }

    [Fact]
    public void Ellipse_PerformCurExt_Types()
    {
        var aAnalyzer = new CurAndInf2d();
        aAnalyzer.PerformCurExt(myEllipse);

        Assert.True(aAnalyzer.IsDone);
        Assert.Equal(4, aAnalyzer.PointCount);

        // Count min and max curvature points
        int aNbMin = 0;
        int aNbMax = 0;
        for (int i = 1; i <= aAnalyzer.PointCount; ++i)
        {
            CIType aType = aAnalyzer.Type(i);
            if (aType == MinCur)
            {
                ++aNbMin;
            }
            else if (aType == MaxCur)
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
        var aAnalyzer = new CurAndInf2d();
        aAnalyzer.PerformCurExt(myEllipse);

        Assert.True(aAnalyzer.IsDone);
        Assert.Equal(4, aAnalyzer.PointCount);

        // Parameters should be sorted in increasing order
        for (int i = 1; i < aAnalyzer.PointCount; ++i)
        {
            Assert.True(aAnalyzer.Parameter(i) < aAnalyzer.Parameter(i + 1));
        }
    }

    [Fact]
    public void Ellipse_PerformInf_NoInflections()
    {
        // An ellipse is a convex curve, so it has no inflection points
        var aAnalyzer = new CurAndInf2d();
        aAnalyzer.PerformInf(myEllipse);

        Assert.True(aAnalyzer.IsDone);
        Assert.Equal(0, aAnalyzer.PointCount);
    }

    [Fact]
    public void Ellipse_Perform_CombinedResult()
    {
        // Perform computes both inflections and curvature extrema
        var aAnalyzer = new CurAndInf2d();
        aAnalyzer.Perform(myEllipse);

        Assert.True(aAnalyzer.IsDone);
        // Ellipse: 4 extrema, 0 inflections
        Assert.Equal(4, aAnalyzer.PointCount);
    }

    [Fact]
    public void Ellipse_CurvatureExtremaAtExpectedParameters()
    {
        // For an ellipse with major axis along X:
        // Curvature extrema at U = 0, PI/2, PI, 3*PI/2
        var aAnalyzer = new CurAndInf2d();
        aAnalyzer.PerformCurExt(myEllipse);

        Assert.True(aAnalyzer.IsDone);
        Assert.Equal(4, aAnalyzer.PointCount);

        double[] anExpectedParams = [0.0, Math.PI / 2.0, Math.PI, 3.0 * Math.PI / 2.0];
        for (int i = 1; i <= 4; ++i)
        {
            Assert.Equal(anExpectedParams[i - 1], aAnalyzer.Parameter(i), 1e-6);
        }
    }

    [Fact]
    public void Hyperbola_PerformCurExt_VertexOnly()
    {
        var aAnalyzer = new CurAndInf2d();
        aAnalyzer.PerformCurExt(myHyperbola);

        Assert.True(aAnalyzer.IsDone);
        Assert.Equal(1, aAnalyzer.PointCount);
        Assert.Equal(0.0, aAnalyzer.Parameter(1), global::Kerf.Engine.Foundation.Precision.PConfusion());
        Assert.Equal(MinCur, aAnalyzer.Type(1));
    }

    [Fact]
    public void Hyperbola_PerformInf_NoInflections()
    {
        var aAnalyzer = new CurAndInf2d();
        aAnalyzer.PerformInf(myHyperbola);

        Assert.True(aAnalyzer.IsDone);
        Assert.Equal(0, aAnalyzer.PointCount);
    }

    [Fact]
    public void Parabola_PerformCurExt_VertexOnly()
    {
        var aAnalyzer = new CurAndInf2d();
        aAnalyzer.PerformCurExt(myParabola);

        Assert.True(aAnalyzer.IsDone);
        Assert.Equal(1, aAnalyzer.PointCount);
        Assert.Equal(0.0, aAnalyzer.Parameter(1), global::Kerf.Engine.Foundation.Precision.PConfusion());
        Assert.Equal(MinCur, aAnalyzer.Type(1));
    }

    [Fact]
    public void PerformInf_ClearsPreviousExtrema()
    {
        var aAnalyzer = new CurAndInf2d();
        aAnalyzer.PerformCurExt(myEllipse);

        Assert.True(aAnalyzer.IsDone);
        Assert.Equal(4, aAnalyzer.PointCount);

        aAnalyzer.PerformInf(myEllipse);

        Assert.True(aAnalyzer.IsDone);
        Assert.Equal(0, aAnalyzer.PointCount);
    }

    // Upstream 8f817f5da5's InflectionsWithAffineParameters (decision 2423): an S curve's two inflections, whatever its
    // size and its parameter's origin, scale and sense. 8.0.1 set its function to 0 where |V2| < 1e-4, so where the
    // parameter's span is long against the curve every sample is a root and it finds none.
    [Fact]
    public void Inflections_dont_depend_on_the_parameters_scale()
    {
        foreach (double aScale in new[] { 1.0, 1000.0 })
        {
            NCollection_Array1<Point2D> aPoles = new(1, 5);
            aPoles[1] = new Point2D(-aScale, 0.0);
            aPoles[2] = new Point2D(-2.0 * aScale / 3.0, aScale / 3.0);
            aPoles[3] = new Point2D(0.0, -aScale / 3.0);
            aPoles[4] = new Point2D(2.0 * aScale / 3.0, aScale / 3.0);
            aPoles[5] = new Point2D(aScale, 0.0);
            NCollection_Array1<int> aMults = NCollection_Array1<int>.From(new[] { 4, 1, 4 });
            foreach (double anOrigin in new[] { 0.0, 2000000.0 })
            {
                foreach (double aSpan in new[] { 1.0, 200.0 })
                {
                    NCollection_Array1<double> aKnots =
                        NCollection_Array1<double>.From(new[] { anOrigin - aSpan, anOrigin, anOrigin + aSpan });
                    var aCurve = new BSplineCurve2D(aPoles, aKnots, aMults, 3);
                    foreach (bool isReversed in new[] { false, true })
                    {
                        Curve2D aCase2d = isReversed ? aCurve.Reversed() : aCurve;
                        string aCase = $"scale {aScale}, origin {anOrigin}, span {aSpan}, reversed {isReversed}";
                        var anAnalyzer = new CurAndInf2d();
                        anAnalyzer.PerformInf(aCase2d);
                        Assert.True(anAnalyzer.IsDone, aCase);
                        Assert.True(anAnalyzer.PointCount == 2, $"{aCase}: {anAnalyzer.PointCount} inflections");
                        Assert.True(Math.Abs((anAnalyzer.Parameter(1) - anOrigin) / aSpan + 1.0 / 3.0) <= 1e-7, aCase);
                        Assert.True(Math.Abs((anAnalyzer.Parameter(2) - anOrigin) / aSpan - 1.0 / 3.0) <= 1e-7, aCase);
                    }
                }
            }
        }
    }

    // Upstream 8f817f5da5's ConcavityChangesBetweenCircularArcs (decision 2423): quarter circles joined with the same
    // tangent and alternating sense; where the sense changes, at the second and fourth knots, is an inflection, though
    // the curvature is zero on neither side
    [Fact]
    public void A_G1_join_that_changes_concavity_is_an_inflection()
    {
        double[] aCoordinates = [0, 0, 1, 0, 1, 1, 1, 2, 0, 2, -1, 2, -1, 3, -1, 4, 0, 4, 1, 4, 1, 5, 1, 6, 0, 6];
        NCollection_Array1<Point2D> aPoles = new(1, 13);
        NCollection_Array1<double> aWeights = new(1, 13);
        for (int i = 1; i <= 13; i++)
        {
            aPoles[i] = new Point2D(aCoordinates[2 * i - 2], aCoordinates[2 * i - 1]);
            aWeights[i] = i % 2 == 0 ? Math.Sqrt(0.5) : 1.0;
        }

        NCollection_Array1<int> aMults = NCollection_Array1<int>.From(new[] { 3, 2, 2, 2, 2, 2, 3 });
        foreach (double anOrigin in new[] { 0.0, 2000000.0 })
        {
            foreach (double aSpan in new[] { 1.0, 200.0 })
            {
                NCollection_Array1<double> aKnots = new(1, 7);
                for (int i = 1; i <= 7; i++)
                {
                    aKnots[i] = anOrigin + aSpan * (i - 1);
                }

                var aCurve = new BSplineCurve2D(aPoles, aWeights, aKnots, aMults, 2);
                foreach (bool isReversed in new[] { false, true })
                {
                    Curve2D aCase2d = isReversed ? aCurve.Reversed() : aCurve;
                    string aCase = $"origin {anOrigin}, span {aSpan}, reversed {isReversed}";
                    var anAnalyzer = new CurAndInf2d();
                    anAnalyzer.PerformInf(aCase2d);
                    Assert.True(anAnalyzer.IsDone, aCase);
                    Assert.True(anAnalyzer.PointCount == 2, $"{aCase}: {anAnalyzer.PointCount} inflections");
                    Assert.Equal(anOrigin + 2.0 * aSpan, anAnalyzer.Parameter(1));
                    Assert.Equal(anOrigin + 4.0 * aSpan, anAnalyzer.Parameter(2));
                }
            }
        }
    }

    // Upstream 8f817f5da5's SmoothInflectionsSurroundConcavityJump (decision 2423): two cubic S curves joined with the
    // same tangent and opposite curvature: an inflection in each, and one at the join
    [Fact]
    public void Inflections_either_side_of_a_concavity_jump_and_at_it()
    {
        double[] aCoordinates = [0, 0, 1, 1, 2, -1, 3, 0, 4, 1, 5, -1, 6, 0];
        NCollection_Array1<Point2D> aPoles = new(1, 7);
        for (int i = 1; i <= 7; i++)
        {
            aPoles[i] = new Point2D(aCoordinates[2 * i - 2], aCoordinates[2 * i - 1]);
        }

        var aCurve = new BSplineCurve2D(aPoles, NCollection_Array1<double>.From(new[] { 0.0, 0.5, 1.0 }),
                                        NCollection_Array1<int>.From(new[] { 4, 3, 4 }), 3);
        var anAnalyzer = new CurAndInf2d();
        anAnalyzer.PerformInf(aCurve);
        Assert.True(anAnalyzer.IsDone);
        Assert.Equal(3, anAnalyzer.PointCount);
        Assert.Equal(0.25, anAnalyzer.Parameter(1), 1e-9);
        Assert.Equal(0.5, anAnalyzer.Parameter(2));
        Assert.Equal(0.75, anAnalyzer.Parameter(3), 1e-9);
        for (int i = 1; i <= anAnalyzer.PointCount; i++)
        {
            Assert.Equal(Inflection, anAnalyzer.Type(i));
        }
    }
}
