// Port of: src/ModelingData/TKGeomBase/GTests/Hermit_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class HermitTest
{
    // Helper to create a simple rational BSpline curve (degree 2, 3 poles)
    // with weights that produce distinct endpoint weight values.
    private static BSplineCurve3D MakeRationalBSpline3D(double theW1, double theW2, double theW3)
    {
        NCollection_Array1<Point3D> aPoles = new(1, 3);
        aPoles[1] = new Point3D(0.0, 0.0, 0.0);
        aPoles[2] = new Point3D(1.0, 1.0, 0.0);
        aPoles[3] = new Point3D(2.0, 0.0, 0.0);

        NCollection_Array1<double> aWeights = new(1, 3);
        aWeights[1] = theW1;
        aWeights[2] = theW2;
        aWeights[3] = theW3;

        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;

        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 3;
        aMults[2] = 3;

        return new BSplineCurve3D(aPoles, aWeights, aKnots, aMults, 2);
    }

    // Helper to create a simple rational BSpline 2D curve (degree 2, 3 poles).
    private static BSplineCurve2D MakeRationalBSpline2D(double theW1, double theW2, double theW3)
    {
        NCollection_Array1<Point2D> aPoles = new(1, 3);
        aPoles[1] = new Point2D(0.0, 0.0);
        aPoles[2] = new Point2D(1.0, 1.0);
        aPoles[3] = new Point2D(2.0, 0.0);

        NCollection_Array1<double> aWeights = new(1, 3);
        aWeights[1] = theW1;
        aWeights[2] = theW2;
        aWeights[3] = theW3;

        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;

        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 3;
        aMults[2] = 3;

        return new BSplineCurve2D(aPoles, aWeights, aKnots, aMults, 2);
    }

    [Fact]
    public void Solution3D_UniformWeights_ReturnsValidCurve()
    {
        // Uniform weights => weight function is constant 1.0 everywhere
        BSplineCurve3D aBS = MakeRationalBSpline3D(1.0, 1.0, 1.0);

        BSplineCurve2D aResult = global::Kerf.Engine.Approximation.Hermit.Solution(aBS);

        Assert.NotNull(aResult);
        Assert.True(aResult.PoleCount >= 4);

        // The result a(u) * D(u) should have value 1 at endpoints
        // For uniform weights D(u)=1 everywhere, so a(0)=1, a(1)=1
        Point2D aP0 = aResult.Value(aResult.FirstParameter);
        Point2D aP1 = aResult.Value(aResult.LastParameter);

        Assert.Equal(1.0, aP0.Y, 1.0e-6);
        Assert.Equal(1.0, aP1.Y, 1.0e-6);
    }

    [Fact]
    public void Solution3D_DistinctWeights_ReturnsValidCurve()
    {
        // Different weights at endpoints: w(0)=2, w(1)=3
        BSplineCurve3D aBS = MakeRationalBSpline3D(2.0, 1.5, 3.0);

        BSplineCurve2D aResult = global::Kerf.Engine.Approximation.Hermit.Solution(aBS);

        Assert.NotNull(aResult);

        // a(0) should be 1/w(0) = 0.5, a(1) should be 1/w(1) = 1/3
        Point2D aP0 = aResult.Value(aResult.FirstParameter);
        Point2D aP1 = aResult.Value(aResult.LastParameter);

        Assert.Equal(0.5, aP0.Y, 1.0e-4);
        Assert.Equal(1.0 / 3.0, aP1.Y, 1.0e-4);
    }

    [Fact]
    public void Solution3D_HighWeightRatio_Endpoint()
    {
        // Large weight ratio: w(0) small, w(1) large - tests the Pole0 < Pole3 branch
        BSplineCurve3D aBS = MakeRationalBSpline3D(0.5, 1.0, 5.0);

        BSplineCurve2D aResult = global::Kerf.Engine.Approximation.Hermit.Solution(aBS);

        Assert.NotNull(aResult);

        Point2D aP0 = aResult.Value(aResult.FirstParameter);
        Point2D aP1 = aResult.Value(aResult.LastParameter);

        // a(u)*D(u) = 1 at endpoints => a(0) = 1/w(0) = 2, a(1) = 1/w(1) = 0.2
        Assert.Equal(1.0 / 0.5, aP0.Y, 1.0e-4);
        Assert.Equal(1.0 / 5.0, aP1.Y, 1.0e-4);
    }

    [Fact]
    public void Solution3D_ReversedWeightRatio_Endpoint()
    {
        // Reversed ratio: w(0) large, w(1) small - tests the Pole0 > Pole3 branch
        BSplineCurve3D aBS = MakeRationalBSpline3D(5.0, 1.0, 0.5);

        BSplineCurve2D aResult = global::Kerf.Engine.Approximation.Hermit.Solution(aBS);

        Assert.NotNull(aResult);

        Point2D aP0 = aResult.Value(aResult.FirstParameter);
        Point2D aP1 = aResult.Value(aResult.LastParameter);

        Assert.Equal(1.0 / 5.0, aP0.Y, 1.0e-4);
        Assert.Equal(1.0 / 0.5, aP1.Y, 1.0e-4);
    }

    [Fact]
    public void Solution3D_PositivePoles()
    {
        // The result curve should have all positive Y coordinates (positive denominator)
        BSplineCurve3D aBS = MakeRationalBSpline3D(2.0, 3.0, 1.5);

        BSplineCurve2D aResult = global::Kerf.Engine.Approximation.Hermit.Solution(aBS);

        Assert.NotNull(aResult);
        for (int i = 1; i <= aResult.PoleCount; ++i)
        {
            Assert.True(aResult.Pole(i).Y > 0.0, $"Pole {i} Y should be positive");
        }
    }

    [Fact]
    public void Solution2D_UniformWeights_ReturnsValidCurve()
    {
        BSplineCurve2D aBS = MakeRationalBSpline2D(1.0, 1.0, 1.0);

        BSplineCurve2D aResult = global::Kerf.Engine.Approximation.Hermit.Solution(aBS);

        Assert.NotNull(aResult);

        Point2D aP0 = aResult.Value(aResult.FirstParameter);
        Point2D aP1 = aResult.Value(aResult.LastParameter);

        Assert.Equal(1.0, aP0.Y, 1.0e-6);
        Assert.Equal(1.0, aP1.Y, 1.0e-6);
    }

    [Fact]
    public void Solution2D_DistinctWeights_ReturnsValidCurve()
    {
        BSplineCurve2D aBS = MakeRationalBSpline2D(2.0, 1.5, 3.0);

        BSplineCurve2D aResult = global::Kerf.Engine.Approximation.Hermit.Solution(aBS);

        Assert.NotNull(aResult);

        Point2D aP0 = aResult.Value(aResult.FirstParameter);
        Point2D aP1 = aResult.Value(aResult.LastParameter);

        Assert.Equal(0.5, aP0.Y, 1.0e-4);
        Assert.Equal(1.0 / 3.0, aP1.Y, 1.0e-4);
    }

    [Fact]
    public void Solution2D_HighWeightRatio_Endpoint()
    {
        // Large ratio tests the Pole0 < Pole3 branch in 2D PolyTest
        BSplineCurve2D aBS = MakeRationalBSpline2D(0.5, 1.0, 5.0);

        BSplineCurve2D aResult = global::Kerf.Engine.Approximation.Hermit.Solution(aBS);

        Assert.NotNull(aResult);

        Point2D aP0 = aResult.Value(aResult.FirstParameter);
        Point2D aP1 = aResult.Value(aResult.LastParameter);

        Assert.Equal(1.0 / 0.5, aP0.Y, 1.0e-4);
        Assert.Equal(1.0 / 5.0, aP1.Y, 1.0e-4);
    }

    [Fact]
    public void Solutionbis_UniformWeights_KnotsUnchanged()
    {
        BSplineCurve3D aBS = MakeRationalBSpline3D(1.0, 1.0, 1.0);

        double aKnotmin = 0.0;
        double aKnotmax = 1.0;

        global::Kerf.Engine.Approximation.Hermit.Solutionbis(aBS, ref aKnotmin, ref aKnotmax);

        // For uniform weights, the Hermite coefficients are trivial (1,0,0,1)
        // and no tolerance knots are needed, so the output knots should be
        // the second and second-to-last of the resulting BS2
        Assert.True(aKnotmin >= 0.0);
        Assert.True(aKnotmax <= 1.0);
    }

    [Fact]
    public void Solutionbis_DistinctWeights_ReturnsValidKnots()
    {
        BSplineCurve3D aBS = MakeRationalBSpline3D(2.0, 1.5, 3.0);

        double aKnotmin = 0.0;
        double aKnotmax = 1.0;

        global::Kerf.Engine.Approximation.Hermit.Solutionbis(aBS, ref aKnotmin, ref aKnotmax);

        Assert.True(aKnotmin >= 0.0);
        Assert.True(aKnotmax <= 1.0);
        Assert.True(aKnotmin <= aKnotmax);
    }

    [Fact]
    public void Solution3D_Symmetric_WeightsProduceSymmetricResult()
    {
        // Symmetric weights: w(0) == w(1), so a(0) == a(1)
        BSplineCurve3D aBS = MakeRationalBSpline3D(2.0, 1.0, 2.0);

        BSplineCurve2D aResult = global::Kerf.Engine.Approximation.Hermit.Solution(aBS);

        Assert.NotNull(aResult);

        Point2D aP0 = aResult.Value(aResult.FirstParameter);
        Point2D aP1 = aResult.Value(aResult.LastParameter);

        // Both endpoints should yield the same value: 1/w = 0.5
        Assert.Equal(0.5, aP0.Y, 1.0e-4);
        Assert.Equal(0.5, aP1.Y, 1.0e-4);
    }

    // Added: Hermit only reads the curve it is given, so a frozen curve works.
    [Fact]
    public void Solution_FrozenInput_NotModified()
    {
        BSplineCurve3D aBS = MakeRationalBSpline3D(2.0, 1.5, 3.0);
        aBS.Freeze();
        BSplineCurve2D aResult = global::Kerf.Engine.Approximation.Hermit.Solution(aBS);
        Assert.False(aResult.IsFrozen);
        Assert.Equal(2, aBS.KnotCount);
    }
}
