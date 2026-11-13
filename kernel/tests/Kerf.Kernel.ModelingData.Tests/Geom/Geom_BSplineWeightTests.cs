// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// A weight that isn't a number is refused, as one at most gp::Resolution() is (decision 35103, after upstream
/// 573b4be651's ValidatesRationalWeights): it compared false with the limit, and the curve was made, its weights taken
/// for equal and dropped, a polynomial curve for a rational one.
/// </summary>
public class Geom_BSplineWeightTests
{
    [Fact]
    public void A_weight_that_is_not_a_number_is_refused()
    {
        Point3D[] aPoles = [new(0, 0, 0), new(1, 1, 0), new(2, 0, 0)];
        double[] aKnots = [0.0, 1.0];
        int[] aMults = [3, 3];
        Assert.True(new BSplineCurve3D(aPoles, aKnots, aMults, 2, false, [1.0, 2.0, 1.0]).IsRational);
        KernelAssert.Throws<ArgumentException>(() => new BSplineCurve3D(aPoles, aKnots, aMults, 2, false, [1.0, double.NaN, 1.0]));
        KernelAssert.Throws<ArgumentException>(() => new BSplineCurve3D(aPoles, aKnots, aMults, 2, false, [1.0, 0.0, 1.0]));

        var aPoles2d = NCollection_Array1<Point2D>.From([new(0, 0), new(1, 1), new(2, 0)]);
        EngineAssert.Throws<ConstructionErrorException>(() => new BSplineCurve2D(aPoles2d, NCollection_Array1<double>.From([1.0, double.NaN, 1.0]),
                                                                                   NCollection_Array1<double>.From(aKnots),
                                                                                   NCollection_Array1<int>.From(aMults), 2));
    }
}
