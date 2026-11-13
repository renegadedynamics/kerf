// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// Upstream 71658f6c8f's Geom_TrimmedCurve and Geom2d_TrimmedCurve tests: a copy, a transformed curve and a reversed
/// one keep the trimmed curve's own parameter range on a periodic basis rather than bringing it back into one period.
/// </summary>
public class Geom_TrimmedCurveChartTests
{
    [Fact]
    public void CopyPreservesAuthoredPeriodicRangesAndOwnership()
    {
        var aCircle = new CircleCurve3D(new Frame3D(new Point3D(1, 2, 3), new Direction3D(0, .3, 1)), 5);
        foreach ((double first, double last) in new[] { (-1.2, -.2), (7.0, 11.0), (-8.0, 8.0), (7.0, 7 + 2 * M_PI) })
        {
            var aCurve = new TrimmedCurve3D(aCircle, first, last, true, false);
            var aCopy = Assert.IsType<TrimmedCurve3D>(aCurve.Copy());
            Assert.Equal(first, aCopy.FirstParameter);
            Assert.Equal(last, aCopy.LastParameter);
            foreach (double aParameter in new[] { first, (first + last) / 2, last })
            {
                Assert.True(aCopy.Value(aParameter).DistanceTo(aCurve.Value(aParameter)) <= 1e-12);
            }
        }
    }

    [Fact]
    public void TransformedPreservesAuthoredPeriodicChart()
    {
        var aCurve = new TrimmedCurve3D(new CircleCurve3D(new Frame3D(), 5), 7, 11, true, false);
        foreach (Transform3D aTransform in new[] { new Transform3D(), Transform3D.Scale(new Point3D(1, 2, 3), 2.5),
                                                   Transform3D.Mirror(new Frame3D(new Point3D(), new Direction3D(1, 0, 0))) })
        {
            var aCopy = Assert.IsAssignableFrom<Curve3D>(aCurve.Transformed(aTransform));
            Assert.Equal(aCurve.TransformedParameter(7, aTransform), aCopy.FirstParameter);
            Assert.Equal(aCurve.TransformedParameter(11, aTransform), aCopy.LastParameter);
            foreach (double aParameter in new[] { 7.0, 9.0, 11.0 })
            {
                Point3D aPoint = aCurve.Value(aParameter).Transformed(aTransform);
                Assert.True(aCopy.Value(aCurve.TransformedParameter(aParameter, aTransform)).DistanceTo(aPoint) <= 1e-12);
            }
        }
        Assert.Equal(7, aCurve.FirstParameter);
        Assert.Equal(11, aCurve.LastParameter);
    }

    [Fact]
    public void ReversedPreservesAuthoredPeriodicChart()
    {
        var aCurve = new TrimmedCurve3D(new CircleCurve3D(new Frame3D(), 5), 7, 11, true, false);
        Curve3D aReversed = aCurve.Reversed();
        Assert.Equal(aCurve.ReversedParameter(11), aReversed.FirstParameter);
        Assert.Equal(aCurve.ReversedParameter(7), aReversed.LastParameter);
        Assert.True(aReversed.Value(aReversed.FirstParameter).DistanceTo(aCurve.EndPoint) <= 1e-12);
        Assert.True(aReversed.Value(aReversed.LastParameter).DistanceTo(aCurve.StartPoint) <= 1e-12);
    }

    [Fact]
    public void CopyPreservesAuthoredPeriodicRange2d()
    {
        var aCircle = new CircleCurve2D(new Circle2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 5.0));
        var aTrim = new TrimmedCurve2D(aCircle, -1.2, -0.2, true, false);
        var aCopy = Assert.IsType<TrimmedCurve2D>(aTrim.Copy());
        Assert.Equal(-1.2, aCopy.FirstParameter);
        Assert.Equal(-0.2, aCopy.LastParameter);
        Assert.True(aCopy.Value(-0.7).DistanceTo(aTrim.Value(-0.7)) <= 1e-12);
    }
}
