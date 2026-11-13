// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// After upstream 71658f6c8f's Geom_TrimmedCurve and Geom2d_TrimmedCurve tests (decision 35102): a copy, a transformed
/// and a reversed trimmed curve on a periodic basis are the same curve, a range over more than one period kept as it
/// is (it came back shorter); a range within a period may be brought into the period, as OCCT 8.0.1 does, the same
/// points by the period.
/// </summary>
public class Geom_TrimmedCurveChartTests
{
    private static void AssertSameCurve(Curve3D theExpected, Curve3D theActual)
    {
        Assert.Equal(theExpected.LastParameter - theExpected.FirstParameter, theActual.LastParameter - theActual.FirstParameter, 1e-12);
        for (int i = 0; i <= 8; i++)
        {
            double aShift = (theExpected.LastParameter - theExpected.FirstParameter) * i / 8;
            Assert.True(theActual.Value(theActual.FirstParameter + aShift).DistanceTo(theExpected.Value(theExpected.FirstParameter + aShift)) <= 1e-9);
        }
    }

    [Fact]
    public void CopyKeepsTheCurveOfAPeriodicRange()
    {
        var aCircle = new CircleCurve3D(new Frame3D(new Point3D(1, 2, 3), new Direction3D(0, .3, 1)), 5);
        foreach ((double first, double last) in new[] { (-1.2, -.2), (7.0, 11.0), (-8.0, 8.0), (7.0, 7 + 2 * M_PI) })
        {
            var aCurve = new TrimmedCurve3D(aCircle, first, last, true, false);
            var aCopy = Assert.IsType<TrimmedCurve3D>(aCurve.Copy());
            AssertSameCurve(aCurve, aCopy);
        }

        // Over more than one period, the range itself is kept (OCCT 8.0.1's copy was (-8, -4.57))
        var aWound = Assert.IsType<TrimmedCurve3D>(new TrimmedCurve3D(aCircle, -8, 8, true, false).Copy());
        Assert.Equal(-8.0, aWound.FirstParameter);
        Assert.Equal(8.0, aWound.LastParameter);
    }

    [Fact]
    public void TransformedAndReversedKeepTheCurveOfAPeriodicRange()
    {
        foreach ((double first, double last) in new[] { (7.0, 11.0), (-8.0, 8.0) })
        {
            var aCurve = new TrimmedCurve3D(new CircleCurve3D(new Frame3D(), 5), first, last, true, false);
            foreach (Transform3D aTransform in new[] { new Transform3D(), Transform3D.Scale(new Point3D(1, 2, 3), 2.5),
                                                       Transform3D.Mirror(new Frame3D(new Point3D(), new Direction3D(1, 0, 0))) })
            {
                var aCopy = Assert.IsAssignableFrom<BoundedCurve3D>(aCurve.Transformed(aTransform));
                Assert.Equal(last - first, aCopy.LastParameter - aCopy.FirstParameter, 1e-12);
                Assert.True(aCopy.StartPoint.DistanceTo(aCurve.StartPoint.Transformed(aTransform)) <= 1e-9);
                Assert.True(aCopy.EndPoint.DistanceTo(aCurve.EndPoint.Transformed(aTransform)) <= 1e-9);
            }
            Curve3D aReversed = aCurve.Reversed();
            Assert.Equal(last - first, aReversed.LastParameter - aReversed.FirstParameter, 1e-12);
            Assert.True(aReversed.Value(aReversed.FirstParameter).DistanceTo(aCurve.EndPoint) <= 1e-9);
            Assert.True(aReversed.Value(aReversed.LastParameter).DistanceTo(aCurve.StartPoint) <= 1e-9);
            Assert.Equal(first, aCurve.FirstParameter);
            Assert.Equal(last, aCurve.LastParameter);
        }
    }

    [Fact]
    public void CopyKeepsARangeOverAPeriod2d()
    {
        var aCircle = new CircleCurve2D(new Circle2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 5.0));
        var aTrim = new TrimmedCurve2D(aCircle, -8, 8, true, false);
        var aCopy = Assert.IsType<TrimmedCurve2D>(aTrim.Copy());
        Assert.Equal(-8.0, aCopy.FirstParameter);
        Assert.Equal(8.0, aCopy.LastParameter);
        Assert.True(aCopy.Value(-0.7).DistanceTo(aTrim.Value(-0.7)) <= 1e-12);
    }
}
