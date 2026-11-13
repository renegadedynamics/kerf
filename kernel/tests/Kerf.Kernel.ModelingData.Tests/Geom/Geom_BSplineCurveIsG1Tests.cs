// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// Upstream 71658f6c8f's IsG1 tests of a periodic B-spline (decision 2434): ranges translated by periods, across the
/// seam and over a whole period; and an offset of a periodic curve with a corner at its seam, which OCCT 8.0.1 made.
/// </summary>
public class Geom_BSplineCurveIsG1Tests
{
    private static readonly Point3D[] Square = [new(0, 0, 0), new(4, 0, 0), new(4, 4, 0), new(0, 4, 0)];

    [Fact]
    public void IsG1TranslatedPeriodicIntervalsAndSeam()
    {
        BSplineCurve3D aCurve = new(Square, [0.0, 1.0, 2.0, 3.0, 4.0], [1, 1, 1, 1, 1], 1, true);
        foreach (double aShift in new[] { -8.0, 0.0, 4.0, 12.0 })
        {
            Assert.True(aCurve.IsG1(aShift + .25, aShift + .75, Precision.Angular()), $"{aShift}");
            Assert.False(aCurve.IsG1(aShift + .25, aShift + 1.75, Precision.Angular()), $"{aShift}");
            Assert.False(aCurve.IsG1(aShift + 3.75, aShift + 4.25, Precision.Angular()), $"{aShift}");
            Assert.False(aCurve.IsG1(aShift, aShift, Precision.Angular()), $"{aShift}");
            Assert.False(aCurve.IsG1(aShift + .25, aShift + 8.25, Precision.Angular()), $"{aShift}");
        }
    }

    [Fact]
    public void IsG1_PeriodicTranslatedIntervals_CheckCornersAndTheSeam2d()
    {
        BSplineCurve2D aSource = new(NCollection_Array1<Point2D>.From([new(0, 0), new(4, 0), new(4, 4), new(0, 4)]),
                                     NCollection_Array1<double>.From([0.0, 1.0, 2.0, 3.0, 4.0]),
                                     NCollection_Array1<int>.From([1, 1, 1, 1, 1]), 1, true);
        foreach (double aShift in new[] { -8.0, 0.0, 4.0, 12.0 })
        {
            Assert.True(aSource.IsG1(aShift + 0.25, aShift + 0.75, Precision.Angular()), $"{aShift}");
            Assert.False(aSource.IsG1(aShift + 0.25, aShift + 1.75, Precision.Angular()), $"{aShift}");
            Assert.False(aSource.IsG1(aShift - 0.25, aShift + 0.25, Precision.Angular()), $"{aShift}");
            Assert.False(aSource.IsG1(aShift + 0.25, aShift + 4.25, Precision.Angular()), $"{aShift}");
        }
        Assert.Equal(0.0, aSource.FirstParameter);
        Assert.Equal(4.0, aSource.LastParameter);
    }

    // A periodic quadratic smooth everywhere but at its seam, where the knot's multiplicity is the degree: a corner
    // of 90 degrees. An offset of a range across the seam was made over the corner, a success. Over the whole period
    // the seam is at the range's ends, the corner an edge's vertex, and the offset is made as before.
    [Fact]
    public void An_offset_of_a_periodic_curve_across_a_corner_at_its_seam_is_refused()
    {
        BSplineCurve3D aCurve = new(Square, [0.0, 1.0, 2.0, 3.0], [2, 1, 1, 2], 2, true);
        // Its continuity is its inner knots', as OCCT's: C1, though it has a corner at the seam
        Assert.Equal(Continuity.C1, aCurve.Continuity);
        Assert.True(aCurve.HasC0Seam);
        Assert.True(aCurve.CrossesSeam(2.5, 3.5));
        Assert.False(aCurve.CrossesSeam(0.0, 3.0));
        Assert.False(aCurve.CrossesSeam(2.5, 3.0));
        EngineAssert.Throws<ConstructionErrorException>(() => new OffsetCurve3D(new TrimmedCurve3D(aCurve, 2.5, 3.5), 0.5, new Direction3D(0, 0, 1)));
        Assert.Equal(Continuity.C1, new OffsetCurve3D(aCurve, 0.5, new Direction3D(0, 0, 1)).GetBasisCurveContinuity());
        _ = new OffsetCurve3D(new TrimmedCurve3D(aCurve, 0.5, 2.5), 0.5, new Direction3D(0, 0, 1));

        BSplineCurve2D aCurve2d = new(NCollection_Array1<Point2D>.From([new(0, 0), new(4, 0), new(4, 4), new(0, 4)]),
                                      NCollection_Array1<double>.From([0.0, 1.0, 2.0, 3.0]),
                                      NCollection_Array1<int>.From([2, 1, 1, 2]), 2, true);
        EngineAssert.Throws<ConstructionErrorException>(() => new OffsetCurve2D(new TrimmedCurve2D(aCurve2d, 2.5, 3.5), 0.5));
        _ = new OffsetCurve2D(aCurve2d, 0.5);

        // The same curve with its corner inside (not at the seam) was refused already, and still is
        BSplineCurve3D aMoved = new(Square, [0.0, 1.0, 2.0, 3.0], [1, 2, 1, 1], 2, true);
        EngineAssert.Throws<ConstructionErrorException>(() => new OffsetCurve3D(aMoved, 0.5, new Direction3D(0, 0, 1)));
    }
}
