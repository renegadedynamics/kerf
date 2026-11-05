// Port of: src/ModelingAlgorithms/TKGeomAlgo/GTests/Geom2dAPI_Interpolate_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Api;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>OCCT's Geom2dAPI_Interpolate GTest.</summary>
public class Geom2dAPI_InterpolateTest
{
    // OCC28594: Geom2dAPI_Interpolate with and without tangent scale produces different curves.
    // Tests that interpolation with tangent vectors (with scale and without scale) both produce
    // valid B-Spline curves passing through the given points.
    [Fact]
    public void OCC28594_InterpolateWithAndWithoutTangentScale()
    {
        NCollection_HArray1<Point2D> aPoints = new(1, 6);
        NCollection_Array1<Point2D> aPointsArray = aPoints.ChangeArray1();
        aPointsArray[1] = new Point2D(-30.4, 8);
        aPointsArray[2] = new Point2D(-16.689912, 17.498217);
        aPointsArray[3] = new Point2D(-23.803064, 24.748543);
        aPointsArray[4] = new Point2D(-16.907466, 32.919615);
        aPointsArray[5] = new Point2D(-8.543829, 26.549421);
        aPointsArray[6] = new Point2D(0, 39.200000);

        NCollection_Array1<Vector2D> aTangents = new(1, 6);
        aTangents[1] = new Vector2D(0.3, 0.4);
        aTangents[2] = new Vector2D(0, 0);
        aTangents[3] = new Vector2D(0, 0);
        aTangents[4] = new Vector2D(0, 0);
        aTangents[5] = new Vector2D(0, 0);
        aTangents[6] = new Vector2D(1, 0);

        NCollection_HArray1<bool> aTangentFlags = new(1, 6);
        NCollection_Array1<bool> aTangentFlagsArray = aTangentFlags.ChangeArray1();
        aTangentFlagsArray[1] = true;
        aTangentFlagsArray[2] = false;
        aTangentFlagsArray[3] = false;
        aTangentFlagsArray[4] = false;
        aTangentFlagsArray[5] = false;
        aTangentFlagsArray[6] = true;

        // Interpolation with tangent scale
        Interpolate2D anInterpWithScale = new(aPoints, false, global::Kerf.Engine.Foundation.Precision.Confusion());
        anInterpWithScale.Load(aTangents, aTangentFlags);
        anInterpWithScale.Perform();
        Assert.True(anInterpWithScale.IsDone);
        BSplineCurve2D aCurveWithScale = anInterpWithScale.Curve();
        Assert.NotNull(aCurveWithScale);

        // Interpolation without tangent scale
        Interpolate2D anInterpWithoutScale = new(aPoints, false, global::Kerf.Engine.Foundation.Precision.Confusion());
        anInterpWithoutScale.Load(aTangents, aTangentFlags, false);
        anInterpWithoutScale.Perform();
        Assert.True(anInterpWithoutScale.IsDone);
        BSplineCurve2D aCurveWithoutScale = anInterpWithoutScale.Curve();
        Assert.NotNull(aCurveWithoutScale);

        // Both curves must pass through all given points
        double aTol = global::Kerf.Engine.Foundation.Precision.Confusion() * 10;
        for (int anIndex = 1; anIndex <= aPoints.Length(); ++anIndex)
        {
            Point2D aPtOnCurveWithScale = aCurveWithScale.EvalD0(aCurveWithScale.Knot(anIndex));
            Point2D aPt = aPointsArray[anIndex];
            Assert.Equal(aPt.X, aPtOnCurveWithScale.X, aTol);
            Assert.Equal(aPt.Y, aPtOnCurveWithScale.Y, aTol);
        }
    }
}
