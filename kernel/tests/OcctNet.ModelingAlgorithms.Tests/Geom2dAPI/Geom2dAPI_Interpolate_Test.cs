// Port of: src/ModelingAlgorithms/TKGeomAlgo/GTests/Geom2dAPI_Interpolate_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>OCCT's Geom2dAPI_Interpolate GTest.</summary>
public class Geom2dAPI_InterpolateTest
{
    // OCC28594: Geom2dAPI_Interpolate with and without tangent scale produces different curves.
    // Tests that interpolation with tangent vectors (with scale and without scale) both produce
    // valid B-Spline curves passing through the given points.
    [Fact]
    public void OCC28594_InterpolateWithAndWithoutTangentScale()
    {
        NCollection_HArray1<gp_Pnt2d> aPoints = new(1, 6);
        NCollection_Array1<gp_Pnt2d> aPointsArray = aPoints.ChangeArray1();
        aPointsArray[1] = new gp_Pnt2d(-30.4, 8);
        aPointsArray[2] = new gp_Pnt2d(-16.689912, 17.498217);
        aPointsArray[3] = new gp_Pnt2d(-23.803064, 24.748543);
        aPointsArray[4] = new gp_Pnt2d(-16.907466, 32.919615);
        aPointsArray[5] = new gp_Pnt2d(-8.543829, 26.549421);
        aPointsArray[6] = new gp_Pnt2d(0, 39.200000);

        NCollection_Array1<gp_Vec2d> aTangents = new(1, 6);
        aTangents[1] = new gp_Vec2d(0.3, 0.4);
        aTangents[2] = new gp_Vec2d(0, 0);
        aTangents[3] = new gp_Vec2d(0, 0);
        aTangents[4] = new gp_Vec2d(0, 0);
        aTangents[5] = new gp_Vec2d(0, 0);
        aTangents[6] = new gp_Vec2d(1, 0);

        NCollection_HArray1<bool> aTangentFlags = new(1, 6);
        NCollection_Array1<bool> aTangentFlagsArray = aTangentFlags.ChangeArray1();
        aTangentFlagsArray[1] = true;
        aTangentFlagsArray[2] = false;
        aTangentFlagsArray[3] = false;
        aTangentFlagsArray[4] = false;
        aTangentFlagsArray[5] = false;
        aTangentFlagsArray[6] = true;

        // Interpolation with tangent scale
        Geom2dAPI_Interpolate anInterpWithScale = new(aPoints, false, Precision.Confusion());
        anInterpWithScale.Load(aTangents, aTangentFlags);
        anInterpWithScale.Perform();
        Assert.True(anInterpWithScale.IsDone());
        Geom2d_BSplineCurve aCurveWithScale = anInterpWithScale.Curve();
        Assert.NotNull(aCurveWithScale);

        // Interpolation without tangent scale
        Geom2dAPI_Interpolate anInterpWithoutScale = new(aPoints, false, Precision.Confusion());
        anInterpWithoutScale.Load(aTangents, aTangentFlags, false);
        anInterpWithoutScale.Perform();
        Assert.True(anInterpWithoutScale.IsDone());
        Geom2d_BSplineCurve aCurveWithoutScale = anInterpWithoutScale.Curve();
        Assert.NotNull(aCurveWithoutScale);

        // Both curves must pass through all given points
        double aTol = Precision.Confusion() * 10;
        for (int anIndex = 1; anIndex <= aPoints.Length(); ++anIndex)
        {
            gp_Pnt2d aPtOnCurveWithScale = aCurveWithScale.EvalD0(aCurveWithScale.Knot(anIndex));
            gp_Pnt2d aPt = aPointsArray[anIndex];
            Assert.Equal(aPt.X(), aPtOnCurveWithScale.X(), aTol);
            Assert.Equal(aPt.Y(), aPtOnCurveWithScale.Y(), aTol);
        }
    }
}
