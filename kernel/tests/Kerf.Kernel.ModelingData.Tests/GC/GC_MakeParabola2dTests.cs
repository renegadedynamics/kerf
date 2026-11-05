// Port of: src/ModelingData/TKGeomBase/GTests/GC_MakeParabola2d_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Curves;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class GC_MakeParabola2d_Test
{
    // Helper: verify a parabola built from directrix+focus against expected values.
    private static void CheckParabola2d(in Axis2D theAxes,
                                        in Point2D theFocus,
                                        bool theSense,
                                        double theExpectedFocal,
                                        double theExpectedVertX,
                                        double theExpectedVertY,
                                        double theExpectedParam,
                                        double[] theExpectedCoeffs)
    {
        const double aCompareTol = 1.0e-12;
        MakeParabola2d aPrb = new(theAxes, theFocus, theSense);
        Assert.NotNull(aPrb.Value());
        Parabola2D aParab = aPrb.Value().Parabola;
        Point2D aVert = aParab.Location;
        Assert.Equal(theExpectedFocal, aParab.Focal, aCompareTol);
        Assert.Equal(theExpectedVertX, aVert.X, aCompareTol);
        Assert.Equal(theExpectedVertY, aVert.Y, aCompareTol);
        Assert.Equal(theExpectedParam, aParab.Parameter, aCompareTol);
        aParab.Coefficients(out double aF0, out double aF1, out double aF2, out double aF3, out double aF4,
                            out double aF5);
        double[] aF = [aF0, aF1, aF2, aF3, aF4, aF5];
        for (int i = 0; i < 6; ++i)
        {
            Assert.Equal(theExpectedCoeffs[i], aF[i], aCompareTol);
        }
    }

    // Test OCC26747 (case 1): parabola with vertex at (0.5, 3.0) opening in +X direction.
    [Fact]
    public void OCC26747_1_ParabolaOpeningRight()
    {
        Axis2D anAxes = new(new Point2D(0.0, 3.0), new Direction2D(Direction2D.D.Y));
        Point2D aFocus = new(1.0, 3.0);
        double[] aCoeffs = [0.0, 1.0, 0.0, -1.0, -3.0, 10.0];
        CheckParabola2d(anAxes, aFocus, true, 0.5, 0.5, 3.0, 1.0, aCoeffs);
    }

    // Test OCC26747 (case 2): parabola with vertex at (-0.5, 3.0) opening in -X direction.
    [Fact]
    public void OCC26747_2_ParabolaOpeningLeft()
    {
        Axis2D anAxes = new(new Point2D(0.0, 0.0), new Direction2D(Direction2D.D.Y));
        Point2D aFocus = new(-1.0, 3.0);
        double[] aCoeffs = [0.0, 1.0, 0.0, 1.0, -3.0, 10.0];
        CheckParabola2d(anAxes, aFocus, false, 0.5, -0.5, 3.0, 1.0, aCoeffs);
    }

    // Test OCC26747 (case 3): degenerate parabola where focus coincides with vertex.
    [Fact]
    public void OCC26747_3_DegenerateParabola()
    {
        Axis2D anAxes = new(new Point2D(0.0, 0.0), new Direction2D(Direction2D.D.Y));
        Point2D aFocus = new(0.0, 3.0);
        double[] aCoeffs = [0.0, 1.0, 0.0, 0.0, -3.0, 9.0];
        CheckParabola2d(anAxes, aFocus, false, 0.0, 0.0, 3.0, 0.0, aCoeffs);
    }
}
