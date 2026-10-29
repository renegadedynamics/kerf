// Port of: src/ModelingData/TKGeomBase/GTests/GC_MakeParabola2d_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

public class GC_MakeParabola2d_Test
{
    // Helper: verify a parabola built from directrix+focus against expected values.
    private static void CheckParabola2d(in gp_Ax2d theAxes,
                                        in gp_Pnt2d theFocus,
                                        bool theSense,
                                        double theExpectedFocal,
                                        double theExpectedVertX,
                                        double theExpectedVertY,
                                        double theExpectedParam,
                                        double[] theExpectedCoeffs)
    {
        const double aCompareTol = 1.0e-12;
        GC_MakeParabola2d aPrb = new(theAxes, theFocus, theSense);
        Assert.NotNull(aPrb.Value());
        gp_Parab2d aParab = aPrb.Value().Parab2d();
        gp_Pnt2d aVert = aParab.Location();
        Assert.Equal(theExpectedFocal, aParab.Focal(), aCompareTol);
        Assert.Equal(theExpectedVertX, aVert.X(), aCompareTol);
        Assert.Equal(theExpectedVertY, aVert.Y(), aCompareTol);
        Assert.Equal(theExpectedParam, aParab.Parameter(), aCompareTol);
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
        gp_Ax2d anAxes = new(new gp_Pnt2d(0.0, 3.0), new gp_Dir2d(gp_Dir2d.D.Y));
        gp_Pnt2d aFocus = new(1.0, 3.0);
        double[] aCoeffs = [0.0, 1.0, 0.0, -1.0, -3.0, 10.0];
        CheckParabola2d(anAxes, aFocus, true, 0.5, 0.5, 3.0, 1.0, aCoeffs);
    }

    // Test OCC26747 (case 2): parabola with vertex at (-0.5, 3.0) opening in -X direction.
    [Fact]
    public void OCC26747_2_ParabolaOpeningLeft()
    {
        gp_Ax2d anAxes = new(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(gp_Dir2d.D.Y));
        gp_Pnt2d aFocus = new(-1.0, 3.0);
        double[] aCoeffs = [0.0, 1.0, 0.0, 1.0, -3.0, 10.0];
        CheckParabola2d(anAxes, aFocus, false, 0.5, -0.5, 3.0, 1.0, aCoeffs);
    }

    // Test OCC26747 (case 3): degenerate parabola where focus coincides with vertex.
    [Fact]
    public void OCC26747_3_DegenerateParabola()
    {
        gp_Ax2d anAxes = new(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(gp_Dir2d.D.Y));
        gp_Pnt2d aFocus = new(0.0, 3.0);
        double[] aCoeffs = [0.0, 1.0, 0.0, 0.0, -3.0, 9.0];
        CheckParabola2d(anAxes, aFocus, false, 0.0, 0.0, 3.0, 0.0, aCoeffs);
    }
}
