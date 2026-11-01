// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// Port of OCCT's GTest src/ModelingAlgorithms/TKHelix/GTests/HelixGeom_Tools_Test.cxx (HelixGeom_Tools_Test).

using static Occt.GeomAbs_Shape;

namespace OcctNet.ModelingAlgorithms.Tests;

public class HelixGeom_ToolsTests
{
    private const double myTolerance = 1.0e-6;

    [Fact]
    public void ApprHelix()
    {
        Geom_BSplineCurve? aBSpline = null;
        double aMaxError = 0;

        int aResult = HelixGeom_Tools.ApprHelix(0.0,         // T1
                                                2.0 * M_PI,  // T2
                                                10.0,        // Pitch
                                                5.0,         // Start radius
                                                0.0,         // Taper angle
                                                true,        // Clockwise
                                                myTolerance, // Tolerance
                                                ref aBSpline, // Result
                                                ref aMaxError // Max error
        );

        Assert.Equal(0, aResult); // Success
        Assert.NotNull(aBSpline);
        Assert.True(aMaxError <= myTolerance);

        // Test curve properties
        Assert.True(aBSpline.Degree() > 0);
        Assert.True(aBSpline.NbPoles() > 0);
    }

    [Fact]
    public void ApprCurve3D()
    {
        // Create a helix curve adaptor
        var aHelix = new HelixGeom_HelixCurve();
        aHelix.Load(0.0, 2.0 * M_PI, 10.0, 3.0, 0.0, true);
        var aHAdaptor = new HelixGeom_HelixCurve(aHelix);

        Geom_BSplineCurve? aBSpline = null;
        double aMaxError = 0;

        int aResult = HelixGeom_Tools.ApprCurve3D(aHAdaptor,
                                                  myTolerance,
                                                  GeomAbs_C1,
                                                  50, // Max segments
                                                  6,  // Max degree
                                                  ref aBSpline,
                                                  ref aMaxError);

        Assert.Equal(0, aResult);
        Assert.NotNull(aBSpline);
        Assert.True(aMaxError <= myTolerance * 10);

        // Verify approximation quality by sampling points
        int aNbSamples = 10;
        for (int i = 0; i <= aNbSamples; i++)
        {
            double aParam = aHelix.FirstParameter() + i * (aHelix.LastParameter() - aHelix.FirstParameter()) / aNbSamples;

            gp_Pnt aOrigPnt = aHelix.Value(aParam);

            // Map parameter to B-spline parameter space
            double aBSplineParam = aBSpline.FirstParameter()
                                   + i * (aBSpline.LastParameter() - aBSpline.FirstParameter()) / aNbSamples;

            aBSpline.D0(aBSplineParam, out gp_Pnt aApproxPnt);

            double aDistance = aOrigPnt.Distance(aApproxPnt);
            Assert.True(aDistance <= aMaxError * 2); // Allow some margin
        }
    }

    [Fact]
    public void DifferentContinuity()
    {
        var aHelix = new HelixGeom_HelixCurve();
        aHelix.Load(0.0, 6.0 * M_PI, 15.0, 4.0, 0.05, true);
        var aHAdaptor = new HelixGeom_HelixCurve(aHelix);

        // Test C0 continuity
        Geom_BSplineCurve? aBSplineC0 = null;
        double aMaxErrorC0 = 0;
        int aResultC0 = HelixGeom_Tools.ApprCurve3D(aHAdaptor, myTolerance, GeomAbs_C0, 30, 4, ref aBSplineC0, ref aMaxErrorC0);

        Assert.Equal(0, aResultC0);
        Assert.NotNull(aBSplineC0);

        // Test C2 continuity
        Geom_BSplineCurve? aBSplineC2 = null;
        double aMaxErrorC2 = 0;
        int aResultC2 = HelixGeom_Tools.ApprCurve3D(aHAdaptor, myTolerance, GeomAbs_C2, 30, 6, ref aBSplineC2, ref aMaxErrorC2);

        Assert.Equal(0, aResultC2);
        Assert.NotNull(aBSplineC2);

        // C2 curve should generally have higher degree
        Assert.True(aBSplineC2.Degree() >= aBSplineC0.Degree());
    }
}
