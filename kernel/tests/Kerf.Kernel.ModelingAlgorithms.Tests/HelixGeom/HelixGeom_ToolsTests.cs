// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// Port of OCCT's GTest src/ModelingAlgorithms/TKHelix/GTests/HelixGeom_Tools_Test.cxx (HelixGeom_Tools_Test).

using Kerf.Engine.Construction.Helix.Geometry;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

public class HelixGeom_ToolsTests
{
    private const double myTolerance = 1.0e-6;

    [Fact]
    public void ApprHelix()
    {
        BSplineCurve3D? aBSpline = null;
        double aMaxError = 0;

        int aResult = GeometryTools.ApprHelix(0.0,         // T1
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
        Assert.True(aBSpline.PoleCount > 0);
    }

    [Fact]
    public void ApprCurve3D()
    {
        // Create a helix curve adaptor
        var aHelix = new HelixCurve();
        aHelix.Load(0.0, 2.0 * M_PI, 10.0, 3.0, 0.0, true);
        var aHAdaptor = new HelixCurve(aHelix);

        BSplineCurve3D? aBSpline = null;
        double aMaxError = 0;

        int aResult = GeometryTools.ApprCurve3D(aHAdaptor,
                                                  myTolerance,
                                                  C1,
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

            Point3D aOrigPnt = aHelix.Value(aParam);

            // Map parameter to B-spline parameter space
            double aBSplineParam = aBSpline.FirstParameter()
                                   + i * (aBSpline.LastParameter() - aBSpline.FirstParameter()) / aNbSamples;

            aBSpline.D0(aBSplineParam, out Point3D aApproxPnt);

            double aDistance = aOrigPnt.Distance(aApproxPnt);
            Assert.True(aDistance <= aMaxError * 2); // Allow some margin
        }
    }

    [Fact]
    public void DifferentContinuity()
    {
        var aHelix = new HelixCurve();
        aHelix.Load(0.0, 6.0 * M_PI, 15.0, 4.0, 0.05, true);
        var aHAdaptor = new HelixCurve(aHelix);

        // Test C0 continuity
        BSplineCurve3D? aBSplineC0 = null;
        double aMaxErrorC0 = 0;
        int aResultC0 = GeometryTools.ApprCurve3D(aHAdaptor, myTolerance, C0, 30, 4, ref aBSplineC0, ref aMaxErrorC0);

        Assert.Equal(0, aResultC0);
        Assert.NotNull(aBSplineC0);

        // Test C2 continuity
        BSplineCurve3D? aBSplineC2 = null;
        double aMaxErrorC2 = 0;
        int aResultC2 = GeometryTools.ApprCurve3D(aHAdaptor, myTolerance, C2, 30, 6, ref aBSplineC2, ref aMaxErrorC2);

        Assert.Equal(0, aResultC2);
        Assert.NotNull(aBSplineC2);

        // C2 curve should generally have higher degree
        Assert.True(aBSplineC2.Degree() >= aBSplineC0.Degree());
    }
}
