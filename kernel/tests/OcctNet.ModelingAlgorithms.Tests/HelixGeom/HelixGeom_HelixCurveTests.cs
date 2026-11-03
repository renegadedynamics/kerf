// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// Port of OCCT's GTest src/ModelingAlgorithms/TKHelix/GTests/HelixGeom_HelixCurve_Test.cxx (HelixGeomTest).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace OcctNet.ModelingAlgorithms.Tests;

public class HelixGeom_HelixCurveTests
{
    private const double myTolerance = 1.0e-6;

    [Fact]
    public void HelixCurve_Derivatives()
    {
        var aHelix = new HelixGeom_HelixCurve();
        aHelix.Load(0.0, 2.0 * M_PI, 5.0, 2.0, 0.0, true);

        double aParam = M_PI / 2.0;

        // Test D1
        aHelix.D1(aParam, out Point3D aP, out Vector3D aV1);
        Assert.Equal(0.0, aP.X(), 1e-15);
        Assert.Equal(2.0, aP.Y(), 1e-15);
        Assert.True(aV1.Magnitude() > 0.0);

        // Test D2
        aHelix.D2(aParam, out aP, out aV1, out Vector3D aV2);
        Assert.Equal(0.0, aP.X(), 1e-15);
        Assert.Equal(2.0, aP.Y(), 1e-15);
        Assert.True(aV2.Magnitude() > 0.0);

        // Test DN
        Vector3D aVN1 = aHelix.DN(aParam, 1);
        Vector3D aVN2 = aHelix.DN(aParam, 2);

        Assert.Equal(aV1.X(), aVN1.X(), 1e-15);
        Assert.Equal(aV1.Y(), aVN1.Y(), 1e-15);
        Assert.Equal(aV1.Z(), aVN1.Z(), 1e-15);

        Assert.Equal(aV2.X(), aVN2.X(), 1e-15);
        Assert.Equal(aV2.Y(), aVN2.Y(), 1e-15);
        Assert.Equal(aV2.Z(), aVN2.Z(), 1e-15);
    }

    [Fact]
    public void HelixCurve_ErrorConditions()
    {
        var aHelix = new HelixGeom_HelixCurve();

        // Test invalid parameter range (T1 >= T2)
        Assert.ThrowsAny<ConstructionErrorException>(() => aHelix.Load(2.0, 1.0, 5.0, 2.0, 0.0, true));

        // Test negative pitch
        Assert.ThrowsAny<ConstructionErrorException>(() => aHelix.Load(0.0, 2.0 * M_PI, -1.0, 2.0, 0.0, true));

        // Test negative radius
        Assert.ThrowsAny<ConstructionErrorException>(() => aHelix.Load(0.0, 2.0 * M_PI, 5.0, -1.0, 0.0, true));

        // Test invalid taper angle
        Assert.ThrowsAny<ConstructionErrorException>(() => aHelix.Load(0.0, 2.0 * M_PI, 5.0, 2.0, M_PI / 2.0, true));
    }

    [Fact]
    public void HelixCurve_CounterClockwise()
    {
        var aHelix = new HelixGeom_HelixCurve();
        aHelix.Load(0.0, 2.0 * M_PI, 5.0, 2.0, 0.0, false); // Counter-clockwise

        Point3D aP0 = aHelix.Value(0.0);
        Point3D aP1 = aHelix.Value(M_PI / 2.0);

        Assert.Equal(2.0, aP0.X(), 1e-15);
        Assert.Equal(0.0, aP0.Y(), 1e-15);

        Assert.Equal(0.0, aP1.X(), 1e-15);
        Assert.Equal(-2.0, aP1.Y(), 1e-15); // Negative Y for counter-clockwise
    }

    [Fact]
    public void HelixCurve_AdaptorInterface()
    {
        var aHelix = new HelixGeom_HelixCurve();
        aHelix.Load(0.0, 4.0 * M_PI, 10.0, 3.0, 0.0, true);

        // Test continuity
        Assert.Equal(CN, aHelix.Continuity());

        // Test intervals
        Assert.Equal(1, aHelix.NbIntervals(C0));
        Assert.Equal(1, aHelix.NbIntervals(C1));
        Assert.Equal(1, aHelix.NbIntervals(C2));

        // Test intervals array
        NCollection_Array1<double> anIntervals = new(1, 2);
        aHelix.Intervals(anIntervals, C0);
        Assert.Equal(0.0, anIntervals[1]);
        Assert.Equal(4.0 * M_PI, anIntervals[2]);

        // Test not implemented methods
        Assert.ThrowsAny<MethodNotImplementedException>(() => aHelix.Resolution(1.0));
        Assert.ThrowsAny<MethodNotImplementedException>(() => aHelix.IsClosed());
        Assert.ThrowsAny<MethodNotImplementedException>(() => aHelix.IsPeriodic());
        Assert.ThrowsAny<DomainErrorException>(() => aHelix.Period());
    }

    [Fact]
    public void BuilderHelixCoil_Approximation()
    {
        var aBuilder = new HelixGeom_BuilderHelixCoil();

        aBuilder.SetTolerance(myTolerance);
        aBuilder.SetApproxParameters(C2, 8, 100);
        // Reduced parameter range and taper angle
        aBuilder.SetCurveParameters(0.0, 2.0 * M_PI, 20.0, 5.0, 0.05, true);

        aBuilder.Perform();

        Assert.Equal(0, aBuilder.ErrorStatus());
        Assert.True(aBuilder.ToleranceReached() <= 0.1); // Allow up to 0.1 tolerance for tapered helix approximation

        var aCurves = aBuilder.Curves();
        Assert.Single(aCurves);

        Geom_Curve aCurve = aCurves[0];
        Assert.NotNull(aCurve);

        // Verify that the curve is a B-spline
        Geom_BSplineCurve? aBSpline = aCurve as Geom_BSplineCurve;
        Assert.NotNull(aBSpline);

        // Check B-spline properties
        Assert.True(aBSpline.Degree() <= 8);
        Assert.True(aBSpline.NbPoles() > 0);
    }

    [Fact]
    public void Tools_ApprCurve3D()
    {
        // Create a helix curve adaptor
        var aHelix = new HelixGeom_HelixCurve();
        aHelix.Load(0.0, 2.0 * M_PI, 10.0, 3.0, 0.0, true);
        var aHAdaptor = new HelixGeom_HelixCurve(aHelix);

        Geom_BSplineCurve? aBSpline = null;
        double aMaxError = 0;

        int aResult = HelixGeom_Tools.ApprCurve3D(aHAdaptor,
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
    public void Tools_DifferentContinuity()
    {
        var aHelix = new HelixGeom_HelixCurve();
        aHelix.Load(0.0, 6.0 * M_PI, 15.0, 4.0, 0.05, true);
        var aHAdaptor = new HelixGeom_HelixCurve(aHelix);

        // Test C0 continuity
        Geom_BSplineCurve? aBSplineC0 = null;
        double aMaxErrorC0 = 0;
        int aResultC0 = HelixGeom_Tools.ApprCurve3D(aHAdaptor, myTolerance, C0, 30, 4, ref aBSplineC0, ref aMaxErrorC0);

        Assert.Equal(0, aResultC0);
        Assert.NotNull(aBSplineC0);

        // Test C2 continuity
        Geom_BSplineCurve? aBSplineC2 = null;
        double aMaxErrorC2 = 0;
        int aResultC2 = HelixGeom_Tools.ApprCurve3D(aHAdaptor, myTolerance, C2, 30, 6, ref aBSplineC2, ref aMaxErrorC2);

        Assert.Equal(0, aResultC2);
        Assert.NotNull(aBSplineC2);

        // C2 curve should generally have higher degree
        Assert.True(aBSplineC2.Degree() >= aBSplineC0.Degree());
    }

    [Fact]
    public void BuilderHelixCoil_DefaultParameters()
    {
        var aBuilder = new HelixGeom_BuilderHelixCoil();

        // Use default parameters
        aBuilder.Perform();

        Assert.Equal(0, aBuilder.ErrorStatus());

        // Check default approximation parameters
        aBuilder.ApproxParameters(out Continuity aCont, out int aMaxDegree, out int aMaxSeg);

        Assert.Equal(C2, aCont);
        Assert.Equal(8, aMaxDegree);
        Assert.Equal(150, aMaxSeg);

        Assert.Equal(0.0001, aBuilder.Tolerance());
    }

    [Fact]
    public void BuilderHelixCoil_ParameterSymmetry()
    {
        var aBuilder = new HelixGeom_BuilderHelixCoil();

        // Set parameters
        double aT1 = 0.5, aT2 = 5.5, aPitch = 12.5, aRStart = 3.5, aTaperAngle = 0.15;
        bool aIsClockwise = false;

        aBuilder.SetCurveParameters(aT1, aT2, aPitch, aRStart, aTaperAngle, aIsClockwise);

        // Get parameters back
        aBuilder.CurveParameters(out double aT1_out, out double aT2_out, out double aPitch_out, out double aRStart_out,
                                 out double aTaperAngle_out, out bool aIsClockwise_out);

        Assert.Equal(aT1, aT1_out);
        Assert.Equal(aT2, aT2_out);
        Assert.Equal(aPitch, aPitch_out);
        Assert.Equal(aRStart, aRStart_out);
        Assert.Equal(aTaperAngle, aTaperAngle_out);
        Assert.Equal(aIsClockwise, aIsClockwise_out);
    }

    // ---- the port's own: OCCT's behaviour the GTests don't pin ----

    /// <summary>OCCT keeps the old taper when a helix is reloaded with a (near) zero taper angle (occt-issues.md, unit 7D).</summary>
    [Fact]
    public void Reloading_with_no_taper_keeps_the_old_taper_as_OCCT()
    {
        var aHelix = new HelixGeom_HelixCurve();
        aHelix.Load(0.0, 2.0 * M_PI, 5.0, 2.0, 0.1, true);
        Point3D aTapered = aHelix.Value(2.0 * M_PI);
        aHelix.Load(0.0, 2.0 * M_PI, 5.0, 2.0, 0.0, true);
        Assert.Equal(aTapered, aHelix.Value(2.0 * M_PI));
        var aFresh = new HelixGeom_HelixCurve();
        aFresh.Load(0.0, 2.0 * M_PI, 5.0, 2.0, 0.0, true);
        Assert.Equal(2.0, aFresh.Value(2.0 * M_PI).X(), 1e-12);
    }

    /// <summary>Curves() hands out frozen curves; the builder's next Perform makes new ones.</summary>
    [Fact]
    public void Curves_are_frozen_when_handed_out()
    {
        var aBuilder = new HelixGeom_BuilderHelix();
        aBuilder.SetCurveParameters(0.0, 6.0 * M_PI, 10.0, 5.0, 0.0, true);
        aBuilder.Perform();
        var aFirst = aBuilder.Curves();
        Assert.All(aFirst, c => Assert.True(c.IsFrozen));
        Assert.Same(aFirst[0], aBuilder.Curves()[0]);
        aBuilder.Perform();
        Assert.NotSame(aFirst[0], aBuilder.Curves()[0]);
    }
}
