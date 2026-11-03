// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;
using static Occt.LocalAnalysis_StatusErrorType;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>LocalAnalysis_SurfaceContinuity on surfaces whose continuity is known in closed form, and its Dump.</summary>
public class LocalAnalysis_SurfaceContinuityTests
{
    // spheres of radius 1 about the origin and of radius 2 about (-1, 0, 0): they touch at (1, 0, 0), which is
    // (u, v) = (0, 0) on both, with the same normal
    private static readonly Geom_SphericalSurface s_s1 = new(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 1.0);
    private static readonly Geom_SphericalSurface s_s2 = new(new CoordinateSystem(new Point3D(-1, 0, 0), new Direction3D(0, 0, 1)), 2.0);

    // the plane x = 1, with u along y and v along z
    private static readonly Geom_Plane s_plane = new(new CoordinateSystem(new Point3D(1, 0, 0), new Direction3D(1, 0, 0), new Direction3D(0, 1, 0)));

    [Fact]
    public void C0_AndC1_CompareThePointsAndFirstDerivatives()
    {
        var c = new LocalAnalysis_SurfaceContinuity(s_s1, 0.0, 0.0, s_s2, 0.0, 0.0, C1);
        Assert.True(c.IsDone());
        Assert.Equal(C1, c.ContinuityStatus());
        Assert.Equal(0.0, c.C0Value(), 1e-15);
        Assert.Equal(0.0, c.C1UAngle(), 1e-15);
        Assert.Equal(0.0, c.C1VAngle(), 1e-15);
        Assert.Equal(0.5, c.C1URatio(), 1e-15); // |D1U| = R
        Assert.Equal(0.5, c.C1VRatio(), 1e-15);
        Assert.True(c.IsC1());

        c = new LocalAnalysis_SurfaceContinuity(s_s1, 0.0, 0.0, s_s2, 0.0, 0.1, C0);
        Assert.Equal(new Point3D(1, 0, 0).Distance(s_s2.Value(0.0, 0.1)), c.C0Value());
        Assert.False(c.IsC0());
    }

    [Fact]
    public void C2_OfTheSameSurface_IsC2()
    {
        var c = new LocalAnalysis_SurfaceContinuity(s_s1, 0.4, 0.2, s_s1, 0.4, 0.2, C2);
        Assert.True(c.IsDone());
        Assert.True(c.IsC2());
        Assert.Equal(1.0, c.C2URatio());
        Assert.Equal(1.0, c.C2VRatio());

        c = new LocalAnalysis_SurfaceContinuity(s_s1, 0.0, 0.0, s_s2, 0.0, 0.0, C2);
        Assert.True(c.IsC1());
        Assert.False(c.IsC2()); // ratios 0.5 and 0.5: Lambda1^2 differs from Lambda2
    }

    [Fact]
    public void G1_AndG2_CompareTheNormalsAndCurvatures()
    {
        var c = new LocalAnalysis_SurfaceContinuity(s_s1, 0.0, 0.0, s_s2, 0.0, 0.0, G2);
        Assert.True(c.IsDone());
        Assert.Equal(0.0, c.G1Angle(), 1e-15);
        Assert.True(c.IsG1());
        // umbilics of curvature 1 and 0.5: the gap is half the difference of the mean curvatures
        Assert.Equal(0.25, c.G2CurvatureGap(), 1e-14);
        Assert.False(c.IsG2());

        c = new LocalAnalysis_SurfaceContinuity(s_s1, 1.0, 0.5, s_s1, 1.0, 0.5, G2);
        Assert.Equal(0.0, c.G2CurvatureGap(), 1e-14);
        Assert.True(c.IsG2());

        // the plane touching the sphere: the gap is |eta| / 2 + |zeta| / 2 with eta = curvature of the sphere, zeta = 0
        c = new LocalAnalysis_SurfaceContinuity(s_s1, 0.0, 0.0, s_plane, 0.0, 0.0, G2);
        Assert.True(c.IsDone());
        Assert.Equal(0.0, c.G1Angle(), 1e-15);
        Assert.Equal(0.5, c.G2CurvatureGap(), 1e-14);

        // a plane tilted by 0.2 rad about z
        var aTilted = new Geom_Plane(new CoordinateSystem(new Point3D(1, 0, 0), new Direction3D(Math.Cos(0.2), Math.Sin(0.2), 0)));
        c = new LocalAnalysis_SurfaceContinuity(s_plane, 0.0, 0.0, aTilted, 0.0, 0.0, G1);
        Assert.Equal(0.2, c.G1Angle(), 1e-14);
        Assert.False(c.IsG1());
    }

    [Fact]
    public void ComputeAnalysis_ReusesTheObjectAsGeomPlateDoes()
    {
        var c = new LocalAnalysis_SurfaceContinuity();
        Assert.True(c.IsDone());
        var aP1 = new GeomLProp_SLProps(s_s1, 0.0, 0.0, 2, 0.001);
        var aP2 = new GeomLProp_SLProps(s_s2, 0.0, 0.0, 2, 0.001);
        c.ComputeAnalysis(aP1, aP2, G2);
        Assert.Equal(G2, c.ContinuityStatus());
        Assert.Equal(0.25, c.G2CurvatureGap(), 1e-14);

        var aP3 = new GeomLProp_SLProps(s_s1, 0.0, 0.0, 2, 0.001);
        c.ComputeAnalysis(aP1, aP3, G2);
        Assert.Equal(0.0, c.G2CurvatureGap(), 1e-14);
        Assert.Equal(0.0, c.C0Value());

        // a failure is kept by later analyses, as in OCCT
        var aPole = new GeomLProp_SLProps(s_s1, 0.0, M_PI / 2, 2, 0.001);
        c.ComputeAnalysis(aPole, aP1, C1);
        Assert.False(c.IsDone());
        Assert.Equal(LocalAnalysis_NullFirstDerivative, c.StatusError());
        c.ComputeAnalysis(aP1, aP3, G2);
        Assert.False(c.IsDone());
        Assert.Throws<NotDoneException>(() => c.C0Value());
    }

    [Fact]
    public void AtThePoleOfASphere_TheFirstDerivativeIsNull()
    {
        var c = new LocalAnalysis_SurfaceContinuity(s_s1, 0.0, M_PI / 2, s_s1, 0.0, M_PI / 2, C2);
        Assert.False(c.IsDone());
        Assert.Equal(LocalAnalysis_NullFirstDerivative, c.StatusError());

        // the plane's second derivatives are null
        c = new LocalAnalysis_SurfaceContinuity(s_plane, 0.0, 0.0, s_plane, 0.0, 0.0, C2);
        Assert.False(c.IsDone());
        Assert.Equal(LocalAnalysis_NullSecondDerivative, c.StatusError());
    }

    [Fact]
    public void ThroughPCurves_OutsideTheirRange_IsNotDone()
    {
        var aLine = new Geom2d_Line(new Point2D(0, 0), new Direction2D(1, 0));
        var aTrimmed = new Geom2d_TrimmedCurve(aLine, 0.0, 1.0);
        var c = new LocalAnalysis_SurfaceContinuity(aTrimmed, aLine, 2.0, s_s1, s_s1, C0);
        Assert.False(c.IsDone());

        c = new LocalAnalysis_SurfaceContinuity(aTrimmed, aLine, 0.5, s_s1, s_s2, G1);
        Assert.True(c.IsDone());
        Assert.Equal(s_s1.Value(0.5, 0.0).Distance(s_s2.Value(0.5, 0.0)), c.C0Value());
    }

    [Fact]
    public void Dump_WritesOcctsText()
    {
        var w = new StringWriter();
        LocalAnalysis.Dump(new LocalAnalysis_SurfaceContinuity(s_s1, 0.0, 0.0, s_s2, 0.0, 0.0, G2), w);
        Assert.Equal(" Continuity Status : G1 \n G0Value = 0\n G1Value = 0\n G2CurvatureGap = 0.25\n", w.ToString());

        w = new StringWriter();
        LocalAnalysis.Dump(new LocalAnalysis_SurfaceContinuity(s_s1, 0.0, 0.0, s_s2, 0.0, 0.0, G1), w);
        Assert.Equal(" Continuity Status : G1 \n G0Value = 0\n G1Angle = 0\n\n", w.ToString());

        w = new StringWriter();
        LocalAnalysis.Dump(new LocalAnalysis_SurfaceContinuity(s_plane, 0.0, 0.0, s_plane, 0.0, 0.0, C2), w);
        Assert.Equal("Problem in the computation \none of the  second derivatives is null\n", w.ToString());
    }
}
