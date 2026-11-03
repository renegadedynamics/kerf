// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Kerf.Kernel.ModelingAlgorithms.Tests.LocationLawTestSupport;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// GeomFill_LocationGuide with a test trihedron with guide (a fixed frame T = Z, N = X); the guided trihedrons
/// (GeomFill_GuideTrihedronAC, GuideTrihedronPlan) are tested with it after the merge of unit 5L1.
/// </summary>
public class GeomFill_LocationGuideTests
{
    private const double Theta = 0.7;

    // The path: the Z axis over [0, 4]; the guide: the parallel line at radius 2 and angle Theta.
    private static (LocationGuide Law, TestFixedGuideTrihedron Tri) Guided()
    {
        var aGuide = new CurveAdaptor(
            new LineCurve3D(new Point3D(2 * Math.Cos(Theta), 2 * Math.Sin(Theta), 0), new Direction3D(0, 0, 1)), -1, 5);
        var aTri = new TestFixedGuideTrihedron(aGuide, new Vector3D(0, 0, 1), new Vector3D(1, 0, 0));
        var aLaw = new LocationGuide(aTri);
        Assert.True(aLaw.SetCurve(new CurveAdaptor(new LineCurve3D(Constants.OZ()), 0, 4)));
        return (aLaw, aTri);
    }

    // The section: the radial segment [0, 3] along X, which the rotation must turn onto the guide.
    private static UniformSection Segment() =>
        new(new TrimmedCurve3D(new LineCurve3D(Constants.OX()), 0, 3), 0, 4);

    [Fact]
    public void WithoutRotation_IsTheTrihedron()
    {
        (LocationGuide aLaw, TestFixedGuideTrihedron aTri) = Guided();
        Assert.Equal(1, aTri.NbOrigine);
        Assert.Equal(SurfacePipeError.PipeOk, aLaw.ErrorStatus());
        Matrix3x3 M = new();
        Vector3D V = new();
        Assert.True(aLaw.D0(1.5, ref M, ref V));
        Assert.Equal(new Matrix3x3(new Xyz(1, 0, 0), new Xyz(0, 1, 0), new Xyz(0, 0, 1)), M);
        Assert.Equal(new Xyz(0, 0, 1.5), V.XYZ());

        // the guide is trimmed a hundredth beyond its ends (it isn't periodic)
        Assert.Equal(-1.06, aLaw.Guide().FirstParameter(), 1e-12);
        Assert.Equal(5.06, aLaw.Guide().LastParameter(), 1e-12);
        Assert.Equal(0, aLaw.TraceNumber());
        double anError = 0;
        Assert.False(aLaw.IsTranslation(ref anError));
        Assert.False(aLaw.IsRotation(ref anError));
        Point3D aCentre = new();
        Assert.Throws<MethodNotImplementedException>(() => aLaw.Rotation(ref aCentre));
    }

    [Fact]
    public void ComputeAutomaticLaw_GivesTheDistanceToTheGuide()
    {
        (LocationGuide aLaw, _) = Guided();
        Assert.Equal(SurfacePipeError.PipeOk, aLaw.ComputeAutomaticLaw(out NCollection_HArray1<Point2D> aParAndRad));
        Assert.Equal(21, aParAndRad.Length());
        Assert.Equal(0.0, aParAndRad[1].X());
        Assert.Equal(4.0, aParAndRad[21].X());
        for (int i = 1; i <= 21; i++)
        {
            Assert.Equal(2.0, aParAndRad[i].Y(), 1e-12);
        }
    }

    [Fact]
    public void Rotation_TurnsTheSectionOntoTheGuide()
    {
        (LocationGuide aLaw, _) = Guided();
        double aLastAngle = 0;
        aLaw.Set(Segment(), true, 0, 4, 0, ref aLastAngle);
        Assert.Equal(SurfacePipeError.PipeOk, aLaw.ErrorStatus());
        Assert.Equal(Theta, aLastAngle, 1e-6);

        foreach (double t in new[] { 0.0, 0.33, 2.0, 3.9 })
        {
            Matrix3x3 M = new();
            Vector3D V = new();
            Assert.True(aLaw.D0(t, ref M, ref V));
            // the normal column now points at the guide; the tangent is unchanged
            AssertNear(new Xyz(Math.Cos(Theta), Math.Sin(Theta), 0), M.Column(1), 1e-6, $"normal at {t}");
            Assert.Equal(new Xyz(0, 0, 1), M.Column(3));
            var aPoles = new NCollection_Array1<Point2D>(1, 1);
            Matrix3x3 M2 = new();
            Assert.True(aLaw.D0(t, ref M2, ref V, aPoles));
            Assert.Equal(M, M2);
        }

        // the derivatives are not computed with a rotation (commented out in OCCT)
        var P = new NCollection_Array1<Point2D>(1, 1);
        var DP = new NCollection_Array1<Vector2D>(1, 1);
        Matrix3x3 Mx = new(), DMx = new(), D2Mx = new();
        Vector3D Vx = new(), DVx = new(), D2Vx = new();
        Assert.False(aLaw.D1(1, ref Mx, ref Vx, ref DMx, ref DVx, P, DP));
        Assert.False(aLaw.D2(1, ref Mx, ref Vx, ref DMx, ref DVx, ref D2Mx, ref D2Vx, P, DP, DP));

        Assert.IsType<TrimmedCurve3D>(aLaw.Section());

        aLaw.EraseRotation();
        Matrix3x3 M3 = new();
        Vector3D V3 = new();
        Assert.True(aLaw.D0(1, ref M3, ref V3));
        Assert.Equal(new Xyz(1, 0, 0), M3.Column(1));
    }

    [Fact]
    public void Copy_WithARotation_ThrowsAsInOcct()
    {
        (LocationGuide aLaw, _) = Guided();
        double aLastAngle = 0;
        aLaw.Set(Segment(), true, 0, 4, 0, ref aLastAngle);
        // OCCT: the copy computes its rotation before it has a path
        Assert.Throws<ConstructionErrorException>(() => aLaw.Copy());

        aLaw.EraseRotation();
        var aCopy = (LocationGuide)aLaw.Copy();
        Assert.Null(aCopy.GetCurve());
    }

    [Fact]
    public void D2_WithATransformation_DoesNotApplyItAsInOcct()
    {
        (LocationGuide aLaw, _) = Guided();
        Matrix3x3 aTrsf = new(0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
        aLaw.SetTrsf(aTrsf);
        var P = new NCollection_Array1<Point2D>(1, 1);
        var DP = new NCollection_Array1<Vector2D>(1, 1);
        Matrix3x3 M0 = new(), M1 = new(), DM1 = new(), M2 = new(), DM2 = new(), D2M2 = new();
        Vector3D V = new(), DV = new(), D2V = new();
        Assert.True(aLaw.D0(1, ref M0, ref V));
        Assert.True(aLaw.D1(1, ref M1, ref V, ref DM1, ref DV, P, DP));
        Assert.True(aLaw.D2(1, ref M2, ref V, ref DM2, ref DV, ref D2M2, ref D2V, P, DP, DP));
        Matrix3x3 anM = new(new Xyz(1, 0, 0), new Xyz(0, 1, 0), new Xyz(0, 0, 1));
        Assert.Equal(anM * aTrsf, M0);
        Assert.Equal(anM * aTrsf, M1);
        Assert.Equal(anM, M2);
    }

    [Fact]
    public void Intervals_Tolerance_Average()
    {
        (LocationGuide aLaw, _) = Guided();
        Assert.Equal(1, aLaw.NbIntervals(Continuity.C2));
        var T = new NCollection_Array1<double>(1, 2);
        aLaw.Intervals(T, Continuity.C2);
        Assert.Equal((0.0, 4.0), (T[1], T[2]));
        aLaw.SetInterval(1, 3);
        aLaw.GetInterval(out double f, out double l);
        Assert.Equal((1.0, 3.0), (f, l));
        aLaw.GetDomain(out f, out l);
        Assert.Equal((0.0, 4.0), (f, l));

        aLaw.Resolution(1, 1e-3, out double aTolU, out double aTolV);
        Assert.Equal(1e-5, aTolU);
        Assert.Equal(1e-5, aTolV);
        aLaw.SetTolerance(1e-4, 1e-5);

        Matrix3x3 AM = new();
        Vector3D AV = new();
        aLaw.GetAverageLaw(ref AM, ref AV);
        // OCCT: 22 points a tenth of [1, 3] apart, so the average is at z = 1 + 2.1 * 2 / 2
        AssertNear(new Xyz(0, 0, 1 + 0.2 * 21 / 2), AV.XYZ(), 1e-12, "AV");
    }
}
