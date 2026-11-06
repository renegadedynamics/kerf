// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Kerf.Kernel.ModelingAlgorithms.Tests.LocationLawTestSupport;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

public class GeomFill_LocationDraftTests
{
    private const double Angle = 0.3;

    // A draft along a circle of radius 5 in the plane z = 0, direction Z; the stop surface is the plane z = -4.
    private static LocationDraft Draft(bool theWithStop = true)
    {
        var aLaw = new LocationDraft(new Direction3D(0, 0, 1), Angle);
        aLaw.SetCurve(new CurveAdaptor(new CircleCurve3D(Constants.XOY(), 5), 0, 3));
        if (theWithStop)
        {
            aLaw.SetStopSurf(new SurfaceAdaptor(new PlaneSurface(new Point3D(0, 0, -4), new Direction3D(0, 0, 1))));
        }
        return aLaw;
    }

    [Fact]
    public void WithoutStopSurface_IsTheDraftTrihedron()
    {
        LocationDraft aLaw = Draft(false);
        Assert.False(aLaw.IsIntersec);
        Assert.Equal(0, aLaw.TraceNumber());
        Assert.False(aLaw.HasLastRestriction);
        Assert.Equal(new Direction3D(0, 0, 1), aLaw.Direction());

        var aTri = new DraftTrihedron(new Vector3D(0, 0, 1), Angle);
        aTri.SetCurve(aLaw.GetCurve()!);
        Matrix3x3 M = new();
        Vector3D V = new(), T = new(), N = new(), B = new();
        Assert.True(aLaw.D0(1.2, ref M, ref V));
        aTri.D0(1.2, ref T, ref N, ref B);
        Assert.Equal(new Matrix3x3(N.XYZ(), B.XYZ(), T.XYZ()), M);

        double anError = 0;
        Assert.False(aLaw.IsTranslation(ref anError));
        Assert.True(aLaw.IsRotation(ref anError));
        Point3D aCentre = new(1, 1, 1);
        aLaw.Rotation(ref aCentre);
        Assert.Equal(new Point3D(0, 0, 0), aCentre);
    }

    [Fact]
    public void D0_TheTraceIsOnTheStopSurfaceAndTheGeneratrix()
    {
        LocationDraft aLaw = Draft();
        Assert.True(aLaw.IsIntersec);
        Assert.Equal(1, aLaw.TraceNumber());
        Assert.False(aLaw.HasFirstRestriction);
        Assert.True(aLaw.HasLastRestriction);
        Assert.Equal(2, aLaw.Nb2dCurves()); // the trace and the last restriction

        var aPlane = new PlaneSurface(new Point3D(0, 0, -4), new Direction3D(0, 0, 1));
        var aCircle = new CircleCurve3D(Constants.XOY(), 5);
        var aPoles = new NCollection_Array1<Point2D>(1, 2);
        foreach (double t in new[] { 0.0, 0.37, 1.5, 2.999 })
        {
            Matrix3x3 M = new();
            Vector3D V = new();
            Assert.True(aLaw.D0(t, ref M, ref V, aPoles));
            Point3D aOnSurf = aPlane.Value(aPoles[1].X, aPoles[1].Y);
            Assert.Equal(t, aPoles[2].Y);
            // the generatrix: from the curve's point along cos(a) B + sin(a) N
            Vector3D D = Math.Cos(Angle) * new Vector3D(M.Column(2)) + Math.Sin(Angle) * new Vector3D(M.Column(1));
            Point3D aOnGen = aCircle.Value(t).Translated(aPoles[2].X * D.Normalized());
            Assert.True(aOnSurf.DistanceTo(aOnGen) < 1e-6, $"t = {t}: {aOnSurf} and {aOnGen}");
            Assert.Equal(-4.0, aOnGen.Z, 1e-6);
        }
    }

    private static (Vector2D Surface, Vector2D Curve, Vector2D FDSurface, Vector2D FDCurve) PoleDerivatives(
        LocationDraft theLaw, double t)
    {
        var P = new NCollection_Array1<Point2D>(1, 2);
        var Pp = new NCollection_Array1<Point2D>(1, 2);
        var Pm = new NCollection_Array1<Point2D>(1, 2);
        var DP = new NCollection_Array1<Vector2D>(1, 2);
        const double h = 1e-5;
        Matrix3x3 M = new(), DM = new();
        Vector3D V = new(), DV = new();
        Assert.True(theLaw.D1(t, ref M, ref V, ref DM, ref DV, P, DP));
        Assert.True(theLaw.D0(t + h, ref M, ref V, Pp));
        Assert.True(theLaw.D0(t - h, ref M, ref V, Pm));
        return (DP[1], DP[2], new Vector2D(Pm[1], Pp[1]).Divided(2 * h), new Vector2D(Pm[2], Pp[2]).Divided(2 * h));
    }

    [Fact]
    public void D1_PolesMatchFiniteDifferencesAlongALine()
    {
        // a straight path (a constant trihedron) and a tilted stop plane: the trace moves along the plane
        var aLaw = new LocationDraft(new Direction3D(0, 0, 1), Angle);
        aLaw.SetCurve(new CurveAdaptor(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), 0, 4));
        aLaw.SetStopSurf(new SurfaceAdaptor(new PlaneSurface(new Point3D(0, 0, -4), new Direction3D(0.2, 0.1, 1))));
        Assert.True(aLaw.IsIntersec);
        (Vector2D aSurf, Vector2D aCurve, Vector2D aFDSurf, Vector2D aFDCurve) = PoleDerivatives(aLaw, 1.7);
        Assert.True(aFDSurf.Subtracted(aSurf).Magnitude < 1e-6, $"{aFDSurf} and {aSurf}");
        Assert.True(aFDCurve.Subtracted(aCurve).Magnitude < 1e-6, $"{aFDCurve} and {aCurve}");
        Assert.Equal(1.0, aCurve.Y);
    }

    [Fact]
    public void D1_PolesMatchFiniteDifferencesAlongACircle()
    {
        // Decision 20605: OCCT 8.0.1 differentiates the generatrix P + w (cos(a) B + sin(a) N) as P' + w sin(a) N',
        // without w cos(a) B'; along a circle B' is not zero, and the trace's derivative was off by 1.6 times
        (Vector2D aSurf, _, Vector2D aFDSurf, _) = PoleDerivatives(Draft(), 1.1);
        Assert.True(aFDSurf.Subtracted(aSurf).Magnitude < 1e-5, $"{aFDSurf} and {aSurf}");
    }

    [Fact]
    public void D2_PolesMatchFiniteDifferencesAlongACircle()
    {
        LocationDraft aLaw = Draft();
        var P = new NCollection_Array1<Point2D>(1, 2);
        var DP = new NCollection_Array1<Vector2D>(1, 2);
        var D2P = new NCollection_Array1<Vector2D>(1, 2);
        var DPm = new NCollection_Array1<Vector2D>(1, 2);
        var DPp = new NCollection_Array1<Vector2D>(1, 2);
        Matrix3x3 M = new(), DM = new(), D2M = new();
        Vector3D V = new(), DV = new(), D2V = new();
        const double t = 1.1, h = 1e-4;
        Assert.True(aLaw.D2(t, ref M, ref V, ref DM, ref DV, ref D2M, ref D2V, P, DP, D2P));
        Assert.True(aLaw.D1(t - h, ref M, ref V, ref DM, ref DV, P, DPm));
        Assert.True(aLaw.D1(t + h, ref M, ref V, ref DM, ref DV, P, DPp));
        Vector2D aFD = DPp[1].Subtracted(DPm[1]).Divided(2 * h);
        Assert.True(aFD.Subtracted(D2P[1]).Magnitude < 1e-4, $"{aFD} and {D2P[1]}");
    }

    [Fact]
    public void D2_MatricesAreTheTrihedrons()
    {
        LocationDraft aLaw = Draft();
        var P = new NCollection_Array1<Point2D>(1, 2);
        var DP = new NCollection_Array1<Vector2D>(1, 2);
        var D2P = new NCollection_Array1<Vector2D>(1, 2);
        double t = 1.1;
        Matrix3x3 M = new(), DM = new(), D2M = new();
        Vector3D V = new(), DV = new(), D2V = new();
        Assert.True(aLaw.D2(t, ref M, ref V, ref DM, ref DV, ref D2M, ref D2V, P, DP, D2P));
        Assert.Equal(0.0, D2P[2].Y);
        Assert.Equal(1.0, DP[2].Y);

        // the matrices are those of the trihedron, as without a stop surface
        LocationDraft aFree = Draft(false);
        Matrix3x3 Mf = new(), DMf = new(), D2Mf = new();
        Vector3D Vf = new(), DVf = new(), D2Vf = new();
        aFree.D2(t, ref Mf, ref Vf, ref DMf, ref DVf, ref D2Mf, ref D2Vf, P, DP, D2P);
        Assert.Equal(Mf, M);
        Assert.Equal(DMf, DM);
        Assert.Equal(D2Mf, D2M);
    }

    [Fact]
    public void Resolution_SetTrsf_Copy()
    {
        LocationDraft aLaw = Draft();
        aLaw.Resolution(1, 1e-3, out double aTolU, out double aTolV);
        Assert.Equal(1e-3, aTolU);
        Assert.Equal(1e-3, aTolV);
        aLaw.Resolution(2, 1e-3, out aTolU, out aTolV);
        Assert.Equal(1e-3, aTolU);

        Matrix3x3 M0 = new(), M1 = new();
        Vector3D V = new();
        aLaw.D0(0.5, ref M0, ref V);
        Matrix3x3 aTrsf = new(0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
        aLaw.SetTrsf(aTrsf);
        aLaw.D0(0.5, ref M1, ref V);
        Assert.Equal(M0 * aTrsf, M1);

        var aCopy = (LocationDraft)aLaw.Copy();
        Assert.True(aCopy.IsIntersec);
        Matrix3x3 Mc = new();
        aCopy.D0(0.5, ref Mc, ref V);
        Assert.Equal(M1, Mc);

        aCopy.SetAngle(0.5); // the original keeps its angle
        Matrix3x3 M2 = new();
        aLaw.D0(0.5, ref M2, ref V);
        Assert.Equal(M1, M2);

        aLaw.SetInterval(1, 2);
        aLaw.GetInterval(out double f, out double l);
        Assert.Equal((1.0, 2.0), (f, l));
        aLaw.GetDomain(out f, out l);
        Assert.Equal((0.0, 3.0), (f, l));
        Assert.Equal(1.0, aLaw.GetMaximalNorm());
    }
}
