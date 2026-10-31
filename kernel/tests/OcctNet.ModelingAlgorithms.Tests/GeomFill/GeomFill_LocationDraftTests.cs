// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static OcctNet.ModelingAlgorithms.Tests.LocationLawTestSupport;

namespace OcctNet.ModelingAlgorithms.Tests;

public class GeomFill_LocationDraftTests
{
    private const double Angle = 0.3;

    // A draft along a circle of radius 5 in the plane z = 0, direction Z; the stop surface is the plane z = -4.
    private static GeomFill_LocationDraft Draft(bool theWithStop = true)
    {
        var aLaw = new GeomFill_LocationDraft(new gp_Dir(0, 0, 1), Angle);
        aLaw.SetCurve(new GeomAdaptor_Curve(new Geom_Circle(gp.XOY(), 5), 0, 3));
        if (theWithStop)
        {
            aLaw.SetStopSurf(new GeomAdaptor_Surface(new Geom_Plane(new gp_Pnt(0, 0, -4), new gp_Dir(0, 0, 1))));
        }
        return aLaw;
    }

    [Fact]
    public void WithoutStopSurface_IsTheDraftTrihedron()
    {
        GeomFill_LocationDraft aLaw = Draft(false);
        Assert.False(aLaw.IsIntersec());
        Assert.Equal(0, aLaw.TraceNumber());
        Assert.False(aLaw.HasLastRestriction());
        Assert.Equal(new gp_Dir(0, 0, 1), aLaw.Direction());

        var aTri = new GeomFill_DraftTrihedron(new gp_Vec(0, 0, 1), Angle);
        aTri.SetCurve(aLaw.GetCurve()!);
        gp_Mat M = new();
        gp_Vec V = new(), T = new(), N = new(), B = new();
        Assert.True(aLaw.D0(1.2, ref M, ref V));
        aTri.D0(1.2, ref T, ref N, ref B);
        Assert.Equal(new gp_Mat(N.XYZ(), B.XYZ(), T.XYZ()), M);

        double anError = 0;
        Assert.False(aLaw.IsTranslation(ref anError));
        Assert.True(aLaw.IsRotation(ref anError));
        gp_Pnt aCentre = new(1, 1, 1);
        aLaw.Rotation(ref aCentre);
        Assert.Equal(new gp_Pnt(0, 0, 0), aCentre);
    }

    [Fact]
    public void D0_TheTraceIsOnTheStopSurfaceAndTheGeneratrix()
    {
        GeomFill_LocationDraft aLaw = Draft();
        Assert.True(aLaw.IsIntersec());
        Assert.Equal(1, aLaw.TraceNumber());
        Assert.False(aLaw.HasFirstRestriction());
        Assert.True(aLaw.HasLastRestriction());
        Assert.Equal(2, aLaw.Nb2dCurves()); // the trace and the last restriction

        var aPlane = new Geom_Plane(new gp_Pnt(0, 0, -4), new gp_Dir(0, 0, 1));
        var aCircle = new Geom_Circle(gp.XOY(), 5);
        var aPoles = new NCollection_Array1<gp_Pnt2d>(1, 2);
        foreach (double t in new[] { 0.0, 0.37, 1.5, 2.999 })
        {
            gp_Mat M = new();
            gp_Vec V = new();
            Assert.True(aLaw.D0(t, ref M, ref V, aPoles));
            gp_Pnt aOnSurf = aPlane.Value(aPoles[1].X(), aPoles[1].Y());
            Assert.Equal(t, aPoles[2].Y());
            // the generatrix: from the curve's point along cos(a) B + sin(a) N
            gp_Vec D = Math.Cos(Angle) * new gp_Vec(M.Column(2)) + Math.Sin(Angle) * new gp_Vec(M.Column(1));
            gp_Pnt aOnGen = aCircle.Value(t).Translated(aPoles[2].X() * D.Normalized());
            Assert.True(aOnSurf.Distance(aOnGen) < 1e-6, $"t = {t}: {aOnSurf} and {aOnGen}");
            Assert.Equal(-4.0, aOnGen.Z(), 1e-6);
        }
    }

    private static (gp_Vec2d Surface, gp_Vec2d Curve, gp_Vec2d FDSurface, gp_Vec2d FDCurve) PoleDerivatives(
        GeomFill_LocationDraft theLaw, double t)
    {
        var P = new NCollection_Array1<gp_Pnt2d>(1, 2);
        var Pp = new NCollection_Array1<gp_Pnt2d>(1, 2);
        var Pm = new NCollection_Array1<gp_Pnt2d>(1, 2);
        var DP = new NCollection_Array1<gp_Vec2d>(1, 2);
        const double h = 1e-5;
        gp_Mat M = new(), DM = new();
        gp_Vec V = new(), DV = new();
        Assert.True(theLaw.D1(t, ref M, ref V, ref DM, ref DV, P, DP));
        Assert.True(theLaw.D0(t + h, ref M, ref V, Pp));
        Assert.True(theLaw.D0(t - h, ref M, ref V, Pm));
        return (DP[1], DP[2], new gp_Vec2d(Pm[1], Pp[1]).Divided(2 * h), new gp_Vec2d(Pm[2], Pp[2]).Divided(2 * h));
    }

    [Fact]
    public void D1_PolesMatchFiniteDifferencesAlongALine()
    {
        // a straight path (a constant trihedron) and a tilted stop plane: the trace moves along the plane
        var aLaw = new GeomFill_LocationDraft(new gp_Dir(0, 0, 1), Angle);
        aLaw.SetCurve(new GeomAdaptor_Curve(new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0)), 0, 4));
        aLaw.SetStopSurf(new GeomAdaptor_Surface(new Geom_Plane(new gp_Pnt(0, 0, -4), new gp_Dir(0.2, 0.1, 1))));
        Assert.True(aLaw.IsIntersec());
        (gp_Vec2d aSurf, gp_Vec2d aCurve, gp_Vec2d aFDSurf, gp_Vec2d aFDCurve) = PoleDerivatives(aLaw, 1.7);
        Assert.True(aFDSurf.Subtracted(aSurf).Magnitude() < 1e-6, $"{aFDSurf} and {aSurf}");
        Assert.True(aFDCurve.Subtracted(aCurve).Magnitude() < 1e-6, $"{aFDCurve} and {aCurve}");
        Assert.Equal(1.0, aCurve.Y());
    }

    [Fact]
    public void D1_PolesOmitTheBinormalDerivativeAsInOcct()
    {
        // OCCT: FunctionDraft::DerivT differentiates the generatrix P + w (cos(a) B + sin(a) N) as
        // P' + w sin(a) N', without w cos(a) B'; along a circle B' is not zero, so the trace's derivative is off
        (gp_Vec2d aSurf, _, gp_Vec2d aFDSurf, _) = PoleDerivatives(Draft(), 1.1);
        Assert.True(aFDSurf.Subtracted(aSurf).Magnitude() > 0.1, $"{aFDSurf} and {aSurf}");
    }

    [Fact]
    public void D2_MatricesAreTheTrihedrons()
    {
        GeomFill_LocationDraft aLaw = Draft();
        var P = new NCollection_Array1<gp_Pnt2d>(1, 2);
        var DP = new NCollection_Array1<gp_Vec2d>(1, 2);
        var D2P = new NCollection_Array1<gp_Vec2d>(1, 2);
        double t = 1.1;
        gp_Mat M = new(), DM = new(), D2M = new();
        gp_Vec V = new(), DV = new(), D2V = new();
        Assert.True(aLaw.D2(t, ref M, ref V, ref DM, ref DV, ref D2M, ref D2V, P, DP, D2P));
        Assert.Equal(0.0, D2P[2].Y());
        Assert.Equal(1.0, DP[2].Y());

        // the matrices are those of the trihedron, as without a stop surface
        GeomFill_LocationDraft aFree = Draft(false);
        gp_Mat Mf = new(), DMf = new(), D2Mf = new();
        gp_Vec Vf = new(), DVf = new(), D2Vf = new();
        aFree.D2(t, ref Mf, ref Vf, ref DMf, ref DVf, ref D2Mf, ref D2Vf, P, DP, D2P);
        Assert.Equal(Mf, M);
        Assert.Equal(DMf, DM);
        Assert.Equal(D2Mf, D2M);
    }

    [Fact]
    public void Resolution_SetTrsf_Copy()
    {
        GeomFill_LocationDraft aLaw = Draft();
        aLaw.Resolution(1, 1e-3, out double aTolU, out double aTolV);
        Assert.Equal(1e-3, aTolU);
        Assert.Equal(1e-3, aTolV);
        aLaw.Resolution(2, 1e-3, out aTolU, out aTolV);
        Assert.Equal(1e-3, aTolU);

        gp_Mat M0 = new(), M1 = new();
        gp_Vec V = new();
        aLaw.D0(0.5, ref M0, ref V);
        gp_Mat aTrsf = new(0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
        aLaw.SetTrsf(aTrsf);
        aLaw.D0(0.5, ref M1, ref V);
        Assert.Equal(M0 * aTrsf, M1);

        var aCopy = (GeomFill_LocationDraft)aLaw.Copy();
        Assert.True(aCopy.IsIntersec());
        gp_Mat Mc = new();
        aCopy.D0(0.5, ref Mc, ref V);
        Assert.Equal(M1, Mc);

        aCopy.SetAngle(0.5); // the original keeps its angle
        gp_Mat M2 = new();
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
