// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static OcctNet.ModelingAlgorithms.Tests.LocationLawTestSupport;

namespace OcctNet.ModelingAlgorithms.Tests;

public class GeomFill_CurveAndTrihedronTests
{
    private static GeomFill_CurveAndTrihedron Law(Geom_Curve theCurve, GeomFill_TrihedronLaw? theTrihedron = null)
    {
        var aLaw = new GeomFill_CurveAndTrihedron(theTrihedron ?? new GeomFill_DraftTrihedron(new gp_Vec(0, 0, 1), 0.2));
        Assert.True(aLaw.SetCurve(new GeomAdaptor_Curve(theCurve)));
        return aLaw;
    }

    private static (gp_Mat M, gp_Vec V) D0(GeomFill_LocationLaw theLaw, double theU)
    {
        gp_Mat M = new();
        gp_Vec V = new();
        Assert.True(theLaw.D0(theU, ref M, ref V));
        return (M, V);
    }

    [Fact]
    public void D0_ColumnsAreNormalBinormalTangent()
    {
        Geom_BSplineCurve aCurve = WavyCurve();
        var aTri = new GeomFill_DraftTrihedron(new gp_Vec(0, 0, 1), 0.2);
        GeomFill_CurveAndTrihedron aLaw = Law(aCurve, aTri);
        Assert.Same(aLaw.GetCurve(), aLaw.GetCurve());
        (gp_Mat M, gp_Vec V) = D0(aLaw, 0.3);
        gp_Vec T = new(), N = new(), B = new();
        aTri.D0(0.3, ref T, ref N, ref B);
        Assert.Equal(N.XYZ(), M.Column(1));
        Assert.Equal(B.XYZ(), M.Column(2));
        Assert.Equal(T.XYZ(), M.Column(3));
        AssertNear(aCurve.Value(0.3).XYZ(), V.XYZ(), 1e-14, "V");

        var aPoles = new NCollection_Array1<gp_Pnt2d>(1, 1);
        gp_Mat M2 = new();
        gp_Vec V2 = new();
        Assert.True(aLaw.D0(0.3, ref M2, ref V2, aPoles));
        AssertEqual(M, M2, "D0 with poles");
        Assert.Equal(0, aLaw.Nb2dCurves());
    }

    [Fact]
    public void D1_D2_MatchFiniteDifferences()
    {
        GeomFill_CurveAndTrihedron aLaw = Law(WavyCurve());
        var P = new NCollection_Array1<gp_Pnt2d>(1, 1);
        var DP = new NCollection_Array1<gp_Vec2d>(1, 1);
        var D2P = new NCollection_Array1<gp_Vec2d>(1, 1);
        const double h = 1e-6;
        foreach (double u in new[] { 0.21, 0.55, 0.83 })
        {
            gp_Mat M = new(), DM = new(), D2M = new();
            gp_Vec V = new(), DV = new(), D2V = new();
            Assert.True(aLaw.D2(u, ref M, ref V, ref DM, ref DV, ref D2M, ref D2V, P, DP, D2P));
            (gp_Mat Mp, gp_Vec Vp) = D0(aLaw, u + h);
            (gp_Mat Mm, gp_Vec Vm) = D0(aLaw, u - h);
            AssertNear((Mp - Mm) / (2 * h), DM, 1e-6, "DM");
            AssertNear((Vp - Vm) / (2 * h), DV, 1e-6, "DV");

            gp_Mat M1 = new(), DMp = new(), DMm = new();
            gp_Vec V1 = new(), DVp = new(), DVm = new();
            Assert.True(aLaw.D1(u + h, ref M1, ref V1, ref DMp, ref DVp, P, DP));
            Assert.True(aLaw.D1(u - h, ref M1, ref V1, ref DMm, ref DVm, P, DP));
            AssertNear((DMp - DMm) / (2 * h), D2M, 1e-4, "D2M");
            AssertNear((DVp - DVm) / (2 * h), D2V, 1e-4, "D2V");
        }
    }

    [Fact]
    public void SetTrsf_MultipliesOnTheRight()
    {
        GeomFill_CurveAndTrihedron aLaw = Law(WavyCurve());
        (gp_Mat M0, gp_Vec V0) = D0(aLaw, 0.4);
        gp_Mat aTrsf = new(0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
        aLaw.SetTrsf(aTrsf);
        (gp_Mat M1, gp_Vec V1) = D0(aLaw, 0.4);
        AssertEqual(M0 * aTrsf, M1, "M Trsf");
        Assert.Equal(V0, V1);

        gp_Mat anId = new();
        anId.SetIdentity();
        aLaw.SetTrsf(anId);
        (gp_Mat M2, _) = D0(aLaw, 0.4);
        AssertEqual(M0, M2, "identity");
    }

    [Fact]
    public void Intervals_FuseTheCurveAndTheTrihedron()
    {
        // the curve has C2 intervals [0, 0.5, 1] at C3; the draft trihedron asks for the curve's CN
        GeomFill_CurveAndTrihedron aLaw = Law(WavyCurve());
        int aNb = aLaw.NbIntervals(GeomAbs_Shape.GeomAbs_C1);
        Assert.Equal(2, aNb);
        var T = new NCollection_Array1<double>(1, aNb + 1);
        aLaw.Intervals(T, GeomAbs_Shape.GeomAbs_C1);
        Assert.Equal(new[] { 0.0, 0.5, 1.0 }, new[] { T[1], T[2], T[3] });

        aLaw.SetInterval(0.25, 0.75);
        aLaw.GetInterval(out double f, out double l);
        Assert.Equal((0.25, 0.75), (f, l));
        aLaw.GetDomain(out f, out l);
        Assert.Equal((0.0, 1.0), (f, l));
    }

    [Fact]
    public void IsTranslation_IsRotation()
    {
        double anError = -1;
        GeomFill_CurveAndTrihedron aLine = Law(new Geom_TrimmedCurve(new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0)), 0, 5));
        // a trimmed line is adapted as a line
        Assert.True(aLine.IsTranslation(ref anError));
        Assert.Equal(0.0, anError);
        Assert.False(aLine.IsRotation(ref anError));

        GeomFill_CurveAndTrihedron aCircle = Law(new Geom_Circle(new gp_Ax2(new gp_Pnt(1, 2, 3), new gp_Dir(0, 0, 1)), 2));
        Assert.False(aCircle.IsTranslation(ref anError));
        Assert.True(aCircle.IsRotation(ref anError));
        gp_Pnt aCentre = new();
        aCircle.Rotation(ref aCentre);
        Assert.Equal(new gp_Pnt(1, 2, 3), aCentre);
    }

    [Fact]
    public void GetAverageLaw_KeepsOcctColumnOrder()
    {
        var aTri = new TestFixedTrihedron(new gp_Vec(1, 0, 0), new gp_Vec(0, 1, 0));
        GeomFill_CurveAndTrihedron aLaw = Law(new Geom_TrimmedCurve(new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0)), 0, 10), aTri);
        gp_Mat AM = new();
        gp_Vec AV = new();
        aLaw.GetAverageLaw(ref AM, ref AV);
        // OCCT: (tangent, normal, binormal) here, (normal, binormal, tangent) in D0
        Assert.Equal(new gp_XYZ(1, 0, 0), AM.Column(1));
        Assert.Equal(new gp_XYZ(0, 1, 0), AM.Column(2));
        AssertNear(new gp_XYZ(5, 0, 0), AV.XYZ(), 1e-12, "AV");
        Assert.Equal(1.0, aLaw.GetMaximalNorm());
    }

    [Fact]
    public void Copy_HasItsOwnTrihedronAndTransformation()
    {
        var aTri = new GeomFill_DraftTrihedron(new gp_Vec(0, 0, 1), 0.2);
        GeomFill_CurveAndTrihedron aLaw = Law(WavyCurve(), aTri);
        gp_Mat aTrsf = new(0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
        aLaw.SetTrsf(aTrsf);
        var aCopy = (GeomFill_CurveAndTrihedron)aLaw.Copy();
        (gp_Mat M, gp_Vec V) = D0(aLaw, 0.6);
        (gp_Mat Mc, gp_Vec Vc) = D0(aCopy, 0.6);
        AssertEqual(M, Mc, "copy");
        Assert.Equal(V, Vc);

        aTri.SetAngle(0.5); // changes the original's trihedron, not the copy's
        (gp_Mat Mc2, _) = D0(aCopy, 0.6);
        AssertEqual(Mc, Mc2, "copy after the original changed");
        (gp_Mat M2, _) = D0(aLaw, 0.6);
        Assert.False(M == M2);
    }
}
