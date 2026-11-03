// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static OcctNet.ModelingAlgorithms.Tests.LocationLawTestSupport;

namespace OcctNet.ModelingAlgorithms.Tests;

public class GeomFill_DraftTrihedronTests
{
    private static GeomFill_DraftTrihedron Draft(Curve3D theCurve, double theAngle = 0.2)
    {
        var aLaw = new GeomFill_DraftTrihedron(new Vector3D(0, 0, 2), theAngle);
        aLaw.SetCurve(new CurveAdaptor(theCurve));
        return aLaw;
    }

    private static (Vector3D T, Vector3D N, Vector3D B) D0(GeomFill_TrihedronLaw theLaw, double theU)
    {
        Vector3D T = new(), N = new(), B = new();
        Assert.True(theLaw.D0(theU, ref T, ref N, ref B));
        return (T, N, B);
    }

    [Fact]
    public void D0_IsOrthonormalWithTheDraftAngle()
    {
        GeomFill_DraftTrihedron aLaw = Draft(WavyCurve(), 0.2);
        for (double u = 0.05; u < 1; u += 0.1)
        {
            (Vector3D T, Vector3D N, Vector3D B) = D0(aLaw, u);
            Assert.Equal(1.0, T.Magnitude(), 1e-12);
            Assert.Equal(1.0, N.Magnitude(), 1e-12);
            Assert.Equal(1.0, B.Magnitude(), 1e-12);
            Assert.Equal(0.0, T.Dot(N), 1e-12);
            Assert.Equal(0.0, T.Dot(B), 1e-12);
            // the tangent is perpendicular to the draft direction
            Assert.Equal(0.0, T.Dot(new Vector3D(0, 0, 1)), 1e-12);
        }
    }

    [Fact]
    public void D1_D2_MatchFiniteDifferences()
    {
        GeomFill_DraftTrihedron aLaw = Draft(WavyCurve(), 0.3);
        const double h = 1e-6;
        foreach (double u in new[] { 0.13, 0.37, 0.71, 0.9 })
        {
            Vector3D T = new(), DT = new(), D2T = new(), N = new(), DN = new(), D2N = new(), B = new(), DB = new(), D2B = new();
            Assert.True(aLaw.D2(u, ref T, ref DT, ref D2T, ref N, ref DN, ref D2N, ref B, ref DB, ref D2B));
            (Vector3D Tp, Vector3D Np, Vector3D Bp) = D0(aLaw, u + h);
            (Vector3D Tm, Vector3D Nm, Vector3D Bm) = D0(aLaw, u - h);
            AssertNear((Tp - Tm) / (2 * h), DT, 1e-6, "DT");
            AssertNear((Np - Nm) / (2 * h), DN, 1e-6, "DN");
            AssertNear((Bp - Bm) / (2 * h), DB, 1e-6, "DB");

            Vector3D T1 = new(), DT1p = new(), N1 = new(), DN1p = new(), B1 = new(), DB1p = new();
            Vector3D DT1m = new(), DN1m = new(), DB1m = new();
            Assert.True(aLaw.D1(u + h, ref T1, ref DT1p, ref N1, ref DN1p, ref B1, ref DB1p));
            Assert.True(aLaw.D1(u - h, ref T1, ref DT1m, ref N1, ref DN1m, ref B1, ref DB1m));
            AssertNear((DT1p - DT1m) / (2 * h), D2T, 1e-4, "D2T");
            AssertNear((DN1p - DN1m) / (2 * h), D2N, 1e-4, "D2N");
            AssertNear((DB1p - DB1m) / (2 * h), D2B, 1e-4, "D2B");

            // D1's and D2's frames are D0's
            (Vector3D T0, Vector3D N0, Vector3D B0) = D0(aLaw, u);
            AssertNear(T0, T, 1e-12, "T");
            AssertNear(N0, N, 1e-12, "N");
            AssertNear(B0, B, 1e-12, "B");
        }
    }

    [Fact]
    public void D0_TangentAlongTheDraftDirection_ReturnsFalse()
    {
        var aLaw = new GeomFill_DraftTrihedron(new Vector3D(0, 0, 1), 0.1);
        aLaw.SetCurve(new CurveAdaptor(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 0, 1));
        Vector3D T = new(), N = new(), B = new();
        Assert.False(aLaw.D0(0.5, ref T, ref N, ref B));
    }

    [Fact]
    public void Intervals_RaiseTheContinuity()
    {
        // C1 at the knot 0.5 (multiplicity 2 in degree 3): C0 needs the curve's C2 intervals
        var aPoles = new NCollection_Array1<Point3D>(1, 6);
        for (int i = 1; i <= 6; i++)
        {
            aPoles[i] = new Point3D(i, Math.Sin(i), 0.1 * i * i);
        }
        var aKnots = new NCollection_Array1<double>(1, 3);
        aKnots[1] = 0;
        aKnots[2] = 0.5;
        aKnots[3] = 1;
        var aMults = new NCollection_Array1<int>(1, 3);
        aMults[1] = 4;
        aMults[2] = 2;
        aMults[3] = 4;
        GeomFill_DraftTrihedron aLaw = Draft(new BSplineCurve3D(aPoles, aKnots, aMults, 3));
        Assert.Equal(2, aLaw.NbIntervals(Continuity.C0));
        var T = new NCollection_Array1<double>(1, 3);
        aLaw.Intervals(T, Continuity.C0);
        Assert.Equal(0.0, T[1]);
        Assert.Equal(0.5, T[2]);
        Assert.Equal(1.0, T[3]);
    }

    [Fact]
    public void IsConstant_IsOnlyBy3dCurve()
    {
        GeomFill_DraftTrihedron aLine = Draft(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)));
        Assert.True(aLine.IsConstant());
        Assert.False(aLine.IsOnlyBy3dCurve());

        GeomFill_DraftTrihedron aCircle = Draft(new CircleCurve3D(Constants.XOY(), 3));
        Assert.False(aCircle.IsConstant());
        Assert.True(aCircle.IsOnlyBy3dCurve()); // the circle's axis is the draft direction

        var aTilted = new GeomFill_DraftTrihedron(new Vector3D(0, 1, 1), 0.2);
        aTilted.SetCurve(new CurveAdaptor(new CircleCurve3D(Constants.XOY(), 3)));
        Assert.False(aTilted.IsOnlyBy3dCurve());
    }

    [Fact]
    public void Copy_IsIndependent()
    {
        GeomFill_DraftTrihedron aLaw = Draft(WavyCurve(), 0.25);
        var aCopy = (GeomFill_DraftTrihedron)aLaw.Copy();
        (Vector3D T, Vector3D N, Vector3D B) = D0(aLaw, 0.4);
        (Vector3D Tc, Vector3D Nc, Vector3D Bc) = D0(aCopy, 0.4);
        Assert.Equal(T, Tc);
        Assert.Equal(N, Nc);
        Assert.Equal(B, Bc);

        aCopy.SetAngle(0.6);
        aCopy.SetInterval(0.2, 0.6);
        (Vector3D T2, Vector3D N2, _) = D0(aLaw, 0.4);
        Assert.Equal(T, T2);
        Assert.Equal(N, N2);
        aLaw.GetInterval(out double f, out double l);
        Assert.Equal(0.0, f);
        Assert.Equal(1.0, l);
        aCopy.GetInterval(out f, out l);
        Assert.Equal(0.2, f);
        Assert.Equal(0.6, l);
    }

    [Fact]
    public void GetAverageLaw_IsTheMeanOf21Frames()
    {
        GeomFill_DraftTrihedron aLaw = Draft(WavyCurve());
        Vector3D AT = new(), AN = new(), AB = new();
        aLaw.GetAverageLaw(ref AT, ref AN, ref AB);
        Vector3D ST = new(0, 0, 0), SN = new(0, 0, 0), SB = new(0, 0, 0);
        for (int i = 0; i <= 20; i++)
        {
            (Vector3D T, Vector3D N, Vector3D B) = D0(aLaw, i * (1.0 / 20));
            ST += T;
            SN += N;
            SB += B;
        }
        AssertNear(ST / 21, AT, 1e-14, "T");
        AssertNear(SN / 21, AN, 1e-14, "N");
        AssertNear(SB / 21, AB, 1e-14, "B");
    }
}
