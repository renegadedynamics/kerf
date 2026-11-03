// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Kerf.Kernel.ModelingAlgorithms.Tests.GeomFill_TrihedronTests;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of GeomFill_CorrectedFrenet (the oracle compares it with native OCCT), and of GeomFill_LocationGuide with
/// the guided trihedrons of unit 5L1.
/// </summary>
public class GeomFill_CorrectedFrenetTests
{
    private static CorrectedFrenet Law(Curve3D theC, bool theForEvaluation = false)
    {
        var aLaw = new CorrectedFrenet(theForEvaluation);
        aLaw.SetCurve(new CurveAdaptor(theC));
        return aLaw;
    }

    // A helix approximated by a B-spline: Frenet turns about the tangent there, the corrected law much less.
    private static BSplineCurve3D Helix()
    {
        Point3D[] aPts = new Point3D[17];
        for (int i = 0; i < aPts.Length; i++)
        {
            double t = i * 0.5;
            aPts[i] = new Point3D(Math.Cos(t), Math.Sin(t), 0.3 * t);
        }
        return BSpline(aPts);
    }

    [Fact]
    public void Conic_IsFrenet()
    {
        var aCircle = new CircleCurve3D(Constants.XOY(), 2);
        CorrectedFrenet aLaw = Law(aCircle);
        var aFrenet = new Frenet();
        aFrenet.SetCurve(new CurveAdaptor(aCircle));
        Frame aC = D0(aLaw, 0.7), aF = D0(aFrenet, 0.7);
        Assert.Equal(aF, aC);
        Assert.Equal(Trihedron.IsFrenet, aLaw.EvaluateBestMode());
        Assert.False(aLaw.IsConstant);
        Assert.True(aLaw.IsOnlyBy3dCurve);
        Assert.True(Law(new LineCurve3D(Constants.OX())).IsConstant);
    }

    [Fact]
    public void Helix_OrthonormalAndDerivatives()
    {
        CorrectedFrenet aLaw = Law(Helix());
        foreach (double u in new[] { 0.3, 1.7, 4.2, 9.5, 12.8 })
        {
            AssertOrthonormal(D0(aLaw, u), 1.0e-9);
            AssertDerivatives(aLaw, u, 1.0e-5, 1.0e-4);
        }
    }

    // The laws on an offset spine use its D3 (occt-issues.md, the offset curves' CalculateD3; Kerf decision 1487):
    // OCCT's D2 there wasn't D2's, so the frames of D1 weren't D0's, nor the derivatives theirs
    [Fact]
    public void Laws_on_an_offset_spine_have_the_derivatives_of_their_frames()
    {
        var anOffset = new OffsetCurve3D(Helix(), 0.2, new Direction3D(0, 0, 1));
        var aFrenet = new Frenet();
        aFrenet.SetCurve(new CurveAdaptor(anOffset));
        foreach (TrihedronLaw aLaw in new TrihedronLaw[] { aFrenet, Law(anOffset) })
        {
            foreach (double u in new[] { 0.3, 1.7, 4.2, 9.5, 12.8 })
            {
                AssertDerivatives(aLaw, u, 1.0e-5, 1.0e-4, theCheckD2: false);
            }
        }
    }

    [Fact]
    public void Helix_TurnsLessThanFrenet()
    {
        BSplineCurve3D aHelix = Helix();
        CorrectedFrenet aLaw = Law(aHelix);
        var aFrenet = new Frenet();
        aFrenet.SetCurve(new CurveAdaptor(aHelix));
        // the rotation of the normal about the tangent: N' . B
        double aCorrected = 0, aFr = 0;
        for (double u = 1; u < 13; u += 0.5)
        {
            (Frame aC, Frame aDC) = D1(aLaw, u);
            (Frame aF, Frame aDF) = D1(aFrenet, u);
            aCorrected += Math.Abs(aDC.N.Dot(aC.B));
            aFr += Math.Abs(aDF.N.Dot(aF.B));
        }
        Assert.True(aCorrected < 0.2 * aFr, $"corrected {aCorrected}, Frenet {aFr}");
        Assert.Equal(Trihedron.IsCorrectedFrenet, aLaw.EvaluateBestMode());
    }

    [Fact]
    public void Planar_SCurve_NormalStaysOnOneSide()
    {
        // Frenet's normal flips at the inflexion; the corrected one keeps a constant binormal (the plane's normal)
        BSplineCurve3D aS = BSpline(new Point3D(0, 0, 0), new Point3D(1, 2, 0), new Point3D(2, -2, 0), new Point3D(3, 0, 0));
        CorrectedFrenet aLaw = Law(aS);
        // (not at the inflexion itself, u = 0.5, where Frenet's frame and the angle law meet from either side)
        Vector3D aB0 = D0(aLaw, 0.05).B;
        foreach (double u in new[] { 0.05, 0.2, 0.35, 0.45, 0.55, 0.65, 0.8, 0.95 })
        {
            Frame aF = D0(aLaw, u);
            AssertOrthonormal(aF, 1.0e-9);
            AssertNear(aB0, aF.B, 1.0e-7, $"binormal at {u}");
        }
    }

    [Fact]
    public void Intervals_FuseFrenetAndTheAngleLaw()
    {
        CorrectedFrenet aLaw = Law(Twisted());
        foreach (Continuity aS in new[] { Continuity.C0, Continuity.C1, Continuity.C2 })
        {
            int aNb = aLaw.NbIntervals(aS);
            Assert.True(aNb >= 1);
            NCollection_Array1<double> aT = new(1, aNb + 1);
            aLaw.Intervals(aT, aS);
            Assert.Equal(0.0, aT[1]);
            Assert.Equal(2.0, aT[aNb + 1]);
            for (int i = 1; i <= aNb; i++)
            {
                Assert.True(aT[i] < aT[i + 1]);
            }
        }
    }

    [Fact]
    public void SetInterval_KeepsTheValues()
    {
        CorrectedFrenet aLaw = Law(Twisted());
        Frame aWhole = D0(aLaw, 0.9);
        aLaw.SetInterval(0.5, 1.5);
        aLaw.GetInterval(out double aF, out double aL);
        Assert.Equal(0.5, aF);
        Assert.Equal(1.5, aL);
        Frame aTrimmed = D0(aLaw, 0.9);
        AssertNear(aWhole.N, aTrimmed.N, 1.0e-9, "normal");
    }

    [Fact]
    public void Copy_EvaluatesTheSameAndIsIndependent()
    {
        CorrectedFrenet aLaw = Law(Twisted());
        var aCopy = (CorrectedFrenet)aLaw.Copy();
        Assert.Equal(D0(aLaw, 1.3), D0(aCopy, 1.3));
        aCopy.SetCurve(new CurveAdaptor(Helix()));
        Assert.Equal(D0(Law(Twisted()), 1.3), D0(aLaw, 1.3));
    }

    [Fact]
    public void AverageLaw_IsAFrame()
    {
        CorrectedFrenet aLaw = Law(Helix());
        Vector3D aT = new(), aN = new(), aB = new();
        aLaw.GetAverageLaw(ref aT, ref aN, ref aB);
        Assert.True(aT.Magnitude() > 0.1);
        AssertNear(aT.Crossed(aN), aB, 1.0e-12, "B = T ^ N");
    }

    [Fact]
    public void ForEvaluation_SkipsThePlaneSearch()
    {
        // On a planar curve the plane search makes the angle law constant on each span; for evaluation it is
        // interpolated, which here gives the same frames to within the interpolation.
        BSplineCurve3D aS = BSpline(new Point3D(0, 0, 0), new Point3D(1, 2, 0), new Point3D(2, -2, 0), new Point3D(3, 0, 0));
        CorrectedFrenet aSweep = Law(aS), anEval = Law(aS, true);
        for (double u = 0.1; u < 0.95; u += 0.2)
        {
            AssertNear(D0(aSweep, u).N, D0(anEval, u).N, 1.0e-6, $"normal at {u}");
        }
    }

    [Fact]
    public void FrozenSpine_IsNotChanged()
    {
        BSplineCurve3D aC = Twisted();
        aC.Freeze();
        CorrectedFrenet aLaw = Law(aC);
        AssertOrthonormal(D0(aLaw, 1.1), 1.0e-9);
    }

    // ------------------------------------------------------------------------------------------------------------
    // GeomFill_LocationGuide with the guided trihedrons

    [Fact]
    public void LocationGuide_GuideTrihedronAC_ParallelLines()
    {
        // path: the Z axis; guide: the parallel line through (2, 0); the normal points at the guide
        // (the guide is matched by curvilinear abscissa over a range LocationGuide extends by a hundredth, so the
        // frame is close to, not exactly, the path's)
        var aGuide = new CurveAdaptor(new LineCurve3D(new Point3D(2, 0, 0), new Direction3D(0, 0, 1)), 0, 4);
        var aTri = new GuideTrihedronAC(aGuide);
        var aLaw = new LocationGuide(aTri);
        Assert.True(aLaw.SetCurve(new CurveAdaptor(new LineCurve3D(Constants.OZ()), 0, 4)));
        Matrix3x3 M = new();
        Vector3D V = new();
        Assert.True(aLaw.D0(1.5, ref M, ref V));
        // the columns are (N, B, T)
        AssertNear(new Vector3D(0, 0, 1.5), V, 1.0e-12, "V");
        AssertNear(new Vector3D(0, 0, 1), new Vector3D(M.Column(3)), 1.0e-2, "T");
        Assert.Equal(1.0, Math.Abs(M.Column(1).X()), 1.0e-3);
        Point3D aOnGuide = aTri.CurrentPointOnGuide();
        Assert.Equal(2.0, aOnGuide.X(), 1.0e-9);
        Assert.Equal(1.5, aOnGuide.Z(), 2.0e-2);
    }

    [Fact]
    public void LocationGuide_GuideTrihedronPlan_ParallelLines()
    {
        var aGuide = new CurveAdaptor(new LineCurve3D(new Point3D(0, 2, 0), new Direction3D(0, 0, 1)), -1, 5);
        var aTri = new GuideTrihedronPlan(aGuide);
        var aLaw = new LocationGuide(aTri);
        Assert.True(aLaw.SetCurve(new CurveAdaptor(new LineCurve3D(Constants.OZ()), 0, 4)));
        Matrix3x3 M = new();
        Vector3D V = new();
        Assert.True(aLaw.D0(2.5, ref M, ref V));
        AssertNear(new Vector3D(0, 0, 2.5), V, 1.0e-12, "V");
        AssertNear(new Vector3D(0, 0, 1), new Vector3D(M.Column(3)), 1.0e-9, "T");
        Assert.Equal(1.0, Math.Abs(M.Column(1).Y()), 1.0e-9);
        Assert.Equal(SurfacePipeError.PipeOk, aLaw.ErrorStatus());
    }

    /// <summary>
    /// Offsets of B-splines: the step of InitInterval's sampling, |S(t)|/2 / |S'(t)| with S = C' ^ C'', fell very low
    /// on them and OCCT had no lower bound, so one SetCurve took hundreds of thousands of Frenet evaluations, seconds
    /// to minutes (occt-issues.md, unit 5L1's GeomFill_CorrectedFrenet row; Kerf unit 10K1, decision 1440). On these
    /// twelve offsets the port kept 16,709 to 32,205 samples of the six turned ones and took up to 4.7 s each before the
    /// bound (Release, alone), 6,910 to 9,782 samples and at most 0.4 s after. The law still gives orthonormal frames.
    /// </summary>
    [Fact]
    public void Offsets_of_BSplines_set_quickly()
    {
        var aRnd = new Random(7);
        for (int k = 0; k < 12; k++)
        {
            int n = 5 + aRnd.Next(4);
            Point3D[] aP = new Point3D[n];
            for (int i = 0; i < n; i++)
            {
                aP[i] = new Point3D(i * 10 + aRnd.NextDouble() * 5, aRnd.NextDouble() * 10, aRnd.NextDouble() * (k % 2 == 0 ? 0 : 10));
            }
            var anOffset = new OffsetCurve3D(BSpline(aP), 1 + aRnd.NextDouble() * 3, new Direction3D(0, 0, 1), true);
            CorrectedFrenet aLaw = HangGuard.Within(TimeSpan.FromSeconds(60), () => Law(anOffset));
            Assert.True(aLaw.SampleCount < 12_000, $"offset {k}: {aLaw.SampleCount} samples");
            foreach (double t in new[] { 0.13, 0.5, 0.77 })
            {
                double u = anOffset.FirstParameter() + t * (anOffset.LastParameter() - anOffset.FirstParameter());
                AssertOrthonormal(D0(aLaw, u), 1.0e-9);
            }
        }
    }
}
