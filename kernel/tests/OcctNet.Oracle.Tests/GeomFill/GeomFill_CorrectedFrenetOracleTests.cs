// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// GeomFill_CorrectedFrenet against native OCCT (<see cref="TrihedronOracle"/>): random spines of every kind, for
/// sweeping and for evaluation, and twisted and planar B-splines (where the angle law is built), with
/// EvaluateBestMode.
/// </summary>
/// <remarks>
/// Offset spines (kind 8) are left out of the random runs: on them CorrectedFrenet's angle-law sampling (steps of
/// |SnglrFunc| / |SnglrFunc'|) takes steps near 1e-6, so a single SetCurve takes seconds to many minutes natively
/// (occt-issues.md). <see cref="OffsetSpines_Short"/> compares a few offsets of circles, which stay fast.
/// </remarks>
public class GeomFill_CorrectedFrenetOracleTests(ITestOutputHelper output)
{
    private static readonly int[] SlowKinds = [8];

    [Fact]
    public void RandomSpines()
    {
        TrihedronOracle.Run(output, "GeomFill_CorrectedFrenet", () => new N.GeomFill_CorrectedFrenet(),
                            () => new GeomFill_CorrectedFrenet(), 40, 20261101, SlowKinds);
    }

    [Fact]
    public void RandomSpines_ForEvaluation()
    {
        TrihedronOracle.Run(output, "GeomFill_CorrectedFrenet(true)", () => new N.GeomFill_CorrectedFrenet(true),
                            () => new GeomFill_CorrectedFrenet(true), 20, 20261102, SlowKinds);
    }

    /// <summary>Offsets of circles (conics' offsets are B-spline-like for the adaptor, so the angle law is built).</summary>
    [Fact]
    public void OffsetSpines_Short()
    {
        var anOracle = new Oracle(output, 20261104);
        for (int i = 0; i < 10; i++)
        {
            (N.gp_Ax2 aNAx2, gp_Ax2 anAx2) = GeomOracleInputs.Ax2sFromDirection(anOracle);
            using var aNAx2Owner = aNAx2;
            double aR = anOracle.Real(1, 10), anOffset = anOracle.Real(0.1, 0.9) * aR * (anOracle.Bool() ? 1 : -1);
            (N.gp_Dir aNDir, gp_Dir aDir) = GeomOracleInputs.Dirs(anOracle);
            using var aNDirOwner = aNDir;
            using var aNBasis = new N.Geom_Circle(aNAx2, aR);
            using var aNC = new N.Geom_OffsetCurve(aNBasis, anOffset, aNDir, true);
            var aPC = new Geom_OffsetCurve(new Geom_Circle(anAx2, aR), anOffset, aDir, true);
            double aF = anOracle.Real(0, 1), aL = aF + anOracle.Real(1, 4);
            using var aNA = new N.GeomAdaptor_Curve(aNC, aF, aL);
            var aPA = new GeomAdaptor_Curve(aPC, aF, aL);
            using var aNLaw = new N.GeomFill_CorrectedFrenet();
            TrihedronOracle.Compare(anOracle, aNLaw, new GeomFill_CorrectedFrenet(), aNA, aPA, $"offset circle #{i}");
        }
        anOracle.Report("GeomFill_CorrectedFrenet on offset circles");
    }

    /// <summary>
    /// B-splines (degree 2 to 5, C0 to C(d-1) knots, rational or not): twisted ones, and planar zigzags with
    /// inflexions (the angle law's flips), each also for evaluation; EvaluateBestMode too.
    /// </summary>
    [Fact]
    public void BSplineSpines_AngleLaw()
    {
        var anOracle = new Oracle(output, 20261103);
        int aNbModes = 0;
        foreach (int i in Tape.Budget(200))
        {
            bool aPlanar = i % 2 == 0;
            int aDeg = anOracle.Int(2, 5);
            int aNbKnots = anOracle.Int(2, 5);
            var aKnots = new NCollection_Array1<double>(1, aNbKnots);
            var aMults = new NCollection_Array1<int>(1, aNbKnots);
            aKnots[1] = anOracle.Real(-2, 2);
            for (int k = 2; k <= aNbKnots; k++)
            {
                aKnots[k] = aKnots[k - 1] + anOracle.Real(0.3, 2);
                aMults[k] = anOracle.Int(1, aDeg);
            }
            aMults[1] = aMults[aNbKnots] = aDeg + 1;
            int aNbPoles = Occt.BSplCLib.NbPoles(aDeg, false, aMults);
            var aPoles = new NCollection_Array1<gp_Pnt>(1, aNbPoles);
            for (int k = 1; k <= aNbPoles; k++)
            {
                double aY = (k % 2 == 0 ? 1 : -1) * anOracle.Real(0.5, 3);
                aPoles[k] = aPlanar ? new gp_Pnt(k * 2.0 + anOracle.Real(-0.3, 0.3), aY, 0)
                                    : new gp_Pnt(anOracle.Real(-5, 5), anOracle.Real(-5, 5), anOracle.Real(-5, 5));
            }
            NCollection_Array1<double>? aWeights = anOracle.Bool() ? null : anOracle.Weights(aNbPoles, true);
            using N.Geom_BSplineCurve aNC = Oracle.ToNativeCurve(aPoles, aWeights, aKnots, aMults, aDeg, false);
            Geom_BSplineCurve aPC = aWeights is null ? new Geom_BSplineCurve(aPoles, aKnots, aMults, aDeg)
                                                     : new Geom_BSplineCurve(aPoles, aWeights, aKnots, aMults, aDeg);
            foreach (bool aForEval in new[] { false, true })
            {
                using var aNA = new N.GeomAdaptor_Curve(aNC);
                var aPA = new GeomAdaptor_Curve(aPC);
                using var aNLaw = new N.GeomFill_CorrectedFrenet(aForEval);
                var aPLaw = new GeomFill_CorrectedFrenet(aForEval);
                string aWhat = $"{(aPlanar ? "planar" : "twisted")} #{i} {(aForEval ? "evaluation" : "sweep")}";
                TrihedronOracle.Compare(anOracle, aNLaw, aPLaw, aNA, aPA, aWhat);

                // EvaluateBestMode on a law over the whole curve
                using var aNA2 = new N.GeomAdaptor_Curve(aNC);
                var aPA2 = new GeomAdaptor_Curve(aPC);
                using var aNLaw2 = new N.GeomFill_CorrectedFrenet(aForEval);
                var aPLaw2 = new GeomFill_CorrectedFrenet(aForEval);
                aNLaw2.SetCurve(aNA2);
                aPLaw2.SetCurve(aPA2);
                N.GeomFill_Trihedron aNMode = aNLaw2.EvaluateBestMode();
                GeomFill_Trihedron aPMode = aPLaw2.EvaluateBestMode();
                Assert.True((int)aNMode == (int)aPMode, $"{aWhat} EvaluateBestMode: native {aNMode}, port {aPMode}");
                if (aPMode != GeomFill_Trihedron.GeomFill_IsFrenet)
                {
                    aNbModes++;
                }
            }
        }
        output.WriteLine($"{aNbModes} of 400 laws built an angle law");
        Assert.True(aNbModes > 100);
        anOracle.Report("GeomFill_CorrectedFrenet on B-splines");
    }
}
