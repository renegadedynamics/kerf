// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation.Advanced;

namespace Kerf.Kernel.ModelingData.Tests;

// AdvApp2Var_MathBase on known values. OCCT has no tests of this code; the expected values marked "native OCCT" were
// computed by OCCT's own C++ compiled as a native library (decision 116), the others by hand. Arrays are Fortran
// arrays: column-major, so COURBE(NDIMAX, NCOEFF) is stored courbe[nd + k * ndimax].
public class AdvApp2Var_MathBaseTests
{
    [Fact]
    public void Precision()
    {
        double anEps = 0;
        MathBase.mmeps1_(ref anEps);
        Assert.Equal(1.0e-9, anEps);
        MathBase.mmveps3_(ref anEps);
        Assert.Equal(1.0e-9, anEps);
    }

    [Fact]
    public void Norms()
    {
        Assert.Equal(5.0, MathBase.mzsnorm_(2, new double[] { 3, 4 }));
        // the large-component path scales by the largest component, avoiding overflow
        Assert.Equal(5.0e200, MathBase.mzsnorm_(3, new double[] { 3e200, 0, -4e200 }), 1e186);
        Assert.Equal(32.0, MathBase.msc_(3, new double[] { 1, 2, 3 }, new double[] { 4, 5, 6 }));
        Assert.Equal(0.125, MathBase.pow__di(2.0, -3));
        Assert.Equal(-27.0, MathBase.pow__di(-3.0, 3));
        Assert.Equal(1.0, MathBase.pow__di(5.0, 0));
    }

    [Fact]
    public void Mdsptpt_DistanceOfLargeDimension()
    {
        var aP1 = new double[120];
        var aP2 = new double[120];
        aP2[7] = 3;
        aP2[100] = 4;
        double aDist = 0;
        MathBase.mdsptpt_(120, aP1, aP2, ref aDist);
        Assert.Equal(5.0, aDist);
    }

    [Fact]
    public void Mdsptpt_SmallDimension_FailsAsOcct()
    {
        // OCCT: the work array is a null pointer that is used unallocated when NDIMEN <= 100 (occt-issues.md)
        double aDist = 0;
        Assert.Throws<IndexOutOfRangeException>(() =>
            MathBase.mdsptpt_(3, new double[] { 0, 0, 0 }, new double[] { 1, 2, 2 }, ref aDist));
    }

    [Fact]
    public void Mmunivt_Mmvncol()
    {
        var aNorm = new double[2];
        int anErr = -1;
        MathBase.mmunivt_(2, new double[] { 3, 4 }, aNorm, 1e-9, ref anErr);
        Assert.Equal(0, anErr);
        Assert.Equal(0.6, aNorm[0], 1e-15);
        Assert.Equal(0.8, aNorm[1], 1e-15);
        MathBase.mmunivt_(2, new double[] { 0, 0 }, aNorm, 1e-9, ref anErr);
        Assert.NotEqual(0, anErr);

        var aVec = new double[3];
        MathBase.mmvncol_(3, new double[] { 1, 2, 3 }, aVec, ref anErr);
        Assert.Equal(0, anErr);
        // not colinear: a non-zero cross product
        double aCross = Math.Abs(2 * aVec[2] - 3 * aVec[1]) + Math.Abs(3 * aVec[0] - aVec[2]) + Math.Abs(aVec[1] - 2 * aVec[0]);
        Assert.True(aCross > 1e-6);
    }

    [Theory]
    [InlineData(2, new[] { 0.57735026918962576 })]
    [InlineData(3, new[] { 0.77459666924148338 })]
    [InlineData(4, new[] { 0.33998104358485626, 0.86113631159405258 })]
    [InlineData(5, new[] { 0.53846931010568309, 0.90617984593866399 })]
    public void Mmrtptt_PositiveLegendreRoots(int theDegree, double[] theRoots)
    {
        var aRoots = new double[theRoots.Length];
        MathBase.mmrtptt_(theDegree, aRoots);
        for (int i = 0; i < theRoots.Length; i++)
        {
            Assert.Equal(theRoots[i], aRoots[i], 1e-15);
        }
    }

    [Fact]
    public void Mmpocrb_EvaluatesAPolynomialCurve()
    {
        // COURBE(2, 3): x = 1 + 2t + 3t^2, y = t
        double[] aCurve = [1, 0, 2, 1, 3, 0];
        var aPnt = new double[2];
        MathBase.mmpocrb_(2, 3, aCurve, 2, 0.5, aPnt);
        Assert.Equal(2.75, aPnt[0]);
        Assert.Equal(0.5, aPnt[1]);
    }

    [Fact]
    public void Mmcglc1_LengthOfASegment()
    {
        // P(t) = (3t, 4t) on [0, 2]
        double[] aCurve = [0, 0, 3, 4];
        double aLength = 0, anError = 0;
        int anErr = -1;
        MathBase.mmcglc1_(2, 2, 2, aCurve, 0.0, 2.0, 1e-10, ref aLength, ref anError, ref anErr);
        Assert.Equal(0, anErr);
        Assert.Equal(10.0, aLength, 1e-12);
    }

    [Fact]
    public void Mmcglc1_LengthOfACubic()
    {
        // native OCCT
        double[] aCurve = [1, 0, -1, 2, 0.5, 0, 2, 1];
        double aLength = 0, anError = 0;
        int anErr = -1;
        MathBase.mmcglc1_(2, 2, 4, aCurve, -1.0, 1.0, 1e-10, ref aLength, ref anError, ref anErr);
        Assert.Equal(0, anErr);
        Assert.Equal(6.982108086557634, aLength);
        Assert.Equal(8.561329423173447E-11, anError);
    }

    [Fact]
    public void Mmpobas_HermiteJacobiBasis()
    {
        // native OCCT: VALBAS(8, 0:2) at t = 0.3 for the order 1 basis
        double[] anExpected =
        [
            0.28175, 0.15925, 0.71825, -0.29575, 0.9185823128221486, 0.913977861206381, -0.0074058473690003694,
            -0.8362897430038388, -0.6825, -0.33249999999999996, 0.6825, -0.0325, -1.2113173355896465,
            1.841347339426775, 4.89762521611475, 1.8796808431293868, 0.44999999999999996, -0.04999999999999999,
            -0.44999999999999996, 0.9500000000000001, -3.23905368124704, -11.2577879293625, 3.427920899091703,
            36.54509739857849,
        ];
        var aValues = new double[24];
        int anErr = -1;
        MathBase.mmpobas_(0.3, 1, 8, 2, aValues, ref anErr);
        Assert.Equal(0, anErr);
        Assert.Equal(anExpected, aValues);

        // order -1: Legendre, normalised (P0 = 1/sqrt(2))
        aValues = new double[12];
        MathBase.mmpobas_(-0.7, -1, 6, 1, aValues, ref anErr);
        Assert.Equal(0, anErr);
        Assert.Equal(
            [
                0.7071067811865476, -0.8573214099741122, 0.3715676250697844, 0.36013452347699204, -0.8741165640680452,
                0.856466986233908, 0, 1.224744871391589, -3.320391543176798, 4.06905240811666, -1.5962935585286284,
                -3.596522859337112,
            ],
            aValues);
    }

    [Fact]
    public void Mmpobas_DerivativesMatchFiniteDifferences()
    {
        const double aStep = 1e-6;
        var aValues = new double[10 * 2];
        var aPlus = new double[10 * 2];
        var aMinus = new double[10 * 2];
        int anErr = 0;
        for (int anOrder = -1; anOrder <= 2; anOrder++)
        {
            MathBase.mmpobas_(0.2, anOrder, 10, 1, aValues, ref anErr);
            MathBase.mmpobas_(0.2 + aStep, anOrder, 10, 0, aPlus, ref anErr);
            MathBase.mmpobas_(0.2 - aStep, anOrder, 10, 0, aMinus, ref anErr);
            for (int j = 0; j < 10; j++)
            {
                Assert.Equal(aValues[10 + j], (aPlus[j] - aMinus[j]) / (2 * aStep), 1e-6);
            }
        }
    }

    [Fact]
    public void Mmpobas_IsSafeAcrossThreads()
    {
        // mmpobas_ goes through mmherm1_ and OCCT's static Hermite cache, per thread in the port (decision 113)
        var aReference = new double[12 * 4];
        int anErr = 0;
        MathBase.mmpobas_(0.4, 2, 12, 3, aReference, ref anErr);
        Parallel.For(0, 64, i =>
        {
            var aValues = new double[12 * 4];
            int anErrLocal = 0;
            MathBase.mmpobas_(0.4, 2, 12, 3, aValues, ref anErrLocal);
            Assert.Equal(aReference, aValues);
        });
    }

    [Fact]
    public void Mmjacan_Mmjaccv()
    {
        // native OCCT
        double[] aJacobi = [0.5, -0.25, 0.125, 1, -1, 0.3, 0.2, 0.1, 0, 0, 0, 0, 0, 0, 0, 0];
        var aCanonical = new double[16];
        MathBase.mmjacan_(1, 7, aJacobi, aCanonical);
        Assert.Equal(
            [
                -0.25565988141198936, -0.816049216840482, 11.098817777436231, 2.3234586147546983, -20.430655910636496,
                -4.59876957898795, 9.837498014612253, 2.391360181073734, 0, 0, 0, 0, 0, 0, 0, 0,
            ],
            aCanonical);

        double[] aLegendre = [1, 0.5, -0.5, 0.25, 0.1, -0.1, 0.05, 0.02, 0.01, -0.01, 0.001, 0.002];
        var anAux = new double[64];
        var aCurve = new double[12];
        MathBase.mmjaccv_(6, 2, 1, aLegendre, anAux, aCurve);
        Assert.Equal(
            [
                1.1109264959331178, 0.13209843368096408, -0.7218529918662356, 0.9858031326380718, 0.11092649593311782,
                -0.3679015663190359, 0.05110926495933118, 0.02735803132638072, 0.007781470081337644,
                -0.02471606265276144, 0.001109264959331178, 0.007358031326380719,
            ],
            aCurve);
    }

    [Fact]
    public void Mmtrpjj_Truncation()
    {
        // native OCCT
        var aCurve = new double[20];
        for (int i = 0; i < 20; i++)
        {
            aCurve[i] = Math.Pow(0.3, i % 10) * ((i % 3) - 1);
        }
        var aMax = new double[2];
        double anError = 0;
        int aNbNew = 0;
        MathBase.mmtrpjj_(10, 2, 10, 1e-4, 1, aCurve, aMax, ref anError, ref aNbNew);
        Assert.Equal(9, aNbNew);
        Assert.Equal(2.4959347634884057E-05, anError);
        Assert.Equal([0.00010431374433372343, 7.935439669883937E-05], aMax);
    }

    [Fact]
    public void Mmarcin_Reparametrisation()
    {
        // C2(v) on [u0, u1] equal to C1(u) on [-1, 1], u = (2v - u0 - u1) / (u1 - u0)
        // CRVOLD(NCOEFF, NDIM) (by dimension), CRVNEW(NDIMAX, NCOEFF) (by coefficient)
        double[] anOld = [1, -2, 0.5, 3, 0, 1, -1, 2]; // x = 1 - 2u + 0.5u^2 + 3u^3, y = u - u^2 + 2u^3
        var aNew = new double[8];
        int anErr = -1;
        MathBase.mmarcin_(2, 2, 4, anOld, 0.2, 0.8, aNew, ref anErr);
        Assert.Equal(0, anErr);
        foreach (double v in new[] { 0.2, 0.35, 0.5, 0.8 })
        {
            double u = (2 * v - 0.2 - 0.8) / (0.8 - 0.2);
            for (int nd = 0; nd < 2; nd++)
            {
                double anExpected = 0, aValue = 0;
                for (int k = 3; k >= 0; k--)
                {
                    anExpected = anExpected * u + anOld[k + nd * 4];
                    aValue = aValue * v + aNew[nd + k * 2];
                }
                Assert.Equal(anExpected, aValue, 1e-12);
            }
        }
        // a null interval
        MathBase.mmarcin_(2, 2, 4, anOld, 0.5, 0.5, aNew, ref anErr);
        Assert.Equal(13, anErr);
    }

    [Fact]
    public void Mmcdriv_Mmdrvck()
    {
        // COURBE(NDIMEN, NCOEFF): x = 1 + t + t^2 + t^3
        double[] aCurve = [1, 1, 1, 1];
        var aDeriv = new double[4];
        int aNbCoef = 0;
        MathBase.mmcdriv_(1, 4, aCurve, 1, ref aNbCoef, aDeriv);
        Assert.Equal(3, aNbCoef);
        Assert.Equal([1.0, 2, 3], aDeriv.AsSpan(0, 3).ToArray());
        // the second derivative at t = 2: 2 + 6t = 14
        var aPnt = new double[1];
        MathBase.mmdrvck_(4, 1, aCurve, 2, 2.0, aPnt);
        Assert.Equal(14.0, aPnt[0]);
    }

    [Fact]
    public void Mmmrslwd_SolvesALinearSystem()
    {
        // AMAT(3, 3) column-major, two right-hand sides
        double[] anA = [4, 1, 2, 1, 5, 1, 2, 1, 6];
        double[] aB = [7, 7, 9, 1, 0, 0];
        var anAux = new double[(3 + 2) * 3];
        var aX = new double[6];
        int anErr = -1;
        MathBase.mmmrslwd_(3, 3, 2, anA, aB, 1e-12, anAux, aX, ref anErr);
        Assert.Equal(0, anErr);
        for (int c = 0; c < 2; c++)
        {
            for (int i = 0; i < 3; i++)
            {
                double aSum = 0;
                for (int j = 0; j < 3; j++)
                {
                    aSum += anA[i + j * 3] * aX[j + c * 3];
                }
                Assert.Equal(aB[i + c * 3], aSum, 1e-13);
            }
        }
        Assert.Equal(1.0, aX[0], 1e-14);
        Assert.Equal(1.0, aX[1], 1e-14);
        Assert.Equal(1.0, aX[2], 1e-14);

        // a singular matrix
        MathBase.mmmrslwd_(2, 2, 1, new double[] { 1, 2, 2, 4 }, new double[] { 1, 1 }, 1e-12,
                                      new double[6], new double[2], ref anErr);
        Assert.Equal(1, anErr);
    }

    [Fact]
    public void Mmfmtb1_Transposes()
    {
        // TABLE1(MAXSZ1 = 3, 2) with 2 x 2 used -> TABLE2(MAXSZ2 = 2, 2)
        double[] aTab1 = [1, 2, 99, 3, 4, 99];
        var aTab2 = new double[4];
        int anISize = 0, aJSize = 0, anErr = -1;
        MathBase.mmfmtb1_(3, aTab1, 2, 2, 2, aTab2, ref anISize, ref aJSize, ref anErr);
        Assert.Equal(0, anErr);
        Assert.Equal(2, anISize);
        Assert.Equal(2, aJSize);
        Assert.Equal([1.0, 3, 2, 4], aTab2);
        // a table too small
        MathBase.mmfmtb1_(3, aTab1, 2, 3, 2, aTab2, ref anISize, ref aJSize, ref anErr);
        Assert.Equal(1, anErr);
    }

    [Fact]
    public void Sorts()
    {
        // DTAB(IS = 2, N = 5) and DTABTR(NBLIGN = 2, NBCOLN = 5): columns sorted on their row ICLE = 2
        double[] aTab = [10, 5, 20, 3, 30, 4, 40, 1, 50, 2];
        MathBase.mvsheld_(5, 2, aTab, 2);
        Assert.Equal([40.0, 1, 50, 2, 20, 3, 30, 4, 10, 5], aTab);
        aTab = [10, 5, 20, 3, 30, 4, 40, 1, 50, 2];
        MathBase.mmbulld_(5, 2, aTab, 2);
        Assert.Equal([40.0, 1, 50, 2, 20, 3, 30, 4, 10, 5], aTab);
    }

    [Theory]
    [InlineData(1.5, 0, 2, 0)]
    [InlineData(0.25, 0, 1, 0)]
    [InlineData(-0.5, 2, 0, 0)]
    [InlineData(3.5, 2, 0, 0)]
    public void Mmsrre2_FindsTheInterval(double theParam, int theErr, int theInterval, int theType)
    {
        int anInterval = 0, aType = 0, anErr = -1;
        MathBase.mmsrre2_(theParam, 4, new double[] { 0, 1, 2, 3 }, 1e-9, ref anInterval, ref aType, ref anErr);
        Assert.Equal(theErr, anErr);
        if (theErr == 0)
        {
            Assert.Equal(theInterval, anInterval);
            Assert.Equal(theType, aType);
        }
    }

    [Fact]
    public void Mmposui_Profile()
    {
        // A 3 x 3 symmetric profile matrix, full lower triangle stored by lines:
        // line 1: (1,1) at 1; line 2: (2,1), (2,2) at 2, 3; line 3: (3,1), (3,2), (3,3) at 4, 5, 6.
        // APOSIT(1, I): number of terms - 1 of line I; APOSIT(2, I): index of its diagonal term.
        int[] aPosit = [0, 1, 1, 3, 2, 6];
        var aNext = new int[6];
        int anErr = -1;
        MathBase.mmposui_(3, 6, aPosit, aNext, ref anErr);
        Assert.Equal(0, anErr);
        // POSUIV(K): the next line below with a term in the same column, or -1
        Assert.Equal([2, 3, 3, -1, -1, -1], aNext);
    }
}
