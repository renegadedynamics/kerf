// Port of: src/FoundationClasses/TKMath/GTests/MathLin_EigenSearch_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;
using static Kerf.Kernel.Foundation.Tests.MathLin_EigenSearch_Test_Helpers;

namespace Kerf.Kernel.Foundation.Tests;

internal static class MathLin_EigenSearch_Test_Helpers
{
    internal const double THE_EIGEN_TOL = 1.0e-10;
    internal const double THE_RESIDUAL_TOL = 1.0e-10;
    internal const double THE_ORTHOGONAL_TOL = 1.0e-10;
    internal const double THE_NORMALIZED_TOL = 1.0e-10;
    internal const int THE_RANDOM_NB_CASES = 120;

    internal static Matrix BuildSymmetricTridiagonal(math_Vector theDiag, math_Vector theSubdiag)
    {
        int aN = theDiag.Length();
        Matrix aM = new(1, aN, 1, aN, 0.0);
        for (int i = 1; i <= aN; ++i)
        {
            aM[i, i] = theDiag[theDiag.Lower() + i - 1];
        }
        for (int i = 2; i <= aN; ++i)
        {
            double aE = theSubdiag[theSubdiag.Lower() + i - 1];
            aM[i, i - 1] = aE;
            aM[i - 1, i] = aE;
        }
        return aM;
    }

    internal static void BuildLegacyArrays(math_Vector theDiag,
                                           math_Vector theSubdiag,
                                           NCollection_Array1<double> theDiagLegacy,
                                           NCollection_Array1<double> theSubdiagLegacy)
    {
        int aN = theDiag.Length();
        for (int i = 1; i <= aN; ++i)
        {
            theDiagLegacy[i] = theDiag[theDiag.Lower() + i - 1];
            theSubdiagLegacy[i] = theSubdiag[theSubdiag.Lower() + i - 1];
        }
    }

    internal static List<double> SortedEigenValuesFromLegacy(EigenValuesSearcher theLegacy)
    {
        int aN = theLegacy.Dimension();
        List<double> aVals = new(aN);
        for (int i = 1; i <= aN; ++i)
        {
            aVals.Add(theLegacy.EigenValue(i));
        }
        aVals.Sort();
        return aVals;
    }

    internal static List<double> SortedEigenValuesFromModern(global::Kerf.Engine.Numerics.Linear.MathLin.EigenResult theModern)
    {
        List<double> aVals = new();
        if (theModern.EigenValues is null)
        {
            return aVals;
        }

        math_Vector aEig = theModern.EigenValues;
        for (int i = aEig.Lower(); i <= aEig.Upper(); ++i)
        {
            aVals.Add(aEig[i]);
        }
        aVals.Sort();
        return aVals;
    }

    internal static double VectorNorm2(math_Vector theVec)
    {
        double aNorm2 = 0.0;
        for (int i = theVec.Lower(); i <= theVec.Upper(); ++i)
        {
            aNorm2 += theVec[i] * theVec[i];
        }
        return Math.Sqrt(aNorm2);
    }

    internal static double PairResidualInfinity(Matrix theMatrix, double theLambda, math_Vector theVector)
    {
        int aN = theMatrix.RowNumber();
        double aMax = 0.0;
        for (int i = 1; i <= aN; ++i)
        {
            double aAx = 0.0;
            for (int j = 1; j <= aN; ++j)
            {
                aAx += theMatrix[i, j] * theVector[j];
            }
            double aRes = Math.Abs(aAx - theLambda * theVector[i]);
            if (aRes > aMax)
            {
                aMax = aRes;
            }
        }
        return aMax;
    }

    internal static double DotProduct(math_Vector theV1, math_Vector theV2)
    {
        int aN = theV1.Length();
        double aDot = 0.0;
        for (int i = 1; i <= aN; ++i)
        {
            aDot += theV1[i] * theV2[i];
        }
        return aDot;
    }

    /// <summary>
    /// C++'s std::mt19937 (the 32-bit Mersenne Twister, same sequence). The test's std::uniform_int_distribution and
    /// std::uniform_real_distribution are library-specific, so they are replaced by simple mappings: the cases
    /// differ from OCCT's, which only needs random symmetric tridiagonal matrices.
    /// </summary>
    internal sealed class Mt19937
    {
        private readonly uint[] myState = new uint[624];
        private int myIndex;

        public Mt19937(uint theSeed)
        {
            myState[0] = theSeed;
            for (int i = 1; i < 624; ++i)
            {
                myState[i] = unchecked(1812433253u * (myState[i - 1] ^ (myState[i - 1] >> 30)) + (uint)i);
            }
            myIndex = 624;
        }

        public uint Next()
        {
            if (myIndex >= 624)
            {
                for (int i = 0; i < 624; ++i)
                {
                    uint y = (myState[i] & 0x80000000u) | (myState[(i + 1) % 624] & 0x7fffffffu);
                    myState[i] = myState[(i + 397) % 624] ^ (y >> 1) ^ ((y & 1u) != 0 ? 0x9908b0dfu : 0u);
                }
                myIndex = 0;
            }
            uint z = myState[myIndex++];
            z ^= z >> 11;
            z ^= (z << 7) & 0x9d2c5680u;
            z ^= (z << 15) & 0xefc60000u;
            z ^= z >> 18;
            return z;
        }

        public int NextInt(int theMin, int theMax) => theMin + (int)(Next() % (uint)(theMax - theMin + 1));

        public double NextReal(double theMin, double theMax) => theMin + (theMax - theMin) * (Next() / 4294967296.0);
    }
}

public class MathLin_EigenSearch_Test
{
    [Fact]
    public void BasicParityWithLegacy_3x3()
    {
        math_Vector aDiag = new(1, 3);
        math_Vector aSubdiag = new(1, 3);
        aDiag[1]    = 4.0;
        aDiag[2]    = 4.0;
        aDiag[3]    = 4.0;
        aSubdiag[1] = 0.0;
        aSubdiag[2] = 1.0;
        aSubdiag[3] = 1.0;

        NCollection_Array1<double> aDiagLegacy = new(1, 3);
        NCollection_Array1<double> aSubdiagLegacy = new(1, 3);
        BuildLegacyArrays(aDiag, aSubdiag, aDiagLegacy, aSubdiagLegacy);

        EigenValuesSearcher aLegacy = new(aDiagLegacy, aSubdiagLegacy);
        global::Kerf.Engine.Numerics.Linear.MathLin.EigenResult     aModern = global::Kerf.Engine.Numerics.Linear.MathLin.EigenTridiagonal(aDiag, aSubdiag);

        Assert.True(aLegacy.IsDone);
        Assert.True(aModern.IsDone);
        Assert.True(aModern.EigenValues is not null);
        Assert.True(aModern.EigenVectors is not null);

        List<double> aLegacySorted = SortedEigenValuesFromLegacy(aLegacy);
        List<double> aModernSorted = SortedEigenValuesFromModern(aModern);
        Assert.Equal(aModernSorted.Count, aLegacySorted.Count);
        for (int i = 0; i < aLegacySorted.Count; ++i)
        {
            Assert.Equal(aModernSorted[i], aLegacySorted[i], THE_EIGEN_TOL);
        }

        Matrix  aA       = BuildSymmetricTridiagonal(aDiag, aSubdiag);
        math_Vector aEigVals = aModern.EigenValues!;
        for (int i = 1; i <= 3; ++i)
        {
            math_Vector aVec = global::Kerf.Engine.Numerics.Linear.MathLin.GetEigenVector(aModern, i);
            Assert.Equal(1.0, VectorNorm2(aVec), THE_NORMALIZED_TOL);
            Assert.Equal(0.0, PairResidualInfinity(aA, aEigVals[i], aVec), THE_RESIDUAL_TOL);
        }
    }

    [Fact]
    public void HandlesNonOneLowerBounds()
    {
        math_Vector aDiag = new(-2, 2);
        math_Vector aSubdiag = new(-2, 2);
        aDiag[-2]    = 1.0;
        aDiag[-1]    = 3.0;
        aDiag[0]     = -2.0;
        aDiag[1]     = 5.0;
        aDiag[2]     = 4.0;
        aSubdiag[-2] = 0.0;
        aSubdiag[-1] = 0.3;
        aSubdiag[0]  = -0.2;
        aSubdiag[1]  = 0.7;
        aSubdiag[2]  = -1.1;

        NCollection_Array1<double> aDiagLegacy = new(-2, 2);
        NCollection_Array1<double> aSubdiagLegacy = new(-2, 2);
        for (int i = -2; i <= 2; ++i)
        {
            aDiagLegacy[i]    = aDiag[i];
            aSubdiagLegacy[i] = aSubdiag[i];
        }

        EigenValuesSearcher aLegacy = new(aDiagLegacy, aSubdiagLegacy);
        global::Kerf.Engine.Numerics.Linear.MathLin.EigenResult     aModern = global::Kerf.Engine.Numerics.Linear.MathLin.EigenTridiagonal(aDiag, aSubdiag);

        Assert.Equal(aModern.IsDone, aLegacy.IsDone);
        Assert.True(aLegacy.IsDone);

        List<double> aLegacySorted = SortedEigenValuesFromLegacy(aLegacy);
        List<double> aModernSorted = SortedEigenValuesFromModern(aModern);
        Assert.Equal(aModernSorted.Count, aLegacySorted.Count);
        for (int i = 0; i < aLegacySorted.Count; ++i)
        {
            Assert.Equal(aModernSorted[i], aLegacySorted[i], THE_EIGEN_TOL);
        }
    }

    [Fact]
    public void RandomParityAndOrthogonality()
    {
        Mt19937 aGen = new(123456);

        for (int aCase = 0; aCase < THE_RANDOM_NB_CASES; ++aCase)
        {
            int aN = aGen.NextInt(2, 32);

            math_Vector aDiag = new(1, aN);
            math_Vector aSubdiag = new(1, aN);
            for (int i = 1; i <= aN; ++i)
            {
                aDiag[i]    = aGen.NextReal(-100.0, 100.0);
                aSubdiag[i] = (i == 1) ? 0.0 : aGen.NextReal(-100.0, 100.0);
            }

            NCollection_Array1<double> aDiagLegacy = new(1, aN);
            NCollection_Array1<double> aSubdiagLegacy = new(1, aN);
            BuildLegacyArrays(aDiag, aSubdiag, aDiagLegacy, aSubdiagLegacy);

            EigenValuesSearcher aLegacy = new(aDiagLegacy, aSubdiagLegacy);
            global::Kerf.Engine.Numerics.Linear.MathLin.EigenResult     aModern = global::Kerf.Engine.Numerics.Linear.MathLin.EigenTridiagonal(aDiag, aSubdiag);

            Assert.Equal(aModern.IsDone, aLegacy.IsDone);
            if (!aLegacy.IsDone)
            {
                continue;
            }

            Assert.True(aModern.EigenValues is not null);
            Assert.True(aModern.EigenVectors is not null);

            List<double> aLegacySorted = SortedEigenValuesFromLegacy(aLegacy);
            List<double> aModernSorted = SortedEigenValuesFromModern(aModern);
            Assert.Equal(aModernSorted.Count, aLegacySorted.Count);
            for (int i = 0; i < aLegacySorted.Count; ++i)
            {
                Assert.Equal(aModernSorted[i], aLegacySorted[i], THE_EIGEN_TOL);
            }

            Matrix  aA       = BuildSymmetricTridiagonal(aDiag, aSubdiag);
            math_Vector aEigVals = aModern.EigenValues!;

            for (int i = 1; i <= aN; ++i)
            {
                math_Vector aVec = global::Kerf.Engine.Numerics.Linear.MathLin.GetEigenVector(aModern, i);
                Assert.Equal(1.0, VectorNorm2(aVec), THE_NORMALIZED_TOL);
                Assert.Equal(0.0, PairResidualInfinity(aA, aEigVals[i], aVec), THE_RESIDUAL_TOL);
            }

            for (int i = 1; i <= aN; ++i)
            {
                math_Vector aVecI = global::Kerf.Engine.Numerics.Linear.MathLin.GetEigenVector(aModern, i);
                for (int j = i + 1; j <= aN; ++j)
                {
                    math_Vector aVecJ = global::Kerf.Engine.Numerics.Linear.MathLin.GetEigenVector(aModern, j);
                    Assert.Equal(0.0, DotProduct(aVecI, aVecJ), THE_ORTHOGONAL_TOL);
                }
            }
        }
    }
}
