// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation.FiniteElements;
using Kerf.Engine.Foundation;

using Kerf.Engine.Numerics;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// FEmTool_ProfileMatrix and FEmTool_Assembly against dense solutions by math_Gauss (OCCT has no GTests for
/// FEmTool).
/// </summary>
public class FEmTool_MatrixTests
{
    private static readonly int[] theFirstIndexes = [1, 1, 1, 2, 3, 3, 6];

    /// <summary>A random symmetric positive definite matrix with the profile, as a profile matrix and dense.</summary>
    private static (FemProfileMatrix, Matrix) MakeSpd(int theSeed)
    {
        Random aRandom = new(theSeed);
        int n = theFirstIndexes.Length;
        FemProfileMatrix aProfile = new(NCollection_Array1<int>.From(theFirstIndexes));
        aProfile.Init(0.0);
        Matrix aDense = new(1, n, 1, n, 0.0);
        for (int i = 1; i <= n; i++)
        {
            for (int j = theFirstIndexes[i - 1]; j < i; j++)
            {
                double aV = aRandom.NextDouble() * 2 - 1;
                aDense[i, j] = aDense[j, i] = aV;
                aProfile.ChangeValue(i, j) = aV;
            }
        }
        for (int i = 1; i <= n; i++)
        {
            double aV = n + aRandom.NextDouble();
            aDense[i, i] = aV;
            aProfile.ChangeValue(i, i) = aV;
        }
        return (aProfile, aDense);
    }

    private static math_Vector RandomVector(int theSeed, int theLower, int theUpper)
    {
        Random aRandom = new(theSeed);
        math_Vector aV = new(theLower, theUpper);
        for (int i = theLower; i <= theUpper; i++)
        {
            aV[i] = aRandom.NextDouble() * 10 - 5;
        }
        return aV;
    }

    [Fact]
    public void ProfileMatrix_solves_like_Gauss()
    {
        for (int aSeed = 1; aSeed <= 20; aSeed++)
        {
            (FemProfileMatrix aProfile, Matrix aDense) = MakeSpd(aSeed);
            int n = aProfile.RowNumber();
            Assert.Equal(n, aProfile.ColNumber());

            math_Vector aB = RandomVector(100 + aSeed, 1, n);
            math_Vector aX = new(1, n), anExpected = new(1, n);
            Assert.True(aProfile.Decompose());
            aProfile.Solve(aB, aX);
            Gauss aGauss = new(aDense);
            aGauss.Solve(aB, anExpected);
            for (int i = 1; i <= n; i++)
            {
                Assert.Equal(anExpected[i], aX[i], 1e-12);
            }

            math_Vector aMX = new(1, n);
            aProfile.Multiplied(aX, aMX);
            math_Vector aDenseMX = aDense * aX;
            for (int i = 1; i <= n; i++)
            {
                Assert.Equal(aDenseMX[i], aMX[i], 1e-12);
                Assert.Equal(aB[i], aMX[i], 1e-11);
            }
        }
    }

    [Fact]
    public void ProfileMatrix_profile_and_symmetry()
    {
        (FemProfileMatrix aProfile, Matrix aDense) = MakeSpd(7);
        for (int i = 1; i <= 7; i++)
        {
            for (int j = 1; j <= 7; j++)
            {
                bool anIn = j <= i ? j >= theFirstIndexes[i - 1] : i >= theFirstIndexes[j - 1];
                Assert.Equal(anIn, aProfile.IsInProfile(i, j));
                if (anIn)
                {
                    Assert.Equal(aDense[i, j], aProfile.ChangeValue(i, j));
                    Assert.Equal(aProfile.ChangeValue(i, j), aProfile.ChangeValue(j, i));
                }
                else
                {
                    int ii = i, jj = j;
                    EngineAssert.ThrowsAny<OutOfRangeException>(() => aProfile.ChangeValue(ii, jj));
                }
            }
        }
    }

    [Fact]
    public void ProfileMatrix_failures()
    {
        FemProfileMatrix aProfile = new(NCollection_Array1<int>.From([1, 1, 2]));
        aProfile.Init(0.0);
        aProfile.ChangeValue(1, 1) = 1;
        aProfile.ChangeValue(2, 1) = 2;
        aProfile.ChangeValue(2, 2) = 1;
        aProfile.ChangeValue(3, 3) = 1;
        math_Vector aB = new(1, 3, 1.0), aX = new(1, 3);
        EngineAssert.ThrowsAny<NotDoneException>(() => aProfile.Solve(aB, aX));
        Assert.False(aProfile.Decompose()); // not positive definite
        EngineAssert.ThrowsAny<MethodNotImplementedException>(() => aProfile.Prepare());
        EngineAssert.ThrowsAny<MethodNotImplementedException>(() => aProfile.Solve(aB, aB, aX, aX));

        StringWriter aWriter = new();
        aProfile.OutM(aWriter);
        Assert.Equal("Matrix A\n1 \n2 1 \n0 0 1 \nNextCoeff\n2 0 3 0 0 \n", aWriter.ToString());
        aWriter = new StringWriter();
        aProfile.OutS(aWriter);
        Assert.StartsWith("Matrix S\n", aWriter.ToString());
    }

    // Two elements of three variables each in one dimension, sharing variable 3
    private static NCollection_HArray2<NCollection_HArray1<int>> Table()
    {
        NCollection_HArray2<NCollection_HArray1<int>> aTable = new(1, 1, 1, 2);
        aTable[1, 1] = new NCollection_HArray1<int>(1, 3);
        aTable[1, 2] = new NCollection_HArray1<int>(1, 3);
        for (int i = 1; i <= 3; i++)
        {
            aTable[1, 1][i] = i;
            aTable[1, 2][i] = i + 2;
        }
        return aTable;
    }

    private static Matrix ElementMatrix(int theSeed)
    {
        Random aRandom = new(theSeed);
        Matrix aM = new(1, 3, 1, 3);
        for (int i = 1; i <= 3; i++)
        {
            for (int j = 1; j < i; j++)
            {
                aM[i, j] = aM[j, i] = aRandom.NextDouble() - 0.5;
            }
            aM[i, i] = 3 + aRandom.NextDouble();
        }
        return aM;
    }

    [Fact]
    public void Assembly_solves_the_assembled_system()
    {
        NCollection_Array2<int> aDep = new(1, 1, 1, 1);
        aDep[1, 1] = 1;
        NCollection_HArray2<NCollection_HArray1<int>> aTable = Table();
        Assembly anAssembly = new(aDep, aTable);
        Assert.Equal(5, anAssembly.GlobVarCount);
        Assert.Same(aTable, anAssembly.AssemblyTable());
        anAssembly.GetAssemblyTable(out NCollection_HArray2<NCollection_HArray1<int>> aTable2);
        Assert.Same(aTable, aTable2);

        Matrix aDense = new(1, 5, 1, 5, 0.0);
        math_Vector aB = new(1, 5, 0.0);
        for (int el = 1; el <= 2; el++)
        {
            Matrix aM = ElementMatrix(el);
            math_Vector aV = RandomVector(10 + el, 1, 3);
            anAssembly.AddMatrix(el, 1, 1, aM);
            anAssembly.AddVector(el, 1, aV);
            for (int i = 1; i <= 3; i++)
            {
                aB[aTable[1, el][i]] += aV[i];
                for (int j = 1; j <= 3; j++)
                {
                    aDense[aTable[1, el][i], aTable[1, el][j]] += aM[i, j];
                }
            }
        }

        math_Vector aX = new(1, 5);
        EngineAssert.ThrowsAny<NotDoneException>(() => anAssembly.Solution(aX));
        Assert.True(anAssembly.Solve());
        anAssembly.Solution(aX);
        math_Vector anExpected = new(1, 5);
        new Gauss(aDense).Solve(aB, anExpected);
        for (int i = 1; i <= 5; i++)
        {
            Assert.Equal(anExpected[i], aX[i], 1e-12);
        }

        // With the constraints x1 + 2 x5 = 3 and x2 - x4 = 0.5, the solution of the KKT system
        anAssembly.AddConstraint(1, 1, 1, Vec(1, 0, 0), 1.0);
        anAssembly.AddConstraint(1, 2, 1, Vec(0, 0, 2), 2.0);
        anAssembly.AddConstraint(2, 1, 1, Vec(0, 1, 0), 0.25);
        anAssembly.AddConstraint(2, 2, 1, Vec(0, -1, 0), 0.25);
        Assert.True(anAssembly.Solve());
        anAssembly.Solution(aX);

        Matrix aKkt = new(1, 7, 1, 7, 0.0);
        math_Vector aRhs = new(1, 7, 0.0);
        for (int i = 1; i <= 5; i++)
        {
            aRhs[i] = aB[i];
            for (int j = 1; j <= 5; j++)
            {
                aKkt[i, j] = aDense[i, j];
            }
        }
        aKkt[6, 1] = aKkt[1, 6] = 1;
        aKkt[6, 5] = aKkt[5, 6] = 2;
        aKkt[7, 2] = aKkt[2, 7] = 1;
        aKkt[7, 4] = aKkt[4, 7] = -1;
        aRhs[6] = 3;
        aRhs[7] = 0.5;
        math_Vector aKktX = new(1, 7);
        new Gauss(aKkt).Solve(aRhs, aKktX);
        for (int i = 1; i <= 5; i++)
        {
            Assert.Equal(aKktX[i], aX[i], 1e-12);
        }
        Assert.Equal(3.0, aX[1] + 2 * aX[5], 1e-12);
        Assert.Equal(0.5, aX[2] - aX[4], 1e-12);

        // Nullified constraints are x = 0 lines: G = 0 and C = 0 make GHGt singular
        anAssembly.NullifyConstraint();
        Assert.False(anAssembly.Solve());
        // Without constraints, the first solution again
        anAssembly.ResetConstraint();
        Assert.True(anAssembly.Solve());
        anAssembly.Solution(aX);
        for (int i = 1; i <= 5; i++)
        {
            Assert.Equal(anExpected[i], aX[i], 1e-12);
        }

        anAssembly.NullifyMatrix();
        Assert.False(anAssembly.Solve());
        anAssembly.NullifyVector();
    }

    [Fact]
    public void Assembly_rejects_independent_dimensions()
    {
        NCollection_Array2<int> aDep = new(1, 2, 1, 2);
        aDep.Init(0);
        aDep[1, 1] = aDep[2, 2] = 1;
        NCollection_HArray2<NCollection_HArray1<int>> aTable = new(1, 2, 1, 1);
        aTable[1, 1] = new NCollection_HArray1<int>(1, 2);
        aTable[2, 1] = new NCollection_HArray1<int>(1, 2);
        aTable[1, 1][1] = 1;
        aTable[1, 1][2] = 2;
        aTable[2, 1][1] = 3;
        aTable[2, 1][2] = 4;
        Assembly anAssembly = new(aDep, aTable);
        Assert.Equal(4, anAssembly.GlobVarCount);
        EngineAssert.ThrowsAny<DomainErrorException>(() => anAssembly.AddMatrix(1, 1, 2, new Matrix(1, 2, 1, 2, 0.0)));
    }

    [Fact]
    public void Assembly_constraint_wider_than_its_segments_reproduces_OCCT()
    {
        // A constraint first on variables 2-3, then on 1-4: no segment contains either end of the new range, and
        // OCCT dereferences a null array (occt-issues.md); the port throws NullReferenceException
        NCollection_Array2<int> aDep = new(1, 1, 1, 1);
        aDep[1, 1] = 1;
        NCollection_HArray2<NCollection_HArray1<int>> aTable = new(1, 1, 1, 2);
        aTable[1, 1] = new NCollection_HArray1<int>(1, 2);
        aTable[1, 1][1] = 2;
        aTable[1, 1][2] = 3;
        aTable[1, 2] = new NCollection_HArray1<int>(1, 4);
        for (int i = 1; i <= 4; i++)
        {
            aTable[1, 2][i] = i;
        }
        Assembly anAssembly = new(aDep, aTable);
        anAssembly.AddConstraint(1, 1, 1, Vec(1, 1), 0);
        Assert.Throws<NullReferenceException>(() => anAssembly.AddConstraint(1, 2, 1, Vec(1, 1, 1, 1), 0));
    }

    private static math_Vector Vec(params double[] theValues)
    {
        math_Vector aV = new(1, theValues.Length);
        for (int i = 0; i < theValues.Length; i++)
        {
            aV[i + 1] = theValues[i];
        }
        return aV;
    }
}
