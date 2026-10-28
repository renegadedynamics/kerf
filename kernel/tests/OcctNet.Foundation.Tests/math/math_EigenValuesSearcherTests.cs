// Port of: src/FoundationClasses/TKMath/GTests/math_EigenValuesSearcher_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class math_EigenValuesSearcherTest
{
    // Helper function to create a tridiagonal matrix from arrays
    private static void createTridiagonalMatrix(NCollection_Array1<double> theDiagonal,
                                 NCollection_Array1<double> theSubdiagonal,
                                 math_Matrix theMatrix)
    {
        int aN = theDiagonal.Length();
        theMatrix.Init(0.0);

        // Set diagonal elements
        for (int i = 1; i <= aN; i++)
        {
            theMatrix[i, i] = theDiagonal[i];
        }

        // Set sub and super diagonal elements
        for (int i = 2; i <= aN; i++)
        {
            theMatrix[i, i - 1] = theSubdiagonal[i];
            theMatrix[i - 1, i] = theSubdiagonal[i];
        }
    }

    // Helper function to verify eigenvalue-eigenvector relationship: A*v = lambda*v
    private static bool verifyEigenPair(math_Matrix theMatrix,
                         double theEigenValue,
                         math_Vector theEigenVector,
                         double theTolerance = 1e-12)
    {
        int aN = theMatrix.RowNumber();
        math_Vector aResult = new(1, aN);

        // Compute A*v
        for (int i = 1; i <= aN; i++)
        {
            double aSum = 0.0;
            for (int j = 1; j <= aN; j++)
            {
                aSum += theMatrix[i, j] * theEigenVector[j];
            }
            aResult[i] = aSum;
        }

        // Check if A*v ~= lambda*v
        for (int i = 1; i <= aN; i++)
        {
            double aExpected = theEigenValue * theEigenVector[i];
            if (Math.Abs(aResult[i] - aExpected) > theTolerance)
            {
                return false;
            }
        }

        return true;
    }

    // Helper function to check if eigenvectors are orthogonal
    private static bool areOrthogonal(math_Vector theVec1,
                       math_Vector theVec2,
                       double theTolerance = 1e-12)
    {
        double aDotProduct = 0.0;
        for (int i = 1; i <= theVec1.Length(); i++)
        {
            aDotProduct += theVec1[i] * theVec2[i];
        }

        return Math.Abs(aDotProduct) < theTolerance;
    }

    // Helper function to compute vector norm
    private static double vectorNorm(math_Vector theVector)
    {
        double aNorm = 0.0;
        for (int i = 1; i <= theVector.Length(); i++)
        {
            aNorm += theVector[i] * theVector[i];
        }
        return Math.Sqrt(aNorm);
    }

    // Test constructor with dimension mismatch
    [Fact]
    public void ConstructorDimensionMismatch()
    {
        NCollection_Array1<double> aDiagonal = new(1, 3);
        NCollection_Array1<double> aSubdiagonal = new(1, 2); // Wrong size

        aDiagonal.SetValue(1, 1.0);
        aDiagonal.SetValue(2, 2.0);
        aDiagonal.SetValue(3, 3.0);

        aSubdiagonal.SetValue(1, 0.5);
        aSubdiagonal.SetValue(2, 1.5);

        math_EigenValuesSearcher searcher = new(aDiagonal, aSubdiagonal);

        // Should not be done due to dimension mismatch
        Assert.False(searcher.IsDone());
    }

    // Test 1x1 matrix (trivial case)
    [Fact]
    public void OneByOneMatrix()
    {
        NCollection_Array1<double> aDiagonal = new(1, 1);
        NCollection_Array1<double> aSubdiagonal = new(1, 1);

        aDiagonal.SetValue(1, 5.0);
        aSubdiagonal.SetValue(1, 0.0); // Not used for 1x1 matrix

        math_EigenValuesSearcher searcher = new(aDiagonal, aSubdiagonal);

        Assert.True(searcher.IsDone());
        Assert.Equal(1, searcher.Dimension());
        Assert.Equal(5.0, searcher.EigenValue(1), Precision.Confusion());

        math_Vector eigenvec = searcher.EigenVector(1);
        Assert.Equal(1, eigenvec.Length());
        Assert.Equal(1.0, eigenvec[1], Precision.Confusion());
    }

    // Test 2x2 symmetric tridiagonal matrix
    [Fact]
    public void TwoByTwoMatrix()
    {
        NCollection_Array1<double> aDiagonal = new(1, 2);
        NCollection_Array1<double> aSubdiagonal = new(1, 2);

        // Matrix: [2 1]
        //         [1 3]
        aDiagonal.SetValue(1, 2.0);
        aDiagonal.SetValue(2, 3.0);
        aSubdiagonal.SetValue(1, 0.0); // Not used
        aSubdiagonal.SetValue(2, 1.0);

        math_EigenValuesSearcher searcher = new(aDiagonal, aSubdiagonal);

        Assert.True(searcher.IsDone());
        Assert.Equal(2, searcher.Dimension());

        // Verify eigenvalues are approximately 0.382 and 4.618
        List<double> eigenvals = [searcher.EigenValue(1), searcher.EigenValue(2)];
        eigenvals.Sort();

        double lambda1 = 2.5 - 0.5 * Math.Sqrt(5.0); // ~= 0.382
        double lambda2 = 2.5 + 0.5 * Math.Sqrt(5.0); // ~= 4.618

        Assert.Equal(lambda1, eigenvals[0], 1e-10);
        Assert.Equal(lambda2, eigenvals[1], 1e-10);

        // Create original matrix for verification
        math_Matrix originalMatrix = new(1, 2, 1, 2);
        createTridiagonalMatrix(aDiagonal, aSubdiagonal, originalMatrix);

        // Verify eigenvalue-eigenvector relationships
        for (int i = 1; i <= 2; i++)
        {
            double eigenval = searcher.EigenValue(i);
            math_Vector eigenvec = searcher.EigenVector(i);

            Assert.True(verifyEigenPair(originalMatrix, eigenval, eigenvec, 1e-10));
            Assert.Equal(1.0, vectorNorm(eigenvec), 1e-10); // Should be normalized
        }

        // Verify eigenvectors are orthogonal
        math_Vector vec1 = searcher.EigenVector(1);
        math_Vector vec2 = searcher.EigenVector(2);
        Assert.True(areOrthogonal(vec1, vec2, 1e-10));
    }

    // Test 3x3 symmetric tridiagonal matrix
    [Fact]
    public void ThreeByThreeMatrix()
    {
        NCollection_Array1<double> aDiagonal = new(1, 3);
        NCollection_Array1<double> aSubdiagonal = new(1, 3);

        // Matrix: [4 1 0]
        //         [1 4 1]
        //         [0 1 4]
        aDiagonal.SetValue(1, 4.0);
        aDiagonal.SetValue(2, 4.0);
        aDiagonal.SetValue(3, 4.0);
        aSubdiagonal.SetValue(1, 0.0); // Not used
        aSubdiagonal.SetValue(2, 1.0);
        aSubdiagonal.SetValue(3, 1.0);

        math_EigenValuesSearcher searcher = new(aDiagonal, aSubdiagonal);

        Assert.True(searcher.IsDone());
        Assert.Equal(3, searcher.Dimension());

        // Create original matrix for verification
        math_Matrix originalMatrix = new(1, 3, 1, 3);
        createTridiagonalMatrix(aDiagonal, aSubdiagonal, originalMatrix);

        // Verify all eigenvalue-eigenvector pairs
        for (int i = 1; i <= 3; i++)
        {
            double eigenval = searcher.EigenValue(i);
            math_Vector eigenvec = searcher.EigenVector(i);

            Assert.True(verifyEigenPair(originalMatrix, eigenval, eigenvec, 1e-10));
            Assert.Equal(1.0, vectorNorm(eigenvec), 1e-10);
        }

        // Verify orthogonality of eigenvectors
        for (int i = 1; i <= 3; i++)
        {
            for (int j = i + 1; j <= 3; j++)
            {
                math_Vector vec_i = searcher.EigenVector(i);
                math_Vector vec_j = searcher.EigenVector(j);
                Assert.True(areOrthogonal(vec_i, vec_j, 1e-10));
            }
        }
    }

    // Test diagonal matrix (eigenvalues should be diagonal elements)
    [Fact]
    public void DiagonalMatrix()
    {
        NCollection_Array1<double> aDiagonal = new(1, 4);
        NCollection_Array1<double> aSubdiagonal = new(1, 4);

        aDiagonal.SetValue(1, 1.0);
        aDiagonal.SetValue(2, 2.0);
        aDiagonal.SetValue(3, 3.0);
        aDiagonal.SetValue(4, 4.0);

        // All subdiagonal elements are zero
        aSubdiagonal.SetValue(1, 0.0);
        aSubdiagonal.SetValue(2, 0.0);
        aSubdiagonal.SetValue(3, 0.0);
        aSubdiagonal.SetValue(4, 0.0);

        math_EigenValuesSearcher searcher = new(aDiagonal, aSubdiagonal);

        Assert.True(searcher.IsDone());
        Assert.Equal(4, searcher.Dimension());

        // For a diagonal matrix, eigenvalues should be the diagonal elements
        List<double> eigenvals = [];
        for (int i = 1; i <= 4; i++)
        {
            eigenvals.Add(searcher.EigenValue(i));
        }

        eigenvals.Sort();

        Assert.Equal(1.0, eigenvals[0], Precision.Confusion());
        Assert.Equal(2.0, eigenvals[1], Precision.Confusion());
        Assert.Equal(3.0, eigenvals[2], Precision.Confusion());
        Assert.Equal(4.0, eigenvals[3], Precision.Confusion());
    }

    // Test identity matrix
    [Fact]
    public void IdentityMatrix()
    {
        NCollection_Array1<double> aDiagonal = new(1, 3);
        NCollection_Array1<double> aSubdiagonal = new(1, 3);

        // Identity matrix
        aDiagonal.SetValue(1, 1.0);
        aDiagonal.SetValue(2, 1.0);
        aDiagonal.SetValue(3, 1.0);

        aSubdiagonal.SetValue(1, 0.0);
        aSubdiagonal.SetValue(2, 0.0);
        aSubdiagonal.SetValue(3, 0.0);

        math_EigenValuesSearcher searcher = new(aDiagonal, aSubdiagonal);

        Assert.True(searcher.IsDone());
        Assert.Equal(3, searcher.Dimension());

        // All eigenvalues should be 1.0
        for (int i = 1; i <= 3; i++)
        {
            Assert.Equal(1.0, searcher.EigenValue(i), Precision.Confusion());
        }
    }

    // Test matrix with negative eigenvalues
    [Fact]
    public void NegativeEigenvalues()
    {
        NCollection_Array1<double> aDiagonal = new(1, 2);
        NCollection_Array1<double> aSubdiagonal = new(1, 2);

        // Matrix: [-1  2]
        //         [ 2 -1]
        aDiagonal.SetValue(1, -1.0);
        aDiagonal.SetValue(2, -1.0);
        aSubdiagonal.SetValue(1, 0.0);
        aSubdiagonal.SetValue(2, 2.0);

        math_EigenValuesSearcher searcher = new(aDiagonal, aSubdiagonal);

        Assert.True(searcher.IsDone());

        // Eigenvalues should be -3 and 1
        List<double> eigenvals = [searcher.EigenValue(1), searcher.EigenValue(2)];
        eigenvals.Sort();

        Assert.Equal(-3.0, eigenvals[0], 1e-10);
        Assert.Equal(1.0, eigenvals[1], 1e-10);

        // Create original matrix for verification
        math_Matrix originalMatrix = new(1, 2, 1, 2);
        createTridiagonalMatrix(aDiagonal, aSubdiagonal, originalMatrix);

        for (int i = 1; i <= 2; i++)
        {
            double eigenval = searcher.EigenValue(i);
            math_Vector eigenvec = searcher.EigenVector(i);
            Assert.True(verifyEigenPair(originalMatrix, eigenval, eigenvec, 1e-10));
        }
    }

    // Test larger matrix (5x5)
    [Fact]
    public void FiveByFiveMatrix()
    {
        NCollection_Array1<double> aDiagonal = new(1, 5);
        NCollection_Array1<double> aSubdiagonal = new(1, 5);

        // Create a symmetric tridiagonal matrix with pattern
        for (int i = 1; i <= 5; i++)
        {
            aDiagonal.SetValue(i, 2.0);
        }

        aSubdiagonal.SetValue(1, 0.0); // Not used
        for (int i = 2; i <= 5; i++)
        {
            aSubdiagonal.SetValue(i, -1.0);
        }

        math_EigenValuesSearcher searcher = new(aDiagonal, aSubdiagonal);

        Assert.True(searcher.IsDone());
        Assert.Equal(5, searcher.Dimension());

        // Create original matrix for verification
        math_Matrix originalMatrix = new(1, 5, 1, 5);
        createTridiagonalMatrix(aDiagonal, aSubdiagonal, originalMatrix);

        // Verify all eigenvalue-eigenvector pairs
        for (int i = 1; i <= 5; i++)
        {
            double eigenval = searcher.EigenValue(i);
            math_Vector eigenvec = searcher.EigenVector(i);

            Assert.True(verifyEigenPair(originalMatrix, eigenval, eigenvec, 1e-9));
            Assert.Equal(1.0, vectorNorm(eigenvec), 1e-9);
        }

        // Check that all eigenvalues are reasonable
        for (int i = 1; i <= 5; i++)
        {
            double eigenval = searcher.EigenValue(i);
            Assert.True(eigenval >= 0.0); // All eigenvalues should be non-negative for this matrix
            Assert.True(eigenval <= 4.0); // Should be bounded
        }
    }

    // Test with very small subdiagonal elements (near singular)
    [Fact]
    public void SmallSubdiagonalElements()
    {
        NCollection_Array1<double> aDiagonal = new(1, 3);
        NCollection_Array1<double> aSubdiagonal = new(1, 3);

        aDiagonal.SetValue(1, 1.0);
        aDiagonal.SetValue(2, 2.0);
        aDiagonal.SetValue(3, 3.0);

        aSubdiagonal.SetValue(1, 0.0);
        aSubdiagonal.SetValue(2, 1e-14);
        aSubdiagonal.SetValue(3, 1e-14);

        math_EigenValuesSearcher searcher = new(aDiagonal, aSubdiagonal);

        Assert.True(searcher.IsDone());

        // Should converge to approximately diagonal values
        List<double> eigenvals = [];
        for (int i = 1; i <= 3; i++)
        {
            eigenvals.Add(searcher.EigenValue(i));
        }

        eigenvals.Sort();

        Assert.Equal(1.0, eigenvals[0], 1e-10);
        Assert.Equal(2.0, eigenvals[1], 1e-10);
        Assert.Equal(3.0, eigenvals[2], 1e-10);
    }

    // Test with zero diagonal elements
    [Fact]
    public void ZeroDiagonalElements()
    {
        NCollection_Array1<double> aDiagonal = new(1, 3);
        NCollection_Array1<double> aSubdiagonal = new(1, 3);

        // Matrix: [0 1 0]
        //         [1 0 1]
        //         [0 1 0]
        aDiagonal.SetValue(1, 0.0);
        aDiagonal.SetValue(2, 0.0);
        aDiagonal.SetValue(3, 0.0);

        aSubdiagonal.SetValue(1, 0.0);
        aSubdiagonal.SetValue(2, 1.0);
        aSubdiagonal.SetValue(3, 1.0);

        math_EigenValuesSearcher searcher = new(aDiagonal, aSubdiagonal);

        Assert.True(searcher.IsDone());
        Assert.Equal(3, searcher.Dimension());

        // Eigenvalues should be -Math.Sqrt(2), 0, Math.Sqrt(2)
        List<double> eigenvals = [];
        for (int i = 1; i <= 3; i++)
        {
            eigenvals.Add(searcher.EigenValue(i));
        }

        eigenvals.Sort();

        Assert.Equal(-Math.Sqrt(2.0), eigenvals[0], 1e-10);
        Assert.Equal(0.0, eigenvals[1], 1e-10);
        Assert.Equal(Math.Sqrt(2.0), eigenvals[2], 1e-10);
    }

    // Test with large diagonal elements (numerical stability)
    [Fact]
    public void LargeDiagonalElements()
    {
        NCollection_Array1<double> aDiagonal = new(1, 3);
        NCollection_Array1<double> aSubdiagonal = new(1, 3);

        aDiagonal.SetValue(1, 1e6);
        aDiagonal.SetValue(2, 2e6);
        aDiagonal.SetValue(3, 3e6);

        aSubdiagonal.SetValue(1, 0.0);
        aSubdiagonal.SetValue(2, 1e3);
        aSubdiagonal.SetValue(3, 2e3);

        math_EigenValuesSearcher searcher = new(aDiagonal, aSubdiagonal);

        Assert.True(searcher.IsDone());

        // Create original matrix for verification
        math_Matrix originalMatrix = new(1, 3, 1, 3);
        createTridiagonalMatrix(aDiagonal, aSubdiagonal, originalMatrix);

        for (int i = 1; i <= 3; i++)
        {
            double eigenval = searcher.EigenValue(i);
            math_Vector eigenvec = searcher.EigenVector(i);

            Assert.True(verifyEigenPair(originalMatrix, eigenval, eigenvec, 1e-6));
            Assert.Equal(1.0, vectorNorm(eigenvec), 1e-10);
        }
    }

    // Test with alternating pattern
    [Fact]
    public void AlternatingPattern()
    {
        NCollection_Array1<double> aDiagonal = new(1, 4);
        NCollection_Array1<double> aSubdiagonal = new(1, 4);

        // Matrix with alternating diagonal pattern
        aDiagonal.SetValue(1, 1.0);
        aDiagonal.SetValue(2, -1.0);
        aDiagonal.SetValue(3, 1.0);
        aDiagonal.SetValue(4, -1.0);

        aSubdiagonal.SetValue(1, 0.0);
        aSubdiagonal.SetValue(2, 0.5);
        aSubdiagonal.SetValue(3, -0.5);
        aSubdiagonal.SetValue(4, 0.5);

        math_EigenValuesSearcher searcher = new(aDiagonal, aSubdiagonal);

        Assert.True(searcher.IsDone());

        // Verify eigenvalue-eigenvector relationships
        math_Matrix originalMatrix = new(1, 4, 1, 4);
        createTridiagonalMatrix(aDiagonal, aSubdiagonal, originalMatrix);

        for (int i = 1; i <= 4; i++)
        {
            double eigenval = searcher.EigenValue(i);
            math_Vector eigenvec = searcher.EigenVector(i);

            Assert.True(verifyEigenPair(originalMatrix, eigenval, eigenvec, 1e-10));
        }
    }

    // Test repeated eigenvalues (multiple eigenvalues)
    [Fact]
    public void RepeatedEigenvalues()
    {
        NCollection_Array1<double> aDiagonal = new(1, 4);
        NCollection_Array1<double> aSubdiagonal = new(1, 4);

        // Matrix designed to have repeated eigenvalues
        aDiagonal.SetValue(1, 2.0);
        aDiagonal.SetValue(2, 2.0);
        aDiagonal.SetValue(3, 2.0);
        aDiagonal.SetValue(4, 2.0);

        aSubdiagonal.SetValue(1, 0.0);
        aSubdiagonal.SetValue(2, 0.0); // Block diagonal structure
        aSubdiagonal.SetValue(3, 0.0);
        aSubdiagonal.SetValue(4, 0.0);

        math_EigenValuesSearcher searcher = new(aDiagonal, aSubdiagonal);

        Assert.True(searcher.IsDone());

        // All eigenvalues should be 2.0
        for (int i = 1; i <= 4; i++)
        {
            Assert.Equal(2.0, searcher.EigenValue(i), Precision.Confusion());
        }
    }

    // Test with very small matrix elements (precision test)
    [Fact]
    public void VerySmallElements()
    {
        NCollection_Array1<double> aDiagonal = new(1, 3);
        NCollection_Array1<double> aSubdiagonal = new(1, 3);

        aDiagonal.SetValue(1, 1e-10);
        aDiagonal.SetValue(2, 2e-10);
        aDiagonal.SetValue(3, 3e-10);

        aSubdiagonal.SetValue(1, 0.0);
        aSubdiagonal.SetValue(2, 1e-11);
        aSubdiagonal.SetValue(3, 2e-11);

        math_EigenValuesSearcher searcher = new(aDiagonal, aSubdiagonal);

        Assert.True(searcher.IsDone());

        math_Matrix originalMatrix = new(1, 3, 1, 3);
        createTridiagonalMatrix(aDiagonal, aSubdiagonal, originalMatrix);

        for (int i = 1; i <= 3; i++)
        {
            double eigenval = searcher.EigenValue(i);
            math_Vector eigenvec = searcher.EigenVector(i);

            Assert.True(verifyEigenPair(originalMatrix, eigenval, eigenvec, 1e-18));
            Assert.Equal(1.0, vectorNorm(eigenvec), 1e-12);
        }
    }

    // Test antisymmetric subdiagonal pattern
    [Fact]
    public void AntisymmetricSubdiagonal()
    {
        NCollection_Array1<double> aDiagonal = new(1, 5);
        NCollection_Array1<double> aSubdiagonal = new(1, 5);

        // Symmetric tridiagonal with specific pattern
        for (int i = 1; i <= 5; i++)
        {
            aDiagonal.SetValue(i, (double)(i));
        }

        aSubdiagonal.SetValue(1, 0.0);
        aSubdiagonal.SetValue(2, 1.0);
        aSubdiagonal.SetValue(3, -2.0);
        aSubdiagonal.SetValue(4, 1.0);
        aSubdiagonal.SetValue(5, -2.0);

        math_EigenValuesSearcher searcher = new(aDiagonal, aSubdiagonal);

        Assert.True(searcher.IsDone());

        math_Matrix originalMatrix = new(1, 5, 1, 5);
        createTridiagonalMatrix(aDiagonal, aSubdiagonal, originalMatrix);

        // Verify all eigenvalue-eigenvector pairs
        for (int i = 1; i <= 5; i++)
        {
            double eigenval = searcher.EigenValue(i);
            math_Vector eigenvec = searcher.EigenVector(i);

            Assert.True(verifyEigenPair(originalMatrix, eigenval, eigenvec, 1e-9));
        }

        // Verify orthogonality
        for (int i = 1; i <= 5; i++)
        {
            for (int j = i + 1; j <= 5; j++)
            {
                math_Vector vec_i = searcher.EigenVector(i);
                math_Vector vec_j = searcher.EigenVector(j);
                Assert.True(areOrthogonal(vec_i, vec_j, 1e-9));
            }
        }
    }

    // Test Wilkinson matrix (known challenging case)
    [Fact]
    public void WilkinsonMatrix()
    {
        int n = 5;
        NCollection_Array1<double> aDiagonal = new(1, n);
        NCollection_Array1<double> aSubdiagonal = new(1, n);

        // Wilkinson matrix W_n pattern
        int m = (n - 1) / 2;
        for (int i = 1; i <= n; i++)
        {
            aDiagonal.SetValue(i, (double)(Math.Abs(i - 1 - m)));
        }

        aSubdiagonal.SetValue(1, 0.0);
        for (int i = 2; i <= n; i++)
        {
            aSubdiagonal.SetValue(i, 1.0);
        }

        math_EigenValuesSearcher searcher = new(aDiagonal, aSubdiagonal);

        Assert.True(searcher.IsDone());

        math_Matrix originalMatrix = new(1, n, 1, n);
        createTridiagonalMatrix(aDiagonal, aSubdiagonal, originalMatrix);

        for (int i = 1; i <= n; i++)
        {
            double eigenval = searcher.EigenValue(i);
            math_Vector eigenvec = searcher.EigenVector(i);

            Assert.True(verifyEigenPair(originalMatrix, eigenval, eigenvec, 1e-9));
            Assert.Equal(1.0, vectorNorm(eigenvec), 1e-9);
        }
    }

    // Test with mixed positive/negative diagonal
    [Fact]
    public void MixedSignDiagonal()
    {
        NCollection_Array1<double> aDiagonal = new(1, 4);
        NCollection_Array1<double> aSubdiagonal = new(1, 4);

        aDiagonal.SetValue(1, -2.0);
        aDiagonal.SetValue(2, 1.0);
        aDiagonal.SetValue(3, -1.0);
        aDiagonal.SetValue(4, 3.0);

        aSubdiagonal.SetValue(1, 0.0);
        aSubdiagonal.SetValue(2, 1.5);
        aSubdiagonal.SetValue(3, 0.8);
        aSubdiagonal.SetValue(4, -0.6);

        math_EigenValuesSearcher searcher = new(aDiagonal, aSubdiagonal);

        Assert.True(searcher.IsDone());

        math_Matrix originalMatrix = new(1, 4, 1, 4);
        createTridiagonalMatrix(aDiagonal, aSubdiagonal, originalMatrix);

        List<double> eigenvals = [];
        for (int i = 1; i <= 4; i++)
        {
            double eigenval = searcher.EigenValue(i);
            math_Vector eigenvec = searcher.EigenVector(i);

            eigenvals.Add(eigenval);
            Assert.True(verifyEigenPair(originalMatrix, eigenval, eigenvec, 1e-9));
        }

        // Check trace preservation (sum of eigenvalues = sum of diagonal)
        double eigenSum = 0.0, diagSum = 0.0;
        for (int i = 1; i <= 4; i++)
        {
            eigenSum += eigenvals[i - 1];
            diagSum += aDiagonal[i];
        }
        Assert.Equal(diagSum, eigenSum, 1e-9);
    }

    // Test maximum size matrix (stress test)
    [Fact]
    public void LargerMatrix()
    {
        int n = 8;
        NCollection_Array1<double> aDiagonal = new(1, n);
        NCollection_Array1<double> aSubdiagonal = new(1, n);

        // Create a more complex pattern
        for (int i = 1; i <= n; i++)
        {
            aDiagonal.SetValue(i, (double)(i * i));
        }

        aSubdiagonal.SetValue(1, 0.0);
        for (int i = 2; i <= n; i++)
        {
            double val = (double)(i % 3 == 0 ? -1.0 : 1.0);
            aSubdiagonal.SetValue(i, val * Math.Sqrt((double)(i)));
        }

        math_EigenValuesSearcher searcher = new(aDiagonal, aSubdiagonal);

        Assert.True(searcher.IsDone());

        math_Matrix originalMatrix = new(1, n, 1, n);
        createTridiagonalMatrix(aDiagonal, aSubdiagonal, originalMatrix);

        // Verify all eigenvalue-eigenvector pairs
        for (int i = 1; i <= n; i++)
        {
            double eigenval = searcher.EigenValue(i);
            math_Vector eigenvec = searcher.EigenVector(i);

            Assert.True(verifyEigenPair(originalMatrix, eigenval, eigenvec, 1e-8));
            Assert.Equal(1.0, vectorNorm(eigenvec), 1e-8);
        }

        // Test orthogonality for larger matrix
        for (int i = 1; i <= n; i++)
        {
            for (int j = i + 1; j <= n; j++)
            {
                math_Vector vec_i = searcher.EigenVector(i);
                math_Vector vec_j = searcher.EigenVector(j);
                Assert.True(areOrthogonal(vec_i, vec_j, 1e-8));
            }
        }
    }

    // Test with rational number patterns
    [Fact]
    public void RationalNumberPattern()
    {
        NCollection_Array1<double> aDiagonal = new(1, 4);
        NCollection_Array1<double> aSubdiagonal = new(1, 4);

        aDiagonal.SetValue(1, 1.0 / 3.0);
        aDiagonal.SetValue(2, 2.0 / 3.0);
        aDiagonal.SetValue(3, 1.0);
        aDiagonal.SetValue(4, 4.0 / 3.0);

        aSubdiagonal.SetValue(1, 0.0);
        aSubdiagonal.SetValue(2, 1.0 / 6.0);
        aSubdiagonal.SetValue(3, 1.0 / 4.0);
        aSubdiagonal.SetValue(4, 1.0 / 5.0);

        math_EigenValuesSearcher searcher = new(aDiagonal, aSubdiagonal);

        Assert.True(searcher.IsDone());

        math_Matrix originalMatrix = new(1, 4, 1, 4);
        createTridiagonalMatrix(aDiagonal, aSubdiagonal, originalMatrix);

        for (int i = 1; i <= 4; i++)
        {
            double eigenval = searcher.EigenValue(i);
            math_Vector eigenvec = searcher.EigenVector(i);

            Assert.True(verifyEigenPair(originalMatrix, eigenval, eigenvec, 1e-12));
            Assert.Equal(1.0, vectorNorm(eigenvec), 1e-12);
        }
    }

    // Test near-degenerate case (eigenvalues very close)
    [Fact]
    public void NearDegenerateEigenvalues()
    {
        NCollection_Array1<double> aDiagonal = new(1, 3);
        NCollection_Array1<double> aSubdiagonal = new(1, 3);

        aDiagonal.SetValue(1, 1.0);
        aDiagonal.SetValue(2, 1.0 + 1e-8);
        aDiagonal.SetValue(3, 1.0 + 2e-8);

        aSubdiagonal.SetValue(1, 0.0);
        aSubdiagonal.SetValue(2, 1e-10);
        aSubdiagonal.SetValue(3, 1e-10);

        math_EigenValuesSearcher searcher = new(aDiagonal, aSubdiagonal);

        Assert.True(searcher.IsDone());

        // Should still find distinct eigenvalues despite near-degeneracy
        List<double> eigenvals = [];
        for (int i = 1; i <= 3; i++)
        {
            eigenvals.Add(searcher.EigenValue(i));
        }

        eigenvals.Sort();

        // Eigenvalues should be close to but distinct from diagonal values
        Assert.Equal(1.0, eigenvals[0], 1e-7);
        Assert.Equal(1.0 + 1e-8, eigenvals[1], 1e-7);
        Assert.Equal(1.0 + 2e-8, eigenvals[2], 1e-7);

        math_Matrix originalMatrix = new(1, 3, 1, 3);
        createTridiagonalMatrix(aDiagonal, aSubdiagonal, originalMatrix);

        for (int i = 1; i <= 3; i++)
        {
            double eigenval = searcher.EigenValue(i);
            math_Vector eigenvec = searcher.EigenVector(i);

            Assert.True(verifyEigenPair(originalMatrix, eigenval, eigenvec, 1e-9));
        }
    }

    // Test deflation condition edge case - subdiagonal element smaller than machine epsilon
    [Fact]
    public void DeflationConditionPrecision()
    {
        NCollection_Array1<double> aDiagonal = new(1, 4);
        NCollection_Array1<double> aSubdiagonal = new(1, 4);

        // Large diagonal elements
        aDiagonal.SetValue(1, 1e6);
        aDiagonal.SetValue(2, 2e6);
        aDiagonal.SetValue(3, 3e6);
        aDiagonal.SetValue(4, 4e6);

        // Subdiagonal elements at machine epsilon level
        aSubdiagonal.SetValue(1, 0.0);
        aSubdiagonal.SetValue(2, 1e-15); // Should trigger deflation
        aSubdiagonal.SetValue(3, 2e-15); // Should trigger deflation
        aSubdiagonal.SetValue(4, 3e-15); // Should trigger deflation

        math_EigenValuesSearcher searcher = new(aDiagonal, aSubdiagonal);

        Assert.True(searcher.IsDone());

        // Should converge quickly due to deflation
        math_Matrix originalMatrix = new(1, 4, 1, 4);
        createTridiagonalMatrix(aDiagonal, aSubdiagonal, originalMatrix);

        for (int i = 1; i <= 4; i++)
        {
            double eigenval = searcher.EigenValue(i);
            math_Vector eigenvec = searcher.EigenVector(i);

            Assert.True(verifyEigenPair(originalMatrix, eigenval, eigenvec, 1e-9));
        }

        // Eigenvalues should be close to diagonal elements due to small off-diagonal
        List<double> eigenvals = [];
        for (int i = 1; i <= 4; i++)
        {
            eigenvals.Add(searcher.EigenValue(i));
        }

        eigenvals.Sort();

        Assert.Equal(1e6, eigenvals[0], 1e3);
        Assert.Equal(2e6, eigenvals[1], 1e3);
        Assert.Equal(3e6, eigenvals[2], 1e3);
        Assert.Equal(4e6, eigenvals[3], 1e3);
    }

    // Test exact zero subdiagonal elements (should deflate immediately)
    [Fact]
    public void ExactZeroSubdiagonal()
    {
        NCollection_Array1<double> aDiagonal = new(1, 5);
        NCollection_Array1<double> aSubdiagonal = new(1, 5);

        aDiagonal.SetValue(1, 1.0);
        aDiagonal.SetValue(2, 4.0);
        aDiagonal.SetValue(3, 9.0);
        aDiagonal.SetValue(4, 16.0);
        aDiagonal.SetValue(5, 25.0);

        aSubdiagonal.SetValue(1, 0.0);
        aSubdiagonal.SetValue(2, 1.0);
        aSubdiagonal.SetValue(3, 0.0); // Exact zero - should cause deflation
        aSubdiagonal.SetValue(4, 2.0);
        aSubdiagonal.SetValue(5, 0.0); // Exact zero - should cause deflation

        math_EigenValuesSearcher searcher = new(aDiagonal, aSubdiagonal);

        Assert.True(searcher.IsDone());

        // Should handle block structure correctly
        math_Matrix originalMatrix = new(1, 5, 1, 5);
        createTridiagonalMatrix(aDiagonal, aSubdiagonal, originalMatrix);

        for (int i = 1; i <= 5; i++)
        {
            double eigenval = searcher.EigenValue(i);
            math_Vector eigenvec = searcher.EigenVector(i);

            Assert.True(verifyEigenPair(originalMatrix, eigenval, eigenvec, 1e-12));
        }
    }

    // Test convergence behavior with maximum iterations edge case
    [Fact]
    public void ConvergenceBehavior()
    {
        NCollection_Array1<double> aDiagonal = new(1, 6);
        NCollection_Array1<double> aSubdiagonal = new(1, 6);

        // Create a matrix that might require more iterations to converge
        for (int i = 1; i <= 6; i++)
        {
            aDiagonal.SetValue(i, (double)(i) * 0.1);
        }

        aSubdiagonal.SetValue(1, 0.0);
        for (int i = 2; i <= 6; i++)
        {
            aSubdiagonal.SetValue(i, 0.99); // Large off-diagonal elements
        }

        math_EigenValuesSearcher searcher = new(aDiagonal, aSubdiagonal);

        Assert.True(searcher.IsDone());

        // Should still converge within maximum iterations
        math_Matrix originalMatrix = new(1, 6, 1, 6);
        createTridiagonalMatrix(aDiagonal, aSubdiagonal, originalMatrix);

        for (int i = 1; i <= 6; i++)
        {
            double eigenval = searcher.EigenValue(i);
            math_Vector eigenvec = searcher.EigenVector(i);

            Assert.True(verifyEigenPair(originalMatrix, eigenval, eigenvec, 1e-9));
        }
    }

    // Test underflow/overflow protection
    [Fact]
    public void NumericalStability()
    {
        NCollection_Array1<double> aDiagonal = new(1, 3);
        NCollection_Array1<double> aSubdiagonal = new(1, 3);

        // Mix of very large and very small values
        aDiagonal.SetValue(1, 1e-100);
        aDiagonal.SetValue(2, 1e100);
        aDiagonal.SetValue(3, 1e-50);

        aSubdiagonal.SetValue(1, 0.0);
        aSubdiagonal.SetValue(2, 1e-75);
        aSubdiagonal.SetValue(3, 1e75);

        math_EigenValuesSearcher searcher = new(aDiagonal, aSubdiagonal);

        Assert.True(searcher.IsDone());

        // Should handle extreme values without overflow/underflow
        for (int i = 1; i <= 3; i++)
        {
            double eigenval = searcher.EigenValue(i);
            Assert.True(double.IsFinite(eigenval));
            Assert.False(double.IsNaN(eigenval));

            math_Vector eigenvec = searcher.EigenVector(i);
            for (int j = 1; j <= 3; j++)
            {
                Assert.True(double.IsFinite(eigenvec[j]));
                Assert.False(double.IsNaN(eigenvec[j]));
            }
        }
    }

    // Test Wilkinson shift effectiveness
    [Fact]
    public void WilkinsonShiftAccuracy()
    {
        NCollection_Array1<double> aDiagonal = new(1, 3);
        NCollection_Array1<double> aSubdiagonal = new(1, 3);

        // Matrix where Wilkinson shift should provide fast convergence
        aDiagonal.SetValue(1, 1.0);
        aDiagonal.SetValue(2, 2.0);
        aDiagonal.SetValue(3, 1.0000001); // Very close to first diagonal element

        aSubdiagonal.SetValue(1, 0.0);
        aSubdiagonal.SetValue(2, 1.0);
        aSubdiagonal.SetValue(3, 0.0000001); // Very small

        math_EigenValuesSearcher searcher = new(aDiagonal, aSubdiagonal);

        Assert.True(searcher.IsDone());

        math_Matrix originalMatrix = new(1, 3, 1, 3);
        createTridiagonalMatrix(aDiagonal, aSubdiagonal, originalMatrix);

        for (int i = 1; i <= 3; i++)
        {
            double eigenval = searcher.EigenValue(i);
            math_Vector eigenvec = searcher.EigenVector(i);

            Assert.True(verifyEigenPair(originalMatrix, eigenval, eigenvec, 1e-10));
        }
    }

    // Test special case when radius becomes zero in QL step
    [Fact]
    public void ZeroRadiusHandling()
    {
        NCollection_Array1<double> aDiagonal = new(1, 4);
        NCollection_Array1<double> aSubdiagonal = new(1, 4);

        // Configuration that might lead to zero radius in computeHypotenuseLength
        aDiagonal.SetValue(1, 0.0);
        aDiagonal.SetValue(2, 0.0);
        aDiagonal.SetValue(3, 1.0);
        aDiagonal.SetValue(4, 1.0);

        aSubdiagonal.SetValue(1, 0.0);
        aSubdiagonal.SetValue(2, 1e-16); // Very small but non-zero
        aSubdiagonal.SetValue(3, 1e-16);
        aSubdiagonal.SetValue(4, 0.0);

        math_EigenValuesSearcher searcher = new(aDiagonal, aSubdiagonal);

        Assert.True(searcher.IsDone());

        math_Matrix originalMatrix = new(1, 4, 1, 4);
        createTridiagonalMatrix(aDiagonal, aSubdiagonal, originalMatrix);

        for (int i = 1; i <= 4; i++)
        {
            double eigenval = searcher.EigenValue(i);
            math_Vector eigenvec = searcher.EigenVector(i);

            Assert.True(verifyEigenPair(originalMatrix, eigenval, eigenvec, 1e-12));
        }
    }

    // Test pathological case with all equal elements
    [Fact]
    public void PathologicalEqualElements()
    {
        NCollection_Array1<double> aDiagonal = new(1, 5);
        NCollection_Array1<double> aSubdiagonal = new(1, 5);

        // All elements equal - degenerate case
        double value = 42.0;
        for (int i = 1; i <= 5; i++)
        {
            aDiagonal.SetValue(i, value);
        }

        aSubdiagonal.SetValue(1, 0.0);
        for (int i = 2; i <= 5; i++)
        {
            aSubdiagonal.SetValue(i, value);
        }

        math_EigenValuesSearcher searcher = new(aDiagonal, aSubdiagonal);

        Assert.True(searcher.IsDone());

        // Algorithm should still work correctly
        math_Matrix originalMatrix = new(1, 5, 1, 5);
        createTridiagonalMatrix(aDiagonal, aSubdiagonal, originalMatrix);

        for (int i = 1; i <= 5; i++)
        {
            double eigenval = searcher.EigenValue(i);
            math_Vector eigenvec = searcher.EigenVector(i);

            Assert.True(verifyEigenPair(originalMatrix, eigenval, eigenvec, 1e-9));
        }
    }

    // Test matrix with systematically increasing subdiagonal elements
    [Fact]
    public void IncreasingSubdiagonal()
    {
        NCollection_Array1<double> aDiagonal = new(1, 6);
        NCollection_Array1<double> aSubdiagonal = new(1, 6);

        for (int i = 1; i <= 6; i++)
        {
            aDiagonal.SetValue(i, 1.0);
        }

        aSubdiagonal.SetValue(1, 0.0);
        for (int i = 2; i <= 6; i++)
        {
            aSubdiagonal.SetValue(i, (double)(i - 1) * 0.1);
        }

        math_EigenValuesSearcher searcher = new(aDiagonal, aSubdiagonal);

        Assert.True(searcher.IsDone());

        math_Matrix originalMatrix = new(1, 6, 1, 6);
        createTridiagonalMatrix(aDiagonal, aSubdiagonal, originalMatrix);

        // Verify all pairs and orthogonality
        for (int i = 1; i <= 6; i++)
        {
            double eigenval = searcher.EigenValue(i);
            math_Vector eigenvec = searcher.EigenVector(i);

            Assert.True(verifyEigenPair(originalMatrix, eigenval, eigenvec, 1e-10));
            Assert.Equal(1.0, vectorNorm(eigenvec), 1e-10);
        }

        // Check orthogonality of all eigenvector pairs
        for (int i = 1; i <= 6; i++)
        {
            for (int j = i + 1; j <= 6; j++)
            {
                math_Vector vec_i = searcher.EigenVector(i);
                math_Vector vec_j = searcher.EigenVector(j);
                Assert.True(areOrthogonal(vec_i, vec_j, 1e-10));
            }
        }
    }

    // Test to demonstrate and verify the deflation condition behavior
    // This test documents the precision semantics of the deflation test:
    // |e[i]| + (|d[i]| + |d[i+1]|) == |d[i]| + |d[i+1]| in floating-point arithmetic
    [Fact]
    public void DeflationConditionSemantics()
    {
        NCollection_Array1<double> aDiagonal = new(1, 3);
        NCollection_Array1<double> aSubdiagonal = new(1, 3);

        // Test 1: Large diagonal elements with subdiagonal at machine epsilon level
        double largeDiag1 = 1e8;
        double largeDiag2 = 2e8;
        double machEpsilonLevel = largeDiag1 * RealEpsilon();

        aDiagonal.SetValue(1, largeDiag1);
        aDiagonal.SetValue(2, largeDiag2);
        aDiagonal.SetValue(3, 3e8);

        aSubdiagonal.SetValue(1, 0.0);
        aSubdiagonal.SetValue(
          2,
          machEpsilonLevel); // Should trigger deflation due to floating-point precision
        aSubdiagonal.SetValue(3, machEpsilonLevel * 0.5); // Even smaller, should definitely deflate

        // Verify the mathematical behavior that the deflation condition tests
        double diagSum = Math.Abs(largeDiag1) + Math.Abs(largeDiag2);
        double testCondition = machEpsilonLevel + diagSum;

        // This should be true in floating-point arithmetic due to precision limits
        Assert.Equal(diagSum, testCondition);

        math_EigenValuesSearcher searcher = new(aDiagonal, aSubdiagonal);
        Assert.True(searcher.IsDone());

        // Should converge efficiently due to deflation
        math_Matrix originalMatrix = new(1, 3, 1, 3);
        createTridiagonalMatrix(aDiagonal, aSubdiagonal, originalMatrix);

        for (int i = 1; i <= 3; i++)
        {
            double eigenval = searcher.EigenValue(i);
            math_Vector eigenvec = searcher.EigenVector(i);
            // Use looser tolerance for very large eigenvalues
            Assert.True(verifyEigenPair(originalMatrix, eigenval, eigenvec, 1e-6));
        }
    }

    // Test edge case where deflation condition is exactly at the boundary
    [Fact]
    public void DeflationBoundaryCondition()
    {
        NCollection_Array1<double> aDiagonal = new(1, 4);
        NCollection_Array1<double> aSubdiagonal = new(1, 4);

        // Create diagonal elements of different scales
        aDiagonal.SetValue(1, 1.0);
        aDiagonal.SetValue(2, 1000.0);
        aDiagonal.SetValue(3, 1e-6);
        aDiagonal.SetValue(4, 1e6);

        aSubdiagonal.SetValue(1, 0.0);

        // Set subdiagonal elements right at the deflation boundary for different scales
        double eps = RealEpsilon();

        // For first pair: should deflate
        double sum1 = Math.Abs(aDiagonal[1]) + Math.Abs(aDiagonal[2]);
        aSubdiagonal.SetValue(2, sum1 * eps * 0.1); // Below machine epsilon relative to sum

        // For second pair: should deflate
        double sum2 = Math.Abs(aDiagonal[2]) + Math.Abs(aDiagonal[3]);
        aSubdiagonal.SetValue(3, sum2 * eps * 0.1);

        // For third pair: should deflate
        double sum3 = Math.Abs(aDiagonal[3]) + Math.Abs(aDiagonal[4]);
        aSubdiagonal.SetValue(4, sum3 * eps * 0.1);

        math_EigenValuesSearcher searcher = new(aDiagonal, aSubdiagonal);
        Assert.True(searcher.IsDone());

        // Verify that algorithm handles the mixed scales correctly
        math_Matrix originalMatrix = new(1, 4, 1, 4);
        createTridiagonalMatrix(aDiagonal, aSubdiagonal, originalMatrix);

        for (int i = 1; i <= 4; i++)
        {
            double eigenval = searcher.EigenValue(i);
            math_Vector eigenvec = searcher.EigenVector(i);

            Assert.True(verifyEigenPair(originalMatrix, eigenval, eigenvec, 1e-9));
            Assert.True(double.IsFinite(eigenval));
            Assert.False(double.IsNaN(eigenval));
        }
    }
}
