// Port of: src/FoundationClasses/TKMath/GTests/math_Matrix_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class math_MatrixTest
{
    // Helper function for comparing matrices with tolerance
    private static void checkMatricesEqual(math_Matrix theM1, math_Matrix theM2, double theTolerance = 1.0e-7 /* Precision::Confusion() */)
    {
        Assert.Equal(theM2.RowNumber(), theM1.RowNumber());
        Assert.Equal(theM2.ColNumber(), theM1.ColNumber());
        Assert.Equal(theM2.LowerRow(), theM1.LowerRow());
        Assert.Equal(theM2.LowerCol(), theM1.LowerCol());

        for (int anI = theM1.LowerRow(); anI <= theM1.UpperRow(); anI++)
        {
            for (int aJ = theM1.LowerCol(); aJ <= theM1.UpperCol(); aJ++)
            {
                Assert.Equal(theM2[anI, aJ], theM1[anI, aJ], theTolerance);
            }
        }
    }

    // Tests for constructors
    [Fact]
    public void Constructors()
    {
        // Test standard constructor
        math_Matrix aMatrix1 = new(1, 3, 1, 4);
        Assert.Equal(3, aMatrix1.RowNumber());
        Assert.Equal(4, aMatrix1.ColNumber());
        Assert.Equal(1, aMatrix1.LowerRow());
        Assert.Equal(3, aMatrix1.UpperRow());
        Assert.Equal(1, aMatrix1.LowerCol());
        Assert.Equal(4, aMatrix1.UpperCol());

        // Test constructor with initial value
        math_Matrix aMatrix2 = new(2, 4, 3, 5, 2.5);
        Assert.Equal(3, aMatrix2.RowNumber());
        Assert.Equal(3, aMatrix2.ColNumber());

        for (int anI = 2; anI <= 4; anI++)
        {
            for (int aJ = 3; aJ <= 5; aJ++)
            {
                Assert.Equal(2.5, aMatrix2[anI, aJ]);
            }
        }

        // Test copy constructor
        math_Matrix aMatrix3 = new(aMatrix2);
        checkMatricesEqual(aMatrix2, aMatrix3);
    }

    // Tests for initialization and access
    [Fact]
    public void InitAndAccess()
    {
        math_Matrix aMatrix = new(1, 3, 1, 3);

        // Test Init
        aMatrix.Init(5.0);

        for (int anI = 1; anI <= 3; anI++)
        {
            for (int aJ = 1; aJ <= 3; aJ++)
            {
                Assert.Equal(5.0, aMatrix[anI, aJ]);
            }
        }

        // Test direct value access and modification
        aMatrix[2, 2] = 10.0;
        Assert.Equal(10.0, aMatrix[2, 2]);

        // Test Value method
        aMatrix.Value(3, 3) = 15.0;
        Assert.Equal(15.0, aMatrix[3, 3]);
    }

    // Tests for matrix operations
    [Fact]
    public void MatrixOperations()
    {
        math_Matrix aMatrix1 = new(1, 3, 1, 3);
        math_Matrix aMatrix2 = new(1, 3, 1, 3);

        // Initialize matrices
        aMatrix1.Init(2.0);
        aMatrix2.Init(3.0);

        // Test addition
        math_Matrix anAddResult = aMatrix1.Added(aMatrix2);
        for (int anI = 1; anI <= 3; anI++)
        {
            for (int aJ = 1; aJ <= 3; aJ++)
            {
                Assert.Equal(5.0, anAddResult[anI, aJ]);
            }
        }

        // Test subtraction
        math_Matrix aSubResult = aMatrix1.Subtracted(aMatrix2);
        for (int anI = 1; anI <= 3; anI++)
        {
            for (int aJ = 1; aJ <= 3; aJ++)
            {
                Assert.Equal(-1.0, aSubResult[anI, aJ]);
            }
        }

        // Test multiplication by scalar
        math_Matrix aMulResult = aMatrix1.Multiplied(2.5);
        for (int anI = 1; anI <= 3; anI++)
        {
            for (int aJ = 1; aJ <= 3; aJ++)
            {
                Assert.Equal(5.0, aMulResult[anI, aJ]);
            }
        }
        // Test division by scalar
        math_Matrix aDivResult = aMatrix1.Divided(0.5);
        for (int anI = 1; anI <= 3; anI++)
        {
            for (int aJ = 1; aJ <= 3; aJ++)
            {
                Assert.Equal(4.0, aDivResult[anI, aJ]);
            }
        }

        // Test in-place multiplication by scalar using Multiply method
        math_Matrix aInPlaceMul = new(1, 3, 1, 3);
        aInPlaceMul.Init(3.0);
        aInPlaceMul.Multiply(2.0);

        for (int anI = 1; anI <= 3; anI++)
        {
            for (int aJ = 1; aJ <= 3; aJ++)
            {
                Assert.Equal(6.0, aInPlaceMul[anI, aJ]);
            }
        }

        // Test in-place multiplication by scalar using operator*=
        math_Matrix aInPlaceOp = new(1, 3, 1, 3);
        aInPlaceOp.Init(2.0);
        aInPlaceOp *= 3.0;

        for (int anI = 1; anI <= 3; anI++)
        {
            for (int aJ = 1; aJ <= 3; aJ++)
            {
                Assert.Equal(6.0, aInPlaceOp[anI, aJ]);
            }
        }
    }

    // Tests for matrix multiplication
    [Fact]
    public void MatrixMultiplication()
    {
        // Create identity matrix
        math_Matrix anIdentity = new(1, 3, 1, 3);
        anIdentity.Init(0.0);
        anIdentity[1, 1] = 1.0;
        anIdentity[2, 2] = 1.0;
        anIdentity[3, 3] = 1.0;

        // Create test matrix
        math_Matrix aMatrix = new(1, 3, 1, 3);
        for (int anI = 1; anI <= 3; anI++)
        {
            for (int aJ = 1; aJ <= 3; aJ++)
            {
                aMatrix[anI, aJ] = anI + aJ;
            }
        }

        // Test matrix multiplication with identity
        math_Matrix aResult = aMatrix.Multiplied(anIdentity);
        checkMatricesEqual(aResult, aMatrix);

        // Test matrix multiplication
        math_Matrix aMatA = new(1, 2, 1, 3);
        math_Matrix aMatB = new(1, 3, 1, 2);

        aMatA[1, 1] = 1.0;
        aMatA[1, 2] = 2.0;
        aMatA[1, 3] = 3.0;
        aMatA[2, 1] = 4.0;
        aMatA[2, 2] = 5.0;
        aMatA[2, 3] = 6.0;

        aMatB[1, 1] = 7.0;
        aMatB[1, 2] = 8.0;
        aMatB[2, 1] = 9.0;
        aMatB[2, 2] = 10.0;
        aMatB[3, 1] = 11.0;
        aMatB[3, 2] = 12.0;

        math_Matrix aMatC = aMatA.Multiplied(aMatB);

        Assert.Equal(1 * 7 + 2 * 9 + 3 * 11, aMatC[1, 1]);
        Assert.Equal(1 * 8 + 2 * 10 + 3 * 12, aMatC[1, 2]);
        Assert.Equal(4 * 7 + 5 * 9 + 6 * 11, aMatC[2, 1]);
        Assert.Equal(4 * 8 + 5 * 10 + 6 * 12, aMatC[2, 2]);

        // Test in-place matrix multiplication using Multiply method
        math_Matrix aInPlaceMulMat1 = new(1, 2, 1, 2);
        math_Matrix aInPlaceMulMat2 = new(1, 2, 1, 2);

        // Set matrix values
        aInPlaceMulMat1[1, 1] = 1.0;
        aInPlaceMulMat1[1, 2] = 2.0;
        aInPlaceMulMat1[2, 1] = 3.0;
        aInPlaceMulMat1[2, 2] = 4.0;

        aInPlaceMulMat2[1, 1] = 2.0;
        aInPlaceMulMat2[1, 2] = 0.0;
        aInPlaceMulMat2[2, 1] = 1.0;
        aInPlaceMulMat2[2, 2] = 3.0;

        // Store original matrix for comparison
        math_Matrix aOrigMat = new(aInPlaceMulMat1);

        // Perform in-place multiplication
        aInPlaceMulMat1.Multiply(aInPlaceMulMat2);

        // Expected result: [1 2; 3 4] * [2 0; 1 3] = [4 6; 10 12]
        Assert.Equal(1 * 2 + 2 * 1, aInPlaceMulMat1[1, 1]); // 4
        Assert.Equal(1 * 0 + 2 * 3, aInPlaceMulMat1[1, 2]); // 6
        Assert.Equal(3 * 2 + 4 * 1, aInPlaceMulMat1[2, 1]); // 10
        Assert.Equal(3 * 0 + 4 * 3, aInPlaceMulMat1[2, 2]); // 12

        // Test in-place matrix multiplication using operator*=
        math_Matrix aInPlaceOpMat = new(aOrigMat);
        aInPlaceOpMat *= aInPlaceMulMat2;

        // Check same result as with direct Multiply method
        Assert.Equal(4, aInPlaceOpMat[1, 1]);
        Assert.Equal(6, aInPlaceOpMat[1, 2]);
        Assert.Equal(10, aInPlaceOpMat[2, 1]);
        Assert.Equal(12, aInPlaceOpMat[2, 2]);
    }

    // Tests for transpose and inverse operations
    [Fact]
    public void TransposeAndInverse()
    {
        // Test transpose
        math_Matrix aMatrix = new(1, 2, 1, 3);
        aMatrix[1, 1] = 1.0;
        aMatrix[1, 2] = 2.0;
        aMatrix[1, 3] = 3.0;
        aMatrix[2, 1] = 4.0;
        aMatrix[2, 2] = 5.0;
        aMatrix[2, 3] = 6.0;

        math_Matrix aTransposed = aMatrix.Transposed();

        Assert.Equal(aMatrix.ColNumber(), aTransposed.RowNumber());
        Assert.Equal(aMatrix.RowNumber(), aTransposed.ColNumber());
        Assert.Equal(aMatrix[1, 1], aTransposed[1, 1]);
        Assert.Equal(aMatrix[1, 2], aTransposed[2, 1]);
        Assert.Equal(aMatrix[1, 3], aTransposed[3, 1]);
        Assert.Equal(aMatrix[2, 1], aTransposed[1, 2]);
        Assert.Equal(aMatrix[2, 2], aTransposed[2, 2]);
        Assert.Equal(aMatrix[2, 3], aTransposed[3, 2]);

        // Test inverse of a matrix
        math_Matrix anInvertible = new(1, 3, 1, 3);
        anInvertible[1, 1] = 1.0;
        anInvertible[1, 2] = 0.0;
        anInvertible[1, 3] = 0.0;
        anInvertible[2, 1] = 0.0;
        anInvertible[2, 2] = 2.0;
        anInvertible[2, 3] = 0.0;
        anInvertible[3, 1] = 0.0;
        anInvertible[3, 2] = 0.0;
        anInvertible[3, 3] = 4.0;

        math_Matrix anInverse = anInvertible.Inverse();

        Assert.Equal(1.0, anInverse[1, 1]);
        Assert.Equal(0.5, anInverse[2, 2]);
        Assert.Equal(0.25, anInverse[3, 3]);

        // Test identity after multiplication
        math_Matrix anIdentity = anInvertible.Multiplied(anInverse);

        Assert.Equal(1.0, anIdentity[1, 1], Precision.Confusion());
        Assert.Equal(1.0, anIdentity[2, 2], Precision.Confusion());
        Assert.Equal(1.0, anIdentity[3, 3], Precision.Confusion());
        Assert.Equal(0.0, anIdentity[1, 2], Precision.Confusion());
        Assert.Equal(0.0, anIdentity[1, 3], Precision.Confusion());
        Assert.Equal(0.0, anIdentity[2, 1], Precision.Confusion());
        Assert.Equal(0.0, anIdentity[2, 3], Precision.Confusion());
        Assert.Equal(0.0, anIdentity[3, 1], Precision.Confusion());
        Assert.Equal(0.0, anIdentity[3, 2], Precision.Confusion());
    }

    // Tests for determinant calculation
    [Fact]
    public void Determinant()
    {
        // Test determinant of identity matrix
        math_Matrix anIdentity = new(1, 3, 1, 3);
        anIdentity.Init(0.0);
        anIdentity[1, 1] = 1.0;
        anIdentity[2, 2] = 1.0;
        anIdentity[3, 3] = 1.0;

        Assert.Equal(1.0, anIdentity.Determinant(), Precision.Confusion());

        // Test determinant of diagonal matrix
        math_Matrix aDiagonal = new(1, 3, 1, 3);
        aDiagonal.Init(0.0);
        aDiagonal[1, 1] = 2.0;
        aDiagonal[2, 2] = 3.0;
        aDiagonal[3, 3] = 4.0;

        Assert.Equal(24.0, aDiagonal.Determinant(), Precision.Confusion());

        // Test determinant of a non-singular matrix
        math_Matrix aMatrix = new(1, 3, 1, 3);
        aMatrix[1, 1] = 3.0;
        aMatrix[1, 2] = 8.0;
        aMatrix[1, 3] = 5.0;
        aMatrix[2, 1] = 6.0;
        aMatrix[2, 2] = 2.0;
        aMatrix[2, 3] = 7.0;
        aMatrix[3, 1] = 4.0;
        aMatrix[3, 2] = 1.0;
        aMatrix[3, 3] = 9.0;

        // det = 3*(2*9-7*1) - 8*(6*9-7*4) + 5*(6*1-2*4) = 3*11 - 8*26 + 5*-2 = 33 - 208 - 10 = -185
        Assert.Equal(-185.0, aMatrix.Determinant(), Precision.Confusion());
    }

    // Tests for Row and Column operations
    [Fact]
    public void RowAndColumnOperations()
    {
        math_Matrix aMatrix = new(1, 3, 1, 3);
        aMatrix.Init(0.0);

        // Setup matrix rows
        aMatrix[1, 1] = 1.0;
        aMatrix[1, 2] = 2.0;
        aMatrix[1, 3] = 3.0;
        aMatrix[2, 1] = 4.0;
        aMatrix[2, 2] = 5.0;
        aMatrix[2, 3] = 6.0;
        aMatrix[3, 1] = 7.0;
        aMatrix[3, 2] = 8.0;
        aMatrix[3, 3] = 9.0;

        // Test Row method
        math_Vector aRow = aMatrix.Row(2);
        Assert.Equal(3, aRow.Length());
        Assert.Equal(4.0, aRow[1]);
        Assert.Equal(5.0, aRow[2]);
        Assert.Equal(6.0, aRow[3]);

        // Test Col method
        math_Vector aCol = aMatrix.Col(3);
        Assert.Equal(3, aCol.Length());
        Assert.Equal(3.0, aCol[1]);
        Assert.Equal(6.0, aCol[2]);
        Assert.Equal(9.0, aCol[3]);

        // Test SetRow
        math_Vector aNewRow = new(1, 3);
        aNewRow[1] = 10.0;
        aNewRow[2] = 11.0;
        aNewRow[3] = 12.0;
        aMatrix.SetRow(2, aNewRow);

        Assert.Equal(10.0, aMatrix[2, 1]);
        Assert.Equal(11.0, aMatrix[2, 2]);
        Assert.Equal(12.0, aMatrix[2, 3]);

        // Test SetCol
        math_Vector aNewCol = new(1, 3);
        aNewCol[1] = 13.0;
        aNewCol[2] = 14.0;
        aNewCol[3] = 15.0;
        aMatrix.SetCol(1, aNewCol);

        Assert.Equal(13.0, aMatrix[1, 1]);
        Assert.Equal(14.0, aMatrix[2, 1]);
        Assert.Equal(15.0, aMatrix[3, 1]);

        // Test SwapRow
        aMatrix.SwapRow(1, 3);
        Assert.Equal(15.0, aMatrix[1, 1]);
        Assert.Equal(8.0, aMatrix[1, 2]);
        Assert.Equal(9.0, aMatrix[1, 3]);
        Assert.Equal(13.0, aMatrix[3, 1]);
        Assert.Equal(2.0, aMatrix[3, 2]);
        Assert.Equal(3.0, aMatrix[3, 3]);

        // Test SwapCol
        aMatrix.SwapCol(2, 3);
        Assert.Equal(9.0, aMatrix[1, 2]);
        Assert.Equal(8.0, aMatrix[1, 3]);
        Assert.Equal(12.0, aMatrix[2, 2]);
        Assert.Equal(11.0, aMatrix[2, 3]);
        Assert.Equal(3.0, aMatrix[3, 2]);
        Assert.Equal(2.0, aMatrix[3, 3]);
    }

    // Tests for SetDiag method
    [Fact]
    public void SetDiag()
    {
        math_Matrix aMatrix = new(1, 3, 1, 3);
        aMatrix.Init(0.0);

        // Set diagonal elements to 5.0
        aMatrix.SetDiag(5.0);

        Assert.Equal(5.0, aMatrix[1, 1]);
        Assert.Equal(5.0, aMatrix[2, 2]);
        Assert.Equal(5.0, aMatrix[3, 3]);

        // Check non-diagonal elements remain 0
        Assert.Equal(0.0, aMatrix[1, 2]);
        Assert.Equal(0.0, aMatrix[1, 3]);
        Assert.Equal(0.0, aMatrix[2, 1]);
        Assert.Equal(0.0, aMatrix[2, 3]);
        Assert.Equal(0.0, aMatrix[3, 1]);
        Assert.Equal(0.0, aMatrix[3, 2]);
    }

    // Tests for vector-matrix operations
    [Fact]
    public void VectorMatrixOperations()
    {
        // Create test matrix
        math_Matrix aMatrix = new(1, 3, 1, 3);
        aMatrix[1, 1] = 1;
        aMatrix[1, 2] = 2;
        aMatrix[1, 3] = 3;
        aMatrix[2, 1] = 4;
        aMatrix[2, 2] = 5;
        aMatrix[2, 3] = 6;
        aMatrix[3, 1] = 7;
        aMatrix[3, 2] = 8;
        aMatrix[3, 3] = 9;

        // Create test vector
        math_Vector aVector = new(1, 3);
        aVector[1] = 2;
        aVector[2] = 3;
        aVector[3] = 4;

        // Test matrix * vector
        math_Vector aResult = aMatrix * aVector;

        Assert.Equal(3, aResult.Length());
        // For row 1: 1*2 + 2*3 + 3*4 = 2 + 6 + 12 = 20
        Assert.Equal(20, aResult[1]);
        // For row 2: 4*2 + 5*3 + 6*4 = 8 + 15 + 24 = 47
        Assert.Equal(47, aResult[2]);
        // For row 3: 7*2 + 8*3 + 9*4 = 14 + 24 + 36 = 74
        Assert.Equal(74, aResult[3]);

        // Test Multiply method with two vectors
        math_Vector aVec1 = new(1, 2), aVec2 = new(1, 3);
        aVec1[1] = 1;
        aVec1[2] = 2;
        aVec2[1] = 3;
        aVec2[2] = 4;
        aVec2[3] = 5;

        math_Matrix aMat = new(1, 2, 1, 3);
        aMat.Multiply(aVec1, aVec2);

        // The result should be the outer product of v1 and v2
        Assert.Equal(1 * 3, aMat[1, 1]); // aVec1[1] * aVec2(1)
        Assert.Equal(1 * 4, aMat[1, 2]); // aVec1[1] * aVec2(2)
        Assert.Equal(1 * 5, aMat[1, 3]); // aVec1[1] * aVec2(3)
        Assert.Equal(2 * 3, aMat[2, 1]); // aVec1[2] * aVec2(1)
        Assert.Equal(2 * 4, aMat[2, 2]); // aVec1[2] * aVec2(2)
        Assert.Equal(2 * 5, aMat[2, 3]); // aVec1[2] * aVec2(3)
    }

    // Special test to ensure that the matrix multiplication bug is fixed
    // Bug description: In-place matrix multiplication modifies matrix elements
    // that are still needed for the computation of remaining elements
    [Fact]
    public void InPlaceMatrixMultiplication()
    {
        // Create a test matrix with specific values to reveal the bug
        math_Matrix aMatrix = new(1, 2, 1, 2);
        aMatrix[1, 1] = 1.0;
        aMatrix[1, 2] = 2.0;
        aMatrix[2, 1] = 3.0;
        aMatrix[2, 2] = 4.0;

        // Create a copy of the matrix for comparison
        math_Matrix aMatrixCopy = new(aMatrix);

        // Expected result of matrix * matrix (calculated by hand)
        // [1 2] * [1 2] = [7 10]
        // [3 4]   [3 4]   [15 22]
        math_Matrix aExpectedResult = new(1, 2, 1, 2);
        aExpectedResult[1, 1] = 7.0;
        aExpectedResult[1, 2] = 10.0;
        aExpectedResult[2, 1] = 15.0;
        aExpectedResult[2, 2] = 22.0;

        // Non-in-place multiplication result (known to be correct)
        math_Matrix aCorrectResult = aMatrixCopy.Multiplied(aMatrixCopy);

        // Verify that the non-in-place multiplication gives the correct result
        Assert.Equal(7.0, aCorrectResult[1, 1]);
        Assert.Equal(10.0, aCorrectResult[1, 2]);
        Assert.Equal(15.0, aCorrectResult[2, 1]);
        Assert.Equal(22.0, aCorrectResult[2, 2]);

        // Perform in-place multiplication with self
        aMatrix.Multiply(aMatrix);

        // If the bug is fixed, both results should be identical
        Assert.Equal(aCorrectResult[1, 1], aMatrix[1, 1], Precision.Confusion());
        Assert.Equal(aCorrectResult[1, 2], aMatrix[1, 2], Precision.Confusion());
        Assert.Equal(aCorrectResult[2, 1], aMatrix[2, 1], Precision.Confusion());
        Assert.Equal(aCorrectResult[2, 2], aMatrix[2, 2], Precision.Confusion());

        // Also test the operator*= version with self
        aMatrixCopy *= aMatrixCopy;

        // Verify the operator*= with self gives correct results
        Assert.Equal(aCorrectResult[1, 1], aMatrixCopy[1, 1], Precision.Confusion());
        Assert.Equal(aCorrectResult[1, 2], aMatrixCopy[1, 2], Precision.Confusion());
        Assert.Equal(aCorrectResult[2, 1], aMatrixCopy[2, 1], Precision.Confusion());
        Assert.Equal(aCorrectResult[2, 2], aMatrixCopy[2, 2], Precision.Confusion());

        // Create two different matrices
        math_Matrix aMatrixA = new(1, 2, 1, 2);
        aMatrixA[1, 1] = 1.0;
        aMatrixA[1, 2] = 2.0;
        aMatrixA[2, 1] = 3.0;
        aMatrixA[2, 2] = 4.0;

        math_Matrix aMatrixB = new(1, 2, 1, 2);
        aMatrixB[1, 1] = 5.0;
        aMatrixB[1, 2] = 6.0;
        aMatrixB[2, 1] = 7.0;
        aMatrixB[2, 2] = 8.0;

        // Create copies for non-in-place calculation
        math_Matrix aMatrixACopy = new(aMatrixA);
        math_Matrix aMatrixBCopy = new(aMatrixB);

        // Calculate expected result using non-in-place multiplication
        math_Matrix aExpectedAB = aMatrixACopy.Multiplied(aMatrixBCopy);

        // Expected result: [1 2] * [5 6] = [19 22]
        //                  [3 4]   [7 8]   [43 50]
        Assert.Equal(19.0, aExpectedAB[1, 1]);
        Assert.Equal(22.0, aExpectedAB[1, 2]);
        Assert.Equal(43.0, aExpectedAB[2, 1]);
        Assert.Equal(50.0, aExpectedAB[2, 2]);

        // Test in-place multiplication with different matrix
        aMatrixA.Multiply(aMatrixB);

        // Check results match expected values
        Assert.Equal(aExpectedAB[1, 1], aMatrixA[1, 1], Precision.Confusion());
        Assert.Equal(aExpectedAB[1, 2], aMatrixA[1, 2], Precision.Confusion());
        Assert.Equal(aExpectedAB[2, 1], aMatrixA[2, 1], Precision.Confusion());
        Assert.Equal(aExpectedAB[2, 2], aMatrixA[2, 2], Precision.Confusion());

        // Test operator*= with different matrices
        aMatrixACopy *= aMatrixBCopy;

        // Check results match expected values
        Assert.Equal(aExpectedAB[1, 1], aMatrixACopy[1, 1], Precision.Confusion());
        Assert.Equal(aExpectedAB[1, 2], aMatrixACopy[1, 2], Precision.Confusion());
        Assert.Equal(aExpectedAB[2, 1], aMatrixACopy[2, 1], Precision.Confusion());
        Assert.Equal(aExpectedAB[2, 2], aMatrixACopy[2, 2], Precision.Confusion());
    }

    // Tests for Move Semantics
    [Fact]
    public void MoveSemantics()
    {
        // --- Move Constructor ---

        // Large matrix (heap allocated)
        int         aRows = 10;
        int         aCols = 10;
        math_Matrix aMat1 = new(1, aRows, 1, aCols);
        aMat1.Init(1.0);
        aMat1[1, 1] = 2.0;

        // Move aMat1 to aMat2. C# has no move constructor: the move is a transfer of the reference, so the
        // OCCT check that the moved-from matrix is empty doesn't apply.
        math_Matrix aMat2 = aMat1;

        Assert.Equal(aRows, aMat2.RowNumber());
        Assert.Equal(aCols, aMat2.ColNumber());
        Assert.Equal(2.0, aMat2[1, 1]);

        // Small matrix (buffer allocated)
        int         aSmallRows = 4;
        int         aSmallCols = 4;
        math_Matrix aSmallMat1 = new(1, aSmallRows, 1, aSmallCols);
        aSmallMat1.Init(1.0);

        // Move aSmallMat1 to aSmallMat2 (OCCT copies because of its inline buffer; the port copies)
        math_Matrix aSmallMat2 = new(aSmallMat1);

        Assert.Equal(aSmallRows, aSmallMat2.RowNumber());
        Assert.Equal(1.0, aSmallMat2[1, 1]);

        // Source remains valid for buffer-based matrix
        Assert.Equal(aSmallRows, aSmallMat1.RowNumber());
        Assert.Equal(1.0, aSmallMat1[1, 1]);

        // --- Move Assignment ---

        // Large matrix move assignment
        math_Matrix aMatAssign1 = new(1, aRows, 1, aCols);
        aMatAssign1.Init(5.0);

        math_Matrix aMatAssign2 = new(1, aRows, 1, aCols);
        aMatAssign2.Init(0.0);

        // C# has no move assignment: the reference is transferred, so the moved-from check doesn't apply.
        aMatAssign2 = aMatAssign1;

        Assert.Equal(5.0, aMatAssign2[1, 1]);
    }
}
