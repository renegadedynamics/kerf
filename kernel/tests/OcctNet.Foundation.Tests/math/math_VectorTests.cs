// Port of: src/FoundationClasses/TKMath/GTests/math_Vector_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Numerics;

namespace OcctNet.Foundation.Tests;

public class math_VectorTest
{
    // Helper function for comparing vectors with tolerance
    private static void checkVectorsEqual(math_Vector theV1, math_Vector theV2, double theTolerance = 1.0e-7 /* Precision::Confusion() */)
    {
        Assert.Equal(theV2.Length(), theV1.Length());
        Assert.Equal(theV2.Lower(), theV1.Lower());
        Assert.Equal(theV2.Upper(), theV1.Upper());

        for (int anI = theV1.Lower(); anI <= theV1.Upper(); anI++)
        {
            Assert.Equal(theV2[anI], theV1[anI], theTolerance);
        }
    }

    // Tests for constructors
    [Fact]
    public void Constructors()
    {
        // Test standard constructor
        math_Vector aVec1 = new(1, 5);
        Assert.Equal(5, aVec1.Length());
        Assert.Equal(1, aVec1.Lower());
        Assert.Equal(5, aVec1.Upper());

        // Test constructor with initial value
        math_Vector aVec2 = new(-2, 3, 2.5);
        Assert.Equal(6, aVec2.Length());
        Assert.Equal(-2, aVec2.Lower());
        Assert.Equal(3, aVec2.Upper());

        for (int anI = -2; anI <= 3; anI++)
        {
            Assert.Equal(2.5, aVec2[anI]);
        }

        // Test constructor with external array
        double[] anArray = [1.0, 2.0, 3.0, 4.0, 5.0];
        math_Vector aVec3 = new(anArray, 0, 4);
        Assert.Equal(5, aVec3.Length());
        Assert.Equal(0, aVec3.Lower());
        Assert.Equal(4, aVec3.Upper());

        for (int anI = 0; anI <= 4; anI++)
        {
            Assert.Equal(anArray[anI], aVec3[anI]);
        }

        // Test copy constructor
        math_Vector aVec4 = new(aVec2);
        checkVectorsEqual(aVec2, aVec4);
    }

    // Tests for gp_XY and gp_XYZ constructors
    [Fact]
    public void GeometryConstructors()
    {
        // Test gp_XY constructor
        Xy anXY = new(3.5, 4.5);
        math_Vector aVecXY = new(anXY);
        Assert.Equal(2, aVecXY.Length());
        Assert.Equal(1, aVecXY.Lower());
        Assert.Equal(2, aVecXY.Upper());
        Assert.Equal(3.5, aVecXY[1]);
        Assert.Equal(4.5, aVecXY[2]);

        // Test gp_XYZ constructor
        Xyz anXYZ = new(1.1, 2.2, 3.3);
        math_Vector aVecXYZ = new(anXYZ);
        Assert.Equal(3, aVecXYZ.Length());
        Assert.Equal(1, aVecXYZ.Lower());
        Assert.Equal(3, aVecXYZ.Upper());
        Assert.Equal(1.1, aVecXYZ[1]);
        Assert.Equal(2.2, aVecXYZ[2]);
        Assert.Equal(3.3, aVecXYZ[3]);
    }

    // Tests for initialization and access
    [Fact]
    public void InitAndAccess()
    {
        math_Vector aVec = new(1, 4);

        // Test Init
        aVec.Init(7.0);
        for (int anI = 1; anI <= 4; anI++)
        {
            Assert.Equal(7.0, aVec[anI]);
        }

        // Test direct value access and modification
        aVec[2] = 15.0;
        Assert.Equal(15.0, aVec[2]);

        // Test Value method
        aVec.Value(3) = 25.0;
        Assert.Equal(25.0, aVec[3]);
        Assert.Equal(25.0, aVec.Value(3));
    }

    // Tests for vector properties
    [Fact]
    public void VectorProperties()
    {
        math_Vector aVec = new(1, 4);
        aVec[1] = 3.0;
        aVec[2] = 4.0;
        aVec[3] = 0.0;
        aVec[4] = -2.0;

        // Test Norm (should be Math.Sqrt(3^2 + 4^2 + 0^2 + (-2)^2) = Math.Sqrt(29))
        double anExpectedNorm = Math.Sqrt(29.0);
        Assert.Equal(anExpectedNorm, aVec.Norm(), global::Kerf.Engine.Foundation.Precision.Confusion());

        // Test Norm2 (should be 29)
        Assert.Equal(29.0, aVec.Norm2(), global::Kerf.Engine.Foundation.Precision.Confusion());

        // Test Max (should return index 2, value 4.0)
        Assert.Equal(2, aVec.Max());

        // Test Min (should return index 4, value -2.0)
        Assert.Equal(4, aVec.Min());
    }

    // Tests for normalization
    [Fact]
    public void Normalization()
    {
        math_Vector aVec = new(1, 3);
        aVec[1] = 3.0;
        aVec[2] = 4.0;
        aVec[3] = 0.0;

        double anOriginalNorm = aVec.Norm();
        Assert.Equal(5.0, anOriginalNorm, global::Kerf.Engine.Foundation.Precision.Confusion());

        // Test Normalized (creates new vector)
        math_Vector aNormalizedVec = aVec.Normalized();
        Assert.Equal(1.0, aNormalizedVec.Norm(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(3.0 / 5.0, aNormalizedVec[1], global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(4.0 / 5.0, aNormalizedVec[2], global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aNormalizedVec[3], global::Kerf.Engine.Foundation.Precision.Confusion());

        // Original vector should remain unchanged
        Assert.Equal(5.0, aVec.Norm(), global::Kerf.Engine.Foundation.Precision.Confusion());

        // Test Normalize (modifies in-place)
        aVec.Normalize();
        Assert.Equal(1.0, aVec.Norm(), global::Kerf.Engine.Foundation.Precision.Confusion());
        checkVectorsEqual(aVec, aNormalizedVec);
    }

    // Tests for normalization exception
    [Fact]
    public void ZeroVectorHandling()
    {
        math_Vector aZeroVec = new(1, 3, 0.0);

        // Test behavior with zero vector - verify it's actually zero
        Assert.Equal(0.0, aZeroVec.Norm());

        // Test with non-zero vector for comparison
        math_Vector aNonZeroVec = new(1, 3);
        aNonZeroVec[1] = 1.0;
        aNonZeroVec[2] = 0.0;
        aNonZeroVec[3] = 0.0;

        Assert.Equal(1.0, aNonZeroVec.Norm());
        aNonZeroVec.Normalize();
        Assert.Equal(1.0, aNonZeroVec.Norm());
    }

    // Tests for inversion
    [Fact]
    public void Inversion()
    {
        math_Vector aVec = new(1, 5);
        aVec[1] = 1.0;
        aVec[2] = 2.0;
        aVec[3] = 3.0;
        aVec[4] = 4.0;
        aVec[5] = 5.0;

        // Test Inverse (creates new vector)
        math_Vector anInverseVec = aVec.Inverse();
        Assert.Equal(5.0, anInverseVec[1]);
        Assert.Equal(4.0, anInverseVec[2]);
        Assert.Equal(3.0, anInverseVec[3]);
        Assert.Equal(2.0, anInverseVec[4]);
        Assert.Equal(1.0, anInverseVec[5]);

        // Original vector should remain unchanged
        Assert.Equal(1.0, aVec[1]);
        Assert.Equal(5.0, aVec[5]);

        // Test Invert (modifies in-place)
        aVec.Invert();
        checkVectorsEqual(aVec, anInverseVec);
    }

    // Tests for scalar operations
    [Fact]
    public void ScalarOperations()
    {
        math_Vector aVec = new(1, 3);
        aVec[1] = 2.0;
        aVec[2] = 4.0;
        aVec[3] = 6.0;

        // Test multiplication by scalar
        math_Vector aMulResult = aVec.Multiplied(2.5);
        Assert.Equal(5.0, aMulResult[1]);
        Assert.Equal(10.0, aMulResult[2]);
        Assert.Equal(15.0, aMulResult[3]);

        // Test TMultiplied (should be same as Multiplied for scalar)
        math_Vector aTMulResult = aVec.TMultiplied(2.5);
        checkVectorsEqual(aMulResult, aTMulResult);

        // Test in-place multiplication
        aVec.Multiply(0.5);
        Assert.Equal(1.0, aVec[1]);
        Assert.Equal(2.0, aVec[2]);
        Assert.Equal(3.0, aVec[3]);

        // Test operator*= for scalar
        aVec *= 3.0;
        Assert.Equal(3.0, aVec[1]);
        Assert.Equal(6.0, aVec[2]);
        Assert.Equal(9.0, aVec[3]);

        // Test division by scalar
        math_Vector aDivResult = aVec.Divided(3.0);
        Assert.Equal(1.0, aDivResult[1]);
        Assert.Equal(2.0, aDivResult[2]);
        Assert.Equal(3.0, aDivResult[3]);

        // Test in-place division
        aVec.Divide(3.0);
        checkVectorsEqual(aVec, aDivResult);

        // Test operator/= for scalar
        aVec *= 6.0; // Set to [6, 12, 18]
        aVec /= 2.0;
        Assert.Equal(3.0, aVec[1]);
        Assert.Equal(6.0, aVec[2]);
        Assert.Equal(9.0, aVec[3]);
    }

    // Tests for division by zero
    [Fact]
    public void DivisionOperations()
    {
        math_Vector aVec = new(1, 3, 2.0);

        // Test normal division operations
        aVec.Divide(2.0);
        Assert.Equal(1.0, aVec[1]);

        math_Vector aVec2 = new(1, 3, 4.0);
        math_Vector aResult = aVec2.Divided(2.0);
        Assert.Equal(2.0, aResult[1]);
    }

    // Tests for vector addition and subtraction
    [Fact]
    public void VectorAdditionSubtraction()
    {
        math_Vector aVec1 = new(1, 3);
        aVec1[1] = 1.0;
        aVec1[2] = 2.0;
        aVec1[3] = 3.0;

        math_Vector aVec2 = new(1, 3);
        aVec2[1] = 4.0;
        aVec2[2] = 5.0;
        aVec2[3] = 6.0;

        // Test Added
        math_Vector anAddResult = aVec1.Added(aVec2);
        Assert.Equal(5.0, anAddResult[1]);
        Assert.Equal(7.0, anAddResult[2]);
        Assert.Equal(9.0, anAddResult[3]);

        // Test operator+
        math_Vector anAddResult2 = aVec1 + aVec2;
        checkVectorsEqual(anAddResult, anAddResult2);

        // Test Subtracted
        math_Vector aSubResult = aVec1.Subtracted(aVec2);
        Assert.Equal(-3.0, aSubResult[1]);
        Assert.Equal(-3.0, aSubResult[2]);
        Assert.Equal(-3.0, aSubResult[3]);

        // Test operator-
        math_Vector aSubResult2 = aVec1 - aVec2;
        checkVectorsEqual(aSubResult, aSubResult2);

        // Test in-place Add
        math_Vector aVecCopy1 = new(aVec1);
        aVecCopy1.Add(aVec2);
        checkVectorsEqual(aVecCopy1, anAddResult);

        // Test operator+=
        math_Vector aVecCopy2 = new(aVec1);
        aVecCopy2 += aVec2;
        checkVectorsEqual(aVecCopy2, anAddResult);

        // Test in-place Subtract
        math_Vector aVecCopy3 = new(aVec1);
        aVecCopy3.Subtract(aVec2);
        checkVectorsEqual(aVecCopy3, aSubResult);

        // Test operator-=
        math_Vector aVecCopy4 = new(aVec1);
        aVecCopy4 -= aVec2;
        checkVectorsEqual(aVecCopy4, aSubResult);
    }

    // Tests for vector operations with different bounds
    [Fact]
    public void VectorOperationsDifferentBounds()
    {
        math_Vector aVec1 = new(0, 2);
        aVec1[0] = 1.0;
        aVec1[1] = 2.0;
        aVec1[2] = 3.0;

        math_Vector aVec2 = new(-1, 1);
        aVec2[-1] = 4.0;
        aVec2[0]  = 5.0;
        aVec2[1]  = 6.0;

        // Should work fine - same length, different bounds
        math_Vector anAddResult = aVec1.Added(aVec2);
        Assert.Equal(5.0, anAddResult[0]); // 1.0 + 4.0
        Assert.Equal(7.0, anAddResult[1]); // 2.0 + 5.0
        Assert.Equal(9.0, anAddResult[2]); // 3.0 + 6.0
    }

    // Tests for dimension errors

    // Tests for dot product
    [Fact]
    public void DotProduct()
    {
        math_Vector aVec1 = new(1, 3);
        aVec1[1] = 1.0;
        aVec1[2] = 2.0;
        aVec1[3] = 3.0;

        math_Vector aVec2 = new(1, 3);
        aVec2[1] = 4.0;
        aVec2[2] = 5.0;
        aVec2[3] = 6.0;

        // Test dot product (1*4 + 2*5 + 3*6 = 4 + 10 + 18 = 32)
        double aDotProduct = aVec1.Multiplied(aVec2);
        Assert.Equal(32.0, aDotProduct);

        // Test operator*
        double aDotProduct2 = aVec1 * aVec2;
        Assert.Equal(aDotProduct2, aDotProduct);
    }

    // Tests for Set operation
    [Fact]
    public void SetOperation()
    {
        math_Vector aVec = new(1, 6);
        aVec.Init(0.0);

        math_Vector aSubVec = new(1, 3);
        aSubVec[1] = 10.0;
        aSubVec[2] = 20.0;
        aSubVec[3] = 30.0;

        // Set elements from index 2 to 4
        aVec.Set(2, 4, aSubVec);

        Assert.Equal(0.0, aVec[1]);
        Assert.Equal(10.0, aVec[2]);
        Assert.Equal(20.0, aVec[3]);
        Assert.Equal(30.0, aVec[4]);
        Assert.Equal(0.0, aVec[5]);
        Assert.Equal(0.0, aVec[6]);
    }

    // Tests for Slice operation
    [Fact]
    public void SliceOperation()
    {
        math_Vector aVec = new(1, 6);
        aVec[1] = 10.0;
        aVec[2] = 20.0;
        aVec[3] = 30.0;
        aVec[4] = 40.0;
        aVec[5] = 50.0;
        aVec[6] = 60.0;

        // Test normal slice (ascending)
        math_Vector aSlice1 = aVec.Slice(2, 4);
        Assert.Equal(3, aSlice1.Length());
        Assert.Equal(2, aSlice1.Lower());
        Assert.Equal(4, aSlice1.Upper());
        Assert.Equal(20.0, aSlice1[2]);
        Assert.Equal(30.0, aSlice1[3]);
        Assert.Equal(40.0, aSlice1[4]);

        // Test reverse slice (descending indices)
        math_Vector aSlice2 = aVec.Slice(4, 2);
        Assert.Equal(3, aSlice2.Length());
        Assert.Equal(2, aSlice2.Lower());
        Assert.Equal(4, aSlice2.Upper());
        Assert.Equal(20.0, aSlice2[2]); // Copy from original aVec[2]
        Assert.Equal(30.0, aSlice2[3]); // Copy from original aVec[3]
        Assert.Equal(40.0, aSlice2[4]); // Copy from original aVec[4]
    }

    // Tests for vector-matrix operations
    [Fact]
    public void VectorMatrixOperations()
    {
        // Create test matrix [2x3]
        Matrix aMat = new(1, 2, 1, 3);
        aMat[1, 1] = 1.0;
        aMat[1, 2] = 2.0;
        aMat[1, 3] = 3.0;
        aMat[2, 1] = 4.0;
        aMat[2, 2] = 5.0;
        aMat[2, 3] = 6.0;

        // Create test vectors
        math_Vector aVec1 = new(1, 2); // For left multiplication
        aVec1[1] = 2.0;
        aVec1[2] = 3.0;

        math_Vector aVec2 = new(1, 3); // For right multiplication
        aVec2[1] = 1.0;
        aVec2[2] = 2.0;
        aVec2[3] = 3.0;

        // Test vector * matrix (should produce vector of size 3)
        math_Vector aResult1 = aVec1.Multiplied(aMat);
        Assert.Equal(3, aResult1.Length());
        // aResult1 = [2, 3] * [[1,2,3],[4,5,6]] = [2*1+3*4, 2*2+3*5, 2*3+3*6] = [14, 19, 24]
        Assert.Equal(14.0, aResult1[1]);
        Assert.Equal(19.0, aResult1[2]);
        Assert.Equal(24.0, aResult1[3]);

        // Test vector multiplication with matrix using Multiply method
        math_Vector aResult2 = new(1, 3);
        aResult2.Multiply(aVec1, aMat);
        checkVectorsEqual(aResult1, aResult2);

        // Test matrix * vector using Multiply method
        math_Vector aResult3 = new(1, 2);
        aResult3.Multiply(aMat, aVec2);
        // aResult3 = [[1,2,3],[4,5,6]] * [1,2,3] = [1*1+2*2+3*3, 4*1+5*2+6*3] = [14, 32]
        Assert.Equal(14.0, aResult3[1]);
        Assert.Equal(32.0, aResult3[2]);
    }

    // Tests for transpose matrix operations
    [Fact]
    public void TransposeMatrixOperations()
    {
        // Create test matrix [2x3]
        Matrix aMat = new(1, 2, 1, 3);
        aMat[1, 1] = 1.0;
        aMat[1, 2] = 2.0;
        aMat[1, 3] = 3.0;
        aMat[2, 1] = 4.0;
        aMat[2, 2] = 5.0;
        aMat[2, 3] = 6.0;

        math_Vector aVec1 = new(1, 2);
        aVec1[1] = 2.0;
        aVec1[2] = 3.0;

        math_Vector aVec2 = new(1, 3);
        aVec2[1] = 1.0;
        aVec2[2] = 2.0;
        aVec2[3] = 3.0;

        // Test TMultiply (matrix^T * vector)
        math_Vector aResult1 = new(1, 3);
        aResult1.TMultiply(aMat, aVec1);
        // aMat^T * aVec1 = [[1,4],[2,5],[3,6]] * [2,3] = [1*2+4*3, 2*2+5*3, 3*2+6*3] = [14, 19, 24]
        Assert.Equal(14.0, aResult1[1]);
        Assert.Equal(19.0, aResult1[2]);
        Assert.Equal(24.0, aResult1[3]);

        // Test TMultiply (vector * matrix^T)
        math_Vector aResult2 = new(1, 2);
        aResult2.TMultiply(aVec2, aMat);
        // aVec2 * aMat^T = [1,2,3] * [[1,4],[2,5],[3,6]] = [1*1+2*2+3*3, 1*4+2*5+3*6] = [14, 32]
        Assert.Equal(14.0, aResult2[1]);
        Assert.Equal(32.0, aResult2[2]);
    }

    // Tests for matrix dimension errors

    // Tests for three-operand operations
    [Fact]
    public void ThreeOperandOperations()
    {
        math_Vector aVec1 = new(1, 3);
        aVec1[1] = 1.0;
        aVec1[2] = 2.0;
        aVec1[3] = 3.0;

        math_Vector aVec2 = new(1, 3);
        aVec2[1] = 4.0;
        aVec2[2] = 5.0;
        aVec2[3] = 6.0;

        math_Vector aResult = new(1, 3);

        // Test Add(left, right)
        aResult.Add(aVec1, aVec2);
        Assert.Equal(5.0, aResult[1]);
        Assert.Equal(7.0, aResult[2]);
        Assert.Equal(9.0, aResult[3]);

        // Test Subtract(left, right)
        aResult.Subtract(aVec1, aVec2);
        Assert.Equal(-3.0, aResult[1]);
        Assert.Equal(-3.0, aResult[2]);
        Assert.Equal(-3.0, aResult[3]);

        // Test Multiply(scalar, vector)
        aResult.Multiply(2.5, aVec1);
        Assert.Equal(2.5, aResult[1]);
        Assert.Equal(5.0, aResult[2]);
        Assert.Equal(7.5, aResult[3]);
    }

    // Tests for Opposite operation
    [Fact]
    public void OppositeOperation()
    {
        math_Vector aVec = new(1, 3);
        aVec[1] = 1.0;
        aVec[2] = -2.0;
        aVec[3] = 3.0;

        math_Vector anOpposite = aVec.Opposite();
        Assert.Equal(-1.0, anOpposite[1]);
        Assert.Equal(2.0, anOpposite[2]);
        Assert.Equal(-3.0, anOpposite[3]);

        // Test unary minus operator
        math_Vector anOpposite2 = -aVec;
        checkVectorsEqual(anOpposite, anOpposite2);
    }

    // Tests for assignment operations
    [Fact]
    public void AssignmentOperations()
    {
        math_Vector aVec1 = new(1, 3);
        aVec1[1] = 1.0;
        aVec1[2] = 2.0;
        aVec1[3] = 3.0;

        math_Vector aVec2 = new(1, 3);
        aVec2.Init(0.0);

        // Test Initialized
        aVec2.Initialized(aVec1);
        checkVectorsEqual(aVec1, aVec2);

        // Test operator=
        math_Vector aVec3 = new(1, 3);
        aVec3.Initialized(aVec1);
        checkVectorsEqual(aVec1, aVec3);
    }

    // Tests for friend operators
    [Fact]
    public void FriendOperators()
    {
        math_Vector aVec = new(1, 3);
        aVec[1] = 2.0;
        aVec[2] = 4.0;
        aVec[3] = 6.0;

        // Test scalar * vector
        math_Vector aResult1 = 3.0 * aVec;
        Assert.Equal(6.0, aResult1[1]);
        Assert.Equal(12.0, aResult1[2]);
        Assert.Equal(18.0, aResult1[3]);

        // Test vector * scalar
        math_Vector aResult2 = aVec * 2.5;
        Assert.Equal(5.0, aResult2[1]);
        Assert.Equal(10.0, aResult2[2]);
        Assert.Equal(15.0, aResult2[3]);

        // Test vector / scalar
        math_Vector aResult3 = aVec / 2.0;
        Assert.Equal(1.0, aResult3[1]);
        Assert.Equal(2.0, aResult3[2]);
        Assert.Equal(3.0, aResult3[3]);
    }

    // Tests for edge cases and boundary conditions
    [Fact]
    public void EdgeCases()
    {
        // Test single element vector
        math_Vector aSingleVec = new(5, 5, 42.0);
        Assert.Equal(1, aSingleVec.Length());
        Assert.Equal(42.0, aSingleVec[5]);
        Assert.Equal(5, aSingleVec.Max());
        Assert.Equal(5, aSingleVec.Min());

        // Test negative indices
        math_Vector aNegVec = new(-2, 1);
        aNegVec[-2] = 10.0;
        aNegVec[-1] = 20.0;
        aNegVec[0]  = 30.0;
        aNegVec[1]  = 40.0;

        Assert.Equal(4, aNegVec.Length());
        Assert.Equal(-2, aNegVec.Lower());
        Assert.Equal(1, aNegVec.Upper());
        Assert.Equal(1, aNegVec.Max());
        Assert.Equal(-2, aNegVec.Min());
    }

    // Tests for Move Semantics
    [Fact]
    public void MoveSemantics()
    {
        // --- Move Constructor ---

        // Large vector (heap allocated)
        int         aLen = 100;
        math_Vector aVec1 = new(1, aLen);
        for (int i = 1; i <= aLen; ++i)
        {
            aVec1[i] = (double)(i);
        }

        // Move aVec1 to aVec2. C# has no move constructor: the move is a transfer of the reference, so the
        // OCCT check that the moved-from vector is empty doesn't apply.
        math_Vector aVec2 = aVec1;

        Assert.Equal(aLen, aVec2.Length());
        Assert.Equal(1.0, aVec2[1]);
        Assert.Equal((double)(aLen), aVec2[aLen]);

        // Small vector (buffer allocated)
        int         aSmallLen = 10;
        math_Vector aSmallVec1 = new(1, aSmallLen);
        for (int i = 1; i <= aSmallLen; ++i)
        {
            aSmallVec1[i] = (double)(i);
        }

        // Move aSmallVec1 to aSmallVec2 (OCCT copies because of its inline buffer; the port copies)
        math_Vector aSmallVec2 = new(aSmallVec1);

        Assert.Equal(aSmallLen, aSmallVec2.Length());
        Assert.Equal(1.0, aSmallVec2[1]);

        // Source remains valid for buffer-based vector
        Assert.Equal(aSmallLen, aSmallVec1.Length());
        Assert.Equal(1.0, aSmallVec1[1]);

        // --- Move Assignment ---

        // Large vector move assignment
        math_Vector aVecAssign1 = new(1, aLen);
        for (int i = 1; i <= aLen; ++i)
        {
            aVecAssign1[i] = (double)(i);
        }

        math_Vector aVecAssign2 = new(1, aLen);
        aVecAssign2.Init(0.0);

        // C# has no move assignment: the reference is transferred, so the moved-from check doesn't apply.
        aVecAssign2 = aVecAssign1;

        Assert.Equal(aLen, aVecAssign2.Length());
        Assert.Equal(1.0, aVecAssign2[1]);
    }

    // Tests for Resize operation
    [Fact]
    public void Resize_StackToStack_SameSize()
    {
        // Small vector that fits in stack buffer (THE_BUFFER_SIZE = 32)
        math_Vector aVec = new(1, 10);
        for (int i = 1; i <= 10; ++i)
        {
            aVec[i] = (double)(i);
        }

        // Resize to same size - data should be preserved
        aVec.Resize(10);

        Assert.Equal(10, aVec.Length());
        Assert.Equal(1, aVec.Lower());
        Assert.Equal(10, aVec.Upper());
        for (int i = 1; i <= 10; ++i)
        {
            Assert.Equal((double)(i), aVec[i]);
        }
    }

    [Fact]
    public void Resize_StackToStack_Grow()
    {
        // Small vector
        math_Vector aVec = new(1, 5);
        for (int i = 1; i <= 5; ++i)
        {
            aVec[i] = (double)(i * 10);
        }

        // Grow but still within stack buffer
        aVec.Resize(20);

        Assert.Equal(20, aVec.Length());
        Assert.Equal(1, aVec.Lower());
        Assert.Equal(20, aVec.Upper());

        // Original data should be preserved
        for (int i = 1; i <= 5; ++i)
        {
            Assert.Equal((double)(i * 10), aVec[i]);
        }
    }

    [Fact]
    public void Resize_StackToStack_Shrink()
    {
        // Small vector
        math_Vector aVec = new(1, 20);
        for (int i = 1; i <= 20; ++i)
        {
            aVec[i] = (double)(i);
        }

        // Shrink but still within stack buffer
        aVec.Resize(10);

        Assert.Equal(10, aVec.Length());
        Assert.Equal(1, aVec.Lower());
        Assert.Equal(10, aVec.Upper());

        // Data within new range should be preserved
        for (int i = 1; i <= 10; ++i)
        {
            Assert.Equal((double)(i), aVec[i]);
        }
    }

    [Fact]
    public void Resize_StackToHeap()
    {
        // Small vector that fits in stack
        math_Vector aVec = new(1, 20);
        for (int i = 1; i <= 20; ++i)
        {
            aVec[i] = (double)(i);
        }

        // Resize to larger than stack buffer (>32)
        aVec.Resize(50);

        Assert.Equal(50, aVec.Length());
        Assert.Equal(1, aVec.Lower());
        Assert.Equal(50, aVec.Upper());

        // Original data should be preserved
        for (int i = 1; i <= 20; ++i)
        {
            Assert.Equal((double)(i), aVec[i]);
        }
    }

    [Fact]
    public void Resize_HeapToStack()
    {
        // Large vector on heap
        math_Vector aVec = new(1, 50);
        for (int i = 1; i <= 50; ++i)
        {
            aVec[i] = (double)(i);
        }

        // Resize to fit in stack buffer
        aVec.Resize(20);

        Assert.Equal(20, aVec.Length());
        Assert.Equal(1, aVec.Lower());
        Assert.Equal(20, aVec.Upper());

        // Data within new range should be preserved
        for (int i = 1; i <= 20; ++i)
        {
            Assert.Equal((double)(i), aVec[i]);
        }
    }

    [Fact]
    public void Resize_HeapToHeap()
    {
        // Large vector on heap
        math_Vector aVec = new(1, 50);
        for (int i = 1; i <= 50; ++i)
        {
            aVec[i] = (double)(i);
        }

        // Resize to different heap size
        aVec.Resize(100);

        Assert.Equal(100, aVec.Length());
        Assert.Equal(1, aVec.Lower());
        Assert.Equal(100, aVec.Upper());

        // Original data should be preserved
        for (int i = 1; i <= 50; ++i)
        {
            Assert.Equal((double)(i), aVec[i]);
        }
    }

    [Fact]
    public void Resize_NegativeLowerBound()
    {
        // Vector with negative lower bound
        math_Vector aVec = new(-5, 5);
        for (int i = -5; i <= 5; ++i)
        {
            aVec[i] = (double)(i);
        }

        // Resize - lower bound preserved
        aVec.Resize(8);

        Assert.Equal(8, aVec.Length());
        Assert.Equal(-5, aVec.Lower());
        Assert.Equal(2, aVec.Upper());

        // Original data should be preserved
        for (int i = -5; i <= 2; ++i)
        {
            Assert.Equal((double)(i), aVec[i]);
        }
    }

    // OCC524: math_Vector::Multiply(vector, matrix) and TMultiply(vector, matrix).
    // Reference values from the original Draw Harness test:
    //   Vector(1..6) filled with 5.0, Matrix(1..6, 1..6) filled with 4.0
    //   Vector1 = Vector * Matrix  -> each component = 6 * 5 * 4 = 120
    //   After Matrix(2,1) += 1 (-> 5):
    //   Vector2 = Vector * Matrix^T -> component 2 = 5*5 + 5*4*5 = 25+100 = 125, rest = 120

    [Fact]
    public void Multiply_RowVectorByMatrix_AllComponentsEqual()
    {
        int   aLow = 1, aHigh = 6;
        math_Vector aVec = new(aLow, aHigh, 5.0);
        Matrix aMat = new(aLow, aHigh, aLow, aHigh, 4.0);
        math_Vector aResult = new(aLow, aHigh);

        aResult.Multiply(aVec, aMat);

        for (int i = aLow; i <= aHigh; ++i)
        {
            Assert.Equal(120.0, aResult[i]);
        }
    }

    [Fact]
    public void TMultiply_RowVectorByTransposedMatrix_ModifiedComponent()
    {
        int   aLow = 1, aHigh = 6;
        math_Vector aVec = new(aLow, aHigh, 5.0);
        Matrix aMat = new(aLow, aHigh, aLow, aHigh, 4.0);
        math_Vector aResult = new(aLow, aHigh);

        // Modify one element to create an asymmetry in the transposed multiply
        aMat[aLow + 1, aLow] += 1.0;

        aResult.TMultiply(aVec, aMat);

        Assert.Equal(120.0, aResult[1]);
        Assert.Equal(125.0, aResult[2]);
        for (int i = 3; i <= aHigh; ++i)
        {
            Assert.Equal(120.0, aResult[i]);
        }
    }
}
