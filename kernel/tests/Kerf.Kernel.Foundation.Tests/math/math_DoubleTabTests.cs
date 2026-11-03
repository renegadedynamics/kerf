// Port of: src/FoundationClasses/TKMath/GTests/math_DoubleTab_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

namespace Kerf.Kernel.Foundation.Tests;

public class math_DoubleTabTest
{
    // Tests for constructors
    [Fact]
    public void DefaultConstructor()
    {
        // Test standard constructor with ranges
        DoubleTab aTab = new(1, 3, 1, 3);

        // Initialize with test values
        aTab.Init(5.0);

        // Check all values are set to 5.0
        for (int anI = 1; anI <= 3; anI++)
        {
            for (int aJ = 1; aJ <= 3; aJ++)
            {
                Assert.Equal(5.0, aTab.Value(anI, aJ));
                Assert.Equal(5.0, aTab[anI, aJ]); // Test operator()
            }
        }
    }

    [Fact]
    public void ExternalArrayConstructor()
    {
        // Create external array
        double[] anArray = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];

        // Create math_DoubleTab from external array
        DoubleTab aTab = new(anArray, 1, 3, 1, 3);

        // Check values are correctly accessed
        int anIndex = 0;
        for (int anI = 1; anI <= 3; anI++)
        {
            for (int aJ = 1; aJ <= 3; aJ++)
            {
                Assert.Equal(anArray[anIndex], aTab.Value(anI, aJ));
                anIndex++;
            }
        }
    }

    [Fact]
    public void CopyConstructor()
    {
        // Create original array
        DoubleTab aOriginal = new(1, 3, 1, 3);
        aOriginal.Init(10.0);

        // Set some specific values
        aOriginal.Value(1, 1) = 1.5;
        aOriginal.Value(2, 2) = 2.5;
        aOriginal.Value(3, 3) = 3.5;

        // Create copy
        DoubleTab aCopy = new(aOriginal);

        // Check values are copied correctly
        Assert.Equal(1.5, aCopy.Value(1, 1));
        Assert.Equal(2.5, aCopy.Value(2, 2));
        Assert.Equal(3.5, aCopy.Value(3, 3));
        Assert.Equal(10.0, aCopy.Value(1, 2)); // Check other values too
    }

    // Tests for basic operations
    [Fact]
    public void InitOperation()
    {
        DoubleTab aTab = new(0, 2, 0, 2);

        // Initialize with specific value
        aTab.Init(7.5);

        // Check all elements are set
        for (int anI = 0; anI <= 2; anI++)
        {
            for (int aJ = 0; aJ <= 2; aJ++)
            {
                Assert.Equal(7.5, aTab.Value(anI, aJ));
            }
        }
    }

    [Fact]
    public void ValueAccess()
    {
        DoubleTab aTab = new(1, 2, 1, 2);

        // Set values using Value method
        aTab.Value(1, 1) = 11.0;
        aTab.Value(1, 2) = 12.0;
        aTab.Value(2, 1) = 21.0;
        aTab.Value(2, 2) = 22.0;

        // Check values using operator()
        Assert.Equal(11.0, aTab[1, 1]);
        Assert.Equal(12.0, aTab[1, 2]);
        Assert.Equal(21.0, aTab[2, 1]);
        Assert.Equal(22.0, aTab[2, 2]);
    }

    [Fact]
    public void OperatorAccess()
    {
        DoubleTab aTab = new(-1, 1, -1, 1);

        // Set values using operator()
        aTab[-1, -1] = -11.0;
        aTab[-1, 0]  = -10.0;
        aTab[-1, 1]  = -9.0;
        aTab[0, -1]  = -1.0;
        aTab[0, 0]   = 0.0;
        aTab[0, 1]   = 1.0;
        aTab[1, -1]  = 9.0;
        aTab[1, 0]   = 10.0;
        aTab[1, 1]   = 11.0;

        // Check values using Value method
        Assert.Equal(-11.0, aTab.Value(-1, -1));
        Assert.Equal(-10.0, aTab.Value(-1, 0));
        Assert.Equal(-9.0, aTab.Value(-1, 1));
        Assert.Equal(-1.0, aTab.Value(0, -1));
        Assert.Equal(0.0, aTab.Value(0, 0));
        Assert.Equal(1.0, aTab.Value(0, 1));
        Assert.Equal(9.0, aTab.Value(1, -1));
        Assert.Equal(10.0, aTab.Value(1, 0));
        Assert.Equal(11.0, aTab.Value(1, 1));
    }

    [Fact]
    public void CopyMethod()
    {
        // Create source array
        DoubleTab aSource = new(1, 2, 1, 2);
        aSource.Value(1, 1) = 100.0;
        aSource.Value(1, 2) = 200.0;
        aSource.Value(2, 1) = 300.0;
        aSource.Value(2, 2) = 400.0;

        // Create destination array
        DoubleTab aDest = new(1, 2, 1, 2);
        aDest.Init(0.0);

        // Copy from source to destination
        aSource.Copy(aDest);

        // Check values are copied
        Assert.Equal(100.0, aDest.Value(1, 1));
        Assert.Equal(200.0, aDest.Value(1, 2));
        Assert.Equal(300.0, aDest.Value(2, 1));
        Assert.Equal(400.0, aDest.Value(2, 2));
    }

    // Tests for buffer optimization
    [Fact]
    public void SmallArrayOptimization()
    {
        // Test that small arrays (<=16 elements) use buffer optimization
        // 4x4 = 16 elements, should use buffer
        DoubleTab aSmallTab = new(1, 4, 1, 4);
        aSmallTab.Init(42.0);

        // Set and read values to ensure it works correctly
        aSmallTab.Value(2, 3) = 123.45;
        Assert.Equal(123.45, aSmallTab.Value(2, 3));
        Assert.Equal(42.0, aSmallTab.Value(1, 1));
    }

    [Fact]
    public void LargeArrayAllocation()
    {
        // Test that large arrays (>16 elements) allocate memory
        // 5x5 = 25 elements, should allocate
        DoubleTab aLargeTab = new(1, 5, 1, 5);
        aLargeTab.Init(99.99);

        // Set and read values to ensure it works correctly
        aLargeTab.Value(3, 4) = 987.65;
        Assert.Equal(987.65, aLargeTab.Value(3, 4));
        Assert.Equal(99.99, aLargeTab.Value(1, 1));
    }

    // Tests for edge cases
    [Fact]
    public void SingleElement()
    {
        // Test with single element array
        DoubleTab aSingle = new(5, 5, 10, 10);
        aSingle.Value(5, 10) = 777.0;

        Assert.Equal(777.0, aSingle.Value(5, 10));
        Assert.Equal(777.0, aSingle[5, 10]);
    }

    [Fact]
    public void NegativeIndices()
    {
        // Test with negative indices
        DoubleTab aNegTab = new(-5, -1, -3, -1);
        aNegTab.Init(-1.0);

        aNegTab.Value(-3, -2) = -999.0;
        Assert.Equal(-999.0, aNegTab.Value(-3, -2));
        Assert.Equal(-1.0, aNegTab.Value(-5, -3));
    }
}
