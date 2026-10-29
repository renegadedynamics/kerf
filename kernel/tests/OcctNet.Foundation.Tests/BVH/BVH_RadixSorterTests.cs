// Port of: src/FoundationClasses/TKMath/GTests/BVH_RadixSorter_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class BVH_RadixSorterTest
{
    [Fact]
    public void EncodeMortonCode_Origin()
    {
        uint aCode = BVH.EncodeMortonCode(0, 0, 0);
        Assert.Equal(0u, aCode);
    }

    [Fact]
    public void EncodeMortonCode_SingleBits()
    {
        // X=1, Y=0, Z=0 should give 1 (bit 0)
        uint aCodeX = BVH.EncodeMortonCode(1, 0, 0);
        Assert.Equal(1u, aCodeX);

        // X=0, Y=1, Z=0 should give 2 (bit 1)
        uint aCodeY = BVH.EncodeMortonCode(0, 1, 0);
        Assert.Equal(2u, aCodeY);

        // X=0, Y=0, Z=1 should give 4 (bit 2)
        uint aCodeZ = BVH.EncodeMortonCode(0, 0, 1);
        Assert.Equal(4u, aCodeZ);

        // X=1, Y=1, Z=1 should give 7 (bits 0, 1, 2)
        uint aCodeXYZ = BVH.EncodeMortonCode(1, 1, 1);
        Assert.Equal(7u, aCodeXYZ);
    }

    [Fact]
    public void EncodeMortonCode_Interleaving()
    {
        // X=2 (binary 10), Y=0, Z=0
        // Should produce binary ...001000 = 8
        uint aCode1 = BVH.EncodeMortonCode(2, 0, 0);
        Assert.Equal(8u, aCode1);

        // X=0, Y=2, Z=0
        // Should produce binary ...010000 = 16
        uint aCode2 = BVH.EncodeMortonCode(0, 2, 0);
        Assert.Equal(16u, aCode2);

        // X=0, Y=0, Z=2
        // Should produce binary ...100000 = 32
        uint aCode3 = BVH.EncodeMortonCode(0, 0, 2);
        Assert.Equal(32u, aCode3);
    }

    [Fact]
    public void EncodeMortonCode_MaxValues()
    {
        // Test with maximum 10-bit values (1023)
        uint aCode = BVH.EncodeMortonCode(1023, 1023, 1023);

        // All 30 bits should be set
        Assert.Equal(0x3FFFFFFFu, aCode);
    }

    [Fact]
    public void EncodeMortonCode_Symmetry()
    {
        // Swapping coordinates should produce predictable results
        uint aCode1 = BVH.EncodeMortonCode(100, 200, 300);
        uint aCode2 = BVH.EncodeMortonCode(200, 100, 300);
        uint aCode3 = BVH.EncodeMortonCode(100, 300, 200);

        // Codes should be different
        Assert.NotEqual(aCode2, aCode1);
        Assert.NotEqual(aCode3, aCode1);
        Assert.NotEqual(aCode3, aCode2);
    }

    [Fact]
    public void EncodeMortonCode_Ordering()
    {
        // Points closer in space should have closer Morton codes
        // (0, 0, 0) vs (1, 0, 0) should be closer than (0, 0, 0) vs (0, 0, 1023)
        uint aCode000    = BVH.EncodeMortonCode(0, 0, 0);
        uint aCode100    = BVH.EncodeMortonCode(1, 0, 0);
        uint aCode001023 = BVH.EncodeMortonCode(0, 0, 1023);

        // (0,0,0) to (1,0,0) difference should be small
        uint aDiff1 = aCode100 - aCode000;

        // (0,0,0) to (0,0,1023) difference should be large
        uint aDiff2 = aCode001023 - aCode000;

        Assert.True(aDiff1 < aDiff2);
    }

    [Fact]
    public void EncodeMortonCode_MidValues()
    {
        // Test with mid-range values
        uint aCode = BVH.EncodeMortonCode(512, 512, 512);

        // Should be non-zero and within 30-bit range
        Assert.True(aCode > 0u);
        Assert.True(aCode <= 0x3FFFFFFFu);
    }

    [Fact]
    public void EncodeMortonCode_PowersOfTwo()
    {
        // Test powers of two for each axis
        uint aCode4  = BVH.EncodeMortonCode(4, 0, 0);  // X=100 binary
        uint aCode8  = BVH.EncodeMortonCode(8, 0, 0);  // X=1000 binary
        uint aCode16 = BVH.EncodeMortonCode(16, 0, 0); // X=10000 binary

        // Each should be 8x the previous (3 bit positions apart in Morton code)
        Assert.Equal(aCode4 * 8, aCode8);
        Assert.Equal(aCode8 * 8, aCode16);
    }

    [Fact]
    public void EncodeMortonCode_ConsistentEncoding()
    {
        // Same inputs should always produce same outputs
        for (int i = 0; i < 100; ++i)
        {
            uint aCode1 = BVH.EncodeMortonCode(100, 200, 300);
            uint aCode2 = BVH.EncodeMortonCode(100, 200, 300);
            Assert.Equal(aCode2, aCode1);
        }
    }

    [Fact]
    public void EncodeMortonCode_BoundaryValues()
    {
        // Test boundary values near max (1023)
        uint aCode1022 = BVH.EncodeMortonCode(1022, 1022, 1022);
        uint aCode1023 = BVH.EncodeMortonCode(1023, 1023, 1023);

        Assert.True(aCode1022 < aCode1023);
        Assert.Equal(0x3FFFFFFFu, aCode1023);
    }

    [Fact]
    public void EncodeMortonCode_SmallValues()
    {
        // Test small values
        uint aCode2 = BVH.EncodeMortonCode(2, 2, 2);
        uint aCode3 = BVH.EncodeMortonCode(3, 3, 3);

        // Both should be small but aCode3 > aCode2
        Assert.True(aCode2 < 100u);
        Assert.True(aCode3 < 100u);
        Assert.True(aCode2 < aCode3);
    }

    [Fact]
    public void EncodeMortonCode_SingleAxisValues()
    {
        // Test with value only in one axis
        uint aCodeX = BVH.EncodeMortonCode(255, 0, 0);
        uint aCodeY = BVH.EncodeMortonCode(0, 255, 0);
        uint aCodeZ = BVH.EncodeMortonCode(0, 0, 255);

        // X bits in positions 0, 3, 6, ...
        // Y bits in positions 1, 4, 7, ...
        // Z bits in positions 2, 5, 8, ...
        Assert.NotEqual(aCodeY, aCodeX);
        Assert.NotEqual(aCodeZ, aCodeY);
        Assert.NotEqual(aCodeZ, aCodeX);
    }

    [Fact]
    public void EncodeMortonCode_SequentialXValues()
    {
        // Test that sequential X values produce increasing codes
        uint aPrev = BVH.EncodeMortonCode(0, 500, 500);
        for (uint x = 1; x <= 10; ++x)
        {
            uint aCurr = BVH.EncodeMortonCode(x, 500, 500);
            Assert.True(aCurr > aPrev);
            aPrev = aCurr;
        }
    }

    [Fact]
    public void EncodeMortonCode_SequentialYValues()
    {
        // Test that sequential Y values produce increasing codes
        uint aPrev = BVH.EncodeMortonCode(500, 0, 500);
        for (uint y = 1; y <= 10; ++y)
        {
            uint aCurr = BVH.EncodeMortonCode(500, y, 500);
            Assert.True(aCurr > aPrev);
            aPrev = aCurr;
        }
    }

    [Fact]
    public void EncodeMortonCode_SequentialZValues()
    {
        // Test that sequential Z values produce increasing codes
        uint aPrev = BVH.EncodeMortonCode(500, 500, 0);
        for (uint z = 1; z <= 10; ++z)
        {
            uint aCurr = BVH.EncodeMortonCode(500, 500, z);
            Assert.True(aCurr > aPrev);
            aPrev = aCurr;
        }
    }

    [Fact]
    public void EncodeMortonCode_ZeroInDimensions()
    {
        // Test with zeros in different dimensions
        uint aCodeXY = BVH.EncodeMortonCode(100, 100, 0);
        uint aCodeXZ = BVH.EncodeMortonCode(100, 0, 100);
        uint aCodeYZ = BVH.EncodeMortonCode(0, 100, 100);

        // All should be different
        Assert.NotEqual(aCodeXZ, aCodeXY);
        Assert.NotEqual(aCodeYZ, aCodeXY);
        Assert.NotEqual(aCodeYZ, aCodeXZ);

        // All should be non-zero
        Assert.True(aCodeXY > 0u);
        Assert.True(aCodeXZ > 0u);
        Assert.True(aCodeYZ > 0u);
    }

    [Fact]
    public void EncodeMortonCode_UpperByteBits()
    {
        // Test values that require upper byte (values >= 256)
        uint aCode256 = BVH.EncodeMortonCode(256, 0, 0);
        uint aCode512 = BVH.EncodeMortonCode(512, 0, 0);
        uint aCode768 = BVH.EncodeMortonCode(768, 0, 0);

        Assert.True(aCode256 > 0u);
        Assert.True(aCode512 > aCode256);
        Assert.True(aCode768 > aCode512);
    }

    [Fact]
    public void EncodeMortonCode_AllOnes()
    {
        // 255 = 0xFF (all ones in lower byte)
        uint aCode = BVH.EncodeMortonCode(255, 255, 255);

        // Should be less than max (1023, 1023, 1023)
        Assert.True(aCode < 0x3FFFFFFFu);
        Assert.True(aCode > 0u);
    }

    [Fact]
    public void EncodeMortonCode_LocalityPreservation()
    {
        // Points close in space should have relatively close Morton codes
        uint aCenter = BVH.EncodeMortonCode(512, 512, 512);
        uint aNear1  = BVH.EncodeMortonCode(513, 512, 512);
        uint aNear2  = BVH.EncodeMortonCode(512, 513, 512);
        uint aFar    = BVH.EncodeMortonCode(0, 0, 0);

        // Near points should have smaller difference than far point
        uint aDiffNear1 = (aNear1 > aCenter) ? (aNear1 - aCenter) : (aCenter - aNear1);
        uint aDiffNear2 = (aNear2 > aCenter) ? (aNear2 - aCenter) : (aCenter - aNear2);
        uint aDiffFar   = (aFar > aCenter) ? (aFar - aCenter) : (aCenter - aFar);

        Assert.True(aDiffNear1 < aDiffFar);
        Assert.True(aDiffNear2 < aDiffFar);
    }

    [Fact]
    public void EncodeMortonCode_BitPattern3()
    {
        // X=3 (binary 11), Y=0, Z=0
        // Interleaved: ...001001 = 9
        uint aCode = BVH.EncodeMortonCode(3, 0, 0);
        Assert.Equal(9u, aCode);
    }

    [Fact]
    public void EncodeMortonCode_BitPattern7()
    {
        // X=7 (binary 111), Y=0, Z=0
        // Interleaved: 001001001 = 73
        uint aCode = BVH.EncodeMortonCode(7, 0, 0);
        Assert.Equal(73u, aCode);
    }

    [Fact]
    public void EncodeMortonCode_XYEqual()
    {
        // When X = Y and Z = 0, code should follow pattern
        uint aCode1 = BVH.EncodeMortonCode(1, 1, 0);
        uint aCode2 = BVH.EncodeMortonCode(2, 2, 0);
        uint aCode3 = BVH.EncodeMortonCode(4, 4, 0);

        // X=1, Y=1, Z=0: bits at 0,1 . 3
        Assert.Equal(3u, aCode1);

        // X=2, Y=2, Z=0: bits at 3,4 . 24
        Assert.Equal(24u, aCode2);

        // X=4, Y=4, Z=0: bits at 6,7 . 192
        Assert.Equal(192u, aCode3);
    }

    [Fact]
    public void EncodeMortonCode_AllAxesEqual()
    {
        // When all axes are equal
        uint aCode1 = BVH.EncodeMortonCode(1, 1, 1);
        uint aCode2 = BVH.EncodeMortonCode(2, 2, 2);

        // X=1, Y=1, Z=1: bits at 0,1,2 . 7
        Assert.Equal(7u, aCode1);

        // X=2, Y=2, Z=2: bits at 3,4,5 . 56
        Assert.Equal(56u, aCode2);
    }

    [Fact]
    public void EncodeMortonCode_HalfMax()
    {
        // Test with half of maximum (512)
        uint aCode = BVH.EncodeMortonCode(512, 512, 512);

        // Should be in upper half of range
        Assert.True(aCode > 0x1FFFFFFFu);
        Assert.True(aCode <= 0x3FFFFFFFu);
    }

    [Fact]
    public void EncodeMortonCode_QuarterMax()
    {
        // Test with quarter of maximum (256)
        uint aCode = BVH.EncodeMortonCode(256, 256, 256);

        // Should be less than half max
        Assert.True(aCode > 0u);
        Assert.True(aCode < 0x3FFFFFFFu);
    }

    [Fact]
    public void EncodeMortonCode_DifferentScales()
    {
        // Test that scaling all coordinates by 2 shifts the Morton code
        uint aCode1 = BVH.EncodeMortonCode(100, 100, 100);
        uint aCode2 = BVH.EncodeMortonCode(200, 200, 200);

        // Code2 should be larger
        Assert.True(aCode2 > aCode1);
    }

    [Fact]
    public void EncodeMortonCode_AdjacentCells()
    {
        // Test adjacent cells in X direction
        uint aCode1 = BVH.EncodeMortonCode(100, 100, 100);
        uint aCode2 = BVH.EncodeMortonCode(101, 100, 100);

        // Difference should be 1 (X bit is at position 0)
        Assert.Equal(1u, aCode2 - aCode1);
    }

    [Fact]
    public void EncodeMortonCode_AdjacentCellsY()
    {
        // Test adjacent cells in Y direction
        uint aCode1 = BVH.EncodeMortonCode(100, 100, 100);
        uint aCode2 = BVH.EncodeMortonCode(100, 101, 100);

        // Difference should be 2 (Y bit is at position 1)
        Assert.Equal(2u, aCode2 - aCode1);
    }

    [Fact]
    public void EncodeMortonCode_AdjacentCellsZ()
    {
        // Test adjacent cells in Z direction
        uint aCode1 = BVH.EncodeMortonCode(100, 100, 100);
        uint aCode2 = BVH.EncodeMortonCode(100, 100, 101);

        // Difference should be 4 (Z bit is at position 2)
        Assert.Equal(4u, aCode2 - aCode1);
    }

    [Fact]
    public void EncodeMortonCode_MonotonicX()
    {
        // Verify monotonicity when only X changes
        uint aPrev = 0;
        for (uint x = 0; x < 100; ++x)
        {
            uint aCode = BVH.EncodeMortonCode(x, 0, 0);
            Assert.True(aCode >= aPrev);
            aPrev = aCode;
        }
    }

    [Fact]
    public void EncodeMortonCode_MonotonicY()
    {
        // Verify monotonicity when only Y changes
        uint aPrev = 0;
        for (uint y = 0; y < 100; ++y)
        {
            uint aCode = BVH.EncodeMortonCode(0, y, 0);
            Assert.True(aCode >= aPrev);
            aPrev = aCode;
        }
    }

    [Fact]
    public void EncodeMortonCode_MonotonicZ()
    {
        // Verify monotonicity when only Z changes
        uint aPrev = 0;
        for (uint z = 0; z < 100; ++z)
        {
            uint aCode = BVH.EncodeMortonCode(0, 0, z);
            Assert.True(aCode >= aPrev);
            aPrev = aCode;
        }
    }

    [Fact]
    public void EncodeMortonCode_NoOverflow()
    {
        // Verify no overflow with maximum values
        uint aCode = BVH.EncodeMortonCode(1023, 1023, 1023);

        // Should fit in 30 bits
        Assert.Equal(0u, aCode & 0xC0000000u);
    }

    [Fact]
    public void EncodeMortonCode_UniqueForDifferentInputs()
    {
        // Different inputs should produce different codes
        uint aCode1 = BVH.EncodeMortonCode(100, 200, 300);
        uint aCode2 = BVH.EncodeMortonCode(100, 200, 301);
        uint aCode3 = BVH.EncodeMortonCode(100, 201, 300);
        uint aCode4 = BVH.EncodeMortonCode(101, 200, 300);

        Assert.NotEqual(aCode2, aCode1);
        Assert.NotEqual(aCode3, aCode1);
        Assert.NotEqual(aCode4, aCode1);
        Assert.NotEqual(aCode3, aCode2);
        Assert.NotEqual(aCode4, aCode2);
        Assert.NotEqual(aCode4, aCode3);
    }

    [Fact]
    public void EncodeMortonCode_SpecificPattern()
    {
        // Test specific known pattern
        // X=5 (101), Y=3 (011), Z=6 (110)
        uint aCode = BVH.EncodeMortonCode(5, 3, 6);

        // Morton code interleaves bits: X at 3i, Y at 3i+1, Z at 3i+2
        // The exact value depends on the interleaving order
        // Just verify it's non-zero and within range
        Assert.True(aCode > 0u);
        Assert.True(aCode <= 0x3FFFFFFFu);

        // Also verify consistency
        uint aCode2 = BVH.EncodeMortonCode(5, 3, 6);
        Assert.Equal(aCode2, aCode);
    }
}
