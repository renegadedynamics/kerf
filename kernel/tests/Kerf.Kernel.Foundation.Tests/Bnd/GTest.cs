// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Kernel.Foundation.Tests;

/// <summary>GoogleTest's floating-point assertions, for ported GTests.</summary>
internal static class GTest
{
    /// <summary><c>EXPECT_DOUBLE_EQ(expected, actual)</c>: equal within 4 units in the last place.</summary>
    public static void ExpectDoubleEq(double theExpected, double theActual)
        => Assert.True(AlmostEqual(BitConverter.DoubleToInt64Bits(theExpected), BitConverter.DoubleToInt64Bits(theActual),
                                   double.IsNaN(theExpected) || double.IsNaN(theActual)),
                       $"Expected {theExpected:R}, actual {theActual:R} (more than 4 ULPs apart)");

    /// <summary><c>EXPECT_FLOAT_EQ(expected, actual)</c>: equal within 4 units in the last place.</summary>
    public static void ExpectFloatEq(float theExpected, float theActual)
        => Assert.True(AlmostEqual((double)theExpected, (double)theActual),
                       $"Expected {theExpected:R}, actual {theActual:R} (more than 4 ULPs apart)");

    // A float widened to double keeps its value; its ULP distance is then counted in float ULPs.
    private static bool AlmostEqual(double theValue1, double theValue2)
    {
        if (double.IsNaN(theValue1) || double.IsNaN(theValue2))
        {
            return false;
        }

        uint aBiased1 = Biased32(BitConverter.SingleToUInt32Bits((float)theValue1));
        uint aBiased2 = Biased32(BitConverter.SingleToUInt32Bits((float)theValue2));
        return (aBiased1 >= aBiased2 ? aBiased1 - aBiased2 : aBiased2 - aBiased1) <= 4;
    }

    private static uint Biased32(uint theBits)
    {
        const uint aSignBit = 1U << 31;
        return (theBits & aSignBit) != 0 ? ~theBits + 1 : aSignBit | theBits;
    }

    // GoogleTest's FloatingPoint::AlmostEquals: compare the bits as biased (sign-magnitude to offset) integers.
    private static bool AlmostEqual(long theBits1, long theBits2, bool theIsNaN)
    {
        if (theIsNaN)
        {
            return false;
        }

        ulong aBiased1 = Biased(theBits1);
        ulong aBiased2 = Biased(theBits2);
        ulong aDistance = aBiased1 >= aBiased2 ? aBiased1 - aBiased2 : aBiased2 - aBiased1;
        return aDistance <= 4;
    }

    private static ulong Biased(long theBits)
    {
        const ulong aSignBit = 1UL << 63;
        ulong aBits = (ulong)theBits;
        return (aBits & aSignBit) != 0 ? ~aBits + 1 : aSignBit | aBits;
    }
}
