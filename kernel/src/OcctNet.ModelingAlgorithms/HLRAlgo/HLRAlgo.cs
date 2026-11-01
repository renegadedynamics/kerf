// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The min-max boxes of the hidden-line removal: sixteen bounds per box, the extents along seven directions of the
/// projection plane (both ways) and the depth, and their encoding in eight integers of two 15-bit indices.
/// </summary>
/// <remarks>
/// HLRBRep_Algo works on the shape and gives exact results; HLRBRep_PolyAlgo works on a polyhedral simplification.
/// The <c>double[16]</c> arrays are spans.
/// </remarks>
public static class HLRAlgo
{
    private static readonly double cosu0 = Math.Cos(0.0 * M_PI / 14.0);
    private static readonly double sinu0 = Math.Sin(0.0 * M_PI / 14.0);
    private static readonly double cosu1 = Math.Cos(1.0 * M_PI / 14.0);
    private static readonly double sinu1 = Math.Sin(1.0 * M_PI / 14.0);
    private static readonly double cosu2 = Math.Cos(2.0 * M_PI / 14.0);
    private static readonly double sinu2 = Math.Sin(2.0 * M_PI / 14.0);
    private static readonly double cosu3 = Math.Cos(3.0 * M_PI / 14.0);
    private static readonly double sinu3 = Math.Sin(3.0 * M_PI / 14.0);
    private static readonly double cosu4 = Math.Cos(4.0 * M_PI / 14.0);
    private static readonly double sinu4 = Math.Sin(4.0 * M_PI / 14.0);
    private static readonly double cosu5 = Math.Cos(5.0 * M_PI / 14.0);
    private static readonly double sinu5 = Math.Sin(5.0 * M_PI / 14.0);
    private static readonly double cosu6 = Math.Cos(6.0 * M_PI / 14.0);
    private static readonly double sinu6 = Math.Sin(6.0 * M_PI / 14.0);

    /// <summary>Widens the box (<paramref name="Min"/>, <paramref name="Max"/>, sixteen each) to the point (x, y, z).</summary>
    public static void UpdateMinMax(double x, double y, double z, Span<double> Min, Span<double> Max)
    {
        Span<double> d = stackalloc double[16];
        d[0] = cosu0 * x + sinu0 * y;
        d[1] = sinu0 * x - cosu0 * y;
        d[2] = cosu1 * x + sinu1 * y;
        d[3] = sinu1 * x - cosu1 * y;
        d[4] = cosu2 * x + sinu2 * y;
        d[5] = sinu2 * x - cosu2 * y;
        d[6] = cosu3 * x + sinu3 * y;
        d[7] = sinu3 * x - cosu3 * y;
        d[8] = cosu4 * x + sinu4 * y;
        d[9] = sinu4 * x - cosu4 * y;
        d[10] = cosu5 * x + sinu5 * y;
        d[11] = sinu5 * x - cosu5 * y;
        d[12] = cosu6 * x + sinu6 * y;
        d[13] = sinu6 * x - cosu6 * y;
        d[14] = z;
        d[15] = z;

        for (int i = 0; i < 16; ++i)
        {
            if (Min[i] > d[i])
            {
                Min[i] = d[i];
            }
            if (Max[i] < d[i])
            {
                Max[i] = d[i];
            }
        }
    }

    /// <summary>Widens the box by <paramref name="tol"/> on each bound.</summary>
    public static void EnlargeMinMax(double tol, Span<double> Min, Span<double> Max)
    {
        int i = 0;
        while (i < 16)
        {
            Min[i] -= tol;
            Max[i] += tol;
            i++;
        }
    }

    /// <summary>Initializes the box to the empty one (bounds at +/- <paramref name="Big"/>).</summary>
    public static void InitMinMax(double Big, Span<double> Min, Span<double> Max)
    {
        int i = 0;
        while (i < 16)
        {
            Min[i] = Big;
            Max[i] = -Big;
            i++;
        }
    }

    /// <summary>Packs the indices of <paramref name="Min"/> and <paramref name="Max"/> into <paramref name="MM"/>, two 15-bit indices per integer.</summary>
    public static void EncodeMinMax(ref HLRAlgo_EdgesBlock.MinMaxIndices Min, ref HLRAlgo_EdgesBlock.MinMaxIndices Max,
                                    ref HLRAlgo_EdgesBlock.MinMaxIndices MM)
    {
        MM.Min[0] = Min.Min[1] & 0x00007fff;
        MM.Max[0] = Max.Min[1] & 0x00007fff;
        MM.Min[0] += (Min.Min[0] & 0x00007fff) << 16;
        MM.Max[0] += (Max.Min[0] & 0x00007fff) << 16;
        MM.Min[1] = Min.Min[3] & 0x00007fff;
        MM.Max[1] = Max.Min[3] & 0x00007fff;
        MM.Min[1] += (Min.Min[2] & 0x00007fff) << 16;
        MM.Max[1] += (Max.Min[2] & 0x00007fff) << 16;
        MM.Min[2] = Min.Min[5] & 0x00007fff;
        MM.Max[2] = Max.Min[5] & 0x00007fff;
        MM.Min[2] += (Min.Min[4] & 0x00007fff) << 16;
        MM.Max[2] += (Max.Min[4] & 0x00007fff) << 16;
        MM.Min[3] = Min.Min[7] & 0x00007fff;
        MM.Max[3] = Max.Min[7] & 0x00007fff;
        MM.Min[3] += (Min.Min[6] & 0x00007fff) << 16;
        MM.Max[3] += (Max.Min[6] & 0x00007fff) << 16;
        MM.Min[4] = Min.Max[1] & 0x00007fff;
        MM.Max[4] = Max.Max[1] & 0x00007fff;
        MM.Min[4] += (Min.Max[0] & 0x00007fff) << 16;
        MM.Max[4] += (Max.Max[0] & 0x00007fff) << 16;
        MM.Min[5] = Min.Max[3] & 0x00007fff;
        MM.Max[5] = Max.Max[3] & 0x00007fff;
        MM.Min[5] += (Min.Max[2] & 0x00007fff) << 16;
        MM.Max[5] += (Max.Max[2] & 0x00007fff) << 16;
        MM.Min[6] = Min.Max[5] & 0x00007fff;
        MM.Max[6] = Max.Max[5] & 0x00007fff;
        MM.Min[6] += (Min.Max[4] & 0x00007fff) << 16;
        MM.Max[6] += (Max.Max[4] & 0x00007fff) << 16;
        MM.Min[7] = Min.Max[7] & 0x00007fff;
        MM.Max[7] = Max.Max[7] & 0x00007fff;
        MM.Min[7] += (Min.Max[6] & 0x00007fff) << 16;
        MM.Max[7] += (Max.Max[6] & 0x00007fff) << 16;
    }

    /// <summary>The product of the sides of the (unpacked) box (<paramref name="Min"/>, <paramref name="Max"/>), the depth's last two excluded.</summary>
    public static double SizeBox(ref HLRAlgo_EdgesBlock.MinMaxIndices Min, ref HLRAlgo_EdgesBlock.MinMaxIndices Max)
    {
        double s = Max.Min[0] - Min.Min[0];
        for (int aI = 1; aI < 8; ++aI)
        {
            s *= Max.Min[aI] - Min.Min[aI];
        }
        for (int aI = 0; aI < 6; ++aI)
        {
            s *= Max.Max[aI] - Min.Max[aI];
        }
        return s;
    }

    /// <summary>Unpacks <paramref name="MM"/> into <paramref name="Min"/> and <paramref name="Max"/>.</summary>
    public static void DecodeMinMax(in HLRAlgo_EdgesBlock.MinMaxIndices MM, ref HLRAlgo_EdgesBlock.MinMaxIndices Min,
                                    ref HLRAlgo_EdgesBlock.MinMaxIndices Max)
    {
        Min.Min[0] = (MM.Min[0] & 0x7fff0000) >> 16;
        Max.Min[0] = (MM.Max[0] & 0x7fff0000) >> 16;
        Min.Min[1] = MM.Min[0] & 0x00007fff;
        Max.Min[1] = MM.Max[0] & 0x00007fff;
        Min.Min[2] = (MM.Min[1] & 0x7fff0000) >> 16;
        Max.Min[2] = (MM.Max[1] & 0x7fff0000) >> 16;
        Min.Min[3] = MM.Min[1] & 0x00007fff;
        Max.Min[3] = MM.Max[1] & 0x00007fff;
        Min.Min[4] = (MM.Min[2] & 0x7fff0000) >> 16;
        Max.Min[4] = (MM.Max[2] & 0x7fff0000) >> 16;
        Min.Min[5] = MM.Min[2] & 0x00007fff;
        Max.Min[5] = MM.Max[2] & 0x00007fff;
        Min.Min[6] = (MM.Min[3] & 0x7fff0000) >> 16;
        Max.Min[6] = (MM.Max[3] & 0x7fff0000) >> 16;
        Min.Min[7] = MM.Min[3] & 0x00007fff;
        Max.Min[7] = MM.Max[3] & 0x00007fff;
        Min.Max[0] = (MM.Min[4] & 0x7fff0000) >> 16;
        Max.Max[0] = (MM.Max[4] & 0x7fff0000) >> 16;
        Min.Max[1] = MM.Min[4] & 0x00007fff;
        Max.Max[1] = MM.Max[4] & 0x00007fff;
        Min.Max[2] = (MM.Min[5] & 0x7fff0000) >> 16;
        Max.Max[2] = (MM.Max[5] & 0x7fff0000) >> 16;
        Min.Max[3] = MM.Min[5] & 0x00007fff;
        Max.Max[3] = MM.Max[5] & 0x00007fff;
        Min.Max[4] = (MM.Min[6] & 0x7fff0000) >> 16;
        Max.Max[4] = (MM.Max[6] & 0x7fff0000) >> 16;
        Min.Max[5] = MM.Min[6] & 0x00007fff;
        Max.Max[5] = MM.Max[6] & 0x00007fff;
        Min.Max[6] = (MM.Min[7] & 0x7fff0000) >> 16;
        Max.Max[6] = (MM.Max[7] & 0x7fff0000) >> 16;
        Min.Max[7] = MM.Min[7] & 0x00007fff;
        Max.Max[7] = MM.Max[7] & 0x00007fff;
    }

    /// <summary>Copies (<paramref name="IMin"/>, <paramref name="IMax"/>) into (<paramref name="OMin"/>, <paramref name="OMax"/>).</summary>
    public static void CopyMinMax(ref HLRAlgo_EdgesBlock.MinMaxIndices IMin, ref HLRAlgo_EdgesBlock.MinMaxIndices IMax,
                                  ref HLRAlgo_EdgesBlock.MinMaxIndices OMin, ref HLRAlgo_EdgesBlock.MinMaxIndices OMax)
    {
        OMin = IMin;
        OMax = IMax;
    }

    /// <summary>Widens (<paramref name="OMin"/>, <paramref name="OMax"/>) to (<paramref name="IMin"/>, <paramref name="IMax"/>).</summary>
    public static void AddMinMax(ref HLRAlgo_EdgesBlock.MinMaxIndices IMin, ref HLRAlgo_EdgesBlock.MinMaxIndices IMax,
                                 ref HLRAlgo_EdgesBlock.MinMaxIndices OMin, ref HLRAlgo_EdgesBlock.MinMaxIndices OMax)
    {
        OMin.Minimize(IMin);
        OMax.Maximize(IMax);
    }
}
