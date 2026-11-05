// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.HiddenLines.Algorithm;

/// <summary>
/// The min-max boxes of the hidden-line removal: sixteen bounds per box, the extents along seven directions of the
/// projection plane (both ways) and the depth, and their encoding in eight integers of two 15-bit indices.
/// </summary>
/// <remarks>
/// HLRBRep_Algo works on the shape and gives exact results; HLRBRep_PolyAlgo works on a polyhedral simplification.
/// The <c>double[16]</c> arrays are spans.
/// </remarks>
internal static class HlrAlgo
{
    private static readonly double Cosu0 = Math.Cos(0.0 * M_PI / 14.0);
    private static readonly double Sinu0 = Math.Sin(0.0 * M_PI / 14.0);
    private static readonly double Cosu1 = Math.Cos(1.0 * M_PI / 14.0);
    private static readonly double Sinu1 = Math.Sin(1.0 * M_PI / 14.0);
    private static readonly double Cosu2 = Math.Cos(2.0 * M_PI / 14.0);
    private static readonly double Sinu2 = Math.Sin(2.0 * M_PI / 14.0);
    private static readonly double Cosu3 = Math.Cos(3.0 * M_PI / 14.0);
    private static readonly double Sinu3 = Math.Sin(3.0 * M_PI / 14.0);
    private static readonly double Cosu4 = Math.Cos(4.0 * M_PI / 14.0);
    private static readonly double Sinu4 = Math.Sin(4.0 * M_PI / 14.0);
    private static readonly double Cosu5 = Math.Cos(5.0 * M_PI / 14.0);
    private static readonly double Sinu5 = Math.Sin(5.0 * M_PI / 14.0);
    private static readonly double Cosu6 = Math.Cos(6.0 * M_PI / 14.0);
    private static readonly double Sinu6 = Math.Sin(6.0 * M_PI / 14.0);

    /// <summary>Widens the box (<paramref name="min"/>, <paramref name="max"/>, sixteen each) to the point (x, y, z).</summary>
    public static void UpdateMinMax(double x, double y, double z, Span<double> min, Span<double> max)
    {
        Span<double> d = stackalloc double[16];
        d[0] = Cosu0 * x + Sinu0 * y;
        d[1] = Sinu0 * x - Cosu0 * y;
        d[2] = Cosu1 * x + Sinu1 * y;
        d[3] = Sinu1 * x - Cosu1 * y;
        d[4] = Cosu2 * x + Sinu2 * y;
        d[5] = Sinu2 * x - Cosu2 * y;
        d[6] = Cosu3 * x + Sinu3 * y;
        d[7] = Sinu3 * x - Cosu3 * y;
        d[8] = Cosu4 * x + Sinu4 * y;
        d[9] = Sinu4 * x - Cosu4 * y;
        d[10] = Cosu5 * x + Sinu5 * y;
        d[11] = Sinu5 * x - Cosu5 * y;
        d[12] = Cosu6 * x + Sinu6 * y;
        d[13] = Sinu6 * x - Cosu6 * y;
        d[14] = z;
        d[15] = z;

        for (int i = 0; i < 16; ++i)
        {
            if (min[i] > d[i])
            {
                min[i] = d[i];
            }
            if (max[i] < d[i])
            {
                max[i] = d[i];
            }
        }
    }

    /// <summary>Widens the box by <paramref name="tol"/> on each bound.</summary>
    public static void EnlargeMinMax(double tol, Span<double> min, Span<double> max)
    {
        int i = 0;
        while (i < 16)
        {
            min[i] -= tol;
            max[i] += tol;
            i++;
        }
    }

    /// <summary>Initializes the box to the empty one (bounds at +/- <paramref name="big"/>).</summary>
    public static void InitMinMax(double big, Span<double> min, Span<double> max)
    {
        int i = 0;
        while (i < 16)
        {
            min[i] = big;
            max[i] = -big;
            i++;
        }
    }

    /// <summary>Packs the indices of <paramref name="min"/> and <paramref name="max"/> into <paramref name="mm"/>, two 15-bit indices per integer.</summary>
    public static void EncodeMinMax(ref EdgesBlock.MinMaxIndices min, ref EdgesBlock.MinMaxIndices max,
                                    ref EdgesBlock.MinMaxIndices mm)
    {
        mm.Min[0] = min.Min[1] & 0x00007fff;
        mm.Max[0] = max.Min[1] & 0x00007fff;
        mm.Min[0] += (min.Min[0] & 0x00007fff) << 16;
        mm.Max[0] += (max.Min[0] & 0x00007fff) << 16;
        mm.Min[1] = min.Min[3] & 0x00007fff;
        mm.Max[1] = max.Min[3] & 0x00007fff;
        mm.Min[1] += (min.Min[2] & 0x00007fff) << 16;
        mm.Max[1] += (max.Min[2] & 0x00007fff) << 16;
        mm.Min[2] = min.Min[5] & 0x00007fff;
        mm.Max[2] = max.Min[5] & 0x00007fff;
        mm.Min[2] += (min.Min[4] & 0x00007fff) << 16;
        mm.Max[2] += (max.Min[4] & 0x00007fff) << 16;
        mm.Min[3] = min.Min[7] & 0x00007fff;
        mm.Max[3] = max.Min[7] & 0x00007fff;
        mm.Min[3] += (min.Min[6] & 0x00007fff) << 16;
        mm.Max[3] += (max.Min[6] & 0x00007fff) << 16;
        mm.Min[4] = min.Max[1] & 0x00007fff;
        mm.Max[4] = max.Max[1] & 0x00007fff;
        mm.Min[4] += (min.Max[0] & 0x00007fff) << 16;
        mm.Max[4] += (max.Max[0] & 0x00007fff) << 16;
        mm.Min[5] = min.Max[3] & 0x00007fff;
        mm.Max[5] = max.Max[3] & 0x00007fff;
        mm.Min[5] += (min.Max[2] & 0x00007fff) << 16;
        mm.Max[5] += (max.Max[2] & 0x00007fff) << 16;
        mm.Min[6] = min.Max[5] & 0x00007fff;
        mm.Max[6] = max.Max[5] & 0x00007fff;
        mm.Min[6] += (min.Max[4] & 0x00007fff) << 16;
        mm.Max[6] += (max.Max[4] & 0x00007fff) << 16;
        mm.Min[7] = min.Max[7] & 0x00007fff;
        mm.Max[7] = max.Max[7] & 0x00007fff;
        mm.Min[7] += (min.Max[6] & 0x00007fff) << 16;
        mm.Max[7] += (max.Max[6] & 0x00007fff) << 16;
    }

    /// <summary>The product of the sides of the (unpacked) box (<paramref name="min"/>, <paramref name="max"/>), the depth's last two excluded.</summary>
    public static double SizeBox(ref EdgesBlock.MinMaxIndices min, ref EdgesBlock.MinMaxIndices max)
    {
        double s = max.Min[0] - min.Min[0];
        for (int i = 1; i < 8; ++i)
        {
            s *= max.Min[i] - min.Min[i];
        }
        for (int i = 0; i < 6; ++i)
        {
            s *= max.Max[i] - min.Max[i];
        }
        return s;
    }

    /// <summary>Unpacks <paramref name="mm"/> into <paramref name="min"/> and <paramref name="max"/>.</summary>
    public static void DecodeMinMax(in EdgesBlock.MinMaxIndices mm, ref EdgesBlock.MinMaxIndices min,
                                    ref EdgesBlock.MinMaxIndices max)
    {
        min.Min[0] = (mm.Min[0] & 0x7fff0000) >> 16;
        max.Min[0] = (mm.Max[0] & 0x7fff0000) >> 16;
        min.Min[1] = mm.Min[0] & 0x00007fff;
        max.Min[1] = mm.Max[0] & 0x00007fff;
        min.Min[2] = (mm.Min[1] & 0x7fff0000) >> 16;
        max.Min[2] = (mm.Max[1] & 0x7fff0000) >> 16;
        min.Min[3] = mm.Min[1] & 0x00007fff;
        max.Min[3] = mm.Max[1] & 0x00007fff;
        min.Min[4] = (mm.Min[2] & 0x7fff0000) >> 16;
        max.Min[4] = (mm.Max[2] & 0x7fff0000) >> 16;
        min.Min[5] = mm.Min[2] & 0x00007fff;
        max.Min[5] = mm.Max[2] & 0x00007fff;
        min.Min[6] = (mm.Min[3] & 0x7fff0000) >> 16;
        max.Min[6] = (mm.Max[3] & 0x7fff0000) >> 16;
        min.Min[7] = mm.Min[3] & 0x00007fff;
        max.Min[7] = mm.Max[3] & 0x00007fff;
        min.Max[0] = (mm.Min[4] & 0x7fff0000) >> 16;
        max.Max[0] = (mm.Max[4] & 0x7fff0000) >> 16;
        min.Max[1] = mm.Min[4] & 0x00007fff;
        max.Max[1] = mm.Max[4] & 0x00007fff;
        min.Max[2] = (mm.Min[5] & 0x7fff0000) >> 16;
        max.Max[2] = (mm.Max[5] & 0x7fff0000) >> 16;
        min.Max[3] = mm.Min[5] & 0x00007fff;
        max.Max[3] = mm.Max[5] & 0x00007fff;
        min.Max[4] = (mm.Min[6] & 0x7fff0000) >> 16;
        max.Max[4] = (mm.Max[6] & 0x7fff0000) >> 16;
        min.Max[5] = mm.Min[6] & 0x00007fff;
        max.Max[5] = mm.Max[6] & 0x00007fff;
        min.Max[6] = (mm.Min[7] & 0x7fff0000) >> 16;
        max.Max[6] = (mm.Max[7] & 0x7fff0000) >> 16;
        min.Max[7] = mm.Min[7] & 0x00007fff;
        max.Max[7] = mm.Max[7] & 0x00007fff;
    }

    /// <summary>Copies (<paramref name="iMin"/>, <paramref name="iMax"/>) into (<paramref name="oMin"/>, <paramref name="oMax"/>).</summary>
    public static void CopyMinMax(ref EdgesBlock.MinMaxIndices iMin, ref EdgesBlock.MinMaxIndices iMax,
                                  ref EdgesBlock.MinMaxIndices oMin, ref EdgesBlock.MinMaxIndices oMax)
    {
        oMin = iMin;
        oMax = iMax;
    }

    /// <summary>Widens (<paramref name="oMin"/>, <paramref name="oMax"/>) to (<paramref name="iMin"/>, <paramref name="iMax"/>).</summary>
    public static void AddMinMax(ref EdgesBlock.MinMaxIndices iMin, ref EdgesBlock.MinMaxIndices iMax,
                                 ref EdgesBlock.MinMaxIndices oMin, ref EdgesBlock.MinMaxIndices oMax)
    {
        oMin.Minimize(iMin);
        oMax.Maximize(iMax);
    }
}
