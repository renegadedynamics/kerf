// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_PrmPrmIntersection_T3Bits.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_PrmPrmIntersection_T3Bits.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Intersection.Patches;

/// <summary>
/// A set of bits over a cubic grid of <c>size</c>³ cells (the voxels of IntPatch_PrmPrmIntersection's start-point
/// search), in 32-bit words.
/// </summary>
/// <remarks>
/// A working object (decision 21): its mutators are public, as in OCCT. The words are <c>int</c>s and the shifts are
/// C++'s (<c>1 &lt;&lt; 31</c> is <see cref="int.MinValue"/>, <c>&gt;&gt;</c> is arithmetic), so the bits are OCCT's.
/// </remarks>
internal sealed class PrmPrmIntersectionT3Bits
{
    private readonly int[] _p;
    private readonly int _isize;

    /// <summary>A set of <paramref name="size"/>³ bits, all cleared.</summary>
    public PrmPrmIntersectionT3Bits(int size)
    {
        //-- ex: size=4  -> 4**3 = 64 bits -> 2 mots 32bits
        int nb = (size * size * size) >> 5;
        _isize = nb;
        _p = new int[nb];
        do
        {
            _p[--nb] = 0;
        } while (nb != 0);
    }

    /// <summary>Sets the bit <paramref name="t"/>.</summary>
    public void Add(int t) => _p[t >> 5] |= 1 << (int)(((uint)t) & 31);

    /// <summary>The bit <paramref name="t"/> in its word (non-zero if set).</summary>
    public int Val(int t) => _p[t >> 5] & (1 << (int)(((uint)t) & 31));

    /// <summary>Clears the bit <paramref name="t"/>.</summary>
    public void Raz(int t) => _p[t >> 5] &= ~(1 << (int)(((uint)t) & 31));

    /// <summary>Does nothing (as in OCCT).</summary>
    public void ResetAnd()
    {
        // ind = 0;
    }

    /// <summary>
    /// Finds, from the word of <paramref name="indiceprecedent"/> on, the first bit set in both sets, clears it in
    /// both, puts it in <paramref name="indiceprecedent"/> and returns 1; returns 0 if there is none.
    /// </summary>
    public int And(PrmPrmIntersectionT3Bits oth, ref int indiceprecedent)
    {
        int k = indiceprecedent >> 5;
        while (k < _isize)
        {
            int r = _p[k] & oth._p[k];
            if (r != 0)
            {
                uint c = 0;
                do
                {
                    if ((r & 1) != 0)
                    {
                        int op = (k << 5) | (int)c;
                        Raz(op);
                        oth.Raz(op);
                        indiceprecedent = op;
                        return 1;
                    }
                    c++;
                    r >>= 1;
                } while (c < 32);
            }
            k++;
        }
        return 0;
    }
}
