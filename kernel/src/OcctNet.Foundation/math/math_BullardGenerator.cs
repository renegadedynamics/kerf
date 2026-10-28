// Port of: src/FoundationClasses/TKMath/math/math_BullardGenerator.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A fast random number generator (the algorithm proposed by Ian C. Bullard).</summary>
public class math_BullardGenerator
{
    private uint myStateHi;
    private uint myStateLo;

    public math_BullardGenerator(uint theSeed = 1)
    {
        myStateHi = theSeed;
        SetSeed(theSeed);
    }

    /// <summary>Sets a new seed, resetting the state.</summary>
    public void SetSeed(uint theSeed = 1)
    {
        myStateHi = theSeed;
        myStateLo = theSeed ^ 0x49616E42;
    }

    /// <summary>The next integer value.</summary>
    public uint NextInt()
    {
        unchecked
        {
            myStateHi = (myStateHi >> 2) + (myStateHi << 2);
            myStateHi += myStateLo;
            myStateLo += myStateHi;
        }

        return myStateHi;
    }

    /// <summary>The next real value, in [0, 1].</summary>
    public double NextReal() => NextInt() / (double)0xFFFFFFFFu;
}
