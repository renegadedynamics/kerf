// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_EdgesBlock.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_EdgesBlock.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.CompilerServices;

namespace Occt;

/// <summary>
/// A block of edges of the hidden-line data structure (the edges of a wire): the edges' indices and flags
/// (orientation, outline, internal, double, isoline), and the block's encoded min-max box.
/// </summary>
/// <remarks>A working object (a transient), as OCCT's: its setters are public.</remarks>
public class HLRAlgo_EdgesBlock : Standard_Transient
{
    /// <summary>Eight integers, as OCCT's <c>int[8]</c>.</summary>
    [InlineArray(8)]
    public struct Int8
    {
        private int myElement0;
    }

    /// <summary>
    /// The sixteen integer bounds of a min-max box (OCCT's <c>int Min[8], Max[8]</c>): raw indices, or two 15-bit
    /// indices packed per integer (<see cref="HLRAlgo.EncodeMinMax"/>).
    /// </summary>
    /// <remarks>
    /// A plain C++ struct, ported as a struct with OCCT's public fields: the hidden-line algorithms read and write
    /// them by index. <c>new()</c> and <c>default</c> are the same (zeros; OCCT's are unset).
    /// </remarks>
    public struct MinMaxIndices : IEquatable<MinMaxIndices>
    {
        public Int8 Min;
        public Int8 Max;

        /// <summary>Takes, index by index, the smaller of this and <paramref name="theMinMaxIndices"/>.</summary>
        [System.Diagnostics.CodeAnalysis.UnscopedRef]
        public ref MinMaxIndices Minimize(in MinMaxIndices theMinMaxIndices)
        {
            for (int aI = 0; aI < 8; ++aI)
            {
                if (Min[aI] > theMinMaxIndices.Min[aI])
                {
                    Min[aI] = theMinMaxIndices.Min[aI];
                }
                if (Max[aI] > theMinMaxIndices.Max[aI])
                {
                    Max[aI] = theMinMaxIndices.Max[aI];
                }
            }
            return ref this;
        }

        /// <summary>Takes, index by index, the larger of this and <paramref name="theMinMaxIndices"/>.</summary>
        [System.Diagnostics.CodeAnalysis.UnscopedRef]
        public ref MinMaxIndices Maximize(in MinMaxIndices theMinMaxIndices)
        {
            for (int aI = 0; aI < 8; ++aI)
            {
                if (Min[aI] < theMinMaxIndices.Min[aI])
                {
                    Min[aI] = theMinMaxIndices.Min[aI];
                }
                if (Max[aI] < theMinMaxIndices.Max[aI])
                {
                    Max[aI] = theMinMaxIndices.Max[aI];
                }
            }
            return ref this;
        }

        public readonly bool Equals(MinMaxIndices other)
            => ((ReadOnlySpan<int>)Min).SequenceEqual(other.Min) && ((ReadOnlySpan<int>)Max).SequenceEqual(other.Max);

        public override readonly bool Equals(object? obj) => obj is MinMaxIndices other && Equals(other);

        public override readonly int GetHashCode() => HashCode.Combine(Min[0], Min[7], Max[0], Max[7]);

        public static bool operator ==(in MinMaxIndices theLeft, in MinMaxIndices theRight) => theLeft.Equals(theRight);

        public static bool operator !=(in MinMaxIndices theLeft, in MinMaxIndices theRight) => !theLeft.Equals(theRight);

        public override readonly string ToString()
            => $"MinMaxIndices([{string.Join(", ", ((ReadOnlySpan<int>)Min).ToArray())}], [{string.Join(", ", ((ReadOnlySpan<int>)Max).ToArray())}])";
    }

    private const int EMaskOrient = 15;
    private const int EMaskOutLine = 16;
    private const int EMaskInternal = 32;
    private const int EMaskDouble = 64;
    private const int EMaskIsoLine = 128;

    private readonly NCollection_Array1<int> myEdges;
    private readonly NCollection_Array1<int> myFlags;
    private MinMaxIndices myMinMax;

    /// <summary>A block of <paramref name="NbEdges"/> edges, for a wire.</summary>
    public HLRAlgo_EdgesBlock(int NbEdges)
    {
        myEdges = new NCollection_Array1<int>(1, NbEdges);
        myFlags = new NCollection_Array1<int>(1, NbEdges);
    }

    public int NbEdges() => myEdges.Upper();

    public void Edge(int I, int EI) => myEdges[I] = EI;

    public int Edge(int I) => myEdges[I];

    public void Orientation(int I, TopAbs_Orientation Or)
    {
        myFlags[I] &= ~EMaskOrient;
        myFlags[I] |= ((int)Or & EMaskOrient);
    }

    public TopAbs_Orientation Orientation(int I) => (TopAbs_Orientation)(myFlags[I] & EMaskOrient);

    public bool OutLine(int I) => (myFlags[I] & EMaskOutLine) != 0;

    public void OutLine(int I, bool B)
    {
        if (B)
        {
            myFlags[I] |= EMaskOutLine;
        }
        else
        {
            myFlags[I] &= ~EMaskOutLine;
        }
    }

    public bool Internal(int I) => (myFlags[I] & EMaskInternal) != 0;

    public void Internal(int I, bool B)
    {
        if (B)
        {
            myFlags[I] |= EMaskInternal;
        }
        else
        {
            myFlags[I] &= ~EMaskInternal;
        }
    }

    public bool Double(int I) => (myFlags[I] & EMaskDouble) != 0;

    public void Double(int I, bool B)
    {
        if (B)
        {
            myFlags[I] |= EMaskDouble;
        }
        else
        {
            myFlags[I] &= ~EMaskDouble;
        }
    }

    public bool IsoLine(int I) => (myFlags[I] & EMaskIsoLine) != 0;

    public void IsoLine(int I, bool B)
    {
        if (B)
        {
            myFlags[I] |= EMaskIsoLine;
        }
        else
        {
            myFlags[I] &= ~EMaskIsoLine;
        }
    }

    public void UpdateMinMax(in MinMaxIndices TotMinMax) => myMinMax = TotMinMax;

    /// <summary>The block's encoded min-max box, by reference.</summary>
    public ref MinMaxIndices MinMax() => ref myMinMax;
}
