// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_EdgesBlock.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_EdgesBlock.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Topology;
using Occt;
using System.Runtime.CompilerServices;

namespace Kerf.Engine.HiddenLines.Algorithm;

/// <summary>
/// A block of edges of the hidden-line data structure (the edges of a wire): the edges' indices and flags
/// (orientation, outline, internal, double, isoline), and the block's encoded min-max box.
/// </summary>
/// <remarks>A working object (a transient), as OCCT's: its setters are public.</remarks>
public class EdgesBlock : Transient
{
    /// <summary>Eight integers, as OCCT's <c>int[8]</c>.</summary>
    [InlineArray(8)]
    public struct Int8
    {
        private int _element0;
    }

    /// <summary>
    /// The sixteen integer bounds of a min-max box (OCCT's <c>int Min[8], Max[8]</c>): raw indices, or two 15-bit
    /// indices packed per integer (<see cref="HlrAlgo.EncodeMinMax"/>).
    /// </summary>
    /// <remarks>
    /// A plain C++ struct, ported as a struct with OCCT's public fields: the hidden-line algorithms read and write
    /// them by index. <c>new()</c> and <c>default</c> are the same (zeros; OCCT's are unset).
    /// </remarks>
    public struct MinMaxIndices : IEquatable<MinMaxIndices>
    {
        public Int8 Min;
        public Int8 Max;

        /// <summary>Takes, index by index, the smaller of this and <paramref name="minMaxIndices"/>.</summary>
        [System.Diagnostics.CodeAnalysis.UnscopedRef]
        public ref MinMaxIndices Minimize(in MinMaxIndices minMaxIndices)
        {
            for (int i = 0; i < 8; ++i)
            {
                if (Min[i] > minMaxIndices.Min[i])
                {
                    Min[i] = minMaxIndices.Min[i];
                }
                if (Max[i] > minMaxIndices.Max[i])
                {
                    Max[i] = minMaxIndices.Max[i];
                }
            }
            return ref this;
        }

        /// <summary>Takes, index by index, the larger of this and <paramref name="minMaxIndices"/>.</summary>
        [System.Diagnostics.CodeAnalysis.UnscopedRef]
        public ref MinMaxIndices Maximize(in MinMaxIndices minMaxIndices)
        {
            for (int i = 0; i < 8; ++i)
            {
                if (Min[i] < minMaxIndices.Min[i])
                {
                    Min[i] = minMaxIndices.Min[i];
                }
                if (Max[i] < minMaxIndices.Max[i])
                {
                    Max[i] = minMaxIndices.Max[i];
                }
            }
            return ref this;
        }

        public readonly bool Equals(MinMaxIndices other)
            => ((ReadOnlySpan<int>)Min).SequenceEqual(other.Min) && ((ReadOnlySpan<int>)Max).SequenceEqual(other.Max);

        public override readonly bool Equals(object? obj) => obj is MinMaxIndices other && Equals(other);

        public override readonly int GetHashCode() => HashCode.Combine(Min[0], Min[7], Max[0], Max[7]);

        public static bool operator ==(in MinMaxIndices left, in MinMaxIndices right) => left.Equals(right);

        public static bool operator !=(in MinMaxIndices left, in MinMaxIndices right) => !left.Equals(right);

        public override readonly string ToString()
            => $"MinMaxIndices([{string.Join(", ", ((ReadOnlySpan<int>)Min).ToArray())}], [{string.Join(", ", ((ReadOnlySpan<int>)Max).ToArray())}])";
    }

    private const int EMaskOrient = 15;
    private const int EMaskOutLine = 16;
    private const int EMaskInternal = 32;
    private const int EMaskDouble = 64;
    private const int EMaskIsoLine = 128;

    private readonly NCollection_Array1<int> _edges;
    private readonly NCollection_Array1<int> _flags;
    private MinMaxIndices _minMax;

    /// <summary>A block of <paramref name="nbEdges"/> edges, for a wire.</summary>
    public EdgesBlock(int nbEdges)
    {
        _edges = new NCollection_Array1<int>(1, nbEdges);
        _flags = new NCollection_Array1<int>(1, nbEdges);
    }

    public int EdgeCount => _edges.Upper();

    public void Edge(int i, int ei) => _edges[i] = ei;

    public int Edge(int i) => _edges[i];

    public void Orientation(int i, Orientation or)
    {
        _flags[i] &= ~EMaskOrient;
        _flags[i] |= ((int)or & EMaskOrient);
    }

    public Orientation Orientation(int i) => (Orientation)(_flags[i] & EMaskOrient);

    public bool OutLine(int i) => (_flags[i] & EMaskOutLine) != 0;

    public void OutLine(int i, bool b)
    {
        if (b)
        {
            _flags[i] |= EMaskOutLine;
        }
        else
        {
            _flags[i] &= ~EMaskOutLine;
        }
    }

    public bool Internal(int i) => (_flags[i] & EMaskInternal) != 0;

    public void Internal(int i, bool b)
    {
        if (b)
        {
            _flags[i] |= EMaskInternal;
        }
        else
        {
            _flags[i] &= ~EMaskInternal;
        }
    }

    public bool Double(int i) => (_flags[i] & EMaskDouble) != 0;

    public void Double(int i, bool b)
    {
        if (b)
        {
            _flags[i] |= EMaskDouble;
        }
        else
        {
            _flags[i] &= ~EMaskDouble;
        }
    }

    public bool IsoLine(int i) => (_flags[i] & EMaskIsoLine) != 0;

    public void IsoLine(int i, bool b)
    {
        if (b)
        {
            _flags[i] |= EMaskIsoLine;
        }
        else
        {
            _flags[i] &= ~EMaskIsoLine;
        }
    }

    public void UpdateMinMax(in MinMaxIndices totMinMax) => _minMax = totMinMax;

    /// <summary>The block's encoded min-max box, by reference.</summary>
    public ref MinMaxIndices MinMax() => ref _minMax;
}
