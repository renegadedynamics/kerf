// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Meshing;

/// <summary>
/// Which of a mesh's links a pass over its triangles has met, by their indices: a bit each. While the structure doesn't
/// change, a link's index is its pair of nodes, so this answers what a set of node pairs would, without hashing or
/// growing one (decision 1977). One face's, on one thread.
/// </summary>
internal sealed class LinkMarks
{
    private ulong[] _bits = [];

    /// <summary>Unmarks every link, for a structure of <paramref name="linkCount"/> links, indexed from 1.</summary>
    public void Reset(int linkCount)
    {
        int words = (linkCount >> 6) + 1;
        if (_bits.Length < words)
        {
            _bits = new ulong[Math.Max(words, 2 * _bits.Length)];
        }
        else
        {
            Array.Clear(_bits, 0, words);
        }
    }

    /// <summary>Marks the link; false if it was marked already.</summary>
    public bool Add(int index)
    {
        ref ulong word = ref _bits[index >> 6];
        ulong bit = 1UL << (index & 63);
        if ((word & bit) != 0)
        {
            return false;
        }

        word |= bit;
        return true;
    }

    /// <summary>Whether the link is marked.</summary>
    public bool IsMarked(int index) => (_bits[index >> 6] & (1UL << (index & 63))) != 0;
}
