// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_BlockBuilder.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_BlockBuilder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Topology;
using Kerf.Engine;

namespace Kerf.Engine.Booleans.Legacy.Build;

/// <summary>
/// Computes the connexity blocks of the elements of a <see cref="BuildShapeSet"/>: each block is a range
/// of indices in a map of the elements, which are valid or not.
/// </summary>
/// <remarks>
/// A working object. The map of elements compares shapes with IsEqual (OCCT's default hasher), so the two orientations
/// of an edge are two elements.
/// </remarks>
internal sealed class BlockBuilder
{
    private readonly Dictionary<int, int> _orientedShapeMapIsValid = new();
    private readonly IndexedSet<Shape> _orientedShapeMap = new(EqualityComparer<Shape>.Default);
    private readonly NCollection_Sequence<int> _blocks = new();
    private int _blockIndex;
#pragma warning disable CS0414 // OCCT sets the flag and never reads it
    private bool _isDone;
#pragma warning restore CS0414
    private readonly NCollection_Sequence<int> _blocksIsRegular = new();

    /// <summary>An empty block builder.</summary>
    public BlockBuilder()
    {
        _isDone = false;
    }

    /// <summary>The blocks of <paramref name="ss"/>.</summary>
    public BlockBuilder(BuildShapeSet ss)
    {
        _isDone = false;
        MakeBlock(ss);
    }

    /// <summary>Computes the connexity blocks of the start elements of <paramref name="ss"/>.</summary>
    public void MakeBlock(BuildShapeSet ss)
    {
        // Compute the set of connexity blocks of elements of element set SS
        //
        // Logic :
        // - A block is a set of connex elements of SS
        // - We assume that any element of SS appears in only 1 connexity block.
        //
        // Implementation :
        // - All the elements of all the blocks are stored in map myOrientedShapeMap(M).
        // - A connexity block is a segment [f,l] of element indices of M.
        // - myBlocks is a sequence of integer; each integer is the index of the
        //   first element (in M) of a connexity block.
        // - Bounds [f,l] of a connexity block are :
        //    f = myBlocks(i)
        //    l = myBlocks(i+1) - 1

        _orientedShapeMap.Clear();
        _orientedShapeMapIsValid.Clear();
        _blocks.Clear();
        _blocksIsRegular.Clear();

        bool isRegular;
        int curNei;
        int mextent;
        int eindex;

        for (ss.InitStartElements(); ss.MoreStartElements(); ss.NextStartElement())
        {
            Shape e = ss.StartElement();
            mextent = _orientedShapeMap.Count;
            eindex = AddElement(e);

            // E = current element of the element set SS
            // Mextent = index of last element stored in map M, before E is added.
            // Eindex  = index of E added to M : Eindex > Mextent => E is new in M

            bool enewinM = eindex > mextent;
            if (enewinM)
            {
                // make a new block starting at element Eindex
                _blocks.Append(eindex);
                isRegular = true;
                curNei = 0;
                // put in current block all the elements connex to E :
                // while an element E has been added to M
                //    - compute neighbours of E : N(E)
                //    - add each element N of N(E) to M

                mextent = _orientedShapeMap.Count;
                bool searchneighbours = eindex <= mextent;
                while (searchneighbours)
                {
                    // E = element of M on which neighbours must be searched
                    // first step : Eindex = index of starting element of SS
                    // next steps : Eindex = index of neighbours of starting element of SS
                    Shape e1 = _orientedShapeMap[eindex - 1];
                    curNei = ss.MaxNumberSubShape(e1);
                    bool condregu = true;
                    if (curNei > 2)
                    {
                        condregu = false;
                    }
                    isRegular = isRegular && condregu;
                    // compute neighbours of E : add them to M to increase M.Extent().
                    ss.InitNeighbours(e1);

                    for (; ss.MoreNeighbours(); ss.NextNeighbour())
                    {
                        Shape n = ss.Neighbour();
                        AddElement(n);
                    }

                    eindex++;
                    mextent = _orientedShapeMap.Count;
                    searchneighbours = eindex <= mextent;
                } // while (searchneighbours)
                int iiregu = isRegular ? 1 : 0;
                _blocksIsRegular.Append(iiregu);
            } // if (EnewinM)
        } // for ()

        // To value the l bound of the last connexity block created above,
        // we create an artificial block of value = myOrientedShapeMap.Extent() + 1
        // The real number of connexity blocks is myBlocks.Length() - 1
        mextent = _orientedShapeMap.Count;
        _blocks.Append(mextent + 1);
        _isDone = true;
    }

    /// <summary>Starts the iteration on the blocks.</summary>
    public void InitBlock()
    {
        _blockIndex = 1;
    }

    /// <summary>Whether there is a current block.</summary>
    public bool MoreBlock()
    {
        // the length of myBlocks is 1 + number of connexity blocks
        int l = _blocks.Length();
        bool b = _blockIndex < l;
        return b;
    }

    /// <summary>Moves to the next block.</summary>
    public void NextBlock()
    {
        _blockIndex++;
    }

    /// <summary>An iterator on the elements of the current block.</summary>
    public BlockIterator BlockIterator()
    {
        int lower = _blocks.Value(_blockIndex);
        int upper = _blocks.Value(_blockIndex + 1) - 1;
        return new BlockIterator(lower, upper);
    }

    /// <summary>The current element of <paramref name="bi"/>.</summary>
    /// <exception cref="FailureException">The iterator is at its end.</exception>
    public Shape Element(in BlockIterator bi)
    {
        bool isbound = bi.More();
        if (!isbound)
        {
            throw new FailureException("OutOfRange");
        }

        int index = bi.Value();
        Shape e = _orientedShapeMap[index - 1];
        return e;
    }

    /// <summary>The element of index <paramref name="index"/>.</summary>
    /// <exception cref="FailureException">There is none.</exception>
    public Shape Element(int index)
    {
        bool isbound = _orientedShapeMapIsValid.ContainsKey(index);
        if (!isbound)
        {
            throw new FailureException("OutOfRange");
        }

        Shape e = _orientedShapeMap[index - 1];
        return e;
    }

    /// <summary>The index of the element <paramref name="e"/>.</summary>
    /// <exception cref="FailureException">It isn't an element.</exception>
    public int Element(in Shape e)
    {
        bool isbound = _orientedShapeMap.Contains(e);
        if (!isbound)
        {
            throw new FailureException("OutOfRange");
        }

        int i = _orientedShapeMap.IndexOf(e) + 1;
        return i;
    }

    /// <summary>Whether the current element of <paramref name="bi"/> is valid.</summary>
    public bool ElementIsValid(in BlockIterator bi)
    {
        bool isbound = bi.More();
        if (!isbound)
        {
            return false;
        }

        int sindex = bi.Value();
        int isb = _orientedShapeMapIsValid.GetValue(sindex);
        bool isvalid = isb == 1;

        return isvalid;
    }

    /// <summary>Whether the element of index <paramref name="sindex"/> is valid.</summary>
    public bool ElementIsValid(int sindex)
    {
        bool isbound = _orientedShapeMapIsValid.ContainsKey(sindex);
        if (!isbound)
        {
            return false;
        }

        int isb = _orientedShapeMapIsValid.GetValue(sindex);
        bool isvalid = isb == 1;

        return isvalid;
    }

    /// <summary>Adds <paramref name="s"/> (valid) to the elements; its index.</summary>
    public int AddElement(in Shape s)
    {
        int sindex = _orientedShapeMap.Add(s) + 1;
        _orientedShapeMapIsValid[sindex] = 1;

        return sindex;
    }

    /// <summary>Marks the current element of <paramref name="bi"/> valid or not.</summary>
    public void SetValid(in BlockIterator bi, bool isvalid)
    {
        bool isbound = bi.More();
        if (!isbound)
        {
            return;
        }

        int sindex = bi.Value();
        int i = isvalid ? 1 : 0;
        _orientedShapeMapIsValid[sindex] = i;
    }

    /// <summary>Marks the element of index <paramref name="sindex"/> valid or not.</summary>
    public void SetValid(int sindex, bool isvalid)
    {
        bool isbound = _orientedShapeMapIsValid.ContainsKey(sindex);
        if (!isbound)
        {
            return;
        }

        int i = isvalid ? 1 : 0;
        _orientedShapeMapIsValid[sindex] = i;
    }

    /// <summary>Whether no element of the current block has more than two neighbours through one sub-shape.</summary>
    public bool CurrentBlockIsRegular()
    {
        bool b = false;
        int i = _blocksIsRegular.Value(_blockIndex);
        if (i == 1)
        {
            b = true;
        }
        return b;
    }
}
