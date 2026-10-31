// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_BlockBuilder.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_BlockBuilder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Computes the connexity blocks of the elements of a <see cref="TopOpeBRepBuild_ShapeSet"/>: each block is a range
/// of indices in a map of the elements, which are valid or not.
/// </summary>
/// <remarks>
/// A working object. The map of elements compares shapes with IsEqual (OCCT's default hasher), so the two orientations
/// of an edge are two elements.
/// </remarks>
public sealed class TopOpeBRepBuild_BlockBuilder
{
    private readonly NCollection_DataMap<int, int> myOrientedShapeMapIsValid = new();
    private readonly NCollection_IndexedMap<TopoDS_Shape> myOrientedShapeMap = new(EqualityComparer<TopoDS_Shape>.Default);
    private readonly NCollection_Sequence<int> myBlocks = new();
    private int myBlockIndex;
#pragma warning disable CS0414 // OCCT sets the flag and never reads it
    private bool myIsDone;
#pragma warning restore CS0414
    private readonly NCollection_Sequence<int> myBlocksIsRegular = new();

    /// <summary>An empty block builder.</summary>
    public TopOpeBRepBuild_BlockBuilder()
    {
        myIsDone = false;
    }

    /// <summary>The blocks of <paramref name="SS"/>.</summary>
    public TopOpeBRepBuild_BlockBuilder(TopOpeBRepBuild_ShapeSet SS)
    {
        myIsDone = false;
        MakeBlock(SS);
    }

    /// <summary>Computes the connexity blocks of the start elements of <paramref name="SS"/>.</summary>
    public void MakeBlock(TopOpeBRepBuild_ShapeSet SS)
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

        myOrientedShapeMap.Clear();
        myOrientedShapeMapIsValid.Clear();
        myBlocks.Clear();
        myBlocksIsRegular.Clear();

        bool IsRegular;
        int CurNei;
        int Mextent;
        int Eindex;

        for (SS.InitStartElements(); SS.MoreStartElements(); SS.NextStartElement())
        {
            TopoDS_Shape E = SS.StartElement();
            Mextent = myOrientedShapeMap.Extent();
            Eindex = AddElement(E);

            // E = current element of the element set SS
            // Mextent = index of last element stored in map M, before E is added.
            // Eindex  = index of E added to M : Eindex > Mextent => E is new in M

            bool EnewinM = Eindex > Mextent;
            if (EnewinM)
            {
                // make a new block starting at element Eindex
                myBlocks.Append(Eindex);
                IsRegular = true;
                CurNei = 0;
                // put in current block all the elements connex to E :
                // while an element E has been added to M
                //    - compute neighbours of E : N(E)
                //    - add each element N of N(E) to M

                Mextent = myOrientedShapeMap.Extent();
                bool searchneighbours = Eindex <= Mextent;
                while (searchneighbours)
                {
                    // E = element of M on which neighbours must be searched
                    // first step : Eindex = index of starting element of SS
                    // next steps : Eindex = index of neighbours of starting element of SS
                    TopoDS_Shape E1 = myOrientedShapeMap.FindKey(Eindex);
                    CurNei = SS.MaxNumberSubShape(E1);
                    bool condregu = true;
                    if (CurNei > 2)
                    {
                        condregu = false;
                    }
                    IsRegular = IsRegular && condregu;
                    // compute neighbours of E : add them to M to increase M.Extent().
                    SS.InitNeighbours(E1);

                    for (; SS.MoreNeighbours(); SS.NextNeighbour())
                    {
                        TopoDS_Shape N = SS.Neighbour();
                        AddElement(N);
                    }

                    Eindex++;
                    Mextent = myOrientedShapeMap.Extent();
                    searchneighbours = Eindex <= Mextent;
                } // while (searchneighbours)
                int iiregu = IsRegular ? 1 : 0;
                myBlocksIsRegular.Append(iiregu);
            } // if (EnewinM)
        } // for ()

        // To value the l bound of the last connexity block created above,
        // we create an artificial block of value = myOrientedShapeMap.Extent() + 1
        // The real number of connexity blocks is myBlocks.Length() - 1
        Mextent = myOrientedShapeMap.Extent();
        myBlocks.Append(Mextent + 1);
        myIsDone = true;
    }

    /// <summary>Starts the iteration on the blocks.</summary>
    public void InitBlock()
    {
        myBlockIndex = 1;
    }

    /// <summary>Whether there is a current block.</summary>
    public bool MoreBlock()
    {
        // the length of myBlocks is 1 + number of connexity blocks
        int l = myBlocks.Length();
        bool b = myBlockIndex < l;
        return b;
    }

    /// <summary>Moves to the next block.</summary>
    public void NextBlock()
    {
        myBlockIndex++;
    }

    /// <summary>An iterator on the elements of the current block.</summary>
    public TopOpeBRepBuild_BlockIterator BlockIterator()
    {
        int lower = myBlocks.Value(myBlockIndex);
        int upper = myBlocks.Value(myBlockIndex + 1) - 1;
        return new TopOpeBRepBuild_BlockIterator(lower, upper);
    }

    /// <summary>The current element of <paramref name="BI"/>.</summary>
    /// <exception cref="Standard_Failure">The iterator is at its end.</exception>
    public TopoDS_Shape Element(in TopOpeBRepBuild_BlockIterator BI)
    {
        bool isbound = BI.More();
        if (!isbound)
        {
            throw new Standard_Failure("OutOfRange");
        }

        int index = BI.Value();
        TopoDS_Shape E = myOrientedShapeMap.FindKey(index);
        return E;
    }

    /// <summary>The element of index <paramref name="index"/>.</summary>
    /// <exception cref="Standard_Failure">There is none.</exception>
    public TopoDS_Shape Element(int index)
    {
        bool isbound = myOrientedShapeMapIsValid.IsBound(index);
        if (!isbound)
        {
            throw new Standard_Failure("OutOfRange");
        }

        TopoDS_Shape E = myOrientedShapeMap.FindKey(index);
        return E;
    }

    /// <summary>The index of the element <paramref name="E"/>.</summary>
    /// <exception cref="Standard_Failure">It isn't an element.</exception>
    public int Element(in TopoDS_Shape E)
    {
        bool isbound = myOrientedShapeMap.Contains(E);
        if (!isbound)
        {
            throw new Standard_Failure("OutOfRange");
        }

        int I = myOrientedShapeMap.FindIndex(E);
        return I;
    }

    /// <summary>Whether the current element of <paramref name="BI"/> is valid.</summary>
    public bool ElementIsValid(in TopOpeBRepBuild_BlockIterator BI)
    {
        bool isbound = BI.More();
        if (!isbound)
        {
            return false;
        }

        int Sindex = BI.Value();
        int isb = myOrientedShapeMapIsValid.Find(Sindex);
        bool isvalid = isb == 1;

        return isvalid;
    }

    /// <summary>Whether the element of index <paramref name="Sindex"/> is valid.</summary>
    public bool ElementIsValid(int Sindex)
    {
        bool isbound = myOrientedShapeMapIsValid.IsBound(Sindex);
        if (!isbound)
        {
            return false;
        }

        int isb = myOrientedShapeMapIsValid.Find(Sindex);
        bool isvalid = isb == 1;

        return isvalid;
    }

    /// <summary>Adds <paramref name="S"/> (valid) to the elements; its index.</summary>
    public int AddElement(in TopoDS_Shape S)
    {
        int Sindex = myOrientedShapeMap.Add(S);
        myOrientedShapeMapIsValid.Bind(Sindex, 1);

        return Sindex;
    }

    /// <summary>Marks the current element of <paramref name="BI"/> valid or not.</summary>
    public void SetValid(in TopOpeBRepBuild_BlockIterator BI, bool isvalid)
    {
        bool isbound = BI.More();
        if (!isbound)
        {
            return;
        }

        int Sindex = BI.Value();
        int i = isvalid ? 1 : 0;
        myOrientedShapeMapIsValid.Bind(Sindex, i);
    }

    /// <summary>Marks the element of index <paramref name="Sindex"/> valid or not.</summary>
    public void SetValid(int Sindex, bool isvalid)
    {
        bool isbound = myOrientedShapeMapIsValid.IsBound(Sindex);
        if (!isbound)
        {
            return;
        }

        int i = isvalid ? 1 : 0;
        myOrientedShapeMapIsValid.Bind(Sindex, i);
    }

    /// <summary>Whether no element of the current block has more than two neighbours through one sub-shape.</summary>
    public bool CurrentBlockIsRegular()
    {
        bool b = false;
        int i = myBlocksIsRegular.Value(myBlockIndex);
        if (i == 1)
        {
            b = true;
        }
        return b;
    }
}
