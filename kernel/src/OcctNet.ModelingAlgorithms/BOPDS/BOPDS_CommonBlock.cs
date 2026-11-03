// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_CommonBlock.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_CommonBlock.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// A common block: pave blocks of different edges that coincide (they share one edge in the result), with the
/// indices of the faces on which they lie and the common block's tolerance.
/// </summary>
/// <remarks>
/// A working object, compared by reference (plan section 6). The first pave block is the one with the smallest
/// original edge index (<see cref="AddPaveBlock"/>), unless <see cref="SetRealPaveBlock"/> chose another.
/// </remarks>
public class BOPDS_CommonBlock : Transient
{
    protected NCollection_List<BOPDS_PaveBlock> myPaveBlocks = new(); //!< Pave blocks of the common block
    protected NCollection_List<int> myFaces = new();                   //!< Faces on which the pave blocks are lying
    protected double myTolerance;                                      //!< Tolerance of the common block

    public BOPDS_CommonBlock()
    {
        myTolerance = 0.0;
    }

    /// <summary>Adds the pave block, first if its original edge has the smallest index so far.</summary>
    internal void AddPaveBlock(BOPDS_PaveBlock aPB)
    {
        if (myPaveBlocks.IsEmpty())
        {
            myPaveBlocks.Append(aPB);
            return;
        }
        //
        // Put the pave block with the minimal index of the original edge in the first place
        if (aPB.OriginalEdge() < myPaveBlocks.First().OriginalEdge())
        {
            myPaveBlocks.Prepend(aPB);
        }
        else
        {
            myPaveBlocks.Append(aPB);
        }
    }

    /// <summary>Replaces the pave blocks, adding each as <see cref="AddPaveBlock"/>.</summary>
    internal void SetPaveBlocks(NCollection_List<BOPDS_PaveBlock> aLPB)
    {
        myPaveBlocks.Clear();
        for (NCollection_List<BOPDS_PaveBlock>.Iterator aIt = new(aLPB); aIt.More(); aIt.Next())
        {
            AddPaveBlock(aIt.Value());
        }
    }

    /// <summary>The pave blocks (the common block's own list; read only for callers).</summary>
    public NCollection_List<BOPDS_PaveBlock> PaveBlocks() => myPaveBlocks;

    internal void AddFace(int aF) => myFaces.Append(aF);

    /// <summary>Replaces the faces by a copy of <paramref name="aLF"/>.</summary>
    internal void SetFaces(NCollection_List<int> aLF) => myFaces.Assign(aLF);

    /// <summary>Appends the faces; as OCCT's <c>NCollection_List::Append(list&amp;)</c>, it empties <paramref name="aLF"/>.</summary>
    internal void AppendFaces(NCollection_List<int> aLF) => myFaces.Append(aLF);

    /// <summary>The faces (the common block's own list; read only for callers).</summary>
    public NCollection_List<int> Faces() => myFaces;

    /// <summary>The first pave block.</summary>
    public BOPDS_PaveBlock PaveBlock1() => myPaveBlocks.First();

    /// <summary>Makes the pave block the first one, if it is in the common block.</summary>
    internal void SetRealPaveBlock(BOPDS_PaveBlock thePB)
    {
        for (NCollection_List<BOPDS_PaveBlock>.Iterator it = new(myPaveBlocks); it.More(); it.Next())
        {
            if (it.Value() == thePB)
            {
                // OCCT prepends, then removes the iterator's node; the port's list is indexed, so it removes
                // first (the same list results)
                myPaveBlocks.Remove(it);
                myPaveBlocks.Prepend(thePB);
                break;
            }
        }
    }

    /// <summary>The pave block on the original edge <paramref name="aIx"/>, or null.</summary>
    public BOPDS_PaveBlock? PaveBlockOnEdge(int aIx)
    {
        int aIOr;
        for (NCollection_List<BOPDS_PaveBlock>.Iterator anIt = new(myPaveBlocks); anIt.More(); anIt.Next())
        {
            BOPDS_PaveBlock aPB = anIt.Value();
            aIOr = aPB.OriginalEdge();
            if (aIOr == aIx)
            {
                return aPB;
            }
        }
        return null;
    }

    /// <summary>True if the common block lies on the face <paramref name="aIx"/>.</summary>
    public bool IsPaveBlockOnFace(int aIx)
    {
        bool bFound;
        int nF;
        //
        bFound = false;
        for (NCollection_List<int>.Iterator anIt = new(myFaces); anIt.More(); anIt.Next())
        {
            nF = anIt.Value();
            if (nF == aIx)
            {
                return !bFound;
            }
        }
        return bFound;
    }

    /// <summary>True if the common block has a pave block on the original edge <paramref name="aIx"/>.</summary>
    public bool IsPaveBlockOnEdge(int aIx)
    {
        bool bFound;
        int aIOr;
        //
        bFound = false;
        for (NCollection_List<BOPDS_PaveBlock>.Iterator anIt = new(myPaveBlocks); anIt.More(); anIt.Next())
        {
            BOPDS_PaveBlock aPB = anIt.Value();
            aIOr = aPB.OriginalEdge();
            if (aIOr == aIx)
            {
                return !bFound;
            }
        }
        return bFound;
    }

    /// <summary>Sets the edge of every pave block.</summary>
    internal void SetEdge(int theEdge)
    {
        for (NCollection_List<BOPDS_PaveBlock>.Iterator anIt = new(myPaveBlocks); anIt.More(); anIt.Next())
        {
            BOPDS_PaveBlock aPB = anIt.Value();
            aPB.SetEdge(theEdge);
        }
    }

    /// <summary>The edge of the first pave block (-1 if it is null).</summary>
    public int Edge()
    {
        BOPDS_PaveBlock? aPB1 = myPaveBlocks.First();
        if (aPB1 is not null)
        {
            return aPB1.Edge();
        }
        return -1;
    }

    /// <summary>True if the common block holds the pave block.</summary>
    public bool Contains(BOPDS_PaveBlock aPBx)
    {
        bool bFound;
        int aNb1;
        //
        bFound = false;
        aNb1 = myPaveBlocks.Extent();
        //
        if (aNb1 == 0)
        {
            return bFound;
        }
        //
        for (NCollection_List<BOPDS_PaveBlock>.Iterator anIt = new(myPaveBlocks); anIt.More(); anIt.Next())
        {
            BOPDS_PaveBlock aPB = anIt.Value();
            if (aPB == aPBx)
            {
                return !bFound;
            }
        }
        return bFound;
    }

    /// <summary>True if the common block lies on the face <paramref name="theF"/>.</summary>
    public bool Contains(int theF)
    {
        bool bFound;
        //
        bFound = false;
        for (NCollection_List<int>.Iterator aIt = new(myFaces); aIt.More(); aIt.Next())
        {
            if (aIt.Value() == theF)
            {
                return !bFound;
            }
        }
        return bFound;
    }

    internal void SetTolerance(double theTol) => myTolerance = theTol;

    public double Tolerance() => myTolerance;

    /// <summary>Writes the common block as OCCT's <c>Dump</c> prints it.</summary>
    public void Dump(TextWriter theOS)
    {
        int nF;
        //
        theOS.Write(" -- CB:\n");
        for (NCollection_List<BOPDS_PaveBlock>.Iterator aItPB = new(myPaveBlocks); aItPB.More(); aItPB.Next())
        {
            BOPDS_PaveBlock aPB = aItPB.Value();
            aPB.Dump(theOS);
            theOS.Write("\n");
        }
        //
        if (myFaces.Extent() != 0)
        {
            theOS.Write(" Faces:");
            for (NCollection_List<int>.Iterator aIt = new(myFaces); aIt.More(); aIt.Next())
            {
                nF = aIt.Value();
                theOS.Write(" " + nF);
            }
            theOS.Write("\n");
        }
    }
}
