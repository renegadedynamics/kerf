// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_CommonBlock.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_CommonBlock.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Booleans.Data;

/// <summary>
/// A common block: pave blocks of different edges that coincide (they share one edge in the result), with the
/// indices of the faces on which they lie and the common block's tolerance.
/// </summary>
/// <remarks>
/// A working object, compared by reference (plan section 6). The first pave block is the one with the smallest
/// original edge index (<see cref="AddPaveBlock"/>), unless <see cref="SetRealPaveBlock"/> chose another.
/// </remarks>
public class CommonBlock : Transient
{
    protected NCollection_List<PaveBlock> _paveBlocks = new(); //!< Pave blocks of the common block
    protected NCollection_List<int> _faces = new();                   //!< Faces on which the pave blocks are lying
    protected double _tolerance;                                      //!< Tolerance of the common block

    public CommonBlock()
    {
        _tolerance = 0.0;
    }

    /// <summary>Adds the pave block, first if its original edge has the smallest index so far.</summary>
    internal void AddPaveBlock(PaveBlock aPB)
    {
        if (_paveBlocks.IsEmpty())
        {
            _paveBlocks.Append(aPB);
            return;
        }
        //
        // Put the pave block with the minimal index of the original edge in the first place
        if (aPB.OriginalEdge() < _paveBlocks.First().OriginalEdge())
        {
            _paveBlocks.Prepend(aPB);
        }
        else
        {
            _paveBlocks.Append(aPB);
        }
    }

    /// <summary>Replaces the pave blocks, adding each as <see cref="AddPaveBlock"/>.</summary>
    internal void SetPaveBlocks(NCollection_List<PaveBlock> aLPB)
    {
        _paveBlocks.Clear();
        for (NCollection_List<PaveBlock>.Iterator it = new(aLPB); it.More(); it.Next())
        {
            AddPaveBlock(it.Value());
        }
    }

    /// <summary>The pave blocks (the common block's own list; read only for callers).</summary>
    public NCollection_List<PaveBlock> PaveBlocks() => _paveBlocks;

    internal void AddFace(int aF) => _faces.Append(aF);

    /// <summary>Replaces the faces by a copy of <paramref name="aLF"/>.</summary>
    internal void SetFaces(NCollection_List<int> aLF) => _faces.Assign(aLF);

    /// <summary>Appends the faces; as OCCT's <c>NCollection_List::Append(list&amp;)</c>, it empties <paramref name="aLF"/>.</summary>
    internal void AppendFaces(NCollection_List<int> aLF) => _faces.Append(aLF);

    /// <summary>The faces (the common block's own list; read only for callers).</summary>
    public NCollection_List<int> Faces() => _faces;

    /// <summary>The first pave block.</summary>
    public PaveBlock PaveBlock1() => _paveBlocks.First();

    /// <summary>Makes the pave block the first one, if it is in the common block.</summary>
    internal void SetRealPaveBlock(PaveBlock pb)
    {
        for (NCollection_List<PaveBlock>.Iterator it = new(_paveBlocks); it.More(); it.Next())
        {
            if (it.Value() == pb)
            {
                // OCCT prepends, then removes the iterator's node; the port's list is indexed, so it removes
                // first (the same list results)
                _paveBlocks.Remove(it);
                _paveBlocks.Prepend(pb);
                break;
            }
        }
    }

    /// <summary>The pave block on the original edge <paramref name="aIx"/>, or null.</summary>
    public PaveBlock? PaveBlockOnEdge(int aIx)
    {
        int iOr;
        for (NCollection_List<PaveBlock>.Iterator it = new(_paveBlocks); it.More(); it.Next())
        {
            PaveBlock pb = it.Value();
            iOr = pb.OriginalEdge();
            if (iOr == aIx)
            {
                return pb;
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
        for (NCollection_List<int>.Iterator it = new(_faces); it.More(); it.Next())
        {
            nF = it.Value();
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
        int iOr;
        //
        bFound = false;
        for (NCollection_List<PaveBlock>.Iterator it = new(_paveBlocks); it.More(); it.Next())
        {
            PaveBlock pb = it.Value();
            iOr = pb.OriginalEdge();
            if (iOr == aIx)
            {
                return !bFound;
            }
        }
        return bFound;
    }

    /// <summary>Sets the edge of every pave block.</summary>
    internal void SetEdge(int edge)
    {
        for (NCollection_List<PaveBlock>.Iterator it = new(_paveBlocks); it.More(); it.Next())
        {
            PaveBlock pb = it.Value();
            pb.SetEdge(edge);
        }
    }

    /// <summary>The edge of the first pave block (-1 if it is null).</summary>
    public int Edge()
    {
        PaveBlock? pb1 = _paveBlocks.First();
        if (pb1 is not null)
        {
            return pb1.Edge();
        }
        return -1;
    }

    /// <summary>True if the common block holds the pave block.</summary>
    public bool Contains(PaveBlock aPBx)
    {
        bool bFound;
        int nb1;
        //
        bFound = false;
        nb1 = _paveBlocks.Extent();
        //
        if (nb1 == 0)
        {
            return bFound;
        }
        //
        for (NCollection_List<PaveBlock>.Iterator it = new(_paveBlocks); it.More(); it.Next())
        {
            PaveBlock pb = it.Value();
            if (pb == aPBx)
            {
                return !bFound;
            }
        }
        return bFound;
    }

    /// <summary>True if the common block lies on the face <paramref name="f"/>.</summary>
    public bool Contains(int f)
    {
        bool bFound;
        //
        bFound = false;
        for (NCollection_List<int>.Iterator it = new(_faces); it.More(); it.Next())
        {
            if (it.Value() == f)
            {
                return !bFound;
            }
        }
        return bFound;
    }

    internal void SetTolerance(double tol) => _tolerance = tol;

    public double Tolerance() => _tolerance;

    /// <summary>Writes the common block as OCCT's <c>Dump</c> prints it.</summary>
    public void Dump(TextWriter os)
    {
        int nF;
        //
        os.Write(" -- CB:\n");
        for (NCollection_List<PaveBlock>.Iterator itPB = new(_paveBlocks); itPB.More(); itPB.Next())
        {
            PaveBlock pb = itPB.Value();
            pb.Dump(os);
            os.Write("\n");
        }
        //
        if (_faces.Extent() != 0)
        {
            os.Write(" Faces:");
            for (NCollection_List<int>.Iterator it = new(_faces); it.More(); it.Next())
            {
                nF = it.Value();
                os.Write(" " + nF);
            }
            os.Write("\n");
        }
    }
}
