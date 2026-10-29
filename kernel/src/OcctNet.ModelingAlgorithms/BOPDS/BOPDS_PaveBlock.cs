// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_PaveBlock.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_PaveBlock.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A pave block: the part of an edge between two adjacent paves, with the extra paves found on it, the index of
/// the edge built for it (or the original edge) and its shrunk data.
/// </summary>
/// <remarks>
/// A working object (plan section 6): maps and lists of pave blocks compare them by reference and iterate in
/// insertion order (decision 5005). The sort of the paves in <see cref="Update"/> keeps MSVC's order of paves with
/// equal parameters (<see cref="std_sort"/>).
/// </remarks>
public class BOPDS_PaveBlock : Standard_Transient
{
    protected int myEdge;
    protected int myOriginalEdge;
    protected BOPDS_Pave myPave1 = new();
    protected BOPDS_Pave myPave2 = new();
    protected NCollection_List<BOPDS_Pave> myExtPaves = new();
    protected double myTS1;
    protected double myTS2;
    protected Bnd_Box myShrunkBox = new();
    protected NCollection_Map<int> myMFence = new();
    protected bool myIsSplittable;

    /// <summary>An empty pave block.</summary>
    public BOPDS_PaveBlock()
    {
        myEdge = -1;
        myOriginalEdge = -1;
        myTS1 = -99.0;
        myTS2 = myTS1;
        myIsSplittable = false;
    }

    internal void SetEdge(int theEdge) => myEdge = theEdge;

    /// <summary>The index of the edge of the pave block.</summary>
    public int Edge() => myEdge;

    public bool HasEdge() => myEdge >= 0;

    public bool HasEdge(out int theEdge)
    {
        theEdge = myEdge;
        return myEdge >= 0;
    }

    internal void SetOriginalEdge(int theEdge) => myOriginalEdge = theEdge;

    /// <summary>The index of the original edge.</summary>
    public int OriginalEdge() => myOriginalEdge;

    /// <summary>True if the edge differs from the original edge.</summary>
    public bool IsSplitEdge() => myEdge != myOriginalEdge;

    internal void SetPave1(in BOPDS_Pave thePave) => myPave1 = thePave;

    public BOPDS_Pave Pave1() => myPave1;

    internal void SetPave2(in BOPDS_Pave thePave) => myPave2 = thePave;

    public BOPDS_Pave Pave2() => myPave2;

    /// <summary>The parameters of the two paves.</summary>
    public void Range(out double theT1, out double theT2)
    {
        theT1 = myPave1.Parameter();
        theT2 = myPave2.Parameter();
    }

    /// <summary>The vertex indices of the two paves.</summary>
    public void Indices(out int theIndex1, out int theIndex2)
    {
        theIndex1 = myPave1.Index();
        theIndex2 = myPave2.Index();
    }

    /// <summary>True if the other pave block's vertex indices are this one's, in either order.</summary>
    public bool HasSameBounds(BOPDS_PaveBlock theOther)
    {
        bool bFlag1, bFlag2;
        int n11, n12, n21, n22;
        //
        Indices(out n11, out n12);
        theOther.Indices(out n21, out n22);
        //
        bFlag1 = (n11 == n21) && (n12 == n22);
        bFlag2 = (n11 == n22) && (n12 == n21);
        //
        return bFlag1 || bFlag2;
    }

    /// <summary>Appends an extra pave, unless one with the same vertex was appended already.</summary>
    internal void AppendExtPave(in BOPDS_Pave thePave)
    {
        if (myMFence.Add(thePave.Index()))
        {
            myExtPaves.Append(thePave);
        }
    }

    /// <summary>Appends an extra pave unconditionally.</summary>
    internal void AppendExtPave1(in BOPDS_Pave thePave) => myExtPaves.Append(thePave);

    /// <summary>Removes the extra paves of the vertex.</summary>
    internal void RemoveExtPave(int theVertNum)
    {
        if (myMFence.Contains(theVertNum))
        {
            NCollection_List<BOPDS_Pave>.Iterator itPaves = new(myExtPaves);
            while (itPaves.More())
            {
                if (itPaves.Value().Index() == theVertNum)
                {
                    myExtPaves.Remove(itPaves);
                }
                else
                {
                    itPaves.Next();
                }
            }
            myMFence.Remove(theVertNum);
        }
    }

    /// <summary>The extra paves (the pave block's own list; read only for callers).</summary>
    public NCollection_List<BOPDS_Pave> ExtPaves() => myExtPaves;

    internal NCollection_List<BOPDS_Pave> ChangeExtPaves() => myExtPaves;

    /// <summary>True if the pave block has extra paves.</summary>
    public bool IsToUpdate() => !myExtPaves.IsEmpty();

    /// <summary>True if an extra pave lies within <paramref name="theTol"/> of the parameter; its index then (otherwise <paramref name="theInd"/> is left as it was).</summary>
    public bool ContainsParameter(double theT, double theTol, ref int theInd)
    {
        bool bRet;
        //
        bRet = false;
        for (NCollection_List<BOPDS_Pave>.Iterator aIt = new(myExtPaves); aIt.More(); aIt.Next())
        {
            BOPDS_Pave aPave = aIt.Value();
            bRet = Math.Abs(aPave.Parameter() - theT) < theTol;
            if (bRet)
            {
                theInd = aPave.Index();
                break;
            }
        }
        return bRet;
    }

    /// <summary>
    /// Splits the pave block at its extra paves (and its two paves if <paramref name="theFlag"/>) into new pave
    /// blocks appended to <paramref name="theLPB"/>, sorted by parameter; the extra paves are cleared.
    /// </summary>
    internal void Update(NCollection_List<BOPDS_PaveBlock> theLPB, bool theFlag = true)
    {
        int i, aNb;
        BOPDS_Pave aPave1 = new(), aPave2;
        BOPDS_PaveBlock aPB;
        //
        aNb = myExtPaves.Extent();
        if (theFlag)
        {
            aNb = aNb + 2;
        }
        //
        if (aNb <= 1)
        {
            myExtPaves.Clear();
            myMFence.Clear();
            return;
        }
        //
        BOPDS_Pave[] pPaves = new BOPDS_Pave[aNb];
        //
        i = 0;
        if (theFlag)
        {
            pPaves[i] = myPave1;
            ++i;
            pPaves[i] = myPave2;
            ++i;
        }
        //
        for (NCollection_List<BOPDS_Pave>.Iterator aIt = new(myExtPaves); aIt.More(); aIt.Next())
        {
            pPaves[i] = aIt.Value();
            ++i;
        }
        myExtPaves.Clear();
        myMFence.Clear();
        //
        std_sort.sort(pPaves.AsSpan(), new BOPDS_Pave.LessThan());
        //
        for (i = 0; i < aNb; ++i)
        {
            BOPDS_Pave aPave = pPaves[i];
            if (i == 0)
            {
                aPave1 = aPave;
                continue;
            }
            //
            aPave2 = aPave;
            aPB = new BOPDS_PaveBlock();
            aPB.SetOriginalEdge(myOriginalEdge);
            aPB.SetPave1(aPave1);
            aPB.SetPave2(aPave2);
            //
            theLPB.Append(aPB);
            //
            aPave1 = aPave2;
        }
    }

    /// <summary>True if the pave block has shrunk data (a non-void shrunk box).</summary>
    public bool HasShrunkData() => !myShrunkBox.IsVoid();

    /// <summary>Sets the shrunk range, its box and whether the edge can be split.</summary>
    internal void SetShrunkData(double theT1, double theT2, in Bnd_Box theBox, bool theIsSplittable)
    {
        myTS1 = theT1;
        myTS2 = theT2;
        myShrunkBox = theBox;
        myIsSplittable = theIsSplittable;
    }

    /// <summary>The shrunk range, its box and whether the edge can be split.</summary>
    public void ShrunkData(out double theT1, out double theT2, out Bnd_Box theBox, out bool theIsSplittable)
    {
        theT1 = myTS1;
        theT2 = myTS2;
        theBox = myShrunkBox;
        theIsSplittable = myIsSplittable;
    }

    /// <summary>False if the shrunk range is too short for the edge to be split.</summary>
    public bool IsSplittable() => myIsSplittable;

    /// <summary>Writes the pave block as OCCT's <c>Dump</c> prints it.</summary>
    public void Dump(TextWriter theOS)
    {
        theOS.Write(" PB:{ E:" + myEdge + " orE:" + myOriginalEdge);
        theOS.Write(" Pave1:");
        myPave1.Dump(theOS);
        theOS.Write(" Pave2:");
        myPave2.Dump(theOS);
        theOS.Write(" }");
    }
}
