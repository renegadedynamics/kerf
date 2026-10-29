// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_CoupleOfPaveBlocks.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Two pave blocks with an index, the index of an interference and a tolerance.</summary>
/// <remarks>
/// A struct, as OCCT's by-value class: it holds the two pave blocks by reference (handles in C++), so a copy shares
/// them as the C++ copy does. OCCT's deprecated <c>PaveBlocks(PB1, PB2)</c> is left out.
/// </remarks>
public partial struct BOPDS_CoupleOfPaveBlocks
{
    private int myIndexInterf;
    private int myIndex;
    private BOPDS_PaveBlock? myPB1;
    private BOPDS_PaveBlock? myPB2;
    private double myTolerance;

    public BOPDS_CoupleOfPaveBlocks()
    {
        myIndexInterf = -1;
        myIndex = -1;
        myTolerance = 0;
    }

    public BOPDS_CoupleOfPaveBlocks(BOPDS_PaveBlock? thePB1, BOPDS_PaveBlock? thePB2)
    {
        myIndexInterf = -1;
        myIndex = -1;
        myTolerance = 0;
        SetPaveBlocks(thePB1, thePB2);
    }

    internal void SetIndex(int theIndex) => myIndex = theIndex;

    public readonly int Index() => myIndex;

    internal void SetIndexInterf(int theIndex) => myIndexInterf = theIndex;

    public readonly int IndexInterf() => myIndexInterf;

    internal void SetPaveBlocks(BOPDS_PaveBlock? thePB1, BOPDS_PaveBlock? thePB2)
    {
        myPB1 = thePB1;
        myPB2 = thePB2;
    }

    internal void SetPaveBlock1(BOPDS_PaveBlock? thePB) => myPB1 = thePB;

    public readonly BOPDS_PaveBlock? PaveBlock1() => myPB1;

    internal void SetPaveBlock2(BOPDS_PaveBlock? thePB) => myPB2 = thePB;

    public readonly BOPDS_PaveBlock? PaveBlock2() => myPB2;

    internal void SetTolerance(double theTol) => myTolerance = theTol;

    public readonly double Tolerance() => myTolerance;
}
