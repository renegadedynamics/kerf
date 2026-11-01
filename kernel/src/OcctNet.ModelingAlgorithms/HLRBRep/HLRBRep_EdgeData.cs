// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_EdgeData.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_EdgeData.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_EdgeData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The hidden-line data of an edge: its flags (selected, used, smooth, vertical, simple, outline vertices, cuts,
/// vertical ends, self-intersection done), hiding count, vertex indices, encoded min-max box, visibility status, and
/// projected curve.
/// </summary>
/// <remarks>
/// OCCT's value class, ported as a class (decisions 1084, 1092): it holds its own HLRAlgo_EdgeStatus and
/// HLRBRep_Curve, and HLRBRep_Data's arrays hold one instance per edge, which OCCT's code reaches by reference or
/// pointer. OCCT's copy assignment (<c>*ed = *e1</c> in <c>HLRBRep_Data::Write</c>) is <see cref="Assign"/>: a deep
/// copy of the status and a copy of the curve. A working object: its setters are public, as OCCT's.
/// </remarks>
public sealed class HLRBRep_EdgeData
{
    private const int EMaskSelected = 1;
    private const int EMaskUsed = 2;
    private const int EMaskRg1Line = 4;
    private const int EMaskVertical = 8;
    private const int EMaskSimple = 16;
    private const int EMaskOutLVSta = 32;
    private const int EMaskOutLVEnd = 64;
    private const int EMaskIntDone = 128;
    private const int EMaskCutAtSta = 256;
    private const int EMaskCutAtEnd = 512;
    private const int EMaskVerAtSta = 1024;
    private const int EMaskVerAtEnd = 2048;
    private const int EMaskRgNLine = 4096;

    private int myFlags;
    private int myHideCount;
    private int myVSta;
    private int myVEnd;
    private HLRAlgo_EdgesBlock.MinMaxIndices myMinMax;
    private readonly HLRAlgo_EdgeStatus myStatus = new();
    private HLRBRep_Curve myGeometry = new();
    private float myTolerance;

    public HLRBRep_EdgeData()
    {
        myFlags = 0;
        myHideCount = 0;
        Selected(true);
    }

    /// <summary>OCCT's copy assignment: the flags, indices and box copied, the status and the curve copied.</summary>
    public HLRBRep_EdgeData Assign(HLRBRep_EdgeData theOther)
    {
        if (ReferenceEquals(this, theOther))
        {
            return this;
        }
        myFlags = theOther.myFlags;
        myHideCount = theOther.myHideCount;
        myVSta = theOther.myVSta;
        myVEnd = theOther.myVEnd;
        myMinMax = theOther.myMinMax;
        myStatus.Assign(theOther.myStatus);
        myGeometry = theOther.myGeometry.Copy();
        myTolerance = theOther.myTolerance;
        return this;
    }

    /// <summary>Sets the edge, its vertices and their flags, and the bounds of its status.</summary>
    public void Set(bool Rg1L, bool RgNL, in TopoDS_Edge EG, int V1, int V2, bool Out1, bool Out2, bool Cut1, bool Cut2,
                    double Start, float TolStart, double End, float TolEnd)
    {
        Rg1Line(Rg1L);
        RgNLine(RgNL);
        Used(false);
        ChangeGeometry().Curve(EG);
        myTolerance = (float)(BRep_Tool.Tolerance(EG));
        VSta(V1);
        VEnd(V2);
        OutLVSta(Out1);
        OutLVEnd(Out2);
        CutAtSta(Cut1);
        CutAtEnd(Cut2);
        Status().Initialize(Start, (float)(ChangeGeometry().Curve().Resolution((double)TolStart)), End,
                            (float)(ChangeGeometry().Curve().Resolution((double)TolEnd)));
    }

    private bool Flag(int theMask) => (myFlags & theMask) != 0;

    private void Flag(int theMask, bool B)
    {
        if (B)
        {
            myFlags |= theMask;
        }
        else
        {
            myFlags &= ~theMask;
        }
    }

    public bool Selected() => Flag(EMaskSelected);

    public void Selected(bool B) => Flag(EMaskSelected, B);

    public bool Rg1Line() => Flag(EMaskRg1Line);

    public void Rg1Line(bool B) => Flag(EMaskRg1Line, B);

    public bool RgNLine() => Flag(EMaskRgNLine);

    public void RgNLine(bool B) => Flag(EMaskRgNLine, B);

    public bool Vertical() => Flag(EMaskVertical);

    public void Vertical(bool B) => Flag(EMaskVertical, B);

    public bool Simple() => Flag(EMaskSimple);

    public void Simple(bool B) => Flag(EMaskSimple, B);

    public bool OutLVSta() => Flag(EMaskOutLVSta);

    public void OutLVSta(bool B) => Flag(EMaskOutLVSta, B);

    public bool OutLVEnd() => Flag(EMaskOutLVEnd);

    public void OutLVEnd(bool B) => Flag(EMaskOutLVEnd, B);

    public bool CutAtSta() => Flag(EMaskCutAtSta);

    public void CutAtSta(bool B) => Flag(EMaskCutAtSta, B);

    public bool CutAtEnd() => Flag(EMaskCutAtEnd);

    public void CutAtEnd(bool B) => Flag(EMaskCutAtEnd, B);

    public bool VerAtSta() => Flag(EMaskVerAtSta);

    public void VerAtSta(bool B) => Flag(EMaskVerAtSta, B);

    public bool VerAtEnd() => Flag(EMaskVerAtEnd);

    public void VerAtEnd(bool B) => Flag(EMaskVerAtEnd, B);

    public bool AutoIntersectionDone() => Flag(EMaskIntDone);

    public void AutoIntersectionDone(bool B) => Flag(EMaskIntDone, B);

    public bool Used() => Flag(EMaskUsed);

    public void Used(bool B) => Flag(EMaskUsed, B);

    public int HideCount() => myHideCount;

    public void HideCount(int I) => myHideCount = I;

    public int VSta() => myVSta;

    public void VSta(int I) => myVSta = I;

    public int VEnd() => myVEnd;

    public void VEnd(int I) => myVEnd = I;

    public void UpdateMinMax(in HLRAlgo_EdgesBlock.MinMaxIndices theTotMinMax) => myMinMax = theTotMinMax;

    /// <summary>The encoded min-max box (a reference, as OCCT's).</summary>
    public ref HLRAlgo_EdgesBlock.MinMaxIndices MinMax() => ref myMinMax;

    /// <summary>The visibility status (the stored instance, as OCCT's reference).</summary>
    public HLRAlgo_EdgeStatus Status() => myStatus;

    public HLRBRep_Curve ChangeGeometry() => myGeometry;

    public HLRBRep_Curve Geometry() => myGeometry;

    /// <summary>The projected curve (OCCT's pointer to it, the curve type of the 2D intersections).</summary>
    public HLRBRep_Curve Curve() => myGeometry;

    public float Tolerance() => myTolerance;
}
