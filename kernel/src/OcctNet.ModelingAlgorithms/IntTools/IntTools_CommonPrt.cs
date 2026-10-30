// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_CommonPrt.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_CommonPrt.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>A common part of two edges in 3D space: a vertex or an edge, with the ranges on each edge.</summary>
/// <remarks>
/// A class, a working object of the edge intersectors (plan section 6), because it holds a sequence of ranges: OCCT's
/// by-value copies are the copy constructor and <see cref="Assign"/>, which ported code writes out.
/// </remarks>
public sealed class IntTools_CommonPrt
{
    private TopoDS_Edge myEdge1 = new();
    private TopoDS_Edge myEdge2 = new();
    private TopAbs_ShapeEnum myType;
    private IntTools_Range myRange1 = new();
    private double myVertPar1;
    private double myVertPar2;
    private readonly NCollection_Sequence<IntTools_Range> myRanges2 = new();
    private bool myAllNullFlag;
    private gp_Pnt myPnt1;
    private gp_Pnt myPnt2;

    /// <summary>An empty common part, of type <see cref="TopAbs_SHAPE"/>.</summary>
    public IntTools_CommonPrt()
    {
        myType = TopAbs_SHAPE;
        myAllNullFlag = false;
        //
        myPnt1.SetCoord(0.0, 0.0, 0.0);
        myPnt2.SetCoord(0.0, 0.0, 0.0);
        //
        myVertPar1 = 0.0;
        myVertPar2 = 0.0;
    }

    /// <summary>A copy of <paramref name="Other"/> (OCCT's copy constructor).</summary>
    public IntTools_CommonPrt(IntTools_CommonPrt Other)
    {
        myEdge1 = Other.myEdge1;
        myEdge2 = Other.myEdge2;
        myType = Other.myType;
        myRange1 = Other.myRange1;
        myVertPar1 = Other.myVertPar1;
        myVertPar2 = Other.myVertPar2;
        myAllNullFlag = false; // OCCT: the copy constructor doesn't copy the flag (occt-issues.md, unit 6A)
        //
        myPnt1 = Other.myPnt1;
        myPnt2 = Other.myPnt2;
        int i, aNb = Other.myRanges2.Length();
        for (i = 1; i <= aNb; i++)
        {
            myRanges2.Append(Other.myRanges2.Value(i));
        }
    }

    /// <summary>OCCT's assignment operator.</summary>
    internal IntTools_CommonPrt Assign(IntTools_CommonPrt Other)
    {
        myEdge1 = Other.myEdge1;
        myEdge2 = Other.myEdge2;
        myType = Other.myType;
        myRange1 = Other.myRange1;
        myVertPar1 = Other.myVertPar1;
        myVertPar2 = Other.myVertPar2;
        //
        myPnt1 = Other.myPnt1;
        myPnt2 = Other.myPnt2;
        //
        // OCCT: the second edge's ranges are appended to the ones already here, not replacing them
        // (occt-issues.md, unit 6A)
        int i, aNb = Other.myRanges2.Length();
        for (i = 1; i <= aNb; i++)
        {
            myRanges2.Append(Other.myRanges2.Value(i));
        }
        myAllNullFlag = Other.myAllNullFlag;
        return this;
    }

    internal void SetEdge1(in TopoDS_Edge anEdge) => myEdge1 = anEdge;

    internal void SetEdge2(in TopoDS_Edge anEdge) => myEdge2 = anEdge;

    /// <summary>Sets the type of the common part: vertex or edge.</summary>
    internal void SetType(TopAbs_ShapeEnum aType) => myType = aType;

    internal void SetRange1(in IntTools_Range aRange) => myRange1 = aRange;

    internal void SetRange1(double tf, double tl)
    {
        myRange1.SetFirst(tf);
        myRange1.SetLast(tl);
    }

    internal void AppendRange2(in IntTools_Range aRange) => myRanges2.Append(aRange);

    internal void AppendRange2(double tf, double tl)
    {
        IntTools_Range aRange = new(tf, tl);
        myRanges2.Append(aRange);
    }

    internal void SetVertexParameter1(double tV) => myVertPar1 = tV;

    internal void SetVertexParameter2(double tV) => myVertPar2 = tV;

    public TopoDS_Edge Edge1() => myEdge1;

    public TopoDS_Edge Edge2() => myEdge2;

    /// <summary>The type of the common part: vertex or edge.</summary>
    public TopAbs_ShapeEnum Type() => myType;

    public IntTools_Range Range1() => myRange1;

    public void Range1(out double tf, out double tl)
    {
        tf = myRange1.First();
        tl = myRange1.Last();
    }

    /// <summary>The ranges on the second edge (the algorithm's own sequence; ported code only reads it).</summary>
    public NCollection_Sequence<IntTools_Range> Ranges2() => myRanges2;

    internal NCollection_Sequence<IntTools_Range> ChangeRanges2() => myRanges2;

    public double VertexParameter1() => myVertPar1;

    public double VertexParameter2() => myVertPar2;

    /// <summary>Copies this into <paramref name="aCP"/> (not the flag nor the bounding points, as in OCCT).</summary>
    internal void Copy(IntTools_CommonPrt aCP)
    {
        aCP.SetEdge1(Edge1());
        aCP.SetEdge2(Edge2());
        aCP.SetType(Type());
        aCP.SetRange1(Range1());
        aCP.SetVertexParameter1(myVertPar1);
        aCP.SetVertexParameter2(myVertPar2);
        int i, aNb;
        aNb = myRanges2.Length();
        for (i = 1; i <= aNb; i++)
        {
            aCP.AppendRange2(myRanges2.Value(i));
        }
    }

    public bool AllNullFlag() => myAllNullFlag;

    internal void SetAllNullFlag(bool aFlag) => myAllNullFlag = aFlag;

    internal void SetBoundingPoints(in gp_Pnt aP1, in gp_Pnt aP2)
    {
        myPnt1 = aP1;
        myPnt2 = aP2;
    }

    public void BoundingPoints(out gp_Pnt aP1, out gp_Pnt aP2)
    {
        aP1 = myPnt1;
        aP2 = myPnt2;
    }
}
