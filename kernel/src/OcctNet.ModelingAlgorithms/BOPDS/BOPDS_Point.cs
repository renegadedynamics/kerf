// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_Point.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_Point.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>An intersection point of two faces: the 3D point, its 2D points on each face and the vertex index.</summary>
/// <remarks>The default point is (99, 99, 99) with 2D points (99, 99) and the index -1, as OCCT's.</remarks>
public partial struct BOPDS_Point : IEquatable<BOPDS_Point>
{
    private gp_Pnt myPnt;
    private gp_Pnt2d myPnt2D1;
    private gp_Pnt2d myPnt2D2;
    private int myIndex;

    public BOPDS_Point()
    {
        myPnt = new gp_Pnt(99.0, 99.0, 99.0);
        myPnt2D1 = new gp_Pnt2d(99.0, 99.0);
        myPnt2D2 = myPnt2D1;
        myIndex = -1;
    }

    internal void SetIndex(int theIndex) => myIndex = theIndex;

    public readonly int Index() => myIndex;

    internal void SetPnt(in gp_Pnt thePnt) => myPnt = thePnt;

    public readonly gp_Pnt Pnt() => myPnt;

    internal void SetPnt2D1(in gp_Pnt2d thePnt) => myPnt2D1 = thePnt;

    public readonly gp_Pnt2d Pnt2D1() => myPnt2D1;

    internal void SetPnt2D2(in gp_Pnt2d thePnt) => myPnt2D2 = thePnt;

    public readonly gp_Pnt2d Pnt2D2() => myPnt2D2;

    // ---- OCCT.NET additions

    public readonly BOPDS_Point WithIndex(int theIndex)
    {
        BOPDS_Point aCopy = this;
        aCopy.SetIndex(theIndex);
        return aCopy;
    }

    public readonly BOPDS_Point WithPnt(in gp_Pnt thePnt)
    {
        BOPDS_Point aCopy = this;
        aCopy.SetPnt(thePnt);
        return aCopy;
    }

    public readonly bool Equals(BOPDS_Point theOther) =>
        myPnt.Equals(theOther.myPnt) && myPnt2D1.Equals(theOther.myPnt2D1) && myPnt2D2.Equals(theOther.myPnt2D2)
        && myIndex == theOther.myIndex;

    public override readonly bool Equals(object? theObj) => theObj is BOPDS_Point anOther && Equals(anOther);

    public override readonly int GetHashCode() => HashCode.Combine(myPnt, myPnt2D1, myPnt2D2, myIndex);

    public static bool operator ==(BOPDS_Point theLeft, BOPDS_Point theRight) => theLeft.Equals(theRight);

    public static bool operator !=(BOPDS_Point theLeft, BOPDS_Point theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() =>
        gp.Format("BOPDS_Point", myPnt.X(), myPnt.Y(), myPnt.Z(), myPnt2D1.X(), myPnt2D1.Y(), myPnt2D2.X(), myPnt2D2.Y(), myIndex);
}
