// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_Point.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_Point.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace Occt;

/// <summary>An intersection point of two faces: the 3D point, its 2D points on each face and the vertex index.</summary>
/// <remarks>The default point is (99, 99, 99) with 2D points (99, 99) and the index -1, as OCCT's.</remarks>
public partial struct BOPDS_Point : IEquatable<BOPDS_Point>
{
    private Point3D myPnt;
    private Point2D myPnt2D1;
    private Point2D myPnt2D2;
    private int myIndex;

    public BOPDS_Point()
    {
        myPnt = new Point3D(99.0, 99.0, 99.0);
        myPnt2D1 = new Point2D(99.0, 99.0);
        myPnt2D2 = myPnt2D1;
        myIndex = -1;
    }

    internal void SetIndex(int theIndex) => myIndex = theIndex;

    public readonly int Index() => myIndex;

    internal void SetPnt(in Point3D thePnt) => myPnt = thePnt;

    public readonly Point3D Pnt() => myPnt;

    internal void SetPnt2D1(in Point2D thePnt) => myPnt2D1 = thePnt;

    public readonly Point2D Pnt2D1() => myPnt2D1;

    internal void SetPnt2D2(in Point2D thePnt) => myPnt2D2 = thePnt;

    public readonly Point2D Pnt2D2() => myPnt2D2;

    // ---- OCCT.NET additions

    public readonly BOPDS_Point WithIndex(int theIndex)
    {
        BOPDS_Point aCopy = this;
        aCopy.SetIndex(theIndex);
        return aCopy;
    }

    public readonly BOPDS_Point WithPnt(in Point3D thePnt)
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
        Constants.Format("BOPDS_Point", myPnt.X(), myPnt.Y(), myPnt.Z(), myPnt2D1.X(), myPnt2D1.Y(), myPnt2D2.X(), myPnt2D2.Y(), myIndex);
}
