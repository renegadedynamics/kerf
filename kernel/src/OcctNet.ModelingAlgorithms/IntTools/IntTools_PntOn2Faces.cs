// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_PntOn2Faces.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_PntOn2Faces.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Two points on faces and a validity flag.</summary>
/// <remarks>A struct, as OCCT's by-value class; the setters are internal.</remarks>
public partial struct IntTools_PntOn2Faces : IEquatable<IntTools_PntOn2Faces>
{
    private bool myIsValid;
    private IntTools_PntOnFace myPnt1;
    private IntTools_PntOnFace myPnt2;

    /// <summary>Two default points, not valid.</summary>
    public IntTools_PntOn2Faces()
    {
        myIsValid = false;
        myPnt1 = new IntTools_PntOnFace();
        myPnt2 = new IntTools_PntOnFace();
    }

    /// <summary>The two points, not valid.</summary>
    public IntTools_PntOn2Faces(in IntTools_PntOnFace aP1, in IntTools_PntOnFace aP2)
    {
        myIsValid = false;
        myPnt1 = aP1;
        myPnt2 = aP2;
    }

    internal void SetP1(in IntTools_PntOnFace aP) => myPnt1 = aP;

    internal void SetP2(in IntTools_PntOnFace aP) => myPnt2 = aP;

    internal void SetValid(bool bF) => myIsValid = bF;

    public readonly IntTools_PntOnFace P1() => myPnt1;

    public readonly IntTools_PntOnFace P2() => myPnt2;

    public readonly bool IsValid() => myIsValid;

    // ---- OCCT.NET additions

    public readonly IntTools_PntOn2Faces WithP1(in IntTools_PntOnFace aP)
    {
        IntTools_PntOn2Faces aCopy = this;
        aCopy.SetP1(aP);
        return aCopy;
    }

    public readonly IntTools_PntOn2Faces WithP2(in IntTools_PntOnFace aP)
    {
        IntTools_PntOn2Faces aCopy = this;
        aCopy.SetP2(aP);
        return aCopy;
    }

    public readonly IntTools_PntOn2Faces WithValid(bool bF)
    {
        IntTools_PntOn2Faces aCopy = this;
        aCopy.SetValid(bF);
        return aCopy;
    }

    public readonly bool Equals(IntTools_PntOn2Faces theOther) =>
        myIsValid == theOther.myIsValid && myPnt1.Equals(theOther.myPnt1) && myPnt2.Equals(theOther.myPnt2);

    public override readonly bool Equals(object? theObj) => theObj is IntTools_PntOn2Faces anOther && Equals(anOther);

    public override readonly int GetHashCode() => HashCode.Combine(myIsValid, myPnt1, myPnt2);

    public static bool operator ==(IntTools_PntOn2Faces theLeft, IntTools_PntOn2Faces theRight) => theLeft.Equals(theRight);

    public static bool operator !=(IntTools_PntOn2Faces theLeft, IntTools_PntOn2Faces theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => $"IntTools_PntOn2Faces({myPnt1}, {myPnt2}, valid {myIsValid})";
}
