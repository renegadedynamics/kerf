// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_PntOnFace.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_PntOnFace.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace Occt;

/// <summary>A face, a 3D point, its UV parameters on the face and a validity flag.</summary>
/// <remarks>
/// A struct, as OCCT's by-value class; the setters are internal. OCCT declares <c>IsValid()</c> without defining it
/// (only <see cref="Valid"/> exists), so it is left out.
/// </remarks>
public partial struct IntTools_PntOnFace : IEquatable<IntTools_PntOnFace>
{
    private bool myIsValid;
    private Point3D myPnt;
    private double myU;
    private double myV;
    private TopoDS_Face myFace;

    /// <summary>An invalid point with the parameters (99, 99).</summary>
    public IntTools_PntOnFace()
    {
        myIsValid = false;
        myPnt = new Point3D();
        myU = 99.0;
        myV = 99.0;
        myFace = new TopoDS_Face();
    }

    /// <summary>Sets the face, the point and its parameters (not the validity flag).</summary>
    internal void Init(in TopoDS_Face aF, in Point3D aP, double anU, double aV)
    {
        myFace = aF;
        myPnt = aP;
        myU = anU;
        myV = aV;
    }

    internal void SetFace(in TopoDS_Face aF) => myFace = aF;

    internal void SetPnt(in Point3D aP) => myPnt = aP;

    internal void SetParameters(double anU, double aV)
    {
        myU = anU;
        myV = aV;
    }

    internal void SetValid(bool bF) => myIsValid = bF;

    public readonly bool Valid() => myIsValid;

    public readonly TopoDS_Face Face() => myFace;

    public readonly Point3D Pnt() => myPnt;

    public readonly void Parameters(out double anU, out double aV)
    {
        anU = myU;
        aV = myV;
    }

    // ---- OCCT.NET additions

    /// <summary>A copy initialised as <c>Init</c> does.</summary>
    public readonly IntTools_PntOnFace Inited(in TopoDS_Face aF, in Point3D aP, double anU, double aV)
    {
        IntTools_PntOnFace aCopy = this;
        aCopy.Init(aF, aP, anU, aV);
        return aCopy;
    }

    public readonly IntTools_PntOnFace WithFace(in TopoDS_Face aF)
    {
        IntTools_PntOnFace aCopy = this;
        aCopy.SetFace(aF);
        return aCopy;
    }

    public readonly IntTools_PntOnFace WithPnt(in Point3D aP)
    {
        IntTools_PntOnFace aCopy = this;
        aCopy.SetPnt(aP);
        return aCopy;
    }

    public readonly IntTools_PntOnFace WithParameters(double anU, double aV)
    {
        IntTools_PntOnFace aCopy = this;
        aCopy.SetParameters(anU, aV);
        return aCopy;
    }

    public readonly IntTools_PntOnFace WithValid(bool bF)
    {
        IntTools_PntOnFace aCopy = this;
        aCopy.SetValid(bF);
        return aCopy;
    }

    /// <summary>Field by field: the face by <see cref="TopoDS_Shape.IsEqual"/>, the numbers exactly.</summary>
    public readonly bool Equals(IntTools_PntOnFace theOther) =>
        myIsValid == theOther.myIsValid && myPnt.Equals(theOther.myPnt) && myU.Equals(theOther.myU)
        && myV.Equals(theOther.myV) && myFace.Equals(theOther.myFace);

    public override readonly bool Equals(object? theObj) => theObj is IntTools_PntOnFace anOther && Equals(anOther);

    public override readonly int GetHashCode() => HashCode.Combine(myIsValid, myPnt, myU, myV, myFace);

    public static bool operator ==(IntTools_PntOnFace theLeft, IntTools_PntOnFace theRight) => theLeft.Equals(theRight);

    public static bool operator !=(IntTools_PntOnFace theLeft, IntTools_PntOnFace theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() =>
        $"IntTools_PntOnFace({myPnt}, {Constants.Format("UV", myU, myV)}, valid {myIsValid})";
}
