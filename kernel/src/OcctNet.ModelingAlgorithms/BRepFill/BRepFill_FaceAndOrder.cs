// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_FaceAndOrder.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_FaceAndOrder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>A face and the order of continuity a filling must have with it (a "free constraint").</summary>
/// <remarks>A value type (rules, section 3); its fields are internal, as OCCT's friend BRepFill_Filling reads them.</remarks>
public readonly struct BRepFill_FaceAndOrder : IEquatable<BRepFill_FaceAndOrder>
{
    internal readonly TopoDS_Face myFace;
    internal readonly Continuity myOrder;

    public BRepFill_FaceAndOrder()
    {
        myFace = new TopoDS_Face();
        myOrder = default;
    }

    public BRepFill_FaceAndOrder(in TopoDS_Face aFace, Continuity anOrder)
    {
        myFace = aFace;
        myOrder = anOrder;
    }

    public bool Equals(BRepFill_FaceAndOrder other) => myFace.Equals(other.myFace) && myOrder == other.myOrder;

    public override bool Equals(object? obj) => obj is BRepFill_FaceAndOrder other && Equals(other);

    public override int GetHashCode() => HashCode.Combine(myFace, myOrder);

    public static bool operator ==(in BRepFill_FaceAndOrder theLeft, in BRepFill_FaceAndOrder theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in BRepFill_FaceAndOrder theLeft, in BRepFill_FaceAndOrder theRight) => !theLeft.Equals(theRight);

    public override string ToString() => $"BRepFill_FaceAndOrder({myFace}, {myOrder})";
}
