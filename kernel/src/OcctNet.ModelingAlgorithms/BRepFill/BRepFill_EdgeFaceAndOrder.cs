// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_EdgeFaceAndOrder.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_EdgeFaceAndOrder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// An edge, its support face (or a null face) and the order of continuity a filling must have along it.
/// </summary>
/// <remarks>A value type (rules, section 3); its fields are internal, as OCCT's friend BRepFill_Filling reads them.</remarks>
public readonly struct BRepFill_EdgeFaceAndOrder : IEquatable<BRepFill_EdgeFaceAndOrder>
{
    internal readonly TopoDS_Edge myEdge;
    internal readonly TopoDS_Face myFace;
    internal readonly GeomAbs_Shape myOrder;

    public BRepFill_EdgeFaceAndOrder()
    {
        myEdge = new TopoDS_Edge();
        myFace = new TopoDS_Face();
        myOrder = default;
    }

    public BRepFill_EdgeFaceAndOrder(in TopoDS_Edge anEdge, in TopoDS_Face aFace, GeomAbs_Shape anOrder)
    {
        myEdge = anEdge;
        myFace = aFace;
        myOrder = anOrder;
    }

    public bool Equals(BRepFill_EdgeFaceAndOrder other)
        => myEdge.Equals(other.myEdge) && myFace.Equals(other.myFace) && myOrder == other.myOrder;

    public override bool Equals(object? obj) => obj is BRepFill_EdgeFaceAndOrder other && Equals(other);

    public override int GetHashCode() => HashCode.Combine(myEdge, myFace, myOrder);

    public static bool operator ==(in BRepFill_EdgeFaceAndOrder theLeft, in BRepFill_EdgeFaceAndOrder theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in BRepFill_EdgeFaceAndOrder theLeft, in BRepFill_EdgeFaceAndOrder theRight) => !theLeft.Equals(theRight);

    public override string ToString() => $"BRepFill_EdgeFaceAndOrder({myEdge}, {myFace}, {myOrder})";
}
