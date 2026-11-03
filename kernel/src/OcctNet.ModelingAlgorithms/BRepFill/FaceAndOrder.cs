// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_FaceAndOrder.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_FaceAndOrder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Construction.Filling;

/// <summary>A face and the order of continuity a filling must have with it (a "free constraint").</summary>
/// <remarks>A value type (rules, section 3); its fields are internal, as OCCT's friend BRepFill_Filling reads them.</remarks>
public readonly struct FaceAndOrder : IEquatable<FaceAndOrder>
{
    internal readonly Face _face;
    internal readonly Continuity _order;

    public FaceAndOrder()
    {
        _face = new Face();
        _order = default;
    }

    public FaceAndOrder(in Face aFace, Continuity anOrder)
    {
        _face = aFace;
        _order = anOrder;
    }

    public bool Equals(FaceAndOrder other) => _face.Equals(other._face) && _order == other._order;

    public override bool Equals(object? obj) => obj is FaceAndOrder other && Equals(other);

    public override int GetHashCode() => HashCode.Combine(_face, _order);

    public static bool operator ==(in FaceAndOrder left, in FaceAndOrder right) => left.Equals(right);

    public static bool operator !=(in FaceAndOrder left, in FaceAndOrder right) => !left.Equals(right);

    public override string ToString() => $"BRepFill_FaceAndOrder({_face}, {_order})";
}
