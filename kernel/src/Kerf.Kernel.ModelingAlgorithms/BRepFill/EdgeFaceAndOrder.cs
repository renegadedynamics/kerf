// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_EdgeFaceAndOrder.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_EdgeFaceAndOrder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Construction.Filling;

/// <summary>
/// An edge, its support face (or a null face) and the order of continuity a filling must have along it.
/// </summary>
/// <remarks>A value type (rules, section 3); its fields are internal, as OCCT's friend BRepFill_Filling reads them.</remarks>
public readonly struct EdgeFaceAndOrder : IEquatable<EdgeFaceAndOrder>
{
    internal readonly Edge _edge;
    internal readonly Face _face;
    internal readonly Continuity _order;

    public EdgeFaceAndOrder()
    {
        _edge = new Edge();
        _face = new Face();
        _order = default;
    }

    public EdgeFaceAndOrder(in Edge anEdge, in Face aFace, Continuity anOrder)
    {
        _edge = anEdge;
        _face = aFace;
        _order = anOrder;
    }

    public bool Equals(EdgeFaceAndOrder other)
        => _edge.Equals(other._edge) && _face.Equals(other._face) && _order == other._order;

    public override bool Equals(object? obj) => obj is EdgeFaceAndOrder other && Equals(other);

    public override int GetHashCode() => HashCode.Combine(_edge, _face, _order);

    public static bool operator ==(in EdgeFaceAndOrder left, in EdgeFaceAndOrder right) => left.Equals(right);

    public static bool operator !=(in EdgeFaceAndOrder left, in EdgeFaceAndOrder right) => !left.Equals(right);

    public override string ToString() => $"BRepFill_EdgeFaceAndOrder({_edge}, {_face}, {_order})";
}
