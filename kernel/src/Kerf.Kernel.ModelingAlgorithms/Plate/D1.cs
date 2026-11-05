// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_D1.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_D1.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_D1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Plate;

/// <summary>The first derivatives (DU, DV) of a surface at a point, as the G1 constraints of a plate take them.</summary>
/// <remarks>A value type (rules, section 3). The fields are internal, as OCCT makes them visible to its friends
/// Plate_GtoCConstraint and Plate_FreeGtoCConstraint (which transform a copy in place).</remarks>
internal struct D1 : IEquatable<D1>
{
    internal Xyz _du;
    internal Xyz _dv;

    public D1(in Xyz du, in Xyz dv)
    {
        _du = du;
        _dv = dv;
    }

    public D1(in D1 @ref)
    {
        _du = @ref._du;
        _dv = @ref._dv;
    }

    public readonly Xyz DU() => _du;

    public readonly Xyz DV() => _dv;

    public readonly bool Equals(D1 other) => _du.Equals(other._du) && _dv.Equals(other._dv);

    public override readonly bool Equals(object? obj) => obj is D1 other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_du, _dv);

    public static bool operator ==(in D1 left, in D1 right) => left.Equals(right);

    public static bool operator !=(in D1 left, in D1 right) => !left.Equals(right);

    public override readonly string ToString() => $"Plate_D1({_du}, {_dv})";
}
