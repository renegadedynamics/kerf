// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_D2.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_D2.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Plate;

/// <summary>The second derivatives (DUU, DUV, DVV) of a surface at a point, as the G2 constraints of a plate take them.</summary>
/// <remarks>A value type (rules, section 3). OCCT has no accessors: only its friends Plate_GtoCConstraint and
/// Plate_FreeGtoCConstraint read the fields, which are internal here.</remarks>
public struct D2 : IEquatable<D2>
{
    internal Xyz _duu;
    internal Xyz _duv;
    internal Xyz _dvv;

    public D2(in Xyz duu, in Xyz duv, in Xyz dvv)
    {
        _duu = duu;
        _duv = duv;
        _dvv = dvv;
    }

    public D2(in D2 @ref)
    {
        _duu = @ref._duu;
        _duv = @ref._duv;
        _dvv = @ref._dvv;
    }

    public readonly bool Equals(D2 other) => _duu.Equals(other._duu) && _duv.Equals(other._duv) && _dvv.Equals(other._dvv);

    public override readonly bool Equals(object? obj) => obj is D2 other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_duu, _duv, _dvv);

    public static bool operator ==(in D2 left, in D2 right) => left.Equals(right);

    public static bool operator !=(in D2 left, in D2 right) => !left.Equals(right);

    public override readonly string ToString() => $"Plate_D2({_duu}, {_duv}, {_dvv})";
}
