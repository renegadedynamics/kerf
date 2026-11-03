// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_D3.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_D3.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Plate;

/// <summary>The third derivatives (DUUU, DUUV, DUVV, DVVV) of a surface at a point, as the G3 constraints of a plate
/// take them.</summary>
/// <remarks>A value type (rules, section 3). OCCT has no accessors: only its friends Plate_GtoCConstraint and
/// Plate_FreeGtoCConstraint read the fields, which are internal here.</remarks>
public struct D3 : IEquatable<D3>
{
    internal Xyz _duuu;
    internal Xyz _duuv;
    internal Xyz _duvv;
    internal Xyz _dvvv;

    public D3(in Xyz duuu, in Xyz duuv, in Xyz duvv, in Xyz dvvv)
    {
        _duuu = duuu;
        _duuv = duuv;
        _duvv = duvv;
        _dvvv = dvvv;
    }

    public D3(in D3 @ref)
    {
        _duuu = @ref._duuu;
        _duuv = @ref._duuv;
        _duvv = @ref._duvv;
        _dvvv = @ref._dvvv;
    }

    public readonly bool Equals(D3 other)
        => _duuu.Equals(other._duuu) && _duuv.Equals(other._duuv) && _duvv.Equals(other._duvv) && _dvvv.Equals(other._dvvv);

    public override readonly bool Equals(object? obj) => obj is D3 other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_duuu, _duuv, _duvv, _dvvv);

    public static bool operator ==(in D3 left, in D3 right) => left.Equals(right);

    public static bool operator !=(in D3 left, in D3 right) => !left.Equals(right);

    public override readonly string ToString() => $"Plate_D3({_duuu}, {_duuv}, {_duvv}, {_dvvv})";
}
