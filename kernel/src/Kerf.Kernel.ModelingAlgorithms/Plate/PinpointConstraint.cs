// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_PinpointConstraint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_PinpointConstraint.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_PinpointConstraint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Plate;

/// <summary>
/// A pinpoint constraint of a plate: the derivative (<c>Idu</c>, <c>Idv</c>) of the plate's deformation at a 2D point
/// takes an imposed value (the value itself for order 0).
/// </summary>
/// <remarks>A value type (rules, section 3); <c>default</c> is OCCT's default constraint (all zero).</remarks>
internal readonly struct PinpointConstraint : IEquatable<PinpointConstraint>
{
    private readonly Xyz _value;
    private readonly Xy _pnt2d;
    private readonly int _idu;
    private readonly int _idv;

    public PinpointConstraint()
    {
        _pnt2d = new Xy(0, 0);
        _value = new Xyz(0, 0, 0);
        _idu = 0;
        _idv = 0;
    }

    public PinpointConstraint(in Xy point2d, in Xyz imposedValue, int iu = 0, int iv = 0)
    {
        _pnt2d = point2d;
        _value = imposedValue;
        _idu = iu;
        _idv = iv;
    }

    public Xy Pnt2d() => _pnt2d;

    public int Idu() => _idu;

    public int Idv() => _idv;

    public Xyz Value() => _value;

    public bool Equals(PinpointConstraint other)
        => _value.Equals(other._value) && _pnt2d.Equals(other._pnt2d) && _idu == other._idu && _idv == other._idv;

    public override bool Equals(object? obj) => obj is PinpointConstraint other && Equals(other);

    public override int GetHashCode() => HashCode.Combine(_value, _pnt2d, _idu, _idv);

    public static bool operator ==(in PinpointConstraint left, in PinpointConstraint right) => left.Equals(right);

    public static bool operator !=(in PinpointConstraint left, in PinpointConstraint right) => !left.Equals(right);

    public override string ToString() => $"Plate_PinpointConstraint({_pnt2d}, {_value}, {_idu}, {_idv})";
}
