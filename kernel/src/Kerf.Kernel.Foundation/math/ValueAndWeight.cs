// Port of: src/FoundationClasses/TKMath/math/math_ValueAndWeight.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine;

namespace Kerf.Engine.Numerics;

/// <summary>A value and its weight, ordered by value.</summary>
internal struct ValueAndWeight : IEquatable<ValueAndWeight>, IComparable<ValueAndWeight>
{
    private double _value;
    private double _weight;

    public ValueAndWeight(double value, double weight)
    {
        _value = value;
        _weight = weight;
    }

    public readonly double Value() => _value;

    public readonly double Weight() => _weight;

    /// <summary>OCCT's <c>operator&lt;</c>, used for sorting: compares the values only.</summary>
    public static bool operator <(ValueAndWeight left, ValueAndWeight right) => left.Value() < right.Value();

    public static bool operator >(ValueAndWeight left, ValueAndWeight right) => right.Value() < left.Value();

    /// <summary>Orders by value, as OCCT's <c>operator&lt;</c>.</summary>
    public readonly int CompareTo(ValueAndWeight other)
        => _value < other._value ? -1 : other._value < _value ? 1 : 0;

    public readonly bool Equals(ValueAndWeight other) => _value.Equals(other._value) && _weight.Equals(other._weight);

    public override readonly bool Equals(object? obj) => obj is ValueAndWeight other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_value, _weight);

    public static bool operator ==(ValueAndWeight left, ValueAndWeight right) => left.Equals(right);

    public static bool operator !=(ValueAndWeight left, ValueAndWeight right) => !left.Equals(right);

    public override readonly string ToString() => Constants.Format("ValueAndWeight", _value, _weight);
}
