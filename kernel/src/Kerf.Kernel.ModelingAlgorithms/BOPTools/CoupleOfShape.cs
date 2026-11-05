// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_CoupleOfShape.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using Kerf.Engine;

namespace Kerf.Engine.Booleans.Tools;

/// <summary>A pair of shapes.</summary>
/// <remarks>
/// A value: a struct with internal setters and <c>With</c> twins (translation rules, section 3). Equality compares the
/// two shapes with <see cref="Shape"/>'s equality (same TShape, location and orientation).
/// </remarks>
internal partial struct CoupleOfShape : IEquatable<CoupleOfShape>
{
    private Shape _shape1;
    private Shape _shape2;

    /// <summary>Sets the first shape.</summary>
    internal void SetShape1(in Shape shape) => _shape1 = shape;

    /// <summary>The first shape.</summary>
    public readonly Shape Shape1() => _shape1;

    /// <summary>Sets the second shape.</summary>
    internal void SetShape2(in Shape shape) => _shape2 = shape;

    /// <summary>The second shape.</summary>
    public readonly Shape Shape2() => _shape2;

    /// <summary>A copy with the first shape replaced.</summary>
    public readonly CoupleOfShape WithShape1(in Shape shape)
    {
        CoupleOfShape copy = this;
        copy.SetShape1(shape);
        return copy;
    }

    /// <summary>A copy with the second shape replaced.</summary>
    public readonly CoupleOfShape WithShape2(in Shape shape)
    {
        CoupleOfShape copy = this;
        copy.SetShape2(shape);
        return copy;
    }

    /// <inheritdoc/>
    public readonly bool Equals(CoupleOfShape other) => _shape1.Equals(other._shape1) && _shape2.Equals(other._shape2);

    /// <inheritdoc/>
    public override readonly bool Equals(object? obj) => obj is CoupleOfShape c && Equals(c);

    /// <inheritdoc/>
    public override readonly int GetHashCode() => HashCode.Combine(_shape1, _shape2);

    /// <summary>Field-by-field equality.</summary>
    public static bool operator ==(CoupleOfShape left, CoupleOfShape right) => left.Equals(right);

    /// <summary>Field-by-field inequality.</summary>
    public static bool operator !=(CoupleOfShape left, CoupleOfShape right) => !left.Equals(right);

    /// <inheritdoc/>
    public override readonly string ToString() => $"BOPTools_CoupleOfShape({_shape1}, {_shape2})";
}
