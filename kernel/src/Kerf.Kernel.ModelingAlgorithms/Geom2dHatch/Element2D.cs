// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dHatch/Geom2dHatch_Element.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dHatch/Geom2dHatch_Element.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Classification.Hatching;

/// <summary>An element of a 2D hatcher: a curve of the boundary and its orientation.</summary>
/// <remarks>
/// A value type (decision 666). The constructor keeps a shallow copy of the caller's adaptor, as OCCT copies it, so
/// that a later <c>Load</c> of the caller's adaptor doesn't change the element; copies of the element share that
/// adaptor, which nothing changes. The orientation setter and <see cref="ChangeCurve"/> are internal, with
/// <see cref="WithOrientation"/> as the twin. OCCT's default constructor leaves the orientation uninitialised; the
/// port's is FORWARD, and <c>default</c> has no curve. Equality compares the adaptors by reference.
/// </remarks>
internal struct Element2D : IEquatable<Element2D>
{
    private readonly CurveAdaptor2D _curve;
    private Orientation _orientation;

    /// <summary>An element on an empty curve.</summary>
    public Element2D()
    {
        _curve = new CurveAdaptor2D();
        _orientation = Forward;
    }

    /// <summary>An element on (a copy of) <paramref name="curve"/>, with the given orientation.</summary>
    public Element2D(CurveAdaptor2D curve, Orientation orientation = Forward)
    {
        _curve = (CurveAdaptor2D)curve.ShallowCopy();
        _orientation = orientation;
    }

    /// <summary>The curve of the element.</summary>
    public readonly CurveAdaptor2D Curve() => _curve;

    /// <summary>The curve of the element, for writing (OCCT's non-const reference).</summary>
    internal readonly CurveAdaptor2D ChangeCurve() => _curve;

    /// <summary>Sets the orientation of the element.</summary>
    internal void Orientation(Orientation orientation)
    {
        _orientation = orientation;
    }

    /// <summary>The orientation of the element.</summary>
    public readonly Orientation Orientation() => _orientation;

    /// <summary>A copy with the orientation <paramref name="orientation"/>.</summary>
    public readonly Element2D WithOrientation(Orientation orientation)
    {
        Element2D copy = this;
        copy.Orientation(orientation);
        return copy;
    }

    public readonly bool Equals(Element2D other) =>
        ReferenceEquals(_curve, other._curve) && _orientation == other._orientation;

    public override readonly bool Equals(object? obj) => obj is Element2D other && Equals(other);

    public override readonly int GetHashCode() =>
        HashCode.Combine(System.Runtime.CompilerServices.RuntimeHelpers.GetHashCode(_curve), _orientation);

    public static bool operator ==(Element2D left, Element2D right) => left.Equals(right);

    public static bool operator !=(Element2D left, Element2D right) => !left.Equals(right);

    public override readonly string ToString() =>
        $"Geom2dHatch_Element({(_curve?.Curve() is { } curve ? global::Kerf.Engine.Foundation.OcctTypeNames.Of(curve.GetType()) : null) ?? "no curve"}, {_orientation})";
}
