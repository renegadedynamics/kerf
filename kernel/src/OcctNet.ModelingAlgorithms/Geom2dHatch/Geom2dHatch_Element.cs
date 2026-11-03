// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dHatch/Geom2dHatch_Element.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dHatch/Geom2dHatch_Element.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>An element of a 2D hatcher: a curve of the boundary and its orientation.</summary>
/// <remarks>
/// A value type (decision 666). The constructor keeps a shallow copy of the caller's adaptor, as OCCT copies it, so
/// that a later <c>Load</c> of the caller's adaptor doesn't change the element; copies of the element share that
/// adaptor, which nothing changes. The orientation setter and <see cref="ChangeCurve"/> are internal, with
/// <see cref="WithOrientation"/> as the twin. OCCT's default constructor leaves the orientation uninitialised; the
/// port's is FORWARD, and <c>default</c> has no curve. Equality compares the adaptors by reference.
/// </remarks>
public struct Geom2dHatch_Element : IEquatable<Geom2dHatch_Element>
{
    private readonly Geom2dAdaptor_Curve myCurve;
    private TopAbs_Orientation myOrientation;

    /// <summary>An element on an empty curve.</summary>
    public Geom2dHatch_Element()
    {
        myCurve = new Geom2dAdaptor_Curve();
        myOrientation = TopAbs_FORWARD;
    }

    /// <summary>An element on (a copy of) <paramref name="Curve"/>, with the given orientation.</summary>
    public Geom2dHatch_Element(Geom2dAdaptor_Curve Curve, TopAbs_Orientation Orientation = TopAbs_FORWARD)
    {
        myCurve = (Geom2dAdaptor_Curve)Curve.ShallowCopy();
        myOrientation = Orientation;
    }

    /// <summary>The curve of the element.</summary>
    public readonly Geom2dAdaptor_Curve Curve() => myCurve;

    /// <summary>The curve of the element, for writing (OCCT's non-const reference).</summary>
    internal readonly Geom2dAdaptor_Curve ChangeCurve() => myCurve;

    /// <summary>Sets the orientation of the element.</summary>
    internal void Orientation(TopAbs_Orientation Orientation)
    {
        myOrientation = Orientation;
    }

    /// <summary>The orientation of the element.</summary>
    public readonly TopAbs_Orientation Orientation() => myOrientation;

    /// <summary>A copy with the orientation <paramref name="Orientation"/>.</summary>
    public readonly Geom2dHatch_Element WithOrientation(TopAbs_Orientation Orientation)
    {
        Geom2dHatch_Element aCopy = this;
        aCopy.Orientation(Orientation);
        return aCopy;
    }

    public readonly bool Equals(Geom2dHatch_Element other) =>
        ReferenceEquals(myCurve, other.myCurve) && myOrientation == other.myOrientation;

    public override readonly bool Equals(object? obj) => obj is Geom2dHatch_Element other && Equals(other);

    public override readonly int GetHashCode() =>
        HashCode.Combine(System.Runtime.CompilerServices.RuntimeHelpers.GetHashCode(myCurve), myOrientation);

    public static bool operator ==(Geom2dHatch_Element left, Geom2dHatch_Element right) => left.Equals(right);

    public static bool operator !=(Geom2dHatch_Element left, Geom2dHatch_Element right) => !left.Equals(right);

    public override readonly string ToString() =>
        $"Geom2dHatch_Element({(myCurve?.Curve() is { } aCurve ? global::Kerf.Engine.Foundation.OcctTypeNames.Of(aCurve.GetType()) : null) ?? "no curve"}, {myOrientation})";
}
