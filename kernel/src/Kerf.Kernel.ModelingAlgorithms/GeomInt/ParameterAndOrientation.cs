// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_ParameterAndOrientation.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_ParameterAndOrientation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Intersection.Surfaces;

/// <summary>A parameter on an intersection line with the orientations of the line on the two surfaces'
/// restrictions there (GeomInt_LineConstructor).</summary>
/// <remarks><c>new()</c> is OCCT's default (0, FORWARD, FORWARD), which is also <c>default</c>.</remarks>
public partial struct ParameterAndOrientation : IEquatable<ParameterAndOrientation>
{
    private double _prm;
    private Orientation _or1;
    private Orientation _or2;

    public ParameterAndOrientation()
    {
        _prm = 0.0;
        _or1 = Forward;
        _or2 = Forward;
    }

    public ParameterAndOrientation(double p, Orientation or1, Orientation or2)
    {
        _prm = p;
        _or1 = or1;
        _or2 = or2;
    }

    internal void SetOrientation1(Orientation or1) => _or1 = or1;

    internal void SetOrientation2(Orientation or2) => _or2 = or2;

    /// <summary>A copy with the first orientation set (the twin of the internal SetOrientation1).</summary>
    public readonly ParameterAndOrientation WithOrientation1(Orientation or1)
    {
        ParameterAndOrientation copy = this;
        copy.SetOrientation1(or1);
        return copy;
    }

    /// <summary>A copy with the second orientation set (the twin of the internal SetOrientation2).</summary>
    public readonly ParameterAndOrientation WithOrientation2(Orientation or2)
    {
        ParameterAndOrientation copy = this;
        copy.SetOrientation2(or2);
        return copy;
    }

    public readonly double Parameter() => _prm;

    public readonly Orientation Orientation1() => _or1;

    public readonly Orientation Orientation2() => _or2;

    public readonly bool Equals(ParameterAndOrientation other)
        => _prm.Equals(other._prm) && _or1 == other._or1 && _or2 == other._or2;

    public override readonly bool Equals(object? obj) => obj is ParameterAndOrientation other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_prm, _or1, _or2);

    public static bool operator ==(in ParameterAndOrientation left, in ParameterAndOrientation right)
        => left.Equals(right);

    public static bool operator !=(in ParameterAndOrientation left, in ParameterAndOrientation right)
        => !left.Equals(right);

    public override readonly string ToString()
        => $"GeomInt_ParameterAndOrientation({_prm.ToString("R", System.Globalization.CultureInfo.InvariantCulture)}, {_or1}, {_or2})";
}
