// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_ParameterAndOrientation.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_ParameterAndOrientation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>A parameter on an intersection line with the orientations of the line on the two surfaces'
/// restrictions there (GeomInt_LineConstructor).</summary>
/// <remarks><c>new()</c> is OCCT's default (0, FORWARD, FORWARD), which is also <c>default</c>.</remarks>
public partial struct GeomInt_ParameterAndOrientation : IEquatable<GeomInt_ParameterAndOrientation>
{
    private double prm;
    private TopAbs_Orientation or1;
    private TopAbs_Orientation or2;

    public GeomInt_ParameterAndOrientation()
    {
        prm = 0.0;
        or1 = TopAbs_FORWARD;
        or2 = TopAbs_FORWARD;
    }

    public GeomInt_ParameterAndOrientation(double P, TopAbs_Orientation Or1, TopAbs_Orientation Or2)
    {
        prm = P;
        or1 = Or1;
        or2 = Or2;
    }

    internal void SetOrientation1(TopAbs_Orientation Or1) => or1 = Or1;

    internal void SetOrientation2(TopAbs_Orientation Or2) => or2 = Or2;

    /// <summary>A copy with the first orientation set (the twin of the internal SetOrientation1).</summary>
    public readonly GeomInt_ParameterAndOrientation WithOrientation1(TopAbs_Orientation Or1)
    {
        GeomInt_ParameterAndOrientation aCopy = this;
        aCopy.SetOrientation1(Or1);
        return aCopy;
    }

    /// <summary>A copy with the second orientation set (the twin of the internal SetOrientation2).</summary>
    public readonly GeomInt_ParameterAndOrientation WithOrientation2(TopAbs_Orientation Or2)
    {
        GeomInt_ParameterAndOrientation aCopy = this;
        aCopy.SetOrientation2(Or2);
        return aCopy;
    }

    public readonly double Parameter() => prm;

    public readonly TopAbs_Orientation Orientation1() => or1;

    public readonly TopAbs_Orientation Orientation2() => or2;

    public readonly bool Equals(GeomInt_ParameterAndOrientation other)
        => prm.Equals(other.prm) && or1 == other.or1 && or2 == other.or2;

    public override readonly bool Equals(object? obj) => obj is GeomInt_ParameterAndOrientation other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(prm, or1, or2);

    public static bool operator ==(in GeomInt_ParameterAndOrientation theLeft, in GeomInt_ParameterAndOrientation theRight)
        => theLeft.Equals(theRight);

    public static bool operator !=(in GeomInt_ParameterAndOrientation theLeft, in GeomInt_ParameterAndOrientation theRight)
        => !theLeft.Equals(theRight);

    public override readonly string ToString()
        => $"GeomInt_ParameterAndOrientation({prm.ToString("R", System.Globalization.CultureInfo.InvariantCulture)}, {or1}, {or2})";
}
