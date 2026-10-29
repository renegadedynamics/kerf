// Port of: src/FoundationClasses/TKMath/Poly/Poly_TriangulationParameters.hxx
// Port of: src/FoundationClasses/TKMath/Poly/Poly_TriangulationParameters.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2021-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;

namespace Occt;

/// <summary>
/// The parameters a triangulation was generated with: linear deflection, angular deflection and minimum size, each
/// -1 if undefined. Immutable.
/// </summary>
public class Poly_TriangulationParameters : Standard_Transient
{
    private readonly double myDeflection;
    private readonly double myAngle;
    private readonly double myMinSize;

    /// <summary>The parameters: linear deflection, angular deflection and minimum size, -1 for undefined.</summary>
    public Poly_TriangulationParameters(double theDeflection = -1.0, double theAngle = -1.0, double theMinSize = -1.0)
    {
        myDeflection = theDeflection;
        myAngle = theAngle;
        myMinSize = theMinSize;
    }

    /// <summary>A copy of these parameters.</summary>
    public Poly_TriangulationParameters Copy() => new(myDeflection, myAngle, myMinSize);

    /// <summary>True if the linear deflection is defined.</summary>
    public bool HasDeflection() => !(myDeflection < 0.0);

    /// <summary>True if the angular deflection is defined.</summary>
    public bool HasAngle() => !(myAngle < 0.0);

    /// <summary>True if the minimum size is defined.</summary>
    public bool HasMinSize() => !(myMinSize < 0.0);

    /// <summary>The linear deflection, or -1 if undefined.</summary>
    public double Deflection() => myDeflection;

    /// <summary>The angular deflection, or -1 if undefined.</summary>
    public double Angle() => myAngle;

    /// <summary>The minimum size, or -1 if undefined.</summary>
    public double MinSize() => myMinSize;

    public override string ToString()
        => string.Create(CultureInfo.InvariantCulture, $"Poly_TriangulationParameters({myDeflection:R}, {myAngle:R}, {myMinSize:R})");
}
