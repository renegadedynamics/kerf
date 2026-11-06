// Port of: src/FoundationClasses/TKMath/Poly/Poly_TriangulationParameters.hxx
// Port of: src/FoundationClasses/TKMath/Poly/Poly_TriangulationParameters.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2021-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Globalization;

namespace Kerf.Engine.Meshing;

/// <summary>
/// The parameters a triangulation was generated with: linear deflection, angular deflection and minimum size, each
/// -1 if undefined. Immutable.
/// </summary>
internal class TriangulationParameters : Transient
{
    private readonly double _deflection;
    private readonly double _angle;
    private readonly double _minSize;

    /// <summary>The parameters: linear deflection, angular deflection and minimum size, -1 for undefined.</summary>
    public TriangulationParameters(double deflection = -1.0, double angle = -1.0, double minSize = -1.0)
    {
        _deflection = deflection;
        _angle = angle;
        _minSize = minSize;
    }

    /// <summary>A copy of these parameters.</summary>
    public TriangulationParameters Copy() => new(_deflection, _angle, _minSize);

    /// <summary>True if the linear deflection is defined.</summary>
    public bool HasDeflection => !(_deflection < 0.0);

    /// <summary>True if the angular deflection is defined.</summary>
    public bool HasAngle => !(_angle < 0.0);

    /// <summary>True if the minimum size is defined.</summary>
    public bool HasMinSize => !(_minSize < 0.0);

    /// <summary>The linear deflection, or -1 if undefined.</summary>
    public double Deflection() => _deflection;

    /// <summary>The angular deflection, or -1 if undefined.</summary>
    public double Angle() => _angle;

    /// <summary>The minimum size, or -1 if undefined.</summary>
    public double MinSize() => _minSize;

    public override string ToString()
        => string.Create(CultureInfo.InvariantCulture, $"Poly_TriangulationParameters({_deflection:R}, {_angle:R}, {_minSize:R})");
}
