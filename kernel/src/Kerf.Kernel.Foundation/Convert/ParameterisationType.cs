// Port of: src/FoundationClasses/TKMath/Convert/Convert_ParameterisationType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Geometry.Conversion;

/// <summary>
/// The parameterisation of a circle or an ellipse converted to a B-spline curve.
/// </summary>
/// <remarks>
/// <list type="bullet">
/// <item><c>Convert_TgtThetaOver2</c>: rational, t = tan(Theta / 2) on each span; the number of spans is
/// (1.2 * Delta / Pi) + 1 for an opening angle Delta. Exact.</item>
/// <item><c>Convert_TgtThetaOver2_1</c> to <c>_4</c>: the same with 1 to 4 spans; Delta must be at most 0.9999 * Pi
/// for one span and 1.9999 * Pi for two.</item>
/// <item><c>Convert_QuasiAngular</c>: rational, with t very close to the angle Theta. Exact.</item>
/// <item><c>Convert_RationalC1</c>: rational with a C1 denominator, for better continuity between spans. Exact.</item>
/// <item><c>Convert_Polynomial</c>: non-rational, 8 poles (degree 7); an approximation.</item>
/// </list>
/// An unscoped enum in C++; ported code imports it with <c>using static Kerf.Engine.Convert_ParameterisationType;</c>.
/// </remarks>
internal enum ParameterisationType
{
    TgtThetaOver2,
    TgtThetaOver2_1,
    TgtThetaOver2_2,
    TgtThetaOver2_3,
    TgtThetaOver2_4,
    QuasiAngular,
    RationalC1,
    Polynomial,
}
