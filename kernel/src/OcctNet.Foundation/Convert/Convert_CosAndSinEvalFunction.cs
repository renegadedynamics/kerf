// Port of: src/FoundationClasses/TKMath/Convert/Convert_CosAndSinEvalFunction.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Evaluates the functions U(t) and V(t) of a rational representation of cos and sin into
/// <paramref name="Result"/> (two values).
/// </summary>
/// <remarks>A C++ function type; <c>double Result[2]</c> is a span. The multiplicities may be null (flat knots).</remarks>
public delegate void Convert_CosAndSinEvalFunction(double Parameter,
                                                   int EvalDegree,
                                                   NCollection_Array1<gp_Pnt2d> EvalPoles,
                                                   NCollection_Array1<double> EvalKnots,
                                                   NCollection_Array1<int>? EvalMults,
                                                   Span<double> Result);
