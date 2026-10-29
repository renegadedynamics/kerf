// Port of: src/FoundationClasses/TKMath/math/math_NotSquare.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Thrown when an operation needs a square matrix.</summary>
public class math_NotSquare : Standard_DimensionError
{
    public math_NotSquare() { }

    public math_NotSquare(string? message) : base(message) { }
}
