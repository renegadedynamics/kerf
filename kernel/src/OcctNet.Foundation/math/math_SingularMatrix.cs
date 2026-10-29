// Port of: src/FoundationClasses/TKMath/math/math_SingularMatrix.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Thrown when a matrix is singular.</summary>
public class math_SingularMatrix : Standard_Failure
{
    public math_SingularMatrix() { }

    public math_SingularMatrix(string? message) : base(message) { }
}
