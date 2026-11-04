// Port of: src/FoundationClasses/TKMath/math/math_SingularMatrix.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Occt;

namespace Kerf.Engine.Numerics;

/// <summary>Thrown when a matrix is singular.</summary>
internal class SingularMatrixException : FailureException
{
    public SingularMatrixException() { }

    public SingularMatrixException(string? message) : base(message) { }
}
