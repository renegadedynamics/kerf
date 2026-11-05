// Port of: src/FoundationClasses/TKMath/math/math_Status.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Numerics;

internal enum NumericStatus
{
    OK,
    TooManyIterations,
    FunctionError,
    DirectionSearchError,
    NotBracketed,
}
