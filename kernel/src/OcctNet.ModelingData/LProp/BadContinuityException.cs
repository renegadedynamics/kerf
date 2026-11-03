// Port of: src/ModelingData/TKGeomBase/LProp/LProp_BadContinuity.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Geometry.Properties;

/// <summary>Thrown when a local property needs more continuity than the curve or surface has.</summary>
/// <remarks>OCCT's <c>LProp_BadContinuity_Raise_if</c> macro is defined but never used, so it isn't ported.</remarks>
public class BadContinuityException : FailureException
{
    public BadContinuityException() { }
    public BadContinuityException(string? message) : base(message) { }
}
