// Port of: src/ModelingData/TKGeomBase/LProp/LProp_NotDefined.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine;

namespace Kerf.Engine.Geometry.Properties;

/// <summary>Thrown when a local property (tangent, normal, curvature, centre of curvature) is not defined.</summary>
internal class NotDefinedException : FailureException
{
    public NotDefinedException() { }
    public NotDefinedException(string? message) : base(message) { }
}
