// Port of: src/ModelingData/TKGeomBase/LProp/LProp_Status.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Geometry.Properties;

/// <summary>The state of a computed local property (tangent, normal, curvature) in the CLProps and SLProps classes.</summary>
internal enum Status
{
    Undecided,
    Undefined,
    Defined,
    Computed,
}
