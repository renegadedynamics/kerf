// Port of: src/ModelingData/TKGeomBase/LProp/LProp_CIType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Geometry.Properties;

/// <summary>The type of a particular point on a curve.</summary>
internal enum CIType
{
    /// <summary>A point of inflection.</summary>
    Inflection,
    /// <summary>A minimum of curvature.</summary>
    MinCur,
    /// <summary>A maximum of curvature.</summary>
    MaxCur,
}
