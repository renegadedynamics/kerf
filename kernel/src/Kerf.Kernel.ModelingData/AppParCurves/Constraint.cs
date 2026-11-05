// Port of: src/ModelingData/TKGeomBase/AppParCurves/AppParCurves_Constraint.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Approximation;

/// <summary>The constraint on a point of a MultiLine.</summary>
/// <remarks>The algorithms compare constraints by order (<c>Cons &gt;= AppParCurves_TangencyPoint</c>).</remarks>
internal enum Constraint
{
    /// <summary>The point has no constraint.</summary>
    NoConstraint,

    /// <summary>The approximation curve passes through the point.</summary>
    PassPoint,

    /// <summary>The point has a tangency constraint.</summary>
    TangencyPoint,

    /// <summary>The point has a curvature constraint.</summary>
    CurvaturePoint,
}
