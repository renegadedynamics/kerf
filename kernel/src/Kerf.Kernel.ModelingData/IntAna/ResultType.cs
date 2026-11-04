// Port of: src/ModelingData/TKGeomBase/IntAna/IntAna_ResultType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Intersection;

/// <summary>The kind of result of an analytic intersection of two quadrics (IntAna_QuadQuadGeo).</summary>
internal enum ResultType
{
    Point,
    Line,
    Circle,
    PointAndCircle,
    Ellipse,
    Parabola,
    Hyperbola,
    Empty,
    Same,
    NoGeometricSolution,
}
