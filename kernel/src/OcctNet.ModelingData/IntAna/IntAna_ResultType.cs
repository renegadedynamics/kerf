// Port of: src/ModelingData/TKGeomBase/IntAna/IntAna_ResultType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The kind of result of an analytic intersection of two quadrics (IntAna_QuadQuadGeo).</summary>
public enum IntAna_ResultType
{
    IntAna_Point,
    IntAna_Line,
    IntAna_Circle,
    IntAna_PointAndCircle,
    IntAna_Ellipse,
    IntAna_Parabola,
    IntAna_Hyperbola,
    IntAna_Empty,
    IntAna_Same,
    IntAna_NoGeometricSolution,
}
