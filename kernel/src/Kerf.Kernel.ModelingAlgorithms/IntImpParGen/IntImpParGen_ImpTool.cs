// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntImpParGen/IntImpParGen_ImpTool.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Intersection.Implicit;

/// <summary>
/// An implicit curve, as the generic intersector
/// <see cref="Intersector{ImpTool, ParCurve, ParTool, ProjectOnPCurveTool, MyImpParTool}"/> sees it
/// (OCCT's template class IntImpParGen_ImpTool; implemented by IntCurve_IConicTool).
/// </summary>
internal interface IImpTool
{
    Point2D Value(double u);

    void D1(double u, out Point2D p, out Vector2D t);

    void D2(double u, out Point2D p, out Vector2D t, out Vector2D n);

    /// <summary>The signed distance between the point <paramref name="p"/> and the implicit curve.</summary>
    double Distance(in Point2D p);

    /// <summary>The gradient of the signed distance between a point and the implicit curve, at the point <paramref name="p"/>.</summary>
    Vector2D GradDistance(in Point2D p);

    /// <summary>
    /// The parameter of the point of the implicit curve corresponding to <paramref name="p"/>, consistently with
    /// the way the signed distance is computed.
    /// </summary>
    double FindParameter(in Point2D p);
}
