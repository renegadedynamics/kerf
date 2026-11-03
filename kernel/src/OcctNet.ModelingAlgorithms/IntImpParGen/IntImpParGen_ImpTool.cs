// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntImpParGen/IntImpParGen_ImpTool.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// An implicit curve, as the generic intersector
/// <see cref="IntImpParGen_Intersector{ImpTool, ParCurve, ParTool, ProjectOnPCurveTool, MyImpParTool}"/> sees it
/// (OCCT's template class IntImpParGen_ImpTool; implemented by IntCurve_IConicTool).
/// </summary>
public interface IIntImpParGen_ImpTool
{
    Point2D Value(double U);

    void D1(double U, out Point2D P, out Vector2D T);

    void D2(double U, out Point2D P, out Vector2D T, out Vector2D N);

    /// <summary>The signed distance between the point <paramref name="P"/> and the implicit curve.</summary>
    double Distance(in Point2D P);

    /// <summary>The gradient of the signed distance between a point and the implicit curve, at the point <paramref name="P"/>.</summary>
    Vector2D GradDistance(in Point2D P);

    /// <summary>
    /// The parameter of the point of the implicit curve corresponding to <paramref name="P"/>, consistently with
    /// the way the signed distance is computed.
    /// </summary>
    double FindParameter(in Point2D P);
}
