// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_HInter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_HInter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;

namespace Kerf.Engine.Intersection.CurveSurface;

/// <summary>
/// The intersection of a curve adaptor and a surface adaptor: the points (with their parameters on both and the
/// transition of the curve), and whether the curve is parallel to or on the surface in the analytic cases.
/// </summary>
/// <remarks>
/// OCCT's instantiation of IntCurveSurface_Inter.pxx, a sealed subclass of the generic
/// <c>IntCurveSurface_Inter</c>, which holds OCCT's Perform overloads and protected methods (decision 337). A
/// working object: <c>Perform</c> can be called again. The adaptors are kept only during <c>Perform</c>.
/// </remarks>
internal sealed class HInter
    : Inter<CurveAdaptorBase, TheHCurveTool, SurfaceAdaptorBase,
                            TheHSurfaceTool, ThePolygonOfHInter,
                            ThePolyhedronOfHInter, TheInterferenceOfHInter,
                            TheCSFunctionOfHInter, TheExactHInter,
                            TheQuadCurvExactHInter>
{
    /// <summary>An empty intersection; <c>Perform</c> computes it.</summary>
    public HInter()
    {
    }

    /// <summary>
    /// Polyhedra of this surface already built, by parameter range, to reuse: many edges of a model meet one face, and each
    /// would otherwise sample the whole surface again. A polyhedron is only read once it's built. Not shared between threads.
    /// </summary>
    internal Dictionary<(double U1, double V1, double U2, double V2), ThePolyhedronOfHInter>? Polyhedra { get; set; }

    /// <summary>
    /// Kerf: a proof, given the surface's range (u0, u1, v0, v1), that the curve is further from the surface over it than any
    /// point the intersection accepts; a range it proves is passed over (unit 15AS). None: every range is computed.
    /// </summary>
    internal Func<double, double, double, double, bool>? Apart { get; set; }

    protected override bool IsApartOn(double u0, double u1, double v0, double v1) => Apart is { } apart && apart(u0, u1, v0, v1);

    protected override ThePolyhedronOfHInter PolyhedronFor(in SurfaceAdaptorBase surface, SurfaceKind surfaceType, double u1, double v1, double u2, double v2)
    {
        if (Polyhedra is null)
        {
            return base.PolyhedronFor(surface, surfaceType, u1, v1, u2, v2);
        }

        if (!Polyhedra.TryGetValue((u1, v1, u2, v2), out ThePolyhedronOfHInter? polyhedron))
        {
            polyhedron = base.PolyhedronFor(surface, surfaceType, u1, v1, u2, v2);
            Polyhedra.Add((u1, v1, u2, v2), polyhedron);
        }

        return polyhedron;
    }
}
