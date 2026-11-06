// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_TheSearchInside.hxx
// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_TheSearchInside_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Engine.Intersection.Walking;

namespace Kerf.Engine.HiddenLines.Contours;

/// <summary>
/// The interior start points of the contours on a surface adaptor (OCCT's instantiation of
/// IntStart_SearchInside.gxx, decision 607).
/// </summary>
internal sealed class TheSearchInside
    : SearchInside<SurfaceAdaptorBase, TheHSurfaceTool, SurfFunction>
{
    public TheSearchInside()
    {
    }

    public TheSearchInside(SurfFunction f, SurfaceAdaptorBase surf, TopolTool t,
                                    double epsilon)
        : base(f, surf, t, epsilon)
    {
    }
}
