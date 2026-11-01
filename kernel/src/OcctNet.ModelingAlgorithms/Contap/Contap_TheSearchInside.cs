// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_TheSearchInside.hxx
// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_TheSearchInside_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The interior start points of the contours on a surface adaptor (OCCT's instantiation of
/// IntStart_SearchInside.gxx, decision 607).
/// </summary>
public sealed class Contap_TheSearchInside
    : IntStart_SearchInside<Adaptor3d_Surface, IntCurveSurface_TheHSurfaceTool, Contap_SurfFunction>
{
    public Contap_TheSearchInside()
    {
    }

    public Contap_TheSearchInside(Contap_SurfFunction F, Adaptor3d_Surface Surf, Adaptor3d_TopolTool T,
                                    double Epsilon)
        : base(F, Surf, T, Epsilon)
    {
    }
}
