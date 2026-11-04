// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_TheIntPCurvePCurveOfGInter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_TheIntPCurvePCurveOfGInter_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Curves;
using Occt;

namespace Kerf.Engine.Intersection.TwoD;

/// <summary>
/// The intersection of two bounded 2D curves, or the self-intersection of one, through their polygons.
/// </summary>
/// <remarks>OCCT's instantiation of IntCurve_IntPolyPolyGen.gxx over Adaptor2d_Curve2d (decision 324).</remarks>
internal sealed class TheIntPCurvePCurveOfGInter
    : IntPolyPolyGen<Curve2DAdaptorBase, Geom2dCurveTool, TheProjPCurOfGInter>
{
    public TheIntPCurvePCurveOfGInter()
    {
    }
}
