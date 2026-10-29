// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_ExactIntersectionPointOfTheIntPCurvePCurveOfGInter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_ExactIntersectionPointOfTheIntPCurvePCurveOfGInter_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The exact intersection point of two 2D curves near the crossing of two segments of their polygons.</summary>
/// <remarks>OCCT's instantiation of IntCurve_ExactIntersectionPoint.gxx over Adaptor2d_Curve2d (decision 3504).</remarks>
public sealed class Geom2dInt_ExactIntersectionPointOfTheIntPCurvePCurveOfGInter
    : IntCurve_ExactIntersectionPoint<Adaptor2d_Curve2d, Geom2dInt_Geom2dCurveTool>
{
    public Geom2dInt_ExactIntersectionPointOfTheIntPCurvePCurveOfGInter(Adaptor2d_Curve2d C1, Adaptor2d_Curve2d C2,
                                                                        double Tol)
        : base(C1, C2, Tol)
    {
    }
}
