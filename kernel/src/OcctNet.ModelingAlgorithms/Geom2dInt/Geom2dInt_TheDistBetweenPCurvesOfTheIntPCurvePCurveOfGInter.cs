// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_TheDistBetweenPCurvesOfTheIntPCurvePCurveOfGInter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_TheDistBetweenPCurvesOfTheIntPCurvePCurveOfGInter_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;

namespace Occt;

/// <summary>The vector C1(u) - C2(v) between the points of two 2D curves, with its Jacobian.</summary>
/// <remarks>OCCT's instantiation of IntCurve_DistBetweenPCurvesGen.gxx over Adaptor2d_Curve2d (decision 324).</remarks>
public sealed class Geom2dInt_TheDistBetweenPCurvesOfTheIntPCurvePCurveOfGInter
    : IntCurve_DistBetweenPCurvesGen<Curve2DAdaptorBase, Geom2dInt_Geom2dCurveTool>
{
    public Geom2dInt_TheDistBetweenPCurvesOfTheIntPCurvePCurveOfGInter(Curve2DAdaptorBase curve1,
                                                                       Curve2DAdaptorBase curve2)
        : base(curve1, curve2)
    {
    }
}
