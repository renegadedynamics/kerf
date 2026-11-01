// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TheIntersectorOfTheIntConicCurveOfCInter.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TheIntersectorOfTheIntConicCurveOfCInter_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The intersection between an implicit conic and a bounded 2D curve.</summary>
/// <remarks>OCCT's instantiation of IntImpParGen_Intersector.gxx over HLRBRep_Curve (decision 284).</remarks>
public sealed class HLRBRep_TheIntersectorOfTheIntConicCurveOfCInter
    : IntImpParGen_Intersector<IntCurve_IConicTool, HLRBRep_Curve, HLRBRep_CurveTool,
                               HLRBRep_TheProjPCurOfCInter,
                               HLRBRep_MyImpParToolOfTheIntersectorOfTheIntConicCurveOfCInter>
{
    /// <summary>Empty constructor.</summary>
    public HLRBRep_TheIntersectorOfTheIntConicCurveOfCInter()
    {
    }

    /// <summary>Intersection between an implicit curve and a parametric curve.</summary>
    /// <exception cref="Standard_ConstructionError">The domain of the parametric curve is not bounded on both sides.</exception>
    public HLRBRep_TheIntersectorOfTheIntConicCurveOfCInter(in IntCurve_IConicTool ITool, in IntRes2d_Domain Dom1,
                                                              HLRBRep_Curve PCurve, in IntRes2d_Domain Dom2,
                                                              double TolConf, double Tol)
        : base(ITool, Dom1, PCurve, Dom2, TolConf, Tol)
    {
    }
}
