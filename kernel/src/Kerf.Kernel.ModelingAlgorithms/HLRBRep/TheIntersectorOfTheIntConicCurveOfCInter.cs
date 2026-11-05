// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TheIntersectorOfTheIntConicCurveOfCInter.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TheIntersectorOfTheIntConicCurveOfCInter_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Intersection.Curves;
using Kerf.Engine.Intersection.Implicit;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine;

namespace Kerf.Engine.HiddenLines;

/// <summary>The intersection between an implicit conic and a bounded 2D curve.</summary>
/// <remarks>OCCT's instantiation of IntImpParGen_Intersector.gxx over HLRBRep_Curve (decision 284).</remarks>
internal sealed class TheIntersectorOfTheIntConicCurveOfCInter
    : Intersector<IConicTool, HiddenLineCurve, HiddenLineCurveTool,
                               TheProjPCurOfCInter,
                               MyImpParToolOfTheIntersectorOfTheIntConicCurveOfCInter>
{
    /// <summary>Empty constructor.</summary>
    public TheIntersectorOfTheIntConicCurveOfCInter()
    {
    }

    /// <summary>Intersection between an implicit curve and a parametric curve.</summary>
    /// <exception cref="ConstructionErrorException">The domain of the parametric curve is not bounded on both sides.</exception>
    public TheIntersectorOfTheIntConicCurveOfCInter(in IConicTool iTool, in TwoDDomain dom1,
                                                              HiddenLineCurve pCurve, in TwoDDomain dom2,
                                                              double tolConf, double tol)
        : base(iTool, dom1, pCurve, dom2, tolConf, tol)
    {
    }
}
