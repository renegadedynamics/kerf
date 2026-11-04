// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_TheIntersectorOfTheIntConicCurveOfGInter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_TheIntersectorOfTheIntConicCurveOfGInter_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Curves;
using Kerf.Engine.Intersection.Implicit;
using Occt;

namespace Kerf.Engine.Intersection.TwoD;

/// <summary>The intersection between an implicit conic and a bounded 2D curve.</summary>
/// <remarks>OCCT's instantiation of IntImpParGen_Intersector.gxx over Adaptor2d_Curve2d (decision 284).</remarks>
internal sealed class TheIntersectorOfTheIntConicCurveOfGInter
    : Intersector<IConicTool, Curve2DAdaptorBase, Geom2dCurveTool,
                               TheProjPCurOfGInter,
                               MyImpParToolOfTheIntersectorOfTheIntConicCurveOfGInter>
{
    /// <summary>Empty constructor.</summary>
    public TheIntersectorOfTheIntConicCurveOfGInter()
    {
    }

    /// <summary>Intersection between an implicit curve and a parametric curve.</summary>
    /// <exception cref="ConstructionErrorException">The domain of the parametric curve is not bounded on both sides.</exception>
    public TheIntersectorOfTheIntConicCurveOfGInter(in IConicTool iTool, in TwoDDomain dom1,
                                                              Curve2DAdaptorBase pCurve, in TwoDDomain dom2,
                                                              double tolConf, double tol)
        : base(iTool, dom1, pCurve, dom2, tolConf, tol)
    {
    }
}
