// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_TheIntersectorOfTheIntConicCurveOfGInter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_TheIntersectorOfTheIntConicCurveOfGInter_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;

namespace Occt;

/// <summary>The intersection between an implicit conic and a bounded 2D curve.</summary>
/// <remarks>OCCT's instantiation of IntImpParGen_Intersector.gxx over Adaptor2d_Curve2d (decision 284).</remarks>
public sealed class Geom2dInt_TheIntersectorOfTheIntConicCurveOfGInter
    : IntImpParGen_Intersector<IntCurve_IConicTool, Curve2DAdaptorBase, Geom2dInt_Geom2dCurveTool,
                               Geom2dInt_TheProjPCurOfGInter,
                               Geom2dInt_MyImpParToolOfTheIntersectorOfTheIntConicCurveOfGInter>
{
    /// <summary>Empty constructor.</summary>
    public Geom2dInt_TheIntersectorOfTheIntConicCurveOfGInter()
    {
    }

    /// <summary>Intersection between an implicit curve and a parametric curve.</summary>
    /// <exception cref="ConstructionErrorException">The domain of the parametric curve is not bounded on both sides.</exception>
    public Geom2dInt_TheIntersectorOfTheIntConicCurveOfGInter(in IntCurve_IConicTool ITool, in IntRes2d_Domain Dom1,
                                                              Curve2DAdaptorBase PCurve, in IntRes2d_Domain Dom2,
                                                              double TolConf, double Tol)
        : base(ITool, Dom1, PCurve, Dom2, TolConf, Tol)
    {
    }
}
