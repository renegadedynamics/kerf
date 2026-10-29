// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_IntImpConicParConic.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_IntImpConicParConic_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The intersection between an implicit conic and a bounded parametric conic.</summary>
/// <remarks>OCCT's instantiation of IntImpParGen_Intersector.gxx over the conic tools (decision 284).</remarks>
public sealed class IntCurve_IntImpConicParConic
    : IntImpParGen_Intersector<IntCurve_IConicTool, IntCurve_PConic, IntCurve_PConicTool,
                               IntCurve_ProjectOnPConicTool, IntCurve_MyImpParToolOfIntImpConicParConic>
{
    /// <summary>Empty constructor.</summary>
    public IntCurve_IntImpConicParConic()
    {
    }

    /// <summary>Intersection between an implicit curve and a parametric curve.</summary>
    /// <exception cref="Standard_ConstructionError">The domain of the parametric curve is not bounded on both sides.</exception>
    public IntCurve_IntImpConicParConic(in IntCurve_IConicTool ITool, in IntRes2d_Domain Dom1,
                                        in IntCurve_PConic PCurve, in IntRes2d_Domain Dom2, double TolConf, double Tol)
        : base(ITool, Dom1, PCurve, Dom2, TolConf, Tol)
    {
    }
}
