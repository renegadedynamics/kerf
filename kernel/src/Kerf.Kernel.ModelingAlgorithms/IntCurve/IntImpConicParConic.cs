// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_IntImpConicParConic.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_IntImpConicParConic_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Intersection.Implicit;
using Kerf.Engine.Intersection.TwoD;
using Occt;

namespace Kerf.Engine.Intersection.Curves;

/// <summary>The intersection between an implicit conic and a bounded parametric conic.</summary>
/// <remarks>OCCT's instantiation of IntImpParGen_Intersector.gxx over the conic tools (decision 284).</remarks>
internal sealed class IntImpConicParConic
    : Intersector<IConicTool, PConic, PConicTool,
                               ProjectOnPConicTool, MyImpParToolOfIntImpConicParConic>
{
    /// <summary>Empty constructor.</summary>
    public IntImpConicParConic()
    {
    }

    /// <summary>Intersection between an implicit curve and a parametric curve.</summary>
    /// <exception cref="ConstructionErrorException">The domain of the parametric curve is not bounded on both sides.</exception>
    public IntImpConicParConic(in IConicTool iTool, in TwoDDomain dom1,
                                        in PConic pCurve, in TwoDDomain dom2, double tolConf, double tol)
        : base(iTool, dom1, pCurve, dom2, tolConf, tol)
    {
    }
}
