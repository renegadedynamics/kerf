// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_TheIWalking.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_TheIWalking_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The walking of the implicit/parametric intersection: the lines on a surface adaptor where a quadric's function
/// vanishes (OCCT's instantiation of IntWalk_IWalking.gxx, decision 607).
/// </summary>
public sealed class IntPatch_TheIWalking : IntWalk_IWalking<IntPatch_TheSurfFunction, IntPatch_TheIWLineOfTheIWalking>
{
    public IntPatch_TheIWalking(double Epsilon, double Deflection, double Step, bool theToFillHoles = false)
        : base(Epsilon, Deflection, Step, theToFillHoles)
    {
    }
}
