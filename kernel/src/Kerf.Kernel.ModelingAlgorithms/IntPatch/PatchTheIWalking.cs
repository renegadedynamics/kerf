// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_TheIWalking.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_TheIWalking_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.Walking;

using Kerf.Engine;

namespace Kerf.Engine.Intersection.Patches;

/// <summary>
/// The walking of the implicit/parametric intersection: the lines on a surface adaptor where a quadric's function
/// vanishes (OCCT's instantiation of IntWalk_IWalking.gxx, decision 607).
/// </summary>
internal sealed class PatchTheIWalking : IWalking<TheSurfFunction, PatchTheIWLineOfTheIWalking>
{
    public PatchTheIWalking(double epsilon, double deflection, double step, bool toFillHoles = false)
        : base(epsilon, deflection, step, toFillHoles)
    {
    }
}
