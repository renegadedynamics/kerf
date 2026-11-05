// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_TheIWalking.hxx
// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_TheIWalking_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.Walking;

using Kerf.Engine;

namespace Kerf.Engine.HiddenLines.Contours;

/// <summary>
/// The walking of the contours: the lines on a surface adaptor where the contour function
/// vanishes (OCCT's instantiation of IntWalk_IWalking.gxx, decision 607).
/// </summary>
internal sealed class TheIWalking : IWalking<SurfFunction, TheIWLineOfTheIWalking>
{
    public TheIWalking(double epsilon, double deflection, double step, bool toFillHoles = false)
        : base(epsilon, deflection, step, toFillHoles)
    {
    }
}
