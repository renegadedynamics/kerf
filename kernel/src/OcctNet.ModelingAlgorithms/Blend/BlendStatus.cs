// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_Status.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Fillets.Blend;

/// <summary>The status of a step of a blend walking.</summary>
public enum BlendStatus
{
    StepTooLarge,
    StepTooSmall,
    Backward,
    SamePoints,
    OnRst1,
    OnRst2,
    OnRst12,
    OK,
}
