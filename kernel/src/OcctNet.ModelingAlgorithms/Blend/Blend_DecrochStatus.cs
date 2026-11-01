// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_DecrochStatus.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Which restrictions a restriction-restriction blend leaves.</summary>
public enum Blend_DecrochStatus
{
    Blend_NoDecroch,
    Blend_DecrochRst1,
    Blend_DecrochRst2,
    Blend_DecrochBoth,
}
