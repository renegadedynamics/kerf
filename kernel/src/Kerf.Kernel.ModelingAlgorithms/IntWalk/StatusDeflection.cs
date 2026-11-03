// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntWalk/IntWalk_StatusDeflection.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Intersection.Walking;

/// <summary>The verdict of a walking step's deflection test (IntWalk_IWalking, IntWalk_PWalking).</summary>
/// <remarks>An unscoped enum in C++; ported code imports it with <c>using static Occt.IntWalk_StatusDeflection;</c>.</remarks>
public enum StatusDeflection
{
    /// <summary>The step is too large: divide it.</summary>
    PasTropGrand,

    /// <summary>The step has become too small.</summary>
    StepTooSmall,

    /// <summary>The new point is the previous one.</summary>
    PointConfondu,

    /// <summary>Stop on the previous point.</summary>
    ArretSurPointPrecedent,

    /// <summary>Stop on the new point.</summary>
    ArretSurPoint,

    /// <summary>The step is accepted.</summary>
    OK,
}
