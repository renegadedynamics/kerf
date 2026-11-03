// Port of: src/ModelingAlgorithms/TKHLR/Intrv/Intrv_Position.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Intersection;

/// <summary>The position of an interval relative to another (see <see cref="IntersectionInterval.Position"/>).</summary>
public enum Position
{
    Before,
    JustBefore,
    OverlappingAtStart,
    JustEnclosingAtEnd,
    Enclosing,
    JustOverlappingAtStart,
    Similar,
    JustEnclosingAtStart,
    Inside,
    JustOverlappingAtEnd,
    OverlappingAtEnd,
    JustAfter,
    After,
}
