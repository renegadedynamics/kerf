// Port of: src/ModelingAlgorithms/TKHLR/Intrv/Intrv_Position.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The position of an interval relative to another (see <see cref="Intrv_Interval.Position"/>).</summary>
public enum Intrv_Position
{
    Intrv_Before,
    Intrv_JustBefore,
    Intrv_OverlappingAtStart,
    Intrv_JustEnclosingAtEnd,
    Intrv_Enclosing,
    Intrv_JustOverlappingAtStart,
    Intrv_Similar,
    Intrv_JustEnclosingAtStart,
    Intrv_Inside,
    Intrv_JustOverlappingAtEnd,
    Intrv_OverlappingAtEnd,
    Intrv_JustAfter,
    Intrv_After,
}
