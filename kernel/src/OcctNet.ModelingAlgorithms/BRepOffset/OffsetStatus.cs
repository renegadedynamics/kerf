// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Status.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Offsets;

/// <summary>The status of an offset surface (<see cref="BRepOffset.Surface"/>, <see cref="Offset"/>).</summary>
public enum OffsetStatus
{
    Good,
    /// <summary>The offset surface is reversed.</summary>
    Reversed,
    /// <summary>The offset surface is degenerated.</summary>
    Degenerated,
    Unknown,
}
