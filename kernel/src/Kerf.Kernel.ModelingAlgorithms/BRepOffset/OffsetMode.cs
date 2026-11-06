// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Mode.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Offsets;

/// <summary>The construction modes of the 3D offset.</summary>
public enum OffsetMode
{
    /// <summary>The offset is a skin around the shape (a side of it).</summary>
    Skin,
    /// <summary>The offset of an open shape is a pipe around it.</summary>
    Pipe,
    /// <summary>The offset is made on both sides of the shape.</summary>
    RectoVerso,
}
