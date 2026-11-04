// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_PerfSelection.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Features;

/// <summary>
/// To declare the type of selection semantics for local operation Perform methods: no selection, from and until faces,
/// until a face, up to the next face of the shape, up to the shape's last face.
/// </summary>
internal enum PerfSelection
{
    NoSelection,
    SelectionFU,
    SelectionU,
    SelectionSh,
    SelectionShU,
}
