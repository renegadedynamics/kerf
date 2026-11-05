// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_FillingStyle.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// The three filling styles: Stretch (the flattest patches), Coons (rounded, less deep than Curved) and Curved
/// (the most rounded patches).
/// </summary>
internal enum FillingStyle
{
    StretchStyle,
    CoonsStyle,
    CurvedStyle,
}
