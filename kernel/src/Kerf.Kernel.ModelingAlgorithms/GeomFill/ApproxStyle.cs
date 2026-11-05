// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_ApproxStyle.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>The approximation strategy of <see cref="SurfaceSweep.Build"/>.</summary>
internal enum ApproxStyle
{
    /// <summary>The composed function, location times section, is approximated directly.</summary>
    Section,

    /// <summary>The location law is approximated and composed with the section law's surface.</summary>
    Location,
}
