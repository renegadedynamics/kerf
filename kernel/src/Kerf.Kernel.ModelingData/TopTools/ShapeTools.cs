// Port of: src/ModelingData/TKBRep/TopTools/TopTools.hxx
// Port of: src/ModelingData/TKBRep/TopTools/TopTools.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;

namespace Kerf.Engine.Exchange.Brep;

/// <summary>
/// Utilities for the topological data structure: <see cref="ShapeHasher"/>, the shape and location sets
/// that read and write BRep text, and the maps of shapes (OCCT's typedefs, written out in the port as NCollection maps
/// with <see cref="ShapeHasher.Instance"/>).
/// </summary>
/// <remarks>
/// <c>Dump</c>, which writes a shape set, is in TopTools.Dump.cs with TopTools_ShapeSet.
/// OCCT's <c>Dummy</c>, a workaround for an old extraction bug, does nothing.
/// </remarks>
internal static partial class ShapeTools
{
    /// <summary>Does nothing (OCCT: a workaround for an old extraction bug).</summary>
    public static void Dummy(int i)
    {
    }
}
