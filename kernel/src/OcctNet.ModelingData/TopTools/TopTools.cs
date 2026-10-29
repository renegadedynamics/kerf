// Port of: src/ModelingData/TKBRep/TopTools/TopTools.hxx
// Port of: src/ModelingData/TKBRep/TopTools/TopTools.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Utilities for the topological data structure: <see cref="TopTools_ShapeMapHasher"/>, the shape and location sets
/// that read and write BRep text, and the maps of shapes (OCCT's typedefs, written out in the port as NCollection maps
/// with <see cref="TopTools_ShapeMapHasher.Instance"/>).
/// </summary>
/// <remarks>
/// <c>Dump</c>, which writes a shape set, comes with TopTools_ShapeSet (unit 3C, as a partial of this class).
/// OCCT's <c>Dummy</c>, a workaround for an old extraction bug, does nothing.
/// </remarks>
public static partial class TopTools
{
    /// <summary>Does nothing (OCCT: a workaround for an old extraction bug).</summary>
    public static void Dummy(int I)
    {
    }
}
