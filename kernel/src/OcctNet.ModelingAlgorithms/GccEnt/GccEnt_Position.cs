// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccEnt/GccEnt_Position.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The qualifier of an argument of a constraint: the relative position of the solution and the argument.</summary>
public enum GccEnt_Position
{
    GccEnt_unqualified,
    GccEnt_enclosing,
    GccEnt_enclosed,
    GccEnt_outside,
    GccEnt_noqualifier,
}
